"""RunController: one job run on the SIM or REAL rig, driven from the GUI (docs/HMI_DESIGN.md section 8).

Threads (section 6): the GUI thread calls the controller; everything that may block - loading / building a job,
opening and closing a rig, the preflight, Sequencer.run(), recovery calls, manual camera grabs - runs in the run
thread "mauer-run" (RunWorker, one job after the other). pause / abort / HALT / the answers to step confirmations
act at once from the GUI thread. Results come back as Qt signals (queued into the GUI thread).

Step confirmations (D-H6): the sequencer's confirm callback blocks the RUN thread until the operator answers in the
ConfirmBar; HALT releases a pending confirmation with False. Pause and soft Abort act at the next motion boundary
with empty jaws (D-H7, Sequencer.held). HALT (D-H8): the MainWindow writes the ADS HALT first; halt() sets the abort
flag, releases the pending confirmation and, REAL, stops the UR program in the helper thread "mauer-halt" (the
AresAds abort pulse only when the HMI's ADS worker is not connected). HALT never blocks the GUI.

Resume (8.3) refuses while a stone may be held, while the ARES pose is not ok and, REAL, while the PLC odometry
moved since the run stopped or the preflight has a blocking item. The Sequencer object is kept: its state (placed
stones, route progress, pose estimate) lives in memory only.
"""
from __future__ import annotations

import datetime as _dt
import itertools
import json
import logging
import math
import queue
import threading
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np
from PySide6.QtCore import QObject, Signal

from mauer.ares.ads import OdomPose
from mauer.reference import Pose2D
from mauer.sequencer import Sequencer, SequencerAborted, SequencerError

from .preflight import PreflightReport, real_preflight, sim_preflight
from .rigs import RealFactories, RealRig, RigInfo, SimRig
from .session import JobSession, SessionError, build_from_config, load_job_file
from .snapshot import RunSnapshot, ShotProcessor, SnapshotTracker, default_shot_view
from .sources import UrSource, compose_odometry, odom_from_status

log = logging.getLogger("hmi.run")

STATES = ("empty", "loading", "loaded", "preparing", "ready", "running", "pausing", "aborting", "paused", "aborted",
          "error", "done", "releasing")
BUSY = frozenset({"loading", "preparing", "running", "pausing", "aborting", "releasing"})
RUNNING = frozenset({"running", "pausing", "aborting"})
STOPPED = frozenset({"paused", "aborted", "error"})
WITH_RIG = frozenset({"ready", "paused", "aborted", "error", "done"})
END_STATES = frozenset({"done", "paused", "aborted", "error"})
ACTIONS = ("load", "build", "browse", "options", "step", "sim_speed", "prepare", "start", "pause", "abort", "resume",
           "confirm_pose", "set_pose", "apply_odometry", "clear_held", "recheck", "grab", "release", "halt")


@dataclass
class RunOptions:
    mode: str                         # "sim" | "real"
    step: bool = False                # confirm every motion (changeable live: set_step)
    start_stop: int = 0
    stop_after: int | None = None     # inclusive
    camera_loop: bool = True
    save_images: bool = False
    scenario: str = "realistic"       # SIM: mauer.simworld.scenario
    seed: int = 1                     # SIM
    lenient_grasp: bool = False       # SIM
    sim_step_s: float | None = None   # SIM pacing; None = [hmi] sim_step_s (changeable live)
    log_dir: Path | None = None       # None = <runs_dir>/<YYYY-mm-dd_HHMMSS>_hmi_<mode>[_n] (never reused: RunLog
                                      # appends)


@dataclass(frozen=True)
class ConfirmRequest:
    id: int
    kind: str                         # "motion" | "station_empty"
    text: str                         # the sequencer's description ("robot: ...", "ARES translate ...")
    holding: dict | None              # seq.held when asked
    pause_pending: bool               # pause or soft abort requested (acts after the held stone is put down)
    t: float


def enables(state: str, mode: str | None, *, ares_enabled: bool = False, preflight_ok: bool = False,
            has_seq: bool = False, pose_status: str = "ok", held: bool = False,
            odom_moved: bool = False) -> dict[str, tuple[bool, str]]:
    """Enable matrix of the design (8.4): action -> (allowed, reason if not). Pure."""
    def rule(ok: bool, why: str) -> tuple[bool, str]:
        return (True, "") if ok else (False, why)

    idle = state in ("empty", "loaded")
    stopped = state in STOPPED
    real = mode == "real"
    out = {
        "load": rule(idle, f"not while {state}"),
        "build": rule(idle, f"not while {state}"),
        "browse": rule(idle, f"not while {state}"),
        "options": rule(idle, f"options are fixed while {state} - Release first"),
        "step": (True, ""),
        "sim_speed": (True, ""),
        "prepare": rule(state == "loaded" and (not real or ares_enabled),
                        "load a job first" if state in ("empty", "loading") else
                        "REAL needs the HMI started with --ares" if state == "loaded" else f"not while {state}"),
        "start": rule(state == "ready" and has_seq and (not real or preflight_ok),
                      f"not while {state}" if state != "ready" else
                      "the rig is incomplete (see the preflight list)" if not has_seq else
                      "REAL preflight has blocking problems (no override)"),
        "pause": rule(state == "running", f"not while {state}"),
        "abort": rule(state in ("running", "pausing"), f"not while {state}"),
        "resume": rule(stopped, f"not while {state}"),
        "confirm_pose": rule(stopped and pose_status != "ok", "pose is ok" if stopped else f"not while {state}"),
        "set_pose": rule(stopped, f"not while {state}"),
        "apply_odometry": rule(real and stopped and odom_moved,
                               "REAL only" if not real else "ARES did not move" if stopped else f"not while {state}"),
        "clear_held": rule(stopped and held, "no stone held" if stopped else f"not while {state}"),
        "recheck": rule(real and state in ("ready", "paused", "aborted", "error"), "REAL only, with a rig"),
        "grab": rule(real and state in WITH_RIG, "REAL only, no run active"),
        "release": rule(state in WITH_RIG, f"not while {state}"),
        "halt": (True, ""),
    }
    return out


class RunWorker(threading.Thread):
    """The run thread "mauer-run": executes the queued jobs one after the other (a daemon threading.Thread, so an
    unclosed controller cannot hang or crash the interpreter at exit)."""

    def __init__(self) -> None:
        super().__init__(name="mauer-run", daemon=True)
        self._q: "queue.Queue[Callable[[], None] | None]" = queue.Queue()

    def submit(self, fn: Callable[[], None]) -> None:
        self._q.put(fn)

    def stop(self) -> None:
        self._q.put(None)

    def run(self) -> None:
        while True:
            fn = self._q.get()
            if fn is None:
                return
            try:
                fn()
            except BaseException:       # noqa: BLE001 - the thread must survive any job
                log.exception("run-thread job failed")


def _json_default(x: Any) -> Any:
    if isinstance(x, Pose2D):
        return x.to_dict()
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, np.generic):
        return x.item()
    if isinstance(x, (Path, OdomPose)):
        return str(x)
    return repr(x)


class RunController(QObject):
    """See the module docstring. Construct in the GUI thread; call shutdown() before dropping it."""

    state_changed = Signal(str, str)          # (state, detail)
    event = Signal(object)                    # every RunLog record (dict) of the current sequencer, in order
    snapshot_changed = Signal(object)         # RunSnapshot after each run-thread record
    shot = Signal(object)                     # ShotView (sequencer shot or manual grab)
    confirm_requested = Signal(object)        # ConfirmRequest
    confirm_cleared = Signal(int)             # request id answered or released
    preflight_done = Signal(object)           # PreflightReport
    rig_changed = Signal(object)              # RigInfo | None
    session_loaded = Signal(object)           # JobSession
    message = Signal(str, str)                # (level "info" | "warning" | "error", text) -> status bar

    def __init__(self, hmi: Mapping, *, config_path: Path | None = None, runs_dir: Path,
                 ads_status_fn: Callable[[], dict | None] = lambda: None,
                 ads_connected_fn: Callable[[], bool] = lambda: False, ares_enabled: bool = False,
                 real_factories: RealFactories | None = None, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._hmi = dict(hmi)
        self._config_path = Path(config_path) if config_path else None
        self._runs_dir = Path(runs_dir)
        self._ads_status_fn, self._ads_connected_fn = ads_status_fn, ads_connected_fn
        self._ares_enabled = bool(ares_enabled)
        self._factories = real_factories or RealFactories()
        self._state = "empty"
        self._session: JobSession | None = None
        self._opts: RunOptions | None = None
        self._rig: SimRig | RealRig | None = None
        self._rig_info: RigInfo | None = None
        self._seq: Sequencer | None = None
        self._tracker: SnapshotTracker | None = None
        self._preflight: PreflightReport | None = None
        self._snapshot: RunSnapshot | None = None
        self._halted = False
        self._halt = threading.Event()
        self._abort_soft = threading.Event()
        self._closing = False
        self._confirm_lock = threading.Lock()
        self._pending: ConfirmRequest | None = None
        self._answer: bool | None = None
        self._answer_evt = threading.Event()
        self._ids = itertools.count(1)
        self._step = False
        self._sim_step_s = float(self._hmi.get("sim_step_s", 0.0))
        self._scale = float(self._hmi.get("camera_preview_scale", 0.25))
        self._processor: ShotProcessor | None = None
        self._odom_at_stop: OdomPose | None = None
        self._t_run: float | None = None
        self._worker = RunWorker()
        self._worker.start()

    # ── properties (GUI thread) ──────────────────────────────────────────────
    @property
    def state(self) -> str:
        return self._state

    @property
    def mode(self) -> str | None:
        return self._opts.mode if self._opts is not None else None

    @property
    def session(self) -> JobSession | None:
        return self._session

    @property
    def opts(self) -> RunOptions | None:
        return self._opts

    @property
    def snapshot(self) -> RunSnapshot | None:
        return self._snapshot

    @property
    def rig(self) -> RigInfo | None:
        return self._rig_info

    @property
    def preflight(self) -> PreflightReport | None:
        return self._preflight

    @property
    def halted(self) -> bool:
        return self._halted

    @property
    def pending(self) -> ConfirmRequest | None:
        return self._pending

    @property
    def step(self) -> bool:
        return self._step

    @property
    def sim_step_s(self) -> float:
        return self._sim_step_s

    @property
    def ares_enabled(self) -> bool:
        return self._ares_enabled

    @property
    def sequencer(self) -> Sequencer | None:
        """The current Sequencer (read-only use from the GUI: held, pose_status, paused)."""
        return self._seq

    @property
    def log_dir(self) -> Path | None:
        return self._seq.log.folder if self._seq is not None else None

    def odom_moved(self) -> tuple[float, float, float] | None:
        """REAL: PLC odometry change (dx mm, dy mm, dtheta deg; body frame at the stop) since the run stopped, None
        if it cannot be computed or stays within [hmi] resume_odom_tol_mm / _deg."""
        now = odom_from_status(self._ads_status_fn())
        if self._odom_at_stop is None or now is None:
            return None
        d = self._odom_at_stop.delta_to(now)
        if math.hypot(d[0], d[1]) <= float(self._hmi.get("resume_odom_tol_mm", 2.0)) and \
                abs(d[2]) <= float(self._hmi.get("resume_odom_tol_deg", 0.2)):
            return None
        return d

    def can(self, action: str, mode: str | None = None) -> tuple[bool, str]:
        """Enable matrix (8.4) for the current state; mode: the mode the GUI would prepare (default: the current)."""
        seq = self._seq
        m = mode or self.mode
        return enables(self._state, m, ares_enabled=self._ares_enabled,
                       preflight_ok=self._preflight is not None and self._preflight.ok, has_seq=seq is not None,
                       pose_status=seq.pose_status if seq is not None else "ok",
                       held=seq is not None and seq.held is not None,
                       odom_moved=m == "real" and self.odom_moved() is not None)[action]

    # ── internals ────────────────────────────────────────────────────────────
    def _set_state(self, state: str, detail: str = "") -> None:
        assert state in STATES, state
        self._state = state
        self.state_changed.emit(state, detail)

    def _refuse(self, action: str, mode: str | None = None) -> bool:
        ok, why = self.can(action, mode)
        if not ok:
            self.message.emit("warning", f"{action}: {why}")
        return not ok

    def _submit(self, fn: Callable[[], None], what: str, on_error_state: str | None = None) -> None:
        def job() -> None:
            try:
                fn()
            except Exception as e:      # noqa: BLE001 - reported, the run thread goes on
                log.exception("%s failed", what)
                self.message.emit("error", f"{what} failed: {type(e).__name__}: {e}")
                if on_error_state is not None:
                    self._set_state(on_error_state, str(e))
        self._worker.submit(job)

    def _emit_snapshot(self, snap: RunSnapshot | None) -> None:
        self._snapshot = snap
        self.snapshot_changed.emit(snap)

    def _new_log_dir(self, mode: str) -> Path:
        stamp = _dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        base = self._runs_dir / f"{stamp}_hmi_{mode}"
        p, n = base, 1
        while p.exists():
            n += 1
            p = base.with_name(f"{base.name}_{n}")
        return p

    # ── sessions (run thread) ────────────────────────────────────────────────
    def _install(self, s: JobSession) -> None:
        self._session = s
        self._emit_snapshot(RunSnapshot.initial(s))
        self.session_loaded.emit(s)
        self._set_state("loaded", s.name)

    def set_session(self, session: JobSession) -> None:
        """Synchronous (GUI thread): use an already loaded / built job."""
        if self._refuse("load"):
            return
        self._install(session)

    def load_file(self, path: str | Path) -> None:
        if self._refuse("load"):
            return
        prev = self._state
        self._set_state("loading", str(path))

        def job() -> None:
            try:
                s = load_job_file(path, self._config_path)
            except SessionError as e:
                self.message.emit("error", f"load failed: {e}")
                self._set_state(prev, str(e))
                return
            self._install(s)
        self._submit(job, "load", prev)

    def build_from_config(self, variant: str | None) -> None:
        if self._refuse("build"):
            return
        prev = self._state
        self._set_state("loading", f"building the nominal job of config {variant or 'main'}")

        def job() -> None:
            try:
                s = build_from_config(variant, self._config_path)
            except SessionError as e:
                self.message.emit("error", str(e))
                self._set_state(prev, str(e))
                return
            self._install(s)
        self._submit(job, "build", prev)

    # ── prepare / release ────────────────────────────────────────────────────
    def prepare(self, opts: RunOptions) -> None:
        """SIM: build the simulated world; REAL: open the rig (Connect). Then the preflight and the Sequencer."""
        if self._refuse("prepare", opts.mode):
            return
        self._opts = replace(opts)
        self._step = bool(opts.step)
        if opts.sim_step_s is not None:
            self._sim_step_s = float(opts.sim_step_s)
        self._halted, self._t_run = False, None
        self._halt.clear()
        self._abort_soft.clear()
        self._set_state("preparing", opts.mode)
        self._submit(self._do_prepare, "prepare", "loaded")

    def _do_prepare(self) -> None:
        s, o = self._session, self._opts
        rig: SimRig | RealRig | None = None
        try:
            if o.mode == "sim":
                rig = SimRig(s.cfg, s.job, o)
            else:
                rig = RealRig(s.cfg, s.job, self._factories)
                rig.open()
        except Exception:
            if rig is not None:
                rig.close()
            raise
        self._rig, self._rig_info = rig, rig.info()
        self.rig_changed.emit(self._rig_info)
        self._preflight = self._report()
        self.preflight_done.emit(self._preflight)
        if rig.complete:
            log_dir = Path(o.log_dir) if o.log_dir else self._new_log_dir(o.mode)
            if o.mode == "sim":
                ares = rig.world.ares
            else:
                from mauer.backends import AdsAres
                ares = AdsAres(rig.ads)
            seq = Sequencer(s.job, s.cfg, rig.robot, ares, rig.camera, rig.intr, rig.T_flange_cam, log_dir=log_dir,
                            confirm=self._confirm_cb, camera_loop=o.camera_loop, save_images=o.save_images,
                            on_station_empty=self._station_empty_cb, on_shot=self._on_shot)
            self._tracker = SnapshotTracker(s, lambda: odom_from_status(self._ads_status_fn()))
            seq.log.add_listener(self._on_record)
            self._seq = seq
            self._emit_snapshot(replace(RunSnapshot.initial(s, o.start_stop), log_path=str(seq.log.path)))
            detail = f"{o.mode.upper()} ready, log {seq.log.folder}"
        else:
            detail = "rig incomplete: " + ", ".join(sorted(rig.errors)) if rig.errors else "rig incomplete"
        self._set_state("ready", detail)

    def _report(self) -> PreflightReport:
        s = self._session
        if self.mode == "sim":
            return sim_preflight(s.cfg, s.job, self._config_path)
        return real_preflight(s.cfg, s.job, self._config_path, ares_enabled=self._ares_enabled,
                              ads_connected=bool(self._ads_connected_fn()), ads_status=self._ads_status_fn(),
                              rig=self._rig)

    def recheck(self) -> None:
        """REAL: the preflight again (AresAds check, UR state) without reconnecting."""
        if self._refuse("recheck"):
            return

        def job() -> None:
            if isinstance(self._rig, RealRig):
                self._rig.check_ads()
            self._preflight = self._report()
            self.preflight_done.emit(self._preflight)
        self._submit(job, "re-check")

    def release(self) -> None:
        """Close the Sequencer and the rig (REAL: camera, AresAds, URLink) -> 'loaded'."""
        if self._refuse("release"):
            return
        self._set_state("releasing")
        self._submit(self._do_release, "release", "loaded")

    def _do_release(self) -> None:
        seq, rig = self._seq, self._rig
        self._seq, self._tracker = None, None
        if seq is not None:
            seq.log.remove_listener(self._on_record)
            seq.close()
        if rig is not None:
            rig.close()
        self._rig, self._rig_info, self._preflight, self._odom_at_stop = None, None, None, None
        self.rig_changed.emit(None)
        if self._session is not None:
            self._emit_snapshot(RunSnapshot.initial(self._session))
        self._set_state("loaded" if self._session is not None else "empty")

    # ── run ──────────────────────────────────────────────────────────────────
    def start(self) -> None:
        if self._refuse("start"):
            return
        self._halt.clear()
        self._abort_soft.clear()
        self._halted = False
        self._set_state("running", "start")
        o = self._opts
        self._submit(lambda: self._do_run(o.start_stop, o.stop_after), "run", "error")

    def resume(self) -> None:
        if self._refuse("resume"):
            return
        prev = self._state
        self._set_state("running", "resume")
        self._submit(lambda: self._do_resume(prev), "resume", prev)

    def _resume_refusal(self) -> str | None:
        seq = self._seq
        if seq.held is not None:
            return (f"a stone may be in the jaws ({seq.held.get('from')} slot {seq.held.get('slot')}) - take it out, "
                    "park the arm, then 'Jaws empty'")
        if seq.pose_status != "ok":
            return f"the ARES pose is {seq.pose_status} ({seq.pose_src}) - 'Confirm pose' or 'Set pose'"
        if self.mode == "real":
            if self._ads_status_fn() is None or not self._ads_connected_fn():
                return "cannot verify that ARES did not move: the HMI's ADS worker is not connected"
            if self._odom_at_stop is None:
                return "the PLC odometry at the stop is unknown - 'Set pose' after checking ARES on the floor"
            d = self.odom_moved()
            if d is not None:
                return (f"ARES moved by ({d[0]:+.1f} mm, {d[1]:+.1f} mm, {d[2]:+.2f} deg) since the run stopped - "
                        "'Apply odometry' or 'Set pose'")
            if isinstance(self._rig, RealRig):
                self._rig.check_ads()
            self._preflight = self._report()
            self.preflight_done.emit(self._preflight)
            if not self._preflight.ok:
                return f"preflight: {self._preflight.n_blocking} blocking problems (see the list)"
        return None

    def _do_resume(self, prev: str) -> None:
        reason = self._resume_refusal()
        if reason is not None:
            self.message.emit("warning", f"resume refused: {reason}")
            self._set_state(prev, f"resume refused: {reason}")
            return
        seq = self._seq
        self._halt.clear()
        self._abort_soft.clear()
        self._halted = False
        if seq.paused:
            seq.resume()
        self._do_run(seq.stop_k if seq.stop_k is not None else self._opts.start_stop, self._opts.stop_after)

    def _do_run(self, start_stop: int, stop_after: int | None) -> None:
        seq, o = self._seq, self._opts
        t0 = time.time()
        if self._t_run is None:
            self._t_run = t0
        exc: BaseException | None = None
        try:
            seq.run(start_stop, stop_after)
        except SequencerError as e:
            exc = e
        except Exception as e:          # noqa: BLE001 - a controller-level failure ends the run as "error"
            log.exception("run failed")
            exc = e
        state = seq.result.state
        if exc is not None and (not isinstance(exc, SequencerError) or state not in END_STATES):
            state = "error"
        if self._tracker is not None:
            self._emit_snapshot(self._tracker.snapshot(seq))
        if o.mode == "real":
            self._odom_at_stop = odom_from_status(self._ads_status_fn())
        if state == "error" or self._halted:
            self._step = True                       # the resume goes step by step (the operator may switch it off)
            o.step = True
        self._write_summary(t0, state, exc)
        detail = seq.result.error or (f"{type(exc).__name__}: {exc}" if exc else "")
        if self._halted and state != "done":
            detail = "HALT - " + detail
        self._set_state(state, detail)

    def _write_summary(self, t0: float, state: str, exc: BaseException | None) -> None:
        seq, o, s = self._seq, self._opts, self._session
        out = {"mode": o.mode, "job": {"name": s.name, "source": s.source, "path": str(s.path) if s.path else None},
               "variant": s.variant, "opts": asdict(o), "state": state, "result": asdict(seq.result),
               "error": f"{type(exc).__name__}: {exc}" if exc else None, "t_start": t0, "t_end": time.time(),
               "t_first_start": self._t_run, "halted": self._halted}
        if o.mode == "sim" and isinstance(self._rig, SimRig):
            out["sim"] = self._rig.world.summary()
        try:
            (seq.log.folder / "hmi_summary.json").write_text(json.dumps(out, indent=1, default=_json_default) + "\n",
                                                            encoding="utf-8")
        except (OSError, TypeError, ValueError) as e:
            self.message.emit("warning", f"hmi_summary.json not written: {e}")

    # ── recovery (run thread) ────────────────────────────────────────────────
    def confirm_pose(self) -> None:
        if not self._refuse("confirm_pose"):
            self._submit(lambda: self._seq.confirm_pose(), "confirm pose")

    def set_pose(self, pose: Pose2D) -> None:
        if not self._refuse("set_pose"):
            self._submit(lambda: self._seq.set_pose(pose, "operator"), "set pose")

    def apply_odometry(self) -> None:
        """REAL: the pose estimate moved by the PLC odometry since the run stopped (ARES jogged while paused)."""
        if self._refuse("apply_odometry"):
            return

        def job() -> None:
            now = odom_from_status(self._ads_status_fn())
            seq = self._seq
            if now is None or self._odom_at_stop is None or seq.pose_est is None:
                raise RuntimeError("no odometry or pose estimate")
            seq.set_pose(compose_odometry(seq.pose_est, self._odom_at_stop, now), "operator: odometry since the stop")
            self._odom_at_stop = now
        self._submit(job, "apply odometry")

    def clear_held(self) -> None:
        """The operator confirms empty jaws (stone taken out, arm parked); SIM: the simulated jaws are emptied."""
        if self._refuse("clear_held"):
            return

        def job() -> None:
            self._seq.clear_held()
            if isinstance(self._rig, SimRig):
                self._rig.world.robot.holding = None
        self._submit(job, "jaws empty")

    def grab(self) -> None:
        """REAL, no run active: one camera frame -> shot (rec None)."""
        if self._refuse("grab"):
            return

        def job() -> None:
            frame = self._rig.camera.grab()
            self.shot.emit(self._shot_view(frame.image, None))
        self._submit(job, "grab")

    # ── immediate controls (GUI thread) ──────────────────────────────────────
    def _release_pending(self, ok: bool, only_if_empty_jaws: bool = False) -> None:
        with self._confirm_lock:
            req = self._pending
            if req is None:
                return
            if only_if_empty_jaws and (req.kind != "motion" or (self._seq is not None and self._seq.held)):
                return
            self._answer = ok
            self._answer_evt.set()

    def answer_confirm(self, req_id: int, ok: bool) -> None:
        with self._confirm_lock:
            if self._pending is None or self._pending.id != int(req_id):
                return
            self._answer = bool(ok)
            self._answer_evt.set()

    def pause(self) -> None:
        """Acts at the next motion boundary with empty jaws (the place of a held stone still runs)."""
        if self._refuse("pause"):
            return
        self._seq.pause()
        self._set_state("pausing", "after this place" if self._seq.held else "")
        self._release_pending(False, only_if_empty_jaws=True)

    def abort(self) -> None:
        """Soft abort: the next confirmation with empty jaws returns False (run ends 'aborted', resumable)."""
        if self._refuse("abort"):
            return
        self._abort_soft.set()
        self._set_state("aborting", "after this place" if self._seq is not None and self._seq.held else "")
        self._release_pending(False, only_if_empty_jaws=True)

    def halt(self) -> None:
        """HALT (after the MainWindow's AdsWorker.halt()): abort flag, pending confirmation released with False, REAL:
        the UR program stopped in the helper thread "mauer-halt". Returns at once."""
        self._halt.set()
        self._release_pending(False)
        if self._rig is None:
            return
        self._halted = True
        if isinstance(self._rig, RealRig):
            rig = self._rig
            threading.Thread(target=self._halt_helper, args=(rig,), name="mauer-halt", daemon=True).start()
        self.message.emit("warning", "HALT: run stopped" + (" - jaws may hold a stone" if self._seq is not None
                                                             and self._seq.held else ""))

    def _halt_helper(self, rig: RealRig) -> None:
        text = rig.halt(bool(self._ads_connected_fn()))
        self.message.emit("warning", f"HALT (REAL): {text}")

    def set_step(self, on: bool) -> None:
        self._step = bool(on)
        if self._opts is not None:
            self._opts.step = bool(on)

    def set_sim_step_s(self, s: float) -> None:
        self._sim_step_s = max(0.0, float(s))

    def set_shot_processor(self, fn: ShotProcessor | None) -> None:
        self._processor = fn

    def ur_source(self) -> UrSource:
        """A new UrSource on the current rig (one per consumer thread)."""
        return UrSource(self._rig_info, self._session.job if self._session is not None else None)

    def shutdown(self, timeout_s: float = 10.0) -> bool:
        """Blocking (MainWindow.closeEvent only): release a pending confirmation, soft abort, let the run thread
        finish, close the Sequencer and the rig, stop the run thread. True if everything ended in time."""
        self._closing = True
        self._abort_soft.set()
        self._release_pending(False)
        done = threading.Event()

        def fin() -> None:
            try:
                if self._seq is not None or self._rig is not None:
                    self._do_release()
            finally:
                done.set()
        self._worker.submit(fin)
        self._worker.stop()
        ok = done.wait(timeout_s)
        self._worker.join(timeout=1.0)
        return ok and not self._worker.is_alive()

    # ── sequencer callbacks (run thread) ─────────────────────────────────────
    def _confirm_cb(self, desc: str) -> bool:
        seq = self._seq
        if self._halt.is_set() or self._closing:
            return False
        if self._abort_soft.is_set() and seq.held is None:        # soft abort: only with empty jaws
            return False
        if self.mode == "sim" and self._sim_step_s > 0 and self._halt.wait(self._sim_step_s):
            return False                                          # SIM pacing, interruptible by HALT
        if (seq.paused or self._abort_soft.is_set()) and seq.held is None:
            return False                                          # pressed during the pacing wait
        if not self._step:
            return True
        return self._ask("motion", desc)

    def _ask(self, kind: str, text: str) -> bool:
        """Blocks the run thread (never the GUI) until the operator answers or HALT releases the request."""
        seq = self._seq
        req = ConfirmRequest(next(self._ids), kind, text, dict(seq.held) if seq.held else None,
                             bool(seq.paused or self._abort_soft.is_set()), time.time())
        with self._confirm_lock:
            self._pending, self._answer = req, None
            self._answer_evt.clear()
        self.confirm_requested.emit(req)
        while not self._answer_evt.wait(0.1):
            if self._halt.is_set() or self._closing:
                break
        with self._confirm_lock:
            ok = bool(self._answer) and not self._halt.is_set() and not self._closing
            self._pending = None
        self.confirm_cleared.emit(req.id)
        return ok

    def _station_empty_cb(self) -> None:
        if self.mode == "sim":
            self._rig.world.refill_station()
            return
        if not self._ask("station_empty", "Pick-up station empty or short of the next stone types: refill every "
                                          "slot, then press Refilled"):
            self._seq.log.write("declined", what="station refill")
            raise SequencerAborted("station refill declined", self._seq.stop_k)

    def _on_record(self, rec: dict) -> None:
        """RunLog listener (writer's thread): every record -> event; run-thread records also -> a new snapshot."""
        self.event.emit(rec)
        if threading.get_ident() != self._worker.ident or self._tracker is None or self._seq is None:
            return
        self._emit_snapshot(self._tracker.record(rec, self._seq))

    def _shot_view(self, image: np.ndarray, rec: dict | None):
        cfg = self._session.cfg if self._session is not None else {}
        if self._processor is not None:
            try:
                return self._processor(image, rec, cfg)
            except Exception as e:      # noqa: BLE001 - the display falls back to the plain preview
                log.warning("shot processor failed: %s", e)
        return default_shot_view(image, rec, cfg, self._scale)

    def _on_shot(self, image: np.ndarray, rec: dict) -> None:
        self.shot.emit(self._shot_view(image, rec))
