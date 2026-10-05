"""Sequencer: runs a job (mauer.job) against the robot, ARES and camera backends (mauer.backends) - the same code for
the real hardware and the pure-Python simulation (mauer.simworld).

Per stop:
  (a) ARES relative move to the stop - CLOSED LOOP: from the ARES pose measured at the previous stop (wall fit) the
      body-frame move that reaches this stop's NOMINAL pose is commanded; a rotation first only when the heading error
      exceeds `rotate_threshold_deg` (>= the PLC minimum 0.2 deg); the deviation from the nominal move is capped at
      `max_correction_mm`. Interlock before every ARES command: robot parked and idle, run not paused.
  (b) look poses -> images -> ChArUco board poses (vision.detect.measure) -> T_base_board = T_base_flange (at the
      exposure) @ T_flange_cam (hand-eye) @ T_cam_board -> reference.fit_frame(parent "wall") -> T_base_wall. Checks:
      every expected board seen (else MeasurementError, arm parked, run paused), fit residual RMS, measured ARES pose
      vs predicted (> max_jump = FrameJumpError, run stops). If ARES is farther than `stop_tol_mm` (or the heading
      beyond the threshold) from the nominal stop pose: a correction move and a new measurement (max_corrections).
  (c) per stone: magazine empty -> reload(); pick from the magazine slot (ARES frame, fixed to the UR base); place at
      pose_trans(T_base_wall_measured, T_wall_tcp) (mauer.ur.script place_stone on the real robot).
  (d) park.
reload(): park; route to the station dock pose (translate in the body frame, rotate at the dock - away from the wall;
  obstacles on the way are the operator's responsibility, logged as a warning); measure the station boards -> station
  frame (correction move if the dock error exceeds dock_tol_mm); move stones station -> magazine; park; route back
  (rotate at the dock, then translate; closed loop from the measured station pose); re-measure the wall (with
  corrections) before the next stone.
Rotations of more than `rotate_keep_dir_deg` keep the direction of the previous rotation (E003: ~1.2 deg loss after
reversing the rotation direction, mauer/ares/ads.py AresAds.rotate).

camera_loop=False runs the same job by pure dead reckoning (nominal moves, nominal frames, no images) - the
comparison case for the simulation.

Every measurement and decision goes to a JSON-lines run log (log_dir/run.jsonl, default data/runs/<timestamp>/),
optionally with the images (save_images). Step mode: `confirm(description) -> bool` is called before every motion
(each ARES command, each robot program); False aborts the run (SequencerAborted).

Real runs: `preflight_real()` refuses to start while safety-relevant values are PLACEHOLDER/unknown and lists all
problems at once (tools/run_job.py --real; there is no override for real runs).

Parameters: [sequencer] in config/station.toml when present, else the ASSUMPTION defaults of SequencerParams.
Conventions: docs/ARCHITECTURE.md (T_a_b, mm, rad; angles in deg only in log text and config names *_deg).
"""
from __future__ import annotations

import datetime as _dt
import json
import logging
import math
import time
from dataclasses import asdict, dataclass, field, fields, replace
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from . import REPO
from . import geometry as g
from .ares.ads import MIN_MOVE_DEG, MIN_MOVE_MM
from .ares.ads import AresError as AdsError
from .backends import BackendError, RobotError, ShotError
from .job import Job, Look, SlotState, StoneTask
from .reference import (FrameFit, Pose2D, T_base_parent_from, T_parent_ares, after_rotation, after_translation,
                        fit_frame, placements, planar_T, relative_move, tilt_deg, wrap_angle)
from .vision.detect import measure
from .vision.targets import board_specs

log = logging.getLogger("mauer.sequencer")


# ── errors ────────────────────────────────────────────────────────────────────
class SequencerError(RuntimeError):
    """The run stopped. `stop` / `stone` say where; the run log has the details."""

    def __init__(self, msg: str, stop: int | None = None, stone: tuple | None = None):
        super().__init__(msg)
        self.stop, self.stone = stop, stone


class InterlockError(SequencerError):
    """ARES move refused: robot not parked / not idle, or the run is paused."""


class MeasurementError(SequencerError):
    """Expected board not seen, no usable image, or the frame fit is inconsistent. The arm is parked and the run
    paused - check the view (occlusion, lighting, board), then resume()."""


class FrameJumpError(SequencerError):
    """Measured frame differs from the prediction by more than the limit - stop (wrong board, slip, bad hand-eye)."""


class SequencerAborted(SequencerError):
    """Step mode: the operator declined a motion."""


class SequencerPaused(SequencerError):
    """pause() was called; resume() to continue."""


class PreflightError(SequencerError):
    def __init__(self, problems: Sequence[str]):
        self.problems = list(problems)
        super().__init__("real run refused:\n  - " + "\n  - ".join(self.problems))


# ── parameters ────────────────────────────────────────────────────────────────
@dataclass
class SequencerParams:
    """Decision limits - all ASSUMPTIONS (requested as [sequencer] keys; the defaults are used until they exist)."""
    rotate_threshold_deg: float = 0.5      # heading error that triggers a rotation: 2.5x the PLC minimum 0.2 deg,
                                           # ~3.5 sigma of the E003 rotation scatter 0.14 deg
    rotate_keep_dir_deg: float = 150.0     # larger rotations keep the previous direction (E003 reversal loss 1.2 deg)
    stop_tol_mm: float = 20.0              # ARES position error at a stop that triggers a correction (reach table
                                           # grid 20 mm; boards stay in view for +-35 mm, README Camera)
    max_corrections: int = 2               # correction moves per arrival
    max_correction_mm: float = 100.0       # cap of (closed-loop move - nominal move)
    max_jump_mm: float = 50.0              # measured vs predicted ARES pose at the wall -> FrameJumpError
    max_jump_deg: float = 3.0
    station_max_jump_mm: float = 100.0     # at the station (nominal station location is a PLACEHOLDER)
    station_max_jump_deg: float = 5.0
    max_fit_rms_mm: float = 1.0            # corner residual RMS of the frame fit -> MeasurementError
    dock_tol_mm: float = 30.0              # dock position error that triggers a correction move
    require_all_boards: bool = True        # every expected board of a look must be measured

    @classmethod
    def from_config(cls, cfg: Mapping) -> "SequencerParams":
        s = cfg.get("sequencer", {}) or {}
        kw = {}
        for f in fields(cls):
            if f.name in s:
                kw[f.name] = type(f.default)(s[f.name])
        return cls(**kw)


# ── run log ───────────────────────────────────────────────────────────────────
def _jsonable(x: Any) -> Any:
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, set)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    if isinstance(x, np.generic):
        return _jsonable(x.item())
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, Pose2D):
        return x.to_dict()
    return x


class RunLog:
    """JSON-lines log: one object per line {"t": now(), "event": ..., ...}; flushed after every line."""

    def __init__(self, folder: Path, now: Callable[[], float] = time.time):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / "run.jsonl"
        self.now = now
        self._f = open(self.path, "a", encoding="utf-8")
        self.n_images = 0

    def write(self, event: str, **data: Any) -> dict:
        rec = {"t": self.now(), "event": event, **_jsonable(data)}
        self._f.write(json.dumps(rec) + "\n")
        self._f.flush()
        return rec

    def image(self, img: np.ndarray, name: str) -> str:
        from .vision.dataset import write_png
        d = self.folder / "images"
        d.mkdir(exist_ok=True)
        self.n_images += 1
        rel = f"images/{self.n_images:04d}_{name}.png"
        write_png(self.folder / rel, img)
        return rel

    def close(self) -> None:
        if not self._f.closed:
            self._f.close()


def read_log(path: str | Path) -> list[dict]:
    p = Path(path)
    if p.is_dir():
        p = p / "run.jsonl"
    return [json.loads(ln) for ln in p.read_text(encoding="utf-8").splitlines() if ln.strip()]


@dataclass
class RunResult:
    state: str = "idle"                    # idle | running | done | paused | error | aborted
    stops_done: list[int] = field(default_factory=list)
    placed: list[tuple] = field(default_factory=list)
    reloads: int = 0
    ares_moves: int = 0
    corrections: int = 0
    measurements: list[dict] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    error: str | None = None
    log_path: str = ""


# ── real-run preflight ────────────────────────────────────────────────────────
def preflight_real(cfg: Mapping, job: Job, *, intrinsics_file: str | Path | None = None,
                   handeye_file: str | Path | None = None, config_path: str | Path | None = None) -> list[str]:
    """Every reason not to run this job on the real robot (empty list = ok). Lists all problems at once."""
    from .job import config_sha256
    p: list[str] = []
    u, b, v = cfg.get("ur", {}), cfg.get("brick", {}), cfg.get("vision", {})
    if not str(u.get("host", "")).strip():
        p.append("[ur] host is empty (PLACEHOLDER) - set the UR5 IP")
    tool = float(u.get("payload_tool_kg", 0.0) or 0.0)
    if tool <= 0.0:
        p.append("[ur] payload_tool_kg <= 0 (PLACEHOLDER: gripper + adapter + camera + bracket unknown) - weigh it")
    stone = float(b.get("mass_kg", 0.0) or 0.0)
    if stone <= 0.0:
        p.append("[brick] mass_kg <= 0 (UNKNOWN) - weigh a stone")
    if tool > 0.0 and stone > 0.0 and tool + stone > 5.0:
        p.append(f"payload {tool + stone:.2f} kg (tool + stone) exceeds the UR5 rated payload 5 kg")
    for key, default, what in ((intrinsics_file, v.get("intrinsics_file", "calib/camera_intrinsics.json"),
                                "camera intrinsics"),
                               (handeye_file, v.get("handeye_file", "calib/handeye.json"), "hand-eye calibration")):
        path = Path(key or default)
        path = path if path.is_absolute() else REPO / path
        if not path.exists():
            p.append(f"{what} missing: {path.relative_to(REPO) if path.is_relative_to(REPO) else path} - calibrate "
                     "first (tools/calib_intrinsics.py, tools/handeye_solve.py)")
    m = job.meta
    if m.get("look_source", "nominal") == "nominal" and m.get("reach_check", "none") != "robodk":
        p.append(f"job built from nominal look poses with reach check {m.get('reach_check', 'none')!r} only - no "
                 "collision/occlusion check (export the job from the RoboDK planner)")
    if any(t.qnear_rad is None for t in job.stones()):
        p.append("job has place poses without an IK branch hint (qnear)")
    try:
        sha = config_sha256(config_path)
        if job.config_sha256 and sha != job.config_sha256:
            p.append("config/station.toml changed since the job was built (sha256 differs) - rebuild the job")
    except OSError as e:
        p.append(f"config not readable: {e}")
    for key, val in (("do_grip_open", u.get("do_grip_open")), ("do_grip_close", u.get("do_grip_close"))):
        if val is None:
            p.append(f"[ur] {key} missing")
    return p


# ── sequencer ─────────────────────────────────────────────────────────────────
class Sequencer:
    """Runs `job` on the backends (see the module docstring). intr / T_flange_cam: the CALIBRATED camera model and
    hand-eye result used to turn images into board poses."""

    def __init__(self, job: Job, cfg: Mapping, robot, ares, camera, intr, T_flange_cam: np.ndarray,
                 log_dir: str | Path | None = None, confirm: Callable[[str], bool] | None = None,
                 now: Callable[[], float] = time.time, *, camera_loop: bool = True, save_images: bool = False,
                 on_station_empty: Callable[[], Any] | None = None, params: SequencerParams | None = None):
        self.job, self.cfg = job, cfg
        self.robot, self.ares, self.camera = robot, ares, camera
        self.intr = intr
        self.T_flange_cam = np.asarray(T_flange_cam, float)
        self.confirm, self.now = confirm, now
        self.camera_loop, self.save_images = bool(camera_loop), bool(save_images)
        self.on_station_empty = on_station_empty
        self.p = params or SequencerParams.from_config(cfg)
        self.vcfg = dict(cfg.get("vision", {}))
        self.specs = board_specs(cfg)
        self.placements = placements(cfg)
        self.T_ares_base = np.asarray(job.T_ares_base, float)
        self.T_base_ares = g.inv(self.T_ares_base)
        self.T_wall_station_nominal = np.asarray(job.station.T_wall_station, float)
        self.T_wall_station = self.T_wall_station_nominal.copy()     # estimate, refined from the trips
        self.station_obs: list[Pose2D] = []
        self.anchor: tuple[str, np.ndarray] | None = None            # last camera measurement of the ARES pose
        self.dr = np.eye(4)                                          # commanded moves since then (body frame)
        self.magazine = SlotState.magazine(job.magazine)
        self.station = SlotState.station(job.station)
        self.placed: set[tuple] = set()
        self.pose_est: Pose2D | None = None      # ARES pose in the wall frame (measured or predicted)
        self.pose_src = "none"
        self.T_base_wall: np.ndarray | None = None
        self.paused = False
        self.last_rot_sign = 0.0
        self.stop_k: int | None = None
        self.at_station = False
        if log_dir is None:
            stamp = _dt.datetime.fromtimestamp(now()).strftime("%Y-%m-%d_%H%M%S")
            log_dir = REPO / "data" / "runs" / stamp
        self.log = RunLog(Path(log_dir), now)
        self.result = RunResult(log_path=str(self.log.path))

    # ── control ──────────────────────────────────────────────────────────────
    def pause(self) -> None:
        """Request a pause: the next motion raises SequencerPaused (ARES is interlocked while paused)."""
        self.paused = True
        self.log.write("pause_requested")

    def resume(self) -> None:
        self.paused = False
        self.log.write("resume")

    def _warn(self, msg: str, **data) -> None:
        log.warning(msg)
        self.result.warnings.append(msg)
        self.log.write("warning", msg=msg, **data)

    def _confirm(self, desc: str) -> None:
        if self.paused:
            raise SequencerPaused("run paused", self.stop_k)
        if self.confirm is not None and not self.confirm(desc):
            self.log.write("declined", what=desc)
            raise SequencerAborted(f"operator declined: {desc}", self.stop_k)

    # ── robot ────────────────────────────────────────────────────────────────
    def _robot(self, action: str, desc: str, *args) -> Any:
        self._confirm(f"robot: {desc}")
        self.log.write("robot", action=action, what=desc)
        try:
            return getattr(self.robot, action)(*args)
        except RobotError as e:
            self.log.write("robot_error", action=action, what=desc, error=str(e))
            raise SequencerError(f"robot {action} failed ({desc}): {e} - the arm is NOT parked; ARES stays "
                                 "interlocked. Inspect, recover the arm (protective stop / stone in the jaws), park "
                                 f"it, then resume from stop {self.stop_k}", self.stop_k) from e

    def _park(self) -> None:
        if not self.robot.is_parked():
            self._robot("park", "park")

    # ── ARES ─────────────────────────────────────────────────────────────────
    def _interlock(self, what: str) -> None:
        problems = []
        if self.paused:
            problems.append("run paused")
        if not self.robot.is_parked():
            problems.append("robot not parked")
        if not self.robot.is_idle():
            problems.append("robot not idle")
        try:
            if hasattr(self.ares, "idle") and not self.ares.idle():
                problems.append("ARES still moving")
        except Exception as e:                       # noqa: BLE001 - a failed status read is a refusal too
            problems.append(f"ARES status unreadable: {e}")
        if problems:
            self.log.write("interlock", what=what, problems=problems)
            raise InterlockError(f"ARES {what} refused: " + ", ".join(problems), self.stop_k)

    def _ares_cmd(self, kind: str, *args) -> Any:
        try:
            out = getattr(self.ares, kind)(*args)
        except (AdsError, BackendError) as e:
            self.log.write("ares_error", kind=kind, args=list(args), error=str(e))
            raise SequencerError(f"ARES {kind}{tuple(round(a, 2) for a in args)} failed: {e}", self.stop_k) from e
        self.result.ares_moves += 1
        ok = bool(getattr(out, "ok", True))
        self.log.write("ares_move", kind=kind, args=list(args), ok=ok,
                       summary=out.summary() if hasattr(out, "summary") else str(out))
        if not ok:
            raise SequencerError(f"ARES {kind} not ok: " + (out.summary() if hasattr(out, "summary") else str(out)),
                                 self.stop_k)
        return out

    def _rotate(self, dtheta: float, why: str) -> None:
        if abs(dtheta) > math.radians(self.p.rotate_keep_dir_deg) and self.last_rot_sign \
                and math.copysign(1.0, dtheta) != self.last_rot_sign:
            dtheta += self.last_rot_sign * 2.0 * math.pi          # the long way round, same direction as before
        deg = math.degrees(dtheta)
        if abs(deg) < MIN_MOVE_DEG:
            return
        self._interlock(f"rotate {deg:+.2f} deg")
        self._confirm(f"ARES rotate {deg:+.2f} deg ({why})")
        self._ares_cmd("rotate", deg)
        self.last_rot_sign = math.copysign(1.0, dtheta)
        self.dr = self.dr @ planar_T(0.0, 0.0, dtheta)
        self.pose_est, self.pose_src = after_rotation(self.pose_est, dtheta), "predicted"

    def _translate(self, dx: float, dy: float, why: str) -> None:
        if math.hypot(dx, dy) < MIN_MOVE_MM:
            return
        self._interlock(f"translate ({dx:+.1f}, {dy:+.1f}) mm")
        self._confirm(f"ARES translate dx {dx:+.1f} mm, dy {dy:+.1f} mm ({why})")
        self._ares_cmd("translate", dx, dy)
        self.dr = self.dr @ planar_T(dx, dy, 0.0)
        self.pose_est, self.pose_src = after_translation(self.pose_est, dx, dy), "predicted"

    def _drive_to(self, target: Pose2D, nominal_from: Pose2D | None, why: str, rotate_first: bool = True) -> None:
        """Closed-loop relative move from pose_est to target (body frame of pose_est). rotate_first: rotation, then
        translation (stops: heading correction first); else translation, then rotation at the target.
        nominal_from: where the move nominally starts - the deviation of pose_est from it (= the correction) is
        capped at max_correction_mm. Does NOT park the arm: the caller parks first, the interlock refuses otherwise."""
        start = self.pose_est
        if nominal_from is not None:
            ex, ey = nominal_from.x_mm - start.x_mm, nominal_from.y_mm - start.y_mm
            corr = math.hypot(ex, ey)
            if corr > self.p.max_correction_mm:
                f = 1.0 - self.p.max_correction_mm / corr
                target = Pose2D(target.x_mm - ex * f, target.y_mm - ey * f, target.theta_rad)
                self._warn(f"{why}: correction {corr:.1f} mm capped at {self.p.max_correction_mm:g} mm")
        dx, dy, dth = relative_move(start, target)
        self.log.write("drive", why=why, start=start, start_src=self.pose_src, target=target, dx_mm=dx, dy_mm=dy,
                       dtheta_deg=math.degrees(dth))
        if rotate_first:
            if abs(math.degrees(dth)) > self.p.rotate_threshold_deg:
                self._rotate(dth, why)
                dx, dy, _ = relative_move(self.pose_est, target)
            self._translate(dx, dy, why)
        else:
            self._translate(dx, dy, why)
            _, _, dth = relative_move(self.pose_est, target)
            if abs(math.degrees(dth)) > self.p.rotate_threshold_deg:
                self._rotate(dth, why)

    # ── station location estimate ────────────────────────────────────────────
    def _anchor(self, parent: str, T_parent_ares: np.ndarray) -> None:
        """A camera measurement of the ARES pose; observes the station location when the previous one was taken in
        the other frame (dead-reckoned commanded moves in between)."""
        if self.anchor is not None and self.anchor[0] != parent:
            T_prev_pred = self.anchor[1] @ self.dr                  # previous frame, predicted to now
            if parent == "station":
                T_ws = T_prev_pred @ g.inv(T_parent_ares)           # wall->ares(pred) . ares->station(meas)
            else:
                T_ws = T_parent_ares @ g.inv(T_prev_pred)           # wall->ares(meas) . ares->station(pred)
            self._observe_station(Pose2D.from_T(T_ws))
        self.anchor = (parent, np.asarray(T_parent_ares, float).copy())
        self.dr = np.eye(4)

    def _observe_station(self, obs: Pose2D) -> None:
        """Station location = component-wise median of all observations (robust to a slip on one trip); starts from
        the nominal [pickup_station] pose (PLACEHOLDER)."""
        self.station_obs.append(obs)
        nom = Pose2D.from_T(self.T_wall_station_nominal)
        x = float(np.median([o.x_mm for o in self.station_obs]))
        y = float(np.median([o.y_mm for o in self.station_obs]))
        th = nom.theta_rad + float(np.median([wrap_angle(o.theta_rad - nom.theta_rad) for o in self.station_obs]))
        T = planar_T(x, y, th)
        T[2, 3] = self.T_wall_station_nominal[2, 3]
        self.T_wall_station = T
        d_mm, d_deg = nom.delta(Pose2D.from_T(T))
        self.log.write("station_estimate", observation=obs, n=len(self.station_obs), estimate=Pose2D.from_T(T),
                       vs_nominal_mm=d_mm, vs_nominal_deg=d_deg)

    # ── measurement ──────────────────────────────────────────────────────────
    def _aim(self, lk: Look, T_parent_base_nom: np.ndarray, T_parent_base_est: np.ndarray) -> Look:
        """Look pose re-aimed for the estimated ARES pose (the job's T_base_flange is for the nominal pose); joint
        targets (q_rad) are used as they are."""
        if lk.q_rad is not None or lk.T_base_flange is None:
            return lk
        return replace(lk, T_base_flange=g.inv(T_parent_base_est) @ T_parent_base_nom @ lk.T_base_flange)

    def _measure(self, parent: str, looks: Sequence[Look], what: str, T_parent_ares_nom: np.ndarray,
                 T_parent_ares_est: np.ndarray) -> FrameFit:
        """Shoot every look (re-aimed with the current estimate, refined after every measured board), retry the
        looks of missing boards once with the refined estimate, fit the parent frame."""
        observed: dict[str, list[np.ndarray]] = {}
        missing: dict[str, str] = {}
        T_nom = np.asarray(T_parent_ares_nom, float) @ self.T_ares_base
        T_est = np.asarray(T_parent_ares_est, float) @ self.T_ares_base

        def shoot(lk: Look, tag: str) -> None:
            nonlocal T_est
            aimed = self._aim(lk, T_nom, T_est)
            self._robot("goto_look", f"look {lk.name}{tag} ({', '.join(lk.boards)})", aimed)
            try:
                shot = self.robot.shot(self.camera)
            except ShotError as e:
                raise self._fail_measurement(f"{what}: no usable image at look {lk.name}: {e}") from e
            img_rel = self.log.image(shot.frame.image, lk.name + tag.replace(" ", "_")) if self.save_images else None
            poses = measure(shot.frame.image, [self.specs[b] for b in lk.boards], self.intr, self.vcfg)
            boards = {}
            for b in lk.boards:
                bp = poses[b]
                if bp.ok:
                    T = shot.T_base_flange @ self.T_flange_cam @ bp.T_cam_board
                    observed.setdefault(b, []).append(T)
                    missing.pop(b, None)
                    boards[b] = {"ok": True, "n_corners": bp.n_corners, "rms_px": bp.rms_px, "T_base_board": T}
                else:
                    if b not in observed:
                        missing[b] = f"look {lk.name}{tag}: {bp.reason}"
                    boards[b] = {"ok": False, "reason": bp.reason, "n_corners": bp.n_corners}
            self.log.write("shot", parent=parent, look=lk.name + tag, T_base_flange=shot.T_base_flange,
                           image=img_rel, max_qd=getattr(shot, "max_qd", None), boards=boards)
            if any(v["ok"] for v in boards.values()):
                obs = {b: Ts[-1] for b, Ts in observed.items()}
                T_est = g.inv(fit_frame(obs, self.placements, self.specs, parent).T_base_parent)

        for lk in looks:
            shoot(lk, "")
        if missing:
            for lk in [lk for lk in looks if any(b in missing for b in lk.boards)]:
                shoot(lk, " retry")
        if missing and (self.p.require_all_boards or not observed):
            raise self._fail_measurement(f"{what}: board(s) not measured: "
                                         + "; ".join(f"{b} ({r})" for b, r in sorted(missing.items())))
        obs = {b: (Ts[0] if len(Ts) == 1 else g.average_T(Ts)) for b, Ts in observed.items()}
        fit = fit_frame(obs, self.placements, self.specs, parent)
        if fit.rms_mm > self.p.max_fit_rms_mm:
            raise self._fail_measurement(f"{what}: frame fit residual RMS {fit.rms_mm:.2f} mm > "
                                         f"{self.p.max_fit_rms_mm:g} mm (boards {fit.boards}) - a board moved or the "
                                         "hand-eye calibration is off", fit=fit.to_dict())
        return fit

    def _fail_measurement(self, msg: str, **data) -> MeasurementError:
        """Log, park the arm (the safe reaction), pause; returns the MeasurementError for the caller to raise.
        A failure while parking propagates instead (SequencerError, arm not parked)."""
        self.log.write("measurement_failed", msg=msg, **data)
        self.paused = False                        # parking is allowed (and confirmed in step mode)
        try:
            self._park()
        finally:
            self.paused = True
        return MeasurementError(msg + " - arm parked, run paused (check the view, then resume())", self.stop_k)

    def _check_jump(self, meas: Pose2D, pred: Pose2D, lim_mm: float, lim_deg: float, what: str) -> tuple:
        d_mm, d_deg = pred.delta(meas)
        if d_mm > lim_mm or d_deg > lim_deg:
            self.log.write("frame_jump", what=what, measured=meas, predicted=pred, d_mm=d_mm, d_deg=d_deg)
            raise FrameJumpError(f"{what}: measured ARES pose {meas.describe()} differs from the predicted "
                                 f"{pred.describe()} by {d_mm:.1f} mm / {d_deg:.2f} deg (limit {lim_mm:g} mm / "
                                 f"{lim_deg:g} deg) - wrong board, large slip or bad calibration; run stopped",
                                 self.stop_k)
        return d_mm, d_deg

    def _measure_wall(self, k: int, why: str) -> None:
        stop = self.job.stops[k]
        if not self.camera_loop:
            self.T_base_wall = T_base_parent_from(self.pose_est, self.T_ares_base)
            self.log.write("wall_frame", stop=k, why=why, source="dead_reckoning", ares=self.pose_est)
            self._park()
            return
        # after a station trip the prediction rests on the station estimate: the station limits apply
        lim = ((self.p.station_max_jump_mm, self.p.station_max_jump_deg) if why == "after reload"
               else (self.p.max_jump_mm, self.p.max_jump_deg))
        for attempt in range(self.p.max_corrections + 1):
            pred = self.pose_est
            fit = self._measure("wall", stop.looks, f"stop {k} wall", stop.ares.T, pred.T)
            T_wa = T_parent_ares(fit.T_base_parent, self.T_ares_base)
            meas = Pose2D.from_T(T_wa)
            d_mm, d_deg = self._check_jump(meas, pred, *lim, f"stop {k} wall")
            self._anchor("wall", T_wa)
            self.pose_est, self.pose_src = meas, "measured"
            self.T_base_wall = fit.T_base_parent
            e_mm, e_deg = stop.ares.delta(meas)
            rec = {"kind": "wall", "stop": k, "why": why, "attempt": attempt, "boards": fit.boards,
                   "rms_mm": fit.rms_mm, "max_mm": fit.max_mm, "baseline_mm": fit.baseline_mm, "measured": meas,
                   "predicted": pred, "jump_mm": d_mm, "jump_deg": d_deg, "err_nominal_mm": e_mm,
                   "err_nominal_deg": e_deg, "tilt_deg": tilt_deg(T_wa)}
            self.result.measurements.append(rec)
            self.log.write("wall_frame", source="camera", T_base_wall=fit.T_base_parent, fit=fit.to_dict(), **rec)
            lim = (self.p.max_jump_mm, self.p.max_jump_deg)
            if e_mm <= self.p.stop_tol_mm and e_deg <= self.p.rotate_threshold_deg:
                break
            if attempt == self.p.max_corrections:
                self._warn(f"stop {k}: ARES still {e_mm:.1f} mm / {e_deg:.2f} deg from the nominal stop after "
                           f"{attempt} corrections - continuing with the measured frame (check reach)")
                break
            self.result.corrections += 1
            self._park()
            self._drive_to(stop.ares, None, f"stop {k} correction {attempt + 1}")
        self._park()

    # ── magazine / reload ────────────────────────────────────────────────────
    def dock_in_wall(self) -> Pose2D:
        """Docking pose in the wall frame with the current station location estimate."""
        return Pose2D.from_T(self.T_wall_station @ self.job.station.dock.T)

    def _reload(self, k: int) -> None:
        stop = self.job.stops[k]
        st = self.job.station
        self.result.reloads += 1
        n_need = self.magazine.n_free
        if self.station.empty():
            self.log.write("station_empty")
            if self.on_station_empty is None:
                raise SequencerError("pick-up station is empty - refill it (operator), then resume", k)
            self.on_station_empty()
            self.station = SlotState.station(st)
        self.log.write("reload_start", stop=k, magazine=len(self.magazine), station=len(self.station))
        self._park()
        dock = self.dock_in_wall()
        dist = math.hypot(dock.x_mm - self.pose_est.x_mm, dock.y_mm - self.pose_est.y_mm)
        self._warn(f"route to the pick-up station: ARES drives {dist:.0f} mm and turns "
                   f"{abs(math.degrees(wrap_angle(dock.theta_rad - self.pose_est.theta_rad))):.0f} deg - keeping the "
                   "path clear is the operator's responsibility")
        self.at_station = True
        self._drive_to(dock, stop.ares, f"stop {k} -> station", rotate_first=False)
        T_base_station = self._measure_station(k)
        n = min(self.magazine.n_free, len(self.station))
        for _ in range(n):
            sid, mid = self.station.next_take(), self.magazine.next_fill()
            if sid is None or mid is None:
                break
            self._robot("pick_station", f"pick station slot {sid}", T_base_station, st.slot(sid))
            self.station.take(sid)
            self._robot("place_magazine", f"place magazine slot {mid}", self.job.magazine.slot(mid), self.T_base_ares)
            self.magazine.fill(mid)
        self._park()
        self.log.write("reload_transfer", moved=n, wanted=n_need, magazine=len(self.magazine),
                       station=len(self.station))
        self._drive_to(stop.ares, self.dock_in_wall(), f"station -> stop {k}", rotate_first=True)
        self.at_station = False
        self._measure_wall(k, "after reload")
        self.log.write("reload_done", stop=k)

    def _measure_station(self, k: int) -> np.ndarray:
        st = self.job.station
        if not self.camera_loop:
            T_st_ares = g.inv(self.T_wall_station) @ self.pose_est.T
            self.log.write("station_frame", source="dead_reckoning", ares_station=Pose2D.from_T(T_st_ares))
            return T_base_parent_from(Pose2D.from_T(T_st_ares), self.T_ares_base)
        T_base_station = None
        for attempt in range(self.p.max_corrections + 1):
            T_st_pred = g.inv(self.T_wall_station) @ self.pose_est.T
            pred_st = Pose2D.from_T(T_st_pred)
            fit = self._measure("station", st.looks, f"stop {k} station", st.dock.T, T_st_pred)
            T_base_station = fit.T_base_parent
            T_sa = T_parent_ares(fit.T_base_parent, self.T_ares_base)
            meas_st = Pose2D.from_T(T_sa)
            d_mm, d_deg = self._check_jump(meas_st, pred_st, self.p.station_max_jump_mm, self.p.station_max_jump_deg,
                                           "station")
            self._anchor("station", T_sa)
            self.pose_est = Pose2D.from_T(self.T_wall_station @ meas_st.T)     # via the station location estimate
            self.pose_src = "measured (station)"
            e_mm, e_deg = st.dock.delta(meas_st)
            rec = {"kind": "station", "stop": k, "attempt": attempt, "boards": fit.boards, "rms_mm": fit.rms_mm,
                   "baseline_mm": fit.baseline_mm, "measured": meas_st, "predicted": pred_st, "jump_mm": d_mm,
                   "jump_deg": d_deg, "err_nominal_mm": e_mm, "err_nominal_deg": e_deg}
            self.result.measurements.append(rec)
            self.log.write("station_frame", source="camera", T_base_station=T_base_station, fit=fit.to_dict(), **rec)
            if e_mm <= self.p.dock_tol_mm or attempt == self.p.max_corrections:
                if e_mm > self.p.dock_tol_mm:
                    self._warn(f"dock error {e_mm:.1f} mm after {attempt} corrections - picking relative to the "
                               "measured station frame anyway")
                break
            self.result.corrections += 1
            self._park()
            self._drive_to(self.dock_in_wall(), None, f"dock correction {attempt + 1}")
        self._park()
        return T_base_station

    # ── stones ───────────────────────────────────────────────────────────────
    def _place(self, k: int, t: StoneTask) -> None:
        if self.magazine.empty():
            self._reload(k)
        mid = self.magazine.next_take(t.slot)
        if mid is None:
            raise SequencerError("magazine empty after a reload (station empty?)", k, t.key)
        if t.slot is not None and mid != t.slot:
            self.log.write("slot_changed", stone=t.key, planned=t.slot, used=mid)
        self._robot("pick_magazine", f"pick magazine slot {mid}", self.job.magazine.slot(mid), self.T_base_ares)
        self.magazine.take(mid)
        T_cmd = self.T_base_wall @ t.T_wall_tcp
        self._robot("place_wall", f"place stone {t.label} (u {t.u_mm:.0f} mm, top {t.z_top_mm:.0f} mm)",
                    self.T_base_wall, t)
        self.placed.add(t.key)
        self.result.placed.append(t.key)
        self.log.write("placed", stop=k, stone=t.key, slot=mid, T_base_tcp=T_cmd, frame_src=self.pose_src)

    # ── run ──────────────────────────────────────────────────────────────────
    def run(self, start_stop: int = 0, stop_after: int | None = None) -> RunResult:
        """Execute stops start_stop .. stop_after (inclusive; None = to the end). ARES is assumed to stand at the
        nominal pose of start_stop (operator at the start mark, or a resumed run); stones already placed by this
        Sequencer are skipped. Raises SequencerError (subclasses) when the run stops; the result is in .result."""
        n = len(self.job.stops)
        last = n - 1 if stop_after is None else min(int(stop_after), n - 1)
        if not 0 <= start_stop <= last:
            raise ValueError(f"start_stop {start_stop} / stop_after {stop_after} outside 0..{n - 1}")
        if self.paused:
            raise SequencerPaused("run is paused - resume() first", start_stop)
        self.result.state = "running"
        self.result.error = None
        self.log.write("run_start", start_stop=start_stop, last_stop=last, camera_loop=self.camera_loop,
                       params=asdict(self.p), job_source=self.job.source, config_sha256=self.job.config_sha256,
                       n_stones=self.job.n_stones, magazine=len(self.magazine))
        resumed = self.pose_est is not None and self.stop_k == start_stop
        if not resumed:
            self.pose_est, self.pose_src = self.job.stops[start_stop].ares, "assumed (start mark)"
            self.at_station = False
        try:
            self._park()
            if resumed and self.at_station:            # stopped during a reload: back to the stop first
                self._drive_to(self.job.stops[start_stop].ares, self.dock_in_wall(),
                               f"station -> stop {start_stop} (resume)", rotate_first=True)
                self.at_station = False
            for k in range(start_stop, last + 1):
                self.stop_k = k
                stop = self.job.stops[k]
                self.log.write("stop_start", stop=k, a_mm=stop.a_mm, nominal=stop.ares, n_stones=len(stop.stones))
                if k != start_stop:
                    prev = self.job.stops[k - 1].ares
                    self._park()
                    self._drive_to(stop.ares, prev, f"stop {k - 1} -> stop {k}")
                self._measure_wall(k, "arrival")
                for t in stop.stones:
                    if t.key in self.placed:
                        continue
                    self._place(k, t)
                self._park()
                self.result.stops_done.append(k)
                self.log.write("stop_done", stop=k, placed=len(self.placed))
            self.result.state = "done"
            self.log.write("run_done", placed=len(self.placed), reloads=self.result.reloads,
                           ares_moves=self.result.ares_moves, corrections=self.result.corrections)
            return self.result
        except SequencerAborted as e:
            self.result.state, self.result.error = "aborted", str(e)
            self.log.write("run_aborted", error=str(e))
            raise
        except (MeasurementError, SequencerPaused) as e:
            self.result.state, self.result.error = "paused", str(e)
            self.log.write("run_paused", error=str(e), stop=self.stop_k)
            raise
        except SequencerError as e:
            self.result.state, self.result.error = "error", str(e)
            self.log.write("run_error", error=str(e), stop=self.stop_k, kind=type(e).__name__)
            raise
        except BackendError as e:                     # e.g. AresError raised by a backend outside _ares_cmd
            self.result.state, self.result.error = "error", str(e)
            self.log.write("run_error", error=str(e), stop=self.stop_k, kind=type(e).__name__)
            raise SequencerError(str(e), self.stop_k) from e

    def close(self) -> None:
        self.log.close()
