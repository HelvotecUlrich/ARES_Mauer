"""TwinLink: the HMI side of the RoboDK digital twin (docs/HMI_DESIGN.md section 11.3, D-H9).

frame_from() turns what the HMI already has - the latest RunSnapshot, a UrSnapshot, the AresLive pose - into the
twin's TwinFrame (robodk/twin_model.py). TwinLink owns one robodk/twin.py Twin for the current job session: start()
builds it (lazy import - RoboDK is only needed when the twin is switched on), frame() is its frame source and runs in
the twin thread "mauer-twin", reading only atomic references (controller.snapshot, controller.rig, the UrSource of
the current rig, ctx.last_ads_status). Nothing here writes to the run, the rig or RoboDK.

What the twin shows:
  SIM   UR joints and ARES at the simulated TRUE pose (consistent with the true joints), the true station frame;
  REAL  UR joints from RTDE, ARES at the sequencer's estimate (+ the HMI's odometry while a move runs), the estimated
        station frame - the twin is the sequencer's belief;
  both  stones where the sequencer believes they are (magazine, station, nominal wall pose, jaws).
"""
from __future__ import annotations

import logging
import math
import sys
import threading
import time
from typing import Any, Callable

from PySide6.QtCore import QObject, Signal

from mauer import REPO

from .snapshot import RunSnapshot, stop_text
from .sources import ares_live

log = logging.getLogger("hmi.twin")

ROBODK_DIR = REPO / "robodk"


def twin_model():
    """robodk/twin_model.py (pure: numpy and mauer) - robodk/ is a script folder, not a package."""
    if str(ROBODK_DIR) not in sys.path:
        sys.path.insert(0, str(ROBODK_DIR))
    import twin_model as tm
    return tm


def _default_factory(cfg: dict, job, frame_fn, settings, on_status):
    """robodk/twin.py Twin (imports the RoboDK API: only when the twin is started)."""
    twin_model()                                   # puts robodk/ on sys.path
    import twin as rdk_twin
    return rdk_twin.Twin(cfg, job, frame_fn, settings, on_status)


def caption(snap: RunSnapshot | None, ares_src: str) -> str:
    """One line for RoboDK's status bar: run state, stop, stones, held stone, action, where the ARES pose is from."""
    if snap is None:
        return ""
    parts = [f"Mauer {snap.seq_state}"]
    if snap.stop_k is not None:
        parts.append(stop_text(snap.stop_k, snap.n_stops))
    parts.append(f"{snap.n_placed}/{snap.n_stones} placed")
    if snap.held:
        parts.append(f"jaws: {snap.held.get('kind', '?')} stone from {snap.held.get('from', '?')}"
                     + (" (UNKNOWN)" if snap.held.get("unknown") else ""))
    if snap.action:
        parts.append(snap.action)
    parts.append(f"ARES: {ares_src}")
    return " | ".join(parts)


def frame_from(snap: RunSnapshot | None, ur, live, session, world=None):
    """TwinFrame of the current state (pure). snap None: RunSnapshot.initial(session) (before a run); ur None: the
    park pose of the job. SIM (live.true_pose set): ARES at the true pose, the station at world.T_wall_station_true
    (world given); REAL: ARES at live.pose (estimate, + odometry during a move), the estimated station frame."""
    tm = twin_model()
    if snap is None and session is not None:
        snap = RunSnapshot.initial(session)
    q = tuple(ur.q_rad) if ur is not None and ur.q_rad is not None else (
        tuple(session.job.park_q_rad) if session is not None else None)
    sim = live is not None and live.true_pose is not None
    ares, label, status = None, "none", "ok"
    if live is not None and (live.true_pose if sim else live.pose) is not None:
        ares, label = (live.true_pose, "sim truth") if sim else (live.pose, live.src)
        status = "ok" if sim else live.status
    elif snap is not None and snap.pose_est is not None:
        ares, status = snap.pose_est, snap.pose_status
        label = "start mark" if str(snap.pose_src).startswith(("assumed", "start mark")) else "estimate"
    if status not in ("", "ok"):
        label += f" ({status})"
    T_ws = getattr(world, "T_wall_station_true", None) if sim and world is not None else None
    if T_ws is None and snap is not None:
        T_ws = snap.T_wall_station
    t = time.time()
    if ur is not None and ur.source == "rtde" and ur.age_s is not None and math.isfinite(ur.age_s):
        t = float(ur.t) - float(ur.age_s)                     # RTDE sample time
    return tm.TwinFrame(q_rad=q, ares=ares, ares_label=label, T_wall_station=T_ws,
                        stones=tm.StoneState.from_snapshot(snap) if snap is not None else None,
                        caption=caption(snap, label), t=t)


class TwinLink(QObject):
    """See the module docstring. GUI thread, except frame() (twin thread). twin_factory(cfg, job, frame_fn, settings,
    on_status) replaces robodk/twin.py Twin (tests)."""

    status = Signal(str, str)                     # (state, detail) of the current twin, queued into the GUI thread

    def __init__(self, ctx, parent: QObject | None = None, *, twin_factory: Callable[..., Any] | None = None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._factory = twin_factory or _default_factory
        self._twin = None
        self._session = None
        self._ur_src = None
        self._closed = False
        self._lock = threading.Lock()             # _twin swaps vs. the delayed start of a restart
        self._ctl.rig_changed.connect(self._on_rig)
        self.status.connect(self._on_status)

    @property
    def twin(self):
        return self._twin

    @property
    def running(self) -> bool:
        return self._twin is not None

    def start(self) -> bool:
        """(Re)start the twin on the current session; False (and a status) without a session or with bad
        [hmi.twin] settings. A running twin is stopped first (in a helper thread: the new one starts after it)."""
        if self._closed:
            return False
        s = self._ctx.session
        if s is None:
            self.status.emit("off", "load a job first")
            return False
        tm = twin_model()
        try:
            settings = tm.TwinSettings.from_config(s.cfg)
        except ValueError as e:
            self.status.emit("lost", f"[hmi.twin]: {e}")
            return False
        self._session = s
        self._ur_src = self._ctl.ur_source()
        holder: list = []

        def on_status(state: str, detail: str) -> None:     # twin thread (or GUI thread from start())
            if holder and self._twin is holder[0]:
                self.status.emit(state, detail)
        twin = self._factory(s.cfg, s.job, lambda: self.frame(s), settings, on_status)
        holder.append(twin)
        with self._lock:
            old, self._twin = self._twin, twin
            if old is None:
                twin.start()
                return True

        def restart() -> None:
            old.stop()
            with self._lock:
                if self._twin is twin and not self._closed:     # not switched off / replaced meanwhile
                    twin.start()
        self.status.emit("starting", "stopping the previous twin")
        threading.Thread(target=restart, name="mauer-twin-restart", daemon=True).start()
        return True

    def ensure(self) -> bool:
        """Start the twin unless one already runs (or starts) for the current session."""
        tw = self._twin
        if tw is not None and self._session is self._ctx.session and getattr(tw, "state", "") != "lost":
            return True
        return self.start()

    def stop(self, wait: bool = False) -> None:
        """Stop the twin (its RoboDK closes); in a helper thread unless wait."""
        with self._lock:
            tw, self._twin = self._twin, None
        if tw is None:
            return
        if wait:
            tw.stop()
        else:
            threading.Thread(target=tw.stop, name="mauer-twin-stop", daemon=True).start()
        self.status.emit("off", "switched off")

    def shutdown(self) -> None:
        """HMI close (ctx shutdown hook): stop and wait; no restart afterwards."""
        self._closed = True
        self.stop(wait=True)

    def stats(self) -> dict | None:
        tw = self._twin
        if tw is None:
            return None
        d = dict(tw.stats()) if hasattr(tw, "stats") else {}
        scene = getattr(tw, "scene", None)
        if scene is not None:
            d["stones"] = scene.stone_count()
        d["rate_set_hz"] = getattr(getattr(tw, "settings", None), "rate_hz", None)
        return d

    def request_snapshot(self, path, done: Callable[[bool, str], None]) -> bool:
        tw = self._twin
        if tw is None or not hasattr(tw, "request_snapshot"):
            return False
        tw.request_snapshot(path, show=not tw.settings.visible, done=done)
        return True

    # ── twin thread ──────────────────────────────────────────────────────────
    def frame(self, session=None):
        """The frame source of the twin built for `session` (default: the current one; twin thread): None while
        that is not the controller's session (a twin being replaced never sees the next job)."""
        s, ctl = session or self._session, self._ctl
        if s is None or s is not self._session or ctl.session is not s:
            return None
        snap, rig, src = ctl.snapshot, ctl.rig, self._ur_src
        world = rig.world if rig is not None and rig.mode == "sim" else None
        ur = src.snapshot() if src is not None else None
        live = ares_live(snap, None if world is not None else self._ctx.last_ads_status, world)
        return frame_from(snap, ur, live, s, world)

    # ── GUI thread ───────────────────────────────────────────────────────────
    def _on_rig(self, _rig) -> None:
        if self._session is not None:
            self._ur_src = self._ctl.ur_source()        # a UrSource on the new rig (used only by the twin thread)

    def _on_status(self, state: str, detail: str) -> None:
        self._ctx.set_twin_state(state, detail)
