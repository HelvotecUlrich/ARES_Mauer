"""Run snapshots and run-log presentation (docs/HMI_DESIGN.md section 8.1) - pure, no Qt.

A RunSnapshot is an immutable copy of the sequencer state (progress, pose estimate, magazine / station / held stone)
built in the run thread after every run-log record (SnapshotTracker); the GUI, the twin and the status panels only
read the latest one. ShotView is a display-sized copy of a camera image. describe_action / format_event / severity /
category turn run-log records into operator text.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Collection, Mapping

import numpy as np

from mauer.job import Job, SlotState, StoneTask
from mauer.reference import Pose2D

ERROR_EVENTS = frozenset({"robot_error", "ares_error", "frame_jump", "run_error"})
WARNING_EVENTS = frozenset({"halt", "halt_result", "warning", "interlock", "measurement_failed", "declined",
                            "odometry_pose", "run_paused", "run_aborted"})
MOTION_EVENTS = frozenset({"robot", "ares_cmd", "ares_move", "route", "drive", "resume_route", "resume_check",
                           "pose_set", "pose_confirmed"})
VISION_EVENTS = frozenset({"shot", "wall_frame", "station_frame", "station_estimate", "coarse_aim", "board_search"})
STONE_EVENTS = frozenset({"placed", "held", "held_cleared", "slot_changed", "reload_start", "reload_transfer",
                          "reload_done", "station_empty", "station_refilled"})
CATEGORIES = ("motion", "vision", "stones", "run", "warning")


# ── stones ────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class StoneInfo:
    key: tuple
    label: str
    leg: str | None
    course: int
    index: int
    kind: str
    u_mm: float
    z_top_mm: float
    slot: str | None            # planned magazine slot
    i: int                      # 1-based position in job.stones()

    def describe(self, n: int | None = None) -> str:
        pos = f"{self.i}/{n}" if n else f"{self.i}"
        leg = f"leg {self.leg}, " if self.leg else ""
        return f"stone {pos} {self.label} ({leg}course {self.course}, {self.kind}, slot {self.slot or '-'})"


def stone_index(job: Job) -> dict[tuple, int]:
    """Stone key -> 1-based position in job.stones()."""
    return {t.key: i + 1 for i, t in enumerate(job.stones())}


def stone_info(t: StoneTask, i: int) -> StoneInfo:
    return StoneInfo(tuple(t.key), t.label, t.leg, int(t.course), int(t.index), t.kind, float(t.u_mm),
                     float(t.z_top_mm), t.slot, int(i))


def current_stone(job: Job, stop_k: int | None, placed: Collection,
                  index: Mapping[tuple, int] | None = None) -> StoneInfo | None:
    """The first stone of stop stop_k not placed yet (= the sequencer's own loop in run()); None outside a stop."""
    if stop_k is None or not 0 <= stop_k < len(job.stops):
        return None
    idx = index if index is not None else stone_index(job)
    for t in job.stops[stop_k].stones:
        if t.key not in placed:
            return stone_info(t, idx[t.key])
    return None


def stop_text(k: int | None, n: int) -> str:
    """'stop 2 (0-3)': stops are numbered from 0 everywhere (run log, Stops spin boxes, plan view, sequencer texts;
    review 2026-10-08), legs of a route from 1 ('leg 1/2')."""
    return f"stop {'-' if k is None else k} (0-{max(n - 1, 0)})"


def _slots(s: SlotState) -> dict[str, str]:
    return {sid: s.kinds.get(sid, "full") for sid in s.filled}


# ── snapshot ──────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RunSnapshot:
    t: float
    seq_state: str                  # RunResult.state: idle | running | done | paused | error | aborted
    n_stops: int
    stop_k: int | None
    stop_leg: str | None
    stops_done: tuple[int, ...]
    n_stones: int
    placed: frozenset               # stone keys (tuples)
    stone: StoneInfo | None         # current_stone(job, stop_k, placed)
    action_kind: str                # robot | ares | route | shot | measure | station | wait | idle
    action: str
    at_station: bool
    route: dict | None              # RouteProgress.to_dict()
    reloads: int
    pose_est: Pose2D | None
    pose_src: str
    pose_status: str                # "ok" | "odometry" | "unknown"
    ares_cmd: dict | None           # {"kind", "args", "why", "pose": Pose2D, "odom": OdomPose | None} until the
                                    # move ends
    T_wall_station: np.ndarray      # copy
    magazine: Mapping[str, str]     # filled slot -> kind (copy)
    station: Mapping[str, str]
    held: dict | None
    pause_requested: bool
    ares_moves: int
    corrections: int
    warnings: tuple[str, ...]
    error: str | None
    last_measurement: dict | None   # last camera 'wall_frame' / 'station_frame' record
    last_event: str
    n_events: int
    log_path: str

    @classmethod
    def initial(cls, session, start_stop: int = 0) -> "RunSnapshot":
        """Before a sequencer exists: ARES at the start mark of start_stop, magazine as planned, station full."""
        job = session.job
        k = min(max(int(start_stop), 0), len(job.stops) - 1) if job.stops else 0
        return cls(t=time.time(), seq_state="idle", n_stops=len(job.stops), stop_k=None,
                   stop_leg=job.stops[k].leg if job.stops else None, stops_done=(), n_stones=job.n_stones,
                   placed=frozenset(), stone=current_stone(job, k, ()), action_kind="idle", action="not started",
                   at_station=False, route=None, reloads=0, pose_est=job.stops[k].ares if job.stops else None,
                   pose_src="start mark", pose_status="ok", ares_cmd=None,
                   T_wall_station=np.array(job.station.T_wall_station, float),
                   magazine=_slots(SlotState.magazine(job.magazine)), station=_slots(SlotState.station(job.station)),
                   held=None, pause_requested=False, ares_moves=0, corrections=0, warnings=(), error=None,
                   last_measurement=None, last_event="", n_events=0, log_path="")

    @property
    def n_placed(self) -> int:
        return len(self.placed)


class SnapshotTracker:
    """Folds the run-log records of one sequencer into RunSnapshots (run thread). odom_fn: the current ARES odometry
    (REAL: the HMI's ADS status) stored with an 'ares_cmd' so that the live pose can follow the move."""

    def __init__(self, session, odom_fn: Callable[[], Any] = lambda: None) -> None:
        self.session = session
        self.index = stone_index(session.job)
        self.odom_fn = odom_fn
        self.action_kind, self.action = "idle", "ready"
        self.ares_cmd: dict | None = None
        self.last_measurement: dict | None = None
        self.last_event = ""
        self.n_events = 0

    def record(self, rec: Mapping, seq) -> RunSnapshot:
        ev = str(rec.get("event", ""))
        self.n_events += 1
        self.last_event = ev
        d = describe_action(rec)
        if d is not None:
            self.action_kind, self.action = d
        if ev == "ares_cmd":
            self.ares_cmd = {"kind": rec.get("kind"), "args": list(rec.get("args") or []), "why": rec.get("why", ""),
                             "pose": seq.pose_est, "odom": self.odom_fn()}
        elif ev in ("ares_move", "ares_error"):
            self.ares_cmd = None
        elif ev in ("wall_frame", "station_frame") and rec.get("source") == "camera":
            self.last_measurement = dict(rec)
        return self.snapshot(seq)

    def snapshot(self, seq) -> RunSnapshot:
        job, r = self.session.job, seq.result
        k = seq.stop_k
        placed = frozenset(seq.placed)
        return RunSnapshot(
            t=time.time(), seq_state=r.state, n_stops=len(job.stops), stop_k=k,
            stop_leg=job.stops[k].leg if k is not None and 0 <= k < len(job.stops) else None,
            stops_done=tuple(r.stops_done), n_stones=job.n_stones, placed=placed,
            stone=current_stone(job, k, placed, self.index), action_kind=self.action_kind, action=self.action,
            at_station=bool(seq.at_station), route=seq.route_progress.to_dict() if seq.route_progress else None,
            reloads=r.reloads, pose_est=seq.pose_est, pose_src=seq.pose_src, pose_status=seq.pose_status,
            ares_cmd=dict(self.ares_cmd) if self.ares_cmd else None,
            T_wall_station=np.array(seq.T_wall_station, float), magazine=_slots(seq.magazine),
            station=_slots(seq.station), held=dict(seq.held) if getattr(seq, "held", None) else None,
            pause_requested=bool(seq.paused) and r.state == "running", ares_moves=r.ares_moves,
            corrections=r.corrections, warnings=tuple(r.warnings), error=r.error,
            last_measurement=self.last_measurement, last_event=self.last_event, n_events=self.n_events,
            log_path=str(seq.log.path))


# ── camera images ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ShotView:
    t: float
    look: str | None                # look name; "live" for a manual grab
    parent: str | None              # "wall" | "station"
    preview: np.ndarray             # uint8 H x W (grey) or H x W x 3 (BGR)
    scale: float                    # preview px per image px
    full_size: tuple[int, int]      # (width, height) of the camera image [px]
    rec: dict | None                # the 'shot' record (boards: ok, n_corners, rms_px | reason); None = manual grab
    detections: dict | None = None  # hmi.core.overlay: {board: {"n", "ids", "pts" (preview px), "markers"}}
    note: str = ""


ShotProcessor = Callable[[np.ndarray, "dict | None", Mapping], ShotView]   # (image, shot record, session cfg)


def to_uint8(image: np.ndarray) -> np.ndarray:
    """Mono8 as is; a 16-bit image (unpacked Mono10/12) scaled by its maximum."""
    if image.dtype == np.uint8:
        return image
    a = np.asarray(image, float)
    m = float(a.max()) if a.size else 0.0
    return np.zeros(a.shape, np.uint8) if m <= 0 else np.clip(a * (255.0 / m), 0, 255).astype(np.uint8)


def default_shot_view(image: np.ndarray, rec: Mapping | None, cfg: Mapping, scale: float) -> ShotView:
    """Preview = the image resized by `scale` (cv2 INTER_AREA), nothing drawn (hmi.core.overlay draws)."""
    import cv2
    h, w = image.shape[:2]
    size = (max(1, int(round(w * scale))), max(1, int(round(h * scale))))
    preview = cv2.resize(to_uint8(image), size, interpolation=cv2.INTER_AREA)
    look = rec.get("look") if rec else "live"
    return ShotView(time.time(), look, rec.get("parent") if rec else None, preview, float(scale), (int(w), int(h)),
                    dict(rec) if rec else None)


# ── run-log presentation ──────────────────────────────────────────────────────
def _pose_text(p: Any) -> str:
    if isinstance(p, Pose2D):
        return f"({p.x_mm:.0f}, {p.y_mm:.0f}) mm {math.degrees(p.theta_rad):+.1f} deg"
    if isinstance(p, Mapping) and "x_mm" in p:
        th = p.get("theta_deg", math.degrees(p.get("theta_rad", 0.0) or 0.0))
        return f"({p['x_mm']:.0f}, {p['y_mm']:.0f}) mm {th:+.1f} deg"
    return "-"


def _ares_args(kind: str, args) -> str:
    a = list(args or [])
    if kind == "rotate" and a:
        return f"{a[0]:+.2f} deg"
    if kind == "translate" and len(a) >= 2:
        return f"dx {a[0]:+.1f} mm, dy {a[1]:+.1f} mm"
    return ", ".join(f"{v:+.2f}" if isinstance(v, (int, float)) else str(v) for v in a)


def boards_text(boards: Mapping | None) -> str:
    """'W0 ok 12 corners 0.01 px, W1 not seen (reason)' from the boards of a 'shot' record."""
    parts = []
    for b, d in (boards or {}).items():
        if d.get("ok"):
            rms = d.get("rms_px")
            parts.append(f"{b} ok {d.get('n_corners', '?')} corners" + (f" {rms:.2f} px" if rms is not None else ""))
        else:
            parts.append(f"{b} NOT measured ({d.get('reason', '?')})")
    return ", ".join(parts) or "no boards"


def describe_action(rec: Mapping) -> tuple[str, str] | None:
    """(action kind, text) of a record that starts or ends an action; None = keep the previous action."""
    ev = rec.get("event", "")
    if ev == "robot":
        return "robot", f"{rec.get('action')}: {rec.get('what')}"
    if ev == "ares_cmd":
        return "ares", f"ARES {rec.get('kind')} {_ares_args(rec.get('kind', ''), rec.get('args'))} ({rec.get('why')})"
    if ev == "ares_move":
        return "ares", f"ARES {rec.get('kind')} {'ok' if rec.get('ok') else 'NOT ok'}: {rec.get('summary')}"
    if ev == "route":
        return "route", f"route: {rec.get('why')}" + (" (resumed)" if rec.get("resume") else "")
    if ev == "drive":
        return "route", f"drive: {rec.get('why')}"
    if ev == "resume_route":
        rt = rec.get("route") or {}
        nl = rt.get("next_leg")
        return "route", (f"resume route {rt.get('why', '')} at leg "
                         f"{nl + 1 if isinstance(nl, int) else '?'}/{rt.get('n_legs', '?')}")
    if ev == "shot":
        return "shot", f"image at look {rec.get('look')}: {boards_text(rec.get('boards'))}"
    if ev in ("wall_frame", "station_frame"):
        what = "wall" if ev == "wall_frame" else "station"
        if rec.get("source") != "camera":
            return "measure", f"{what} frame (dead reckoning)"
        return "measure", (f"{what} frame stop {rec.get('stop')}: rms {rec.get('rms_mm', float('nan')):.2f} mm, "
                           f"{rec.get('err_nominal_mm', float('nan')):.1f} mm from nominal")
    if ev == "reload_start":
        return "station", f"station trip from stop {rec.get('stop')} ({len(rec.get('plan') or [])} stones)"
    if ev == "station_empty":
        return "wait", "pick-up station empty - refill it"
    if ev == "station_refilled":
        return "station", f"station refilled ({rec.get('station')} stones)"
    if ev.startswith("run_"):
        text = {"run_start": "run started", "run_done": "run done"}.get(ev)
        return "idle", text or f"{ev[4:]}: {rec.get('error', '')}"
    return None


def severity(rec: Mapping) -> str:
    """'error' | 'warning' | 'info'."""
    ev = rec.get("event", "")
    if ev in ERROR_EVENTS:
        return "error"
    if ev in WARNING_EVENTS or (ev == "ares_move" and not rec.get("ok", True)):
        return "warning"
    return "info"


def category(rec: Mapping) -> str:
    """'motion' | 'vision' | 'stones' | 'run' | 'warning' (warnings and errors first)."""
    ev = rec.get("event", "")
    if severity(rec) != "info":
        return "warning"
    if ev in MOTION_EVENTS:
        return "motion"
    if ev in VISION_EVENTS:
        return "vision"
    if ev in STONE_EVENTS:
        return "stones"
    return "run"


_SKIP = ("t", "event", "params", "fit", "plan")


def _value(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.2f}"
    if isinstance(v, Mapping) and "x_mm" in v and "y_mm" in v:
        return _pose_text(v)
    if isinstance(v, Mapping):
        return "{" + ", ".join(f"{k}: {_value(x)}" for k, x in list(v.items())[:6]) + ("..." if len(v) > 6 else "") \
            + "}"
    if isinstance(v, (list, tuple)):
        if len(v) > 6:
            return f"[{len(v)} items]"
        return "[" + ", ".join(_value(x) for x in v) + "]"
    return str(v)


def format_event(rec: Mapping, max_len: int = 240) -> str:
    """'HH:MM:SS.mmm  event  details' - one line (matrices, fits and plans left out)."""
    t = float(rec.get("t", 0.0) or 0.0)
    stamp = time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1.0) * 1000):03d}"
    ev = str(rec.get("event", "?"))
    if ev == "shot":
        details = f"look {rec.get('look')}: {boards_text(rec.get('boards'))}"
    else:
        details = "  ".join(f"{k}={_value(v)}" for k, v in rec.items() if k not in _SKIP and not k.startswith("T_"))
    line = f"{stamp}  {ev}  {details}".rstrip()
    return line if len(line) <= max_len else line[:max_len - 3] + "..."
