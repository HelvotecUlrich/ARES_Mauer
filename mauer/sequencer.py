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
reload(): park; drive to the station dock pose - along the stop's validated route_to_station (job v2, mauer/floor.py)
  when the job has one, else (v1) translate in the body frame and rotate at the dock (obstacles on the way are then
  the operator's responsibility, logged as a warning); measure the station boards -> station frame (correction move
  if the dock error exceeds dock_tol_mm); move stones station -> magazine (job.reload_plan: the types of the next
  stones, in the order they will be taken; the operator tops the station up first when it would bring fewer stones
  than a full one); park; drive back (route_from_station, else rotate at the dock, then translate; closed loop from the
  measured station pose); re-measure the wall (with corrections) before the next stone.
Routes (job v2: moves between stops, leg change between the legs of an L, station trips): the intermediate legs are
  steered dead-reckoned (from the current estimate = last measurement + commanded moves to the next waypoint; one
  translation or one rotation per PLC command, no measurement on the way); the last leg goes to the dock, or to
  `arrival_standoff_mm` before the stop (the job's meta.route_check, [sequencer] arrival_standoff_mm): the wall is
  measured there and the stop is reached by the closed-loop correction from that measurement, so the dead-reckoned
  route error never meets the plates 10 mm in front of ARES. The first wall measurement after a route uses the
  route_max_jump limits, after a station trip the station limits.
Interruptions (pause, a declined step, an ARES or robot error) keep the route progress (route, next leg); run() with
  the same stop resumes it: an interrupted station trip continues to the station (and does the reload), an interrupted
  return or move between stops continues its remaining legs - never a straight line from wherever ARES stands to the
  first waypoint of another route. A direct move between stops (no route) counts as a two-waypoint route while it
  runs, so an interrupted one is driven to its end on resume (2026-10-07). The first move of a resume is checked
  against the floor model of the job (legs, plates, table; mauer.floor.job_obstacles) and refused if it would cross
  an obstacle. A move that ended not ok
  (aborted by the PLC) updates the estimate from the odometry of the outcome and marks it unverified; an ARES error
  without an outcome marks the pose unknown - a resume then needs confirm_pose() / set_pose(pose) by the operator.
Held stone (2026-10-07): `held` tracks the stone between pick and put-down; a pause takes effect only with empty
  jaws (the place of a held stone still runs), a robot error during a pick or place marks the jaw state unknown, and
  run() refuses to resume while a stone may be held until the operator calls clear_held().
Rotations of more than `rotate_keep_dir_deg` keep the direction of the previous rotation (E003: ~1.2 deg loss after
reversing the rotation direction, mauer/ares/ads.py AresAds.rotate).

camera_loop=False runs the same job by pure dead reckoning (nominal moves, nominal frames, no images) - the
comparison case for the simulation.

Every measurement and decision goes to a JSON-lines run log (log_dir/run.jsonl, default data/runs/<timestamp>/),
optionally with the images (save_images). Step mode: `confirm(description) -> bool` is called before every motion
(each ARES command, each robot program); False aborts the run (SequencerAborted).
Hooks for the Mauer HMI (hmi/, docs/HMI_DESIGN.md section 7): RunLog listeners get every record in the writer's
thread (writes are serialised by a lock, so the GUI thread may call pause()); `on_shot(image, shot_record)` taps
every camera image after its measurement; an `ares_cmd` record precedes every ARES command, `station_refilled`
follows the operator's station refill.

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
import threading
import time
from dataclasses import asdict, dataclass, field, fields, replace
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from . import REPO
from . import geometry as g
from . import floor as _floor
from .ares.ads import MIN_MOVE_DEG, MIN_MOVE_MM, AresNotReady, MoveRefused
from .ares.ads import AresError as AdsError
from .backends import BackendError, RobotError, ShotError
from .job import Job, Look, SlotState, StoneTask, reload_plan, reload_short, restart_fill
from .reference import (FrameFit, Pose2D, T_base_parent_from, T_parent_ares, after_rotation, after_translation,
                        fit_frame, placements, planar_T, relative_move, tilt_deg, wrap_angle)
from .vision.detect import coarse_poses, measure
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
    route_max_jump_mm: float = 100.0       # first wall measurement after a route (leg change: several dead-reckoned
    route_max_jump_deg: float = 5.0        # moves and a 90 deg rotation)
    max_fit_rms_mm: float = 1.0            # corner residual RMS of the frame fit -> MeasurementError
    dock_tol_mm: float = 30.0              # dock position error that triggers a correction move
    require_all_boards: bool = True        # every expected board of a look must be measured
    search_step_mm: float = 50.0           # board search when nothing of the expected boards is usable (ARES further
    search_rings: int = 2                  # off than predicted, e.g. after a long route): look poses for ARES
                                           # displaced by +-k * step (k = 1..rings, 8 directions), IK-checked

    @classmethod
    def from_config(cls, cfg: Mapping) -> "SequencerParams":
        s = cfg.get("sequencer", {}) or {}
        kw = {}
        for f in fields(cls):
            if f.name in s:
                kw[f.name] = type(f.default)(s[f.name])
        return cls(**kw)


@dataclass
class RouteProgress:
    """An ARES route being driven - kept when the run stops on the way, so that run() can resume it."""
    kind: str                         # "stop" (move between stops / leg change into `stop`) | "to_station" |
                                      # "from_station"
    stop: int                         # the stop the route goes to (stop routes) or whose station trip it is
    route: list[Pose2D]
    why: str
    next_leg: int = 0                 # route[next_leg] -> route[next_leg + 1] is the next leg to drive
    target: Pose2D | None = None      # closed-loop goal of the last leg (dock estimate); None = the last waypoint
    standoff_mm: float = 0.0          # the last leg ends this far before the goal (stops only)

    def to_dict(self) -> dict:
        return {"kind": self.kind, "stop": self.stop, "why": self.why, "next_leg": self.next_leg,
                "n_legs": len(self.route) - 1, "standoff_mm": self.standoff_mm}


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
    """JSON-lines log: one object per line {"t": now(), "event": ..., ...}; flushed after every line. Writes may come
    from several threads (the run thread, the HMI's pause()); listeners get each record in the writer's thread."""

    def __init__(self, folder: Path, now: Callable[[], float] = time.time):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / "run.jsonl"
        self.now = now
        self._f = open(self.path, "a", encoding="utf-8")
        self.n_images = 0
        self._lock = threading.Lock()
        self._listeners: list[Callable[[dict], None]] = []

    def add_listener(self, fn: Callable[[dict], None]) -> None:
        """fn(record) after every write, in the writer's thread; its exceptions are logged and swallowed."""
        self._listeners = [*self._listeners, fn]

    def remove_listener(self, fn: Callable[[dict], None]) -> None:
        self._listeners = [f for f in self._listeners if f != fn]

    def write(self, event: str, **data: Any) -> dict:
        rec = {"t": self.now(), "event": event, **_jsonable(data)}
        line = json.dumps(rec) + "\n"
        with self._lock:
            self._f.write(line)
            self._f.flush()
        for fn in self._listeners:
            try:
                fn(rec)
            except Exception as e:                   # noqa: BLE001 - a listener must never stop the run
                log.warning("run-log listener %r failed on %s: %s", fn, event, e)
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
        with self._lock:
            if not self._f.closed:
                self._f.close()


def read_log(path: str | Path) -> list[dict]:
    p = Path(path)
    if p.is_dir():
        p = p / "run.jsonl"
    return [json.loads(ln) for ln in p.read_text(encoding="utf-8").splitlines() if ln.strip()]


def standing_in_log(path: str | Path) -> set[tuple]:
    """Stones that stand after the run of this log: its "placed" stones and those it took over as standing from an
    earlier log ("declared_placed") - the input of Sequencer.declare_placed for the next run (resume chain)."""
    out: set[tuple] = set()
    for e in read_log(path):
        if e.get("event") == "placed":
            out.add(tuple(e["stone"]))
        elif e.get("event") == "declared_placed":
            out |= {tuple(k) for k in e.get("stones") or []}
    return out


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
# Config values that act on the motion without a camera correction (test plan Anhang C5, 2026-10-08): the real run is
# refused while one of them is PLACEHOLDER / UNKNOWN. Key as mauer.job.config_status -> what a wrong value does.
MEASURE_BEFORE_REAL = {
    "[ur5] mount_z": "magazine pick and place height 1:1",
    "[ur5] mount_rz": "magazine pick position (1 deg moves magazine row 0, 654 mm behind the UR axis, by 11 mm)",
    "[deck] holder_z": "magazine pick and place height 1:1",
    "[ur] payload_cog_mm": "the UR payload model (its collision detection)",
    "[boards.ref] square_mm": "the scale of every wall and station measurement",
    "[camera] settle_s": "measurements while ARES still sways",
}


def preflight_real(cfg: Mapping, job: Job, *, intrinsics_file: str | Path | None = None,
                   handeye_file: str | Path | None = None, config_path: str | Path | None = None) -> list[str]:
    """Every reason not to run this job on the real robot (empty list = ok). Lists all problems at once."""
    from .job import config_sha256
    p: list[str] = []
    variant = job.meta.get("config_variant") or None        # config/variants/<variant>.toml the job was built with
    if (cfg.get("_variant") or None) != variant:
        p.append(f"config variant {cfg.get('_variant')!r} loaded, the job was built with {variant!r} - load the "
                 "same variant (--variant)")
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
    try:
        from .job import config_status
        status = config_status(config_path, variant)
    except OSError:
        status = {}
    for key, what in MEASURE_BEFORE_REAL.items():
        if status.get(key, {}).get("status") in ("PLACEHOLDER", "UNKNOWN"):
            p.append(f"{key} {status[key]['status']} - measure it (no camera correction: {what})")
    if any(t.kind == "half" for t in job.stones()):
        hb = cfg.get("half_brick", {}) or {}
        half = float(hb.get("mass_kg", 0.0) or 0.0)
        if half <= 0.0:
            p.append("[half_brick] mass_kg <= 0 (UNKNOWN) - weigh a half stone")
        if tool > 0.0 and half > 0.0 and tool + half > 5.0:
            p.append(f"payload {tool + half:.2f} kg (tool + half stone) exceeds the UR5 rated payload 5 kg")
        ph = sorted(k.split(" ", 1)[1] for k, v in status.items()
                    if k.startswith("[half_brick] ") and v["status"] == "PLACEHOLDER")
        if ph:
            p.append(f"[half_brick] {', '.join(ph)} PLACEHOLDER (current half-stone CAD not available) - measure the "
                     "half stone and its pin pair")
    if (job.legs or any(st.route or st.route_to_station for st in job.stops)) and not job.meta.get("route_check"):
        p.append("job has legs / ARES routes without a recorded route check (mauer.floor.validate_route)")
    p += route_problems(cfg, job)
    for key, default, what in ((intrinsics_file, v.get("intrinsics_file", "calib/camera_intrinsics.json"),
                                "camera intrinsics"),
                               (handeye_file, v.get("handeye_file", "calib/handeye.json"), "hand-eye calibration")):
        path = Path(key or default)
        path = path if path.is_absolute() else REPO / path
        if not path.exists():
            p.append(f"{what} missing: {path.relative_to(REPO) if path.is_relative_to(REPO) else path} - calibrate "
                     "first (tools/calib_intrinsics.py, tools/handeye_solve.py)")
    m = job.meta
    if m.get("reach_check") == "robodk":             # stamped by robodk/simulate.py: still the code it verified?
        from .job import stamp_problems
        p += stamp_problems(job)
    if m.get("look_source", "nominal") == "nominal" and m.get("reach_check", "none") != "robodk":
        p.append(f"job built from nominal look poses with reach check {m.get('reach_check', 'none')!r} only - no "
                 "collision/occlusion check (export the job from the RoboDK planner)")
    if any(t.qnear_rad is None for t in job.stones()):
        p.append("job has place poses without an IK branch hint (qnear)")
    try:
        sha = config_sha256(config_path, variant)
        if job.config_sha256 and sha != job.config_sha256:
            p.append("config/station.toml" + (f" + variants/{variant}.toml" if variant else "")
                     + " changed since the job was built (sha256 differs) - rebuild the job")
    except OSError as e:
        p.append(f"config not readable: {e}")
    for key, val in (("do_grip_open", u.get("do_grip_open")), ("do_grip_close", u.get("do_grip_close"))):
        if val is None:
            p.append(f"[ur] {key} missing")
    return p


def route_problems(cfg: Mapping, job: Job) -> list[str]:
    """Re-check every route of an L job (legs present) against the floor model of the CURRENT config
    (mauer.floor.job_obstacles, [routes] clearance_mm): a job built for other plates / legs / station is refused.
    Moves between stops without a route are checked as a direct move (no clearance, only overlaps). [] without legs."""
    if not job.legs:
        return []
    try:
        specs = board_specs(cfg)
        obst = _floor.job_obstacles(cfg, job.legs, job.station.T_wall_station,
                                    {n: sp.size_mm for n, sp in specs.items()})
        ares = _floor.AresShape.from_config(cfg)
    except (KeyError, ValueError, TypeError) as e:
        return [f"floor model of the job not available ({e}) - routes not checkable"]
    clr = float((cfg.get("routes", {}) or {}).get("clearance_mm", 50.0))
    p = []
    dock = job.station.dock_in_wall
    for k, st in enumerate(job.stops):
        if k > 0:
            if st.route:
                p += _floor.validate_route(st.route, obst, ares, clr, name=f"stop {k} route")
            else:
                p += _floor.validate_route([job.stops[k - 1].ares, st.ares], obst, ares, 0.0, min_mm=0.0, min_deg=0.0,
                                           name=f"stop {k - 1} -> stop {k} (direct)")
        for r, nm in ((st.route_to_station, f"stop {k} route_to_station"),
                      (st.route_from_station, f"stop {k} route_from_station")):
            if r:
                p += _floor.validate_route(r, obst, ares, clr, name=nm)
        if not st.route_to_station or not st.route_from_station:
            p.append(f"stop {k}: no station route (an L job needs route_to_station / route_from_station)")
        for o in obst:
            if _floor.overlaps(ares.footprint(st.ares), o.poly):
                p.append(f"stop {k}: ARES footprint overlaps {o.name}")
    if any(dock.delta(st.route_to_station[-1])[0] > 0.5 for st in job.stops if st.route_to_station):
        p.append("a station route does not end at the job's dock")
    return p


# ── sequencer ─────────────────────────────────────────────────────────────────
class Sequencer:
    """Runs `job` on the backends (see the module docstring). intr / T_flange_cam: the CALIBRATED camera model and
    hand-eye result used to turn images into board poses."""

    def __init__(self, job: Job, cfg: Mapping, robot, ares, camera, intr, T_flange_cam: np.ndarray,
                 log_dir: str | Path | None = None, confirm: Callable[[str], bool] | None = None,
                 now: Callable[[], float] = time.time, *, camera_loop: bool = True, save_images: bool = False,
                 on_station_empty: Callable[[], Any] | None = None, params: SequencerParams | None = None,
                 on_shot: Callable[[np.ndarray, dict], None] | None = None):
        self.job, self.cfg = job, cfg
        self.robot, self.ares, self.camera = robot, ares, camera
        self.intr = intr
        self.T_flange_cam = np.asarray(T_flange_cam, float)
        self.confirm, self.now = confirm, now
        self.camera_loop, self.save_images = bool(camera_loop), bool(save_images)
        self.on_station_empty = on_station_empty
        self.on_shot = on_shot                   # frame tap (image, 'shot' record), run thread, after the measurement
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
        self.T_base_station: np.ndarray | None = None      # measured at the dock (motion guard world)
        self._wall_meas: tuple | None = None   # (measured T_base_wall, ARES pose estimate at that measurement)
        self._aim_frame: tuple | None = None   # (parent, T_parent_base) the current look is aimed with
        self._prebuilt: set[tuple] = set()     # stones of earlier stops / runs that stand already (run start)
        self.route_progress: RouteProgress | None = None
        self.pending_why: str | None = None      # limits of the next wall measurement after a finished route / return
        self.pose_status = "ok"                # "ok" | "odometry" (move not ok: odometry estimate) | "unknown"
        self.held: dict | None = None            # stone in the jaws: {"from", "slot", "kind", "stone", "unknown"}
        self.board_leg = {str(t["name"]): t.get("leg") for t in cfg.get("targets", []) if t.get("parent") == "wall"}
        # the standoff the job's looks were checked for (tools/make_job.py); 0 = route ends at the stop (old jobs)
        self.standoff_mm = float((job.meta.get("route_check") or {}).get("arrival_standoff_mm", 0.0) or 0.0)
        self.ares_shape = _floor.AresShape.from_config(cfg) if "ares" in cfg else _floor.AresShape()
        self.floor: list | None = None         # obstacles for the resume check (L jobs)
        self._floor_note = ""
        if job.legs:
            try:
                self.floor = _floor.job_obstacles(cfg, job.legs, job.station.T_wall_station,
                                                  {n: sp.size_mm for n, sp in self.specs.items()})
            except (KeyError, ValueError, TypeError) as e:
                self._floor_note = f"floor model not available ({e}) - resumed moves are not checked"
        if log_dir is None:
            stamp = _dt.datetime.fromtimestamp(now()).strftime("%Y-%m-%d_%H%M%S")
            log_dir = REPO / "data" / "runs" / stamp
        self.log = RunLog(Path(log_dir), now)
        self.result = RunResult(log_path=str(self.log.path))

    # ── control ──────────────────────────────────────────────────────────────
    def declare_placed(self, keys, source: str = "operator") -> None:
        """Stones that already stand (e.g. from the run log of an interrupted run, tools/run_job.py --resume-log):
        skipped by run() and part of the motion guard's wall. Unknown keys -> ValueError."""
        known = {t.key for t in self.job.stones()}
        keys = {tuple(k) for k in keys}
        bad = keys - known
        if bad:
            raise ValueError(f"declare_placed: not stones of this job: {sorted(bad)[:5]}")
        self.placed |= keys
        self.log.write("declared_placed", source=source, stones=sorted(keys))

    def pause(self) -> None:
        """Request a pause: the next motion raises SequencerPaused (ARES is interlocked while paused)."""
        self.paused = True
        self.log.write("pause_requested")

    def resume(self) -> None:
        self.paused = False
        self.log.write("resume")

    def set_pose(self, pose: Pose2D, source: str = "operator") -> None:
        """Operator: the ARES pose in the wall frame (e.g. after an ARES error, ARES jogged to / measured at a known
        pose). Clears an unknown / unverified pose; the dead-reckoning chain since the last measurement is dropped."""
        self.pose_est, self.pose_src, self.pose_status = pose, source, "ok"
        self.anchor, self.dr = None, np.eye(4)
        self.log.write("pose_set", pose=pose, source=source)

    def confirm_pose(self) -> None:
        """Operator: accept the odometry estimate after a move that ended not ok (ARES checked on the floor)."""
        if self.pose_est is None:
            raise SequencerError("no pose estimate to confirm - use set_pose(pose)")
        if self.pose_status == "unknown":
            raise SequencerError("the ARES pose is unknown (ARES error without odometry) - use set_pose(pose)")
        self.pose_status = "ok"
        self.log.write("pose_confirmed", pose=self.pose_est, source=self.pose_src)

    def _standoff(self) -> float:
        return self.standoff_mm if self.camera_loop else 0.0      # dead reckoning: nothing measures the last bit

    def _warn(self, msg: str, **data) -> None:
        log.warning(msg)
        self.result.warnings.append(msg)
        self.log.write("warning", msg=msg, **data)

    def _confirm(self, desc: str) -> None:
        # a pause takes effect only with empty jaws: while a stone is held its place still runs (and is confirmed)
        if self.paused and self.held is None:
            raise SequencerPaused("run paused", self.stop_k)
        if self.confirm is not None and not self.confirm(desc):
            if self.paused and self.held is None:       # pause pressed while the step waited for the operator
                raise SequencerPaused("run paused", self.stop_k)
            self.log.write("declined", what=desc)
            raise SequencerAborted(f"operator declined: {desc}", self.stop_k)

    def _set_held(self, held: dict | None) -> None:
        self.held = held
        self.log.write("held", held=held)

    def clear_held(self, source: str = "operator") -> None:
        """Operator: the jaws are empty (stone taken out or placed by hand, arm parked) - lifts the resume refusal."""
        self.log.write("held_cleared", held=self.held, source=source)
        self.held = None

    # ── robot ────────────────────────────────────────────────────────────────
    def _robot(self, action: str, desc: str, *args) -> Any:
        self._confirm(f"robot: {desc}")
        self.log.write("robot", action=action, what=desc)
        guard = getattr(self.robot, "guard", None)
        if guard is not None:                        # mauer.motionguard: what the arm must not touch right now
            guard.set_world(self._guard_world())
        try:
            return getattr(self.robot, action)(*args)
        except RobotError as e:
            self.log.write("robot_error", action=action, what=desc, error=str(e))
            if action in ("pick_magazine", "pick_station"):        # the jaws may or may not hold the stone now
                slot = next((a for a in args if hasattr(a, "id")), None)
                kind = args[2] if action == "pick_magazine" and len(args) > 2 else getattr(slot, "kind", "full")
                self._set_held({"from": "magazine" if action == "pick_magazine" else "station",
                                "slot": getattr(slot, "id", None), "kind": kind, "stone": None, "unknown": True})
            elif action.startswith("place_") and self.held is not None:
                self._set_held({**self.held, "unknown": True})
            raise SequencerError(f"robot {action} failed ({desc}): {e} - the arm is NOT parked; ARES stays "
                                 "interlocked. Inspect, recover the arm (protective stop / stone in the jaws), park "
                                 f"it, then resume from stop {self.stop_k}", self.stop_k) from e

    def _guard_world(self):
        """The world of mauer.motionguard (review 2026-10-07): magazine slots holding a stone; the stones standing (placed
        by this run + those of earlier stops / runs, _prebuilt) in the wall frame - while a look is aimed: the frame it
        is aimed with, else the last measured 6-DoF frame moved by the planar ARES motion since (keeps tilt and
        height); docked: the station slots holding a stone in the station frame (measured at this dock, else the aim /
        predicted frame)."""
        from .motionguard import GuardWorld
        aim = self._aim_frame
        T_bw = None
        if aim is not None and aim[0] == "wall":
            T_bw = g.inv(aim[1])
        elif self._wall_meas is not None and self.pose_est is not None:
            T_meas, pose_meas = self._wall_meas
            D = g.inv(pose_meas.T) @ self.pose_est.T                  # planar ARES motion since the measurement
            T_bw = g.inv(g.inv(T_meas) @ self.T_ares_base @ D @ self.T_base_ares)
        elif self.pose_est is not None:
            T_bw = T_base_parent_from(self.pose_est, self.T_ares_base)
        T_bs = None
        if self.at_station:
            if aim is not None and aim[0] == "station":
                T_bs = g.inv(aim[1])
            elif self.T_base_station is not None:
                T_bs = self.T_base_station
            elif self.pose_est is not None:                           # docked, not measured yet: the prediction
                T_bs = T_base_parent_from(Pose2D.from_T(g.inv(self.T_wall_station) @ self.pose_est.T),
                                          self.T_ares_base)
        standing = self.placed | self._prebuilt
        return GuardWorld(magazine=[self.job.magazine.slot(sid) for sid in self.magazine.filled],
                          wall_stones=[t for t in self.job.stones() if t.key in standing],
                          legs=list(self.job.legs), T_base_wall=T_bw,
                          station=[self.job.station.slot(sid) for sid in self.station.filled] if T_bs is not None
                          else [], T_base_station=T_bs)

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

    def _ares_cmd(self, kind: str, *args, why: str = "") -> Any:
        what = f"ARES {kind}{tuple(round(a, 2) for a in args)}"
        self.log.write("ares_cmd", kind=kind, args=list(args), why=why, pose=self.pose_est, pose_src=self.pose_src)
        try:
            out = getattr(self.ares, kind)(*args)
        except (MoveRefused, AresNotReady) as e:               # nothing was written: ARES did not move
            self.log.write("ares_error", kind=kind, args=list(args), error=str(e), moved=False)
            raise SequencerError(f"{what} refused: {e} (ARES did not move)", self.stop_k) from e
        except (AdsError, BackendError) as e:
            self.log.write("ares_error", kind=kind, args=list(args), error=str(e), moved=None)
            self.pose_status, self.pose_src = "unknown", "unknown (ARES error)"
            raise SequencerError(f"{what} failed: {e} - the ARES pose is UNKNOWN now: measure it or jog ARES to a "
                                 "known pose, call set_pose(pose), then resume", self.stop_k) from e
        self.result.ares_moves += 1
        ok = bool(getattr(out, "ok", True))
        self.log.write("ares_move", kind=kind, args=list(args), ok=ok, why=why, n=self.result.ares_moves,
                       summary=out.summary() if hasattr(out, "summary") else str(out))
        if not ok:
            text = out.summary() if hasattr(out, "summary") else str(out)
            if not getattr(out, "rejected", False):          # a rejected command never moved ARES
                self._from_odometry(out, kind)
            raise SequencerError(f"{what} not ok: {text} - pose estimate {self.pose_est.describe()} "
                                 f"({self.pose_src}); check ARES on the floor and confirm_pose() / set_pose(pose) "
                                 "before resuming", self.stop_k)
        return out

    def _from_odometry(self, out, kind: str) -> None:
        """A move that ended not ok (PLC abort: scanner, HALT, jog, timeout): ARES moved by the outcome's odometry
        (odom_after - odom_before, body frame of the start); the estimate follows it and is marked unverified."""
        b, a = getattr(out, "odom_before", None), getattr(out, "odom_after", None)
        vals = [] if b is None or a is None else [b.x_mm, b.y_mm, b.theta_deg, a.x_mm, a.y_mm, a.theta_deg]
        if not vals or not all(math.isfinite(float(v)) for v in vals):
            self.pose_status, self.pose_src = "unknown", "unknown (move not ok, no odometry)"
            self.log.write("odometry_pose", kind=kind, pose=None, status=self.pose_status)
            return
        t0 = math.radians(b.theta_deg)
        dxw, dyw = a.x_mm - b.x_mm, a.y_mm - b.y_mm
        bx, by = math.cos(t0) * dxw + math.sin(t0) * dyw, -math.sin(t0) * dxw + math.cos(t0) * dyw
        dth = wrap_angle(math.radians(a.theta_deg - b.theta_deg))
        step = planar_T(bx, by, dth)
        self.dr = self.dr @ step
        self.pose_est = Pose2D.from_T(self.pose_est.T @ step)
        self.pose_src, self.pose_status = "odometry (move not ok)", "odometry"
        if kind == "rotate" and abs(dth) > 1e-9:
            self.last_rot_sign = math.copysign(1.0, dth)
        self.log.write("odometry_pose", kind=kind, body_move=[bx, by, math.degrees(dth)], pose=self.pose_est,
                       status=self.pose_status)

    def _rotate(self, dtheta: float, why: str) -> None:
        if abs(dtheta) > math.radians(self.p.rotate_keep_dir_deg) and self.last_rot_sign \
                and math.copysign(1.0, dtheta) != self.last_rot_sign:
            dtheta += self.last_rot_sign * 2.0 * math.pi          # the long way round, same direction as before
        deg = math.degrees(dtheta)
        if abs(deg) < MIN_MOVE_DEG:
            return
        if self.paused:                              # a pause stops the run as "paused" (like a robot motion)
            raise SequencerPaused("run paused", self.stop_k)
        self._interlock(f"rotate {deg:+.2f} deg")
        self._confirm(f"ARES rotate {deg:+.2f} deg ({why})")
        self._ares_cmd("rotate", deg, why=why)
        self.last_rot_sign = math.copysign(1.0, dtheta)
        self.dr = self.dr @ planar_T(0.0, 0.0, dtheta)
        self.pose_est, self.pose_src = after_rotation(self.pose_est, dtheta), "predicted"

    def _translate(self, dx: float, dy: float, why: str) -> None:
        if math.hypot(dx, dy) < MIN_MOVE_MM:
            return
        if self.paused:                              # a pause stops the run as "paused" (like a robot motion)
            raise SequencerPaused("run paused", self.stop_k)
        self._interlock(f"translate ({dx:+.1f}, {dy:+.1f}) mm")
        self._confirm(f"ARES translate dx {dx:+.1f} mm, dy {dy:+.1f} mm ({why})")
        self._ares_cmd("translate", dx, dy, why=why)
        self.dr = self.dr @ planar_T(dx, dy, 0.0)
        self.pose_est, self.pose_src = after_translation(self.pose_est, dx, dy), "predicted"

    def _follow_route(self, route: Sequence[Pose2D], why: str, target: Pose2D | None = None, *, kind: str = "stop",
                      stop: int | None = None, start_leg: int = 0, resume: bool = False,
                      standoff_mm: float = 0.0) -> None:
        """Drive a route (wall-frame waypoints, job v2). Intermediate legs are steered DEAD-RECKONED: a translation leg
        moves from the current estimate (last measurement + commanded moves) to the waypoint position (body frame, one
        PLC translation), a rotation leg turns to the waypoint heading - no measurement on the way, so the start error
        of the route does not grow along it (a nominal relative move would carry a 1 deg start heading error into
        17 mm per metre). The last leg is the closed-loop move to `target` (default: the last waypoint), ending
        standoff_mm before it when it is a translation (the caller measures and corrects from there); a last rotation
        corrects the position first, a last translation the heading first. Progress is kept in self.route_progress
        (resume: start at start_leg from the current estimate, after a floor check of that first move). Does NOT park
        the arm (the interlock refuses otherwise)."""
        route = list(route)
        n = len(route)
        pr = RouteProgress(kind, self.stop_k if stop is None else stop, route, why, start_leg, target, standoff_mm)
        self.route_progress = pr
        self.log.write("route", why=why, waypoints=route, start=self.pose_est, start_src=self.pose_src,
                       start_leg=start_leg, resume=resume, standoff_mm=standoff_mm)
        if resume:
            self._check_resume_move(pr)
        for i in range(start_leg, n - 2):
            pr.next_leg = i
            a, b = route[i], route[i + 1]
            if math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) >= 0.5:
                dx, dy, _ = relative_move(self.pose_est, b)
                self._translate(dx, dy, f"{why} leg {i} (dead reckoning)")
            else:
                nominal = wrap_angle(b.theta_rad - a.theta_rad)
                d = wrap_angle(b.theta_rad - self.pose_est.theta_rad)
                # a planned half turn keeps its direction - but only while (nearly) half of it is still to go: after a
                # resume a few degrees (or nothing) may be left, which must not become a full turn
                if abs(abs(nominal) - math.pi) < 1e-3 and abs(d) > math.pi / 2 \
                        and math.copysign(1.0, d) != math.copysign(1.0, nominal):
                    d += math.copysign(2.0 * math.pi, nominal)
                self._rotate(d, f"{why} leg {i} (dead reckoning)")
            pr.next_leg = i + 1
        pr.next_leg = n - 2
        a, b = route[-2], route[-1]
        last_is_rotation = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) < 0.5
        goal = target or b
        tag = "closed loop"
        if standoff_mm > 0 and not last_is_rotation:
            L = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
            s_ = min(standoff_mm, L)
            ux, uy = (b.x_mm - a.x_mm) / L, (b.y_mm - a.y_mm) / L
            goal = Pose2D(goal.x_mm - s_ * ux, goal.y_mm - s_ * uy, goal.theta_rad)
            tag = f"closed loop to {s_:.0f} mm before the stop"
        # resuming inside the last leg: its nominal start no longer applies (no correction cap)
        nominal_from = None if (resume and start_leg >= n - 2) else a
        self._drive_to(goal, nominal_from, f"{why} last leg ({tag})", rotate_first=not last_is_rotation)
        self.route_progress = None
        if kind in ("stop", "from_station"):         # until the wall is measured, a resume keeps these limits
            self.pending_why = "after route" if kind == "stop" else "after reload"

    def _check_resume_move(self, pr: RouteProgress) -> None:
        """The first move of a resumed route goes from the CURRENT estimate to the next waypoint (a translation keeps
        the current heading, a rotation turns on the spot): refused when it would overlap a leg, plate or the table
        of the job's floor model (no clearance asked - only a real crossing)."""
        if self.floor is None:
            if self._floor_note:
                self._warn(f"resume of '{pr.why}': {self._floor_note}")
            return
        i = pr.next_leg
        a0, b = pr.route[i], pr.route[i + 1]
        p = self.pose_est
        if math.hypot(b.x_mm - a0.x_mm, b.y_mm - a0.y_mm) >= 0.5:
            seg = [p, Pose2D(b.x_mm, b.y_mm, p.theta_rad)]
        else:
            seg = [p, Pose2D(p.x_mm, p.y_mm, b.theta_rad)]
        if seg[0].delta(seg[1])[0] < 0.5 and seg[0].delta(seg[1])[1] < 1e-3:
            return
        probs = _floor.validate_route(seg, self.floor, self.ares_shape, 0.0, min_mm=0.0, min_deg=0.0,
                                      name="resumed move")
        self.log.write("resume_check", route=pr.to_dict(), start=p, to=seg[1], problems=probs)
        if probs:
            raise SequencerError(f"resume of '{pr.why}' refused: the next move from the current estimate "
                                 f"{p.describe()} to {seg[1].describe()} would cross an obstacle: " + "; ".join(probs)
                                 + " - drive ARES manually onto the route, set_pose(pose), then resume", self.stop_k)

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
        try:
            return self._measure_looks(parent, looks, what, T_parent_ares_nom, T_parent_ares_est)
        finally:
            self._aim_frame = None                   # the guard falls back to the measured / predicted frames

    def _measure_looks(self, parent: str, looks: Sequence[Look], what: str, T_parent_ares_nom: np.ndarray,
                       T_parent_ares_est: np.ndarray) -> FrameFit:
        observed: dict[str, list[np.ndarray]] = {}
        missing: dict[str, str] = {}
        coarse: list[np.ndarray] = []            # T_parent_base from partly seen boards (re-aiming only)
        T_nom = np.asarray(T_parent_ares_nom, float) @ self.T_ares_base
        T_est = np.asarray(T_parent_ares_est, float) @ self.T_ares_base

        def shoot(lk: Look, tag: str, T_aim: np.ndarray | None = None) -> None:
            nonlocal T_est
            T_pb = T_est if T_aim is None else T_aim
            aimed = self._aim(lk, T_nom, T_pb)
            self._aim_frame = (parent, T_pb)         # the guard's frame of `parent` = the one the look is aimed with
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
            rec = self.log.write("shot", parent=parent, look=lk.name + tag, T_base_flange=shot.T_base_flange,
                                 image=img_rel, max_qd=getattr(shot, "max_qd", None), boards=boards)
            if self.on_shot is not None:
                try:
                    self.on_shot(shot.frame.image, rec)
                except Exception as e:               # noqa: BLE001 - a display tap must never stop the run
                    log.warning("on_shot failed: %s", e)
            if any(v["ok"] for v in boards.values()):
                obs = {b: Ts[-1] for b, Ts in observed.items()}
                T_est = g.inv(fit_frame(obs, self.placements, self.specs, parent).T_base_parent)
            elif not observed:
                # a board only partly in the image (ARES further off than the prediction, e.g. after a long route):
                # its rough position re-aims the retries - it never enters the frame fit
                part = [b for b in lk.boards if not boards[b]["ok"]]
                rough = coarse_poses(shot.frame.image, [self.specs[b] for b in part], self.intr, self.vcfg)
                if rough:
                    # only the POSITION of the rough pose is used (a pose from one marker can be tilted by degrees):
                    # shift the estimate horizontally so that the board's predicted centre meets the measured one
                    b, bp = next(iter(rough.items()))
                    c = [[*self.specs[b].centre_mm, 0.0]]
                    p_meas = g.apply(shot.T_base_flange @ self.T_flange_cam @ bp.T_cam_board, c)[0]
                    pl_b = next(p_ for p_ in self.placements if p_.name == b)
                    p_pred = g.apply(g.inv(T_est) @ pl_b.T_parent_board, c)[0]
                    d = p_meas - p_pred
                    T_est = T_est @ g.transl(-d[0], -d[1], 0.0)
                    coarse.append(T_est)
                    self.log.write("coarse_aim", parent=parent, look=lk.name + tag, board=b,
                                   n_corners=bp.n_corners, rms_px=bp.rms_px,
                                   ares=Pose2D.from_T(T_est @ self.T_base_ares))

        for lk in looks:
            shoot(lk, "")
        if missing and not observed and not coarse:
            self._search(looks, parent, T_nom, T_est, shoot, coarse, observed)
            if coarse:
                T_est = coarse[-1]
        if missing:
            for lk in [lk for lk in looks if any(b in missing for b in lk.boards)]:
                shoot(lk, " retry")
        if missing and (self.p.require_all_boards or not observed):
            raise self._fail_measurement(f"{what}: board(s) not measured: "
                                         + "; ".join(f"{b} ({r})" for b, r in sorted(missing.items())))
        obs = {b: (Ts[0] if len(Ts) == 1 else g.average_T(Ts)) for b, Ts in observed.items()}
        fit = fit_frame(obs, self.placements, self.specs, parent)
        if fit.rms_mm > self.p.max_fit_rms_mm:
            legs = sorted({str(self.board_leg[b]) for b in fit.boards if self.board_leg.get(b)})
            cause = (f"the boards lie on legs {' and '.join(legs)}: one leg may stand off its nominal pose "
                     "([[wall.legs]], its base blocks) - or a board moved / the hand-eye calibration is off"
                     if len(legs) > 1 else "a board moved or the hand-eye calibration is off")
            raise self._fail_measurement(f"{what}: frame fit residual RMS {fit.rms_mm:.2f} mm > "
                                         f"{self.p.max_fit_rms_mm:g} mm (boards {fit.boards}) - {cause}",
                                         fit=fit.to_dict())
        return fit

    def _search(self, looks: Sequence[Look], parent: str, T_nom: np.ndarray, T_est: np.ndarray, shoot, coarse: list,
                observed: dict) -> None:
        """Nothing usable of the expected boards in view: shoot the looks again for ARES assumed displaced by
        +-k * search_step_mm (k = 1..search_rings, 8 directions in the parent frame), skipping poses without a
        nominal-kinematics IK solution in the planner's family, until a board is measured or partly seen."""
        from .simworld import ik_near
        q_ref = np.asarray(self.job.park_q_rad, float)
        dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        self.log.write("board_search", parent=parent, looks=[lk.name for lk in looks])
        for k in range(1, int(self.p.search_rings) + 1):
            for dx, dy in dirs:
                n = math.hypot(dx, dy)
                off = k * self.p.search_step_mm / n
                T_try = g.transl(dx * off, dy * off, 0.0) @ T_est
                for lk in looks:
                    aimed = self._aim(lk, T_nom, T_try)
                    if aimed.T_base_flange is not None and ik_near(aimed.T_base_flange, q_ref) is None:
                        continue
                    shoot(lk, f" search {k}{'+' if dx > 0 else '-' if dx < 0 else '0'}"
                              f"{'+' if dy > 0 else '-' if dy < 0 else '0'}", T_try)
                    if observed or coarse:
                        return

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
            self._wall_meas = (self.T_base_wall, self.pose_est)
            self.log.write("wall_frame", stop=k, why=why, source="dead_reckoning", ares=self.pose_est)
            self._park()
            return
        # after a station trip the prediction rests on the station estimate: the station limits apply; after a route
        # (several dead-reckoned moves and a large rotation) the route limits
        lim = ((self.p.station_max_jump_mm, self.p.station_max_jump_deg) if why == "after reload"
               else (self.p.route_max_jump_mm, self.p.route_max_jump_deg) if why == "after route"
               else (self.p.max_jump_mm, self.p.max_jump_deg))
        for attempt in range(self.p.max_corrections + 1):
            pred = self.pose_est
            fit = self._measure("wall", stop.looks, f"stop {k} wall", stop.ares.T, pred.T)
            T_wa = T_parent_ares(fit.T_base_parent, self.T_ares_base)
            meas = Pose2D.from_T(T_wa)
            d_mm, d_deg = self._check_jump(meas, pred, *lim, f"stop {k} wall")
            self._anchor("wall", T_wa)
            self.pose_est, self.pose_src, self.pose_status = meas, "measured", "ok"
            self.pending_why = None
            self.T_base_wall = fit.T_base_parent
            self._wall_meas = (self.T_base_wall, meas)      # measured 6-DoF frame (tilt, height) + its ARES pose
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

    def _upcoming_kinds(self) -> list[str]:
        """Types of the stones not placed yet, in job order (what the magazine has to supply next)."""
        return [t.kind for t in self.job.stones() if t.key not in self.placed]

    def _reload(self, k: int, resume: RouteProgress | None = None) -> None:
        """Station trip from stop k. resume: the interrupted route to the station (continue it from its next leg)."""
        stop = self.job.stops[k]
        st = self.job.station
        if resume is None:
            self.result.reloads += 1
        n_need = self.magazine.n_free
        upcoming = self._upcoming_kinds()
        if self.station.empty() or reload_short(self.station, SlotState.station(st), self.magazine, upcoming):
            self.log.write("station_empty", station=len(self.station), kinds={k_: self.station.count(k_)
                                                                              for k_ in ("full", "half")})
            if self.on_station_empty is None:
                raise SequencerError("pick-up station is empty (or short of the next stone types) - refill it "
                                     "(operator), then resume", k)
            self.on_station_empty()
            self.station = SlotState.station(st)
            self.log.write("station_refilled", station=len(self.station))
        plan = reload_plan(self.magazine, self.station, upcoming)
        if not plan:
            raise SequencerError(f"pick-up station holds no {upcoming[0] if upcoming else ''} stone for the next "
                                 "stone - refill it, then resume", k)
        self.log.write("reload_start", stop=k, magazine=len(self.magazine), station=len(self.station),
                       plan=[list(x) for x in plan], resumed=resume is not None)
        self._park()
        self.at_station = True
        self.T_base_station = None                   # measured anew at this dock (guard: predicted until then)
        if stop.route_to_station:
            if resume is not None:
                self._follow_route(resume.route, resume.why, target=self.dock_in_wall(), kind="to_station", stop=k,
                                   start_leg=resume.next_leg, resume=True)
            else:
                self._follow_route(stop.route_to_station, f"stop {k} -> station", target=self.dock_in_wall(),
                                   kind="to_station", stop=k)
        else:
            dock = self.dock_in_wall()
            dist = math.hypot(dock.x_mm - self.pose_est.x_mm, dock.y_mm - self.pose_est.y_mm)
            self._warn(f"route to the pick-up station: ARES drives {dist:.0f} mm and turns "
                       f"{abs(math.degrees(wrap_angle(dock.theta_rad - self.pose_est.theta_rad))):.0f} deg - keeping "
                       "the path clear is the operator's responsibility")
            self._drive_to(dock, stop.ares if resume is None else None, f"stop {k} -> station", rotate_first=False)
        T_base_station = self._measure_station(k)
        self.T_base_station = T_base_station
        n = 0
        for sid, mid, kind in plan:
            self._robot("pick_station", f"pick station slot {sid} ({kind})", T_base_station, st.slot(sid))
            self.station.take(sid)
            self._set_held({"from": "station", "slot": sid, "kind": kind, "stone": None, "unknown": False})
            self._robot("place_magazine", f"place magazine slot {mid} ({kind})", self.job.magazine.slot(mid),
                        self.T_base_ares, kind)
            self.magazine.fill(mid, kind)
            self._set_held(None)
            n += 1
        self._park()
        self.log.write("reload_transfer", moved=n, wanted=n_need, magazine=len(self.magazine),
                       station=len(self.station))
        self._return_from_station(k)
        self._measure_wall(k, "after reload")
        self.log.write("reload_done", stop=k)

    def _return_from_station(self, k: int, resume: RouteProgress | bool | None = None) -> None:
        """Drive back from the station to stop k (route_from_station, ending at the arrival standoff). resume: the
        interrupted return (continue from its next leg), or True = ARES stands at the dock after an interrupted
        reload (the whole route, with the resume check of its first move)."""
        stop = self.job.stops[k]
        if stop.route_from_station:
            pr = resume if isinstance(resume, RouteProgress) else None
            self._follow_route(stop.route_from_station if pr is None else pr.route, f"station -> stop {k}",
                               kind="from_station", stop=k, start_leg=0 if pr is None else pr.next_leg,
                               resume=bool(resume), standoff_mm=self._standoff())
        else:
            self._drive_to(stop.ares, None if resume else self.dock_in_wall(), f"station -> stop {k}",
                           rotate_first=True)
        self.at_station = False

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
            self.pose_src, self.pose_status = "measured (station)", "ok"
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
        mid = self.magazine.next_take(t.slot, kind=t.kind)
        if mid is None:
            raise SequencerError(f"no {t.kind} stone can be taken from the magazine (empty after a reload, or covered "
                                 "by stones of the other type)", k, t.key)
        if t.slot is not None and mid != t.slot:
            self.log.write("slot_changed", stone=t.key, planned=t.slot, used=mid)
        self._robot("pick_magazine", f"pick magazine slot {mid} ({t.kind})", self.job.magazine.slot(mid),
                    self.T_base_ares, t.kind)
        self.magazine.take(mid)
        self._set_held({"from": "magazine", "slot": mid, "kind": t.kind, "stone": list(t.key), "unknown": False})
        T_cmd = self.T_base_wall @ t.T_wall_tcp
        self._robot("place_wall", f"place stone {t.label} (u {t.u_mm:.0f} mm, top {t.z_top_mm:.0f} mm)",
                    self.T_base_wall, t)
        self.placed.add(t.key)
        self.result.placed.append(t.key)
        self._set_held(None)                      # after placed: a snapshot at 'held' shows the stone in the wall
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
        if self.held is not None:
            raise SequencerError(f"a stone may be in the jaws ({self.held}) - take it out / check the gripper, park "
                                 "the arm, then clear_held() and resume", start_stop)
        self.result.state = "running"
        self.result.error = None
        self.log.write("run_start", start_stop=start_stop, last_stop=last, camera_loop=self.camera_loop,
                       params=asdict(self.p), job_source=self.job.source, config_sha256=self.job.config_sha256,
                       n_stones=self.job.n_stones, magazine=len(self.magazine))
        resumed = self.pose_est is not None and self.stop_k == start_stop
        if not resumed:
            earlier = {t.key for st in self.job.stops[:start_stop] for t in st.stones}
            if earlier - self.placed - self._prebuilt:   # a run started later: those stones stand (the guard must see
                self._prebuilt |= earlier                # them; review 2026-10-07)
                self.log.write("prebuilt", start_stop=start_stop, stones=sorted(earlier))
            if (self.placed | self._prebuilt) and not self.result.placed:   # not the job's first stone: the operator
                fill = restart_fill(self.job, self.placed | self._prebuilt)  # loaded the magazine for the next stones
                self.magazine = SlotState.magazine(self.job.magazine, filled=[sid for sid, _ in fill],
                                                   kinds=dict(fill))
                self.station = SlotState.station(self.job.station)
                self.log.write("restart_fill", magazine=[list(x) for x in fill])
            self.pose_est, self.pose_src = self.job.stops[start_stop].ares, "assumed (start mark)"
            self.at_station = False
            self.route_progress = None
            self.pending_why = None
            self.pose_status = "ok"
        try:
            if resumed and self.pose_status != "ok":
                raise SequencerError(
                    f"resume refused: the ARES pose is {'unknown' if self.pose_status == 'unknown' else 'unverified'}"
                    f" after an ARES move that did not end ok (estimate {self.pose_est.describe()}, {self.pose_src})"
                    " - check ARES on the floor, then confirm_pose() or set_pose(pose)", start_stop)
            self._park()
            # stopped after a route / return but before the wall was measured: keep its jump limits
            why0, measured = (self.pending_why if resumed and self.pending_why else "arrival"), False
            pr = self.route_progress if resumed else None
            if pr is not None:                         # stopped on the way: finish THAT route first
                self.log.write("resume_route", route=pr.to_dict(), pose=self.pose_est, pose_src=self.pose_src)
                if pr.kind == "to_station":
                    self._reload(pr.stop, resume=pr)    # on to the station, reload, back, measure
                    measured = True
                elif pr.kind == "from_station":
                    self._return_from_station(pr.stop, resume=pr)
                    why0 = "after reload"
                else:
                    self._follow_route(pr.route, pr.why, kind="stop", stop=pr.stop, start_leg=pr.next_leg,
                                       resume=True, standoff_mm=pr.standoff_mm)
                    why0 = "after route"
            elif resumed and self.at_station:          # stopped at the dock (reload): back to the stop first
                self._return_from_station(start_stop, resume=True)
                why0 = "after reload"
            for k in range(start_stop, last + 1):
                self.stop_k = k
                stop = self.job.stops[k]
                self.log.write("stop_start", stop=k, a_mm=stop.a_mm, nominal=stop.ares, n_stones=len(stop.stones))
                why = why0 if k == start_stop else "arrival"
                if k != start_stop:
                    prev = self.job.stops[k - 1].ares
                    self._park()
                    if stop.route:
                        self._follow_route(stop.route, f"stop {k - 1} -> stop {k}", kind="stop", stop=k,
                                           standoff_mm=self._standoff())
                        why = "after route"
                    else:
                        # kept as a two-waypoint route while it runs: an interrupted direct move (declined, paused,
                        # ARES error) is resumed like a route leg instead of measuring stop k from stop k - 1
                        self.route_progress = RouteProgress("stop", k, [prev, stop.ares], f"stop {k - 1} -> stop {k}")
                        self._drive_to(stop.ares, prev, f"stop {k - 1} -> stop {k}")
                        self.route_progress = None
                if not (k == start_stop and measured):
                    self._measure_wall(k, why)
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
        except Exception as e:                        # anything else (CameraError, a bug): a defined end state
            self.result.state, self.result.error = "error", f"{type(e).__name__}: {e}"
            self.log.write("run_error", error=str(e), stop=self.stop_k, kind=type(e).__name__)
            raise SequencerError(f"{type(e).__name__}: {e}", self.stop_k) from e

    def close(self) -> None:
        self.log.close()
