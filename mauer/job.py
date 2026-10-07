"""Job file v2: what the sequencer executes - stops, look poses, stones, magazine, pick-up station, ARES routes (JSON).

A job is exported by the RoboDK planner (collision-checked joints, via points) or built without RoboDK by
tools/make_job.py (`build_nominal`: config + robodk/wallplan.py + results/reach_table.json, Cartesian look poses, a
kinematic IK check only). The sequencer (mauer.sequencer) runs it against real or simulated backends.

Frames and units (docs/ARCHITECTURE.md): T_a_b = pose of frame b in frame a, 4x4 nested lists (row-major) in the
JSON, mm and rad; joint angles in rad; planar ARES poses {"x_mm", "y_mm", "theta_rad"} (ARES base_link in a
floor-level parent frame, theta = heading of the ARES x axis). "wall" = wall frame (origin at the start of course 0
on the wall centreline, floor level, x along the wall = ARES travel direction, y towards ARES, z up); "station" =
pick-up station frame; "ares" = ARES base_link; "base" = UR5 base.

JSON layout (format "ares-mauer-job", version 2; version 1 = the same without the fields marked v2, still loaded):
{
  "format": "ares-mauer-job", "version": 2,
  "created": iso8601, "source": "who/what built it",
  "config_sha256": sha256 of config/station.toml at build time,
  "depends_on": [{"key": "[ur5] mount_z", "status": "PLACEHOLDER", "value": 333.6}, ...],  # non-CONFIRMED inputs
  "T_flange_tcp": 4x4, "T_ares_base": 4x4,          # as used for the plan (config.T_flange_tcp / T_ares_base)
  "park_q_rad": [6],                                # joint park pose: ARES only moves with the arm here
  "approach_mm": 150.0,                             # vertical approach/retract above every pick/place pose
  "legs": [{"name": "A", "n0": 12, "T_wall_leg": 4x4}],   # v2: wall legs ([] = straight wall)
  "stops": [{
     "index": 0, "a_mm": 920.0,                     # wallplan: ARES centre at wall (leg) position a
     "leg": "A" | null,                             # v2: leg of the stop (null = straight wall)
     "ares": {"x_mm", "y_mm", "theta_rad"},         # nominal ARES pose in the WALL frame
     "route": [pose, ...],                          # v2: waypoints (wall frame) from the previous stop's nominal pose
                                                    #     to this one ([] = direct closed-loop move, v1 behaviour)
     "route_to_station": [pose, ...],               # v2: this stop -> nominal dock (wall frame); [] = v1 behaviour
     "route_from_station": [pose, ...],             # v2: nominal dock -> this stop
     "looks": [{"name": "S0-W1", "boards": ["W1"],  # boards expected in the image
                "T_base_flange": 4x4 | null,        # nominal flange pose (base frame, nominal ARES pose)
                "q_rad": [6] | null,                # exact joint target (planner), used instead of the pose
                "qnear_rad": [6] | null}],          # IK branch hint for the controller's get_inverse_kin
     "stones": [{"course": 0, "index": 0, "u_mm": 100.0, "z_top_mm": 140.0,
                 "leg": "A" | null, "kind": "full" | "half", "length_mm": 200.0,   # v2
                 "T_wall_tcp": 4x4,                 # place pose = T_wall_leg @ transl(u, 0, z_top) @ rotx(pi)
                                                    #   [@ rotz(pi) if flip]; TCP = top centre of the stone
                 "flip": false, "slot": "m03" | null,   # magazine slot the stone is taken from (planned)
                 "qnear_rad": [6] | null,           # IK hint of the place pose
                 "via_q_rad": [[6], ...]}]}],       # joint waypoints of the transfer magazine -> wall (planner)
  "magazine": {"slots": [{"id": "m00", "T_ares_tcp": 4x4, "layer": 1, "stack": "r0y0",
                          "qnear_rad": [6] | null, "ik_ok": true | false | null,
                          "kind": "full" | "half"}],        # 2026-10-07: holder type (missing = either type)
               "take_order": [ids], "fill_order": [ids], "initial_fill": [ids],
               "initial_kinds": {id: "full" | "half"}},   # v2: stone type in each initially filled slot
  "station": {"T_wall_station": 4x4,                # nominal station frame in the wall frame
              "dock": {"x_mm", "y_mm", "theta_rad"},   # ARES pose in the STATION frame when docked
              "boards": ["S0", "S1"],
              "slots": [{"id": "s00l1", "T_station_tcp": 4x4, "qnear_rad": [6] | null, "ik_ok": ...,
                         "kind": "full" | "half",          # v2
                         "layer": 1, "stack": "s00"}],     # v2: stacked holders (missing = layer 1, own stack)
              "take_order": [ids], "looks": [look, ...]},   # looks: base frame at the nominal dock pose
  "meta": {"wall_dist_mm", "length_stones", "look_source": "nominal" | "planner",
           "reach_check": "none" | "kinematic" | "robodk", "warnings": [...], ...}
}
`validate(job)` lists every problem (empty list = valid); `load` raises JobError for an invalid file.

Magazine stacking: slots of one `stack` lie on top of each other (layer 1 = lowest). A slot can only be emptied when
no filled slot lies above it and only be filled when the slot below holds a stone (`SlotState`); robodk/simulate.py
empties the magazine layer by layer, highest first (simulate.py:122-149). Station holders stack the same way (v2
`layer` / `stack`, one stone type per stack; a v1 file or a slot without them = layer 1 in its own stack). Deck
slots: since 2026-10-07 every slot has a `kind` (4-cone holders behind the UR: one full stone, or two half stones end
to end in the [deck] half_positions); in an older file without kinds every slot holds either type. `SlotState` tracks the type per
filled slot; `fill_plan` / `reload_plan` load the empty magazine with the next stones in placement order - as many as
fit the free slots of each type and the station holds (shared by tools/make_job.py and the sequencer).

Routes (v2): waypoint lists in the wall frame; consecutive waypoints differ by a pure translation or a pure rotation
(PLC v2.9 relative move, mauer/floor.py). The sequencer drives the intermediate legs dead-reckoned (nominal relative
moves) and closes the loop only on the last leg to the stop / dock. A job with legs must have a route for every leg
change and both station routes of every stop (validate); their clearance is re-checked against the current config by
mauer.sequencer.preflight_real (route_problems).
"""
from __future__ import annotations

import datetime as _dt
import fnmatch
import hashlib
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from .config import STATION_TOML
from .reference import Pose2D, wrap_angle

JOB_FORMAT = "ares-mauer-job"
JOB_VERSION = 2
SUPPORTED_VERSIONS = (1, 2)
KINDS = ("full", "half")
REACH_CHECKS = ("none", "kinematic", "robodk")
STATUS_TAGS = ("CONFIRMED", "ASSUMPTION", "PLACEHOLDER", "UNKNOWN")


class JobError(ValueError):
    """Invalid job file; `problems` lists every reason."""

    def __init__(self, problems: Sequence[str]):
        self.problems = list(problems)
        super().__init__("invalid job:\n  - " + "\n  - ".join(self.problems))


def now_iso() -> str:
    return _dt.datetime.now().astimezone().isoformat(timespec="seconds")


# ── dataclasses ───────────────────────────────────────────────────────────────
@dataclass
class Look:
    """One image pose: the camera should see `boards`. T_base_flange is nominal (nominal ARES pose); q_rad, when
    given by a planner, is the exact joint target; qnear_rad is only the IK branch hint."""
    name: str
    boards: list[str]
    T_base_flange: np.ndarray | None = None
    q_rad: list[float] | None = None
    qnear_rad: list[float] | None = None


@dataclass
class StoneTask:
    course: int
    index: int
    u_mm: float
    z_top_mm: float
    T_wall_tcp: np.ndarray
    flip: bool = False
    slot: str | None = None
    qnear_rad: list[float] | None = None
    via_q_rad: list[list[float]] = field(default_factory=list)
    leg: str | None = None                 # v2: wall leg (None = straight wall)
    kind: str = "full"                     # v2: "full" | "half"
    length_mm: float | None = None         # v2: stone length along the leg

    @property
    def key(self) -> tuple:
        return (self.course, self.index) if self.leg is None else (self.leg, self.course, self.index)

    @property
    def label(self) -> str:
        return f"{self.leg or ''}c{self.course}i{self.index}" + ("h" if self.kind == "half" else "")


@dataclass
class Stop:
    index: int
    a_mm: float
    ares: Pose2D
    looks: list[Look]
    stones: list[StoneTask]
    leg: str | None = None                                   # v2
    route: list[Pose2D] = field(default_factory=list)        # v2: previous stop -> this stop ([] = direct)
    route_to_station: list[Pose2D] = field(default_factory=list)     # v2: this stop -> dock ([] = v1 behaviour)
    route_from_station: list[Pose2D] = field(default_factory=list)   # v2: dock -> this stop


@dataclass
class MagazineSlot:
    id: str
    T_ares_tcp: np.ndarray
    layer: int
    stack: str
    qnear_rad: list[float] | None = None
    ik_ok: bool | None = None
    kind: str = ""                         # 2026-10-07: stone type this holder takes ("" = either, older files)


@dataclass
class Magazine:
    slots: list[MagazineSlot]
    take_order: list[str]
    fill_order: list[str]
    initial_fill: list[str]
    initial_kinds: dict[str, str] = field(default_factory=dict)     # v2: slot -> "full" | "half" (missing = full)

    @property
    def capacity(self) -> int:
        return len(self.take_order)

    def slot(self, sid: str) -> MagazineSlot:
        for s in self.slots:
            if s.id == sid:
                return s
        raise KeyError(f"no magazine slot {sid!r}")


@dataclass
class StationSlot:
    id: str
    T_station_tcp: np.ndarray
    qnear_rad: list[float] | None = None
    ik_ok: bool | None = None
    kind: str = "full"                     # v2: stone type this holder takes
    layer: int = 1                         # v2: 1 = on the table, n = on the stone of layer n - 1 of the same stack
    stack: str = ""                        # v2: holder position the layers share ("" = a stack of its own)

    @property
    def stack_id(self) -> str:
        return self.stack or self.id


@dataclass
class Station:
    T_wall_station: np.ndarray
    dock: Pose2D
    boards: list[str]
    slots: list[StationSlot]
    take_order: list[str]
    looks: list[Look]

    def slot(self, sid: str) -> StationSlot:
        for s in self.slots:
            if s.id == sid:
                return s
        raise KeyError(f"no station slot {sid!r}")

    @property
    def dock_in_wall(self) -> Pose2D:
        """Nominal docking pose of ARES in the wall frame."""
        return Pose2D.from_T(np.asarray(self.T_wall_station, float) @ self.dock.T)


@dataclass
class Job:
    stops: list[Stop]
    magazine: Magazine
    station: Station
    T_flange_tcp: np.ndarray
    T_ares_base: np.ndarray
    park_q_rad: list[float]
    approach_mm: float
    config_sha256: str = ""
    depends_on: list[dict] = field(default_factory=list)
    source: str = ""
    created: str = field(default_factory=now_iso)
    version: int = JOB_VERSION
    meta: dict = field(default_factory=dict)
    legs: list[dict] = field(default_factory=list)          # v2: [{"name", "n0", "T_wall_leg"}]

    @property
    def n_stones(self) -> int:
        return sum(len(s.stones) for s in self.stops)

    def stones(self) -> list[StoneTask]:
        return [t for s in self.stops for t in s.stones]

    def summary(self) -> str:
        m = self.meta
        n_half = sum(t.kind == "half" for t in self.stones())
        lines = [f"job v{self.version} ({self.source}), {len(self.stops)} stops, {self.n_stones} stones"
                 + (f" ({n_half} half)" if n_half else "") + f", magazine "
                 f"{self.magazine.capacity} slots, station {len(self.station.take_order)} slots, look source "
                 f"{m.get('look_source', '?')}, reach check {m.get('reach_check', '?')}"]
        if self.legs:
            lines.append("  legs: " + ", ".join(f"{lg['name']} ({lg['n0']} stones in course 0, origin "
                                                 f"({lg['T_wall_leg'][0][3]:.0f}, {lg['T_wall_leg'][1][3]:.0f}) mm)"
                                                 for lg in self.legs))
        prev = None
        for s in self.stops:
            mv = ("" if prev is None else
                  f", ARES move {math.hypot(s.ares.x_mm - prev.x_mm, s.ares.y_mm - prev.y_mm):.0f} mm")
            if s.route:
                mv += f" via a {len(s.route)}-waypoint route"
            leg = f"leg {s.leg} " if s.leg else ""
            lines.append(f"  stop {s.index}: {leg}a = {s.a_mm:.0f} mm, ARES {s.ares.describe()} (wall), "
                         f"{len(s.stones)} stones, looks {[(lk.name, lk.boards) for lk in s.looks]}{mv}")
            prev = s.ares
        st = self.station
        lines.append(f"  station: dock {st.dock_in_wall.describe()} (wall), boards {st.boards}, looks "
                     f"{[(lk.name, lk.boards) for lk in st.looks]}")
        for w in m.get("warnings", []):
            lines.append(f"  WARNING: {w}")
        return "\n".join(lines)


# ── JSON ──────────────────────────────────────────────────────────────────────
def _m(T) -> list | None:
    return None if T is None else np.asarray(T, float).tolist()


def _q(q) -> list | None:
    return None if q is None else [float(v) for v in q]


def _T(x) -> np.ndarray | None:
    return None if x is None else np.asarray(x, float)


def _look_to(lk: Look) -> dict:
    return {"name": lk.name, "boards": list(lk.boards), "T_base_flange": _m(lk.T_base_flange), "q_rad": _q(lk.q_rad),
            "qnear_rad": _q(lk.qnear_rad)}


def _look_from(d: Mapping) -> Look:
    return Look(str(d["name"]), [str(b) for b in d["boards"]], _T(d.get("T_base_flange")), _q(d.get("q_rad")),
                _q(d.get("qnear_rad")))


def _route_to(r: Sequence[Pose2D]) -> list[dict]:
    return [p.to_dict() for p in r]


def _route_from(r) -> list[Pose2D]:
    return [Pose2D.from_dict(p) for p in (r or [])]


def _stone_to(t: StoneTask, v2: bool) -> dict:
    d = {"course": t.course, "index": t.index, "u_mm": float(t.u_mm), "z_top_mm": float(t.z_top_mm),
         "T_wall_tcp": _m(t.T_wall_tcp), "flip": bool(t.flip), "slot": t.slot, "qnear_rad": _q(t.qnear_rad),
         "via_q_rad": [_q(v) for v in t.via_q_rad]}
    if v2:
        d.update(leg=t.leg, kind=t.kind, length_mm=None if t.length_mm is None else float(t.length_mm))
    return d


def _stop_to(s: Stop, v2: bool) -> dict:
    d = {"index": s.index, "a_mm": float(s.a_mm), "ares": s.ares.to_dict(), "looks": [_look_to(lk) for lk in s.looks],
         "stones": [_stone_to(t, v2) for t in s.stones]}
    if v2:
        d.update(leg=s.leg, route=_route_to(s.route), route_to_station=_route_to(s.route_to_station),
                 route_from_station=_route_to(s.route_from_station))
    return d


def to_dict(job: Job) -> dict:
    """JSON dict; a version-1 job (loaded from an old file) is written in the version-1 layout."""
    v2 = job.version >= 2
    d = {
        "format": JOB_FORMAT, "version": job.version, "created": job.created, "source": job.source,
        "config_sha256": job.config_sha256, "depends_on": _jsonable(job.depends_on),
        "T_flange_tcp": _m(job.T_flange_tcp), "T_ares_base": _m(job.T_ares_base),
        "park_q_rad": _q(job.park_q_rad), "approach_mm": float(job.approach_mm),
        "stops": [_stop_to(s, v2) for s in job.stops],
        "magazine": {"slots": [{"id": s.id, "T_ares_tcp": _m(s.T_ares_tcp), "layer": int(s.layer), "stack": s.stack,
                                "qnear_rad": _q(s.qnear_rad), "ik_ok": s.ik_ok, **({"kind": s.kind} if s.kind else {})}
                               for s in job.magazine.slots],
                     "take_order": list(job.magazine.take_order), "fill_order": list(job.magazine.fill_order),
                     "initial_fill": list(job.magazine.initial_fill)},
        "station": {"T_wall_station": _m(job.station.T_wall_station), "dock": job.station.dock.to_dict(),
                    "boards": list(job.station.boards),
                    "slots": [{"id": s.id, "T_station_tcp": _m(s.T_station_tcp), "qnear_rad": _q(s.qnear_rad),
                               "ik_ok": s.ik_ok, **({"kind": s.kind, "layer": int(s.layer), "stack": s.stack_id}
                                                    if v2 else {})} for s in job.station.slots],
                    "take_order": list(job.station.take_order),
                    "looks": [_look_to(lk) for lk in job.station.looks]},
        "meta": _jsonable(job.meta),
    }
    if v2:
        d["magazine"]["initial_kinds"] = dict(job.magazine.initial_kinds)
        d["legs"] = [{"name": str(lg["name"]), "n0": _n0(lg["n0"]), "T_wall_leg": _m(lg["T_wall_leg"])}
                     for lg in job.legs]
    return d


def from_dict(d: Mapping) -> Job:
    """Job from its JSON dict. JobError for a wrong format/version or missing fields (structure only; call
    validate() for the content)."""
    if d.get("format") != JOB_FORMAT:
        raise JobError([f"format {d.get('format')!r} is not {JOB_FORMAT!r}"])
    if d.get("version") not in SUPPORTED_VERSIONS:
        raise JobError([f"version {d.get('version')!r} not supported (this code reads versions "
                        f"{', '.join(map(str, SUPPORTED_VERSIONS))})"])
    try:
        stops = [Stop(int(s["index"]), float(s["a_mm"]), Pose2D.from_dict(s["ares"]),
                      [_look_from(lk) for lk in s["looks"]],
                      [StoneTask(int(t["course"]), int(t["index"]), float(t["u_mm"]), float(t["z_top_mm"]),
                                 _T(t["T_wall_tcp"]), bool(t.get("flip", False)), t.get("slot"),
                                 _q(t.get("qnear_rad")), [_q(v) for v in t.get("via_q_rad") or []],
                                 None if t.get("leg") is None else str(t["leg"]), str(t.get("kind", "full")),
                                 None if t.get("length_mm") is None else float(t["length_mm"]))
                       for t in s["stones"]], None if s.get("leg") is None else str(s["leg"]),
                      _route_from(s.get("route")), _route_from(s.get("route_to_station")),
                      _route_from(s.get("route_from_station"))) for s in d["stops"]]
        m = d["magazine"]
        mag = Magazine([MagazineSlot(str(s["id"]), _T(s["T_ares_tcp"]), int(s["layer"]), str(s["stack"]),
                                     _q(s.get("qnear_rad")), s.get("ik_ok"), str(s.get("kind") or ""))
                        for s in m["slots"]],
                       [str(i) for i in m["take_order"]], [str(i) for i in m["fill_order"]],
                       [str(i) for i in m["initial_fill"]],
                       {str(k): str(v) for k, v in (m.get("initial_kinds") or {}).items()})
        st = d["station"]
        station = Station(_T(st["T_wall_station"]), Pose2D.from_dict(st["dock"]), [str(b) for b in st["boards"]],
                          [StationSlot(str(s["id"]), _T(s["T_station_tcp"]), _q(s.get("qnear_rad")), s.get("ik_ok"),
                                       str(s.get("kind", "full")), int(s.get("layer", 1)), str(s.get("stack") or ""))
                           for s in st["slots"]],
                          [str(i) for i in st["take_order"]], [_look_from(lk) for lk in st["looks"]])
        legs = [{"name": str(lg["name"]), "n0": _n0(lg["n0"]), "T_wall_leg": _T(lg["T_wall_leg"])}
                for lg in d.get("legs") or []]
        return Job(stops, mag, station, _T(d["T_flange_tcp"]), _T(d["T_ares_base"]), _q(d["park_q_rad"]),
                   float(d["approach_mm"]), str(d.get("config_sha256", "")), list(d.get("depends_on") or []),
                   str(d.get("source", "")), str(d.get("created", "")), int(d["version"]), dict(d.get("meta") or {}),
                   legs)
    except (KeyError, TypeError, ValueError) as e:
        raise JobError([f"malformed job: {type(e).__name__}: {e}"]) from e


def save(job: Job, path: str | Path) -> Path:
    """Write the job JSON (parent folders created). Refuses an invalid job (JobError)."""
    problems = validate(job)
    if problems:
        raise JobError(problems)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_dict(job), indent=1) + "\n", encoding="utf-8")
    return path


def load(path: str | Path, check: bool = True) -> Job:
    job = from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
    if check:
        problems = validate(job)
        if problems:
            raise JobError(problems)
    return job


def _jsonable(x: Any) -> Any:
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    if isinstance(x, np.generic):
        return x.item()
    return x


# ── validation ────────────────────────────────────────────────────────────────
def _rigid_problem(T, what: str) -> str | None:
    if T is None:
        return f"{what}: missing pose"
    T = np.asarray(T, float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return f"{what}: not a finite 4x4 matrix"
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0 or not np.allclose(T[3], [0, 0, 0, 1]):
        return f"{what}: not a rigid transform"
    return None


def _q_problem(q, what: str) -> str | None:
    if q is None:
        return None
    a = np.asarray(q, float).reshape(-1)
    if a.shape != (6,) or not np.isfinite(a).all():
        return f"{what}: need 6 finite joint angles [rad]"
    if np.abs(a).max() > 2 * math.pi + 1e-9:
        return f"{what}: joint angle beyond +-2 pi (UR joint range)"
    return None


def _look_problems(lk: Look, what: str) -> list[str]:
    p = []
    if not lk.boards:
        p.append(f"{what}: no expected boards")
    if lk.T_base_flange is None and lk.q_rad is None:
        p.append(f"{what}: needs T_base_flange or q_rad")
    if lk.T_base_flange is not None:
        p.append(_rigid_problem(lk.T_base_flange, f"{what} T_base_flange"))
    p += [_q_problem(lk.q_rad, f"{what} q_rad"), _q_problem(lk.qnear_rad, f"{what} qnear_rad")]
    return [x for x in p if x]


def _order_problems(order: Sequence[str], ids: set[str], what: str) -> list[str]:
    p = []
    unknown = [i for i in order if i not in ids]
    if unknown:
        p.append(f"{what}: unknown slot ids {unknown}")
    if len(set(order)) != len(order):
        p.append(f"{what}: duplicate slot ids")
    return p


def validate(job: Job, boards: Iterable[str] | None = None) -> list[str]:
    """All problems of a job (empty list = valid). `boards`: known board names (e.g. reference.placements) to check
    the expected boards of the looks against."""
    p: list[str] = []
    known = None if boards is None else set(boards)
    if job.version not in SUPPORTED_VERSIONS:
        p.append(f"version {job.version} not in {SUPPORTED_VERSIONS}")
    for T, what in ((job.T_flange_tcp, "T_flange_tcp"), (job.T_ares_base, "T_ares_base")):
        p.append(_rigid_problem(T, what))
    if job.park_q_rad is None:
        p.append("park_q_rad missing")
    else:
        p.append(_q_problem(job.park_q_rad, "park_q_rad"))
    if not (np.isfinite(job.approach_mm) and job.approach_mm > 0):
        p.append(f"approach_mm must be > 0, got {job.approach_mm}")
    if not job.stops:
        p.append("no stops")
    mag_ids = {s.id for s in job.magazine.slots}
    if len(mag_ids) != len(job.magazine.slots):
        p.append("magazine: duplicate slot ids")
    for s in job.magazine.slots:
        p.append(_rigid_problem(s.T_ares_tcp, f"magazine slot {s.id} T_ares_tcp"))
        p.append(_q_problem(s.qnear_rad, f"magazine slot {s.id} qnear_rad"))
    p += _order_problems(job.magazine.take_order, mag_ids, "magazine take_order")
    p += _order_problems(job.magazine.fill_order, mag_ids, "magazine fill_order")
    p += _order_problems(job.magazine.initial_fill, mag_ids, "magazine initial_fill")
    if set(job.magazine.take_order) != set(job.magazine.fill_order):
        p.append("magazine: take_order and fill_order must hold the same slots")
    if not set(job.magazine.initial_fill) <= set(job.magazine.take_order):
        p.append("magazine: initial_fill holds slots that are not in take_order")
    stacks: dict[str, list[int]] = {}
    for s in job.magazine.slots:
        stacks.setdefault(s.stack, []).append(s.layer)
    for k, layers in stacks.items():
        if sorted(layers) != list(range(min(layers), min(layers) + len(layers))):
            p.append(f"magazine stack {k}: layers {sorted(layers)} not contiguous")
    if job.magazine.capacity == 0:
        p.append("magazine: no usable slot")
    fixed = {s.id: s.kind for s in job.magazine.slots if s.kind}
    for s in job.magazine.slots:
        if s.kind and s.kind not in KINDS:
            p.append(f"magazine slot {s.id}: kind {s.kind!r} not in {KINDS}")
    for sid, kind in job.magazine.initial_kinds.items():
        if sid not in job.magazine.initial_fill:
            p.append(f"magazine initial_kinds: slot {sid} is not in initial_fill")
        if kind not in KINDS:
            p.append(f"magazine initial_kinds: slot {sid} kind {kind!r} not in {KINDS}")
        elif sid in fixed and fixed[sid] in KINDS and fixed[sid] != kind:
            p.append(f"magazine initial_kinds: slot {sid} holds {fixed[sid]} stones, not {kind}")
    leg_names = [str(lg.get("name")) for lg in job.legs]
    if len(set(leg_names)) != len(leg_names):
        p.append(f"legs: duplicate names {leg_names}")
    for lg in job.legs:
        p.append(_rigid_problem(lg.get("T_wall_leg"), f"leg {lg.get('name')} T_wall_leg"))
    keys = set()
    for i, st in enumerate(job.stops):
        if st.index != i:
            p.append(f"stop {i}: index {st.index} (must be 0..n-1 in order)")
        if not all(np.isfinite([st.ares.x_mm, st.ares.y_mm, st.ares.theta_rad])):
            p.append(f"stop {i}: non-finite ARES pose")
        if not st.looks:
            p.append(f"stop {i}: no look poses")
        for lk in st.looks:
            p += _look_problems(lk, f"stop {i} look {lk.name}")
            if known is not None and not set(lk.boards) <= known:
                p.append(f"stop {i} look {lk.name}: unknown boards {sorted(set(lk.boards) - known)}")
        if st.leg is not None and st.leg not in leg_names:
            p.append(f"stop {i}: unknown leg {st.leg!r}")
        p += _route_problems(st.route, job.stops[i - 1].ares if i > 0 else None, st.ares, f"stop {i} route")
        if st.route and i == 0:
            p.append("stop 0: has a route from a previous stop")
        if job.legs:
            # an L job without routes would fall back to the v1 direct moves: rotating on the spot over the plates and
            # driving straight through the legs (review 2026-10-05)
            if i > 0 and not st.route and st.leg != job.stops[i - 1].leg:
                p.append(f"stop {i}: leg change {job.stops[i - 1].leg} -> {st.leg} without a route")
            if not st.route_to_station or not st.route_from_station:
                p.append(f"stop {i}: no route_to_station / route_from_station in a job with legs")
        dock = stn_dock_in_wall(job)
        p += _route_problems(st.route_to_station, st.ares, dock, f"stop {i} route_to_station")
        p += _route_problems(st.route_from_station, dock, st.ares, f"stop {i} route_from_station")
        for t in st.stones:
            what = f"stop {i} stone {t.key}"
            if t.kind not in KINDS:
                p.append(f"{what}: kind {t.kind!r} not in {KINDS}")
            if t.leg is not None and t.leg not in leg_names:
                p.append(f"{what}: unknown leg {t.leg!r}")
            if job.legs and t.leg is None:
                p.append(f"{what}: no leg in a job with legs")
            if t.key in keys:
                p.append(f"{what}: duplicate stone")
            keys.add(t.key)
            p.append(_rigid_problem(t.T_wall_tcp, f"{what} T_wall_tcp"))
            if t.T_wall_tcp is not None and np.asarray(t.T_wall_tcp, float)[2, 2] > -0.99:
                p.append(f"{what}: place pose TCP z must point down (wall frame)")
            if t.slot is not None and t.slot not in mag_ids:
                p.append(f"{what}: unknown magazine slot {t.slot!r}")
            p.append(_q_problem(t.qnear_rad, f"{what} qnear_rad"))
            for j, v in enumerate(t.via_q_rad):
                p.append(_q_problem(v, f"{what} via {j}") if v is not None else f"{what} via {j}: missing")
    stn = job.station
    p.append(_rigid_problem(stn.T_wall_station, "station T_wall_station"))
    if not all(np.isfinite([stn.dock.x_mm, stn.dock.y_mm, stn.dock.theta_rad])):
        p.append("station: non-finite dock pose")
    st_ids = {s.id for s in stn.slots}
    if len(st_ids) != len(stn.slots):
        p.append("station: duplicate slot ids")
    for s in stn.slots:
        p.append(_rigid_problem(s.T_station_tcp, f"station slot {s.id} T_station_tcp"))
        p.append(_q_problem(s.qnear_rad, f"station slot {s.id} qnear_rad"))
        if s.kind not in KINDS:
            p.append(f"station slot {s.id}: kind {s.kind!r} not in {KINDS}")
    p += _order_problems(stn.take_order, st_ids, "station take_order")
    st_stacks: dict[str, list[StationSlot]] = {}
    for s in stn.slots:
        st_stacks.setdefault(s.stack_id, []).append(s)
    usable = set(stn.take_order)
    for k, ss in st_stacks.items():
        layers = sorted(s.layer for s in ss)
        if layers != list(range(1, len(layers) + 1)):
            p.append(f"station stack {k}: layers {layers} must be 1..n without gaps or repeats")
        if len({s.kind for s in ss}) > 1:
            p.append(f"station stack {k}: mixed stone types {sorted({s.kind for s in ss})}")
        for s in ss:
            below = [o.id for o in ss if o.layer < s.layer and o.id not in usable]
            if s.id in usable and below:
                p.append(f"station slot {s.id}: in take_order but the slot(s) below it are not ({below})")
    for lk in stn.looks:
        p += _look_problems(lk, f"station look {lk.name}")
        if not set(lk.boards) <= set(stn.boards):
            p.append(f"station look {lk.name}: boards {lk.boards} not in station boards {stn.boards}")
    if known is not None and not set(stn.boards) <= known:
        p.append(f"station: unknown boards {sorted(set(stn.boards) - known)}")
    rc = job.meta.get("reach_check")
    if rc is not None and rc not in REACH_CHECKS:
        p.append(f"meta.reach_check {rc!r} not in {REACH_CHECKS}")
    return [x for x in p if x]


def stn_dock_in_wall(job: Job) -> Pose2D:
    return job.station.dock_in_wall


def _route_problems(route: Sequence[Pose2D], start: Pose2D | None, end: Pose2D, what: str,
                    tol_mm: float = 0.5, tol_deg: float = 0.01) -> list[str]:
    """Structure of a route (geometry/clearance: mauer.floor.validate_route at build time): >= 2 finite waypoints,
    starts at `start` and ends at `end`, every leg a pure translation or a pure rotation."""
    if not route:
        return []
    p = []
    if len(route) < 2:
        return [f"{what}: needs at least 2 waypoints"]
    if not all(np.isfinite([w.x_mm, w.y_mm, w.theta_rad]).all() for w in route):
        return [f"{what}: non-finite waypoint"]
    for ref, w, name in ((start, route[0], "start"), (end, route[-1], "end")):
        if ref is not None:
            d_mm, d_deg = ref.delta(w)
            if d_mm > tol_mm or d_deg > tol_deg:
                p.append(f"{what}: {name} {w.describe()} is not {ref.describe()}")
    for j, (a, b) in enumerate(zip(route, route[1:])):
        d = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
        th = abs(math.degrees(wrap_angle(b.theta_rad - a.theta_rad)))
        if d >= tol_mm and th >= tol_deg:
            p.append(f"{what} leg {j}: translation AND rotation (the PLC moves one or the other)")
        elif d < tol_mm and th < tol_deg:
            p.append(f"{what} leg {j}: duplicate waypoint")
    return p


# ── slot bookkeeping (shared by tools/make_job.py, the sequencer and the simulation) ─────────────────────────────
class SlotState:
    """Which slots hold a stone, and of which type ("full" / "half"). Stacked slots (magazine, station holders with
    layers) can only be emptied from the top and filled from the bottom; station slots take one type each.
    take()/fill() raise ValueError for an impossible action."""

    def __init__(self, ids_layers: Mapping[str, tuple[str, int]], take_order: Sequence[str],
                 fill_order: Sequence[str], filled: Iterable[str], kinds: Mapping[str, str] | None = None,
                 fixed_kinds: Mapping[str, str] | None = None):
        self.stack = {k: v[0] for k, v in ids_layers.items()}
        self.layer = {k: int(v[1]) for k, v in ids_layers.items()}
        self.take_order = list(take_order)
        self.fill_order = list(fill_order)
        self.filled = set(filled)
        self.fixed = dict(fixed_kinds or {})            # station holders: one stone type per slot
        self.kinds = {k: str((kinds or {}).get(k, self.fixed.get(k, "full"))) for k in self.filled}
        unknown = self.filled - set(self.take_order)
        if unknown:
            raise ValueError(f"filled slots {sorted(unknown)} are not usable slots")

    @classmethod
    def magazine(cls, mag: Magazine, filled: Iterable[str] | None = None,
                 kinds: Mapping[str, str] | None = None) -> "SlotState":
        """Deck magazine; slots with a `kind` take only that stone type (2026-10-07), the others either."""
        return cls({s.id: (s.stack, s.layer) for s in mag.slots}, mag.take_order, mag.fill_order,
                   mag.initial_fill if filled is None else filled, mag.initial_kinds if kinds is None else kinds,
                   {s.id: s.kind for s in mag.slots if s.kind})

    @classmethod
    def station(cls, st: Station, filled: Iterable[str] | None = None) -> "SlotState":
        """Station holders: one stone type per slot, stacked like the magazine when the job has layers (v2)."""
        fixed = {s.id: s.kind for s in st.slots}
        return cls({s.id: (s.stack_id, s.layer) for s in st.slots}, st.take_order, list(reversed(st.take_order)),
                   st.take_order if filled is None else filled, fixed, fixed)

    def copy(self) -> "SlotState":
        out = SlotState.__new__(SlotState)
        out.stack, out.layer = self.stack, self.layer
        out.take_order, out.fill_order = self.take_order, self.fill_order
        out.filled, out.kinds, out.fixed = set(self.filled), dict(self.kinds), self.fixed
        return out

    def count(self, kind: str) -> int:
        return sum(1 for k in self.filled if self.kinds.get(k, "full") == kind)

    def __len__(self) -> int:
        return len(self.filled)

    @property
    def capacity(self) -> int:
        return len(self.take_order)

    @property
    def n_free(self) -> int:
        return self.capacity - len(self.filled)

    def empty(self) -> bool:
        return not self.filled

    def _above(self, sid: str) -> list[str]:
        return [k for k in self.filled if self.stack[k] == self.stack[sid] and self.layer[k] > self.layer[sid]]

    def can_take(self, sid: str) -> bool:
        return sid in self.filled and not self._above(sid)

    def can_fill(self, sid: str) -> bool:
        if sid in self.filled or sid not in self.take_order:
            return False
        below = [k for k in self.stack if self.stack[k] == self.stack[sid] and self.layer[k] < self.layer[sid]]
        return all(k in self.filled for k in below)

    def next_take(self, prefer: str | None = None, kind: str | None = None) -> str | None:
        """`prefer` if it can be taken (and holds `kind`), else the first slot in take order that can be taken and
        holds `kind` (any kind if None)."""
        ok = lambda k: self.can_take(k) and (kind is None or self.kinds.get(k, "full") == kind)   # noqa: E731
        if prefer is not None and ok(prefer):
            return prefer
        return next((k for k in self.take_order if ok(k)), None)

    def next_fill(self, kind: str | None = None) -> str | None:
        return next((k for k in self.fill_order if self.can_fill(k)
                     and (kind is None or self.fixed.get(k, kind) == kind)), None)

    def take(self, sid: str) -> str:
        """Empty `sid`; returns the type of the stone taken."""
        if not self.can_take(sid):
            raise ValueError(f"slot {sid} cannot be emptied (empty or covered)")
        self.filled.discard(sid)
        return self.kinds.pop(sid, "full")

    def fill(self, sid: str, kind: str = "full") -> None:
        if not self.can_fill(sid):
            raise ValueError(f"slot {sid} cannot be filled (full or the slot below is empty)")
        if sid in self.fixed and self.fixed[sid] != kind:
            raise ValueError(f"slot {sid} holds {self.fixed[sid]} stones, not {kind}")
        self.filled.add(sid)
        self.kinds[sid] = kind


def _prefix(kinds: Sequence[str], n_max: int, supply: Mapping[str, int]) -> int:
    """Longest prefix of `kinds` (at most n_max) the supply covers."""
    used: dict[str, int] = {}
    n = 0
    for k in kinds[:n_max]:
        if used.get(k, 0) + 1 > supply.get(k, 0):
            break
        used[k] = used.get(k, 0) + 1
        n += 1
    return n


def _loadable(mag: SlotState, upcoming: Sequence[str], supply: Mapping[str, int]) -> int:
    """How many of the next stones the empty magazine takes: limited by the free slots (per type when the slots are
    typed) and by `supply` (stones per type available)."""
    if mag.fixed:
        free = {k: sum(1 for s in mag.take_order if s not in mag.filled and mag.fixed.get(s) == k) for k in KINDS}
        return _prefix(list(upcoming), len(upcoming), {k: min(free[k], supply.get(k, 0)) for k in KINDS})
    return _prefix(list(upcoming), mag.n_free, supply)


def fill_plan(mag: SlotState, upcoming: Sequence[str], supply: Mapping[str, int] | None = None
              ) -> list[tuple[str, str]]:
    """Slots and types to fill the EMPTY magazine with the next stones (`upcoming` = their types in placement order),
    [(magazine slot, kind), ...] in fill order (bottom first): as many stones as `_loadable` (supply None = unlimited,
    the initial fill). Typed slots: each stone into a free slot of its type. Untyped slots (older jobs): the next slots
    in fill order, each with the type of the stone the sequencer will take from it (replaying the takes - first slot
    that can be taken in take order - the j-th take meets the j-th upcoming type)."""
    if not mag.empty():
        raise ValueError("fill_plan: the magazine is not empty")
    supply = {k: len(upcoming) for k in KINDS} if supply is None else supply
    n = _loadable(mag, upcoming, supply)
    m = mag.copy()
    if mag.fixed:
        out = []
        for k in upcoming[:n]:
            sid = m.next_fill(kind=k)
            if sid is None:
                break
            m.fill(sid, k)
            out.append((sid, k))
        return sorted(out, key=lambda x: mag.fill_order.index(x[0]))
    slots = []
    for _ in range(n):
        sid = m.next_fill()
        if sid is None:
            break
        m.fill(sid, "?")
        slots.append(sid)
    kind_of: dict[str, str] = {}
    t = m.copy()
    for j in range(len(slots)):
        sid = t.next_take()
        t.take(sid)
        kind_of[sid] = upcoming[j]
    return [(sid, kind_of[sid]) for sid in slots]


def reload_plan(mag: SlotState, station: SlotState, upcoming: Sequence[str]) -> list[tuple[str, str, str]]:
    """One reload of the EMPTY magazine from the station: [(station slot, magazine slot, kind), ...] in fill order -
    `fill_plan` with the station's stones as the supply (a missing type ends the batch)."""
    st = station.copy()
    out = []
    for sid, k in fill_plan(mag, upcoming, {k: station.count(k) for k in KINDS}):
        ssid = st.next_take(kind=k)
        st.take(ssid)
        out.append((ssid, sid, k))
    return out


def reload_short(station: SlotState, full_station: SlotState, mag: SlotState, upcoming: Sequence[str]) -> bool:
    """True when the station as it is would bring fewer of the next stones than a refilled station - the operator
    tops it up before ARES drives there (with a station smaller than the magazine this is every time it is empty)."""
    have = _loadable(mag, upcoming, {k: station.count(k) for k in KINDS})
    full = _loadable(mag, upcoming, {k: full_station.count(k) for k in KINDS})
    return have < full


def _n0(v) -> int | float:
    """Stones in course 0 of a leg: an int, or x.5 for a leg ending with a half stone (robodk/wallplan.stones_n0)."""
    f = float(v)
    return int(round(f)) if abs(f - round(f)) < 1e-9 else f


# ── config provenance ─────────────────────────────────────────────────────────
def config_sha256(path: str | Path | None = None, variant: str | None = None) -> str:
    """sha256 of config/station.toml - and of the variant overlay after it, if the config is a variant."""
    data = Path(path or STATION_TOML).read_bytes()
    if variant:
        from .config import variant_path
        data += b"\n# variant " + variant.encode() + b"\n" + variant_path(variant).read_bytes()
    return hashlib.sha256(data).hexdigest()


_KEY = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)$")


def _tag(text: str) -> str | None:
    hits = [(text.find(t), t) for t in STATUS_TAGS if t in text]
    return min(hits)[1] if hits else None


def _split_comment(rest: str) -> str:
    """Inline comment of a TOML value (a '#' outside a string)."""
    q = None
    for i, ch in enumerate(rest):
        if ch in "\"'":
            q = None if q == ch else (ch if q is None else q)
        elif ch == "#" and q is None:
            return rest[i + 1:]
    return ""


def config_status(path: str | Path | None = None, variant: str | None = None) -> dict[str, dict]:
    """Status tag of every key in station.toml: {"[section] key": {"status", "source"}}; [[targets]] entries as
    "[[targets]] <name>.<key>". The tag comes from the key's own inline comment, else from the comment lines that
    continue it, else from the comment block of its section (or array of tables); "UNTAGGED" if none. variant: the tags
    of config/variants/<variant>.toml override those of its keys; an array of tables in the overlay (e.g.
    [[wall.legs]]) replaces all entries of that array, as in mauer.config.merge."""
    out = _status_of_file(Path(path or STATION_TOML))
    if variant:
        from .config import variant_path
        over = _status_of_file(variant_path(variant))
        arrays = {label.split(" ", 1)[0] for label in over if label.startswith("[[")}
        out = {k: v for k, v in out.items() if k.split(" ", 1)[0] not in arrays}
        out.update(over)
    return out


def _status_of_file(path: Path) -> dict[str, dict]:
    lines = path.read_text(encoding="utf-8").splitlines()
    out: dict[str, dict] = {}
    section, sec_tag, array_tags = "", None, {}
    pending: list[str] = []            # comment block before the next header
    in_header_block = False
    entry: dict[str, tuple[str | None, str]] = {}
    entry_name = None

    def flush_entry():
        if section.startswith("[[") and entry:
            name = entry_name or "?"
            for k, (tag, src) in entry.items():
                out[f"{section} {name}.{k}"] = {"status": tag or sec_tag or "UNTAGGED", "source": src}

    i = 0
    while i < len(lines):
        raw = lines[i]
        s = raw.strip()
        if not s:
            pending = []
            in_header_block = False
            i += 1
            continue
        if s.startswith("["):
            flush_entry()
            entry, entry_name = {}, None
            header = s.split("#")[0].strip()
            block = " ".join(pending)
            if header.startswith("[["):
                section = header
                if _tag(block):
                    array_tags[section] = _tag(block)
                sec_tag = array_tags.get(section)
            else:
                section = header
                sec_tag = _tag(block)
            pending = []
            in_header_block = True
            i += 1
            continue
        if s.startswith("#"):
            if in_header_block and not section.startswith("[["):
                sec_tag = sec_tag or _tag(s)          # comment right below a header belongs to the section
            pending.append(s)
            i += 1
            continue
        m = _KEY.match(s)
        in_header_block = False
        pending = []
        if not m:
            i += 1
            continue
        key, rest = m.group(1), m.group(2)
        comment = _split_comment(rest)
        j = i + 1
        cont = []
        while j < len(lines) and lines[j].startswith((" ", "\t")) and lines[j].strip().startswith("#"):
            cont.append(lines[j].strip().lstrip("#").strip())
            j += 1
        tag = _tag(comment) or _tag(" ".join(cont))
        src = " ".join([comment.strip()] + cont).strip()
        if section.startswith("[["):
            if key == "name":
                entry_name = rest.split("#")[0].strip().strip("\"'")
            entry[key] = (tag, src)
        else:
            out[f"{section} {key}"] = {"status": tag or sec_tag or "UNTAGGED", "source": src}
        i = j
    flush_entry()
    return out


def _cfg_value(cfg: Mapping, label: str):
    sec, key = label.split(" ", 1)
    if sec.startswith("[["):
        name, k = key.split(".", 1)
        node: Any = cfg
        for part in sec.strip("[]").split("."):          # [[wall.legs]] -> cfg["wall"]["legs"]
            node = node.get(part, {}) if isinstance(node, Mapping) else {}
        for t in node if isinstance(node, list) else []:
            if str(t.get("name")) == name:
                return t.get(k)
        return None
    node: Any = cfg
    for part in sec.strip("[]").split("."):
        node = node.get(part, {}) if isinstance(node, Mapping) else {}
    return node.get(key) if isinstance(node, Mapping) else None


def _label_match(label: str, pattern: str) -> bool:
    """'[section] key' against '[section] key-pattern': the section must be equal (brackets are literal), the key is
    an fnmatch pattern."""
    lsec, lkey = label.split(" ", 1)
    psec, pkey = pattern.split(" ", 1)
    return lsec == psec and fnmatch.fnmatchcase(lkey, pkey)


def depends_on(cfg: Mapping, patterns: Sequence[str], path: str | Path | None = None,
               statuses: Sequence[str] = ("PLACEHOLDER", "ASSUMPTION", "UNKNOWN", "UNTAGGED")) -> list[dict]:
    """Config keys matching the fnmatch `patterns` (e.g. "[ur5] *", "[[targets]] *") whose status is one of
    `statuses`, with their current value - the provenance list a result or job must carry (CLAUDE.md). The tags of
    a variant config (cfg["_variant"]) include its overlay file."""
    st = config_status(path, cfg.get("_variant"))
    out = []
    for label, info in st.items():
        if info["status"] in statuses and any(_label_match(label, p) for p in patterns):
            out.append({"key": label, "status": info["status"], "value": _jsonable(_cfg_value(cfg, label))})
    return out
