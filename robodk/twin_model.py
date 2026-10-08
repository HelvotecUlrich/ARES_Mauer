"""RoboDK digital twin of a Mauer run - the pure part (numpy and mauer only, no RoboDK import; docs/HMI_DESIGN.md
section 11.3). robodk/twin.py draws it, hmi/core/twin_link.py feeds it from the HMI.

The twin is driven by STATE, not by events: every tick it gets a TwinFrame (UR joints, ARES pose in the wall frame,
station frame, StoneState) and plan_ops() turns the difference between the stone state on screen and the new one into
moves / removes / adds of stone objects. The twin polls at ~10 Hz while a SIM run can do several steps in between
(sim_step_s = 0: a whole stop in under 2 s), so a diff may skip states (a stone taken from the magazine and already
in the wall); the pairing rules below still move the right objects.

Stone locations (Loc): ("mag", magazine slot id), ("station", station slot id), ("wall", stone key), ("tool", None).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Mapping

import numpy as np

from mauer.job import SlotState
from mauer.reference import Pose2D

Loc = tuple[str, Any]                     # ("mag", sid) | ("station", sid) | ("wall", key) | ("tool", None)
TOOL: Loc = ("tool", None)

# removed / added locations are paired (one object moved) in this order: the real transfers first (pick from the
# magazine or the station, place in the wall or the magazine), then the transfers of states skipped between two
# ticks (magazine -> wall: picked and placed; station -> magazine: a whole reload transfer)
PAIR_ORDER = (("mag", "tool"), ("station", "tool"), ("tool", "wall"), ("tool", "mag"), ("mag", "wall"),
              ("station", "mag"))
_LOC_RANK = {"mag": 0, "station": 1, "tool": 2, "wall": 3}

MIN_PORT = 20630                          # same rule as hmi/core/config.py TWIN_MIN_PORT
USER_PORTS = (20500, 20501)               # the user's RoboDK (API) - never used by the twin


def held_tuple(held: Mapping | None) -> tuple[str, str | None, str] | None:
    """Sequencer.held dict -> (from, slot, kind); from = "unknown" when the jaw state is unknown (a failed pick or
    place: the stone may or may not be in the jaws)."""
    if not held:
        return None
    src = "unknown" if held.get("unknown") else str(held.get("from") or "unknown")
    slot = held.get("slot")
    return src, None if slot is None else str(slot), str(held.get("kind") or "full")


def _slots(s: SlotState) -> dict[str, str]:
    return {sid: s.kinds.get(sid, "full") for sid in s.filled}


@dataclass(frozen=True)
class StoneState:
    """Where the stones are, as the sequencer believes (RunSnapshot): filled magazine / station slots with their
    stone type, placed stone keys, the stone in the jaws."""
    magazine: Mapping[str, str]                       # slot id -> kind
    station: Mapping[str, str]
    placed: frozenset                                 # stone keys
    held: tuple[str, str | None, str] | None          # (from "magazine" | "station" | "unknown", slot, kind)

    @classmethod
    def initial(cls, job) -> "StoneState":
        """Before a run: magazine as planned (initial_fill / initial_kinds), full station, nothing placed."""
        return cls(_slots(SlotState.magazine(job.magazine)), _slots(SlotState.station(job.station)), frozenset(),
                   None)

    @classmethod
    def from_snapshot(cls, snap) -> "StoneState":
        """From an hmi.core.snapshot.RunSnapshot (or anything with magazine / station / placed / held)."""
        return cls(dict(snap.magazine), dict(snap.station), frozenset(tuple(k) for k in snap.placed),
                   held_tuple(snap.held))

    def locations(self, wall_kinds: Mapping[tuple, str] | None = None) -> dict[Loc, tuple[str, bool]]:
        """Every occupied location -> (stone kind, failed); failed = a held stone whose jaw state is unknown."""
        wk = wall_kinds or {}
        out: dict[Loc, tuple[str, bool]] = {("mag", s): (k, False) for s, k in self.magazine.items()}
        out.update({("station", s): (k, False) for s, k in self.station.items()})
        out.update({("wall", tuple(key)): (wk.get(tuple(key), "full"), False) for key in self.placed})
        if self.held is not None:
            out[TOOL] = (self.held[2], self.held[0] == "unknown")
        return out

    def counts(self) -> dict[str, int]:
        """{"mag", "station", "wall", "tool"}: number of stones per place (as TwinScene.stone_count)."""
        return {"mag": len(self.magazine), "station": len(self.station), "wall": len(self.placed),
                "tool": int(self.held is not None)}


@dataclass(frozen=True)
class Op:
    kind: str                     # "move" | "add" | "remove"
    src: Loc | None               # move / remove
    dst: Loc | None               # move / add
    stone_kind: str               # "full" | "half"
    failed: bool = False          # the stone at dst (add / move to the tool) is a held stone of unknown state


def _order(loc: Loc) -> tuple:
    return _LOC_RANK.get(loc[0], 9), str(loc[1])


def plan_ops(old: StoneState, new: StoneState, wall_kinds: Mapping[tuple, str] | None = None) -> list[Op]:
    """Object operations that turn the stones of `old` into those of `new`: locations that changed are paired by
    stone kind in PAIR_ORDER (one object moved), the rest is removed / added. Moves first, then removes, then adds;
    a location whose kind or failed flag changed counts as removed and added (the object is replaced).
    wall_kinds: stone key -> kind of the wall stones (job), "full" if missing."""
    o, n = old.locations(wall_kinds), new.locations(wall_kinds)
    removed = {loc: v for loc, v in o.items() if n.get(loc) != v}
    added = {loc: v for loc, v in n.items() if o.get(loc) != v}
    ops: list[Op] = []
    for src_t, dst_t in PAIR_ORDER:
        for src in sorted((l for l in removed if l[0] == src_t), key=_order):
            kind = removed[src][0]
            dst = next((l for l in sorted(added, key=_order) if l[0] == dst_t and added[l][0] == kind), None)
            if dst is None:
                continue
            ops.append(Op("move", src, dst, kind, added[dst][1]))
            del removed[src], added[dst]
    ops += [Op("remove", loc, None, v[0], v[1]) for loc, v in sorted(removed.items(), key=lambda x: _order(x[0]))]
    ops += [Op("add", None, loc, v[0], v[1]) for loc, v in sorted(added.items(), key=lambda x: _order(x[0]))]
    return ops


def apply_ops(state: Mapping[Loc, Any], ops: list[Op], make: Callable[[Op], Any] = lambda op: op,
              place: Callable[[Any, Op], None] = lambda obj, op: None,
              drop: Callable[[Any], None] = lambda obj: None) -> dict[Loc, Any]:
    """`state` (location -> object) after `ops` (TwinScene.apply uses it with RoboDK items). Two phases: every
    source is detached first, so the order of the moves does not matter (a held stone placed and the next one picked
    between two ticks). make(op) creates the object of an add (or of a move whose source is missing), place(obj, op)
    puts a moved object at op.dst, drop(obj) deletes a removed object or one in the way of a destination."""
    out = dict(state)
    moving = []
    for op in ops:
        if op.kind == "move":
            moving.append((op, out.pop(op.src) if op.src in out else None))
        elif op.kind == "remove" and op.src in out:
            drop(out.pop(op.src))
    for op, obj in moving:
        if op.dst in out:                       # occupied (a scene out of step with its state): replace
            drop(out.pop(op.dst))
        if obj is None:
            out[op.dst] = make(op)
        else:
            place(obj, op)
            out[op.dst] = obj
    for op in ops:
        if op.kind == "add":
            if op.dst in out:
                drop(out.pop(op.dst))
            out[op.dst] = make(op)
    return out


def wall_kinds(job) -> dict[tuple, str]:
    """Stone key -> kind for every stone of the job."""
    return {tuple(t.key): t.kind for t in job.stones()}


# ── frames and settings ───────────────────────────────────────────────────────
@dataclass(frozen=True)
class TwinFrame:
    """What the twin shows at one tick (immutable; built by hmi/core/twin_link.frame_from)."""
    q_rad: tuple[float, ...] | None             # UR joints
    ares: Pose2D | None                         # ARES base_link in the wall frame
    ares_label: str                             # where the pose comes from ("sim truth", "estimate", ...)
    T_wall_station: np.ndarray | None           # station frame in the wall frame (z of the nominal is kept)
    stones: StoneState | None
    caption: str = ""                           # one line for RoboDK's status bar
    t: float = 0.0                              # time.time() of the data (lag = render time - t)


@dataclass(frozen=True)
class TwinSettings:
    port: int = MIN_PORT
    port_tries: int = 10
    rate_hz: float = 10.0
    socket_timeout_s: float = 5.0
    build_timeout_s: float = 120.0
    visible: bool = True
    ghost_wall: bool = True

    def __post_init__(self) -> None:
        if self.port < MIN_PORT:
            raise ValueError(f"twin API port {self.port} < {MIN_PORT}: the twin needs its own RoboDK port")
        if self.port_tries < 1:
            raise ValueError("port_tries must be >= 1")
        bad = [p for p in USER_PORTS if self.port <= p < self.port + self.port_tries]
        if bad:
            raise ValueError(f"twin ports {self.port}..{self.port + self.port_tries - 1} include the user's RoboDK "
                             f"{bad}")
        if not self.rate_hz > 0:
            raise ValueError("rate_hz must be > 0")

    @property
    def ports(self) -> range:
        return range(self.port, self.port + self.port_tries)

    @classmethod
    def from_config(cls, cfg: Mapping) -> "TwinSettings":
        """[hmi.twin] of the station config; ValueError for a port < 20630 or a range with 20500 / 20501."""
        tw = dict((cfg.get("hmi", {}) or {}).get("twin", {}) or {})
        d = cls()
        return cls(port=int(tw.get("port", d.port)), port_tries=int(tw.get("port_tries", d.port_tries)),
                   rate_hz=float(tw.get("rate_hz", d.rate_hz)),
                   socket_timeout_s=float(tw.get("socket_timeout_s", d.socket_timeout_s)),
                   build_timeout_s=float(tw.get("build_timeout_s", d.build_timeout_s)),
                   visible=bool(tw.get("visible", d.visible)), ghost_wall=bool(tw.get("ghost_wall", d.ghost_wall)))


def scene_bounds(job) -> tuple[float, float, float, float]:
    """(x_min, y_min, x_max, y_max) [mm] in the wall frame of the stones, the stops and the dock."""
    pts = [np.asarray(t.T_wall_tcp, float)[:2, 3] for t in job.stones()]
    pts += [np.array([s.ares.x_mm, s.ares.y_mm]) for s in job.stops]
    try:
        d = job.station.dock_in_wall
        pts.append(np.array([d.x_mm, d.y_mm]))
    except (AttributeError, ValueError, TypeError):
        pass
    a = np.array(pts, float) if pts else np.zeros((1, 2))
    return float(a[:, 0].min()), float(a[:, 1].min()), float(a[:, 0].max()), float(a[:, 1].max())


def default_view(job, margin_mm: float = 1000.0) -> tuple[list[float], list[float]]:
    """(eye, target) [mm, wall frame] of the twin's start view: the whole scene from the -y side, looking down at
    about 45 deg."""
    x0, y0, x1, y1 = scene_bounds(job)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    span = max(x1 - x0, y1 - y0, 0.0) + 2.0 * margin_mm
    d = 1.15 * span                                  # RoboDK's view angle: the whole span in the picture
    return [cx, cy - d * math.cos(math.radians(50.0)), d * math.sin(math.radians(50.0))], [cx, cy, 300.0]
