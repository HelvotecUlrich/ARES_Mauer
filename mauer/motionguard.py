"""Motion guard for the UR5 (2026-10-07): every joint path the robot backend is about to command is checked with the
capsule model of mauer.armcheck before it is sent - the tool (and a held stone) against the arm itself, the arm,
tool and held stone against ARES with the magazine stones, the wall built so far and, docked, the pick-up station
with its table and stones. A blocked direct move gets a detour; no clear path -> the move is refused before anything
moves (the backend raises RobotError, the sequencer stops with the arm where it is, ARES interlocked).

Why: the RoboDK simulation (robodk/simulate.py) plans every transfer with collision checks, but the robot backend
(mauer.backends.URRobot) sends a joint move straight from the end of one action to the approach pose of the next -
only wall places carry the job's via points, and the real park pose ([ur] park_q_deg) is not the one the simulation
parks at. The camera adapter touched wrist 1 on such a straight move on 2026-10-06. The model is coarse (capsules
and boxes with ASSUMED radii, mauer.armcheck): RoboDK stays the reference check of the plan, this is the net under
the real motions.

What is checked (joint space, linear interpolation like movej, every STEP_DEG of the largest joint change, both ends
included): the tool >= armcheck.SELF_CLEARANCE_MM from the arm's own links (self_clearance of armcheck, plus the
held stone); every capsule >= armcheck.CLEARANCE_MM from the boxes of the world (GuardWorld). Not checked: the
vertical approach / contact moves of the pick and place scripts (contact by design, as in robodk/motion.py) - for a
wall stone set from the side (StoneTask.side_mm, 2026-10-09) the descent column is the side point, the sideways move
at config.side_lift_mm is a contact move as well -, and the wall stones of OTHER legs than the job's (all legs of the
job are included).

Detours (plan), the clear one with the least joint travel wins (travel_deg, Samuel 2026-10-09 "keine unnoetigen
Wege"; the job's own vias are tried first): direct; via the park pose; straight above the target (transfer height, else
OVER_TARGET_MM higher), alone or after the park pose; lift the TCP vertically to the transfer height at the start;
lift at the start and above the target; lift - park - lift; compact poses near the base towards the target (as
robodk/motion.Planner.compact), with and without a via above the target; up from the start by OVER_TARGET_MM; then
the high arc (2026-10-16: the move between the two station looks, low between stacks up to 5 layers high, found no
fixed detour and waited 9..15 s for the RRT - a lift by joint interpolation bows into the stacks): straight up from
the start and down onto the target in ARC_STEP_MM steps over the higher end by ARC_RISE_MM (lowest free one),
shortcut; then a seeded joint-space RRT-Connect (RRT_*) with shortcut. park_last (magazine dry run): the same list with every detour
through the park pose moved behind the others. The transfer height carries a stone with its
pins LIFT_MARGIN_MM over a full magazine (as robodk/simulate.py safe_z(full)).

Look moves (plan look=True, 2026-10-18, Samuel: "zu den Codes und weg von den Codes ... am besten nach oben wegfahren,
sinnvolle und effiziente Bewegungen"; run 2026-10-16_182116: 577 of 847 s were look and park moves, the move between
two looks swung low over the magazine when it was free and otherwise took a stop-and-go detour of up to 6 vias): park
-> look, look -> look and look -> park leave a look straight up (ARC_STEP_MM steps) to LOOK_RISE_MM over the higher
look, swing over (a waypoint every LOOK_ARC_STEP_DEG of base turn, the TCP height interpolated, pulled LOOK_PULL_MM
towards the UR base axis mid-swing to keep clear of a high wall or station stacks) and come straight down onto the
next look; the free candidate with the least joint travel wins (the job's vias first, a re-aimed retry of the same look
directly). Its vias are blended (URScript movej r = blend_radii, BLEND_MM): the arm stops only at the looks (image) and
at the park pose. The blended corners are checked too (blend_curves). Without a free candidate: the high arc, then the
detours above - blended where their corners are free, else stopping at every via as before.

Units mm, rad. Frames: docs/ARCHITECTURE.md (UR base frame for joints and poses).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np

from . import armcheck
from . import geometry as g
from .config import grasp_above_top_mm
from .simworld import ik_near, ur5_fk

STEP_DEG = 2.0                    # sampling of a joint move (robodk/motion.py SELF_STEP_DEG)
LIFT_MARGIN_MM = 60.0             # held stone (incl. pins) over a full magazine at the transfer height
OVER_TARGET_MM = (300.0, 200.0, 100.0)    # vias straight above the target when the transfer height is out of reach
COMPACT_R_MM = (300.0, 250.0, 350.0, 200.0)   # compact transfer poses: TCP this far from the UR base axis
COLUMN_TOL_MM = 5.0               # TCP this close (horizontally) to the target TCP: gripper and stone are in their
                                  # descent column, where butt joints and the 5 mm magazine gaps put them next to stones
GRIPPER_PARTS = ("gripper", "jaw bar", "jaw +x", "jaw -x", "held stone")   # may touch (not overlap) there
COLUMN_ABOVE_MM = 20.0            # ... up to the approach height + this,
COLUMN_TILT_DEG = 3.0             # ... with the tool axis this close to the target's
FLOOR_SLAB_MM, FLOOR_HALF_MM = 50.0, 4000.0   # the floor (ARES frame z <= 0) as a box in the world
ARC_STEP_MM = 100.0               # high arc: vertical steps of the lift chains (one joint move each, then shortcut)
ARC_RISE_MM = tuple(float(d) for d in range(200, 901, 100))   # ... TCP this far over the higher end, lowest first
MEMO_TOL_DEG = 10.0               # a high arc / RRT path found before is tried again (after the fixed detours) when
MEMO_N = 16                       # both ends are this close per joint (the same look pair on the next station trip)
RRT_ITER = 1000                   # RRT-Connect fallback (robodk/motion.Planner.rrt): iterations,
RRT_STEP_DEG = 15.0               # largest joint step of an extension,
RRT_MARGIN_DEG = (120.0, 60.0, 60.0, 120.0, 60.0, 120.0)   # sampling box around start / goal per joint
LOOK_RISE_MM = (150.0, 250.0, 350.0, 450.0, 550.0)   # look moves: TCP this far over the higher look in the swing
LOOK_PULL_MM = (0.0, 150.0, 300.0)            # ... pulled this far towards the UR base axis mid-swing
LOOK_SLANT = (0.0, 0.5, 1.0)      # ... leaving / reaching a look: straight up, or also this many mm towards the UR base
                                  # axis per mm of rise (27 / 45 deg: a code at the foot of a high wall or stack)
LOOK_ARC_STEP_DEG = 30.0          # ... one swing waypoint per this much base-joint turn
LOOK_R_MIN_MM = 250.0             # ... the TCP of a swing waypoint at least this far from the UR base axis
LOOK_DIRECT_MM = 150.0            # look -> look with the TCPs closer than this (a re-aimed retry): direct move first
BLEND_MM = 100.0                  # ASSUMPTION: blend radius of the vias of a look move (URScript movej r) ...
BLEND_FRAC = 0.45                 # ... at most this fraction of the TCP distance to either neighbour: two blends never
                                  # overlap (the UR skips a waypoint whose blend overlaps the next one)
BLEND_SAMPLES = 40                # search of the point where the TCP enters a blend sphere along a joint move
PARK_TOL_RAD = 0.01               # joints this close to the park pose are at the park pose (backends.URRobot park_tol_rad)


@dataclass
class GuardWorld:
    """What the arm must not touch at the moment of a move (set by the sequencer before every robot action):
    magazine slots that hold a stone (mauer.job.MagazineSlot), the stones placed so far (mauer.job.StoneTask) with
    the job's legs ({"name", "n0", "T_wall_leg"}) and the current estimate T_base_wall, and - docked - the station
    slots that hold a stone with T_base_station."""
    magazine: Sequence = ()
    wall_stones: Sequence = ()
    legs: Sequence[Mapping] = ()
    T_base_wall: np.ndarray | None = None
    station: Sequence = ()
    T_base_station: np.ndarray | None = None


@dataclass(frozen=True)
class _Leg:
    name: str
    x: float
    y: float
    theta: float

    def to_wall(self, u: float, v: float) -> tuple[float, float]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return self.x + c * u - s * v, self.y + s * u + c * v


@dataclass(frozen=True)
class _WallStone:                 # the attributes armcheck.stone_boxes reads, from a StoneTask
    u: float
    z_top: float
    length: float
    leg: str
    label: str


@dataclass
class Verdict:
    ok: bool
    vias: list = field(default_factory=list)           # joint vias before the target (rad)
    problems: list = field(default_factory=list)       # why the direct move (and every detour) was refused
    blend_mm: list = field(default_factory=list)       # look moves: blend radius per via (URScript movej r), else []


def _coarse_to_fine(n: int) -> list[int]:
    """0..n, ends first, then the middles of the halves, quarters, ... (every index once)."""
    out, spans, i = [0, n] if n else [0], [(0, n)], 0
    while i < len(spans):
        a, b = spans[i]
        i += 1
        if b - a >= 2:
            m = (a + b) // 2
            out.append(m)
            spans += [(a, m), (m, b)]
    return out


def travel_deg(qs: Sequence[Sequence[float]]) -> float:
    """Joint travel of a joint path [deg]: the largest joint angle of every segment, summed - a movej takes about
    that long (the joints move together, the largest one sets the time)."""
    return float(sum(np.degrees(np.max(np.abs(np.asarray(b, float) - np.asarray(a, float))))
                     for a, b in zip(qs, qs[1:])))


def blend_radii(qs: Sequence[Sequence[float]], T_flange_tcp: np.ndarray) -> list[float]:
    """Blend radius [mm] of every via qs[1:-1] of a look move: BLEND_MM, at most BLEND_FRAC of the TCP distance to
    either neighbour (the start and the target are exact stops, radius 0)."""
    P = [(ur5_fk(np.asarray(q, float)) @ np.asarray(T_flange_tcp, float))[:3, 3] for q in qs]
    return [min(BLEND_MM, BLEND_FRAC * float(np.linalg.norm(P[i] - P[i - 1])),
                BLEND_FRAC * float(np.linalg.norm(P[i + 1] - P[i]))) for i in range(1, len(qs) - 1)]


def blend_curves(qs: Sequence[Sequence[float]], radii: Sequence[float], T_flange_tcp: np.ndarray
                 ) -> list[list[np.ndarray]]:
    """The blended corners of the joint path qs (radius radii[i - 1] at via qs[i]): per via with a radius >= 1 mm the
    quadratic Bezier from where the TCP enters the blend sphere on the incoming move to where it leaves it on the
    outgoing one, the via as control point, sampled at quarters (5 joint vectors) - the UR's blend lies in that
    corner. Checked like moves (MotionGuard._blend_problems, robodk/simulate.py)."""
    T_ft = np.asarray(T_flange_tcp, float)

    def tcp(q):
        return (ur5_fk(q) @ T_ft)[:3, 3]

    def enter(q_far, q_via, r):
        p = tcp(q_via)
        for i in range(BLEND_SAMPLES + 1):
            q = q_far + (q_via - q_far) * (i / BLEND_SAMPLES)
            if float(np.linalg.norm(tcp(q) - p)) <= r:
                return q
        return q_via
    qs = [np.asarray(q, float) for q in qs]
    out = []
    for i, r in enumerate(radii, 1):
        if r >= 1.0:
            qa, qb = enter(qs[i - 1], qs[i], r), enter(qs[i + 1], qs[i], r)
            out.append([(1 - t) ** 2 * qa + 2 * t * (1 - t) * qs[i] + t * t * qb for t in (0.0, 0.25, 0.5, 0.75, 1.0)])
    return out


def move_time_s(qs: Sequence[Sequence[float]], v: float, a: float, blended: bool = False) -> float:
    """Rough duration [s] of the joint path qs at joint speed v [rad/s] and acceleration a [rad/s^2]: every movej one
    trapezoid of its largest joint change (it stops at every via), blended one trapezoid over their sum (a lower bound:
    a sharp blend slows down)."""
    def trapezoid(d: float) -> float:
        return d / v + v / a if d > v * v / a else 2.0 * math.sqrt(max(d, 0.0) / a)
    ds = [float(np.max(np.abs(np.asarray(b, float) - np.asarray(q, float)))) for q, b in zip(qs, qs[1:])]
    return trapezoid(sum(ds)) if blended else float(sum(trapezoid(d) for d in ds))


class MotionGuard:
    """Capsule-model check and detour planning for UR5 joint moves (module docstring). `T_ares_base` = UR base in the
    ARES frame (job.T_ares_base), `park_q` = the park joints (job.park_q_rad)."""

    def __init__(self, cfg: Mapping, T_ares_base: np.ndarray, park_q: Sequence[float], T_flange_tcp: np.ndarray,
                 step_deg: float = STEP_DEG, approach_mm: float = 150.0):
        self.cfg = cfg
        self.T_ares_base = np.asarray(T_ares_base, float)
        self.park_q = np.asarray(park_q, float)
        self.T_flange_tcp = np.asarray(T_flange_tcp, float)
        self.step = math.radians(step_deg)
        self.world = GuardWorld()
        self._checkers: list | None = None
        self._column: np.ndarray | None = None     # target TCP pose (base frame) of the plan in progress
        self.approach_mm = float(approach_mm)
        self._memo: list[tuple[np.ndarray, np.ndarray, list[np.ndarray]]] = []   # (q_from, q_to, vias) of high arcs /
        # RRT paths found (MEMO_N latest): tried again after the fixed detours before searching anew
        self.park_last = False                     # True: detours through the park pose only after every other one
        # of the fixed list (mauer.magtest, Samuel 2026-10-09: the park pose between the front and the magazine "is not
        # really necessary") - lift / over-target / compact first, RRT still last
        self.stone_clear_mm = 0.0                  # > CLEARANCE_MM: the held stone keeps this from every stone of the
        # world outside the descent column (mauer.magtest, Samuel 2026-10-09 "enough separation to the stones next to it")
        b, dk, a = cfg["brick"], cfg["deck"], cfg["ares"]
        layers = int(dk.get("magazine_layers", dk.get("layers", 1)))
        top_ares = (float(a["deck_top_z"]) + float(dk["holder_z"])
                    + layers * float(b["height"]) + (layers - 1) * float(b.get("bed_joint", 0.0)))
        hb = cfg.get("half_brick") or {}
        hang = max(float(b["height"]),                       # a half stone hangs grasp_above_top_mm lower (2026-10-09)
                   float(hb.get("height", b["height"])) + grasp_above_top_mm(dict(cfg), "half"))
        held = hang + float(b.get("pin_length", 0.0))
        self.z_safe_base = top_ares + held + LIFT_MARGIN_MM - float(self.T_ares_base[2, 3])   # TCP z, base frame

    # ── world ────────────────────────────────────────────────────────────────
    def set_world(self, world: GuardWorld) -> None:
        self.world = world
        self._checkers = None

    def _held_capsules(self, kind: str) -> list:
        """The held stone in the flange frame: TCP z into the stone, the top face config.grasp_above_top_mm below the
        TCP (0 for a full stone, a half stone is held 20 mm higher - 2026-10-09), length along TCP x, width incl.
        the ribs of the long faces ([brick] rib_mm). A 3 x 3 grid of capsules along the length (radius r = min(width,
        height) / 6, ends within the stone: the faces between the grid lines at most (sqrt 2 - 1) r = 8.3 mm inside
        the box, < armcheck.CLEARANCE_MM) plus the 12 box edges as zero-radius segments (the grid alone missed the
        corners by 14.6 mm, review 2026-10-07). (One fat capsule overshot the end faces of a half stone by 10 mm and
        refused picks next to a stack 5 mm away.)"""
        dims = self.cfg["half_brick"] if kind == "half" else self.cfg["brick"]
        rib = float(self.cfg["brick"].get("rib_mm", 0.0))
        L, W, H = float(dims["length"]), float(dims["width"]) + 2.0 * rib, float(dims["height"])
        r = min(W, H) / 6.0
        half = max(L / 2.0 - r, 0.0)
        T = self.T_flange_tcp
        o = grasp_above_top_mm(dict(self.cfg), kind)
        out = []
        for v in (-W / 2.0 + r, 0.0, W / 2.0 - r):
            for z in (o + r, o + H / 2.0, o + H - r):
                out.append(("held stone", g.apply(T, [[-half, v, z]])[0], g.apply(T, [[half, v, z]])[0], r))
        x, y = L / 2.0, W / 2.0
        edges = ([((-x, sy, z), (x, sy, z)) for sy in (-y, y) for z in (o, o + H)]
                 + [((sx, -y, z), (sx, y, z)) for sx in (-x, x) for z in (o, o + H)]
                 + [((sx, sy, o), (sx, sy, o + H)) for sx in (-x, x) for sy in (-y, y)])
        for a, b in edges:
            out.append(("held stone", g.apply(T, [a])[0], g.apply(T, [b])[0], 0.0))
        return out

    def _boxes(self):
        if self._checkers is not None:
            return self._checkers
        w, cfg = self.world, self.cfg
        floor = armcheck.Box("floor", np.array([0.0, 0.0, -FLOOR_SLAB_MM / 2.0]), np.eye(3),
                             np.array([FLOOR_HALF_MM, FLOOR_HALF_MM, FLOOR_SLAB_MM / 2.0]))   # ARES frame z = 0 = floor
        out = [("ares", armcheck.ares_boxes(cfg, list(w.magazine)) + [floor], None)]
        if w.T_base_wall is not None and w.wall_stones:
            legs = {}
            for lg in w.legs or ():
                T = np.asarray(lg["T_wall_leg"], float)
                legs[str(lg["name"])] = _Leg(str(lg["name"]), float(T[0, 3]), float(T[1, 3]),
                                             math.atan2(T[1, 0], T[0, 0]))
            L = float(cfg["brick"]["length"])
            stones = [_WallStone(float(t.u_mm), float(t.z_top_mm), float(t.length_mm or L), t.leg or "", t.label)
                      for t in w.wall_stones]
            out.append(("wall", armcheck.stone_boxes(cfg, stones, legs), g.inv(np.asarray(w.T_base_wall, float))))
        if w.T_base_station is not None:
            from .floor import station_table_extent
            out.append(("station", armcheck.station_boxes(cfg, list(w.station), station_table_extent(cfg)),
                        g.inv(np.asarray(w.T_base_station, float))))
        self._checkers = out
        return out

    # ── checks ───────────────────────────────────────────────────────────────
    def state_problems(self, q: Sequence[float], holding: str | None = None,
                       tolerate: Mapping[tuple[str, str], float] | None = None) -> list[str]:
        """Collisions of the arm (+ tool, + held stone of kind `holding`) at joints q; [] = free. tolerate = the
        contacts at the start of a move {(part, other): depth mm} (_contacts): those may stay as long as they do not
        get deeper (by > 1 mm) - the arm is already there and moves away (e.g. a stone just lifted 5 mm beside the
        next magazine stack)."""
        out = []
        for (part, other), depth in self._contacts(q, holding).items():
            if tolerate is not None and depth <= tolerate.get((part, other), -math.inf) + 1.0:
                continue
            out.append(f"{part} vs {other} {depth:.0f} mm too close")
        return out

    def _contacts(self, q: Sequence[float], holding: str | None) -> dict[tuple[str, str], float]:
        """{(part, other): depth mm} of every pair closer than its limit: tool / held stone vs an arm link
        (SELF_CLEARANCE_MM) and arm / tool / held stone vs a box of the world (CLEARANCE_MM)."""
        tool = armcheck.tool_capsules(self.cfg) + (self._held_capsules(holding) if holding else [])
        out: dict[tuple[str, str], float] = {}
        for (part, link), d in self._self_pairs(q, tool).items():       # every pair (review: a tolerated start
            if d < armcheck.SELF_CLEARANCE_MM:                          # pair hid the others)
                out[(part, link)] = armcheck.SELF_CLEARANCE_MM - d
        for what, boxes, T_parent_base in self._boxes():
            if not boxes:
                continue
            if T_parent_base is None:              # ARES frame: the arm in the UR base pose on ARES
                caps = [c for c in armcheck.arm_capsules(q, self.T_ares_base, tool)
                        if c.name not in ("base", "shoulder")]
            else:
                caps = armcheck.arm_capsules(q, T_parent_base, tool)
            in_column = self._in_column(q)
            for cap, box, depth in armcheck.collisions(caps, boxes, armcheck.CLEARANCE_MM):
                if in_column and cap in GRIPPER_PARTS and depth <= armcheck.CLEARANCE_MM:
                    continue                       # gripper / stone over the slot, beside its neighbours: touching ok
                out[(cap, f"{box} ({what})")] = max(out.get((cap, f"{box} ({what})"), 0.0), depth)
            if holding and self.stone_clear_mm > armcheck.CLEARANCE_MM and not in_column:
                held = [c for c in caps if c.name == "held stone"]
                stones = [b for b in boxes if "stone" in b.name]
                for cap, box, depth in armcheck.collisions(held, stones, self.stone_clear_mm):
                    out[(cap, f"{box} ({what})")] = max(out.get((cap, f"{box} ({what})"), 0.0), depth)
        return out

    def _self_pairs(self, q, tool) -> dict[tuple[str, str], float]:
        """{(tool part, arm link): smallest distance mm} for every pair (armcheck.self_clearance pair by pair)."""
        caps = armcheck.arm_capsules(q, np.eye(4), tool)
        arm = [c for c in caps if c.name in armcheck.ARM_LINKS]
        out: dict[tuple[str, str], float] = {}
        for t in caps[len(arm):]:
            n = max(2, int(math.ceil(float(np.linalg.norm(t.b - t.a)) / 5.0)) + 1)
            P = t.a + np.linspace(0.0, 1.0, n)[:, None] * (t.b - t.a)
            for a in arm:
                if (t.name, a.name) in armcheck.SELF_SKIP or (t.name == "held stone" and a.name == "wrist 3"):
                    continue
                d = float(armcheck._point_segment_distance(P, a.a, a.b).min()) - t.r - a.r
                out[(t.name, a.name)] = min(out.get((t.name, a.name), math.inf), d)
        return out

    def _in_column(self, q) -> bool:
        """The TCP is in the descent column of the plan's target (plan column): within COLUMN_TOL_MM horizontally,
        between the target and approach_mm + COLUMN_ABOVE_MM above it, tool axis within COLUMN_TILT_DEG of the
        target's (review 2026-10-07: the exemption had no height / orientation limit)."""
        if self._column is None:
            return False
        T = ur5_fk(np.asarray(q, float)) @ self.T_flange_tcp
        Tc = self._column
        if float(np.hypot(*(T[:2, 3] - Tc[:2, 3]))) > COLUMN_TOL_MM:
            return False
        dz = float(T[2, 3] - Tc[2, 3])
        if not -COLUMN_TOL_MM <= dz <= self.approach_mm + COLUMN_ABOVE_MM:
            return False
        c = float(np.clip(np.dot(T[:3, 2], Tc[:3, 2]), -1.0, 1.0))
        return math.degrees(math.acos(c)) <= COLUMN_TILT_DEG

    def segment_problems(self, q0, q1, holding: str | None = None, from_start: bool = False) -> list[str]:
        """Problems of the joint move q0 -> q1 (the first failing sample found), [] = free. The samples (every
        STEP_DEG) are checked coarse to fine - ends, middle, quarters, ... (2026-10-16): a long blocked move is found
        after a few samples (the shortcut and the detours try many). from_start: q0 is where the arm stands now - its
        contacts may stay, not deepen (state_problems tolerate)."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        tol = self._contacts(q0, holding) if from_start else None
        n = max(1, int(math.ceil(float(np.max(np.abs(q1 - q0))) / self.step)))
        for i in _coarse_to_fine(n):
            q = q0 + (q1 - q0) * (i / n)
            p = self.state_problems(q, holding, tol)
            if p:
                return [f"at {i}/{n} of the move: " + "; ".join(p)]
        return []

    def path_problems(self, qs: Sequence[Sequence[float]], holding: str | None = None,
                      from_start: bool = True) -> list[str]:
        """Problems of the joint path qs[0] -> qs[1] -> ...; from_start: qs[0] is the current pose (first segment
        tolerates its start contacts, every later point must be free)."""
        for i, (a, b) in enumerate(zip(qs, qs[1:])):
            p = self.segment_problems(a, b, holding, from_start=from_start and i == 0)
            if p:
                return p
        return []

    # ── detours ──────────────────────────────────────────────────────────────
    def lift(self, q: Sequence[float], dz: float | None = None) -> np.ndarray | None:
        """Joints with the TCP lifted vertically (base z) by dz mm - default up to the transfer height - same
        orientation (None: no IK); q itself if there is nothing to lift."""
        q = np.asarray(q, float)
        T = ur5_fk(q) @ self.T_flange_tcp
        dz = self.z_safe_base - T[2, 3] if dz is None else float(dz)
        if dz <= 0.0:
            return q
        return ik_near(g.transl(0.0, 0.0, dz) @ T @ g.inv(self.T_flange_tcp), q)

    def compact(self, toward: Sequence[float], yaw_of: Sequence[float]) -> list[np.ndarray]:
        """Compact transfer poses near the UR base (as robodk/motion.Planner.compact): TCP at the transfer height
        COMPACT_R_MM from the base in the direction of the TCP of `toward`, tool down with the TCP yaw of `yaw_of`,
        IK near `toward`; every radius that has a solution."""
        T_to = ur5_fk(np.asarray(toward, float)) @ self.T_flange_tcp
        T_y = ur5_fk(np.asarray(yaw_of, float)) @ self.T_flange_tcp
        th = math.atan2(T_to[1, 3], T_to[0, 3])
        yaw = math.atan2(T_y[1, 0], T_y[0, 0])
        out = []
        for r in COMPACT_R_MM:
            T = g.transl(r * math.cos(th), r * math.sin(th), self.z_safe_base) @ g.rotz(yaw) @ g.rotx(math.pi)
            q = ik_near(T @ g.inv(self.T_flange_tcp), np.asarray(toward, float))
            if q is not None:
                out.append(q)
        return out

    def plan(self, q_from: Sequence[float], q_to: Sequence[float], holding: str | None = None,
             vias: Sequence[Sequence[float]] = (), column: np.ndarray | None = None,
             vias_first: bool = True, look: bool = False) -> Verdict:
        """Joint path q_from -> (vias) -> q_to: the given vias if that path is free, else the first free detour
        (module docstring). Verdict.ok False -> refuse the move. column = target TCP pose (4 x 4, base frame) of a
        pick / place: while the TCP is in its descent column (_in_column), gripper, jaws and held stone (GRIPPER_PARTS) may
        touch (not overlap) the stones of the world - they are above the slot, next to a butt joint or a magazine
        stack 5 mm away, as in the descent that follows (robodk/motion.py: the last 60 mm untested, above that
        RoboDK's own clearance 0). Camera, adapter and arm links keep CLEARANCE_MM. vias_first False (the offline
        look vias, tools/make_job.py plan_look_vias - planned with this model for the worst case): the direct move
        first when it is free, then the vias. look (a move to a look pose, or from one to the park pose): the up -
        over - down candidates first (look_candidates, the job's vias before them), blended (Verdict.blend_mm); if
        none is free, the high arc and the detours (blended where their corners are free)."""
        self._column = None if column is None else np.asarray(column, float)
        try:
            return self._plan(q_from, q_to, holding, vias, vias_first, look)
        finally:
            self._column = None

    # ── look moves ───────────────────────────────────────────────────────────
    def _tcp(self, q: Sequence[float]) -> np.ndarray:
        return ur5_fk(np.asarray(q, float)) @ self.T_flange_tcp

    def _lift_chain(self, q: Sequence[float], h: float, slant: float = 0.0) -> list[np.ndarray] | None:
        """Joints with the TCP lifted by ARC_STEP_MM, 2 ARC_STEP_MM, ... and finally h mm (same orientation; one short
        joint move each stays close to the line) - vertically, or with slant mm towards the UR base axis per mm of
        rise; [] for h <= 0, None if a step has no IK."""
        if h <= 1e-6:
            return []
        q = np.asarray(q, float)
        if slant <= 0.0:
            out = [self.lift(q, float(d)) for d in [*np.arange(ARC_STEP_MM, h - 1e-6, ARC_STEP_MM), h]]
            return None if any(v is None for v in out) else out
        T = self._tcp(q)
        r = float(np.hypot(T[0, 3], T[1, 3]))
        u = -T[:2, 3] / max(r, 1e-9)                          # horizontal unit vector towards the UR base axis
        out, prev = [], q
        for d in [*np.arange(ARC_STEP_MM, h - 1e-6, ARC_STEP_MM), h]:
            s_ = min(slant * float(d), max(r - LOOK_R_MIN_MM, 0.0))
            qn = ik_near(g.transl(u[0] * s_, u[1] * s_, float(d)) @ T @ g.inv(self.T_flange_tcp), prev)
            if qn is None:
                return None
            out.append(qn)
            prev = qn
        return out

    def _swing(self, qa: Sequence[float], qb: Sequence[float], pull: float) -> list[np.ndarray] | None:
        """Waypoints of the swing qa -> qb (the tops of a look move): none for pull 0 (one joint move - it bows
        towards the UR base by itself), else one per LOOK_ARC_STEP_DEG of base-joint turn (at least one), each at the
        pose of the linear joint interpolation with the TCP height interpolated linearly and the TCP pulled
        pull * sin(pi t) towards the UR base axis (not under LOOK_R_MIN_MM from it); IK near the interpolation. None
        if a waypoint has no IK."""
        if pull <= 0.0:
            return []
        qa, qb = np.asarray(qa, float), np.asarray(qb, float)
        n = max(int(math.ceil(math.degrees(abs(float(qb[0] - qa[0]))) / LOOK_ARC_STEP_DEG - 1e-9)) - 1, 1)
        za, zb = float(self._tcp(qa)[2, 3]), float(self._tcp(qb)[2, 3])
        out = []
        for i in range(1, n + 1):
            t = i / (n + 1)
            ql = qa + (qb - qa) * t
            T = self._tcp(ql)
            r0 = float(np.hypot(T[0, 3], T[1, 3]))
            r = max(r0 - pull * math.sin(math.pi * t), min(LOOK_R_MIN_MM, r0))
            T[:2, 3] *= r / max(r0, 1e-9)
            T[2, 3] = (1.0 - t) * za + t * zb
            q = ik_near(T @ g.inv(self.T_flange_tcp), ql)
            if q is None:
                return None
            out.append(q)
        return out

    def look_candidates(self, q_from: Sequence[float], q_to: Sequence[float]) -> list[tuple[list[np.ndarray], str]]:
        """[(vias, label)] of a look move (module docstring), least joint travel first: from a look (any pose but the
        park pose) straight up, the swing, straight down onto a look - for every LOOK_RISE_MM x LOOK_PULL_MM that has
        IK. A look -> look move of under LOOK_DIRECT_MM (a re-aimed retry) gets the direct move first."""
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        Tf, Tt = self._tcp(q_from), self._tcp(q_to)
        low_f = not np.allclose(q_from, self.park_q, atol=PARK_TOL_RAD)
        low_t = not np.allclose(q_to, self.park_q, atol=PARK_TOL_RAD)
        first = ([([], "direct (re-aimed look)")] if low_f and low_t
                 and float(np.hypot(*(Tt[:2, 3] - Tf[:2, 3]))) < LOOK_DIRECT_MM else [])
        zs = [float(T[2, 3]) for T, low in ((Tf, low_f), (Tt, low_t)) if low]
        cands: list[tuple[list[np.ndarray], str]] = []
        for rise in (LOOK_RISE_MM if zs else ()):
            top = max(zs) + rise
            for su in (LOOK_SLANT if low_f else (0.0,)):
                up = self._lift_chain(q_from, top - float(Tf[2, 3]), su) if low_f else []
                for sd in (LOOK_SLANT if low_t else (0.0,)):
                    down = self._lift_chain(q_to, top - float(Tt[2, 3]), sd) if low_t else []
                    if up is None or down is None:
                        continue
                    for pull in LOOK_PULL_MM:
                        sw = self._swing(up[-1] if up else q_from, down[-1] if down else q_to, pull)
                        if sw is not None:
                            cands.append(([*up, *sw, *down[::-1]],
                                          f"rise {rise:g} mm, slant {su:g}/{sd:g}, pull {pull:g} mm"))
        cands.sort(key=lambda c: travel_deg([q_from, *c[0], q_to]))
        return first + cands

    def _blend_problems(self, qs: Sequence[Sequence[float]], radii: Sequence[float], holding: str | None) -> list[str]:
        """The blended corners of a look move (blend_curves) must be free too. [] = free."""
        for i, curve in enumerate(blend_curves(qs, radii, self.T_flange_tcp), 1):
            p = self.path_problems(curve, holding, from_start=False)
            if p:
                return [f"blend {i}: {p[0]}"]
        return []

    def _plan_look(self, q_from: np.ndarray, q_to: np.ndarray, holding: str | None,
                   vias: Sequence[Sequence[float]], problems: list[str]) -> Verdict | None:
        """The first free look candidate (the job's vias first), blended; None if none is free (problems appended)."""
        cands = ([([np.asarray(v, float) for v in vias], "the job's vias")] if len(vias) else [])
        for c, label in cands + self.look_candidates(q_from, q_to):
            path = [q_from, *c, q_to]
            radii = blend_radii(path, self.T_flange_tcp)
            p = self.path_problems(path, holding) or self._blend_problems(path, radii, holding)
            if not p:
                return Verdict(True, c, problems, radii)
            problems.append(f"look move ({label}): {p[0]}")
        return None

    def candidates(self, q_from, q_to, vias: Sequence[Sequence[float]] = (),
                   vias_first: bool = True) -> list[list[np.ndarray]]:
        """The via lists plan() tries, in order: the job's vias first (if any), then direct and every detour of the
        module docstring sorted by joint travel (travel_deg ~ movej time; Samuel 2026-10-09 "keine unnoetigen Wege" -
        the list order used to send 43 of 134 blocked picks / places of the C through the park pose); park_last puts
        the detours through the park pose behind the others."""
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        cands: list[list[np.ndarray]] = [[]]
        up_from, up_to = self.lift(q_from), self.lift(q_to)
        cands.append([self.park_q])
        for over in [up_to] + [self.lift(q_to, dz) for dz in OVER_TARGET_MM]:   # come down onto the target
            if over is not None:                                                  # from above
                cands += [[over], [self.park_q, over]]
        if up_from is not None:
            cands.append([up_from])
            if up_to is not None:
                cands.append([up_from, up_to])
                cands.append([up_from, self.park_q, up_to])
        overs = [o for o in [up_to] + [self.lift(q_to, dz) for dz in OVER_TARGET_MM] if o is not None]
        ups = [u for u in [self.lift(q_from, dz) for dz in OVER_TARGET_MM] if u is not None]
        for u in ups:                                              # leave the start upwards first (low looks)
            cands += [[u]] + [[u, o] for o in overs[:2]] + [[u, self.park_q]] + [[u, self.park_q, o]
                                                                                for o in overs[:2]]
        for c_to in self.compact(q_to, q_to):                      # via compact poses towards the target
            cands += [[c_to] + ([o] if o is not None else []) for o in [None] + overs[:2]]
            for c_from in self.compact(q_from, q_to)[:1]:
                cands += [[c_from, c_to] + ([o] if o is not None else []) for o in [None] + overs[:1]]
        uniq: list[list[np.ndarray]] = []
        for c in cands:                                            # lift() returns q itself when already high
            c = [v for i, v in enumerate(c) if not (np.allclose(v, q_from) or np.allclose(v, q_to)
                                                    or (i and np.allclose(v, c[i - 1])))]
            if not any(len(c) == len(u) and all(np.allclose(x, y) for x, y in zip(c, u)) for u in uniq):
                uniq.append(c)
        via_park = [any(np.allclose(v, self.park_q) for v in c) for c in uniq]
        order = sorted(range(len(uniq)), key=lambda i: ((via_park[i] if self.park_last else False),
                                                        travel_deg([q_from, *uniq[i], q_to]), i))
        cands = [uniq[i] for i in order]
        if vias:
            job = [np.asarray(v, float) for v in vias]
            rest = [c for c in cands if not (len(c) == len(job) and all(np.allclose(x, y) for x, y in zip(c, job)))]
            cands = [job] + rest if vias_first else [[]] + [job] + [c for c in rest if c]
        return cands

    def _plan(self, q_from, q_to, holding, vias, vias_first: bool = True, look: bool = False) -> Verdict:
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        end = self.state_problems(q_to, holding)
        if end:
            return Verdict(False, [], ["target: " + "; ".join(end)])
        problems: list[str] = []
        if not look:
            return self._detour(q_from, q_to, holding, vias, vias_first, problems)
        v = self._plan_look(q_from, q_to, holding, vias, problems)
        if v is not None:
            return v
        v = self._detour(q_from, q_to, holding, vias, vias_first, problems, arc_first=True)
        if v.ok:                                     # a detour of a look move: blended where its corners are free
            path = [q_from, *v.vias, q_to]
            radii = blend_radii(path, self.T_flange_tcp)
            if not self._blend_problems(path, radii, holding):
                v.blend_mm = radii
        return v

    def _detour(self, q_from, q_to, holding, vias, vias_first: bool, problems: list[str],
                arc_first: bool = False) -> Verdict:
        """The fixed detours (candidates), a detour found before (memo) and the high arc - in this order, or memo and
        high arc first (arc_first: a look move should leave upwards rather than swing low) - then the RRT."""
        def fixed() -> Verdict | None:
            for c in self.candidates(q_from, q_to, vias, vias_first):
                p = self.path_problems([q_from, *c, q_to], holding)
                if not p:
                    return Verdict(True, c, problems)
                problems.append(f"{len(c)} via(s): {p[0]}")
            return None

        def memo() -> Verdict | None:
            tol = math.radians(MEMO_TOL_DEG)
            for qa, qb, mv in reversed(self._memo):
                if (np.max(np.abs(qa - q_from)) <= tol and np.max(np.abs(qb - q_to)) <= tol
                        and not self.path_problems([q_from, *mv, q_to], holding)):
                    return Verdict(True, mv, problems + ["a detour found before"])
            return None

        def arc() -> Verdict | None:
            found = self.high_arc(q_from, q_to, holding, keep_ends=arc_first)
            if found is not None:
                self._remember(q_from, q_to, found)
                return Verdict(True, found, problems)
            problems.append(f"high arc (rise {ARC_RISE_MM[0]:g}..{ARC_RISE_MM[-1]:g} mm): no free one")
            return None

        for step in ((memo, arc, fixed) if arc_first else (fixed, memo, arc)):
            v = step()
            if v is not None:
                return v
        path = self.rrt(q_from, q_to, holding)
        if path is not None:
            self._remember(q_from, q_to, path)
            return Verdict(True, path, problems)
        problems.append(f"RRT ({RRT_ITER} iterations): no path")
        return Verdict(False, [], problems)

    def _remember(self, q_from, q_to, vias) -> None:
        self._memo = (self._memo + [(np.asarray(q_from, float).copy(), np.asarray(q_to, float).copy(),
                                     [np.asarray(v, float).copy() for v in vias])])[-MEMO_N:]

    def high_arc(self, q_from, q_to, holding: str | None = None, keep_ends: bool = False) -> list[np.ndarray] | None:
        """Vias straight up from q_from and straight down onto q_to (TCP lifted in ARC_STEP_MM steps, same orientation)
        to a common height ARC_RISE_MM over the higher end - the lowest rise whose path is free - shortcut (_shortcut);
        None when no rise is free or reachable. keep_ends (look moves): the first ARC_STEP_MM up from q_from and the
        last down onto q_to stay vertical (only what lies between is shortcut), except at the park pose."""
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        zf = float((ur5_fk(q_from) @ self.T_flange_tcp)[2, 3])
        zt = float((ur5_fk(q_to) @ self.T_flange_tcp)[2, 3])
        for rise in ARC_RISE_MM:
            top = max(zf, zt) + rise
            legs = []
            for q, z in ((q_from, zf), (q_to, zt)):
                h = top - z                                      # >= rise > 0: steps, then exactly to the top
                legs.append([self.lift(q, dz) for dz in [float(d) for d in np.arange(ARC_STEP_MM, h, ARC_STEP_MM)]
                             + [h]])
            if any(v is None for leg in legs for v in leg):
                return None                                      # the arm cannot reach that high: higher won't either
            path = [q_from, *legs[0], *legs[1][::-1], q_to]
            if not self.path_problems(path, holding):
                if not keep_ends:
                    return self._shortcut(path, holding)[1:-1]
                a = 0 if np.allclose(q_from, self.park_q, atol=PARK_TOL_RAD) else 1      # first kept via (lift 1)
                b = len(path) - (1 if np.allclose(q_to, self.park_q, atol=PARK_TOL_RAD) else 2)   # last (descent 1)
                mid = self._shortcut(path[a:b + 1], holding, from_start=a == 0)
                return mid[(1 if a == 0 else 0):(len(mid) - 1 if b == len(path) - 1 else len(mid))]
        return None

    def rrt(self, q_from, q_to, holding: str | None = None, iters: int | None = None, seed: int = 1
            ) -> list[np.ndarray] | None:
        """Joint-space RRT-Connect in the capsule model (as robodk/motion.Planner.rrt): nodes free, edges checked
        every STEP_DEG, samples within RRT_MARGIN_DEG of the start / goal joint box, then shortcut. The vias, or None.
        Deterministic (seeded)."""
        rnd = np.random.default_rng(seed)
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        margin = np.radians(RRT_MARGIN_DEG)
        lo, hi = np.minimum(q_from, q_to) - margin, np.maximum(q_from, q_to) + margin
        step = math.radians(RRT_STEP_DEG)
        tol = self._contacts(q_from, holding)

        def free(qa, qb, first=False) -> bool:
            if first:
                return not self.segment_problems(qa, qb, holding, from_start=True)
            return not self.segment_problems(qa, qb, holding)

        def steer(a, b):
            d = float(np.max(np.abs(b - a)))
            return b.copy() if d <= step else a + (b - a) * (step / d)

        ta, tb = [(q_from, None)], [(q_to, None)]
        for _ in range(iters or RRT_ITER):
            x = rnd.uniform(lo, hi)
            for tree, other in ((ta, tb), (tb, ta)):
                i = min(range(len(tree)), key=lambda k: float(np.sum(np.abs(tree[k][0] - x))))
                qn = steer(tree[i][0], x)
                start_edge = tree is ta and i == 0
                if self.state_problems(qn, holding, tol if start_edge else None) or not free(tree[i][0], qn,
                                                                                          start_edge):
                    continue
                tree.append((qn, i))
                j = min(range(len(other)), key=lambda k: float(np.sum(np.abs(other[k][0] - qn))))
                if free(qn, other[j][0], other is ta and j == 0):
                    def chain(t, k):
                        out = []
                        while k is not None:
                            out.append(t[k][0])
                            k = t[k][1]
                        return out
                    pa, pb = (chain(tree, len(tree) - 1), chain(other, j))
                    full = (pa[::-1] + pb) if tree is ta else (pb[::-1] + pa)
                    return self._shortcut(full, holding)[1:-1]
        return None

    def _shortcut(self, path: list, holding: str | None, from_start: bool = True) -> list:
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and self.segment_problems(path[i], path[j], holding, from_start=from_start and i == 0):
                j -= 1
            out.append(path[j])
            i = j
        return out
