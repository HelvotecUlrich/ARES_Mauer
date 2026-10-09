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

Detours (plan), first clear one wins: direct; via the park pose; straight above the target (transfer height, else
OVER_TARGET_MM higher), alone or after the park pose; lift the TCP vertically to the transfer height at the start;
lift at the start and above the target; lift - park - lift; compact poses near the base towards the target (as
robodk/motion.Planner.compact), with and without a via above the target; up from the start by OVER_TARGET_MM; then a
seeded joint-space RRT-Connect (RRT_*) with shortcut. park_last (magazine dry run): the same list with every detour
through the park pose moved behind the others. The transfer height carries a stone with its
pins LIFT_MARGIN_MM over a full magazine (as robodk/simulate.py safe_z(full)).

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
RRT_ITER = 1000                   # RRT-Connect fallback (robodk/motion.Planner.rrt): iterations,
RRT_STEP_DEG = 15.0               # largest joint step of an extension,
RRT_MARGIN_DEG = (120.0, 60.0, 60.0, 120.0, 60.0, 120.0)   # sampling box around start / goal per joint


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
        """Problems of the joint move q0 -> q1 (first sample that fails), [] = free. from_start: q0 is where the arm
        stands now - its contacts may stay, not deepen (state_problems tolerate)."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        tol = self._contacts(q0, holding) if from_start else None
        n = max(1, int(math.ceil(float(np.max(np.abs(q1 - q0))) / self.step)))
        for i in range(n + 1):
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
             vias: Sequence[Sequence[float]] = (), column: np.ndarray | None = None) -> Verdict:
        """Joint path q_from -> (vias) -> q_to: the given vias if that path is free, else the first free detour
        (module docstring). Verdict.ok False -> refuse the move. column = target TCP pose (4 x 4, base frame) of a
        pick / place: while the TCP is in its descent column (_in_column), gripper, jaws and held stone (GRIPPER_PARTS) may
        touch (not overlap) the stones of the world - they are above the slot, next to a butt joint or a magazine
        stack 5 mm away, as in the descent that follows (robodk/motion.py: the last 60 mm untested, above that
        RoboDK's own clearance 0). Camera, adapter and arm links keep CLEARANCE_MM."""
        self._column = None if column is None else np.asarray(column, float)
        try:
            return self._plan(q_from, q_to, holding, vias)
        finally:
            self._column = None

    def _plan(self, q_from, q_to, holding, vias) -> Verdict:
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        end = self.state_problems(q_to, holding)
        if end:
            return Verdict(False, [], ["target: " + "; ".join(end)])
        problems: list[str] = []
        cands: list[list[np.ndarray]] = [[np.asarray(v, float) for v in vias]] if vias else []
        cands.append([])
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
        cands = uniq
        if self.park_last:
            via_park = [any(np.allclose(v, self.park_q) for v in c) for c in cands]
            cands = [c for c, p in zip(cands, via_park) if not p] + [c for c, p in zip(cands, via_park) if p]
        for c in cands:
            p = self.path_problems([q_from, *c, q_to], holding)
            if not p:
                return Verdict(True, c, problems)
            problems.append(f"{len(c)} via(s): {p[0]}")
        path = self.rrt(q_from, q_to, holding)
        if path is not None:
            return Verdict(True, path, problems)
        problems.append(f"RRT ({RRT_ITER} iterations): no path")
        return Verdict(False, [], problems)

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

    def _shortcut(self, path: list, holding: str | None) -> list:
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and self.segment_problems(path[i], path[j], holding, from_start=(i == 0)):
                j -= 1
            out.append(path[j])
            i = j
        return out
