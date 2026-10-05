"""Wall layout and placement sequence (pure Python, no RoboDK, no numpy) for interlocking stones in running bond.

Rule: a stone can only be placed when every stone of the course below that lies under it is already placed (the
pins make it impossible to slide a stone in under another one later). Supports are found by FOOTPRINT OVERLAP along
the wall: a full stone above a half stone sits on the half stone and the next full stone, a half stone sits on one
stone below.

Rule 2: a stone is never placed "in the open": a course-0 stone must sit next to an already placed course-0 stone of
the same leg (only the very first stone of a leg is free); stones of higher courses sit on their complete supports.

Straight wall (`layout`, `sequence`, `check_plan` - used by robodk/simulate.py, look_study.py, tools/make_job.py):
full stones only, so both wall ends become slopes: course k starts and ends half a stone (k x bond offset)
later/earlier than course 0 - a trapezoid.

Legs (L wall, Samuel 2026-10-05, config [wall] shape = "L", [[wall.legs]]): every leg is its own straight wall with
its own frame in the wall frame (origin on the leg centreline at the leg start, x along the leg, y towards the ARES
side of that leg, z up). With half stones ([half_brick], Samuel 2026-10-05: "half stones will go on the end and maybe
also the corners") a leg is a RECTANGLE in running bond: even courses n full stones, odd courses half + (n-1) full +
half, so both leg ends are vertical. Without half stones (`half_stones=False`) a leg falls back to the trapezoid.
Legs are independent: no stone is supported across legs (the pin pattern 53 x 100.5 mm is not square - a stone
turned by 90 deg can never engage the sockets of a stone below), the corner is a vertical BUTT joint
(`butt_corner`): the next leg's flat end face stands [brick] rib_mm (ribs on the long faces) + [wall] corner_gap_mm
off the previous leg's body. `check_legs` checks the cross-leg footprints over the ribs (`stone_footprint(rib=True)`).

Sequencing: ARES stops at leg positions a_j (leg coordinate of the ARES centre). At each stop the wall is continued
where it ends: of all reachable stones that satisfy both rules, the one closest to the start of the leg is placed
next (lower course first on a tie). This grows the wall as a staircase - course 0 one stone ahead, the courses above
following on top. Then ARES moves on by a whole number of stone lengths, as far as possible while every unfinished
stone stays reachable; the next stop first completes the slope left behind, then continues. `sequence_leg` (legs)
additionally starts at the farthest grid position that leaves no stone behind and puts the LAST stop of a leg in
the middle of the positions from which all remaining stones are reachable.

Reach: `reach_fn(table)` on the RoboDK reach table {course: {u_rel: ok}} (results/reach_table.json, 20 mm grid, full
stone centres). Half stones use the full-stone table (a half stone at the same TCP lies inside the full stone's
volume -> conservative); centres between grid points need BOTH neighbouring grid points ok; with margin_mm > 0 every
grid point within +-margin_mm must be ok (ARES may stand up to the sequencer's stop tolerance off the nominal stop).

Units mm, angles rad (config *_deg in degrees).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Mapping, Sequence

FULL, HALF = "full", "half"
EPS = 1e-6


@dataclass(frozen=True)
class Stone:
    course: int
    index: int
    u: float          # centre along the wall/leg [mm] (leg frame x, origin = start of course 0)
    z_top: float      # top of the stone [mm]
    leg: str = ""     # "" = the straight wall (no legs)
    kind: str = FULL  # FULL | HALF
    length: float = 0.0   # along the leg [mm]; 0 = [brick] length (straight wall)

    @property
    def key(self) -> tuple:
        return (self.course, self.index) if not self.leg else (self.leg, self.course, self.index)

    @property
    def label(self) -> str:
        return f"{self.leg}c{self.course}i{self.index}" + ("h" if self.kind == HALF else "")

    def len_(self, default: float) -> float:
        return self.length or default


@dataclass(frozen=True)
class Leg:
    """A straight leg of the wall: n0 full stones in course 0, frame (x, y, theta) in the wall frame."""
    name: str
    n0: int
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0      # rad, heading of the leg x axis in the wall frame

    def to_wall(self, u: float, v: float) -> tuple[float, float]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return self.x + c * u - s * v, self.y + s * u + c * v

    def from_wall(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        dx, dy = x - self.x, y - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def pose_to_wall(self, u: float, v: float, th: float) -> tuple[float, float, float]:
        """Planar pose (u, v, th) in the leg frame -> (x, y, theta) in the wall frame."""
        x, y = self.to_wall(u, v)
        return x, y, _wrap(th + self.theta)


def _wrap(a: float) -> float:
    w = (a + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if abs(w + math.pi) < 1e-12 else w


# ── config ────────────────────────────────────────────────────────────────────
def pitch(cfg: Mapping) -> float:
    b = cfg["brick"]
    return float(b["length"]) + float(b["head_joint"])


def half_length(cfg: Mapping) -> float:
    """Half stone length ([half_brick] length, PLACEHOLDER), default half a full stone."""
    return float(cfg.get("half_brick", {}).get("length", cfg["brick"]["length"] / 2.0))


def leg_length(cfg: Mapping, n0: int) -> float:
    """Length of course 0 of a leg with n0 full stones [mm]."""
    return n0 * pitch(cfg) - float(cfg["brick"]["head_joint"])


def course_top(cfg: Mapping, k: int) -> float:
    b, w = cfg["brick"], cfg["wall"]
    return w["base_z"] + k * (b["height"] + b["bed_joint"]) + b["height"]


def is_l(cfg: Mapping) -> bool:
    return bool(cfg.get("wall", {}).get("legs"))


def legs(cfg: Mapping) -> list[Leg]:
    """Legs from [[wall.legs]] (name, n0, xyz_in_wall [mm], rpy_in_wall_deg [deg]); [] for a straight wall.
    ValueError for a leg that is not on the floor (z, roll, pitch must be 0) or a duplicate name."""
    out = []
    for d in cfg.get("wall", {}).get("legs", []) or []:
        xyz = [float(v) for v in d.get("xyz_in_wall", (0.0, 0.0, 0.0))]
        rpy = [float(v) for v in d.get("rpy_in_wall_deg", (0.0, 0.0, 0.0))]
        if abs(xyz[2]) > EPS or abs(rpy[0]) > EPS or abs(rpy[1]) > EPS:
            raise ValueError(f"leg {d.get('name')!r}: only floor-level legs rotated about z are supported")
        out.append(Leg(str(d["name"]), int(d["n0"]), xyz[0], xyz[1], math.radians(rpy[2])))
    names = [lg.name for lg in out]
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate leg names {names}")
    return out


def rib(cfg: Mapping) -> float:
    """[brick] rib_mm: height of the horizontal ribs on both long faces of a stone (0 if missing)."""
    return float(cfg["brick"].get("rib_mm", 0.0))


def corner_gap(cfg: Mapping) -> float:
    """[wall] corner_gap_mm (ASSUMPTION): clearance between the next leg's end face and prev's rib crests."""
    return float(cfg.get("wall", {}).get("corner_gap_mm", 0.0))


def butt_corner(cfg: Mapping, prev: Leg, n0: int, name: str) -> Leg:
    """The next leg of an L: perpendicular at the far end of `prev`, on the side away from prev's ARES side (-y of
    prev). prev runs through the corner (its course-0 end is flush with the body of the new leg's outer face), the new
    leg starts at prev's inside face (vertical butt joint) - beyond prev's ribs and the corner gap: origin = prev
    (length - width/2, -(width/2 + rib_mm + corner_gap_mm)), heading prev - 90 deg."""
    W = float(cfg["brick"]["width"])
    x, y = prev.to_wall(leg_length(cfg, prev.n0) - W / 2.0, -(W / 2.0 + rib(cfg) + corner_gap(cfg)))
    return Leg(name, int(n0), x, y, _wrap(prev.theta - math.pi / 2.0))


# ── layout ────────────────────────────────────────────────────────────────────
def layout(cfg: dict, n0: int) -> list:
    """Trapezoid wall: course 0 has n0 stones, every course above one stone less (bond offset 0.5)."""
    b, w = cfg["brick"], cfg["wall"]
    pitch_ = b["length"] + b["head_joint"]
    shift = w["bond_offset"] * pitch_
    stones = []
    for k in range(w["courses"]):
        z_top = w["base_z"] + k * (b["height"] + b["bed_joint"]) + b["height"]
        n_k = int((n0 * pitch_ - 2 * k * shift) / pitch_ + 1e-9)      # both ends slope back by `shift` per course
        for i in range(n_k):
            stones.append(Stone(k, i, k * shift + b["length"] / 2 + i * pitch_, z_top))
    return stones


def layout_leg(cfg: Mapping, leg: Leg, half_stones: bool = True) -> list[Stone]:
    """Stones of one leg (leg frame u). half_stones: rectangle (half stones at both ends of odd courses); else the
    trapezoid of `layout`. ValueError if the half stone does not fit the bond (length + head joint = bond offset)
    or is not as high as a full stone."""
    b, w = cfg["brick"], cfg["wall"]
    L, hj = float(b["length"]), float(b["head_joint"])
    p = L + hj
    if not half_stones:
        return [replace(s, leg=leg.name, length=L) for s in layout(dict(cfg), leg.n0)]
    hb = cfg.get("half_brick", {})
    Lh = half_length(cfg)
    shift = float(w["bond_offset"]) * p
    if abs(Lh + hj - shift) > EPS:
        raise ValueError(f"half stone length {Lh:g} + head joint {hj:g} != bond offset {shift:g} mm: no rectangle")
    if abs(float(hb.get("height", b["height"])) - float(b["height"])) > EPS:
        raise ValueError("half stone height differs from the full stone height: courses would not line up")
    stones = []
    for k in range(int(w["courses"])):
        z = course_top(cfg, k)
        if (k * float(w["bond_offset"])) % 1.0 < EPS:
            row = [(L / 2 + i * p, FULL, L) for i in range(leg.n0)]
        else:
            row = ([(Lh / 2, HALF, Lh)] + [(Lh + hj + L / 2 + i * p, FULL, L) for i in range(leg.n0 - 1)]
                   + [(leg_length(cfg, leg.n0) - Lh / 2, HALF, Lh)])
        stones += [Stone(k, i, u, z, leg.name, kind, ln) for i, (u, kind, ln) in enumerate(row)]
    return stones


def layout_legs(cfg: Mapping, legs_: Sequence[Leg], half_stones: bool = True) -> list[Stone]:
    return [s for lg in legs_ for s in layout_leg(cfg, lg, half_stones)]


# ── supports ──────────────────────────────────────────────────────────────────
def overlap(a: Stone, b: Stone, length: float) -> bool:
    """Footprints overlap along the leg (positive length; touching ends do not count)."""
    return abs(a.u - b.u) < (a.len_(length) + b.len_(length)) / 2.0 - EPS


def adjacent(a: Stone, b: Stone, length: float, head_joint: float = 0.0) -> bool:
    """Neighbours in a course: end to end (with the head joint)."""
    return abs(abs(a.u - b.u) - ((a.len_(length) + b.len_(length)) / 2.0 + head_joint)) < EPS


def supports(stone: Stone, by_course: dict, length: float) -> list:
    """Stones of the course below (same leg) whose footprint overlaps the footprint of `stone`."""
    if stone.course == 0:
        return []
    return [s for s in by_course.get(stone.course - 1, []) if s.leg == stone.leg and overlap(s, stone, length)]


def _by_course(stones: Sequence[Stone]) -> dict:
    out: dict = {}
    for s in stones:
        out.setdefault(s.course, []).append(s)
    return out


# ── reach ─────────────────────────────────────────────────────────────────────
def table_grid(table: Mapping) -> float:
    us = sorted({float(u) for d in table.values() for u in d})
    return min(b - a for a, b in zip(us, us[1:]))


def reach_fn(table: Mapping, grid: float | None = None, margin_mm: float = 0.0
             ) -> tuple[Callable[[int, float], bool], dict, dict, float]:
    """(reach(course, u_rel) -> bool, lo {course: smallest ok u_rel}, hi {course: largest}, grid) from a reach table
    {course: {u_rel: ok}}. A centre on a grid point needs that point ok, a centre between grid points needs both
    neighbours ok (conservative), and every grid point within +-margin_mm must be ok as well."""
    g = float(grid or table_grid(table))
    tab = {int(k): {round(float(u), 3): bool(v) for u, v in d.items()} for k, d in table.items()}
    lo = {k: min(u for u, ok in d.items() if ok) for k, d in tab.items()}
    hi = {k: max(u for u, ok in d.items() if ok) for k, d in tab.items()}

    def reach(k: int, u_rel: float) -> bool:
        a = math.floor((u_rel - margin_mm) / g + 1e-6)
        b = math.ceil((u_rel + margin_mm) / g - 1e-6)
        d = tab.get(k, {})
        return all(d.get(round(i * g, 3), False) for i in range(a, b + 1))

    return reach, lo, hi, g


def _a_window(stones: Sequence[Stone], lo: Mapping, hi: Mapping, grid: float, margin: float) -> tuple[float, float]:
    """[a_min, a_max] (grid multiples) of the ARES positions from which every stone lies inside its course's reach
    interval (holes in the table not considered - the caller checks with the reach function)."""
    a_max, a_min = math.inf, -math.inf
    for s in stones:
        # u_rel = s.u - a; floor_grid(u_rel - margin) >= lo, ceil_grid(u_rel + margin) <= hi, a multiple of grid
        a_max = min(a_max, grid * math.floor((s.u - margin - lo[s.course]) / grid + 1e-6))
        a_min = max(a_min, grid * math.ceil((s.u + margin - hi[s.course]) / grid - 1e-6))
    return a_min, a_max


# ── sequence ──────────────────────────────────────────────────────────────────
def _fill_stop(cfg: Mapping, stones: Sequence[Stone], by_course: dict, placed: set, reach, a: float) -> list:
    """Greedy batch at ARES position a (rules 1 + 2, staircase order). Mutates `placed`."""
    L = float(cfg["brick"]["length"])
    hj = float(cfg["brick"]["head_joint"])
    batch = []
    while True:
        cand = [s for s in stones if s.key not in placed and reach(s.course, s.u - a)
                and all(p.key in placed for p in supports(s, by_course, L))
                and (s.course > 0 or not any(t.key in placed for t in by_course[0])
                     or any(adjacent(t, s, L, hj) for t in by_course[0] if t.key in placed))]
        if not cand:
            return batch
        nxt = min(cand, key=lambda s: (s.u, s.course))          # continue where the wall ends
        placed.add(nxt.key)
        batch.append(nxt)


def sequence(cfg: dict, stones: list, reach: Callable[[int, float], bool], reach_lo: dict,
             a0: float, max_stops: int = 50) -> list:
    """Greedy plan: [(a_j, [Stone, ...]), ...]. `reach(course, u_rel)`, `reach_lo[course]` = smallest reachable
    u_rel of that course; a0 = first ARES position."""
    pitch_ = pitch(cfg)
    by_course = _by_course(stones)
    placed: set = set()
    plan, a = [], a0
    for _ in range(max_stops):
        batch = _fill_stop(cfg, stones, by_course, placed, reach, a)
        plan.append((a, batch))
        open_ = [s for s in stones if s.key not in placed]
        if not open_:
            return plan
        # advance by m stone pitches: every unfinished stone must stay at or ahead of the trailing reach edge
        m = 0
        while all(s.u - (a + (m + 1) * pitch_) >= reach_lo[s.course] - 1e-6 for s in open_):
            m += 1
        if m == 0:
            raise RuntimeError(f"stuck at a = {a:.0f}: {len(open_)} stones left that cannot be reached/supported")
        a += m * pitch_
    raise RuntimeError("too many stops")


def sequence_leg(cfg: Mapping, stones: Sequence[Stone], reach: Callable[[int, float], bool], lo: Mapping,
                 hi: Mapping, grid: float, margin_mm: float = 0.0, max_stops: int = 50) -> list:
    """Plan of ONE leg: [(a_j, [Stone, ...]), ...] (a in the leg frame, multiples of `grid` so that full stones lie on
    reach-table grid points). First stop: the farthest position that leaves no stone behind the trailing reach edge;
    then as `sequence` (advance by whole pitches); a stop from which ALL remaining stones are reachable is put in the
    middle of that window (never behind the previous stop). RuntimeError when stuck."""
    if len({s.leg for s in stones}) > 1:
        raise ValueError("sequence_leg: stones of more than one leg")
    pitch_ = pitch(cfg)
    by_course = _by_course(stones)
    placed: set = set()
    plan: list = []
    a_prev = None
    for _ in range(max_stops):
        open_ = [s for s in stones if s.key not in placed]
        a_min, a_max = _a_window(open_, lo, hi, grid, margin_mm)
        if a_prev is not None:
            a_min = max(a_min, a_prev + grid)
        a = None
        if a_min <= a_max:                                    # the rest fits in one stop: centre it
            mid = grid * round((a_min + a_max) / 2.0 / grid)
            for c in sorted({min(max(mid, a_min), a_max), a_max, a_min}, key=lambda v: abs(v - mid)):
                if all(reach(s.course, s.u - c) for s in open_):
                    a = c
                    break
        if a is None:
            if a_prev is None:
                a = a_max                                      # farthest start that leaves nothing behind
            else:
                m = 0
                while all(math.floor((s.u - (a_prev + (m + 1) * pitch_) - margin_mm) / grid + 1e-6) * grid
                          >= lo[s.course] - EPS for s in open_):
                    m += 1
                if m == 0:
                    raise RuntimeError(f"leg {stones[0].leg}: stuck at a = {a_prev:.0f}: {len(open_)} stones left")
                a = a_prev + m * pitch_
        batch = _fill_stop(cfg, stones, by_course, placed, reach, a)
        if not batch:
            raise RuntimeError(f"leg {stones[0].leg}: nothing placeable at a = {a:.0f} ({len(open_)} stones left)")
        plan.append((a, batch))
        a_prev = a
        if len(placed) == len(stones):
            return plan
    raise RuntimeError("too many stops")


def check_plan(cfg: dict, stones: list, plan: list) -> list:
    """Independent check of a plan (one leg or the straight wall): every stone once, supports (footprint overlap)
    before the stone, never a stone under a placed one, never a course-0 stone in the open."""
    L = cfg["brick"]["length"]
    hj = cfg["brick"]["head_joint"]
    by_course = _by_course(stones)
    errors, placed = [], set()
    order = [s for _, batch in plan for s in batch]
    if sorted(s.key for s in order) != sorted(s.key for s in stones):
        errors.append("not every stone placed exactly once")
    for s in order:
        for p in supports(s, by_course, L):
            if p.key not in placed:
                errors.append(f"stone {s.key} placed before its support {p.key}")
        above = [t for t in by_course.get(s.course + 1, []) if t.leg == s.leg and overlap(t, s, L)]
        for t in above:
            if t.key in placed:
                errors.append(f"stone {s.key} would have to go under the already placed {t.key}")
        if s.course == 0 and any(t.key in placed for t in by_course[0] if t.leg == s.leg) and not any(
                adjacent(t, s, L, hj) for t in by_course[0] if t.key in placed and t.leg == s.leg):
            errors.append(f"stone {s.key} placed in the open (no neighbour in course 0)")
        placed.add(s.key)
    return errors


# ── legs: footprints and cross-leg checks ─────────────────────────────────────
def stone_footprint(cfg: Mapping, s: Stone, leg: Leg | None = None, ribs: bool = False) -> list[tuple[float, float]]:
    """Footprint corners of a stone in the wall frame (counter-clockwise); ribs: across over the ribs of both long
    faces (width + 2 rib_mm), the end faces are flat."""
    L = s.len_(float(cfg["brick"]["length"]))
    W = float(cfg["brick"]["width"]) + (2.0 * rib(cfg) if ribs else 0.0)
    lg = leg or Leg("", 0)
    return [lg.to_wall(s.u + du, dv) for du, dv in ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2))]


def leg_footprint(cfg: Mapping, leg: Leg) -> list[tuple[float, float]]:
    """Course-0 footprint of a leg (= its base blocks) in the wall frame: u 0..length, v -width/2..width/2."""
    Ln = leg_length(cfg, leg.n0)
    W = float(cfg["brick"]["width"])
    return [leg.to_wall(u, v) for u, v in ((0.0, -W / 2), (Ln, -W / 2), (Ln, W / 2), (0.0, W / 2))]


def polys_overlap(P: Sequence, Q: Sequence, tol: float = 1e-6) -> bool:
    """Convex polygons overlap with positive area (separating axis test; touching edges do not count)."""
    for poly in (P, Q):
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            nx, ny = y2 - y1, x1 - x2
            ln = math.hypot(nx, ny)
            if ln < 1e-12:
                continue
            nx, ny = nx / ln, ny / ln
            pa = [nx * x + ny * y for x, y in P]
            pb = [nx * x + ny * y for x, y in Q]
            if max(pa) <= min(pb) + tol or max(pb) <= min(pa) + tol:
                return False
    return True


def check_legs(cfg: Mapping, legs_: Sequence[Leg], stones: Sequence[Stone]) -> list[str]:
    """Cross-leg checks: no two stones of the same course overlap (any legs; footprints over the ribs - the ribs of a
    leg's long face reach into the corner), no stone overlaps a stone of ANOTHER leg in the course below (= would be
    supported across legs) or above, every stone belongs to a known leg."""
    by_name = {lg.name: lg for lg in legs_}
    errors = [f"stone {s.key}: unknown leg {s.leg!r}" for s in stones if s.leg not in by_name]
    fp = {s.key: stone_footprint(cfg, s, by_name.get(s.leg), ribs=True) for s in stones}
    by_course = _by_course(stones)
    for k, row in by_course.items():
        for i, a in enumerate(row):
            for b in row[i + 1:]:
                if polys_overlap(fp[a.key], fp[b.key]):
                    errors.append(f"stones {a.key} and {b.key} overlap (course {k})")
        for a in row:
            for b in by_course.get(k - 1, []):
                if b.leg != a.leg and polys_overlap(fp[a.key], fp[b.key]):
                    errors.append(f"stone {a.key} would be supported by {b.key} of another leg")
    return errors


def check_plan_legs(cfg: Mapping, legs_: Sequence[Leg], stones: Sequence[Stone], plans: Mapping[str, list]) -> list:
    """check_plan per leg + check_legs + every leg planned."""
    errors = []
    for lg in legs_:
        own = [s for s in stones if s.leg == lg.name]
        if lg.name not in plans:
            errors.append(f"leg {lg.name}: no plan")
            continue
        errors += [f"leg {lg.name}: {e}" for e in check_plan(dict(cfg), own, plans[lg.name])]
        if any(s.leg != lg.name for _, batch in plans[lg.name] for s in batch):
            errors.append(f"leg {lg.name}: plan holds stones of another leg")
    return errors + check_legs(cfg, legs_, stones)


def plan_legs(cfg: Mapping, legs_: Sequence[Leg], table: Mapping, half_stones: bool = True,
              margin_mm: float = 0.0) -> tuple[list[Stone], dict[str, list]]:
    """(stones, {leg: [(a, batch), ...]}) for every leg on the reach table; ValueError for an invalid plan."""
    reach, lo, hi, grid = reach_fn(table, margin_mm=margin_mm)
    stones = layout_legs(cfg, legs_, half_stones)
    plans = {lg.name: sequence_leg(cfg, [s for s in stones if s.leg == lg.name], reach, lo, hi, grid, margin_mm)
             for lg in legs_}
    errors = check_plan_legs(cfg, legs_, stones, plans)
    if errors:
        raise ValueError("invalid L plan: " + "; ".join(errors))
    return stones, plans
