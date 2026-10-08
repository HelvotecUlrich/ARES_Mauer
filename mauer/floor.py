"""Floor plan in the wall frame: ARES footprint, obstacles (wall legs, base blocks, reference-board plates, pick-up
station table), ARES routes as explicit waypoint lists, the route clearance validator and a small route planner.

Pure Python (math only). Frames and units: docs/ARCHITECTURE.md - wall frame, mm, rad; waypoints are planar ARES
poses (mauer.reference.Pose2D: base_link position, heading of the ARES x axis).

ARES footprint: the chassis rectangle [ares] length x width (CONFIRMED, chassis mesh x +-560, y +-300) centred on
base_link. A rotation on the spot sweeps the circle of the half diagonal about base_link.

Routes (PLC v2.9 relative move: ONE translation (body frame) OR ONE rotation per command, >= 2 mm / 0.2 deg,
mauer/ares/ads.py check_move): every pair of consecutive waypoints must be a pure translation (same heading) or a
pure rotation (same position). `validate_route` checks a route against the obstacles:
- every waypoint footprint is free (no overlap); INTERMEDIATE waypoints keep >= clearance_mm ([routes]
  clearance_mm, ASSUMPTION 50 mm) to every obstacle - the end points are stops / the dock, whose own geometry is
  given (the ARES front edge stands 10 mm from the floor plates at a stop by design, [plates] plate_depth);
- a translation sweeps the convex hull of its start and end footprints (exact for a translated convex shape); it
  must keep >= min(clearance, distance at its start, distance at its end) to every obstacle - it never comes closer
  than the clearance except while leaving / approaching a stop or the dock;
- a rotation sweeps the circle of the half diagonal; it must keep >= clearance to every obstacle.

`plan_route` builds routes of the form: back off from the start (body -x) -> one to three axis-parallel translations
-> one rotation (if the heading changes) at a point whose circle is clear -> one to three translations -> back-off
point of the goal -> approach (body +x). Translations after the rotation are weighted (after_rotation_weight, ASSUMPTION 5):
a heading error from the rotation (E003: 0.14 deg scatter, 1.2 deg after reversing the direction) grows into a lateral
error along every later translation and the route is only corrected at its end (sequencer closed loop), so the rotation
is put as close to the goal as the clearances allow.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Sequence

from .reference import Pose2D, wrap_angle

Pt = tuple[float, float]
TOL_MM = 0.5            # waypoints closer than this are the same position
TOL_RAD = 1e-6          # same heading


# ── polygons ──────────────────────────────────────────────────────────────────
def box_poly(x0: float, y0: float, x1: float, y1: float) -> list[Pt]:
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def rect_poly(cx: float, cy: float, theta: float, length: float, width: float) -> list[Pt]:
    """Rectangle centred at (cx, cy), `length` along the heading theta, `width` across (counter-clockwise)."""
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for du, dv in ((-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2),
                   (-length / 2, width / 2)):
        out.append((cx + c * du - s * dv, cy + s * du + c * dv))
    return out


def transform(poly: Iterable[Pt], x: float, y: float, theta: float) -> list[Pt]:
    c, s = math.cos(theta), math.sin(theta)
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in poly]


def hull(points: Iterable[Pt]) -> list[Pt]:
    """Convex hull (Andrew's monotone chain), counter-clockwise."""
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def overlaps(P: Sequence[Pt], Q: Sequence[Pt], tol: float = 1e-6) -> bool:
    """Convex polygons overlap with positive area (separating axis test; touching does not count)."""
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


def penetration(P: Sequence[Pt], Q: Sequence[Pt]) -> float:
    """Penetration depth of two convex polygons [mm]: the smallest overlap along the edge normals of both (separating
    axis test), 0 if they do not overlap."""
    depth = math.inf
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
            d = min(max(pa), max(pb)) - max(min(pa), min(pb))
            if d <= 0:
                return 0.0
            depth = min(depth, d)
    return 0.0 if depth == math.inf else depth


def _pt_seg(p: Pt, a: Pt, b: Pt) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def point_inside(p: Pt, P: Sequence[Pt]) -> bool:
    """Point strictly inside a convex polygon (either orientation)."""
    sign = 0.0
    n = len(P)
    for i in range(n):
        (x1, y1), (x2, y2) = P[i], P[(i + 1) % n]
        c = (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)
        if abs(c) < 1e-9:
            return False
        if sign == 0.0:
            sign = c
        elif (c > 0) != (sign > 0):
            return False
    return True


def point_dist(p: Pt, P: Sequence[Pt]) -> float:
    """Distance from a point to a convex polygon (0 inside)."""
    if len(P) >= 3 and point_inside(p, P):
        return 0.0
    n = len(P)
    return min(_pt_seg(p, P[i], P[(i + 1) % n]) for i in range(n))


def poly_dist(P: Sequence[Pt], Q: Sequence[Pt]) -> float:
    """Distance between two convex polygons (0 if they overlap or touch)."""
    if overlaps(P, Q):
        return 0.0
    nP, nQ = len(P), len(Q)
    d = math.inf
    for i in range(nP):
        a, b = P[i], P[(i + 1) % nP]
        for q in Q:
            d = min(d, _pt_seg(q, a, b))
    for j in range(nQ):
        a, b = Q[j], Q[(j + 1) % nQ]
        for p in P:
            d = min(d, _pt_seg(p, a, b))
    return d


def circle_dist(c: Pt, r: float, P: Sequence[Pt]) -> float:
    """Distance from a circle to a convex polygon (negative = overlap)."""
    return point_dist(c, P) - r


# ── ARES ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class AresShape:
    length: float = 1120.0          # [ares] length (CONFIRMED: chassis mesh x +-560)
    width: float = 600.0            # [ares] width (CONFIRMED: chassis mesh y +-300)
    back_mm: float = 0.0            # beyond the -x end: the UR control box ([ares] controller_out_mm, 2026-10-08)

    @classmethod
    def from_config(cls, cfg: Mapping) -> "AresShape":
        a = cfg["ares"]
        return cls(float(a["length"]), float(a["width"]), float(a.get("controller_out_mm", 0.0) or 0.0))

    @property
    def radius(self) -> float:
        """Radius of the circle swept by a rotation about base_link: the farthest footprint corner."""
        return math.hypot(self.length / 2.0 + self.back_mm, self.width / 2.0)

    def footprint(self, p: Pose2D) -> list[Pt]:
        """Chassis plus the control box sticking out at the -x end (one rectangle over the full width)."""
        c, s, off = math.cos(p.theta_rad), math.sin(p.theta_rad), -self.back_mm / 2.0
        return rect_poly(p.x_mm + c * off, p.y_mm + s * off, p.theta_rad, self.length + self.back_mm, self.width)


@dataclass
class Obstacle:
    name: str
    kind: str                 # "leg" | "block" | "plate" | "table"
    poly: list[Pt]


# ── routes ────────────────────────────────────────────────────────────────────
@dataclass
class Segment:
    kind: str                 # "translate" | "rotate"
    start: Pose2D
    end: Pose2D

    @property
    def length_mm(self) -> float:
        return math.hypot(self.end.x_mm - self.start.x_mm, self.end.y_mm - self.start.y_mm)

    @property
    def angle_rad(self) -> float:
        return wrap_angle(self.end.theta_rad - self.start.theta_rad)


def same_pose(a: Pose2D, b: Pose2D, tol_mm: float = TOL_MM, tol_rad: float = TOL_RAD) -> bool:
    return (math.hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm) < tol_mm
            and abs(wrap_angle(a.theta_rad - b.theta_rad)) < tol_rad)


def segments(route: Sequence[Pose2D], tol_mm: float = TOL_MM, tol_rad: float = 1e-4) -> tuple[list[Segment], list[str]]:
    """Segments of a route and the problems (a pair that translates AND rotates, a duplicate waypoint)."""
    segs, problems = [], []
    for i in range(len(route) - 1):
        p, q = route[i], route[i + 1]
        d = math.hypot(q.x_mm - p.x_mm, q.y_mm - p.y_mm)
        th = abs(wrap_angle(q.theta_rad - p.theta_rad))
        if d >= tol_mm and th >= tol_rad:
            problems.append(f"segment {i}: translates {d:.1f} mm AND rotates {math.degrees(th):.2f} deg - the PLC "
                            "relative move does one or the other per command")
        elif d >= tol_mm:
            segs.append(Segment("translate", p, q))
        elif th >= tol_rad:
            segs.append(Segment("rotate", p, q))
        else:
            problems.append(f"segment {i}: duplicate waypoint {p.describe()}")
    return segs, problems


def _bbox(P: Sequence[Pt]) -> tuple[float, float, float, float]:
    xs, ys = [p[0] for p in P], [p[1] for p in P]
    return min(xs), min(ys), max(xs), max(ys)


def _bbox_dist(a: tuple, b: tuple) -> float:
    """Distance between two axis-aligned boxes - a lower bound of the distance of what they contain."""
    dx = max(0.0, b[0] - a[2], a[0] - b[2])
    dy = max(0.0, b[1] - a[3], a[1] - b[3])
    return math.hypot(dx, dy)


class _Checker:
    """validate_route's rules with bounding-box lower bounds (exact distances only where an obstacle is near) and
    memoised waypoint / segment results (the planner validates many routes that share them)."""

    def __init__(self, obstacles: Sequence[Obstacle], ares: AresShape, clearance_mm: float, min_mm: float,
                 min_deg: float):
        self.obst = list(obstacles)
        self.boxes = [_bbox(o.poly) for o in self.obst]
        self.ares, self.clr, self.min_mm, self.min_deg = ares, float(clearance_mm), float(min_mm), float(min_deg)
        self._wp: dict = {}
        self._seg: dict = {}

    @staticmethod
    def _key(w: Pose2D) -> tuple:
        return (round(w.x_mm, 6), round(w.y_mm, 6), round(wrap_angle(w.theta_rad), 9))

    def waypoint(self, w: Pose2D) -> tuple[list[float], list[tuple[str, float]]]:
        """(distance to every obstacle capped at the clearance, [(obstacle, distance) overlapping or < clearance])"""
        k = self._key(w)
        if k not in self._wp:
            fp = self.ares.footprint(w)
            bb = _bbox(fp)
            d = []
            for o, ob in zip(self.obst, self.boxes):
                if _bbox_dist(bb, ob) >= self.clr:
                    d.append(self.clr)
                elif overlaps(fp, o.poly):
                    d.append(-1.0)
                else:
                    d.append(min(self.clr, poly_dist(fp, o.poly)))
            self._wp[k] = (fp, d)
        return self._wp[k]

    def check(self, route: Sequence[Pose2D], name: str = "route") -> list[str]:
        p: list[str] = []
        n = len(route)
        if n < 2:
            return [f"{name}: needs at least 2 waypoints"]
        info = [self.waypoint(w) for w in route]
        for i, (w, (fp, d)) in enumerate(zip(route, info)):
            terminal = i in (0, n - 1)
            for o, di in zip(self.obst, d):
                if di < 0:
                    p.append(f"{name} waypoint {i} {w.describe()}: ARES footprint overlaps {o.name}")
                elif not terminal and di < self.clr - 1e-6:
                    p.append(f"{name} waypoint {i} {w.describe()}: {di:.0f} mm from {o.name} "
                             f"(< clearance {self.clr:g} mm)")
        for i in range(n - 1):
            p += [f"{name} {x}" for x in self.segment(i, route[i], route[i + 1], info[i], info[i + 1])]
        return p

    def segment(self, i: int, a: Pose2D, b: Pose2D, ia, ib) -> list[str]:
        k = (self._key(a), self._key(b))
        if k in self._seg:
            return [x.replace("segment ?", f"segment {i}") for x in self._seg[k]]
        out: list[str] = []
        d = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
        th = wrap_angle(b.theta_rad - a.theta_rad)
        if d >= TOL_MM and abs(th) >= 1e-4:
            out.append(f"segment ?: translates {d:.1f} mm AND rotates {math.degrees(abs(th)):.2f} deg - the PLC "
                       "relative move does one or the other per command")
        elif d < TOL_MM and abs(th) < 1e-4:
            out.append(f"segment ?: duplicate waypoint {a.describe()}")
        elif d >= TOL_MM:
            if d < self.min_mm:
                out.append(f"segment ?: translation {d:.2f} mm below the PLC minimum {self.min_mm:g} mm")
            swept = hull(ia[0] + ib[0])
            sb = _bbox(swept)
            for o, ob, da, db in zip(self.obst, self.boxes, ia[1], ib[1]):
                need = min(self.clr, max(da, 0.0), max(db, 0.0))
                if _bbox_dist(sb, ob) >= max(need, 1e-6):
                    continue
                ds = poly_dist(swept, o.poly)
                # an overlap of the swept area counts whenever both end footprints are clear of the obstacle (an end
                # overlap is reported at its waypoint) - also with clearance 0 (it was missed there before)
                if ds < need - 1e-6 or (da >= 0 and db >= 0 and overlaps(swept, o.poly)):
                    out.append(f"segment ? (translate {d:.0f} mm from {a.describe()}): passes {ds:.0f} mm from "
                               f"{o.name} (needs {need:.0f} mm)")
        else:
            if abs(math.degrees(th)) < self.min_deg:
                out.append(f"segment ?: rotation {math.degrees(th):.3f} deg below the PLC minimum {self.min_deg:g} deg")
            r = self.ares.radius
            cb = (a.x_mm - r, a.y_mm - r, a.x_mm + r, a.y_mm + r)
            for o, ob in zip(self.obst, self.boxes):
                if _bbox_dist(cb, ob) >= self.clr:
                    continue
                dc = circle_dist((a.x_mm, a.y_mm), r, o.poly)
                if dc < self.clr - 1e-6:
                    out.append(f"segment ? (rotate {math.degrees(th):+.1f} deg at ({a.x_mm:.0f}, {a.y_mm:.0f})): swept "
                               f"circle r {r:.0f} mm is {dc:.0f} mm from {o.name} (< clearance {self.clr:g} mm)")
        self._seg[k] = out
        return [x.replace("segment ?", f"segment {i}") for x in out]


def validate_route(route: Sequence[Pose2D], obstacles: Sequence[Obstacle], ares: AresShape, clearance_mm: float, *,
                   min_mm: float = 2.0, min_deg: float = 0.2, name: str = "route") -> list[str]:
    """Every problem of a route (empty = valid); rules in the module docstring. min_mm / min_deg: the PLC minimum
    move (mauer/ares/ads.py MIN_MOVE_MM / MIN_MOVE_DEG)."""
    return _Checker(obstacles, ares, clearance_mm, min_mm, min_deg).check(route, name)


def route_stats(route: Sequence[Pose2D]) -> dict:
    segs, _ = segments(route)
    return {"translations": sum(s.kind == "translate" for s in segs), "rotations": sum(s.kind == "rotate" for s in segs),
            "length_mm": sum(s.length_mm for s in segs if s.kind == "translate"),
            "rotation_deg": sum(abs(math.degrees(s.angle_rad)) for s in segs if s.kind == "rotate")}


class RouteError(ValueError):
    pass


def _dedupe(route: list[Pose2D]) -> list[Pose2D]:
    """Drop repeated waypoints and the middle one of two translations in the same direction (one PLC command)."""
    out = [route[0]]
    for w in route[1:]:
        if same_pose(out[-1], w):
            continue
        if len(out) >= 2:
            a, b = out[-2], out[-1]
            if (abs(wrap_angle(a.theta_rad - b.theta_rad)) < TOL_RAD and abs(wrap_angle(b.theta_rad - w.theta_rad))
                    < TOL_RAD):
                d1 = (b.x_mm - a.x_mm, b.y_mm - a.y_mm)
                d2 = (w.x_mm - b.x_mm, w.y_mm - b.y_mm)
                n1, n2 = math.hypot(*d1), math.hypot(*d2)
                if n1 > TOL_MM and n2 > TOL_MM and abs(d1[0] * d2[1] - d1[1] * d2[0]) < 1e-6 * n1 * n2 \
                        and d1[0] * d2[0] + d1[1] * d2[1] > 0:
                    out[-1] = w
                    continue
        out.append(w)
    return out


def _backed_off(p: Pose2D, b: float) -> Pose2D:
    return Pose2D(p.x_mm - b * math.cos(p.theta_rad), p.y_mm - b * math.sin(p.theta_rad), p.theta_rad)


def _min_dist(fp: Sequence[Pt], obstacles: Sequence[Obstacle]) -> float:
    return min((poly_dist(fp, o.poly) for o in obstacles), default=math.inf)


def _back_off_point(p: Pose2D, obstacles, ares: AresShape, clearance: float, backoff: float) -> Pose2D:
    """p itself when its footprint is already clear, else p backed off (body -x) by k * backoff, k = 1, 2, ..."""
    if _min_dist(ares.footprint(p), obstacles) >= clearance:
        return p
    for k in range(1, 21):
        q = _backed_off(p, k * backoff)
        if _min_dist(ares.footprint(q), obstacles) >= clearance:
            return q
    raise RouteError(f"no free back-off point behind {p.describe()}")


def _connections(a: Pose2D, b: Pose2D, corridors: Sequence[float] = ()) -> list[list[Pose2D]]:
    """Ways from a to b at a's heading: direct, along wall x then y, y then x, and - for every corridor offset c -
    out to a corridor parallel to wall x (y = a.y + c) or wall y (x = a.x + c), along it, then to b (three
    translations: around a corner of the L)."""
    th = a.theta_rad
    out = [[a, b]]
    for mid in (Pose2D(b.x_mm, a.y_mm, th), Pose2D(a.x_mm, b.y_mm, th)):
        out.append([a, mid, b])
    for c in corridors:
        yc, xc = a.y_mm + c, a.x_mm + c
        out.append([a, Pose2D(a.x_mm, yc, th), Pose2D(b.x_mm, yc, th), b])
        out.append([a, Pose2D(xc, a.y_mm, th), Pose2D(xc, b.y_mm, th), b])
    res = []
    for r in out:
        r = _dedupe(r)
        if r not in res:
            res.append(r)
    return res


def _length(route: Sequence[Pose2D]) -> float:
    return sum(math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) for a, b in zip(route, route[1:]))


@dataclass
class RoutePlan:
    waypoints: list[Pose2D]
    cost: float
    stats: dict = field(default_factory=dict)


def plan_route(start: Pose2D, goal: Pose2D, obstacles: Sequence[Obstacle], ares: AresShape, clearance_mm: float,
               backoff_mm: float, *, after_rotation_weight: float = 5.0, grid_mm: float = 100.0,
               search_mm: float = 2500.0, bounds: tuple[float, float, float, float] | None = None,
               corridor_mm: float = 100.0, n_corridors: int = 15, min_approach_mm: float = 0.0) -> RoutePlan:
    """Cheapest valid route start -> goal of the form in the module docstring (at most one rotation). RouteError if
    none is found. Rotation points: the back-off points, the goal's back-off ray and a grid_mm grid within search_mm
    of start/goal (or `bounds` = (x0, y0, x1, y1)); between the back-off points and the rotation point up to two
    axis-parallel translations, or three via a corridor offset by k * corridor_mm (|k| <= n_corridors) on one side
    of the rotation. min_approach_mm: the route ends with a translation (body +x) of at least this length - the
    goal's back-off point lies at least that far behind the goal even where the goal is clear (2026-10-06: a dock ->
    stop route of the L ended with a rotation on the stop; the sequencer stops arrival_standoff_mm before the goal
    along the last translation)."""
    S1 = _back_off_point(start, obstacles, ares, clearance_mm, backoff_mm)
    G1 = _back_off_point(goal, obstacles, ares, clearance_mm, backoff_mm)
    if math.hypot(G1.x_mm - goal.x_mm, G1.y_mm - goal.y_mm) < min_approach_mm - 1e-9:
        G1 = _backed_off(goal, min_approach_mm)
    head = [start] if same_pose(S1, start) else [start, S1]
    tail = [goal] if same_pose(G1, goal) else [G1, goal]
    w = float(after_rotation_weight)

    checker = _Checker(obstacles, ares, clearance_mm, 2.0, 0.2)

    def check(route: list[Pose2D]) -> bool:
        return not checker.check(route)

    turn = abs(wrap_angle(goal.theta_rad - start.theta_rad)) > 1e-4
    best: RoutePlan | None = None
    if not turn:
        G1s = Pose2D(G1.x_mm, G1.y_mm, start.theta_rad)
        for mid in _connections(S1, G1s):
            r = _dedupe(head + mid[1:] + tail[1:] if len(tail) == 2 else head + mid[1:])
            c = _length(r)
            if (best is None or c < best.cost) and check(r):
                best = RoutePlan(r, c)
        if best is None:
            raise RouteError(f"no route {start.describe()} -> {goal.describe()} without rotation")
        best.stats = route_stats(best.waypoints)
        return best
    # rotation candidates
    if bounds is None:
        xs = [start.x_mm, goal.x_mm]
        ys = [start.y_mm, goal.y_mm]
        bounds = (min(xs) - search_mm, min(ys) - search_mm, max(xs) + search_mm, max(ys) + search_mm)
    x0, y0, x1, y1 = bounds
    cands: list[tuple[float, float]] = [(S1.x_mm, S1.y_mm), (G1.x_mm, G1.y_mm)]
    for k in range(1, int(search_mm / grid_mm) + 1):
        for P in (G1, S1):
            q = _backed_off(P, k * grid_mm / 2.0)
            cands.append((q.x_mm, q.y_mm))
    nx, ny = int((x1 - x0) / grid_mm) + 1, int((y1 - y0) / grid_mm) + 1
    for i in range(nx):
        for j in range(ny):
            cands.append((x0 + i * grid_mm, y0 + j * grid_mm))
    free = []
    boxes = [_bbox(o.poly) for o in obstacles]
    r_need = ares.radius + clearance_mm
    for c in cands:
        cb = (c[0] - ares.radius, c[1] - ares.radius, c[0] + ares.radius, c[1] + ares.radius)
        if all(_bbox_dist(cb, ob) >= clearance_mm or point_dist(c, o.poly) >= r_need
               for o, ob in zip(obstacles, boxes)):
            lb = math.hypot(c[0] - S1.x_mm, c[1] - S1.y_mm) + w * math.hypot(c[0] - G1.x_mm, c[1] - G1.y_mm)
            free.append((lb, c))
    free.sort()
    fixed_after = w * _length(tail)
    fixed_before = _length(head)
    corr = [k * corridor_mm for k in range(-n_corridors, n_corridors + 1) if k]
    for lb, (cx, cy) in free:
        if best is not None and lb + fixed_after + fixed_before >= best.cost - 1e-9:
            break
        Ra, Rb = Pose2D(cx, cy, start.theta_rad), Pose2D(cx, cy, goal.theta_rad)
        G1b = Pose2D(G1.x_mm, G1.y_mm, goal.theta_rad)
        c1, c2 = _connections(S1, Ra), _connections(Rb, G1b)
        combos = ([(m1, m2) for m1 in c1 for m2 in c2] + [(m1, m2) for m1 in _connections(S1, Ra, corr)[len(c1):]
                                                          for m2 in c2]
                  + [(m1, m2) for m1 in c1 for m2 in _connections(Rb, G1b, corr)[len(c2):]])
        for m1, m2 in combos:
            c = fixed_before + _length(m1) + w * (_length(m2) + _length(tail))
            if best is not None and c >= best.cost - 1e-9:
                continue
            r = _dedupe(head + m1[1:] + m2 + tail[1:])
            if check(r):
                best = RoutePlan(r, c)
    if best is None:
        raise RouteError(f"no route {start.describe()} -> {goal.describe()}: no rotation point with a clear circle "
                         f"(r {ares.radius:.0f} + {clearance_mm:g} mm) reachable within the search area")
    best.stats = route_stats(best.waypoints)
    return best


# ── obstacles from the config ─────────────────────────────────────────────────
def station_table_extent(cfg: Mapping) -> tuple[float, float]:
    """(x_max, y_max) of the pick-up station table top in the station frame (origin = front-left corner):
    [pickup_station] table_size (PLACEHOLDER); without it 50 mm around the stone holders (slots_xy, half_slots_xy,
    stone length along the station x axis). Shared by robodk/build_station.py and the floor model."""
    p = cfg["pickup_station"]
    if "table_size" in p:
        return float(p["table_size"][0]), float(p["table_size"][1])
    b, h = cfg["brick"], cfg.get("half_brick", cfg["brick"])
    ext = [(x + b["length"] / 2, y + b["width"] / 2) for x, y in p.get("slots_xy", [])]
    ext += [(x + h["length"] / 2, y + h["width"] / 2) for x, y in p.get("half_slots_xy", [])]
    if not ext:
        raise KeyError("[pickup_station]: neither table_size nor slots_xy / half_slots_xy")
    return max(e[0] for e in ext) + 50.0, max(e[1] for e in ext) + 50.0


def station_table_poly(cfg: Mapping, T_wall_station) -> list[Pt]:
    """Pick-up station table top in the wall frame: station frame x 0 .. x_max, y 0 .. y_max
    (station_table_extent, PLACEHOLDER layout)."""
    x_max, y_max = station_table_extent(cfg)
    T = T_wall_station
    th = math.atan2(T[1][0], T[0][0])
    return transform(box_poly(0.0, 0.0, x_max, y_max), T[0][3], T[1][3], th)


@dataclass
class PlateSite:
    """A floor plate on the ARES side of a leg, its notched edge against the base blocks, centred on block k
    (k < 0 or k >= n0: on a spare block beyond a free leg end). u = 100 + 200 k (block centre), board centre at
    v = block_width/2 + plate_depth/2 in the leg frame ([plates], PLACEHOLDER block size)."""
    leg: str
    k: int
    u: float
    xyz: tuple[float, float, float]           # board origin in the LEG frame ([[targets]] xyz with leg = ...)
    rpy_deg: tuple[float, float, float]
    plate: list[Pt]                           # wall frame
    block: list[Pt] | None                    # spare block (wall frame), None on the leg's own blocks
    board: str = ""

    @property
    def spare(self) -> bool:
        return self.block is not None

    @property
    def label(self) -> str:
        return f"{self.leg}k{self.k}"


def plate_site(cfg: Mapping, leg: Any, k: int, board_size_mm: tuple[float, float]) -> PlateSite:
    """Plate on block k of `leg` (any object with name, n0, to_wall(u, v)); board_size_mm = BoardSpec.size_mm.
    Board origin (top-left corner, face up: rpy (180, 0, 0)) = (u - board_w/2, v_c + board_h/2, mdf_t + paper_t)."""
    pc = cfg["plates"]
    bl, bw = float(pc["block_length"]), float(pc["block_width"])
    L, D = float(pc["plate_length"]), float(pc["plate_depth"])
    u = bl / 2.0 + k * bl
    v0 = bw / 2.0
    vc = v0 + D / 2.0
    w_b, h_b = board_size_mm
    xyz = (u - w_b / 2.0, vc + h_b / 2.0, float(pc["mdf_t"]) + float(pc["paper_t"]))
    plate = [leg.to_wall(a, b) for a, b in ((u - L / 2, v0), (u + L / 2, v0), (u + L / 2, v0 + D), (u - L / 2, v0 + D))]
    block = None
    if k < 0 or k >= math.floor(leg.n0 + 1e-9):            # beyond the full stones of course 0 (x.5 legs: the half)
        block = [leg.to_wall(a, b) for a, b in ((u - bl / 2, -bw / 2), (u + bl / 2, -bw / 2), (u + bl / 2, bw / 2),
                                                (u - bl / 2, bw / 2))]
    return PlateSite(leg.name, k, u, xyz, (180.0, 0.0, 0.0), plate, block)


def site_from_target(cfg: Mapping, leg: Any, t: Mapping, board_size_mm: tuple[float, float]) -> PlateSite:
    """PlateSite of a [[targets]] wall entry with `leg`; ValueError if its xyz/rpy is not a plate-site pose."""
    pc = cfg["plates"]
    bl = float(pc["block_length"])
    u = float(t["xyz"][0]) + board_size_mm[0] / 2.0
    k = round((u - bl / 2.0) / bl)
    s = plate_site(cfg, leg, k, board_size_mm)
    if (max(abs(a - b) for a, b in zip(s.xyz, t["xyz"])) > 1e-6
            or max(abs(a - b) for a, b in zip(s.rpy_deg, t.get("rpy_deg", (0, 0, 0)))) > 1e-6):
        raise ValueError(f"target {t.get('name')}: xyz {list(t['xyz'])} / rpy {list(t.get('rpy_deg', []))} is not "
                         f"a plate on a block centre of leg {leg.name} (nearest: block {k}, xyz {list(s.xyz)})")
    s.board = str(t.get("name", ""))
    return s


def check_sites(sites: Sequence[PlateSite], leg_polys: Mapping[str, list[Pt]], stop_footprints: Sequence[tuple],
                table: list[Pt] | None = None) -> list[str]:
    """Plates and spare blocks must not overlap each other, any leg footprint (a plate touches its own leg's block
    faces only), an ARES footprint at a stop (stop_footprints = [(label, poly)]) or the station table."""
    p = []
    for s in sites:
        parts = [("plate", s.plate)] + ([("spare block", s.block)] if s.block else [])
        nm = s.board or s.label
        for what, poly in parts:
            for leg, lp in leg_polys.items():
                if overlaps(poly, lp):
                    p.append(f"{nm} {what} overlaps leg {leg}")
            for lab, fp in stop_footprints:
                if overlaps(poly, fp):
                    p.append(f"{nm} {what} overlaps ARES at {lab}")
            if table is not None and overlaps(poly, table):
                p.append(f"{nm} {what} overlaps the station table")
    for i, a in enumerate(sites):
        for b in sites[i + 1:]:
            pa = [a.plate] + ([a.block] if a.block else [])
            pb = [b.plate] + ([b.block] if b.block else [])
            if any(overlaps(x, y) for x in pa for y in pb):
                p.append(f"{a.board or a.label} and {b.board or b.label} overlap")
    return p


def obstacles(leg_polys: Mapping[str, list[Pt]], sites: Sequence[PlateSite], table: list[Pt] | None) -> list[Obstacle]:
    out = [Obstacle(f"leg {k}", "leg", list(v)) for k, v in leg_polys.items()]
    for s in sites:
        out.append(Obstacle(f"plate {s.board or s.label}", "plate", list(s.plate)))
        if s.block:
            out.append(Obstacle(f"spare block {s.board or s.label}", "block", list(s.block)))
    if table is not None:
        out.append(Obstacle("station table", "table", list(table)))
    return out


@dataclass(frozen=True)
class _LegFrame:
    """A leg as plate_site needs it (name, n0, to_wall) from a job's T_wall_leg."""
    name: str
    n0: float
    x: float
    y: float
    theta: float

    def to_wall(self, u: float, v: float) -> Pt:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return self.x + c * u - s * v, self.y + s * u + c * v


def job_obstacles(cfg: Mapping, legs: Sequence[Mapping], T_wall_station, board_sizes: Mapping[str, tuple]
                  ) -> list[Obstacle]:
    """The floor model of an L job without robodk/wallplan.py: leg footprints from the job's legs ({"name", "n0",
    "T_wall_leg"}) and [brick] length / head_joint / width, the plates of the [[targets]] wall boards with a `leg`
    (board_sizes = {name: BoardSpec.size_mm}), the station table at T_wall_station. Same obstacles as
    tools/make_job.py floor_model (used by the sequencer's resume check, preflight_real and mauer.simworld)."""
    b = cfg["brick"]
    pitch = float(b["length"]) + float(b["head_joint"])
    W = float(b["width"])
    frames: dict[str, _LegFrame] = {}
    polys: dict[str, list[Pt]] = {}
    for lg in legs:
        T = lg["T_wall_leg"]
        f = _LegFrame(str(lg["name"]), float(lg["n0"]), float(T[0][3]), float(T[1][3]),
                      math.atan2(T[1][0], T[0][0]))
        frames[f.name] = f
        Ln = f.n0 * pitch - float(b["head_joint"])
        polys[f.name] = [f.to_wall(u, v) for u, v in ((0.0, -W / 2), (Ln, -W / 2), (Ln, W / 2), (0.0, W / 2))]
    sites = [site_from_target(cfg, frames[str(t["leg"])], t, board_sizes[str(t["name"])])
             for t in cfg.get("targets", []) if t.get("parent") == "wall" and "leg" in t and str(t["leg"]) in frames]
    return obstacles(polys, sites, station_table_poly(cfg, T_wall_station))


def swept_translation(ares: AresShape, a: Pose2D, b: Pose2D) -> list[Pt]:
    """Area swept by a translation a -> b (convex hull of both footprints; exact for a pure translation)."""
    return hull(ares.footprint(a) + ares.footprint(b))


def swept_rotation(ares: AresShape, a: Pose2D, b: Pose2D, dtheta_rad: float | None = None,
                   step_rad: float = math.radians(1.0)) -> list[list[Pt]]:
    """Convex pieces covering a rotation a -> b on the spot (hulls of footprints step_rad apart; the drift of a
    rotation is the move a -> b of the centre, spread evenly). dtheta_rad: the turned angle incl. its direction
    (default: the wrapped heading difference)."""
    d = wrap_angle(b.theta_rad - a.theta_rad) if dtheta_rad is None else float(dtheta_rad)
    n = max(1, int(math.ceil(abs(d) / step_rad)))
    poses = [Pose2D(a.x_mm + (b.x_mm - a.x_mm) * i / n, a.y_mm + (b.y_mm - a.y_mm) * i / n, a.theta_rad + d * i / n)
             for i in range(n + 1)]
    return [hull(ares.footprint(p) + ares.footprint(q)) for p, q in zip(poses, poses[1:])]

