"""Floor layout of a wall of legs (L, C, ...): candidate evaluation (leg lengths, board plates, routes), top-view
drawing and plan report.

    py.exe tools/plan_layout.py --evaluate                   # the leg lengths of the config (one candidate)
    py.exe tools/plan_layout.py --evaluate --legs 9-12,5-8   # trade-off grid (here the L of 2026-10-05)
                                 # -> results/l_wall_candidates.json, prints the table and the [[wall.legs]] /
                                 #    [[targets]] to enter
    py.exe tools/plan_layout.py                              # from the CONFIG: results/l_wall_layout.png + l_wall_plan.md

Evaluation per candidate (n0 of every leg A, B, C, ...), all pure Python (no RoboDK):
- legs: A = wall frame, every further leg = wallplan.butt_corner of the one before (perpendicular at its far end, away
  from its ARES side: two legs make an L, three a C); plan = wallplan.plan_legs (rectangles with half stones, [wall]
  reach_margin_mm) -> stops and nominal ARES poses (tools/make_job.py stop_pose);
- plate sites (mauer.floor.plate_site): a plate on every base-block centre on the ARES side of a leg (u = 100 + 200 k)
  and on spare blocks beyond FREE leg ends (k < 0, k >= n0); a site is valid when its plate / spare block overlaps no
  leg, no ARES footprint at a stop and not the station table;
- look reach: tools/make_job.py find_look (fronto-parallel at [camera] working_dist, family IK, +-50 mm ARES margin,
  ARES at the arrival standoff) for every (stop, site of the STOP'S OWN LEG), clear of the stones built by the end of
  that stop (mauer.armcheck) - the same rule make_job uses;
- board selection: at most the 8 existing wall boards W0..W7, plates must not overlap each other; lexicographic:
  max. boards per stop (>= 2 required, counted up to 4) > fewer spare blocks > longer worst-stop baseline > more
  boards; exhaustive search with pruning;
- routes (mauer.floor.plan_route, [routes]): every move between stops (same leg: back off / along / approach), the
  leg change and the station trips from every stop and back, validated;
- ranking of the feasible candidates: fewer stops > fewer ARES moves between stops (incl. the leg-change routes) >
  more stones in course 0 within 14..18 (the L request 2026-10-05: "the wall can be a bit shorter") > shorter
  leg-change route.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TOOLS = Path(__file__).resolve().parent
for p in (REPO, TOOLS):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np  # noqa: E402

import make_job as mj  # noqa: E402
from mauer import armcheck  # noqa: E402
from mauer import config as mconfig  # noqa: E402
from mauer import floor  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer.vision.targets import board_specs  # noqa: E402

RESULTS = REPO / "results"
CANDIDATES_JSON = RESULTS / "l_wall_candidates.json"
LEG_NAMES = "ABCDEFGH"
COURSE0_RANGE = (14, 18)          # lead's target range of course-0 stones
MAX_BOARDS = 8                    # existing wall boards W0..W7 (Samuel: no new markers)
MIN_BOARDS = 2                    # per stop (heading from the baseline)
CAP_BOARDS = 4                    # boards per stop counted for robustness


def with_legs(cfg: dict, n0s) -> dict:
    """Copy of cfg with [[wall.legs]] = A (wall frame, n0s[0]) and every further leg the butt corner of the one before
    (B = n0s[1], C = n0s[2], ...): away from ARES's side, or towards it with [wall] ares_inside (ARES works inside the
    corners). A leg keeps the side / dist / runs_through of the config's leg of the same name (runs_through: the leg
    runs through its corner with the one before, wallplan.butt_corner through="next")."""
    wp = mj.load_wallplan()
    c = copy.deepcopy(cfg)
    inside = bool(c["wall"].get("ares_inside", False))
    old = {str(d["name"]): d for d in cfg["wall"].get("legs") or []}
    legs_ = [wp.Leg(LEG_NAMES[0], int(n0s[0]))]
    for name, n in zip(LEG_NAMES[1:], n0s[1:]):
        through = "next" if old.get(name, {}).get("runs_through") else "prev"
        legs_.append(wp.butt_corner(c, legs_[-1], int(n), name, towards_ares=inside, through=through))
    c["wall"]["legs"] = [{"name": lg.name, "n0": lg.n0, "xyz_in_wall": [round(lg.x, 6), round(lg.y, 6), 0.0],
                          "rpy_in_wall_deg": [0.0, 0.0, round(math.degrees(lg.theta), 9)],
                          **{k: old[lg.name][k] for k in ("side", "dist", "runs_through")
                             if k in old.get(lg.name, {})}}
                         for lg in legs_]
    return c


def parse_legs(spec: str) -> list[tuple[int, ...]]:
    """Candidates from "10,7,5" (one) or "9-12,5-8" (every combination of the per-leg ranges)."""
    import itertools
    ranges = []
    for part in spec.split(","):
        a, _, b = part.strip().partition("-")
        ranges.append(range(int(a), int(b or a) + 1))
    return list(itertools.product(*ranges))


def legs_label(r: dict) -> str:
    return ", ".join(f"{n} {k}" for n, k in zip(LEG_NAMES, r["n0"]))


REQUESTS = {
    "L": ["- Samuel 2026-10-05: a small L with the wall (it can be a bit shorter), 4 courses, keep the 8 existing wall "
          "boards (placed differently), half stones at the ends and maybe the corners."],
    "C": ["- Samuel 2026-10-06: a small C - \"one length is 2 m, then 1.5 across and 1 m back\" - and keep the existing "
          "boards (no new ones): A 10 stones (2.0 m), B 7 (1.4 m; 1.52 m over the outer faces of A and C - B = 6 would "
          "give 1.32 m and one stop less), C 5 (1.0 m), C parallel to A on the far side from ARES. Before: the L of "
          "2026-10-05 (A 12, B 6)."],
}


def shape_name(cfg: dict) -> str:
    """ "L wall", "C wall", ... from [wall] shape (fallback: the number of legs)."""
    shape = str(cfg.get("wall", {}).get("shape", ""))
    n = len(cfg.get("wall", {}).get("legs") or [])
    return f"{shape} wall" if shape and shape != "straight" else f"wall of {n} legs"


def all_sites(cfg: dict, legs_: list, size_mm: tuple, extra: int = 3) -> list:
    return [floor.plate_site(cfg, lg, k, size_mm) for lg in legs_ for k in range(-extra, lg.n0 + extra)]


def select_boards(sites: list, reach: list[set[int]], conflicts: set, centres: list, max_boards: int = MAX_BOARDS,
                  min_per_stop: int = MIN_BOARDS, cap: int = CAP_BOARDS) -> tuple[list[int], dict] | None:
    """Indices of the chosen sites (see the module docstring) or None if no set gives >= min_per_stop per stop."""
    n_st = len(reach)
    useful = sorted({i for r in reach for i in r}, key=lambda i: (sites[i].leg, sites[i].k))
    for t in range(cap, min_per_stop - 1, -1):
        best: dict = {"score": None, "set": None}
        rem = [[sum(1 for i in useful[j:] if i in reach[s]) for j in range(len(useful) + 1)] for s in range(n_st)]

        def score(chosen: list[int]) -> tuple:
            spares = sum(sites[i].spare for i in chosen)
            base = []
            for s in range(n_st):
                pts = [centres[i] for i in chosen if i in reach[s]]
                base.append(max((float(np.linalg.norm(a - b)) for x, a in enumerate(pts) for b in pts[x + 1:]),
                                default=0.0))
            return (spares, -round(min(base), 0), -len(chosen))

        def dfs(j: int, chosen: list[int], counts: list[int], spares: int) -> None:
            if best["score"] is not None and spares > best["score"][0]:
                return
            if any(counts[s] + rem[s][j] < t for s in range(n_st)):
                return
            if all(c >= t for c in counts):
                sc = score(chosen)
                if best["score"] is None or sc < best["score"]:
                    best["score"], best["set"] = sc, list(chosen)
            if j == len(useful) or len(chosen) == max_boards:
                return
            i = useful[j]
            if not any((min(i, c), max(i, c)) in conflicts for c in chosen):
                dfs(j + 1, chosen + [i], [counts[s] + (i in reach[s]) for s in range(n_st)],
                    spares + sites[i].spare)
            dfs(j + 1, chosen, counts, spares)

        dfs(0, [], [0] * n_st, 0)
        if best["set"] is not None:
            return best["set"], {"per_stop_target": t, "score": best["score"]}
    return None


def evaluate(cfg0: dict, n0s, ctx: "mj._Ctx", route_all: bool = True) -> dict:
    """One candidate (module docstring). Never raises for an infeasible candidate: row["feasible"] = False + reason.
    Every leg on the reach table of its side / distance (make_job.leg_side / leg_dist), ARES stops within
    make_job.stop_limits (between the other legs when ARES works inside the corners)."""
    wp = mj.load_wallplan()
    cfg = with_legs(cfg0, n0s)
    legs_ = wp.legs(cfg)
    row: dict = {"n0": [int(n) for n in n0s], "course0": int(sum(n0s)), "feasible": False, "why": ""}
    margin = float(cfg["wall"].get("reach_margin_mm", 0.0))
    try:
        tables = {lg.name: mj.load_reach_table(cfg, mj.leg_dist(cfg, lg, ctx.dist), side=mj.leg_side(cfg, lg))[0]
                  for lg in legs_}
        limits = mj.stop_limits(cfg, legs_, ctx.dist)
        stones, plans = wp.plan_legs(cfg, legs_, tables[legs_[0].name], True, margin, tables=tables, a_limits=limits)
    except (ValueError, RuntimeError) as e:
        row["why"] = f"plan: {e}"
        return row
    T_legs = mconfig.leg_frames(cfg)
    by_name = {lg.name: lg for lg in legs_}
    stops = []
    built: list = []
    for lg in legs_:
        for a, batch in plans[lg.name]:
            built = built + list(batch)
            stops.append({"leg": lg.name, "a": float(a), "n": len(batch), "built": built,
                          "pose": mj.stop_pose(cfg, mj.leg_dist(cfg, lg, ctx.dist), a, T_legs[lg.name],
                                               mj.leg_side(cfg, lg))})
    row.update(stones=len(stones), half=sum(s.kind == "half" for s in stones),
               stops=[(s["leg"], s["a"], s["n"]) for s in stops], n_stops=len(stops))
    spec = board_specs(cfg)["W0"]
    sites = all_sites(cfg, legs_, spec.size_mm)
    leg_polys = {lg.name: wp.leg_footprint(cfg, lg) for lg in legs_}
    ares = floor.AresShape.from_config(cfg)
    fps = [(f"stop {i}", ares.footprint(s["pose"])) for i, s in enumerate(stops)]
    table_poly = floor.station_table_poly(cfg, mj.T_wall_station(cfg))
    sites = [s for s in sites if not floor.check_sites([s], leg_polys, fps, table_poly)]
    conflicts = {(i, j) for i in range(len(sites)) for j in range(i + 1, len(sites))
                 if floor.check_sites([sites[i], sites[j]], {}, [])}
    centres, T_wb = [], []
    for s in sites:
        T = T_legs[s.leg] @ g.pose_xyz_rpy(s.xyz, s.rpy_deg)
        T_wb.append(T)
        centres.append(g.apply(T, [[*spec.centre_mm, 0.0]])[0])
    reach: list[set[int]] = []
    for s in stops:
        T_base_wall = ctx.T_base_ares @ g.inv(s["pose"].T)
        R_pref = mj.preferred_flange_R((T_base_wall @ T_legs[s["leg"]])[:3, :3], ctx.T_flange_tcp)
        arm = armcheck.Checkers(armcheck.ArmChecker(cfg, armcheck.stone_boxes(cfg, s["built"], by_name)),
                                ctx.ares_arm())
        offs = mj.standoff_offset(s["pose"], mj.arrival_standoff(cfg))
        ok = set()
        for i, T in enumerate(T_wb):
            if sites[i].leg != s["leg"]:                       # own leg only (make_job.wall_look_candidates)
                continue
            if float(np.linalg.norm(centres[i][:2] - np.array([s["pose"].x_mm, s["pose"].y_mm]))) > 2000.0:
                continue
            if mj.find_look(T_base_wall @ T, spec, ctx.T_flange_cam, ctx.work, R_pref, ctx.q_park,
                            g.inv(T_base_wall), ctx.look_margin, arm=arm, offsets=offs) is not None:
                ok.add(i)
        reach.append(ok)
    row["reachable_sites_per_stop"] = [len(r) for r in reach]
    sel = select_boards(sites, reach, conflicts, centres)
    if sel is None:
        row["why"] = "fewer than 2 reachable plate sites at some stop"
        return row
    chosen, info = sel
    chosen = sorted(chosen, key=lambda i: (sites[i].leg, sites[i].k))
    row["boards"] = [{"leg": sites[i].leg, "k": sites[i].k, "u": sites[i].u, "spare": sites[i].spare,
                      "xyz": list(sites[i].xyz)} for i in chosen]
    row["boards_per_stop"] = [len([i for i in chosen if i in r]) for r in reach]
    row["spares"] = sum(sites[i].spare for i in chosen)
    row["min_baseline_mm"] = -info["score"][1]
    # routes
    obst = floor.obstacles(leg_polys, [sites[i] for i in chosen], table_poly)
    rp = mj.route_params(cfg)
    kw = dict(after_rotation_weight=rp["after_rotation_weight"], grid_mm=rp["grid_mm"], search_mm=rp["search_mm"])
    dock = mj.dock_in_wall(cfg)
    routes, problems = {}, []
    wall_moves = 0
    for i in range(1, len(stops)):
        a, b = stops[i - 1], stops[i]
        name = f"leg_change_{a['leg']}{b['leg']}" if a["leg"] != b["leg"] else f"stop_move_{i}"
        try:
            r = floor.plan_route(a["pose"], b["pose"], obst, ares, rp["clearance_mm"], rp["backoff_mm"], **kw)
            routes[name] = r.stats | {"waypoints": [p.to_dict() for p in r.waypoints]}
            wall_moves += r.stats["translations"] + r.stats["rotations"]
        except floor.RouteError as e:
            problems.append(f"{name.replace('_', ' ')}: {e}")
    trip_len = []
    if route_all:
        for i, s in enumerate(stops):
            for name, (a, b) in ((f"stop {i} -> station", (s["pose"], dock)), (f"station -> stop {i}", (dock, s["pose"]))):
                try:
                    r = floor.plan_route(a, b, obst, ares, rp["clearance_mm"], rp["backoff_mm"], **kw)
                    trip_len.append(r.stats["length_mm"])
                except floor.RouteError as e:
                    problems.append(f"{name}: {e}")
    row.update(wall_moves=wall_moves, routes=routes, route_problems=problems,
               leg_change_mm=(sum(v["length_mm"] for k, v in routes.items() if k.startswith("leg_change"))
                              if any(k.startswith("leg_change") for k in routes) else None),
               station_trip_mm=(max(trip_len) if trip_len else None))
    if problems:
        row["why"] = "; ".join(problems[:3])
        return row
    row["feasible"] = all(c >= MIN_BOARDS for c in row["boards_per_stop"])
    return row


def rank_key(r: dict) -> tuple:
    lo, hi = COURSE0_RANGE
    in_range = lo <= r["course0"] <= hi
    return (not r["feasible"], r.get("n_stops", 99), r.get("wall_moves", 99), not in_range, -r["course0"],
            -min(r.get("boards_per_stop") or [0]), r.get("spares", 99), r.get("leg_change_mm") or 1e9)


def evaluate_all(cfg: dict, candidates, quiet: bool = False) -> list[dict]:
    """Evaluate every candidate (tuples of n0 per leg), ranked (rank_key)."""
    dist = float(cfg["wall"]["dist_nominal"])
    ctx = mj._Ctx(cfg, dist, 2, mj.LOOK_MARGIN_MM)
    rows = []
    for n0s in candidates:
        r = evaluate(cfg, n0s, ctx)
        rows.append(r)
        if not quiet:
            print(f"  {legs_label(r)}: {'ok ' if r['feasible'] else 'NO '} stops {r.get('n_stops', '-')}, "
                  f"boards/stop {r.get('boards_per_stop', '-')}, spares {r.get('spares', '-')}, "
                  f"wall moves {r.get('wall_moves', '-')} {r['why'][:100]}", flush=True)
    rows.sort(key=rank_key)
    return rows


def table_md(rows: list[dict], chosen: tuple[int, ...] | None = None) -> list[str]:
    out = ["| legs (n0) | course 0 | stones (half) | stops (leg: a mm / stones) | wall moves | boards / stop | "
           "spare blocks | worst baseline mm | leg changes mm | longest station trip mm | feasible |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(rows, key=lambda r: r["n0"]):
        mark = " **chosen**" if chosen is not None and tuple(chosen) == tuple(r["n0"]) else ""
        st = ", ".join(f"{leg}: {a:.0f}/{n}" for leg, a, n in r.get("stops", []))
        lc = r.get("leg_change_mm")
        trip = r.get("station_trip_mm")
        out.append(f"| {legs_label(r)} | {r['course0']} | {r.get('stones', '-')} ({r.get('half', '-')}) | "
                   f"{st or '-'} | {r.get('wall_moves', '-')} | {r.get('boards_per_stop', '-')} | "
                   f"{r.get('spares', '-')} | {r.get('min_baseline_mm', '-')} | "
                   f"{'-' if lc is None else f'{lc:.0f}'} | {'-' if trip is None else f'{trip:.0f}'} | "
                   f"{'yes' if r['feasible'] else 'no: ' + r['why'][:80]}{mark} |")
    return out


def config_snippet(cfg: dict, best: dict) -> str:
    """[[wall.legs]] and the wall [[targets]] (existing names / ids, new places) of a candidate, ready to paste."""
    c = with_legs(cfg, best["n0"])
    lines = []
    for lg in c["wall"]["legs"]:
        lines += ["[[wall.legs]]", f'name = "{lg["name"]}"', f"n0 = {lg['n0']}",
                  f"xyz_in_wall = {lg['xyz_in_wall']}", f"rpy_in_wall_deg = {lg['rpy_in_wall_deg']}"]
    walls = [t for t in cfg["targets"] if t["parent"] == "wall"]
    for t, b in zip(walls, best["boards"]):
        lines += ["[[targets]]", f'name = "{t["name"]}"', 'parent = "wall"', f'leg = "{b["leg"]}"',
                  f"first_id = {t['first_id']}", f"xyz = [{b['xyz'][0]:.1f}, {b['xyz'][1]:.1f}, {b['xyz'][2]:.1f}]",
                  "rpy_deg = [180.0, 0.0, 0.0]"]
    return "\n".join(lines)


# ── drawing ───────────────────────────────────────────────────────────────────
STOP_COLOURS = [(60, 160, 40), (0, 140, 255), (180, 60, 160), (40, 40, 200), (140, 120, 0), (90, 90, 90)]   # BGR


class Canvas:
    """Top view to scale: x right, y up (wall frame), s px/mm, anti-aliased OpenCV drawing."""

    def __init__(self, x0: float, y0: float, x1: float, y1: float, s: float, pad_top: int = 0, pad_bottom: int = 0):
        import cv2
        self.cv2 = cv2
        self.x0, self.y1, self.s = x0, y1, s
        self.pad_top = pad_top
        self.W = int((x1 - x0) * s)
        self.H = int((y1 - y0) * s) + pad_top + pad_bottom
        self.img = np.full((self.H, self.W, 3), 255, np.uint8)

    def P(self, x: float, y: float) -> tuple[int, int]:
        return int(round((x - self.x0) * self.s)), int(round((self.y1 - y) * self.s)) + self.pad_top

    def poly(self, pts, colour, width: int = 1, fill=None, alpha: float = 1.0) -> None:
        q = np.array([self.P(x, y) for x, y in pts], np.int32)
        if fill is not None:
            if alpha >= 1.0:
                self.cv2.fillPoly(self.img, [q], fill, self.cv2.LINE_AA)
            else:
                over = self.img.copy()
                self.cv2.fillPoly(over, [q], fill, self.cv2.LINE_AA)
                self.img[:] = self.cv2.addWeighted(over, alpha, self.img, 1.0 - alpha, 0)
        if width > 0:
            self.cv2.polylines(self.img, [q], True, colour, width, self.cv2.LINE_AA)

    def line(self, a, b, colour, width: int = 1, dashed: float = 0.0) -> None:
        if dashed <= 0:
            self.cv2.line(self.img, self.P(*a), self.P(*b), colour, width, self.cv2.LINE_AA)
            return
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(d / dashed))
        for i in range(0, n, 2):
            t0, t1 = i / n, min(1.0, (i + 1) / n)
            self.cv2.line(self.img, self.P(a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0),
                          self.P(a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1), colour, width, self.cv2.LINE_AA)

    def circle(self, c, r: float, colour, width: int = 1, dashed: bool = False) -> None:
        if not dashed:
            self.cv2.circle(self.img, self.P(*c), int(r * self.s), colour, width, self.cv2.LINE_AA)
            return
        n = 72
        for i in range(0, n, 2):
            a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            self.line((c[0] + r * math.cos(a0), c[1] + r * math.sin(a0)),
                      (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1)), colour, width)

    def text(self, xy, s: str, colour=(30, 30, 30), scale: float = 0.45, thick: int = 1, px: bool = False,
             centre: bool = False, align: str = "") -> None:
        p = (int(xy[0]), int(xy[1])) if px else self.P(*xy)
        (w, h), _ = self.cv2.getTextSize(s, self.cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
        if centre or align == "centre":
            p = (p[0] - w // 2, p[1] + h // 2)
        elif align == "right":
            p = (p[0] - w, p[1] + h // 2)
        elif align == "left":
            p = (p[0], p[1] + h // 2)
        self.cv2.putText(self.img, s, p, self.cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thick, self.cv2.LINE_AA)

    def arrow(self, a, b, colour, width: int = 2) -> None:
        self.cv2.arrowedLine(self.img, self.P(*a), self.P(*b), colour, width, self.cv2.LINE_AA, tipLength=0.25)


def l_geometry(cfg: dict) -> dict:
    """Everything the drawing and the report need, from the CONFIG (tools/make_job.py build_l)."""
    wp = mj.load_wallplan()
    job = mj.build_l(cfg)
    legs_ = wp.legs(cfg)
    sites = mj.target_sites(cfg, legs_)
    obst, problems = mj.floor_model(cfg, legs_, sites, [(f"stop {s.index}", s.ares) for s in job.stops])
    ctx = mj._Ctx(cfg, float(cfg["wall"]["dist_nominal"]), 2, mj.LOOK_MARGIN_MM)
    T_legs = mconfig.leg_frames(cfg)
    by_name = {lg.name: lg for lg in legs_}
    stones = {s_.key: s_ for s_ in wp.layout_legs(cfg, legs_, bool(cfg["wall"].get("half_stones", True)))}
    reach, built = [], []
    for st in job.stops:
        built += [stones[t.key] for t in st.stones]
        cands, _ = mj.wall_look_candidates(ctx, st.ares, T_legs[st.leg], st.leg, built, by_name)
        reach.append([p.name for p in ctx.pl if p.name in cands])
    return {"job": job, "legs": legs_, "sites": sites, "obstacles": obst, "problems": problems, "reach": reach,
            "ares": floor.AresShape.from_config(cfg), "table": floor.station_table_poly(cfg, mj.T_wall_station(cfg))}


def draw_layout(cfg: dict, geo: dict, path: Path, s: float = 0.3) -> None:
    """results/l_wall_layout.png: top view to scale + an elevation of every leg (stones by stop, half stones)."""
    wp = mj.load_wallplan()
    job, legs_, sites, ares = geo["job"], geo["legs"], geo["sites"], geo["ares"]
    by = {lg.name: lg for lg in legs_}
    pts = [p for o in geo["obstacles"] for p in o.poly]
    for st in job.stops:
        pts += ares.footprint(st.ares)
        for r in (st.route, st.route_to_station, st.route_from_station):
            for w in r:
                pts += [(w.x_mm - ares.radius, w.y_mm - ares.radius), (w.x_mm + ares.radius, w.y_mm + ares.radius)]
    dock = job.station.dock_in_wall
    pts += ares.footprint(dock)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x0, x1 = min(xs) - 250, max(xs) + 250
    y0, y1 = min(ys) - 250, max(ys) + 250
    H_el = int((cfg["wall"]["base_z"] + cfg["wall"]["courses"] * cfg["brick"]["height"]) * s * 1.6) + 90
    c = Canvas(x0, y0, x1, y1, s, pad_top=70, pad_bottom=len(legs_) * (H_el + 40) + 80)
    cv2 = c.cv2
    # grid
    for gx in range(int(math.ceil(x0 / 500) * 500), int(x1) + 1, 500):
        c.line((gx, y0), (gx, y1), (238, 238, 238), 1)
        c.text((gx + 10, y1 - 80), f"{gx}", (170, 170, 170), 0.35)
    for gy in range(int(math.ceil(y0 / 500) * 500), int(y1) + 1, 500):
        c.line((x0, gy), (x1, gy), (238, 238, 238), 1)
        c.text((x0 + 20, gy + 15), f"{gy}", (170, 170, 170), 0.35)
    # scale bar
    c.line((x1 - 900, y0 + 120), (x1 - 400, y0 + 120), (20, 20, 20), 3)
    c.text((x1 - 650, y0 + 190), "500 mm", (20, 20, 20), 0.4, centre=True)
    # station
    c.poly(geo["table"], (40, 90, 140), 2, fill=(200, 225, 240))
    T_ws = mj.T_wall_station(cfg)
    for sl in job.station.slots:
        p = g.apply(T_ws, [sl.T_station_tcp[:3, 3]])[0]
        col = (0, 120, 0) if sl.ik_ok else (150, 150, 150)
        r = 50 if sl.kind == "full" else 30
        c.circle((p[0], p[1]), r, col, 2 if sl.ik_ok else 1)
    tc = np.mean(np.array(geo["table"]), axis=0)
    ty = max(p[1] for p in geo["table"])
    c.text((tc[0], ty + 140), "pick-up station table (PLACEHOLDER)", (40, 90, 140), 0.45, centre=True)
    c.text((tc[0], ty + 70), "slots: large o full, small o half (green = reachable from the dock)", (40, 90, 140),
           0.35, centre=True)
    c.poly(ares.footprint(dock), (40, 90, 140), 1, fill=(220, 235, 245), alpha=0.5)
    c.text((dock.x_mm, dock.y_mm), "dock", (40, 90, 140), 0.45, centre=True)
    # legs: course-0 stones = base blocks, joints
    for lg in legs_:
        for stn in wp.layout_leg(cfg, lg, True):
            if stn.course != 0:
                continue
            fp = wp.stone_footprint(cfg, stn, lg)
            c.poly(fp, (60, 60, 60), 1, fill=(205, 205, 205))
        L = wp.leg_length(cfg, lg.n0)
        c.poly(wp.leg_footprint(cfg, lg), (40, 40, 40), 2)
        mx, my = lg.to_wall(L / 2, -cfg["brick"]["width"] / 2 - 180)      # inside of the wall
        vert = abs(math.sin(lg.theta)) > 0.5
        ins = lg.to_wall(L / 2, -1.0)[0] - lg.to_wall(L / 2, 0.0)[0]
        c.text((mx, my), f"leg {lg.name}: {lg.n0} x 200 = {L:.0f} mm", (20, 20, 20), 0.45, 1,
               align=("right" if ins < 0 else "left") if vert else "centre")
        for u in range(0, int(L) + 1, 400):
            px, py = lg.to_wall(u, -95)
            c.text((px, py), f"{u}", (90, 90, 90), 0.3, centre=True)
    # plates
    for st_ in sites:
        c.poly(st_.plate, (160, 60, 0), 2, fill=(245, 220, 200))
        if st_.block:
            c.poly(st_.block, (0, 0, 160), 2, fill=(200, 200, 255))
        cx = np.mean([p[0] for p in st_.plate])
        cy = np.mean([p[1] for p in st_.plate])
        c.text((cx, cy + 15), st_.board, (130, 40, 0), 0.42, 1, centre=True)
        c.text((cx, cy - 35), f"{st_.leg} u{st_.u:.0f}", (130, 40, 0), 0.3, centre=True)
    c.arrow((0, 0), (400, 0), (0, 0, 200), 2)
    c.arrow((0, 0), (0, 400), (0, 150, 0), 2)
    c.text((-60, -110), "wall frame x", (0, 0, 200), 0.4)
    c.text((-60, 450), "y", (0, 150, 0), 0.4)
    # station trips (thin), ARES at stops, routes
    for st in job.stops:
        for r in (st.route_to_station, st.route_from_station):
            for a, b in zip(r, r[1:]):
                c.line((a.x_mm, a.y_mm), (b.x_mm, b.y_mm), (170, 170, 170), 1, dashed=40)
    T_ab = mconfig.T_ares_base(cfg)
    for st in job.stops:
        col = STOP_COLOURS[st.index % len(STOP_COLOURS)]
        c.poly(ares.footprint(st.ares), col, 2, fill=col, alpha=0.12)
        hx, hy = math.cos(st.ares.theta_rad), math.sin(st.ares.theta_rad)
        c.arrow((st.ares.x_mm, st.ares.y_mm), (st.ares.x_mm + 300 * hx, st.ares.y_mm + 300 * hy), col, 2)
        ub = (st.ares.T @ T_ab)[:2, 3]
        c.circle((ub[0], ub[1]), 40, col, 2)
        if abs(hy) > 0.5:          # heading across the drawing's x: labels behind the centre
            l1 = (st.ares.x_mm - 200 * hx, st.ares.y_mm - 200 * hy)
            l2 = (st.ares.x_mm - 330 * hx, st.ares.y_mm - 330 * hy)
        else:                      # heading along x: labels below the centre line
            l1, l2 = (st.ares.x_mm + 80 * hx, st.ares.y_mm - 170), (st.ares.x_mm + 80 * hx, st.ares.y_mm - 240)
        c.text(l1, f"stop {st.index}: {st.leg} a={st.a_mm:.0f}", col, 0.45, 1, centre=True)
        c.text(l2, f"{len(st.stones)} stones, looks {','.join(b for lk in st.looks for b in lk.boards)}", col, 0.35,
               centre=True)
        if st.route:
            for a, b in zip(st.route, st.route[1:]):
                c.line((a.x_mm, a.y_mm), (b.x_mm, b.y_mm), (0, 0, 220), 2)
                if math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm) < 0.5:
                    c.circle((a.x_mm, a.y_mm), ares.radius, (0, 0, 220), 1, dashed=True)
                    c.circle((a.x_mm, a.y_mm), ares.radius + mj.route_params(cfg)["clearance_mm"], (0, 0, 120), 1,
                             dashed=True)
                    c.text((a.x_mm + 60, a.y_mm + 60), f"rotate {math.degrees(wrap(b.theta_rad - a.theta_rad)):+.0f} deg",
                           (0, 0, 200), 0.45)
            for w in st.route:
                c.circle((w.x_mm, w.y_mm), 18, (0, 0, 220), 2)
            if job.stops[st.index - 1].leg != st.leg:
                mid = st.route[len(st.route) // 2]
                c.text((mid.x_mm + 60, mid.y_mm - 120), "leg change", (0, 0, 200), 0.5, 1)
    # title / legend
    c.text((15, 28), f"{shape_name(cfg)} - top view to scale, wall frame [mm], 1 px = {1 / s:.2f} mm, grid 500 mm "
                     "(PLACEHOLDER: station, block size; ASSUMPTION: leg lengths, corner, route clearances)",
           (20, 20, 20), 0.55, 1, px=True)
    c.text((15, 55), "grey: course-0 stones = base blocks | orange: board plates (name, leg u) | coloured: ARES "
                     "footprint per stop (arrow = heading, circle = UR base) | red: routes between stops (back off / "
                     "along / approach; leg change with its rotation circle r 635 + 50 mm clearance) | dashed grey: "
                     "station trips", (60, 60, 60), 0.4, 1, px=True)
    # elevations
    y_el = int((y1 - y0) * s) + c.pad_top + 40
    stop_of = {t.key: st.index for st in job.stops for t in st.stones}
    for lg in legs_:
        L = wp.leg_length(cfg, lg.n0)
        ox = 40
        base = y_el + H_el - 20
        sc = s * 1.6
        c.text((ox, y_el - 5), f"leg {lg.name} elevation, seen from the inside of the wall (u from the leg start to the "
                               "right; colour = stop as above, hatched = half stone; grey bar = base plate)",
               (20, 20, 20), 0.45, 1, px=True)
        for stn in wp.layout_leg(cfg, lg, True):
            u0 = (stn.u - stn.length / 2) * sc + ox
            u1 = (stn.u + stn.length / 2) * sc + ox
            z0 = base - (stn.z_top - cfg["brick"]["height"]) * sc
            z1 = base - stn.z_top * sc
            col = STOP_COLOURS[stop_of.get(stn.key, 0) % len(STOP_COLOURS)]
            light = tuple(int(255 - (255 - v) * 0.35) for v in col)
            cv2.rectangle(c.img, (int(u0), int(z1)), (int(u1), int(z0)), light, -1)
            if stn.kind == "half":
                for k in range(int(u0) - 40, int(u1), 8):
                    cv2.line(c.img, (max(k, int(u0)), int(z0) - max(0, int(u0) - k)),
                             (min(k + int(z0 - z1), int(u1)), int(z0) - min(int(z0 - z1), int(u1) - k)), col, 1,
                             cv2.LINE_AA)
            cv2.rectangle(c.img, (int(u0), int(z1)), (int(u1), int(z0)), (60, 60, 60), 1)
        cv2.rectangle(c.img, (ox, base), (int(ox + L * sc), int(base + cfg["wall"]["base_z"] * sc)), (120, 120, 120), -1)
        for u in range(0, int(L) + 1, 200):
            cv2.line(c.img, (int(ox + u * sc), base + 2), (int(ox + u * sc), base + 8), (60, 60, 60), 1)
            if u % 400 == 0:
                c.text((int(ox + u * sc) - 10, base + 25), f"{u}", (90, 90, 90), 0.35, px=True)
        y_el += H_el + 40
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), c.img)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ── report ────────────────────────────────────────────────────────────────────
def write_report(cfg: dict, geo: dict, path: Path, rows: list[dict] | None) -> None:
    from mauer.floor import route_stats, segments
    from mauer.reference import relative_move
    wp = mj.load_wallplan()
    job, legs_ = geo["job"], geo["legs"]
    m = job.meta
    shape = str(cfg["wall"].get("shape", ""))
    lines = [f"# {shape_name(cfg)} plan (simulation)", "",
             "Generated by `py.exe tools/plan_layout.py` from `config/station.toml` (+ `results/reach_table.json`, "
             f"key `{m['reach_table']['key']}`); drawing `results/l_wall_layout.png`. Pure Python: kinematic checks "
             "and a coarse arm-vs-built-wall check of the looks (mauer.armcheck), no occlusion; the RoboDK collision "
             "run of the job is `results/l_wall_sim.md`. Values marked PLACEHOLDER / ASSUMPTION are listed at the "
             "end - every result here depends on them.", "",
             "## Request and design"] + REQUESTS.get(shape, []) + [
             "- The 53 x 100.5 mm pin pattern is not square: a stone turned by 90 deg never engages the sockets below, "
             "so the legs cannot interlock - every corner is a vertical BUTT joint: the earlier leg runs through (its "
             "end flush with the body of the next leg's outer face; the next leg's ribs stand 1.69 mm proud), the next "
             "leg starts at the earlier leg's inside face - beyond its ribs ([brick] rib_mm "
             f"{cfg['brick'].get('rib_mm', 0):g} mm, CAD) and a corner gap ([wall] corner_gap_mm "
             f"{cfg['wall'].get('corner_gap_mm', 0):g} mm, ASSUMPTION): set the next leg's base blocks that far off. "
             "Half stones at both ends of every odd course make each leg a rectangle (no notch at a corner, no stone "
             "supported across legs).",
             "- Wall frame = leg A frame; ARES works " + ("INSIDE the corners (legs turn towards its side; it "
             "stops only where it keeps the route clearance from the other legs and their plates)"
             if cfg["wall"].get("ares_inside") else "from the OUTSIDE of every leg") + "; side and distance of "
             "every leg below; the legs are built one after the other (" + ", ".join(lg.name for lg in legs_) + ").",
             "",
             "## Geometry", "", "| leg | stones in course 0 | length mm | frame in the wall frame (x, y mm, heading deg) "
             "| side of ARES | ARES centre -> leg mm | stop window a mm |", "|---|---|---|---|---|---|---|"]
    lim = {d["name"]: d.get("a_limits_mm") for d in m.get("legs", [])}
    for lg in legs_:
        a_lim = lim.get(lg.name)
        win = "-" if not a_lim else f"{a_lim[0]:.0f} .. {a_lim[1]:.0f}"
        lines.append(f"| {lg.name} | {lg.n0} | {wp.leg_length(cfg, lg.n0):.0f} | ({lg.x:.0f}, {lg.y:.0f}), "
                     f"{math.degrees(lg.theta):.0f} | {mj.leg_side(cfg, lg)} | {mj.leg_dist(cfg, lg):.0f} | {win} |")
    stones = [t for st in job.stops for t in st.stones]
    lines += ["", "Stones per leg and course (full + half; odd courses: half + (n-1) full + half):", "",
              "| leg | " + " | ".join(f"course {k}" for k in range(cfg["wall"]["courses"])) + " | total |",
              "|---|" + "---|" * (cfg["wall"]["courses"] + 1)]
    for lg in legs_:
        cells = []
        for k in range(cfg["wall"]["courses"]):
            f = sum(1 for t in stones if t.leg == lg.name and t.course == k and t.kind == "full")
            h = sum(1 for t in stones if t.leg == lg.name and t.course == k and t.kind == "half")
            cells.append(f"{f} + {h}" if h else f"{f}")
        tot = sum(1 for t in stones if t.leg == lg.name)
        lines.append(f"| {lg.name} | " + " | ".join(cells) + f" | {tot} |")
    n_half = sum(t.kind == "half" for t in stones)
    lines += ["", f"Total {len(stones)} stones ({len(stones) - n_half} full, {n_half} half), course top z = "
              + ", ".join(f"{wp.course_top(cfg, k):.0f}" for k in range(cfg["wall"]["courses"])) + " mm.", "",
              "## Stops", "",
              "| stop | leg | a mm | ARES pose in the wall frame | stones | boards with a usable look | "
              "looks (longest baseline) |", "|---|---|---|---|---|---|---|"]
    for st, rb in zip(job.stops, geo["reach"]):
        lines.append(f"| {st.index} | {st.leg} | {st.a_mm:.0f} | ({st.ares.x_mm:.0f}, {st.ares.y_mm:.0f}) mm, "
                     f"{st.ares.theta_deg:.0f} deg | {len(st.stones)} | {', '.join(rb)} | "
                     f"{', '.join(b for lk in st.looks for b in lk.boards)} |")
    lines += ["", "Usable look = a board of the stop's OWN leg (a board of another leg lies across a built corner, "
              "and a frame fitted over two legs would push one leg's pose error into the other leg's stones), camera "
              "fronto-parallel at the working distance, a family IK solution for ARES at the stop, +-50 mm around it "
              "and at the arrival standoff, every one of them clear of the stones built by the END of that stop "
              "(mauer.armcheck: UR5 links, gripper and camera as capsules with ASSUMED radii vs. stone boxes over the "
              "ribs). Coarse envelopes, no occlusion check - robodk/simulate.py checks both in RoboDK."]
    lines += ["", "## Boards (existing W0..W7, new places)", "",
              "| board | ids | leg | u mm | block | board origin in the leg frame (xyz mm) |", "|---|---|---|---|---|---|"]
    specs = board_specs(cfg)
    for s_ in geo["sites"]:
        blk = (f"k = {s_.k} ({s_.k + 1}. block from the leg start)" if not s_.spare
               else f"spare block k = {s_.k} beyond the leg end")
        sp = specs[s_.board]
        lines.append(f"| {s_.board} | {sp.first_id}-{sp.last_id} | {s_.leg} | {s_.u:.0f} | {blk} | "
                     f"({s_.xyz[0]:.1f}, {s_.xyz[1]:.1f}, {s_.xyz[2]:.1f}) |")
    lines += ["", "No two plates overlap; none overlaps the other leg, ARES at a stop or the station table "
              "(mauer/floor.py check_sites)." if not geo["problems"] else "PROBLEMS: " + "; ".join(geo["problems"]), ""]
    rc = m.get("route_check", {})
    lines += ["## ARES routes", "",
              f"Waypoints in the wall frame, one PLC command per leg (translation in the body frame OR rotation, "
              f">= 2 mm / 0.2 deg). Checked with the chassis {rc.get('ares_radius_mm', 0) * 2:.0f} mm diagonal "
              f"([ares] length x width), clearance {rc.get('clearance_mm', 0):g} mm to legs, plates, blocks and the "
              f"station table (rotation: swept circle r {rc.get('ares_radius_mm', 0):.0f} mm); every move between "
              "stops is a route too (same leg: back off, along the leg, approach - the plates end 10 mm before the ARES "
              "front at a stop, so the stop line itself keeps no clearance). The sequencer steers intermediate legs "
              f"dead-reckoned and ends a route to a stop {rc.get('arrival_standoff_mm', 0):g} mm before it ([sequencer] "
              "arrival_standoff_mm: ARES front 50 mm from the plates), measures the wall there and reaches the stop by "
              "the closed-loop correction; the dock is reached closed loop on the last leg (camera at the dock).", ""]
    for st in job.stops:
        if st.route:
            sts = route_stats(st.route)
            prev = job.stops[st.index - 1]
            what = "Leg change" if prev.leg != st.leg else "Move along leg " + str(st.leg)
            lines.append(f"{what} to stop {st.index} ({sts['length_mm']:.0f} mm, {sts['translations']} "
                         f"translations, {sts['rotations']} rotation{'s' if sts['rotations'] != 1 else ''}):")
            lines.append("")
            segs, _ = segments(st.route)
            for i, sg in enumerate(segs):
                if sg.kind == "translate":
                    dx, dy, _ = relative_move(sg.start, sg.end)
                    lines.append(f"{i + 1}. translate body dx {dx:+.0f} / dy {dy:+.0f} mm -> ({sg.end.x_mm:.0f}, "
                                 f"{sg.end.y_mm:.0f}) mm, {sg.end.theta_deg:.0f} deg")
                else:
                    lines.append(f"{i + 1}. rotate {math.degrees(sg.angle_rad):+.0f} deg at ({sg.start.x_mm:.0f}, "
                                 f"{sg.start.y_mm:.0f}) mm")
            lines.append("")
    lines += ["| stop | to station mm (waypoints) | back mm (waypoints) |", "|---|---|---|"]
    for st in job.stops:
        a, b = route_stats(st.route_to_station), route_stats(st.route_from_station)
        lines.append(f"| {st.index} | {a['length_mm']:.0f} ({len(st.route_to_station)}) | {b['length_mm']:.0f} "
                     f"({len(st.route_from_station)}) |")
    used = [job.station.slot(i) for i in job.station.take_order]
    n_full, n_half = sum(s_.kind == "full" for s_ in used), sum(s_.kind == "half" for s_ in used)
    lines += ["", "## Magazine and reloads", "",
              f"Magazine {job.magazine.capacity} deck slots (each holds a full or a half stone), station "
              f"{len(used)} reachable slots ({n_full} full, {n_half} half, stacks of up to "
              f"{max((s_.layer for s_ in used), default=0)}). Planned: {m.get('planned_reloads')} reloads (station "
              f"trips), {m.get('planned_station_refills')} station refills by the operator. A trip brings at most "
              f"min(magazine, station) = {min(job.magazine.capacity, len(used))} stones, fewer when the next stones "
              f"need more half stones than the station holds - the number of trips is a property of the PLACEHOLDER "
              "station layout ([pickup_station]).", ""]
    if rows:
        best = tuple(lg.n0 for lg in legs_)
        lines += ["## Candidates", "",
                  "`py.exe tools/plan_layout.py --evaluate [--legs ...]` (results/l_wall_candidates.json). Per "
                  "candidate: plan with "
                  "a 20 mm reach margin, plate sites on block centres / spare blocks, usable looks as above (own leg, "
                  "clear of the built wall), best board set (<= 8 boards, boards per stop counted up to 4), routes "
                  "validated. Wall moves = PLC commands of the routes between stops (same leg: 3, leg change: 5). "
                  "Ranking: feasible > fewer stops > fewer wall moves > course-0 stones within 14..18, more is better "
                  "(\"a bit shorter\" than 24) > more boards per stop > fewer spare blocks.", ""]
        lines += table_md(rows, best)
        lines.append("")
    sim_path = RESULTS / "l_wall_sim.json"
    if sim_path.exists():
        sim = json.loads(sim_path.read_text())
        lines += ["## World simulation (mauer.simworld)", "", sim.get("note", ""), "",
                  "| seed | scenario | mode | placed | seated (half) | horizontal mean / p95 / max mm | at the pins "
                  "max mm | ARES moves | slips | corrections | reloads | shots | ARES on a plate / leg (max mm, "
                  "move) | smallest gap mm | stopped |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in sim["runs"]:
            pz = r["placement"]
            h = pz.get("horiz_mm", {})
            bk = pz.get("by_kind", {}).get("half", {})
            fl = r.get("floor", {})
            hits = fl.get("hits", [])
            hit_txt = "-" if not hits else (f"{len(hits)} ({max(x['depth_mm'] for x in hits):.1f}, "
                                            f"{', '.join(sorted({x.get('why', '?') for x in hits}))})")
            gap = fl.get("min_gap_mm")
            lines.append(f"| {r['seed']} | {r.get('scenario', 'realistic')} | {r['mode']} | {pz['n']} | {pz['seated']} "
                         f"({bk.get('seated', 0)}/{bk.get('n', 0)}) | {h.get('mean', 0):.2f} / {h.get('p95', 0):.2f} / "
                         f"{h.get('max', 0):.2f} | {pz.get('pin_mm', {}).get('max', 0):.2f} | {r['ares']['moves']} | "
                         f"{r['ares']['slips']} | {r['corrections']} | {r['reloads']} | {r['shots']} | {hit_txt} | "
                         f"{'-' if gap is None else f'{gap:.1f}'} | {(r['error'] or '-')[:70]} |")
        lines += [""] + list(sim.get("discussion", [])) + [""]
    lines += ["## Depends on (non-CONFIRMED config values)", "",
              "| key | status | value |", "|---|---|---|"]
    for d in job.depends_on:
        v = json.dumps(d["value"])
        lines.append(f"| `{d['key']}` | {d['status']} | {v if len(v) < 60 else v[:57] + '...'} |")
    lines += ["", "Warnings of the job build:", ""] + [f"- {w}" for w in m.get("warnings", [])] + [""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--evaluate", action="store_true", help="evaluate the leg-length candidates")
    ap.add_argument("--legs", default=None, help='candidates: "10,7,5" or ranges "9-12,5-8" (default: the config)')
    ap.add_argument("--config", default=None)
    ap.add_argument("--png", default=str(RESULTS / "l_wall_layout.png"))
    ap.add_argument("--md", default=str(RESULTS / "l_wall_plan.md"))
    args = ap.parse_args(argv)
    cfg = mconfig.load(args.config)
    if args.evaluate:
        cands = (parse_legs(args.legs) if args.legs
                 else [tuple(int(lg["n0"]) for lg in cfg["wall"]["legs"])])
        rows = evaluate_all(cfg, cands)
        RESULTS.mkdir(exist_ok=True)
        CANDIDATES_JSON.write_text(json.dumps({"rows": rows}, indent=1, default=str) + "\n", encoding="utf-8")
        best = rows[0]
        print("\n".join(table_md(rows, tuple(best["n0"]))))
        print(f"\nbest: {legs_label(best)} -> enter in config/station.toml:\n")
        print(config_snippet(cfg, best))
        print(f"\nwritten {CANDIDATES_JSON}")
        return 0
    geo = l_geometry(cfg)
    if geo["problems"]:
        print("PROBLEMS:\n  - " + "\n  - ".join(geo["problems"]))
    rows = json.loads(CANDIDATES_JSON.read_text())["rows"] if CANDIDATES_JSON.exists() else None
    draw_layout(cfg, geo, Path(args.png))
    write_report(cfg, geo, Path(args.md), rows)
    print(f"written {args.png}\nwritten {args.md}")
    return 1 if geo["problems"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
