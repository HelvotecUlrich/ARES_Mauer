"""robodk/wallplan.py: straight wall (unchanged API) and the legs of the L (rectangles with half stones, supports by
footprint overlap, cross-leg checks) on the cached reach table (results/reach_table.json, 4 courses)."""
import copy
import importlib.util
import math

import pytest

from conftest import l_config
from mauer import REPO, config


def _wallplan():
    spec = importlib.util.spec_from_file_location("wallplan_under_test", REPO / "robodk" / "wallplan.py")
    mod = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


wp = _wallplan()


@pytest.fixture(scope="module")
def cfg():
    return l_config()


@pytest.fixture(scope="module")
def table(cfg):
    from mauer import reach_cache
    return reach_cache.get(cfg, cfg["wall"]["dist_nominal"])


@pytest.fixture(scope="module")
def lplan(cfg, table):
    legs = wp.legs(cfg)
    stones, plans = wp.plan_legs(cfg, legs, table, True, cfg["wall"]["reach_margin_mm"])
    return legs, stones, plans


def test_straight_wall_is_the_old_trapezoid(cfg, table):
    stones = wp.layout(cfg, 24)
    assert [sum(s.course == k for s in stones) for k in range(4)] == [24, 23, 22, 21]
    assert all(s.leg == "" and s.kind == "full" and s.key == (s.course, s.index) for s in stones)
    reach, lo, hi, grid = wp.reach_fn(table)
    plan = wp.sequence(cfg, stones, reach, lo, stones[0].u - lo[0])
    assert wp.check_plan(cfg, stones, plan) == []
    assert [(a, len(b)) for a, b in plan] == [(860.0, 26), (1860.0, 20), (2860.0, 20), (3860.0, 20), (4860.0, 4)]


def test_leg_is_a_rectangle_with_half_stones(cfg):
    leg = wp.Leg("A", 7)
    stones = wp.layout_leg(cfg, leg)
    L = wp.leg_length(cfg, 7)
    for k in range(cfg["wall"]["courses"]):
        row = sorted((s for s in stones if s.course == k), key=lambda s: s.u)
        assert row[0].u - row[0].length / 2 == pytest.approx(0.0) and row[-1].u + row[-1].length / 2 == pytest.approx(L)
        for a, b in zip(row, row[1:]):                                    # no gaps, no overlaps
            assert wp.adjacent(a, b, cfg["brick"]["length"])
        if k % 2:
            assert [s.kind for s in row] == ["half"] + ["full"] * 6 + ["half"]
            assert row[0].length == cfg["half_brick"]["length"]
        else:
            assert [s.kind for s in row] == ["full"] * 7
    # trapezoid fallback (no half stones): the old slopes
    trap = wp.layout_leg(cfg, leg, half_stones=False)
    assert [sum(s.course == k for s in trap) for k in range(4)] == [7, 6, 5, 4] and all(s.leg == "A" for s in trap)


def test_supports_by_footprint_overlap(cfg):
    stones = wp.layout_leg(cfg, wp.Leg("A", 5))
    by = {}
    for s in stones:
        by.setdefault(s.course, []).append(s)
    L = cfg["brick"]["length"]
    half = next(s for s in stones if s.course == 1 and s.kind == "half")
    assert [p.key for p in wp.supports(half, by, L)] == [("A", 0, 0)]        # a half stone sits on ONE stone
    full2 = next(s for s in stones if s.course == 2 and s.index == 0)      # full above half + full
    sup = wp.supports(full2, by, L)
    assert sorted(p.kind for p in sup) == ["full", "half"] and len(sup) == 2
    mid = next(s for s in stones if s.course == 1 and s.kind == "full")
    assert len(wp.supports(mid, by, L)) == 2


def test_l_plan_every_stone_once_supports_and_no_cross_leg(cfg, lplan):
    legs, stones, plans = lplan
    assert wp.check_plan_legs(cfg, legs, stones, plans) == []
    order = [s for lg in legs for _, b in plans[lg.name] for s in b]
    assert sorted(s.key for s in order) == sorted(s.key for s in stones) and len(set(s.key for s in order)) == len(order)
    assert sum(s.kind == "half" for s in stones) == 4 * len(legs)          # 2 odd courses x 2 ends per leg
    assert [len(plans[lg.name]) for lg in legs] == [2, 1]                  # 3 stops (trade-off choice)
    # stones never overlap across legs and nothing is supported by the other leg
    assert wp.check_legs(cfg, legs, stones) == []
    # reach margin: every stone reachable with ARES 20 mm off the stop in both directions
    reach, *_ = wp.reach_fn(__import__("mauer.reach_cache", fromlist=["get"]).get(cfg, cfg["wall"]["dist_nominal"]))
    for lg in legs:
        for a, batch in plans[lg.name]:
            for s in batch:
                for d in (-20.0, 0.0, 20.0):
                    assert reach(s.course, s.u - a - d), (s.key, a, d)


def test_l_plan_trapezoid_fallback(cfg, table):
    legs = wp.legs(cfg)
    stones, plans = wp.plan_legs(cfg, legs, table, half_stones=False, margin_mm=20.0)
    assert all(s.kind == "full" for s in stones)
    assert wp.check_plan_legs(cfg, legs, stones, plans) == []


def test_cross_leg_checks_catch_a_bad_corner(cfg):
    A = wp.Leg("A", 4)
    B_bad = wp.Leg("B", 3, A.n0 * 200.0 - 60.0, 0.0, -math.pi / 2)       # B starting on A's centreline: overlap
    stones = wp.layout_legs(cfg, [A, B_bad])
    errs = "\n".join(wp.check_legs(cfg, [A, B_bad], stones))
    assert "overlap" in errs and "supported by" in errs
    B = wp.butt_corner(cfg, A, 3, "B")
    assert wp.check_legs(cfg, [A, B], wp.layout_legs(cfg, [A, B])) == []


def test_check_plan_catches_order_errors(cfg, table):
    legs = [wp.Leg("A", 6)]
    stones, plans = wp.plan_legs(cfg, legs, table)
    a, batch = plans["A"][0]
    bad = [(a, list(reversed(batch)))]                                     # tops first
    errs = "\n".join(wp.check_plan(cfg, stones, bad + plans["A"][1:]))
    assert "before its support" in errs


def test_check_plan_catches_a_stone_set_under_the_corner_of_one_above(cfg):
    """Guarded world sim 2026-10-09 (the C with a window in A): the jamb stone A.2.2h (u 700..800, course 2) was set
    before A.1.3 (u 500..700, course 1) below the window - A.1.3's corner then ends 1 mm under the jamb stone (the
    motion guard refuses the target). A stone of the course above that touches a stone end to end (aligned joint at
    an opening) counts like one lying on it."""
    A = wp.Leg("A", 5, openings=(wp.Opening(300.0, 700.0, 2, 3, "window"),))
    st = wp.layout_leg(cfg, A)
    late = next(s for s in st if s.course == 1 and s.u == 600.0)
    jamb = next(s for s in st if s.course == 2 and s.u == 750.0)
    rest = sorted((s for s in st if s.key not in (late.key, jamb.key)), key=lambda s: (s.course, s.u))
    k = max(i for i, s in enumerate(rest) if s.course <= 1) + 1
    bad = rest[:k] + [jamb, late] + rest[k:]
    errs = wp.check_plan(cfg, st, [(0.0, bad)])
    assert any(str(late.key) in e and str(jamb.key) in e and "corner" in e for e in errs), errs
    good = rest[:k] + [late, jamb] + rest[k:]
    assert wp.check_plan(cfg, st, [(0.0, good)]) == []


def test_the_window_leg_built_from_its_far_end_sets_no_stone_under_a_corner(cfg):
    """The C with a door and a window (main config): A is built from its corner with B; the stone below the window
    next to the jamb comes before the jamb stones above it."""
    c = config.load()
    legs = wp.legs(c)
    A = next(lg for lg in legs if lg.openings)
    _, plans = _plans(c)
    order = [s for _, b in plans[A.name] for s in b]
    pos = {s.key: i for i, s in enumerate(order)}
    L = float(c["brick"]["length"])
    by = wp._by_course(order)
    for s in order:
        for t in by.get(s.course + 1, []):
            if wp.adjacent(t, s, L):
                assert pos[s.key] < pos[t.key], (s.label, t.label)


def test_reach_fn_is_conservative(table):
    reach, lo, hi, grid = wp.reach_fn(table)
    assert grid == 20.0 and lo[3] == -620.0 and hi[3] == 620.0             # 840 mm, pins up (2026-10-06)
    assert reach(3, -620.0) and not reach(3, -630.0)                       # between grid points: both must be ok
    assert reach(3, -610.0)
    reach_m, *_ = wp.reach_fn(table, margin_mm=20.0)
    assert not reach_m(3, -620.0) and reach_m(3, -600.0)


def test_legs_from_config_and_validation(cfg):
    legs = wp.legs(cfg)
    assert [lg.name for lg in legs] == ["A", "B"]
    T = config.leg_frames(cfg)
    for lg in legs:
        assert (lg.x, lg.y, lg.theta) == pytest.approx((T[lg.name][0, 3], T[lg.name][1, 3],
                                                        math.atan2(T[lg.name][1, 0], T[lg.name][0, 0])))
        u, v = 123.0, -45.0
        assert lg.from_wall(*lg.to_wall(u, v)) == pytest.approx((u, v))
    bad = copy.deepcopy(cfg)
    bad["wall"]["legs"][1]["rpy_in_wall_deg"] = [5.0, 0.0, -90.0]
    with pytest.raises(ValueError, match="floor-level"):
        wp.legs(bad)
    bad = copy.deepcopy(cfg)
    bad["half_brick"]["length"] = 120.0
    with pytest.raises(ValueError, match="bond offset"):
        wp.layout_leg(bad, legs[0])


# ── legs built inside a corner (inside C, 2026-10-06) ─────────────────────────
def test_butt_corner_towards_ares_mirrors_the_outside_corner(cfg):
    A = wp.Leg("A", 10)
    out, ins = wp.butt_corner(cfg, A, 7, "B"), wp.butt_corner(cfg, A, 7, "B", towards_ares=True, side="front", dist=840.0)
    assert (ins.x, ins.y, math.degrees(ins.theta)) == pytest.approx((out.x, -out.y, -math.degrees(out.theta)))
    assert ins.y == pytest.approx(cfg["brick"]["width"] / 2 + cfg["brick"]["rib_mm"] + cfg["wall"]["corner_gap_mm"])
    assert (ins.side, ins.dist) == ("front", 840.0) and (out.side, out.dist) == ("", None)
    C = wp.butt_corner(cfg, ins, 5, "C", towards_ares=True)
    assert math.degrees(C.theta) == pytest.approx(180.0) and C.y > ins.y          # C runs back above A: a C
    lg = [A, ins, C]
    assert wp.check_legs(cfg, lg, wp.layout_legs(cfg, lg)) == []


def test_a_leg_running_through_a_corner_is_built_before_the_one_butting_on_it(cfg):
    """2026-10-07: C runs through its corner with B (ends of A and C lined up) -> built A, C, B (RoboDK: with B first,
    C's corner stones beyond B's end could not be gripped); the corners are recognised with their gap."""
    A = wp.Leg("A", 5)
    B = wp.butt_corner(cfg, A, 7, "B", towards_ares=True)
    C_old = wp.butt_corner(cfg, B, 5, "C", towards_ares=True)
    C = wp.butt_corner(cfg, B, 5, "C", towards_ares=True, through="next")
    gap = cfg["wall"]["corner_gap_mm"]
    assert wp.corner_of(cfg, A, B) == pytest.approx({"through": "prev", "towards_ares": True, "gap_mm": gap})
    assert wp.corner_of(cfg, B, C) == pytest.approx({"through": "next", "towards_ares": True, "gap_mm": gap})
    assert [lg.name for lg in wp.build_order(cfg, [A, B, C_old])] == ["A", "B", "C"]
    assert [lg.name for lg in wp.build_order(cfg, [A, B, C])] == ["A", "C", "B"]
    assert A.to_wall(0.0, 0.0)[0] == pytest.approx(C.to_wall(wp.leg_length(cfg, 5), 0.0)[0])     # ends line up
    assert wp.check_legs(cfg, [A, B, C], wp.layout_legs(cfg, [A, B, C])) == []


def test_stop_limits_keep_ares_between_the_other_legs(cfg):
    """Leg B of the inside C on the front table at 840 mm: without limits the last stop is centred on the remaining
    stones (a = 1280 - ARES would stand in leg C); with a_lim every stop lies in the window and every stone is placed."""
    from mauer import reach_cache
    table = reach_cache.get(cfg, 840.0, side="front")
    B = wp.Leg("B", 7)
    free = wp.plan_legs(cfg, [B], table, True, 20.0)[1]["B"]
    assert max(a for a, _ in free) > 820.0
    stones, plans = wp.plan_legs(cfg, [B], {}, True, 20.0, tables={"B": table}, a_limits={"B": (457.3, 820.0)})
    assert all(460.0 <= a <= 820.0 for a, _ in plans["B"])
    assert sorted(s.key for _, batch in plans["B"] for s in batch) == sorted(s.key for s in stones)
    with pytest.raises(RuntimeError):                                           # no position reaches B's far end
        wp.plan_legs(cfg, [B], table, True, 20.0, a_limits={"B": (0.0, 300.0)})


def test_leg_side_and_distance_from_the_config(cfg):
    import copy
    c = copy.deepcopy(cfg)
    c["wall"]["legs"][0]["side"], c["wall"]["legs"][0]["dist"] = "right", 580.0
    lg = wp.legs(c)
    assert (lg[0].side, lg[0].dist) == ("right", 580.0) and (lg[1].side, lg[1].dist) == ("", None)
    c["wall"]["legs"][1]["side"] = "inside"
    with pytest.raises(ValueError, match="side 'inside'"):
        wp.legs(c)


def test_a_leg_of_five_and_a_half_stones(cfg):
    """2026-10-07, main config: A 5 1/2 stones - even courses 5 full + a half stone at the end, odd courses a half
    stone at the start + 5 full; length 1100 mm; 24 stones; the full stones of course 0 keep whole-pitch joints."""
    A = wp.Leg("A", wp.stones_n0(5.5))
    st = wp.layout_leg(cfg, A)
    assert len(st) == 24 and wp.leg_length(cfg, A.n0) == pytest.approx(1100.0)
    c0 = sorted((s.u, s.kind) for s in st if s.course == 0)
    c1 = sorted((s.u, s.kind) for s in st if s.course == 1)
    assert c0 == [(100.0, "full"), (300.0, "full"), (500.0, "full"), (700.0, "full"), (900.0, "full"), (1050.0, "half")]
    assert c1 == [(50.0, "half"), (200.0, "full"), (400.0, "full"), (600.0, "full"), (800.0, "full"), (1000.0, "full")]
    assert wp.stones_n0(5) == 5 and isinstance(wp.stones_n0(5.0), int)
    for bad in (5.25, 0.5, "x"):
        with pytest.raises(ValueError):
            wp.stones_n0(bad)
    with pytest.raises(ValueError, match="need half stones"):
        wp.layout_leg(cfg, A, half_stones=False)
    B = wp.butt_corner(cfg, A, 7, "B", towards_ares=True)
    C = wp.butt_corner(cfg, B, 5, "C", towards_ares=True)
    assert (B.x, B.y) == pytest.approx((1040.0, 62.69)) and (C.x, C.y) == pytest.approx((977.31, 1402.69))
    assert C.to_wall(wp.leg_length(cfg, 5), 0.0)[0] == pytest.approx(-22.69)     # C's end 22.7 mm beyond A's start
    assert [lg.name for lg in wp.build_order(cfg, [A, B, C])] == ["A", "B", "C"]
    assert wp.check_legs(cfg, [A, B, C], wp.layout_legs(cfg, [A, B, C])) == []


def test_balanced_stops_avoid_a_stop_for_a_single_stone(cfg):
    """2026-10-07 (Samuel: ARES drives once more for one stone): leg B (7 stones) at 840 mm in front needs two stops
    - its top course is 1.30 m long, the UR reaches +-0.62 m along the wall there - and the greedy plan put 29 + 1
    stones on them; balancing moves the first stop back so that the second does real work."""
    from mauer import reach_cache
    tab = reach_cache.get(cfg, 840.0, side="front")
    if tab is None:
        pytest.skip("no cached reach table for the front at 840 mm (results/reach_table.json)")
    reach, lo, hi, grid = wp.reach_fn(tab, margin_mm=20.0)
    B = wp.Leg("B", 7)
    own = wp.layout_leg(cfg, B)
    greedy = wp.sequence_leg(cfg, own, reach, lo, hi, grid, 20.0, a_lim=(460.0, 940.0), balance=False)
    best = wp.sequence_leg(cfg, own, reach, lo, hi, grid, 20.0, a_lim=(460.0, 940.0))
    assert [len(b) for _, b in greedy] == [29, 1]
    assert len(best) == 2 and min(len(b) for _, b in best) > 1 and sum(len(b) for _, b in best) == 30
    assert wp.check_plan(dict(cfg), own, best) == []


# ── the C of 2026-10-08: openings (door, window), the back leg through both corners ─────────────────────────────
def _rows(stones):
    return {k: [(s.u, s.kind) for s in sorted((t for t in stones if t.course == k), key=lambda t: t.u)]
            for k in sorted({s.course for s in stones})}


def test_a_window_in_the_upper_courses_keeps_the_bond_and_straight_jambs(cfg):
    """Samuel's photos 2026-10-08: leg A (5 stones) with a window 400 mm wide, 300 mm from both ends, in courses 3 + 4
    (index 2, 3). The running bond of the leg is cut by the leg ends and the opening; 100 mm pieces are half stones -
    the jambs are straight and every joint stays offset from the one below."""
    A = wp.Leg("A", 5, openings=(wp.Opening(300.0, 700.0, 2, 3, "window"),))
    st = wp.layout_leg(cfg, A)
    F, H = "full", "half"
    assert _rows(st) == {0: [(100.0, F), (300.0, F), (500.0, F), (700.0, F), (900.0, F)],
                         1: [(50.0, H), (200.0, F), (400.0, F), (600.0, F), (800.0, F), (950.0, H)],
                         2: [(100.0, F), (250.0, H), (750.0, H), (900.0, F)],
                         3: [(50.0, H), (200.0, F), (800.0, F), (950.0, H)]}
    by = wp._by_course(st)
    assert all(wp.supports(s, by, 200.0) for s in st if s.course)
    assert len({s.key for s in st}) == len(st) == 19
    joints = {k: {round(s.u - s.length / 2) for s in st if s.course == k} - {0, 300, 700} for k in range(4)}
    assert all(not (joints[k] & joints[k + 1]) for k in range(3))     # no joint over a joint (besides the jambs)


def test_a_door_leaves_the_piece_at_the_free_end_and_start_half_flips_the_bond(cfg):
    """Leg C: a door 600 mm wide next to the back leg (u 0..600, all courses) - only the 400 mm piece at the free end
    stays; start_half: course 1 (index 0) half + full + half, course 2 two full stones, as in the photos."""
    C = wp.Leg("C", 5, openings=(wp.Opening(0.0, 600.0, 0, 3, "door"),), start_half=True)
    st = wp.layout_leg(cfg, C)
    F, H = "full", "half"
    assert _rows(st) == {0: [(650.0, H), (800.0, F), (950.0, H)], 1: [(700.0, F), (900.0, F)],
                         2: [(650.0, H), (800.0, F), (950.0, H)], 3: [(700.0, F), (900.0, F)]}
    plain = wp.layout_leg(cfg, wp.Leg("C", 5))
    assert _rows(plain)[0] == [(100.0, F), (300.0, F), (500.0, F), (700.0, F), (900.0, F)]   # unchanged without


def test_openings_and_start_half_from_the_config_and_validation(cfg):
    c = copy.deepcopy(cfg)
    c["wall"]["legs"][0]["openings"] = [{"name": "window", "u_from": 300.0, "u_to": 700.0, "courses": [2, 3]}]
    c["wall"]["legs"][0]["start_half"] = True
    lg = wp.legs(c)[0]
    assert lg.openings == (wp.Opening(300.0, 700.0, 2, 3, "window"),) and lg.start_half
    assert not wp.legs(cfg)[0].openings and not wp.legs(cfg)[0].start_half
    for bad, msg in (({"u_from": 250.0, "u_to": 700.0, "courses": [2, 3]}, "half-stone grid"),
                     ({"u_from": 300.0, "u_to": 99900.0, "courses": [2, 3]}, "outside the leg"),
                     ({"u_from": 300.0, "u_to": 700.0, "courses": [2, 9]}, "courses"),
                     ({"u_from": 700.0, "u_to": 300.0, "courses": [2, 3]}, "u_from < u_to")):
        c["wall"]["legs"][0]["openings"] = [bad]
        with pytest.raises(ValueError, match=msg):
            wp.layout_leg(c, wp.legs(c)[0])


def test_the_back_leg_runs_through_both_corners(cfg):
    """The C of 2026-10-08: B (9 stones, 1.8 m) runs through both corners, A and C (5 stones) butt against its
    inside face; the free ends of A and C line up; built B, A, C."""
    A = wp.Leg("A", 5, openings=(wp.Opening(300.0, 700.0, 2, 3, "window"),))
    B = wp.butt_corner(cfg, A, 9, "B", towards_ares=True, through="next")
    C = wp.butt_corner(cfg, B, 5, "C", towards_ares=True, through="prev")
    C = wp.Leg(C.name, C.n0, C.x, C.y, C.theta, openings=(wp.Opening(0.0, 600.0, 0, 3, "door"),), start_half=True)
    gap = cfg["wall"]["corner_gap_mm"]
    assert wp.corner_of(cfg, A, B) == pytest.approx({"through": "next", "towards_ares": True, "gap_mm": gap})
    assert wp.corner_of(cfg, B, C) == pytest.approx({"through": "prev", "towards_ares": True, "gap_mm": gap})
    assert [lg.name for lg in wp.build_order(cfg, [A, B, C])] == ["B", "A", "C"]
    assert (B.x, B.y) == pytest.approx((1062.69, -60.0)) and (C.x, C.y) == pytest.approx((1000.0, 1680.0))
    assert C.to_wall(wp.leg_length(cfg, 5), 0.0)[0] == pytest.approx(A.to_wall(0.0, 0.0)[0])     # free ends line up
    st = wp.layout_legs(cfg, [A, B, C])
    assert wp.check_legs(cfg, [A, B, C], st) == []
    assert [sum(s.leg == n for s in st) for n in "ABC"] == [19, 38, 10]
    assert sum(s.kind == "half" for s in st) == 14


def test_a_door_in_the_middle_of_course_0_frees_the_first_stone_of_each_piece(cfg):
    """Rule 2 per piece: a course-0 stone needs a placed neighbour unless it is the first of its contiguous piece
    (a door in the middle splits course 0) - the plan does not get stuck and check_plan agrees."""
    A = wp.Leg("A", 7, openings=(wp.Opening(600.0, 800.0, 0, 3, "door"),))
    st = wp.layout_leg(cfg, A)
    reach = lambda k, u: abs(u) <= 2000.0                                     # noqa: E731 - everything reachable
    lo = {k: -2000.0 for k in range(4)}
    hi = {k: 2000.0 for k in range(4)}
    plan = wp.sequence_leg(cfg, st, reach, lo, hi, 20.0)
    assert wp.check_plan(cfg, st, plan) == []
    order = [s.key for _, b in plan for s in b]
    assert sorted(order) == sorted(s.key for s in st)


def _plans(c):
    """(stones, plans) of the config's legs as make_job plans them (own reach table per leg side, stop limits)."""
    import sys
    if str(REPO / "tools") not in sys.path:
        sys.path.insert(0, str(REPO / "tools"))
    import make_job as mj
    from mauer import reach_cache
    legs = wp.legs(c)
    tables = {lg.name: reach_cache.get(c, mj.leg_dist(c, lg, c["wall"]["dist_nominal"]), side=mj.leg_side(c, lg))
              for lg in legs}
    return wp.plan_legs(c, legs, {}, True, float(c["wall"]["reach_margin_mm"]), tables=tables,
                        a_limits=mj.stop_limits(c, legs, c["wall"]["dist_nominal"]))


def _closures(c):
    """Stones placed while BOTH end faces already have a placed stone of the same course next to them (any leg,
    probed 3 mm beyond each end face over the ribs) - they drop into a slot with only the corner gap as play."""
    import sys
    if str(REPO / "tools") not in sys.path:
        sys.path.insert(0, str(REPO / "tools"))
    import make_job as mj
    from mauer import reach_cache
    legs = wp.legs(c)
    tables = {lg.name: reach_cache.get(c, mj.leg_dist(c, lg, c["wall"]["dist_nominal"]), side=mj.leg_side(c, lg))
              for lg in legs}
    stones, plans = wp.plan_legs(c, legs, {}, True, float(c["wall"]["reach_margin_mm"]), tables=tables,
                                 a_limits=mj.stop_limits(c, legs, c["wall"]["dist_nominal"]))
    by = {lg.name: lg for lg in legs}
    order = [s for lg in wp.build_order(c, legs) for _, b in plans[lg.name] for s in b]

    def inside(q, P):
        n = len(P)
        return all(((P[(i + 1) % n][0] - P[i][0]) * (q[1] - P[i][1]) - (P[(i + 1) % n][1] - P[i][1]) * (q[0] - P[i][0]))
                   > 0 for i in range(n))

    placed, out = [], []
    for s in order:
        lg = by[s.leg]
        ends = [lg.to_wall(s.u + d * (s.length / 2 + 3.0), 0.0) for d in (-1, 1)]
        same = [wp.stone_footprint(c, t, by[t.leg], ribs=True) for t in placed if t.course == s.course]
        if all(any(inside(q, P) for P in same) for q in ends):
            out.append(s.label)
        placed.append(s)
    return out


@pytest.mark.parametrize("variant, n", [(None, 0), ("c_a55", 0), ("c_acb", 4)])
def test_no_stone_drops_into_a_slot_unless_a_leg_closes_between_two(variant, n):
    """Review 2026-10-08: in the C with a door and a window B runs through both corners and is built first; A, built
    from its free end, put its last stone of every course between its neighbour and B (1 mm play). A leg whose far
    end butts against a finished leg is now built from that corner. c_acb's B closes between A and C on purpose
    (risk accepted, make_job warns): one closure per course."""
    c = config.load(variant=variant)
    got = _closures(c)
    assert len(got) == n, got
