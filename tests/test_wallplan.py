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
