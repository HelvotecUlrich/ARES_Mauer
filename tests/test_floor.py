"""mauer/floor.py: polygons, the route clearance validator (deliberately bad routes must fail), the route planner and
the floor-plate sites of the L (config [[targets]] with legs, tools/make_job.py floor_model)."""
import copy
import importlib.util
import math

import pytest

from mauer import REPO, config
from mauer import floor
from mauer.reference import Pose2D
from mauer.vision.targets import board_specs


def _tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


make_job = _tool("make_job")
wp = make_job.load_wallplan()
DEG = math.pi / 180.0


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def model(cfg):
    legs = wp.legs(cfg)
    dist = cfg["wall"]["dist_nominal"]
    T = config.leg_frames(cfg)
    stops = [("A1", make_job.stop_pose(cfg, dist, 1780.0, T["A"])), ("B0", make_job.stop_pose(cfg, dist, 600.0, T["B"]))]
    sites = make_job.target_sites(cfg, legs)
    obst, problems = make_job.floor_model(cfg, legs, sites, stops)
    return {"legs": legs, "stops": dict(stops), "sites": sites, "obst": obst, "problems": problems,
            "ares": floor.AresShape.from_config(cfg), "clr": cfg["routes"]["clearance_mm"]}


# ── polygons ──────────────────────────────────────────────────────────────────
def test_polygon_helpers():
    a = floor.box_poly(0, 0, 100, 50)
    b = floor.box_poly(130, 0, 200, 50)
    assert floor.poly_dist(a, b) == pytest.approx(30.0)
    assert not floor.overlaps(a, floor.box_poly(100, 0, 150, 50))           # touching does not overlap
    assert floor.overlaps(a, floor.box_poly(99, 0, 150, 50))
    assert floor.poly_dist(a, floor.box_poly(50, 10, 60, 20)) == 0.0        # inside
    r = floor.rect_poly(0, 0, math.pi / 2, 1120, 600)                       # ARES heading +y: long side along y
    assert max(p[1] for p in r) == pytest.approx(560) and max(p[0] for p in r) == pytest.approx(300)
    h = floor.hull(a + b)
    assert len(h) == 4 and floor.point_inside((150, 25), h)
    assert floor.circle_dist((0, 200), 100, a) == pytest.approx(50.0)
    assert floor.AresShape(1120, 600).radius == pytest.approx(math.hypot(560, 300))


# ── validator ─────────────────────────────────────────────────────────────────
def test_the_planned_leg_change_is_valid(model):
    s, g_ = model["stops"]["A1"], model["stops"]["B0"]
    r = floor.plan_route(s, g_, model["obst"], model["ares"], model["clr"], 100.0)
    assert floor.validate_route(r.waypoints, model["obst"], model["ares"], model["clr"]) == []
    segs, problems = floor.segments(r.waypoints)
    assert problems == [] and [sg.kind for sg in segs].count("rotate") == 1
    assert segs[0].kind == "translate" and segs[0].length_mm == pytest.approx(100.0)    # back off first
    rot = next(sg for sg in segs if sg.kind == "rotate")
    # rotation close to the goal: only the approach comes after it
    assert segs[-1].kind == "translate" and segs.index(rot) == len(segs) - 2
    assert r.stats["rotations"] == 1 and r.stats["length_mm"] > 0


def test_zero_clearance_still_reports_a_crossing(model):
    """Review 2026-10-05: with clearance 0 the swept area of a translation was never checked (need = 0), so a straight
    move through a leg passed. Now an overlap of the swept area counts whenever both ends are clear of it."""
    ares, obst = model["ares"], model["obst"]
    from mauer.reference import Pose2D
    a, b = Pose2D(1200.0, -700.0, math.pi / 2), Pose2D(1200.0, 900.0, math.pi / 2)     # through leg A and plate W3
    probs = floor.validate_route([a, b], obst, ares, 0.0, min_mm=0.0, min_deg=0.0)
    assert any("leg A" in p for p in probs) and any("plate W3" in p for p in probs)
    s0 = model["stops"]["A1"]                                                         # along the stop line: clear
    along = Pose2D(s0.x_mm - 300.0, s0.y_mm, s0.theta_rad)
    assert floor.validate_route([s0, along], obst, ares, 0.0) == []
    sq = floor.box_poly(0, 0, 100, 100)
    assert floor.penetration(sq, floor.box_poly(90, 20, 200, 80)) == pytest.approx(10.0)
    assert floor.penetration(sq, floor.box_poly(100, 0, 200, 100)) == 0.0


def test_bad_routes_fail(model):
    ares, obst, clr = model["ares"], model["obst"], model["clr"]
    s, g_ = model["stops"]["A1"], model["stops"]["B0"]
    th = s.theta_rad
    # 1. straight across the corner: translation AND rotation in one leg, through the legs
    p = "\n".join(floor.validate_route([s, g_], obst, ares, clr))
    assert "translates" in p and "AND rotates" in p
    # 2. rotate right at the stop: the swept circle hits leg A and the plates
    p = "\n".join(floor.validate_route([s, Pose2D(s.x_mm, s.y_mm, th - 90 * DEG)], obst, ares, clr))
    assert "swept circle" in p and "leg A" in p and "plate" in p
    # 3. from the B stop forward (towards leg B) by 1 m: the swept rectangle runs through plates and leg B, and
    #    a route that ends there with the footprint over leg B
    into = Pose2D(g_.x_mm - 1000.0, g_.y_mm, g_.theta_rad)
    p = "\n".join(floor.validate_route([g_, Pose2D(g_.x_mm + 200.0, g_.y_mm, g_.theta_rad), into], obst, ares, clr))
    assert "overlaps leg B" in p and "overlaps plate W6" in p
    #    ... and straight through leg B to the inside of the L (both end points free): the swept hull catches it
    through = Pose2D(g_.x_mm - 2000.0, g_.y_mm, g_.theta_rad)
    p = "\n".join(floor.validate_route([g_, through], obst, ares, clr))
    assert "passes 0 mm from leg B" in p and "overlaps" not in p
    # 4. an intermediate waypoint 30 mm from the plates (< 50 mm clearance), then away again
    near = Pose2D(s.x_mm, s.y_mm + 20.0, th)
    far = Pose2D(s.x_mm, s.y_mm + 300.0, th)
    p = "\n".join(floor.validate_route([s, near, far], obst, ares, clr))
    assert "waypoint 1" in p and "< clearance" in p
    # 5. below the PLC minimum move and a duplicate waypoint
    p = "\n".join(floor.validate_route([s, Pose2D(s.x_mm + 1.0, s.y_mm, th), Pose2D(s.x_mm + 1.0, s.y_mm, th)],
                                       obst, ares, clr))
    assert "below the PLC minimum" in p and "duplicate waypoint" in p
    # 6. ARES footprint overlapping a leg at the goal
    p = "\n".join(floor.validate_route([s, Pose2D(s.x_mm, 200.0, th)], obst, ares, clr))
    assert "overlaps leg A" in p
    # the back-off from the stop (10 mm from the plates, moving away) is fine on its own
    assert floor.validate_route([s, Pose2D(s.x_mm, s.y_mm + 100.0, th)], obst, ares, clr) == []


def test_planner_fails_cleanly_without_room(model):
    s, g_ = model["stops"]["A1"], model["stops"]["B0"]
    box = floor.Obstacle("wall around", "leg", floor.box_poly(s.x_mm - 400, s.y_mm + 650, s.x_mm + 4000, s.y_mm + 700))
    tight = model["obst"] + [box, floor.Obstacle("box", "leg", floor.box_poly(g_.x_mm + 600, -3000, g_.x_mm + 650,
                                                                                 3000))]
    with pytest.raises(floor.RouteError):
        floor.plan_route(s, g_, tight, model["ares"], model["clr"], 100.0, search_mm=500.0, grid_mm=200.0)


def test_route_without_rotation_and_collinear_merge(model):
    a = Pose2D(0.0, 3000.0, 0.0)
    b = Pose2D(1000.0, 3000.0, 0.0)
    r = floor.plan_route(a, b, model["obst"], model["ares"], model["clr"], 100.0)
    assert len(r.waypoints) == 2 and r.stats["translations"] == 1
    m = floor._dedupe([a, Pose2D(400.0, 3000.0, 0.0), b])
    assert m == [a, b]


# ── plates ────────────────────────────────────────────────────────────────────
def test_config_boards_sit_on_block_centres_and_are_clear(cfg, model):
    assert model["problems"] == []
    specs = board_specs(cfg)
    walls = [t for t in cfg["targets"] if t["parent"] == "wall"]
    assert len(walls) == 8 and {t["name"] for t in walls} == {f"W{i}" for i in range(8)}
    assert [t["first_id"] for t in walls] == list(range(30, 110, 10))         # existing boards, unchanged ids
    for s in model["sites"]:
        assert s.u == pytest.approx(100.0 + 200.0 * s.k)
        assert not s.spare and 0 <= s.k < next(lg.n0 for lg in model["legs"] if lg.name == s.leg)
        T = config.leg_frames(cfg)[s.leg]
        from mauer.reference import placements
        pl = next(p for p in placements(cfg) if p.name == s.board)
        import numpy as np
        from mauer import geometry as g
        assert np.allclose(pl.T_parent_board, T @ g.pose_xyz_rpy(s.xyz, s.rpy_deg))
        c = g.apply(pl.T_parent_board, [[*specs[s.board].centre_mm, 0.0]])[0]
        assert floor.point_inside((c[0], c[1]), s.plate)                     # board centre on its plate
    # every stop of the job sees >= 2 boards (kinematic) - checked when building the job; here: plates never under
    # ARES at a stop and >= 2 boards within the look window (|d| < 1000 mm) of every stop
    job = make_job.build_nominal(cfg)
    for st in job.stops:
        near = [s for s in model["sites"]
                if math.hypot(sum(p[0] for p in s.plate) / 4 - st.ares.x_mm, sum(p[1] for p in s.plate) / 4 -
                              st.ares.y_mm) < 1600.0]
        assert len(near) >= 2


def test_bad_plate_sites_are_reported(cfg, model):
    legs = model["legs"]
    A, B = legs
    size = board_specs(cfg)["W0"].size_mm
    leg_polys = {lg.name: wp.leg_footprint(cfg, lg) for lg in legs}
    stops = [(k, model["ares"].footprint(p)) for k, p in model["stops"].items()]
    # neighbouring blocks: the 220 mm plates overlap each other
    p = floor.check_sites([floor.plate_site(cfg, A, 3, size), floor.plate_site(cfg, A, 4, size)], leg_polys, stops)
    assert any("overlap" in x for x in p)
    # a spare block before B's start lies on leg A
    p = floor.check_sites([floor.plate_site(cfg, B, -1, size)], leg_polys, stops)
    assert any("spare block overlaps leg A" in x for x in p)
    # a spare block beyond A's free start is fine
    assert floor.check_sites([floor.plate_site(cfg, A, -1, size)], leg_polys, stops) == []
    # a target off the block grid is refused
    t = copy.deepcopy(next(t for t in cfg["targets"] if t["name"] == "W1"))
    t["xyz"][0] += 30.0
    with pytest.raises(ValueError, match="not a plate on a block centre"):
        floor.site_from_target(cfg, A, t, size)
