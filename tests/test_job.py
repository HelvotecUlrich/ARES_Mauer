"""Job format (mauer.job v2, v1 still loaded) and the nominal job builder tools/make_job.py (config +
robodk/wallplan.py + results/reach_table.json, no RoboDK).

Straight wall (config without legs, conftest.straight_config): the plan is wallplan.sequence on the reach table, as in
robodk/simulate.py. With 3 courses that was 69 stones, 4 stops at a = 920 + k * 1400 mm (README / simulate.log); with
4 courses (Samuel 2026-10-05, reach table recomputed in RoboDK for 4 courses) it is 90 stones, 4 stops at
a = 920 + k * 1200 mm with 30, 24, 24, 12 stones (simulate.py itself not re-run yet). Magazine 15 usable slots
(simulate.py: "15 reachable"). The L of the config is tested in the second half.
"""
import copy
import importlib.util
import json
import math

import numpy as np
import pytest

from conftest import l_config, straight_config
from mauer import REPO, config
from mauer import floor
from mauer import geometry as g
from mauer import job as mjob
from mauer.reference import Pose2D, placements, relative_move
from mauer.simworld import ur5_fk
from mauer.vision.targets import board_specs


def _tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


make_job = _tool("make_job")


@pytest.fixture(scope="module")
def cfg():
    return straight_config()


@pytest.fixture(scope="module")
def job(cfg):
    return make_job.build_nominal(cfg, length=24)


def test_plan_matches_the_robodk_simulation(job, cfg):
    assert [len(s.stones) for s in job.stops] == [30, 24, 24, 12]
    assert [s.a_mm for s in job.stops] == pytest.approx([920.0, 2120.0, 3320.0, 4520.0])
    assert job.n_stones == 90 and job.meta["n_stones"] == 90 and job.legs == []
    assert all(s.leg is None and not s.route and not s.route_to_station for s in job.stops)
    dist = cfg["wall"]["dist_nominal"]
    for s in job.stops:                       # front wall: ARES faces the wall (-wall y) at y = dist
        assert (s.ares.x_mm, s.ares.y_mm, s.ares.theta_deg) == pytest.approx((s.a_mm, dist, -90.0), abs=1e-9)
    dx, dy, dth = relative_move(job.stops[0].ares, job.stops[1].ares)
    assert (dx, dy, dth) == pytest.approx((0.0, 1200.0, 0.0), abs=1e-9)      # sideways, +y body = left
    assert job.magazine.capacity == 15
    assert sorted(set(s.id for s in job.magazine.slots) - set(job.magazine.take_order)) == \
        ["r0y0l3", "r0y1l3", "r0y2l3"]


def test_place_pose_convention(job):
    """rdk_common.place_pose: wall_frame * transl(u, 0, z_top) * rotx(pi) (no flip in a nominal job)."""
    for t in job.stones():
        assert not t.flip
        assert np.allclose(t.T_wall_tcp, g.transl(t.u_mm, 0.0, t.z_top_mm) @ g.rotx(math.pi), atol=1e-12)
    tops = sorted({t.z_top_mm for t in job.stones()})
    assert tops == pytest.approx([140.0, 260.0, 380.0, 500.0])     # base 20 + k * 120 + 120 ([wall], [brick])
    assert all(t.kind == "full" and t.leg is None and t.key == (t.course, t.index) for t in job.stones())


def test_magazine_pick_poses_and_order(job, cfg):
    """robodk/simulate.py:106-108: transl(x, y, deck + layer H) @ rotz(pi/2) @ rotx(pi), emptied top layer first."""
    deck = cfg["ares"]["deck_top_z"] + cfg["deck"]["holder_z"]
    x0 = cfg["ur5"]["mount_x"]
    s = job.magazine.slot("r1y2l2")
    assert np.allclose(s.T_ares_tcp, g.transl(x0 - 433.6, 205.0, deck + 2 * 120.0) @ g.rotz(math.pi / 2)
                       @ g.rotx(math.pi), atol=1e-9)
    layers = [job.magazine.slot(i).layer for i in job.magazine.take_order]
    assert layers == sorted(layers, reverse=True)
    assert job.magazine.fill_order == list(reversed(job.magazine.take_order))
    assert all(t.slot in job.magazine.take_order for t in job.stones())


def test_look_poses_aim_the_camera_at_the_boards(job, cfg):
    specs = board_specs(cfg)
    pl = {p.name: p for p in placements(cfg)}
    X = config.T_flange_cam_nominal(cfg)
    T_ab = config.T_ares_base(cfg)
    work = cfg["camera"]["working_dist"]
    for s in job.stops:
        assert 1 <= len(s.looks) <= 2
        T_base_wall = g.inv(T_ab) @ g.inv(s.ares.T)
        for lk in s.looks:
            (b,) = lk.boards
            assert pl[b].parent == "wall"
            T_cam_board = g.inv(lk.T_base_flange @ X) @ T_base_wall @ pl[b].T_parent_board
            c = g.apply(T_cam_board, [[*specs[b].centre_mm, 0.0]])[0]
            assert c == pytest.approx([0.0, 0.0, work], abs=1e-6)            # centre on the axis at working_dist
            assert T_cam_board[2, 2] == pytest.approx(1.0)                   # fronto-parallel
            assert np.allclose(ur5_fk(lk.qnear_rad), lk.T_base_flange, atol=1e-6)   # the hint reaches the pose
    assert len(job.stops[0].looks) == 2
    assert job.station.looks and all(lk.boards[0] in ("S0", "S1") for lk in job.station.looks)


def test_station_and_park(job, cfg):
    st = job.station
    ps = cfg["pickup_station"]
    assert np.allclose(st.T_wall_station, g.pose_xyz_rpy(ps["xyz_in_wall"], ps["rpy_in_wall_deg"]))
    assert (st.dock.x_mm, st.dock.y_mm, st.dock.theta_deg) == pytest.approx((*ps["ares_xyz"][:2], ps["ares_rpy_deg"][2]))
    n_full = len(ps["slots_xy"]) * ps["slot_layers"]
    n_half = len(ps["half_slots_xy"]) * ps["half_slot_layers"]
    assert len(st.slots) == n_full + n_half and sum(s.kind == "half" for s in st.slots) == n_half
    # the PLACEHOLDER layout of 2026-10-06 was laid out from the kinematic reach at the dock: every holder is usable
    assert sorted(st.take_order) == sorted(s.id for s in st.slots) and all(s.ik_ok for s in st.slots)
    assert not any("station slot" in w for w in job.meta["warnings"])
    for s in st.slots:
        h = cfg["brick"]["height"] if s.kind == "full" else cfg["half_brick"]["height"]
        assert s.T_station_tcp[2, 3] == pytest.approx(ps["table_z"] + ps["holder_z"] + s.layer * h)
        x, y = (ps["slots_xy"] if s.kind == "full" else ps["half_slots_xy"])[int(s.stack_id[1:])]
        assert s.T_station_tcp[:2, 3] == pytest.approx([x, y]) and s.id == f"{s.stack_id}l{s.layer}"
    layers = [st.slot(i).layer for i in st.take_order]
    assert layers == sorted(layers, reverse=True)            # emptied top layer first
    T_park = ur5_fk(job.park_q_rad)
    assert T_park[2, 2] == pytest.approx(-1.0, abs=1e-6)     # tool down
    expected = "[ur] park_q_deg" if "park_q_deg" in cfg["ur"] else "simulate.py"
    assert expected in job.meta["park_q_source"]


def test_provenance(job, cfg):
    assert job.config_sha256 == mjob.config_sha256()        # provenance always from the file on disk
    keys = {d["key"]: d for d in job.depends_on}
    assert keys["[ur5] mount_z"]["status"] == "PLACEHOLDER" and keys["[ur5] mount_z"]["value"] == 333.6
    assert keys["[tool] tcp_z"]["status"] == "ASSUMPTION"
    assert keys["[brick] mass_kg"]["status"] == "UNKNOWN"
    assert keys["[[targets]] W0.xyz"]["status"] == "PLACEHOLDER"
    assert "[ares] steer_axis_x" not in keys and "[brick] length" not in keys        # CONFIRMED keys are not listed
    assert job.meta["reach_check"] == "kinematic" and job.meta["look_source"] == "nominal"


def test_round_trip(job, tmp_path):
    p = mjob.save(job, tmp_path / "job.json")
    back = mjob.load(p)
    assert json.dumps(mjob.to_dict(back), sort_keys=True) == json.dumps(mjob.to_dict(job), sort_keys=True)
    assert back.stops[2].stones[3].key == job.stops[2].stones[3].key
    assert isinstance(back.stops[0].ares, Pose2D) and np.allclose(back.T_flange_tcp, job.T_flange_tcp)


def test_validation_lists_every_problem(job):
    bad = copy.deepcopy(job)
    bad.stops[0].stones[0].T_wall_tcp = bad.stops[0].stones[0].T_wall_tcp @ np.diag([1.0, 1.0, 1.01, 1.0])
    bad.stops[0].stones[1].slot = "nope"
    bad.stops[1].stones.append(copy.deepcopy(bad.stops[0].stones[2]))          # duplicate stone
    bad.stops[1].looks[0].T_base_flange = None
    bad.stops[2].stones[0].T_wall_tcp = g.transl(1, 2, 3)                      # TCP z up: not a place pose
    bad.park_q_rad = [0.0] * 5
    bad.magazine.initial_fill.append("ghost")
    problems = mjob.validate(bad, boards=[p.name for p in placements(config.load())])
    text = "\n".join(problems)
    for frag in ("not a rigid transform", "unknown magazine slot 'nope'", "duplicate stone",
                 "needs T_base_flange or q_rad", "TCP z must point down", "park_q_rad: need 6",
                 "initial_fill: unknown slot ids ['ghost']"):
        assert frag in text, frag
    with pytest.raises(mjob.JobError):
        mjob.save(bad, "unused.json")
    assert mjob.validate(job) == []


def test_from_dict_rejects_other_formats(job):
    d = mjob.to_dict(job)
    with pytest.raises(mjob.JobError, match="version"):
        mjob.from_dict({**d, "version": 99})
    with pytest.raises(mjob.JobError, match="format"):
        mjob.from_dict({**d, "format": "something"})
    broken = copy.deepcopy(d)
    del broken["stops"][0]["ares"]
    with pytest.raises(mjob.JobError, match="malformed"):
        mjob.from_dict(broken)


def test_slot_state_respects_stacking(job):
    mag = mjob.SlotState.magazine(job.magazine)
    assert len(mag) == mag.capacity == 15
    assert not mag.can_take("r1y0l1") and mag.can_take("r1y0l3")             # covered / top
    with pytest.raises(ValueError):
        mag.take("r1y0l2")
    taken = [mag.next_take() for _ in range(1)]
    mag.take(taken[0])
    assert job.magazine.slot(taken[0]).layer == 3
    while not mag.empty():
        mag.take(mag.next_take())
    first = mag.next_fill()
    assert job.magazine.slot(first).layer == 1                               # refilled bottom first
    assert not mag.can_fill("r1y0l2")
    st = mjob.SlotState.station(job.station)
    assert len(st) == len(job.station.take_order) and st.next_take() == job.station.take_order[0]


def test_station_stacks(job, cfg):
    """Stacked station holders: a stone can only be taken when nothing lies on it, the next stone of a type is the
    first takeable one in take order."""
    stn = job.station
    st = mjob.SlotState.station(stn)
    low, top = stn.slot("s00l1"), stn.slot("s00l2")
    assert low.stack_id == top.stack_id == "s00" and (low.layer, top.layer) == (1, 2)
    assert top.T_station_tcp[2, 3] - low.T_station_tcp[2, 3] == pytest.approx(cfg["brick"]["height"])
    assert not st.can_take("s00l1") and st.can_take("s00l2")
    with pytest.raises(ValueError):
        st.take("s00l1")
    taken = []
    while st.next_take(kind="full"):
        sid = st.next_take(kind="full")
        assert all(o not in st.filled for o in st.stack if st.stack[o] == st.stack[sid] and st.layer[o] > st.layer[sid])
        st.take(sid)
        taken.append(sid)
    assert len(taken) == sum(s.kind == "full" for s in stn.slots)
    assert all(stn.slot(i).layer == 2 for i in taken[:8]) and all(stn.slot(i).layer == 1 for i in taken[8:])


def test_validate_station_stacks(job):
    bad = copy.deepcopy(job)
    bad.station.slot("s01l2").layer = 3                                       # gap in stack s01
    bad.station.slot("s02l2").kind = "half"                                   # a half stone on a full one
    bad.station.take_order.remove("s03l1")                                    # s03l2 would float
    text = "\n".join(mjob.validate(bad))
    for frag in ("station stack s01: layers [1, 3]", "station stack s02: mixed stone types",
                 "station slot s03l2: in take_order but the slot(s) below it are not (['s03l1'])"):
        assert frag in text, frag


def test_config_status_parser():
    st = mjob.config_status()
    assert st["[ares] steer_axis_x"]["status"] == "CONFIRMED"
    assert st["[ur] host"]["status"] == "PLACEHOLDER"
    assert st["[camera.mount] xyz"]["status"] == "ASSUMPTION"               # inline tag (design value 2026-10-05)
    assert st["[[targets]] W3.xyz"]["status"] == "PLACEHOLDER"              # from the [[targets]] block comment
    assert st["[[targets]] S0.xyz"]["status"] == "PLACEHOLDER"              # own inline tag
    assert st["[brick] mass_kg"]["status"] == "UNKNOWN"


def test_make_job_cli(tmp_path, capsys):
    out = tmp_path / "j.json"
    assert make_job.main(["--length", "6", "--out", str(out)]) == 0      # straight wall with the config's boards
    j = mjob.load(out)
    assert j.meta["length_stones"] == 6 and j.n_stones == 6 + 5 + 4 + 3
    assert "written" in capsys.readouterr().out


def test_stale_reach_table_is_refused(cfg, tmp_path):
    t = tmp_path / "reach.json"
    from mauer import reach_cache
    table = reach_cache.get(cfg, cfg["wall"]["dist_nominal"])
    t.write_text(json.dumps({"key": "000000000000", "dist": cfg["wall"]["dist_nominal"], "table": table}))   # v1
    with pytest.raises(ValueError, match="another configuration"):
        make_job.build_nominal(cfg, 6, reach_table_path=t)
    assert make_job.build_nominal(cfg, 6, reach_table_path=t, allow_stale_reach=True).n_stones == 18


# ── the L of the config (job v2) ──────────────────────────────────────────────
@pytest.fixture(scope="module")
def lcfg():
    return l_config()


@pytest.fixture(scope="module")
def ljob(lcfg):
    return make_job.build_nominal(lcfg)                       # shape from the config: the L


def test_l_job_stops_and_stones(ljob, lcfg):
    wp = make_job.load_wallplan()
    legs = {lg.name: lg for lg in wp.legs(lcfg)}
    assert [lg["name"] for lg in ljob.legs] == ["A", "B"] and ljob.version == 2
    assert [s.leg for s in ljob.stops] == ["A", "A", "B"]                  # leg A completely, then leg B
    n = {k: lg.n0 for k, lg in legs.items()}
    assert ljob.n_stones == sum(4 * n0 + 2 for n0 in n.values())         # rectangle: 2 n + 2 (n + 1)
    dist = lcfg["wall"]["dist_nominal"]
    T_legs = config.leg_frames(lcfg)
    for s in ljob.stops:                                                   # ARES faces its leg from the outside
        T = g.inv(T_legs[s.leg]) @ s.ares.T
        assert (T[0, 3], T[1, 3]) == pytest.approx((s.a_mm, dist), abs=1e-9)
        assert math.atan2(T[1, 0], T[0, 0]) == pytest.approx(-math.pi / 2, abs=1e-12)
    assert ljob.stops[0].ares.theta_deg == pytest.approx(-90.0) and abs(ljob.stops[2].ares.theta_deg) == \
        pytest.approx(180.0)
    for t in ljob.stones():                                                # place poses in the WALL frame
        T_exp = T_legs[t.leg] @ g.transl(t.u_mm, 0.0, t.z_top_mm) @ g.rotx(math.pi)
        assert np.allclose(t.T_wall_tcp if not t.flip else t.T_wall_tcp @ g.rotz(math.pi), T_exp, atol=1e-9)
        assert t.key == (t.leg, t.course, t.index)
    for leg, lg in legs.items():
        L = wp.leg_length(lcfg, lg.n0)
        for k in range(lcfg["wall"]["courses"]):
            row = sorted((t for t in ljob.stones() if t.leg == leg and t.course == k), key=lambda t: t.u_mm)
            kinds = [t.kind for t in row]
            if k % 2:
                assert kinds[0] == kinds[-1] == "half" and set(kinds[1:-1]) == {"full"} and len(row) == lg.n0 + 1
            else:
                assert set(kinds) == {"full"} and len(row) == lg.n0
            lo = row[0].u_mm - row[0].length_mm / 2
            hi = row[-1].u_mm + row[-1].length_mm / 2
            assert (lo, hi) == pytest.approx((0.0, L))                     # vertical leg ends in every course
    assert ljob.meta["n_half_stones"] == 8 and ljob.meta["shape"] == "L"


def test_l_corner_is_a_butt_joint(lcfg):
    """Leg B is perpendicular at A's far end: A's course-0 end flush with the body of B's outer face, B starts at A's
    inside face beyond A's ribs and the corner gap ([brick] rib_mm, [wall] corner_gap_mm), from the config (no
    hard-coded 200/60). Over the ribs no stone of one leg touches the other leg; with B at A's BODY face (the corner of
    the first L plan) the first B stone of every course would overlap A's ribs - check_legs reports it."""
    wp = make_job.load_wallplan()
    A, B = wp.legs(lcfg)
    exp = wp.butt_corner(lcfg, A, B.n0, "B")
    assert (B.x, B.y, B.theta) == pytest.approx((exp.x, exp.y, exp.theta), abs=1e-9)
    L, W = lcfg["brick"]["length"], lcfg["brick"]["width"]
    rib, gap = lcfg["brick"]["rib_mm"], lcfg["wall"]["corner_gap_mm"]
    assert rib == pytest.approx(1.69) and gap > 0
    assert (B.x, B.y, math.degrees(B.theta)) == pytest.approx((L * A.n0 - W / 2, -(W / 2 + rib + gap), -90.0))
    fa, fb = wp.leg_footprint(lcfg, A), wp.leg_footprint(lcfg, B)
    assert max(p[0] for p in fa) == pytest.approx(max(p[0] for p in fb))   # flush outer face (bodies)
    assert min(p[1] for p in fa) - max(p[1] for p in fb) == pytest.approx(rib + gap)
    stones = wp.layout_legs(lcfg, [A, B])
    assert wp.check_legs(lcfg, [A, B], stones) == []
    old = wp.Leg("B", B.n0, B.x, -W / 2, B.theta)                           # B at A's body face: on A's ribs
    errs = wp.check_legs(lcfg, [A, old], wp.layout_legs(lcfg, [A, old]))
    for k in range(lcfg["wall"]["courses"]):                                 # B's first stone in every course
        assert any(f"('B', {k}, 0) overlap (course {k})" in e for e in errs), k


def test_l_looks_use_boards_of_the_stops_own_leg_clear_of_the_built_wall(ljob, lcfg):
    """Review 2026-10-05: the stop-2 look at W4 (leg A) put wrist 1 / wrist 2 / the flange inside the finished leg A,
    the stop-1 look at W5 hit A's end - a kinematic check alone accepted both. Now every look is on a board of the
    stop's own leg and clear of the stones built by the end of its stop (mauer.armcheck); the old cross-leg looks
    are rejected by the same check."""
    from mauer import armcheck
    pl = {p.name: p for p in placements(lcfg)}
    specs = board_specs(lcfg)
    X = config.T_flange_cam_nominal(lcfg)
    T_ab = config.T_ares_base(lcfg)
    leg_of = {t["name"]: t.get("leg") for t in lcfg["targets"] if t["parent"] == "wall"}
    wp = make_job.load_wallplan()
    by_name = {lg.name: lg for lg in wp.legs(lcfg)}
    stones = {s.key: s for s in wp.layout_legs(lcfg, list(by_name.values()))}
    built = []
    for s in ljob.stops:
        built += [stones[t.key] for t in s.stones]
        arm = armcheck.ArmChecker(lcfg, armcheck.stone_boxes(lcfg, built, by_name))
        assert len(s.looks) == 2
        T_base_wall = g.inv(T_ab) @ g.inv(s.ares.T)
        for lk in s.looks:
            (b,) = lk.boards
            assert pl[b].parent == "wall" and leg_of[b] == s.leg
            T_cam_board = g.inv(lk.T_base_flange @ X) @ T_base_wall @ pl[b].T_parent_board
            assert g.apply(T_cam_board, [[*specs[b].centre_mm, 0.0]])[0] == pytest.approx(
                [0.0, 0.0, lcfg["camera"]["working_dist"]], abs=1e-6)
            assert arm.hits(lk.qnear_rad, s.ares.T @ T_ab) == []
    assert [[lk.boards[0] for lk in s.looks] for s in ljob.stops] == [["W0", "W3"], ["W2", "W4"], ["W5", "W7"]]
    # the rejected looks of the first L plan: W4 from stop 2 and W5 from stop 1, once leg A is complete
    ctx = make_job._Ctx(lcfg, lcfg["wall"]["dist_nominal"], 2, make_job.LOOK_MARGIN_MM)
    T_legs = config.leg_frames(lcfg)
    leg_a = [st for st in stones.values() if st.leg == "A"]
    for k, b in ((2, "W4"), (1, "W5")):
        st = ljob.stops[k]
        free, _ = make_job.wall_look_candidates(ctx, st.ares, T_legs[st.leg], None)            # kinematic only
        built_, _ = make_job.wall_look_candidates(ctx, st.ares, T_legs[st.leg], None, leg_a, by_name)
        assert b in free and b not in built_


def test_l_routes_are_valid(ljob, lcfg):
    wp = make_job.load_wallplan()
    legs = wp.legs(lcfg)
    obst, problems = make_job.floor_model(lcfg, legs, make_job.target_sites(lcfg, legs),
                                          [(f"stop {s.index}", s.ares) for s in ljob.stops])
    assert problems == []
    ares = floor.AresShape.from_config(lcfg)
    clr = lcfg["routes"]["clearance_mm"]
    dock = ljob.station.dock_in_wall
    assert not ljob.stops[0].route
    m = ljob.stops[1].route                                                  # same leg: back off, along, approach
    segs, sp = floor.segments(m)
    assert sp == [] and [sg.kind for sg in segs] == ["translate"] * 3
    assert floor.validate_route(m, obst, ares, clr) == []
    for w in m[1:-1]:                                                       # real clearance along the leg
        assert min(floor.poly_dist(ares.footprint(w), o.poly) for o in obst) >= clr - 1e-6
    r = ljob.stops[2].route                                                  # leg change A -> B
    assert r[0].delta(ljob.stops[1].ares) == pytest.approx((0, 0), abs=1e-9)
    assert r[-1].delta(ljob.stops[2].ares) == pytest.approx((0, 0), abs=1e-9)
    segs, sp = floor.segments(r)
    assert sp == [] and sum(sg.kind == "rotate" for sg in segs) == 1
    assert floor.validate_route(r, obst, ares, clr) == []
    rot = next(sg for sg in segs if sg.kind == "rotate")
    assert math.degrees(abs(rot.angle_rad)) == pytest.approx(90.0)
    for o in obst:                                                          # the rotation circle keeps its clearance
        assert floor.circle_dist((rot.start.x_mm, rot.start.y_mm), ares.radius, o.poly) >= clr - 1e-6
    assert segs[-1].kind == "translate" and segs[-1].length_mm < 300.0     # rotate right before the approach
    # the approach of every route to a stop is longer than the arrival standoff (the sequencer stops that far before)
    so = lcfg["sequencer"]["arrival_standoff_mm"]
    assert ljob.meta["route_check"]["arrival_standoff_mm"] == so > 0
    for s in ljob.stops:
        for route, a, b in ((s.route_to_station, s.ares, dock), (s.route_from_station, dock, s.ares)):
            assert route[0].delta(a)[0] < 1e-6 and route[-1].delta(b)[0] < 1e-6
            assert floor.validate_route(route, obst, ares, clr) == []
        for route in (s.route, s.route_from_station):
            if route:
                last = floor.segments(route)[0][-1]
                assert last.kind == "translate" and last.length_mm > so
    assert ljob.meta["route_check"]["clearance_mm"] == clr
    # mauer.floor.job_obstacles (sequencer, simulation) is the same floor model
    jo = floor.job_obstacles(lcfg, ljob.legs, ljob.station.T_wall_station,
                             {n: sp_.size_mm for n, sp_ in board_specs(lcfg).items()})
    assert [(o.name, o.kind) for o in jo] == [(o.name, o.kind) for o in obst]
    for a, b in zip(jo, obst):
        assert np.allclose(a.poly, b.poly, atol=1e-6)


def test_l_job_without_routes_is_refused(ljob, lcfg, tmp_path):
    """Review 2026-10-05: an L job without routes passed validate() and the preflight and the sequencer fell back to
    the v1 direct moves (rotating over the plates, driving through both legs)."""
    from mauer.sequencer import preflight_real, route_problems
    bad = copy.deepcopy(ljob)
    for st in bad.stops:
        st.route, st.route_to_station, st.route_from_station = [], [], []
    bad.meta.pop("route_check")
    text = "\n".join(mjob.validate(bad))
    assert "stop 2: leg change A -> B without a route" in text
    assert all(f"stop {i}: no route_to_station" in text for i in range(3))
    pf = "\n".join(preflight_real(lcfg, bad, intrinsics_file=tmp_path / "i", handeye_file=tmp_path / "h"))
    assert "without a recorded route check" in pf and "no station route" in pf
    assert route_problems(lcfg, ljob) == []                                  # the planned job is clean
    moved = copy.deepcopy(ljob)                                               # a plate moved under a route
    r = moved.stops[2].route
    r[2] = Pose2D(r[2].x_mm, r[2].y_mm - 900.0, r[2].theta_rad)
    r[3] = Pose2D(r[3].x_mm, r[3].y_mm - 900.0, r[3].theta_rad)
    assert any("stop 2 route" in x for x in route_problems(lcfg, moved))


def test_l_round_trip_and_v1_still_loads(ljob, job, tmp_path):
    p = mjob.save(ljob, tmp_path / "l.json")
    back = mjob.load(p)
    assert json.dumps(mjob.to_dict(back), sort_keys=True) == json.dumps(mjob.to_dict(ljob), sort_keys=True)
    assert back.stops[2].route == ljob.stops[2].route and back.stops[0].route_to_station
    assert back.magazine.initial_kinds == ljob.magazine.initial_kinds
    assert [t.kind for t in back.stones()] == [t.kind for t in ljob.stones()]
    assert {s.kind for s in back.station.slots} == {"full", "half"}
    # a version-1 file (no legs, kinds, routes) loads with the v1 defaults and is written back as v1
    d = mjob.to_dict(job)
    d["version"] = 1
    for st in d["stops"]:
        for k in ("leg", "route", "route_to_station", "route_from_station"):
            st.pop(k)
        for t in st["stones"]:
            for k in ("leg", "kind", "length_mm"):
                t.pop(k)
    d["magazine"].pop("initial_kinds")
    d.pop("legs")
    for s in d["station"]["slots"]:
        for k in ("kind", "layer", "stack"):
            s.pop(k)
    v1 = mjob.from_dict(d)
    assert v1.version == 1 and mjob.validate(v1) == [] and all(t.kind == "full" for t in v1.stones())
    assert all(not s.route for s in v1.stops) and mjob.to_dict(v1) == d
    assert all(s.layer == 1 and s.stack_id == s.id for s in v1.station.slots)   # v1: single holders
    # a v2 file written before the stacked station (no layer / stack) loads as single holders too
    d2 = mjob.to_dict(ljob)
    for s in d2["station"]["slots"]:
        s.pop("layer"), s.pop("stack")
    old = mjob.from_dict(d2)
    assert all(s.layer == 1 and s.stack_id == s.id for s in old.station.slots)
    st = mjob.SlotState.station(old.station)
    assert all(st.can_take(i) for i in old.station.take_order)


def test_l_validation_catches_bad_routes(ljob):
    bad = copy.deepcopy(ljob)
    r = bad.stops[2].route
    r[2] = Pose2D(r[2].x_mm, r[2].y_mm, r[2].theta_rad + 0.3)               # translate AND rotate in one leg
    bad.stops[0].route_to_station[-1] = Pose2D(0.0, 0.0, 0.0)               # does not end at the dock
    bad.stops[1].stones[0].kind = "quarter"
    text = "\n".join(mjob.validate(bad))
    for frag in ("translation AND rotation", "route_to_station: end", "kind 'quarter'"):
        assert frag in text, frag


def test_l_magazine_types_follow_the_stones(ljob):
    """The magazine is filled with the types of the next stones in take order (mauer.job.reload_plan) - replaying
    the plan gives every stone a slot holding its type."""
    mag = mjob.SlotState.magazine(ljob.magazine)
    full = mjob.SlotState.station(ljob.station)
    st = full.copy()
    stones = ljob.stones()
    reloads = 0
    for j, t in enumerate(stones):
        if mag.empty():
            if mjob.reload_short(st, full, mag, [x.kind for x in stones[j:]]):
                st = full.copy()
            for ssid, mid, kind in mjob.reload_plan(mag, st, [x.kind for x in stones[j:]]):
                st.take(ssid)
                mag.fill(mid, kind)
            reloads += 1
        sid = mag.next_take(t.slot, kind=t.kind)
        assert sid == t.slot                                                # the planned slot holds the right type
        assert mag.take(sid) == t.kind
    assert reloads == ljob.meta["planned_reloads"] > 0
    first = [t.kind for t in stones[:len(ljob.magazine.initial_fill)]]
    assert sorted(ljob.magazine.initial_kinds.values()) == sorted(first)


def test_reload_plan_unit(ljob):
    mag = mjob.SlotState.magazine(ljob.magazine, filled=[], kinds={})
    st = mjob.SlotState.station(ljob.station)
    n_half = st.count("half")
    up = ["half", "full", "full"] + ["half"] * n_half + ["full"]
    plan = mjob.reload_plan(mag, st, up)
    # the station holds n_half half stones -> the batch ends before the (n_half + 1)-th half stone
    n = 3 + n_half - 1
    assert len(plan) == n and sorted(k for *_, k in plan) == sorted(up[:n])
    assert [mid for _, mid, _ in plan] == mag.fill_order[:n]                  # fill order (bottom first)
    for ssid, mid, kind in plan:
        assert ljob.station.slot(ssid).kind == kind
        st.take(ssid)                                                        # raises if a stone lies on it
        mag.fill(mid, kind)
    got = []
    for k in up[:n]:
        sid = mag.next_take(kind=k)
        assert sid == mag.next_take()                                        # the first takeable slot holds it
        got.append(mag.take(sid))
    assert got == up[:n]
    with pytest.raises(ValueError):
        mjob.reload_plan(mjob.SlotState.magazine(ljob.magazine), st, up)    # magazine not empty


def test_make_job_cli_builds_the_l(tmp_path, capsys):
    out = tmp_path / "l.json"
    assert make_job.main(["--out", str(out)]) == 0
    j = mjob.load(out)
    assert j.legs and j.meta["shape"] == "L" and "leg change" in json.dumps(j.meta["route_check"])
    assert "legs: A" in capsys.readouterr().out
