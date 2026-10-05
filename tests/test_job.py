"""Job format v1 (mauer.job) and the nominal job builder tools/make_job.py (config + robodk/wallplan.py +
results/reach_table.json, no RoboDK).

The wall plan must match the RoboDK simulation (README / results/simulate.log, `simulate.py --length 24`): 69 stones,
4 stops at a = 920 + k * 1400 mm with 24, 21, 21, 3 stones; magazine 15 usable slots (simulate.py: "15 reachable").
"""
import copy
import importlib.util
import json
import math

import numpy as np
import pytest

from mauer import REPO, config
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
    return config.load()


@pytest.fixture(scope="module")
def job(cfg):
    return make_job.build_nominal(cfg, length=24)


def test_plan_matches_the_robodk_simulation(job, cfg):
    assert [len(s.stones) for s in job.stops] == [24, 21, 21, 3]
    assert [s.a_mm for s in job.stops] == pytest.approx([920.0, 2320.0, 3720.0, 5120.0])
    assert job.n_stones == 69 and job.meta["n_stones"] == 69
    dist = cfg["wall"]["dist_nominal"]
    for s in job.stops:                       # front wall: ARES faces the wall (-wall y) at y = dist
        assert (s.ares.x_mm, s.ares.y_mm, s.ares.theta_deg) == pytest.approx((s.a_mm, dist, -90.0), abs=1e-9)
    dx, dy, dth = relative_move(job.stops[0].ares, job.stops[1].ares)
    assert (dx, dy, dth) == pytest.approx((0.0, 1400.0, 0.0), abs=1e-9)      # sideways, +y body = left
    assert job.magazine.capacity == 15
    assert sorted(set(s.id for s in job.magazine.slots) - set(job.magazine.take_order)) == \
        ["r0y0l3", "r0y1l3", "r0y2l3"]


def test_place_pose_convention(job):
    """rdk_common.place_pose: wall_frame * transl(u, 0, z_top) * rotx(pi) (no flip in a nominal job)."""
    for t in job.stones():
        assert not t.flip
        assert np.allclose(t.T_wall_tcp, g.transl(t.u_mm, 0.0, t.z_top_mm) @ g.rotx(math.pi), atol=1e-12)
    tops = sorted({t.z_top_mm for t in job.stones()})
    assert tops == pytest.approx([140.0, 260.0, 380.0])      # base 20 + k * 120 + 120 ([wall], [brick])


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
    assert (st.dock.x_mm, st.dock.y_mm, st.dock.theta_deg) == pytest.approx((400.0, -800.0, 90.0))
    assert len(st.slots) == ps["slot_rows"] * ps["slot_cols"]
    assert 0 < len(st.take_order) < len(st.slots)            # PLACEHOLDER layout: most slots out of kinematic reach
    z = ps["table_z"] + cfg["brick"]["height"]
    assert all(s.T_station_tcp[2, 3] == pytest.approx(z) for s in st.slots)
    assert any("station slots without a kinematic IK" in w for w in job.meta["warnings"])
    T_park = ur5_fk(job.park_q_rad)
    assert T_park[2, 2] == pytest.approx(-1.0, abs=1e-6)     # tool down
    expected = "[ur] park_q_deg" if "park_q_deg" in cfg["ur"] else "simulate.py"
    assert expected in job.meta["park_q_source"]


def test_provenance(job, cfg):
    assert job.config_sha256 == mjob.config_sha256()
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
        mjob.from_dict({**d, "version": 2})
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


def test_config_status_parser():
    st = mjob.config_status()
    assert st["[ares] steer_axis_x"]["status"] == "CONFIRMED"
    assert st["[ur] host"]["status"] == "PLACEHOLDER"
    assert st["[camera.mount] xyz"]["status"] == "PLACEHOLDER"              # from the section comment
    assert st["[[targets]] W3.xyz"]["status"] == "PLACEHOLDER"              # from the [[targets]] block comment
    assert st["[[targets]] S0.xyz"]["status"] == "PLACEHOLDER"              # own inline tag
    assert st["[brick] mass_kg"]["status"] == "UNKNOWN"


def test_make_job_cli(tmp_path, capsys):
    out = tmp_path / "j.json"
    assert make_job.main(["--length", "6", "--out", str(out)]) == 0
    j = mjob.load(out)
    assert j.meta["length_stones"] == 6 and j.n_stones == 6 + 5 + 4
    assert "written" in capsys.readouterr().out


def test_stale_reach_table_is_refused(cfg, tmp_path):
    t = tmp_path / "reach.json"
    data = json.loads((REPO / "results" / "reach_table.json").read_text())
    data["key"] = "000000000000"
    t.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="another configuration"):
        make_job.build_nominal(cfg, 6, reach_table_path=t)
    assert make_job.build_nominal(cfg, 6, reach_table_path=t, allow_stale_reach=True).n_stones == 15
