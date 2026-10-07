"""Sequencer (mauer.sequencer) in the pure-Python simulated world (mauer.simworld), the UR5 kinematics and the
real-hardware backends (mauer.backends) against a fake link.

Error scenarios are ASSUMPTIONS documented in mauer/simworld.py (E003-like drive errors, slip events, hand-eye,
mount, station placement). Short jobs (2 stops, a few stones) keep the default run fast; the full straight wall and
the full L run only with MAUER_SLOW=1 (marker `slow`) or via tools/run_job.py --sim. The straight-wall tests use the
config without legs (conftest.straight_config), the L tests the L of 2026-10-05 (conftest.l_config; job v2:
routes, half stones).
"""
import copy
import importlib.util
import math
import os

import numpy as np
import pytest

from conftest import l_config, straight_config
from mauer import REPO
from mauer import geometry as g
from mauer import job as mjob
from mauer.ares.ads import MoveRefused
from mauer.backends import RobotError, URRobot, stone_payload
from mauer.sequencer import (FrameJumpError, InterlockError, MeasurementError, Sequencer, SequencerAborted,
                             SequencerError, SequencerParams, SequencerPaused, preflight_real, read_log)
from mauer.reference import Pose2D
from mauer.simworld import (DriveErrors, SimWorld, WorldErrors, ik_near, in_family, scenario, ur5_fk, ur5_ik)

SLOW = pytest.mark.skipif(not os.environ.get("MAUER_SLOW"), reason="full-wall run: set MAUER_SLOW=1")


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
def job10(cfg):
    return make_job.build_nominal(cfg, length=10)            # 34 stones, 2 stops (30 + 4)


def short_job(job, n0=6, n1=3, fill=None):
    """First n0 stones of stop 0 and n1 of stop 1; fill = stones in the magazine at the start: the first `fill` stones
    of the short job, each in a slot of its type (mauer.job.fill_plan, as tools/make_job.py plan_slots)."""
    j = copy.deepcopy(job)
    j.stops = j.stops[:2]
    j.stops[0].stones = j.stops[0].stones[:n0]
    j.stops[1].stones = j.stops[1].stones[:n1]
    if fill is not None:
        kinds = [t.kind for st in j.stops for t in st.stones][:fill]
        plan = dict(mjob.fill_plan(mjob.SlotState.magazine(j.magazine, filled=[], kinds={}), kinds))
        j.magazine.initial_fill = [sid for sid in j.magazine.take_order if sid in plan]
        j.magazine.initial_kinds = plan
    return j


def sim(cfg, job, errors, tmp_path, *, seed=1, camera_loop=True, confirm=None, params=None, **kw):
    w = SimWorld(cfg, job, errors, seed=seed, **kw)
    seq = Sequencer(job, cfg, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=tmp_path / "run",
                    confirm=confirm, camera_loop=camera_loop, on_station_empty=w.refill_station, params=params)
    return w, seq


def _events(seq):
    return read_log(seq.log.path)


# ── UR5 kinematics ────────────────────────────────────────────────────────────
def test_ur5_ik_recovers_every_configuration():
    rng = np.random.default_rng(0)
    for _ in range(300):
        q = rng.uniform(-math.pi, math.pi, 6)
        sols = ur5_ik(ur5_fk(q))
        assert any(np.allclose([math.remainder(a - b, 2 * math.pi) for a, b in zip(q, s)], 0.0, atol=1e-6)
                   for s in sols)
        for s in sols:
            assert g.pose_delta(ur5_fk(s), ur5_fk(q))[0] < 1e-3


def test_ur5_nominal_dh_values():
    """UR article DH: zero pose = arm stretched along base -x at flange height d1 - d5, reach a2 + a3 + d... ."""
    T = ur5_fk([0.0] * 6)
    assert T[:3, 3] == pytest.approx([-425.0 - 392.25, -109.15 - 82.3, 89.159 - 94.65], abs=1e-9)
    q = ik_near(ur5_fk(np.radians([0, -100, 52, -42, -90, 0])), np.radians([0, -100, 52, -42, -90, 0]))
    assert np.degrees(q) == pytest.approx([0, -100, 52, -42, -90, 0], abs=1e-6)
    assert in_family(q) and not in_family(np.radians([0, -100, -52, -42, -90, 0]))


# ── simulated ARES ────────────────────────────────────────────────────────────
def test_sim_ares_drive_model(cfg, job10):
    w = SimWorld(cfg, job10, WorldErrors(drive=DriveErrors(slip_prob=0.0)), seed=4)
    a = w.ares
    head, along = [], []
    for _ in range(200):
        p0 = w.ares_true
        a.translate(0.0, 1400.0)
        p1 = w.ares_true
        head.append(math.degrees(p1.theta_rad - p0.theta_rad))
        c, s = math.cos(p0.theta_rad), math.sin(p0.theta_rad)
        along.append(-s * (p1.x_mm - p0.x_mm) + c * (p1.y_mm - p0.y_mm))      # body y of the start pose
    assert np.mean(head) == pytest.approx(0.11, abs=0.015) and np.std(head) == pytest.approx(0.06, abs=0.015)
    assert np.mean(along) - 1400.0 == pytest.approx(-0.42, abs=0.1)          # -0.03 % (E003)
    assert a.odom.y_mm == pytest.approx(200 * 1400.0)                         # odometry = commanded, slip unseen
    w2 = SimWorld(cfg, job10, WorldErrors(drive=DriveErrors(rot_sigma_deg=0.0, rot_drift_sigma_mm=0.0)), seed=4)
    t0 = w2.ares_true.theta_rad
    w2.ares.rotate(10.0)
    w2.ares.rotate(-10.0)                                                     # reversal: 1.2 deg lost (E003)
    assert math.degrees(w2.ares_true.theta_rad - t0) == pytest.approx(1.2, abs=1e-9)
    with pytest.raises(MoveRefused):
        w2.ares.translate(1.0, 0.0)                                           # below the PLC minimum 2 mm
    with pytest.raises(MoveRefused):
        w2.ares.rotate(0.1)                                                   # below 0.2 deg


# ── (i) no errors ─────────────────────────────────────────────────────────────
def test_zero_errors_places_within_half_a_millimetre(cfg, job10, tmp_path):
    job = short_job(job10)
    w, seq = sim(cfg, job, WorldErrors(), tmp_path)
    res = seq.run()
    p = w.placement_stats()
    assert res.state == "done" and p["n"] == 9 and p["seated"] == 9
    assert p["horiz_mm"]["max"] < 0.5 and p["dz_mm"]["max"] < 0.5 and p["yaw_deg"]["max"] < 0.05
    assert len(w.ares.moves) == 1
    assert w.ares.moves[0]["kind"] == "translate" and w.ares.moves[0]["dy_mm"] == pytest.approx(1000.0, abs=0.5)
    assert w.violations == [] and res.reloads == 0
    ev = [e["event"] for e in _events(seq)]
    assert ev[0] == "run_start" and ev[-1] == "run_done" and ev.count("placed") == 9 and "wall_frame" in ev


# ── (ii) drive errors + slip: camera loop vs dead reckoning ───────────────────
def test_drive_errors_and_slip_closed_loop_vs_dead_reckoning(cfg, job10, tmp_path, capsys):
    job = short_job(job10, n0=8, n1=3)
    out = {}
    for loop in (True, False):
        w, seq = sim(cfg, job, scenario("slip"), tmp_path / str(loop), seed=3, camera_loop=loop)
        w.forced_slips[1] = (-12.0, 20.0, 0.25)          # stop 0 -> 1: a 23 mm slip (task: up to ~30 mm)
        seq.run()
        out[loop] = w.placement_stats()
        assert w.violations == []
    cam, dead = out[True], out[False]
    with capsys.disabled():
        for name, p in (("camera loop", cam), ("dead reckoning", dead)):
            print(f"\n  slip scenario, {name}: {p['seated']}/{p['n']} seated, horizontal error mean "
                  f"{p['horiz_mm']['mean']:.2f} / max {p['horiz_mm']['max']:.2f} mm, at the pins max "
                  f"{p['pin_mm']['max']:.2f} mm", end="")
    assert cam["seated"] == cam["n"] == 11 and cam["horiz_mm"]["max"] < 2.0
    assert dead["horiz_mm"]["max"] > 10.0 and dead["horiz_mm"]["max"] > 10 * cam["horiz_mm"]["max"]


# ── (iii) reload via the pick-up station ──────────────────────────────────────
def test_reload_via_station_and_remeasure(cfg, job10, tmp_path):
    job = short_job(job10, n0=4, n1=2, fill=2)
    errors = scenario("slip")
    errors.station_xyz_mm, errors.station_rpy_deg = (25.0, -15.0, 0.0), (0.0, 0.0, 0.6)   # station not where planned
    w, seq = sim(cfg, job, errors, tmp_path, seed=5)
    res = seq.run()
    p = w.placement_stats()
    assert res.state == "done" and p["n"] == 6 and p["seated"] == 6 and p["horiz_mm"]["max"] < 2.0
    assert res.reloads >= 1 and w.robot.calls["pick_station"] == w.robot.calls["place_magazine"] >= 3
    ev = _events(seq)
    names = [e["event"] for e in ev]
    i = names.index("reload_start")
    after = names[i:]
    for e in ("station_frame", "reload_transfer", "wall_frame", "reload_done"):
        assert e in after
    st = [e for e in ev if e["event"] == "station_frame"]
    assert st[0]["source"] == "camera" and set(st[0]["boards"]) == {"S0", "S1"}
    assert any(e["event"] == "wall_frame" and e.get("why") == "after reload" for e in ev)
    # the station location estimate moved from the nominal towards the true placement
    est = [e for e in ev if e["event"] == "station_estimate"][-1]["estimate"]
    true = w.T_wall_station_true
    nom = job.station.T_wall_station
    d_est = math.hypot(est["x_mm"] - true[0, 3], est["y_mm"] - true[1, 3])
    assert d_est < math.hypot(nom[0, 3] - true[0, 3], nom[1, 3] - true[1, 3])
    assert any(e["event"] == "warning" and "operator's responsibility" in e["msg"] for e in ev)
    assert w.violations == []


# ── (iv) failure handling ─────────────────────────────────────────────────────
def test_board_not_visible_pauses_with_the_arm_parked(cfg, job10, tmp_path):
    job = short_job(job10, n0=3, n1=1)
    w, seq = sim(cfg, job, WorldErrors(), tmp_path)
    hidden = job.stops[0].looks[1].boards[0]
    w.hidden_boards.add(hidden)
    with pytest.raises(MeasurementError, match=f"{hidden}.*not detected"):
        seq.run()
    assert seq.result.state == "paused" and seq.paused and w.robot.is_parked()
    assert w.ares.moves == [] and w.records == []
    with pytest.raises(InterlockError, match="paused"):
        seq._interlock("test")
    with pytest.raises(SequencerPaused):
        seq.run()
    w.hidden_boards.clear()                                  # operator cleared the view
    seq.resume()
    assert seq.run().state == "done" and len(w.records) == 4 and w.violations == []


def test_frame_jump_beyond_the_limit_stops(cfg, job10, tmp_path):
    job = short_job(job10, n0=2, n1=2)
    # a 300 mm slip on the move to stop 1: nothing of the boards in view, not even after the board search (+-50 mm)
    # -> MeasurementError, arm parked, run paused
    w, seq = sim(cfg, job, WorldErrors(), tmp_path / "a", params=SequencerParams(search_rings=1))   # 8 search poses
    w.forced_slips[1] = (0.0, -300.0, 0.0)
    with pytest.raises(MeasurementError, match="stop 1 wall"):
        seq.run()
    assert seq.result.state == "paused" and w.robot.is_parked() and len(w.records) == 2
    assert any(e["event"] == "board_search" for e in _events(seq))
    # a 75 mm slip: the boards are only partly in view (+-35..45 mm slack, README Camera); their marker corners re-aim
    # the looks, the board is measured - and the 75 mm jump exceeds the 50 mm limit -> FrameJumpError, run stopped
    w, seq = sim(cfg, job, WorldErrors(), tmp_path / "a2")
    w.forced_slips[1] = (0.0, -75.0, 0.0)
    with pytest.raises(FrameJumpError, match="75.0 mm"):
        seq.run()
    assert any(e["event"] == "coarse_aim" for e in _events(seq)) and w.violations == []
    # the same slip with a 100 mm limit (as after a route or a station trip) is measured and corrected
    w, seq = sim(cfg, job, WorldErrors(), tmp_path / "a3", params=SequencerParams(max_jump_mm=100.0))
    w.forced_slips[1] = (0.0, -75.0, 0.0)
    assert seq.run().state == "done" and seq.result.corrections >= 1 and w.placement_stats()["horiz_mm"]["max"] < 0.5
    # a 30 mm slip stays in view but exceeds a 20 mm jump limit -> FrameJumpError, run stopped
    w, seq = sim(cfg, job, WorldErrors(), tmp_path / "b", params=SequencerParams(max_jump_mm=20.0))
    w.forced_slips[1] = (0.0, -30.0, 0.0)
    with pytest.raises(FrameJumpError, match="stop 1 wall.*limit 20 mm"):
        seq.run()
    assert seq.result.state == "error" and len(w.records) == 2 and w.violations == []
    assert any(e["event"] == "frame_jump" for e in _events(seq))
    # the same slip with the default 50 mm limit is measured and corrected
    w, seq = sim(cfg, job, WorldErrors(), tmp_path / "c")
    w.forced_slips[1] = (0.0, -30.0, 0.0)
    assert seq.run().state == "done" and seq.result.corrections >= 1 and w.placement_stats()["horiz_mm"]["max"] < 0.5


def test_robot_block_error_stops_safely(cfg, job10, tmp_path):
    job = short_job(job10, n0=5, n1=2)
    w, seq = sim(cfg, job, WorldErrors(), tmp_path, fail_on={"place_wall": 3})
    with pytest.raises(SequencerError, match="NOT parked"):
        seq.run()
    assert seq.result.state == "error" and not w.robot.is_parked()
    n_moves = len(w.ares.moves)
    with pytest.raises(InterlockError, match="robot not parked"):
        seq._drive_to(job.stops[1].ares, None, "test")        # ARES never moves with the arm out
    assert len(w.ares.moves) == n_moves == 0 and w.violations == []
    ev = [e["event"] for e in _events(seq)]
    assert ev.index("robot_error") < ev.index("run_error") < ev.index("interlock") == len(ev) - 1
    # operator recovers: takes the stone out of the jaws, parks; resume places the rest without repeating any
    w.robot.holding = None
    w.robot.park()
    seq.run(start_stop=0)
    keys = [r.key for r in w.records]
    assert len(keys) == len(set(keys)) == 7 and w.violations == []


def test_step_mode_confirms_every_motion(cfg, job10, tmp_path):
    job = short_job(job10, n0=1, n1=1)
    asked = []

    def confirm(desc):
        asked.append(desc)
        return not desc.startswith("ARES")                   # decline the first ARES move

    w, seq = sim(cfg, job, WorldErrors(), tmp_path, confirm=confirm)
    with pytest.raises(SequencerAborted):
        seq.run()
    assert seq.result.state == "aborted" and w.ares.moves == []
    assert any(a.startswith("robot: look") for a in asked) and any(a.startswith("robot: place stone") for a in asked)
    assert asked[-1].startswith("ARES translate")
    assert sum(a.startswith("robot:") for a in asked) == w.robot.calls.total()


# ── (v) real-run preflight ────────────────────────────────────────────────────
def test_real_preflight_lists_the_placeholders(cfg, job10, tmp_path):
    no_host = copy.deepcopy(cfg)
    no_host["ur"]["host"] = ""                              # host, payload and stone mass are set since 2026-10-06
    no_host["ur"]["payload_tool_kg"] = 0.0
    no_host["brick"]["mass_kg"] = 0.0
    problems = preflight_real(no_host, job10, intrinsics_file=tmp_path / "none_i.json",
                              handeye_file=tmp_path / "none_h.json")
    text = "\n".join(problems)
    for frag in ("[ur] host is empty", "[ur] payload_tool_kg <= 0", "[brick] mass_kg <= 0", "camera intrinsics missing",
                 "hand-eye calibration missing", "nominal look poses"):
        assert frag in text, frag
    good = copy.deepcopy(cfg)
    good["ur"].update(host="192.0.2.10", payload_tool_kg=1.6)
    good["brick"]["mass_kg"] = 3.0
    j = copy.deepcopy(job10)
    j.meta.update(look_source="planner", reach_check="robodk")
    for f in ("i.json", "h.json"):
        (tmp_path / f).write_text("{}")
    assert preflight_real(good, j, intrinsics_file=tmp_path / "i.json", handeye_file=tmp_path / "h.json") == []
    good["brick"]["mass_kg"] = 4.0
    assert any("exceeds the UR5 rated payload" in p for p in
               preflight_real(good, j, intrinsics_file=tmp_path / "i.json", handeye_file=tmp_path / "h.json"))


# ── real backend against a fake link ──────────────────────────────────────────
class _State:
    def __init__(self, q):
        self.actual_q, self.actual_qd = np.asarray(q, float), np.zeros(6)
        self.program_running, self.runtime_state = False, 1


class _FakeLink:
    def __init__(self, q):
        self.st, self.blocks = _State(q), []

    def state(self):
        return self.st

    def state_age_s(self):
        return 0.0

    def run_block(self, body, name="b", timeout_s=0.0, settle_s=0.0):
        from mauer.ur.link import BlockResult
        self.blocks.append((name, body))
        return BlockResult(ok="FAIL" not in body, block_id=len(self.blocks), name=name, error="boom")


def test_ur_robot_programs(cfg, job10):
    c = copy.deepcopy(cfg)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"] = 0.0, 3.0
    with pytest.raises(ValueError, match="payload_tool_kg"):
        URRobot(_FakeLink(job10.park_q_rad), c, job10)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"] = 1.5, 0.0
    with pytest.raises(ValueError, match="mass_kg"):
        URRobot(_FakeLink(job10.park_q_rad), c, job10)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"] = 1.5, 3.0
    link = _FakeLink(job10.park_q_rad)
    r = URRobot(link, c, job10)
    assert r.is_parked() and r.is_idle()
    T_base_wall = g.inv(job10.T_ares_base) @ g.inv(job10.stops[0].ares.T)
    stone = job10.stops[0].stones[0]
    r.goto_look(job10.stops[0].looks[0])
    r.pick_magazine(job10.magazine.slot(stone.slot), g.inv(job10.T_ares_base))
    r.place_wall(T_base_wall, stone)
    r.park()
    names = [n for n, _ in link.blocks]
    assert names == ["mauer_look", "mauer_pick_mag", "mauer_place_wall", "mauer_park"]
    look, pick, place, park = (b for _, b in link.blocks)
    assert look.startswith("set_tcp(") and "get_inverse_kin(look" in look
    kg, cog = stone_payload(1.5, c["ur"]["payload_cog_mm"], 3.0, job10.T_flange_tcp, 120.0)
    tcp_z = c["tool"]["tcp_z"]
    assert kg == 4.5 and cog == pytest.approx([0.0, 0.0, (1.5 * 60.0 + 3.0 * (tcp_z + 60.0)) / 4.5])
    assert "set_payload(1.5," in pick.splitlines()[1] and "set_payload(4.5," in pick      # stone after closing
    assert "set_payload(4.5," in place.splitlines()[1] and "set_payload(1.5," in place    # empty after opening
    assert f"set_standard_digital_out({c['ur']['do_grip_close']}, True)" in pick
    assert f"set_standard_digital_out({c['ur']['do_grip_open']}, True)" in place
    assert "pl_at = pose_trans(pl_F" in place and "pk_at = pose_trans(pk_F" in pick
    assert park.splitlines()[-1].startswith("movej([")
    link.st.actual_q = link.st.actual_q + 0.1
    assert not r.is_parked()
    link.run_block = lambda body, **kw: _FakeLink.run_block(link, "FAIL", **kw)
    with pytest.raises(RobotError, match="boom"):
        r.park()


# ── full wall (slow) ──────────────────────────────────────────────────────────
@pytest.mark.slow
@SLOW
@pytest.mark.parametrize("scen,limit_mm", [("slip", 2.0), ("realistic", 5.0)])
def test_full_wall(cfg, tmp_path, capsys, scen, limit_mm):
    job = make_job.build_nominal(cfg, length=24)
    res = {}
    for loop in (True, False):
        w, seq = sim(cfg, job, scenario(scen), tmp_path / str(loop), seed=1, camera_loop=loop, grasp_check=loop)
        try:
            seq.run()
        except SequencerError:
            assert not loop
        res[loop] = w.placement_stats()
        assert w.violations == []
    with capsys.disabled():
        for loop, p in res.items():
            print(f"\n  full wall {scen} {'camera' if loop else 'dead reckoning'}: {p['seated']}/{p['n']} seated, "
                  f"horizontal mean {p['horiz_mm']['mean']:.2f} / p95 {p['horiz_mm']['p95']:.2f} / max "
                  f"{p['horiz_mm']['max']:.2f} mm", end="")
    assert res[True]["n"] == res[True]["seated"] == job.n_stones == 90 and res[True]["horiz_mm"]["max"] < limit_mm
    assert res[False]["horiz_mm"]["max"] > 5 * res[True]["horiz_mm"]["max"]


# ── the L (job v2: routes, half stones) ───────────────────────────────────────
@pytest.fixture(scope="module")
def lcfg():
    return l_config()


@pytest.fixture(scope="module")
def ljob(lcfg):
    return make_job.build_nominal(lcfg)


def short_l_job(ljob, n_a=3, n_b=5, fill=3):
    """The last stop of leg A (n_a stones) and the stop of leg B (n_b stones, its first course-0 stones and the half
    stone of course 1), connected by the leg-change route; `fill` stones in the magazine at the start -> at least one
    station trip from leg B (the longest route)."""
    j = copy.deepcopy(ljob)
    a, b = j.stops[1], j.stops[2]
    a.stones = a.stones[:n_a]
    b.stones = b.stones[:n_b]
    assert any(t.kind == "half" for t in b.stones)
    a.index, b.index = 0, 1
    a.route = []
    j.stops = [a, b]
    j.magazine.initial_fill = list(j.magazine.fill_order[:fill])
    make_job.plan_slots(j.stops, j.magazine, j.station, [])
    assert not __import__("mauer.job", fromlist=["validate"]).validate(j)
    return j


def test_l_leg_change_and_station_trip_with_half_stones(lcfg, ljob, tmp_path):
    """Realistic errors (E003 drive, slip, mount, hand-eye, holders, station placement): the leg change follows its
    route (dead reckoning + closed loop at B), the station trip from leg B follows its routes, half and full stones are
    picked by type, every stone seated, ARES never moves with the arm out."""
    job = short_l_job(ljob)
    w, seq = sim(lcfg, job, scenario("realistic"), tmp_path, seed=2)
    res = seq.run()
    p = w.placement_stats()
    assert res.state == "done" and p["n"] == p["seated"] == 8
    # the true ARES path (mauer.simworld floor check): never into a leg or the table; a plate only on the short
    # closed-loop approach from a measured pose (a slip of up to 30 mm, ASSUMPTION, against the 10 mm plate gap) -
    # every route ends arrival_standoff_mm (40) before its stop
    assert [v for v in w.violations if not v.startswith("ARES footprint overlaps plate")] == []
    for h, why in _floor_hits(w, seq):
        assert h["obstacle"].startswith("plate") and "correction" in why and h["depth_mm"] < 30.0, (h, why)
    assert p["by_kind"]["half"]["n"] >= 1 and p["by_kind"]["half"]["seated"] == p["by_kind"]["half"]["n"]
    assert p["horiz_mm"]["max"] < 6.0
    assert res.reloads >= 1
    ev = _events(seq)
    routes = [e for e in ev if e["event"] == "route"]
    whys = [e["why"] for e in routes]
    assert "stop 0 -> stop 1" in whys and "stop 1 -> station" in whys and "station -> stop 1" in whys
    assert any(e["event"] == "wall_frame" and e.get("why") == "after route" for e in ev)
    assert not any(e["event"] == "warning" and "operator's responsibility" in e["msg"] for e in ev)
    # every ARES command is a pure translation or a pure rotation (PLC v2.9) and the route's rotation was commanded
    moves = [e for e in ev if e["event"] == "ares_move"]
    assert all(m["kind"] in ("translate", "rotate") for m in moves)
    assert any(m["kind"] == "rotate" and abs(abs(m["args"][0]) - 90.0) < 3.0 for m in moves)
    st = [e for e in ev if e["event"] == "station_frame" and e.get("source") == "camera"]
    assert st and all(e["jump_mm"] < seq.p.station_max_jump_mm for e in st)


def test_l_route_drives_the_waypoints_without_errors(lcfg, ljob, tmp_path):
    """No drive errors: the commanded moves of the leg change are the route legs (steered from the camera-measured
    start, so up to the image noise) - the last one ends arrival_standoff_mm before the stop - and one closed-loop
    correction from the wall measurement there brings ARES onto the B stop."""
    job = short_l_job(ljob, n_a=1, n_b=2, fill=3)            # no reload: the only ARES moves are the leg change
    w, seq = sim(lcfg, job, WorldErrors(), tmp_path)
    assert seq.run().state == "done" and w.violations == []
    r = job.stops[1].route
    from mauer.floor import segments
    from mauer.reference import relative_move
    segs, _ = segments(r)
    so = job.meta["route_check"]["arrival_standoff_mm"]
    assert so > 0 and seq.standoff_mm == so
    cmds = [(m["kind"], m.get("dx_mm"), m.get("dy_mm"), m.get("dtheta_deg")) for m in w.ares.moves]
    assert len(cmds) == len(segs) + 1
    for i, ((kind, dx, dy, dth), sg) in enumerate(zip(cmds, segs)):
        assert kind == sg.kind
        if kind == "translate":
            ex, ey, _ = relative_move(sg.start, sg.end)
            if i == len(segs) - 1:                                  # the approach stops `so` before the stop
                ex, ey = ex * (1.0 - so / sg.length_mm), ey * (1.0 - so / sg.length_mm)
            assert (dx, dy) == pytest.approx((ex, ey), abs=0.5)    # steered from the measured start (image noise)
        else:
            assert dth == pytest.approx(math.degrees(sg.angle_rad), abs=0.05)
    kind, dx, dy, _ = cmds[-1]                                       # closed loop from the measurement
    assert kind == "translate" and (dx, dy) == pytest.approx((so, 0.0), abs=0.5)
    ev = _events(seq)
    assert [e["why"] for e in ev if e["event"] == "wall_frame" and e.get("stop") == 1][:2] == ["after route"] * 2
    d_mm, d_deg = w.ares_true.delta(job.stops[1].ares)
    assert d_mm < 0.5 and d_deg < 0.05


# ── interruptions on a route and resume (review 2026-10-05: R1, R2, R3, R8, R9) ─────────────────────────────────
def _floor_hits(w, seq):
    """(floor hit of the simulated world, `why` of the sequencer move it happened on)."""
    by_n = {e["n"]: e.get("why", "") for e in _events(seq) if e["event"] == "ares_move"}
    return [(h, by_n.get(h["move"], "?")) for h in w.floor_hits if h["kind"] != "start"]


def _decline_once(match):
    state = {"declined": None}

    def confirm(desc):
        if state["declined"] is None and match in desc:
            state["declined"] = desc
            return False
        return True
    return confirm, state


LEG_CHANGE_LEGS = ["stop 0 -> stop 1 leg 0", "stop 0 -> stop 1 leg 1", "stop 0 -> stop 1 leg 2",
                   "stop 0 -> stop 1 last leg"]                                # L at 840 mm, floor station (2026-10-06)
STATION_LEGS = ["stop 1 -> station leg 0", "stop 1 -> station leg 1", "stop 1 -> station leg 2",
                "stop 1 -> station last leg", "station -> stop 1 leg 0", "station -> stop 1 leg 1",
                "station -> stop 1 leg 2", "station -> stop 1 last leg"]


def _leg_names(why: str, route) -> list[str]:
    """Move names of mauer.sequencer._follow_route: leg 0 .. leg n-3, then the last leg."""
    return [f"{why} leg {i}" for i in range(len(route) - 2)] + [f"{why} last leg"]


def test_route_leg_lists_match_the_job(ljob):
    """The parametrised leg lists below follow the routes of the short L job (they change with the config)."""
    job = short_l_job(ljob, n_a=1, n_b=2, fill=1)
    b = job.stops[1]
    assert LEG_CHANGE_LEGS == _leg_names("stop 0 -> stop 1", b.route)
    assert STATION_LEGS == (_leg_names("stop 1 -> station", b.route_to_station)
                            + _leg_names("station -> stop 1", b.route_from_station))


@pytest.mark.parametrize("leg", LEG_CHANGE_LEGS + STATION_LEGS)
def test_l_declined_route_move_resumes_the_same_route(lcfg, ljob, tmp_path, leg):
    """Step mode declines one move of the leg change or of a station trip from leg B; run() with the same stop
    continues THAT route from the declined leg (an interrupted station trip still reloads) - never a straight line to
    the first waypoint of another route (the true path is checked against legs, plates and table) - and the first
    wall measurement uses the route / station limits."""
    job = short_l_job(ljob, n_a=1, n_b=2, fill=1)             # 1 stone in the magazine: a station trip from leg B
    confirm, state = _decline_once(leg)
    w, seq = sim(lcfg, job, WorldErrors(), tmp_path, confirm=confirm)
    with pytest.raises(SequencerAborted):
        seq.run()
    assert state["declined"] and "ARES" in state["declined"]
    pr = seq.route_progress
    assert pr is not None and seq.stop_k == 1 and pr.stop == 1
    assert pr.kind == ("stop" if leg.startswith("stop 0") else "to_station" if "-> station" in leg else "from_station")
    seq.confirm = None
    res = seq.run(seq.stop_k)
    assert res.state == "done" and len(res.placed) == 3 and res.reloads == 1
    assert w.violations == [] and _floor_hits(w, seq) == []
    ev = _events(seq)
    assert any(e["event"] == "resume_route" for e in ev)
    first = next(e for e in ev if e["event"] == "wall_frame" and e.get("stop") == 1
                 and ev.index(e) > next(i for i, x in enumerate(ev) if x["event"] == "resume_route"))
    assert first["why"] == ("after route" if pr.kind == "stop" else "after reload")
    assert w.ares_true.delta(job.stops[1].ares)[0] < 0.5


def test_l_pause_on_the_leg_change_and_resume(lcfg, ljob, tmp_path):
    job = short_l_job(ljob, n_a=1, n_b=2, fill=3)
    holder = {}

    def confirm(desc):
        if "stop 0 -> stop 1 leg 1" in desc:
            holder["seq"].pause()                                # the move runs, the next one is refused
        return True
    w, seq = sim(lcfg, job, scenario("realistic"), tmp_path, seed=3, confirm=confirm)
    holder["seq"] = seq
    with pytest.raises(SequencerPaused):
        seq.run()
    assert seq.route_progress.next_leg == 2 and seq.paused
    seq.resume()
    seq.confirm = None
    assert seq.run(seq.stop_k).state == "done"
    assert [v for v in w.violations if not v.startswith("ARES footprint overlaps plate")] == []
    assert all("correction" in why for _, why in _floor_hits(w, seq))


def test_l_pause_between_route_and_measurement_keeps_the_route_limits(lcfg, ljob, tmp_path):
    """R8: a route finished at the arrival standoff, the run paused before the wall was measured - the resumed
    measurement still uses the route jump limits (the pose is dead-reckoned), not the 50 mm arrival limit."""
    job = short_l_job(ljob, n_a=1, n_b=2, fill=3)
    holder = {}

    def confirm(desc):
        if "look stop2" in desc and not holder.get("paused"):     # looks keep the full job's names
            holder["paused"] = True
            holder["seq"].pause()
            return False                                        # the first look of stop 1 is not driven
        return True
    w, seq = sim(lcfg, job, WorldErrors(), tmp_path, confirm=confirm)
    holder["seq"] = seq
    with pytest.raises((SequencerAborted, SequencerPaused)):
        seq.run()
    assert seq.route_progress is None and seq.pending_why == "after route"
    seq.resume()
    seq.confirm = None
    assert seq.run(seq.stop_k).state == "done" and w.violations == []
    ev = _events(seq)
    i0 = next(i for i, e in enumerate(ev) if e["event"] == "resume")
    assert next(e for e in ev[i0:] if e["event"] == "wall_frame")["why"] == "after route"
    assert seq.pending_why is None


def test_l_move_aborted_by_the_plc_updates_the_estimate_from_odometry(lcfg, ljob, tmp_path):
    """R2: the PLC aborts the 1450 mm leg of the leg change after 70 % (safety scanner, HALT, jog): the estimate
    follows the outcome's odometry, the pose is unverified (resume refused until the operator confirms it), then the
    route continues from there - ARES does not overshoot by the distance already driven."""
    from dataclasses import replace
    job = short_l_job(ljob, n_a=1, n_b=2, fill=3)
    w, seq = sim(lcfg, job, WorldErrors(), tmp_path)
    real = w.ares.translate
    state = {"hit": False}

    def aborting(dx, dy):
        if not state["hit"] and math.hypot(dx, dy) > 1000.0:
            state["hit"] = True
            out = real(0.7 * dx, 0.7 * dy)
            return replace(out, ok=False, dx_mm=dx, dy_mm=dy, text="aborted (simulated safety scanner)")
        return real(dx, dy)
    w.ares.translate = aborting
    with pytest.raises(SequencerError, match="not ok"):
        seq.run()
    assert seq.pose_status == "odometry" and seq.pose_est.delta(w.ares_true)[0] < 0.5
    with pytest.raises(SequencerError, match="resume refused"):
        seq.run(seq.stop_k)
    seq.confirm_pose()
    res = seq.run(seq.stop_k)
    assert res.state == "done" and w.violations == [] and _floor_hits(w, seq) == []
    assert w.ares_true.delta(job.stops[1].ares)[0] < 0.5


def test_l_ares_error_without_outcome_needs_the_operator_pose(lcfg, ljob, tmp_path):
    """An ARES error without an outcome (connection lost mid-move): pose unknown, resume refused until set_pose();
    a pose from which the next move would cross a leg is refused by the floor check of the resume."""
    from mauer.backends import AresError
    job = short_l_job(ljob, n_a=1, n_b=2, fill=3)
    w, seq = sim(lcfg, job, WorldErrors(), tmp_path)
    real = w.ares.translate
    state = {"hit": False}

    def failing(dx, dy):
        if not state["hit"] and math.hypot(dx, dy) > 1000.0:
            state["hit"] = True
            real(0.5 * dx, 0.5 * dy)
            raise AresError("ADS connection lost (simulated)")
        return real(dx, dy)
    w.ares.translate = failing
    with pytest.raises(SequencerError, match="UNKNOWN"):
        seq.run()
    assert seq.pose_status == "unknown"
    with pytest.raises(SequencerError, match="resume refused"):
        seq.run(seq.stop_k)
    with pytest.raises(SequencerError, match="unknown.*set_pose"):
        seq.confirm_pose()                                     # nothing to confirm: no odometry
    pr = seq.route_progress
    b = pr.route[pr.next_leg + 1]
    seq.set_pose(Pose2D(1200.0, -700.0 if b.y_mm > 0 else 700.0, b.theta_rad))   # behind leg A: crossing
    with pytest.raises(SequencerError, match="would cross an obstacle"):
        seq.run(seq.stop_k)
    seq.set_pose(w.ares_true, "operator (measured on the floor)")
    res = seq.run(seq.stop_k)
    assert res.state == "done" and w.violations == [] and _floor_hits(w, seq) == []


def test_resumed_half_turn_is_not_a_full_turn(lcfg, ljob, tmp_path):
    """R1: a planned half turn keeps its direction only while about half of it is left - a resume that finds the
    heading (nearly) reached must not spin ARES by 360 deg."""
    job = short_l_job(ljob, n_a=1, n_b=2, fill=3)
    w, seq = sim(lcfg, job, WorldErrors(), tmp_path)
    r = job.stops[0].route_from_station                        # dock -> leg A stop: one 180 deg rotation
    from mauer.floor import segments
    segs, _ = segments(r)
    j = next(i for i, sg in enumerate(segs) if sg.kind == "rotate")
    assert abs(math.degrees(segs[j].angle_rad)) == pytest.approx(180.0)
    after = r[j + 1]
    for err in (0.3, -0.3):                                    # 0.3 deg short of / past the planned heading
        p = Pose2D(after.x_mm, after.y_mm, after.theta_rad - math.radians(err))
        w.ares_true = p
        seq.pose_est, seq.stop_k = p, 0
        n0 = len(w.ares.moves)
        seq._follow_route(r, "test", kind="from_station", stop=0, start_leg=j, resume=True)
        rot = [m for m in w.ares.moves[n0:] if m["kind"] == "rotate"]
        assert all(abs(m["dtheta_deg"]) < 1.0 for m in rot), rot


def test_l_preflight_asks_for_the_half_stone(lcfg, ljob, tmp_path):
    text = "\n".join(preflight_real(lcfg, ljob, intrinsics_file=tmp_path / "i", handeye_file=tmp_path / "h"))
    assert "[half_brick] mass_kg <= 0" in text and "[half_brick] " in text and "PLACEHOLDER" in text


def test_ur_robot_payload_per_stone_type(lcfg, ljob):
    c = copy.deepcopy(lcfg)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"], c["half_brick"]["mass_kg"] = 1.5, 3.0, 1.4
    link = _FakeLink(ljob.park_q_rad)
    r = URRobot(link, c, ljob)
    slot = ljob.magazine.slot(ljob.magazine.take_order[0])
    r.pick_magazine(slot, g.inv(ljob.T_ares_base), "half")
    half = next(t for t in ljob.stones() if t.kind == "half")
    r.place_wall(g.inv(ljob.T_ares_base) @ g.inv(ljob.stops[0].ares.T), half)
    pick, place = link.blocks[0][1], link.blocks[1][1]
    assert "set_payload(2.9," in pick and "set_payload(2.9," in place.splitlines()[1]
    c["half_brick"]["mass_kg"] = 0.0
    with pytest.raises(ValueError, match="half_brick"):
        URRobot(_FakeLink(ljob.park_q_rad), c, ljob)


@pytest.mark.slow
@SLOW
def test_full_l(lcfg, ljob, tmp_path, capsys):
    res = {}
    for loop in (True, False):
        w, seq = sim(lcfg, ljob, scenario("realistic"), tmp_path / str(loop), seed=1, camera_loop=loop,
                     grasp_check=loop)
        try:
            seq.run()
        except SequencerError:
            assert not loop
        res[loop] = w.placement_stats()
        if loop:                    # plate contacts only from a slip on the closed-loop approach (l_wall_plan.md)
            assert [v for v in w.violations if not v.startswith("ARES footprint overlaps plate")] == []
            assert all("correction" in why for _, why in _floor_hits(w, seq))
        else:
            assert [v for v in w.violations if not v.startswith("ARES footprint")] == []
    with capsys.disabled():
        for loop, p in res.items():
            print(f"\n  full L realistic {'camera' if loop else 'dead reckoning'}: {p['seated']}/{p['n']} seated, "
                  f"horizontal max {p['horiz_mm']['max']:.2f} mm", end="")
    assert res[True]["n"] == res[True]["seated"] == ljob.n_stones and res[True]["horiz_mm"]["max"] < 6.0


# ── the C of the config (Samuel 2026-10-06; wall distance 840 mm) ─────────────────────────────────────────────────
@pytest.fixture(scope="module")
def ccfg():
    from mauer import config
    return config.load()


@pytest.fixture(scope="module")
def cjob(ccfg):
    return make_job.build_nominal(ccfg)


def test_c_leg_change_c_to_b_and_station_trip_from_b(ccfg, cjob, tmp_path):
    """The second leg change of the C (built A, C, B since 2026-10-07: C runs through its corner with B): from leg C
    to leg B and a station trip from leg B, with realistic errors - every stone seated, the true ARES path clear of
    legs, plates and table (840 mm: 110 mm between the ARES front and the plates at a stop)."""
    j = copy.deepcopy(cjob)
    c, b = j.stops[1], j.stops[2]
    assert (c.leg, b.leg) == ("C", "B")
    c.stones, b.stones = c.stones[:2], b.stones[:4]
    n = len(c.stones) + len(b.stones)
    c.index, b.index = 0, 1
    c.route = []
    j.stops = [c, b]
    j.magazine.initial_fill = list(j.magazine.fill_order[:3])
    make_job.plan_slots(j.stops, j.magazine, j.station, [])
    assert not __import__("mauer.job", fromlist=["validate"]).validate(j)
    w, seq = sim(ccfg, j, scenario("realistic"), tmp_path, seed=1)
    res = seq.run()
    p = w.placement_stats()
    assert res.state == "done" and p["n"] == p["seated"] == n and res.reloads >= 1
    assert w.violations == [] and _floor_hits(w, seq) == []
    whys = [e["why"] for e in _events(seq) if e["event"] == "route"]
    assert "stop 0 -> stop 1" in whys and "stop 1 -> station" in whys and "station -> stop 1" in whys


@pytest.mark.slow
@SLOW
def test_full_c(ccfg, cjob, tmp_path, capsys):
    """The whole C with realistic errors and the camera loop: every stone seated, the true ARES path never touches a
    leg, plate or the table (at 840 mm the plates are 110 mm from the ARES front at a stop)."""
    w, seq = sim(ccfg, cjob, scenario("realistic"), tmp_path, seed=1)
    seq.run()
    p = w.placement_stats()
    with capsys.disabled():
        print(f"\n  full C realistic camera: {p['seated']}/{p['n']} seated, horizontal max {p['horiz_mm']['max']:.2f} mm",
              end="")
    assert p["n"] == p["seated"] == cjob.n_stones and p["horiz_mm"]["max"] < 6.0
    assert w.violations == [] and _floor_hits(w, seq) == []
