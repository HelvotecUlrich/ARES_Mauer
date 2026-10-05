"""Sequencer (mauer.sequencer) in the pure-Python simulated world (mauer.simworld), the UR5 kinematics and the
real-hardware backends (mauer.backends) against a fake link.

Error scenarios are ASSUMPTIONS documented in mauer/simworld.py (E003-like drive errors, slip events, hand-eye,
mount, station placement). Short jobs (2 stops, a few stones) keep the default run fast; the full 69-stone wall runs
only with MAUER_SLOW=1 (marker `slow`; pytest.ini registration requested) or via tools/run_job.py --sim.
"""
import copy
import importlib.util
import math
import os

import numpy as np
import pytest

from mauer import REPO, config
from mauer import geometry as g
from mauer.ares.ads import MoveRefused
from mauer.backends import RobotError, URRobot, stone_payload
from mauer.sequencer import (FrameJumpError, InterlockError, MeasurementError, Sequencer, SequencerAborted,
                             SequencerError, SequencerParams, SequencerPaused, preflight_real, read_log)
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
    return config.load()


@pytest.fixture(scope="module")
def job10(cfg):
    return make_job.build_nominal(cfg, length=10)            # 27 stones, 2 stops (24 + 3)


def short_job(job, n0=6, n1=3, fill=None):
    """First n0 stones of stop 0 and n1 of stop 1; fill = stones in the magazine at the start (bottom layer first)."""
    j = copy.deepcopy(job)
    j.stops = j.stops[:2]
    j.stops[0].stones = j.stops[0].stones[:n0]
    j.stops[1].stones = j.stops[1].stones[:n1]
    if fill is not None:
        j.magazine.initial_fill = j.magazine.fill_order[:fill]
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
    assert w.ares.moves[0]["kind"] == "translate" and w.ares.moves[0]["dy_mm"] == pytest.approx(1400.0, abs=0.5)
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
    # a 75 mm slip on the move to stop 1: the boards leave the field of view (+-35..45 mm slack, README Camera) ->
    # MeasurementError, arm parked, run paused
    w, seq = sim(cfg, job, WorldErrors(), tmp_path / "a")
    w.forced_slips[1] = (0.0, -75.0, 0.0)
    with pytest.raises(MeasurementError, match="stop 1 wall"):
        seq.run()
    assert seq.result.state == "paused" and w.robot.is_parked() and len(w.records) == 2
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
    problems = preflight_real(cfg, job10, intrinsics_file=tmp_path / "none_i.json",
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
    with pytest.raises(ValueError, match="payload_tool_kg"):
        URRobot(_FakeLink(job10.park_q_rad), cfg, job10)
    c = copy.deepcopy(cfg)
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
    assert kg == 4.5 and cog == pytest.approx([0.0, 0.0, (1.5 * 60.0 + 3.0 * (135.0 + 60.0)) / 4.5])
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
    assert res[True]["n"] == res[True]["seated"] == 69 and res[True]["horiz_mm"]["max"] < limit_mm
    assert res[False]["horiz_mm"]["max"] > 5 * res[True]["horiz_mm"]["max"]
