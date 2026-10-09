"""Magazine dry run on ARES (Samuel 2026-10-08, mauer/magtest.py, tools/make_magtest.py): two full stones walk through
every magazine position (layers 1 and 2) and back; between pick and put-down every move takes the stone forward over
a place pose of the front leg and lowers it to [magtest] hover_mm above the place height WITHOUT opening the jaws.
ARES stands still, no camera, no ADS. Built from the config only - no reach table, no RoboDK."""
from __future__ import annotations

import copy
import importlib.util
import math
import re

import numpy as np
import pytest

from mauer import REPO, config
from mauer import geometry as g
from mauer import job as mjob
from mauer.backends import URRobot
from mauer.job import SlotState
from mauer.reference import Pose2D
from mauer.sequencer import SequencerError, SequencerPaused


def _tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


make_magtest = _tool("make_magtest")


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def job(cfg):
    return make_magtest.build(cfg)


@pytest.fixture(scope="module")
def job1(cfg):
    """One round (there and back) - the SIM runs."""
    return make_magtest.build(cfg, rounds=1)


def _moves(job):
    return job.meta["magtest"]["moves"]


# ── plan ──────────────────────────────────────────────────────────────────────
def test_two_stones_walk_every_position_in_layers_1_and_2_and_back(cfg, job):
    m = job.meta["magtest"]
    assert job.meta["kind"] == "magtest" and m["positions"] == cfg["magtest"]["positions"]
    mag = SlotState.magazine(job.magazine)                      # the start fill: one stack of two stones
    start = set(mag.filled)
    p0 = m["positions"][0]
    assert start == {f"{p0}l1", f"{p0}l2"} and m["start_fill"] == [f"{p0}l1", f"{p0}l2"]
    picked, put = set(), set()
    for mv in m["moves"]:
        assert mv["from"] in mag.filled and mag.can_take(mv["from"]), mv
        mag.take(mv["from"])
        assert mag.can_fill(mv["to"]), mv
        mag.fill(mv["to"], "full")
        picked.add(mv["from"])
        put.add(mv["to"])
    every = {f"{p}l{lay}" for p in m["positions"] for lay in (1, 2)}
    assert picked == every and put == every
    assert set(mag.filled) == start                             # back where it started: rounds can follow
    assert len(m["moves"]) == m["rounds"] * 2 * 2 * (len(m["positions"]) - 1)
    assert all(s.kind == "full" and s.layer <= 2 for s in job.magazine.slots)


def test_dry_place_targets_stand_in_front_of_ares_inside_out(cfg, job):
    front = next(lg for lg in cfg["wall"]["legs"] if lg["side"] == "front")
    b, w = cfg["brick"], cfg["wall"]
    stones = job.stops[0].stones
    assert len(stones) == len(_moves(job)) and job.stops[0].ares == Pose2D(0.0, 0.0, 0.0)
    for t, mv in zip(stones, _moves(job)):
        T = np.asarray(t.T_wall_tcp)                            # wall frame = ARES frame (ARES at the origin)
        assert T[0, 3] == pytest.approx(front["dist"]) and T[1, 3] == pytest.approx(t.u_mm)
        assert T[2, 3] == pytest.approx(w["base_z"] + t.course * (b["height"] + b["bed_joint"]) + b["height"])
        assert T[2, 2] < -0.99 and abs(T[1, 0]) > 0.99          # TCP down, stone length along the front leg
        assert tuple(mv["key"]) == t.key and t.slot == mv["from"] and t.kind == "full"
        assert t.qnear_rad is not None
    mt = cfg["magtest"]
    targets = {(float(u), int(c)) for u in mt["front_u_mm"] for c in mt["front_courses"]}
    dropped = {(d["u_mm"], d["course"]) for d in job.meta["magtest"]["dropped_targets"]}
    # the top course at u = +-600 is beyond the reach at the hover pose (RoboDK reach table: top course +-600 mm at
    # the place height) - listed, not driven; every other target is visited by the two rounds
    assert dropped == {(-600.0, 3), (600.0, 3)}
    assert {(t.u_mm, t.course) for t in stones} == targets - dropped
    first = [abs(t.u_mm) for t in stones[:len(targets - dropped)]]
    assert first == sorted(first)                               # inside out: the far reach (tipping) comes last
    assert job.meta["magtest"]["hover_mm"] == mt["hover_mm"] and job.meta["magtest"]["dwell_s"] == mt["dwell_s"]


def test_the_test_job_validates_and_survives_save_and_load(job, tmp_path):
    from mauer import magtest
    assert mjob.validate(job) == [] and magtest.is_magtest(job)
    j2 = mjob.load(mjob.save(job, tmp_path / "magtest.json"))
    assert magtest.is_magtest(j2) and j2.meta["magtest"] == job.meta["magtest"]
    assert [t.key for t in j2.stones()] == [t.key for t in job.stones()]
    plain = copy.deepcopy(job)
    plain.meta.pop("kind")
    assert any("no look poses" in p for p in mjob.validate(plain))     # a wall job still needs its looks


# ── robot programs ────────────────────────────────────────────────────────────
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
        return BlockResult(ok=True, block_id=len(self.blocks), name=name, error="")


def _guard(cfg, job):
    from mauer.motionguard import MotionGuard
    return MotionGuard(cfg, job.T_ares_base, job.park_q_rad, job.T_flange_tcp, approach_mm=job.approach_mm)


def _pose_xyz_m(body: str, name: str) -> np.ndarray:
    m = re.search(rf"{name} = pose_trans\(\w+, p\[([^\]]+)\]\)", body)
    assert m, f"{name} not in the program"
    return np.array([float(v) for v in m.group(1).split(",")[:3]])


def test_dry_place_lowers_the_held_stone_to_the_hover_pose_and_never_opens(cfg, job):
    link = _FakeLink(job.park_q_rad)
    r = URRobot(link, cfg, job, guard=_guard(cfg, job))
    T_base_ares = g.inv(np.asarray(job.T_ares_base))
    t = job.stops[0].stones[0]
    r.pick_magazine(job.magazine.slot(t.slot), T_base_ares)
    link.st.actual_q = np.asarray(job.park_q_rad, float)          # the fake arm stays at rest
    r.dry_place(T_base_ares, t, 50.0, 2.0)
    assert r.holding == "full"                                   # the stone stays in the jaws
    name, body = link.blocks[-1]
    assert name == "mauer_dry_place"
    assert "set_standard_digital_out" not in body               # no gripper command at all
    from mauer.ur import script
    assert body.splitlines()[1] == script.set_payload(*r.payloads["full"])          # payload WITH the stone
    assert "sleep(2.0)" in body
    at = _pose_xyz_m(body, "dp_at")                              # local pose in the ARES frame [m]
    assert at[2] * 1000.0 == pytest.approx(t.z_top_mm + 50.0, abs=0.05)
    above = _pose_xyz_m(body, "dp_above")
    assert above[2] * 1000.0 == pytest.approx(t.z_top_mm + 50.0 + job.approach_mm, abs=0.05)
    assert body.index("movel(dp_at") < body.index("sleep(2.0)") < body.rindex("movel(dp_above")


# ── runs in the simulated world (motion guard on) ─────────────────────────────
def _sim(cfg, job, tmp_path, **kw):
    from mauer.magtest import MagazineTest
    from mauer.simworld import SimWorld
    w = SimWorld(cfg, job, guard=True, **kw)
    seq = MagazineTest(job, cfg, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=tmp_path / "run")
    return w, seq


def test_sim_run_moves_both_stones_through_every_slot_and_back(cfg, job1, tmp_path):
    w, seq = _sim(cfg, job1, tmp_path)
    n = len(_moves(job1))
    res = seq.run()
    assert res.state == "done" and len(res.placed) == n and seq.held is None
    assert w.robot.calls["dry_place"] == w.robot.calls["pick_magazine"] == w.robot.calls["place_magazine"] == n
    assert w.robot.calls["place_wall"] == 0 and w.records == [] and w.violations == []
    assert set(w.mag_stones) == set(job1.meta["magtest"]["start_fill"])
    assert not any("not seated" in x for x in w.notes)
    assert res.ares_moves == 0                                   # ARES never asked to move (no ares_cmd below)
    events = [r["event"] for r in mjob_read(seq)]
    assert events.count("dry_placed") == n and "wall_frame" not in events and "ares_cmd" not in events


def mjob_read(seq):
    from mauer.sequencer import read_log
    return read_log(seq.log.path)


def test_pause_takes_effect_after_the_put_down_and_resume_continues(cfg, job1, tmp_path):
    w, seq = _sim(cfg, job1, tmp_path)

    def confirm(desc: str) -> bool:
        if "dry place" in desc and not seq.paused:
            seq.pause()                                          # pressed while the stone is held
        return True
    seq.confirm = confirm
    with pytest.raises(SequencerPaused):
        seq.run()
    assert seq.held is None and len(seq.placed) == 1             # the move was finished with the put-down
    assert w.robot.calls["place_magazine"] == 1
    seq.confirm = None
    seq.resume()
    assert seq.run().state == "done"
    assert set(w.mag_stones) == set(job1.meta["magtest"]["start_fill"])


def test_a_robot_error_at_the_front_needs_jaws_empty_and_repeats_the_move(cfg, job1, tmp_path):
    w, seq = _sim(cfg, job1, tmp_path, fail_on={"dry_place": 2})
    with pytest.raises(SequencerError):
        seq.run()
    mv = _moves(job1)[1]
    assert seq.held is not None and seq.held["slot"] == mv["from"]
    with pytest.raises(SequencerError, match="jaws"):
        seq.run()                                                # refused while a stone may be held
    # the operator takes the stone out and puts it back on the slot of this move, parks the arm, 'Jaws empty'
    sid, _ = w.robot.holding
    w.mag_stones[mv["from"]] = (sid, np.asarray(job1.magazine.slot(mv["from"]).T_ares_tcp, float))
    w.robot.holding = None
    seq.clear_held()
    assert mv["from"] in seq.magazine.filled and mv["to"] not in seq.magazine.filled
    assert seq.run().state == "done"
    assert set(w.mag_stones) == set(job1.meta["magtest"]["start_fill"]) and w.violations == []


# ── real-run preflight ────────────────────────────────────────────────────────
def test_preflight_needs_the_ur_values_but_not_the_camera_or_ares(cfg, job):
    from mauer import magtest
    assert magtest.preflight(cfg, job) == []                     # camera, hand-eye, RoboDK stamp: not needed
    assert any("payload_cog_mm" in x for x in magtest.warnings(cfg, job))      # PLACEHOLDER: shown, not blocking
    stale = copy.deepcopy(job)
    stale.config_sha256 = "0" * 64
    assert any("rebuild" in p for p in magtest.preflight(cfg, stale))
    bad = copy.deepcopy(cfg)
    bad["ur"]["host"], bad["ur"]["payload_tool_kg"] = "", 0.0
    probs = magtest.preflight(bad, job)
    assert any("[ur] host" in p for p in probs) and any("payload_tool_kg" in p for p in probs)
    with pytest.raises(ValueError, match="magtest"):
        magtest.MagazineTest(_plain(job), cfg, None, None, None, None, None)


def _plain(job):
    j = copy.deepcopy(job)
    j.meta = {}
    return j


def test_cli_writes_the_job(tmp_path, capsys):
    out = tmp_path / "mt.json"
    assert make_magtest.main(["--rounds", "1", "--out", str(out)]) == 0
    j = mjob.load(out)
    assert len(_moves(j)) == 20 and "20 moves" in capsys.readouterr().out


def test_magtest_settings_do_not_change_the_wall_plan_hash():
    """[magtest] is not planning input of the wall jobs (like [hmi*], test plan Anhang C7): hover or rounds changed
    for the dry run must not invalidate the wall job or its RoboDK stamp."""
    base = mjob.planning_bytes(b"[wall]\nx = 1\n[magtest]\nhover_mm = 50.0\n[ur]\ny = 2\n")
    assert base == mjob.planning_bytes(b"[wall]\nx = 1\n[magtest]\nhover_mm = 80.0\nrounds = 3\n[ur]\ny = 2\n")
    assert b"[ur]" in base and b"hover_mm" not in base


def test_the_pendant_check_names_the_directions_in_the_driving_direction(cfg):
    """Samuel 2026-10-08 on the pendant (Move tab, feature Base): X growing moves the TCP to the RIGHT, Y growing
    FORWARD towards the ARES centre (seen in the driving direction) - the UR sits at the vehicle rear, this repo's ARES
    frame (+x towards the UR end) is base_link turned by 180 deg ([ares] frame_x_points_to = "rear")."""
    from mauer.magtest import pendant_check
    assert cfg["ares"]["frame_x_points_to"] == "rear" and cfg["ur5"]["mount_rz"] == 90.0
    assert pendant_check(cfg) == ("RIGHT", "FORWARD")
    front = copy.deepcopy(cfg)
    front["ares"]["frame_x_points_to"] = "front"
    assert pendant_check(front) == ("LEFT", "BACKWARD")
    front["ur5"]["mount_rz"] = 0.0
    assert pendant_check(front) == ("FORWARD", "LEFT")
    front["ur5"]["mount_rz"] = 45.0
    assert pendant_check(front)[0].startswith("between")


def test_wall_runs_are_refused_while_the_frame_points_to_the_vehicle_rear(cfg, job):
    """The sequencer's ARES moves are in this repo's frame; with +x = vehicle rear the PLC would drive the other way.
    The dry run moves no ARES (its preflight does not care)."""
    from mauer.sequencer import preflight_real
    assert any("vehicle REAR" in p for p in preflight_real(cfg, job))
    front = copy.deepcopy(cfg)
    front["ares"]["frame_x_points_to"] = "front"
    assert not any("vehicle REAR" in p for p in preflight_real(front, job))
    from mauer import magtest
    assert magtest.preflight(cfg, job) == []


def test_speed_factor_scales_every_following_move_and_the_timeout(cfg, job):
    """HMI "REAL speed" (2026-10-09, no speed slider on the pendant): speeds and accelerations times f."""
    from mauer.ur import script
    link = _FakeLink(job.park_q_rad)
    r = URRobot(link, cfg, job, guard=_guard(cfg, job))
    t0 = r.timeout_s
    r.set_speed_factor(0.25)
    u = cfg["ur"]
    assert r.speeds == script.Speeds(0.25 * u["v_joint"], 0.25 * u["a_joint"], 0.25 * u["v_lin"],
                                     0.25 * u["a_lin"], 0.25 * u["v_contact"])
    assert r.timeout_s == pytest.approx(4.0 * t0)
    r.pick_magazine(job.magazine.slot(job.stops[0].stones[0].slot), g.inv(np.asarray(job.T_ares_base)))
    body = link.blocks[-1][1]
    assert f"v={script.num(0.25 * u['v_joint'])})" in body
    r.set_speed_factor(1.0)                                      # back to the [ur] values, not 0.25 * 0.25
    assert r.speeds == script.Speeds.from_config(dict(cfg)) and r.timeout_s == pytest.approx(t0)
    with pytest.raises(ValueError):
        r.set_speed_factor(0.0)


# ── put-down height and separation (real dry run 2026-10-09) ──────────────────
def test_put_down_lets_go_above_the_slot_and_comes_from_above_the_neighbours(cfg, job):
    """Samuel 2026-10-09: the put-down pressed the stone onto the cones (protective stop) - the jaws open [ur]
    release_above_mm above the slot pose; the approach is high enough for the held stone to clear the neighbours."""
    from mauer.magtest import magazine_approach_mm
    rel = float(cfg["ur"]["release_above_mm"])
    assert rel == 10.0
    link = _FakeLink(job.park_q_rad)
    r = URRobot(link, cfg, job, guard=_guard(cfg, job))
    T_base_ares = g.inv(np.asarray(job.T_ares_base))
    src, dst = job.magazine.slot("r0y0l2"), job.magazine.slot("r0y1l1")
    r.pick_magazine(src, T_base_ares)
    link.st.actual_q = np.asarray(job.park_q_rad, float)
    a = magazine_approach_mm(cfg, job, {"r0y0l1"}, {}, dst.id, "full", float(dst.T_ares_tcp[2, 3]) + rel,
                             job.approach_mm)
    b = cfg["brick"]                      # held stone bottom at the approach = neighbour pin tips + approach_clear_mm
    assert a == pytest.approx(b["pin_length"] + cfg["magtest"]["approach_clear_mm"] - rel + b["height"])
    assert a > job.approach_mm
    r.place_magazine(dst, T_base_ares, "full", approach_mm=a)
    name, body = link.blocks[-1]
    assert name == "mauer_place_mag"
    at, above = _pose_xyz_m(body, "pl_at"), _pose_xyz_m(body, "pl_above")     # local poses in the ARES frame [m]
    z_slot = float(dst.T_ares_tcp[2, 3])
    assert at[2] * 1000.0 == pytest.approx(z_slot + rel, abs=0.05)
    assert above[2] * 1000.0 == pytest.approx(z_slot + rel + a, abs=0.05)
    # nothing in the other stacks, or a stone below in the same stack: the job's approach
    assert magazine_approach_mm(cfg, job, set(), {}, dst.id, "full", 0.0, 150.0) == 150.0
    assert magazine_approach_mm(cfg, job, {"r0y1l1"}, {}, "r0y1l2", "full",
                                float(job.magazine.slot("r0y1l2").T_ares_tcp[2, 3]) + rel, 150.0) == 150.0


def test_sim_run_never_detours_via_the_park_pose_and_keeps_the_stone_clear(cfg, job1, tmp_path, monkeypatch):
    """Samuel 2026-10-09: no park pose between the front and the magazine; the held stone keeps [magtest]
    stone_clear_mm from every magazine stone on every joint move (the guard enforces it, park detours last)."""
    from mauer import motionguard as mg
    plans = []
    orig = mg.MotionGuard.plan

    def plan(self, q_from, q_to, holding=None, vias=(), column=None):
        v = orig(self, q_from, q_to, holding, vias, column)
        plans.append((self.park_last, self.stone_clear_mm, [np.allclose(q, self.park_q) for q in v.vias]))
        return v
    monkeypatch.setattr(mg.MotionGuard, "plan", plan)
    w, seq = _sim(cfg, job1, tmp_path)
    res = seq.run()
    assert res.state == "done" and len(res.placed) == len(_moves(job1)) and w.violations == []
    assert plans and all(p[0] and p[1] == cfg["magtest"]["stone_clear_mm"] for p in plans)
    assert not any(any(p[2]) for p in plans)                      # no via is the park pose
    rel = float(cfg["ur"]["release_above_mm"])
    from mauer.sequencer import read_log
    moves = [r for r in read_log(seq.log.path) if r["event"] == "magtest_move"]
    assert len(moves) == len(_moves(job1)) and all(r["release_above_mm"] == rel for r in moves)
    assert all(r["approach_put_mm"] >= job1.approach_mm and r["approach_pick_mm"] >= job1.approach_mm for r in moves)


def test_guard_park_last_and_stone_clearance(cfg, job):
    """MotionGuard.park_last moves the park detours behind the others; stone_clear_mm flags a held stone closer than
    that to a magazine stone outside the descent column (default off: wall jobs unchanged)."""
    gd = _guard(cfg, job)
    assert gd.park_last is False and gd.stone_clear_mm == 0.0
    from mauer.motionguard import GuardWorld
    slot = job.magazine.slot("r0y0l1")
    gd.set_world(GuardWorld(magazine=[slot]))
    from mauer.simworld import ik_near
    T_base_ares = g.inv(np.asarray(job.T_ares_base))
    # held stone 20 mm above the neighbour's pin tips, straight above slot r0y1l1 (5 mm beside r0y0l1)
    b = cfg["brick"]
    dst = job.magazine.slot("r0y1l1")
    T = T_base_ares @ g.transl(0.0, 0.0, b["pin_length"] + 20.0 + b["height"]) @ dst.T_ares_tcp
    q = ik_near(T @ g.inv(np.asarray(job.T_flange_tcp)), np.asarray(dst.qnear_rad, float))
    assert q is not None
    assert gd.state_problems(q, "full") == []                     # 20 mm > CLEARANCE_MM
    gd.stone_clear_mm = 30.0
    assert any("magazine stone r0y0l1" in p for p in gd.state_problems(q, "full"))
    assert gd.state_problems(q, None) == []                       # only a held stone keeps the larger distance


def test_wall_place_lets_go_above_the_place_pose_too(cfg, job):
    """Samuel 2026-10-09 "immer 10 mm drueber loslassen": place_wall opens [ur] release_above_mm above the place pose
    (wall frame z), like the magazine put-down."""
    rel = float(cfg["ur"]["release_above_mm"])
    link = _FakeLink(job.park_q_rad)
    r = URRobot(link, cfg, job, guard=_guard(cfg, job))
    T_base_ares = g.inv(np.asarray(job.T_ares_base))
    t = job.stops[0].stones[0]
    r.pick_magazine(job.magazine.slot(t.slot), T_base_ares)
    link.st.actual_q = np.asarray(job.park_q_rad, float)
    r.place_wall(T_base_ares, t)                                  # magtest: wall frame = ARES frame
    name, body = link.blocks[-1]
    assert name == "mauer_place_wall"
    at, above = _pose_xyz_m(body, "pl_at"), _pose_xyz_m(body, "pl_above")
    assert at[2] * 1000.0 == pytest.approx(t.T_wall_tcp[2, 3] + rel, abs=0.05)
    assert above[2] * 1000.0 == pytest.approx(t.T_wall_tcp[2, 3] + rel + job.approach_mm, abs=0.05)
    assert r.holding is None
