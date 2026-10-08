"""Pure parts of the HMI core (docs/HMI_DESIGN.md sections 8 and 9): current stone, action texts, run-log
formatting, initial snapshot, the enable matrix, the live ARES pose, the UR source in SIM / without a rig."""
from __future__ import annotations

import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from hmi.core.run_controller import ACTIONS, enables
from hmi.core.snapshot import (RunSnapshot, SnapshotTracker, category, current_stone, default_shot_view,
                               describe_action, format_event, severity, stone_index)
from hmi.core.sources import AresLive, UrSource, ares_live, compose_odometry, odom_from_status
from hmi_fakes import short_sim_session
from mauer.ares.ads import OdomPose
from mauer.reference import Pose2D

pytestmark = pytest.mark.usefixtures("no_lab_network")


def test_current_stone_follows_the_sequencer_loop():
    job = short_sim_session().job
    s0, s1 = job.stops[0].stones, job.stops[1].stones
    idx = stone_index(job)
    assert [idx[t.key] for t in job.stones()] == [1, 2, 3]
    st = current_stone(job, 0, set())
    assert st.key == s0[0].key and st.i == 1 and st.label == s0[0].label and st.kind == "full"
    assert current_stone(job, 0, {s0[0].key}).key == s0[1].key
    assert current_stone(job, 0, {t.key for t in s0}) is None
    assert current_stone(job, 1, {t.key for t in s0}).i == 3 == idx[s1[0].key]
    assert current_stone(job, None, set()) is None and current_stone(job, 5, set()) is None
    assert "stone 3/3" in current_stone(job, 1, set()).describe(3)


@pytest.mark.parametrize("rec, kind, frag", [
    ({"event": "robot", "action": "pick_magazine", "what": "pick magazine slot r1 (full)"}, "robot",
     "pick_magazine: pick magazine slot r1"),
    ({"event": "ares_cmd", "kind": "translate", "args": [0.0, 1000.0], "why": "stop 0 -> stop 1"}, "ares",
     "ARES translate dx +0.0 mm, dy +1000.0 mm (stop 0 -> stop 1)"),
    ({"event": "ares_cmd", "kind": "rotate", "args": [90.0], "why": "leg change"}, "ares", "ARES rotate +90.00 deg"),
    ({"event": "ares_move", "kind": "translate", "ok": True, "summary": "move #3 ..."}, "ares", "ok: move #3"),
    ({"event": "ares_move", "kind": "rotate", "ok": False, "summary": "aborted"}, "ares", "NOT ok: aborted"),
    ({"event": "route", "why": "stop 1 -> station"}, "route", "route: stop 1 -> station"),
    ({"event": "drive", "why": "stop 1 correction 1"}, "route", "drive: stop 1 correction 1"),
    ({"event": "resume_route", "route": {"why": "x", "next_leg": 2, "n_legs": 5}}, "route", "at leg 3/5"),
    ({"event": "shot", "look": "stop0-W0", "boards": {"W0": {"ok": True, "n_corners": 12, "rms_px": 0.01}}}, "shot",
     "image at look stop0-W0: W0 ok 12 corners 0.01 px"),
    ({"event": "shot", "look": "s", "boards": {"W1": {"ok": False, "reason": "not detected"}}}, "shot",
     "W1 NOT measured (not detected)"),
    ({"event": "wall_frame", "source": "camera", "stop": 1, "rms_mm": 0.12, "err_nominal_mm": 3.4}, "measure",
     "wall frame stop 1: rms 0.12 mm, 3.4 mm"),
    ({"event": "station_frame", "source": "dead_reckoning"}, "measure", "station frame (dead reckoning)"),
    ({"event": "reload_start", "stop": 0, "plan": [[1, 2, 3]]}, "station", "station trip from stop 0 (1 stones)"),
    ({"event": "station_empty"}, "wait", "refill"),
    ({"event": "station_refilled", "station": 12}, "station", "refilled (12 stones)"),
    ({"event": "run_start"}, "idle", "run started"),
    ({"event": "run_paused", "error": "run paused"}, "idle", "paused: run paused"),
])
def test_describe_action_table(rec, kind, frag):
    k, text = describe_action(rec)
    assert k == kind and frag in text


@pytest.mark.parametrize("ev", ["placed", "held", "stop_start", "warning", "pose_set", "station_estimate"])
def test_describe_action_keeps_the_previous_action(ev):
    assert describe_action({"event": ev}) is None


def test_severity_and_category():
    for ev in ("robot_error", "ares_error", "frame_jump", "run_error"):
        assert severity({"event": ev}) == "error" and category({"event": ev}) == "warning"
    for ev in ("warning", "interlock", "measurement_failed", "declined", "odometry_pose", "run_paused",
               "run_aborted"):
        assert severity({"event": ev}) == "warning"
    assert severity({"event": "ares_move", "ok": False}) == "warning"
    assert severity({"event": "ares_move", "ok": True}) == "info"
    assert [category({"event": e}) for e in ("robot", "shot", "placed", "run_start")] == \
        ["motion", "vision", "stones", "run"]


def test_format_event_is_one_short_line():
    rec = {"t": 1791400000.25, "event": "placed", "stop": 0, "stone": [0, 1], "slot": "r1y0l3",
           "T_base_tcp": [[1.0] * 4] * 4, "frame_src": "measured"}
    line = format_event(rec)
    assert "\n" not in line and line[8:12] == ".250" and "  placed  " in line
    assert "stone=[0, 1]" in line and "T_base_tcp" not in line
    pose = {"t": 0.0, "event": "pose_set", "pose": {"x_mm": 10.0, "y_mm": 20.0, "theta_rad": math.pi / 2}}
    assert "pose=(10, 20) mm +90.0 deg" in format_event(pose)
    long = {"t": 0.0, "event": "warning", "msg": "x" * 1000}
    assert len(format_event(long)) == 240 and format_event(long).endswith("...")
    shot = {"t": 0.0, "event": "shot", "look": "L", "boards": {"W0": {"ok": True, "n_corners": 4, "rms_px": 0.1}},
            "T_base_flange": [[0.0] * 4] * 4}
    assert "look L: W0 ok 4 corners 0.10 px" in format_event(shot)


def test_initial_snapshot():
    s = short_sim_session()
    snap = RunSnapshot.initial(s, start_stop=1)
    assert snap.seq_state == "idle" and snap.n_stops == 2 and snap.n_stones == 3 and snap.placed == frozenset()
    assert snap.pose_est == s.job.stops[1].ares and snap.pose_src == "start mark" and snap.pose_status == "ok"
    assert set(snap.magazine) == set(s.job.magazine.initial_fill) and len(snap.station) == len(
        s.job.station.take_order)
    assert snap.stone.i == 3 and snap.held is None and snap.n_placed == 0


def test_tracker_keeps_the_ares_command_until_the_move_ends():
    s = short_sim_session()
    odom = OdomPose(100.0, 0.0, 0.0)
    tr = SnapshotTracker(s, lambda: odom)
    from mauer.job import SlotState
    from mauer.sequencer import RunResult
    seq = SimpleNamespace(result=RunResult(state="running"), stop_k=0, placed=set(), at_station=False,
                          route_progress=None, pose_est=s.job.stops[0].ares, pose_src="measured", pose_status="ok",
                          T_wall_station=np.eye(4), magazine=SlotState.magazine(s.job.magazine),
                          station=SlotState.station(s.job.station), held=None, paused=False,
                          log=SimpleNamespace(path="x/run.jsonl"))
    snap = tr.record({"event": "ares_cmd", "kind": "translate", "args": [0.0, 1000.0], "why": "w"}, seq)
    assert snap.ares_cmd["odom"] == odom and snap.ares_cmd["pose"] == seq.pose_est and snap.action_kind == "ares"
    snap = tr.record({"event": "held", "held": None}, seq)
    assert snap.ares_cmd is not None and snap.action.startswith("ARES translate")
    snap = tr.record({"event": "ares_move", "kind": "translate", "ok": True, "summary": "s"}, seq)
    assert snap.ares_cmd is None and snap.n_events == 3 and snap.last_event == "ares_move"
    snap = tr.record({"event": "wall_frame", "source": "camera", "stop": 0, "rms_mm": 0.1, "err_nominal_mm": 1.0},
                     seq)
    assert snap.last_measurement["rms_mm"] == 0.1 and snap.stone.i == 1


def test_enable_matrix():
    e = enables("empty", None)
    assert e["load"][0] and e["build"][0] and not e["prepare"][0] and e["halt"][0] and e["step"][0]
    assert set(e) == set(ACTIONS)
    e = enables("loaded", "sim")
    assert e["prepare"][0] and not e["start"][0]
    e = enables("loaded", "real")
    assert not e["prepare"][0] and "--ares" in e["prepare"][1]
    assert enables("loaded", "real", ares_enabled=True)["prepare"][0]
    assert enables("ready", "sim", has_seq=True)["start"][0]
    assert not enables("ready", "real", has_seq=True)["start"][0]
    assert enables("ready", "real", has_seq=True, preflight_ok=True)["start"][0]
    assert not enables("ready", "sim", has_seq=False)["start"][0]
    e = enables("running", "sim", has_seq=True)
    assert e["pause"][0] and e["abort"][0] and not e["load"][0] and not e["release"][0] and not e["resume"][0]
    assert enables("pausing", "sim")["abort"][0] and not enables("pausing", "sim")["pause"][0]
    for st in ("paused", "aborted", "error"):
        e = enables(st, "real", has_seq=True, pose_status="odometry", held=True, odom_moved=True)
        assert all(e[a][0] for a in ("resume", "confirm_pose", "set_pose", "apply_odometry", "clear_held", "recheck",
                                     "grab", "release"))
        e = enables(st, "sim", has_seq=True)
        assert not e["confirm_pose"][0] and not e["clear_held"][0] and not e["apply_odometry"][0]
        assert not e["recheck"][0] and not e["grab"][0] and e["set_pose"][0]
    assert enables("done", "real")["grab"][0] and not enables("done", "real")["recheck"][0]
    assert enables("done", "sim")["release"][0] and not enables("done", "sim")["resume"][0]


def test_ares_live_follows_the_odometry_during_a_real_move():
    s = short_sim_session()
    snap = RunSnapshot.initial(s)
    p0 = Pose2D(1000.0, 840.0, -math.pi / 2)
    o0 = OdomPose(5000.0, 2000.0, 90.0)
    st = {"fPosX_m": 5.0, "fPosY_m": 2.3, "fPosTheta_deg": 90.0}          # odometry y + 300 mm
    live = ares_live(replace(snap, pose_est=p0, pose_src="predicted", ares_cmd={"pose": p0, "odom": o0}), st)
    assert live.src == "estimate+odometry" and live.odom == odom_from_status(st)
    # odometry heading 90 deg: + 300 mm in odometry y = 300 mm forward (body x); ARES faces -y in the wall frame
    assert (live.pose.x_mm, live.pose.y_mm) == pytest.approx((1000.0, 540.0), abs=1e-6)
    assert compose_odometry(p0, o0, o0) == pytest.approx(p0)
    idle = ares_live(snap, st)
    assert idle.src == "start mark" and idle.pose == snap.pose_est
    world = SimpleNamespace(ares_true=Pose2D(1.0, 2.0, 0.0), ares=SimpleNamespace(odom=OdomPose(0.0, 0.0, 0.0)))
    sim = ares_live(None, None, world)
    assert sim == AresLive(world.ares_true, "sim truth", "ok", world.ares_true, world.ares.odom)
    assert ares_live(None, None).pose is None


def test_ur_source_without_rig_and_in_sim():
    s = short_sim_session()
    none = UrSource(None, s.job).snapshot()
    assert none.source == "none" and np.allclose(none.q_rad, s.job.park_q_rad) and none.T_base_tcp_mm is None
    from hmi.core.rigs import SimRig
    from hmi.core.run_controller import RunOptions
    rig = SimRig(s.cfg, s.job, RunOptions("sim", scenario="none"))
    src = UrSource(rig.info(), s.job)
    snap = src.snapshot()
    assert snap.source == "sim" and snap.parked and snap.held_kind is None and np.allclose(snap.q_rad,
                                                                                           s.job.park_q_rad)
    look = s.job.stops[0].looks[0]
    rig.world.robot.goto_look(look)                                        # Cartesian move: SimRobot.q is None
    assert rig.world.robot.q is None
    snap = src.snapshot()
    from mauer.simworld import ur5_fk
    assert not snap.parked and np.allclose(ur5_fk(snap.q_rad), rig.world.robot.T_bf, atol=1e-6)
    assert np.allclose(snap.T_base_tcp_mm, rig.world.robot.T_bf @ rig.world.robot.T_flange_tcp)
    rig.close()


def test_default_shot_view_scales_and_converts():
    img = (np.arange(40 * 60) % 256).astype(np.uint8).reshape(40, 60)
    v = default_shot_view(img, {"event": "shot", "look": "L", "parent": "wall"}, {}, 0.5)
    assert v.preview.shape == (20, 30) and v.full_size == (60, 40) and v.look == "L" and v.parent == "wall"
    v16 = default_shot_view(img.astype(np.uint16) * 16, None, {}, 0.25)
    assert v16.preview.dtype == np.uint8 and v16.look == "live" and v16.rec is None
