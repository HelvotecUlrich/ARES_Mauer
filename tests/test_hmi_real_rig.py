"""REAL rig and preflight with fakes only (docs/HMI_DESIGN.md sections 8.1, 8.3): StubLink for the UR, AresAds on
the fake PLC with an emulated HMI heartbeat, StubCamera, nominal calibration. Nothing outside loopback is contacted
(no_lab_network) and no robot program is run."""
from __future__ import annotations

import copy
import math
import threading
import time

import numpy as np
import pytest

from fake_plc import FakePlc, HmiHeartbeat
from hmi.core import run_controller as rc
from hmi.core.preflight import IK_GUARD_33, PreflightItem, PreflightReport, real_preflight
from hmi.core.rigs import HaltGate, RealFactories, RealRig
from hmi.core.run_controller import MOVING, RunController, RunOptions
from hmi.core.sources import UrSource
from hmi_fakes import StubCamera, StubLink, short_sim_session, wait_until
from mauer import config
from mauer.ares.ads import AresAds
from mauer.camera.base import CameraError
from mauer.sequencer import SequencerAborted, read_log
from mauer.ur import script as urscript
from mauer.vision import intrinsics as _intrinsics

pytestmark = pytest.mark.usefixtures("no_lab_network")

MANUAL_STATUS = {"nIfVersion": 2, "eAmrState": 7, "bExtActive": False, "fPosX_m": 0.0, "fPosY_m": 0.0,
                 "fPosTheta_deg": 0.0}


class RecAds(AresAds):
    """AresAds on the fake PLC that records close() and abort() into the shared order list."""

    def __init__(self, cfg, plc: FakePlc, order: list):
        super().__init__(cfg, connection_factory=plc.factory("client"))
        self.order = order

    def close(self) -> None:
        self.order.append("ads.close")
        super().close()

    def abort(self) -> None:
        self.order.append("ads.abort")
        super().abort()


@pytest.fixture
def plc():
    p = FakePlc(realtime=True, hmi_heartbeat=False)
    hb = HmiHeartbeat(p).start()
    yield p
    hb.stop()


def factories(plc: FakePlc, order: list, **link_kw) -> RealFactories:
    return RealFactories(link=lambda cfg: StubLink(order=order, **link_kw),
                         ads=lambda cfg: RecAds(cfg["ares_ads"], plc, order),
                         camera=lambda cfg: StubCamera(order=order),
                         intrinsics=lambda cfg: _intrinsics.nominal(cfg),
                         handeye=lambda cfg: config.T_flange_cam_nominal(cfg))


def test_real_rig_opens_in_order_and_reports_every_blocker(plc):
    s = short_sim_session()
    order: list = []
    rig = RealRig(s.cfg, s.job, factories(plc, order, controller_version=(3, 3, 3, 176)))
    rig.open()
    assert rig.complete and rig.errors == {}
    assert rig.ads_check is not None and rig.ads_check.problems == []          # MANUAL, heartbeat changing
    rep = real_preflight(s.cfg, s.job, ares_enabled=True, ads_connected=True, ads_status=MANUAL_STATUS, rig=rig)
    texts = [(i.source, i.text) for i in rep.items]
    assert all(i.blocking for i in rep.items) and not rep.ok
    assert s.cfg["ur"]["ik_check"] == "get_inverse_kin"                        # station.toml: the 3.3 IK check
    assert ("UR", IK_GUARD_33) not in texts                                     # PolyScope 3.3.3 on the lab UR5
    cfg = {**s.cfg, "ur": {**s.cfg["ur"], "ik_check": "has_solution"}}         # the URSim 3.15 check on 3.3
    assert ("UR", IK_GUARD_33) in [(i.source, i.text) for i in real_preflight(
        cfg, s.job, ares_enabled=True, ads_connected=True, ads_status=MANUAL_STATUS, rig=rig).items]
    assert any(src == "job/config" and "nominal look poses" in t for src, t in texts)
    assert not any(src in ("HMI", "ARES", "camera") for src, _ in texts)
    assert rep.header().startswith("REAL refused")
    rig.close()
    assert order == ["camera.close", "ads.close", "link.stop"]


def test_preflight_lists_hmi_ur_and_camera_problems(plc):
    s = short_sim_session()
    cfg = copy.deepcopy(s.cfg)
    cfg["brick"]["mass_kg"] = 0.0
    order: list = []
    f = factories(plc, order, robot_mode=5, safety_mode=3, status_bits=0x3)
    f.camera = lambda c: (_ for _ in ()).throw(RuntimeError("no IDS device"))
    rig = RealRig(cfg, s.job, f)
    rig.open()
    assert rig.robot is None and "mass_kg" in rig.errors["robot"] and rig.camera is None
    rep = real_preflight(cfg, s.job, ares_enabled=True, ads_connected=True,
                         ads_status={**MANUAL_STATUS, "eAmrState": 6, "bExtActive": True}, rig=rig)
    text = "\n".join(f"[{i.source}] {i.text}" for i in rep.items)
    for frag in ("[job/config] [brick] mass_kg <= 0", "[HMI] AMR state 6 READY", "[HMI] external control",
                 "[UR] robot mode IDLE", "[UR] safety mode PROTECTIVE_STOP", "[UR] a program is running",
                 "[UR] URRobot not created: ValueError: [brick] mass_kg is 0", "[camera] camera not open"):
        assert frag in text, frag
    rep2 = real_preflight(cfg, s.job, ares_enabled=False, ads_connected=False, ads_status=None, rig=None)
    t2 = "\n".join(i.text for i in rep2.items)
    assert "without --ares" in t2 and "Prepare / Connect" in t2
    rig.close()


def test_stale_rtde_and_failed_connections(plc):
    s = short_sim_session()
    order: list = []
    f = factories(plc, order, age_s=3.0)
    f.ads = lambda cfg: (_ for _ in ()).throw(OSError("ARES PLC not reachable"))
    rig = RealRig(s.cfg, s.job, f)
    rig.open()
    rep = real_preflight(s.cfg, s.job, ares_enabled=True, ads_connected=True, ads_status=MANUAL_STATUS, rig=rig)
    text = "\n".join(f"[{i.source}] {i.text}" for i in rep.items)
    assert "[UR] no current RTDE sample (age 3.0 s)" in text
    assert "[ARES] AresAds not connected: OSError: ARES PLC not reachable" in text
    f2 = factories(plc, [], start_error=OSError("connection refused"))
    rig2 = RealRig(s.cfg, s.job, f2)
    rig2.open()
    assert rig2.link is None and rig2.robot is None and not rig2.complete
    rep2 = real_preflight(s.cfg, s.job, ares_enabled=True, ads_connected=True, ads_status=MANUAL_STATUS, rig=rig2)
    assert any(i.text.startswith("URLink not started: OSError: connection refused") for i in rep2.items)
    rig.close()
    rig2.close()


def test_halt_helper_stops_the_ur_and_aborts_ares_only_without_the_worker(plc):
    s = short_sim_session()
    order: list = []
    rig = RealRig(s.cfg, s.job, factories(plc, order))
    rig.open()
    text = rig.halt(ads_worker_connected=True)
    assert rig.link.calls.count("abort") == 1 and "ads.abort" not in order and "UR:" in text
    text = rig.halt(ads_worker_connected=False)
    assert rig.link.calls.count("abort") == 2 and order.count("ads.abort") == 1 and "AresAds" in text
    assert [w.value for w in plc.written("client") if w.name == "bCmdMoveAbort"] == [True, False]
    rig.close()


def test_ur_source_reads_the_link_only(plc):
    s = short_sim_session()
    rig = RealRig(s.cfg, s.job, factories(plc, [], digital_outputs=0b10))
    rig.open()
    snap = UrSource(rig.info(), s.job).snapshot()
    assert snap.source == "rtde" and snap.robot_mode == "RUNNING" and snap.safety_ok and snap.power_on
    assert snap.do_close is True and snap.do_open is False and snap.payload_kg == pytest.approx(1.68)
    assert np.degrees(snap.q_rad)[1] == pytest.approx(-90.0) and snap.controller_version == (3, 15, 8, 0)
    assert rig.link.calls == ["start"]                                       # no abort, no program
    rig.close()


def test_controller_real_prepare_refuses_start_and_releases_in_order(qapp, tmp_path, plc):
    s = short_sim_session()
    order: list = []
    status = {"st": dict(MANUAL_STATUS)}
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=True,
                      ads_status_fn=lambda: status["st"], ads_connected_fn=lambda: True,
                      real_factories=factories(plc, order, controller_version=(3, 3, 3, 176)))
    try:
        c.set_session(s)
        assert c.can("prepare", "real") == (True, "")
        reports = []
        c.preflight_done.connect(reports.append)
        c.prepare(RunOptions("real"))
        assert wait_until(lambda: c.state == "ready" and reports, 30.0, qapp), c.state
        assert c.rig.mode == "real" and c.sequencer is not None and not reports[-1].ok
        ok, why = c.can("start")
        assert not ok and "no override" in why
        c.start()                                                           # refused: nothing runs
        assert c.state == "ready"
        assert c.can("grab")[0]
        shots = []
        c.shot.connect(shots.append)
        c.grab()
        assert wait_until(lambda: shots, 10.0, qapp) and shots[0].look == "live" and shots[0].rec is None
        failed = []
        c.grab_failed.connect(failed.append)

        def no_frame():
            raise CameraError("no frame within 3000 ms")
        c._rig.camera.grab = no_frame                                       # the RealRig behind c.rig
        c.grab()                                                            # reported, the rig stays ready
        assert wait_until(lambda: failed, 10.0, qapp) and failed[0].startswith("CameraError: no frame")
        assert c.state == "ready" and len(shots) == 1
        c.halt()
        assert c.rig.link.inhibited == "HALT" and c.rig.ads.inhibited == "HALT"     # latched at once (GUI thread)
        assert wait_until(lambda: "abort" in c.rig.link.calls, 5.0, qapp)
        assert "ads.abort" not in order                                     # the HMI's worker is connected
        assert wait_until(lambda: any(r["event"] == "halt_result" and "UR: stub" in r["text"]
                                      for r in read_log(c.log_dir)), 5.0, qapp)   # what the helper did, logged
        seq = c.sequencer
        orig = seq.close
        seq.close = lambda: (order.append("seq.close"), orig())
        c.release()
        assert wait_until(lambda: c.state == "loaded", 10.0, qapp)
        assert order[-4:] == ["seq.close", "camera.close", "ads.close", "link.stop"]
    finally:
        assert c.shutdown(10.0)


def _ok_report(*a, **k) -> PreflightReport:
    return PreflightReport("real", (), time.time())


def test_real_start_rechecks_right_before_the_first_motion(qapp, tmp_path, plc, monkeypatch):
    """Review 2026-10-08: Start (REAL) runs the preflight again in the run thread - the one of Prepare may be old - and
    refuses while ARES moves; a HALT during these checks keeps the run from starting; Start lifts the HALT latch."""
    s = short_sim_session()
    status = {"st": dict(MANUAL_STATUS)}
    reports = {"next": _ok_report}
    monkeypatch.setattr(rc, "real_preflight", lambda *a, **k: reports["next"](*a, **k))
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=True,
                      ads_status_fn=lambda: status["st"], ads_connected_fn=lambda: True,
                      real_factories=factories(plc, []))
    msgs = []
    c.message.connect(lambda lvl, t: msgs.append(t))
    try:
        c.set_session(s)
        c.prepare(RunOptions("real", step=True))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp) and c.can("start") == (True, "")
        status["st"] = {**MANUAL_STATUS, "bMoveActive": True}             # a GO move of the ARES control tab
        assert c.can("start") == (False, MOVING)
        status["st"] = dict(MANUAL_STATUS)
        reports["next"] = lambda *a, **k: PreflightReport(
            "real", (PreflightItem("UR", "robot mode IDLE, needs RUNNING", True),), time.time())
        c.start()                                                           # Prepare's preflight was ok ...
        assert wait_until(lambda: any("start refused: preflight: 1 blocking" in t for t in msgs), 10.0, qapp)
        assert wait_until(lambda: c.state == "ready", 5.0, qapp)           # ... the one before the motion is not
        entered, go = threading.Event(), threading.Event()

        def slow_ok(*a, **k):
            entered.set()
            go.wait(10.0)
            return _ok_report()
        reports["next"] = slow_ok
        c._preflight = _ok_report()                                         # as shown after the Re-check
        c.halt()                                                            # latched ...
        assert c.rig.link.inhibited == "HALT"
        c.start()
        assert c.rig.link.inhibited is None and c.rig.ads.inhibited is None    # ... lifted by Start (GUI thread)
        assert wait_until(entered.is_set, 10.0, qapp)
        c.halt()                                                            # HALT while the checks run
        go.set()
        assert wait_until(lambda: any("HALT pressed during the start checks" in t for t in msgs), 10.0, qapp)
        assert wait_until(lambda: c.state == "ready", 5.0, qapp) and c.rig.link.inhibited == "HALT"
        assert "run_start" not in [r["event"] for r in read_log(c.log_dir)]   # nothing ran (StubLink: no program)
    finally:
        assert c.shutdown(10.0)


def test_real_jaws_empty_clears_the_robots_stone(qapp, tmp_path, plc):
    """Review 2026-10-08: 'Jaws empty' after a failed place also clears URRobot.holding - else the next park runs
    with the stone's payload and the motion guard plans around a stone that is not there."""
    s = short_sim_session()
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=True,
                      ads_status_fn=lambda: dict(MANUAL_STATUS), ads_connected_fn=lambda: True,
                      real_factories=factories(plc, []))
    try:
        c.set_session(s)
        c.prepare(RunOptions("real"))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp)
        robot = c.rig.robot                                                 # the real URRobot on the StubLink
        assert type(robot).__name__ == "URRobot" and isinstance(c.sequencer.robot, HaltGate)
        robot.holding = "full"                                              # as after a place that failed
        c.sequencer.held = {"from": "magazine", "slot": "m0", "kind": "full", "stone": None, "unknown": True}
        c._state = "aborted"
        assert c.can("clear_held") == (True, "")
        c.clear_held()
        assert wait_until(lambda: c.sequencer.held is None and robot.holding is None, 5.0, qapp)
        c._state = "ready"
    finally:
        assert c.shutdown(10.0)


def test_halt_while_the_robot_plans_sends_no_block(qapp):
    """Review 2026-10-08: HALT after the confirmation, while URRobot plans (IK, motion guard: tens of ms), must not
    let the block go out afterwards. The URLink latch refuses it; the gate turns that into an abort. A real URLink on
    the loopback fake controller of tests/test_ur_link.py."""
    from mauer.backends import URRobot
    from test_ur_link import FakeUR, make_link
    s = short_sim_session()
    fake = FakeUR(regs={24: 41, 25: 40})
    link = make_link(fake)
    try:
        robot = URRobot(link, s.cfg, s.job, guard=object())
        planning = threading.Event()

        def slow_guarded(q, name, vias=None, column=None):                 # MotionGuard.plan with detours
            planning.set()
            time.sleep(0.4)
            return []
        robot._qnear = lambda hint, T: np.zeros(6)
        robot._q_above = lambda *a: np.zeros(6)
        robot._guarded = slow_guarded
        halt = threading.Event()
        gate = HaltGate(robot, halt)
        slot = s.job.magazine.slot(s.job.magazine.initial_fill[0])
        out = {}

        def pick():
            try:
                gate.pick_magazine(slot, np.linalg.inv(s.job.T_ares_base), "full")
                out["ok"] = True
            except Exception as e:      # noqa: BLE001 - the result of the test
                out["exc"] = e
        th = threading.Thread(target=pick)
        th.start()
        assert planning.wait(5.0)
        halt.set()                                                          # RunController.halt(): flag, latch,
        link.inhibit("HALT")
        link.abort()                                                        # then the helper's abort
        th.join(10.0)
        assert isinstance(out.get("exc"), SequencerAborted) and "refused before anything was sent" in str(out["exc"])
        assert fake.programs == [urscript.abort_program()]                 # the pick block never left the laptop
        assert robot.holding is None
    finally:
        link.stop()
        fake.close()


def test_a_failed_prepare_closes_the_rig(qapp, tmp_path, plc):
    s = short_sim_session()
    order: list = []
    blocker = tmp_path / "blocker"
    blocker.write_text("a file where the run log folder should go")
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=True,
                      ads_status_fn=lambda: dict(MANUAL_STATUS), ads_connected_fn=lambda: True,
                      real_factories=factories(plc, order))
    msgs = []
    c.message.connect(lambda lvl, t: msgs.append((lvl, t)))
    try:
        c.set_session(s)
        c.prepare(RunOptions("real", log_dir=blocker / "run"))
        assert wait_until(lambda: c.state == "loaded" and msgs, 30.0, qapp), (c.state, msgs)
        assert msgs[-1][0] == "error" and "prepare failed" in msgs[-1][1]
        assert c.rig is None and c.sequencer is None
        assert order == ["camera.close", "ads.close", "link.stop"]       # nothing left open
    finally:
        assert c.shutdown(10.0)


def test_real_resume_checks_follow_the_odometry(qapp, tmp_path, plc):
    s = short_sim_session()
    st = {"st": dict(MANUAL_STATUS)}
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=True,
                      ads_status_fn=lambda: st["st"], ads_connected_fn=lambda: True,
                      real_factories=factories(plc, []))
    msgs = []
    c.message.connect(lambda lvl, t: msgs.append(t))
    try:
        c.set_session(s)
        c.prepare(RunOptions("real"))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp)
        c._state = "paused"                       # as after a REAL run that stopped (the preflight blocks a start)
        c.sequencer.pose_est = s.job.stops[0].ares
        c.resume()
        assert wait_until(lambda: any("odometry at the stop is unknown" in t for t in msgs), 10.0, qapp)
        assert wait_until(lambda: c.state == "paused", 5.0, qapp)
        c.set_pose(s.job.stops[0].ares)          # the operator vouches for the pose: odometry reference = now
        assert wait_until(lambda: c._odom_at_stop is not None, 5.0, qapp)
        assert c.odom_moved() is None and not c.can("apply_odometry")[0]
        st["st"] = {**MANUAL_STATUS, "fPosX_m": 0.05}                    # ARES jogged 50 mm forward (odometry x):
        plc.x_mm = 50.0                          # the HMI worker's status and the PLC the run's AresAds reads
        assert c.odom_moved() == pytest.approx((50.0, 0.0, 0.0)) and c.can("apply_odometry") == (True, "")
        c.resume()
        assert wait_until(lambda: any("ARES moved by (+50.0 mm" in t for t in msgs), 10.0, qapp)
        assert wait_until(lambda: c.state == "paused", 5.0, qapp)
        p0 = c.sequencer.pose_est
        c.apply_odometry()
        assert wait_until(lambda: c.sequencer.pose_est != p0, 5.0, qapp)
        p1 = c.sequencer.pose_est
        assert math.hypot(p1.x_mm - p0.x_mm, p1.y_mm - p0.y_mm) == pytest.approx(50.0, abs=1e-6)
        assert wait_until(lambda: c.odom_moved() is None, 5.0, qapp)    # the job sets the new reference last
        c.resume()                                # odometry ok now: the blocking preflight (job, PolyScope) refuses
        assert wait_until(lambda: any("blocking problems" in t for t in msgs), 10.0, qapp)
        assert wait_until(lambda: c.state == "paused", 5.0, qapp)
        c._state = "ready"
    finally:
        assert c.shutdown(10.0)


def test_real_start_checklist_and_a_later_start(qapp, tmp_path, plc, monkeypatch):
    """Test plan Anhang C1 / C2 (2026-10-08), as tools/run_job.py --real: Start (REAL) shows the magazine fill and
    waits for the operator's checklist (jaws empty, magazine as listed, station full) before the first motion - a
    decline starts nothing; a start at stop k > 0 needs the run log of the interrupted run (its stones are declared
    standing, the fill is the one for the stones after them) or the confirmation that stop k is untouched."""
    import json

    from mauer import job as mjob
    s = short_sim_session(n0=2, n1=2)
    monkeypatch.setattr(rc, "real_preflight", _ok_report)
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=True,
                      ads_status_fn=lambda: dict(MANUAL_STATUS), ads_connected_fn=lambda: True,
                      real_factories=factories(plc, []))
    msgs, asks = [], []
    c.message.connect(lambda lvl, t: msgs.append(t))
    c.confirm_requested.connect(asks.append)
    try:
        c.set_session(s)
        c.prepare(RunOptions("real", start_stop=1))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp)
        c.start()
        assert wait_until(lambda: any("start refused: start at stop 1: choose the run log" in t for t in msgs),
                          10.0, qapp)
        assert wait_until(lambda: c.state == "ready", 5.0, qapp) and asks == []
        c.release()
        assert wait_until(lambda: c.state == "loaded", 10.0, qapp)
        old = tmp_path / "old_run"
        old.mkdir()
        key = s.job.stops[1].stones[0].key
        (old / "run.jsonl").write_text(json.dumps({"event": "placed", "stone": list(key)}) + "\n", encoding="utf-8")
        c.prepare(RunOptions("real", start_stop=1, resume_log=old))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp)
        assert key in c.sequencer.placed
        standing = {t.key for t in s.job.stops[0].stones} | {key}
        assert c.start_fill() == mjob.restart_fill(s.job, standing) and c.start_fill()
        c.start()
        assert wait_until(lambda: asks, 10.0, qapp)
        req = asks[-1]
        assert req.kind == "start" and "1 stones of old_run stand" in req.text and "jaws EMPTY" in req.text
        assert all(f"{sid}: {kind}" in req.text for sid, kind in c.start_fill())
        c.answer_confirm(req.id, False)
        assert wait_until(lambda: any("start checklist not confirmed" in t for t in msgs), 10.0, qapp)
        assert wait_until(lambda: c.state == "ready", 5.0, qapp)
        assert "run_start" not in [r["event"] for r in read_log(c.log_dir)]   # nothing ran (StubLink: no program)
    finally:
        assert c.shutdown(10.0)
