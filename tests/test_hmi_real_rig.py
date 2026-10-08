"""REAL rig and preflight with fakes only (docs/HMI_DESIGN.md sections 8.1, 8.3): StubLink for the UR, AresAds on
the fake PLC with an emulated HMI heartbeat, StubCamera, nominal calibration. Nothing outside loopback is contacted
(no_lab_network) and no robot program is run."""
from __future__ import annotations

import copy
import math

import numpy as np
import pytest

from fake_plc import FakePlc, HmiHeartbeat
from hmi.core.preflight import IK_GUARD_33, real_preflight
from hmi.core.rigs import RealFactories, RealRig
from hmi.core.run_controller import RunController, RunOptions
from hmi.core.sources import UrSource
from hmi_fakes import StubCamera, StubLink, short_sim_session, wait_until
from mauer import config
from mauer.ares.ads import AresAds
from mauer.camera.base import CameraError
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
    assert ("UR", IK_GUARD_33) in texts                                         # PolyScope 3.3.3 on the lab UR5
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
        assert wait_until(lambda: "abort" in c.rig.link.calls, 5.0, qapp)
        assert "ads.abort" not in order                                     # the HMI's worker is connected
        seq = c.sequencer
        orig = seq.close
        seq.close = lambda: (order.append("seq.close"), orig())
        c.release()
        assert wait_until(lambda: c.state == "loaded", 10.0, qapp)
        assert order[-4:] == ["seq.close", "camera.close", "ads.close", "link.stop"]
    finally:
        assert c.shutdown(10.0)


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
        st["st"] = {**MANUAL_STATUS, "fPosX_m": 0.05}                    # ARES jogged 50 mm forward (odometry x)
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
