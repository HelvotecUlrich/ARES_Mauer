"""REAL rig and preflight with fakes only (docs/HMI_DESIGN.md sections 8.1, 8.3): StubLink for the UR, AresAds on
the fake PLC with an emulated HMI heartbeat, StubCamera, nominal calibration. Nothing outside loopback is contacted
(no_lab_network) and no robot program is run."""
from __future__ import annotations

import copy

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
        assert wait_until(lambda: c.state == "ready", 30.0, qapp), c.state
        assert c.rig.mode == "real" and c.sequencer is not None and reports and not reports[-1].ok
        ok, why = c.can("start")
        assert not ok and "no override" in why
        c.start()                                                           # refused: nothing runs
        assert c.state == "ready"
        assert c.can("grab")[0]
        shots = []
        c.shot.connect(shots.append)
        c.grab()
        assert wait_until(lambda: shots, 10.0, qapp) and shots[0].look == "live" and shots[0].rec is None
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
