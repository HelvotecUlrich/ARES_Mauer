"""STATUS widgets offscreen (docs/HMI_DESIGN.md section 11.2): the UR panel on a stub URLink (REAL texts, safety
colours, joints in deg, stale RTDE) and on the simulated robot, the Wall pose tab (live odometry pose during an ARES
command, SIM truth error, route / dock target, camera fits), the status strip, the run feedback helpers, and a short
SIM run with a station trip through the main window (placed, trips done / planned, warnings, SIM placement).
No hardware: FakeRunController / the real RunController on mauer.simworld, StubLink, the no_lab_network fixture."""
from __future__ import annotations

import json
import math
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from hmi.core.rigs import RigInfo, SimRig
from hmi.core.run_controller import RunOptions
from hmi.core.session import JobSession
from hmi.core.snapshot import RunSnapshot
from hmi.core.sources import compose_odometry
from hmi.views.ares_panel import AresPanel, pose_text, target_of
from hmi.views.feedback import RunFeedback, issue_text, placement_text, planned_trips
from hmi.views.status_strip import StatusStrip
from hmi.views.ur_panel import BAD, OK, WARN, UrPanel, UrPoller, ur_short
from hmi_fakes import (FakeRunController, StubLink, make_ctx, make_window, short_job, short_sim_session,
                       straight_job10, wait_until)
from mauer import config
from mauer.ares.ads import OdomPose
from mauer.reference import Pose2D

pytestmark = pytest.mark.usefixtures("no_lab_network")

PARK_DEG = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0)          # StubLink default joints


@pytest.fixture
def fake_ctx(qapp, tmp_path):
    """(HmiContext with ARES enabled, FakeRunController) - the states are set by the test."""
    fake = FakeRunController(config.load()["hmi"], ares_enabled=True)
    ctx = make_ctx(qapp, tmp_path, controller=fake, ares_enabled=True)
    yield ctx, fake
    ctx.run_shutdown_hooks()                               # stops the shared UR poller


def real_rig(session, link, robot=None) -> RigInfo:
    return RigInfo("real", session.cfg, session.job, robot, link, None, None, "ids")


def stub_robot(holding=None):
    """URRobot stand-in with the attributes UrSource / the payload line read (commanded values only)."""
    return SimpleNamespace(tool_kg=1.68, tool_cog=[0.0, 0.0, 60.0], park_q=np.radians(PARK_DEG), park_tol_rad=0.01,
                           holding=holding, payloads={"full": (4.12, [0.0, 0.0, 101.0])})


def colour(w) -> str:
    return w.styleSheet()


# ── UR panel ──────────────────────────────────────────────────────────────────
def test_ur_panel_real_on_a_stub_link(fake_ctx, qapp):
    ctx, fake = fake_ctx
    s = short_sim_session()
    fake.set_session(s)
    panel = UrPanel(ctx)
    panel.show()
    assert UrPoller.of(ctx) is panel.poller                # one poller per context (shared with the strip)
    assert panel.poller.interval_ms == round(1000 / float(ctx.hmi["ur_poll_hz"]))
    link = StubLink(digital_outputs=0b10, age_s=0.05)      # DO1 (close) on, DO0 (open) off
    fake.rig = real_rig(s, link, stub_robot(holding="full"))
    fake.rig_changed.emit(fake.rig)
    qapp.processEvents()
    assert panel.source_lbl.text().startswith("RTDE") and panel.age_lbl.text() == "0.05 s"
    assert BAD not in colour(panel.age_lbl) and panel.version_lbl.text() == "3.15.8.0"
    assert panel.robot_mode_lbl.text() == "RUNNING" and OK in colour(panel.robot_mode_lbl)
    assert panel.safety_lbl.text() == "NORMAL" and OK in colour(panel.safety_lbl)
    assert panel.runtime_lbl.text() == "STOPPED" and panel.power_lbl.text() == "yes"
    assert panel.program_lbl.text() == "no" and panel.parked_lbl.text() == "yes"
    assert [w.text() for w in panel.joint_lbls] == [f"{v:+.2f}" for v in PARK_DEG]
    # StubLink TCP: (0.3, 0.1, 0.4) m, rotation vector (0, 3.14, 0) rad
    assert [w.text() for w in panel.tcp_lbls[:3]] == ["+300.0", "+100.0", "+400.0"]
    assert float(panel.tcp_lbls[4].text()) == pytest.approx(math.degrees(3.14), abs=0.01)
    assert panel.do_open_lbl.text() == "off" and panel.do_close_lbl.text() == "ON" and OK in colour(panel.do_close_lbl)
    assert panel.voltage_lbl.text() == "24 V"
    assert panel.payload_lbl.text() == "tool 1.68 kg, CoG (0, 0, 60) mm; with the full stone 4.12 kg"
    # a protective stop and a stale RTDE stream are red
    link.safety_mode, link.age_s = 3, 2.0
    panel.poller.poll()
    assert panel.safety_lbl.text() == "PROTECTIVE_STOP" and BAD in colour(panel.safety_lbl)
    assert panel.age_lbl.text() == "2.00 s - STALE" and BAD in colour(panel.age_lbl)
    assert ur_short(panel.poller.last) == ("UR: RUNNING / PROTECTIVE_STOP (RTDE stale)", BAD)
    # last robot action and last error from the run log
    t = time.time()
    fake.event.emit({"event": "robot", "t": t, "action": "pick_magazine", "what": "pick magazine slot m0"})
    fake.event.emit({"event": "robot_error", "t": t, "action": "place_wall", "what": "place c0i0",
                     "error": "protective stop"})
    fake.snapshot = replace(RunSnapshot.initial(s), held={"from": "magazine", "slot": "m0", "kind": "full",
                                                         "stone": None, "unknown": True})
    fake.snapshot_changed.emit(fake.snapshot)
    qapp.processEvents()
    assert "pick_magazine: pick magazine slot m0" in panel.action_lbl.text()
    assert "place_wall: protective stop" in panel.error_lbl.text() and BAD in colour(panel.error_lbl)
    assert "UNKNOWN" in panel.held_lbl.text() and BAD in colour(panel.held_lbl)
    # hidden: no work on the poll; shown again: the latest state at once
    panel.hide()
    link.safety_mode = 1
    panel.poller.poll()
    assert panel.safety_lbl.text() == "PROTECTIVE_STOP"
    panel.show()
    assert panel.safety_lbl.text() == "NORMAL"
    # a new rig starts a new run log
    fake.rig_changed.emit(fake.rig)
    qapp.processEvents()
    assert panel.action_lbl.text() == "-"


def test_ur_panel_sim_and_without_rig(fake_ctx, qapp):
    ctx, fake = fake_ctx
    s = short_sim_session()
    fake.set_session(s)
    panel = UrPanel(ctx)
    panel.show()
    qapp.processEvents()
    assert panel.source_lbl.text().startswith("no rig") and ur_short(panel.poller.last)[0] == "UR: no rig"
    rig = SimRig(s.cfg, s.job, RunOptions("sim", scenario="none"))
    try:
        fake.rig = rig.info()
        fake.rig_changed.emit(fake.rig)
        qapp.processEvents()
        assert panel.source_lbl.text() == "SIM - no RTDE" and panel.robot_mode_lbl.text() == "RUNNING"
        assert [w.text() for w in panel.joint_lbls] == [f"{math.degrees(v):+.2f}" for v in s.job.park_q_rad]
        assert panel.payload_lbl.text() == "- (SIM: no payload)" and panel.do_open_lbl.text() == "-"
        assert ur_short(panel.poller.last)[0] == "UR: SIM"
    finally:
        rig.close()


# ── Wall pose ─────────────────────────────────────────────────────────────────
def test_ares_panel_follows_the_odometry_during_a_real_move(fake_ctx, qapp):
    ctx, fake = fake_ctx
    s = short_sim_session()
    fake.set_session(s)
    panel = AresPanel(ctx)
    panel.show()
    fake.set_state("running", mode="real")
    p0 = s.job.stops[0].ares
    o0 = OdomPose(5000.0, 2000.0, 90.0)
    snap = replace(RunSnapshot.initial(s), stop_k=0, pose_est=p0, pose_src="measured", seq_state="running",
                   ares_cmd={"kind": "translate", "args": [300.0, 0.0], "why": "stop 0 correction", "pose": p0,
                             "odom": o0})
    fake.snapshot = snap
    fake.snapshot_changed.emit(snap)
    st = {"fPosX_m": 5.0, "fPosY_m": 2.3, "fPosTheta_deg": 90.0, "eAmrState": 7, "bMoveActive": True}
    ctx.publish_ads_connection(True, "connected")
    ctx.publish_ads_status(st)
    qapp.processEvents()
    live = compose_odometry(p0, o0, OdomPose(5000.0, 2300.0, 90.0))
    assert panel.live_lbl.text().startswith(pose_text(live)) and "odometry" in panel.live_lbl.text()
    assert panel.plan._live is not None and panel.plan._live.src == "estimate+odometry"
    assert panel.est_lbl.text() == pose_text(p0) and "measured, status ok" in panel.est_src_lbl.text()
    assert panel.odom_lbl.text() == "x 5000.0 mm, y 2300.0 mm, +90.00 deg"
    assert panel.amr_lbl.text() == "MANUAL MODE - move active"
    assert panel.cmd_lbl.text() == "translate dx +300.0 mm, dy +0.0 mm (stop 0 correction)"
    assert panel.target_lbl.text().startswith("stop 0") and panel.to_target_lbl.text() == "0.0 mm, 0.00 deg"
    assert panel.truth_lbl.text() == "- (REAL: no ground truth)"
    # the move ends: no live pose; the last move and the counters
    fake.event.emit({"event": "ares_move", "t": time.time(), "kind": "translate", "args": [300.0, 0.0], "ok": False,
                     "why": "stop 0 correction", "n": 1, "summary": "ABORTED (HALT)"})
    snap = replace(snap, ares_cmd=None, ares_moves=1, pose_status="odometry", pose_src="odometry (move not ok)")
    fake.snapshot = snap
    fake.snapshot_changed.emit(snap)
    qapp.processEvents()
    assert panel.live_lbl.text() == "-" and "1 ARES moves" in panel.counts_lbl.text()
    assert "NOT ok: ABORTED (HALT)" in panel.last_move_lbl.text() and BAD in colour(panel.last_move_lbl)
    assert "#FFAA00" in colour(panel.est_lbl)                    # odometry status: amber


def test_ares_panel_sim_truth_route_dock_and_fits(fake_ctx, qapp):
    ctx, fake = fake_ctx
    s = short_sim_session()
    fake.set_session(s)
    panel = AresPanel(ctx)
    panel.show()
    p0 = s.job.stops[1].ares
    true = Pose2D(p0.x_mm + 30.0, p0.y_mm, p0.theta_rad)
    world = SimpleNamespace(ares_true=true, ares=SimpleNamespace(odom=OdomPose(12.0, -3.0, 1.5)))
    fake.rig = RigInfo("sim", s.cfg, s.job, None, None, None, world, "synth")
    fake.set_state("running", mode="sim")
    snap = replace(RunSnapshot.initial(s), stop_k=1, pose_est=p0, pose_src="measured", reloads=1,
                   route={"kind": "to_station", "stop": 1, "why": "stop 1 -> station", "next_leg": 1, "n_legs": 3,
                          "standoff_mm": 0.0})
    fake.snapshot = snap
    fake.snapshot_changed.emit(snap)
    qapp.processEvents()
    assert panel.truth_lbl.text() == pose_text(true)
    assert panel.truth_err_lbl.text() == "30.0 mm, 0.00 deg" and WARN in colour(panel.truth_err_lbl)   # > 20 mm
    assert panel.odom_lbl.text() == "x 12.0 mm, y -3.0 mm, +1.50 deg (SIM)" and panel.amr_lbl.text() == "SIM"
    assert panel.route_lbl.text() == "to the station (from stop 1): leg 2/3 (stop 1 -> station)"
    dock = s.job.station.dock_in_wall
    assert target_of(snap, s.job)[1] == pytest.approx(dock)
    assert panel.target_lbl.text() == "dock (station estimate)" and panel.target_pose_lbl.text() == pose_text(dock)
    fit = {"event": "wall_frame", "t": time.time(), "source": "camera", "kind": "wall", "stop": 1, "why": "arrival",
           "rms_mm": 0.12, "err_nominal_mm": 21.1, "err_nominal_deg": 0.13, "jump_mm": 4.2}
    fake.event.emit(fit)
    fake.event.emit({"event": "wall_frame", "t": time.time(), "source": "dead_reckoning", "stop": 1})
    fake.event.emit({"event": "station_estimate", "t": time.time(), "n": 2, "vs_nominal_mm": 20.8,
                     "vs_nominal_deg": 0.53})
    fake.snapshot_changed.emit(snap)
    qapp.processEvents()
    assert panel.fits.rowCount() == 1                      # camera fits only
    assert [panel.fits.item(0, i).text() for i in range(7)] == ["1", "wall", "arrival", "0.12", "21.1", "0.13", "4.2"]
    assert panel.station_lbl.text() == "20.8 mm, 0.53 deg (2 observations)"
    fake.rig_changed.emit(fake.rig)                        # a new rig: a new table
    qapp.processEvents()
    assert panel.fits.rowCount() == 0


# ── status strip ──────────────────────────────────────────────────────────────
def test_status_strip_texts(fake_ctx, qapp):
    ctx, fake = fake_ctx
    strip = StatusStrip(ctx)
    run, mode, stop, stone, action, ur, ares, twin = strip.texts()
    assert (run, mode, ur, ares, twin) == ("EMPTY", "-", "UR: no rig", "ARES: not connected", "Twin: off")
    s = short_sim_session()
    fake.snapshot = RunSnapshot.initial(s)
    fake.snapshot_changed.emit(fake.snapshot)
    fake.set_session(s)
    qapp.processEvents()
    first = s.job.stops[0].stones[0]
    assert strip.texts()[:5] == ["LOADED", "-", "stop - (0-1)", f"stone 1/3 {first.label}", "not started"]
    fake.set_state("running", mode="real")
    link = StubLink()
    fake.rig = real_rig(s, link, stub_robot())
    fake.rig_changed.emit(fake.rig)
    snap = replace(fake.snapshot, stop_k=0, seq_state="running", action="robot: place stone c0i0",
                   held={"from": "magazine", "slot": "m0", "kind": "full", "stone": None, "unknown": False})
    fake.snapshot_changed.emit(snap)
    ctx.publish_ads_connection(True, "connected")
    ctx.publish_ads_status({"eAmrState": 7, "bMoveActive": True})
    ctx.set_twin_state("running", "port 20630")
    qapp.processEvents()
    run, mode, stop, stone, action, ur, ares, twin = strip.texts()
    assert (run, mode, stop) == ("RUNNING", "REAL", "stop 0 (0-1)")
    assert action == "robot: place stone c0i0 [holding full]" and WARN in colour(strip.action_lbl)
    assert ur == "UR: RUNNING / NORMAL" and OK in colour(strip.ur_lbl)
    assert ares == "ARES: MANUAL MODE move" and twin == "Twin: running" and strip.twin_lbl.toolTip() == "port 20630"
    fake.halted = True
    fake.set_state("aborted", "HALT")
    ctx.publish_ads_connection(False, "lost")
    ctx.set_twin_state("lost", "RoboDK closed")
    qapp.processEvents()
    assert strip.texts()[0] == "ABORTED HALTED" and BAD in colour(strip.run_lbl)
    assert strip.texts()[6] == "ARES: not connected" and strip.texts()[7] == "Twin: lost"


def test_status_strip_without_ads(qapp, tmp_path):
    ctx = make_ctx(qapp, tmp_path, controller=FakeRunController(config.load()["hmi"]))
    try:
        assert StatusStrip(ctx).texts()[6] == "ARES: ADS off"
    finally:
        ctx.run_shutdown_hooks()


# ── run feedback ──────────────────────────────────────────────────────────────
def test_planned_trips_equal_the_job_plan():
    _cfg, job10 = straight_job10()
    assert planned_trips(job10) == (job10.meta["planned_reloads"], job10.meta["planned_station_refills"]) == (2, 1)
    trip = short_job(job10, n0=2, n1=1, fill=2)            # 2 stones in the magazine, 3 to place
    assert planned_trips(trip) == (1, 0) and planned_trips(trip, start_stop=1) == (0, 0)
    assert planned_trips(trip, stop_after=0) == (0, 0)


def test_issue_and_placement_texts():
    assert issue_text({"event": "warning", "msg": "keep the path clear"}) == "keep the path clear"
    assert issue_text({"event": "frame_jump", "what": "stop 1 wall", "d_mm": 41.25, "d_deg": 0.5}) == \
        "frame jump stop 1 wall: 41.2 mm / 0.50 deg"
    assert issue_text({"event": "run_error", "error": "CameraError: no frame"}) == "run error: CameraError: no frame"
    assert issue_text({"event": "ares_move", "kind": "rotate", "ok": False, "summary": "ABORTED"}) == \
        "ARES rotate not ok: ABORTED"
    text, col = placement_text({"n": 2, "seated": 1, "horiz_mm": {"n": 2, "mean": 1.0, "p95": 1.9, "max": 2.0},
                                "pin_mm": {"n": 2, "max": 0.5},
                                "by_kind": {"full": {"n": 2, "seated": 1}, "half": {"n": 0, "seated": 0}}})
    assert text == ("SIM placement: 2 stones, 1 seated; horizontal error mean 1.00 / p95 1.90 / max 2.00 mm; pin max "
                    "0.50 mm; full 2 (1 seated)") and col == BAD


def test_feedback_counts_warnings_and_preflight(fake_ctx, qapp):
    ctx, fake = fake_ctx
    s = short_sim_session()
    fb = RunFeedback(ctx)
    fake.snapshot = RunSnapshot.initial(s)
    fake.set_session(s)
    qapp.processEvents()
    assert fb.tiles["stones"].text() == "0 / 3 placed" and fb.tiles["time"].text() == "not started"
    assert fb.tiles["station"].text() == "0 / 0 (0 / 0)"
    from hmi.core.preflight import PreflightItem, PreflightReport
    fake.preflight_done.emit(PreflightReport("real", (PreflightItem("UR", "PolyScope 3.3", True),), time.time()))
    t0 = time.time() - 10.0
    for rec in ({"event": "run_start", "t": t0}, {"event": "warning", "t": t0 + 1, "msg": "dock error 31 mm"},
                {"event": "measurement_failed", "t": t0 + 2, "msg": "W1 not seen"},
                {"event": "robot_error", "t": t0 + 3, "action": "place_wall", "error": "protective stop"},
                {"event": "run_error", "t": t0 + 4, "error": "robot place_wall failed"}):
        fake.event.emit(rec)
    fake.message.emit("warning", "resume refused: a stone may be in the jaws")
    qapp.processEvents()
    assert fb.tiles["preflight"].text().startswith("REAL refused: 1 problems") and BAD in colour(fb.tiles["preflight"])
    assert fb.tiles["issues"].text() == "3 warnings, 2 errors" and BAD in colour(fb.tiles["issues"])
    items = [fb.issues.item(i).text() for i in range(fb.issues.count())]
    assert items[0].endswith("dock error 31 mm") and items[-1].endswith("HMI: resume refused: a stone may be in the "
                                                                         "jaws")
    assert time.strftime("%H:%M:%S", time.localtime(t0 + 1)) in items[0]
    assert "1 failed" in fb.tiles["camera"].text() and WARN in colour(fb.tiles["camera"])
    assert fb.tiles["time"].text().startswith("0:04 elapsed, 0:04 running")


def test_short_sim_run_with_a_station_trip_in_the_window(qapp, tmp_path):
    """The STATUS widgets in the main window during a real SIM run (3 stones, 2 in the magazine: one station trip)."""
    w, fw, ctx = make_window(qapp, tmp_path)
    w.show()                                                 # hidden tabs skip their work
    c = ctx.controller
    try:
        cfg, job10 = straight_job10()
        c.set_session(JobSession(cfg, None, short_job(job10, n0=2, n1=1, fill=2), None, "test", "short_trip"))
        qapp.processEvents()
        fb = w.mauer.feedback
        assert fb.tiles["station"].text() == "0 / 1 (0 / 0)"
        w.mauer.scenario.setCurrentText("none")
        w.mauer.sim_speed.setValue(0.0)
        w.mauer.prepare_btn.click()
        assert wait_until(lambda: w.mauer.start_btn.isEnabled(), 30.0, qapp)
        assert fb.tiles["preflight"].text().startswith("SIM (informative)")
        w.tabs.setCurrentWidget(w.wall_pose)                 # the Wall pose tab follows the run while shown
        w.mauer.start_btn.click()
        assert wait_until(lambda: w.status_strip.texts()[0] == "DONE", 60.0, qapp), w.status_strip.texts()
        assert w.status_strip.texts()[1:4] == ["SIM", "stop 1 (0-1)", "placed 3/3"]
        assert fb.tiles["stones"].text() == "3 / 3 placed" and OK in colour(fb.tiles["stones"])
        assert fb.tiles["station"].text() == "1 / 1 (0 / 0)"
        assert fb.tiles["time"].text().endswith("s / stone")
        assert not fb.sim_lbl.isHidden() and fb.sim_lbl.text().startswith("SIM placement: 3 stones, 3 seated")
        items = [fb.issues.item(i).text() for i in range(fb.issues.count())]
        assert any("operator's responsibility" in t for t in items)        # the route warning of the trip
        assert fb.summary_btn.isEnabled() and json.loads(fb._summary_path.read_text())["state"] == "done"
        kinds = [w.wall_pose.fits.item(r, 1).text() for r in range(w.wall_pose.fits.rowCount())]
        assert kinds.count("station") >= 1 and kinds.count("wall") >= 3   # arrival x 2, after reload
        assert w.wall_pose.station_lbl.text().endswith("observations)")
        assert "SIM" in w.wall_pose.odom_lbl.text() and w.wall_pose.truth_err_lbl.text().endswith("deg")
        w.tabs.setCurrentWidget(w.ur)
        qapp.processEvents()
        assert w.ur.source_lbl.text() == "SIM - no RTDE" and w.ur.parked_lbl.text() == "yes"
        assert "park" in w.ur.action_lbl.text() and w.ur.held_lbl.text() == "empty"
        w.mauer.release_btn.click()
        assert wait_until(lambda: w.status_strip.texts()[0] == "LOADED", 10.0, qapp)   # after the release snapshot
        assert fb.tiles["stones"].text() == "3 / 3 placed" and not fb.sim_lbl.isHidden()  # the board stays
        assert fb.issues.count() == len(items) and w.mauer.stone_lbl.text().endswith("placed 0/3")
    finally:
        w.close()
