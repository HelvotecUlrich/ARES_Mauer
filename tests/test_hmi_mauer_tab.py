"""The Mauer main window and its CORE widgets, offscreen (docs/HMI_DESIGN.md section 10): a SIM run through the
Mauer tab, the step-mode ConfirmBar, HALT keys with a pending confirmation and from a child window, the run lock of
the amr ARES control tab, the close refusal, the window title, the plan view and the run log."""
from __future__ import annotations

import time

import pytest
from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QApplication, QDialog

from hmi.amr.ui import constants as C
from hmi.amr.ui.control_widget import RUN_LOCK_BANNER
from hmi.core.ads_link import ADS_OFF, NullAdsWorker
from hmi.core.run_controller import ConfirmRequest
from hmi.core.session import JobSession
from hmi.main_window import CLOSE_REFUSED, RESET_LOCK, RUN_LOCK
from hmi.views.confirm_bar import HELD_NOTE, PENDING_NOTE
from hmi.views.plan_view import plan_geometry
from hmi_fakes import FakeRunController, make_window, short_sim_session, wait_until
from mauer import config

pytestmark = pytest.mark.usefixtures("no_lab_network")


def _key(w, key: int, press: bool = True) -> bool:
    ev = QKeyEvent(QEvent.KeyPress if press else QEvent.KeyRelease, key, Qt.NoModifier, "", False)
    return w.eventFilter(w, ev)


def status(**kw):
    from hmi.amr import plc_vars as pv
    d = pv.default_status(pv.IF_V2)
    d.update({"bPlcRunning": True, "bHmiWatchdogOK": True, "nPlcHeartbeat": 1, "nIfVersion": 2,
              "sPlcBuild": "ARES CX9240 v2 2026-09-25"})
    d.update(kw)
    return d


@pytest.fixture
def real_win(qapp, tmp_path):
    """MainWindow with the real RunController (SIM runs)."""
    w, fw, ctx = make_window(qapp, tmp_path)
    yield w, fw, ctx
    if ctx.controller.state in ("running", "pausing", "aborting"):
        ctx.controller.halt()
        wait_until(lambda: ctx.controller.state not in ("running", "pausing", "aborting"), 30.0, qapp)
    w.close()


@pytest.fixture
def fake_win(qapp, tmp_path):
    """MainWindow with a FakeRunController (states set by the test)."""
    fake = FakeRunController(config.load()["hmi"], ares_enabled=True)
    w, fw, ctx = make_window(qapp, tmp_path, controller=fake, ares_enabled=True)
    yield w, fw, fake
    fake.state = "empty"
    w.close()


def test_sim_run_through_the_mauer_tab(real_win, qapp):
    w, fw, ctx = real_win
    c, tab = ctx.controller, w.mauer
    c.set_session(short_sim_session())
    qapp.processEvents()
    assert "short_2_1" in w.windowTitle() and "config main" in w.windowTitle()
    assert "2 stops, 3 stones" in tab.summary.text() and tab.prepare_btn.isEnabled() and not tab.start_btn.isEnabled()
    assert not tab.real_rb.isEnabled()                     # ARES off
    tab.scenario.setCurrentText("none")
    tab.sim_speed.setValue(0.0)
    tab.prepare_btn.click()
    assert wait_until(lambda: c.state == "ready", 30.0, qapp)
    assert wait_until(lambda: tab.start_btn.isEnabled(), 5.0, qapp)
    assert tab.preflight_hdr.text().startswith("SIM (informative)") and tab.preflight_list.count() >= 1
    assert " - SIM" in w.windowTitle()
    tab.start_btn.click()
    assert wait_until(lambda: c.state == "done", 60.0, qapp), c.state
    assert wait_until(lambda: "placed 3/3" in tab.stone_lbl.text(), 5.0, qapp), tab.stone_lbl.text()
    assert tab.state_lbl.text() == "DONE" and tab.release_btn.isEnabled() and not tab.pause_btn.isEnabled()
    assert tab.plan._snap is not None and len(tab.plan._snap.placed) == 3
    lines = w.run_log.lines()
    assert any("  run_start  " in ln for ln in lines) and any("  run_done  " in ln for ln in lines)
    w.run_log.filters["vision"].setChecked(False)
    assert not any("  shot  " in ln for ln in w.run_log.lines())
    w.grab()                                               # paint every widget once (plan view with stones)
    tab.release_btn.click()
    assert wait_until(lambda: c.state == "loaded", 10.0, qapp)


def test_confirm_bar_arms_after_the_delay(fake_win, qapp):
    w, fw, fake = fake_win
    bar = w.confirm_bar
    arm_s = float(w._ctx.hmi["confirm_arm_s"])
    req = ConfirmRequest(7, "motion", "robot: place stone c0i0", {"from": "magazine", "slot": "s"}, False,
                         time.time())
    fake.confirm_requested.emit(req)
    qapp.processEvents()
    assert bar.isVisibleTo(w) and not bar.go.isEnabled() and bar.decline.isEnabled()
    assert "place stone" in bar._text.text() and HELD_NOTE in bar._note.text()
    bar.go.click()
    assert fake.of("answer_confirm") == []                 # not armed yet
    assert wait_until(lambda: bar.go.isEnabled(), arm_s + 1.0, qapp)
    bar.go.click()
    assert fake.of("answer_confirm") == [(7, True)] and not bar.go.isEnabled()
    fake.confirm_cleared.emit(7)
    qapp.processEvents()
    assert not bar.isVisibleTo(w)
    fake.confirm_requested.emit(ConfirmRequest(8, "station_empty", "refill", None, True, time.time()))
    qapp.processEvents()
    assert bar.go.text() == "Refilled" and bar.decline.text() == "Abort run" and PENDING_NOTE in bar._note.text()
    bar.decline.click()
    assert fake.of("answer_confirm")[-1] == (8, False)


def test_space_while_a_step_waits_halts_and_releases_it(real_win, qapp, monkeypatch):
    w, fw, ctx = real_win
    c = ctx.controller
    c.set_session(short_sim_session())
    w.mauer.scenario.setCurrentText("none")
    w.mauer.sim_speed.setValue(0.0)
    w.mauer.step.setChecked(True)
    w.mauer.prepare_btn.click()
    assert wait_until(lambda: w.mauer.start_btn.isEnabled(), 30.0, qapp)      # after the queued state signal
    w.mauer.start_btn.click()
    assert wait_until(lambda: w.confirm_bar.isVisibleTo(w), 30.0, qapp)
    monkeypatch.setattr(w, "isActiveWindow", lambda: True)
    assert _key(w, Qt.Key_Space) is True
    assert fw.of("halt") == [None]                         # the ADS HALT first
    assert wait_until(lambda: not w.confirm_bar.isVisibleTo(w), 1.0, qapp)
    assert wait_until(lambda: c.state == "aborted", 30.0, qapp) and c.halted and w.mauer.halted_lbl.isVisibleTo(w)
    assert w.mauer.step.isChecked()                        # step mode forced on for the resume


def test_space_in_an_hmi_child_window_is_halt(fake_win, qapp, monkeypatch):
    w, fw, fake = fake_win
    dlg = QDialog(w)
    monkeypatch.setattr(w, "isActiveWindow", lambda: False)
    monkeypatch.setattr(QApplication, "activeWindow", staticmethod(lambda: dlg))
    assert _key(w, Qt.Key_Escape) is True
    assert fw.of("halt") == [None] and fake.of("halt") == [()]
    other = QDialog()                                       # a window that is not ours: not handled
    monkeypatch.setattr(QApplication, "activeWindow", staticmethod(lambda: other))
    assert _key(w, Qt.Key_Space) is False and len(fw.of("halt")) == 1


def test_real_run_locks_jog_go_and_the_odometry_reset(fake_win, qapp):
    w, fw, fake = fake_win
    ctl = w.control
    fw.status.emit(status(eAmrState=C.ST_MANUAL))
    qapp.processEvents()
    assert ctl.move.go_state() == (True, "") and ctl.jog._buttons["forward"].isEnabled()
    assert ctl.odom._btn_reset.isEnabled()
    fake.sequencer = object()
    fake.set_state("running", mode="real")
    qapp.processEvents()
    assert ctl.move.go_state() == (False, RUN_LOCK) and not ctl.jog._buttons["forward"].isEnabled()
    assert not ctl.odom._btn_reset.isEnabled() and ctl.odom._reset_hint.text() == RESET_LOCK
    assert ctl._run_banner.isVisibleTo(ctl) and ctl._run_banner.text() == RUN_LOCK_BANNER
    fw.status.emit(status(eAmrState=C.ST_MANUAL))          # stays locked on status updates
    qapp.processEvents()
    assert not ctl.move._go.isEnabled() and RUN_LOCK in ctl.move._reason.text()
    fake.set_state("paused")
    qapp.processEvents()
    assert ctl.move.go_state() == (True, "") and ctl.jog._buttons["forward"].isEnabled()   # repositioning allowed
    assert not ctl.odom._btn_reset.isEnabled() and not ctl._run_banner.isVisibleTo(ctl)
    fake.sequencer = None
    fake.set_state("loaded")
    qapp.processEvents()
    assert ctl.odom._btn_reset.isEnabled()
    fake.set_state("running", mode="sim")                  # a SIM run never locks the ARES tab
    fake.sequencer = object()
    qapp.processEvents()
    assert ctl.move.go_state() == (True, "")


def test_close_refused_while_a_run_is_active(fake_win, qapp):
    w, fw, fake = fake_win
    fake.set_state("running", mode="sim")
    assert w.close() is False
    assert fw.of("stop") == [] and w.statusBar().currentMessage() == CLOSE_REFUSED
    fake.set_state("done")
    assert w.close() is True and fw.of("stop") == [None] and fake.of("shutdown") == [10.0]


def test_title_names_the_variant_and_ads_off(qapp, tmp_path):
    fake = FakeRunController(config.load()["hmi"])
    w, fw, ctx = make_window(qapp, tmp_path, controller=fake)
    try:
        s = short_sim_session()
        fake.set_session(JobSession(s.cfg, "c_acb", s.job, None, "test", "nominal_C_c_acb"))
        qapp.processEvents()
        assert w.windowTitle() == "Mauer HMI - nominal_C_c_acb - config c_acb - - - ADS off"
        assert w.mauer.variant_combo.currentText() in ("c_acb", "main")
    finally:
        w.close()


def test_main_build_without_ares_uses_the_null_worker(qapp):
    from hmi.main import build
    w, ctx = build([])
    try:
        qapp.processEvents()
        assert isinstance(w._worker, NullAdsWorker) and not ctx.ares_enabled
        assert ADS_OFF in w._lbl_conn.text() and w.windowTitle().endswith("ADS off")
        w.trigger_halt()
        assert w._worker.calls == [("halt", None)] and "HALT NOT sent" in w.statusBar().currentMessage()
    finally:
        w.close()


def test_plan_geometry_of_the_main_c_and_the_straight_job():
    from hmi.core.session import build_from_config
    s = build_from_config(None)
    g = plan_geometry(s.job, s.cfg)
    assert len(g.stones) == s.job.n_stones == len(g.order) and len(g.stops) == len(s.job.stops)
    assert g.routes and {k for k, _ in g.routes} == {"stop", "to_station", "from_station"}
    assert any(kind == "leg" for _, kind, _ in g.obstacles) and g.table is not None and not g.notes
    x0, x1, y0, y1 = g.bounds
    for poly in g.stones.values():
        assert all(x0 < x < x1 and y0 < y < y1 for x, y in poly)
    half = [k for k, t in zip(g.order, s.job.stones()) if t.kind == "half"]
    if half:                                               # the half stone of leg A (A 5 1/2): 100 mm long
        xs = [p[0] for p in g.stones[half[0]]]
        ys = [p[1] for p in g.stones[half[0]]]
        assert min(max(xs) - min(xs), max(ys) - min(ys)) == pytest.approx(100.0, abs=0.5)
    st = short_sim_session()
    g2 = plan_geometry(st.job, st.cfg)
    assert len(g2.stones) == 3 and g2.obstacles == () and g2.routes == ()


def test_plan_view_paints_every_layer(qapp):
    from dataclasses import replace

    from hmi.core.snapshot import RunSnapshot
    from hmi.core.sources import AresLive
    from hmi.views.plan_view import PlanView
    from mauer.reference import Pose2D
    s = short_sim_session()
    v = PlanView()
    v.resize(500, 400)
    v.grab()                                               # no job
    v.set_session(s)
    snap = RunSnapshot.initial(s)
    first = s.job.stones()[0].key
    v.set_snapshot(replace(snap, placed=frozenset({first}), pose_status="odometry"))
    est = snap.pose_est
    v.set_live(AresLive(Pose2D(est.x_mm + 50, est.y_mm, est.theta_rad), "estimate+odometry", "ok",
                        Pose2D(est.x_mm, est.y_mm + 20, est.theta_rad), None))
    img = v.grab().toImage()
    assert img.width() == 500 and v.geometry_ is not None


def test_mauer_tab_options_follow_the_state(fake_win, qapp):
    w, fw, fake = fake_win
    tab = w.mauer
    fake.set_session(short_sim_session())
    qapp.processEvents()
    assert tab.real_rb.isEnabled()                         # --ares
    tab.real_rb.setChecked(True)
    assert tab.prepare_btn.isEnabled()
    opts = tab.options()
    assert opts.mode == "real" and opts.start_stop == 0 and opts.stop_after is None
    tab.stop_to.setValue(0)
    assert tab.options().stop_after == 0
    tab.prepare_btn.click()
    assert fake.of("prepare")[0][0].mode == "real"
    fake.set_state("running", mode="real")
    qapp.processEvents()
    assert not tab.scenario.isEnabled() and not tab.browse_btn.isEnabled() and tab.pause_btn.isEnabled()
    assert tab.step.isEnabled()                             # live
    fake.set_state("paused")
    qapp.processEvents()
    assert tab.recovery.isVisibleTo(tab) and tab.set_pose_btn.isEnabled() and not tab.jaws_btn.isEnabled()
    tab.set_x.setValue(1234.0)
    tab.set_pose_btn.click()
    pose = fake.of("set_pose")[0][0]
    assert pose.x_mm == pytest.approx(1234.0)
    fake.set_state("loaded")
    fw.status.emit(status(eAmrState=C.ST_MANUAL, bMoveActive=True))
    qapp.processEvents()
    assert not tab.browse_btn.isEnabled() and "ARES move" in tab.browse_btn.toolTip()
    fw.status.emit(status(eAmrState=C.ST_MANUAL))
    qapp.processEvents()
    assert tab.browse_btn.isEnabled()
