"""Offscreen GUI smoke tests: MainWindow with a fake worker, synthetic v1 / v2 status dicts.

Copied 2026-10-07 from MA 10_robot/hmi/amr_hmi/tests/test_gui_smoke.py (commit 5935c5b) and adapted to the Mauer
MainWindow: it gets an HmiContext (config/station.toml) and the worker, the amr Control tab is the "ARES control"
tab (selected explicitly where the amr tests relied on it being the first tab), the tab list is the Mauer one."""

from __future__ import annotations

import copy
import time
from typing import Any, Dict

import pytest
from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QKeyEvent

from hmi.amr import frame as F
from hmi.amr import plc_vars as pv
from hmi.amr.ui import constants as C
from hmi.amr.ui.jog_panel import PRECISE_TIP
from hmi.amr.ui.odom_map import OdomMap, axis_labels, rotation_label
from hmi.amr.ui.widgets import LatchBtn
from hmi.main_window import TAB_NAMES, MainWindow
from hmi_fakes import FakeWorker, make_window as _make_window
from mauer import config

pytestmark = pytest.mark.usefixtures("no_lab_network")


def status(version: int = pv.IF_V2, **kw: Any) -> Dict[str, Any]:
    d = pv.default_status(version)
    d.update({"bPlcRunning": True, "bHmiWatchdogOK": True, "nPlcHeartbeat": 1})
    if version >= pv.IF_V2:
        d.update({"nIfVersion": 2, "sPlcBuild": "ARES CX9240 v2 2026-09-25"})
    d.update(kw)
    return d


def make_window(qapp, tmp_path, frame: Dict[str, Any] | None = None):
    cfg = copy.deepcopy(config.load())
    if frame is not None:
        cfg["hmi"]["frame"] = frame
    w, fw, _ctx = _make_window(qapp, tmp_path, station_cfg=cfg)
    return w, fw


@pytest.fixture
def win(qapp, tmp_path):
    w, fw = make_window(qapp, tmp_path)
    yield w, fw
    w.close()


@pytest.fixture
def win_y_left_unverified(qapp, tmp_path):
    """Default mapping (plus_y_is_left: true) but verified: false -> banner and 'unverified' texts shown."""
    w, fw = make_window(qapp, tmp_path / "a", {"plus_y_is_left": True, "plus_omega_is_ccw": True, "verified": False})
    yield w, fw
    w.close()


@pytest.fixture
def win_y_right(qapp, tmp_path):
    """Window with the other Y mapping (plus_y_is_left: false, the PLC v2 behaviour before D21)."""
    w, fw = make_window(qapp, tmp_path / "b", {"plus_y_is_left": False, "plus_omega_is_ccw": True,
                                               "verified": False})
    yield w, fw
    w.close()


def feed(fw: FakeWorker, qapp, **kw: Any) -> None:
    fw.status.emit(status(**kw))
    qapp.processEvents()


def test_window_defaults(win, qapp):
    w, _fw = win
    assert (w.width(), w.height()) == (1280, 800)
    assert [w.tabs.tabText(i) for i in range(w.tabs.count())] == list(TAB_NAMES) == [
        "Mauer", "Camera", "UR", "ARES control", "Wall pose", "Dashboard", "Diagnostics", "Battery", "Run log", "Twin"]
    assert w.tabs.widget(TAB_NAMES.index("ARES control")) is w.control
    assert w.halt_button.isVisible() or not w.isVisible()   # offscreen: not shown, but part of the window


def test_all_tabs_accept_v1_and_v2_status(win, qapp):
    w, fw = win
    for i in range(w.tabs.count()):
        w.tabs.setCurrentIndex(i)
        feed(fw, qapp, eAmrState=C.ST_MANUAL, bModeManual=True, fPosX_m=0.5, fPosY_m=0.1, fPosTheta_deg=30.0)
        fw.status.emit(status(pv.IF_LEGACY, eAmrState=C.ST_PRECHARGE))
        qapp.processEvents()
    fw.interface.emit(1, "legacy PLC (no nIfVersion)")
    for s in range(-1, 18):
        feed(fw, qapp, eAmrState=s)
    w.grab()   # render once (paint paths incl. odometry map)
    w.control.sub_tabs.setCurrentIndex(1)
    w.control.odom.map.grab()


def test_state_16_is_precharge(win, qapp):
    w, fw = win
    feed(fw, qapp, eAmrState=16)
    assert w.control._state_lbl.text() == "PRECHARGE"


def test_halt_button_calls_worker_halt(win, qapp):
    w, fw = win
    w.halt_button.click()
    assert fw.of("halt") == [None]


def _key(w: MainWindow, key: int, press: bool = True, auto: bool = False) -> bool:
    ev = QKeyEvent(QEvent.KeyPress if press else QEvent.KeyRelease, key, Qt.NoModifier, "", auto)
    return w.eventFilter(w, ev)


def test_space_and_esc_trigger_halt_only_when_window_active(win, qapp, monkeypatch):
    w, fw = win
    monkeypatch.setattr(w, "isActiveWindow", lambda: False)
    assert _key(w, Qt.Key_Space) is False and not fw.of("halt")
    monkeypatch.setattr(w, "isActiveWindow", lambda: True)
    assert _key(w, Qt.Key_Space) is True
    assert _key(w, Qt.Key_Escape) is True
    assert _key(w, Qt.Key_Space, auto=True) is True        # auto-repeat consumed, no extra HALT
    assert len(fw.of("halt")) == 2
    w.tabs.setCurrentIndex(2)                               # HALT works on every tab
    assert _key(w, Qt.Key_Escape) is True and len(fw.of("halt")) == 3


def _jog_sends(fw: FakeWorker):
    return [v for v in fw.of("send") if set(v) == set(pv.JOG_FIELDS)]


def test_keyboard_jog_gating_and_mapping(win, qapp, monkeypatch):
    w, fw = win
    monkeypatch.setattr(w, "isActiveWindow", lambda: True)
    w.tabs.setCurrentWidget(w.control)                      # the Mauer tab is the first tab
    # not MANUAL -> key consumed on the jog tab but nothing sent
    feed(fw, qapp, eAmrState=C.ST_READY)
    _key(w, Qt.Key_A)
    assert not _jog_sends(fw)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    assert _key(w, Qt.Key_A) is True
    sends = _jog_sends(fw)
    # default frame.plus_y_is_left = true (PLC v2.1, D21) -> physical Left = PLC +vy -> bCmdJogLeft
    assert sends[-1]["bCmdJogLeft"] is True and sends[-1]["bCmdJogRight"] is False
    assert _key(w, Qt.Key_A, press=False) is True
    assert not any(_jog_sends(fw)[-1].values())
    # auto-repeat does not resend
    n = len(_jog_sends(fw))
    _key(w, Qt.Key_W)
    _key(w, Qt.Key_W, auto=True)
    assert len(_jog_sends(fw)) == n + 1
    # tab change releases everything
    w.tabs.setCurrentWidget(w.dashboard)
    assert not any(_jog_sends(fw)[-1].values())
    # other tab: jog keys are not handled
    n = len(_jog_sends(fw))
    assert _key(w, Qt.Key_W) is False and len(_jog_sends(fw)) == n
    # Relative-move sub-tab: jog keys not handled either
    w.tabs.setCurrentWidget(w.control)
    w.control.sub_tabs.setCurrentIndex(1)
    assert _key(w, Qt.Key_W) is False


def test_jog_released_on_focus_loss_disconnect_and_ext(win, qapp, monkeypatch):
    w, fw = win
    monkeypatch.setattr(w, "isActiveWindow", lambda: True)
    w.tabs.setCurrentWidget(w.control)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    _key(w, Qt.Key_W)
    assert w.control.jog.active_fields == {"bCmdJogFwd"}
    w._on_app_state(Qt.ApplicationInactive)
    assert not w.control.jog.active_fields and not any(_jog_sends(fw)[-1].values())
    _key(w, Qt.Key_W)
    fw.connection.emit(False, "read failed")
    qapp.processEvents()
    assert not w.control.jog.active_fields
    fw.connection.emit(True, "connected")
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    _key(w, Qt.Key_W)
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bExtActive=True)
    assert not w.control.jog.active_fields
    assert w.control._ext_banner.isVisibleTo(w.control)


def test_jog_buttons_enabled_only_in_manual_without_ext(win, qapp):
    w, fw = win
    b = w.control.jog._buttons["forward"]
    feed(fw, qapp, eAmrState=C.ST_READY)
    assert not b.isEnabled()
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    assert b.isEnabled()
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bExtActive=True)
    assert not b.isEnabled()


def test_manual_entry_pushes_jog_parameters_once(win, qapp):
    w, fw = win
    feed(fw, qapp, eAmrState=C.ST_READY)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    params = [v for v in fw.of("send") if "fJogSpeed_mms" in v]
    assert len(params) == 1
    assert set(params[0]) == {"fJogSpeed_mms", "fJogRotSpeed_degs", "fSpeedLimit_mms", "fAccel_mms"}
    feed(fw, qapp, eAmrState=C.ST_READY)
    assert fw.of("send")[-1] == {"fAccel_mms": 200.0}


def test_go_needs_two_clicks_and_sends_one_pulse(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=0)
    assert mv._go.isEnabled()
    mv._go.click()
    assert not fw.of("pulse") and mv.armed and "CONFIRM" in mv._go.text()
    assert "+X 1000 mm" in mv._go.text() and "150 mm/s" in mv._go.text()
    mv._go.click()
    pulses = fw.of("pulse")
    assert len(pulses) == 1
    vals, fields = pulses[0]
    assert fields == ("bCmdMoveStart",)
    assert vals["fMoveX_mm"] == pytest.approx(1000.0) and vals["fMoveY_mm"] == 0.0
    assert vals["fMoveTheta_deg"] == 0.0 and vals["nMoveCmdId"] == 1 and vals["bCmdMoveStart"] is True
    assert (vals["fMoveSpeed_mms"], vals["fMoveRotSpeed_degs"], vals["fMoveAccel_mms2"]) == (150.0, 10.0, 200.0)
    # waiting for the ack -> GO locked
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=0)
    assert not mv._go.isEnabled()
    # accepted -> target marker on the map
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, bMoveActive=True, eMoveState=C.MOVE_RUN)
    assert w.control.odom.map.target == pytest.approx((1.0, 0.0, 0.0))
    assert not mv._go.isEnabled()        # move running
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveState=C.MOVE_DONE, eMoveResult=C.RES_OK,
         fMoveFinalErr=0.4, fMoveProgress_pct=100.0)
    assert mv._go.isEnabled()
    assert "within tolerance" in mv._result.text()


def test_confirmation_expires_and_parameter_change_disarms(win, qapp, monkeypatch):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    mv._go.click()
    assert mv.armed
    mv._spd.setValue(100.0)
    assert not mv.armed
    mv._go.click()
    t0 = time.monotonic()
    monkeypatch.setattr("hmi.amr.ui.move_panel.time.monotonic", lambda: t0 + 3.5)
    assert not mv.armed
    mv._go.click()              # counts as a first click again
    assert not fw.of("pulse")


def test_rejection_is_reported(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=7)
    mv._go.click()
    mv._go.click()
    vals, _ = fw.of("pulse")[0]
    assert vals["nMoveCmdId"] == 1
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveCmdResult=C.RES_REJ_STATE,
         eMoveResult=C.RES_REJ_STATE, sMoveText="Rejected: AMR not in MANUAL mode")
    assert "REJECTED" in mv._reason.text() and "not in MANUAL" in mv._reason.text()
    assert w.control.odom.map.target is None


def test_verdict_comes_from_eMoveCmdResult(win, qapp):
    """Spec section 9: eMoveResult may still show an old rejection / the running move - the verdict is separate."""
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=0, eMoveResult=C.RES_REJ_PARAM)
    mv._go.click()
    mv._go.click()
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveCmdResult=0, eMoveResult=C.RES_REJ_PARAM)
    assert "accepted" in mv._reason.text() and "REJECTED" not in mv._reason.text()
    assert w.control.odom.map.target is not None
    # next command rejected while eMoveResult shows "no result" -> still reported as rejected
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveState=C.MOVE_DONE, eMoveResult=C.RES_OK)
    mv._go.click()
    mv._go.click()
    assert fw.of("pulse")[-1][0]["nMoveCmdId"] == 2
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=2, eMoveCmdResult=C.RES_REJ_BUSY, eMoveResult=C.RES_NONE,
         sMoveText="Rejected: robot still moving")
    assert "REJECTED" in mv._reason.text() and "still moving" in mv._reason.text()


def test_invalid_verdict_is_not_accepted(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=0)
    mv._go.click()
    mv._go.click()
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveCmdResult=42)
    assert "unexpected PLC verdict" in mv._reason.text()
    assert w.control.odom.map.target is None


def test_limited_by_plc_is_shown_during_the_move(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=0)
    mv._go.click()
    mv._go.click()
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveCmdResult=0, bMoveLimited=True, bMoveActive=True,
         eMoveState=C.MOVE_RUN)
    assert "LIMITED" in mv._reason.text()
    assert "LIMITED by PLC" in mv._limited.text() and mv._limited.isVisibleTo(mv)
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, bMoveLimited=False, bMoveActive=True, eMoveState=C.MOVE_RUN)
    assert mv._limited.text() == ""
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, bMoveLimited=True, bMoveActive=False,
         eMoveState=C.MOVE_DONE, eMoveResult=C.RES_OK)
    assert mv._limited.text() == "" and not mv._limited.isVisibleTo(mv)    # only during the move


def test_abort_odom_result_text(win, qapp):
    w, fw = win
    feed(fw, qapp, eAmrState=C.ST_MANUAL, eMoveState=C.MOVE_ABORTED, eMoveResult=C.RES_ABORT_ODOM,
         sMoveText="Aborted: progress implausible")
    txt = w.control.move._result.text()
    assert "measured progress implausible (odometry)" in txt and "PLC: Aborted" in txt


def test_min_move_disables_go_with_clear_reason(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    mv._dx.setValue(1.0)
    mv._dy.setValue(1.0)                                # free vector 1.41 mm < 2 mm
    assert not mv._go.isEnabled()
    assert "fMinMove_mm" in mv._reason.text()
    mv._dy.setValue(2.0)                                # 2.24 mm
    assert mv._go.isEnabled()
    assert mv._dist.minimum() == 2.0 and mv._ang.minimum() == pytest.approx(0.2)


def test_rotation_and_direction_pad(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    mv._dir_buttons["cw"].click()
    req = mv.current_request()
    assert req.is_rotation and req.dtheta_deg == pytest.approx(-90.0)   # CW = -theta with default config
    mv._dir_buttons["fwd_left"].click()
    req = mv.current_request()
    assert not req.is_rotation
    # default config (PLC v2.1, D21): physical left = PLC +Y
    assert (req.dx_mm, req.dy_mm) == pytest.approx((707.1, 707.1), abs=0.1)
    mv._dx.setValue(300.0)                   # free vector
    assert mv._dir is None and mv._dist.value() == pytest.approx((300 ** 2 + 707.1 ** 2) ** 0.5, abs=0.2)
    mv._go.click()
    mv._go.click()
    vals, _ = fw.of("pulse")[-1]
    assert vals["fMoveX_mm"] == pytest.approx(300.0) and vals["fMoveTheta_deg"] == 0.0


def test_go_disabled_on_legacy_plc_and_outside_manual(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_READY)
    assert not mv._go.isEnabled() and "MANUAL" in mv._reason.text()
    fw.interface.emit(1, "legacy PLC (no nIfVersion)")
    fw.status.emit(status(pv.IF_LEGACY, eAmrState=C.ST_MANUAL))
    qapp.processEvents()
    assert not mv._go.isEnabled() and "interface v1" in mv._reason.text()
    assert w.control._legacy_banner.isVisibleTo(w.control)


def test_load_test_move(win, qapp):
    w, fw = win
    w.control._btn_test.click()
    mv = w.control.move
    assert w.control.sub_tabs.currentIndex() == 1
    req = mv.current_request()
    # default config plus_y_is_left: true (PLC v2.1, D21) -> physical Left = PLC +Y
    assert (req.dx_mm, req.dy_mm, req.speed_mms) == (0.0, 100.0, 50.0)
    assert "+Y 100 mm (left)" in mv._cmd_lbl.text()
    assert not fw.of("pulse")                                          # GO stays manual


def test_load_test_move_other_mapping(win_y_right, qapp):
    w, _fw = win_y_right
    w.control._btn_test.click()
    req = w.control.move.current_request()
    assert (req.dx_mm, req.dy_mm) == (0.0, -100.0)


def test_direction_texts_are_neutral_and_follow_config(win_y_left_unverified, win_y_right, qapp):
    """No text claims 'PLC +Y = left' as a fact; captions name the physical side under the current config."""
    for (w, _fw), side in ((win_y_left_unverified, "left"), (win_y_right, "right")):
        c = w.control
        banner = c._dir_banner.findChildren(type(c._state_lbl))[0].text()
        assert f"PLC +Y (= {side} per config, unverified)" in banner
        assert "PLC +omega (= CCW per config, unverified)" in banner
        assert f"dy (+Y {side}*)" in c.move._dy_caption.text()
        assert f"PLC +Y (= {side} per config, unverified)" in c.move._frame_note.text()
        assert f"PLC +Y (= {side} per config, unverified)" in c.move._dy.toolTip()
        assert f"PLC +Y (= {side} per config, unverified)" in c.jog._map_lbl.text()
        for txt in (banner, c.move._frame_note.text(), c.jog._map_lbl.text(), c.move._dy.toolTip()):
            assert "+Y = left" not in txt and "+Y left" not in txt
    assert "left = PLC +Y (config, unverified)" in axis_labels(F.FrameConfig())[1]
    assert "left = PLC +Y (config, verified)" in axis_labels(F.FrameConfig(True, True, True))[1]
    assert "CCW = PLC +theta" in rotation_label(F.FrameConfig())
    assert "CCW = PLC -theta" in rotation_label(F.FrameConfig(True, False))


def test_standby_and_startup_buttons(win, qapp):
    w, fw = win
    c = w.control
    feed(fw, qapp, eAmrState=C.ST_STANDBY, bSafetyRunActive=True)
    assert c._btn_start.isEnabled() and not c._btn_standby.isEnabled()
    c._btn_start.click()
    assert fw.of("pulse")[-1] == ({}, ("bCmdStart",))
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bSafetyRunActive=True, bModeManual=True)
    c._btn_standby.click()
    vals, fields = fw.of("pulse")[-1]
    assert vals == {"bCmdStop": True, "bCmdManualMode": False, "bCmdAutoMode": False}
    assert fields == ("bCmdStop",)
    modes = [b.set_mode for b in c._badges]
    assert modes   # badges exist
    assert c._badges[3]._hmi_css == c._badges[0]._CSS["done"]


def _green_rows(img) -> float:
    """Mean pixel row of the bright-green heading line (#55DD55) of the robot outline."""
    rows = []
    for yy in range(img.height()):
        for xx in range(0, img.width(), 2):
            c = img.pixelColor(xx, yy)
            if c.green() > 180 and c.red() < 140 and c.blue() < 140:
                rows.append(yy)
    assert rows, "robot heading line not found"
    return sum(rows) / len(rows)


def _green_cols(img) -> float:
    cols = []
    for yy in range(0, img.height(), 2):
        for xx in range(img.width()):
            c = img.pixelColor(xx, yy)
            if c.green() > 180 and c.red() < 140 and c.blue() < 140:
                cols.append(xx)
    return sum(cols) / len(cols)


@pytest.mark.parametrize("y_left, drawn_below_origin", [(False, True), (True, False)])
def test_odometry_map_draws_plc_plus_y_on_the_configured_side(qapp, y_left, drawn_below_origin):
    """PLC +Y 1 m: drawn below the origin (= physical right) with the default config, above with +Y = left."""
    m = OdomMap(frame_cfg=F.FrameConfig(y_left, True))
    m.resize(400, 400)
    m.set_pose(0.0, 0.0, 0.0)
    m.set_pose(0.0, 0.3, 0.0)
    m.set_pose(0.0, 0.6, 0.0)
    m.set_pose(0.0, 1.0, 0.0)
    assert m.screen_pose(m.pose)[1] == pytest.approx(-1.0 if not y_left else 1.0)
    img = m.grab().toImage()
    # view is centred between origin and robot -> robot below the centre = drawn on the physical right
    assert (_green_rows(img) > img.height() / 2) is drawn_below_origin


@pytest.mark.parametrize("ccw, heading_up", [(True, True), (False, False)])
def test_odometry_map_heading_follows_rotation_flag(qapp, ccw, heading_up):
    """PLC theta +90 deg: heading drawn up (physical CCW) if +omega = CCW, else down."""
    m = OdomMap(frame_cfg=F.FrameConfig(False, ccw))
    m.resize(400, 400)
    m.set_pose(0.0, 0.0, 90.0)
    img = m.grab().toImage()
    # heading line from the centre (200, 200) along the robot's front
    assert (_green_rows(img) < img.height() / 2) is heading_up
    assert abs(_green_cols(img) - img.width() / 2) < 6


def test_target_marker_is_stored_in_plc_frame_and_drawn_mapped(win, qapp):
    w, fw = win
    mv = w.control.move
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=0)
    mv._dir_buttons["left"].click()                 # physical left, default config (D21) -> PLC +Y
    mv._go.click()
    mv._go.click()
    feed(fw, qapp, eAmrState=C.ST_MANUAL, nMoveCmdAck=1, eMoveCmdResult=0, bMoveActive=True)
    m = w.control.odom.map
    assert m.target == pytest.approx((0.0, 1.0, 0.0))               # PLC odom frame (as the PLC measures)
    assert m.screen_pose(m.target) == pytest.approx((0.0, 1.0, 0.0))  # drawn on the physical left (up)


def test_reset_pose_locked_while_move_active(win, qapp):
    w, fw = win
    od = w.control.odom
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_RUN)
    assert not od._btn_reset.isEnabled()
    assert "locked" in od._reset_hint.text()
    od._btn_reset.click()                           # disabled button: no click
    od._reset_pose()                                # defensive path (queued click)
    assert ("pulse", ({}, ("bCmdOdomReset",))) not in fw.calls
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=False, eMoveState=C.MOVE_DONE)
    assert od._btn_reset.isEnabled() and od._reset_hint.text() == ""
    od._btn_reset.click()
    assert fw.of("pulse")[-1] == ({}, ("bCmdOdomReset",))
    # legacy PLC: disabled, no hint about moves
    fw.interface.emit(1, "legacy PLC (no nIfVersion)")
    fw.status.emit(status(pv.IF_LEGACY, eAmrState=C.ST_MANUAL))
    qapp.processEvents()
    assert not od._btn_reset.isEnabled() and od._reset_hint.text() == ""


def _accel_sends(fw: FakeWorker):
    return [v for v in fw.of("send") if "fAccel_mms" in v]


def test_jog_parameters_not_sent_during_a_move(win, qapp):
    """Item 7: no fAccel_mms write while bMoveActive; the latest value is sent after the move."""
    w, fw = win
    jog = w.control.jog
    feed(fw, qapp, eAmrState=C.ST_READY)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    n0 = len(_accel_sends(fw))
    assert n0 == 1                                   # pushed on MANUAL entry
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_RUN)
    assert not jog._acc.isEnabled() and not jog._spd.isEnabled()
    assert "Locked" in jog._param_lock.text() and jog._param_lock.isVisibleTo(jog)
    jog._acc.setValue(300.0)                          # e.g. programmatic change: deferred, not sent
    jog._acc.setValue(400.0)
    assert len(_accel_sends(fw)) == n0
    assert jog.pending_params["fAccel_mms"] == 400.0
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_SETTLE)
    assert len(_accel_sends(fw)) == n0
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=False, eMoveState=C.MOVE_DONE)
    sends = _accel_sends(fw)
    assert len(sends) == n0 + 1 and sends[-1]["fAccel_mms"] == 400.0
    assert jog._acc.isEnabled() and not jog._param_lock.isVisibleTo(jog) and not jog.pending_params


def test_manual_exit_during_move_defers_accel_restore(win, qapp):
    w, fw = win
    feed(fw, qapp, eAmrState=C.ST_READY)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    n0 = len(_accel_sends(fw))
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_RUN)
    feed(fw, qapp, eAmrState=C.ST_READY, bMoveActive=True, eMoveState=C.MOVE_ABORTING)   # MANUAL left -> abort
    assert len(_accel_sends(fw)) == n0
    feed(fw, qapp, eAmrState=C.ST_READY, bMoveActive=False, eMoveState=C.MOVE_ABORTED)
    assert _accel_sends(fw)[-1] == {"fAccel_mms": 200.0}


def _precise_sends(fw: FakeWorker):
    return [v["bCmdPreciseMode"] for v in fw.of("send") if "bCmdPreciseMode" in v]


def test_precise_mode_locked_and_marked_forced_during_a_move(win, qapp, monkeypatch):
    """Finding N3: PLC v2 bPreciseModeActive = own setting OR move. The latch is locked during the move, shows the
    forced state, and toggles the OWN setting (last written), never the display (old code: always wrote FALSE)."""
    monkeypatch.setattr(LatchBtn, "HOLD_S", 0.0)
    w, fw = win
    jog = w.control.jog
    btn = jog._btn_precise
    feed(fw, qapp, eAmrState=C.ST_READY)
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    assert btn.isEnabled() and not btn.is_active() and jog.precise_setting is False
    assert btn.text().endswith("[OFF]") and btn.toolTip() == PRECISE_TIP
    # move running: the PLC forces precise mode, the own setting stays OFF
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_RUN, bPreciseModeActive=True)
    assert not btn.isEnabled()
    assert btn.is_active() and btn.text().endswith("[ON, forced by move]")
    assert "Own setting: OFF" in btn.toolTip()
    assert jog.precise_setting is False                 # forced display is not adopted as own setting
    btn.click()                                         # disabled -> nothing is written
    assert _precise_sends(fw) == []
    jog._toggle_precise()                               # a click that still gets through toggles the OWN setting
    assert _precise_sends(fw) == [True]
    assert btn.is_active() and btn.text().endswith("[ON, forced by move]")      # display stays forced
    assert jog.precise_setting is True and "Own setting: ON" in btn.toolTip()
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_SETTLE, bPreciseModeActive=True)
    assert jog.precise_setting is True
    # move ended: display = own setting (ON), latch enabled, click toggles it OFF
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=False, eMoveState=C.MOVE_DONE, bPreciseModeActive=True)
    assert btn.isEnabled() and btn.is_active() and btn.text().endswith("[ON ]") and btn.toolTip() == PRECISE_TIP
    btn.click()
    assert _precise_sends(fw) == [True, False] and jog.precise_setting is False


def test_precise_mode_own_setting_off_survives_the_move(win, qapp, monkeypatch):
    monkeypatch.setattr(LatchBtn, "HOLD_S", 0.0)
    w, fw = win
    jog = w.control.jog
    btn = jog._btn_precise
    feed(fw, qapp, eAmrState=C.ST_MANUAL)
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=True, eMoveState=C.MOVE_RUN, bPreciseModeActive=True)
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bMoveActive=False, eMoveState=C.MOVE_DONE, bPreciseModeActive=False)
    assert btn.isEnabled() and not btn.is_active() and btn.text().endswith("[OFF]")
    btn.click()
    assert _precise_sends(fw) == [True]


def test_precise_mode_adopts_plc_value_while_no_move_runs(win, qapp, monkeypatch):
    """Without a move bPreciseModeActive == bCmdPreciseMode: a value the PLC kept (HMI restart) or a write that was
    not sent is adopted as the own setting, so the next click toggles the real PLC value."""
    monkeypatch.setattr(LatchBtn, "HOLD_S", 0.0)
    w, fw = win
    jog = w.control.jog
    btn = jog._btn_precise
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bPreciseModeActive=True)
    assert jog.precise_setting is True and btn.is_active()
    btn.click()
    assert _precise_sends(fw) == [False]
    feed(fw, qapp, eAmrState=C.ST_MANUAL, bPreciseModeActive=True)     # write did not arrive -> PLC still ON
    assert jog.precise_setting is True and btn.is_active()
    btn.click()
    assert _precise_sends(fw) == [False, False]


def test_precise_mode_on_legacy_plc(win, qapp, monkeypatch):
    monkeypatch.setattr(LatchBtn, "HOLD_S", 0.0)
    w, fw = win
    btn = w.control.jog._btn_precise
    fw.interface.emit(1, "legacy PLC (no nIfVersion)")
    fw.status.emit(status(pv.IF_LEGACY, eAmrState=C.ST_MANUAL))
    qapp.processEvents()
    assert btn.isEnabled() and not btn.is_active()
    btn.click()
    fw.status.emit(status(pv.IF_LEGACY, eAmrState=C.ST_MANUAL, bPreciseModeActive=True))
    qapp.processEvents()
    assert btn.is_active() and btn.text().endswith("[ON ]")
    btn.click()
    assert _precise_sends(fw) == [True, False]


def test_latch_button_note_and_hold(qapp):
    b = LatchBtn("Precise Mode")
    assert b.text().endswith("[OFF]") and not b.holding()
    b.sync(True)
    assert b.text().endswith("[ON ]")                   # unchanged format without a note
    b.set_note("forced by move")
    assert b.text().endswith("[ON, forced by move]")
    b.set_note("")
    assert b.text().endswith("[ON ]")
    b.set_local(False)
    assert b.holding() and not b.is_active()
    b.sync(True)                                        # PLC value ignored during the hold-off
    assert not b.is_active()


def test_interface_mismatch_is_shown_in_full(win, qapp):
    w, fw = win
    c = w.control
    msg = (f"{pv.MISMATCH_TEXT}: interface v2 but missing stFromPlc.eMoveCmdResult, stFromPlc.bMoveLimited - "
           "load the matching PLC build v2")
    fw.connection.emit(False, msg)
    qapp.processEvents()
    assert c._mismatch_banner.isVisibleTo(c)
    assert "stFromPlc.eMoveCmdResult, stFromPlc.bMoveLimited" in c._mismatch_banner.text()
    assert len(c._conn_lbl.text()) < len(msg) and c._conn_lbl.toolTip() == msg      # header elided, full tooltip
    assert w._lbl_conn.toolTip() == msg
    fw.connection.emit(False, "connect failed: timeout")                             # other reasons: no banner
    qapp.processEvents()
    assert not c._mismatch_banner.isVisibleTo(c)
    fw.connection.emit(False, msg)
    fw.connection.emit(True, "connected")
    qapp.processEvents()
    assert not c._mismatch_banner.isVisibleTo(c) and c._conn_lbl.text() == "Connected"


def test_odometry_map_trail_and_reset(win, qapp):
    w, fw = win
    m = w.control.odom.map
    for i in range(10):
        feed(fw, qapp, eAmrState=C.ST_MANUAL, fPosX_m=i * 0.01)
    assert len(m.trail) >= 9
    feed(fw, qapp, eAmrState=C.ST_MANUAL, fPosX_m=0.0901)     # < 5 mm step -> not appended
    n = len(m.trail)
    feed(fw, qapp, eAmrState=C.ST_MANUAL, fPosX_m=0.0902)
    assert len(m.trail) == n
    w.control.odom._btn_reset.click()
    assert fw.of("pulse")[-1] == ({}, ("bCmdOdomReset",))
    assert len(m.trail) == 1
    feed(fw, qapp, eAmrState=C.ST_MANUAL, fPosX_m=5.0)          # jump > 0.5 m -> new trail
    assert len(m.trail) == 1


def test_dashboard_and_diagnostics_show_v2_fields(win, qapp):
    w, fw = win
    w.tabs.setCurrentWidget(w.diagnostics)
    feed(fw, qapp, eAmrState=C.ST_MANUAL, eMoveState=2, sMoveText="running")
    row = w.diagnostics._row_map["sMoveText"]
    assert w.diagnostics._table.item(row, 2).text() == "running"
    fw.status.emit(status(pv.IF_LEGACY))
    qapp.processEvents()
    assert w.diagnostics._table.item(row, 2).text() == "n/a"
    w.tabs.setCurrentWidget(w.dashboard)
    feed(fw, qapp, eAmrState=16)
    assert w.dashboard._lbl_state.text() == "PRECHARGE"


def test_close_stops_worker(qapp, tmp_path):
    w, fw, ctx = _make_window(qapp, tmp_path, connect=False)
    assert isinstance(w, MainWindow) and isinstance(fw, FakeWorker)
    w.close()
    assert fw.of("stop") == [None] and not ctx.controller._worker.is_alive()
