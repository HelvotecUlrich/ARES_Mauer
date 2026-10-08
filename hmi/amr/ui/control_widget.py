"""
control_widget.py - Operator control tab (spec 6.4).

Layout:
  status header   state (coloured), connection, PLC build / interface, warnings
                  (external control active, legacy PLC, direction mapping unverified + "Load test move")
  left column     guided startup sequence (Safety Run -> AMR Reset -> Start -> Manual, current step
                  highlighted; Re-arm Safety; Standby (drives off)), compact drive status, SOC
  right           sub-tabs "Jog" (ui/jog_panel.py) and "Relative move" (ui/move_panel.py + ui/odom_map.py)
HALT lives in the MainWindow (visible on every tab). All commands go through the ADS worker.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QTabWidget, QVBoxLayout, QWidget,
)

from .. import frame as F
from ..logic import (
    STARTUP_STEPS, button_enables, jog_enabled, standby_values, startup_hint, startup_step,
)
from ..plc_vars import IF_V2, MISMATCH_TEXT
from . import constants as C
from .jog_panel import JogPanel
from .move_panel import MovePanel
from .odom_map import OdomPanel
from .widgets import GROUP_CSS, LatchBtn, PulseBtn, StepBadge, hsep, lbl, num_label, set_style, set_text

_BANNER_RED = "background:#5A0000;color:#FFFFFF;font-weight:bold;padding:6px;border-radius:4px;"
_BANNER_AMBER = "background:#4A3300;color:#FFDD88;font-weight:bold;padding:6px;border-radius:4px;"

TAB_JOG = 0
TAB_MOVE = 1
RUN_LOCK_BANNER = "Mauer REAL run active - jog / GO locked"
CONN_MSG_MAX = 60      # header: longer connection messages are elided (full text in the tooltip / status bar)


def elide(text: str, max_len: int) -> str:
    """Shorten text to max_len characters with '...' (full text goes into a tooltip)."""
    return text if len(text) <= max_len else text[:max(0, max_len - 3)] + "..."


class ControlWidget(QWidget):
    def __init__(self, cfg: Dict[str, Any], worker: Any, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._worker = worker
        self._frame = F.FrameConfig.from_config(cfg)
        self._state = -1
        self._connected = False
        self._if_version = 0
        self._status: Dict[str, Any] = {}
        self._run_lock = ""                     # Mauer HMI: reason a Mauer REAL run locks jog / GO ("" = none)
        self._build_ui()
        self.setStyleSheet(GROUP_CSS)
        self._apply_enables()

    # ── construction ──────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 6, 8, 6)
        outer.setSpacing(6)
        outer.addWidget(self._build_header())

        body = QHBoxLayout()
        body.setSpacing(8)
        left = QVBoxLayout()
        left.addWidget(self._build_startup_group())
        left.addWidget(self._build_drive_group())
        left.addStretch()
        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(350)
        body.addWidget(left_w)

        self.sub_tabs = QTabWidget()
        self.jog = JogPanel(self._cfg, self._frame, self._worker)
        move_page = QWidget()
        mp = QHBoxLayout(move_page)
        mp.setContentsMargins(0, 0, 0, 0)
        self.move = MovePanel(self._cfg, self._frame, self._worker)
        self.odom = OdomPanel(self._worker, frame_cfg=self._frame)
        self.move.target_changed.connect(self.odom.map.set_target)
        mp.addWidget(self.move, 0)
        mp.addWidget(self.odom, 1)
        self.sub_tabs.addTab(self.jog, "Jog (W/A/S/D/Q/E)")
        self.sub_tabs.addTab(move_page, "Relative move")
        self.sub_tabs.currentChanged.connect(lambda _i: self.release_jog())
        body.addWidget(self.sub_tabs, 1)
        outer.addLayout(body, 1)

    def _build_header(self) -> QWidget:
        box = QFrame()
        box.setStyleSheet("QFrame{background:#1B1B1B;border-radius:6px;}")
        lay = QVBoxLayout(box)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(4)
        row = QHBoxLayout()
        self._state_lbl = QLabel("-")
        f = QFont()
        f.setPointSize(18)
        f.setBold(True)
        self._state_lbl.setFont(f)
        self._conn_lbl = lbl("Disconnected", C.COL_BAD, bold=True)
        self._build_lbl = lbl("PLC: -", "#AAAAAA")
        self._soc_lbl = lbl("SOC -", "#AAAAAA", bold=True)
        row.addWidget(self._state_lbl)
        row.addSpacing(20)
        row.addWidget(self._conn_lbl)
        row.addSpacing(20)
        row.addWidget(self._build_lbl)
        row.addStretch()
        row.addWidget(self._soc_lbl)
        lay.addLayout(row)

        self._ext_banner = QLabel("External control active (C6030) - HMI motion blocked. "
                                  "Stop the ROS bridge / C6030 PLC to jog or move from the HMI.")
        self._ext_banner.setStyleSheet(_BANNER_RED)
        self._legacy_banner = QLabel("Legacy PLC (interface v1): relative move and odometry are not available. "
                                     "Load PLC build v2 (10_robot/twincat/PLC_CX9240_v2).")
        self._legacy_banner.setStyleSheet(_BANNER_AMBER)
        self._wd_banner = QLabel("PLC does not see the HMI heartbeat (bHmiWatchdogOK = FALSE): "
                                 "HMI commands are ignored.")
        self._wd_banner.setStyleSheet(_BANNER_RED)
        self._mismatch_banner = QLabel("")        # full text of a PLC/HMI interface mismatch (not connected)
        self._mismatch_banner.setStyleSheet(_BANNER_RED)
        self._run_banner = QLabel(RUN_LOCK_BANNER)  # Mauer HMI: a REAL run moves ARES
        self._run_banner.setStyleSheet(_BANNER_AMBER)
        for b in (self._ext_banner, self._legacy_banner, self._wd_banner, self._mismatch_banner, self._run_banner):
            b.setWordWrap(True)
            b.hide()
            lay.addWidget(b)

        self._dir_banner = QWidget()
        dl = QHBoxLayout(self._dir_banner)
        dl.setContentsMargins(0, 0, 0, 0)
        fr = self._frame
        txt = QLabel(
            "Direction mapping NOT verified on the robot (config frame.verified: false). Config: "
            f"{F.plus_y_text(fr)}, {F.plus_omega_text(fr)}. "
            "Runbook block R: jacked-up checks, then Y and rotation test moves, then frame.verified: true.")
        txt.setWordWrap(True)
        txt.setStyleSheet(_BANNER_AMBER)
        self._btn_test = QPushButton("Load test move")
        self._btn_test.setFocusPolicy(Qt.NoFocus)
        self._btn_test.setToolTip("Loads: physical Left 100 mm at 50 mm/s into 'Relative move'. GO stays manual.")
        self._btn_test.clicked.connect(self.load_test_move)
        dl.addWidget(txt, 1)
        dl.addWidget(self._btn_test)
        self._dir_banner.setVisible(not fr.verified)
        lay.addWidget(self._dir_banner)
        return box

    def _build_startup_group(self) -> QGroupBox:
        box = QGroupBox("Startup sequence")
        g = QGridLayout(box)
        g.setVerticalSpacing(5)
        self._badges = [StepBadge(i + 1) for i in range(len(STARTUP_STEPS))]

        self._btn_safety_run = LatchBtn("Safety Run", amber=True, min_w=150)
        self._btn_safety_run.setToolTip("Level command: TwinSAFE start-up sequence (bCmdSafetyRun).")
        self._btn_safety_run.clicked.connect(self._toggle_safety_run)
        self._btn_rearm = PulseBtn("Re-arm Safety", "#7A0000", min_w=120)
        self._btn_rearm.setToolTip("Pulse bCmdSafetyReset: re-arms TwinSAFE after an E-stop while Safety Run is ON.")
        self._btn_rearm.clicked.connect(lambda: self._worker.pulse({}, ("bCmdSafetyReset",), "re-arm safety"))

        self._btn_reset = PulseBtn("AMR Reset", "#6A4400", min_w=150)
        self._btn_reset.setToolTip("Pulse bCmdReset: leaves RESET REQUIRED / ERROR (state machine, not safety).")
        self._btn_reset.clicked.connect(lambda: self._worker.pulse({}, ("bCmdReset",), "AMR reset"))

        self._btn_start = PulseBtn("Start", "#0A5E30", min_w=150)
        self._btn_start.setToolTip("Pulse bCmdStart: STANDBY -> DRIVES ENABLE -> READY.")
        self._btn_start.clicked.connect(lambda: self._worker.pulse({}, ("bCmdStart",), "start"))

        self._btn_manual = LatchBtn("Manual", min_w=150)
        self._btn_manual.setToolTip("Level bCmdManualMode: READY <-> MANUAL (jog and relative move).")
        self._btn_manual.clicked.connect(self._toggle_manual)
        self._btn_auto = LatchBtn("Auto", min_w=100)
        self._btn_auto.setToolTip("Level bCmdAutoMode (AUTO mode; the HMI does not drive in AUTO).")
        self._btn_auto.clicked.connect(self._toggle_auto)

        rows = [(self._btn_safety_run, self._btn_rearm), (self._btn_reset, None),
                (self._btn_start, None), (self._btn_manual, self._btn_auto)]
        for i, (main, extra) in enumerate(rows):
            g.addWidget(self._badges[i], i, 0)
            g.addWidget(main, i, 1)
            if extra is not None:
                g.addWidget(extra, i, 2)
        self._hint = lbl("", "#DDDDDD")
        self._hint.setWordWrap(True)
        g.addWidget(self._hint, 4, 0, 1, 3)
        g.addWidget(hsep(), 5, 0, 1, 3)
        self._btn_standby = PulseBtn("Standby (drives off)", "#5A1A1A", min_w=150)
        self._btn_standby.setToolTip(
            "bCmdStop pulse + Manual/Auto OFF: state machine -> STANDBY, drives are DISABLED\n"
            "(no PLC ramp; coast-down depends on the Kinco setting). For a ramped stop use HALT.")
        self._btn_standby.clicked.connect(self._standby)
        g.addWidget(self._btn_standby, 6, 1, 1, 2)
        return box

    def _build_drive_group(self) -> QGroupBox:
        box = QGroupBox("Drives")
        g = QGridLayout(box)
        g.setVerticalSpacing(2)
        g.addWidget(lbl("", "#AAAAAA"), 0, 0)
        g.addWidget(lbl("Left", "#AAAAAA", True), 0, 1, alignment=Qt.AlignRight)
        g.addWidget(lbl("Right", "#AAAAAA", True), 0, 2, alignment=Qt.AlignRight)
        self._d: Dict[str, QLabel] = {}
        rows = [("spd", "Speed [mm/s]"), ("ang", "Angle act [deg]"), ("set", "Angle set [deg]"), ("st", "Drive")]
        for r, (key, cap) in enumerate(rows, start=1):
            g.addWidget(lbl(cap), r, 0)
            for c, side in ((1, "L"), (2, "R")):
                w = num_label()
                g.addWidget(w, r, c)
                self._d[key + side] = w
        g.addWidget(hsep(), 5, 0, 1, 3)
        g.addWidget(lbl("Actual", "#AAAAAA", True), 6, 1, alignment=Qt.AlignRight)
        g.addWidget(lbl("Setpoint", "#AAAAAA", True), 6, 2, alignment=Qt.AlignRight)
        for r, (key, cap) in enumerate((("vx", "Vx [mm/s]"), ("vy", "Vy [mm/s]"), ("om", "omega [deg/s]")),
                                       start=7):
            g.addWidget(lbl(cap), r, 0)
            for c, side in ((1, "a"), (2, "s")):
                w = num_label()
                g.addWidget(w, r, c)
                self._d[key + side] = w
        return box

    # ── commands ──────────────────────────────────────────────────────────────
    def _toggle_safety_run(self) -> None:
        new = not self._btn_safety_run.is_active()
        self._worker.send({"bCmdSafetyRun": new}, "safety run")
        self._btn_safety_run.set_local(new)

    def _toggle_manual(self) -> None:
        new = not self._btn_manual.is_active()
        vals = {"bCmdManualMode": new}
        if new:
            vals["bCmdAutoMode"] = False
            self._btn_auto.set_local(False)
        self._worker.send(vals, "manual mode")
        self._btn_manual.set_local(new)

    def _toggle_auto(self) -> None:
        new = not self._btn_auto.is_active()
        vals = {"bCmdAutoMode": new}
        if new:
            vals["bCmdManualMode"] = False
            self._btn_manual.set_local(False)
        self._worker.send(vals, "auto mode")
        self._btn_auto.set_local(new)

    def _standby(self) -> None:
        self.release_jog(send=False)
        vals = standby_values()
        self._worker.pulse(vals, ("bCmdStop",), "standby")
        self._btn_manual.set_local(False)
        self._btn_auto.set_local(False)

    def load_test_move(self) -> None:
        self.sub_tabs.setCurrentIndex(TAB_MOVE)
        self.move.load_test_move()

    # ── Mauer HMI locks ───────────────────────────────────────────────────────
    def set_run_lock(self, reason: str) -> None:
        """Lock jog and GO while a Mauer REAL run moves ARES ("" = unlock); releases a held jog."""
        if reason == self._run_lock:
            return
        self._run_lock = reason
        if reason:
            self.release_jog()
        self.move.set_run_lock(reason)
        self._run_banner.setToolTip(reason)
        self._run_banner.setVisible(bool(reason))
        self._apply_enables()

    def set_reset_lock(self, reason: str) -> None:
        """Lock the odometry 'Reset pose' while a REAL run is loaded ("" = unlock)."""
        self.odom.set_reset_lock(reason)

    # ── jog gating (called by MainWindow) ─────────────────────────────────────
    def release_jog(self, send: bool = True) -> None:
        self.jog.release_all(send=send)

    def jog_tab_active(self) -> bool:
        return self.sub_tabs.currentIndex() == TAB_JOG

    def handle_jog_key(self, key: int, pressed: bool) -> bool:
        """Keyboard jog; MainWindow already checked 'window active' and 'Control tab current'."""
        if not self.jog_tab_active():
            return False
        return self.jog.key_event(key, pressed)

    # ── status ────────────────────────────────────────────────────────────────
    def set_connection(self, connected: bool, msg: str = "") -> None:
        self._connected = connected
        if not connected:
            self.release_jog(send=False)
            set_text(self._conn_lbl, f"Disconnected ({elide(msg, CONN_MSG_MAX)})" if msg else "Disconnected")
            self._conn_lbl.setToolTip(msg)
            set_style(self._conn_lbl, f"color:{C.COL_BAD}; font-weight:bold;")
            mismatch = msg.startswith(MISMATCH_TEXT)
            set_text(self._mismatch_banner, f"Not connected - {msg}" if mismatch else "")
            self._mismatch_banner.setVisible(mismatch)
        else:
            set_text(self._conn_lbl, "Connected")
            self._conn_lbl.setToolTip("")
            set_style(self._conn_lbl, f"color:{C.COL_OK}; font-weight:bold;")
            self._mismatch_banner.hide()
        self.move.set_link(connected, self._if_version)
        self._apply_enables()

    def set_interface(self, version: int, build: str) -> None:
        self._if_version = version
        set_text(self._build_lbl, f"PLC: {build or '-'}  |  interface v{version}")
        self._legacy_banner.setVisible(version < IF_V2)
        self.odom.set_available(version >= IF_V2)
        self.move.set_link(self._connected, version)
        self._apply_enables()

    def update_status(self, data: Dict[str, Any]) -> None:
        self._status = data
        s = int(data.get("eAmrState", -1))
        self._state = s
        color, name = C.state_info(s)
        set_text(self._state_lbl, name)
        set_style(self._state_lbl, f"color:{color};")

        ext = bool(data.get("bExtActive", False))
        self._ext_banner.setVisible(ext)
        self._wd_banner.setVisible(self._connected and not bool(data.get("bHmiWatchdogOK", True)))

        # latch buttons follow the PLC (hold-off after a local click)
        self._btn_safety_run.sync(data.get("bSafetyRunActive", False))
        self._btn_manual.sync(data.get("bModeManual", False))
        self._btn_auto.sync(data.get("bModeAuto", False))

        # relative move running -> jog parameters (fAccel_mms) are locked/deferred; must precede set_manual so a
        # MANUAL exit that aborts a move defers its accel restore until the move has ended
        self.jog.set_move_active(bool(data.get("bMoveActive", False)))
        # MANUAL entry/exit: push jog parameters / restore accel + release jog (no-op without change)
        self.jog.set_manual(s == C.ST_MANUAL)

        self.jog.update_status(data)
        self.move.update_status(data)
        self.odom.update_status(data)      # trail also grows while hidden; a hidden map does not repaint
        self._update_drives(data)
        self._update_soc(data)
        self._apply_enables()

    def _apply_enables(self) -> None:
        s = self._state
        safety_run = self._btn_safety_run.is_active()
        en = button_enables(s, safety_run) if self._connected else {k: False for k in button_enables(0, False)}
        self._btn_safety_run.setEnabled(en["safety_run"])
        self._btn_rearm.setEnabled(en["rearm"])
        self._btn_reset.setEnabled(en["reset"])
        self._btn_start.setEnabled(en["start"])
        self._btn_manual.setEnabled(en["manual"])
        self._btn_auto.setEnabled(en["auto"])
        self._btn_standby.setEnabled(en["standby"])
        step = startup_step(s, safety_run) if self._connected else -1
        for i, b in enumerate(self._badges):
            b.set_mode("done" if i < step else ("current" if i == step else "pending"))
        set_text(self._hint, startup_hint(s, safety_run) if self._connected else "Not connected to the PLC.")
        self.jog.set_jog_enabled(jog_enabled(self._status, self._connected, self._run_lock))

    def _update_drives(self, d: Dict[str, Any]) -> None:
        w = self._d
        set_text(w["spdL"], f"{d.get('fActSpeed_Left_mms', 0.0):+.1f}")
        set_text(w["spdR"], f"{d.get('fActSpeed_Right_mms', 0.0):+.1f}")
        aal, aar = d.get("fActAngle_Left_deg", 0.0), d.get("fActAngle_Right_deg", 0.0)
        asl, asr = d.get("fSetAngle_Left_deg", 0.0), d.get("fSetAngle_Right_deg", 0.0)
        set_text(w["angL"], f"{aal:+.1f}")
        set_text(w["angR"], f"{aar:+.1f}")
        set_text(w["setL"], f"{asl:+.1f}")
        set_text(w["setR"], f"{asr:+.1f}")
        base = "font-family: monospace; background: transparent;"
        set_style(w["angL"], f"color:{'#FFAA00' if abs(aal - asl) > 5.0 else '#CCCCCC'}; {base}")
        set_style(w["angR"], f"color:{'#FFAA00' if abs(aar - asr) > 5.0 else '#CCCCCC'}; {base}")
        fault = d.get("bTractionFault", False) or d.get("bSteeringFault", False)
        ready = d.get("bTractionReady", False) and d.get("bSteeringReady", False)
        for side, flip, lim in (("L", "bWheelL_Flipped", "bWheelL_AtSoftLimit"),
                                ("R", "bWheelR_Flipped", "bWheelR_AtSoftLimit")):
            if fault:
                txt, col = "FAULT", "#FF5555"
            else:
                txt, col = ("Ready" if ready else "Not ready"), ("#55CC55" if ready else "#888888")
                if d.get(flip, False):
                    txt += " (F)"
                if d.get(lim, False):
                    txt += " LIM"
                    col = "#FFAA00"
            set_text(w["st" + side], txt)
            set_style(w["st" + side], f"color:{col}; font-weight:bold;")
        pairs = (("vx", "fActVx_mms", "fSetVx_mms"), ("vy", "fActVy_mms", "fSetVy_mms"),
                 ("om", "fActOmega_degs", "fSetOmega_degs"))
        moving = bool(d.get("bAmrMoving", False))
        for key, act, sp in pairs:
            set_text(w[key + "a"], f"{d.get(act, 0.0):+.1f}")
            set_text(w[key + "s"], f"{d.get(sp, 0.0):+.1f}")
            set_style(w[key + "a"], f"color:{'#44DD44' if moving else '#AAAAAA'}; {base}")

    def _update_soc(self, d: Dict[str, Any]) -> None:
        if not d.get("bBattConnected", False):
            set_text(self._soc_lbl, "SOC - (battery not connected)")
            set_style(self._soc_lbl, "color:#888888; font-weight:bold;")
            return
        soc = float(d.get("fBattSOC_pct", 0.0))
        col = "#66CC66" if soc > 30 else ("#FFA040" if soc > 15 else "#FF6060")
        set_text(self._soc_lbl, f"SOC {soc:.0f} %" + ("  LOW" if soc <= 15 else ""))
        set_style(self._soc_lbl, f"color:{col}; font-weight:bold;")
