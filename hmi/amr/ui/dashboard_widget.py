"""
dashboard_widget.py  –  AMR status overview tab.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QScrollArea, QVBoxLayout, QWidget,
)

from . import constants as C
from .widgets import bool_css, set_style, set_text

log = logging.getLogger(__name__)


def _bool_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setFixedHeight(26)
    return lbl


def _set_bool_label(lbl: QLabel, value: bool, true_color: str = C.COL_OK,
                    false_color: str = C.COL_IDLE) -> None:
    """Style only on change (review r5 2.2)."""
    set_style(lbl, bool_css(bool(value), true_color, false_color))


class _LampWidget(QWidget):
    """Round colored lamp indicator."""

    def __init__(self, size: int = 48, parent=None):
        super().__init__(parent)
        self._color = QColor("#222222")
        self._blink_state = False
        self._blink = False
        self.setFixedSize(size, size)

        self._blink_timer = QTimer(self)
        self._blink_timer.timeout.connect(self._toggle_blink)
        self._blink_timer.start(500)

    def set_color(self, hex_color: str, blink: bool = False) -> None:
        if QColor(hex_color) == self._color and blink == self._blink:
            return
        self._color = QColor(hex_color)
        self._blink = blink
        if not blink:
            self._blink_state = False
        self.update()

    def _toggle_blink(self) -> None:
        if self._blink:
            self._blink_state = not self._blink_state
            self.update()

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        color = self._color if (not self._blink or self._blink_state) else QColor("#222222")
        p.setBrush(QBrush(color))
        p.setPen(QPen(QColor("#666666"), 2))
        r = self.rect().adjusted(4, 4, -4, -4)
        p.drawEllipse(r)


class DashboardWidget(QWidget):
    """Read-only status overview (works with interface v1 and v2 status dicts)."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        content = QWidget()
        root = QHBoxLayout(content)
        root.setContentsMargins(8, 8, 8, 8)

        left = QVBoxLayout()
        right = QVBoxLayout()
        root.addLayout(left, 3)
        root.addLayout(right, 2)

        # --- State panel ---
        state_box = QGroupBox("AMR State")
        state_lay = QVBoxLayout(state_box)

        self._lamp = _LampWidget(64)
        lamp_row = QHBoxLayout()
        lamp_row.addStretch()
        lamp_row.addWidget(self._lamp)
        lamp_row.addStretch()
        state_lay.addLayout(lamp_row)

        self._lbl_state = QLabel("—")
        font = QFont()
        font.setPointSize(20)
        font.setBold(True)
        self._lbl_state.setFont(font)
        self._lbl_state.setAlignment(Qt.AlignCenter)
        state_lay.addWidget(self._lbl_state)

        self._lbl_state_idx = QLabel("State: —")
        self._lbl_state_idx.setAlignment(Qt.AlignCenter)
        self._lbl_state_idx.setStyleSheet("color: #AAAAAA;")
        state_lay.addWidget(self._lbl_state_idx)

        left.addWidget(state_box)

        # --- Velocity panel ---
        vel_box = QGroupBox("Velocity (actual)")
        vel_grid = QGridLayout(vel_box)
        vel_grid.setHorizontalSpacing(16)

        headers = ["Vx [mm/s]", "Vy [mm/s]", "ω [°/s]"]
        self._vel_labels: list[QLabel] = []
        for col, h in enumerate(headers):
            vel_grid.addWidget(QLabel(h), 0, col, alignment=Qt.AlignCenter)
            lbl = QLabel("0.0")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Courier", 14))
            lbl.setStyleSheet("color: #44CCFF; font-weight: bold;")
            vel_grid.addWidget(lbl, 1, col)
            self._vel_labels.append(lbl)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        vel_grid.addWidget(sep, 2, 0, 1, 3)

        wheel_headers = ["V_L [mm/s]", "V_R [mm/s]", "δ_L [°]", "δ_R [°]"]
        self._wheel_labels: list[QLabel] = []
        for col, h in enumerate(wheel_headers):
            vel_grid.addWidget(QLabel(h), 3, col, alignment=Qt.AlignCenter)
            lbl = QLabel("0.0")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Courier", 12))
            vel_grid.addWidget(lbl, 4, col)
            self._wheel_labels.append(lbl)

        left.addWidget(vel_box)

        # --- Safety & Mode panel ---
        sm_box = QGroupBox("Safety & Mode")
        sm_grid = QGridLayout(sm_box)
        sm_grid.setSpacing(4)

        self._lbl_safety   = _bool_label("Safety OK")
        self._lbl_estop    = _bool_label("E-Stop")
        self._lbl_safetyRun = _bool_label("Safety Run")
        self._lbl_manual   = _bool_label("Manual")
        self._lbl_auto     = _bool_label("Auto")
        self._lbl_moving   = _bool_label("Moving")
        self._lbl_fault    = _bool_label("Fault")
        self._lbl_warning  = _bool_label("Warning")
        self._lbl_ext      = _bool_label("Ext active (C6030)")
        self._lbl_move     = _bool_label("Rel. move active")

        for i, w in enumerate([self._lbl_safety, self._lbl_estop, self._lbl_safetyRun,
                                 self._lbl_manual, self._lbl_auto, self._lbl_moving]):
            sm_grid.addWidget(w, i // 3, i % 3)
        for i, w in enumerate([self._lbl_fault, self._lbl_warning,
                                 self._lbl_ext, self._lbl_move]):
            sm_grid.addWidget(w, 2 + i // 2, i % 2)

        left.addWidget(sm_box)

        # --- Drives panel ---
        drv_box = QGroupBox("Drives")
        drv_grid = QGridLayout(drv_box)
        drv_grid.setSpacing(4)

        drv_labels = [
            ("Enabled",     "_lbl_drv_en"),
            ("Traction Rdy","_lbl_tr_rdy"),
            ("Traction Flt","_lbl_tr_flt"),
            ("Steering Rdy","_lbl_st_rdy"),
            ("Steering Flt","_lbl_st_flt"),
            ("L Flipped",   "_lbl_flip_l"),
            ("R Flipped",   "_lbl_flip_r"),
            ("L SoftLimit", "_lbl_sl_l"),
            ("R SoftLimit", "_lbl_sl_r"),
        ]
        for i, (text, attr) in enumerate(drv_labels):
            lbl = _bool_label(text)
            setattr(self, attr, lbl)
            drv_grid.addWidget(lbl, i // 3, i % 3)

        right.addWidget(drv_box)

        # --- Fault / Warning panel ---
        fw_box = QGroupBox("Fault / Warning")
        fw_lay = QVBoxLayout(fw_box)
        self._lbl_fault_text   = QLabel("No fault")
        self._lbl_fault_text.setStyleSheet("color: #FF8888;")
        self._lbl_warning_text = QLabel("No warning")
        self._lbl_warning_text.setStyleSheet("color: #FFCC44;")
        fw_lay.addWidget(self._lbl_fault_text)
        fw_lay.addWidget(self._lbl_warning_text)
        right.addWidget(fw_box)

        # --- Battery panel ---
        bat_box = QGroupBox("Battery")
        bat_grid = QGridLayout(bat_box)

        bat_headers = ["SOC [%]", "Voltage [V]", "Current [A]"]
        self._bat_labels: list[QLabel] = []
        for col, h in enumerate(bat_headers):
            bat_grid.addWidget(QLabel(h), 0, col, alignment=Qt.AlignCenter)
            lbl = QLabel("—")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Courier", 13))
            lbl.setStyleSheet("color: #44FF88; font-weight: bold;")
            bat_grid.addWidget(lbl, 1, col)
            self._bat_labels.append(lbl)

        self._lbl_charging = _bool_label("Charging")
        bat_grid.addWidget(self._lbl_charging, 2, 0)
        self._lbl_bat_conn = _bool_label("Connected")
        bat_grid.addWidget(self._lbl_bat_conn, 2, 1)
        right.addWidget(bat_box)

        right.addStretch()

        scroll = QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._apply_dark_style()

    def _apply_dark_style(self) -> None:
        self.setStyleSheet("""
            QGroupBox { border: 1px solid #555; border-radius:6px; margin-top:6px;
                        font-weight:bold; color:#CCCCCC; }
            QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; }
            QLabel { color: #CCCCCC; }
        """)

    # ------------------------------------------------------------------

    def update_status(self, data: Dict[str, Any]) -> None:
        state_idx = data.get("eAmrState", -1)
        color, name = C.state_info(state_idx)
        set_text(self._lbl_state, name)
        set_style(self._lbl_state, f"color: {color}; font-weight: bold;")
        set_text(self._lbl_state_idx, f"State index: {state_idx}")

        lamp_color = C.LAMP_COLORS.get(int(data.get("eActiveLampColor", 0)), C.LAMP_OFF)
        self._lamp.set_color(lamp_color, bool(data.get("bLampBlink", False)))

        # Velocity
        set_text(self._vel_labels[0], f"{data.get('fActVx_mms', 0.0):+.1f}")
        set_text(self._vel_labels[1], f"{data.get('fActVy_mms', 0.0):+.1f}")
        set_text(self._vel_labels[2], f"{data.get('fActOmega_degs', 0.0):+.1f}")
        set_text(self._wheel_labels[0], f"{data.get('fActSpeed_Left_mms', 0.0):+.1f}")
        set_text(self._wheel_labels[1], f"{data.get('fActSpeed_Right_mms', 0.0):+.1f}")
        set_text(self._wheel_labels[2], f"{data.get('fActAngle_Left_deg', 0.0):+.1f}")
        set_text(self._wheel_labels[3], f"{data.get('fActAngle_Right_deg', 0.0):+.1f}")

        red, amber = "#FF3333", "#FFAA00"
        # Safety & mode
        _set_bool_label(self._lbl_safety, data.get("bSafetyOK", False))
        _set_bool_label(self._lbl_estop, data.get("bEmergencyStopActive", False), red)
        _set_bool_label(self._lbl_safetyRun, data.get("bSafetyRunActive", False))
        _set_bool_label(self._lbl_manual, data.get("bModeManual", False))
        _set_bool_label(self._lbl_auto, data.get("bModeAuto", False))
        _set_bool_label(self._lbl_moving, data.get("bAmrMoving", False))
        _set_bool_label(self._lbl_fault, data.get("bFaultActive", False), red)
        _set_bool_label(self._lbl_warning, data.get("bWarningActive", False), amber)
        _set_bool_label(self._lbl_ext, data.get("bExtActive", False), red)
        _set_bool_label(self._lbl_move, data.get("bMoveActive", False), amber)

        # Drives
        _set_bool_label(self._lbl_drv_en, data.get("bDrivesEnabled", False))
        _set_bool_label(self._lbl_tr_rdy, data.get("bTractionReady", False))
        _set_bool_label(self._lbl_tr_flt, data.get("bTractionFault", False), red)
        _set_bool_label(self._lbl_st_rdy, data.get("bSteeringReady", False))
        _set_bool_label(self._lbl_st_flt, data.get("bSteeringFault", False), red)
        _set_bool_label(self._lbl_flip_l, data.get("bWheelL_Flipped", False), amber)
        _set_bool_label(self._lbl_flip_r, data.get("bWheelR_Flipped", False), amber)
        _set_bool_label(self._lbl_sl_l, data.get("bWheelL_AtSoftLimit", False), amber)
        _set_bool_label(self._lbl_sl_r, data.get("bWheelR_AtSoftLimit", False), amber)

        # Fault / Warning texts
        if data.get("bFaultActive", False):
            set_text(self._lbl_fault_text, f"Fault {data.get('nFaultCode', 0)}: {data.get('sFaultText', '')}")
        else:
            set_text(self._lbl_fault_text, "No fault")
        if data.get("bWarningActive", False):
            set_text(self._lbl_warning_text,
                     f"Warning {data.get('nWarningCode', 0)}: {data.get('sWarningText', '')}")
        else:
            set_text(self._lbl_warning_text, "No warning")

        # Battery
        set_text(self._bat_labels[0], f"{data.get('fBattSOC_pct', 0.0):.1f}")
        set_text(self._bat_labels[1], f"{data.get('fBattVoltage_V', 0.0):.2f}")
        set_text(self._bat_labels[2], f"{data.get('fBattCurrent_A', 0.0):+.2f}")
        _set_bool_label(self._lbl_charging, data.get("bBattCharging", False), "#44CCFF")
        _set_bool_label(self._lbl_bat_conn, data.get("bBattConnected", False))
