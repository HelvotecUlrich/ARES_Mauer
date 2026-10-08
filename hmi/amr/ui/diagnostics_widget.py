"""
diagnostics_widget.py  –  Raw variable table and ADS diagnostics.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QSplitter,
    QTableWidget, QTableWidgetItem, QLabel, QGroupBox, QGridLayout,
    QHeaderView, QAbstractItemView,
)


def _item(text: str, align=Qt.AlignLeft) -> QTableWidgetItem:
    it = QTableWidgetItem(text)
    it.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
    it.setTextAlignment(align | Qt.AlignVCenter)
    return it


_VAR_GROUPS = [
    ("Watchdog", [
        ("nPlcHeartbeat",    "UINT",  "PLC heartbeat counter"),
        ("nCommandAck",      "UINT",  "Last acknowledged command ID"),
        ("bHmiWatchdogOK",   "BOOL",  "HMI watchdog active"),
        ("bPlcRunning",      "BOOL",  "PLC in RUN state"),
    ]),
    ("State", [
        ("eAmrState",        "INT",   "AMR state index (E_AMR_State)"),
        ("sAmrStateText",    "STRING","Human-readable state name"),
    ]),
    ("Mode", [
        ("bAmrReady",        "BOOL",  "AMR is ready for commands"),
        ("bModeManual",      "BOOL",  "Manual mode active"),
        ("bModeAuto",        "BOOL",  "Auto mode active"),
        ("bAmrMoving",       "BOOL",  "AMR is in motion"),
    ]),
    ("Safety", [
        ("bSafetyOK",            "BOOL", "Safety system OK"),
        ("bEmergencyStopActive", "BOOL", "E-Stop pressed"),
        ("bSafetyRunActive",     "BOOL", "Safety Run active"),
    ]),
    ("Drives", [
        ("bDrivesEnabled",   "BOOL",  "Drive power enabled"),
        ("bDrivesFault",     "BOOL",  "Any drive in fault"),
        ("bTractionReady",   "BOOL",  "Traction drives ready"),
        ("bTractionFault",   "BOOL",  "Traction drive fault"),
        ("bSteeringReady",   "BOOL",  "Steering drives ready"),
        ("bSteeringFault",   "BOOL",  "Steering drive fault"),
        ("nTractionErrorL",  "UINT",  "Left traction error code"),
        ("nTractionErrorR",  "UINT",  "Right traction error code"),
        ("nSteeringErrorL",  "UINT",  "Left steering error code"),
        ("nSteeringErrorR",  "UINT",  "Right steering error code"),
    ]),
    ("Velocity", [
        ("fActVx_mms",           "LREAL", "Body velocity X [mm/s]"),
        ("fActVy_mms",           "LREAL", "Body velocity Y [mm/s]"),
        ("fActOmega_degs",       "LREAL", "Body angular velocity [°/s]"),
        ("fActSpeed_Left_mms",   "LREAL", "Left wheel speed [mm/s]"),
        ("fActSpeed_Right_mms",  "LREAL", "Right wheel speed [mm/s]"),
        ("fActAngle_Left_deg",   "LREAL", "Left steering angle [°]"),
        ("fActAngle_Right_deg",  "LREAL", "Right steering angle [°]"),
        ("fSetVx_mms",           "LREAL", "Setpoint Vx [mm/s]"),
        ("fSetVy_mms",           "LREAL", "Setpoint Vy [mm/s]"),
        ("fSetOmega_degs",       "LREAL", "Setpoint omega [°/s]"),
    ]),
    ("Lamp", [
        ("eActiveLampColor", "INT",  "Active lamp color (E_LampColor)"),
        ("bLampBlink",       "BOOL", "Lamp blink active"),
    ]),
    ("Kinematics", [
        ("bWheelL_Flipped",      "BOOL", "Left wheel 180° flip active"),
        ("bWheelR_Flipped",      "BOOL", "Right wheel 180° flip active"),
        ("bWheelL_AtSoftLimit",  "BOOL", "Left wheel at soft limit"),
        ("bWheelR_AtSoftLimit",  "BOOL", "Right wheel at soft limit"),
    ]),
    ("Faults", [
        ("bFaultActive",   "BOOL",   "Active fault present"),
        ("nFaultCode",     "UINT",   "Fault code"),
        ("sFaultText",     "STRING", "Fault description"),
        ("bWarningActive", "BOOL",   "Active warning present"),
        ("nWarningCode",   "UINT",   "Warning code"),
        ("sWarningText",   "STRING", "Warning description"),
    ]),
    ("Battery", [
        ("fBattSOC_pct",    "LREAL", "State of charge [%]"),
        ("fBattVoltage_V",  "LREAL", "Battery voltage [V]"),
        ("fBattCurrent_A",  "LREAL", "Battery current [A]"),
        ("bBattCharging",   "BOOL",  "Charging active"),
        ("bBattConnected",  "BOOL",  "Battery connected"),
    ]),
    ("Navigation", [
        ("fPosX_m",           "LREAL", "Position X [m] (v2: odometry, PLC frame)"),
        ("fPosY_m",           "LREAL", "Position Y [m] (v2: odometry, PLC frame)"),
        ("fPosTheta_deg",     "LREAL", "Heading [°] (v2: odometry, PLC frame)"),
        ("bLocalizationOK",   "BOOL",  "Localization OK"),
        ("bObstacleDetected", "BOOL",  "Obstacle detected"),
    ]),
    ("Mission (not implemented in the PLC)", [
        ("nActiveMissionId", "UINT",   "Active mission ID"),
        ("sMissionStatus",   "STRING", "Mission status text"),
    ]),
    # ── interface v2 (PLC build v2, spec 5.2); "n/a" on a legacy PLC ──
    ("Interface v2", [
        ("nIfVersion",       "UINT",   "HMI interface version (GVL_Build)"),
        ("sPlcBuild",        "STRING", "PLC build text"),
        ("bExtActive",       "BOOL",   "External source (C6030) active - HMI motion blocked"),
        ("bFkLegacy",        "BOOL",   "fActV* computed with the old (wrong) FK formula"),
    ]),
    ("Relative move", [
        ("eMoveState",        "INT",    "E_MoveState"),
        ("eMoveResult",       "INT",    "E_MoveResult of the current/last move"),
        ("eMoveCmdResult",    "INT",    "Verdict on nMoveCmdAck: 0 accepted, 10..13 rejected"),
        ("nMoveCmdAck",       "UINT",   "Last acknowledged nMoveCmdId"),
        ("bMoveLimited",      "BOOL",   "Speed/accel of the current/last move limited by the PLC"),
        ("sMoveText",         "STRING", "PLC move text"),
        ("bMoveActive",       "BOOL",   "Move owns the setpoints"),
        ("bMoveIsRotation",   "BOOL",   "Rotation (else translation)"),
        ("fMoveTarget",       "LREAL",  "Target [mm or deg]"),
        ("fMoveDone",         "LREAL",  "Done [mm or deg]"),
        ("fMoveRemaining",    "LREAL",  "Remaining [mm or deg]"),
        ("fMoveLatErr_mm",    "LREAL",  "Lateral error / position drift [mm]"),
        ("fMoveHeadErr_deg",  "LREAL",  "Heading change [deg]"),
        ("fMoveProgress_pct", "LREAL",  "Progress [%]"),
        ("fMoveFinalErr",     "LREAL",  "Final error [mm or deg] (+ = short)"),
        ("fMoveElapsed_s",    "LREAL",  "Elapsed [s]"),
    ]),
    ("Odometry", [
        ("fOdomDist_m",      "LREAL", "Path length since reset [m]"),
        ("nOdomGlitchCnt",   "UINT",  "Discarded encoder jumps"),
    ]),
]


class DiagnosticsWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._row_map: Dict[str, int] = {}
        self._last_text: Dict[str, str] = {}
        self._setup_ui()

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        splitter = QSplitter(Qt.Horizontal)

        # Variable table
        self._table = QTableWidget()
        self._table.setColumnCount(4)
        self._table.setHorizontalHeaderLabels(["Variable", "Type", "Value", "Description"])
        self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setAlternatingRowColors(True)
        self._table.setStyleSheet("""
            QTableWidget { background:#1E1E1E; alternate-background-color:#252525;
                           color:#CCCCCC; gridline-color:#333; }
            QHeaderView::section { background:#333333; color:#AAAAAA;
                                   border:1px solid #555; padding:4px; font-weight:bold; }
        """)

        row = 0
        for group, vars_ in _VAR_GROUPS:
            # Group header row
            self._table.insertRow(row)
            hdr = QTableWidgetItem(f"  {group}")
            hdr.setFlags(Qt.ItemIsEnabled)
            hdr.setBackground(Qt.darkGray)
            hdr.setForeground(Qt.white)
            font = hdr.font()
            font.setBold(True)
            hdr.setFont(font)
            self._table.setItem(row, 0, hdr)
            self._table.setSpan(row, 0, 1, 4)
            row += 1

            for name, tc_type, desc in vars_:
                self._table.insertRow(row)
                self._table.setItem(row, 0, _item(f"  {name}"))
                self._table.setItem(row, 1, _item(tc_type, Qt.AlignCenter))
                self._table.setItem(row, 2, _item("—", Qt.AlignRight))
                self._table.setItem(row, 3, _item(desc))
                self._row_map[name] = row
                row += 1

        splitter.addWidget(self._table)

        # Right panel: counters and timestamps
        right_panel = QWidget()
        right_lay = QVBoxLayout(right_panel)
        right_lay.setContentsMargins(8, 0, 0, 0)

        stats_box = QGroupBox("ADS Statistics")
        stats_grid = QGridLayout(stats_box)
        stats_grid.setSpacing(4)

        self._lbl_plc_hb   = self._stat_label("—")
        self._lbl_hmi_cmd  = self._stat_label("—")
        self._lbl_last_upd = self._stat_label("—")
        self._lbl_poll_cnt = self._stat_label("0")
        self._lbl_if       = self._stat_label("—")
        self._poll_count   = 0

        stats_grid.addWidget(QLabel("PLC Heartbeat:"),   0, 0)
        stats_grid.addWidget(self._lbl_plc_hb,          0, 1)
        stats_grid.addWidget(QLabel("Last Cmd ACK:"),    1, 0)
        stats_grid.addWidget(self._lbl_hmi_cmd,         1, 1)
        stats_grid.addWidget(QLabel("Last Update:"),     2, 0)
        stats_grid.addWidget(self._lbl_last_upd,        2, 1)
        stats_grid.addWidget(QLabel("Poll Count:"),      3, 0)
        stats_grid.addWidget(self._lbl_poll_cnt,        3, 1)
        stats_grid.addWidget(QLabel("Interface:"),       4, 0)
        stats_grid.addWidget(self._lbl_if,              4, 1)

        right_lay.addWidget(stats_box)
        right_lay.addStretch()

        splitter.addWidget(right_panel)
        splitter.setSizes([800, 250])

        root.addWidget(splitter)

    @staticmethod
    def _stat_label(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet("color:#44CCFF; font-family: Courier; font-weight: bold;")
        return lbl

    def set_interface(self, version: int, build: str) -> None:
        self._lbl_if.setText(f"v{version}  {build}")

    def update_status(self, data: Dict[str, Any]) -> None:
        """Called only while the tab is visible (MainWindow); only changed cells are touched."""
        self._poll_count += 1
        self._lbl_poll_cnt.setText(str(self._poll_count))
        self._lbl_last_upd.setText(time.strftime("%H:%M:%S"))
        self._lbl_plc_hb.setText(str(data.get("nPlcHeartbeat", "—")))
        self._lbl_hmi_cmd.setText(str(data.get("nCommandAck", "—")))

        for name, row in self._row_map.items():
            val = data.get(name)
            if val is None:
                text = "n/a"
            elif isinstance(val, bool):
                text = "TRUE" if val else "FALSE"
            elif isinstance(val, float):
                text = f"{val:.4f}"
            else:
                text = str(val)
            if self._last_text.get(name) == text:
                continue
            self._last_text[name] = text
            item = self._table.item(row, 2)
            if item:
                item.setText(text)
                if isinstance(val, bool):
                    item.setForeground(Qt.green if val else Qt.darkGray)
