"""
move_panel.py - Relative move (spec 6.5): direction pad, parameters, GO with second-click confirmation,
progress and result display.

Directions are PHYSICAL (pad) and mapped to the PLC frame through frame.py; dx/dy are shown and editable in
the PLC frame, their captions name the physical side of PLC +Y under the current config. GO sends ONE write
(fMove*, nMoveCmdId, bCmdMoveStart TRUE); the worker resets bCmdMoveStart after 300 ms.
Acknowledgement (spec section 9): nMoveCmdAck == sent id, verdict from eMoveCmdResult (0 accepted, 10..13
rejected with sMoveText); eMoveResult is the result of the move. bMoveLimited during a move -> "limited by PLC".
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional, Tuple

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QButtonGroup, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QProgressBar, QPushButton,
    QRadioButton, QVBoxLayout, QWidget,
)

from .. import frame as F
from ..logic import (
    ACK_ACCEPTED, ACK_INVALID, ACK_REJECTED, MoveDefaults, MoveLimits, MoveRequest, build_move_command,
    go_enabled, move_ack_state, next_move_id, target_pose, validate_move,
)
from . import constants as C
from .widgets import PulseBtn, lbl, num_label, set_style, set_text

CONFIRM_S = 3.0          # second click must follow within this time
ACK_TIMEOUT_S = 2.0      # no nMoveCmdAck within this time -> warning

_PAD = [
    (F.FWD_LEFT, "↖ Fwd-L"), (F.FORWARD, "↑ Fwd"), (F.FWD_RIGHT, "↗ Fwd-R"),
    (F.LEFT, "← Left"), (None, ""), (F.RIGHT, "Right →"),
    (F.BACK_LEFT, "↙ Back-L"), (F.BACK, "↓ Back"), (F.BACK_RIGHT, "↘ Back-R"),
]
_PAD_CSS = (
    "QPushButton{background:#1A2A3A;color:#AACCEE;border-radius:5px;border:1px solid #2A4A6A;}"
    "QPushButton:checked{background:#1A5A9A;color:#FFFFFF;border:2px solid #7ABAFA;}"
    "QPushButton:disabled{background:#141414;color:#383838;border:1px solid #252525;}")
_GO_IDLE = ("QPushButton{background:#0A6E30;color:#FFFFFF;border-radius:6px;font-weight:bold;"
            "border:2px solid #22AA55;min-height:40px;}"
            "QPushButton:disabled{background:#1E2A22;color:#4A5A4E;border:2px solid #2A3A2E;}")
_GO_ARMED = ("QPushButton{background:#B87800;color:#000000;border-radius:6px;font-weight:bold;"
             "border:2px solid #FFCC44;min-height:40px;}")


def _spin(lo: float, hi: float, step: float, value: float, suffix: str, decimals: int = 0) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    s.setValue(value)
    s.setSuffix(suffix)
    s.setMinimumWidth(92)
    return s


class MovePanel(QWidget):
    """Relative move controls. target_changed emits (x_m, y_m, theta_deg) or None for the odometry map."""

    target_changed = Signal(object)

    def __init__(self, cfg: Dict[str, Any], frame_cfg: F.FrameConfig, worker: Any,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._worker = worker
        self._frame = frame_cfg
        self._lim = MoveLimits.from_config(cfg)
        self._def = MoveDefaults.from_config(cfg)
        self._dir: Optional[str] = F.FORWARD       # selected translation direction, None = free dx/dy
        self._rot_dir: str = F.CCW
        self._status: Dict[str, Any] = {}
        self._connected = False
        self._if_version = 0
        self._armed_at: Optional[float] = None
        self._last_id = 0
        self._await_id: Optional[int] = None
        self._await_since = 0.0
        self._sent_req: Optional[MoveRequest] = None
        self._start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._feedback = ""
        self._build_ui()
        self._select_direction(F.FORWARD)
        self._refresh()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(6)

        # direction pad
        pad_box = QGroupBox("Direction (physical, seen from above)")
        pad = QGridLayout(pad_box)
        pad.setSpacing(4)
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        self._dir_buttons: Dict[str, QPushButton] = {}
        for i, (d, text) in enumerate(_PAD):
            if d is None:
                continue
            b = self._make_dir_button(text, d)
            pad.addWidget(b, i // 3, i % 3)
        pad.setColumnMinimumWidth(3, 12)
        for row, d, text in ((0, F.CCW, "⟲ CCW"), (2, F.CW, "⟳ CW")):
            b = self._make_dir_button(text, d)
            pad.addWidget(b, row, 4)
        pad.addWidget(lbl("Rotate", "#777777"), 1, 4, alignment=Qt.AlignCenter)
        root.addWidget(pad_box)

        # target + profile in one compact grid
        tgt = QGroupBox("Target and profile")
        tg = QGridLayout(tgt)
        tg.setVerticalSpacing(3)
        self._rb_trans = QRadioButton("Translation")
        self._rb_rot = QRadioButton("Rotation")
        self._rb_trans.setChecked(True)
        self._rb_trans.toggled.connect(lambda _c: self._changed())
        tg.addWidget(self._rb_trans, 0, 0, 1, 2)
        tg.addWidget(self._rb_rot, 0, 2, 1, 2)
        self._dist = _spin(self._lim.min_distance_mm, self._lim.max_distance_mm, 50,
                           self._def.distance_mm, " mm")
        self._dx = _spin(-self._lim.max_distance_mm, self._lim.max_distance_mm, 50, 0.0, " mm", 1)
        self._dy = _spin(-self._lim.max_distance_mm, self._lim.max_distance_mm, 50, 0.0, " mm", 1)
        self._ang = _spin(self._lim.min_angle_deg, self._lim.max_angle_deg, 5, self._def.angle_deg, " deg", 1)
        self._spd = _spin(1, self._lim.max_speed_mms, 10, self._def.speed_mms, " mm/s")
        self._rspd = _spin(0.5, self._lim.max_rot_speed_degs, 1, self._def.rot_speed_degs, " deg/s", 1)
        self._acc = _spin(20, self._lim.max_accel_mms2, 20, self._def.accel_mms2, " mm/s2")
        self._acc.setToolTip("Path acceleration/deceleration (rotation: at the wheel).\n"
                             "The PLC fixes a = min(this, GVL_Move.fMaxAccel_mms2, 'Accel (MANUAL)' of the Jog tab)\n"
                             "when it accepts the move and forces it as traction ramp during the move (incl. abort).")
        side = F.physical_y_name(1.0, self._frame)
        mark = "" if self._frame.verified else "*"
        self._dx.setToolTip("PLC frame: +X = forward")
        self._dy.setToolTip(f"PLC frame: {F.plus_y_text(self._frame)} - config frame.plus_y_is_left: "
                            f"{str(self._frame.plus_y_is_left).lower()}")
        grid = [
            (1, 0, "Distance:", self._dist), (1, 2, "Angle:", self._ang),
            (2, 0, "dx (+X fwd):", self._dx), (2, 2, f"dy (+Y {side}{mark}):", self._dy),
            (3, 0, "Speed:", self._spd), (3, 2, "Rot speed:", self._rspd),
            (4, 0, "Accel:", self._acc),
        ]
        for r, c, cap, w in grid:
            w_lbl = lbl(cap)
            tg.addWidget(w_lbl, r, c)
            tg.addWidget(w, r, c + 1)
            if w is self._dy:
                self._dy_caption = w_lbl
        self._frame_note = lbl(self.frame_note_text(), "#888888")
        self._frame_note.setToolTip(f"dx/dy are sent in the PLC frame: +X = forward, {F.plus_y_text(self._frame)}.\n"
                                    f"Rotation: {F.plus_omega_text(self._frame)}.\n"
                                    "Config section 'frame'; verify with the test moves (README).")
        tg.addWidget(self._frame_note, 5, 0, 1, 4)
        self._ang.setToolTip(f"Angle magnitude; the pad chooses CCW/CW. {F.plus_omega_text(self._frame)}")
        self._dist.valueChanged.connect(self._on_distance)
        self._dx.valueChanged.connect(self._on_dxdy)
        self._dy.valueChanged.connect(self._on_dxdy)
        self._ang.valueChanged.connect(lambda _v: self._changed())
        for sp in (self._spd, self._rspd, self._acc):
            sp.valueChanged.connect(lambda _v: self._changed())
        root.addWidget(tgt)

        # command + GO
        self._cmd_lbl = QLabel("")
        self._cmd_lbl.setWordWrap(True)
        self._cmd_lbl.setStyleSheet("color:#DDDDDD; font-family: monospace;")
        root.addWidget(self._cmd_lbl)
        row = QHBoxLayout()
        self._go = QPushButton("GO")
        f = QFont()
        f.setPointSize(12)
        f.setBold(True)
        self._go.setFont(f)
        self._go.setFocusPolicy(Qt.NoFocus)
        self._go.clicked.connect(self.on_go_clicked)
        set_style(self._go, _GO_IDLE)
        self._abort = PulseBtn("Abort move", "#7A1515", min_w=100, min_h=40)
        self._abort.setToolTip("bCmdMoveAbort pulse: ramped stop of the relative move only (HALT does more).")
        self._abort.clicked.connect(self._on_abort)
        row.addWidget(self._go, 3)
        row.addWidget(self._abort, 1)
        root.addLayout(row)
        self._reason = lbl("", "#FFAA00")
        self._reason.setWordWrap(True)
        root.addWidget(self._reason)

        # progress
        pr = QGroupBox("Progress")
        pl = QGridLayout(pr)
        pl.setVerticalSpacing(2)
        self._bar = QProgressBar()
        self._bar.setRange(0, 100)
        self._bar.setTextVisible(True)
        pl.addWidget(self._bar, 0, 0, 1, 4)
        self._vals: Dict[str, QLabel] = {}
        items = [("state", "State"), ("elapsed", "Elapsed"), ("target", "Target"), ("done", "Done"),
                 ("remaining", "Remaining"), ("final", "Final error"), ("lat", "Lateral error"),
                 ("head", "Heading change")]
        for i, (key, text) in enumerate(items):
            r, c = 1 + i // 2, (i % 2) * 2
            cap = lbl(text + ":")
            pl.addWidget(cap, r, c)
            v = num_label()
            pl.addWidget(v, r, c + 1)
            self._vals[key] = v
            if key == "lat":
                self._lat_caption = cap
        self._limited = lbl("", "#FFAA00", bold=True)
        self._limited.setToolTip("bMoveLimited: the PLC reduced the requested speed and/or acceleration\n"
                                 "(GVL_Move.fMaxSpeed_mms / fMaxRotSpeed_degs / fMaxAccel_mms2 or the Jog-tab "
                                 "'Accel (MANUAL)').")
        self._limited.hide()
        pl.addWidget(self._limited, 5, 0, 1, 4)
        self._result = QLabel("-")
        self._result.setWordWrap(True)
        pl.addWidget(self._result, 6, 0, 1, 4)
        root.addWidget(pr)
        root.addStretch()

    def _make_dir_button(self, text: str, direction: str) -> QPushButton:
        b = QPushButton(text)
        b.setCheckable(True)
        b.setFocusPolicy(Qt.NoFocus)
        b.setMinimumSize(76, 34)
        b.setStyleSheet(_PAD_CSS)
        b.clicked.connect(lambda _c=False, d=direction: self._select_direction(d))
        self._group.addButton(b)
        self._dir_buttons[direction] = b
        return b

    # ── parameter handling ────────────────────────────────────────────────────
    def _select_direction(self, direction: str) -> None:
        if direction in F.ROTATIONS:
            self._rot_dir = direction
            self._rb_rot.setChecked(True)
        else:
            self._dir = direction
            self._rb_trans.setChecked(True)
            self._apply_direction()
        self._dir_buttons[direction].setChecked(True)
        self._changed()

    def _apply_direction(self) -> None:
        if self._dir is None:
            return
        dx, dy = F.translation_vector(self._dir, self._dist.value(), self._frame)
        self._set_dxdy(dx, dy)

    def _set_dxdy(self, dx: float, dy: float) -> None:
        for s, v in ((self._dx, dx), (self._dy, dy)):
            s.blockSignals(True)
            s.setValue(v)
            s.blockSignals(False)

    def _on_distance(self, value: float) -> None:
        if self._dir is not None:
            self._apply_direction()
        else:   # free vector: keep the direction, scale to the new length
            dx, dy = self._dx.value(), self._dy.value()
            n = (dx * dx + dy * dy) ** 0.5
            if n > 1e-6:
                self._set_dxdy(dx * value / n, dy * value / n)
        self._changed()

    def _on_dxdy(self, _value: float) -> None:
        # manual edit -> free vector, pad selection cleared
        self._dir = None
        self._group.setExclusive(False)
        for d in F.TRANSLATIONS:
            self._dir_buttons[d].setChecked(False)
        self._group.setExclusive(True)
        self._rb_trans.setChecked(True)
        n = (self._dx.value() ** 2 + self._dy.value() ** 2) ** 0.5
        self._dist.blockSignals(True)
        self._dist.setValue(max(n, self._dist.minimum()))
        self._dist.blockSignals(False)
        self._changed()

    def frame_note_text(self) -> str:
        """One line below dx/dy: which physical side PLC +Y is under the current config."""
        star = "" if self._frame.verified else "* "
        return f"{star}dx/dy: PLC frame, {F.plus_y_text(self._frame)}"

    def current_request(self) -> MoveRequest:
        common = dict(speed_mms=float(self._spd.value()), rot_speed_degs=float(self._rspd.value()),
                      accel_mms2=float(self._acc.value()))
        if self._rb_rot.isChecked():
            theta = F.rotation_angle(self._rot_dir, self._ang.value(), self._frame)
            return MoveRequest(True, dtheta_deg=theta, **common)
        return MoveRequest(False, dx_mm=float(self._dx.value()), dy_mm=float(self._dy.value()), **common)

    def describe(self, req: MoveRequest) -> str:
        if req.is_rotation:
            return f"{F.describe_rotation(req.dtheta_deg, self._frame)} at {req.rot_speed_degs:.1f} deg/s"
        return f"{F.describe_translation(req.dx_mm, req.dy_mm, self._frame)} at {req.speed_mms:.0f} mm/s"

    def load_test_move(self) -> None:
        """Spec 6.7: physical Left 100 mm at 50 mm/s; GO still needs two clicks."""
        self._disarm()
        self._dist.blockSignals(True)
        self._dist.setValue(self._def.test_distance_mm)
        self._dist.blockSignals(False)
        self._spd.blockSignals(True)
        self._spd.setValue(self._def.test_speed_mms)
        self._spd.blockSignals(False)
        self._select_direction(F.LEFT)
        self._feedback = "Test move loaded: watch in which direction ARES moves (expected: physical LEFT)."

    # ── GO / abort ────────────────────────────────────────────────────────────
    @property
    def armed(self) -> bool:
        return self._armed_at is not None and time.monotonic() - self._armed_at <= CONFIRM_S

    def go_state(self) -> Tuple[bool, str]:
        err = validate_move(self.current_request(), self._lim)
        return go_enabled(self._status, self._connected, self._if_version, err, self._await_id is not None)

    def on_go_clicked(self) -> None:
        ok, reason = self.go_state()
        if not ok:
            self._disarm()
            self._feedback = f"GO not possible: {reason}"
            self._refresh()
            return
        req = self.current_request()
        if not self.armed:
            self._armed_at = time.monotonic()
            token = self._armed_at
            QTimer.singleShot(int(CONFIRM_S * 1000), lambda: self._disarm_if(token))
            self._refresh()
            return
        self._send(req)

    def _send(self, req: MoveRequest) -> None:
        ack = int(self._status.get("nMoveCmdAck", 0))
        mid = next_move_id(self._last_id, ack)
        cmd = build_move_command(req, mid)
        self._worker.pulse(cmd, ("bCmdMoveStart",), f"GO move #{mid}")
        self._last_id = mid
        self._await_id = mid
        self._await_since = time.monotonic()
        self._sent_req = req
        self._start_pose = (float(self._status.get("fPosX_m", 0.0)), float(self._status.get("fPosY_m", 0.0)),
                            float(self._status.get("fPosTheta_deg", 0.0)))
        self._feedback = f"Sent move #{mid}: {self.describe(req)}"
        self.target_changed.emit(None)        # marker of the previous move is removed until the ack
        self._disarm()

    def _on_abort(self) -> None:
        self._worker.pulse({}, ("bCmdMoveAbort",), "abort move")
        self._disarm()

    def _disarm_if(self, token: float) -> None:
        if self._armed_at == token:
            self._disarm()

    def _disarm(self) -> None:
        if self._armed_at is not None:
            self._armed_at = None
            self._refresh()

    # ── status ────────────────────────────────────────────────────────────────
    def set_link(self, connected: bool, if_version: int) -> None:
        self._connected = connected
        self._if_version = if_version
        if not connected:
            self._await_id = None
        self._refresh()

    def update_status(self, data: Dict[str, Any]) -> None:
        self._status = data
        self._track_ack(data)
        self._show_progress(data)
        self._refresh()

    def _track_ack(self, data: Dict[str, Any]) -> None:
        if self._await_id is None:
            return
        st = move_ack_state(data, self._await_id)
        if st == ACK_REJECTED:
            res = int(data.get("eMoveCmdResult", 0))
            self._feedback = (f"REJECTED by PLC (move #{self._await_id}): {C.move_result_text(res)}"
                              f" - {data.get('sMoveText', '')}")
            self._await_id = None
        elif st == ACK_INVALID:
            self._feedback = (f"Move #{self._await_id}: unexpected PLC verdict eMoveCmdResult = "
                              f"{data.get('eMoveCmdResult')} (expected 0 or 10..13) - check the PLC build.")
            self._await_id = None
        elif st == ACK_ACCEPTED:
            limited = " (LIMITED by PLC)" if data.get("bMoveLimited", False) else ""
            self._feedback = f"Move #{self._await_id} accepted by the PLC{limited}."
            self._await_id = None
            if self._sent_req is not None:
                self.target_changed.emit(target_pose(self._start_pose, self._sent_req))
        elif time.monotonic() - self._await_since > ACK_TIMEOUT_S:
            self._feedback = (f"No acknowledgement for move #{self._await_id} within {ACK_TIMEOUT_S:.0f} s "
                              "(command lost or PLC without move support).")
            self._await_id = None

    def _show_progress(self, d: Dict[str, Any]) -> None:
        rot = bool(d.get("bMoveIsRotation", False))
        u = "deg" if rot else "mm"
        self._bar.setValue(int(round(max(0.0, min(100.0, float(d.get("fMoveProgress_pct", 0.0)))))))
        v = self._vals
        set_text(v["state"], C.move_state_text(int(d.get("eMoveState", 0))).split(" (")[0])
        set_text(v["elapsed"], f"{float(d.get('fMoveElapsed_s', 0.0)):.1f} s")
        set_text(v["target"], f"{float(d.get('fMoveTarget', 0.0)):.1f} {u}")
        set_text(v["done"], f"{float(d.get('fMoveDone', 0.0)):.1f} {u}")
        set_text(v["remaining"], f"{float(d.get('fMoveRemaining', 0.0)):.1f} {u}")
        state = int(d.get("eMoveState", 0))
        final = float(d.get("fMoveFinalErr", 0.0))
        set_text(v["final"], f"{final:+.2f} {u}" if state in (C.MOVE_DONE, C.MOVE_ABORTED) else "-")
        set_text(self._lat_caption, "Position drift:" if rot else "Lateral error:")
        set_text(v["lat"], f"{float(d.get('fMoveLatErr_mm', 0.0)):+.1f} mm")
        set_text(v["head"], "-" if rot else f"{float(d.get('fMoveHeadErr_deg', 0.0)):+.2f} deg")
        limited = bool(d.get("bMoveActive", False)) and bool(d.get("bMoveLimited", False))
        set_text(self._limited, "LIMITED by PLC: speed/accel reduced" if limited else "")
        self._limited.setVisible(limited)
        res = int(d.get("eMoveResult", 0))
        plc_txt = str(d.get("sMoveText", "") or "")
        lines = [] if (res == C.RES_NONE and plc_txt) else [C.move_result_text(res)]   # no "-" line while running
        if plc_txt:
            lines.append(f"PLC: {plc_txt}")
        set_text(self._result, "\n".join(lines))
        if C.is_reject(res) or C.is_abort(res) or res == C.RES_OK_TOL:
            col = "#FF6060" if (C.is_reject(res) or C.is_abort(res)) else "#FFAA00"
        elif res == C.RES_OK:
            col = "#55DD55"
        else:
            col = "#CCCCCC"
        set_style(self._result, f"color:{col}; font-weight:bold;")

    def _changed(self) -> None:
        """A parameter changed: a pending confirmation is cancelled (GO must be armed again)."""
        self._armed_at = None
        self._refresh()

    def _refresh(self) -> None:
        req = self.current_request()
        rot = req.is_rotation
        for w in (self._dist, self._dx, self._dy):
            w.setEnabled(not rot)
        self._ang.setEnabled(rot)
        ok, reason = self.go_state()
        if not ok and self._armed_at is not None:
            self._armed_at = None
        self._go.setEnabled(ok)
        v2 = self._connected and self._if_version >= 2
        self._abort.setEnabled(v2)
        desc = self.describe(req)
        set_text(self._cmd_lbl, f"Command: {desc}")
        if self.armed:
            set_text(self._go, f"CONFIRM: {desc} - click again")
            set_style(self._go, _GO_ARMED)
        else:
            set_text(self._go, "GO  (click twice)")
            set_style(self._go, _GO_IDLE)
        msg = self._feedback if ok or not reason else f"GO disabled: {reason}" + (
            f"\n{self._feedback}" if self._feedback else "")
        set_text(self._reason, msg)
