"""
jog_panel.py - Press-and-hold jog in PHYSICAL directions (spec 6.3, 5.4).

The physical direction (Forward/Back/Left/Right/CCW/CW) is mapped to the PLC jog bit via frame.jog_field()
(config frame.plus_y_is_left / plus_omega_is_ccw). Every change sends the complete jog state (all six bits) in
one write, so a lost release is repaired by the next write. Keyboard: W/S/A/D/Q/E (gated by MainWindow +
ControlWidget: window active, Control tab, Jog sub-tab).

While a relative move runs (bMoveActive) the jog parameters (incl. fAccel_mms, the traction ramp that the PLC
forces to the move acceleration anyway) are NOT written: the spin boxes are disabled and a parameter write that
falls into the move (MANUAL entry/exit) is deferred; the latest values are sent once the move has ended.

Precise Mode (review N3): PLC v2 reports bPreciseModeActive = bCmdPreciseMode OR bMoveActive (FB_HMI_Interface), so
during a move the display shows the precise mode FORCED by the move, not the operator's own setting. The latch
therefore toggles the own setting from the value this HMI last wrote (self._precise_cmd), never from the display;
the button is disabled while bMoveActive and shows "[ON, forced by move]". While no move runs the display equals
bCmdPreciseMode and is adopted as the own setting (covers a PLC restart or a write that was not sent).
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Set, Tuple

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from .. import frame as F
from ..logic import jog_values
from .widgets import LatchBtn, lbl, set_text

# keyboard -> physical direction
KEY_DIRECTIONS: Dict[int, str] = {
    int(Qt.Key_W): F.FORWARD, int(Qt.Key_S): F.BACK,
    int(Qt.Key_A): F.LEFT, int(Qt.Key_D): F.RIGHT,
    int(Qt.Key_Q): F.CCW, int(Qt.Key_E): F.CW,
}
_KEY_NAME = {F.FORWARD: "W", F.BACK: "S", F.LEFT: "A", F.RIGHT: "D", F.CCW: "Q", F.CW: "E"}


PRECISE_TIP = "Traction held at zero until both wheels reached the target angle."
FORCED_NOTE = "forced by move"


class _JogButton(QPushButton):
    def __init__(self, direction: str, field: str, panel: "JogPanel") -> None:
        super().__init__(f"{F.LABELS[direction]}\n({_KEY_NAME[direction]})")
        self.direction = direction
        self.field = field
        self.setMinimumSize(78, 62)
        self.setFont(QFont("", 11))
        self.setFocusPolicy(Qt.NoFocus)
        self.setToolTip(f"PLC bit {field} (mapping from config 'frame')")
        self.setStyleSheet(
            "QPushButton{background:#1A2A3A;color:#AACCEE;border-radius:5px;font-weight:bold;"
            "border:1px solid #2A4A6A;}"
            "QPushButton:pressed{background:#1A5A9A;color:#FFFFFF;border:1px solid #4A8ACA;}"
            "QPushButton:disabled{background:#141414;color:#383838;border:1px solid #252525;}")
        self.pressed.connect(lambda: panel.press(self.field))
        self.released.connect(lambda: panel.release(self.field))


def _spin(lo: float, hi: float, step: float, value: float, suffix: str, decimals: int = 0) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    s.setValue(value)
    s.setSuffix(suffix)
    s.setMinimumWidth(110)
    s.setKeyboardTracking(False)   # send on Enter / arrows, not on every typed digit
    return s


class JogPanel(QWidget):
    def __init__(self, cfg: Dict[str, Any], frame_cfg: F.FrameConfig, worker: Any,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._worker = worker
        self._frame = frame_cfg
        self._active: Set[str] = set()
        self._enabled = False
        self._manual = False            # parameters are pushed on change only while in MANUAL
        self._move_active = False       # relative move owns the setpoints -> no parameter writes
        self._pending: Dict[str, float] = {}
        self._precise_cmd = False       # own setting bCmdPreciseMode as last written (re-synced while no move runs)
        hmi = cfg.get("hmi", {}) or {}
        self._accel_outside = float(hmi.get("accel_outside_manual_mms2", 200.0))

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        box = QGroupBox("Jog  -  MANUAL only  -  press and hold (physical directions)")
        grid = QGridLayout(box)
        grid.setSpacing(6)
        self._buttons: Dict[str, _JogButton] = {}
        pos = {F.FORWARD: (0, 1), F.LEFT: (1, 0), F.RIGHT: (1, 2), F.BACK: (2, 1),
               F.CCW: (0, 4), F.CW: (2, 4)}
        for d, (r, c) in pos.items():
            b = _JogButton(d, F.jog_field(d, frame_cfg), self)
            self._buttons[d] = b
            grid.addWidget(b, r, c)
        grid.addWidget(lbl("Rotate:", "#777777"), 1, 4, alignment=Qt.AlignCenter)
        grid.setColumnMinimumWidth(3, 20)
        root.addWidget(box)

        self._map_lbl = QLabel(self._mapping_text())
        self._map_lbl.setWordWrap(True)
        self._map_lbl.setStyleSheet("color:#888888; font-size:11px;")
        root.addWidget(self._map_lbl)

        par = QGroupBox("Jog parameters (sent on change and when MANUAL is entered)")
        pg = QGridLayout(par)
        self._spd = _spin(10, 800, 10, float(hmi.get("jog_speed_mms", 200.0)), " mm/s")
        self._rot = _spin(1, 90, 5, float(hmi.get("jog_rot_speed_degs", 20.0)), " deg/s")
        self._lim = _spin(0, 1500, 50, float(hmi.get("default_speed_limit_mms", 500.0)), " mm/s")
        self._acc = _spin(20, 3000, 50, float(hmi.get("jog_accel_mms2", 1000.0)), " mm/s2")
        self._lim.setToolTip("fSpeedLimit_mms: cap of the jog XY speed (0 = no limit)")
        self._acc.setToolTip("fAccel_mms: traction ramp while in MANUAL (also caps the relative move accel).\n"
                             f"Outside MANUAL the HMI restores {self._accel_outside:.0f} mm/s2 "
                             "(config hmi.accel_outside_manual_mms2).\n"
                             "Locked while a relative move runs (sent after the move).")
        self._param_spins: Tuple[QDoubleSpinBox, ...] = (self._spd, self._rot, self._lim, self._acc)
        pg.addWidget(lbl("Jog speed:"), 0, 0)
        pg.addWidget(self._spd, 0, 1)
        pg.addWidget(lbl("Rot speed:"), 0, 2)
        pg.addWidget(self._rot, 0, 3)
        pg.addWidget(lbl("Speed limit:"), 1, 0)
        pg.addWidget(self._lim, 1, 1)
        pg.addWidget(lbl("Accel (MANUAL):"), 1, 2)
        pg.addWidget(self._acc, 1, 3)
        for s in self._param_spins:
            s.valueChanged.connect(self._on_param_changed)
        self._param_lock = lbl("", "#FFAA00")
        self._param_lock.setWordWrap(True)
        self._param_lock.hide()
        pg.addWidget(self._param_lock, 2, 0, 1, 4)
        root.addWidget(par)

        opt = QHBoxLayout()
        self._btn_precise = LatchBtn("Precise Mode", amber=True, min_w=170)
        self._btn_precise.setToolTip(PRECISE_TIP)
        self._btn_precise.clicked.connect(self._toggle_precise)
        self._hold_lbl = lbl("", "#FFAA00", bold=True)
        self._btn_horn = QPushButton("Horn (hold)")
        self._btn_horn.setFocusPolicy(Qt.NoFocus)
        self._btn_horn.setMinimumHeight(34)
        self._btn_horn.setMinimumWidth(120)
        self._btn_horn.setStyleSheet(
            "QPushButton{background:#444400;color:#EEEE44;border-radius:4px;font-weight:bold;"
            "border:1px solid #666600;}QPushButton:pressed{background:#888800;color:#FFFFFF;}")
        self._btn_horn.pressed.connect(lambda: self._worker.send({"bCmdHorn": True}, "horn on"))
        self._btn_horn.released.connect(lambda: self._worker.send({"bCmdHorn": False}, "horn off"))
        self._btn_show = LatchBtn("Show Mode", min_w=140)
        self._btn_show.clicked.connect(self._toggle_show)
        opt.addWidget(self._btn_precise)
        opt.addWidget(self._hold_lbl)
        opt.addStretch()
        opt.addWidget(self._btn_horn)
        opt.addWidget(self._btn_show)
        root.addLayout(opt)
        root.addStretch()
        self._enabled = True            # force the initial disable below (guard compares with the old value)
        self.set_jog_enabled(False)

    # ── jog state ─────────────────────────────────────────────────────────────
    @property
    def active_fields(self) -> Set[str]:
        return set(self._active)

    def press(self, field: str) -> None:
        if not self._enabled or field in self._active:
            return
        self._active.add(field)
        self._worker.send(jog_values(self._active), "jog")

    def release(self, field: str) -> None:
        if field not in self._active:
            return
        self._active.discard(field)
        self._worker.send(jog_values(self._active), "jog")

    def release_all(self, send: bool = True) -> None:
        """Release every pressed jog bit (focus loss, tab change, disconnect, state change)."""
        had = bool(self._active)
        self._active.clear()
        for b in self._buttons.values():
            if b.isDown():
                b.setDown(False)
        if had and send:
            self._worker.send(jog_values(set()), "jog release")

    def key_event(self, key: int, pressed: bool) -> bool:
        """Keyboard jog. Returns True if the key is a jog key (event consumed)."""
        direction = KEY_DIRECTIONS.get(int(key))
        if direction is None:
            return False
        field = F.jog_field(direction, self._frame)
        if pressed:
            self.press(field)
        else:
            self.release(field)
        return True

    def set_jog_enabled(self, enabled: bool) -> None:
        if enabled == self._enabled:
            return
        self._enabled = enabled
        for b in self._buttons.values():
            b.setEnabled(enabled)
        if not enabled:
            self.release_all()

    # ── parameters / options ──────────────────────────────────────────────────
    def _send_params(self, values: Dict[str, float], label: str) -> None:
        """Write jog parameters now, or defer them (latest value wins) while a relative move runs."""
        if self._move_active:
            self._pending.update(values)
            return
        self._worker.send(dict(values), label)

    def push_params(self) -> None:
        """Jog speed, rotation speed, speed limit and MANUAL accel in one write (deferred during a move)."""
        self._send_params({
            "fJogSpeed_mms": float(self._spd.value()),
            "fJogRotSpeed_degs": float(self._rot.value()),
            "fSpeedLimit_mms": float(self._lim.value()),
            "fAccel_mms": float(self._acc.value()),
        }, "jog parameters")

    def _on_param_changed(self, _value: float) -> None:
        if self._manual:
            self.push_params()

    def set_manual(self, manual: bool) -> None:
        """Called on MANUAL entry/exit: entry pushes the jog parameters, exit restores the accel."""
        if manual == self._manual:
            return
        self._manual = manual
        if manual:
            self.push_params()
        else:
            self.release_all()
            self.restore_outside_accel()

    def restore_outside_accel(self) -> None:
        self._send_params({"fAccel_mms": self._accel_outside}, "accel outside MANUAL")

    def set_move_active(self, active: bool) -> None:
        """bMoveActive from the PLC: lock the parameter spin boxes and the Precise Mode latch (shown as forced by
        the move); after the move send deferred parameters."""
        active = bool(active)
        if active == self._move_active:
            return
        self._move_active = active
        for s in self._param_spins:
            s.setEnabled(not active)
        set_text(self._param_lock, "Locked while a relative move runs (the PLC forces the move ramp); "
                                   "changes are sent after the move." if active else "")
        self._param_lock.setVisible(active)
        # Precise Mode: the PLC forces it during the move -> lock the latch, mark the display as forced
        self._btn_precise.setEnabled(not active)
        self._btn_precise.set_note(FORCED_NOTE if active else "")
        self._update_precise_tip()
        if not active and self._pending:
            vals, self._pending = self._pending, {}
            self._worker.send(vals, "jog parameters (after move)")

    @property
    def pending_params(self) -> Dict[str, float]:
        return dict(self._pending)

    @property
    def manual_accel(self) -> float:
        return float(self._acc.value())

    @property
    def precise_setting(self) -> bool:
        """Own Precise Mode setting (bCmdPreciseMode) as last written / adopted - not the forced display."""
        return self._precise_cmd

    def _toggle_precise(self) -> None:
        """Toggle the OWN setting from the value last written, never from the display (which includes the precise
        mode forced by a relative move). The button is disabled during a move; should a click still arrive, it
        toggles the own setting and leaves the forced display alone."""
        new = not self._precise_cmd
        self._precise_cmd = new
        self._worker.send({"bCmdPreciseMode": new}, "precise mode")
        if not self._move_active:
            self._btn_precise.set_local(new)
        self._update_precise_tip()

    def _update_precise_tip(self) -> None:
        if self._move_active:
            own = "ON" if self._precise_cmd else "OFF"
            self._btn_precise.setToolTip(f"{PRECISE_TIP}\nLocked while a relative move runs: the PLC forces "
                                         f"precise mode during the move.\nOwn setting: {own} (applies again "
                                         "after the move).")
        else:
            self._btn_precise.setToolTip(PRECISE_TIP)

    def _toggle_show(self) -> None:
        new = not self._btn_show.is_active()
        self._worker.send({"bCmdShowMode": new}, "show mode")
        self._btn_show.set_local(new)

    def update_status(self, data: Dict[str, Any]) -> None:
        shown = bool(data.get("bPreciseModeActive", False))
        if not data.get("bMoveActive", False) and not self._btn_precise.holding() and shown != self._precise_cmd:
            # no move in this sample -> bPreciseModeActive == bCmdPreciseMode (legacy: no forced mode at all)
            self._precise_cmd = shown
        self._btn_precise.sync(shown)
        self._btn_show.sync(data.get("bShowModeActive", False))
        set_text(self._hold_lbl, "Waiting for wheels..." if data.get("bPreciseModeHolding", False) else "")

    def _mapping_text(self) -> str:
        f = self._frame
        left = F.jog_field(F.LEFT, f)
        ccw = F.jog_field(F.CCW, f)
        return (f"Direction mapping (config, {f.status_word} on the robot): physical Left -> {left} "
                f"({'+' if f.plus_y_is_left else '-'}Y), CCW -> {ccw} "
                f"({'+' if f.plus_omega_is_ccw else '-'}omega). {F.plus_y_text(f)}, {F.plus_omega_text(f)}.")
