"""
widgets.py - Shared small widgets and helpers for the HMI tabs.

  PulseBtn  - one-shot command button (the worker resets the bit after 300 ms)
  LatchBtn  - level command with ON/OFF indicator; PLC is the source of truth (hold-off after a click)
  HaltButton- big red HALT button (spec 6.2)
  set_style - setStyleSheet only when the style changed (review r5 2.2)
"""

from __future__ import annotations

import time
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QFrame, QLabel, QPushButton, QWidget

GROUP_CSS = """
    QGroupBox { border: 1px solid #484848; border-radius: 6px; margin-top: 8px;
                font-weight: bold; color: #BBBBBB; padding-top: 4px; }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
    QLabel { color: #AAAAAA; }
    QDoubleSpinBox { background: #1C1C1C; color: #DDDDDD; border: 1px solid #484848;
                     border-radius: 3px; padding: 2px 4px; }
    QDoubleSpinBox:focus { border: 1px solid #5A8ABB; }
    QDoubleSpinBox:disabled { color: #555555; }
"""


def set_style(widget: QWidget, css: str) -> None:
    """setStyleSheet only if css differs from the last value set through this helper."""
    if getattr(widget, "_hmi_css", None) != css:
        widget.setStyleSheet(css)
        widget._hmi_css = css  # type: ignore[attr-defined]


def set_text(label: QLabel, text: str) -> None:
    """setText only on change (cheap guard for 10 Hz updates)."""
    if label.text() != text:
        label.setText(text)


def lbl(text: str, color: str = "#999999", bold: bool = False) -> QLabel:
    w = QLabel(text)
    w.setStyleSheet(f"color:{color};" + ("font-weight:bold;" if bold else ""))
    return w


def num_label(text: str = "-") -> QLabel:
    w = QLabel(text)
    w.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    w.setStyleSheet("color:#CCCCCC; font-family: monospace; background: transparent;")
    return w


def hsep() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setStyleSheet("color:#444;")
    return f


def bool_css(value: bool, true_color: str = "#44CC44", false_color: str = "#444444") -> str:
    color = true_color if value else false_color
    return f"background:{color}; color:#FFFFFF; border-radius:4px; font-weight:bold;"


class PulseBtn(QPushButton):
    """One-shot command button (colour fixed at construction)."""

    _BASE = (
        "QPushButton{{background:{bg};color:#FFFFFF;border-radius:4px;"
        "font-weight:bold;border:1px solid {border};min-height:{h}px;}}"
        "QPushButton:hover{{background:{hover};}}"
        "QPushButton:pressed{{background:{pressed};border:1px solid #FFFFFF;}}"
        "QPushButton:disabled{{background:#252525;color:#484848;border:1px solid #383838;}}"
    )

    def __init__(self, text: str, bg: str, min_w: int = 100, min_h: int = 34,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(text, parent)
        self.setStyleSheet(self._BASE.format(bg=bg, border=bg, hover=bg + "CC", pressed=bg + "77", h=min_h))
        self.setMinimumWidth(min_w)
        self.setFocusPolicy(Qt.NoFocus)   # Space must never click a focused button (Space = HALT)


class LatchBtn(QPushButton):
    """Level command button with ON/OFF display; set_active() from the PLC status."""

    _STYLE_ON = (
        "QPushButton{background:#0D2E0D;color:#33EE33;border-radius:4px;font-weight:bold;"
        "border:2px solid #2A8A2A;min-height:34px;text-align:left;padding-left:8px;}"
        "QPushButton:hover{background:#133A13;}"
        "QPushButton:disabled{background:#1A1A1A;color:#3A5A3A;border:2px solid #2A3A2A;}"
    )
    _STYLE_OFF = (
        "QPushButton{background:#1E1E1E;color:#8A8A8A;border-radius:4px;font-weight:bold;"
        "border:2px solid #3A3A3A;min-height:34px;text-align:left;padding-left:8px;}"
        "QPushButton:hover{background:#282828;}"
        "QPushButton:disabled{background:#181818;color:#404040;border:2px solid #303030;}"
    )
    _STYLE_ON_AMBER = (
        "QPushButton{background:#2A1800;color:#FFAA00;border-radius:4px;font-weight:bold;"
        "border:2px solid #8A5500;min-height:34px;text-align:left;padding-left:8px;}"
        "QPushButton:hover{background:#3A2200;}"
        "QPushButton:disabled{background:#1A1A1A;color:#5A4A2A;border:2px solid #3A3020;}"
    )
    HOLD_S = 0.6

    def __init__(self, label: str, amber: bool = False, min_w: int = 140,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._label = label
        self._amber = amber
        self._active = False
        self._note = ""
        self._hold_until = 0.0
        self.setMinimumWidth(min_w)
        self.setFocusPolicy(Qt.NoFocus)
        self._refresh()

    def is_active(self) -> bool:
        return self._active

    def holding(self) -> bool:
        """True while a local click is in its hold-off window (PLC value not yet trusted)."""
        return time.monotonic() < self._hold_until

    def set_local(self, active: bool) -> None:
        """Set by a local click; PLC sync is suppressed for HOLD_S (ADS round trip)."""
        self._hold_until = time.monotonic() + self.HOLD_S
        self._set(active)

    def sync(self, active: bool) -> None:
        """Set from the PLC status unless a local click is still in its hold-off window."""
        if not self.holding():
            self._set(bool(active))

    def set_note(self, note: str) -> None:
        """Extra text in the state bracket, e.g. 'forced by move' -> '[ON, forced by move]'; '' = none."""
        if note != self._note:
            self._note = note
            self._refresh()

    def _set(self, active: bool) -> None:
        if active != self._active:
            self._active = active
            self._refresh()

    def _refresh(self) -> None:
        dot = "●" if self._active else "○"
        state = "ON " if self._active else "OFF"
        if self._note:
            state = f"{state.strip()}, {self._note}"
        self.setText(f"  {dot}  {self._label}   [{state}]")
        if self._active:
            set_style(self, self._STYLE_ON_AMBER if self._amber else self._STYLE_ON)
        else:
            set_style(self, self._STYLE_OFF)


class HaltButton(QPushButton):
    """Big red HALT button: ramped stop of jog and relative move (not drives off)."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("HALT  (Space / Esc)", parent)
        f = QFont()
        f.setPointSize(16)
        f.setBold(True)
        self.setFont(f)
        self.setMinimumSize(260, 54)
        self.setFocusPolicy(Qt.NoFocus)
        self.setToolTip(
            "Ramped stop: aborts the relative move (bCmdMoveAbort) and releases all jog bits in ONE write.\n"
            "The AMR stays in MANUAL with drives enabled. Not a safety function - the E-stop is.")
        self.setStyleSheet(
            "QPushButton{background:#B00000;color:#FFFFFF;border:3px solid #FF5050;border-radius:8px;}"
            "QPushButton:hover{background:#D00000;}"
            "QPushButton:pressed{background:#700000;}")


class StepBadge(QLabel):
    """Numbered badge of a startup step: pending (grey), current (amber), done (green)."""

    _CSS = {
        "pending": "background:#333333;color:#888888;border-radius:11px;font-weight:bold;",
        "current": "background:#FFAA00;color:#000000;border-radius:11px;font-weight:bold;",
        "done":    "background:#2A8A2A;color:#FFFFFF;border-radius:11px;font-weight:bold;",
    }

    def __init__(self, number: int, parent: Optional[QWidget] = None) -> None:
        super().__init__(str(number), parent)
        self._number = number
        self.setFixedSize(22, 22)
        self.setAlignment(Qt.AlignCenter)
        self.set_mode("pending")

    def set_mode(self, mode: str) -> None:
        set_style(self, self._CSS[mode])
        set_text(self, "✓" if mode == "done" else str(self._number))
