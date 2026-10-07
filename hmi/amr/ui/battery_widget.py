"""
battery_widget.py  –  Battery status and charging overview.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPen, QBrush
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QSizePolicy,
)

from .widgets import set_style, set_text


class _SocGauge(QWidget):
    """Vertical bar gauge showing battery SOC percentage."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._soc = 0.0
        self._charging = False
        self.setMinimumSize(80, 200)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

    def set_soc(self, pct: float, charging: bool = False) -> None:
        soc = max(0.0, min(100.0, float(pct)))
        if soc == self._soc and charging == self._charging:
            return
        self._soc = soc
        self._charging = charging
        self.update()

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        margin = 10
        tip_h = 14
        body_h = h - margin * 2 - tip_h

        # Battery tip (positive terminal)
        tip_w = 20
        tip_rect = QRectF((w - tip_w) / 2, margin, tip_w, tip_h)
        p.setBrush(QBrush(QColor("#888888")))
        p.setPen(QPen(QColor("#AAAAAA"), 1))
        p.drawRoundedRect(tip_rect, 3, 3)

        # Battery body border
        body_rect = QRectF(margin, margin + tip_h, w - margin * 2, body_h)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor("#888888"), 2))
        p.drawRoundedRect(body_rect, 4, 4)

        # Fill level
        fill_h = (self._soc / 100.0) * (body_h - 4)
        fill_rect = QRectF(
            body_rect.left() + 2,
            body_rect.bottom() - 2 - fill_h,
            body_rect.width() - 4,
            fill_h,
        )

        if self._soc > 50:
            fill_color = QColor("#44CC44")
        elif self._soc > 20:
            fill_color = QColor("#FFAA00")
        else:
            fill_color = QColor("#FF3333")

        grad = QLinearGradient(fill_rect.topLeft(), fill_rect.bottomLeft())
        grad.setColorAt(0.0, fill_color.lighter(130))
        grad.setColorAt(1.0, fill_color)
        p.setBrush(QBrush(grad))
        p.setPen(Qt.NoPen)
        p.drawRect(fill_rect)

        # SOC text
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)
        p.setFont(font)
        p.setPen(QColor("#FFFFFF"))
        p.drawText(body_rect, Qt.AlignCenter, f"{self._soc:.1f}%")

        # Charging indicator text
        if self._charging:
            p.setPen(QColor("#FFFF44"))
            font.setPointSize(9)
            p.setFont(font)
            charge_rect = body_rect.adjusted(0, body_rect.height() * 0.55, 0, 0)
            p.drawText(charge_rect, Qt.AlignCenter, "CHG")


def _value_label() -> QLabel:
    lbl = QLabel("—")
    lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    font = QFont("Courier", 14)
    font.setBold(True)
    lbl.setFont(font)
    lbl.setStyleSheet("color: #44FF88;")
    return lbl


def _unit_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet("color: #AAAAAA;")
    return lbl


def _status_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setFixedHeight(30)
    lbl.setStyleSheet("background:#333333; color:#CCCCCC; border-radius:4px;")
    return lbl


def _set_status(lbl: QLabel, active: bool, on_color: str, off_color: str = "#333333") -> None:
    color = on_color if active else off_color
    set_style(lbl, f"background:{color}; color:#FFFFFF; border-radius:4px; font-weight:bold;")


class BatteryWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(24)

        # Gauge
        self._gauge = _SocGauge()
        root.addWidget(self._gauge, 0, Qt.AlignTop | Qt.AlignHCenter)

        # Data column
        data_col = QVBoxLayout()

        # Main metrics
        metrics_box = QGroupBox("Battery Metrics")
        metrics_grid = QGridLayout(metrics_box)
        metrics_grid.setHorizontalSpacing(16)
        metrics_grid.setVerticalSpacing(8)

        rows = [
            ("State of Charge", "_lbl_soc",     "%"),
            ("Voltage",         "_lbl_voltage",  "V"),
            ("Current",         "_lbl_current",  "A"),
            ("Power",           "_lbl_power",    "W"),
        ]
        for i, (caption, attr, unit) in enumerate(rows):
            metrics_grid.addWidget(QLabel(caption + ":"), i, 0)
            lbl = _value_label()
            setattr(self, attr, lbl)
            metrics_grid.addWidget(lbl, i, 1)
            metrics_grid.addWidget(_unit_label(unit), i, 2)

        data_col.addWidget(metrics_box)

        # Status indicators
        status_box = QGroupBox("Status")
        status_lay = QHBoxLayout(status_box)

        self._lbl_charging  = _status_label("Charging")
        self._lbl_connected = _status_label("Connected")
        self._lbl_low_bat   = _status_label("Low Battery")

        status_lay.addWidget(self._lbl_charging)
        status_lay.addWidget(self._lbl_connected)
        status_lay.addWidget(self._lbl_low_bat)

        data_col.addWidget(status_box)
        data_col.addStretch()

        root.addLayout(data_col, 1)

        self.setStyleSheet("""
            QGroupBox { border:1px solid #555; border-radius:6px; margin-top:6px;
                        font-weight:bold; color:#CCCCCC; }
            QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; }
            QLabel { color: #CCCCCC; }
        """)

    def update_status(self, data: Dict[str, Any]) -> None:
        soc     = data.get("fBattSOC_pct",   0.0)
        voltage = data.get("fBattVoltage_V", 0.0)
        current = data.get("fBattCurrent_A", 0.0)
        charging   = data.get("bBattCharging",  False)
        connected  = data.get("bBattConnected", False)

        self._gauge.set_soc(soc, charging)

        set_text(self._lbl_soc, f"{soc:.1f}")
        set_text(self._lbl_voltage, f"{voltage:.2f}")
        set_text(self._lbl_current, f"{current:+.2f}")
        set_text(self._lbl_power, f"{voltage * current:+.1f}")

        _set_status(self._lbl_charging,  charging,  "#44AAFF")
        _set_status(self._lbl_connected, connected, "#44CC44")

        low = soc < 20.0 and connected
        _set_status(self._lbl_low_bat, low, "#FF3333")
        set_text(self._lbl_low_bat, "Low Battery" if low else "Battery OK")
