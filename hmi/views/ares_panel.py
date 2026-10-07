"""Wall pose - STUB created by the CORE step, replaced by the STATUS step (docs/HMI_DESIGN.md section 11.2):
plan view with the live ARES pose, estimate vs SIM truth, odometry, the camera fits per stop.
The constructor is fixed: AresPanel(ctx, parent=None)."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class AresPanel(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        self.label = QLabel("Wall pose: implemented by the STATUS step")
        self.label.setStyleSheet("color:#777777;")
        self.label.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.label)
