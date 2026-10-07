"""Status strip - STUB created by the CORE step, replaced by the STATUS step (docs/HMI_DESIGN.md section 11.2):
run state | mode | stop | stone | action | UR mode | ARES state | twin state. The constructor is fixed: StatusStrip(ctx, parent=None)."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class StatusStrip(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        self.label = QLabel("Status strip: implemented by the STATUS step")
        self.label.setStyleSheet("color:#777777;")
        self.label.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.label)
