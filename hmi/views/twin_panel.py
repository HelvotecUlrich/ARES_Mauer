"""RoboDK twin - STUB created by the CORE step, replaced by the TWIN step (docs/HMI_DESIGN.md section 11.3):
RoboDK digital twin toggle and status. The constructor is fixed: TwinPanel(ctx, parent=None)."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class TwinPanel(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        self.label = QLabel("RoboDK twin: implemented by the TWIN step")
        self.label.setStyleSheet("color:#777777;")
        self.label.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.label)
