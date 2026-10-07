"""Camera view - STUB created by the CORE step, replaced by the CAMERA step (docs/HMI_DESIGN.md section 11.1):
the run's camera image with the detected ChArUco corners, the last measurement, Grab / Live (REAL). The constructor is fixed: CameraView(ctx, parent=None)."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class CameraView(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        self.label = QLabel("Camera view: implemented by the CAMERA step")
        self.label.setStyleSheet("color:#777777;")
        self.label.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.label)
