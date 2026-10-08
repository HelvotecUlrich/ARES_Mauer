"""Run log tab (docs/HMI_DESIGN.md section 10.5): one line per run-log record of the current run, coloured by
severity, filtered by category; the log folder (run.jsonl, hmi_summary.json, images) opens in the file browser."""
from __future__ import annotations

from PySide6.QtCore import Qt, QUrl
from PySide6.QtGui import QDesktopServices, QTextCharFormat, QColor
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPlainTextEdit, QVBoxLayout, QWidget

from ..amr.ui.widgets import PulseBtn, set_text
from ..core.run_controller import RUNNING, WINDOW_LOCK
from ..core.snapshot import CATEGORIES, category, format_event, severity

COLOURS = {"info": "#CCCCCC", "warning": "#FFAA00", "error": "#FF5555"}


class LogView(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._records: list[dict] = []
        self._max = int(ctx.hmi.get("log_view_max_lines", 5000))
        lay = QVBoxLayout(self)
        row = QHBoxLayout()
        self.filters: dict[str, QCheckBox] = {}
        for c in CATEGORIES:
            cb = QCheckBox(c)
            cb.setChecked(True)
            cb.setFocusPolicy(Qt.NoFocus)
            cb.toggled.connect(lambda _on: self._refill())
            self.filters[c] = cb
            row.addWidget(cb)
        row.addStretch()
        self._path = QLabel("log: -")
        self._path.setStyleSheet("color:#999999;")
        self._path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        row.addWidget(self._path)
        self.open_btn = PulseBtn("Open log folder", "#2A4A6A", min_w=130)
        self.open_btn.clicked.connect(self.open_folder)
        row.addWidget(self.open_btn)
        lay.addLayout(row)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(self._max)
        self.text.setFocusPolicy(Qt.NoFocus)
        self.text.setStyleSheet("QPlainTextEdit{background:#121212; font-family: monospace;}")
        lay.addWidget(self.text, 1)
        self._ctl.event.connect(self.add)
        self._ctl.rig_changed.connect(self._on_rig)
        self._ctl.snapshot_changed.connect(self._on_snapshot)
        self._ctl.state_changed.connect(self._on_state)

    def _shown(self, rec: dict) -> bool:
        return self.filters[category(rec)].isChecked()

    def _append(self, rec: dict) -> None:
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(COLOURS[severity(rec)]))
        cur = self.text.textCursor()
        cur.movePosition(cur.MoveOperation.End)
        if not self.text.document().isEmpty():
            cur.insertBlock()
        cur.insertText(format_event(rec), fmt)

    def add(self, rec: dict) -> None:
        self._records.append(rec)
        if len(self._records) > self._max:
            del self._records[: len(self._records) - self._max]
        if self._shown(rec):
            self._append(rec)
            self.text.verticalScrollBar().setValue(self.text.verticalScrollBar().maximum())

    def _refill(self) -> None:
        self.text.clear()
        for rec in self._records:
            if self._shown(rec):
                self._append(rec)

    def lines(self) -> list[str]:
        return self.text.toPlainText().splitlines()

    def _on_rig(self, rig) -> None:
        if rig is not None:                      # a new rig = a new run log
            self._records.clear()
            self.text.clear()

    def _on_snapshot(self, snap) -> None:
        if snap is not None and snap.log_path:
            set_text(self._path, f"log: {snap.log_path}")

    def _on_state(self, state: str, _detail: str) -> None:
        run = state in RUNNING                   # Explorer in front would take Space / Esc (HALT)
        self.open_btn.setEnabled(not run)
        self.open_btn.setToolTip(WINDOW_LOCK if run else "")

    def open_folder(self) -> None:
        folder = self._ctl.log_dir
        if folder is not None:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))
