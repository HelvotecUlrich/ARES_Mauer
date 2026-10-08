"""ConfirmBar: the step-mode confirmation, inline above the tabs on every tab (docs/HMI_DESIGN.md D-H6, 10.4).

Not a modal dialog: a dialog would block the HALT button and the Space/Esc HALT keys. Go / Refilled are enabled only
[hmi] confirm_arm_s after the request appears (a double click must not carry over to the next step); the buttons
take no keyboard focus and have no shortcuts (Space / Esc are HALT).
"""
from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from ..amr.ui.widgets import PulseBtn, set_text

HELD_NOTE = "Declining leaves the stone in the jaws - resume then needs 'Jaws empty'"
PENDING_NOTE = "Pause / abort after this place"


class ConfirmBar(QFrame):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._arm_s = float(ctx.hmi.get("confirm_arm_s", 0.5))
        self._req = None
        self.setStyleSheet("ConfirmBar{background:#3A3000;border:2px solid #FFCC33;border-radius:6px;}"
                           "QLabel{color:#FFEEAA;background:transparent;}")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        col = QVBoxLayout()
        self._kind = QLabel("")
        self._text = QLabel("")
        f = QFont()
        f.setPointSize(14)
        f.setBold(True)
        self._text.setFont(f)
        self._text.setWordWrap(True)
        self._note = QLabel("")
        self._note.setStyleSheet("color:#FFAA00; font-weight:bold;")
        self._note.setWordWrap(True)
        for w in (self._kind, self._text, self._note):
            col.addWidget(w)
        lay.addLayout(col, 1)
        self.go = PulseBtn("Go", "#0A6E30", min_w=130, min_h=44)
        self.decline = PulseBtn("Decline", "#7A1515", min_w=130, min_h=44)
        self.go.clicked.connect(lambda: self._answer(True))
        self.decline.clicked.connect(lambda: self._answer(False))
        lay.addWidget(self.go)
        lay.addWidget(self.decline)
        self._arm = QTimer(self)
        self._arm.setSingleShot(True)
        self._arm.timeout.connect(self._armed)
        self._ctl.confirm_requested.connect(self.show_request)
        self._ctl.confirm_cleared.connect(self._cleared)
        self._ctl.state_changed.connect(self._on_state)
        self.hide()

    @property
    def request(self):
        return self._req

    def show_request(self, req) -> None:
        self._req = req
        kind, go, decline = {"station_empty": ("Pick-up station", "Refilled", "Abort run"),
                             "start": ("REAL start - checklist:", "Checked - start", "Not ready")}.get(
            req.kind, ("Step mode - next motion:", "Go", "Decline"))
        set_text(self._kind, kind)
        set_text(self._text, req.text)
        self.go.setText(go)
        self.decline.setText(decline)
        self._update_note()
        self.go.setEnabled(False)
        self.decline.setEnabled(True)
        self._arm.start(int(self._arm_s * 1000))
        self.show()

    def _update_note(self) -> None:
        req = self._req
        if req is None:
            return
        notes = []
        state = self._ctl.state
        if req.holding:
            notes.append(HELD_NOTE)
        if req.pause_pending or (req.holding and state in ("pausing", "aborting")):
            notes.append(PENDING_NOTE)
        set_text(self._note, " | ".join(notes))
        self._note.setVisible(bool(notes))

    def _on_state(self, _state: str, _detail: str) -> None:
        self._update_note()

    def _armed(self) -> None:
        if self._req is not None:
            self.go.setEnabled(True)

    def _answer(self, ok: bool) -> None:
        req = self._req
        if req is None:
            return
        self.go.setEnabled(False)
        self.decline.setEnabled(False)
        self._ctl.answer_confirm(req.id, ok)

    def _cleared(self, req_id: int) -> None:
        if self._req is not None and self._req.id == req_id:
            self._req = None
            self._arm.stop()
            self.hide()
