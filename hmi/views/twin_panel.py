"""Twin tab (docs/HMI_DESIGN.md section 11.3): the RoboDK digital twin switch, its status (state, API port, rate,
render time per changed tick, lag, stones on screen) and Restart / Save view. The twin itself is
hmi/core/twin_link.TwinLink + robodk/twin.py; it restarts with every new job session (job and config variant stay
consistent), --twin switches it on with the first session, closing the HMI stops it and closes its RoboDK.
"""
from __future__ import annotations

import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from mauer import REPO

from ..amr.ui import constants as C
from ..amr.ui.widgets import PulseBtn, set_style, set_text
from ..core.twin_link import TwinLink

STATE_COLOURS = {"running": C.COL_OK, "starting": C.COL_WARN, "lost": C.COL_BAD}
NOTE = ("An own RoboDK instance on an API port >= 20630 (never your RoboDK on 20500/20501), no collision checks, no "
        "cameras. Passive: it mirrors the run and never controls it - the run never waits for it.\n"
        "SIM: UR joints and ARES at the simulated true pose. REAL: twin = sequencer belief - UR joints from RTDE, "
        "ARES at the estimated pose (+ odometry during a move), stones at their nominal wall pose.")


class TwinPanel(QWidget):
    _snapshot_done = Signal(bool, str)              # from the twin thread

    def __init__(self, ctx, parent=None, *, twin_factory=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self.link = TwinLink(ctx, self, twin_factory=twin_factory)
        lay = QVBoxLayout(self)
        box = QGroupBox("RoboDK digital twin")
        bl = QVBoxLayout(box)
        row = QHBoxLayout()
        self.enable = QCheckBox("RoboDK twin")
        self.enable.setFocusPolicy(Qt.NoFocus)
        self.enable.toggled.connect(self._on_toggle)
        row.addWidget(self.enable)
        self.restart_btn = PulseBtn("Restart", "#2A4A6A", min_w=110)
        self.restart_btn.clicked.connect(self.restart)
        row.addWidget(self.restart_btn)
        self.save_btn = PulseBtn("Save view PNG", "#2A4A6A", min_w=130)
        self.save_btn.clicked.connect(self.save_view)
        row.addWidget(self.save_btn)
        row.addStretch()
        bl.addLayout(row)
        grid = QGridLayout()
        self.values: dict[str, QLabel] = {}
        for i, (key, name) in enumerate((("state", "State"), ("port", "RoboDK API port"), ("rate", "Rate"),
                                         ("tick", "Render per changed tick"), ("lag", "Lag (data -> rendered)"),
                                         ("stones", "Stones on screen"), ("error", "Last error"))):
            grid.addWidget(QLabel(name), i, 0)
            v = QLabel("-")
            v.setTextInteractionFlags(Qt.TextSelectableByMouse)
            v.setWordWrap(True)
            self.values[key] = v
            grid.addWidget(v, i, 1)
        grid.setColumnStretch(1, 1)
        bl.addLayout(grid)
        self.message = QLabel("")
        self.message.setWordWrap(True)
        bl.addWidget(self.message)
        lay.addWidget(box)
        note = QLabel(NOTE)
        note.setWordWrap(True)
        note.setStyleSheet("color:#888888;")
        lay.addWidget(note)
        lay.addStretch()

        self.link.status.connect(self._on_status)
        self._snapshot_done.connect(self._on_snapshot)
        ctx.session_changed.connect(self._on_session)
        ctx.add_shutdown_hook(self.link.shutdown)
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()
        self._on_status("off", "")
        if getattr(ctx, "start_twin", False):      # --twin: on with the first session
            self.enable.setChecked(True)

    # ── actions ──────────────────────────────────────────────────────────────
    def _on_toggle(self, on: bool) -> None:
        if on:
            if self._ctx.session is None:
                self._on_status("off", "starts when a job is loaded")
            else:
                self.link.ensure()
        else:
            self.link.stop()
        self.refresh()

    def _on_session(self, _session) -> None:
        if self.enable.isChecked():
            self.link.ensure()                      # the twin is built for one job and config variant

    def restart(self) -> None:
        if not self.enable.isChecked():
            self.enable.setChecked(True)            # starts it
        else:
            self.link.start()

    def save_view(self) -> None:
        stamp = time.strftime("%Y-%m-%d_%H%M%S")
        log_dir = getattr(self._ctx.controller, "log_dir", None)
        folder = Path(log_dir) if log_dir else REPO / "results"
        path = folder / f"twin_{stamp}.png"
        if not self.link.request_snapshot(path, lambda ok, p: self._snapshot_done.emit(ok, p)):
            set_text(self.message, "Save view: the twin is not running")

    # ── display ──────────────────────────────────────────────────────────────
    def _on_snapshot(self, ok: bool, path: str) -> None:
        set_text(self.message, f"view saved: {path}" if ok else f"Save view FAILED ({path})")

    def _on_status(self, state: str, detail: str) -> None:
        set_text(self.values["state"], state + (f" - {detail}" if detail else ""))
        set_style(self.values["state"], f"color:{STATE_COLOURS.get(state, C.COL_TEXT)}; font-weight:bold;")
        self._enables(state)

    def _enables(self, state: str | None = None) -> None:
        self.restart_btn.setEnabled(self._ctx.session is not None)
        self.save_btn.setEnabled(self.link.running and (state or self._ctx.twin_state[0]) == "running")

    def refresh(self) -> None:
        """1 Hz: the twin's numbers (attributes its thread updates)."""
        d = self.link.stats() or {}
        v = self.values
        set_text(v["port"], str(d["port"]) if d.get("port") else "-")
        if d.get("state") == "running":
            set_text(v["rate"], f"{d.get('rate_hz', 0.0):.1f} Hz (set {d.get('rate_set_hz') or 0:.0f} Hz), "
                                f"{d.get('renders', 0)} renders in {d.get('ticks', 0)} ticks")
            set_text(v["tick"], f"{d.get('tick_ms', 0.0):.1f} ms (median)")
            set_text(v["lag"], f"{d.get('lag_ms', 0.0):.0f} ms (median) + up to one tick of polling")
        else:
            for k in ("rate", "tick", "lag"):
                set_text(v[k], "-")
        st = d.get("stones")
        set_text(v["stones"], "-" if not st else (f"magazine {st['mag']}, station {st['station']}, wall "
                                                   f"{st['wall']}, jaws {st['tool']}"))
        err = d.get("last_error")
        if d.get("frame_errors"):
            err = f"{err} ({d['frame_errors']} frame errors)"
        set_text(v["error"], err or "-")
        self._enables()
