"""Mauer HMI main window (docs/HMI_DESIGN.md section 10.1) - adapted from MA amr_hmi ui/main_window.py (commit
5935c5b): HALT button and note, the step-mode ConfirmBar and the status strip above the tabs, the amr tabs plus the
Mauer tabs, the status bar, key handling and shutdown.

Changes against amr_hmi:
- The window gets an HmiContext and the ADS worker from hmi/main.py and NEVER creates an AdsWorker itself (D-H5):
  without --ares it is a NullAdsWorker that never connects.
- HALT (button, Space, Esc) = the amr worker HALT first, then RunController.halt() (abort flag, pending confirmation
  released, REAL: UR program stopped in a helper thread); it never blocks. Space / Esc also act while an HMI child
  window (the non-native file dialog) is active.
- A Mauer REAL run locks jog / GO while it runs and the odometry reset while it is loaded (_apply_run_lock).
- Closing is refused while a run is active (the heartbeat stops on close: the PLC aborts a move after 500 ms).

Key handling (application event filter):
  Space / Esc      -> HALT (any tab; this window or one of its child windows active)
  W/S/A/D/Q/E      -> jog, only on the ARES control tab AND its Jog sub-tab, this window active
All jog bits are released on application focus loss, tab changes and connection loss.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional

from PySide6.QtCore import QEvent, QObject, Qt, QThread
from PySide6.QtWidgets import QApplication, QHBoxLayout, QLabel, QMainWindow, QTabWidget, QVBoxLayout, QWidget

from .amr.ui import constants as C
from .amr.ui.battery_widget import BatteryWidget
from .amr.ui.control_widget import ControlWidget
from .amr.ui.dashboard_widget import DashboardWidget
from .amr.ui.diagnostics_widget import DiagnosticsWidget
from .amr.ui.jog_panel import KEY_DIRECTIONS
from .amr.ui.widgets import HaltButton, set_style, set_text
from .core.run_controller import BUSY, RUNNING
from .views.ares_panel import AresPanel
from .views.camera_view import CameraView
from .views.confirm_bar import ConfirmBar
from .views.log_view import LogView
from .views.mauer_tab import MauerTab
from .views.status_strip import StatusStrip
from .views.twin_panel import TwinPanel
from .views.ur_panel import UrPanel

log = logging.getLogger(__name__)

HB_FROZEN_S = 1.0     # PLC heartbeat unchanged this long while connected -> warning
_HALT_KEYS = (int(Qt.Key_Space), int(Qt.Key_Escape))
TAB_NAMES = ("Mauer", "Camera", "UR", "ARES control", "Wall pose", "Dashboard", "Diagnostics", "Battery", "Run log",
             "Twin")
RUN_LOCK = "Mauer REAL run active - pause the run first"
RESET_LOCK = "Mauer REAL run loaded - an odometry reset would break the resume check"
CLOSE_REFUSED = "Stop the run first (Pause / Abort / HALT), then close"


class MainWindow(QMainWindow):
    def __init__(self, ctx, worker: Any, worker_thread: Optional[QThread] = None) -> None:
        super().__init__()
        self._ctx = ctx
        self._ctl = ctx.controller
        self._cfg = ctx.amr_cfg
        self._worker = worker
        self._thread = worker_thread
        self._connected = False
        self._if_version = 0
        self._plc_build = ""
        self._last_status: Dict[str, Any] = {}
        self._last_hb: Optional[int] = None
        self._last_hb_change = time.monotonic()
        self._closing = False

        self._setup_ui()
        worker.status.connect(self._on_status)
        worker.connection.connect(self._on_connection)
        worker.interface.connect(self._on_interface)
        worker.command_error.connect(self._on_command_error)
        self._ctl.state_changed.connect(self._on_run_state)
        self._ctl.message.connect(self._on_message)
        self._ctl.session_loaded.connect(lambda _s: self.update_title())

        app = QApplication.instance()
        app.installEventFilter(self)
        app.applicationStateChanged.connect(self._on_app_state)
        self.update_title()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _setup_ui(self) -> None:
        self.resize(1280, 800)
        self.setMinimumSize(1100, 700)
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(6, 6, 6, 0)
        lay.setSpacing(4)
        top = QHBoxLayout()
        self.halt_button = HaltButton()
        self.halt_button.clicked.connect(self.trigger_halt)
        top.addWidget(self.halt_button)
        note = QLabel("HALT = ramped stop of jog / relative move (drives stay on) and stop of the Mauer run (UR "
                      "program stopped). The E-stops on ARES and the UR remain the safety function.")
        note.setWordWrap(True)
        note.setStyleSheet("color:#999999;")
        top.addWidget(note, 1)
        lay.addLayout(top)
        self.confirm_bar = ConfirmBar(self._ctx)
        lay.addWidget(self.confirm_bar)
        self.status_strip = StatusStrip(self._ctx)
        lay.addWidget(self.status_strip)

        self.tabs = QTabWidget()
        self.tabs.setFocusPolicy(Qt.NoFocus)
        self.mauer = MauerTab(self._ctx)
        self.camera = CameraView(self._ctx)
        self.ur = UrPanel(self._ctx)
        self.control = ControlWidget(self._cfg, self._worker)
        self.wall_pose = AresPanel(self._ctx)
        self.dashboard = DashboardWidget()
        self.diagnostics = DiagnosticsWidget()
        self.battery = BatteryWidget()
        self.run_log = LogView(self._ctx)
        self.twin = TwinPanel(self._ctx)
        for w, name in zip((self.mauer, self.camera, self.ur, self.control, self.wall_pose, self.dashboard,
                            self.diagnostics, self.battery, self.run_log, self.twin), TAB_NAMES):
            self.tabs.addTab(w, name)
        self.tabs.currentChanged.connect(self._on_tab_changed)
        lay.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self._lbl_conn = QLabel("Disconnected")
        self._lbl_state = QLabel("State: -")
        self._lbl_build = QLabel("PLC: -")
        self._lbl_if = QLabel("Interface: -")
        self._lbl_hb = QLabel("PLC HB: -")
        self._lbl_mode = QLabel("Mode: -")
        self._lbl_run = QLabel("Run: -")
        sb = self.statusBar()
        for w in (self._lbl_conn, self._lbl_state, self._lbl_build, self._lbl_if, self._lbl_hb, self._lbl_mode,
                  self._lbl_run):
            w.setStyleSheet("padding: 0 8px;")
            sb.addPermanentWidget(w)
        self._set_conn_label(False, "not connected yet")

    def update_title(self) -> None:
        s = self._ctl.session
        mode = (self._ctl.mode or "-").upper() if self._ctl.state not in ("empty", "loaded", "loading") else "-"
        title = (f"Mauer HMI - {s.name if s else 'no job'} - config {s.variant_label if s else 'main'} - {mode}"
                 + ("" if self._ctx.ares_enabled else " - ADS off"))
        self.setWindowTitle(title)

    # ── HALT / keys ───────────────────────────────────────────────────────────
    def trigger_halt(self) -> None:
        """HALT: the worker's ONE write (abort + jog release) first, then the Mauer run (never blocks)."""
        self.control.release_jog(send=False)          # amr: local state, no extra write
        self._worker.halt()                            # amr: FIRST hardware action, unchanged
        self._ctl.halt()                               # Mauer: abort flag, pending confirmation released, REAL: UR
        run = " - Mauer run stopped" if self._ctl.state in RUNNING else ""
        if self._connected:
            self.statusBar().showMessage("HALT sent" + run, 3000)
        else:
            self.statusBar().showMessage("HALT NOT sent: no ADS connection - use the E-stop" + run, 8000)

    def _halt_scope(self) -> bool:
        """This window or one of its child windows (e.g. the non-native file dialog) is the active window."""
        if self.isActiveWindow():
            return True
        w = QApplication.activeWindow()
        while w is not None:
            if w is self:
                return True
            w = w.parentWidget()
        return False

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:  # noqa: N802 - Qt API
        et = event.type()
        if et not in (QEvent.KeyPress, QEvent.KeyRelease) or self._closing:
            return False
        key = int(event.key())
        if key in _HALT_KEYS:
            if not self._halt_scope():
                return False
            if et == QEvent.KeyPress and not event.isAutoRepeat():
                self.trigger_halt()
            return True     # never let Space/Esc reach a widget (no accidental button clicks)
        if not self.isActiveWindow():
            return False
        if key not in KEY_DIRECTIONS or self.tabs.currentWidget() is not self.control \
                or not self.control.jog_tab_active():
            return False
        if not event.isAutoRepeat():
            self.control.handle_jog_key(key, et == QEvent.KeyPress)
        return True

    def _on_app_state(self, state: Qt.ApplicationState) -> None:
        if state != Qt.ApplicationActive:
            self.control.release_jog()

    def _on_tab_changed(self, _index: int) -> None:
        self.control.release_jog()
        if self._last_status:
            self._update_visible_tab(self._last_status)

    # ── Mauer run ─────────────────────────────────────────────────────────────
    def _apply_run_lock(self) -> None:
        c = self._ctl
        real = c.mode == "real"
        self.control.set_run_lock(RUN_LOCK if real and c.state in RUNNING else "")
        self.control.set_reset_lock(RESET_LOCK if real and c.sequencer is not None else "")

    def _on_run_state(self, state: str, detail: str) -> None:
        set_text(self._lbl_run, f"Run: {state}" + (" (HALTED)" if self._ctl.halted else ""))
        set_text(self._lbl_mode, f"Mode: {(self._ctl.mode or '-').upper()}")
        colour = {"error": C.COL_BAD, "aborted": C.COL_WARN, "paused": C.COL_WARN, "running": C.COL_OK}.get(state)
        set_style(self._lbl_run, f"padding: 0 8px;{f' color:{colour};' if colour else ''}")
        self._apply_run_lock()
        self.update_title()

    def _on_message(self, level: str, text: str) -> None:
        log.log({"error": logging.ERROR, "warning": logging.WARNING}.get(level, logging.INFO), "%s", text)
        self.statusBar().showMessage(text, 10000 if level != "info" else 4000)

    # ── worker signals ────────────────────────────────────────────────────────
    def _on_status(self, data: Dict[str, Any]) -> None:
        self._last_status = data
        self.control.update_status(data)       # always (safety-relevant enables)
        self._update_visible_tab(data)
        color, name = C.state_info(data.get("eAmrState", -1))
        set_text(self._lbl_state, f"State: {name}")
        hb = int(data.get("nPlcHeartbeat", 0))
        now = time.monotonic()
        if hb != self._last_hb:
            self._last_hb = hb
            self._last_hb_change = now
        frozen = now - self._last_hb_change > HB_FROZEN_S
        set_text(self._lbl_hb, "PLC HB: FROZEN (PLC stopped?)" if frozen else f"PLC HB: {hb}")
        set_style(self._lbl_hb, f"padding: 0 8px; color:{C.COL_BAD if frozen else '#AAAAAA'};")
        self._ctx.publish_ads_status(data)
        self._apply_run_lock()

    def _update_visible_tab(self, data: Dict[str, Any]) -> None:
        w = self.tabs.currentWidget()
        if w is self.dashboard:
            self.dashboard.update_status(data)
        elif w is self.diagnostics:
            self.diagnostics.update_status(data)
        elif w is self.battery:
            self.battery.update_status(data)

    def _on_connection(self, ok: bool, msg: str) -> None:
        self._connected = ok
        if not ok:
            self.control.release_jog(send=False)
        self.control.set_connection(ok, "" if ok else msg)
        self._set_conn_label(ok, msg)
        self._last_hb_change = time.monotonic()
        self._ctx.publish_ads_connection(ok, msg)
        self._apply_run_lock()

    def _on_interface(self, version: int, build: str) -> None:
        self._if_version = version
        self._plc_build = build
        self.control.set_interface(version, build)
        self.diagnostics.set_interface(version, build)
        set_text(self._lbl_build, f"PLC: {build or '-'}")
        set_text(self._lbl_if, f"Interface: v{version}" + ("" if version >= 2 else " (legacy, no move)"))

    def _on_command_error(self, msg: str) -> None:
        log.warning("command: %s", msg)
        self.statusBar().showMessage(msg, 5000)

    def _set_conn_label(self, ok: bool, msg: str) -> None:
        text = "Connected" if ok else f"Disconnected: {msg}"
        set_text(self._lbl_conn, text if len(text) <= 110 else text[:107] + "...")
        self._lbl_conn.setToolTip("" if ok else msg)
        set_style(self._lbl_conn, f"padding: 0 8px; font-weight:bold; color:{C.COL_OK if ok else C.COL_BAD};")

    # ── shutdown ──────────────────────────────────────────────────────────────
    def closeEvent(self, event: Any) -> None:  # noqa: N802 - Qt API
        if self._closing:                              # closed before: everything is stopped already
            super().closeEvent(event)
            return
        if self._ctl.state in BUSY:
            event.ignore()
            self.statusBar().showMessage(CLOSE_REFUSED, 8000)
            return
        self._closing = True
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)
            try:
                app.applicationStateChanged.disconnect(self._on_app_state)
            except (RuntimeError, TypeError):
                pass
        self._ctx.run_shutdown_hooks()                 # twin and similar
        try:
            if not self._ctl.shutdown(10.0):
                log.warning("run thread did not end within 10 s")
        except Exception as exc:   # pragma: no cover - shutdown best effort
            log.warning("run controller shutdown failed: %s", exc)
        try:
            self._worker.request_stop()                # the heartbeat stops LAST
        except Exception as exc:   # pragma: no cover - shutdown best effort
            log.warning("worker stop failed: %s", exc)
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait(3000)
        super().closeEvent(event)
