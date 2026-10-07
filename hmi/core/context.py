"""HmiContext: what every Mauer widget gets (docs/HMI_DESIGN.md section 10.1) - the station config, the amr config
dict, the RunController, the latest ADS status of the HMI's own worker and the twin state, as Qt signals and plain
attributes. Lives in the GUI thread; last_ads_status / ads_connected are replaced (never mutated), so other threads
(run, twin) may read them.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Callable

from PySide6.QtCore import QObject, Signal

from mauer import REPO

from .config import amr_cfg

log = logging.getLogger("hmi.context")


class HmiContext(QObject):
    session_changed = Signal(object)          # JobSession | None (re-emitted RunController.session_loaded)
    ads_status = Signal(dict)                 # AdsWorker status (10 Hz), forwarded by MainWindow
    ads_connection = Signal(bool, str)
    twin_state_changed = Signal(str, str)

    def __init__(self, station_cfg: dict, *, config_path: Path | None = None, ares_enabled: bool = False,
                 controller=None, runs_dir: Path | None = None, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.station_cfg = station_cfg
        self.hmi: dict = dict(station_cfg["hmi"])
        self.amr_cfg: dict = amr_cfg(station_cfg)
        self.config_path = Path(config_path) if config_path else None
        self.ares_enabled = bool(ares_enabled)
        self.last_ads_status: dict | None = None
        self.ads_connected = False
        self.twin_state: tuple[str, str] = ("off", "")
        self._hooks: list[Callable[[], Any]] = []
        if controller is None:
            from .run_controller import RunController
            rd = Path(runs_dir) if runs_dir else REPO / str(self.hmi.get("runs_dir", "data/runs"))
            controller = RunController(self.hmi, config_path=self.config_path, runs_dir=rd,
                                       ads_status_fn=lambda: self.last_ads_status,
                                       ads_connected_fn=lambda: self.ads_connected, ares_enabled=self.ares_enabled,
                                       parent=self)
        self.controller = controller
        controller.session_loaded.connect(self.session_changed.emit)

    @property
    def session(self):
        return self.controller.session

    def publish_ads_status(self, d: dict) -> None:
        self.last_ads_status = d
        self.ads_status.emit(d)

    def publish_ads_connection(self, ok: bool, msg: str) -> None:
        self.ads_connected = bool(ok)
        if not ok:
            self.last_ads_status = None
        self.ads_connection.emit(bool(ok), msg)

    def set_twin_state(self, state: str, detail: str = "") -> None:
        self.twin_state = (state, detail)
        self.twin_state_changed.emit(state, detail)

    def add_shutdown_hook(self, fn: Callable[[], Any]) -> None:
        self._hooks.append(fn)

    def run_shutdown_hooks(self) -> None:
        for fn in self._hooks:
            try:
                fn()
            except Exception as e:      # noqa: BLE001 - one failing hook must not stop the others
                log.warning("shutdown hook %r failed: %s", fn, e)
