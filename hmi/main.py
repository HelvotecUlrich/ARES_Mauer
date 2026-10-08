"""Mauer HMI entry point (docs/HMI_DESIGN.md section 10.1) - adapted from MA amr_hmi main.py (commit 5935c5b): no
config.yaml (config/station.toml [hmi] / [ares_ads]), command-line options for the job and the ADS worker.

    py.exe -m hmi [--job PATH] [--variant NAME] [--config PATH] [--ares] [--twin]

--job      load this job at start (with its own config variant, D-H11)
--variant  preselect the variant for "Build from config"
--config   station.toml to use (default config/station.toml)
--ares     create the real AdsWorker: it connects to the [ares_ads] host (heartbeat, MANUAL, HALT, jog). Without it
           the HMI never opens an ADS connection and REAL mode is disabled
--twin     switch the RoboDK twin on (Twin tab): it starts with the first job, in an own RoboDK instance on the
           first free API port from [hmi.twin] port (>= 20630, never the user's RoboDK on 20500 / 20501)
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from mauer import config as mconfig

from .core.ads_link import NullAdsWorker, create_ads_worker
from .core.config import check_amr_cfg
from .core.context import HmiContext
from .main_window import MainWindow

log = logging.getLogger("hmi")


def apply_dark_theme(app: QApplication) -> None:
    """Fusion + dark palette: the widget colours of this HMI assume a dark background."""
    app.setStyle("Fusion")
    pal = QPalette()
    for role, col in ((QPalette.Window, "#202020"), (QPalette.WindowText, "#DDDDDD"),
                      (QPalette.Base, "#1A1A1A"), (QPalette.AlternateBase, "#252525"),
                      (QPalette.Text, "#DDDDDD"), (QPalette.Button, "#2A2A2A"),
                      (QPalette.ButtonText, "#DDDDDD"), (QPalette.Highlight, "#2A5A8A"),
                      (QPalette.HighlightedText, "#FFFFFF"), (QPalette.ToolTipBase, "#333333"),
                      (QPalette.ToolTipText, "#EEEEEE")):
        pal.setColor(role, QColor(col))
    app.setPalette(pal)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="py.exe -m hmi", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--job", default=None, help="job JSON to load at start (its own config variant)")
    ap.add_argument("--variant", default=None, help="preselected config variant for 'Build from config'")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--ares", action="store_true", help="connect the HMI's ADS worker to the ARES PLC")
    ap.add_argument("--twin", action="store_true", help="start the RoboDK twin once a job is loaded")
    return ap.parse_args(argv)


def build(argv: list[str] | None = None) -> tuple[MainWindow, HmiContext]:
    """Context, ADS worker and window without app.exec() (needs a QApplication; used by main() and a test)."""
    args = parse_args(argv)
    station_cfg = mconfig.load(args.config)
    problems = check_amr_cfg(station_cfg)
    ares = bool(args.ares)
    if ares and problems:
        log.error("ADS refused, HMI settings: %s", "; ".join(problems))
        ares = False
    ctx = HmiContext(station_cfg, config_path=Path(args.config) if args.config else None, ares_enabled=ares)
    ctx.start_twin = bool(args.twin)          # read by the Twin tab (hmi/views/twin_panel.py)
    if ares:
        worker, thread = create_ads_worker(ctx.amr_cfg)
    else:
        worker, thread = NullAdsWorker(), None
    win = MainWindow(ctx, worker, thread)
    if thread is None:
        worker.start()                        # "ADS off: start the HMI with --ares"
    else:
        thread.start()                        # only now: the window receives 'connected' and the interface
    if problems:
        win.statusBar().showMessage("HMI settings: " + "; ".join(problems), 15000)
    if args.variant:
        win.mauer.preselect_variant(args.variant)
    if args.job:
        ctx.controller.load_file(Path(args.job))
    return win, ctx


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setApplicationName("Mauer HMI")
    app.setOrganizationName("ARES_Mauer")
    apply_dark_theme(app)
    win, _ctx = build(argv)
    win.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())
