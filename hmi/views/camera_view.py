"""Camera view (docs/HMI_DESIGN.md section 11.1): the run's camera image with the detected ChArUco corners and board
names, the board results of the shot (accepted / rejected, corners, reprojection RMS), the last frame fit (fit
residual, error vs nominal, jump vs prediction; accepted or rejected), Grab / Live (REAL, no run active) and a
snapshot of the shown image into the run log folder.

Images arrive as ShotViews through RunController.shot (sequencer shots and manual grabs). They are made in the run
thread by the OverlayProcessor installed here (hmi.core.overlay: detection + drawing); this widget never opens or
grabs the camera itself - Grab / Live ask the controller, which grabs in the run thread with the rig's one camera
(D-H4). The display is rate-limited ([hmi] camera_display_hz): shots arriving faster replace the pending one, so a
fast SIM run never queues paint work in the GUI thread; while the tab is hidden nothing is painted. Snapshots are
written by a short-lived helper thread "camera-snapshot" (file IO only).
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from PySide6.QtCore import Qt, QTimer, QUrl, Signal
from PySide6.QtGui import QColor, QDesktopServices, QImage, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QSizePolicy,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from ..amr.ui.widgets import GROUP_CSS, PulseBtn, lbl, set_style, set_text
from ..core.overlay import CameraShot, OverlayProcessor, board_results, redraw, write_snapshot

SNAP_DIR = "snapshots"           # <run log folder>/snapshots/
SOURCES = {"synth": "SIM camera (synthetic render)", "ids": "IDS camera (REAL)"}
STATUS_COLOURS = {"accepted": "#44CC44", "seen": "#33CCEE", "partial": "#FFAA00", "rejected": "#FF8800",
                  "not seen": "#FF5555"}
BANNER_CSS = {"error": "background:#3A1010; color:#FF6666; border:1px solid #AA3333;",
              "warning": "background:#33260A; color:#FFAA00; border:1px solid #8A5A00;"}
LOOP_OFF = "Dead reckoning: camera loop off - this run takes no images"
FIT_KEYS = ("Result", "Frame", "Boards", "Fit RMS / max", "Baseline", "Error vs nominal", "Jump vs prediction",
            "Measured ARES", "Per board")
RIGHT_W = 400


# ── pure presentation helpers (tested) ────────────────────────────────────────
def _num(v: Any, fmt: str = ".2f", unit: str = "") -> str:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return "-"
    return f"{f:{fmt}}{unit}" if math.isfinite(f) else "-"


def _pose(p: Any) -> str:
    if isinstance(p, Mapping) and "x_mm" in p:
        th = p.get("theta_deg", math.degrees(float(p.get("theta_rad") or 0.0)))
        return f"({_num(p['x_mm'], '.0f')}, {_num(p['y_mm'], '.0f')}) mm {_num(th, '+.2f')} deg"
    return "-"


def fit_rows(rec: Mapping) -> list[tuple[str, str]]:
    """(label, text) rows of a 'wall_frame' / 'station_frame' record (camera fit or dead reckoning)."""
    kind = "wall" if rec.get("event") == "wall_frame" else "station"
    if rec.get("source") != "camera":
        return [("Result", "dead reckoning (no image)"), ("Frame", f"{kind} stop {rec.get('stop', '-')}")]
    why = f" ({rec['why']})" if rec.get("why") else ""
    rows = [("Result", "ACCEPTED"),
            ("Frame", f"{kind} stop {rec.get('stop', '-')}{why}, attempt {rec.get('attempt', 0)}"),
            ("Boards", ", ".join(rec.get("boards") or []) or "-"),
            ("Fit RMS / max", f"{_num(rec.get('rms_mm'))} / {_num(rec.get('max_mm'))} mm"),
            ("Baseline", _num(rec.get("baseline_mm"), ".0f", " mm")),
            ("Error vs nominal", f"{_num(rec.get('err_nominal_mm'), '.1f')} mm / "
                                 f"{_num(rec.get('err_nominal_deg'), '.2f')} deg"),
            ("Jump vs prediction", f"{_num(rec.get('jump_mm'), '.1f')} mm / {_num(rec.get('jump_deg'), '.2f')} deg"),
            ("Measured ARES", _pose(rec.get("measured")))]
    res = (rec.get("fit") or {}).get("residuals") or {}
    if res:
        rows.append(("Per board", ", ".join(f"{b} {_num(r.get('rms_mm'))} mm ({r.get('n_corners', '?')} c)"
                                            for b, r in res.items())))
    return rows


def measurement_banner(rec: Mapping) -> tuple[str, str] | None:
    """(level, text) of a record that rejects a measurement or warns about the view; None = no banner change."""
    ev = rec.get("event", "")
    if ev == "measurement_failed":
        return "error", f"Measurement REJECTED: {rec.get('msg', '?')}"
    if ev == "frame_jump":
        return "error", (f"Frame REJECTED ({rec.get('what', '?')}): measured {_pose(rec.get('measured'))} is "
                         f"{_num(rec.get('d_mm'), '.1f')} mm / {_num(rec.get('d_deg'), '.2f')} deg from the "
                         f"predicted {_pose(rec.get('predicted'))}")
    if ev == "coarse_aim":
        return "warning", (f"Coarse aim: board {rec.get('board', '?')} only partly in view "
                           f"({rec.get('n_corners', '?')} corners) - looks re-aimed")
    if ev == "board_search":
        return "warning", "Board search: no board of the looks in view - shooting around the predicted pose"
    return None


def to_qimage(arr: np.ndarray) -> QImage:
    """A deep-copied QImage of a uint8 grey (H x W) or BGR (H x W x 3) array."""
    a = np.ascontiguousarray(arr)
    h, w = a.shape[:2]
    if a.ndim == 2:
        return QImage(a.data, w, h, w, QImage.Format.Format_Grayscale8).copy()
    return QImage(a.data, w, h, 3 * w, QImage.Format.Format_BGR888).copy()


class ImageLabel(QLabel):
    """Shows a pixmap scaled to the label with the aspect ratio kept (never grows the window)."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._pix: QPixmap | None = None
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background:#101010; color:#666666;")

    def set_image(self, pix: QPixmap | None, text: str = "") -> None:
        self._pix = pix
        if pix is None:
            self.clear()
            self.setText(text)
        else:
            self._rescale()

    def source(self) -> QPixmap | None:
        return self._pix

    def resizeEvent(self, e) -> None:
        super().resizeEvent(e)
        self._rescale()

    def _rescale(self) -> None:
        if self._pix is not None and not self._pix.isNull():
            self.setPixmap(self._pix.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class CameraView(QWidget):
    snapshot_saved = Signal(str, str)         # (first written path or "", error or "") from the snapshot thread

    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        hmi = ctx.hmi
        self._display_hz = float(hmi["camera_display_hz"])
        self._live_hz = float(hmi["camera_live_hz"])
        # a grab that neither delivered an image nor failed within twice the camera's own grab timeout is lost
        self._grab_lost_s = 2.0 * float(ctx.station_cfg.get("camera", {}).get("timeout_ms", 3000)) / 1000.0
        self._min_corners = int(ctx.station_cfg.get("vision", {}).get("min_corners", 8))
        self._proc = OverlayProcessor(hmi)
        self._view = None                     # the ShotView shown
        self._view_dir: Path | None = None    # run log folder of the shown image
        self._pending = None                  # (ShotView, folder) waiting for the display limit
        self._last_paint = -math.inf
        self._grab_t: float | None = None     # monotonic time of the grab in flight
        self._fit: dict | None = None         # last accepted camera frame fit record
        self._snap_dir: Path | None = None
        self.n_shots = self.n_shown = self.n_fits = self.n_rejected = 0
        self._build_ui()
        self.setStyleSheet(GROUP_CSS)

        self._paint_timer = QTimer(self)
        self._paint_timer.setSingleShot(True)
        self._paint_timer.timeout.connect(self._flush)
        self._live_timer = QTimer(self)
        self._live_timer.setInterval(max(50, int(round(1000.0 / max(self._live_hz, 0.01)))))
        self._live_timer.timeout.connect(self._live_tick)

        c = self._ctl
        c.set_shot_processor(self._proc)
        c.shot.connect(self._on_shot)
        c.event.connect(self._on_event)
        c.state_changed.connect(self._on_state)
        c.rig_changed.connect(self._on_rig)
        c.grab_failed.connect(self._on_grab_failed)
        ctx.session_changed.connect(self._on_session)
        self.snapshot_saved.connect(self._on_saved)
        self._refresh()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(4, 2, 4, 2)
        left = QVBoxLayout()
        row = QHBoxLayout()
        self.grab_btn = PulseBtn("Grab", "#2A4A6A", min_w=90)
        self.grab_btn.clicked.connect(self.grab)
        row.addWidget(self.grab_btn)
        self.live = QCheckBox(f"Live ({self._live_hz:g} Hz)")
        self.live.setFocusPolicy(Qt.NoFocus)
        self.live.toggled.connect(self._on_live)
        row.addWidget(self.live)
        self.snap_btn = PulseBtn("Snapshot", "#2A5A3A", min_w=110)
        self.snap_btn.clicked.connect(self.save_snapshot)
        row.addWidget(self.snap_btn)
        self.ids = QCheckBox("corner ids")
        self.ids.setFocusPolicy(Qt.NoFocus)
        self.ids.toggled.connect(self._on_ids)
        row.addWidget(self.ids)
        row.addStretch()
        self.source = lbl("camera: -")
        row.addWidget(self.source)
        left.addLayout(row)
        self.loop_note = QLabel(LOOP_OFF)
        self.loop_note.setStyleSheet(BANNER_CSS["warning"] + "padding:4px;")
        self.loop_note.hide()
        left.addWidget(self.loop_note)
        self.image = ImageLabel()
        self.image.set_image(None, "no image yet")
        left.addWidget(self.image, 1)
        root.addLayout(left, 1)

        right = QVBoxLayout()
        g_img = QGroupBox("Image")
        f = QFormLayout(g_img)
        self.info: dict[str, QLabel] = {}
        for key in ("Look", "Time", "Image", "Processing", "Shots", "Note"):
            self.info[key] = QLabel("-")
            self.info[key].setWordWrap(True)
            f.addRow(key, self.info[key])
        right.addWidget(g_img)

        g_b = QGroupBox("Boards of this image")
        vb = QVBoxLayout(g_b)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Board", "Result", "Corners", "RMS px", "Reason"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setStyleSheet("QTableWidget{background:#151515; color:#CCCCCC; gridline-color:#333333;}")
        self.table.setMinimumHeight(110)
        vb.addWidget(self.table)
        right.addWidget(g_b)

        g_f = QGroupBox("Last frame fit")
        ff = QFormLayout(g_f)
        self.fit: dict[str, QLabel] = {}
        for key in (*FIT_KEYS, "Fits"):
            self.fit[key] = QLabel("-")
            self.fit[key].setWordWrap(True)
            ff.addRow(key, self.fit[key])
        right.addWidget(g_f)

        self.banner = QLabel("")
        self.banner.setWordWrap(True)
        self.banner.hide()
        right.addWidget(self.banner)
        self.snap_status = QLabel("")
        self.snap_status.setWordWrap(True)
        self.snap_status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.snap_status.setStyleSheet("color:#999999;")
        right.addWidget(self.snap_status)
        self.open_btn = PulseBtn("Open snapshot folder", "#2A4A6A", min_w=150, min_h=26)
        self.open_btn.clicked.connect(self.open_folder)
        self.open_btn.setEnabled(False)
        right.addWidget(self.open_btn)
        right.addStretch()
        panel = QWidget()
        panel.setLayout(right)
        panel.setFixedWidth(RIGHT_W)
        root.addWidget(panel)

    # ── shots ─────────────────────────────────────────────────────────────────
    def _run_folder(self) -> Path | None:
        """The run log folder now (the sequencer's), else the folder of the latest snapshot's run.jsonl."""
        d = self._ctl.log_dir
        if d is not None:
            return Path(d)
        snap = self._ctl.snapshot
        return Path(snap.log_path).parent if snap is not None and snap.log_path else None

    def _on_shot(self, view) -> None:
        self.n_shots += 1
        if view.rec is None:
            self._grab_t = None                    # the manual grab in flight has arrived
        self._pending = (view, self._run_folder())
        self._schedule()
        self._show_counts()

    def _schedule(self) -> None:
        if self._pending is None or self._paint_timer.isActive():
            return
        wait_s = max(0.0, self._last_paint + 1.0 / max(self._display_hz, 0.1) - time.monotonic())   # first: 0
        self._paint_timer.start(int(math.ceil(wait_s * 1000.0)))

    def _flush(self) -> None:
        if self._pending is None or not self.isVisible():
            return                                 # hidden: painted by showEvent
        (view, folder), self._pending = self._pending, None
        self._last_paint = time.monotonic()
        self.show_view(view, folder)

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self._schedule()

    def show_view(self, view, folder: Path | None = None) -> None:
        """Paint one ShotView (GUI thread; the display limit is applied by the caller)."""
        if isinstance(view, CameraShot):
            view = redraw(view, self.ids.isChecked())     # processed before the corner-id switch changed
        self._view, self._view_dir = view, folder
        self.n_shown += 1
        self.image.set_image(QPixmap.fromImage(to_qimage(view.preview)))
        rec = view.rec
        set_text(self.info["Look"], "manual grab" if rec is None else f"{view.look} ({view.parent})")
        set_text(self.info["Time"], time.strftime("%H:%M:%S", time.localtime(view.t)))
        w, h = view.full_size
        set_text(self.info["Image"], f"{w} x {h} px, preview {view.preview.shape[1]} x {view.preview.shape[0]} px")
        ms = getattr(view, "detect_ms", None)
        set_text(self.info["Processing"], f"{ms:.0f} ms (detect + draw, run thread)" if ms else "plain preview")
        set_text(self.info["Note"], view.note or "-")
        boards = getattr(view, "boards", None)
        if boards is None:                         # the controller's plain fallback view
            boards = board_results(rec, view.detections, self._min_corners)
        self._fill_table(boards)
        self._show_counts()
        self._refresh()

    def _fill_table(self, boards) -> None:
        self.table.setRowCount(len(boards))
        for i, r in enumerate(boards):
            vals = (r.board, r.status, str(r.n_corners), _num(r.rms_px) if r.rms_px is not None else "-",
                    r.reason or "")
            for j, v in enumerate(vals):
                it = QTableWidgetItem(v)
                if j == 1:
                    it.setForeground(QColor(STATUS_COLOURS.get(r.status, "#CCCCCC")))
                self.table.setItem(i, j, it)

    def _show_counts(self) -> None:
        set_text(self.info["Shots"], f"{self.n_shots} received, {self.n_shown} shown "
                                     f"(display limit {self._display_hz:g} Hz)")

    def _on_ids(self, on: bool) -> None:
        self._proc.show_ids = bool(on)
        if isinstance(self._view, CameraShot):
            self._view = redraw(self._view, bool(on))
            self.image.set_image(QPixmap.fromImage(to_qimage(self._view.preview)))

    # ── measurement records ───────────────────────────────────────────────────
    def _on_event(self, rec: dict) -> None:
        ev = rec.get("event", "")
        if ev in ("wall_frame", "station_frame"):
            if rec.get("source") == "camera":
                self._fit = dict(rec)
                self.n_fits += 1
                self._set_banner(None)
            self._show_fit(rec)
            return
        b = measurement_banner(rec)
        if b is not None:
            if b[0] == "error":
                self.n_rejected += 1
                set_text(self.fit["Result"], "REJECTED")
                self.fit["Result"].setStyleSheet("color:#FF5555; font-weight:bold;")
                self._show_fit_count()
            self._set_banner(b)

    def _show_fit(self, rec: Mapping) -> None:
        for key in FIT_KEYS:
            set_text(self.fit[key], "-")
        for key, text in fit_rows(rec):
            set_text(self.fit[key], text)
        ok = rec.get("source") == "camera"
        self.fit["Result"].setStyleSheet("color:#44CC44; font-weight:bold;" if ok else "color:#FFAA00;")
        self._show_fit_count()

    def _show_fit_count(self) -> None:
        set_text(self.fit["Fits"], f"{self.n_fits} accepted, {self.n_rejected} rejected (this run log)")

    def _set_banner(self, b: tuple[str, str] | None) -> None:
        if b is None:
            self.banner.hide()
            return
        level, text = b
        set_style(self.banner, BANNER_CSS[level] + "padding:4px; font-weight:bold;")
        set_text(self.banner, text)
        self.banner.show()

    def banner_text(self) -> str:
        return "" if self.banner.isHidden() else self.banner.text()

    # ── controller state ──────────────────────────────────────────────────────
    def _on_state(self, _state: str, _detail: str) -> None:
        self._refresh()
        ok, why = self._ctl.can("grab")
        if self.live.isChecked() and not ok:
            self._stop_live(f"Live stopped: {why}")

    def _on_rig(self, rig) -> None:
        if rig is not None:                        # a new rig = a new run log: fresh measurement counters
            self._fit = None
            self.n_fits = self.n_rejected = 0
            for lab in self.fit.values():
                set_text(lab, "-")
                lab.setStyleSheet("")
            self._set_banner(None)
            self._grab_t = None
        self._refresh()

    def _on_session(self, _session) -> None:
        self._view, self._view_dir, self._pending = None, None, None
        self.n_shots = self.n_shown = 0
        self.image.set_image(None, "no image yet")
        self.table.setRowCount(0)
        for lab in self.info.values():
            set_text(lab, "-")
        self._refresh()

    def _on_grab_failed(self, error: str) -> None:
        self._grab_t = None
        text = f"grab failed: {error}"
        set_text(self.snap_status, text)
        if self.live.isChecked():
            self._stop_live(f"Live stopped: {text}")

    def _refresh(self) -> None:
        c = self._ctl
        rig = c.rig
        kind = getattr(rig, "camera_kind", None) if rig is not None else None
        set_text(self.source, "camera: " + (SOURCES.get(kind, kind) if kind else "- (Prepare a run first)"))
        opts = c.opts
        self.loop_note.setVisible(opts is not None and not getattr(opts, "camera_loop", True))
        ok, why = c.can("grab")
        self.grab_btn.setEnabled(ok)
        self.grab_btn.setToolTip("" if ok else why)
        self.live.setEnabled(ok or self.live.isChecked())
        self.live.setToolTip("" if ok else why)
        can_snap = self._view is not None and self._view_dir is not None
        self.snap_btn.setEnabled(can_snap)
        self.snap_btn.setToolTip("" if can_snap else "no image" if self._view is None else
                                 "no run log folder for this image (Prepare a run first)")

    # ── Grab / Live (REAL, no run active) ─────────────────────────────────────
    def grab(self) -> None:
        if not self._ctl.can("grab")[0]:
            return
        self._grab_t = time.monotonic()
        self._ctl.grab()

    def _on_live(self, on: bool) -> None:
        if not on:
            self._live_timer.stop()
            self._refresh()
            return
        ok, why = self._ctl.can("grab")
        if not ok:
            self._stop_live(f"Live not started: {why}")
            return
        set_text(self.snap_status, "")
        self._live_timer.start()
        self._live_tick()

    def _live_tick(self) -> None:
        ok, why = self._ctl.can("grab")
        if not ok:
            self._stop_live(f"Live stopped: {why}")
            return
        if self._grab_t is not None and time.monotonic() - self._grab_t < self._grab_lost_s:
            return                                 # the previous grab is still in the run thread
        self.grab()

    def _stop_live(self, why: str) -> None:
        self._live_timer.stop()
        if self.live.isChecked():
            self.live.blockSignals(True)
            self.live.setChecked(False)
            self.live.blockSignals(False)
        set_text(self.snap_status, why)
        self._refresh()

    @property
    def live_active(self) -> bool:
        return self._live_timer.isActive()

    # ── snapshot ──────────────────────────────────────────────────────────────
    def save_snapshot(self) -> None:
        """The shown image (+ the full camera image, + a JSON sidecar) -> <run log folder>/snapshots/, written by a
        helper thread (a full-resolution PNG takes ~100 ms)."""
        view, folder, fit = self._view, self._view_dir, self._fit
        if view is None or folder is None:
            return
        target = Path(folder) / SNAP_DIR
        set_text(self.snap_status, "saving snapshot ...")

        def work() -> None:
            try:
                paths = write_snapshot(target, view, fit)
                res = (str(paths[0]), "")
            except Exception as e:                 # noqa: BLE001 - reported in the view
                res = ("", f"{type(e).__name__}: {e}")
            try:
                self.snapshot_saved.emit(*res)
            except RuntimeError:                   # the view was closed meanwhile
                pass
        threading.Thread(target=work, name="camera-snapshot", daemon=True).start()

    def _on_saved(self, path: str, error: str) -> None:
        if error:
            set_text(self.snap_status, f"snapshot NOT saved: {error}")
            return
        self._snap_dir = Path(path).parent
        set_text(self.snap_status, f"snapshot saved: {path}")
        self.open_btn.setEnabled(True)

    def open_folder(self) -> None:
        if self._snap_dir is not None:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self._snap_dir)))
