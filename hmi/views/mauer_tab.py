"""Mauer tab (docs/HMI_DESIGN.md section 10.2): load or build a job, choose SIM / REAL and the run options, prepare /
connect, start / pause / resume / abort / release, follow the progress (stop, stone, action, station trip, ARES
pose), recover a stopped run (confirm / set pose, odometry, jaws empty, re-check) and read the preflight list.

Every button is enabled from RunController.can() (the enable matrix 8.4) and takes no keyboard focus (Space / Esc
are HALT). All work runs in the controller's run thread; this widget only calls the controller and shows signals.
"""
from __future__ import annotations

import math
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QRadioButton, QScrollArea, QSpinBox, QSplitter, QVBoxLayout,
    QWidget,
)

from mauer.reference import Pose2D
from mauer.simworld import SCENARIOS

from ..amr.ui.widgets import GROUP_CSS, PulseBtn, lbl, set_style, set_text
from ..core.run_controller import RunOptions
from ..core.session import JOBS_DIR, list_jobs, list_variants
from ..core.snapshot import stop_text
from ..core.sources import ares_live
from .feedback import RunFeedback
from .plan_view import STATUS_COLOURS, PlanView

STATE_COLOURS = {"empty": "#888888", "loading": "#66AACC", "loaded": "#AAAAAA", "preparing": "#66AACC",
                 "ready": "#44CC44", "running": "#33EE33", "pausing": "#FFAA00", "aborting": "#FFAA00",
                 "paused": "#FFAA00", "aborted": "#FF8844", "error": "#FF4444", "done": "#44CCFF",
                 "releasing": "#66AACC"}
LEFT_W = 430


def _spin(lo: float, hi: float, value: float, suffix: str, decimals: int = 1, step: float = 1.0) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setValue(value)
    s.setSuffix(suffix)
    return s


class MauerTab(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._snap = None
        self._last_state = ""
        self._build_ui()
        self.setStyleSheet(GROUP_CSS)
        c = self._ctl
        c.state_changed.connect(self._on_state)
        c.session_loaded.connect(self._on_session)
        c.snapshot_changed.connect(self._on_snapshot)
        c.preflight_done.connect(self._on_preflight)
        c.rig_changed.connect(lambda _r: self._update_enables())
        ctx.ads_status.connect(self._on_ads_status)
        ctx.ads_connection.connect(lambda _ok, _m: self._update_enables())
        self.refresh_jobs()
        if c.session is not None:
            self._on_session(c.session)
        self._on_state(c.state, "")

    # ── construction ─────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        left = QWidget()
        col = QVBoxLayout(left)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self._job_group())
        col.addWidget(self._run_group())
        col.addWidget(self._buttons())
        col.addWidget(self._progress_group())
        col.addWidget(self._recovery_group())
        col.addWidget(self._preflight_group())
        col.addStretch()
        scroll = QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(LEFT_W + 24)
        scroll.setFocusPolicy(Qt.NoFocus)
        outer.addWidget(scroll)
        # right: the plan view above the run feedback board (visible during the run without scrolling the left column)
        right = QSplitter(Qt.Vertical)
        right.setChildrenCollapsible(False)
        self.plan = PlanView()
        self.feedback = RunFeedback(self._ctx)
        right.addWidget(self.plan)
        right.addWidget(self.feedback)
        right.setStretchFactor(0, 3)
        right.setStretchFactor(1, 1)
        outer.addWidget(right, 1)

    def _button(self, text: str, colour: str, slot, min_w: int = 90) -> PulseBtn:
        b = PulseBtn(text, colour, min_w=min_w, min_h=30)
        b.clicked.connect(slot)
        return b

    def _job_group(self) -> QGroupBox:
        box = QGroupBox("Job")
        g = QGridLayout(box)
        self.job_combo = QComboBox()
        self.job_combo.setFocusPolicy(Qt.NoFocus)
        self.load_btn = self._button("Load", "#2A4A6A", self._load)
        self.variant_combo = QComboBox()
        self.variant_combo.setFocusPolicy(Qt.NoFocus)
        self.build_btn = self._button("Build from config", "#2A4A6A", self._build, 130)
        self.browse_btn = self._button("Browse...", "#333333", self._browse)
        self.summary = QLabel("no job loaded")
        self.summary.setWordWrap(True)
        self.summary.setStyleSheet("color:#DDDDDD;")
        g.addWidget(self.job_combo, 0, 0, 1, 2)
        g.addWidget(self.load_btn, 0, 2)
        g.addWidget(lbl("config"), 1, 0)
        g.addWidget(self.variant_combo, 1, 1)
        g.addWidget(self.build_btn, 1, 2)
        g.addWidget(self.browse_btn, 2, 2)
        g.addWidget(self.summary, 3, 0, 1, 3)
        self._file_dialog: QFileDialog | None = None
        return box

    def _run_group(self) -> QGroupBox:
        box = QGroupBox("Run")
        f = QFormLayout(box)
        row = QHBoxLayout()
        self.sim_rb, self.real_rb = QRadioButton("SIM"), QRadioButton("REAL")
        self.sim_rb.setChecked(True)
        grp = QButtonGroup(self)
        for rb in (self.sim_rb, self.real_rb):
            rb.setFocusPolicy(Qt.NoFocus)
            grp.addButton(rb)
            rb.toggled.connect(lambda _on: self._update_enables())
            row.addWidget(rb)
        if not self._ctl.ares_enabled:
            self.real_rb.setToolTip("REAL needs the HMI started with --ares (the ADS worker owns heartbeat, MANUAL "
                                    "and HALT)")
        f.addRow(row)
        self.scenario = QComboBox()
        self.scenario.addItems(list(SCENARIOS))
        self.scenario.setCurrentText("realistic")
        self.seed = QSpinBox()
        self.seed.setRange(0, 9999)
        self.seed.setValue(1)
        self.lenient = QCheckBox("lenient grasp (SIM gripper never misses)")
        self.stop_from, self.stop_to = QSpinBox(), QSpinBox()
        self.camera_loop = QCheckBox("camera loop (off: dead reckoning)")
        self.camera_loop.setChecked(True)
        self.save_images = QCheckBox("save images in the run log")
        self.guard = QCheckBox("SIM motion guard (every joint move planned as on the real robot)")
        self.guard.setChecked(True)
        self.untouched = QCheckBox("start stop untouched (no stone of it placed yet)")
        self.resume_log = QLabel("")                    # run log of an interrupted run (set by the dialog only)
        self.resume_log.setWordWrap(True)
        self.resume_btn_log = self._button("Run log...", "#333333", self._browse_log, 90)
        self.resume_clear = self._button("Clear", "#333333", lambda: self._set_resume_log(None), 60)
        self._set_resume_log(None)
        self.step = QCheckBox("Step mode: confirm every motion")
        self.step.toggled.connect(self._ctl.set_step)
        self.sim_speed = _spin(0.0, 5.0, float(self._ctx.hmi.get("sim_step_s", 0.3)), " s", 2, 0.1)
        self.sim_speed.valueChanged.connect(self._ctl.set_sim_step_s)
        stops = QHBoxLayout()
        stops.addWidget(self.stop_from)
        stops.addWidget(lbl("to"))
        stops.addWidget(self.stop_to)
        for w in (self.scenario, self.seed, self.lenient, self.stop_from, self.stop_to, self.camera_loop,
                  self.save_images, self.step, self.sim_speed, self.guard, self.untouched):
            w.setFocusPolicy(Qt.NoFocus)
        later = QHBoxLayout()
        later.addWidget(self.resume_log, 1)
        later.addWidget(self.resume_btn_log)
        later.addWidget(self.resume_clear)
        f.addRow("SIM scenario", self.scenario)
        f.addRow("SIM seed", self.seed)
        f.addRow(self.lenient)
        f.addRow("stops", stops)
        f.addRow("earlier run", later)
        f.addRow(self.untouched)
        f.addRow(self.guard)
        f.addRow(self.camera_loop)
        f.addRow(self.save_images)
        f.addRow(self.step)
        f.addRow("SIM s per motion", self.sim_speed)
        self._option_widgets = (self.sim_rb, self.real_rb, self.scenario, self.seed, self.lenient, self.stop_from,
                                self.stop_to, self.camera_loop, self.save_images, self.guard, self.untouched,
                                self.resume_btn_log, self.resume_clear)
        return box

    def _buttons(self) -> QWidget:
        w = QWidget()
        g = QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        self.prepare_btn = self._button("Prepare / Connect", "#1A4A7A", self._prepare, 130)
        self.start_btn = self._button("Start", "#0A6E30", self._ctl.start)
        self.pause_btn = self._button("Pause", "#8A6A00", self._ctl.pause)
        self.resume_btn = self._button("Resume", "#0A6E30", self._ctl.resume)
        self.abort_btn = self._button("Abort", "#7A1515", self._ctl.abort)
        self.release_btn = self._button("Release", "#444444", self._ctl.release)
        for i, b in enumerate((self.prepare_btn, self.start_btn, self.pause_btn, self.resume_btn, self.abort_btn,
                               self.release_btn)):
            g.addWidget(b, i // 3, i % 3)
        return w

    def _progress_group(self) -> QGroupBox:
        box = QGroupBox("Progress")
        g = QFormLayout(box)
        self.state_lbl = QLabel("-")
        self.state_lbl.setStyleSheet("font-weight:bold; font-size:14pt;")
        self.halted_lbl = QLabel("HALTED")
        self.halted_lbl.setStyleSheet("background:#B00000;color:#FFFFFF;font-weight:bold;padding:2px 6px;"
                                      "border-radius:3px;")
        self.halted_lbl.hide()
        top = QHBoxLayout()
        top.addWidget(self.state_lbl)
        top.addWidget(self.halted_lbl)
        top.addStretch()
        g.addRow(top)
        self.stop_lbl, self.stone_lbl, self.action_lbl, self.trip_lbl = QLabel("-"), QLabel("-"), QLabel("-"), \
            QLabel("-")
        self.pose_lbl, self.held_lbl, self.pending_lbl, self.error_lbl = QLabel("-"), QLabel("-"), QLabel(""), \
            QLabel("")
        for w in (self.stone_lbl, self.action_lbl, self.trip_lbl, self.pose_lbl, self.held_lbl, self.error_lbl):
            w.setWordWrap(True)
        self.error_lbl.setStyleSheet("color:#FF6666;")
        self.pending_lbl.setStyleSheet("color:#FFAA00; font-weight:bold;")
        g.addRow("stop", self.stop_lbl)
        g.addRow("stone", self.stone_lbl)
        g.addRow("action", self.action_lbl)
        g.addRow("station", self.trip_lbl)
        g.addRow("ARES", self.pose_lbl)
        g.addRow("jaws", self.held_lbl)
        g.addRow(self.pending_lbl)
        g.addRow(self.error_lbl)
        return box

    def _recovery_group(self) -> QGroupBox:
        box = QGroupBox("Recovery")
        g = QGridLayout(box)
        self.confirm_pose_btn = self._button("Confirm pose", "#2A4A6A", self._ctl.confirm_pose, 110)
        self.apply_odom_btn = self._button("Apply odometry", "#2A4A6A", self._ctl.apply_odometry, 110)
        self.jaws_btn = self._button("Jaws empty", "#6A4400", self._ctl.clear_held, 110)
        self.recheck_btn = self._button("Re-check", "#333333", self._ctl.recheck, 110)
        self.set_x = _spin(-50000, 50000, 0.0, " mm", 1, 10.0)
        self.set_y = _spin(-50000, 50000, 0.0, " mm", 1, 10.0)
        self.set_th = _spin(-360, 360, 0.0, " deg", 2, 0.5)
        for w in (self.set_x, self.set_y, self.set_th):
            w.setFocusPolicy(Qt.ClickFocus)
        self.set_pose_btn = self._button("Set pose", "#2A4A6A", self._set_pose, 110)
        g.addWidget(self.confirm_pose_btn, 0, 0)
        g.addWidget(self.apply_odom_btn, 0, 1)
        g.addWidget(self.jaws_btn, 0, 2)
        g.addWidget(self.set_x, 1, 0)
        g.addWidget(self.set_y, 1, 1)
        g.addWidget(self.set_th, 1, 2)
        g.addWidget(self.set_pose_btn, 2, 0)
        g.addWidget(self.recheck_btn, 2, 2)
        self.recovery = box
        box.hide()
        return box

    def _preflight_group(self) -> QGroupBox:
        box = QGroupBox("Preflight")
        v = QVBoxLayout(box)
        self.preflight_hdr = QLabel("-")
        self.preflight_hdr.setStyleSheet("font-weight:bold;")
        self.preflight_list = QListWidget()
        self.preflight_list.setFocusPolicy(Qt.NoFocus)
        self.preflight_list.setMinimumHeight(110)
        self.preflight_list.setWordWrap(True)
        v.addWidget(self.preflight_hdr)
        v.addWidget(self.preflight_list)
        return box

    # ── job ──────────────────────────────────────────────────────────────────
    def refresh_jobs(self) -> None:
        cur = self.job_combo.currentData()
        self.job_combo.clear()
        for p in list_jobs():
            self.job_combo.addItem(p.name, str(p))
        if cur:
            i = self.job_combo.findData(cur)
            if i >= 0:
                self.job_combo.setCurrentIndex(i)
        self.variant_combo.clear()
        self.variant_combo.addItem("main", None)
        for v in list_variants():
            self.variant_combo.addItem(v, v)

    def preselect_variant(self, variant: str | None) -> None:
        i = self.variant_combo.findData(variant) if variant else 0
        if i >= 0:
            self.variant_combo.setCurrentIndex(i)

    def _load(self) -> None:
        path = self.job_combo.currentData()
        if path:
            self._ctl.load_file(Path(path))

    def _build(self) -> None:
        self._ctl.build_from_config(self.variant_combo.currentData())

    def _browse(self) -> None:
        if not self._browse_allowed()[0]:
            return
        dlg = QFileDialog(self, "Load job", str(JOBS_DIR if JOBS_DIR.is_dir() else Path.cwd()), "Jobs (*.json)")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)    # an HMI child window: Space / Esc stay HALT
        dlg.setFileMode(QFileDialog.ExistingFile)
        dlg.setModal(False)                                      # the HALT button stays usable
        dlg.fileSelected.connect(lambda p: self._ctl.load_file(Path(p)))
        self._file_dialog = dlg
        dlg.show()

    def _browse_allowed(self) -> tuple[bool, str]:
        ok, why = self._ctl.can("browse")
        st = self._ctx.last_ads_status or {}
        if ok and st.get("bMoveActive"):
            return False, "an ARES move is active"
        return ok, why

    def _on_session(self, s) -> None:
        job = s.job
        n_half = sum(t.kind == "half" for t in job.stones())
        m = job.meta
        set_text(self.summary,
                 f"{s.name}  (config {s.variant_label}, {s.source})\n{len(job.stops)} stops, {job.n_stones} stones "
                 f"({job.n_stones - n_half} full / {n_half} half), magazine {job.magazine.capacity}, station "
                 f"{len(job.station.take_order)} slots\ncreated {job.created}, reach check "
                 f"{m.get('reach_check', '?')}, {len(m.get('warnings', []) or [])} warnings")
        for sp in (self.stop_from, self.stop_to):
            sp.setRange(0, max(len(job.stops) - 1, 0))
        self.stop_from.setValue(0)
        self.stop_to.setValue(max(len(job.stops) - 1, 0))
        self.plan.set_session(s)
        self.preselect_variant(s.variant)
        if s.path is not None:                           # show the loaded file in the job list
            target = Path(s.path).resolve()
            for i in range(self.job_combo.count()):
                if Path(self.job_combo.itemData(i)).resolve() == target:
                    self.job_combo.setCurrentIndex(i)
                    break

    # ── run ──────────────────────────────────────────────────────────────────
    def selected_mode(self) -> str:
        return "real" if self.real_rb.isChecked() else "sim"

    def options(self) -> RunOptions:
        a, b = self.stop_from.value(), self.stop_to.value()
        n = len(self._ctl.session.job.stops) if self._ctl.session is not None else 0
        return RunOptions(self.selected_mode(), step=self.step.isChecked(), start_stop=a,
                          stop_after=None if b >= n - 1 else max(a, b), camera_loop=self.camera_loop.isChecked(),
                          save_images=self.save_images.isChecked(), scenario=self.scenario.currentText(),
                          seed=self.seed.value(), lenient_grasp=self.lenient.isChecked(),
                          sim_step_s=self.sim_speed.value(), guard=self.guard.isChecked(),
                          resume_log=self._resume_log, stop_untouched=self.untouched.isChecked())

    def _set_resume_log(self, path) -> None:
        """The run log of an interrupted run: its stones stand (placed / declared), the run starts after them."""
        self._resume_log = Path(path) if path else None
        set_text(self.resume_log, self._resume_log.name if self._resume_log else "none (start from the chosen stop)")

    def _browse_log(self) -> None:
        if not self._browse_allowed()[0]:
            return
        start = self._ctl.runs_dir if self._ctl.runs_dir.is_dir() else Path.cwd()
        dlg = QFileDialog(self, "Run log folder of the interrupted run", str(start))
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)    # an HMI child window: Space / Esc stay HALT
        dlg.setFileMode(QFileDialog.Directory)
        dlg.setOption(QFileDialog.ShowDirsOnly, True)
        dlg.setModal(False)                                      # the HALT button stays usable
        dlg.fileSelected.connect(self._set_resume_log)
        self._file_dialog = dlg
        dlg.show()

    def _prepare(self) -> None:
        self._ctl.prepare(self.options())

    def _set_pose(self) -> None:
        self._ctl.set_pose(Pose2D(self.set_x.value(), self.set_y.value(), math.radians(self.set_th.value())))

    # ── updates ──────────────────────────────────────────────────────────────
    def _update_enables(self) -> None:
        c = self._ctl
        mode = self.selected_mode() if c.state in ("empty", "loaded") else c.mode
        pairs = ((self.load_btn, "load"), (self.build_btn, "build"), (self.prepare_btn, "prepare"),
                 (self.start_btn, "start"), (self.pause_btn, "pause"), (self.resume_btn, "resume"),
                 (self.abort_btn, "abort"), (self.release_btn, "release"), (self.confirm_pose_btn, "confirm_pose"),
                 (self.set_pose_btn, "set_pose"), (self.apply_odom_btn, "apply_odometry"),
                 (self.jaws_btn, "clear_held"), (self.recheck_btn, "recheck"))
        for b, action in pairs:
            ok, why = c.can(action, mode)
            b.setEnabled(ok)
            b.setToolTip(why)
        ok, why = self._browse_allowed()
        self.browse_btn.setEnabled(ok)
        self.browse_btn.setToolTip(why)
        if not self.job_combo.count():
            self.load_btn.setEnabled(False)
        opts_ok = c.can("options")[0]
        for w in self._option_widgets:
            w.setEnabled(opts_ok)
        self.real_rb.setEnabled(opts_ok and c.ares_enabled)
        self.recovery.setVisible(c.state in ("paused", "aborted", "error"))
        self.step.blockSignals(True)
        self.step.setChecked(c.step)
        self.step.blockSignals(False)

    def _on_state(self, state: str, detail: str) -> None:
        c = self._ctl
        set_text(self.state_lbl, state.upper())
        set_style(self.state_lbl, f"font-weight:bold; font-size:14pt; color:{STATE_COLOURS.get(state, '#CCCCCC')};")
        self.halted_lbl.setVisible(c.halted)
        if state in ("paused", "aborted", "error") and state != self._last_state:
            snap = c.snapshot
            if snap is not None and snap.pose_est is not None:     # prefill Set pose with the estimate
                self.set_x.setValue(snap.pose_est.x_mm)
                self.set_y.setValue(snap.pose_est.y_mm)
                self.set_th.setValue(math.degrees(snap.pose_est.theta_rad))
        if state in ("paused", "aborted", "error", "done") and detail:
            set_text(self.error_lbl, detail if state != "done" else "")
        elif state in ("running", "ready", "loaded"):
            set_text(self.error_lbl, "")
        self._last_state = state
        self._update_enables()
        self._show_pending()

    def _show_pending(self) -> None:
        c, seq = self._ctl, self._ctl.sequencer
        held = seq is not None and seq.held is not None
        text = ""
        if c.state == "pausing":
            text = "pause requested" + (" - after this place" if held else "")
        elif c.state == "aborting":
            text = "abort requested" + (" - after this place" if held else "")
        set_text(self.pending_lbl, text)

    def _on_snapshot(self, snap) -> None:
        self._snap = snap
        if snap is None:
            return
        k = snap.stop_k
        if k is not None and self._ctl.session is not None:
            st = self._ctl.session.job.stops[k]
            set_text(self.stop_lbl, stop_text(k, snap.n_stops) + (f"  leg {st.leg}" if st.leg else "")
                     + f", a {st.a_mm:.0f} mm, done {len(snap.stops_done)}")
        else:
            set_text(self.stop_lbl, stop_text(None, snap.n_stops))
        set_text(self.stone_lbl, (snap.stone.describe(snap.n_stones) if snap.stone else "-")
                 + f"  | placed {snap.n_placed}/{snap.n_stones}")
        set_text(self.action_lbl, snap.action)
        trip = "at the station" if snap.at_station else "-"
        if snap.route:
            r = snap.route
            trip = f"route {r['kind']} (stop {r['stop']}), leg {min(r['next_leg'] + 1, r['n_legs'])}/{r['n_legs']}"
        set_text(self.trip_lbl, f"{trip}, reloads {snap.reloads}")
        self._show_pose(snap)
        if snap.held:
            h = snap.held
            set_text(self.held_lbl, f"{h.get('kind')} stone from {h.get('from')} slot {h.get('slot')}"
                     + (" - UNKNOWN (robot error)" if h.get("unknown") else ""))
            set_style(self.held_lbl, "color:#FF6666;" if h.get("unknown") else "color:#FFAA00;")
        else:
            set_text(self.held_lbl, "empty")
            set_style(self.held_lbl, "color:#AAAAAA;")
        world = self._ctl.rig.world if self._ctl.rig is not None else None
        self.plan.set_snapshot(snap)
        self.plan.set_live(ares_live(snap, self._ctx.last_ads_status, world))
        self._show_pending()
        if self._ctl.state in ("paused", "aborted", "error"):
            self._update_enables()                       # recovery buttons follow held / pose status

    def _show_pose(self, snap, live=None) -> None:
        p = live.pose if live is not None and live.pose is not None else snap.pose_est
        src = live.src if live is not None else snap.pose_src
        if p is None:
            set_text(self.pose_lbl, "unknown")
        else:
            set_text(self.pose_lbl, f"x {p.x_mm:.0f} mm, y {p.y_mm:.0f} mm, {math.degrees(p.theta_rad):+.2f} deg "
                                    f"({src}, {snap.pose_status})")
        set_style(self.pose_lbl, f"color:{STATUS_COLOURS.get(snap.pose_status, '#FF4444')};")

    def _on_ads_status(self, d: dict) -> None:
        """REAL: the live pose follows the HMI's odometry during an ARES command; Browse / Apply odometry enables."""
        snap = self._snap
        if snap is not None and snap.ares_cmd is not None and self._ctl.mode == "real" and self.isVisible():
            live = ares_live(snap, d, None)
            self.plan.set_live(live)
            self._show_pose(snap, live)
        if self._ctl.state in ("empty", "loaded", "paused", "aborted", "error"):
            self._update_enables()

    def _on_preflight(self, rep) -> None:
        set_text(self.preflight_hdr, rep.header())
        set_style(self.preflight_hdr, f"font-weight:bold; color:{'#44CC44' if rep.ok else '#FF5555'};")
        self.preflight_list.clear()
        for it in rep.items:
            w = QListWidgetItem(f"[{it.source}] {it.text}")
            w.setForeground(QColor("#FF6666" if it.blocking else "#999999"))
            self.preflight_list.addItem(w)
