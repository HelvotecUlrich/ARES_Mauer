"""Wall pose tab (docs/HMI_DESIGN.md section 11.2): where ARES is in the wall frame and where it is going.

Left, a PlanView with the live pose. Right, the numbers:
- the sequencer's pose estimate (source, status), the live pose during a REAL move (estimate + the HMI's odometry
  since the command), and in SIM the true pose with the error of the estimate;
- the target: the current stop's nominal pose, the route leg being driven, the dock during a station trip, the ARES
  command in progress;
- the PLC wheel odometry next to it (REAL: the HMI's ADS status; SIM: the simulated odometry), the AMR state;
- the last ARES move, the move and correction counters, the station location estimate vs nominal;
- a table of the camera fits per stop (wall and station).

Stops are named by their run-log index (0-based, as in the plan view and the Mauer tab's route line).
Read-only: RunController signals, ctx.ads_status and the SIM world's attributes. Skips work while hidden.
"""
from __future__ import annotations

import math
import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QScrollArea, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from mauer.reference import Pose2D
from mauer.sequencer import SequencerParams

from ..amr.ui import constants as C
from ..amr.ui.widgets import GROUP_CSS, lbl, set_style, set_text
from ..core.sources import AresLive, ares_live, odom_from_status
from .plan_view import STATUS_COLOURS, PlanView

OK, WARN, BAD, GREY, INFO, TEXT = "#44CC44", "#FFAA00", "#FF4444", "#888888", "#66CCFF", "#DDDDDD"
RIGHT_W = 430
FIT_COLUMNS = ("stop", "kind", "why", "rms mm", "err mm", "err deg", "jump mm")
ODOM_NOTE = "PLC wheel odometry since its last reset - not a localisation (slip is invisible)"


def pose_text(p: Pose2D | None) -> str:
    return "-" if p is None else f"x {p.x_mm:.1f} mm, y {p.y_mm:.1f} mm, {p.theta_deg:+.2f} deg"


def delta_text(a: Pose2D | None, b: Pose2D | None) -> str:
    """Distance and heading difference from a to b."""
    if a is None or b is None:
        return "-"
    d_mm, d_deg = a.delta(b)
    return f"{d_mm:.1f} mm, {d_deg:.2f} deg"


def target_of(snap, job) -> tuple[str, Pose2D | None]:
    """(what, pose) ARES is heading for: the dock during a station trip (current station estimate), else the
    current stop's nominal pose (the sequencer measures and corrects towards it)."""
    if snap is None or job is None:
        return "-", None
    route = snap.route or {}
    to_dock = snap.at_station or route.get("kind") == "to_station"
    if to_dock:
        T = snap.T_wall_station @ job.station.dock.T
        return "dock (station estimate)", Pose2D.from_T(T)
    k = snap.stop_k
    if k is None:                                        # before the run: the stop whose start mark ARES is on
        k = next((i for i, s in enumerate(job.stops) if s.ares == snap.pose_est), None)
    if k is None or not 0 <= k < len(job.stops):
        return "-", None
    st = job.stops[k]
    return f"stop {k}" + (f" (leg {st.leg})" if st.leg else "") + " nominal", st.ares


def route_text(snap) -> str:
    r = snap.route if snap is not None else None
    if not r:
        return "at the station" if snap is not None and snap.at_station else "-"
    what = {"stop": f"to stop {r['stop']}", "to_station": f"to the station (from stop {r['stop']})",
            "from_station": f"back to stop {r['stop']}"}.get(r["kind"], r["kind"])
    return f"{what}: leg {min(r['next_leg'] + 1, r['n_legs'])}/{r['n_legs']} ({r['why']})"


def command_text(cmd: dict | None) -> str:
    if not cmd:
        return "-"
    a = list(cmd.get("args") or [])
    if cmd.get("kind") == "rotate" and a:
        args = f"{a[0]:+.2f} deg"
    elif cmd.get("kind") == "translate" and len(a) >= 2:
        args = f"dx {a[0]:+.1f} mm, dy {a[1]:+.1f} mm"
    else:
        args = ", ".join(str(x) for x in a)
    return f"{cmd.get('kind')} {args} ({cmd.get('why')})"


class AresPanel(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._last_move: dict | None = None
        self._station_est: dict | None = None
        self._build_ui()
        self.setStyleSheet(GROUP_CSS)
        c = self._ctl
        c.snapshot_changed.connect(self._on_snapshot)
        c.event.connect(self._on_event)
        c.rig_changed.connect(self._on_rig)
        ctx.session_changed.connect(self._on_session)
        ctx.ads_status.connect(self._on_ads_status)
        if c.session is not None:
            self._on_session(c.session)

    # ── construction ─────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        self.plan = PlanView()
        outer.addWidget(self.plan, 1)
        right = QWidget()
        col = QVBoxLayout(right)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self._pose_group())
        col.addWidget(self._target_group())
        col.addWidget(self._odom_group())
        col.addWidget(self._moves_group())
        col.addWidget(self._fits_group(), 1)
        scroll = QScrollArea()
        scroll.setWidget(right)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(RIGHT_W + 24)
        scroll.setFocusPolicy(Qt.NoFocus)
        outer.addWidget(scroll)

    @staticmethod
    def _value() -> QLabel:
        w = QLabel("-")
        w.setWordWrap(True)
        set_style(w, f"color:{TEXT};")
        w.setTextInteractionFlags(Qt.TextSelectableByMouse)
        return w

    def _pose_group(self) -> QGroupBox:
        box = QGroupBox("ARES pose in the wall frame")
        f = QFormLayout(box)
        self.est_lbl, self.est_src_lbl, self.live_lbl = self._value(), self._value(), self._value()
        self.truth_lbl, self.truth_err_lbl = self._value(), self._value()
        f.addRow("estimate", self.est_lbl)
        f.addRow("source", self.est_src_lbl)
        f.addRow("live (move)", self.live_lbl)
        f.addRow("SIM truth", self.truth_lbl)
        f.addRow("estimate error", self.truth_err_lbl)
        return box

    def _target_group(self) -> QGroupBox:
        box = QGroupBox("Target")
        f = QFormLayout(box)
        self.target_lbl, self.target_pose_lbl, self.to_target_lbl = self._value(), self._value(), self._value()
        self.route_lbl, self.cmd_lbl = self._value(), self._value()
        f.addRow("going to", self.target_lbl)
        f.addRow("target pose", self.target_pose_lbl)
        f.addRow("estimate -> target", self.to_target_lbl)
        f.addRow("route", self.route_lbl)
        f.addRow("ARES command", self.cmd_lbl)
        return box

    def _odom_group(self) -> QGroupBox:
        box = QGroupBox("PLC odometry")
        f = QFormLayout(box)
        self.odom_lbl, self.amr_lbl = self._value(), self._value()
        f.addRow("odometry", self.odom_lbl)
        f.addRow("AMR state", self.amr_lbl)
        f.addRow(lbl(ODOM_NOTE, "#777777"))
        return box

    def _moves_group(self) -> QGroupBox:
        box = QGroupBox("Moves and station")
        f = QFormLayout(box)
        self.counts_lbl, self.last_move_lbl, self.station_lbl = self._value(), self._value(), self._value()
        f.addRow("counters", self.counts_lbl)
        f.addRow("last move", self.last_move_lbl)
        f.addRow("station vs nominal", self.station_lbl)
        return box

    def _fits_group(self) -> QGroupBox:
        box = QGroupBox("Camera fits")
        v = QVBoxLayout(box)
        self.fits = QTableWidget(0, len(FIT_COLUMNS))
        self.fits.setHorizontalHeaderLabels(list(FIT_COLUMNS))
        self.fits.verticalHeader().setVisible(False)
        self.fits.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.fits.setSelectionMode(QAbstractItemView.NoSelection)
        self.fits.setFocusPolicy(Qt.NoFocus)
        self.fits.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.fits.setMinimumHeight(120)
        v.addWidget(self.fits)
        return box

    # ── signals ──────────────────────────────────────────────────────────────
    def _world(self):
        rig = self._ctl.rig
        return rig.world if rig is not None and rig.mode == "sim" else None

    def _on_session(self, s) -> None:
        self.plan.set_session(s)
        self._clear_run()
        self.refresh()

    def _on_rig(self, rig) -> None:
        if rig is not None:                   # a new rig = a new run log
            self._clear_run()

    def _clear_run(self) -> None:
        self._last_move, self._station_est = None, None
        self.fits.setRowCount(0)

    def _on_event(self, rec) -> None:
        ev = rec.get("event")
        if ev == "ares_move":
            self._last_move = rec
        elif ev == "station_estimate":
            self._station_est = rec
        elif ev in ("wall_frame", "station_frame") and rec.get("source") == "camera":
            self._add_fit(rec)

    def _add_fit(self, rec: dict) -> None:
        def num(key: str, fmt: str) -> str:
            v = rec.get(key)
            return "-" if v is None else format(float(v), fmt)
        row = self.fits.rowCount()
        self.fits.insertRow(row)
        cells = (str(rec.get("stop", "-")), str(rec.get("kind", "-")), str(rec.get("why", "-")),
                 num("rms_mm", ".2f"), num("err_nominal_mm", ".1f"), num("err_nominal_deg", ".2f"),
                 num("jump_mm", ".1f"))
        for i, text in enumerate(cells):
            it = QTableWidgetItem(text)
            it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter if i >= 3 else Qt.AlignLeft | Qt.AlignVCenter)
            self.fits.setItem(row, i, it)
        self.fits.scrollToBottom()

    def _on_snapshot(self, _snap) -> None:
        if self.isVisible():
            self.refresh()

    def _on_ads_status(self, d: dict) -> None:
        if not self.isVisible():
            return
        snap = self._ctl.snapshot
        if snap is not None and snap.ares_cmd is not None and self._ctl.mode == "real":
            self._show_live(ares_live(snap, d, None))     # the live pose follows the HMI's odometry
        self._show_odom(d, None)

    def showEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().showEvent(event)
        self.refresh()

    # ── display ──────────────────────────────────────────────────────────────
    def refresh(self) -> None:
        """Everything from the latest snapshot, ADS status and SIM world (also while hidden)."""
        snap, world = self._ctl.snapshot, self._world()
        live = ares_live(snap, self._ctx.last_ads_status, world)
        self.plan.set_snapshot(snap)
        self._show_live(live)
        self._show_estimate(snap, live)
        self._show_target(snap)
        self._show_odom(self._ctx.last_ads_status, world)
        self._show_moves(snap)

    def _show_live(self, live: AresLive) -> None:
        self.plan.set_live(live)
        moving = live.src == "estimate+odometry" and live.pose is not None
        set_text(self.live_lbl, pose_text(live.pose) + " (estimate + odometry since the command)" if moving else "-")
        set_style(self.live_lbl, f"color:{INFO if moving else TEXT};")

    def _show_estimate(self, snap, live: AresLive) -> None:
        if snap is None:
            for w in (self.est_lbl, self.est_src_lbl):
                set_text(w, "-")
        else:
            set_text(self.est_lbl, pose_text(snap.pose_est))
            set_style(self.est_lbl, f"color:{STATUS_COLOURS.get(snap.pose_status, BAD)}; font-weight:bold;")
            set_text(self.est_src_lbl, f"{snap.pose_src}, status {snap.pose_status}")
        if live.true_pose is None:
            set_text(self.truth_lbl, "- (REAL: no ground truth)" if self._ctl.mode == "real" else "-")
            set_text(self.truth_err_lbl, "-")
            set_style(self.truth_err_lbl, f"color:{TEXT};")
            return
        set_text(self.truth_lbl, pose_text(live.true_pose))
        est = snap.pose_est if snap is not None else None
        set_text(self.truth_err_lbl, delta_text(live.true_pose, est))
        err_mm = live.true_pose.delta(est)[0] if est is not None else 0.0
        s = self._ctl.session                 # amber beyond the error that would trigger a correction move
        tol = SequencerParams.from_config(s.cfg).stop_tol_mm if s is not None else math.inf
        set_style(self.truth_err_lbl, f"color:{WARN if err_mm > tol else TEXT};")

    def _show_target(self, snap) -> None:
        job = self._ctl.session.job if self._ctl.session is not None else None
        what, target = target_of(snap, job)
        set_text(self.target_lbl, what)
        set_text(self.target_pose_lbl, pose_text(target))
        set_text(self.to_target_lbl, delta_text(snap.pose_est if snap is not None else None, target))
        set_text(self.route_lbl, route_text(snap))
        cmd = snap.ares_cmd if snap is not None else None
        set_text(self.cmd_lbl, command_text(cmd))
        set_style(self.cmd_lbl, f"color:{INFO if cmd else TEXT};")

    def _show_odom(self, ads_status, world) -> None:
        if world is not None:
            o = world.ares.odom
            set_text(self.odom_lbl, f"x {o.x_mm:.1f} mm, y {o.y_mm:.1f} mm, {o.theta_deg:+.2f} deg (SIM)")
            set_text(self.amr_lbl, "SIM")
            set_style(self.amr_lbl, f"color:{INFO};")
            return
        o = odom_from_status(ads_status)
        set_text(self.odom_lbl, "-" if o is None else f"x {o.x_mm:.1f} mm, y {o.y_mm:.1f} mm, {o.theta_deg:+.2f} deg")
        if not self._ctx.ares_enabled:
            set_text(self.amr_lbl, "ADS off (start the HMI with --ares)")
            set_style(self.amr_lbl, f"color:{GREY};")
        elif not ads_status:
            set_text(self.amr_lbl, "not connected")
            set_style(self.amr_lbl, f"color:{BAD};")
        else:
            colour, name = C.state_info(ads_status.get("eAmrState", -1))
            move = ads_status.get("bMoveActive")
            set_text(self.amr_lbl, name + (" - move active" if move else ""))
            set_style(self.amr_lbl, f"color:{INFO if move else colour}; font-weight:bold;")

    def _show_moves(self, snap) -> None:
        set_text(self.counts_lbl, "-" if snap is None else f"{snap.ares_moves} ARES moves, {snap.corrections} "
                                                           f"corrections")
        m = self._last_move
        if m is None:
            set_text(self.last_move_lbl, "-")
            set_style(self.last_move_lbl, f"color:{TEXT};")
        else:
            clock = time.strftime("%H:%M:%S", time.localtime(float(m.get("t", 0.0) or 0.0)))
            set_text(self.last_move_lbl, f"{clock} {m.get('kind')} {'ok' if m.get('ok') else 'NOT ok'}: "
                                         f"{m.get('summary')} ({m.get('why')})")
            set_style(self.last_move_lbl, f"color:{TEXT if m.get('ok') else BAD};")
        s = self._station_est
        if s is None:
            set_text(self.station_lbl, "nominal (not measured yet)")
        else:
            set_text(self.station_lbl, f"{float(s.get('vs_nominal_mm', math.nan)):.1f} mm, "
                                       f"{float(s.get('vs_nominal_deg', math.nan)):.2f} deg "
                                       f"({s.get('n')} observations)")
