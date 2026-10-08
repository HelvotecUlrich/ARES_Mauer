"""Run feedback in the Mauer tab (docs/HMI_DESIGN.md section 11.2): a colour-coded status board of the current run.

Tiles: stones placed / total, stop / leg / course, station trips and refills done / planned, ARES moves and
corrections, the camera fits, elapsed and running time with seconds per stone, the last preflight. Below them, the
warnings and errors of the run with their time, and the SIM placement statistics once a SIM run has ended.
"Open summary" opens the run's hmi_summary.json.

Accumulated from RunController signals in the GUI thread (event, snapshot_changed, preflight_done, state_changed,
message); a new rig starts a new board. planned_trips() is pure: the sequencer's magazine bookkeeping without motion.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Mapping

from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtGui import QColor, QDesktopServices
from PySide6.QtWidgets import QGridLayout, QGroupBox, QLabel, QListWidget, QListWidgetItem, QVBoxLayout, QWidget

from mauer.job import Job, SlotState, reload_plan, reload_short

from ..amr.ui.widgets import GROUP_CSS, PulseBtn, set_style, set_text
from ..core.run_controller import END_STATES, RUNNING
from ..core.snapshot import format_event, severity, stop_text

OK, WARN, BAD, GREY, TEXT = "#44CC44", "#FFAA00", "#FF4444", "#999999", "#DDDDDD"
ISSUE_COLOURS = {"warning": WARN, "error": BAD}
MAX_ISSUES = 500                              # list entries kept (oldest dropped)
TILES = ("stones", "stop", "station", "ares", "camera", "time", "preflight", "issues")
TILE_COLS = 4                                 # tiles per row (the board sits under the plan view of the Mauer tab)


def planned_trips(job: Job, start_stop: int = 0, stop_after: int | None = None) -> tuple[int, int]:
    """(station trips, station refills) a run of the stops start_stop..stop_after needs: the sequencer's own
    bookkeeping (Sequencer._place / _reload) without motion, from the planned magazine fill and a full station.
    Over the whole job it equals tools/make_job.py meta planned_reloads / planned_station_refills."""
    n = len(job.stops)
    last = n - 1 if stop_after is None else min(int(stop_after), n - 1)
    mag, full = SlotState.magazine(job.magazine), SlotState.station(job.station)
    st, placed = full.copy(), set()
    trips = refills = 0
    for k in range(max(int(start_stop), 0), last + 1):
        for t in job.stops[k].stones:
            if mag.empty():
                trips += 1
                upcoming = [x.kind for x in job.stones() if x.key not in placed]
                if st.empty() or reload_short(st, full, mag, upcoming):
                    st = full.copy()
                    refills += 1
                plan = reload_plan(mag, st, upcoming)
                if not plan:                  # the sequencer stops here
                    return trips, refills
                for sid, mid, kind in plan:
                    st.take(sid)
                    mag.fill(mid, kind)
            mid = mag.next_take(t.slot, kind=t.kind)
            if mid is None:
                return trips, refills
            mag.take(mid)
            placed.add(t.key)
    return trips, refills


def issue_text(rec: Mapping) -> str:
    """One line for a warning / error record of the run log."""
    ev = rec.get("event", "")
    if ev == "warning":
        return str(rec.get("msg", ""))
    if ev == "robot_error":
        return f"robot {rec.get('action')} failed: {rec.get('error')}"
    if ev == "ares_error":
        return f"ARES {rec.get('kind')} failed: {rec.get('error')}"
    if ev == "ares_move":
        return f"ARES {rec.get('kind')} not ok: {rec.get('summary')}"
    if ev == "interlock":
        return f"interlock {rec.get('what')}: {', '.join(str(p) for p in rec.get('problems') or [])}"
    if ev == "measurement_failed":
        return f"measurement failed: {rec.get('msg')}"
    if ev == "frame_jump":
        return (f"frame jump {rec.get('what')}: {float(rec.get('d_mm', 0.0)):.1f} mm / "
                f"{float(rec.get('d_deg', 0.0)):.2f} deg")
    if ev == "declined":
        return f"declined: {rec.get('what')}"
    if ev == "odometry_pose":
        return f"ARES pose from odometry after a {rec.get('kind')} that did not end ok (status {rec.get('status')})"
    if ev in ("run_paused", "run_aborted", "run_error"):
        return f"run {ev[4:]}: {rec.get('error')}"
    return format_event(rec).split("  ", 1)[-1]


def _mmss(s: float) -> str:
    s = max(0, int(round(s)))
    return f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}" if s >= 3600 else f"{s // 60}:{s % 60:02d}"


def _clock(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t))


def placement_text(p: Mapping) -> tuple[str, str]:
    """(text, colour) of SimWorld.placement_stats(): count, seated, horizontal error, pin error, per stone type."""
    n, seated = int(p.get("n", 0)), int(p.get("seated", 0))
    h, pin = p.get("horiz_mm") or {}, p.get("pin_mm") or {}
    text = f"SIM placement: {n} stones, {seated} seated"
    if h.get("n"):
        text += f"; horizontal error mean {h['mean']:.2f} / p95 {h['p95']:.2f} / max {h['max']:.2f} mm"
    if pin.get("n"):
        text += f"; pin max {pin['max']:.2f} mm"
    kinds = [f"{k} {d['n']} ({d['seated']} seated)" for k, d in (p.get("by_kind") or {}).items() if d.get("n")]
    if kinds:
        text += "; " + ", ".join(kinds)
    return text, OK if n and seated == n else BAD if n else GREY


class RunFeedback(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._build_ui()
        self.setStyleSheet(GROUP_CSS)
        self._planned: tuple[int, int] | None = None
        self._summary_path: Path | None = None
        self._reset()
        self._clock = QTimer(self)                  # elapsed time while a run is active
        self._clock.setInterval(1000)
        self._clock.timeout.connect(self._show_time)
        c = self._ctl
        c.event.connect(self._on_event)
        c.snapshot_changed.connect(self._on_snapshot)
        c.preflight_done.connect(self._on_preflight)
        c.state_changed.connect(self._on_state)
        c.rig_changed.connect(self._on_rig)
        c.session_loaded.connect(self._on_session)
        c.message.connect(self._on_message)
        if c.session is not None:
            self._on_session(c.session)

    # ── construction ─────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        box = QGroupBox("Run feedback")
        v = QVBoxLayout(box)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        self.tiles: dict[str, QLabel] = {}
        names = {"stones": "stones", "stop": "stop / leg / course", "station": "station trips (refills)",
                 "ares": "ARES", "camera": "camera fits", "time": "time", "preflight": "preflight",
                 "issues": "warnings / errors"}
        for i, key in enumerate(TILES):
            title = QLabel(names[key])
            title.setStyleSheet("color:#777777; font-size:8pt;")
            val = QLabel("-")
            val.setWordWrap(True)
            set_style(val, f"color:{TEXT}; font-weight:bold;")
            self.tiles[key] = val
            grid.addWidget(title, 2 * (i // TILE_COLS), i % TILE_COLS)
            grid.addWidget(val, 2 * (i // TILE_COLS) + 1, i % TILE_COLS)
        for c in range(TILE_COLS):
            grid.setColumnStretch(c, 1)
        v.addLayout(grid)
        self.issues = QListWidget()
        self.issues.setFocusPolicy(Qt.NoFocus)
        self.issues.setWordWrap(True)
        self.issues.setMinimumHeight(70)
        self.issues.setMaximumHeight(140)
        self.issues.setStyleSheet("QListWidget{background:#121212;}")
        v.addWidget(self.issues)
        self.sim_lbl = QLabel("")
        self.sim_lbl.setWordWrap(True)
        self.sim_lbl.hide()
        v.addWidget(self.sim_lbl)
        self.summary_btn = PulseBtn("Open summary", "#2A4A6A", min_w=120, min_h=24)
        self.summary_btn.clicked.connect(self.open_summary)
        self.summary_btn.setEnabled(False)
        v.addWidget(self.summary_btn, 0, Qt.AlignLeft)
        lay.addWidget(box)

    def _tile(self, key: str, text: str, colour: str = TEXT) -> None:
        w = self.tiles[key]
        set_text(w, text)
        set_style(w, f"color:{colour}; font-weight:bold;")

    # ── state ────────────────────────────────────────────────────────────────
    def _reset(self) -> None:
        """A new run board (new rig = new sequencer and run log); the preflight tile is kept."""
        self._t_first: float | None = None          # first run_start of this rig
        self._t_seg: float | None = None            # run_start of the active segment
        self._active_s = 0.0                        # time inside run() (all segments)
        self._t_end: float | None = None
        self._refills = 0
        self._fits: list[dict] = []
        self._n_failed = self._n_jumps = 0
        self._n_warn = self._n_err = 0
        self._snap = self._ctl.snapshot             # the initial snapshot arrives before session_loaded
        self._frozen = False                        # released after a run: the board keeps that run's results
        self.issues.clear()
        self.sim_lbl.hide()
        for key in TILES:
            if key != "preflight":
                self._tile(key, "-")
        self._show_counts()
        self._show_issues()
        self._show_time()

    def _on_session(self, s) -> None:
        self._reset()
        self._planned = self._plan(s.job, 0, None)
        self._tile("preflight", "-")
        self._summary_path = None
        self.summary_btn.setEnabled(False)
        self._show_snapshot()

    def _on_rig(self, rig) -> None:
        if rig is None:                         # released: the last run's board stays until Prepare / a new job
            self._frozen = self._t_first is not None
            return
        self._reset()
        o, s = self._ctl.opts, self._ctl.session
        if s is not None:
            self._planned = self._plan(s.job, o.start_stop if o else 0, o.stop_after if o else None)
        self._summary_path = None
        self.summary_btn.setEnabled(False)

    @staticmethod
    def _plan(job: Job, a: int, b: int | None) -> tuple[int, int] | None:
        try:
            return planned_trips(job, a, b)
        except (KeyError, ValueError, TypeError):      # an inconsistent magazine plan: shown as unknown
            return None

    # ── signals ──────────────────────────────────────────────────────────────
    def _on_event(self, rec) -> None:
        ev = rec.get("event", "")
        t = float(rec.get("t", time.time()) or time.time())
        if ev == "run_start":
            self._t_first = self._t_first if self._t_first is not None else t
            self._t_seg, self._t_end = t, None
        elif ev in ("run_done", "run_paused", "run_aborted", "run_error"):
            if self._t_seg is not None:
                self._active_s += max(0.0, t - self._t_seg)
            self._t_seg, self._t_end = None, t
        elif ev == "station_refilled":
            self._refills += 1
        elif ev in ("wall_frame", "station_frame") and rec.get("source") == "camera":
            self._fits.append(rec)
        elif ev == "measurement_failed":
            self._n_failed += 1
        elif ev == "frame_jump":
            self._n_jumps += 1
        sev = severity(rec)
        if sev != "info":
            self._add_issue(t, sev, issue_text(rec))
        if ev in ("station_refilled", "wall_frame", "station_frame", "measurement_failed", "frame_jump"):
            self._show_counts()
        if ev.startswith("run_"):
            self._show_time()

    def _on_message(self, level: str, text: str) -> None:
        if level in ISSUE_COLOURS:              # HMI-level: refusals, HALT, failed prepare
            self._add_issue(time.time(), level, f"HMI: {text}")

    def _add_issue(self, t: float, sev: str, text: str) -> None:
        if sev == "error":
            self._n_err += 1
        else:
            self._n_warn += 1
        it = QListWidgetItem(f"{_clock(t)}  {text}")
        it.setForeground(QColor(ISSUE_COLOURS.get(sev, TEXT)))
        it.setToolTip(text)
        self.issues.addItem(it)
        while self.issues.count() > MAX_ISSUES:
            self.issues.takeItem(0)
        self.issues.scrollToBottom()
        self._show_issues()

    def _on_snapshot(self, snap) -> None:
        if self._frozen:                        # the release snapshot is an initial one
            return
        self._snap = snap
        self._show_snapshot()

    def _on_preflight(self, rep) -> None:
        colour = GREY if rep.mode == "sim" else OK if rep.ok else BAD
        self._tile("preflight", f"{rep.header()} ({_clock(rep.t)})", colour)
        self.tiles["preflight"].setToolTip("\n".join(f"[{i.source}] {i.text}" for i in rep.items))

    def _on_state(self, state: str, _detail: str) -> None:
        if state in RUNNING:
            if not self._clock.isActive():
                self._clock.start()
            self.summary_btn.setEnabled(False)  # an editor in front would take Space / Esc (HALT)
        else:
            self._clock.stop()
            self.summary_btn.setEnabled(self._summary_path is not None and self._summary_path.is_file())
        if state in END_STATES:
            if self._ctl.log_dir is not None:
                p = Path(self._ctl.log_dir) / "hmi_summary.json"
                self._summary_path = p
                self.summary_btn.setEnabled(p.is_file())
            self._show_sim_placement()
        self._show_time()

    # ── display ──────────────────────────────────────────────────────────────
    def _show_snapshot(self) -> None:
        snap = self._snap
        if snap is None:
            return
        n, done = snap.n_stones, snap.n_placed
        self._tile("stones", f"{done} / {n} placed", OK if n and done >= n else TEXT)
        k = snap.stop_k
        course = f", course {snap.stone.course}" if snap.stone is not None else ""
        if k is None:
            self._tile("stop", f"{stop_text(None, snap.n_stops)}{course}", GREY)
        else:
            leg = f", leg {snap.stop_leg}" if snap.stop_leg else ""
            self._tile("stop", f"{stop_text(k, snap.n_stops)}{leg}{course}, {len(snap.stops_done)} done")
        trip_now = snap.at_station or (snap.route or {}).get("kind") in ("to_station", "from_station")
        pl = self._planned
        planned = f" / {pl[0]}" if pl else ""
        refills = f" ({self._refills}" + (f" / {pl[1]})" if pl else ")")
        self._tile("station", f"{snap.reloads}{planned}{refills}" + ("  - on a trip" if trip_now else ""),
                   WARN if trip_now else TEXT)
        self._tile("ares", f"{snap.ares_moves} moves, {snap.corrections} corrections",
                   WARN if snap.pose_status != "ok" else TEXT)
        self._show_time()

    def _show_counts(self) -> None:
        if self._fits:
            rms = [float(f["rms_mm"]) for f in self._fits if f.get("rms_mm") is not None]
            err = [float(f["err_nominal_mm"]) for f in self._fits if f.get("err_nominal_mm") is not None]
            text = f"{len(self._fits)} fits"
            if rms:
                text += f", rms mean {sum(rms) / len(rms):.2f} / max {max(rms):.2f} mm"
            if err:
                text += f", vs nominal max {max(err):.1f} mm"
        else:
            text = "no camera fit yet" if self._ctl.opts is None or self._ctl.opts.camera_loop else \
                "dead reckoning (camera loop off)"
        if self._n_failed:
            text += f", {self._n_failed} failed"
        if self._n_jumps:
            text += f", {self._n_jumps} jumps"
        self._tile("camera", text, BAD if self._n_jumps else WARN if self._n_failed else TEXT)

    def _show_issues(self) -> None:
        self._tile("issues", f"{self._n_warn} warnings, {self._n_err} errors",
                   BAD if self._n_err else WARN if self._n_warn else OK)

    def _show_time(self) -> None:
        if self._t_first is None:
            self._tile("time", "not started", GREY)
            return
        now = time.time()
        active = self._active_s + (now - self._t_seg if self._t_seg is not None else 0.0)
        elapsed = (self._t_end if self._t_seg is None and self._t_end is not None else now) - self._t_first
        n = self._snap.n_placed if self._snap is not None else 0
        per = f", {active / n:.1f} s / stone" if n else ""
        self._tile("time", f"{_mmss(elapsed)} elapsed, {_mmss(active)} running{per}")

    def _show_sim_placement(self) -> None:
        rig = self._ctl.rig
        if rig is None or rig.mode != "sim" or rig.world is None:
            return
        try:
            text, colour = placement_text(rig.world.summary()["placement"])
        except Exception as e:      # noqa: BLE001 - display only
            text, colour = f"SIM placement not available: {e}", GREY
        set_text(self.sim_lbl, text)
        set_style(self.sim_lbl, f"color:{colour};")
        self.sim_lbl.show()

    def open_summary(self) -> None:
        if self._summary_path is not None and self._summary_path.is_file():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self._summary_path)))
