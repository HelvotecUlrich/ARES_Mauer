"""UR panel (docs/HMI_DESIGN.md section 11.2): what the UR5 is doing now. It shows the connection (RTDE sample age,
controller version, missing recipe fields), the robot / safety / runtime state, the joints, the TCP, the gripper
outputs, the COMMANDED payload, the held stone, the last robot action and the last error.

Read-only (D-H3). There is one UrSource on the current rig, polled in the GUI thread at [hmi] ur_poll_hz by the
UrPoller that the UR tab and the status strip share. REAL reads URLink.state(), which is lock-protected (no second
RTDE client); SIM reads the simulated robot. Nothing here writes to the robot.
"""
from __future__ import annotations

import logging
import math
import time

import numpy as np
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QWidget

from mauer.geometry import R_to_rotvec

from ..amr.ui.widgets import GROUP_CSS, lbl, num_label, set_style, set_text
from ..core.preflight import RTDE_MAX_AGE_S
from ..core.sources import UrSnapshot

log = logging.getLogger("hmi.ur")

JOINTS = ("base", "shoulder", "elbow", "wrist 1", "wrist 2", "wrist 3")
OK, WARN, BAD, GREY, INFO, TEXT = "#44CC44", "#FFAA00", "#FF4444", "#888888", "#66CCFF", "#DDDDDD"
SAFETY_COLOURS = {"NORMAL": OK, "REDUCED": WARN}           # every other safety mode is a stop: red
DO_NOTE = "pulses - not a held-state signal"
PAYLOAD_NOTE = "commanded (no read-back)"


def safety_colour(mode: str | None) -> str:
    return GREY if mode is None else SAFETY_COLOURS.get(mode, BAD)


def robot_mode_colour(mode: str | None) -> str:
    """RUNNING green; POWER_ON / IDLE (brakes still applied) amber; anything else red."""
    if mode is None:
        return GREY
    return OK if mode == "RUNNING" else WARN if mode in ("POWER_ON", "IDLE") else BAD


def ur_short(u: UrSnapshot | None) -> tuple[str, str]:
    """(text, colour) of the UR in one line (status strip)."""
    if u is None or u.source == "none":
        return "UR: no rig", GREY
    if u.source == "sim":
        return "UR: SIM", INFO
    if u.robot_mode is None:
        return "UR: no RTDE sample" + (f" ({u.rtde_error})" if u.rtde_error else ""), BAD
    text = f"UR: {u.robot_mode} / {u.safety_mode}" + (" / program" if u.program_running else "")
    if u.age_s is None or u.age_s > RTDE_MAX_AGE_S:
        return text + " (RTDE stale)", BAD
    if not u.safety_ok:
        return text, BAD
    return text, OK if u.robot_mode == "RUNNING" else WARN


def _onoff(v: bool | None) -> tuple[str, str]:
    return ("-", GREY) if v is None else ("ON", OK) if v else ("off", GREY)


def _yesno(v: bool | None) -> str:
    return "-" if v is None else "yes" if v else "no"


def _clock(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t))


class UrPoller(QObject):
    """The context's ONE UrSource, polled in the GUI thread at [hmi] ur_poll_hz; `updated` carries each UrSnapshot.
    It gets a new source on every rig or session change. UrPoller.of(ctx) creates it on first use, as a child of
    the context, and stops it with the context's shutdown hooks."""

    updated = Signal(object)                  # UrSnapshot

    def __init__(self, ctx, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._ctl = ctx.controller
        self._src = self._ctl.ur_source()
        self.last: UrSnapshot | None = None
        self.error = ""                       # last failed snapshot (the previous one stays shown)
        hz = float(ctx.hmi.get("ur_poll_hz", 10.0))
        self._timer = QTimer(self)
        self._timer.setInterval(max(20, int(round(1000.0 / max(hz, 0.1)))))
        self._timer.timeout.connect(self.poll)
        self._ctl.rig_changed.connect(self._renew)
        self._ctl.session_loaded.connect(self._renew)
        ctx.add_shutdown_hook(self._timer.stop)
        self._timer.start()
        self.poll()                           # `last` is valid at once for the widgets created after it

    @classmethod
    def of(cls, ctx) -> "UrPoller":
        p = ctx.findChild(cls)
        return p if p is not None else cls(ctx, parent=ctx)

    @property
    def interval_ms(self) -> int:
        return self._timer.interval()

    def _renew(self, *_a) -> None:
        self._src = self._ctl.ur_source()
        self.poll()

    def poll(self) -> None:
        try:
            snap = self._src.snapshot()
        except Exception as e:      # noqa: BLE001 - SIM: the run thread rebinds the robot state while it is read
            self.error = f"{type(e).__name__}: {e}"
            log.debug("UR snapshot failed: %s", self.error)
            return
        self.error = ""
        self.last = snap
        self.updated.emit(snap)


class UrPanel(QWidget):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self._last_robot: dict | None = None  # last 'robot' record
        self._last_error: dict | None = None  # last 'robot_error' record
        self._build_ui()
        self.setStyleSheet(GROUP_CSS)
        self.poller = UrPoller.of(ctx)
        self.poller.updated.connect(self._on_ur)
        self._ctl.event.connect(self._on_event)
        self._ctl.snapshot_changed.connect(lambda _s: self.isVisible() and self._render_run())
        self._ctl.rig_changed.connect(self._on_rig)

    # ── construction ─────────────────────────────────────────────────────────
    def _build_ui(self) -> None:
        g = QGridLayout(self)
        g.setContentsMargins(6, 6, 6, 6)
        g.addWidget(self._conn_group(), 0, 0)
        g.addWidget(self._state_group(), 1, 0)
        g.addWidget(self._tool_group(), 2, 0)
        g.addWidget(self._joint_group(), 0, 1)
        g.addWidget(self._tcp_group(), 1, 1)
        g.addWidget(self._run_group(), 2, 1)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        g.setRowStretch(3, 1)

    @staticmethod
    def _value(wrap: bool = False) -> QLabel:
        w = QLabel("-")
        set_style(w, f"color:{TEXT};")
        w.setWordWrap(wrap)
        w.setTextInteractionFlags(Qt.TextSelectableByMouse)
        return w

    def _conn_group(self) -> QGroupBox:
        box = QGroupBox("Connection")
        f = QFormLayout(box)
        self.source_lbl, self.age_lbl, self.version_lbl = self._value(), self._value(), self._value()
        self.missing_lbl, self.rtde_err_lbl = self._value(True), self._value(True)
        f.addRow("source", self.source_lbl)
        f.addRow("RTDE sample age", self.age_lbl)
        f.addRow("controller", self.version_lbl)
        f.addRow("missing RTDE fields", self.missing_lbl)
        f.addRow("RTDE error", self.rtde_err_lbl)
        return box

    def _state_group(self) -> QGroupBox:
        box = QGroupBox("Robot state")
        f = QFormLayout(box)
        self.robot_mode_lbl, self.safety_lbl, self.runtime_lbl = self._value(), self._value(), self._value()
        self.power_lbl, self.program_lbl = self._value(), self._value()
        f.addRow("robot mode", self.robot_mode_lbl)
        f.addRow("safety mode", self.safety_lbl)
        f.addRow("runtime state", self.runtime_lbl)
        f.addRow("power on", self.power_lbl)
        f.addRow("program running", self.program_lbl)
        return box

    def _joint_group(self) -> QGroupBox:
        box = QGroupBox("Joints [deg]")
        f = QFormLayout(box)
        self.joint_lbls = [num_label() for _ in JOINTS]
        for name, w in zip(JOINTS, self.joint_lbls):
            f.addRow(name, w)
        return box

    def _tcp_group(self) -> QGroupBox:
        box = QGroupBox("TCP in the UR base frame")
        g = QGridLayout(box)
        self.tcp_lbls = [num_label() for _ in range(6)]
        for i, (name, unit) in enumerate((("x", "mm"), ("y", "mm"), ("z", "mm"), ("rx", "deg"), ("ry", "deg"),
                                          ("rz", "deg"))):
            row, col = i % 3, 3 * (i // 3)
            g.addWidget(lbl(name), row, col)
            g.addWidget(self.tcp_lbls[i], row, col + 1)
            g.addWidget(lbl(unit), row, col + 2)
        g.addWidget(lbl("rx, ry, rz: rotation vector (axis x angle)", "#777777"), 3, 0, 1, 6)
        return box

    def _tool_group(self) -> QGroupBox:
        box = QGroupBox("Gripper and payload")
        f = QFormLayout(box)
        u = self._ctx.station_cfg.get("ur", {})
        self.do_open_lbl, self.do_close_lbl = self._value(), self._value()
        self.voltage_lbl, self.payload_lbl = self._value(), self._value(True)
        dos = QHBoxLayout()
        dos.addWidget(lbl(f"open DO{u.get('do_grip_open', '?')}"))
        dos.addWidget(self.do_open_lbl)
        dos.addWidget(lbl(f"close DO{u.get('do_grip_close', '?')}"))
        dos.addWidget(self.do_close_lbl)
        dos.addStretch()
        f.addRow("outputs", dos)
        f.addRow(lbl(DO_NOTE, "#777777"))
        f.addRow("tool voltage", self.voltage_lbl)
        f.addRow("payload", self.payload_lbl)
        f.addRow(lbl(PAYLOAD_NOTE, "#777777"))
        return box

    def _run_group(self) -> QGroupBox:
        box = QGroupBox("Run")
        f = QFormLayout(box)
        self.held_lbl, self.parked_lbl = self._value(True), self._value()
        self.action_lbl, self.error_lbl = self._value(True), self._value(True)
        f.addRow("stone in the jaws", self.held_lbl)
        f.addRow("parked", self.parked_lbl)
        f.addRow("last robot action", self.action_lbl)
        f.addRow("last error", self.error_lbl)
        return box

    # ── updates ──────────────────────────────────────────────────────────────
    def showEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().showEvent(event)
        self.poller.poll()                    # a fresh sample, not one up to a poll interval old
        self.refresh()

    def refresh(self) -> None:
        """Render the latest UR snapshot and run data (also while hidden)."""
        self._render_ur(self.poller.last)
        self._render_run()

    def _on_ur(self, u: UrSnapshot) -> None:
        if self.isVisible():                  # the strip shows the UR state while this tab is hidden
            self._render_ur(u)
            self._render_run()

    def _on_event(self, rec) -> None:
        ev = rec.get("event")
        if ev == "robot":
            self._last_robot = rec
        elif ev == "robot_error":
            self._last_error = rec

    def _on_rig(self, rig) -> None:
        if rig is not None:                   # a new rig = a new run log
            self._last_robot, self._last_error = None, None
            if self.isVisible():
                self._render_run()

    def _render_ur(self, u: UrSnapshot | None) -> None:
        if u is None:
            return
        set_text(self.source_lbl, {"rtde": "RTDE (URLink, read-only)", "sim": "SIM - no RTDE",
                                   "none": "no rig (Prepare / Connect)"}.get(u.source, u.source))
        stale = u.source == "rtde" and (u.age_s is None or u.age_s > RTDE_MAX_AGE_S)
        set_text(self.age_lbl, "-" if u.age_s is None else f"{u.age_s:.2f} s" + (" - STALE" if stale else ""))
        set_style(self.age_lbl, f"color:{BAD if stale else TEXT};")
        v = u.controller_version
        set_text(self.version_lbl, ".".join(str(x) for x in v) if v else "-")
        set_style(self.version_lbl, f"color:{WARN if v and tuple(v[:2]) == (3, 3) else TEXT};")   # IK guard
        set_text(self.missing_lbl, ", ".join(u.missing_fields) if u.missing_fields else
                 "none" if u.source == "rtde" else "-")
        set_style(self.missing_lbl, f"color:{WARN if u.missing_fields else TEXT};")
        set_text(self.rtde_err_lbl, u.rtde_error or "-")
        set_style(self.rtde_err_lbl, f"color:{BAD if u.rtde_error else TEXT};")
        # state
        set_text(self.robot_mode_lbl, u.robot_mode or "-")
        set_style(self.robot_mode_lbl, f"font-weight:bold; color:{robot_mode_colour(u.robot_mode)};")
        set_text(self.safety_lbl, u.safety_mode or "-")
        set_style(self.safety_lbl, f"font-weight:bold; color:{safety_colour(u.safety_mode)};")
        set_text(self.runtime_lbl, u.runtime_state or "-")
        set_text(self.power_lbl, _yesno(u.power_on))
        set_text(self.program_lbl, _yesno(u.program_running))
        set_style(self.program_lbl, f"color:{INFO if u.program_running else TEXT};")
        # joints and TCP
        q = u.q_rad if u.q_rad is not None else (None,) * len(JOINTS)
        for w, v_rad in zip(self.joint_lbls, q):
            set_text(w, "-" if v_rad is None else f"{math.degrees(v_rad):+.2f}")
        if u.T_base_tcp_mm is None:
            for w in self.tcp_lbls:
                set_text(w, "-")
        else:
            T = np.asarray(u.T_base_tcp_mm, float)
            for i, v in enumerate((*T[:3, 3], *np.degrees(R_to_rotvec(T[:3, :3])))):
                set_text(self.tcp_lbls[i], f"{v:+.1f}" if i < 3 else f"{v:+.2f}")
        # gripper and payload
        for w, val in ((self.do_open_lbl, u.do_open), (self.do_close_lbl, u.do_close)):
            text, col = _onoff(val)
            set_text(w, text)
            set_style(w, f"font-weight:bold; color:{col};")
        set_text(self.voltage_lbl, "-" if u.tool_voltage_v is None else f"{u.tool_voltage_v} V")
        set_text(self.payload_lbl, self._payload_text(u))
        set_text(self.parked_lbl, _yesno(u.parked))

    def _payload_text(self, u: UrSnapshot) -> str:
        if u.payload_kg is None:
            return "- (SIM: no payload)" if u.source == "sim" else "-"
        cog = ", ".join(f"{c:.0f}" for c in (u.payload_cog_mm or []))
        text = f"tool {u.payload_kg:.2f} kg, CoG ({cog}) mm"
        robot = self._ctl.rig.robot if self._ctl.rig is not None else None
        kind, loads = getattr(robot, "holding", None), getattr(robot, "payloads", None) or {}
        if kind and kind in loads:            # URRobot commands the stone payload while it holds one
            text += f"; with the {kind} stone {float(loads[kind][0]):.2f} kg"
        return text

    def _render_run(self) -> None:
        snap = self._ctl.snapshot
        held = snap.held if snap is not None else None
        u = self.poller.last
        if held:
            set_text(self.held_lbl, f"{held.get('kind')} stone from {held.get('from')} slot {held.get('slot')}"
                     + (" - UNKNOWN (robot error)" if held.get("unknown") else ""))
            set_style(self.held_lbl, f"color:{BAD if held.get('unknown') else WARN};")
        else:
            sim_kind = u.held_kind if u is not None and u.source == "sim" else None
            set_text(self.held_lbl, f"{sim_kind} stone (SIM jaws)" if sim_kind else "empty")
            set_style(self.held_lbl, f"color:{WARN if sim_kind else TEXT};")
        r = self._last_robot
        set_text(self.action_lbl, f"{_clock(r['t'])}  {r.get('action')}: {r.get('what')}" if r else "-")
        e = self._last_error
        text = f"{_clock(e['t'])}  {e.get('action')}: {e.get('error')}" if e else ""
        if self.poller.error:
            text = (text + "\n" if text else "") + f"UR state not readable: {self.poller.error}"
        set_text(self.error_lbl, text or "-")
        set_style(self.error_lbl, f"color:{BAD if text else TEXT};")
