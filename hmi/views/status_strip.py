"""Status strip above the tabs (docs/HMI_DESIGN.md section 11.2), one line on every tab:
run state | mode | stop k/n | stone i/n | current action | UR mode / safety | ARES PLC state (+ move) | twin state.

Sources: the RunController signals, the shared UrPoller (hmi/views/ur_panel.py), the HMI's own ADS status
(ctx.ads_status, 10 Hz) and the twin state (ctx.twin_state_changed). Read-only, colour coded.
"""
from __future__ import annotations

from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel

from ..amr.ui import constants as C
from ..amr.ui.widgets import set_style, set_text
from ..core.snapshot import stop_text
from .mauer_tab import STATE_COLOURS
from .ur_panel import BAD, GREY, INFO, OK, WARN, UrPoller, ur_short

ACTION_MAX = 70                               # characters of the current action shown (full text in the tooltip)
TWIN_COLOURS = {"running": OK, "starting": INFO, "lost": BAD, "stopped": GREY, "off": GREY}
CSS = "padding: 0 6px; font-family: monospace;"


def _short(text: str, n: int = ACTION_MAX) -> str:
    return text if len(text) <= n else text[:n - 3] + "..."


class StatusStrip(QFrame):
    def __init__(self, ctx, parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self._ctl = ctx.controller
        self.setObjectName("statusStrip")
        self.setStyleSheet("#statusStrip{background:#1A1A1A; border:1px solid #333333; border-radius:4px;}")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(0)
        self.run_lbl, self.mode_lbl, self.stop_lbl, self.stone_lbl = QLabel(), QLabel(), QLabel(), QLabel()
        self.action_lbl, self.ur_lbl, self.ares_lbl, self.twin_lbl = QLabel(), QLabel(), QLabel(), QLabel()
        fields = (self.run_lbl, self.mode_lbl, self.stop_lbl, self.stone_lbl, self.action_lbl, self.ur_lbl,
                  self.ares_lbl, self.twin_lbl)
        for i, w in enumerate(fields):
            if i:
                sep = QLabel("|")
                sep.setStyleSheet("color:#444444;")
                lay.addWidget(sep)
            set_style(w, CSS + "color:#CCCCCC;")
            lay.addWidget(w, 1 if w is self.action_lbl else 0)
        c = self._ctl
        c.state_changed.connect(self._on_state)
        c.snapshot_changed.connect(self._on_snapshot)
        UrPoller.of(ctx).updated.connect(self._on_ur)
        ctx.ads_status.connect(lambda _d: self._show_ares())
        ctx.ads_connection.connect(lambda _ok, _m: self._show_ares())
        ctx.twin_state_changed.connect(self._on_twin)
        self._on_state(c.state, "")
        self._on_snapshot(c.snapshot)
        self._on_ur(UrPoller.of(ctx).last)
        self._show_ares()
        self._on_twin(*ctx.twin_state)

    def texts(self) -> list[str]:
        """The field texts, left to right (tests, screenshots)."""
        return [w.text() for w in (self.run_lbl, self.mode_lbl, self.stop_lbl, self.stone_lbl, self.action_lbl,
                                   self.ur_lbl, self.ares_lbl, self.twin_lbl)]

    @staticmethod
    def _show(w: QLabel, text: str, colour: str, bold: bool = False, tip: str = "") -> None:
        set_text(w, text)
        set_style(w, CSS + f"color:{colour};" + ("font-weight:bold;" if bold else ""))
        w.setToolTip(tip)

    def _on_state(self, state: str, detail: str) -> None:
        c = self._ctl
        halted = " HALTED" if c.halted else ""
        self._show(self.run_lbl, f"{state.upper()}{halted}", BAD if halted else STATE_COLOURS.get(state, "#CCCCCC"),
                   True, detail)
        mode = (c.mode or "-").upper() if state not in ("empty", "loaded", "loading") else "-"
        self._show(self.mode_lbl, mode, WARN if mode == "REAL" else INFO if mode == "SIM" else GREY, mode == "REAL")

    def _on_snapshot(self, snap) -> None:
        if snap is None:
            for w in (self.stop_lbl, self.stone_lbl, self.action_lbl):
                self._show(w, "-", GREY)
            return
        k = snap.stop_k
        leg = f" {snap.stop_leg}" if snap.stop_leg and k is not None else ""
        self._show(self.stop_lbl, stop_text(k, snap.n_stops) + leg, "#CCCCCC")
        st = snap.stone
        done = snap.n_placed >= snap.n_stones > 0
        self._show(self.stone_lbl, f"stone {st.i}/{snap.n_stones} {st.label}" if st is not None else
                   f"placed {snap.n_placed}/{snap.n_stones}", OK if done else "#CCCCCC",
                   tip=st.describe(snap.n_stones) if st is not None else "")
        held = f" [holding {snap.held.get('kind')}]" if snap.held else ""
        colour = BAD if snap.error and snap.seq_state == "error" else WARN if snap.held else "#AAAAAA"
        self._show(self.action_lbl, _short(snap.action + held), colour, tip=snap.action + held)

    def _on_ur(self, u) -> None:
        text, colour = ur_short(u)
        self._show(self.ur_lbl, text, colour, colour == BAD)

    def _show_ares(self) -> None:
        ctx = self._ctx
        if not ctx.ares_enabled:
            self._show(self.ares_lbl, "ARES: ADS off", GREY, tip="start the HMI with --ares for the PLC connection")
            return
        st = ctx.last_ads_status
        if not ctx.ads_connected or not st:
            self._show(self.ares_lbl, "ARES: not connected", BAD, True)
            return
        colour, name = C.state_info(st.get("eAmrState", -1))
        move = " move" if st.get("bMoveActive") else ""
        self._show(self.ares_lbl, f"ARES: {name}{move}", INFO if move else colour, bool(move))

    def _on_twin(self, state: str, detail: str) -> None:
        self._show(self.twin_lbl, f"Twin: {state}", TWIN_COLOURS.get(state, "#CCCCCC"), state == "lost", detail)
