"""The integrated Mauer HMI (docs/HMI_DESIGN.md section 13, INTEGRATE): one MainWindow with the real RunController
and every feature widget - camera view, UR panel, wall pose, run feedback, status strip and the twin (a fake twin
in place of robodk/twin.py, no RoboDK) - through a step-mode SIM run with a station trip that is HALTed from another
tab while a stone is in the jaws, recovered ('Jaws empty') and resumed to the end. Offscreen, fakes only."""
from __future__ import annotations

import pytest
from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QKeyEvent

from hmi.core import twin_link
from hmi.core.session import JobSession
from hmi_fakes import make_window, short_job, straight_job10, wait_until

pytestmark = pytest.mark.usefixtures("no_lab_network")

RUN_S = 60.0


def test_window_with_every_feature_through_a_halted_and_resumed_sim_run(qapp, tmp_path, monkeypatch):
    from test_hmi_twin import FakeTwin, tm
    FakeTwin.instances = []
    monkeypatch.setattr(twin_link, "_default_factory", lambda *a: FakeTwin(*a, poll_s=0.005))
    cfg, job10 = straight_job10()
    session = JobSession(cfg, None, short_job(job10, n0=6, n1=3, fill=4), None, "test", "short_trip")
    w, fw, ctx = make_window(qapp, tmp_path)
    c = ctx.controller
    w.show()
    asked: list = []

    def answer(req) -> None:                       # Go for every step, except the first place of a held stone
        if req.text.startswith("robot: place stone") and req.holding and not asked:
            asked.append(req)
            return
        c.answer_confirm(req.id, True)

    c.confirm_requested.connect(answer)
    msgs: list = []
    c.message.connect(lambda lvl, t: msgs.append((lvl, t)))
    try:
        c.set_session(session)
        w.twin.enable.setChecked(True)
        assert wait_until(lambda: ctx.twin_state[0] == "running", 5.0, qapp)
        m = w.mauer
        m.scenario.setCurrentText("none")
        m.sim_speed.setValue(0.0)
        m.step.setChecked(True)
        assert wait_until(lambda: m.prepare_btn.isEnabled(), 5.0, qapp)
        m.prepare_btn.click()
        assert wait_until(lambda: m.start_btn.isEnabled(), 30.0, qapp), c.state
        m.start_btn.click()
        assert wait_until(lambda: asked and w.confirm_bar.isVisibleTo(w), RUN_S, qapp), (c.state, msgs)
        tw = FakeTwin.instances[-1]
        assert wait_until(lambda: tw.shown.held is not None, 5.0, qapp)        # the twin shows the stone in the jaws

        # HALT (Space) from the Camera tab: the ADS HALT first, the pending place is released, the run ends
        w.tabs.setCurrentWidget(w.camera)
        monkeypatch.setattr(w, "isActiveWindow", lambda: True)
        ev = QKeyEvent(QEvent.KeyPress, Qt.Key_Space, Qt.NoModifier, "", False)
        assert w.eventFilter(w, ev) is True
        assert fw.of("halt") == [None]
        assert wait_until(lambda: c.state == "aborted" and w.status_strip.texts()[0] == "ABORTED HALTED", RUN_S,
                          qapp), (c.state, w.status_strip.texts())
        assert c.sequencer.held is not None and m.step.isChecked()             # step mode on for the resume
        assert w.camera.n_shots >= 1 and wait_until(lambda: w.camera.n_shown >= 1, 5.0, qapp)

        # resume refused while the jaws may hold the stone; 'Jaws empty', step mode off, resume to the end
        m.resume_btn.click()
        assert wait_until(lambda: any("jaws" in t for _l, t in msgs) and c.state == "aborted", 10.0, qapp), msgs
        assert wait_until(lambda: m.jaws_btn.isEnabled(), 5.0, qapp)
        m.jaws_btn.click()
        assert wait_until(lambda: c.sequencer.held is None and not m.jaws_btn.isEnabled(), 10.0, qapp)
        m.step.setChecked(False)
        m.resume_btn.click()
        assert wait_until(lambda: c.state == "done" and w.status_strip.texts()[0] == "DONE", RUN_S, qapp), \
            (c.state, msgs)

        # every feature saw the same run
        world = c.rig.world
        n = len(session.job.stones())
        assert len(c.snapshot.placed) == n == len({r.key for r in world.records}) == 9
        assert c.snapshot.reloads >= 1                                          # a station trip
        assert wait_until(lambda: w.mauer.feedback.tiles["stones"].text().startswith(f"{n} / {n}"), 5.0, qapp)
        assert w.camera.n_fits >= 1 and w.wall_pose.fits.rowCount() >= 1
        w.tabs.setCurrentWidget(w.ur)
        assert wait_until(lambda: "SIM" in w.ur.source_lbl.text(), 5.0, qapp)
        final = tm.StoneState.from_snapshot(c.snapshot)
        assert wait_until(lambda: tw.shown == final, 5.0, qapp)
        assert tw.bad == [] and final.counts()["wall"] == 9 and final.counts()["tool"] == 0
        assert "short_trip" in w.windowTitle() and "SIM" in w.windowTitle()
    finally:
        if c.state in ("running", "pausing", "aborting"):
            c.halt()
            wait_until(lambda: c.state not in ("running", "pausing", "aborting"), 30.0, qapp)
        w.close()
    assert FakeTwin.instances[-1].stopped == 1                                 # closing the HMI stops the twin
