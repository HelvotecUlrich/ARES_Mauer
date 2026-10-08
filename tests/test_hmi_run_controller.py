"""RunController end to end in the simulated world, headless (docs/HMI_DESIGN.md section 8): the real run thread,
the short straight-wall job (3 stones), scenario none, no SIM pacing. Step confirmations are answered by slots in
the GUI thread, as the ConfirmBar does."""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import pytest

from hmi.core.run_controller import ConfirmRequest, RunController, RunOptions
from hmi_fakes import short_sim_session, wait_until
from mauer import config
from mauer.sequencer import RunLog, SequencerAborted, read_log
from hmi.core.rigs import HaltGate

pytestmark = pytest.mark.usefixtures("no_lab_network")

RUN_S = 60.0                     # generous: a short SIM run takes ~2 s


class Rec:
    """Everything the controller emits, collected in the GUI thread."""

    def __init__(self, c: RunController) -> None:
        self.events, self.shots, self.reports, self.states, self.messages = [], [], [], [], []
        self.requests: list[ConfirmRequest] = []
        self.cleared: list[int] = []
        c.event.connect(self.events.append)
        c.shot.connect(self.shots.append)
        c.preflight_done.connect(self.reports.append)
        c.state_changed.connect(lambda s, d: self.states.append((s, d)))
        c.message.connect(lambda lvl, txt: self.messages.append((lvl, txt)))
        c.confirm_requested.connect(self.requests.append)
        c.confirm_cleared.connect(self.cleared.append)

    def names(self) -> list[str]:
        return [e["event"] for e in self.events]

    def settled(self, state: str) -> bool:
        """The GUI thread has received the state signal (emitted after every record of the run before it, so all
        of those have arrived too)."""
        return bool(self.states) and self.states[-1][0] == state


@pytest.fixture
def ctl(qapp, tmp_path):
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs")
    yield c
    assert c.shutdown(15.0)


def prepared(c: RunController, qapp, **opts) -> Rec:
    rec = Rec(c)
    c.set_session(short_sim_session())
    assert c.state == "loaded"
    c.prepare(RunOptions("sim", scenario="none", sim_step_s=0.0, **opts))
    assert wait_until(lambda: rec.settled("ready"), 30.0, qapp), (c.state, rec.messages)
    return rec


def answer_all(c: RunController, rec: Rec, decide=lambda req: True):
    """Slot: answer every request with decide(req) (GUI thread, like the ConfirmBar)."""
    def on_req(req: ConfirmRequest) -> None:
        c.answer_confirm(req.id, decide(req))
    c.confirm_requested.connect(on_req)
    return on_req


def test_sim_run_end_to_end(ctl, qapp, tmp_path):
    c = ctl
    rec = prepared(c, qapp)
    assert rec.reports and rec.reports[-1].mode == "sim" and rec.reports[-1].ok      # informative only
    assert all(not i.blocking for i in rec.reports[-1].items)
    assert c.rig.mode == "sim" and c.rig.world is not None and c.can("start") == (True, "")
    c.start()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    assert wait_until(lambda: rec.events and rec.events[-1]["event"] == "run_done", 5.0, qapp)
    names = rec.names()
    assert names[0] == "run_start" and names[-1] == "run_done"
    world = c.rig.world
    assert len(c.snapshot.placed) == 3 == len(world.records) and c.snapshot.seq_state == "done"
    assert c.snapshot.stone is None and c.snapshot.n_events == len(names)
    assert len(rec.shots) == names.count("shot") == world.n_shots > 0
    v = rec.shots[0]
    assert v.preview.shape == (round(0.25 * world.intr.height), round(0.25 * world.intr.width))
    assert v.full_size == (world.intr.width, world.intr.height) and v.rec["event"] == "shot" and v.look
    log_dir = c.log_dir
    assert log_dir.parent == tmp_path / "runs" and log_dir.name.endswith("_hmi_sim")
    assert [r["event"] for r in read_log(log_dir)] == names
    summary = json.loads((log_dir / "hmi_summary.json").read_text(encoding="utf-8"))
    assert summary["mode"] == "sim" and summary["state"] == "done" and summary["sim"]["placement"]["n"] == 3
    assert summary["result"]["placed"] and summary["halted"] is False
    c.release()
    assert wait_until(lambda: rec.settled("loaded"), 10.0, qapp) and c.rig is None


def test_log_dirs_are_never_reused(ctl):
    p1 = ctl._new_log_dir("sim")
    p1.mkdir(parents=True)
    p2 = ctl._new_log_dir("sim")
    assert p2 != p1 and p2.name.startswith(p1.name)


def test_step_mode_decline_then_resume(ctl, qapp):
    c = ctl
    rec = prepared(c, qapp, step=True)
    declined = []

    def decide(req):
        if req.text.startswith("ARES") and not declined:
            declined.append(req.text)
            return False
        return True

    answer_all(c, rec, decide)
    c.start()
    assert wait_until(lambda: rec.settled("aborted"), RUN_S, qapp), (c.state, rec.messages)
    assert declined and "declined" in rec.names()
    assert len(c.rig.world.records) == 2 and c.snapshot.stop_k == 1     # the ARES move to stop 1 was declined
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    keys = [r.key for r in c.rig.world.records]
    assert len(keys) == len(set(keys)) == 3
    assert rec.cleared and set(rec.cleared) == {r.id for r in rec.requests}


def test_pause_at_a_place_pauses_after_the_place(ctl, qapp):
    c = ctl
    rec = prepared(c, qapp, step=True)
    paused = []

    def decide(req):
        if req.text.startswith("robot: place stone") and not paused:
            paused.append(req.id)
            c.pause()                                   # with a stone held the request stays pending ...
            assert c.pending is not None and c.pending.id == req.id
        return True                                     # ... and the operator presses Go

    answer_all(c, rec, decide)
    c.start()
    assert wait_until(lambda: rec.settled("paused"), RUN_S, qapp), (c.state, rec.messages)
    names = rec.names()
    assert names.index("pause_requested") < names.index("placed") < names.index("run_paused")
    assert len(c.rig.world.records) == 1 and c.sequencer.held is None
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    assert len(c.rig.world.records) == 3


def test_halt_releases_a_pending_request(ctl, qapp):
    c = ctl
    rec = prepared(c, qapp, step=False)
    c.set_step(True)                                   # changeable live
    c.start()
    assert wait_until(lambda: rec.requests, RUN_S, qapp)
    req = rec.requests[0]
    t0 = time.monotonic()
    c.halt()
    assert wait_until(lambda: req.id in rec.cleared, 1.0, qapp)
    assert time.monotonic() - t0 < 1.0
    assert wait_until(lambda: rec.settled("aborted"), RUN_S, qapp), (c.state, rec.messages)
    assert c.halted and c.step and c.opts.step                            # step forced on for the resume
    assert any(s == "aborted" and d.startswith("HALT") for s, d in rec.states)
    halt = [r for r in read_log(c.log_dir) if r["event"] == "halt"]       # HALT itself is in the run log
    assert len(halt) == 1 and halt[0]["pending"] == req.text and halt[0]["mode"] == "sim"
    answer_all(c, rec)
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    assert not c.halted and len(c.rig.world.records) == 3


def test_soft_abort_with_a_stone_held_waits_for_the_place(ctl, qapp):
    c = ctl
    rec = prepared(c, qapp, step=True)
    aborted = []

    def decide(req):
        if req.text.startswith("robot: place stone") and not aborted:
            aborted.append(req.id)
            c.abort()
            assert c.pending is not None                 # not released: a stone is held
        return True

    answer_all(c, rec, decide)
    c.start()
    assert wait_until(lambda: rec.settled("aborted"), RUN_S, qapp), (c.state, rec.messages)
    assert len(c.rig.world.records) == 1 and c.sequencer.held is None
    names = rec.names()
    assert names.index("placed") < names.index("declined") < names.index("run_aborted")


def test_resume_refused_while_held_until_jaws_empty(ctl, qapp):
    c = ctl
    rec = prepared(c, qapp, step=True)
    declined = []

    def decide(req):
        if req.text.startswith("robot: place stone") and not declined:
            declined.append(req.id)
            return False                                 # Decline with the stone in the jaws
        return True

    answer_all(c, rec, decide)
    c.start()
    assert wait_until(lambda: rec.settled("aborted"), RUN_S, qapp)
    assert c.sequencer.held is not None and c.can("clear_held") == (True, "")
    c.resume()
    assert wait_until(lambda: any("resume refused" in t and "jaws" in t for _, t in rec.messages), 10.0, qapp)
    assert wait_until(lambda: rec.settled("aborted"), 5.0, qapp)
    c.clear_held()
    assert wait_until(lambda: c.sequencer.held is None, 5.0, qapp)
    assert c.rig.world.robot.holding is None
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    keys = [r.key for r in c.rig.world.records]
    assert len(keys) == len(set(keys)) == 3


def test_resume_refused_while_the_pose_is_unknown_until_set_pose(ctl, qapp):
    c = ctl
    rec = prepared(c, qapp, step=True)
    answer_all(c, rec, lambda req: not req.text.startswith("ARES"))
    c.start()
    assert wait_until(lambda: rec.settled("aborted"), RUN_S, qapp)
    seq = c.sequencer
    seq.pose_status, seq.pose_src = "unknown", "unknown (ARES error)"      # as after an ARES error without odometry
    assert c.can("confirm_pose") == (False, "pose unknown - use Set pose") and c.can("set_pose")[0]
    seq.pose_status = "odometry"                                         # Sequencer.confirm_pose accepts only this
    assert c.can("confirm_pose") == (True, "")
    seq.pose_status = "unknown"
    c.resume()
    assert wait_until(lambda: any("pose is unknown" in t for _, t in rec.messages), 10.0, qapp)
    assert wait_until(lambda: rec.settled("aborted"), 5.0, qapp)
    c.set_pose(seq.pose_est)
    assert wait_until(lambda: seq.pose_status == "ok", 5.0, qapp)
    c.set_step(False)
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    assert "pose_set" in rec.names()


def test_halt_pause_abort_during_the_resume_checks_stop_the_run(ctl, qapp):
    """Review 2026-10-08: Resume clears HALT / Abort / the pause BEFORE its checks (REAL: ADS check, preflight -
    0.3-1 s). HALT, Pause or Abort pressed while they run stop the run before its first motion."""
    c = ctl
    rec = prepared(c, qapp, step=True)
    declined = []

    def decide(req):
        if req.text.startswith("ARES") and not declined:
            declined.append(req.text)
            return False
        return True

    answer_all(c, rec, decide)
    c.start()
    assert wait_until(lambda: rec.settled("aborted"), RUN_S, qapp), (c.state, rec.messages)
    world = c.rig.world
    blocks = world.n_blocks
    checks = c._resume_refusal
    entered, go = threading.Event(), threading.Event()

    def slow_checks():                           # stands for the REAL check (AresAds 250 ms window, preflight)
        entered.set()
        go.wait(10.0)
        return checks()

    c._resume_refusal = slow_checks
    for press, end in ((c.halt, "aborted"), (c.pause, "paused"), (c.abort, "paused")):
        entered.clear()
        go.clear()
        n_states = len(rec.states)
        c.resume()
        assert c.state == "running" and wait_until(entered.is_set, 10.0, qapp)
        press()
        go.set()
        assert wait_until(lambda: len(rec.states) > n_states + 1 and rec.settled(end), RUN_S, qapp), \
            (press.__name__, c.state, rec.states[-3:])
        assert world.n_blocks == blocks, press.__name__          # not one robot motion after the press
    msgs = [t for _, t in rec.messages]
    assert any("HALT pressed during the resume checks" in t for t in msgs)
    assert any("Abort pressed during the resume checks" in t for t in msgs)
    assert c.halted is False
    c._resume_refusal = checks
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)
    assert len(world.records) == 3


def test_pause_as_the_run_ends_keeps_the_end_state(ctl, qapp):
    """Review 2026-10-08: a Pause whose check passed while the run was ending must not leave the controller in
    'pausing' (BUSY: no close, release or resume). The pause request is held up in Sequencer.pause (1 s) while the
    run thread finishes."""
    c = ctl
    rec = prepared(c, qapp, step=True)
    pressed = []

    def on_req(req):
        c.answer_confirm(req.id, True)
        if req.text == "robot: park" and len(c.sequencer.placed) == 3 and not pressed:
            pressed.append(req.id)                # the last motion of the run is on its way
            seq = c.sequencer
            plain = seq.pause

            def slow_pause():
                time.sleep(1.0)
                plain()
            seq.pause = slow_pause
            c.pause()

    c.confirm_requested.connect(on_req)
    c.start()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.states[-3:])
    assert pressed and c.state == "done" and not c.sequencer.paused
    assert c.can("release") == (True, "")


def test_pause_before_the_run_got_going_is_a_pause(ctl, qapp):
    """Review 2026-10-08: Pause between Start and Sequencer.run (the run thread still busy) ends 'paused', not
    'error' with step mode forced."""
    c = ctl
    rec = prepared(c, qapp)
    busy = threading.Event()
    c._submit(lambda: busy.wait(10.0), "busy")     # e.g. a Live grab still in the run thread
    c.start()
    c.pause()
    busy.set()
    assert wait_until(lambda: rec.settled("paused"), RUN_S, qapp), (c.state, rec.states[-3:])
    assert "error" not in [s for s, _ in rec.states] and not c.step
    c.resume()
    assert wait_until(lambda: rec.settled("done"), RUN_S, qapp), (c.state, rec.messages)


def test_halt_gate_refuses_motions_after_halt(ctl, qapp):
    c = ctl
    prepared(c, qapp)
    gate = c.sequencer.robot
    assert isinstance(gate, HaltGate) and gate.backend is c.rig.world.robot
    c._halt.set()
    with pytest.raises(SequencerAborted, match="HALT: park not started"):
        gate.park()
    assert gate.is_parked() is True                             # reads pass through
    gate.holding = None                                          # writes reach the backend
    assert c.rig.world.robot.holding is None
    c._halt.clear()


class _SeqStub:
    def __init__(self, folder: Path) -> None:
        self.held, self.paused, self.stop_k = None, False, 1
        self.log = RunLog(folder)


def test_real_station_empty_callback(ctl, qapp, tmp_path):
    c = ctl
    c._opts = RunOptions("real")
    c._seq = _SeqStub(tmp_path / "stub")
    rec = Rec(c)
    try:
        for ok in (True, False):
            out = {}

            def call():
                try:
                    c._station_empty_cb()
                    out["returned"] = True
                except SequencerAborted as e:
                    out["exc"] = e

            th = threading.Thread(target=call)
            th.start()
            assert wait_until(lambda: c.pending is not None, 5.0, qapp)
            assert c.pending.kind == "station_empty" and "Refilled" in c.pending.text
            c.answer_confirm(c.pending.id + 1, True)            # a stale id is ignored
            assert c.pending is not None
            c.answer_confirm(c.pending.id, ok)
            th.join(5.0)
            assert out == ({"returned": True} if ok else {"exc": out.get("exc")}) and (ok or out["exc"])
        c._seq.log.close()
        assert read_log(tmp_path / "stub")[-1] == {**read_log(tmp_path / "stub")[-1], "event": "declined",
                                                   "what": "station refill"}
    finally:
        c._seq, c._opts = None, None
