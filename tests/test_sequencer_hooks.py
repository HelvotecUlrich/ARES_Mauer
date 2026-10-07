"""Sequencer hooks for the Mauer HMI (docs/HMI_DESIGN.md section 7) in the simulated world: run-log listeners and the
write lock (H1), the frame tap (H2), the ares_cmd (H3) and station_refilled (H4) records, the held-stone guard (H5)
and the catch-all run_error (H6)."""
from __future__ import annotations

import json
import threading

import pytest

from hmi_fakes import short_job, straight_job10
from mauer.camera import CameraError
from mauer.job import SlotState
from mauer.sequencer import RunLog, Sequencer, SequencerAborted, SequencerError, SequencerPaused, read_log
from mauer.simworld import SimWorld, WorldErrors

pytestmark = pytest.mark.usefixtures("no_lab_network")


def sim(job, tmp_path, *, errors=None, seed=1, **kw):
    cfg, _ = straight_job10()
    w = SimWorld(cfg, job, errors or WorldErrors(), seed=seed, **{k: v for k, v in kw.items() if k == "fail_on"})
    seq = Sequencer(job, cfg, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=tmp_path / "run",
                    on_station_empty=w.refill_station, **{k: v for k, v in kw.items() if k != "fail_on"})
    return w, seq


# ── H1: listeners, lock ───────────────────────────────────────────────────────
def test_listeners_get_every_record_in_order(tmp_path):
    _, job10 = straight_job10()
    w, seq = sim(short_job(job10, n0=2, n1=1), tmp_path)
    a, b = [], []
    seq.log.add_listener(a.append)
    seq.log.add_listener(lambda rec: b.append(rec["event"]))
    seq.run()
    seq.close()
    logged = read_log(seq.log.path)
    assert [r["event"] for r in a] == b == [r["event"] for r in logged]
    assert [r["t"] for r in a] == [r["t"] for r in logged]
    assert b[0] == "run_start" and b[-1] == "run_done" and b.count("placed") == 3 == len(w.records)
    seq.log.remove_listener(a.append)
    assert len(seq.log._listeners) == 1


def test_a_failing_listener_never_stops_the_run(tmp_path, caplog):
    _, job10 = straight_job10()
    w, seq = sim(short_job(job10, n0=2, n1=0), tmp_path)

    def boom(rec):
        raise RuntimeError("listener bug")

    seq.log.add_listener(boom)
    assert seq.run(0, 0).state == "done" and len(w.records) == 2
    assert "listener bug" in caplog.text


def test_parallel_writes_give_whole_lines(tmp_path):
    log = RunLog(tmp_path / "par")
    n, k = 4, 300

    def writer(i):
        for j in range(k):
            log.write("x", writer=i, j=j, pad="y" * 200)

    th = [threading.Thread(target=writer, args=(i,)) for i in range(n)]
    for t in th:
        t.start()
    for t in th:
        t.join()
    log.close()
    lines = log.path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == n * k
    recs = [json.loads(ln) for ln in lines]                     # every line is one whole record
    for i in range(n):
        assert [r["j"] for r in recs if r["writer"] == i] == list(range(k))


# ── H2: frame tap ─────────────────────────────────────────────────────────────
def test_on_shot_once_per_shot_record(tmp_path):
    _, job10 = straight_job10()
    taps, recs = [], []
    w, seq = sim(short_job(job10, n0=2, n1=1), tmp_path, on_shot=lambda img, rec: taps.append((img, rec)))
    seq.log.add_listener(lambda r: recs.append(r) if r["event"] == "shot" else None)
    seq.run()
    assert len(taps) == len(recs) == w.n_shots > 0
    for (img, rec), r in zip(taps, recs):
        assert rec is r and img.shape == (w.intr.height, w.intr.width)


def test_a_failing_tap_never_stops_the_run(tmp_path):
    _, job10 = straight_job10()

    def boom(img, rec):
        raise ValueError("tap bug")

    w, seq = sim(short_job(job10, n0=1, n1=0), tmp_path, on_shot=boom)
    assert seq.run(0, 0).state == "done" and len(w.records) == 1


# ── H3: ares_cmd ──────────────────────────────────────────────────────────────
def test_ares_cmd_precedes_every_ares_move(tmp_path):
    _, job10 = straight_job10()
    w, seq = sim(short_job(job10, n0=1, n1=1), tmp_path)
    seq.run()
    ev = read_log(seq.log.path)
    moves = [i for i, e in enumerate(ev) if e["event"] == "ares_move"]
    assert len(moves) == len(w.ares.moves) >= 1
    for i in moves:
        cmd = ev[i - 1]
        assert cmd["event"] == "ares_cmd" and cmd["kind"] == ev[i]["kind"] and cmd["args"] == ev[i]["args"]
        assert cmd["why"] == ev[i]["why"] and cmd["pose"] is not None and cmd["pose_src"]


# ── H4: station_refilled ──────────────────────────────────────────────────────
def test_station_refilled_follows_station_empty(tmp_path):
    _, job10 = straight_job10()
    job = short_job(job10, n0=2, n1=0, fill=1)                 # reload after the first stone
    w, seq = sim(job, tmp_path)
    seq.station = SlotState.station(job.station, filled=[])     # the operator has not filled the station yet
    assert seq.run(0, 0).state == "done" and len(w.records) == 2
    names = [e["event"] for e in read_log(seq.log.path)]
    i = names.index("station_empty")
    assert names[i + 1] == "station_refilled" and names.index("reload_start") > i
    rec = next(e for e in read_log(seq.log.path) if e["event"] == "station_refilled")
    assert rec["station"] == len(job.station.take_order)


# ── H5: held-stone guard ──────────────────────────────────────────────────────
def _stones_conserved(w) -> int:
    """Stones in the magazine + placed + in the jaws (the simulated world's own bookkeeping)."""
    return len(w.mag_stones) + len(w.records) + (1 if w.robot.holding is not None else 0)


def test_pause_at_a_place_finishes_the_place_then_pauses(tmp_path):
    _, job10 = straight_job10()
    job = short_job(job10, n0=2, n1=1)
    holder = {}

    def confirm(desc):
        if desc.startswith("robot: place stone") and not holder.get("paused"):
            holder["paused"] = True
            holder["seq"].pause()                       # pressed while the place step is shown, then Go
        return True

    w, seq = sim(job, tmp_path, confirm=confirm)
    holder["seq"] = seq
    n0 = _stones_conserved(w)
    with pytest.raises(SequencerPaused):
        seq.run()
    assert seq.result.state == "paused" and seq.held is None and w.robot.holding is None
    assert len(w.records) == 1 and _stones_conserved(w) == n0          # placed, not lost
    names = [e["event"] for e in read_log(seq.log.path)]
    assert names.index("pause_requested") < names.index("placed") < names.index("run_paused")
    seq.resume()
    assert seq.run(seq.stop_k).state == "done"
    keys = [r.key for r in w.records]
    assert len(keys) == len(set(keys)) == 3 and _stones_conserved(w) == n0


def test_pause_while_an_empty_jaw_step_waits_is_a_pause_not_a_decline(tmp_path):
    _, job10 = straight_job10()
    holder = {}

    def confirm(desc):
        if desc.startswith("robot: pick magazine"):
            holder["seq"].pause()
            return False                                 # the HMI releases the pending step on Pause
        return True

    w, seq = sim(short_job(job10, n0=1, n1=0), tmp_path, confirm=confirm)
    holder["seq"] = seq
    with pytest.raises(SequencerPaused):
        seq.run(0, 0)
    assert seq.result.state == "paused" and seq.held is None
    assert "declined" not in [e["event"] for e in read_log(seq.log.path)]


def test_decline_while_held_needs_clear_held_before_resume(tmp_path):
    _, job10 = straight_job10()
    job = short_job(job10, n0=2, n1=1)
    w, seq = sim(job, tmp_path, confirm=lambda desc: not desc.startswith("robot: place stone"))
    with pytest.raises(SequencerAborted):
        seq.run()
    assert seq.held == {"from": "magazine", "slot": seq.held["slot"], "kind": "full",
                        "stone": list(job.stops[0].stones[0].key), "unknown": False}
    assert w.robot.holding is not None
    seq.confirm = None
    with pytest.raises(SequencerError, match="stone may be in the jaws"):
        seq.run(seq.stop_k)
    w.robot.holding = None                               # the operator takes the stone out of the jaws
    seq.clear_held()
    assert seq.held is None and seq.run(seq.stop_k).state == "done"
    keys = [r.key for r in w.records]
    assert len(keys) == len(set(keys)) == 3
    ev = read_log(seq.log.path)
    assert any(e["event"] == "held_cleared" and e["held"]["slot"] for e in ev)


@pytest.mark.parametrize("action, frm", [("pick_magazine", "magazine"), ("place_wall", "magazine")])
def test_robot_error_with_the_jaws_involved_marks_them_unknown(tmp_path, action, frm):
    _, job10 = straight_job10()
    w, seq = sim(short_job(job10, n0=2, n1=0), tmp_path, fail_on={action: 2})
    with pytest.raises(SequencerError, match="NOT parked"):
        seq.run(0, 0)
    assert seq.held is not None and seq.held["unknown"] is True and seq.held["from"] == frm
    assert seq.held["slot"] is not None
    with pytest.raises(SequencerError, match="stone may be in the jaws"):
        seq.run(0, 0)


def test_station_pick_error_marks_the_jaws_unknown(tmp_path):
    _, job10 = straight_job10()
    job = short_job(job10, n0=2, n1=0, fill=1)
    w, seq = sim(job, tmp_path, fail_on={"pick_station": 1})
    with pytest.raises(SequencerError):
        seq.run(0, 0)
    assert seq.held["from"] == "station" and seq.held["unknown"] and seq.held["slot"] in job.station.take_order


# ── H6: catch-all ─────────────────────────────────────────────────────────────
class _DeadCamera:
    def grab(self):
        raise CameraError("camera unplugged")


def test_an_unexpected_exception_ends_the_run_as_error(tmp_path):
    _, job10 = straight_job10()
    w, seq = sim(short_job(job10, n0=1, n1=0), tmp_path)
    seq.camera = _DeadCamera()
    with pytest.raises(SequencerError, match="CameraError: camera unplugged") as ei:
        seq.run(0, 0)
    assert isinstance(ei.value.__cause__, CameraError)
    assert seq.result.state == "error" and seq.result.error.startswith("CameraError")
    last = read_log(seq.log.path)[-1]
    assert last["event"] == "run_error" and last["kind"] == "CameraError"
