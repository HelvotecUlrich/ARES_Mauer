"""Sequencer hooks for the Mauer HMI (docs/HMI_DESIGN.md section 7) in the simulated world: run-log listeners and the
write lock (H1), the frame tap (H2), the ares_cmd (H3) and station_refilled (H4) records."""
from __future__ import annotations

import json
import threading

import pytest

from hmi_fakes import short_job, straight_job10
from mauer.job import SlotState
from mauer.sequencer import RunLog, Sequencer, read_log
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
