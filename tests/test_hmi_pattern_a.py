"""Operating pattern A in ONE process (docs/HMI_DESIGN.md D-H2): the HMI's AdsWorker (heartbeat, MANUAL, HALT) and
the sequencer's AresAds (move fields only) on two connections to the same fake PLC (tests/fake_plc.py, real time).

Proven on the robot only with two processes (E003); this covers the protocol and the heartbeat timing in one
process. The jacked-up check on the robot stays in the test plan (section 14 item 1).
"""
from __future__ import annotations

import threading
import time

import pytest
from PySide6.QtCore import QThread

from fake_plc import FakePlc
from hmi.amr.ads_worker import AdsWorker
from hmi.core.config import amr_cfg
from hmi_fakes import wait_until
from mauer import config
from mauer.ares import ads
from mauer.ares.ads import AresAds

pytestmark = pytest.mark.usefixtures("no_lab_network")

HB_GAP_MAX_S = ads.HB_WINDOW_S          # the AresAds preflight needs a heartbeat change within this window


@pytest.fixture
def plc_hmi(qapp):
    """FakePlc in real time without its emulated HMI heartbeat; the HMI's AdsWorker in its own QThread on the
    connection "hmi" brings the AMR from READY (after its safe defaults) to MANUAL as the operator would."""
    cfg = config.load()
    plc = FakePlc(realtime=True, hmi_heartbeat=False)
    worker = AdsWorker(amr_cfg(cfg), connection_factory=plc.factory("hmi"))
    thread = QThread()
    thread.setObjectName("ads-worker")
    worker.moveToThread(thread)
    thread.started.connect(worker.start)
    status: list[dict] = []
    worker.status.connect(status.append)
    thread.start()
    try:
        assert wait_until(lambda: worker.is_connected and status, 5.0, qapp), "AdsWorker did not connect"
        assert wait_until(lambda: status[-1]["eAmrState"] == 6, 3.0, qapp)      # safe defaults dropped MANUAL
        worker.send({"bCmdManualMode": True}, "manual mode")                     # amr startup step "Manual"
        assert wait_until(lambda: status[-1]["eAmrState"] == 7, 3.0, qapp), "no MANUAL after the Manual request"
        yield plc, worker, status, cfg
    finally:
        worker.request_stop()
        thread.quit()
        assert thread.wait(3000)


def heartbeat_gaps(plc: FakePlc, t0: float = 0.0) -> list[float]:
    t = [w.t_s for w in plc.written("hmi") if w.name == "nHeartbeat" and w.t_s >= t0]
    return [b - a for a, b in zip(t, t[1:])]


def test_hmi_and_ares_ads_share_the_plc(plc_hmi, qapp):
    plc, worker, status, cfg = plc_hmi
    client = AresAds(cfg["ares_ads"], connection_factory=plc.factory("client"))
    client.connect()
    try:
        assert client.preflight() == []
        out = client.translate(100.0, 0.0)
        assert out.ok and out.result == ads.RES_OK, out.summary()
        assert plc.pose[0] == pytest.approx(100.0, abs=5.0)
    finally:
        client.close()
    assert plc.written_names("client") <= ads.WRITABLE                       # pattern A: move fields only
    assert {"nHeartbeat", "bCmdManualMode"} <= plc.written_names("hmi")
    gaps = heartbeat_gaps(plc)
    assert len(gaps) > 10 and max(gaps) < HB_GAP_MAX_S, max(gaps)


def test_hmi_halt_aborts_the_clients_move(plc_hmi, qapp):
    plc, worker, status, cfg = plc_hmi
    client = AresAds(cfg["ares_ads"], connection_factory=plc.factory("client"))
    client.connect()
    timer = threading.Timer(1.8, worker.halt)                 # HALT button while the run thread waits for the move
    try:
        timer.start()
        t0 = time.monotonic()
        out = client.translate(1000.0, 0.0)
    finally:
        timer.cancel()
        client.close()
    assert not out.ok and out.result == ads.RES_ABORT_USER, out.summary()
    assert time.monotonic() - t0 < 10.0 and plc.pose[0] < 900.0
    halt = [w for w in plc.written("hmi") if w.name == "bCmdMoveAbort" and w.value is True]
    assert len(halt) == 1
    wait_until(lambda: False, 0.5, qapp)                      # the worker resets the pulse after 300 ms
    assert plc.T["bCmdMoveAbort"] is False
    assert plc.written_names("client") <= ads.WRITABLE
