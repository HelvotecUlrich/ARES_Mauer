"""AdsWorker running in a real QThread (timers, queued signals, pulse reset, blocking stop) - fake connection.

Copied 2026-10-07 from MA 10_robot/hmi/amr_hmi/tests/test_worker_thread.py (commit 5935c5b), imports from the package."""

from __future__ import annotations

import threading
import time
from typing import Any, List, Tuple

import pytest
from PySide6.QtCore import QThread

from hmi.amr import plc_vars as pv
from hmi.amr.ads_worker import AdsWorker
from hmi_fakes import TO, FakeConnection, plc_symbols

pytestmark = pytest.mark.usefixtures("no_lab_network")

CFG = {
    "ads": {"ams_net_id": "1.2.3.4.1.1", "ads_port": 851, "host_ip": "", "timeout_ms": 500},
    "plc": {"to_plc_struct": "GVL_HMI.stToPlc", "from_plc_struct": "GVL_HMI.stFromPlc"},
    "hmi": {"poll_interval_ms": 100, "heartbeat_interval_ms": 100, "reconnect_interval_s": 1},
}


def _spin(qapp, cond, timeout_s: float) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        qapp.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_worker_in_thread(qapp):
    log: List[Tuple] = []
    symbols = plc_symbols(pv.IF_V2)
    conn = FakeConnection(symbols, log)
    worker_threads: List[int] = []

    def factory() -> Any:
        worker_threads.append(threading.get_ident())
        return conn

    worker = AdsWorker(CFG, connection_factory=factory)
    thread = QThread()
    worker.moveToThread(thread)
    thread.started.connect(worker.start)
    got_status: List[dict] = []
    conn_ev: List[bool] = []
    worker.status.connect(got_status.append)
    worker.connection.connect(lambda ok, _m: conn_ev.append(ok))
    thread.start()
    try:
        assert _spin(qapp, lambda: conn_ev and got_status, 3.0), "no connection/status from the worker thread"
        assert worker_threads[0] != threading.get_ident()          # connection opened in the worker thread
        _spin(qapp, lambda: False, 0.55)          # let the timers run ~0.5 s
        hb = [e for e in log if e[0] == "write" and TO + "nHeartbeat" in e[1]]
        assert len(hb) >= 4, f"only {len(hb)} heartbeats in ~0.5 s"
        # HALT from the GUI thread -> abort TRUE now, FALSE after ~300 ms (QTimer in the worker thread)
        worker.halt()
        assert _spin(qapp, lambda: symbols[TO + "bCmdMoveAbort"] is True, 1.0)
        t_set = time.monotonic()
        assert _spin(qapp, lambda: symbols[TO + "bCmdMoveAbort"] is False, 1.5)
        assert time.monotonic() - t_set >= 0.2
    finally:
        worker.request_stop()        # BlockingQueuedConnection into the worker thread
        thread.quit()
        assert thread.wait(3000)
    assert not worker.is_connected
    assert log[-1] == ("close",)
    n = len(log)
    time.sleep(0.25)
    assert len(log) == n             # timers stopped: no more heartbeats
