"""AdsWorker with a fake pyads connection: safe defaults, heartbeat order, version fallback, sum-writes.

Copied 2026-10-07 from MA 10_robot/hmi/amr_hmi/tests/test_ads_worker.py (commit 5935c5b); the lazy-import guard
covers hmi/**/*.py and the camera / RoboDK modules too."""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

import pytest

from hmi.amr import plc_vars as pv
from hmi.amr.ads_worker import AdsWorker, InterfaceMismatch, is_symbol_not_found
from hmi_fakes import FROM, TO, FakeADSError, FakeConnection, Scheduler, plc_symbols

pytestmark = pytest.mark.usefixtures("no_lab_network")

CFG = {
    "ads": {"ams_net_id": "1.2.3.4.1.1", "ads_port": 851, "host_ip": "", "timeout_ms": 750},
    "plc": {"to_plc_struct": "GVL_HMI.stToPlc", "from_plc_struct": "GVL_HMI.stFromPlc"},
    "hmi": {"poll_interval_ms": 100, "heartbeat_interval_ms": 100, "reconnect_interval_s": 5},
}


class Rig:
    def __init__(self, version: int = pv.IF_V2, **sym_overrides: Any) -> None:
        self.log: List[Tuple] = []
        self.symbols = plc_symbols(version, **sym_overrides)
        self.conn = FakeConnection(self.symbols, self.log)
        self.sched = Scheduler()
        self.worker = AdsWorker(CFG, connection_factory=lambda: self.conn, scheduler=self.sched)
        self.conn_events: List[Tuple[bool, str]] = []
        self.iface: List[Tuple[int, str]] = []
        self.errors: List[str] = []
        self.status: List[Dict[str, Any]] = []
        self.worker.connection.connect(lambda ok, msg: self.conn_events.append((ok, msg)))
        self.worker.interface.connect(lambda v, b: self.iface.append((v, b)))
        self.worker.command_error.connect(self.errors.append)
        self.worker.status.connect(self.status.append)

    def writes(self) -> List[Dict[str, Any]]:
        return [e[1] for e in self.log if e[0] == "write"]

    def command_writes(self) -> List[Dict[str, Any]]:
        return [w for w in self.writes() if TO + "nHeartbeat" not in w]


@pytest.fixture
def rig(qapp):
    return Rig()


def test_connect_applies_timeout_and_writes_safe_defaults_before_heartbeat(qapp):
    # stale TRUE bits in the PLC (e.g. release lost before a disconnect)
    r = Rig(**{TO + "bCmdJogFwd": True, TO + "bCmdManualMode": True, TO + "bCmdMoveStart": True,
               TO + "bCmdSafetyRun": True})
    assert r.worker.connect_now() is True
    ops = [e[0] for e in r.log]
    assert ops[:2] == ["open", "set_timeout"] and ("set_timeout", 750) in r.log
    first_write = ops.index("write")
    first_hb = next(i for i, e in enumerate(r.log) if e[0] == "write" and TO + "nHeartbeat" in e[1])
    assert first_write < first_hb
    safe = r.log[first_write][1]
    for f, v in pv.safe_default_values(pv.IF_V2).items():
        assert safe[TO + f] is v is False
    assert TO + "nCommandId" in safe and TO + "nHeartbeat" not in safe
    assert r.symbols[TO + "bCmdJogFwd"] is False and r.symbols[TO + "bCmdMoveStart"] is False
    assert r.symbols[TO + "bCmdSafetyRun"] is True          # Safety Run is not touched
    assert r.conn_events[-1][0] is True
    assert r.iface == [(2, "ARES CX9240 v2 2026-09-25")]
    assert r.worker.if_version == pv.IF_V2


def test_legacy_plc_falls_back_to_v1_lists(qapp):
    r = Rig(version=pv.IF_LEGACY)
    assert r.worker.connect_now() is True
    assert r.iface[0][0] == pv.IF_LEGACY
    safe = r.writes()[0]
    assert TO + "bCmdMoveStart" not in safe and TO + "bCmdJogFwd" in safe
    data = r.worker.poll_once()
    assert data is not None and "eMoveState" not in data and "eAmrState" in data
    # move fields are not written to a legacy PLC
    r.worker.send({"fMoveX_mm": 10.0, "bCmdMoveStart": True}, "GO")
    assert any("not available" in e for e in r.errors)
    # HALT still releases the jog bits, without bCmdMoveAbort
    r.worker.halt()
    halt = r.command_writes()[-1]
    assert TO + "bCmdMoveAbort" not in halt and halt[TO + "bCmdJogFwd"] is False


def test_connection_failure_reports_and_stays_disconnected(qapp):
    r = Rig()
    r.conn.fail_open = True
    assert r.worker.connect_now() is False
    assert r.conn_events[-1][0] is False and not r.worker.is_connected
    assert r.worker.heartbeat_once() is False
    assert not r.writes()


def test_unreachable_plc_is_not_mistaken_for_legacy(qapp):
    r = Rig()
    r.conn.fail_reads = True
    assert r.worker.connect_now() is False
    assert not r.iface
    assert r.log[-1] == ("close",)               # opened connection closed again


@pytest.mark.parametrize("error", [
    FakeADSError("timeout elapsed (1861)", 1861),
    FakeADSError("target machine not found (7)", 7),
    OSError("connection reset"),
])
def test_detection_error_other_than_symbol_not_found_is_not_legacy(qapp, error):
    """Finding N8: only 'symbol not found' means legacy. A timeout on the nIfVersion read of a v2 PLC (while the v1
    heartbeat would still be readable) fails the connect; the retry then connects as v2 and HALT sends
    bCmdMoveAbort."""
    r = Rig()
    real_read = r.conn.read_list_by_name
    fail = {"on": True}

    def read(names, cache_symbol_info=True, **kw):
        if fail["on"] and FROM + "nIfVersion" in names:
            r.log.append(("read", tuple(names)))
            raise error
        return real_read(names, cache_symbol_info, **kw)

    r.conn.read_list_by_name = read
    assert r.worker.connect_now() is False
    ok, msg = r.conn_events[-1]
    assert ok is False and msg.startswith("connect failed") and str(error) in msg
    assert not r.iface and not r.worker.is_connected
    assert not r.writes()                                   # no legacy safe defaults, no heartbeat
    assert not any(e[0] == "read" and FROM + "nPlcHeartbeat" in e[1] for e in r.log)   # no legacy probe
    assert r.log[-1] == ("close",)
    assert r.worker.if_version == pv.IF_LEGACY              # unchanged initial value, nothing was selected
    # transient error gone -> the reconnect selects v2 with move support
    fail["on"] = False
    assert r.worker.connect_now() is True
    assert r.iface == [(pv.IF_V2, "ARES CX9240 v2 2026-09-25")]
    assert TO + "bCmdMoveStart" in r.writes()[0]           # v2 safe defaults
    r.worker.halt()
    assert r.command_writes()[-1][TO + "bCmdMoveAbort"] is True


def test_legacy_fallback_only_after_symbol_not_found(qapp):
    """The legacy probe (v1 heartbeat) runs only after nIfVersion was reported missing (1808)."""
    r = Rig(version=pv.IF_LEGACY)
    assert r.worker.connect_now() is True
    reads = [e[1] for e in r.log if e[0] == "read"]
    assert reads[0] == (FROM + "nIfVersion", FROM + "sPlcBuild")
    assert reads[1] == (FROM + "nPlcHeartbeat",)
    assert r.iface == [(pv.IF_LEGACY, "legacy PLC (no nIfVersion)")]


def test_v2_plc_without_rev2_fields_is_reported_clearly(qapp):
    """Spec section 9: eMoveCmdResult / bMoveLimited belong to the same build - no fallback, clear message."""
    r = Rig()
    del r.symbols[FROM + "eMoveCmdResult"]
    del r.symbols[FROM + "bMoveLimited"]
    assert r.worker.connect_now() is False
    ok, msg = r.conn_events[-1]
    assert ok is False
    assert msg.startswith(pv.MISMATCH_TEXT) and "does not match this HMI" in msg
    assert "stFromPlc.eMoveCmdResult" in msg and "stFromPlc.bMoveLimited" in msg
    assert not msg.startswith("connect failed")
    assert not r.writes()                        # no safe defaults / heartbeat into a foreign build
    assert not r.worker.is_connected and not r.iface
    assert r.worker.heartbeat_once() is False
    assert r.log[-1] == ("close",)               # the half-open connection is closed (no ADS port leak)
    # matching build loaded -> the reconnect succeeds
    r.symbols[FROM + "eMoveCmdResult"] = 0
    r.symbols[FROM + "bMoveLimited"] = False
    assert r.worker.connect_now() is True and r.iface[-1][0] == pv.IF_V2


def test_v2_plc_without_a_to_plc_field_is_reported(qapp):
    r = Rig()
    del r.symbols[TO + "bCmdOdomReset"]
    assert r.worker.connect_now() is False
    assert "stToPlc.bCmdOdomReset" in r.conn_events[-1][1]


def test_mismatch_message_lists_at_most_four_names():
    exc = InterfaceMismatch([f"stFromPlc.f{i}" for i in range(7)])
    assert "stFromPlc.f3" in str(exc) and "stFromPlc.f4" not in str(exc) and "(+3 more)" in str(exc)
    assert len(exc.missing) == 7


def test_symbol_check_timeout_is_not_probed_symbol_by_symbol(qapp):
    """A timeout during the symbol check fails the connect at once (no ~130 single reads on a dead link)."""
    r = Rig()
    real_read = r.conn.read_list_by_name

    def read(names, cache_symbol_info=True, **kw):
        if len(names) > 5:
            r.log.append(("read", tuple(names)))
            raise FakeADSError("timeout elapsed (1861)", 1861)
        return real_read(names, cache_symbol_info, **kw)

    r.conn.read_list_by_name = read
    assert r.worker.connect_now() is False
    assert r.conn_events[-1][1].startswith("connect failed") and "1861" in r.conn_events[-1][1]
    assert len([e for e in r.log if e[0] == "read"]) == 2      # detection + one failed check read
    assert r.log[-1] == ("close",)


def test_symbol_removed_after_connect_disconnects_then_names_it(rig):
    """A poll that fails with 1808 -> disconnect; the reconnect's symbol check names the missing field.

    Model only: the fake resolves every name on every read. pyads caches the symbol info per connection, so after
    a real online change of ST_HMI_* the HMI has to be restarted (README troubleshooting), which runs the check.
    """
    rig.worker.connect_now()
    del rig.symbols[FROM + "bMoveLimited"]
    assert rig.worker.poll_once() is None
    assert rig.conn_events[-1][0] is False and "symbol not found" in rig.conn_events[-1][1]
    assert rig.worker.connect_now() is False
    assert "stFromPlc.bMoveLimited" in rig.conn_events[-1][1]


def test_is_symbol_not_found():
    class E(Exception):
        err_code = 1808
    assert is_symbol_not_found(E("x"))
    assert is_symbol_not_found(Exception("ADSError: symbol not found (1808). "))
    assert not is_symbol_not_found(FakeADSError("timeout elapsed (1861)", 1861))
    assert not is_symbol_not_found(OSError("connection refused"))


def test_poll_returns_all_fields_with_defaults(rig):
    rig.symbols[FROM + "eAmrState"] = 7
    rig.symbols[FROM + "sMoveText"] = None          # pyads may deliver None for a failed item
    rig.worker.connect_now()
    data = rig.worker.poll_once()
    assert data["eAmrState"] == 7 and data["sMoveText"] == ""
    assert set(data) == set(pv.from_plc_vars(pv.IF_V2))
    assert rig.status[-1] == data


def test_each_command_is_one_sum_write_with_command_id(rig):
    rig.worker.connect_now()
    n0 = len(rig.command_writes())
    rig.worker.send({"bCmdManualMode": True, "bCmdAutoMode": False}, "manual")
    w = rig.command_writes()[n0:]
    assert len(w) == 1
    assert w[0][TO + "bCmdManualMode"] is True and w[0][TO + "bCmdAutoMode"] is False
    ids = [x[TO + "nCommandId"] for x in rig.command_writes()]
    assert ids == sorted(ids) and len(set(ids)) == len(ids)


def test_halt_one_write_then_abort_reset_after_300ms(rig):
    rig.worker.connect_now()
    n0 = len(rig.command_writes())
    rig.worker.halt()
    w = rig.command_writes()[n0:]
    assert len(w) == 1
    assert w[0][TO + "bCmdMoveAbort"] is True
    assert all(w[0][TO + f] is False for f in pv.JOG_FIELDS)
    assert TO + "bCmdStop" not in w[0]
    assert [ms for ms, _ in rig.sched.pending] == [300]
    rig.sched.run_all()
    reset = rig.command_writes()[-1]
    assert reset[TO + "bCmdMoveAbort"] is False and TO + "bCmdJogFwd" not in reset


def test_pulse_resets_and_newer_pulse_owns_reset(rig):
    rig.worker.connect_now()
    rig.worker.pulse({}, ("bCmdStart",), "start")
    rig.worker.pulse({}, ("bCmdStart",), "start")
    assert len(rig.sched.pending) == 2
    n0 = len(rig.command_writes())
    rig.sched.run_all()
    resets = rig.command_writes()[n0:]
    assert len(resets) == 1 and resets[0][TO + "bCmdStart"] is False


def test_move_start_write_contains_all_fields(rig):
    from hmi.amr.logic import MoveRequest, build_move_command
    rig.worker.connect_now()
    cmd = build_move_command(MoveRequest(False, dx_mm=1000.0), 5)
    rig.worker.pulse(cmd, ("bCmdMoveStart",), "GO")
    w = rig.command_writes()[-1]
    for k, v in cmd.items():
        assert w[TO + k] == v
    rig.sched.run_all()
    assert rig.command_writes()[-1] == {TO + "bCmdMoveStart": False,
                                        TO + "nCommandId": rig.command_writes()[-1][TO + "nCommandId"]}


def test_read_failure_disconnects_drops_commands_and_reconnect_rewrites_defaults(rig):
    rig.worker.connect_now()
    rig.conn.fail_reads = True
    assert rig.worker.poll_once() is None
    assert not rig.worker.is_connected and rig.conn_events[-1][0] is False
    assert rig.worker.heartbeat_once() is False
    n0 = len(rig.writes())
    rig.worker.send({"bCmdJogFwd": True}, "jog")         # issued while disconnected
    assert len(rig.writes()) == n0 and any("not connected" in e for e in rig.errors)
    rig.conn.fail_reads = False
    assert rig.worker.connect_now() is True
    after = rig.writes()[n0:]
    assert after[0][TO + "bCmdJogFwd"] is False          # safe defaults first
    assert not any(w.get(TO + "bCmdJogFwd") is True for w in after)   # stale jog never replayed


def test_write_failure_marks_disconnected(rig):
    rig.worker.connect_now()
    rig.conn.fail_writes = True
    rig.worker.send({"bCmdHorn": True}, "horn")
    assert not rig.worker.is_connected
    assert any("write failed" in e for e in rig.errors)


def test_heartbeat_counts_and_wraps(rig):
    rig.worker.connect_now()
    rig.worker._hb = 0xFFFE
    rig.worker.heartbeat_once()
    rig.worker.heartbeat_once()
    hbs = [w[TO + "nHeartbeat"] for w in rig.writes() if TO + "nHeartbeat" in w]
    assert hbs[-2:] == [0xFFFF, 0]


def test_shutdown_releases_jog_and_closes(rig):
    rig.worker.connect_now()
    rig.worker.request_stop()
    last = rig.writes()[-1]
    assert all(last[TO + f] is False for f in pv.JOG_FIELDS)
    assert rig.log[-1] == ("close",) and not rig.worker.is_connected


LAZY_ONLY = ("pyads", "ids_peak", "robodk", "robolink", "rdk_common", "twin")


def test_hardware_modules_are_imported_lazily_only():
    """No module of the HMI imports pyads (or the camera / RoboDK modules) at module level: tests and the GUI start
    without pyads, a PLC, the IDS runtime or RoboDK (docs/HMI_DESIGN.md section 3, import rules)."""
    import ast
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent / "hmi"
    files = sorted(root.rglob("*.py"))
    assert len(files) > 10
    for path in files:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in tree.body:
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""] + [a.name for a in node.names if not node.module]
            for n in names:
                assert n.split(".")[0] not in LAZY_ONLY and n.split(".")[-1] not in LAZY_ONLY, \
                    f"{path.relative_to(root.parent)} imports {n} at module level"
