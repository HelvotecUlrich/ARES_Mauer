"""Tests of mauer.ares (ADS client for the ARES relative move) against tests/fake_plc.py, and - if the Masterarbeit
repo is next to this one - against its PLC reference model sim_relmove.Sim (imported read-only, no bytecode written).

All runs use simulated PLC time (FakePlc.clock / sleep) except the one realtime test with a heartbeat thread.
"""
from __future__ import annotations

import math
import os
import re
import socket
import sys
from pathlib import Path
from typing import Any

import pytest

from fake_plc import FROM, TO, FakeADSError, FakeConnection, FakePlc, HmiHeartbeat
from mauer import REPO, config
from mauer.ares import (AresAds, AresConnectionError, AresNotReady, MoveRefused, OdomPose, check_move, next_move_id,
                        parse_build_version, plc_timeout_s, tcp_reachable)
from mauer.ares import ads as A

CFG = config.load()["ares_ads"]
FORBIDDEN = {"nHeartbeat", "nCommandId", "bCmdManualMode", "bCmdAutoMode", "bCmdStop", "bCmdStart", "bCmdReset",
             "bCmdSafetyRun", "bCmdSafetyReset", "fAccel_mms", "bCmdOdomReset", "bCmdPreciseMode", "bCmdHorn",
             *A.JOG_FIELDS}


def make(plc: FakePlc | None = None, **kw: Any) -> tuple[FakePlc, AresAds]:
    plc = plc or FakePlc(**kw)
    plc.run(0.5)                                  # heartbeat alive, PLC settled
    ares = AresAds(CFG, connection_factory=plc.factory(), clock=plc.clock, sleep=plc.sleep)
    ares.connect()
    return plc, ares


def hmi_move(plc: FakePlc, dy: float, cmd_id: int = 900) -> None:
    """Start a move the way amr_hmi does (one sum write, start bit reset 300 ms later) on the HMI's connection."""
    c = plc.connection("hmi")
    c.open()
    c.write_list_by_name({TO + "fMoveX_mm": 0.0, TO + "fMoveY_mm": dy, TO + "fMoveTheta_deg": 0.0,
                          TO + "fMoveSpeed_mms": 150.0, TO + "fMoveRotSpeed_degs": 10.0, TO + "fMoveAccel_mms2": 200.0,
                          TO + "nMoveCmdId": cmd_id, TO + "bCmdMoveStart": True})
    plc.run(0.3)
    c.write_by_name(TO + "bCmdMoveStart", False)


# ── pure helpers ──────────────────────────────────────────────────────────────
def test_next_move_id_rule():
    assert next_move_id(0, 0) == 1                 # fresh PLC: ack 0, never send 0
    assert next_move_id(0, 1) == 2                 # never the PLC's last id
    assert next_move_id(65535, 0) == 1             # wrap 65535 -> 1, skipping 0
    assert next_move_id(65535, 1) == 2
    assert next_move_id(65534, 65535) == 1
    assert next_move_id(7, 7) == 8
    import random
    rng = random.Random(1)
    for _ in range(2000):
        last, ack = rng.randrange(65536), rng.choice([0, 1, 65535, rng.randrange(65536)])
        n = next_move_id(last, ack)
        assert 1 <= n <= 65535 and n != ack and n != last


def test_plc_timeout_and_build_parsing():
    assert plc_timeout_s(1400.0, 150.0, 200.0, False) == pytest.approx(2 * (1400 / 150 + 150 / 200) + 10)
    assert plc_timeout_s(1400.0, 150.0, 200.0, False, hmi_accel_mms2=100.0) == pytest.approx(
        2 * (1400 / 150 + 150 / 100) + 10)      # GVL_AMR.fAccel_mms caps the move acceleration
    assert plc_timeout_s(10.0, 900.0, 5000.0, False) == pytest.approx(2 * (10 / 500 + 500 / 1000) + 10)
    a_deg = 200.0 / 353.625 * 180.0 / math.pi
    assert plc_timeout_s(90.0, 10.0, 200.0, True) == pytest.approx(2 * (90 / 10 + 10 / a_deg) + 10)
    assert parse_build_version("ARES CX9240 v2.9 2026-10-01") == (2, 9)
    assert parse_build_version("ARES CX9240 v2 2026-09-25") == (2, 0)
    assert parse_build_version("") is None


@pytest.mark.parametrize("args, part", [
    ((100.0, 0.0, 5.0, 150, 10, 200), "translation and rotation in one command"),
    ((1.9, 0.0, 0.0, 150, 10, 200), "below the PLC minimum 2 mm"),
    ((1.0, 1.0, 0.0, 150, 10, 200), "below the PLC minimum 2 mm"),      # hypot 1.41 mm
    ((0.0, 0.0, 0.0, 150, 10, 200), "below the PLC minimum 2 mm"),
    ((0.0, 0.0, 0.15, 150, 10, 200), "below the PLC minimum 0.2 deg"),
    ((0.0, 25000.0, 0.0, 150, 10, 200), "above the PLC maximum"),
    ((0.0, 0.0, 800.0, 150, 10, 200), "above the PLC maximum"),
    ((0.0, 100.0, 0.0, 900, 10, 200), "speed 900 mm/s outside"),
    ((0.0, 100.0, 0.0, 0.0, 10, 200), "speed 0 mm/s outside"),
    ((0.0, 0.0, 90.0, 150, 60, 200), "rotation speed 60 deg/s outside"),
    ((0.0, 100.0, 0.0, 150, 10, 0.0), "acceleration 0 mm/s2"),
    ((0.0, 100.0, 0.0, 150, 10, 2000), "acceleration 2000 mm/s2"),
    ((float("nan"), 100.0, 0.0, 150, 10, 200), "not a finite number"),
])
def test_check_move_refuses(args, part):
    assert part in check_move(*args)


def test_check_move_accepts():
    assert check_move(0.0, 1400.0, 0.0, 150, 10, 200) is None
    assert check_move(2.0, 0.0, 0.0, 150, 10, 200) is None
    assert check_move(0.0, 0.0, -0.2, 150, 10, 200) is None
    assert check_move(0.0, 0.0, 720.0, 150, 45, 1000) is None


def test_odom_delta_in_start_frame():
    p0 = OdomPose(1000.0, 500.0, 90.0)          # facing +y of the odometry frame
    p1 = OdomPose(1000.0 - 1400.0, 500.0, 90.5)  # moved along -x odom = +y body (left)
    dx, dy, dth = p0.delta_to(p1)
    assert (dx, dy, dth) == pytest.approx((0.0, 1400.0, 0.5), abs=1e-9)
    assert OdomPose(0, 0, 179.0).delta_to(OdomPose(0, 0, -179.0))[2] == pytest.approx(2.0)


def test_tcp_reachable_fast():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    assert tcp_reachable("127.0.0.1", port, 0.5) is None
    srv.close()
    assert tcp_reachable("127.0.0.1", port, 0.5) is not None


# ── connect ───────────────────────────────────────────────────────────────────
def test_connect_fails_fast_on_tcp_precheck():
    calls = []
    ares = AresAds(CFG, connection_factory=lambda: calls.append(1),
                   tcp_probe=lambda host, port, t: f"probe {host}:{port} {t:.1f} s timed out")
    with pytest.raises(AresConnectionError) as ei:
        ares.connect()
    msg = str(ei.value)
    assert "192.168.1.10:48898" in msg and "ARES network" in msg and "station.toml" in msg
    assert calls == []                                       # pyads (factory) never touched
    assert not ares.connected


def test_connect_interface_v1_and_plc_stopped():
    plc = FakePlc(if_version=1)
    with pytest.raises(AresConnectionError, match="relative-move interface"):
        AresAds(CFG, connection_factory=plc.factory(), clock=plc.clock, sleep=plc.sleep).connect()
    plc = FakePlc()
    plc.ads_state = 6                                        # ADSSTATE_STOP
    with pytest.raises(AresConnectionError, match="not in RUN"):
        AresAds(CFG, connection_factory=plc.factory(), clock=plc.clock, sleep=plc.sleep).connect()
    plc = FakePlc()
    plc.offline = True
    with pytest.raises(AresConnectionError, match="TwinCAT route"):
        AresAds(CFG, connection_factory=plc.factory(), clock=plc.clock, sleep=plc.sleep).connect()


def test_status_fields_and_context_manager():
    plc = FakePlc()
    plc.run(0.5)
    with AresAds(CFG, connection_factory=plc.factory(), clock=plc.clock, sleep=plc.sleep) as ares:
        s = ares.status()
        assert s.if_version == 2 and s.build == "ARES CX9240 v2.9 2026-10-01"
        assert s.amr_state == 7 and s.amr_state_name == "MANUAL MODE" and s.hmi_watchdog_ok
        assert not s.ext_active and not s.move_active and s.move_state_name == "IDLE"
        assert s.odom == OdomPose(0.0, 0.0, 0.0) and s.hmi_accel_mms2 == 1000.0
    assert not ares.connected
    assert plc.written("client") == []                       # connect / status / close write nothing


def test_per_symbol_errors_are_failures():
    plc, ares = make()
    plc.read_errors[FROM + "eAmrState"] = "device symbol not found"     # adsSumRead: text instead of the value
    with pytest.raises(AresConnectionError, match="eAmrState"):
        ares.status()
    plc.read_errors.clear()
    plc.write_errors[TO + "nMoveCmdId"] = "device access denied"         # adsSumWrite: per-symbol error text
    with pytest.raises(AresConnectionError, match="refused write"):
        ares.translate(0.0, 100.0)
    assert "bCmdMoveStart" not in {w.name for w in plc.written() if w.value is True}   # never got to the edge


# ── moves ─────────────────────────────────────────────────────────────────────
def test_nominal_plus_y_move_done():
    plc, ares = make()
    o = ares.translate(0.0, 1400.0)                          # +Y = left, 150 mm/s, 200 mm/s2 from the config
    assert o.ok and o.in_tolerance and not o.rejected and not o.client_timeout
    assert (o.cmd_id, o.cmd_result, o.state, o.result) == (1, 0, A.MOVE_DONE, A.RES_OK)
    assert abs(o.final_err_mm) < 5.0 and math.isnan(o.final_err_deg) and not o.limited
    dx, dy, dth = o.odom_delta()
    assert dx == pytest.approx(0.0, abs=1e-6) and dy == pytest.approx(1400.0 - o.final_err_mm, abs=1e-6)
    assert plc.pose[1] == pytest.approx(o.odom_after.y_mm)
    assert 9.0 < o.elapsed_s < 30.0 and o.text == "Done"
    # handshake: parameters with the start bit FALSE, >= 50 ms later the edge, reset after the ack (< 300 ms)
    w = plc.written()
    first = [r for r in w if r.name == "nMoveCmdId"][0]
    starts = [r for r in w if r.name == "bCmdMoveStart"]
    assert [r.value for r in starts] == [False, True, False]
    assert starts[0].t_s == first.t_s and starts[1].t_s - first.t_s >= 0.05
    assert 0.0 < starts[2].t_s - starts[1].t_s <= 0.3
    assert {r.name for r in w if r.t_s == first.t_s} == {"fMoveX_mm", "fMoveY_mm", "fMoveTheta_deg", "fMoveSpeed_mms",
                                                         "fMoveRotSpeed_degs", "fMoveAccel_mms2", "nMoveCmdId",
                                                         "bCmdMoveStart"}
    assert "OK - DONE" in o.summary()
    o2 = ares.translate(0.0, 1400.0)                         # next id, from the new start pose
    assert o2.ok and o2.cmd_id == 2 and o2.odom_before == o.odom_after


def test_rotation_and_forced_final_error():
    plc, ares = make(final_err_mm=7.0)
    o = ares.translate(-300.0, 0.0)
    assert o.ok and not o.in_tolerance and o.result == A.RES_OK_TOL and o.final_err_mm == pytest.approx(7.0)
    assert "outside tolerance" in o.text
    o = ares.rotate(90.0)
    assert o.ok and o.kind == "rotate" and o.result == A.RES_OK and abs(o.final_err_deg) < 0.5
    assert math.isnan(o.final_err_mm) and o.odom_after.theta_deg == pytest.approx(90.0 - o.final_err_deg)


def test_limited_by_plc_accel():
    plc, ares = make()
    c = plc.connection("hmi")
    c.open()
    c.write_by_name(TO + "fAccel_mms", 100.0)                # HMI traction ramp below the requested move accel
    plc.run(0.01)
    o = ares.translate(0.0, 200.0, accel_mms2=200.0)
    assert o.ok and o.limited and "limited by the PLC" in o.summary()


def test_combination_and_minimum_refused_locally():
    plc, ares = make()
    with pytest.raises(MoveRefused, match="translation and rotation"):
        ares.move(dx_mm=100.0, dtheta_deg=5.0)
    with pytest.raises(MoveRefused, match="2 mm"):
        ares.translate(1.5, 0.0)
    with pytest.raises(MoveRefused, match="0.2 deg"):
        ares.rotate(0.1)
    assert plc.written() == [] and plc.reads > 0             # refused before any write


@pytest.mark.parametrize("code, setup, text", [
    (10, lambda p: setattr(p, "odom_params_ok", False), "odometry parameters implausible"),
    (11, lambda p: setattr(p, "test_mode", True), "TEST_PRG test mode"),
    # races between the preflight and the edge (state changed by someone else meanwhile)
    (12, lambda p: p.on_write("nMoveCmdId", lambda q, v: setattr(q, "ext_active", True)), "external control"),
    (13, lambda p: p.on_write("nMoveCmdId", lambda q, v: q.T.__setitem__("bCmdJogLeft", True)), "jog command"),
    (11, lambda p: p.on_write("nMoveCmdId", lambda q, v: setattr(q, "amr_state", 6)), "not in MANUAL"),
])
def test_rejection_reasons_mapped(code, setup, text):
    plc, ares = make()
    setup(plc)
    o = ares.translate(0.0, 500.0)
    assert not o.ok and o.rejected and o.cmd_result == code and o.result == code
    assert text in o.text and o.result_text.startswith("rejected")
    assert "REJECTED" in o.summary() and o.elapsed_s == 0.0
    assert "bCmdMoveAbort" not in plc.written_names()       # nothing to abort
    assert plc.T["bCmdMoveStart"] is False                   # start bit reset


def test_heartbeat_lost_reports_abort_26():
    plc, ares = make()
    plc.at(plc.clock() + 4.0, lambda p: setattr(p, "hmi_alive", False))     # amr_hmi closed mid-move
    o = ares.translate(0.0, 1400.0)
    assert not o.ok and o.state == A.MOVE_ABORTED and o.result == A.RES_ABORT_HMI
    assert "HMI heartbeat lost" in o.text and o.final_err_mm > 100.0 and not o.client_timeout
    assert "bCmdMoveAbort" not in plc.written_names()        # the PLC aborted by itself
    plc.run(2.0)                                             # 2 s watchdog: MANUAL dropped
    problems = ares.preflight()
    assert any("did not change" in p for p in problems) and any("bHmiWatchdogOK" in p for p in problems)
    assert any("needs 7 MANUAL" in p for p in problems)


def test_client_timeout_sends_abort_pulse():
    plc, ares = make()
    o = ares.translate(0.0, 1400.0, timeout_s=3.0)
    assert o.client_timeout and not o.ok
    assert o.state == A.MOVE_ABORTED and o.result == A.RES_ABORT_USER and "client deadline 3.0 s" in o.text
    pulse = [w for w in plc.written() if w.name == "bCmdMoveAbort"]
    assert [w.value for w in pulse] == [True, False]
    assert pulse[1].t_s - pulse[0].t_s == pytest.approx(0.3, abs=0.01)
    assert pulse[0].t_s - [w for w in plc.written() if w.name == "bCmdMoveStart" and w.value][0].t_s > 3.0
    assert ares.preflight() == []                            # abort bit released, ARES still in MANUAL


def test_abort_on_ctrl_c_during_move():
    plc, ares = make()
    n = {"polls": 0}

    def progress(s):
        n["polls"] += 1
        if n["polls"] == 30:
            raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        ares.translate(0.0, 1400.0, progress=progress)
    assert [w.value for w in plc.written() if w.name == "bCmdMoveAbort"] == [True, False]
    plc.run(12.0)
    assert plc.F["eMoveState"] == A.MOVE_ABORTED and plc.F["eMoveResult"] == A.RES_ABORT_USER


def test_torn_read_early_not_counted_as_done():
    plc, ares = make()
    assert ares.translate(0.0, 500.0).ok                    # leaves DONE / inactive behind
    stale = ["eMoveState", "eMoveResult", "bMoveActive", "sMoveText", "fMoveDone", "fMoveFinalErr", "fMoveElapsed_s"]
    plc.on_write("bCmdMoveStart", lambda p, v: v and p.freeze(stale, 0.5))   # new ack next to the old DONE
    o = ares.translate(0.0, 1400.0)
    assert o.ok and o.cmd_id == 2 and o.wall_s > 9.0
    assert o.odom_delta()[1] == pytest.approx(1400.0 - o.final_err_mm, abs=1e-6)


def test_torn_read_never_active_is_not_done():
    plc, ares = make()
    assert ares.translate(0.0, 500.0).ok
    stale = ["eMoveState", "eMoveResult", "bMoveActive", "sMoveText"]
    plc.on_write("bCmdMoveStart", lambda p, v: v and p.freeze(stale))       # stale for the whole move
    o = ares.translate(0.0, 1400.0, timeout_s=5.0)
    assert not o.ok and o.client_timeout and "never seen running" in o.text
    assert [w.value for w in plc.written() if w.name == "bCmdMoveAbort"] == [True, False]


def test_torn_verdict_confirmed_by_second_read():
    plc, ares = make()
    plc.test_mode = True
    assert ares.translate(0.0, 300.0).cmd_result == 11       # previous command rejected
    plc.test_mode = False
    plc.on_write("bCmdMoveStart", lambda p, v: v and p.freeze(["eMoveCmdResult"], 0.002))
    o = ares.translate(0.0, 300.0)                           # first ack read shows the stale verdict 11
    assert o.ok and o.cmd_result == 0


def test_no_acknowledgement_raises_and_aborts():
    plc, ares = make()
    plc.freeze(["nMoveCmdAck"])
    with pytest.raises(AresConnectionError, match="no acknowledgement"):
        ares.translate(0.0, 500.0)
    assert [w.value for w in plc.written() if w.name == "bCmdMoveAbort"] == [True, False]
    assert plc.T["bCmdMoveStart"] is False


class LossyConnection(FakeConnection):
    """The write that sets the start edge reaches the PLC, but its reply is lost (ADS timeout 1861)."""

    def write_list_by_name(self, data_names_and_values: dict[str, Any], **kw: Any) -> dict[str, str]:
        res = super().write_list_by_name(data_names_and_values, **kw)
        if data_names_and_values.get(TO + "bCmdMoveStart") is True:
            raise FakeADSError("timeout elapsed (1861). ", 1861)
        return res


def test_failed_edge_write_aborts_and_resets():
    plc = FakePlc()
    plc.run(0.5)
    ares = AresAds(CFG, connection_factory=lambda: LossyConnection(plc), clock=plc.clock, sleep=plc.sleep)
    ares.connect()
    with pytest.raises(AresConnectionError, match="may have started, abort sent") as ei:
        ares.translate(0.0, 500.0)
    assert ei.value.err_code == 1861
    assert [w.value for w in plc.written() if w.name == "bCmdMoveAbort"] == [True, False]
    assert plc.T["bCmdMoveStart"] is False
    plc.run(3.0)                                             # the started move was stopped by the abort pulse
    assert plc.F["eMoveState"] == A.MOVE_ABORTED and plc.F["eMoveResult"] == A.RES_ABORT_USER


def test_halt_latch_refuses_the_start_edge():
    """Review 2026-10-08: the Mauer HMI latches HALT on the sequencer's AresAds. Inhibited, move() writes nothing; a
    latch set while the preflight's heartbeat window runs stops the start edge (the parameters stay unused)."""
    plc, ares = make()
    ares.inhibit("HALT")
    with pytest.raises(A.AresInhibited, match="HALT: ARES moves inhibited"):
        ares.translate(0.0, 300.0)
    assert plc.written() == []
    ares.release_inhibit()
    plain_sleep = ares._sleep

    def sleep(s: float) -> None:
        if s == A.HB_WINDOW_S:                              # inside check(): HALT pressed now
            ares.inhibit("HALT")
        plain_sleep(s)
    ares._sleep = sleep
    with pytest.raises(A.AresInhibited):
        ares.translate(0.0, 300.0)
    assert [w.value for w in plc.written() if w.name == "bCmdMoveStart"] == [False, False]
    ares._sleep = plain_sleep
    ares.release_inhibit()
    plc.run(0.5)
    o = ares.translate(0.0, 300.0)
    assert o.ok and ares.inhibited is None


# ── preflight ─────────────────────────────────────────────────────────────────
def _hb_stopped(p: FakePlc) -> None:
    p.hmi_alive = False


@pytest.mark.parametrize("setup, part", [
    (lambda p: setattr(p, "amr_state", 6), "AMR state 6 READY, needs 7 MANUAL"),
    (lambda p: setattr(p, "ext_active", True), "external control active"),
    (_hb_stopped, "did not change within 250 ms"),
    (lambda p: (_hb_stopped(p), p.run(2.5)), "bHmiWatchdogOK FALSE"),
    (lambda p: p.T.__setitem__("bCmdMoveAbort", True), "bCmdMoveAbort is TRUE"),
    (lambda p: p.T.__setitem__("bCmdStop", True), "bCmdStop is TRUE"),
    (lambda p: p.T.__setitem__("bCmdJogRotLeft", True), "jog bit"),
    (lambda p: (hmi_move(p, 1000.0), p.run(0.5)), "a relative move is running"),
    (lambda p: setattr(p, "safety_stop", True), "safety stop active"),
    (lambda p: setattr(p, "fault", True), "drive / system fault: Drive system error"),
    (lambda p: setattr(p, "build", "ARES CX9240 v2 2026-09-25"), "older than v2.1"),
    (lambda p: setattr(p, "build", "unknown build"), "unreadable"),
])
def test_preflight_messages(setup, part):
    plc, ares = make()
    assert ares.preflight() == []
    setup(plc)
    plc.run(0.01)
    problems = ares.preflight()
    assert any(part in p for p in problems), problems
    with pytest.raises(AresNotReady) as ei:                  # never moves without a passing preflight
        ares.translate(0.0, 100.0)
    assert any(part in p for p in ei.value.problems)
    assert plc.written() == []


def test_preflight_not_connected():
    assert AresAds(CFG, connection_factory=FakePlc().factory()).preflight()[0].startswith("not connected")


# ── pattern A ─────────────────────────────────────────────────────────────────
def test_never_writes_forbidden_symbols():
    plc, ares = make()
    ares.translate(0.0, 800.0)
    ares.rotate(-45.0)
    plc.test_mode = True
    ares.translate(0.0, 800.0)                               # rejected
    plc.test_mode = False
    ares.translate(0.0, 1400.0, timeout_s=2.0)               # client abort
    plc.at(plc.clock() + 3.0, lambda p: setattr(p, "hmi_alive", False))
    ares.translate(0.0, 1400.0)                              # PLC abort 26
    names = plc.written_names("client")
    assert names <= A.WRITABLE and not names & FORBIDDEN
    assert {"bCmdMoveStart", "bCmdMoveAbort", "nMoveCmdId", "fMoveY_mm"} <= names
    n = len(plc.writes)
    for bad in ({"bCmdStop": True}, {"bCmdManualMode": False}, {"nHeartbeat": 1}, {"bCmdJogLeft": False}):
        with pytest.raises(ValueError, match="pattern A"):
            ares._write(bad)
    assert len(plc.writes) == n


# ── realtime: emulated HMI heartbeat thread, real clock and sleep ─────────────
def test_realtime_with_hmi_heartbeat_thread():
    plc = FakePlc(realtime=True, hmi_heartbeat=False, align_s=0.2)
    ares = AresAds(CFG, connection_factory=plc.factory())    # real time.monotonic / time.sleep
    ares.connect()
    with HmiHeartbeat(plc, period_s=0.1):
        assert ares.preflight() == []
        o = ares.translate(0.0, 30.0, speed_mms=100.0)
        assert o.ok and o.result == A.RES_OK and abs(o.final_err_mm) < 5.0
    problems = ares.preflight()                              # thread stopped: heartbeat frozen
    assert any("did not change" in p for p in problems)
    assert plc.written_names("hmi") == {"nHeartbeat"} and "nHeartbeat" not in plc.written_names("client")
    ares.close()


# ── MA reference model (line-by-line port of MOVE_PRG / FB_RelMove / ODOM_PRG) ─
MA_REPO = Path(os.environ.get("ARES_MA_REPO", str(REPO.parent / "Masterarbeit")))
MA_TEST = MA_REPO / "10_robot" / "twincat" / "PLC_CX9240_v2" / "test"
MA_PYCACHE = [MA_TEST / "__pycache__", MA_REPO / "10_robot" / "ros2_ws" / "src" / "ares_plc_sim" / "ares_plc_sim"
              / "__pycache__"]


def _pycache_state() -> dict[str, Any]:
    return {str(p): sorted((f.name, f.stat().st_mtime_ns) for f in p.iterdir()) if p.is_dir() else None
            for p in MA_PYCACHE}


@pytest.fixture(scope="module")
def refmodel():
    if not (MA_TEST / "sim_relmove.py").is_file():
        pytest.skip(f"Masterarbeit reference model not found at {MA_TEST}")
    before = _pycache_state()
    old_flag, old_path = sys.dont_write_bytecode, list(sys.path)
    sys.dont_write_bytecode = True                           # read-only use: no __pycache__ in the MA repo
    try:
        sys.path.insert(0, str(MA_TEST))
        import sim_relmove
    finally:
        sys.path[:] = old_path
        sys.dont_write_bytecode = old_flag
    gvl_build = (MA_TEST.parent / "src" / "GVL_Build.st").read_text(encoding="utf-8")
    build = re.search(r"sPlcBuild\s*:\s*STRING\(\d+\)\s*:=\s*'([^']*)'", gvl_build).group(1)
    return sim_relmove, build, before


class RefModelConnection:
    """pyads-like connection on sim_relmove.Sim (adapted from the research scratch try_sim.py). stFromPlc fields
    the model does not keep are derived as FB_HMI_Interface.TcPOU:176-340 builds them; bAmrMoving from the wheel
    speeds (approximation)."""
    MOVE = {"eMoveState": "eState", "eMoveResult": "eResult", "eMoveCmdResult": "eCmdResult",
            "nMoveCmdAck": "nCmdAck", "bMoveLimited": "bLimited", "sMoveText": "sText", "bMoveActive": "bActive",
            "bMoveIsRotation": "bIsRotation", "fMoveTarget": "fTarget", "fMoveDone": "fDone",
            "fMoveRemaining": "fRemaining", "fMoveLatErr_mm": "fLatErr_mm", "fMoveHeadErr_deg": "fHeadErr_deg",
            "fMoveProgress_pct": "fProgress_pct", "fMoveFinalErr": "fFinalErr", "fMoveElapsed_s": "fElapsed_s"}

    def __init__(self, sim: Any, build: str) -> None:
        self.sim, self.build = sim, build

    def open(self) -> None: ...
    def close(self) -> None: ...
    def set_timeout(self, ms: int) -> None: ...
    def read_state(self) -> tuple[int, int]:
        return 5, 0

    def _get(self, name: str) -> Any:
        sim, g = self.sim, self.sim.g
        A_, O = g.GVL_AMR, g.GVL_Odom
        if name.startswith(TO):
            return getattr(g.stToPlc, name[len(TO):])
        f = name[len(FROM):]
        if f in self.MOVE:
            v = getattr(g.GVL_Move, self.MOVE[f])
            return int(v) if isinstance(v, int) and not isinstance(v, bool) else v
        state = 7 if A_.bManualControlEnable else (6 if A_.bSystemReady else 4)
        return {
            "nIfVersion": 2, "sPlcBuild": self.build, "eAmrState": state, "sAmrStateText": A.AMR_STATE_NAMES[state],
            "bHmiWatchdogOK": not sim.hmi_wd.Q, "nPlcHeartbeat": sim.clock.now_ms & 0xFFFF,
            "bExtActive": g.stStatus.bExtActive, "bAmrReady": A_.bSystemReady,
            "bAmrMoving": 0.5 * (abs(A_.fActSpeed_Left) + abs(A_.fActSpeed_Right)) > 1.0,
            "bSafetyOK": not A_.bSafetyStopActive, "bEmergencyStopActive": A_.bSafetyStopActive,
            "bDrivesEnabled": A_.bEnableTraction and A_.bEnableSteering,
            "bFaultActive": A_.bSystemError or A_.bTractionFault or A_.bSteeringFault, "sFaultText": "",
            "fPosX_m": O.fX_mm * 1e-3, "fPosY_m": O.fY_mm * 1e-3, "fPosTheta_deg": O.fThetaWrapped_deg,
            "fOdomDist_m": O.fDist_mm * 1e-3,
        }[f]

    def read_list_by_name(self, names: list[str], **_: Any) -> dict[str, Any]:
        return {n: self._get(n) for n in names}

    def write_list_by_name(self, values: dict[str, Any], **_: Any) -> dict[str, str]:
        for n, v in values.items():
            assert n.startswith(TO) and n[len(TO):] in A.WRITABLE, n
            setattr(self.sim.g.stToPlc, n[len(TO):], v)
        return {n: "no error" for n in values}


def _ref_client(refmodel) -> tuple[Any, AresAds]:
    SR, build, _ = refmodel
    sim = SR.Sim(fAccel_mms=1000.0)          # amr_hmi writes fAccel_mms 1000 on MANUAL (config jog_accel_mms2)
    sim.run(1.0)                             # model's HMI emulation: heartbeat every 100 ms, odometry valid
    clock = lambda: sim.clock.now_ms / 1000.0
    ares = AresAds(CFG, connection_factory=lambda: RefModelConnection(sim, build), clock=clock,
                   sleep=lambda s: sim.run(max(s, 0.001)))
    ares.connect()
    return sim, ares


def test_refmodel_nominal_plus_y(refmodel):
    sim, ares = _ref_client(refmodel)
    assert ares.preflight() == []
    o = ares.translate(0.0, 1400.0)
    assert o.ok and o.result == A.RES_OK and abs(o.final_err_mm) < 5.0, o.summary()
    assert sim.y == pytest.approx(1400.0, abs=5.0) and abs(sim.x) < 5.0     # true pose (model: no slip)
    assert o.odom_delta()[1] == pytest.approx(1400.0 - o.final_err_mm, abs=0.5)
    o = ares.rotate(90.0)
    assert o.ok and abs(o.final_err_deg) < 0.5, o.summary()


def test_refmodel_heartbeat_lost_and_abort(refmodel):
    sim, ares = _ref_client(refmodel)
    t_lost = sim.clock.now_ms + 4000
    sim.hooks.append(lambda s: setattr(s, "hb_alive", s.clock.now_ms < t_lost))
    o = ares.translate(0.0, 1400.0)
    assert o.state == A.MOVE_ABORTED and o.result == A.RES_ABORT_HMI, o.summary()
    sim, ares = _ref_client(refmodel)
    o = ares.translate(0.0, 1400.0, timeout_s=3.0)
    assert o.client_timeout and o.result == A.RES_ABORT_USER, o.summary()
    sim.g.stToPlc.bCmdMoveAbort = True                       # left TRUE: preflight refuses, PLC would say 13
    assert any("bCmdMoveAbort is TRUE" in p for p in ares.preflight())


def test_refmodel_left_masterarbeit_untouched(refmodel):
    assert _pycache_state() == refmodel[2]                   # no new or rewritten bytecode in the MA repo
