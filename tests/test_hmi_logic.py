"""Command building (moves, HALT, ids), enable rules and startup step - pure functions, no Qt/ADS.

Copied 2026-10-07 from MA 10_robot/hmi/amr_hmi/tests/test_logic.py (commit 5935c5b), imports from the package."""

from __future__ import annotations

import math

import pytest

from hmi.amr import logic as L
from hmi.amr import plc_vars as pv
from hmi.amr.ui import constants as C

pytestmark = pytest.mark.usefixtures("no_lab_network")

MANUAL_V2 = {"eAmrState": C.ST_MANUAL, "bExtActive": False, "bMoveActive": False}


# ── move command building ──────────────────────────────────────────────────────

def test_translation_command_fields():
    req = L.MoveRequest(False, dx_mm=1000.0, dy_mm=0.0, dtheta_deg=33.0,
                        speed_mms=150.0, rot_speed_degs=10.0, accel_mms2=200.0)
    cmd = L.build_move_command(req, 7)
    assert cmd == {
        "fMoveX_mm": 1000.0, "fMoveY_mm": 0.0, "fMoveTheta_deg": 0.0,   # theta forced 0
        "fMoveSpeed_mms": 150.0, "fMoveRotSpeed_degs": 10.0, "fMoveAccel_mms2": 200.0,
        "nMoveCmdId": 7, "bCmdMoveStart": True,
    }
    assert set(cmd) <= set(pv.TO_PLC_VARS_V2_EXTRA)


def test_rotation_command_is_exclusive():
    req = L.MoveRequest(True, dx_mm=500.0, dy_mm=-20.0, dtheta_deg=-90.0)
    cmd = L.build_move_command(req, 1)
    assert cmd["fMoveX_mm"] == 0.0 and cmd["fMoveY_mm"] == 0.0
    assert cmd["fMoveTheta_deg"] == -90.0
    assert cmd["bCmdMoveStart"] is True


def test_defaults_match_spec():
    d = L.MoveDefaults()
    assert (d.distance_mm, d.angle_deg, d.speed_mms, d.rot_speed_degs, d.accel_mms2) == \
        (1000.0, 90.0, 150.0, 10.0, 200.0)
    assert (d.test_distance_mm, d.test_speed_mms) == (100.0, 50.0)
    cfg = {"move": {"default_speed_mms": 99.0}}
    assert L.MoveDefaults.from_config(cfg).speed_mms == 99.0
    assert L.MoveDefaults.from_config({}).accel_mms2 == 200.0


@pytest.mark.parametrize("bad_id", [0, -1, 65536])
def test_build_rejects_invalid_ids(bad_id):
    with pytest.raises(ValueError):
        L.build_move_command(L.MoveRequest(False, dx_mm=10), bad_id)


def test_validate_move():
    lim = L.MoveLimits()
    assert L.validate_move(L.MoveRequest(False, dx_mm=10.0), lim) is None
    assert L.validate_move(L.MoveRequest(False, dx_mm=0.2), lim) is not None
    assert L.validate_move(L.MoveRequest(False, dx_mm=20001.0), lim) is not None
    assert L.validate_move(L.MoveRequest(False, dx_mm=100.0, speed_mms=0.0), lim) is not None
    assert L.validate_move(L.MoveRequest(False, dx_mm=100.0, accel_mms2=0.0), lim) is not None
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=1.0), lim) is None
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=0.0), lim) is not None
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=721.0), lim) is not None
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=90.0, rot_speed_degs=46.0), lim) is not None
    # rotation ignores the (unused) translation speed
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=90.0, speed_mms=0.0), lim) is None


def test_min_move_mirrors_plc_fminmove():
    """GVL_Move.fMinMove_mm = 2 mm / fMinMove_deg = 0.2 deg (spec section 9); the PLC accepts >= the minimum."""
    lim = L.MoveLimits()
    assert (lim.min_distance_mm, lim.min_angle_deg) == (2.0, 0.2)
    err = L.validate_move(L.MoveRequest(False, dx_mm=1.99), lim)
    assert err and "2 mm" in err and "fMinMove_mm" in err
    assert L.validate_move(L.MoveRequest(False, dx_mm=1.2, dy_mm=-1.5), lim) is not None     # |d| = 1.92 mm
    assert L.validate_move(L.MoveRequest(False, dx_mm=2.0), lim) is None
    assert L.validate_move(L.MoveRequest(False, dy_mm=-2.0), lim) is None
    err = L.validate_move(L.MoveRequest(True, dtheta_deg=-0.19), lim)
    assert err and "0.2 deg" in err and "fMinMove_deg" in err
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=0.2), lim) is None
    assert L.validate_move(L.MoveRequest(True, dtheta_deg=-0.2), lim) is None
    cfg = {"move": {"min_distance_mm": 5.0, "min_angle_deg": 1.0}}
    lim5 = L.MoveLimits.from_config(cfg)
    assert (lim5.min_distance_mm, lim5.min_angle_deg) == (5.0, 1.0)
    assert L.validate_move(L.MoveRequest(False, dx_mm=4.0), lim5) is not None
    assert L.MoveLimits.from_config({}) == L.MoveLimits()


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("field", ["dx_mm", "dy_mm", "speed_mms", "rot_speed_degs", "accel_mms2"])
def test_non_finite_translation_values_rejected(field, bad):
    kw = {"dx_mm": 100.0, "dy_mm": 0.0, "speed_mms": 150.0, "rot_speed_degs": 10.0, "accel_mms2": 200.0}
    kw[field] = bad
    err = L.validate_move(L.MoveRequest(False, **kw))
    assert err is not None and "not a finite number" in err


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("field", ["dtheta_deg", "speed_mms", "rot_speed_degs", "accel_mms2"])
def test_non_finite_rotation_values_rejected(field, bad):
    kw = {"dtheta_deg": 90.0, "speed_mms": 150.0, "rot_speed_degs": 10.0, "accel_mms2": 200.0}
    kw[field] = bad
    err = L.validate_move(L.MoveRequest(True, **kw))
    assert err is not None and "not a finite number" in err


def test_non_finite_value_that_is_not_sent_is_ignored():
    # rotation: build_move_command sends fMoveX/Y = 0, so a stale NaN dx is irrelevant; the reverse for theta
    assert L.validate_move(L.MoveRequest(True, dx_mm=math.nan, dtheta_deg=90.0)) is None
    assert L.build_move_command(L.MoveRequest(True, dx_mm=math.nan, dtheta_deg=90.0), 1)["fMoveX_mm"] == 0.0
    assert L.validate_move(L.MoveRequest(False, dx_mm=100.0, dtheta_deg=math.nan)) is None


# ── command ids ────────────────────────────────────────────────────────────────

def test_next_move_id_increments():
    assert L.next_move_id(0, 0) == 1
    assert L.next_move_id(1, 1) == 2
    assert L.next_move_id(41, 41) == 42


def test_next_move_id_wraps_and_skips_zero():
    assert L.next_move_id(65535, 65535) == 1
    assert L.next_move_id(65535, 0) == 1
    assert L.next_move_id(65535, 1) == 2


def test_next_move_id_never_equals_plc_ack():
    # HMI restarted (last_sent 0) while the PLC still holds id 1 from an earlier session
    assert L.next_move_id(0, 1) == 2
    for last in range(0, 70000, 997):
        for ack in (last & 0xFFFF, (last + 1) & 0xFFFF, 0, 1, 65535):
            nid = L.next_move_id(last, ack)
            assert 1 <= nid <= 65535 and nid != ack and nid != (last & 0xFFFF)


# ── HALT / standby / jog ───────────────────────────────────────────────────────

def test_halt_is_one_write_with_abort_and_jog_release():
    v2 = L.halt_values(pv.IF_V2)
    assert v2["bCmdMoveAbort"] is True
    assert all(v2[f] is False for f in pv.JOG_FIELDS)
    assert "bCmdStop" not in v2           # HALT must not switch the drives off
    v1 = L.halt_values(pv.IF_LEGACY)
    assert "bCmdMoveAbort" not in v1 and all(v1[f] is False for f in pv.JOG_FIELDS)


def test_standby_values():
    assert L.standby_values() == {"bCmdStop": True, "bCmdManualMode": False, "bCmdAutoMode": False}


def test_jog_values_complete_state():
    vals = L.jog_values({"bCmdJogFwd"})
    assert set(vals) == set(pv.JOG_FIELDS)
    assert vals["bCmdJogFwd"] is True and sum(vals.values()) == 1


def test_v2_status_fields_rev2():
    v2 = pv.from_plc_vars(pv.IF_V2)
    assert v2["eMoveCmdResult"] == "INT" and v2["bMoveLimited"] == "BOOL"
    assert "eMoveCmdResult" not in pv.from_plc_vars(pv.IF_LEGACY)
    st = pv.default_status(pv.IF_V2)
    assert st["eMoveCmdResult"] == 0 and st["bMoveLimited"] is False


def test_safe_defaults():
    v1 = pv.safe_default_values(pv.IF_LEGACY)
    v2 = pv.safe_default_values(pv.IF_V2)
    for f in pv.JOG_FIELDS + ("bCmdHorn", "bCmdManualMode", "bCmdAutoMode"):
        assert v1[f] is False and v2[f] is False
    assert v2["bCmdMoveStart"] is False and v2["bCmdMoveAbort"] is False
    assert "bCmdMoveStart" not in v1
    assert "bCmdSafetyRun" not in v2      # never dropped by a reconnect
    assert set(v2) <= set(pv.to_plc_vars(pv.IF_V2)) and set(v1) <= set(pv.to_plc_vars(pv.IF_LEGACY))


# ── ack / target ───────────────────────────────────────────────────────────────

def test_move_ack_state():
    """Spec section 9: ack = nMoveCmdAck == sent id, verdict = eMoveCmdResult (not eMoveResult)."""
    assert L.move_ack_state({"nMoveCmdAck": 3, "eMoveCmdResult": 0}, 4) == L.ACK_PENDING
    assert L.move_ack_state({"nMoveCmdAck": 3, "eMoveCmdResult": 13}, 4) == L.ACK_PENDING   # verdict of another id
    assert L.move_ack_state({"nMoveCmdAck": 4, "eMoveCmdResult": 0}, 4) == L.ACK_ACCEPTED
    for res in (10, 11, 12, 13):
        assert L.move_ack_state({"nMoveCmdAck": 4, "eMoveCmdResult": res}, 4) == L.ACK_REJECTED
    # unknown verdicts are never treated as accepted
    for res in (1, 2, 20, 29, 99, -1):
        assert L.move_ack_state({"nMoveCmdAck": 4, "eMoveCmdResult": res}, 4) == L.ACK_INVALID
    assert L.move_ack_state({"nMoveCmdAck": 4}, 4) == L.ACK_INVALID                       # field missing


def test_move_ack_uses_command_verdict_not_move_result():
    # second command while a move runs: PLC rejects it (REJ_BUSY) but eMoveResult still describes the running move
    running = {"nMoveCmdAck": 6, "eMoveCmdResult": C.RES_REJ_BUSY, "eMoveResult": C.RES_NONE}
    assert L.move_ack_state(running, 6) == L.ACK_REJECTED
    # accepted command while eMoveResult still shows the previous rejection (published at the next state change)
    accepted = {"nMoveCmdAck": 7, "eMoveCmdResult": C.RES_NONE, "eMoveResult": C.RES_REJ_PARAM}
    assert L.move_ack_state(accepted, 7) == L.ACK_ACCEPTED


def test_target_pose():
    req = L.MoveRequest(False, dx_mm=1000.0, dy_mm=0.0)
    assert L.target_pose((0.0, 0.0, 0.0), req) == pytest.approx((1.0, 0.0, 0.0))
    # robot at 90 deg: robot-forward = odom +Y
    assert L.target_pose((1.0, 2.0, 90.0), req) == pytest.approx((1.0, 3.0, 90.0))
    req_y = L.MoveRequest(False, dx_mm=0.0, dy_mm=500.0)
    assert L.target_pose((0.0, 0.0, 90.0), req_y) == pytest.approx((-0.5, 0.0, 90.0))
    rot = L.MoveRequest(True, dtheta_deg=-90.0)
    assert L.target_pose((0.3, 0.4, 10.0), rot) == pytest.approx((0.3, 0.4, -80.0))


# ── enable rules ───────────────────────────────────────────────────────────────

def test_go_enabled_happy_path():
    assert L.go_enabled(MANUAL_V2, True, pv.IF_V2) == (True, "")


@pytest.mark.parametrize("status, connected, ver, perr, awaiting", [
    (MANUAL_V2, False, 2, None, False),
    (MANUAL_V2, True, 1, None, False),
    ({**MANUAL_V2, "bExtActive": True}, True, 2, None, False),
    ({**MANUAL_V2, "eAmrState": C.ST_READY}, True, 2, None, False),
    ({**MANUAL_V2, "bMoveActive": True}, True, 2, None, False),
    (MANUAL_V2, True, 2, "distance < 1 mm", False),
    (MANUAL_V2, True, 2, None, True),
])
def test_go_disabled(status, connected, ver, perr, awaiting):
    ok, reason = L.go_enabled(status, connected, ver, perr, awaiting)
    assert ok is False and reason


def test_jog_enabled():
    assert L.jog_enabled(MANUAL_V2, True)
    assert L.jog_enabled({"eAmrState": C.ST_MANUAL}, True)            # legacy status: no bExtActive key
    assert not L.jog_enabled(MANUAL_V2, False)
    assert not L.jog_enabled({**MANUAL_V2, "bExtActive": True}, True)
    assert not L.jog_enabled({**MANUAL_V2, "bMoveActive": True}, True)
    for s in range(0, 17):
        if s != C.ST_MANUAL:
            assert not L.jog_enabled({**MANUAL_V2, "eAmrState": s}, True)


def test_startup_step():
    assert L.startup_step(C.ST_INIT, False) == 0
    assert L.startup_step(C.ST_SAFETY_STOP, True) == 0
    assert L.startup_step(C.ST_RESET_REQUIRED, True) == 1
    assert L.startup_step(C.ST_RESET_REQUIRED, False) == 0
    assert L.startup_step(C.ST_PRECHARGE, True) == 2
    assert L.startup_step(C.ST_STANDBY, True) == 2
    assert L.startup_step(C.ST_READY, True) == 3
    assert L.startup_step(C.ST_MANUAL, True) == 4


def test_button_enables():
    b = L.button_enables(C.ST_STANDBY, True)
    assert b["start"] and not b["standby"] and not b["manual"]
    b = L.button_enables(C.ST_PRECHARGE, True)     # r5 1.5: 16 is PRECHARGE, Stop has no effect there
    assert not b["standby"] and not b["start"]
    b = L.button_enables(C.ST_MANUAL, True)
    assert b["manual"] and b["standby"] and not b["reset"]
    assert L.button_enables(C.ST_SAFETY_STOP, True)["rearm"]
    assert not L.button_enables(C.ST_SAFETY_STOP, False)["rearm"]
    assert L.button_enables(C.ST_ERROR, True)["reset"]


def test_state_table_complete():
    assert set(C.AMR_STATES) == set(range(17))
    assert C.state_info(16)[1] == "PRECHARGE"
    assert C.state_info(99)[1] == "UNKNOWN"
    assert C.state_info(None)[1] == "UNKNOWN"
    for r in (10, 11, 12, 13):
        assert C.is_reject(r)
    assert not C.is_reject(1) and C.is_abort(28) and not C.is_abort(2)
    assert C.RES_ABORT_ODOM == 29 and C.is_abort(29)
    assert C.move_result_text(29) == "Aborted: measured progress implausible (odometry)"
    assert set(C.MOVE_RESULTS) == {0, 1, 2, 10, 11, 12, 13} | set(range(20, 30))   # E_MoveResult complete
    assert all(math.isfinite(v) for v in (C.ROBOT_LENGTH_M, C.ROBOT_WIDTH_M))


def test_run_lock_blocks_jog_and_go():
    """Mauer HMI (2026-10-07): a Mauer REAL run locks jog and GO with a reason; the default keeps amr behaviour."""
    assert L.jog_enabled(MANUAL_V2, True) and L.jog_enabled(MANUAL_V2, True, run_lock="")
    assert not L.jog_enabled(MANUAL_V2, True, run_lock="Mauer REAL run active")
    assert L.go_enabled(MANUAL_V2, True, pv.IF_V2) == (True, "")
    assert L.go_enabled(MANUAL_V2, True, pv.IF_V2, run_lock="Mauer REAL run active") == \
        (False, "Mauer REAL run active")
    assert L.go_enabled(MANUAL_V2, False, pv.IF_V2, run_lock="x") == (False, "not connected")   # connection first
