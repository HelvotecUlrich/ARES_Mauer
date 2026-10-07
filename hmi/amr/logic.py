"""
logic.py - Pure (Qt-free) HMI logic: move command building, command ids, enable rules, startup step.

Spec: 10_robot/hmi/SPEC_2026-09-25_Relativfahrt_HMI_SPS_v2.md sections 5 and 6.
Everything here is unit-tested without Qt or ADS (tests/test_logic.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple

from .plc_vars import IF_V2, JOG_FIELDS
from .ui import constants as C

UINT_MAX = 0xFFFF


# ── Relative move ──────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class MoveLimits:
    """HMI-side sanity limits (mirror of the GVL_Move init values, spec 3.6 / section 9; the PLC enforces its own).

    min_distance_mm / min_angle_deg mirror GVL_Move.fMinMove_mm (2 mm) / fMinMove_deg (0.2 deg): smaller moves
    would end inside the stop tolerance and are rejected by the PLC (REJ_PARAM).
    """
    max_distance_mm: float = 20000.0
    max_angle_deg: float = 720.0
    max_speed_mms: float = 500.0
    max_rot_speed_degs: float = 45.0
    max_accel_mms2: float = 1000.0
    min_distance_mm: float = 2.0
    min_angle_deg: float = 0.2

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "MoveLimits":
        sec = (cfg or {}).get("move") or {}
        base = cls()
        return cls(
            max_distance_mm=float(sec.get("max_distance_mm", base.max_distance_mm)),
            max_angle_deg=float(sec.get("max_angle_deg", base.max_angle_deg)),
            max_speed_mms=float(sec.get("max_speed_mms", base.max_speed_mms)),
            max_rot_speed_degs=float(sec.get("max_rot_speed_degs", base.max_rot_speed_degs)),
            max_accel_mms2=float(sec.get("max_accel_mms2", base.max_accel_mms2)),
            min_distance_mm=float(sec.get("min_distance_mm", base.min_distance_mm)),
            min_angle_deg=float(sec.get("min_angle_deg", base.min_angle_deg)),
        )


@dataclass(frozen=True)
class MoveDefaults:
    """Panel defaults (spec 6.5)."""
    distance_mm: float = 1000.0
    angle_deg: float = 90.0
    speed_mms: float = 150.0
    rot_speed_degs: float = 10.0
    accel_mms2: float = 200.0
    test_distance_mm: float = 100.0   # spec 6.7: test move physical Left 100 mm at 50 mm/s
    test_speed_mms: float = 50.0

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "MoveDefaults":
        sec = (cfg or {}).get("move") or {}
        base = cls()
        return cls(
            distance_mm=float(sec.get("default_distance_mm", base.distance_mm)),
            angle_deg=float(sec.get("default_angle_deg", base.angle_deg)),
            speed_mms=float(sec.get("default_speed_mms", base.speed_mms)),
            rot_speed_degs=float(sec.get("default_rot_speed_degs", base.rot_speed_degs)),
            accel_mms2=float(sec.get("default_accel_mms2", base.accel_mms2)),
            test_distance_mm=float(sec.get("test_distance_mm", base.test_distance_mm)),
            test_speed_mms=float(sec.get("test_speed_mms", base.test_speed_mms)),
        )


@dataclass(frozen=True)
class MoveRequest:
    """One relative move in the PLC frame. Translation (dx, dy) and rotation (dtheta) are exclusive."""
    is_rotation: bool
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    dtheta_deg: float = 0.0
    speed_mms: float = 150.0
    rot_speed_degs: float = 10.0
    accel_mms2: float = 200.0

    @property
    def distance_mm(self) -> float:
        return math.hypot(self.dx_mm, self.dy_mm)


def _not_finite(req: MoveRequest) -> Optional[str]:
    """Name of the first non-finite value that build_move_command() would send, else None."""
    fields = [("angle", req.dtheta_deg)] if req.is_rotation else [("dx", req.dx_mm), ("dy", req.dy_mm)]
    fields += [("speed", req.speed_mms), ("rotation speed", req.rot_speed_degs), ("acceleration", req.accel_mms2)]
    for name, value in fields:
        try:
            if math.isfinite(float(value)):
                continue
        except (TypeError, ValueError):
            pass
        return name
    return None


def validate_move(req: MoveRequest, limits: MoveLimits = MoveLimits()) -> Optional[str]:
    """Return None if the request is valid, otherwise a short reason text.

    Mirrors the PLC checks (MOVE_PRG, spec section 9): finite numbers only, minimum move
    (GVL_Move.fMinMove_mm / fMinMove_deg), maximum distance / angle / speed / acceleration.
    """
    bad = _not_finite(req)
    if bad is not None:
        return f"{bad} is not a finite number (NaN/inf)"
    if req.accel_mms2 <= 0.0:
        return "acceleration must be > 0"
    if req.accel_mms2 > limits.max_accel_mms2:
        return f"acceleration > {limits.max_accel_mms2:.0f} mm/s2"
    if req.is_rotation:
        a = abs(req.dtheta_deg)
        if a < limits.min_angle_deg:
            return (f"angle {a:.2f} deg is below the minimum {limits.min_angle_deg:g} deg "
                    "(PLC GVL_Move.fMinMove_deg)")
        if a > limits.max_angle_deg:
            return f"angle > {limits.max_angle_deg:.0f} deg"
        if req.rot_speed_degs <= 0.0:
            return "rotation speed must be > 0"
        if req.rot_speed_degs > limits.max_rot_speed_degs:
            return f"rotation speed > {limits.max_rot_speed_degs:.0f} deg/s"
    else:
        d = req.distance_mm
        if d < limits.min_distance_mm:
            return (f"distance {d:.1f} mm is below the minimum {limits.min_distance_mm:g} mm "
                    "(PLC GVL_Move.fMinMove_mm)")
        if d > limits.max_distance_mm:
            return f"distance > {limits.max_distance_mm:.0f} mm"
        if req.speed_mms <= 0.0:
            return "speed must be > 0"
        if req.speed_mms > limits.max_speed_mms:
            return f"speed > {limits.max_speed_mms:.0f} mm/s"
    return None


def build_move_command(req: MoveRequest, move_id: int) -> Dict[str, Any]:
    """Field dict for ONE write_list_by_name that starts a move (bCmdMoveStart is reset 300 ms later).

    Translation -> fMoveTheta_deg = 0; rotation -> fMoveX_mm = fMoveY_mm = 0 (PLC rejects both at once).
    """
    if not 1 <= int(move_id) <= UINT_MAX:
        raise ValueError(f"move id out of range: {move_id}")
    rot = bool(req.is_rotation)
    return {
        "fMoveX_mm": 0.0 if rot else float(req.dx_mm),
        "fMoveY_mm": 0.0 if rot else float(req.dy_mm),
        "fMoveTheta_deg": float(req.dtheta_deg) if rot else 0.0,
        "fMoveSpeed_mms": float(req.speed_mms),
        "fMoveRotSpeed_degs": float(req.rot_speed_degs),
        "fMoveAccel_mms2": float(req.accel_mms2),
        "nMoveCmdId": int(move_id),
        "bCmdMoveStart": True,
    }


def next_move_id(last_sent: int, plc_ack: int) -> int:
    """Next nMoveCmdId: last_sent + 1, wraps 65535 -> 1, never 0, never equal to the PLC's last id.

    The PLC starts a move only if nMoveCmdId differs from the last processed id (= nMoveCmdAck) and starts
    with 0 after a PLC restart, so 0 is skipped and the ack of a previous HMI session is avoided.
    """
    last = int(last_sent) & UINT_MAX
    ack = int(plc_ack) & UINT_MAX
    cand = last
    for _ in range(3):
        cand = cand % UINT_MAX + 1        # 1..65535
        if cand != ack and cand != last:
            return cand
    raise AssertionError("unreachable")


def halt_values(version: int) -> Dict[str, bool]:
    """HALT (spec 6.2): ONE write, bCmdMoveAbort TRUE (v2 only) and all jog bits FALSE. Not bCmdStop."""
    vals: Dict[str, bool] = {f: False for f in JOG_FIELDS}
    if version >= IF_V2:
        vals["bCmdMoveAbort"] = True
    return vals


def standby_values() -> Dict[str, bool]:
    """'Standby (drives off)' = old Stop: bCmdStop pulse + mode requests FALSE, in one write."""
    return {"bCmdStop": True, "bCmdManualMode": False, "bCmdAutoMode": False}


def jog_values(active_fields: set) -> Dict[str, bool]:
    """Complete jog state (all six bits) so that every write also repairs a lost release."""
    return {f: (f in active_fields) for f in JOG_FIELDS}


ACK_PENDING = "pending"
ACK_ACCEPTED = "accepted"
ACK_REJECTED = "rejected"
ACK_INVALID = "invalid"


def move_ack_state(status: Mapping[str, Any], sent_id: int) -> str:
    """Verdict on the move command sent with nMoveCmdId = sent_id (spec section 9).

    Acknowledged when nMoveCmdAck == sent_id; the verdict is eMoveCmdResult (written by the PLC in the same cycle
    as the ack, read in the same sum-read): 0 -> 'accepted', 10..13 -> 'rejected', anything else -> 'invalid'
    (never treated as accepted). eMoveResult is the result of the MOVE, not of the command.
    """
    if int(status.get("nMoveCmdAck", -1)) != int(sent_id):
        return ACK_PENDING
    verdict = int(status.get("eMoveCmdResult", -1))
    if verdict == C.RES_NONE:
        return ACK_ACCEPTED
    if C.is_reject(verdict):
        return ACK_REJECTED
    return ACK_INVALID


def target_pose(start: Tuple[float, float, float], req: MoveRequest) -> Tuple[float, float, float]:
    """Target (x_m, y_m, theta_deg) in the odometry frame from the start pose and the commanded move.

    dx/dy are relative to the robot at the start pose (PLC: d = R(-theta0) (P - P0)).
    """
    x0, y0, th0 = start
    if req.is_rotation:
        return x0, y0, th0 + req.dtheta_deg
    t = math.radians(th0)
    dx, dy = req.dx_mm / 1000.0, req.dy_mm / 1000.0
    return (x0 + dx * math.cos(t) - dy * math.sin(t),
            y0 + dx * math.sin(t) + dy * math.cos(t),
            th0)


# ── Enable rules ───────────────────────────────────────────────────────────────

def jog_enabled(status: Mapping[str, Any], connected: bool, run_lock: str = "") -> bool:
    """Jog only in MANUAL, not while ext (C6030) controls the robot and not while a relative move runs.
    run_lock (Mauer HMI, 2026-10-07): reason why a Mauer REAL run locks jog; "" = no lock."""
    return (bool(connected)
            and not run_lock
            and int(status.get("eAmrState", -1)) == C.ST_MANUAL
            and not bool(status.get("bExtActive", False))
            and not bool(status.get("bMoveActive", False)))


def go_enabled(status: Mapping[str, Any], connected: bool, if_version: int,
               param_error: Optional[str] = None, awaiting_ack: bool = False,
               run_lock: str = "") -> Tuple[bool, str]:
    """GO allowed? Returns (enabled, reason-if-not). run_lock (Mauer HMI): reason of a Mauer REAL run lock."""
    if not connected:
        return False, "not connected"
    if run_lock:
        return False, run_lock
    if if_version < IF_V2:
        return False, "PLC without relative move (interface v1) - load PLC build v2"
    if bool(status.get("bExtActive", False)):
        return False, "external control (C6030) active"
    if int(status.get("eAmrState", -1)) != C.ST_MANUAL:
        return False, "AMR not in MANUAL mode"
    if bool(status.get("bMoveActive", False)):
        return False, "a move is running"
    if awaiting_ack:
        return False, "waiting for PLC acknowledgement"
    if param_error:
        return False, param_error
    return True, ""


# Startup sequence: 0 Safety Run -> 1 AMR Reset -> 2 Start -> 3 Manual -> 4 done (MANUAL)
STARTUP_STEPS = ("Safety Run", "AMR Reset", "Start", "Manual")


def startup_step(state: int, safety_run: bool) -> int:
    """Index of the step the operator has to do next (4 = sequence complete)."""
    s = int(state)
    if s == C.ST_MANUAL:
        return 4
    if s in (C.ST_READY, C.ST_AUTO, C.ST_NAVIGATING):
        return 3
    if s in (C.ST_STANDBY, C.ST_DRIVES_ENABLE, C.ST_PRECHARGE):
        return 2
    if s in (C.ST_RESET_REQUIRED, C.ST_ERROR, C.ST_ERROR_ACK):
        return 1 if safety_run else 0
    return 0


def startup_hint(state: int, safety_run: bool) -> str:
    """One-line operator hint for the current state."""
    s = int(state)
    if not safety_run and s in (C.ST_INIT, C.ST_WAIT_SAFETY, C.ST_SAFETY_STOP, C.ST_RESET_REQUIRED):
        return "Switch Safety Run ON (TwinSAFE start-up)."
    if s == C.ST_SAFETY_STOP:
        return "Safety stop: release the E-stop, then 'Re-arm Safety'."
    if s == C.ST_WAIT_SAFETY:
        return "Waiting for TwinSAFE (SAFETY_OK)..."
    if s == C.ST_RESET_REQUIRED:
        return "Press 'AMR Reset' (safety was lost)."
    if s == C.ST_ERROR:
        return "ERROR: check drives, then 'AMR Reset'."
    if s == C.ST_ERROR_ACK:
        return "Error acknowledge running..."
    if s == C.ST_PRECHARGE:
        return "Pre-charge running (waiting for TwinSAFE precharge signal)..."
    if s == C.ST_STANDBY:
        return "Press 'Start' (enables the drives)."
    if s == C.ST_DRIVES_ENABLE:
        return "Enabling drives..."
    if s == C.ST_READY:
        return "Switch 'Manual' ON for jog / relative move."
    if s == C.ST_MANUAL:
        return "MANUAL: jog and relative move available."
    if s == C.ST_AUTO:
        return "AUTO mode: HMI motion is disabled."
    return ""


def button_enables(state: int, safety_run: bool) -> Dict[str, bool]:
    """Enable state of the startup-sequence buttons (FB_AMR_StateMachine transitions, r3 1.4)."""
    s = int(state)
    return {
        "safety_run": True,
        "rearm": s == C.ST_SAFETY_STOP and bool(safety_run),
        "reset": s in (C.ST_RESET_REQUIRED, C.ST_ERROR),
        "start": s == C.ST_STANDBY,
        "manual": s in (C.ST_READY, C.ST_MANUAL),
        "auto": s in (C.ST_READY, C.ST_AUTO, C.ST_NAVIGATING),
        # bStopRequest is only evaluated in DRIVES_ENABLE, READY, MANUAL, AUTO
        "standby": s in (C.ST_DRIVES_ENABLE, C.ST_READY, C.ST_MANUAL, C.ST_AUTO),
    }
