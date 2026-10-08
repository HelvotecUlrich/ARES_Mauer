"""ARES relative move over ADS: client for the CX9240 PLC (build >= v2.1, HMI interface v2).

ARES drives a relative move by itself once commanded: a translation (dx, dy) in its body frame at the start pose, or
a rotation dtheta on the spot, never both in one command. The PLC closes the loop on wheel odometry, so wheel slip is
invisible to it (MA 10_robot/twincat/PLC_CX9240_v2/README.md:127-128) - the camera corrects that, not this module.
Contract: MA 10_robot/hmi/SPEC_2026-09-25_Relativfahrt_HMI_SPS_v2.md (sections 3, 5, 9); PLC src/MOVE_PRG.st,
src/FB_RelMove.st, src/GVL_Move.st; overlay POUs/FB_HMI_Interface.TcPOU. "MA" = the Masterarbeit repo (read only).

Operating pattern A (proven in E003, 01.10.2026, MA 10_robot/tools/imu_cal_run.py:1-14): the HMI amr_hmi v2 stays
open and connected. It owns the heartbeat (stToPlc.nHeartbeat every 100 ms), the MANUAL level bit and HALT. This
client writes ONLY fMove*, nMoveCmdId, bCmdMoveStart and bCmdMoveAbort (enforced by WRITABLE). It never writes mode,
jog, heartbeat, bCmdStop or "safe defaults": clearing bCmdManualMode drops MANUAL and aborts the move (22), and
bCmdStop switches the drives off via STANDBY without a PLC ramp (SPEC:294-299).

Handshake (MOVE_PRG.st:99-249, FB_RelMove.st:127-387):
1. preflight: two status reads 250 ms apart (interface/build, MANUAL, no external control, HMI heartbeat changing,
   no move running, no abort/stop/jog bit). A move is never sent without a passing preflight.
2. parameters + new nMoveCmdId with bCmdMoveStart FALSE, then ~50 ms later bCmdMoveStart TRUE (guaranteed rising
   edge even if a crashed client left the bit TRUE; the PLC cannot see the edge before the parameters).
3. acknowledgement: nMoveCmdAck == id with the verdict eMoveCmdResult (0 accepted, 10..13 rejected), confirmed by a
   second read; the start bit is reset after the ack or after 300 ms, whichever comes first.
4. done: bMoveActive seen TRUE at least once, then FALSE with eMoveState DONE (4) or ABORTED (6). ADS reads are not
   guaranteed to be cycle-consistent (MA ares_plc_bridge/INTERFACE.md section 8), so a torn read that shows the new
   ack next to the old DONE never counts as done.
5. client deadline = PLC timeout 2 (d/v + v/a) + 10 s, + tAbortMax 10 s, + margin; on expiry a bCmdMoveAbort pulse.

Frames and units: PLC body frame at the start pose, +x forward, +y left, +theta CCW (MA DECISIONS.md D21, verified on
the robot 28.09.2026 with PLC v2.1) = ARES base_link (docs/ARCHITECTURE.md). Distances mm, angles deg (PLC native),
speeds mm/s and deg/s, acceleration mm/s^2 at the wheel. The odometry pose is stFromPlc.fPosX_m / fPosY_m (converted
to mm) and fPosTheta_deg (wrapped to (-180, 180]): wheel odometry since the last reset, not a localisation.

Only read_list_by_name / write_list_by_name (ADS sum read / sum write) are used, so any object with these methods
works (pyads.Connection, tests/fake_plc.py). One AresAds per thread; abort() may be called from another thread
(connection calls are serialised by a lock), inhibit() / release_inhibit() from any thread: the HALT latch of the
Mauer HMI (review 2026-10-08) - while set, move() writes no start edge (AresInhibited).
"""
from __future__ import annotations

import math
import re
import socket
import threading
import time
from dataclasses import dataclass, field, fields
from typing import Any, Callable, Mapping, Sequence

# ── ADS symbols ───────────────────────────────────────────────────────────────
TO = "GVL_HMI.stToPlc."          # HMI -> PLC, MA overlay/DUTs/ST_HMI_ToPlc.TcDUT
FROM = "GVL_HMI.stFromPlc."      # PLC -> HMI, MA overlay/DUTs/ST_HMI_FromPlc.TcDUT

# The only stToPlc fields this client ever writes (pattern A, see module docstring).
WRITABLE = frozenset({
    "fMoveX_mm", "fMoveY_mm", "fMoveTheta_deg", "fMoveSpeed_mms", "fMoveRotSpeed_degs", "fMoveAccel_mms2",
    "nMoveCmdId", "bCmdMoveStart", "bCmdMoveAbort",
})
JOG_FIELDS = ("bCmdJogFwd", "bCmdJogBwd", "bCmdJogLeft", "bCmdJogRight", "bCmdJogRotLeft", "bCmdJogRotRight")

ADS_TCP_PORT = 48898             # AMS router TCP port of the target (MA ares_plc_bridge/transport.py:51)
ADS_STATE_RUN = 5                # pyads.ADSSTATE_RUN
ADSERR_SYMBOL_NOT_FOUND = 1808   # pyads.ADSError.err_code (MA amr_hmi/ads_worker.py:43)
TCP_PROBE_MAX_S = 1.0            # pyads blocks ~20 s per connect to an unreachable target (measured 02.10.2026,
                                 # MA transport.py:55-66), so a plain TCP connect is tried first, 0.3..1 s

# ── PLC enums (MA src/E_MoveState.st, src/E_MoveResult.st, amr_hmi/ui/constants.py:17-135) ──────────────────────
ST_MANUAL = 7
AMR_STATE_NAMES = {
    0: "INIT", 1: "WAIT SAFETY", 2: "SAFETY STOP", 3: "RESET REQUIRED", 4: "STANDBY", 5: "DRIVES ENABLE",
    6: "READY", 7: "MANUAL MODE", 8: "AUTO MODE", 9: "NAVIGATING", 10: "OBSTACLE STOP", 11: "DOCKING",
    12: "CHARGING", 13: "ERROR", 14: "ERROR ACK", 15: "SHUTDOWN", 16: "PRECHARGE",
}
MOVE_IDLE, MOVE_ALIGN, MOVE_RUN, MOVE_SETTLE, MOVE_DONE, MOVE_ABORTING, MOVE_ABORTED = range(7)
MOVE_STATE_NAMES = {0: "IDLE", 1: "ALIGN", 2: "RUN", 3: "SETTLE", 4: "DONE", 5: "ABORTING", 6: "ABORTED"}
ACTIVE_STATES = frozenset({MOVE_ALIGN, MOVE_RUN, MOVE_SETTLE, MOVE_ABORTING})   # = FB_RelMove.bOwns
END_STATES = frozenset({MOVE_DONE, MOVE_ABORTED})

RES_NONE, RES_OK, RES_OK_TOL = 0, 1, 2
RES_REJ_PARAM, RES_REJ_STATE, RES_REJ_EXT, RES_REJ_BUSY = 10, 11, 12, 13
RES_ABORT_USER, RES_ABORT_HMI, RES_ABORT_TIMEOUT = 20, 26, 27
MOVE_RESULT_TEXTS = {
    0: "-",
    1: "done, final error within tolerance",
    2: "done, final error OUTSIDE tolerance",
    10: "rejected: parameters (invalid, too small/large, odometry parameters or pose invalid)",
    11: "rejected: AMR not ready / not MANUAL / test mode / HMI heartbeat",
    12: "rejected: external control (C6030) active",
    13: "rejected: busy (move/SysId/jog/abort/stop active or robot still moving)",
    20: "aborted: HALT / bCmdMoveAbort / bCmdStop",
    21: "aborted: jog command",
    22: "aborted: MANUAL left / test mode / drives not ready",
    23: "aborted: safety stop",
    24: "aborted: system or drive fault",
    25: "aborted: external control became active",
    26: "aborted: HMI heartbeat lost",
    27: "aborted: timeout (steering alignment or total time)",
    28: "aborted: drives do not follow (stall)",
    29: "aborted: odometry (progress implausible, parameters or pose invalid)",
}
REJECT_CODES = frozenset({10, 11, 12, 13})
OK_CODES = frozenset({RES_OK, RES_OK_TOL})

# ── PLC limits: GVL_Move.st init values (design values; the PLC enforces its own) ───────────────────────────────
MIN_MOVE_MM = 2.0                # fMinMove_mm (GVL_Move.st:24): smaller translations are rejected (10)
MIN_MOVE_DEG = 0.2               # fMinMove_deg
MAX_DIST_MM = 20000.0            # fMaxDist_mm
MAX_ANGLE_DEG = 720.0            # fMaxAngle_deg
MAX_SPEED_MMS = 500.0            # fMaxSpeed_mms (the PLC clamps and sets bMoveLimited; refused here like the HMI)
MAX_ROT_SPEED_DEGS = 45.0        # fMaxRotSpeed_degs
MAX_ACCEL_MMS2 = 1000.0          # fMaxAccel_mms2
MIN_ACCEL_MMS2 = 20.0            # fMinAccel_mms2 (smaller values are raised by the PLC)
CREEP_SPEED_MMS = 5.0            # fCreepSpeed_mms (FB_RelMove.st:155: MAX(creep, 2))
CREEP_ROT_DEGS = 1.0             # fCreepRot_degs (FB_RelMove.st:149: MAX(creep, 0.5))
TIMEOUT_FACTOR = 2.0             # fTimeoutFactor, FB_RelMove.st:168
TIMEOUT_EXTRA_S = 10.0           # fTimeoutExtra_s
T_ABORT_MAX_S = 10.0             # tAbortMax: longest ABORTING phase
ARM_MM = 353.625                 # |x| of the steering axes, GVL_Odom.fModuleX_L/R_mm (rotation: wheel = omega * arm);
                                 # MA imu_cal_run.py:57, ARES_Mauer config [ares] steer_axis_x (CONFIRMED)

# ── client timing ─────────────────────────────────────────────────────────────
PULSE_S = 0.3                    # HMI pulse length (MA amr_hmi/ads_worker.py:40 PULSE_MS)
EDGE_DELAY_S = 0.05              # parameters first, then the edge (>> 1 ms PLC cycle)
ACK_TIMEOUT_S = 2.0              # ASSUMPTION: the PLC acks in the cycle of the edge; 2 s covers ADS timeouts
ACK_POLL_S = 0.05
POLL_S = 0.1                     # status poll during the move (as amr_hmi and imu_cal_run.py:248)
HB_WINDOW_S = 0.25               # heartbeat must change within this window (HMI writes every 100 ms)
DEADLINE_MARGIN_S = 15.0         # ASSUMPTION: margin on top of PLC timeout + tAbortMax
ABORT_WAIT_S = T_ABORT_MAX_S + 2.0   # after a client abort: wait this long for ABORTED

DEFAULT_ROT_SPEED_DEGS = 10.0    # used if [ares_ads] rot_speed_degs is missing: E003 rotations at 10 deg/s
                                 # (MA imu_cal_run.py:133), amr_hmi default (logic.py:58)
DEFAULT_MIN_PLC_BUILD = "2.1"    # used if [ares_ads] min_plc_build is missing: +Y = left only since v2.1
                                 # (MA DECISIONS.md D21; v2.0 mirrored the kinematics, +Y went right)


# ── errors ────────────────────────────────────────────────────────────────────
class AresError(RuntimeError):
    """Base class of all errors of this module."""


class AresConnectionError(AresError):
    """Not reachable, ADS call failed, per-symbol ADS error, interface mismatch or no acknowledgement."""

    def __init__(self, msg: str, err_code: int | None = None) -> None:
        super().__init__(msg)
        self.err_code = err_code


class AresNotReady(AresError):
    """Preflight failed - no move command was written. `problems` lists the reasons."""

    def __init__(self, problems: Sequence[str]) -> None:
        self.problems = list(problems)
        super().__init__("ARES not ready for a move:\n  - " + "\n  - ".join(self.problems))


class MoveRefused(AresError, ValueError):
    """Move parameters refused locally (the PLC would reject them) - nothing was written."""


class AresInhibited(AresNotReady):
    """The HALT latch is set (AresAds.inhibit) - no start edge was written."""


class _NoAck(AresConnectionError):
    """The PLC did not acknowledge the start edge (abort already sent)."""


# ── data ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class OdomPose:
    """PLC wheel-odometry pose since the last odometry reset (not a localisation; slip invisible)."""
    x_mm: float
    y_mm: float
    theta_deg: float             # wrapped to (-180, 180] by the PLC

    def delta_to(self, after: "OdomPose") -> tuple[float, float, float]:
        """Displacement (dx_mm, dy_mm, dtheta_deg) from this pose to `after`, in the body frame at this pose -
        directly comparable with the commanded move (same formula as FB_RelMove.st:193-196)."""
        t = math.radians(self.theta_deg)
        dxw, dyw = after.x_mm - self.x_mm, after.y_mm - self.y_mm
        dth = (after.theta_deg - self.theta_deg + 180.0) % 360.0 - 180.0
        return (math.cos(t) * dxw + math.sin(t) * dyw, -math.sin(t) * dxw + math.cos(t) * dyw, dth)


# (attribute, ADS symbol, kind, scale) - one ADS sum read per status()
_STATUS_FIELDS: tuple[tuple[str, str, str, float], ...] = (
    ("if_version", FROM + "nIfVersion", "int", 1.0),
    ("amr_state", FROM + "eAmrState", "int", 1.0),
    ("amr_state_text", FROM + "sAmrStateText", "str", 1.0),
    ("hmi_watchdog_ok", FROM + "bHmiWatchdogOK", "bool", 1.0),
    ("plc_heartbeat", FROM + "nPlcHeartbeat", "int", 1.0),
    ("ext_active", FROM + "bExtActive", "bool", 1.0),
    ("amr_ready", FROM + "bAmrReady", "bool", 1.0),
    ("amr_moving", FROM + "bAmrMoving", "bool", 1.0),
    ("safety_ok", FROM + "bSafetyOK", "bool", 1.0),
    ("safety_stop_active", FROM + "bEmergencyStopActive", "bool", 1.0),
    ("drives_enabled", FROM + "bDrivesEnabled", "bool", 1.0),
    ("fault_active", FROM + "bFaultActive", "bool", 1.0),
    ("fault_text", FROM + "sFaultText", "str", 1.0),
    ("x_mm", FROM + "fPosX_m", "float", 1000.0),
    ("y_mm", FROM + "fPosY_m", "float", 1000.0),
    ("theta_deg", FROM + "fPosTheta_deg", "float", 1.0),
    ("odom_dist_mm", FROM + "fOdomDist_m", "float", 1000.0),
    ("move_state", FROM + "eMoveState", "int", 1.0),
    ("move_result", FROM + "eMoveResult", "int", 1.0),
    ("move_cmd_result", FROM + "eMoveCmdResult", "int", 1.0),
    ("move_cmd_ack", FROM + "nMoveCmdAck", "int", 1.0),
    ("move_active", FROM + "bMoveActive", "bool", 1.0),
    ("move_limited", FROM + "bMoveLimited", "bool", 1.0),
    ("move_is_rotation", FROM + "bMoveIsRotation", "bool", 1.0),
    ("move_text", FROM + "sMoveText", "str", 1.0),
    ("move_target", FROM + "fMoveTarget", "float", 1.0),
    ("move_done", FROM + "fMoveDone", "float", 1.0),
    ("move_remaining", FROM + "fMoveRemaining", "float", 1.0),
    ("move_lat_err_mm", FROM + "fMoveLatErr_mm", "float", 1.0),
    ("move_head_err_deg", FROM + "fMoveHeadErr_deg", "float", 1.0),
    ("move_progress_pct", FROM + "fMoveProgress_pct", "float", 1.0),
    ("move_final_err", FROM + "fMoveFinalErr", "float", 1.0),
    ("move_elapsed_s", FROM + "fMoveElapsed_s", "float", 1.0),
    # stToPlc, READ only: written by amr_hmi (heartbeat, accel, stop, jog) or by this client (start, abort)
    ("hmi_heartbeat", TO + "nHeartbeat", "int", 1.0),
    ("hmi_accel_mms2", TO + "fAccel_mms", "float", 1.0),
    ("start_bit", TO + "bCmdMoveStart", "bool", 1.0),
    ("abort_bit", TO + "bCmdMoveAbort", "bool", 1.0),
    ("stop_bit", TO + "bCmdStop", "bool", 1.0),
)
_JOG_SYMBOLS = tuple(TO + f for f in JOG_FIELDS)
_BUILD_SYMBOL = FROM + "sPlcBuild"


@dataclass
class AresStatus:
    """One consistent-as-ADS-allows snapshot of the HMI interface (one sum read). Units in the names; move_target /
    move_done / move_remaining / move_final_err are mm for a translation and deg for a rotation (move_is_rotation)."""
    t_s: float                   # client clock at the read
    build: str                   # stFromPlc.sPlcBuild ("" if unreadable)
    if_version: int
    amr_state: int
    amr_state_text: str
    hmi_watchdog_ok: bool        # FB_HMI_Interface 2 s watchdog (MANUAL is dropped when it expires)
    plc_heartbeat: int
    ext_active: bool             # C6030 / ROS bridge in control -> moves rejected (12) / aborted (25)
    amr_ready: bool
    amr_moving: bool
    safety_ok: bool
    safety_stop_active: bool
    drives_enabled: bool
    fault_active: bool
    fault_text: str
    x_mm: float                  # odometry pose (stFromPlc.fPosX_m * 1000)
    y_mm: float
    theta_deg: float
    odom_dist_mm: float          # travelled wheel path since reset
    move_state: int              # E_MoveState
    move_result: int             # E_MoveResult of the current/last move (shows a rejection while no move runs)
    move_cmd_result: int         # verdict on command move_cmd_ack: 0 accepted, 10..13 rejected
    move_cmd_ack: int            # last processed nMoveCmdId
    move_active: bool            # the move owns the setpoints (ALIGN, RUN, SETTLE, ABORTING)
    move_limited: bool
    move_is_rotation: bool
    move_text: str
    move_target: float
    move_done: float
    move_remaining: float
    move_lat_err_mm: float
    move_head_err_deg: float
    move_progress_pct: float
    move_final_err: float        # target - progress, + = short
    move_elapsed_s: float
    hmi_heartbeat: int           # stToPlc.nHeartbeat (written by amr_hmi)
    hmi_accel_mms2: float        # stToPlc.fAccel_mms (traction ramp written by amr_hmi; caps the move accel)
    start_bit: bool
    abort_bit: bool
    stop_bit: bool
    jog_active: bool             # any stToPlc.bCmdJog* bit TRUE
    raw: dict[str, Any] = field(default_factory=dict, repr=False)

    @property
    def amr_state_name(self) -> str:
        return AMR_STATE_NAMES.get(self.amr_state, f"state {self.amr_state}")

    @property
    def move_state_name(self) -> str:
        return MOVE_STATE_NAMES.get(self.move_state, f"state {self.move_state}")

    @property
    def move_result_text(self) -> str:
        return MOVE_RESULT_TEXTS.get(self.move_result, f"result {self.move_result}")

    @property
    def odom(self) -> OdomPose:
        return OdomPose(self.x_mm, self.y_mm, self.theta_deg)


@dataclass
class MoveOutcome:
    """Result of one move command that reached the PLC (accepted, rejected, done, aborted or client timeout).

    ok = accepted, seen running, ended DONE with result 1 (OK) or 2 (OK_TOL, final error > 5 mm / 0.5 deg - check
    final_err_mm), no client timeout and no other move command in between (the convention of MA imu_cal_run.py:258).
    """
    ok: bool
    cmd_id: int
    cmd_result: int              # eMoveCmdResult: 0 accepted, 10..13 rejected (PLC reason in text)
    state: int                   # E_MoveState at the end
    result: int                  # E_MoveResult: 1/2 done, 10..13 rejected, 20..29 aborted
    final_err_mm: float          # translation: fMoveFinalErr, target - progress (+ = short); NaN for a rotation
    text: str                    # PLC sMoveText + client notes
    elapsed_s: float             # PLC fMoveElapsed_s (duration of the move); 0 if not accepted
    limited: bool                # bMoveLimited: speed/accel reduced by the PLC
    odom_before: OdomPose | None
    odom_after: OdomPose | None
    kind: str = "translate"      # "translate" | "rotate"
    dx_mm: float = 0.0           # commanded move
    dy_mm: float = 0.0
    dtheta_deg: float = 0.0
    final_err_deg: float = math.nan   # rotation: fMoveFinalErr [deg]; NaN for a translation
    lat_err_mm: float = 0.0      # translation: lateral deviation (+ = left, odometry); rotation: position drift
    head_err_deg: float = 0.0    # translation: heading change by odometry (misses ~0.11 deg/run, E003)
    client_timeout: bool = False # client deadline expired, bCmdMoveAbort pulse sent
    wall_s: float = 0.0          # client time from the start edge to the end of the move

    @property
    def state_name(self) -> str:
        return MOVE_STATE_NAMES.get(self.state, f"state {self.state}")

    @property
    def result_text(self) -> str:
        return MOVE_RESULT_TEXTS.get(self.result, f"result {self.result}")

    @property
    def rejected(self) -> bool:
        return self.cmd_result in REJECT_CODES

    @property
    def in_tolerance(self) -> bool:
        return self.ok and self.result == RES_OK

    def odom_delta(self) -> tuple[float, float, float] | None:
        """Odometry displacement (dx_mm, dy_mm, dtheta_deg) in the start frame, or None."""
        if self.odom_before is None or self.odom_after is None:
            return None
        return self.odom_before.delta_to(self.odom_after)

    def summary(self) -> str:
        cmd = (f"dtheta={self.dtheta_deg:+.2f} deg" if self.kind == "rotate"
               else f"dx={self.dx_mm:+.1f} mm dy={self.dy_mm:+.1f} mm")
        verdict = "OK" if self.ok else ("REJECTED" if self.rejected else "FAILED")
        s = (f"move #{self.cmd_id} {self.kind} {cmd}: {verdict} - {self.state_name}, result {self.result} "
             f"({self.result_text})")
        if not self.rejected:
            err = (f"{self.final_err_deg:+.3f} deg" if self.kind == "rotate" else f"{self.final_err_mm:+.2f} mm")
            s += f", final error {err} (+ = short), {self.elapsed_s:.1f} s"
            if self.limited:
                s += ", limited by the PLC"
        return s + (f" - {self.text}" if self.text else "")


# ── pure helpers ──────────────────────────────────────────────────────────────
def next_move_id(last_sent: int, plc_ack: int) -> int:
    """Next nMoveCmdId: last_sent + 1, wraps 65535 -> 1, never 0, never the PLC's last processed id, never last_sent.
    Same rule as MA amr_hmi/logic.py:166-179 (the PLC starts only if the id differs from its last one, which is 0
    after a PLC start, MOVE_PRG.st:102)."""
    last, ack = int(last_sent) & 0xFFFF, int(plc_ack) & 0xFFFF
    cand = last
    for _ in range(3):
        cand = cand % 0xFFFF + 1          # 1..65535
        if cand != ack and cand != last:
            return cand
    raise AssertionError("unreachable")


def parse_build_version(build: str) -> tuple[int, int] | None:
    """(major, minor) from the PLC build text, e.g. 'ARES CX9240 v2.9 2026-10-01' -> (2, 9); 'v2' -> (2, 0)."""
    m = re.search(r"\bv(\d+)(?:\.(\d+))?\b", build or "")
    return (int(m.group(1)), int(m.group(2) or 0)) if m else None


def plc_timeout_s(target: float, speed: float, accel_mms2: float, rotation: bool,
                  hmi_accel_mms2: float = 0.0) -> float:
    """Total timeout the PLC will apply: 2 (target/v + v/a) + 10 s with the clamped speed and acceleration
    (MOVE_PRG.st:199-220, FB_RelMove.st:148-171). target/speed in mm, mm/s or deg, deg/s; accel at the wheel."""
    if rotation:
        v = min(max(speed, max(CREEP_ROT_DEGS, 0.5)), MAX_ROT_SPEED_DEGS)
    else:
        v = min(max(speed, max(CREEP_SPEED_MMS, 2.0)), MAX_SPEED_MMS)
    a = min(max(accel_mms2, MIN_ACCEL_MMS2), MAX_ACCEL_MMS2)
    if 0.0 < hmi_accel_mms2 < a:           # GVL_AMR.fAccel_mms (= stToPlc.fAccel_mms from the HMI) caps it
        a = hmi_accel_mms2
    a_move = a / ARM_MM * math.degrees(1.0) if rotation else a
    return TIMEOUT_FACTOR * (abs(target) / v + v / max(a_move, 1e-3)) + TIMEOUT_EXTRA_S


def check_move(dx_mm: float, dy_mm: float, dtheta_deg: float, speed_mms: float, rot_speed_degs: float,
               accel_mms2: float) -> str | None:
    """None if the PLC would accept the parameters, else the reason (mirror of MOVE_PRG.st:136-163 and MA amr_hmi
    logic.validate_move; speeds/accels above the PLC maximum are refused instead of silently clamped)."""
    for name, v in (("dx", dx_mm), ("dy", dy_mm), ("dtheta", dtheta_deg), ("speed", speed_mms),
                    ("rotation speed", rot_speed_degs), ("acceleration", accel_mms2)):
        try:
            if not math.isfinite(float(v)):
                return f"{name} is not a finite number"
        except (TypeError, ValueError):
            return f"{name} is not a number: {v!r}"
    dist = math.hypot(dx_mm, dy_mm)
    if dist > 0.0 and dtheta_deg != 0.0:
        return ("translation and rotation in one command: the PLC rejects that (REJ_PARAM, MOVE_PRG.st:139-141) "
                "and has no heading control during a translation - send two moves")
    if not 0.0 < accel_mms2 <= MAX_ACCEL_MMS2:
        return f"acceleration {accel_mms2:g} mm/s2 outside (0, {MAX_ACCEL_MMS2:g}] (GVL_Move.fMaxAccel_mms2)"
    if dtheta_deg != 0.0:
        a = abs(dtheta_deg)
        if a < MIN_MOVE_DEG:
            return (f"rotation {a:.3f} deg below the PLC minimum {MIN_MOVE_DEG:g} deg (GVL_Move.fMinMove_deg) - "
                    "do not fine-position ARES, let the camera absorb the error")
        if a > MAX_ANGLE_DEG:
            return f"rotation {a:g} deg above the PLC maximum {MAX_ANGLE_DEG:g} deg"
        if not 0.0 < rot_speed_degs <= MAX_ROT_SPEED_DEGS:
            return f"rotation speed {rot_speed_degs:g} deg/s outside (0, {MAX_ROT_SPEED_DEGS:g}]"
        return None
    if dist < MIN_MOVE_MM:
        return (f"distance {dist:.2f} mm below the PLC minimum {MIN_MOVE_MM:g} mm (GVL_Move.fMinMove_mm) - "
                "do not fine-position ARES, let the camera absorb the error")
    if dist > MAX_DIST_MM:
        return f"distance {dist:g} mm above the PLC maximum {MAX_DIST_MM:g} mm"
    if not 0.0 < speed_mms <= MAX_SPEED_MMS:
        return f"speed {speed_mms:g} mm/s outside (0, {MAX_SPEED_MMS:g}] (GVL_Move.fMaxSpeed_mms)"
    return None


def tcp_reachable(host: str, port: int, timeout_s: float) -> str | None:
    """None if a TCP connection to host:port opens within timeout_s, else the reason. Closed at once, no AMS frame
    is sent (MA ares_plc_bridge/transport.py:55-66)."""
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return None
    except OSError as exc:
        return str(exc) or exc.__class__.__name__


def target_host(ams_net_id: str, host_ip: str | None) -> str:
    """IP of the PLC: host_ip if set, else the first four octets of the AMS NetID (MA transport.py:69-71)."""
    return host_ip or ".".join(str(ams_net_id).split(".")[:4])


def _is_symbol_not_found(exc: BaseException) -> bool:
    return (getattr(exc, "err_code", None) == ADSERR_SYMBOL_NOT_FOUND
            or "symbol not found" in str(exc).lower())


def _convert(name: str, kind: str, value: Any) -> Any:
    """Type-check one value of a sum read. pyads (adsSumRead) puts the error TEXT in place of a value whose
    sub-read failed, so a str in a numeric field is an ADS error. A failed STRING field cannot be told apart from
    text - only display texts are STRING fields here."""
    ok = {"bool": isinstance(value, int),                 # bool is an int subclass
          "int": isinstance(value, int),
          "float": isinstance(value, (int, float)) and not isinstance(value, bool),
          "str": isinstance(value, str)}[kind]
    if not ok:
        raise AresConnectionError(f"ADS read of {name} failed: {value!r}")
    return {"bool": bool, "int": int, "float": float, "str": str}[kind](value)


# ── client ────────────────────────────────────────────────────────────────────
@dataclass
class Preflight:
    """Result of AresAds.check(): problems (empty = a move may start) and the two status reads."""
    problems: list[str]
    before: AresStatus | None
    status: AresStatus | None

    @property
    def ok(self) -> bool:
        return not self.problems

    @property
    def heartbeat_delta(self) -> int | None:
        if self.before is None or self.status is None:
            return None
        return (self.status.hmi_heartbeat - self.before.hmi_heartbeat) & 0xFFFF


class AresAds:
    """ADS client for the ARES relative move (pattern A: amr_hmi v2 open, owns heartbeat/MANUAL/HALT).

    cfg_ares_ads: the [ares_ads] table of config/station.toml (ams_net_id, host_ip, port, timeout_ms, speed_mms,
    accel_mms2, min_if_version; optional rot_speed_degs, min_plc_build, ack_timeout_s, deadline_margin_s).
    connection_factory: zero-argument callable returning an unopened pyads-like connection (default: pyads.Connection,
    imported lazily). tcp_probe: reachability check (host, port, timeout_s) -> reason | None run before opening;
    default = TCP connect to port 48898 when the pyads factory is used, none with a custom factory. clock / sleep:
    injectable for simulated time.
    """

    def __init__(self, cfg_ares_ads: Mapping[str, Any], connection_factory: Callable[[], Any] | None = None,
                 clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep,
                 tcp_probe: Callable[[str, int, float], str | None] | None = None) -> None:
        c = dict(cfg_ares_ads)
        self.ams_net_id = str(c["ams_net_id"])
        self.port = int(c["port"])
        self.host_ip = str(c.get("host_ip") or "") or None
        self.timeout_ms = int(c.get("timeout_ms", 1000))
        self.speed_mms = float(c["speed_mms"])
        self.accel_mms2 = float(c["accel_mms2"])
        self.rot_speed_degs = float(c.get("rot_speed_degs", DEFAULT_ROT_SPEED_DEGS))
        self.min_if_version = int(c.get("min_if_version", 2))
        self.ack_timeout_s = float(c.get("ack_timeout_s", ACK_TIMEOUT_S))
        self.deadline_margin_s = float(c.get("deadline_margin_s", DEADLINE_MARGIN_S))
        mb = parse_build_version("v" + str(c.get("min_plc_build", DEFAULT_MIN_PLC_BUILD)).lstrip("vV"))
        if mb is None:
            raise ValueError(f"[ares_ads] min_plc_build {c.get('min_plc_build')!r} is not like '2.1'")
        self.min_plc_build = mb
        self._factory = connection_factory or self._pyads_factory
        self._probe = tcp_probe if tcp_probe is not None else (tcp_reachable if connection_factory is None else None)
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()
        self._conn: Any = None
        self._build = ""
        self._names: list[str] = []
        self._last_id = 0
        self._inhibit: str | None = None          # HALT latch: reason; None = moves allowed

    # ── connection ────────────────────────────────────────────────────────────
    @property
    def host(self) -> str:
        return target_host(self.ams_net_id, self.host_ip)

    @property
    def connected(self) -> bool:
        return self._conn is not None

    @property
    def build(self) -> str:
        return self._build

    def _pyads_factory(self) -> Any:
        try:
            import pyads
        except Exception as exc:        # ImportError, or OSError if the ADS DLL / TwinCAT router is missing
            raise AresConnectionError(f"pyads not usable ({exc}): py.exe -m pip install pyads; on Windows the "
                                      "TwinCAT ADS router (TcAdsDll) is needed") from exc
        # Windows: TcAdsDll + the local TwinCAT routes, host_ip unused; Linux: pyads' own router (MA ads_worker:73-81)
        return pyads.Connection(self.ams_net_id, self.port, self.host_ip)

    def connect(self) -> None:
        """Open the ADS connection: TCP pre-check, open, set_timeout, PLC in RUN, interface v2 detection, one full
        status read (all symbols exist). Writes nothing. Raises AresConnectionError with an actionable message."""
        if self._conn is not None:
            return
        host = self.host
        if self._probe is not None:
            probe_s = min(TCP_PROBE_MAX_S, max(0.3, self.timeout_ms / 1000.0))
            reason = self._probe(host, ADS_TCP_PORT, probe_s)
            if reason is not None:
                raise AresConnectionError(
                    f"ARES PLC not reachable: TCP {host}:{ADS_TCP_PORT} ({reason}). Check: laptop on the ARES network "
                    f"(cable/Wi-Fi, address in {'.'.join(host.split('.')[:3])}.x), CX9240 powered and booted, "
                    "[ares_ads] host_ip / ams_net_id in config/station.toml.")
        conn = None
        try:
            conn = self._factory()
            conn.open()
            if hasattr(conn, "set_timeout"):
                conn.set_timeout(self.timeout_ms)
            if hasattr(conn, "read_state"):
                ads_state = conn.read_state()[0]
                if ads_state != ADS_STATE_RUN:
                    raise AresConnectionError(f"PLC runtime {self.ams_net_id}:{self.port} not in RUN (ADS state "
                                              f"{ads_state}) - start the PLC in TwinCAT XAE")
            self._build, build_ok = self._detect_interface(conn)
            self._names = ([n for _, n, _, _ in _STATUS_FIELDS] + list(_JOG_SYMBOLS)
                           + ([_BUILD_SYMBOL] if build_ok else []))
            self._conn = conn
            self.status()                     # every symbol of the status read exists (else build mismatch)
        except AresError:
            self._discard(conn)
            raise
        except Exception as exc:
            self._discard(conn)
            tcp = f"TCP {host}:{ADS_TCP_PORT} answers, so " if self._probe is not None else ""
            detail = str(exc).strip().rstrip(".")
            raise AresConnectionError(
                f"ADS connection to {self.ams_net_id}:{self.port} failed: {detail}. {tcp}check the TwinCAT route "
                "laptop <-> CX9240 (both directions, MA 10_robot/hmi/amr_hmi/SETUP.md) and that the PLC project is "
                "running.", getattr(exc, "err_code", None)) from exc

    def _detect_interface(self, conn: Any) -> tuple[str, bool]:
        """(build text, build readable). stFromPlc.nIfVersion must exist (interface v2, MA ads_worker:308-328)."""
        try:
            raw = conn.read_list_by_name([FROM + "nIfVersion", _BUILD_SYMBOL])
            build = raw.get(_BUILD_SYMBOL)
            if isinstance(build, str) and not isinstance(raw.get(FROM + "nIfVersion"), str):
                return build, True
        except Exception as exc:
            if not _is_symbol_not_found(exc):
                raise
        try:
            raw = conn.read_list_by_name([FROM + "nIfVersion"])
        except Exception as exc:
            if not _is_symbol_not_found(exc):
                raise
            raise AresConnectionError("PLC without the relative-move interface (no stFromPlc.nIfVersion, interface "
                                      "v1): load PLC build >= v2.1 (MA 10_robot/twincat/PLC_CX9240_v2)",
                                      ADSERR_SYMBOL_NOT_FOUND) from exc
        _convert(FROM + "nIfVersion", "int", raw.get(FROM + "nIfVersion"))
        return "", False

    def _discard(self, conn: Any) -> None:
        self._conn = None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def close(self) -> None:
        """Close the connection. Writes nothing (the HMI owns the safe defaults)."""
        with self._lock:
            self._discard(self._conn)

    def __enter__(self) -> "AresAds":
        self.connect()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    # ── low level ─────────────────────────────────────────────────────────────
    def _read(self, names: list[str]) -> dict[str, Any]:
        with self._lock:
            if self._conn is None:
                raise AresConnectionError("not connected to the ARES PLC (connect() first)")
            try:
                raw = self._conn.read_list_by_name(names)
            except Exception as exc:
                raise AresConnectionError(f"ADS read failed: {exc}", getattr(exc, "err_code", None)) from exc
        missing = [n for n in names if n not in (raw or {})]
        if missing:
            raise AresConnectionError(f"ADS read returned no value for {missing}")
        return raw

    def _write(self, values: Mapping[str, Any]) -> None:
        """ONE sum write of stToPlc fields. Only WRITABLE names (pattern A); any per-symbol error is a failure."""
        bad = sorted(set(values) - WRITABLE)
        if bad:
            raise ValueError(f"refusing to write {bad}: pattern A - amr_hmi owns mode, jog, heartbeat and stop bits")
        symbols = {TO + k: v for k, v in values.items()}
        with self._lock:
            if self._conn is None:
                raise AresConnectionError("not connected to the ARES PLC (connect() first)")
            try:
                res = self._conn.write_list_by_name(symbols)
            except Exception as exc:
                raise AresConnectionError(f"ADS write {sorted(values)} failed: {exc}",
                                          getattr(exc, "err_code", None)) from exc
        errors = {k: (res or {}).get(k, "no result") for k in symbols
                  if str((res or {}).get(k, "no result")).strip().lower() != "no error"}
        if errors:                            # MA amr_hmi/ads_worker.py:364-377
            raise AresConnectionError(f"PLC refused write: {errors}")

    # ── status / preflight ────────────────────────────────────────────────────
    def status(self) -> AresStatus:
        """One sum read of all status fields (stFromPlc + read-only stToPlc fields)."""
        raw = self._read(self._names)
        t = self._clock()
        vals: dict[str, Any] = {}
        for attr, name, kind, scale in _STATUS_FIELDS:
            v = _convert(name, kind, raw[name])
            vals[attr] = v * scale if kind == "float" else v
        vals["jog_active"] = any(_convert(n, "bool", raw[n]) for n in _JOG_SYMBOLS)
        vals["build"] = str(raw[_BUILD_SYMBOL]) if _BUILD_SYMBOL in raw else self._build
        return AresStatus(t_s=t, raw=dict(raw), **vals)

    def check(self) -> Preflight:
        """Preflight with details: two status reads HB_WINDOW_S apart, then the checks (see problems())."""
        if self._conn is None:
            return Preflight(["not connected to the ARES PLC (connect() first)"], None, None)
        try:
            s0 = self.status()
            self._sleep(HB_WINDOW_S)
            s = self.status()
        except AresConnectionError as exc:
            return Preflight([f"status read failed: {exc}"], None, None)
        return Preflight(self.problems(s0, s), s0, s)

    def preflight(self) -> list[str]:
        """Human-readable problems that would stop a move (empty = ok). Takes HB_WINDOW_S (heartbeat check)."""
        return self.check().problems

    def problems(self, s0: AresStatus, s: AresStatus) -> list[str]:
        """Checks on two status reads taken HB_WINDOW_S apart (MA imu_cal_run.py:179-184, amr_hmi logic.go_enabled
        and the PLC start conditions MOVE_PRG.st:136-195)."""
        p: list[str] = []
        if s.if_version < self.min_if_version:
            p.append(f"PLC interface version {s.if_version} < {self.min_if_version} (stFromPlc.nIfVersion): no "
                     "relative move - load PLC build >= v2.1")
        ver = parse_build_version(s.build)
        need = "v%d.%d" % self.min_plc_build
        if ver is None:
            p.append(f"PLC build {s.build!r} unreadable - need >= {need} (+Y = left only since v2.1, MA D21)")
        elif ver < self.min_plc_build:
            p.append(f"PLC build {s.build!r} older than {need}: before v2.1 the kinematics were mirrored and +Y "
                     "moved RIGHT (MA DECISIONS.md D21) - load the current PLC build")
        if s.ext_active:
            p.append("external control active (stFromPlc.bExtActive): the C6030 / ROS bridge overrides the HMI - stop "
                     "the C6030 TwinCAT runtime (it autostarts)")
        if s.amr_state != ST_MANUAL:
            p.append(f"AMR state {s.amr_state} {s.amr_state_name}, needs 7 MANUAL MODE: in amr_hmi run Safety Run -> "
                     "AMR Reset -> Start -> Manual")
        if not s.hmi_watchdog_ok:
            p.append("PLC does not see the HMI heartbeat (stFromPlc.bHmiWatchdogOK FALSE): open amr_hmi v2 and connect")
        if s.hmi_heartbeat == s0.hmi_heartbeat:
            p.append(f"HMI heartbeat stToPlc.nHeartbeat did not change within {HB_WINDOW_S * 1000:.0f} ms: the PLC "
                     "would reject (11) or abort (26) the move - is amr_hmi v2 open and connected?")
        if s.move_active:
            p.append(f"a relative move is running ({s.move_state_name}: {s.move_text})")
        if s.abort_bit:
            p.append("stToPlc.bCmdMoveAbort is TRUE (HALT within the last 300 ms, or left set by a crashed client): "
                     "the PLC would reject the move (13)")
        if s.stop_bit:
            p.append("stToPlc.bCmdStop is TRUE (Standby request): the PLC would reject the move (13)")
        if s.jog_active:
            p.append("a jog bit is set in stToPlc (jog key held in amr_hmi?): the PLC would reject the move (13)")
        if s.safety_stop_active:
            p.append("safety stop active (E-stop or laser scanner field): clear it and reset in amr_hmi")
        if s.fault_active:
            p.append(f"drive / system fault: {s.fault_text or 'see amr_hmi'}")
        if s.amr_moving:
            p.append("ARES is still moving (stFromPlc.bAmrMoving): the PLC needs standstill (13)")
        return p

    # ── commands ──────────────────────────────────────────────────────────────
    def abort(self) -> None:
        """Ramped stop of the running move: bCmdMoveAbort pulse of 300 ms (= amr_hmi HALT without the jog bits,
        MA SPEC:294-299). ARES stays in MANUAL, result 20. Not a safety function - the E-stop is."""
        self._write({"bCmdMoveAbort": True})
        try:
            self._sleep(PULSE_S)
        finally:
            self._write({"bCmdMoveAbort": False})

    def inhibit(self, reason: str = "HALT") -> None:
        """Latch a HALT: no start edge is written until release_inhibit() (move() raises AresInhibited). Sets a flag
        only (never blocks - callable from a GUI thread); the edge write checks it under the connection lock."""
        self._inhibit = str(reason) or "HALT"

    def release_inhibit(self) -> None:
        """Lift the HALT latch (the operator restarts / resumes explicitly)."""
        self._inhibit = None

    @property
    def inhibited(self) -> str | None:
        """Reason of the HALT latch, None when moves are allowed."""
        return self._inhibit

    def _refuse_if_inhibited(self) -> None:
        reason = self._inhibit
        if reason is not None:
            raise AresInhibited([f"{reason}: ARES moves inhibited until the run is started / resumed again - no "
                                 "move command written"])

    def _abort_quietly(self) -> str:
        try:
            self.abort()
            return "abort sent"
        except Exception as exc:              # the original error matters more
            return f"ABORT FAILED ({exc}) - press HALT in amr_hmi"

    def translate(self, dx_mm: float, dy_mm: float, speed_mms: float | None = None,
                  accel_mms2: float | None = None, timeout_s: float | None = None,
                  progress: Callable[[AresStatus], None] | None = None) -> MoveOutcome:
        """Relative translation (dx forward, dy left) in the body frame at the start pose. Blocks until DONE/ABORTED.
        Defaults from [ares_ads] speed_mms / accel_mms2; timeout_s = client deadline from the start edge (default:
        PLC timeout + tAbortMax + margin). Raises MoveRefused, AresNotReady (nothing written) or AresConnectionError."""
        return self.move(dx_mm=dx_mm, dy_mm=dy_mm, speed_mms=speed_mms, accel_mms2=accel_mms2,
                         timeout_s=timeout_s, progress=progress)

    def rotate(self, dtheta_deg: float, rot_speed_degs: float | None = None, accel_mms2: float | None = None,
               timeout_s: float | None = None, progress: Callable[[AresStatus], None] | None = None) -> MoveOutcome:
        """Rotation on the spot (+ = CCW). Rotations show 0.14 deg scatter and ~1.2 deg loss after reversing the
        direction (E003) - use sparingly. Otherwise as translate()."""
        return self.move(dtheta_deg=dtheta_deg, rot_speed_degs=rot_speed_degs, accel_mms2=accel_mms2,
                         timeout_s=timeout_s, progress=progress)

    def move(self, dx_mm: float = 0.0, dy_mm: float = 0.0, dtheta_deg: float = 0.0,
             speed_mms: float | None = None, rot_speed_degs: float | None = None, accel_mms2: float | None = None,
             timeout_s: float | None = None, progress: Callable[[AresStatus], None] | None = None) -> MoveOutcome:
        """One relative move: a translation OR a rotation (a combination is refused). See translate()."""
        speed = self.speed_mms if speed_mms is None else speed_mms
        rot_speed = self.rot_speed_degs if rot_speed_degs is None else rot_speed_degs
        accel = self.accel_mms2 if accel_mms2 is None else accel_mms2
        reason = check_move(dx_mm, dy_mm, dtheta_deg, speed, rot_speed, accel)
        if reason is not None:
            raise MoveRefused(f"move refused: {reason}")
        self._refuse_if_inhibited()
        pf = self.check()
        if not pf.ok:
            raise AresNotReady(pf.problems)
        assert pf.status is not None
        return self._run(float(dx_mm), float(dy_mm), float(dtheta_deg), float(speed), float(rot_speed),
                         float(accel), pf.status, timeout_s, progress)

    def _run(self, dx: float, dy: float, dth: float, speed: float, rot_speed: float, accel: float,
             s0: AresStatus, timeout_s: float | None, progress: Callable[[AresStatus], None] | None) -> MoveOutcome:
        rot = dth != 0.0
        mid = next_move_id(self._last_id, s0.move_cmd_ack)
        self._last_id = mid                   # never reuse an id that may have reached the PLC
        out = dict(cmd_id=mid, kind="rotate" if rot else "translate", dx_mm=dx, dy_mm=dy, dtheta_deg=dth,
                   odom_before=s0.odom)

        # (1) parameters + id with the start bit FALSE, (2) the rising edge in a separate write
        self._write({"fMoveX_mm": dx, "fMoveY_mm": dy, "fMoveTheta_deg": dth, "fMoveSpeed_mms": speed,
                     "fMoveRotSpeed_degs": rot_speed, "fMoveAccel_mms2": accel, "nMoveCmdId": mid,
                     "bCmdMoveStart": False})
        self._sleep(EDGE_DELAY_S)
        t_edge = self._clock()

        # (3) edge, acknowledgement; start bit reset after the ack or PULSE_S. From the edge write on, any failure
        # is treated as "the move may have started": abort pulse, start bit reset.
        start_reset = False
        try:
            with self._lock:                  # HALT latch checked together with the edge write (inhibit())
                self._refuse_if_inhibited()
                self._write({"bCmdMoveStart": True})
            while True:
                s = self.status()
                acked = s.move_cmd_ack == mid
                if not start_reset and (acked or self._clock() - t_edge >= PULSE_S):
                    self._write({"bCmdMoveStart": False})
                    start_reset = True
                if acked:
                    break
                if self._clock() - t_edge > self.ack_timeout_s:
                    note = self._abort_quietly()
                    raise _NoAck(f"no acknowledgement for move #{mid} within {self.ack_timeout_s:g} s "
                                 f"(nMoveCmdAck = {s.move_cmd_ack}) - {note}")
                self._sleep(ACK_POLL_S)
            s2 = self.status()                # confirmation: ack and verdict from a read after the ack was visible
            if s2.move_cmd_ack == mid:
                s = s2
        except (_NoAck, AresInhibited):       # no ack: abort already sent; inhibited: no edge was written
            raise
        except AresConnectionError as exc:    # the command may have started
            raise AresConnectionError(f"{exc} - move #{mid} may have started, {self._abort_quietly()}",
                                      exc.err_code) from exc
        except BaseException:                 # Ctrl+C while the command may be running
            self._abort_quietly()
            raise
        finally:
            if not start_reset:
                try:
                    self._write({"bCmdMoveStart": False})
                except Exception:
                    pass

        verdict = s.move_cmd_result
        if verdict != RES_NONE:
            text = s.move_text
            if verdict not in REJECT_CODES:   # neither accepted nor a known rejection: never treat as accepted
                text = f"invalid verdict eMoveCmdResult={verdict} - {self._abort_quietly()}; {text}"
            elif s.move_active:
                text += f" (a move is active: {s.move_state_name})"
            return MoveOutcome(ok=False, cmd_result=verdict, state=s.move_state, result=verdict,
                               final_err_mm=math.nan, text=text, elapsed_s=0.0, limited=False,
                               odom_after=s.odom, wall_s=self._clock() - t_edge, **out)

        # (4) wait for the end of the move
        seen = s.move_active or s.move_state in ACTIVE_STATES
        target = abs(dth) if rot else math.hypot(dx, dy)
        limit = (timeout_s if timeout_s is not None else
                 plc_timeout_s(target, rot_speed if rot else speed, accel, rot, s.hmi_accel_mms2)
                 + T_ABORT_MAX_S + self.deadline_margin_s)
        deadline = t_edge + limit
        notes: list[str] = []
        client_timeout = False
        try:
            while not (seen and not s.move_active and s.move_state in END_STATES):
                if self._clock() > deadline:
                    client_timeout = True
                    notes.append(f"client deadline {limit:.1f} s exceeded"
                                 + ("" if seen else " (move never seen running)") + " - " + self._abort_quietly())
                    s = self._wait_end(ABORT_WAIT_S, progress)
                    if s.move_active:
                        notes.append("move still active after the abort - press HALT / E-stop")
                    break
                self._sleep(POLL_S)
                s = self.status()
                seen = seen or s.move_active or s.move_state in ACTIVE_STATES
                if progress is not None:
                    progress(s)
        except AresConnectionError as exc:
            raise AresConnectionError(f"{exc} - move #{mid} may still be running, {self._abort_quietly()}",
                                      exc.err_code) from exc
        except BaseException:                 # Ctrl+C: stop ARES, then let the caller see the interrupt
            self._abort_quietly()
            raise

        if s.move_cmd_ack != mid and s.move_cmd_result == RES_NONE:
            notes.append(f"another move command #{s.move_cmd_ack} was accepted meanwhile - result ambiguous")
        ok = (not client_timeout and seen and s.move_state == MOVE_DONE and s.move_result in OK_CODES
              and not (s.move_cmd_ack != mid and s.move_cmd_result == RES_NONE))
        err = s.move_final_err
        return MoveOutcome(
            ok=ok, cmd_result=verdict, state=s.move_state, result=s.move_result,
            final_err_mm=math.nan if rot else err, final_err_deg=err if rot else math.nan,
            text="; ".join([s.move_text] + notes), elapsed_s=s.move_elapsed_s, limited=s.move_limited,
            odom_after=s.odom, lat_err_mm=s.move_lat_err_mm, head_err_deg=s.move_head_err_deg,
            client_timeout=client_timeout, wall_s=self._clock() - t_edge, **out)

    def _wait_end(self, wait_s: float, progress: Callable[[AresStatus], None] | None) -> AresStatus:
        """After a client abort: poll until no move is active (ABORTED/DONE) or wait_s has passed."""
        t_end = self._clock() + wait_s
        while True:
            s = self.status()
            if progress is not None:
                progress(s)
            if (not s.move_active and s.move_state not in ACTIVE_STATES) or self._clock() > t_end:
                return s
            self._sleep(POLL_S)


# sanity: every status attribute is a field of AresStatus (catches table/dataclass drift at import)
assert {a for a, _, _, _ in _STATUS_FIELDS} <= {f.name for f in fields(AresStatus)}
