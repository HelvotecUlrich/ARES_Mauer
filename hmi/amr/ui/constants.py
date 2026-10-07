"""
constants.py - Single source of truth for PLC enums and display colours.

Values mirror the TwinCAT DUTs:
  E_AMR_State   10_robot/twincat/PLC_CX9240_snapshot_2026-08/DUTs/E_AMR_State.TcDUT (0..16, 16 = PRECHARGE)
  E_LampColor   .../DUTs/E_LampColor.TcDUT
  E_MoveState   10_robot/twincat/PLC_CX9240_v2/src/E_MoveState.st  (spec 3.2)
  E_MoveResult  10_robot/twincat/PLC_CX9240_v2/src/E_MoveResult.st (spec 3.3; 29 = ABORT_ODOM, spec section 9)
Spec: 10_robot/hmi/SPEC_2026-09-25_Relativfahrt_HMI_SPS_v2.md
"""

from __future__ import annotations

from typing import Dict, Tuple

# ── E_AMR_State ────────────────────────────────────────────────────────────────
ST_INIT = 0
ST_WAIT_SAFETY = 1
ST_SAFETY_STOP = 2
ST_RESET_REQUIRED = 3
ST_STANDBY = 4
ST_DRIVES_ENABLE = 5
ST_READY = 6
ST_MANUAL = 7
ST_AUTO = 8
ST_NAVIGATING = 9
ST_OBSTACLE_STOP = 10
ST_DOCKING = 11
ST_CHARGING = 12
ST_ERROR = 13
ST_ERROR_ACK = 14
ST_SHUTDOWN = 15
ST_PRECHARGE = 16   # pre-charge before STANDBY (GVL_TWINSAFE.toPlc_Precharge); not a drive/fault reset state

# state -> (display colour, display name); names equal the PLC's sAmrStateText
AMR_STATES: Dict[int, Tuple[str, str]] = {
    ST_INIT:           ("#888888", "INIT"),
    ST_WAIT_SAFETY:    ("#FFA500", "WAIT SAFETY"),
    ST_SAFETY_STOP:    ("#FF4444", "SAFETY STOP"),
    ST_RESET_REQUIRED: ("#CC44CC", "RESET REQUIRED"),
    ST_STANDBY:        ("#4488FF", "STANDBY"),
    ST_DRIVES_ENABLE:  ("#00CCCC", "DRIVES ENABLE"),
    ST_READY:          ("#44CC44", "READY"),
    ST_MANUAL:         ("#FFFFFF", "MANUAL MODE"),
    ST_AUTO:           ("#44CC44", "AUTO MODE"),
    ST_NAVIGATING:     ("#44CC44", "NAVIGATING"),
    ST_OBSTACLE_STOP:  ("#FFA500", "OBSTACLE STOP"),
    ST_DOCKING:        ("#00CCCC", "DOCKING"),
    ST_CHARGING:       ("#4488FF", "CHARGING"),
    ST_ERROR:          ("#FF4444", "ERROR"),
    ST_ERROR_ACK:      ("#FF8888", "ERROR ACK"),
    ST_SHUTDOWN:       ("#777777", "SHUTDOWN"),
    ST_PRECHARGE:      ("#66AACC", "PRECHARGE"),
}
UNKNOWN_STATE: Tuple[str, str] = ("#888888", "UNKNOWN")


def state_info(state: int) -> Tuple[str, str]:
    """Return (colour, name) for an E_AMR_State value; UNKNOWN for anything else."""
    try:
        return AMR_STATES.get(int(state), UNKNOWN_STATE)
    except (TypeError, ValueError):
        return UNKNOWN_STATE


# ── E_LampColor ────────────────────────────────────────────────────────────────
LAMP_COLORS: Dict[int, str] = {
    0: "#222222",  # Off
    1: "#FF3333",  # Red
    2: "#33FF33",  # Green
    3: "#3366FF",  # Blue
    4: "#FFAA00",  # Yellow (R+G)
    5: "#00FFFF",  # Cyan (G+B)
    6: "#FF33FF",  # Magenta (R+B)
    7: "#FFFFFF",  # White
}
LAMP_OFF = "#222222"

# ── E_MoveState ────────────────────────────────────────────────────────────────
MOVE_IDLE = 0
MOVE_ALIGN = 1
MOVE_RUN = 2
MOVE_SETTLE = 3
MOVE_DONE = 4
MOVE_ABORTING = 5
MOVE_ABORTED = 6

MOVE_STATES: Dict[int, str] = {
    MOVE_IDLE:     "IDLE",
    MOVE_ALIGN:    "ALIGN (steering to direction)",
    MOVE_RUN:      "RUN",
    MOVE_SETTLE:   "SETTLE (waiting for standstill)",
    MOVE_DONE:     "DONE",
    MOVE_ABORTING: "ABORTING (ramped stop)",
    MOVE_ABORTED:  "ABORTED",
}

# ── E_MoveResult ───────────────────────────────────────────────────────────────
RES_NONE = 0
RES_OK = 1
RES_OK_TOL = 2
RES_REJ_PARAM = 10
RES_REJ_STATE = 11
RES_REJ_EXT = 12
RES_REJ_BUSY = 13
RES_ABORT_USER = 20
RES_ABORT_JOG = 21
RES_ABORT_MODE = 22
RES_ABORT_SAFETY = 23
RES_ABORT_FAULT = 24
RES_ABORT_EXT = 25
RES_ABORT_HMI = 26
RES_ABORT_TIMEOUT = 27
RES_ABORT_STALL = 28
RES_ABORT_ODOM = 29

MOVE_RESULTS: Dict[int, str] = {
    RES_NONE:          "-",
    RES_OK:            "Done - final error within tolerance",
    RES_OK_TOL:        "Done - final error OUTSIDE tolerance",
    RES_REJ_PARAM:     "Rejected: parameters (invalid, too small/large, odometry parameters)",
    RES_REJ_STATE:     "Rejected: AMR not ready / not MANUAL / heartbeat",
    RES_REJ_EXT:       "Rejected: external control (C6030) active",
    RES_REJ_BUSY:      "Rejected: busy (move/SysId/jog/abort active or robot still moving)",
    RES_ABORT_USER:    "Aborted: HALT / abort / standby",
    RES_ABORT_JOG:     "Aborted: jog command",
    RES_ABORT_MODE:    "Aborted: MANUAL left / test mode / drives not ready",
    RES_ABORT_SAFETY:  "Aborted: safety stop",
    RES_ABORT_FAULT:   "Aborted: system or drive fault",
    RES_ABORT_EXT:     "Aborted: external control became active",
    RES_ABORT_HMI:     "Aborted: HMI heartbeat lost",
    RES_ABORT_TIMEOUT: "Aborted: timeout (alignment or total time)",
    RES_ABORT_STALL:   "Aborted: drives do not follow (stall)",
    RES_ABORT_ODOM:    "Aborted: measured progress implausible (odometry)",
}

REJECT_RESULTS = frozenset({RES_REJ_PARAM, RES_REJ_STATE, RES_REJ_EXT, RES_REJ_BUSY})
OK_RESULTS = frozenset({RES_OK, RES_OK_TOL})


def move_state_text(state: int) -> str:
    return MOVE_STATES.get(int(state), f"state {state}")


def move_result_text(result: int) -> str:
    return MOVE_RESULTS.get(int(result), f"result {result}")


def is_reject(result: int) -> bool:
    return int(result) in REJECT_RESULTS


def is_abort(result: int) -> bool:
    return 20 <= int(result) <= 29


# ── Common colours ─────────────────────────────────────────────────────────────
COL_OK = "#44CC44"
COL_WARN = "#FFAA00"
COL_BAD = "#FF4444"
COL_IDLE = "#444444"
COL_TEXT = "#CCCCCC"
COL_DIM = "#888888"

# Robot outline for the odometry map (CAD contour, PROJECT_STATUS item 27)
ROBOT_LENGTH_M = 1.12
ROBOT_WIDTH_M = 0.60
