"""
plc_vars.py - Symbol lists of the HMI <-> PLC interface (GVL_HMI.stToPlc / stFromPlc).

Interface version 1 = PLC state of 05/2026 (ST_HMI_* without relative move).
Interface version 2 = PLC build v2 (10_robot/twincat/PLC_CX9240_v2): v1 fields plus the fields appended
by spec sections 5.1 / 5.2. Access is symbolic (read_list_by_name / write_list_by_name), so field order
does not matter. Type strings are informative (pyads resolves the real type from the symbol info).
"""

from __future__ import annotations

from typing import Any, Dict

IF_LEGACY = 1
IF_V2 = 2

# Start of the connection message when a v2 PLC lacks symbols of the v2 lists (ads_worker.InterfaceMismatch);
# the Control tab shows such a message in full in a red banner.
MISMATCH_TEXT = "PLC build does not match this HMI"

# ── PLC -> HMI (ST_HMI_FromPlc) ───────────────────────────────────────────────
FROM_PLC_VARS_V1: Dict[str, str] = {
    # Watchdog
    "nPlcHeartbeat":        "UINT",
    "nCommandAck":          "UINT",
    "bHmiWatchdogOK":       "BOOL",
    "bPlcRunning":          "BOOL",
    # State
    "eAmrState":            "INT",
    "sAmrStateText":        "STRING",
    # Mode
    "bAmrReady":            "BOOL",
    "bModeManual":          "BOOL",
    "bModeAuto":            "BOOL",
    "bAmrMoving":           "BOOL",
    "bPreciseModeActive":   "BOOL",
    "bPreciseModeHolding":  "BOOL",
    # Safety
    "bSafetyOK":            "BOOL",
    "bEmergencyStopActive": "BOOL",
    "bSafetyRunActive":     "BOOL",
    # Drives
    "bDrivesEnabled":       "BOOL",
    "bDrivesFault":         "BOOL",
    "bTractionReady":       "BOOL",
    "bTractionFault":       "BOOL",
    "bSteeringReady":       "BOOL",
    "bSteeringFault":       "BOOL",
    "nTractionErrorL":      "UINT",
    "nTractionErrorR":      "UINT",
    "nSteeringErrorL":      "UINT",
    "nSteeringErrorR":      "UINT",
    # Velocity
    "fActVx_mms":           "LREAL",
    "fActVy_mms":           "LREAL",
    "fActOmega_degs":       "LREAL",
    "fActSpeed_Left_mms":   "LREAL",
    "fActSpeed_Right_mms":  "LREAL",
    "fActAngle_Left_deg":   "LREAL",
    "fActAngle_Right_deg":  "LREAL",
    "fSetAngle_Left_deg":   "LREAL",
    "fSetAngle_Right_deg":  "LREAL",
    "fSetVx_mms":           "LREAL",
    "fSetVy_mms":           "LREAL",
    "fSetOmega_degs":       "LREAL",
    # Lamp
    "eActiveLampColor":     "INT",
    "bLampBlink":           "BOOL",
    "bShowModeActive":      "BOOL",
    # Kinematics
    "bWheelL_Flipped":      "BOOL",
    "bWheelR_Flipped":      "BOOL",
    "bWheelL_AtSoftLimit":  "BOOL",
    "bWheelR_AtSoftLimit":  "BOOL",
    # Faults
    "bFaultActive":         "BOOL",
    "nFaultCode":           "UINT",
    "sFaultText":           "STRING",
    "bWarningActive":       "BOOL",
    "nWarningCode":         "UINT",
    "sWarningText":         "STRING",
    # Battery
    "fBattSOC_pct":         "LREAL",
    "fBattSOH_pct":         "LREAL",
    "fBattVoltage_V":       "LREAL",
    "fBattCurrent_A":       "LREAL",
    "fBattTempMax_degC":    "LREAL",
    "fBattTempMin_degC":    "LREAL",
    "nBattCellVmax_mV":     "UINT",
    "nBattCellVmin_mV":     "UINT",
    "fBattCapacity_Ah":     "LREAL",
    "fBattFullCap_Ah":      "LREAL",
    "nBattCycleCnt":        "UINT",
    "nBattProtLvl1":        "UINT",
    "nBattProtLvl2":        "UINT",
    "bBattCharging":        "BOOL",
    "bBattConnected":       "BOOL",
    "bBattError":           "BOOL",
    # Navigation (v1: constant 0; v2: odometry pose)
    "fPosX_m":              "LREAL",
    "fPosY_m":              "LREAL",
    "fPosTheta_deg":        "LREAL",
    "bLocalizationOK":      "BOOL",
    "bObstacleDetected":    "BOOL",
    # Mission (not implemented in the PLC)
    "nActiveMissionId":     "UINT",
    "sMissionStatus":       "STRING",
}

# spec 5.2 + section 9 (rev. 2) - appended at the end of ST_HMI_FromPlc in PLC build v2. All fields belong to
# the same build: a v2 PLC without one of them is reported as a build mismatch (ads_worker, no fallback).
FROM_PLC_VARS_V2_EXTRA: Dict[str, str] = {
    "nIfVersion":        "UINT",
    "sPlcBuild":         "STRING",
    "bExtActive":        "BOOL",
    "eMoveState":        "INT",
    "eMoveResult":       "INT",     # result of the current/last move (shows a rejection while no move runs)
    "eMoveCmdResult":    "INT",     # section 9: verdict on the command nMoveCmdAck: 0 accepted, 10..13 rejected
    "nMoveCmdAck":       "UINT",
    "bMoveLimited":      "BOOL",    # section 9: speed/accel of the current/last move limited by the PLC
    "sMoveText":         "STRING",
    "bMoveActive":       "BOOL",
    "bMoveIsRotation":   "BOOL",
    "fMoveTarget":       "LREAL",
    "fMoveDone":         "LREAL",
    "fMoveRemaining":    "LREAL",
    "fMoveLatErr_mm":    "LREAL",
    "fMoveHeadErr_deg":  "LREAL",
    "fMoveProgress_pct": "LREAL",
    "fMoveFinalErr":     "LREAL",
    "fMoveElapsed_s":    "LREAL",
    "fOdomDist_m":       "LREAL",
    "nOdomGlitchCnt":    "UINT",
    "bFkLegacy":         "BOOL",
}

# ── HMI -> PLC (ST_HMI_ToPlc) ─────────────────────────────────────────────────
TO_PLC_VARS_V1: Dict[str, str] = {
    "nHeartbeat":         "UINT",
    "nCommandId":         "UINT",
    "bCmdStart":          "BOOL",
    "bCmdStop":           "BOOL",
    "bCmdReset":          "BOOL",
    "bCmdManualMode":     "BOOL",
    "bCmdAutoMode":       "BOOL",
    "bCmdSafetyRun":      "BOOL",
    "bCmdSafetyReset":    "BOOL",
    "bCmdJogFwd":         "BOOL",
    "bCmdJogBwd":         "BOOL",
    "bCmdJogLeft":        "BOOL",
    "bCmdJogRight":       "BOOL",
    "bCmdJogRotLeft":     "BOOL",
    "bCmdJogRotRight":    "BOOL",
    "fJogSpeed_mms":      "LREAL",
    "fJogRotSpeed_degs":  "LREAL",
    "fSpeedLimit_mms":    "LREAL",
    "bCmdHorn":           "BOOL",
    "bCmdMissionStart":   "BOOL",
    "bCmdMissionPause":   "BOOL",
    "bCmdMissionResume":  "BOOL",
    "bCmdMissionCancel":  "BOOL",
    "nMissionId":         "UINT",
    "fAccel_mms":         "LREAL",
    "bCmdPreciseMode":    "BOOL",
    "bCmdShowMode":       "BOOL",
}

# spec 5.1 - appended at the end of ST_HMI_ToPlc in PLC build v2
TO_PLC_VARS_V2_EXTRA: Dict[str, str] = {
    "bCmdMoveStart":      "BOOL",
    "bCmdMoveAbort":      "BOOL",
    "nMoveCmdId":         "UINT",
    "fMoveX_mm":          "LREAL",
    "fMoveY_mm":          "LREAL",
    "fMoveTheta_deg":     "LREAL",
    "fMoveSpeed_mms":     "LREAL",
    "fMoveRotSpeed_degs": "LREAL",
    "fMoveAccel_mms2":    "LREAL",
    "bCmdOdomReset":      "BOOL",
}

JOG_FIELDS = (
    "bCmdJogFwd", "bCmdJogBwd", "bCmdJogLeft", "bCmdJogRight",
    "bCmdJogRotLeft", "bCmdJogRotRight",
)

# Written FALSE after every (re)connect before the first heartbeat (spec 6.1). bCmdSafetyRun, bCmdPreciseMode
# and bCmdShowMode are deliberately not touched (dropping Safety Run would force SAFETY_STOP).
_SAFE_FALSE_V1 = JOG_FIELDS + (
    "bCmdHorn", "bCmdManualMode", "bCmdAutoMode",
    "bCmdStart", "bCmdStop", "bCmdReset", "bCmdSafetyReset",
)
_SAFE_FALSE_V2 = ("bCmdMoveStart", "bCmdMoveAbort", "bCmdOdomReset")

TYPE_DEFAULTS: Dict[str, Any] = {
    "BOOL": False, "INT": 0, "UINT": 0, "DINT": 0, "UDINT": 0,
    "LREAL": 0.0, "REAL": 0.0, "USINT": 0, "STRING": "",
}


def from_plc_vars(version: int) -> Dict[str, str]:
    """FROM symbol list (field -> type) for an interface version."""
    if version >= IF_V2:
        return {**FROM_PLC_VARS_V1, **FROM_PLC_VARS_V2_EXTRA}
    return dict(FROM_PLC_VARS_V1)


def to_plc_vars(version: int) -> Dict[str, str]:
    """TO symbol list (field -> type) for an interface version."""
    if version >= IF_V2:
        return {**TO_PLC_VARS_V1, **TO_PLC_VARS_V2_EXTRA}
    return dict(TO_PLC_VARS_V1)


def safe_default_values(version: int) -> Dict[str, bool]:
    """Motion/command bits that must be FALSE after a (re)connect."""
    fields = _SAFE_FALSE_V1 + (_SAFE_FALSE_V2 if version >= IF_V2 else ())
    return {f: False for f in fields}


def default_status(version: int = IF_V2) -> Dict[str, Any]:
    """Status dict with type defaults for every FROM field (used when disconnected / key missing)."""
    return {k: TYPE_DEFAULTS[t] for k, t in from_plc_vars(version).items()}
