"""Fake ARES PLC for tests: pyads-like connections on a small state machine of MOVE_PRG / FB_RelMove (PLC v2.9).

Faithful where the ADS client depends on it - "MA" = Masterarbeit repo, 10_robot/twincat/PLC_CX9240_v2/:
- start: rising edge of bCmdMoveStart AND nMoveCmdId <> last processed id; ack (nMoveCmdAck) and verdict
  (eMoveCmdResult) in the same cycle; rejection reasons in the order of src/MOVE_PRG.st:136-195 (10/12/11/13);
  speed/accel clamping with bMoveLimited (MOVE_PRG.st:199-220);
- abort causes while a move runs, first match wins (MOVE_PRG.st:254-273): safety 23 > fault 24 > ext 25 > odometry 29
  > HMI heartbeat 26 > not ready/MANUAL 22 > bCmdMoveAbort/bCmdStop 20 > jog 21;
- FB_RelMove states ALIGN/RUN/SETTLE/DONE and ABORTING/ABORTED with the PLC timers (GVL_Move.st: tAlignStable
  100 ms, tAlignTimeout 4 s, tSettle 300 ms, tSettleTimeout 3 s, tAbortHold 1 s, tAbortMax 10 s, tStall 2 s), the
  speed profile of FB_RelMove.st:320-328, total timeout, stall/plausibility checks; status published as in
  MOVE_PRG.st:326-352 (eResult/sText only on a state change, a rejection is not overwritten in its cycle);
- heartbeat watchdogs: MOVE_PRG 500 ms (needs one change since PLC start), FB_HMI_Interface 2 s (bHmiWatchdogOK;
  on expiry the MANUAL request is cleared and the AMR drops to READY), overlay FB_HMI_Interface.TcPOU:60-66,153-163.
Simplified plant (ASSUMPTIONS, not measured): no kinematics, the steering is aligned `align_s` after the start
(precise mode holds traction until then), wheels follow the command with the move acceleration, no slip, jog bits
are only evaluated as reject/abort reasons (no jog motion). The final error can be forced (final_err_mm/deg).
For closed-loop checks against the line-by-line ST port see the MA reference-model test in tests/test_ares_ads.py.

Time: simulated by default - plc.clock() / plc.sleep(s) advance the PLC in 1 ms cycles (pass both to AresAds); each
ADS request costs `ads_latency_s`. realtime=True follows time.monotonic() instead (cycles are caught up at every
access), for tests with a real heartbeat thread (HmiHeartbeat). Heartbeat sources: the emulated HMI inside the cycle
(hmi_alive, every 100 ms like amr_hmi), HmiHeartbeat (own connection, like the HMI's ADS worker) or tick_heartbeat().
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable

TO = "GVL_HMI.stToPlc."
FROM = "GVL_HMI.stFromPlc."
BUILD_V29 = "ARES CX9240 v2.9 2026-10-01"     # MA src/GVL_Build.st:8

# ST_HMI_ToPlc (MA overlay/DUTs/ST_HMI_ToPlc.TcDUT): name -> type; v2 fields marked
TO_TYPES_V1 = {
    "nHeartbeat": "UINT", "nCommandId": "UINT", "bCmdStart": "BOOL", "bCmdStop": "BOOL", "bCmdReset": "BOOL",
    "bCmdManualMode": "BOOL", "bCmdAutoMode": "BOOL", "bCmdSafetyRun": "BOOL", "bCmdSafetyReset": "BOOL",
    "bCmdJogFwd": "BOOL", "bCmdJogBwd": "BOOL", "bCmdJogLeft": "BOOL", "bCmdJogRight": "BOOL",
    "bCmdJogRotLeft": "BOOL", "bCmdJogRotRight": "BOOL", "fJogSpeed_mms": "LREAL", "fJogRotSpeed_degs": "LREAL",
    "fSpeedLimit_mms": "LREAL", "bCmdHorn": "BOOL", "bCmdShowMode": "BOOL", "fAccel_mms": "LREAL",
    "bCmdPreciseMode": "BOOL", "bCmdMissionStart": "BOOL", "bCmdMissionPause": "BOOL", "bCmdMissionResume": "BOOL",
    "bCmdMissionCancel": "BOOL", "nMissionId": "UINT",
}
TO_TYPES_V2 = {
    "bCmdMoveStart": "BOOL", "bCmdMoveAbort": "BOOL", "nMoveCmdId": "UINT", "fMoveX_mm": "LREAL",
    "fMoveY_mm": "LREAL", "fMoveTheta_deg": "LREAL", "fMoveSpeed_mms": "LREAL", "fMoveRotSpeed_degs": "LREAL",
    "fMoveAccel_mms2": "LREAL", "bCmdOdomReset": "BOOL",
}
# ST_HMI_FromPlc (MA overlay/DUTs/ST_HMI_FromPlc.TcDUT)
FROM_TYPES_V1 = {
    "nPlcHeartbeat": "UINT", "nCommandAck": "UINT", "bHmiWatchdogOK": "BOOL", "bPlcRunning": "BOOL",
    "eAmrState": "INT", "sAmrStateText": "STRING", "bAmrReady": "BOOL", "bModeManual": "BOOL", "bModeAuto": "BOOL",
    "bAmrMoving": "BOOL", "bPreciseModeActive": "BOOL", "bPreciseModeHolding": "BOOL", "bSafetyOK": "BOOL",
    "bEmergencyStopActive": "BOOL", "bSafetyRunActive": "BOOL", "bDrivesEnabled": "BOOL", "bDrivesFault": "BOOL",
    "bTractionReady": "BOOL", "bTractionFault": "BOOL", "bSteeringReady": "BOOL", "bSteeringFault": "BOOL",
    "nTractionErrorL": "UINT", "nTractionErrorR": "UINT", "nSteeringErrorL": "UINT", "nSteeringErrorR": "UINT",
    "fActVx_mms": "LREAL", "fActVy_mms": "LREAL", "fActOmega_degs": "LREAL", "fActSpeed_Left_mms": "LREAL",
    "fActSpeed_Right_mms": "LREAL", "fActAngle_Left_deg": "LREAL", "fActAngle_Right_deg": "LREAL",
    "fSetAngle_Left_deg": "LREAL", "fSetAngle_Right_deg": "LREAL", "fSetVx_mms": "LREAL", "fSetVy_mms": "LREAL",
    "fSetOmega_degs": "LREAL", "eActiveLampColor": "INT", "bLampBlink": "BOOL", "bShowModeActive": "BOOL",
    "bWheelL_Flipped": "BOOL", "bWheelR_Flipped": "BOOL", "bWheelL_AtSoftLimit": "BOOL",
    "bWheelR_AtSoftLimit": "BOOL", "bFaultActive": "BOOL", "nFaultCode": "UINT", "sFaultText": "STRING",
    "bWarningActive": "BOOL", "nWarningCode": "UINT", "sWarningText": "STRING", "fBattSOC_pct": "LREAL",
    "fBattSOH_pct": "LREAL", "fBattVoltage_V": "LREAL", "fBattCurrent_A": "LREAL", "fBattTempMax_degC": "LREAL",
    "fBattTempMin_degC": "LREAL", "nBattCellVmax_mV": "UINT", "nBattCellVmin_mV": "UINT",
    "fBattCapacity_Ah": "LREAL", "fBattFullCap_Ah": "LREAL", "nBattCycleCnt": "UINT", "nBattProtLvl1": "UINT",
    "nBattProtLvl2": "UINT", "bBattCharging": "BOOL", "bBattConnected": "BOOL", "bBattError": "BOOL",
    "fPosX_m": "LREAL", "fPosY_m": "LREAL", "fPosTheta_deg": "LREAL", "bLocalizationOK": "BOOL",
    "bObstacleDetected": "BOOL", "nActiveMissionId": "UINT", "sMissionStatus": "STRING",
}
FROM_TYPES_V2 = {
    "nIfVersion": "UINT", "sPlcBuild": "STRING", "bExtActive": "BOOL", "eMoveState": "INT", "eMoveResult": "INT",
    "eMoveCmdResult": "INT", "nMoveCmdAck": "UINT", "bMoveLimited": "BOOL", "sMoveText": "STRING",
    "bMoveActive": "BOOL", "bMoveIsRotation": "BOOL", "fMoveTarget": "LREAL", "fMoveDone": "LREAL",
    "fMoveRemaining": "LREAL", "fMoveLatErr_mm": "LREAL", "fMoveHeadErr_deg": "LREAL", "fMoveProgress_pct": "LREAL",
    "fMoveFinalErr": "LREAL", "fMoveElapsed_s": "LREAL", "fOdomDist_m": "LREAL", "nOdomGlitchCnt": "UINT",
    "bFkLegacy": "BOOL",
}
DEFAULTS = {"BOOL": False, "UINT": 0, "INT": 0, "LREAL": 0.0, "STRING": ""}
JOG = ("bCmdJogFwd", "bCmdJogBwd", "bCmdJogLeft", "bCmdJogRight", "bCmdJogRotLeft", "bCmdJogRotRight")
AMR_NAMES = {0: "INIT", 1: "WAIT SAFETY", 2: "SAFETY STOP", 3: "RESET REQUIRED", 4: "STANDBY", 5: "DRIVES ENABLE",
             6: "READY", 7: "MANUAL MODE", 8: "AUTO MODE", 9: "NAVIGATING", 13: "ERROR", 16: "PRECHARGE"}
ST_SAFETY_STOP, ST_STANDBY, ST_READY, ST_MANUAL, ST_ERROR = 2, 4, 6, 7, 13

# E_MoveState / E_MoveResult (MA src/E_MoveState.st, src/E_MoveResult.st)
IDLE, ALIGN, RUN, SETTLE, DONE, ABORTING, ABORTED = range(7)
OWNING = (ALIGN, RUN, SETTLE, ABORTING)
ABORT_TEXT = {  # FB_RelMove.st:232-243
    20: "Aborted by operator (halt / abort / standby)", 21: "Aborted: jog command during the move",
    22: "Aborted: left MANUAL, test mode or drives not ready", 23: "Aborted: safety stop",
    24: "Aborted: system error or drive fault", 25: "Aborted: external control (C6030) became active",
    26: "Aborted: HMI heartbeat lost", 29: "Aborted: odometry parameters or pose invalid",
}

# GVL_Move.st init values
MAX_SPEED, MAX_ROT, MAX_ACC, MIN_ACC = 500.0, 45.0, 1000.0, 20.0
MAX_DIST, MAX_ANGLE, MIN_MM, MIN_DEG = 20000.0, 720.0, 2.0, 0.2
STOP_TOL_MM, STOP_TOL_DEG, DONE_TOL_MM, DONE_TOL_DEG = 1.0, 0.1, 5.0, 0.5
CREEP_MMS, CREEP_DEGS, BRAKE = 5.0, 1.0, 0.8
STANDSTILL_MMS, PLAUS_MM, PLAUS_DEG, STALL_MIN_CMD, STALL_RATIO = 2.0, 50.0, 5.0, 4.0, 0.2
T_ALIGN_STABLE, T_ALIGN_TIMEOUT, T_HB, T_SETTLE, T_SETTLE_TO = 100, 4000, 500, 300, 3000
T_ABORT_HOLD, T_ABORT_MAX, T_STALL, T_HMI_WD = 1000, 10000, 2000, 2000
ARM_MM = 353.625                    # GVL_Odom.fModuleX_L/R_mm


def wrap180(deg: float) -> float:
    """Wrap an angle to (-180, 180] deg like GVL_Odom.fThetaWrapped_deg."""
    w = deg - 360.0 * math.floor((deg + 180.0) / 360.0)
    return 180.0 if w == -180.0 else w


class FakeADSError(Exception):
    """Stands in for pyads.ADSError (err_code like pyads: 1808 symbol not found, 1861 timeout)."""

    def __init__(self, msg: str, err_code: int | None = None) -> None:
        super().__init__(f"ADSError: {msg}")
        self.err_code = err_code


@dataclass(frozen=True)
class WriteRecord:
    t_s: float
    conn: str                       # connection label ("client", "hmi", ...)
    name: str                       # short field name, e.g. "bCmdMoveStart"
    value: Any
    error: str                      # "no error" or the injected error text


class _Ton:
    """IEC TON on the PLC millisecond clock."""

    def __init__(self, plc: "FakePlc") -> None:
        self.plc = plc
        self.since: int | None = None
        self.q = False

    def __call__(self, IN: bool, PT: int) -> bool:
        if IN:
            if self.since is None:
                self.since = self.plc.now_ms
            self.q = self.plc.now_ms - self.since >= PT
        else:
            self.since, self.q = None, False
        return self.q


class FakePlc:
    """Simulated CX9240 with amr_hmi v2 open (MANUAL, drives enabled). Flags for tests: amr_state, ext_active,
    safety_stop, fault, test_mode, sysid_busy, odom_params_ok, block_wheels, hmi_alive, offline, ads_state,
    read_errors / write_errors (per-symbol, full names), align_s, final_err_mm / final_err_deg."""

    def __init__(self, *, if_version: int = 2, build: str = BUILD_V29, realtime: bool = False,
                 hmi_heartbeat: bool = True, align_s: float = 1.0, final_err_mm: float | None = None,
                 final_err_deg: float | None = None, ads_latency_s: float = 0.001) -> None:
        self.lock = threading.RLock()
        self.realtime = realtime
        self._t0 = time.monotonic()
        self.now_ms = 0
        self.if_version = if_version
        self.build = build
        self.ads_latency_s = ads_latency_s
        # ASSUMPTION: steering alignment time per move (E007: 90 deg steering in 1.12 s, MA sim_relmove.py:16)
        self.align_s = align_s
        self.final_err_mm = final_err_mm
        self.final_err_deg = final_err_deg
        # environment flags
        self.amr_state = ST_MANUAL
        self.ext_active = False
        self.safety_stop = False
        self.fault = False
        self.test_mode = False
        self.sysid_busy = False
        self.odom_params_ok = True
        self.block_wheels = False
        self.hmi_alive = hmi_heartbeat
        self.offline = False
        self.ads_state = 5
        self.read_errors: dict[str, str] = {}
        self.write_errors: dict[str, str] = {}
        # symbols
        self.to_types = dict(TO_TYPES_V1, **(TO_TYPES_V2 if if_version >= 2 else {}))
        self.from_types = dict(FROM_TYPES_V1, **(FROM_TYPES_V2 if if_version >= 2 else {}))
        self.T: dict[str, Any] = {k: DEFAULTS[t] for k, t in dict(TO_TYPES_V1, **TO_TYPES_V2).items()}
        self.F: dict[str, Any] = {k: DEFAULTS[t] for k, t in dict(FROM_TYPES_V1, **FROM_TYPES_V2).items()}
        # what amr_hmi v2 holds while open and in MANUAL (MA amr_hmi config.yaml: jog_accel_mms2 1000 on MANUAL)
        self.T.update(bCmdManualMode=True, bCmdSafetyRun=True, fAccel_mms=1000.0, fJogSpeed_mms=200.0,
                      fJogRotSpeed_degs=20.0, fSpeedLimit_mms=500.0)
        self.writes: list[WriteRecord] = []
        self.reads = 0
        self._schedule: list[tuple[int, Callable[["FakePlc"], None]]] = []
        self._on_write: dict[str, list[Callable[["FakePlc", Any], None]]] = {}
        self._frozen: dict[str, Any] = {}
        self._frozen_until: int | None = None
        # plant / odometry (true pose = odometry, no slip)
        self.x_mm = self.y_mm = self.th_rad = 0.0
        self.dist_mm = 0.0
        self.v_act = 0.0                     # actual speed in move units (mm/s or deg/s), >= 0 along the command
        self.amr_accel = 1000.0              # GVL_AMR.fAccel_mms
        # FB_HMI_Interface
        self._hmi_last_hb = 0
        self._hmi_wd = _Ton(self)
        self._hmi_ok = True
        self._manual_req = True
        self._manual_req_prev = True         # READY -> MANUAL on a rising edge of the request (_amr_state_machine)
        self._stop_req = False
        # MOVE_PRG
        self._start_prev = False
        self._odom_reset_prev = False
        self._last_id = 0
        self._hb_last = 0
        self._hb_seen = False
        self._hb_ton = _Ton(self)
        self.cmd_ack = 0
        self.cmd_result = 0
        self.result = 0                      # GVL_Move.eResult
        self.text = ""
        self.limited = False
        self._state_prev = IDLE
        # FB_RelMove
        self.state = IDLE
        self.fb_result = 0
        self.fb_text = ""
        self.rot = False
        self.ux = self.uy = 0.0
        self.sign = 1.0
        self.target = self.vmax = self.accel = self.a_move = self.creep = 0.0
        self.stop_tol = self.done_tol = self.margin = self.timeout_s = 0.0
        self.elapsed_s = self.s = self.rem = self.v = self.s0run = self.cmdint = 0.0
        self.final_err = self.lat = self.head = 0.0
        self.cmd = 0.0
        self._x0 = self._y0 = self._th0 = 0.0
        self._align_start = 0
        self._t = {k: _Ton(self) for k in ("align_stable", "align_timeout", "settle", "settle_to", "abort_hold",
                                             "abort_still", "abort_max", "stall")}
        self._publish()

    # ── test controls ─────────────────────────────────────────────────────────
    def clock(self) -> float:
        """PLC time [s] (simulated, or since construction in realtime mode)."""
        with self.lock:
            self._sync()
            return self.now_ms / 1000.0

    def sleep(self, s: float) -> None:
        """Simulated: run the PLC for s seconds. Realtime: time.sleep."""
        if self.realtime:
            time.sleep(max(s, 0.0))
            return
        self.run(s)

    def run(self, s: float) -> None:
        with self.lock:
            for _ in range(max(0, int(round(s * 1000.0)))):
                self._cycle()

    def at(self, t_s: float, fn: Callable[["FakePlc"], None]) -> None:
        """Run fn(plc) at the start of the first cycle at or after PLC time t_s."""
        with self.lock:
            self._schedule.append((int(round(t_s * 1000.0)), fn))
            self._schedule.sort(key=lambda e: e[0])

    def on_write(self, name: str, fn: Callable[["FakePlc", Any], None]) -> None:
        """Call fn(plc, value) after every ADS write of stToPlc field `name` (short name)."""
        self._on_write.setdefault(name, []).append(fn)

    def freeze(self, names: Iterable[str], duration_s: float | None = None) -> None:
        """Serve these stFromPlc fields (short names) with their current values - a stale/torn view of the PLC."""
        with self.lock:
            self._frozen = {n: self.F[n] for n in names}
            self._frozen_until = None if duration_s is None else self.now_ms + int(round(duration_s * 1000))

    def unfreeze(self) -> None:
        with self.lock:
            self._frozen, self._frozen_until = {}, None

    def tick_heartbeat(self) -> None:
        """One heartbeat write as amr_hmi would do it (manual tick)."""
        with self.lock:
            self._apply_write("hmi", "nHeartbeat", (self.T["nHeartbeat"] + 1) & 0xFFFF)

    def connection(self, label: str = "client", fail_open: bool = False) -> "FakeConnection":
        return FakeConnection(self, label, fail_open)

    def factory(self, label: str = "client") -> Callable[[], "FakeConnection"]:
        """Zero-argument connection factory for AresAds(connection_factory=...)."""
        return lambda: self.connection(label)

    def written(self, conn: str = "client") -> list[WriteRecord]:
        return [w for w in self.writes if w.conn == conn]

    def written_names(self, conn: str = "client") -> set[str]:
        return {w.name for w in self.written(conn)}

    @property
    def pose(self) -> tuple[float, float, float]:
        """True pose (x_mm, y_mm, theta_deg) - equals the odometry here (no slip)."""
        return self.x_mm, self.y_mm, math.degrees(self.th_rad)

    # ── ADS access (used by FakeConnection) ───────────────────────────────────
    def _sync(self) -> None:
        if self.realtime:
            target = int((time.monotonic() - self._t0) * 1000.0)
            while self.now_ms < target:
                self._cycle()

    def _latency(self) -> None:
        if not self.realtime:
            for _ in range(int(round(self.ads_latency_s * 1000.0))):
                self._cycle()

    def _split(self, name: str) -> tuple[str, str]:
        if name.startswith(TO) and name[len(TO):] in self.to_types:
            return "TO", name[len(TO):]
        if name.startswith(FROM) and name[len(FROM):] in self.from_types:
            return "FROM", name[len(FROM):]
        raise FakeADSError("symbol not found (1808). ", 1808)   # pyads: the error does not name the symbol

    def read_values(self, names: list[str]) -> dict[str, Any]:
        with self.lock:
            if self.offline:
                raise FakeADSError("timeout elapsed (1861). ", 1861)
            for n in names:                          # adsGetSymbolInfo for every name first
                self._split(n)
            self._sync()
            self._latency()
            self.reads += 1
            if self._frozen_until is not None and self.now_ms >= self._frozen_until:
                self._frozen, self._frozen_until = {}, None
            out: dict[str, Any] = {}
            for n in names:
                if n in self.read_errors:            # like adsSumRead: the error text replaces the value
                    out[n] = self.read_errors[n]
                    continue
                kind, f = self._split(n)
                out[n] = self.T[f] if kind == "TO" else self._frozen.get(f, self.F[f])
            return out

    def write_values(self, conn: str, values: dict[str, Any]) -> dict[str, str]:
        with self.lock:
            if self.offline:
                raise FakeADSError("timeout elapsed (1861). ", 1861)
            parsed = [(n, *self._split(n)) for n in values]
            self._sync()
            self._latency()
            res: dict[str, str] = {}
            for n, kind, f in parsed:
                err = self.write_errors.get(n)
                if err is not None:
                    self.writes.append(WriteRecord(self.now_ms / 1000.0, conn, f, values[n], err))
                    res[n] = err
                    continue
                if kind == "TO":
                    self._apply_write(conn, f, values[n])
                else:                                # writable over ADS, overwritten in the next cycle
                    self.writes.append(WriteRecord(self.now_ms / 1000.0, conn, f, values[n], "no error"))
                    self.F[f] = values[n]
                res[n] = "no error"
            return res

    def _apply_write(self, conn: str, f: str, value: Any) -> None:
        t = self.to_types.get(f) or TO_TYPES_V2[f]
        if t == "BOOL":
            if not isinstance(value, int):
                raise TypeError(f"{f}: BOOL needs bool, got {value!r}")
            v: Any = bool(value)
        elif t in ("UINT", "INT"):
            lo, hi = (0, 0xFFFF) if t == "UINT" else (-32768, 32767)
            if not isinstance(value, int) or isinstance(value, bool) or not lo <= value <= hi:
                raise TypeError(f"{f}: {t} needs an int in {lo}..{hi}, got {value!r}")   # pyads: struct.error
            v = int(value)
        else:
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise TypeError(f"{f}: LREAL needs a number, got {value!r}")
            v = float(value)
        self.T[f] = v
        self.writes.append(WriteRecord(self.now_ms / 1000.0, conn, f, v, "no error"))
        for fn in self._on_write.get(f, []):
            fn(self, v)

    # ── one 1 ms cycle: MAIN order ODOM -> MOVE -> AMR -> plant -> HMI_PRG ──
    def _cycle(self) -> None:
        while self._schedule and self._schedule[0][0] <= self.now_ms:
            _, fn = self._schedule.pop(0)
            fn(self)
        T = self.T
        if self.hmi_alive and self.now_ms % 100 == 0:       # emulated amr_hmi: heartbeat every 100 ms
            T["nHeartbeat"] = (T["nHeartbeat"] + 1) & 0xFFFF
        self._odom_reset()
        self._move_prg()
        self._amr_state_machine()
        self._plant(0.001)
        self._hmi_prg()
        self._publish()
        self.now_ms += 1

    def _odom_reset(self) -> None:
        edge = self.T["bCmdOdomReset"] and not self._odom_reset_prev
        self._odom_reset_prev = self.T["bCmdOdomReset"]
        if edge and self.state not in OWNING:                 # (deferred during a move in the PLC; ignored here)
            self.x_mm = self.y_mm = self.th_rad = self.dist_mm = 0.0

    def _wheel_mms(self) -> float:
        return self.v_act * (math.radians(1.0) * ARM_MM if self.rot else 1.0)

    def _ready(self) -> bool:                                   # MOVE_PRG.st:83-89 bAmrReady
        return (self.amr_state == ST_MANUAL and not self.safety_stop and not self.fault and not self.test_mode)

    def _move_prg(self) -> None:
        T = self.T
        # (1) heartbeat watchdog, MOVE_PRG.st:65-71
        if T["nHeartbeat"] != self._hb_last:
            self._hb_last, self._hb_seen = T["nHeartbeat"], True
            self._hb_ton(False, T_HB)
        hb_ok = self._hb_seen and not self._hb_ton(True, T_HB)
        any_jog = any(T[j] for j in JOG)
        standstill = abs(self._wheel_mms()) < STANDSTILL_MMS
        owns = self.state in OWNING
        # (3) start: rising edge + new id, MOVE_PRG.st:99-249
        edge = T["bCmdMoveStart"] and not self._start_prev
        self._start_prev = T["bCmdMoveStart"]
        start = rejected = False
        if edge and T["nMoveCmdId"] != self._last_id:
            self._last_id = self.cmd_ack = T["nMoveCmdId"]
            ang = T["fMoveTheta_deg"]
            rot = abs(ang) > 0.001
            speed = T["fMoveRotSpeed_degs"] if rot else T["fMoveSpeed_mms"]
            acc = T["fMoveAccel_mms2"]
            finite = all(math.isfinite(v) for v in (T["fMoveX_mm"], T["fMoveY_mm"], ang, speed, acc))
            dist = math.hypot(T["fMoveX_mm"], T["fMoveY_mm"]) if finite else 0.0
            checks = (
                (not finite, 10, "Rejected: parameter not a finite number"),
                (dist > 0.001 and rot, 10, "Rejected: translation and rotation in one command"),
                (not rot and not dist >= MIN_MM, 10, "Rejected: distance below GVL_Move.fMinMove_mm"),
                (rot and not abs(ang) >= MIN_DEG, 10, "Rejected: angle below GVL_Move.fMinMove_deg"),
                (not dist <= MAX_DIST, 10, "Rejected: distance above GVL_Move.fMaxDist_mm"),
                (not abs(ang) <= MAX_ANGLE, 10, "Rejected: angle above GVL_Move.fMaxAngle_deg"),
                (not (speed > 0.0 and acc > 0.0), 10, "Rejected: speed and acceleration must be > 0"),
                (not self.odom_params_ok, 10, "Rejected: odometry parameters implausible (GVL_Odom)"),
                (self.ext_active, 12, "Rejected: external control (C6030) active - stop the ROS bridge"),
                (self.amr_state != ST_MANUAL, 11, "Rejected: AMR not in MANUAL mode"),
                (self.test_mode, 11, "Rejected: TEST_PRG test mode active"),
                (not self._ready(), 11, "Rejected: drives not ready, safety stop or fault"),
                (not hb_ok, 11, "Rejected: HMI heartbeat too slow (needs < tHbTimeout)"),
                (owns, 13, "Rejected: a move is already running"),
                (self.sysid_busy, 13, "Rejected: E007 SysId run active"),
                (any_jog, 13, "Rejected: jog command active"),
                (T["bCmdMoveAbort"] or T["bCmdStop"], 13, "Rejected: abort / stop request still active"),
                (not standstill, 13, "Rejected: robot still moving"),
            )
            reject = next(((code, txt) for cond, code, txt in checks if cond), None)
            if reject is None:
                vmax, lim = speed, False
                mx, mn = (MAX_ROT, CREEP_DEGS) if rot else (MAX_SPEED, CREEP_MMS)
                if vmax > mx:
                    vmax, lim = mx, True
                vmax = max(vmax, mn)
                a = acc
                if a > MAX_ACC:
                    a, lim = MAX_ACC, True
                a = max(a, MIN_ACC)
                if 0.0 < self.amr_accel < a:
                    a, lim = self.amr_accel, True
                self.cmd_result, self.limited = 0, lim
                self._start(rot, T["fMoveX_mm"], T["fMoveY_mm"], ang, dist, vmax, a)
                start = True
            else:
                rejected = True
                self.cmd_result, self.text = reject
                if not owns:
                    self.result = reject[0]
        # (4) abort causes while a move runs, first match wins (MOVE_PRG.st:254-273)
        abort = 0
        if self.state in OWNING and not start:
            for cond, code in ((self.safety_stop, 23), (self.fault, 24), (self.ext_active, 25),
                               (not self.odom_params_ok, 29), (not hb_ok, 26), (not self._ready(), 22),
                               (T["bCmdMoveAbort"] or T["bCmdStop"], 20), (any_jog, 21)):
                if cond:
                    abort = code
                    break
        self._fb_relmove(abort)
        # (7) status, MOVE_PRG.st:326-336
        if self.state != self._state_prev:
            self.result = self.fb_result
            if not rejected:
                self.text = self.fb_text + (" (limited by PLC)" if self.limited and self.state in OWNING else "")
        self._state_prev = self.state

    def _start(self, rot: bool, dx: float, dy: float, ang: float, dist: float, vmax: float, a: float) -> None:
        """FB_RelMove.st:127-186: latch the command and the start pose."""
        self.rot = rot
        self.ux, self.uy = (0.0, 0.0) if rot else (dx / dist, dy / dist)
        self.sign = -1.0 if ang < 0.0 else 1.0
        self.target = abs(ang) if rot else dist
        self.accel = a
        if rot:
            self.creep, self.stop_tol = max(CREEP_DEGS, 0.5), STOP_TOL_DEG
            self.done_tol, self.margin = DONE_TOL_DEG, PLAUS_DEG
            self.a_move = a / ARM_MM * math.degrees(1.0)
        else:
            self.creep, self.stop_tol = max(CREEP_MMS, 2.0), STOP_TOL_MM
            self.done_tol, self.margin = DONE_TOL_MM, PLAUS_MM
            self.a_move = a
        self.vmax = max(vmax, self.creep)
        self.timeout_s = 2.0 * (self.target / self.vmax + self.vmax / self.a_move) + 10.0
        self._x0, self._y0, self._th0 = self.x_mm, self.y_mm, self.th_rad
        self.v = self.s = self.s0run = self.cmdint = self.elapsed_s = 0.0
        self.rem, self.lat, self.head, self.final_err = self.target, 0.0, 0.0, 0.0
        self.fb_result, self.fb_text, self.state = 0, "Aligning steering", ALIGN
        self._align_start = self.now_ms

    def _fb_relmove(self, abort: int) -> None:
        # (2) measurement in the start frame (FB_RelMove.st:191-225)
        if self.state in OWNING:
            self.elapsed_s += 0.001
            self._measure()
        # (3) external abort
        if abort and self.state in (ALIGN, RUN, SETTLE):
            self.fb_result, self.fb_text, self.state = abort, ABORT_TEXT.get(abort, "Aborted"), ABORTING
        # (4) timers
        still = abs(self._wheel_mms()) < STANDSTILL_MMS
        aligned = self.now_ms - self._align_start >= int(round(self.align_s * 1000))
        t = self._t
        t["align_stable"](self.state == ALIGN and aligned, T_ALIGN_STABLE)
        t["align_timeout"](self.state == ALIGN, T_ALIGN_TIMEOUT)
        t["settle"](self.state == SETTLE and still, T_SETTLE)
        t["settle_to"](self.state == SETTLE, T_SETTLE_TO)
        t["abort_hold"](self.state == ABORTING, T_ABORT_HOLD)
        t["abort_still"](self.state == ABORTING and still, T_SETTLE)
        t["abort_max"](self.state == ABORTING, T_ABORT_MAX)
        # (5) state machine
        cmd = 0.0
        if self.state == ALIGN:
            cmd = self.creep if aligned else 0.0              # precise mode holds traction until aligned
            if self.rem <= self.stop_tol:
                cmd, self.fb_text, self.state = 0.0, "Settling", SETTLE
            elif t["align_stable"].q:
                self.s0run, self.cmdint, self.fb_text, self.state = self.s, 0.0, "Moving", RUN
            elif t["align_timeout"].q:
                cmd, self.fb_result, self.state = 0.0, 27, ABORTING
                self.fb_text = "Aborted: steering not aligned within GVL_Move.tAlignTimeout"
            elif self.elapsed_s > self.timeout_s:
                cmd, self.fb_result, self.fb_text, self.state = 0.0, 27, "Aborted: total time exceeded", ABORTING
        elif self.state == RUN:
            exp_wheel = self.v * (math.radians(1.0) * ARM_MM if self.rot else 1.0)
            wheel = abs(self._wheel_mms())
            if self.rem <= self.stop_tol:
                self.v, cmd, self.fb_text, self.state = 0.0, 0.0, "Settling", SETTLE
            elif (self.s < self.s0run + 0.5 * self.cmdint - self.margin
                  or self.s > self.s0run + 2.0 * self.cmdint + self.margin):
                stall = wheel < STALL_RATIO * exp_wheel
                self.fb_result = 28 if stall else 29
                self.fb_text = ("Aborted: drives do not follow (stall)" if stall
                                else "Aborted: measured progress implausible (odometry)")
                self.v, cmd, self.state = 0.0, 0.0, ABORTING
            else:
                vbrake = math.sqrt(2.0 * BRAKE * self.a_move * max(self.rem - self.stop_tol, 0.0))
                self.v = max(min(self.vmax, self.v + self.a_move * 0.001, vbrake), self.creep)
                cmd = self.v
                exp_wheel = self.v * (math.radians(1.0) * ARM_MM if self.rot else 1.0)
                if t["stall"](exp_wheel >= STALL_MIN_CMD and wheel < STALL_RATIO * exp_wheel, T_STALL):
                    cmd, self.fb_result, self.fb_text = 0.0, 28, "Aborted: drives do not follow (stall)"
                    self.state = ABORTING
                elif self.elapsed_s > self.timeout_s:
                    cmd, self.fb_result, self.fb_text, self.state = 0.0, 27, "Aborted: total time exceeded", ABORTING
            self.cmdint += cmd * 0.001
        elif self.state == SETTLE:
            if t["settle"].q or t["settle_to"].q:
                forced = self.final_err_deg if self.rot else self.final_err_mm
                if forced is not None:                         # plant over/undershoot as configured
                    self._advance(self.rem - forced)
                    self._measure()
                self.final_err = self.rem
                ok = abs(self.rem) <= self.done_tol
                self.fb_result = 1 if ok else 2
                self.fb_text = "Done" if ok else "Done - final error outside tolerance"
                if t["settle_to"].q and not t["settle"].q:
                    self.fb_text += " (no standstill)"
                self.state = DONE
        elif self.state == ABORTING:
            if (t["abort_hold"].q and t["abort_still"].q) or t["abort_max"].q:
                self.final_err = self.rem
                if not t["abort_still"].q:
                    self.fb_text += " (no standstill)"
                self.state = ABORTED
        if self.state != RUN:
            t["stall"](False, T_STALL)
        self.cmd = cmd if self.state in OWNING else 0.0

    def _measure(self) -> None:
        dxw, dyw = self.x_mm - self._x0, self.y_mm - self._y0
        c, s = math.cos(self._th0), math.sin(self._th0)
        dxs, dys = c * dxw + s * dyw, -s * dxw + c * dyw
        if self.rot:
            self.s = self.sign * math.degrees(self.th_rad - self._th0)
            self.lat, self.head = math.hypot(dxs, dys), 0.0
        else:
            self.s = dxs * self.ux + dys * self.uy
            self.lat, self.head = dys * self.ux - dxs * self.uy, math.degrees(self.th_rad - self._th0)
        self.rem = self.target - self.s

    def _advance(self, ds: float) -> None:
        """Move the plant by ds move units along the command direction (pose and wheel path)."""
        if self.rot:
            self.th_rad += math.radians(self.sign * ds)
            self.dist_mm += abs(math.radians(ds)) * ARM_MM
        else:
            c, s = math.cos(self._th0), math.sin(self._th0)
            self.x_mm += ds * (c * self.ux - s * self.uy)
            self.y_mm += ds * (s * self.ux + c * self.uy)
            self.dist_mm += abs(ds)

    def _amr_state_machine(self) -> None:
        """Subset of FB_AMR_StateMachine (MA snapshot POUs/FB_AMR_StateMachine.TcPOU:209-240). READY -> MANUAL (the
        amr_hmi startup step "Manual", TcPOU:218) is taken on a RISING edge of the manual request here: the PLC
        uses the level, but tests force READY (amr_state = 6) while the emulated HMI holds the request TRUE."""
        rise = self._manual_req and not self._manual_req_prev
        self._manual_req_prev = self._manual_req
        if self.safety_stop:
            self.amr_state = ST_SAFETY_STOP
        elif self.fault and self.amr_state != ST_SAFETY_STOP:
            self.amr_state = ST_ERROR
        elif self.amr_state == ST_MANUAL:
            if self._stop_req:
                self.amr_state = ST_STANDBY                    # drives off, no PLC ramp
            elif not self._manual_req:
                self.amr_state = ST_READY
        elif self.amr_state == ST_READY and rise and not self._stop_req:
            self.amr_state = ST_MANUAL

    def _plant(self, dt: float) -> None:
        """Wheels follow the command with the move acceleration (forced traction ramp); no slip."""
        a = self.a_move if self.a_move > 0.0 else 1000.0
        if self.block_wheels or self.amr_state == ST_STANDBY:
            self.v_act = 0.0
        else:
            d = self.cmd - self.v_act
            self.v_act += max(-a * dt, min(a * dt, d))
        if self.v_act != 0.0:
            self._advance(self.v_act * dt)

    def _hmi_prg(self) -> None:
        """FB_HMI_Interface subset: 2 s watchdog and level requests (overlay FB_HMI_Interface.TcPOU:60-98,152-164)."""
        T = self.T
        if T["nHeartbeat"] != self._hmi_last_hb:
            self._hmi_last_hb = T["nHeartbeat"]
            self._hmi_wd(False, T_HMI_WD)
        self._hmi_ok = not self._hmi_wd(True, T_HMI_WD)
        if self._hmi_ok:
            self._manual_req, self._stop_req = T["bCmdManualMode"], T["bCmdStop"]
            if T["fAccel_mms"] > 0.0 and self.state not in OWNING:
                self.amr_accel = T["fAccel_mms"]
        else:
            self._manual_req = False

    def _publish(self) -> None:
        """stFromPlc as FB_HMI_Interface.TcPOU:176-340 builds it (modelled fields)."""
        F, T = self.F, self.T
        wheel = self._wheel_mms()
        vx = self.v_act * self.ux if not self.rot else 0.0
        vy = self.v_act * self.uy if not self.rot else 0.0
        om = self.sign * self.v_act if self.rot else 0.0
        F.update(
            nPlcHeartbeat=self.now_ms & 0xFFFF, nCommandAck=T["nCommandId"], bHmiWatchdogOK=self._hmi_ok,
            bPlcRunning=True, eAmrState=self.amr_state, sAmrStateText=AMR_NAMES.get(self.amr_state, "UNKNOWN"),
            bAmrReady=self.amr_state in (ST_READY, ST_MANUAL) and not self.fault, bModeManual=self._manual_req,
            bAmrMoving=abs(vx) > 1.0 or abs(vy) > 1.0 or abs(om) > 0.1,
            bPreciseModeActive=T["bCmdPreciseMode"] or self.state in OWNING,
            bSafetyOK=not self.safety_stop, bEmergencyStopActive=self.safety_stop, bSafetyRunActive=T["bCmdSafetyRun"],
            bDrivesEnabled=self.amr_state in (5, ST_READY, ST_MANUAL, 8, 9), bDrivesFault=self.fault,
            bTractionReady=not self.fault, bSteeringReady=not self.fault, bFaultActive=self.fault,
            nFaultCode=10 if self.safety_stop else (1 if self.fault else 0),
            sFaultText="Safety stop active" if self.safety_stop else ("Drive system error" if self.fault else ""),
            fActVx_mms=vx, fActVy_mms=vy, fActOmega_degs=om, fActSpeed_Left_mms=wheel, fActSpeed_Right_mms=wheel,
            fSetVx_mms=0.0 if self.rot else self.cmd * self.ux, fSetVy_mms=0.0 if self.rot else self.cmd * self.uy,
            fSetOmega_degs=self.sign * self.cmd if self.rot else 0.0, bBattConnected=True,
            fPosX_m=self.x_mm * 0.001, fPosY_m=self.y_mm * 0.001, fPosTheta_deg=wrap180(math.degrees(self.th_rad)),
            sMissionStatus="NOT_IMPLEMENTED",
            nIfVersion=self.if_version, sPlcBuild=self.build, bExtActive=self.ext_active,
            eMoveState=self.state, eMoveResult=self.result, eMoveCmdResult=self.cmd_result, nMoveCmdAck=self.cmd_ack,
            bMoveLimited=self.limited, sMoveText=self.text, bMoveActive=self.state in OWNING,
            bMoveIsRotation=self.rot, fMoveTarget=self.target, fMoveDone=self.s, fMoveRemaining=self.rem,
            fMoveLatErr_mm=self.lat, fMoveHeadErr_deg=self.head,
            fMoveProgress_pct=min(max(100.0 * self.s / self.target, 0.0), 100.0) if self.target > 0.001 else 0.0,
            fMoveFinalErr=self.final_err, fMoveElapsed_s=self.elapsed_s, fOdomDist_m=self.dist_mm * 0.001,
        )


class FakeConnection:
    """pyads.Connection stand-in on a FakePlc (open/close/set_timeout/read_state, *_by_name, *_list_by_name)."""

    def __init__(self, plc: FakePlc, label: str = "client", fail_open: bool = False) -> None:
        self.plc = plc
        self.label = label
        self.fail_open = fail_open
        self.is_open = False
        self.timeout_ms: int | None = None

    def open(self) -> None:
        if self.fail_open or self.plc.offline:
            raise FakeADSError("target machine not found (7). ", 7)
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def set_timeout(self, ms: int) -> None:
        self.timeout_ms = int(ms)

    def _check(self) -> None:
        if not self.is_open:
            raise FakeADSError("port is not open")

    def read_state(self) -> tuple[int, int]:
        self._check()
        return self.plc.ads_state, 0

    def read_list_by_name(self, data_names: list[str], cache_symbol_info: bool = True, **_: Any) -> dict[str, Any]:
        self._check()
        return self.plc.read_values(list(data_names))

    def write_list_by_name(self, data_names_and_values: dict[str, Any], cache_symbol_info: bool = True,
                           **_: Any) -> dict[str, str]:
        self._check()
        return self.plc.write_values(self.label, dict(data_names_and_values))

    def read_by_name(self, data_name: str, plc_datatype: Any = None, **_: Any) -> Any:
        self._check()
        v = self.plc.read_values([data_name])[data_name]
        if data_name in self.plc.read_errors:
            raise FakeADSError(str(v))
        return v

    def write_by_name(self, data_name: str, value: Any, plc_datatype: Any = None, **_: Any) -> None:
        self._check()
        err = self.plc.write_values(self.label, {data_name: value})[data_name]
        if err != "no error":
            raise FakeADSError(err)


class HmiHeartbeat:
    """Emulated amr_hmi v2 heartbeat writer: own connection (as the HMI's ADS worker), nHeartbeat + 1 every
    period_s of real time. Use with FakePlc(realtime=True, hmi_heartbeat=False)."""

    def __init__(self, plc: FakePlc, period_s: float = 0.1) -> None:
        self.conn = plc.connection("hmi")
        self.period_s = period_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="fake-hmi-heartbeat", daemon=True)

    def _run(self) -> None:
        self.conn.open()
        n = int(self.conn.read_by_name(TO + "nHeartbeat"))
        while not self._stop.is_set():
            n = n % 0xFFFF + 1
            self.conn.write_by_name(TO + "nHeartbeat", n)
            self._stop.wait(self.period_s)

    def start(self) -> "HmiHeartbeat":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def __enter__(self) -> "HmiHeartbeat":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.stop()
