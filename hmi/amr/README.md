> Copied 2026-10-07 from the MA repo `10_robot/hmi/amr_hmi/README.md` (commit 5935c5b, unchanged below this note).
> In ARES_Mauer the config is `config/station.toml` [hmi], [hmi.frame], [hmi.move] and [ares_ads] (no config.yaml); start with `py.exe -m hmi`.
> Not copied: config.yaml, list_symbols.py, ads_diag.py, ads_portscan.py, requirements*.txt (see hmi/README.md).

# ARES HMI v2

External Python HMI for the ARES AMR (Beckhoff CX9240, TwinCAT 3).
Communicates with the PLC via ADS (pyads, symbolic access to `GVL_HMI.stToPlc` / `GVL_HMI.stFromPlc`)
and presents a PySide6 GUI.

Binding contract: `10_robot/hmi/SPEC_2026-09-25_Relativfahrt_HMI_SPS_v2.md` (sections 5 and 6, **section 9 = rev. 2
after the review, overrides 3-6 where they differ**).
Origin: working copy of `C:\Users\samue\VM2\amr_hmi\` (29.05.2026), reworked 25.09.2026 (rev. 2 the same day).

---

## What is new in v2

| Area | v1 (05/2026) | v2 |
|---|---|---|
| ADS | reads in a thread, **writes and heartbeat on the GUI thread**, one `write_by_name` per field, `timeout_ms` ignored | `AdsWorker` in its own `QThread` owns the connection: poll 100 ms (sum-read), heartbeat 100 ms, command queue, each command = **one** `write_list_by_name` incl. `nCommandId`, `set_timeout(timeout_ms)`, reconnect in the worker |
| Reconnect | stale jog/mode bits in the PLC could restart motion | after every (re)connect the **safe defaults** (all jog bits, horn, move start/abort, manual/auto request, pulse bits = FALSE) are written **before** the first heartbeat |
| Stop | one "Stop" button (= drives off via STANDBY) | big red **HALT** (Space/Esc, every tab): ramped stop, ARES stays in MANUAL. Old Stop kept as **Standby (drives off)** |
| Jog | global key filter; keys could stick on focus loss | keys only with active window + Control tab + Jog sub-tab; all jog bits released on focus loss, tab change, disconnect, MANUAL exit, ext active; physical directions mapped via config `frame` |
| New | - | **Relative move** (translate dx/dy or rotate dtheta, GO with second click), progress / final error, **odometry map** |
| PLC info | - | interface version detection (`nIfVersion`), PLC build, **external control (C6030) active** warning |
| Fixes | state 16 named "DRIVE_RESET", two colour tables, `setStyleSheet` every poll, Mission UI without PLC logic, duplicated battery block | state 16 = PRECHARGE, one table (`ui/constants.py`), styles only on change, Mission group removed, SOC only on the Control tab |

**28.09.2026 (DECISIONS D21, PLC v2.1):** the direction test on the robot found the PLC kinematics mirrored
("Left" module at the rear, drive angle CW +). PLC v2.1 fixes it; since then `bCmdJogLeft -> +vy`, the defaults are
`plus_y_is_left: true` / `plus_omega_is_ccw: true`, and the shipped config has `verified: true`. **This HMI needs
PLC v2.1** (with PLC v2 the jog Left/Right bits would drive the wrong way). The rev. 2 text below describes v2.

**Rev. 2 (25.09.2026, spec section 9):** direction default `frame.plus_y_is_left: false` (= previous HMI);
odometry map drawn physically oriented; "Reset pose" locked during a move; HMI rejects non-finite values and moves
below 2 mm / 0.2 deg; command verdict from the new field `eMoveCmdResult`, "LIMITED by PLC" from `bMoveLimited`;
result 29 = odometry implausible; jog parameters (`fAccel_mms`) are not written while a move runs; a v2 PLC whose
symbols do not match this HMI is reported by name instead of a connect/disconnect loop.

## Requirement: PLC build v2

Relative move, odometry pose, `bExtActive` and the applied setpoints need the **PLC build v2**
(`10_robot/twincat/PLC_CX9240_v2`, interface version 2). With an older PLC the HMI detects the missing
symbol `GVL_HMI.stFromPlc.nIfVersion`, falls back to the v1 variable list, shows a "Legacy PLC" banner and
disables the relative move. Jog, startup sequence and HALT (jog release) keep working. Only "symbol not found"
(ADS 1808) counts as "older PLC": any other error of the detection read (timeout, connection) fails the connect
and the reconnect timer retries, so a v2 PLC is never run as legacy (no move, HALT without `bCmdMoveAbort`)
because of a transient error.

A PLC that reports interface v2 must provide **every** field of the v2 lists in `plc_vars.py` (incl. the rev. 2
fields `eMoveCmdResult`, `bMoveLimited`). There is no reduced v2 mode: on connect the worker reads all v2 symbols
once; if some are missing (e.g. an older intermediate v2 build) the HMI does **not** connect, writes nothing and shows
"Disconnected: PLC build does not match this HMI: interface v2 but missing stFromPlc.eMoveCmdResult, ... - load the
matching PLC build v2" (full text in the status-bar tooltip). It retries every `reconnect_interval_s` and connects
as soon as the matching build runs. Without this check the first sum-read would fail with pyads
"symbol not found (1808)" and the HMI would connect and disconnect every 5 s.

## Setup (Windows)

```powershell
cd 10_robot\hmi\amr_hmi
py -3 -m venv .venv                   # or: <path to python.exe> -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe main.py      # optional: --config <path>
```

`requirements.txt`: `pyads>=3.5,<4`, `PySide6>=6.5,<7`, `PyYAML>=6,<7`. ADS route and
`config.yaml` (`ads` section) as described in `SETUP.md`. The HMI runs from a laptop connected directly to the
CX9240; the **C6030 / ROS bridge must be stopped** for HMI motion (see safety notes).

Tests (no PLC, no network; Qt offscreen):

```powershell
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.venv\Scripts\python.exe -m pytest -q tests
```

## Operation

### Startup sequence (Control tab, left)
Numbered badges show the step to do next (amber) and completed steps (green):
1. **Safety Run** ON (TwinSAFE start-up). After an E-stop with Safety Run ON: release the E-stop, **Re-arm Safety**.
2. **AMR Reset** (only in RESET REQUIRED / ERROR).
3. **Start** (STANDBY -> DRIVES ENABLE -> READY).
4. **Manual** ON (READY -> MANUAL). Jog and relative move are only possible in MANUAL.

**Standby (drives off)** = the old Stop: `bCmdStop` pulse + Manual/Auto OFF in one write. The state machine
goes to STANDBY and the drives are **disabled** (no PLC ramp; coast-down depends on the Kinco drive setting).

### HALT
Big red button above the tabs, also **Space** or **Esc** whenever the HMI window is active (any tab).
One write: `bCmdMoveAbort := TRUE` and all six jog bits FALSE; `bCmdMoveAbort` is reset after 300 ms by
the worker. Effect: the relative move is aborted with the ramp, jog stops with the traction ramp, ARES stays in
MANUAL with drives enabled. HALT does **not** use `bCmdStop`. HALT is an operating function, not a safety function.

### Jog (sub-tab "Jog")
Press and hold the buttons or W/S/A/D (forward/back/left/right) and Q/E (CCW/CW). Directions are **physical**
and mapped to the PLC bits through `config.yaml` `frame`. PLC v2.1 maps `bCmdJogLeft -> +vy`,
`bCmdJogRight -> -vy`, `bCmdJogRotLeft -> +omega`, `bCmdJogRotRight -> -omega` (v2: `bCmdJogLeft -> -vy`); with the
default `plus_y_is_left: true` physical Left uses `bCmdJogLeft`, with `false` it would use `bCmdJogRight`. Jog parameters (speed, rotation speed, speed limit, accel) are sent on change and when MANUAL is
entered; on leaving MANUAL `fAccel_mms` is restored to `hmi.accel_outside_manual_mms2`. **While a relative move
runs** (`bMoveActive`) the parameter boxes are locked and no jog parameter write (incl. `fAccel_mms`) is sent; a
write that falls into the move (e.g. MANUAL left during the move) is deferred and the latest values are sent once
the move has ended. (The PLC forces the move ramp anyway; this only avoids pointless writes.)
**Precise Mode** (latch `bCmdPreciseMode`): with PLC v2, `bPreciseModeActive` = own setting OR `bMoveActive` (the
PLC forces precise mode during a relative move). The button therefore toggles the own setting from the value the HMI
last wrote, never from the display; while no move runs the display equals the own setting and is taken over (e.g.
after an HMI restart). While a move runs the button is locked and reads `[ON, forced by move]` (tooltip: own
setting); the own setting applies again after the move.

### Relative move (sub-tab "Relative move")
1. Select a direction on the 3x3 pad (8 translations) or CCW/CW (rotation). The pad only selects, it does not move.
2. Translation: distance [mm]; dx/dy are shown in the PLC frame and can be edited freely (pad selection is
   cleared). The caption names the physical side of PLC +Y under the current config, e.g. `dy (+Y right*)` and
   "* dx/dy: PLC frame, PLC +Y (= right per config, unverified)". Rotation: angle [deg].
3. Speed [mm/s], rotation speed [deg/s], acceleration [mm/s2] (defaults 150 mm/s, 10 deg/s, 200 mm/s2,
   distance 1000 mm, angle 90 deg). When the PLC accepts the move it fixes
   a = min(move accel, `GVL_Move.fMaxAccel_mms2`, `fAccel_mms` of the Jog tab) and forces this ramp during the move,
   including the abort ramp.
   HMI-side checks (GO disabled, reason shown): finite numbers only (no NaN/inf), minimum move 2 mm / 0.2 deg
   (`move.min_distance_mm` / `min_angle_deg` = `GVL_Move.fMinMove_mm` / `fMinMove_deg`), maxima as in `move.max_*`.
4. **GO**: the first click shows the command in plain text ("CONFIRM: +X 1000 mm (forward) at 150 mm/s - click
   again"); a second click within 3 s sends it. Any parameter change cancels the confirmation.
   The command is one write: `fMoveX_mm, fMoveY_mm, fMoveTheta_deg, fMoveSpeed_mms, fMoveRotSpeed_degs,
   fMoveAccel_mms2, nMoveCmdId` (+1, wraps 65535 -> 1, never equal to the PLC's last id) and
   `bCmdMoveStart := TRUE` (reset after 300 ms).
5. GO is enabled only with interface v2, state MANUAL, no external control, no move running and valid
   parameters. Acknowledgement (spec section 9): `nMoveCmdAck` = sent id, verdict from **`eMoveCmdResult`**
   (0 = accepted, 10..13 = rejected, shown with `sMoveText`; any other value is reported as "unexpected PLC verdict",
   never as accepted). `eMoveResult` is the result of the move itself (while no move runs it also shows a rejection).
   The PLC also rejects a start while the robot still rolls (REJ_BUSY "robot still moving").
6. Progress bar and values: target, done, remaining, lateral error (+ = left of the path) or position drift
   (rotation), heading change, final error (+ = short), elapsed time, result and `sMoveText`. While the move runs
   with `bMoveLimited` the panel shows **"LIMITED by PLC: speed/accel reduced"**. Result 29 = "Aborted: measured
   progress implausible (odometry)" (PLC plausibility check in RUN).
7. **Abort move** sends only a `bCmdMoveAbort` pulse. HALT also works.

Odometry map (right): trail of `fPosX_m / fPosY_m`, robot outline 1.12 x 0.60 m rotated by `fPosTheta_deg`,
target marker of the accepted move, grid 0.5 m. The map is drawn **physically oriented, seen from above**, through the
same `frame` flags: screen right = PLC +X (forward at the odometry origin), screen up = physical left
(`screen_y = (+1 if plus_y_is_left else -1) * y`), heading drawn CCW = physical CCW
(`(+1 if plus_omega_is_ccw else -1) * theta`). The axis captions show the mapping ("left = PLC -Y (config,
unverified)"); the readout below stays in PLC values with the physical side in brackets. Values are only as right as
the config: until the test moves are done the drawing is an assumption. If the two tests end with exactly one flag
different from the PLC documentation (+Y left, +omega CCW) - e.g. the default combination +Y = right, +omega = CCW -
the PLC odometry frame is mirrored against its rotation sense: single straight moves and rotations are drawn
correctly, but positions integrated after a rotation are mirrored by the PLC itself (report to Claude: PLC IK/FK
issue, not an HMI setting; see "Direction verification", the two flags are confirmed as a pair).
**Reset pose** = `bCmdOdomReset` pulse (locked while `bMoveActive`, with a hint; the PLC also defers a reset until
the move has ended), **Clear trail** is local only. The pose is wheel odometry (no localisation, drifts).

## Direction verification (required once, `frame.verified: false`)

Done on 28.09.2026 with PLC v2.1 (D21): Left -> PLC +Y, CCW -> PLC +theta, odometry consistent after a rotation.
Repeat after any change of the PLC kinematics or wiring. Historic text (PLC v2):

Which physical side PLC +Y is and whether PLC +omega is counter-clockwise is **not verified** on the robot. The
default `plus_y_is_left: false` / `plus_omega_is_ccw: true` keeps the behaviour of the previous HMI ("Left" ->
`bCmdJogLeft` -> PLC -vy, "CCW" -> `bCmdJogRotLeft` -> PLC +omega). Hints, not evidence: the jog sign was deliberately
changed to -vy in 05/2026, and the ROS bridge of the 2026 internship also maps ROS +Y (left) to PLC -vy. The texts in
the HMI therefore say "PLC +Y (= right per config, unverified)" instead of stating a side as fact. Until the tests
are done the Control tab shows a banner with **Load test move**. Procedure (details: runbook block R):
1. Robot on a free floor area (after the jacked-up checks), E-stop within reach, C6030 stopped.
2. MANUAL, **Load test move** (physical Left 100 mm at 50 mm/s; with the default config the command reads
   "-Y 100 mm (left)"), GO twice.
3. If ARES moves to **its** left (seen from behind): the config is right. If it moves right: flip
   `plus_y_is_left` (`false` <-> `true`), restart the HMI, repeat.
4. Same with a small CCW rotation (e.g. 45 deg at 10 deg/s): CCW seen from above -> keep `plus_omega_is_ccw`, else flip.
5. **Confirm the two flags together, as a pair.** With the PLC kinematics (rigid-body IK
   `v_i = [vx - omega*y_i ; vy + omega*x_i]` in one frame, `FB_AmrKinematics`) a physically consistent result is
   only `plus_y_is_left: true` + `plus_omega_is_ccw: true` (+Y left, +omega CCW, as documented in `GVL_AMR`) or
   both `false` (+Y right, +omega CW, frame mirrored as a whole). A **mixed** result (one `true`, one `false`)
   means a sign error in the PLC kinematics or in the wiring / drive direction of a module: **stop, do not just
   set the flags** and do not set `verified: true`; record the observation (date, PLC build, which test moved
   which way) and report it. The HMI flags would only hide the error in single moves - the PLC odometry would
   still integrate positions mirrored after a rotation. Note: the default pair (`false` / `true`) is itself
   mixed, so with a correct PLC exactly one of the two tests leads to a flip.
6. Only with a consistent pair: check the jog keys A/D/Q/E and the odometry map once more (a Left move must be drawn
   upwards, a CCW rotation counter-clockwise), then set `frame.verified: true` and restart the HMI (banner gone,
   "unverified" disappears).
Record the result (date, PLC build, observation) in the experiment log / `PROJECT_STATUS.md` item 5.

## First drive after installing PLC v2 (checklist)

Details and the order of all steps: `40_experiments/RUNBOOK_Robotertag.md`, **block R**.
- [ ] C6030 / ROS bridge stopped; laptop directly on the CX9240 network; E-stop in reach.
- [ ] PLC v2 activated; HMI status bar shows `Interface: v2` and the PLC build `ARES CX9240 v2 2026-09-25`
      (a "PLC build does not match this HMI" message means: wrong build active - do not continue).
- [ ] **Jacked up** (wheels free): startup sequence, jog, HALT (Space), relative move +X 200 mm, HALT during a move,
      heartbeat loss (LAN cable), ALIGN of Left and CCW moves - as listed in block R step 4.
- [ ] **On the floor**: Y test move ("Load test move"), then rotation test move; flip the `frame` flags if needed.
- [ ] Both directions right **and the flag pair consistent** (both `true` or both `false`) -> `frame.verified: true`
      in `config.yaml`, restart the HMI, record the result. Mixed pair -> stop, record and report (PLC
      kinematics / wiring sign error), do not set the flags.
- [ ] Only then: longer moves / 1 m checks (block R step 5d).

## Safety notes

- The **E-stop on the robot** (TwinSAFE) remains the safety function. HALT, Abort and Standby are operating
  functions of the standard PLC and of this HMI; they rely on ADS, the PLC program and the drives.
- **External control:** while the C6030 (ROS bridge) is active it overrides the HMI setpoints. The HMI shows
  "External control active (C6030) - HMI motion blocked" and disables jog / GO; the PLC rejects or aborts a
  relative move. Stop the C6030 PLC / bridge before operating from the HMI.
- **HALT** = ramped stop, drives stay enabled. **Standby (drives off)** = drives disabled via STANDBY, no PLC ramp.
- The PLC aborts a relative move after 500 ms without HMI heartbeat (`GVL_Move.tHbTimeout`); jog and mode
  requests are cleared after 2 s without heartbeat (`FB_HMI_Interface`).
- After every reconnect the HMI writes all motion bits FALSE and Manual/Auto OFF: re-select Manual after a
  connection loss.

## Configuration (`config.yaml`)

| Section | Key | Meaning |
|---|---|---|
| `ads` | `ams_net_id`, `ads_port`, `host_ip` | ADS target (CX9240, port 851) |
| `ads` | `timeout_ms` | ADS timeout, applied with `Connection.set_timeout()` (default 1000) |
| `hmi` | `poll_interval_ms`, `heartbeat_interval_ms` | 100 / 100 ms (worker thread) |
| `hmi` | `reconnect_interval_s` | retry interval while disconnected |
| `hmi` | `jog_*`, `default_speed_limit_mms`, `accel_outside_manual_mms2` | jog defaults |
| `frame` | `plus_y_is_left` (default `false`), `plus_omega_is_ccw` (default `true`), `verified` | physical direction mapping for jog, relative move and map (spec 5.4 / section 9); `FrameConfig()` uses the same defaults |
| `move` | `default_*`, `test_*`, `max_*`, `min_distance_mm`, `min_angle_deg` | relative-move presets and HMI-side limits (PLC limits apply independently; `min_*` mirror `GVL_Move.fMinMove_*`) |

## Code structure

| File | Content |
|---|---|
| `main.py` | entry point, dark theme |
| `ads_worker.py` | `AdsWorker` (QThread): connection, poll, heartbeat, command queue, pulses, safe defaults, version detection, v2 symbol check |
| `plc_vars.py` | symbol lists interface v1 / v2, safe defaults |
| `frame.py` | physical direction -> PLC sign / jog bit, neutral mapping texts, pose transform for the map (pure) |
| `logic.py` | move command building, command ids, enable rules, startup step (pure) |
| `ui/constants.py` | E_AMR_State (0..16), lamp colours, E_MoveState, E_MoveResult |
| `ui/main_window.py` | window, HALT, tabs, status bar, key handling |
| `ui/control_widget.py` | Control tab: header, startup sequence, drives, sub-tabs |
| `ui/jog_panel.py`, `ui/move_panel.py`, `ui/odom_map.py`, `ui/widgets.py` | panels and shared widgets |
| `ui/dashboard_widget.py`, `ui/diagnostics_widget.py`, `ui/battery_widget.py` | read-only tabs (v1 and v2 status) |
| `tests/` | pytest, no ADS: frame mapping, logic, worker with fake pyads, offscreen GUI |
| `ads_diag.py`, `ads_portscan.py`, `list_symbols.py` | stand-alone ADS diagnostics (unchanged) |

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Status bar "Disconnected: connect failed ..." | route / AMS Net ID / IP (see `SETUP.md`), `python ads_diag.py` |
| "Legacy PLC (interface v1)" banner | PLC build v2 not active; relative move disabled |
| "Disconnected: PLC build does not match this HMI: interface v2 but missing ..." | the active v2 build lacks fields this HMI expects (e.g. `eMoveCmdResult`, `bMoveLimited`); load the matching PLC build v2 - the HMI retries by itself |
| "PLC HB: FROZEN" | PLC in STOP or not running |
| Banner "PLC does not see the HMI heartbeat" | heartbeat writes do not arrive; check route / firewall; commands are ignored by the PLC |
| GO disabled | the reason is shown below the GO button (not MANUAL, ext active, move running, ...) |
| Move rejected | red text with PLC reason (`eMoveCmdResult` 10..13, `sMoveText`) |
| GO disabled: "distance ... below the minimum 2 mm" / "not a finite number" | HMI-side check mirroring the PLC (`GVL_Move.fMinMove_*`, finite values) |
| "Reset pose" greyed out | a relative move runs (`bMoveActive`); available again after the move |
| Jog parameter boxes greyed out | a relative move runs; changes are sent after the move |
| Precise Mode greyed out, `[ON, forced by move]` | a relative move runs; the PLC forces precise mode, the own setting applies again after the move |
| Direction tests give a mixed flag pair (one `true`, one `false`) | sign error in the PLC kinematics / wiring - stop and report, do not set the flags |
| "LIMITED by PLC" | requested speed/accel above `GVL_Move` maxima or above the Jog-tab "Accel (MANUAL)" |
| Map drawn mirrored against the observed motion | `frame` flags wrong - repeat the direction verification |
| "No acknowledgement ... within 2 s" | PLC did not process the start edge (same id as last, PLC without move support, connection) |
| Jog does nothing | not MANUAL, ext active, relative move running, or focus not on the HMI window / Jog sub-tab |
| Robot moves in the wrong physical direction | `frame` config wrong - repeat the direction verification |
| Implausible values after a PLC online change | the worker caches symbol info per connection; restart the HMI after an online change of `ST_HMI_*` (a full download/restart triggers a reconnect anyway) |

## State reference (`E_AMR_State`)

| Index | Name | Index | Name |
|---|---|---|---|
| 0 | INIT | 9 | NAVIGATING (unreachable in the current PLC) |
| 1 | WAIT SAFETY | 10 | OBSTACLE STOP |
| 2 | SAFETY STOP | 11 | DOCKING |
| 3 | RESET REQUIRED | 12 | CHARGING |
| 4 | STANDBY | 13 | ERROR |
| 5 | DRIVES ENABLE | 14 | ERROR ACK |
| 6 | READY | 15 | SHUTDOWN |
| 7 | MANUAL MODE | 16 | PRECHARGE (waits for TwinSAFE `toPlc_Precharge`, then STANDBY) |
| 8 | AUTO MODE | | |
