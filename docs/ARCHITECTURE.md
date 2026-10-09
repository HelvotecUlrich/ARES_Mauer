# Camera + robot architecture (2026-10-05)

How the eye-in-hand camera, the UR5 and ARES work together, and the contracts between the modules. Parameters and
their status tags live in `config/station.toml`; nothing here repeats values.

## Overview

```
                        ┌────────────── laptop (Windows, py.exe 3.14) ──────────────┐
IDS GV-51F0CP ──GigE──► │ mauer.camera ──► mauer.vision ──► mauer.reference          │
 (UR tool 24 V)         │      (IDS peak)     (OpenCV)        (frame fit)            │
                        │                                         │                  │
UR5 CB3 ◄──30002 URScript / 30004 RTDE / 29999 Dashboard── mauer.ur ◄── mauer.sequencer
                        │                                         │                  │
CX9240 v2.9 ◄──ADS (GVL_HMI, pattern A: amr_hmi v2 owns heartbeat)── mauer.ares ◄───┘
                        └───────────────────────────────────────────────────────────┘
RoboDK (robodk/): offline planning, simulated camera, look-pose study, end-to-end simulation.
```

Free software only: OpenCV 4.14.0.94 (ArUco/ChArUco, solvePnP, calibrateCamera, calibrateHandEye), IDS peak
(free runtime + `ids_peak` wheels), the official UR RTDE client (BSD-3, vendored), pyads. No HALCON.

## Frames and naming

| Frame | Definition |
|---|---|
| `ares` | ARES base_link: floor, centre between the steering axes, x forward, y left, z up |
| `base` | UR5 base frame = UR controller base = RoboDK UR5 base (`T_ares_base` = `[ur5]` mount) |
| `flange` | UR5 tool flange (tool0); RTDE `actual_TCP_pose` with TCP offset removed |
| `tcp` | gripper TCP = flange · transl(0, 0, tcp_z) · rotz(90°): x = stone length, y = jaw closing direction, z out of the flange; origin = top centre of the held stone (a half stone: [half_brick] grasp_above_top_mm = 20 mm above it, 2026-10-09 - every half-stone TCP pose is raised by it, `mauer.config.grasp_above_top_mm`) |
| `cam` | OpenCV camera frame: origin = projection centre, z = optical axis, x = image right, y = image down |
| `board` | OpenCV 4.14 CharucoBoard frame: origin = top-left outer corner of the printed board, x right, y down, z into the board |
| `wall` | origin on the wall centreline at floor level, x along the wall (ARES travel direction), y towards ARES, z up |
| `station` | pick-up station: origin at the front-left corner of the table top, x along the front edge, y away from ARES, z up |
| `leg` | L wall ([[wall.legs]]): one frame per leg, same convention as `wall` (origin on the leg centreline at the leg start, x along the leg, y towards that leg's ARES side, z up); leg A = `wall`; `T_wall_leg` from xyz_in_wall / rpy_in_wall_deg |

`T_a_b` = pose of frame b in frame a (`p_a = T_a_b @ p_b`), numpy 4x4 float64, **mm and rad**. UR poses on the wire
are `[x, y, z (m), rx, ry, rz (rad rotation vector)]` – convert only at the URScript/RTDE boundary
(`mauer.geometry.ur_to_T` / `T_to_ur`). Never average or compare rotation vectors component-wise.

Measurement chain: `T_base_board = T_base_flange @ T_flange_cam @ T_cam_board`, with `T_base_flange` from RTDE at the
time of the exposure, `T_flange_cam` from the hand-eye calibration, `T_cam_board` from solvePnP.

## Modules and contracts

All in package `mauer/` (no RoboDK imports). Scripts with argparse in `tools/`. RoboDK glue in `robodk/`.

### `mauer.geometry`, `mauer.config` (done)
4x4 helpers (`transl`, `rotx/y/z`, `inv`, `pose_xyz_rpy`, `ur_to_T`, `T_to_ur`, `ur_str`, `average_T`, `spread`,
`fit_rigid`, `pose_delta`, `from_robodk`, `to_robodk`); `config.load()`, `config.pose(table)`,
`T_flange_cam_nominal`, `T_flange_tcp`, `T_ares_base`.

### `mauer.vision`
- `targets.py`: `BoardSpec` (frozen dataclass: `name, squares_x, squares_y, square_mm, marker_mm, dictionary,
  first_id`), `board_specs(cfg) -> dict[str, BoardSpec]` (the calib board as `"calib"` plus one spec per
  `[[targets]]` entry, named like the target), `make_board(spec) -> cv2.aruco.CharucoBoard` (ids
  `first_id .. first_id + n_markers - 1`), `corners_obj(spec) -> (N, 3)` chessboard corners in the board frame.
- `detect.py`: `BoardDetection` (`board, corner_ids (N,), img_pts (N, 2), obj_pts (N, 3), marker_ids`),
  `detect_boards(img, specs, border_px) -> dict[str, BoardDetection]`; `BoardPose` (`board, T_cam_board, n_corners,
  rms_px, ok, reason`), `estimate_pose(det, intr, min_corners, max_reproj_px) -> BoardPose` (both SOLVEPNP_IPPE
  solutions via solvePnPGeneric, SQPNP fallback, each refined with solvePnPRefineLM from its normalised rotation
  vector (`normalised_rvec`, |rvec| <= pi), lowest RMS kept; finite/plausibility checks),
  `measure(img, specs, intr, vcfg) -> dict[str, BoardPose]`.
- `intrinsics.py`: `Intrinsics` (`K (3x3), D (5,), width, height, rms_px, n_views, meta`), `save/load` JSON
  (`calib/camera_intrinsics.json`), `nominal(cfg)` (pinhole from focal/pixel, zero distortion, cx = (W-1)/2 – the
  RoboDK simulated camera), `calibrate(images, spec) -> (Intrinsics, report)`.
- `handeye.py`: `solve(T_base_flange_list, T_cam_board_list) -> HandEyeResult` (`T_flange_cam`, all five OpenCV
  methods with their mutual disagreement, chosen method PARK, residual stats), `save/load` (`calib/handeye.json`),
  `board_in_base(...)`, `holdout_check(...)`, `plan_poses(T_base_board, T_flange_cam, n, ...) -> list[T_base_flange]`.
- `synth.py`: `render_board(spec, intr, T_cam_board, ...) -> uint8 image` (supersampled, optional distortion, blur,
  noise) for tests without hardware.
- `printables.py`: exact-scale vector PDF (reportlab) and PNG (for RoboDK) of a board with origin and axis marks.

### `mauer.camera`
`Frame` (dataclass: `image` uint8 (H, W), `t_start`, `t_end` = laptop `time.time()` around the exposure, `meta`).
`Camera` ABC: `open()`, `close()`, `is_open`, `grab() -> Frame`, `set_exposure_us(us)`, `set_gain(g)` (both return
the applied value), `info() -> dict`, context manager. `open_camera(cfg, kind='ids'|'files', **kw)` returns an
opened camera. Implementations: `ids.IdsCamera` (IDS peak, software trigger, Mono8), `files.FileCamera` (replay a
folder, JSON sidecars keep the timestamps), RoboDK simulated camera in `robodk/sim_camera.py` (duck-typed).

### `mauer.ur`
- `rtde/`: vendored official UR RTDE client (BSD-3, with LICENSE).
- `link.py`: `URLink.from_config(cfg, host=None)` (or explicit ports/registers): RTDE thread at 125 Hz with a
  time-stamped history (`state()`, `samples(t0, t1)`), one persistent drained 30002 socket,
  `run_block(body, name=..., timeout_s=..., settle_s=...) -> BlockResult` (wraps `def`, writes start/done ids to
  output_int_register 24/25 after `is_steady()`, error codes to register 26; detects compile errors, protective
  stops, stopped-without-done), `abort()` (stopl program + Dashboard stop), `flange_T(sample) -> T_base_flange`
  (mm, TCP offset from RTDE `tcp_offset`).
- `dashboard.py`: `Dashboard(host)`: power on/off, brake release, robotmode, safetymode, unlock protective stop,
  close popup, stop, PolyScope version, robot model.
- `script.py`: pure functions that return URScript text (set_tcp, set_payload, movej to a Cartesian pose via
  `get_inverse_kin(p, qnear)`, movel, place/pick relative to a measured frame with `pose_trans`, gripper pulses, tool
  voltage) – unit-testable without a robot.

### `mauer.ares`
`AresAds(cfg["ares_ads"], connection_factory=None)`: `connect()` (TCP pre-check on 48898 first), `close()`,
`status() -> AresStatus`, `preflight() -> list[str]` (empty = ok), `translate(dx_mm, dy_mm, speed, accel) ->
MoveOutcome`, `rotate(dtheta_deg, ...)`, `abort()`. Raises `MoveRefused` (local check), `AresNotReady` (preflight
failed, nothing written), `AresConnectionError`; a `MoveOutcome` means the command reached the PLC. Pattern A only:
writes only the allow-list `WRITABLE` (move fields, id, start, abort), never mode, jog, heartbeat or bCmdStop.
`tests/fake_plc.py`: pyads-like fake PLC with the MOVE_PRG handshake.

### `mauer.reference`
`placements(cfg) -> list[Placement]` (`name, parent, T_parent_board`), `fit_frame(observed: dict[name,
T_base_board], placements, specs) -> FrameFit` (`T_base_parent`, per-board residuals, rms/max mm): rigid fit of all
board corner points (heading from the baseline between boards), single board = its full 6D pose.

### `mauer.floor` (L wall)
Floor plan in the wall frame (pure Python): `AresShape` (chassis footprint, rotation radius), `Obstacle`, plate sites
(`plate_site`, `site_from_target`, `check_sites`), `validate_route(route, obstacles, ares, clearance_mm) -> [problems]`
(one translation OR one rotation per leg, PLC minimum move, swept hull per translation, swept circle per rotation,
intermediate waypoints >= clearance; a swept overlap counts also at clearance 0), `plan_route(start, goal, ...) ->
RoutePlan` (back off, axis-parallel legs, one rotation close to the goal, approach), `job_obstacles(cfg, legs,
T_wall_station, board_sizes)` (the same floor model from a job, without robodk/wallplan.py), `penetration`,
`swept_translation` / `swept_rotation`. Used by tools/make_job.py (build fails on any violation), tools/plan_layout.py,
the sequencer (resume check, `preflight_real` route re-check) and mauer.simworld (floor check of the true ARES path).

### `mauer.armcheck` (look planning)
Coarse collision test of a UR5 configuration (nominal DH links with the real shoulder / elbow offsets, gripper with
open jaws, camera and adapter - capsules with ASSUMED radii) against boxes in the wall frame (`stone_boxes`: the
stones built so far, over the ribs, plus the base plate). tools/make_job.py and tools/plan_layout.py accept a look
only if it is clear of the wall built by the END of its stop, for ARES at the stop, +-50 mm around it and at the
arrival standoff. A planning filter, not RoboDK's collision check.

### `mauer.job` v2
Adds `legs`, per stop `leg`, `route` (from the previous stop), `route_to_station` / `route_from_station` (wall-frame
Pose2D lists), per stone `leg` / `kind` ("full" | "half") / `length_mm` (key `(leg, course, index)`), station slot
`kind` / `layer` / `stack` (stacked holders, emptied from the top like the magazine), magazine `initial_kinds`;
version-1 files still load (defaults: no legs, full stones, direct moves, single station holders).
`SlotState` tracks the stone type per slot; `reload_plan` / `reload_short` are the shared reload rule.

### `mauer.sequencer`
Runs a job (JSON exported by the RoboDK planner) against backends with the same interfaces for real hardware and
simulation: per stop ARES relative move → look poses → images → `fit_frame` → place stones with poses relative to
the measured wall frame; magazine empty → drive to the pick-up station, measure it, reload, drive back, re-measure.
Job v2 routes: intermediate legs steered dead-reckoned to the waypoints; a route to a stop ends
`arrival_standoff_mm` before it (from the job's meta.route_check), the wall is measured there and the closed-loop
correction reaches the stop; a partly visible board (too few ChArUco corners) re-aims the looks by its position
(`vision.detect.coarse_poses`), nothing in view → bounded board search; the stone type selects the magazine slot and
the payload. Route progress (`RouteProgress`: route, next leg) survives an interruption: `run(stop_k)` continues the
interrupted route (a station trip still reloads), its first move is checked against `floor.job_obstacles`. A move
that ends not ok updates the estimate from the outcome's odometry (pose "unverified" → `confirm_pose()`), an ARES
error without outcome makes it unknown (→ `set_pose(pose)`); the first measurement after a resumed route / return uses
the route / station jump limits. A frame fit over boards of two legs that does not agree names the legs.
Real runs refuse to start while safety-relevant config values are PLACEHOLDER/unknown (payload, host, calibration).

## HMI (`hmi/`)
The Mauer HMI (`py.exe -m hmi`, usage `hmi/README.md`, design and contract `docs/HMI_DESIGN.md`). `hmi.amr` is the
MA amr_hmi v2 (commit 5935c5b) with relative imports and the run-lock hooks; its config comes from `station.toml`
`[hmi*]` / `[ares_ads]` (`hmi.core.config.amr_cfg`). `hmi.core` holds the services (no widgets): `session`
(job + its config variant), `rigs` (`SimRig` on `mauer.simworld`, `RealRig` = URLink + AresAds + camera +
calibration), `preflight`, `snapshot` (immutable `RunSnapshot` / `ShotView`), `sources` (read-only UR / ARES state),
`run_controller` (`RunController`: states, enable matrix, confirm protocol, pause / abort / HALT, resume checks),
`overlay` (ChArUco detections drawn on the preview), `twin_link` (frames for `robodk/twin.py`). `hmi.views` holds the
widgets; `hmi.main_window` the window. `mauer` never imports `hmi`; pyads, ids_peak and RoboDK are imported lazily.

| Resource | Owner | Driven by | Read by |
|---|---|---|---|
| ADS #1 (heartbeat, MANUAL, HALT, jog, GO, odometry reset) | `AdsWorker` (only with `--ares`) | `ads-worker` QThread | GUI via the worker queue (HALT priority 0) |
| ADS #2 (the run's relative moves) | `RealRig` (`AresAds`) | `mauer-run` | `mauer-halt`: abort pulse only without the worker |
| UR (RTDE, script, Dashboard) | `RealRig` (`URLink`, one for all) | `mauer-run` (URRobot blocks) | UR panel, status strip, twin: `link.state()`; `mauer-halt`: `link.abort()` |
| camera | `RealRig` / `SimRig` | `mauer-run` only (shots, manual grabs, overlay) | GUI: `ShotView` previews by signal |
| `Sequencer` + `RunLog` | `RunController` | `mauer-run` | GUI: run-log listener signals, `pause()` |
| RoboDK twin instance (port >= 20630) | `robodk/twin.py` `Twin` | `mauer-twin` | nobody (passive mirror) |

| Trigger | ARES | UR | Run |
|---|---|---|---|
| HALT (button, Space, Esc, any tab) | `AdsWorker.halt()` first (abort + jog bits FALSE); the run's `AresAds` latched (no start edge until Start / Resume); `AresAds.abort()` only without the worker | REAL: `URLink` latched (no program but the abort until Start / Resume), `URLink.abort()` in `mauer-halt` | pending confirmation released with "no", "halt" in the run log, run "error" / "aborted", step mode on for the resume |
| Pause / Abort | - | - | at the next motion boundary with empty jaws (`Sequencer.held`) |
| Decline (confirmation bar) | - | - | at once, also with a stone held ("Jaws empty" before the resume) |
| E-stop | hardware | hardware | the sequencer sees the failure ("error") |

HALT is an operating function; the E-stops remain the safety function. The latch (`URLink.inhibit` /
`AresAds.inhibit`, review 2026-10-08) closes the gap between a confirmation and the program / start edge it allows
(motion planning, the AresAds preflight): a motion is either sent before the abort, which stops it, or refused; the
Sequencer sees the backends through `hmi.core.rigs.HaltGate`, so a refused motion ends the run "aborted".

## Calibration files (`calib/`, versioned)
`camera_intrinsics.json` and `handeye.json` with values, date, number of views/poses, residuals, OpenCV version and
the dataset folder they came from. Raw images and robot poses go to `data/` (not versioned, large).

## Run modes
| Mode | Camera | Robot | ARES |
|---|---|---|---|
| unit tests (`py.exe -m pytest`) | `vision.synth` | `ur.script` text, fake sockets | fake PLC |
| URSim | `vision.synth` / files | URSim CB3 3.15.8 (Docker) | fake PLC |
| RoboDK simulation | RoboDK Cam2D on the flange | RoboDK UR5 | ARES frame moved with injected error |
| real | IDS peak | UR5 | CX9240 via ADS, amr_hmi open |
| HMI SIM (`py.exe -m hmi`) | `mauer.simcam.SynthCamera` (rendered) | `mauer.simworld.SimRobot` | `SimAres` (no ADS) |
| HMI REAL (`py.exe -m hmi --ares`) | IDS peak | UR5 (URLink) | CX9240: HMI ADS worker + the sequencer's AresAds (pattern A, one process) |
