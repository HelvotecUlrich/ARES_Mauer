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
| `tcp` | gripper TCP = flange · transl(0, 0, tcp_z) · rotz(90°): x = stone length, y = jaw closing direction, z out of the flange; origin = top centre of the held stone |
| `cam` | OpenCV camera frame: origin = projection centre, z = optical axis, x = image right, y = image down |
| `board` | OpenCV 4.14 CharucoBoard frame: origin = top-left outer corner of the printed board, x right, y down, z into the board |
| `wall` | origin on the wall centreline at floor level, x along the wall (ARES travel direction), y towards ARES, z up |
| `station` | pick-up station: origin at the front-left corner of the table top, x along the front edge, y away from ARES, z up |

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
  rms_px, ok, reason`), `estimate_pose(det, intr, min_corners, max_reproj_px) -> BoardPose` (SOLVEPNP_IPPE +
  solvePnPRefineLM, finite/plausibility checks), `measure(img, specs, intr, vcfg) -> dict[str, BoardPose]`.
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

### `mauer.sequencer`
Runs a job (JSON exported by the RoboDK planner) against backends with the same interfaces for real hardware and
simulation: per stop ARES relative move → look poses → images → `fit_frame` → place stones with poses relative to
the measured wall frame; magazine empty → drive to the pick-up station, measure it, reload, drive back, re-measure.
Real runs refuse to start while safety-relevant config values are PLACEHOLDER/unknown (payload, host, calibration).

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
