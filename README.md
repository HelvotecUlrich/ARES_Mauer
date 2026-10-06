# ARES_Mauer – UR5 on ARES lays the first bricks

Side project to the master thesis (not part of it). A UR5 (CB3) on the mobile platform ARES places bricks from a
pallet on the robot into a wall beside it. ARES drives to the start, the UR5 places every brick it can reach, then
ARES drives forward by a fixed stop pitch and the UR5 continues.

Goal for now: **it just has to work** – no localisation of ARES; accuracy comes from the ARES relative move
(PLC v2.9, calibrated 01.10.2026) and the UR5 repeatability, corrected by a camera on the UR5 flange that looks at
markers at the wall and at the pick-up station (planned, see "Camera").

## Concept (2026-10-02)

```
UR5 controller ──Ethernet, Dashboard :29999──► Laptop: sequencer ──ADS (as HMI v2)──► CX9240 v2.9 relative move
  load stop_k.urp / play / programState           loop                       fMoveX_mm, nMoveCmdId, heartbeat
```

1. Operator loads bricks (jig), jogs ARES to the start mark, MANUAL mode.
2. Per stop: UR program `stop_k` places all reachable bricks, parks the arm, ends → sequencer sees `STOPPED` →
   ARES relative move `L` (150 mm/s) → next stop.
3. RoboDK (this repo) computes the reachable bricks per stop, the stop pitch `L`, the wall distance and the pick area
   on the deck, and generates the UR programs.

Accuracy estimate (unloaded calibration, details in the conversation of 2026-10-02): joints between stops ±2–5 mm,
lateral drift ~1 cm over 2–3 m (systematic ~0.11° heading change per move), absolute ±1–2 cm (start alignment);
height/tilt on the sprung casters not yet measured.

## Layout

| Path | Content |
|---|---|
| `config/station.toml` | all parameters, each tagged CONFIRMED / ASSUMPTION / PLACEHOLDER |
| `robodk/rdk_common.py` | config, RoboDK connection, poses, snapshots |
| `robodk/build_station.py` | builds the station: ARES (STEP), UR5, placeholder gripper, nominal wall |
| `robodk/reach_study.py` | reach study wall + pick area on the deck → `results/` |
| `robodk/simulate.py` | simulation: reach table → wall plan → collision-checked pick and place, ARES drives between stops |
| `robodk/wallplan.py` | wall layout (trapezoid; L legs as rectangles with half stones) and placement sequence: stones only on complete supports; plan and cross-leg checker |
| `mauer/floor.py`, `tools/plan_layout.py` | floor plan of the wall of legs (C, L): plates, ARES routes and their clearance check; candidates, drawing, plan report |
| `robodk/motion.py` | collision-checked motion planner (IK ranking, safe waypoints, contact phases) |
| `robodk/show_stop.py` | colours the stones of one stop in the station (from `results/reach_wall.csv`) |
| `cad/gripper_backengreifer_gino.tool` | gripper "Backengreifer" (EHPS-20-A + ribbed jaws) exported from Gino's `UR5_sim_v3.rdk` |
| `cad/` | ARES STEP (not versioned, 37.6 MB, see `cad/README.md`) |
| `robodk/library/UR5.robot` | RoboDK library file (not versioned, download link in `cad/README.md`) |
| `results/` | study outputs (CSV, summary, snapshots) |

## Running

RoboDK 6.0 is installed at `C:\RoboDK`; the scripts use its Python API directly (no pip). Run with Windows Python,
e.g. from WSL:

```bash
py.exe robodk/build_station.py --snapshot      # (re)build station, save robodk/ARES_UR5_Mauer.rdk
py.exe robodk/reach_study.py --coarse --deck   # ~15 min; full grid without --coarse ~45 min
py.exe robodk/simulate.py --animate          # the wall of the config (the C, ~35 min) + station trips, results/l_wall_sim.md
```

From Windows (cmd/PowerShell in `C:\Users\samue\ARES_Mauer`): `build` rebuilds the station, `sim` runs the
simulation (`sim --animate`; `sim --no-trips` refills the magazine without driving or picking). The simulation runs in
its own RoboDK instance (minimised - restore it from the taskbar to watch; the camera window opens minimised too) and
saves `robodk/ARES_UR5_Mauer_L.rdk` at the end: open that file in RoboDK to look at the result. Simulated:
kinematics, reach, collisions in planning, order of operations, station trips. Not simulated: dynamics, jaw motion,
tipping, pin engagement, ARES drive error (`tools/run_job.py --sim` covers that without RoboDK).

## Frames

`ARES base_link` = floor, midpoint between the steering axes, x forward, y left, z up (same as `ares_description`
in the MA repo). The STEP export of 2026-09-23 is already in this frame (checked: steering plates at x = ±353.6 mm,
front scanner at (452, 192) mm, deck top z = 333.6 mm by ray test in RoboDK).

## Update 2026-10-02 (afternoon): real stone, gripper, wall at the front

- Stones: full stone "Kompletter_Stein" 120 × 200 × 120 mm, dry-stacked with conical pins/sockets (self-centring,
  capture range ≈ 10 mm), ribbed long faces matching the ribbed jaws (`cad/README.md`). Mass unknown (2880 cm³; the
  2270 cm³ quoted here first came from an STL with flipped faces, corrected 2026-10-05).
- Gripper: Festo EHPS-20-A with ribbed jaws (+ Bota SensONE?) – envelope and TCP 190 mm are placeholders.
- Wall on the **front (short) edge** of ARES (Samuel: more moves, but less prone to tipping); ARES moves sideways
  (+y) between stops – relative move dy, not yet calibrated (E003 covered x only).
- Reference archive from Gino (stones, moulds, gripper, UR5 docs, RoboDK scripts, bachelor report Heini):
  `reference/2026-10-02_Gino/INDEX.md`.

## Simulation (2026-10-02 evening, revised after Samuel's feedback)

- **Placement rules** (`robodk/wallplan.py`, verified independently by `check_plan()`):
  1. a stone only goes onto complete supports – it can never be pushed in under an already placed stone later;
  2. never "in the open": a course-0 stone must sit next to an already placed course-0 stone (only the very first
     stone of the wall is free); higher stones sit on their supports.
  The wall is continued where it ends: the reachable stone closest to the wall start is placed next, so the wall
  grows as a staircase (course 0 one stone ahead, the courses above following). Both wall ends slope back half a
  stone per course (full stones only). ARES moves on by whole stone lengths as far as the unfinished slope stays
  reachable; the next stop first completes that slope, then continues.
- **Paths** (`robodk/motion.py`): every segment is tested (MoveJ_Test 1° / MoveL_Test 2 mm – RoboDK's default 4°
  missed a forearm contact) against ARES, magazine, wall and the arm itself, held stone included; RoboDK's live
  collision check stays on during the run as a second safety net. Contact phases (last/first 60 mm at the stone)
  are not tested by design. Only one arm configuration is used: upper arm leaning towards the target, elbow up,
  wrist down – the arm never reaches back over the robot; moves between magazine and wall are turns of the base.
  Transfers: direct, else lift / base turn at height / compact pose near the base, else a joint-space RRT-Connect
  (seeded, every node and edge tested) with path shortcutting.
- **Magazine**: stones emptied layer by layer; 2 rows at x = UR − 653.6 / − 433.6 mm, 3 × 3 layers (15 of 18 slots
  reachable). No row closer to the UR: when the base turns, the upper arm (≈ 136 mm off the base axis) sweeps a
  circle of ≈ 190 mm radius from shoulder height upwards – a row at UR − 213.6 mm lies inside it.
- **Result** (`py.exe robodk/simulate.py --length 24`): 24 + 23 + 22 = 69 stones, 4 stops (24, 21, 21, 3 stones),
  ARES moves 3 × 1400 mm, 5 magazine reloads; **69/69 stones placed, 617 collision tests, no collision** (also no
  stop by RoboDK's live collision check).
- **UR position**: 450 mm instead of 353.6 mm gains ≈ 40 mm reach per side (±860 instead of ±820 mm), not enough
  for one more stone per stop (still 21 per stop, 1.4 m) – kept at the front steering axis (better for tipping).

## Camera (2026-10-02, planned – parameters in `config/station.toml` `[camera]`)

- **Hardware**: IDS GV-51F0CP-M-GL (Sony IMX547 mono, global shutter, 2472 × 2064 px, 2.74 µm, GigE, 29 mm cube)
  with IDS-12M23-C1228 (12 mm, F2.8–16, 2/3" image circle). Chosen over the IDS-8M118-C1220: same 12 mm, so the same
  field of view, but the larger image circle keeps the corners of the 8.8 mm sensor diagonal sharp and the lens is
  8 mm shorter; its smaller maximum aperture does not matter, it runs stopped down to F5.6–8 for depth of field.
- **Use**: eye-in-hand on the tool flange beside the gripper. (1) Markers at the wall give the place frame per
  stop. (2) When the deck magazine is empty, ARES drives to a fixed **pick-up station** and the UR5 reloads the
  deck there; a marker at the station references it (ARES drive error, wheel slip).
- **Field of view** 31.5° × 26.5°; at 320 mm (ASSUMPTION) 181 × 151 mm and 0.073 mm/px – a 60 mm marker stays fully in
  view for ARES errors up to ±45 mm (80 mm marker: ±35 mm, limited by the short side). Accuracy will be limited by the hand-eye calibration (Samuel's plate from
  the Heini thesis, HALCON `calibrate_hand_eye` ≥ 15 poses, or OpenCV) and by ARES swaying on its springs, not by
  pixels.
- **Power**: PoE (802.3af/at) or 12–24 V on the Hirose connector, max 4.2 W – never both. Planned: UR tool connector
  24 V (max 600 mA), so only the GigE cable (high-flex, screw-lock RJ45, slack at the wrist) runs along the arm.
  The gripper EHPS-20-A draws up to 2 A at 24 V: not from the tool connector, and at the 2 A limit of the control
  box's internal 24 V → supply it from ARES 24 V, only the open/close signals from the UR.
- **Open**: camera bracket and position on the flange (mass adds to a payload that the stone alone, ≈ 6.6 kg
  estimated from 2880 cm³ × 2.3 g/cm³ - first given as 5.2 kg from a wrong volume - already exceeds); marker positions that stay visible as the wall grows (the 120 mm gap between ARES
  front edge and wall face may need an inclined view – check in RoboDK); two markers per stop for the heading (a
  0.1° error gives 1.4 mm at u = ±800 mm); ARES tilt on the sprung casters while the arm reaches out (a look pose
  in the middle does not see it); place poses relative to a frame measured at run time (laptop → UR, instead of
  fixed `stop_k.urp`); tipping during reload starts with an empty deck → station within short reach.

## Camera + robot software (2026-10-05)

Runtime package `mauer/` (no RoboDK), tools in `tools/`, runbook **`docs/CAMERA_SETUP.md`**, contracts and frames
`docs/ARCHITECTURE.md`. Free software only (IDS peak, OpenCV 4.14.0.94, UR RTDE client, pyads) – no HALCON.

- Camera: IDS peak acquisition (`tools/cam_check.py`); the camera (serial 4110073444) is found but answers at
  192.168.0.1, outside the laptop NIC subnets → set its persistent IP first (runbook step 2).
- Vision: ChArUco boards (`tools/print_targets.py`), intrinsics (`tools/calib_intrinsics.py`), eye-in-hand hand-eye
  calibration on the robot (`tools/calib_handeye.py`), repeatability/settle time (`tools/measure_target.py`).
- UR5: RTDE state + one URScript block per step with done markers (`tools/ur_check.py`); tested on URSim CB3 3.15.8.
- ARES: relative move over ADS, pattern A (amr_hmi open, owns heartbeat/MANUAL) (`tools/ares_check.py`).
- Sequencer (`tools/make_job.py`, `tools/run_job.py`): per stop measure the wall boards, place relative to the
  measured wall frame, closed-loop ARES moves, reload at the pick-up station. World simulation, full wall, E003-like
  drive errors + slip up to 30 mm: **69/69 stones seated, max 0.66 mm with the camera loop vs up to 672 mm dead
  reckoning**; with an additional 1 mm UR-mount error (outside the loop, magazine picks) max 4 mm.
- Reference boards: PLACEHOLDER layout = face up on the floor in the 120 mm gap ARES front – wall face.
  `robodk/look_study.py` (quick grid, `results/look_study_quick.md`; the full study was stopped): this placement gives
  ≥ 2 visible boards in every stop and wall state; recommended pitch 800 mm. Their positions along the wall must be
  known exactly – a board misplaced by x mm moves the wall by x mm.
- Not done yet: real hardware tests, adversarial code review (started, stopped for time), RoboDK export of
  collision-checked jobs, deck/magazine referencing with the camera.

## C wall (2026-10-06, simulation only)

Samuel: "make it a small C, one length is 2m, then 1.5 across and 1m back", ARES a bit farther from the wall, keep the
existing boards. Plan and drawing: **`results/l_wall_plan.md`**, `results/l_wall_layout.png` (`py.exe
tools/plan_layout.py`; `--evaluate [--legs 10,7,5]`). RoboDK collision run (`results/l_wall_sim.md`, 13.4 min, saved as `robodk/ARES_UR5_Mauer_L.rdk`): **94/94 stones placed and 79/79 moved station → magazine in 6 trips with collision-checked motion**, 16 ARES routes (both leg changes, every station trip) without a colliding sample, every planned board seen in every state. **Open**: 4 of the 10 job looks (W2 at stop 0, W3 at stop 1, W4 at stop 2, W5 at stop 3) bring the wrist / gripper into ARES - RoboDK finds a grid pose for each, but `make_job`'s coarse arm check does not model ARES above the deck (at 740 mm only W4 of the L): the looks need the RoboDK check (planner export) or an ARES envelope before a real run.

- **Legs** (`[[wall.legs]]`): A 10 stones (2.0 m), B 7 (1.4 m; 1.52 m over the outer faces of A and C - B = 6 would
  give 1.32 m and one stop less), C 5 (1.0 m) back along A, a C open towards the station side; 94 stones (12 half). Every
  corner is a vertical butt joint: the 53 × 100.5 mm pin pattern is not square, so a stone turned by 90° never engages
  the sockets below. The earlier leg runs through (its end flush with the body of the next leg's outer face), the next
  leg starts at its inside face beyond the 1.69 mm ribs and a 1 mm gap (`[brick] rib_mm` CAD, `[wall] corner_gap_mm`
  ASSUMPTION). Half stones (`[half_brick]`, PLACEHOLDER) close both ends of every odd course, so every leg is a
  rectangle (`robodk/wallplan.py`). ARES works from the outside of every leg.
- **Wall distance 840 mm** (was 740): ARES front 110 mm from the board plates at a stop (was 10 mm), reach of the top
  course ±600 mm (±660), still **5 stops** (A 26 + 16, B 26 + 4, C 22 stones). Study 2026-10-06 (world simulation,
  5 seeds each, realistic errors + assumed board placement errors σ 0.3 mm / 0.1°): 740 and 840 mm give the same
  placement error (max 2.8-4.6 mm), 740 mm touched a plate once in 10 runs, 840 mm never (≥ 65 mm), 9 instead of 16
  ARES moves between stops. **One board in front** of each stop instead of two would work with 5 of the existing
  boards (one look per measurement), but the heading then comes from that board's rotation (stone yaw up to 0.24° vs
  0.07°; a board 0.3° crooked ≈ 3.5 mm at the stop ends) and a bumped board goes unnoticed (two boards: fit residual)
  - Samuel chose two boards. The reach table cache (`results/reach_table.json`, `mauer/reach_cache.py`) holds 740 and
  840 mm.
- **Boards**: the 8 existing boards on base-block centres (`[[targets]]`, `plan_layout --evaluate`): A 4 (W0-W3), B 2
  (W4, W5), C 2 (W6, W7); two own-leg boards at every stop, looks clear of the wall built by the end of the stop,
  worst baseline 800 mm.
- **ARES routes** (`mauer/floor.py`, `[routes]`): explicit wall-frame waypoints, one PLC translation or rotation per
  leg, validated against the legs, plates and station table (50 mm clearance, rotation circle r 635 mm), incl. both
  leg changes (A → B, B → C around the second corner) and the station trips from every stop. The sequencer steers them
  dead-reckoned, ends a route 40 mm before its stop (`[sequencer] arrival_standoff_mm`), measures the wall there and
  closes the loop to the stop; interrupted routes resume from their next leg.
- **Pick-up station** (2026-10-06, Samuel: "put the stones on the loading dock, as many as fit"; all PLACEHOLDER,
  `[pickup_station]`): ARES docks 100 mm from the table, the UR5 reaches 8 stacks of 2 full stones + 4 half stones =
  20 stones, so a trip refills the whole magazine (job v2 station slots stack: `layer` / `stack`). S0/S1 on the table
  top at the table ends; station looks are checked against a full station (`mauer.armcheck.station_boxes`).
- **World simulation** (`results/l_wall_sim.json`, realistic errors, seeds 1-3): camera loop 94/94 seated, max
  3.9-4.0 mm (out-of-loop mount and holder errors; 0.83 mm without them), 6 station trips, no ARES contact with a leg,
  plate or the table (smallest gap 60 mm); dead reckoning stops at a station pick or misses by up to 172 mm.

## L wall (2026-10-05, superseded by the C; `tests/conftest.py` l_config keeps it for the tests)

Samuel: a small L, A = 12, B = 6 stones (76, 8 half), 3 stops at the wall distance 740 mm, the 8 boards W0-W4 on A and
W5-W7 on B; a stop looks only at boards of its own leg (review 2026-10-05: a look across the corner put the wrist
inside leg A, and a fit over both legs moved the leg-A stones with any leg-B error). RoboDK run with the stacked
station (commit db78e1a): 76/76 placed, 61/61 moved station → magazine in 5 trips, no route collision. World
simulation: 76/76 seated, max 3.9-4.0 mm; the plates ended 10 mm before the ARES front at a stop - with the old
19-trip station a slip on the last approach put ARES up to 14.5 mm onto a plate in all 3 seeds (now settled by 840 mm).

## Results front wall (2026-10-02, `results/reach_summary.md`, coarse grid; placeholders: mount height, TCP 190 mm, base plate 20 mm, 3 courses)

- Wall distance (ARES centre → wall centreline) 690–740 mm: **8 stones per course per stop, ARES moves 1.6 m
  sideways**; 790–890 mm: 7; 940–990 mm: 6; 640 mm: blocked straight ahead (arm folds over the deck edge).
- Chosen: 740 mm (120 mm gap front edge – wall face). One stop = 24 stones over ≈ 1.7 m, arm reaches up to
  ≈ 800 mm sideways at the ends.
- Pick area on the deck (2 layers): x ≈ −300 … +140 mm, y ±240 mm → roughly 9–12 stones per layer with jaw
  clearance, 18–24 for 2 layers ≈ one stop per load (rough, magazine not modelled).
- Rough static tipping check (assumptions: ARES 80 kg with CoG at the centre, arm 18.4 kg midway base–TCP, 4 kg
  stone + 1 kg gripper at the TCP, support diamond drive wheels ±353.6 / casters (10, ±173)): CoG margin to the
  tipping edge 99 mm straight ahead, **≈ 1 mm at the wall ends (u = ±800) with an empty deck**, 60–95 mm with
  12–24 stones on the deck. Side layout for comparison: worst 16 mm. → place the outer stones while the deck is
  still full, or limit the stop to |u| ≤ 600 mm; real tipping check with measured masses still open.

## Results of the first study (side wall, NF brick placeholder – superseded)

Reach study (`results/reach_summary.md`, coarse grid 20 mm × 50 mm, 3 courses NF, TCP 150 mm, UR5 on the deck
without adapter):

- Wall distance (ARES centreline → wall centreline) 400–550 mm: **6 bricks per course per stop, stop pitch
  L = 1500 mm**; 600–700 mm: 5 (L = 1250 mm); 800 mm: 3; ≥ 900 mm: nothing reachable for all 3 courses.
- Chosen for the station view: 500 mm (142 mm gap between ARES side and wall face).
- Pick area on the deck (all 3 stack layers): x ≈ −380 … +100 mm over almost the full width, behind the UR5
  → roughly 15 NF bricks per layer, ~45 for 3 layers ≈ 2.5 stops (rough, pallet layout not modelled).
- Not checked yet: collisions with the brick stack on the deck, the UR control box, cables; motion between pick
  and place; payload (UR5 5 kg incl. gripper; ARES 200 kg incl. arm and bricks).
