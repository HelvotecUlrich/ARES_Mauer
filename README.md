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
| `robodk/wallplan.py` | wall layout (trapezoid) and placement sequence: stones only on complete supports; plan checker |
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
py.exe robodk/simulate.py --stops 2 --speed 3  # watch in RoboDK: magazine on the deck -> wall, ARES drives sideways
```

From Windows (cmd/PowerShell in `C:\Users\samue\ARES_Mauer`): `build` rebuilds the station, `sim` runs the
simulation (`sim --stops 3 --speed 5`). Simulated: kinematics, reach, collisions in planning, order of operations.
Not simulated: dynamics, jaw motion, tipping, pin engagement, ARES drive error.

## Frames

`ARES base_link` = floor, midpoint between the steering axes, x forward, y left, z up (same as `ares_description`
in the MA repo). The STEP export of 2026-09-23 is already in this frame (checked: steering plates at x = ±353.6 mm,
front scanner at (452, 192) mm, deck top z = 333.6 mm by ray test in RoboDK).

## Update 2026-10-02 (afternoon): real stone, gripper, wall at the front

- Stones: full stone "Kompletter_Stein" 120 × 200 × 120 mm, dry-stacked with conical pins/sockets (self-centring,
  capture range ≈ 10 mm), ribbed long faces matching the ribbed jaws (`cad/README.md`). Mass unknown (2270 cm³).
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
- **Open**: camera bracket and position on the flange (mass adds to a payload that the stone alone, ≈ 5.2 kg
  estimated, already exceeds); marker positions that stay visible as the wall grows (the 120 mm gap between ARES
  front edge and wall face may need an inclined view – check in RoboDK); two markers per stop for the heading (a
  0.1° error gives 1.4 mm at u = ±800 mm); ARES tilt on the sprung casters while the arm reaches out (a look pose
  in the middle does not see it); place poses relative to a frame measured at run time (laptop → UR, instead of
  fixed `stop_k.urp`); tipping during reload starts with an empty deck → station within short reach.

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
