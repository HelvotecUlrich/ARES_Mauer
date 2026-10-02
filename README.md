# ARES_Mauer – UR5 on ARES lays the first bricks

Side project to the master thesis (not part of it). A UR5 (CB3) on the mobile platform ARES places bricks from a
pallet on the robot into a wall beside it. ARES drives to the start, the UR5 places every brick it can reach, then
ARES drives forward by a fixed stop pitch and the UR5 continues.

Goal for now: **it just has to work** – no localisation, no sensors; accuracy comes from the ARES relative move
(PLC v2.9, calibrated 01.10.2026) and the UR5 repeatability.

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
