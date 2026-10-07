# Plan 2026-10-07: wall variants, Mauer HMI, RoboDK digital twin, real test with ARES

Request (Samuel, 2026-10-07, German): keep the current simulation (A, C, B) but optimise its last moves (ARES drives
once more for a single stone); add a C that can be built strictly A -> B -> C with one side slightly longer (sim +
video, stone list, DXF); prepare the real test with ARES: take the ARES HMI and modify it for this application
(camera image, feedback on the test and the robot state); RoboDK as a digital twin alongside. Work autonomously,
also across context limits - THIS FILE is the state to resume from.

## Decisions (Samuel, AskUserQuestion 2026-10-07)

| # | Decision |
|---|---|
| D1 | A -> B -> C variant: **A 5 1/2 stones (1.10 m)**, B 7, C 5; A and B run through their corners (through="prev"), C butts against B; free ends 22.7 mm apart (C sticks out). No closure stone. |
| D2 | **The A -> B -> C variant becomes the main config** (config/station.toml, floor guides to be laser-cut, stone list, real test). The current A, C, B layout (A and C run through, B closes) is kept as a **variant config** with its own sim + video. |
| D3 | **The UR is mounted on ARES.** Test plan starts with the mount check and the hand-eye calibration on ARES. |
| D4 | **Copy amr_hmi** (MA repo `10_robot/hmi/amr_hmi`, PySide6, ADS worker) into this repo and extend it into the Mauer HMI. The MA repo stays untouched. One program, one ADS connection. |
| D5 | Single-stone stops: the planner must not send ARES to a stop for one stone (e.g. C A,C,B: stop 3 = 1 stone of B). |

Environment: Windows py.exe 3.14.3, PySide6 6.11.0, pyads 3.5.2, ids_peak, OpenCV 4.14 contrib, RoboDK 6.0.

## Tracks

### A - planning core (main tree)
- [x] A1 config variants: `config/variants/<name>.toml` overlays merged over station.toml (`mauer.config.load(variant=)`),
      status tags of the overlay keys, `--variant` on make_job / make_guides / make_stone_list / simulate / run_job;
      outputs per variant (job, guides dir, stone list, sim report, video). Variant `c_acb` = today's layout.
- [x] A2 legs with a half stone extra (n0 = 5.5): wallplan layout (course 0: 5 full + half at the corner end; odd
      courses: half at the start + 5 full), sequencing, make_job, make_guides (course-0 half stone: cones, footprint,
      joints/V-tabs from the course-0 stones), stone list. Boards on A unchanged (joints at 200, 400, ...).
- [x] A3 stop optimisation (D5) in wallplan.sequence_leg: no last stop with a few stones if the stones can be placed
      from the previous stop (moved within the window) or the batches rebalanced.
- [x] A4 main config = D1 layout; regenerate job, guides/DXF, floor map, stone list; tests.
- [ ] A5 RoboDK runs + time-lapse videos: main (A 5 1/2 -> B -> C) and variant c_acb (optimised stops).

### B - Mauer HMI (hmi/, copy of amr_hmi)
- [ ] B1 copy amr_hmi (attribution, tests green with offscreen Qt)
- [ ] B2 Mauer tab: job/plan view, progress (stop, stone, trip), run modes (sim / real), step confirmations as
      dialogs, pause / abort, preflight list, run log view
- [ ] B3 camera view: live image (IDS via mauer camera backend, or the sim camera), ChArUco detections overlaid,
      last measurement (board, reprojection error)
- [ ] B4 UR panel: connection, robot/safety mode, joints, TCP, gripper, payload, program state
- [ ] B5 ARES panel: amr_hmi widgets (state, battery, odometry map, relative move) + sequencer pose estimate
- [ ] B6 shared ownership: one ADS worker, one UR link, one camera for HMI + sequencer (no second connections)

### C - RoboDK digital twin
- [ ] C1 `robodk/twin.py`: station, mirror UR joints (RTDE), ARES pose (sequencer estimate / odometry), stones
      placed / held / magazine / station from the sequencer events; ~5-10 Hz; works with --sim and --real
- [ ] C2 HMI toggle + status

### D - real test preparation
- [ ] D1 test plan (German, operator document): network, preflight, ADS + small relative move, UR mount check,
      hand-eye calibration on ARES, floor layout check with the boards, single stone pick/place (step mode), one
      stop, station trip, leg change, full wall; pass criteria and what to log
- [ ] D2 preflight additions found while writing D1; HMI shows them

## Status log
- 2026-10-07: plan written; decisions D1-D5.
- 2026-10-07 evening: A1-A4 done and committed (variants c_acb, A 5.5 legs, balanced stops B 22 + 8, main config
  A 5.5 -> B -> C, guides/map/stone lists for both). OPEN: A5 RoboDK runs + videos for main and --variant c_acb.
  Track B/C/D: workflow wf_7a4d2776-741 (mauer-hmi-twin) STOPPED at the usage limit - resume with
  Workflow({scriptPath: ~/.claude/projects/-mnt-c-Users-samue-ARES-Mauer/39cd2889-e019-422e-bafa-4b360197709b/workflows/scripts/mauer-hmi-twin-wf_7a4d2776-741.js,
  resumeFromRunId: "wf_7a4d2776-741"}) (finished agents are cached; check worktrees in /mnt/c/Users/samue/ARES_Mauer_wt
  and branches hmi-* first). Note: ARES moves keep operating pattern A (two ADS connections in one program).
