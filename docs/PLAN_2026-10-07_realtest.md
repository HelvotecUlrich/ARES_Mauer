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
- [x] A5 RoboDK runs + time-lapse videos: main (A 5 1/2 -> B -> C) 76/76 and variant c_acb 74/74 (optimised stops);
      rerun after the park / look changes in progress (stamps the jobs RoboDK-verified).

### B - Mauer HMI (hmi/, copy of amr_hmi)
- [x] B1 copy amr_hmi (attribution, tests green with offscreen Qt)
- [x] B2 Mauer tab: job/plan view, progress (stop, stone, trip), run modes (sim / real), step confirmations as
      dialogs, pause / abort, preflight list, run log view
- [x] B3 camera view: live image (IDS via mauer camera backend, or the sim camera), ChArUco detections overlaid,
      last measurement (board, reprojection error)
- [x] B4 UR panel: connection, robot/safety mode, joints, TCP, gripper, payload, program state
- [x] B5 ARES panel: amr_hmi widgets (state, battery, odometry map, relative move) + sequencer pose estimate
- [x] B6 shared ownership: one ADS worker, one UR link, one camera for HMI + sequencer (no second connections)

### C - RoboDK digital twin
- [x] C1 `robodk/twin.py`: station, mirror UR joints (RTDE), ARES pose (sequencer estimate / odometry), stones
      placed / held / magazine / station from the sequencer events; ~5-10 Hz; works with --sim and --real
- [x] C2 HMI toggle + status

### D - real test preparation
- [x] D1 test plan (German, operator document): network, preflight, ADS + small relative move, UR mount check,
      hand-eye calibration on ARES, floor layout check with the boards, single stone pick/place (step mode), one
      stop, station trip, leg change, full wall; pass criteria and what to log
- [x] D2 preflight additions found while writing D1; HMI shows them
- [x] D3 motion guard (mauer/motionguard.py, required by URRobot) + checked park pose + look rolls by smallest
      joint change; guarded world sims main 76/76, c_acb 74/74
- [x] D4 RoboDK-verified job stamp (simulate.py robodk_stamp) for preflight_real
- [x] D5 review of the planning core (workflow review-planning-core: 10 findings fixed, 48e79b9)
- [x] D6 merge the HMI branch (hmi-integration) into main (277d1e8): main merged into it first (348ba08), then the
      D6 changes (7f21181: REAL start checklist + magazine fill, start at stop k > 0 with a run log / "stop untouched",
      SIM with the motion guard, approach_mm, IK_GUARD_33 only for ik_check has_solution; c773dec standing_in_log,
      unsent pick leaves the jaws empty); 669 passed / 40 skipped; offscreen smoke SIM + twin ok
- [x] D7 PolyScope 3.3 IK check ([ur] ik_check = "get_inverse_kin", 8894751; found by the test plan agent) + the IK
      error code into [ur] reg_error (was register 26); stamp carries the git state of the run start (87757fa)
- [~] D8 after D6: RoboDK rerun main done (76/76 + 62/62, job stamped at 277d1e8, clean); c_acb rerun stopped on
      request (Samuel wanted to wrap up) - its stamp is stale: `py.exe robodk/simulate.py --variant c_acb --animate
      --video results/l_wall_sim_c_acb.mp4` (only for the variant; the real test uses main)
- [x] D9 test plan Anhang C (software gaps found while writing it): C5 preflight blocks PLACEHOLDERs outside the camera
      loop (7d9459f), C7 [hmi*] tables out of the config hash / stamp (fce214f), C9 stale texts (c10d72a). C1-C4 go
      with D6 (HMI REAL start: magazine fill, jaws empty, start at stop k > 0, approach_mm, IK_GUARD_33). C6 (HMI test
      drive button) and C8 (warn when the PLC odometry says ARES is not at stop k's mark) open

## Status log
- 2026-10-07: plan written; decisions D1-D5.
- 2026-10-07 evening: A1-A4 done and committed (variants c_acb, A 5.5 legs, balanced stops B 22 + 8, main config
  A 5.5 -> B -> C, guides/map/stone lists for both). OPEN: A5 RoboDK runs + videos for main and --variant c_acb.
  Track B/C/D: workflow wf_7a4d2776-741 (mauer-hmi-twin) STOPPED at the usage limit - resume with
  Workflow({scriptPath: ~/.claude/projects/-mnt-c-Users-samue-ARES-Mauer/39cd2889-e019-422e-bafa-4b360197709b/workflows/scripts/mauer-hmi-twin-wf_7a4d2776-741.js,
  resumeFromRunId: "wf_7a4d2776-741"}) (finished agents are cached; check worktrees in /mnt/c/Users/samue/ARES_Mauer_wt
  and branches hmi-* first). Note: ARES moves keep operating pattern A (two ADS connections in one program).
- 2026-10-07 night: A5 done; motion guard (74e8328), stamp (f954da9), review fixes (48e79b9). RoboDK reruns for the
  new park / looks running (they also write the stamped jobs). HMI workflow wf_7a4d2776-741 resumed: maps cached,
  design done (88 kB), core implementing in /mnt/c/Users/samue/ARES_Mauer_wt/hmi-core.
- 2026-10-08 02:20: second review of the guard world fixed (22340be: wall that stands at a later start, pins, stone
  edges, floor, frames, restart fill, stamp from a clean tree). Guarded full sims (main, c_acb) running against
  22340be; then the RoboDK reruns (stamps must match 22340be or later). HMI workflow: core, camera, status, twin done
  (branches hmi-*), integrate running in /mnt/c/Users/samue/ARES_Mauer_wt/hmi-integration; then review, test plan, fix.
  hmi-integration is based on ed5ea2a: merging main brings 21b7731 + 22340be (rigs.py URRobot guard needs
  approach_mm=job.approach_mm; REAL start at stop k > 0 needs restart_fill / declare_placed like tools/run_job.py).
- 2026-10-08 02:47: guarded full sims against 22340be: main 76/76 (243 detours, 661 s), c_acb 74/74 (230, 605 s),
  nothing refused. RoboDK reruns started (main, then c_acb, both with video). HMI integrate done (639 passed; smoke run
  SIM + twin on port 20640; screenshot /mnt/c/Users/samue/ARES_Mauer_wt/hmi_screenshot.png); review + test plan running.
- 2026-10-08 03:20: RoboDK main rerun 76/76 + 62/62 (fb8ed83, stamp at 7a98fab, now stale by D7). c_acb rerun did
  not start (the wait loop's grep is ugrep -I: it skipped the log with Windows bytes - use /usr/bin/grep -a); both
  reruns after D6 (D8). D7 done. HMI workflow: reviews (3 lenses) + German test plan done, fix agent running in
  /mnt/c/Users/samue/ARES_Mauer_wt/hmi-integration. After it: merge main into hmi-integration (D6 notes above, also
  SimRig with guard=True, REAL start at stop k > 0: magazine fill + jaws-empty confirmation; hmi preflight IK_GUARD_33
  block -> [ur] ik_check), then into main.
- 2026-10-08 03:40: HMI workflow wf_7a4d2776-741 COMPLETE (14 agents). Fix commits on hmi-integration (worktree
  /mnt/c/Users/samue/ARES_Mauer_wt/hmi-integration, clean): 7ca642c URLink/AresAds HALT latch, d99f854 RunController
  (HALT latch + HaltGate, state lock, Start/Resume checks), e0360de window close order / keyboard-HALT banner,
  a443ba5 docs, 3c3fe08 docs/TESTPLAN_REALTEST_ARES_DE.md. Full workflow result:
  /tmp/claude-1000/-mnt-c-Users-samue-ARES-Mauer/fc936a49-c30e-44a7-9ec4-c90055219217/tasks/wweshl7rm.output.
  NEXT (D6): in the worktree `git merge main` (main has 8894751 ik_check, fce214f [hmi] out of the hash,
  7d9459f preflight blockers, c10d72a texts); then: rigs.py MotionGuard(..., approach_mm=job.approach_mm); SimRig
  guard=True (option, tests may switch it off); hmi/core/preflight.py IK_GUARD_33 -> only when [ur] ik_check is
  "has_solution" on PolyScope 3.3; REAL start: show magazine fill (job.restart_fill / initial fill), confirm jaws empty
  / magazine as listed / station full, start at stop k > 0 needs a run log (declare_placed) or "stop untouched";
  update the test plan (S1, S2, S6, Anhang C done items); full suite; merge into main; then D8 (RoboDK reruns).
- 2026-10-08 07:30: D6 done (HMI in main, 277d1e8; B1-B6, C1-C2, D1-D2 done: docs/HMI_DESIGN.md,
  hmi/README.md, docs/TESTPLAN_REALTEST_ARES_DE.md). Jobs rebuilt (make_job main + c_acb; the config hash no longer
  counts [hmi*] and blank lines). D8 running: RoboDK main then c_acb with video, logs scratchpad rdk_main_d8.log /
  rdk_cacb_d8.log (task bj5tc78fm). After it: commit the reports, send the videos (phone copies), final report.
- 2026-10-08 07:40: wrap-up on request. Main RoboDK rerun 76/76 + 62/62, stamp 277d1e8 (valid for the real test).
  c_acb rerun stopped (open, D8). Cron auto-resume deleted. Open: C6 (HMI test drive button), C8 (warn when the PLC
  odometry says ARES is not at stop k's mark), D8 c_acb; lab: measurements of T3 (half stone, mount z / rz,
  holder_z, payload COG, board print scale, settle time), park pose slowly, hand-eye on ARES, qnear on PolyScope 3.3
  (tools/calib_handeye.py plan --check), then the freeze (new stamp).
- 2026-10-08: Samuel mounts the UR base 200 mm behind the ARES front edge, centred in y -> [ur5] mount_x 353.625 ->
  360.0, mount_y CONFIRMED. Invalidates the reach tables, the jobs and the RoboDK stamp: rerun robodk/simulate.py
  (main; c_acb if wanted) - NOT started yet (Samuel: "starte noch nichts").
- 2026-10-08 late morning: Samuel - UR on a 10 mm plate, cable to the back (pendant Base +X = ARES left): [ur5] mount_z
  343.6, mount_rz 90 CONFIRMED (b8137f3; park_q_deg base joint 71.83). Quick movement test on ARES before the big
  simulation: magazine dry run (mauer/magtest.py, tools/make_magtest.py, magtest.cmd, HMI integration,
  docs/MAGTEST_DE.md) - 2 stones through the magazine (layers 1 + 2), a dry place 50 mm above the front leg between pick and put-down. HMI
  SIM 40/40. The wall jobs, reach tables and the RoboDK stamp stay invalid (mount x / z / rz): rerun robodk/simulate.py
  when the big simulation is due (Samuel: later).
- 2026-10-08 afternoon: network T0 on the switch (UR, ARES CX9240, camera PoE): ping all three, UR PolyScope 3.3.3
  RUNNING, camera 5/5 frames 107 ms; ARES ADS TCP 48898 times out (ping ok, Beckhoff MAC) - open. UR tool voltage set
  24 -> 0 V (camera on PoE, Hirose unplugged). Samuel: the UR sits at the vehicle REAR; pendant check X+ -> right,
  Y+ -> forward: mount_rz 90 confirmed in this repo's frame, which is base_link turned by 180 deg ([ares]
  frame_x_points_to = "rear"); preflight_real refuses wall runs until the ARES moves are mirrored. NEXT (Samuel,
  autonomous): controller monitor at the far end (25 cm out, 20 cm over the deck) into the collision models; hand-eye
  calibration on ARES + stone test tomorrow; new C with a door and a window from the photos in input/; new sim, validate.

