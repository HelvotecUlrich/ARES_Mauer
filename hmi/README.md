# Mauer HMI (`hmi/`)

The ARES HMI v2 (amr_hmi, copied from the MA repo) extended for the brick-laying runs of this repo: load a job,
run it in the pure-Python simulation (SIM) or on ARES + UR5 + IDS camera (REAL), confirm every motion in step mode,
pause / abort / HALT, recover a stopped run, watch the camera images with their ChArUco detections, the UR and
ARES state and the run feedback, mirror the run in a RoboDK digital twin, and keep all amr_hmi functions
(connection, startup sequence, jog, relative move, odometry map, dashboard, diagnostics, battery). Design and
contract: `docs/HMI_DESIGN.md`.

## Start

```
py.exe -m hmi                                      # SIM only - no ADS connection ("ADS off"), REAL disabled
py.exe -m hmi --job data/jobs/nominal_C.json       # load a job at start (with the config variant it was built with)
py.exe -m hmi --job data/jobs/nominal_C.json --twin   # ... and mirror it in an own RoboDK (Twin tab)
py.exe -m hmi --ares                               # the HMI's ADS worker connects to the ARES PLC ([ares_ads])
hmi.cmd --job data/jobs/nominal_C.json             # the same from a Windows prompt (py -m hmi %*)
```

| Option | Meaning |
|---|---|
| `--job PATH` | load this job at start; its `meta.config_variant` selects the config variant (as `tools/run_job.py`) |
| `--variant NAME` | preselect the variant for "Build from config" (`config/variants/NAME.toml`) |
| `--config PATH` | another `station.toml` |
| `--ares` | create the ADS worker (heartbeat, MANUAL, HALT, jog, GO): it connects to `[ares_ads] host_ip` at once. Without it the HMI never opens an ADS connection; tests and SIM runs never use it |
| `--twin` | switch the RoboDK twin on: it starts with the first job in an own RoboDK instance (API port from `[hmi.twin] port`, >= 20630) |

The HMI never connects to hardware on its own: ADS only with `--ares`; the UR (RTDE / URScript / Dashboard), the
sequencer's own ARES connection and the IDS camera only when **Prepare / Connect** is pressed in REAL mode.

## Tabs

| Tab | Content |
|---|---|
| Mauer | job (list of `data/jobs/*.json`, Build from config, Browse), SIM / REAL options, Prepare / Connect, Start, Pause, Resume, Abort, Release, progress (stop, stone, action, station trip, ARES pose estimate, jaws), recovery, preflight list, plan view of the wall |
| Camera | last / live image of the run's camera with the detected ChArUco corners (green accepted, orange rejected, red missing), board table (corners, RMS px, reason), last frame fit (RMS / max mm, error vs nominal, jump) and a REJECTED banner; **Grab** / **Live** only in REAL with a prepared rig and no run; **Snapshot** saves view + full image + JSON to `<run log>/snapshots/` |
| UR | URLink state (REAL: RTDE sample age, controller version, robot / safety / runtime mode, power, program running; SIM: the simulated arm): joints, TCP, gripper outputs (pulses, not a held state), commanded payload, held stone, parked, last robot action |
| ARES control | amr_hmi Control tab: startup sequence, jog (W/A/S/D/Q/E), relative move, odometry map |
| Wall pose | ARES pose estimate in the wall frame (plan view) next to the PLC odometry, the live pose during an ARES move (REAL), SIM truth and the estimate error, target / route leg / dock, last ARES move, the camera fits per stop |
| Dashboard, Diagnostics, Battery | amr_hmi tabs |
| Run log | every record of the current run log, coloured by severity, filtered by category; Open log folder |
| Twin | RoboDK digital twin on/off, Restart, Save view PNG; state, API port, rate, render time, lag, stones on screen |

Above the tabs: HALT, the step-mode confirmation bar (visible on every tab) and the status strip (run state, mode,
stop, stone, action, UR mode / safety, ARES PLC state / move, twin state). The Mauer tab shows the plan view with the
run feedback board below it (stones, stop / leg / course, station trips and refills, ARES moves and corrections,
camera fits, time per stone, preflight, warnings / errors; SIM placement statistics at the end; Open summary).

## Starting a run

- **SIM**: the motion guard plans every joint move as on the real robot (option "SIM motion guard", on by default;
  off = the simulated arm moves straight, faster).
- **REAL Start**: the preflight runs again right before the first motion; then the bar above the tabs shows "REAL
  start - checklist" with the magazine fill (slot: stone type; the job's initial fill, or for a later start the
  fill for the stones after the standing ones, `mauer.job.restart_fill`) and waits for **Checked - start** (gripper
  jaws empty, magazine as listed and every other slot empty, pick-up station full). **Not ready** starts nothing.
- **Start at stop k > 0** (as `tools/run_job.py --resume-log / --stop-untouched`): "earlier run: Run log..." takes the
  run log folder of the interrupted run (its placed stones, and those it took over itself, stand: skipped and part
  of the motion guard's wall), or "start stop untouched" confirms that no stone of stop k is placed yet (stops < k
  count as built). REAL refuses the start without one of them.

## Magazine dry run (magtest)

A job built by `tools/make_magtest.py` (meta `kind` "magtest", `docs/MAGTEST_DE.md`) runs with
`mauer.magtest.MagazineTest` instead of the Sequencer: one stop, ARES at the origin, one "stone" per move (pick slot,
front pose, put-down slot). REAL opens only the UR (`RealRig.ur_only`: no AresAds, camera or calibration) and is
selectable without `--ares`; the preflight is `preflight.magtest_preflight` (job / config + the UR block; `[ur]
payload_cog_mm` PLACEHOLDER shown, not blocking); the start checklist asks for the two stones on the start slots,
empty jaws, a free front area and the pendant check of `[ur5] mount_rz` (Base +X = ARES left); Resume does not check
the ARES odometry. A robot error in a move: take the stone out, put it back on the move's pick slot, "Jaws empty",
Resume (the move runs again). The plan view shows ARES and the front poses (moves done = placed), no station table.

## RoboDK digital twin

The Twin tab (or `--twin`) starts an OWN RoboDK (`rdk_common.connect(new_instance=True)`) on the first free API
port of `[hmi.twin] port .. port + port_tries - 1` (>= 20630; never the user's RoboDK on 20500 / 20501, and a port
held by a RoboDK it did not start is skipped untouched), builds the station there (`build_station.build`, ~4 s, no
camera window, collisions off) and mirrors the run at `[hmi.twin] rate_hz`: UR joints, ARES frame, pick-up station
frame and the stones (magazine, station, wall, jaws). It is passive (thread `mauer-twin`, it reads snapshots only;
the run never waits for it). SIM: ARES and the station at the simulated true pose. REAL: twin = sequencer belief
(UR joints from RTDE, ARES at the estimate + the HMI's odometry during a move, stones at their nominal wall pose).
A new job restarts it (job and config variant stay consistent); closing the HMI closes its RoboDK. If RoboDK is
closed by hand the twin goes "lost"; **Restart** builds it again.

## A run

1. Load or build a job. A job always comes with its own config variant (window title "config ...").
2. Choose SIM (scenario, seed) or REAL (needs `--ares`), the stops, camera loop, step mode.
3. **Prepare / Connect**: SIM builds the simulated world; REAL opens URLink, the sequencer's AresAds, the camera and
   the calibration (in this order, as `tools/run_job.py --real`). The preflight list follows: SIM = informative,
   REAL = every item blocks Start, there is no override.
4. **Start**. Step mode asks before every motion in the bar above the tabs; **Go** is enabled 0.5 s after the
   request appears ([hmi] confirm_arm_s), **Decline** ends the run as "aborted" (resumable). The pick-up station
   refill (REAL) is asked the same way (**Refilled** / **Abort run**).
5. **Release** closes the Sequencer and the rig. Run logs: `data/runs/<stamp>_hmi_<sim|real>/run.jsonl`,
   `hmi_summary.json` (and `images/` with "save images").

**Pause** and **Abort** act at the next motion boundary with empty jaws: the place of a held stone still runs
(and is still confirmed in step mode), so no stone is left in the jaws by a pause. **Decline** acts at once, also
with a stone held.

**Resume** is refused (with the reason in the status bar) while
- a stone may be in the jaws (after a decline / HALT / robot error between pick and place): take it out or put it
  down by hand, park the arm, then **Jaws empty**;
- the ARES pose is not ok (an ARES move that ended not ok / an ARES error): **Confirm pose** (odometry estimate
  checked on the floor) or **Set pose** (x, y, theta in the wall frame);
- REAL: the PLC odometry moved since the run stopped (ARES jogged while paused): **Apply odometry** (the estimate
  follows the odometry) or **Set pose** (also takes the odometry of that moment as the new reference); or the
  HMI's ADS worker is not connected (it cannot verify that);
- REAL: the preflight has a blocking item (**Re-check** runs it again).

Start and Resume clear HALT and the abort request at once (GUI thread), before their checks run. A HALT, Pause or
Abort pressed while the checks run (REAL: about 0.3-1 s) therefore stops the run before its first motion. REAL
**Start** runs the preflight again right before the first motion (the one of Prepare may be old) and waits while
the HMI's ADS status shows ARES moving (relative move or jog); **Resume** waits for that too. **Confirm pose** accepts
an odometry estimate only; an unknown pose needs **Set pose**. Stops are numbered from 0 everywhere (as in the run
log and the Stops boxes), the legs of a route from 1.

After an error or a HALT the resume runs in step mode (the operator may switch it off). The run state lives in
memory only: closing the HMI loses the resume ability (a new run of stop k assumes ARES at its start mark).

## HALT

HALT (button, Space, Esc - on every tab, also while an HMI child window such as the file dialog is active):

| | ARES | UR | Run |
|---|---|---|---|
| HALT | ADS worker: ONE write `bCmdMoveAbort` TRUE + all jog bits FALSE, reset after 300 ms (amr_hmi, first); the sequencer's AresAds latched (no start edge until Start / Resume); its abort pulse only if the worker is not connected | REAL: URLink latched (no robot program but the abort until Start / Resume), `URLink.abort()` (stopl program + Dashboard stop) in a helper thread | pending confirmation released with "no", "halt" (and REAL "halt_result") in the run log, run ends "error" (a motion was stopped) / "aborted" (nothing was sent), step mode on for the resume |
| Pause / Abort | - | - | at the next motion boundary with empty jaws |
| E-stop | hardware | hardware | the sequencer sees the failure ("error") |

HALT is an operating function, not a safety function: the E-stops on ARES and the UR are the safety function.
Space / Esc reach the HMI only while it (or one of its dialogs) is the active window. With a REAL rig open, a red
banner above the tabs says when another window is active; while a run is active the buttons that open other windows
(Open log folder, Open snapshot folder, Open summary, twin Restart / Save view / switching the twin on) are disabled.

A robot block whose RTDE stream stays silent for 5 s (`mauer/ur/link.py` RTDE_LOST_S) is aborted and fails, instead
of waiting for the block timeout (180 s); a block running when HALT latches gives up within 3 s (HALT_WAIT_S).

Closing the HMI is refused while a run is active or a relative move of the ARES control tab runs. Closing stops the
ADS worker first (jog bits FALSE, heartbeat ends: the PLC aborts a move after 500 ms and drops MANUAL after 2 s), then
the twin (its RoboDK closes) and the run controller.

## Operating pattern A in one process

Two ADS connections, as proven in E003 (there with two processes): connection #1 is the HMI's AdsWorker (heartbeat
every 100 ms, MANUAL, HALT, jog, GO, odometry reset); connection #2 is the sequencer's `mauer.ares.AresAds`, which
writes only `fMove*`, `nMoveCmdId`, `bCmdMoveStart` and `bCmdMoveAbort`. Tested against `tests/fake_plc.py`
(`tests/test_hmi_pattern_a.py`, also the heartbeat gaps while a SIM run loads the CPU). **Not yet checked on the
robot:** before the first wall run, with ARES jacked up: HMI with `--ares`, MANUAL, a REAL step-mode move of
100 mm, then HALT.

During a REAL run jog and GO are locked (banner on the ARES control tab); while a REAL run is paused they are
allowed (repositioning - the resume check sees the movement); "Reset pose" of the odometry map is locked while a
REAL run is loaded.

## Known limits (2026-10-08)

- PolyScope 3.3.3 on the lab UR5 has no `get_inverse_kin_has_solution`: every look / pick / place block fails until
  `mauer/ur/script.py` ik_guard has a 3.3 variant. The REAL preflight shows it as a blocker.
- Nominal jobs (`tools/make_job.py`) never pass the real-run preflight (reach check "kinematic", no collision /
  occlusion check). The RoboDK-verified jobs (`data/jobs/nominal_C[_c_acb]_robodk.json`, written by a clean
  `robodk/simulate.py` run) pass that item.
- The `[hmi]` sections changed `config/station.toml` (`config_sha256` hashes the whole file), so every job built
  before them reports "config changed since the job was built": rebuild the nominal jobs (`py.exe tools/make_job.py`,
  `py.exe tools/make_job.py --variant c_acb`) and re-run `robodk/simulate.py` for the RoboDK-verified ones.
- The C has 12 half stones and `[half_brick]` is still PLACEHOLDER / UNKNOWN (mass, dimensions): the real-run
  preflight blocks on it (2026-10-08).
- In REAL the plan view (and the twin) show the sequencer's belief: the nominal wall and the estimated ARES pose.
- A REAL start shows neither the magazine fill the run expects nor asks "jaws empty / magazine as listed / station
  full" (tools/run_job.py --real does since main 22340be); a start at stop k > 0 has no counterpart of run_job's
  `--resume-log` / `--stop-untouched` yet (to be added with the merge into main: docs/TESTPLAN_REALTEST_ARES_DE.md
  S1, Anhang C1 / C2).

## Configuration

`config/station.toml` `[hmi]` (refresh rates, SIM pacing, confirmation delay, resume odometry tolerance, run-log
folder), `[hmi.frame]` (direction mapping, amr config.yaml `frame`), `[hmi.move]` (relative-move presets),
`[hmi.twin]` (RoboDK twin). The ADS target and the move defaults come from `[ares_ads]`, the PLC limits from
`mauer/ares/ads.py` (`hmi/core/config.py` maps them to the amr config dict; there is no config.yaml).

## Code

| Path | Content |
|---|---|
| `hmi/amr/` | amr_hmi v2 (MA `10_robot/hmi/amr_hmi`, commit 5935c5b, copied 2026-10-07): ADS worker, PLC symbols, logic, direction mapping, the amr widgets. Changes: relative imports, the run-lock hooks (list in `hmi/__init__.py`) |
| `hmi/core/` | config mapping, ADS worker setup (`ads_link`), `HmiContext`, job sessions, rigs (SIM / REAL), preflight, run snapshots, read-only UR / ARES sources, `RunController` (run thread `mauer-run`), camera overlay (`overlay`: detection + drawing in the run thread), twin link (`twin_link`: frames for the twin) |
| `hmi/views/` | Mauer tab, plan view, confirmation bar, run log; camera view, UR panel (`UrPoller`: one read-only UR source shared with the status strip), wall pose, run feedback, status strip, twin panel |
| `robodk/twin.py`, `robodk/twin_model.py` | the twin's RoboDK side (station, stone objects, thread loop) and its pure stone-state model (`plan_ops`, `TwinSettings`) |
| `hmi/main_window.py`, `hmi/main.py` | window (adapted amr `ui/main_window.py`), entry point |

Threads: GUI; `ads-worker` (amr AdsWorker, QThread); `mauer-run` (everything that blocks: job load / build, rig
open / close, preflight, `Sequencer.run()`, recovery, camera grabs and the camera overlay); `mauer-halt` (UR abort
after HALT); `mauer-twin` (RoboDK); `camera-snapshot` (Snapshot file writes). Widgets connect to the controller's
signals (queued into the GUI thread) and never call hardware objects.

## Tests

```
py.exe -m pytest -q tests -k "hmi or twin_model or sequencer_hooks"   # the HMI tests (~1 min, offscreen Qt, fakes only)
py.exe -m pytest -q tests                                             # the whole suite
py.exe -m pytest -m robodk tests/test_twin_robodk.py -o addopts="" -q  # opt-in: the twin in an own RoboDK (20630+)
```

Every HMI test runs with the `no_lab_network` fixture (only loopback connections, pyads / ids_peak not importable).

## Provenance

`hmi/amr/` and the adapted `hmi/main_window.py` / `hmi/main.py` / `tests/test_hmi_{logic,frame,ads_worker,
worker_thread,gui_smoke}.py` / `tests/hmi_fakes.py` come from the Masterarbeit repo, `10_robot/hmi/amr_hmi`
(commit 5935c5b, 2026-09-28), copied 2026-10-07. The MA repo is not modified; `hmi/amr/README.md` and `SETUP.md` are
the MA documents with a provenance note.
