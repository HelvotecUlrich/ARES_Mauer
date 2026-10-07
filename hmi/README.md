# Mauer HMI (`hmi/`)

The ARES HMI v2 (amr_hmi, copied from the MA repo) extended for the brick-laying runs of this repo: load a job,
run it in the pure-Python simulation (SIM) or on ARES + UR5 + IDS camera (REAL), confirm every motion in step mode,
pause / abort / HALT, recover a stopped run, and keep all amr_hmi functions (connection, startup sequence, jog,
relative move, odometry map, dashboard, diagnostics, battery). Design and contract: `docs/HMI_DESIGN.md`.

## Start

```
py.exe -m hmi                                      # SIM only - no ADS connection ("ADS off"), REAL disabled
py.exe -m hmi --job data/jobs/nominal_C.json       # load a job at start (with the config variant it was built with)
py.exe -m hmi --ares                               # the HMI's ADS worker connects to the ARES PLC ([ares_ads])
```

| Option | Meaning |
|---|---|
| `--job PATH` | load this job at start; its `meta.config_variant` selects the config variant (as `tools/run_job.py`) |
| `--variant NAME` | preselect the variant for "Build from config" (`config/variants/NAME.toml`) |
| `--config PATH` | another `station.toml` |
| `--ares` | create the ADS worker (heartbeat, MANUAL, HALT, jog, GO): it connects to `[ares_ads] host_ip` at once. Without it the HMI never opens an ADS connection; tests and SIM runs never use it |
| `--twin` | start the RoboDK twin with the first job (TWIN step) |

The HMI never connects to hardware on its own: ADS only with `--ares`; the UR (RTDE / URScript / Dashboard), the
sequencer's own ARES connection and the IDS camera only when **Prepare / Connect** is pressed in REAL mode.

## Tabs

| Tab | Content |
|---|---|
| Mauer | job (list of `data/jobs/*.json`, Build from config, Browse), SIM / REAL options, Prepare / Connect, Start, Pause, Resume, Abort, Release, progress (stop, stone, action, station trip, ARES pose estimate, jaws), recovery, preflight list, plan view of the wall |
| Camera | the run's camera images with the ChArUco detections (CAMERA step; stub in CORE) |
| UR | UR state: modes, joints, TCP, gripper outputs, payload (STATUS step; stub in CORE) |
| ARES control | amr_hmi Control tab: startup sequence, jog (W/A/S/D/Q/E), relative move, odometry map |
| Wall pose | ARES pose in the wall frame, camera fits per stop (STATUS step; stub in CORE) |
| Dashboard, Diagnostics, Battery | amr_hmi tabs |
| Run log | every record of the current run log, coloured by severity, filtered by category; Open log folder |
| Twin | RoboDK digital twin (TWIN step; stub in CORE) |

Above the tabs: HALT, the step-mode confirmation bar (visible on every tab) and the status strip.

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

After an error or a HALT the resume runs in step mode (the operator may switch it off). The run state lives in
memory only: closing the HMI loses the resume ability (a new run of stop k assumes ARES at its start mark).

## HALT

HALT (button, Space, Esc - on every tab, also while an HMI child window such as the file dialog is active):

| | ARES | UR | Run |
|---|---|---|---|
| HALT | ADS worker: ONE write `bCmdMoveAbort` TRUE + all jog bits FALSE, reset after 300 ms (amr_hmi, first); the sequencer's AresAds abort pulse only if the worker is not connected | REAL: `URLink.abort()` (stopl program + Dashboard stop) in a helper thread | pending confirmation released with "no", run ends "error" / "aborted", step mode on for the resume |
| Pause / Abort | - | - | at the next motion boundary with empty jaws |
| E-stop | hardware | hardware | the sequencer sees the failure ("error") |

HALT is an operating function, not a safety function: the E-stops on ARES and the UR are the safety function.
Closing the HMI is refused while a run is active; closing stops the heartbeat (the PLC aborts a running move after
500 ms and drops MANUAL after 2 s).

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

## Known limits (2026-10-07)

- PolyScope 3.3.3 on the lab UR5 has no `get_inverse_kin_has_solution`: every look / pick / place block fails until
  `mauer/ur/script.py` ik_guard has a 3.3 variant. The REAL preflight shows it as a blocker.
- Nominal jobs (`tools/make_job.py`) never pass the real-run preflight (reach check "kinematic", no collision /
  occlusion check): REAL Start stays disabled until a RoboDK-checked job exists.
- The `[hmi]` sections changed `config/station.toml`, so jobs built before them report "config changed since the
  job was built": rebuild them (`py.exe tools/make_job.py`, `py.exe tools/make_job.py --variant c_acb`).
- In REAL the plan view (and the twin) show the sequencer's belief: the nominal wall and the estimated ARES pose.

## Configuration

`config/station.toml` `[hmi]` (refresh rates, SIM pacing, confirmation delay, resume odometry tolerance, run-log
folder), `[hmi.frame]` (direction mapping, amr config.yaml `frame`), `[hmi.move]` (relative-move presets),
`[hmi.twin]` (RoboDK twin). The ADS target and the move defaults come from `[ares_ads]`, the PLC limits from
`mauer/ares/ads.py` (`hmi/core/config.py` maps them to the amr config dict; there is no config.yaml).

## Code

| Path | Content |
|---|---|
| `hmi/amr/` | amr_hmi v2 (MA `10_robot/hmi/amr_hmi`, commit 5935c5b, copied 2026-10-07): ADS worker, PLC symbols, logic, direction mapping, the amr widgets. Changes: relative imports, the run-lock hooks (list in `hmi/__init__.py`) |
| `hmi/core/` | config mapping, ADS worker setup (`ads_link`), `HmiContext`, job sessions, rigs (SIM / REAL), preflight, run snapshots, read-only UR / ARES sources, `RunController` (run thread `mauer-run`) |
| `hmi/views/` | Mauer tab, plan view, confirmation bar, run log; camera / UR / wall pose / feedback / status strip / twin widgets |
| `hmi/main_window.py`, `hmi/main.py` | window (adapted amr `ui/main_window.py`), entry point |

Threads: GUI; `ads-worker` (amr AdsWorker, QThread); `mauer-run` (everything that blocks: job load / build, rig
open / close, preflight, `Sequencer.run()`, recovery, camera grabs); `mauer-halt` (UR abort after HALT);
`mauer-twin` (RoboDK). Widgets connect to the controller's signals (queued into the GUI thread) and never call
hardware objects.

## Tests

```
py.exe -m pytest -q tests -k "hmi or sequencer_hooks"      # the HMI tests (~40 s, offscreen Qt, fakes only)
py.exe -m pytest -q tests                                  # the whole suite
```

Every HMI test runs with the `no_lab_network` fixture (only loopback connections, pyads / ids_peak not importable).

## Provenance

`hmi/amr/` and the adapted `hmi/main_window.py` / `hmi/main.py` / `tests/test_hmi_{logic,frame,ads_worker,
worker_thread,gui_smoke}.py` / `tests/hmi_fakes.py` come from the Masterarbeit repo, `10_robot/hmi/amr_hmi`
(commit 5935c5b, 2026-09-28), copied 2026-10-07. The MA repo is not modified; `hmi/amr/README.md` and `SETUP.md` are
the MA documents with a provenance note.
