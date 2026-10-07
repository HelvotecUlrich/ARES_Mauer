# Mauer HMI and RoboDK digital twin: design (2026-10-07)

This design is the contract for the implementers of plan tracks B (Mauer HMI) and C (RoboDK twin) in `docs/PLAN_2026-10-07_realtest.md`.

**Sources**
- Base: `main` at `adb69cc`. Line numbers refer to `adb69cc` unless a line says otherwise.
- `amr_hmi` = `/mnt/c/Users/samue/Masterarbeit/10_robot/hmi/amr_hmi`. It is READ ONLY. Last commit touching it: MA `5935c5b` (2026-09-28).

**Parallel work in the main tree.** The main tree is changing alongside this work.
- Its uncommitted `tools/run_job.py` now refuses a job whose `config_variant` differs from the loaded config.
- The HMI enforces the same rule in its own code (section 8.1), so it does not depend on that change.

**Implementation notes** (what the CORE step changed against this contract, and why) are collected in section 15.

---

## 0. Decisions

| # | Decision |
|---|---|
| D-H1 | **Copy amr_hmi v2.** amr_hmi v2 is copied into the package `hmi/`. The copy lives in `hmi/amr/`; its logic is unchanged and its imports become relative. The MA repo is not touched. Entry point: `py.exe -m hmi`. |
| D-H2 | **ADS uses operating pattern A**, the only pattern proven on the robot (E003). Connection #1 belongs to the HMI's `AdsWorker`: heartbeat, MANUAL, HALT, jog, GO and odometry reset. Connection #2 belongs to the sequencer's `AresAds` and writes only `fMove*`, `nMoveCmdId`, `bCmdMoveStart` and `bCmdMoveAbort`. This is the deliberate exception to plan item B6 ("no second connections"); the plan's own status note says "ARES moves keep operating pattern A (two ADS connections in one program)". What is new: both connections live in ONE process. This is tested against `tests/fake_plc.py` and must be checked on the jacked-up robot (section 14). |
| D-H3 | **One `URLink`, shared.** The run thread runs the URScript blocks. The UR panel and the twin only read `link.state()`, which is lock-protected (`mauer/ur/link.py:413-442`). There is no second RTDE client; whether one would work on PolyScope 3.3.3 is not verified, and it is not needed. |
| D-H4 | **One camera object, used only by the run thread.** That covers sequencer shots and manual grabs while no run is active. `IdsCamera.grab` has no lock (`ids.py:662-704`). The camera view gets images through a frame tap (`Sequencer(on_shot=...)`). |
| D-H5 | **The HMI never connects to hardware on its own.** ADS is used only with `--ares`. UR, camera and `AresAds` are opened only by **Connect** in REAL mode. `MainWindow` never creates an `AdsWorker` itself (amr did, `main_window.py:45-51`). As a result, neither tests nor SIM can auto-connect. |
| D-H6 | **Step-mode confirmations go in an inline `ConfirmBar`** above the tabs, visible on every tab, not in a modal dialog. A modal dialog would block the HALT button. It would also disable the Space/Esc HALT keys, because the amr event filter only acts while the main window is active (`main_window.py:133-134`), and Space could click its "Go". There is **no modal `QMessageBox` anywhere** in the HMI. A file dialog uses `QFileDialog.DontUseNativeDialog` and is only reachable when no run and no ARES move is active. |
| D-H7 | **Pause and soft Abort wait until no stone is held.** They take effect at the next motion boundary where the jaws are empty (sequencer change, section 7). Today a pause between pick and place loses the stone; this is confirmed in the sim. HALT is immediate. |
| D-H8 | **HALT has three steps, in this order:** (1) `AdsWorker.halt()`, first and unchanged; (2) the run's abort flag is set and a pending confirm is released with `False`; (3) in REAL mode, a helper thread calls `URLink.abort()`, and calls `AresAds.abort()` only as a fallback when the `AdsWorker` is not connected. HALT never blocks the GUI thread. |
| D-H9 | **The twin is passive.** It runs in its own thread with its own RoboDK instance on a port >= 20630, with collisions off and no Cam2D. It polls immutable snapshots and never calls into the sequencer; the run never waits for it. It is driven by state (a diff of stone states), not by inferring state from events. |
| D-H10 | **HMI settings go into `config/station.toml`** sections `[hmi]`, `[hmi.frame]`, `[hmi.move]` and `[hmi.twin]`. The ADS target and the move defaults reuse `[ares_ads]`. The PLC limits come from the constants in `mauer/ares/ads.py`. |
| D-H11 | **A job always loads its own config variant** (`job.meta["config_variant"]`), as `tools/run_job.py` does. The variant appears in the window title and the Mauer tab, and is passed on to `preflight_real` (`cfg["_variant"]`). |

## 1. Scope

**In scope**
- Everything amr_hmi v2 does.
- **Mauer tab:** job loading, plan view, SIM/REAL run, progress, step confirmations, pause/resume/abort, preflight list, recovery actions.
- Run-log view, camera view, UR panel, wall-pose (ARES) panel, run feedback, status strip.
- RoboDK twin toggle.

**Out of scope (later)**
- Replaying an old `run.jsonl` in the twin.
- Driving the ARES tabs from a fake PLC in SIM.
- Manual UR actions from the HMI (power on, unlock protective stop, park).
- Restoring a run after an HMI crash; the sequencer state lives in memory only.
- Running the REAL sequencer as a subprocess; this is a fallback only (section 14).
- The German test plan (track D).

## 2. Hard rules for every implementer

**No real hardware**
- Never talk to real hardware: no ADS to 192.168.1.10, no RTDE or socket to 192.168.56.101, no IDS camera open, no robot motion.
- Tests and smoke runs use fakes and `mauer.simworld`.
- Every HMI test file starts with `pytestmark = pytest.mark.usefixtures("no_lab_network")` (section 12.1).

**RoboDK**
- Only a NEW instance, via `rdk_common.connect(new_instance=True, port>=20630)`, closed again with `rdk_common.close_instance`.
- Never use ports 20500/20501.
- Never open a Cam2D (`SimCamera`, `timelapse.Recorder`, `rdk_common.snapshot`).
- Never pass `-SKIPINI` or `-HIDDEN`.
- Never set `_SkipStatus`.

**Python and Qt**
- Use Windows Python: `py.exe -m pytest -q tests`, run from the worktree.
- Qt tests rely on `os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")` in `tests/conftest.py`, because WSL environment variables do not cross over.

**Repo conventions** (`CLAUDE.md`)
- Every parameter goes into `station.toml` with a status tag and a source.
- Units appear in names.
- English only.
- Small thematic commits.
- Comment density as in the surrounding code.

**Worktrees**
- Work only in `/mnt/c/Users/samue/ARES_Mauer_wt/<branch>`, created with `git -C /mnt/c/Users/samue/ARES_Mauer worktree add /mnt/c/Users/samue/ARES_Mauer_wt/<branch> -b <branch> <start>`.
- Never edit, check out or commit in `/mnt/c/Users/samue/ARES_Mauer`. Never push.
- Every commit message ends with the two attribution lines of the session (`Co-Authored-By: ...`, `Claude-Session: ...`).

**Git-ignored assets** (copy them, never commit them)
- `cp -p` keeps the mtimes, so the STL caches stay newer than their STEP files and `build()` does not start a slow STEP import:
  ```
  cp -p /mnt/c/Users/samue/ARES_Mauer/cad/*.{stl,stp,step,tool} <wt>/cad/
  mkdir -p <wt>/robodk/library && cp -p /mnt/c/Users/samue/ARES_Mauer/robodk/library/UR5.robot <wt>/robodk/library/
  mkdir -p <wt>/data/jobs && cp -p /mnt/c/Users/samue/ARES_Mauer/data/jobs/*.json <wt>/data/jobs/   # manual GUI use only
  ```

**Do not touch amr_hmi's hardware tools**
- Never import or run amr_hmi's `list_symbols.py` (it connects at import), `ads_diag.py` or `ads_portscan.py`. They are not copied.

---

## 3. Package layout

```
hmi/
  __init__.py          provenance docstring: copied 2026-10-07 from MA 10_robot/hmi/amr_hmi (commit 5935c5b);
                       list of the adaptations (section 4)
  __main__.py          `py.exe -m hmi` -> main.main()
  main.py              CLI, dark theme (amr main.py apply_dark_theme unchanged), builds ctx / worker / window
  main_window.py       Mauer MainWindow (adapted amr ui/main_window.py, section 10.1)
  README.md            usage, operating notes, provenance (CORE step C8; INTEGRATE extends)
  amr/                 the amr_hmi copy
    __init__.py
    README.md, SETUP.md        copied, 3-line provenance header ("config now in config/station.toml [hmi]")
    ads_worker.py plc_vars.py logic.py frame.py
    ui/__init__.py constants.py widgets.py jog_panel.py move_panel.py odom_map.py control_widget.py
       dashboard_widget.py diagnostics_widget.py battery_widget.py
  core/                services, no widgets (QObject allowed)
    __init__.py
    config.py          station.toml -> amr config dict + checks                       CORE
    ads_link.py        probing pyads factory, start_ads_worker(), NullAdsWorker       CORE
    context.py         HmiContext                                                     CORE
    session.py         JobSession, load_job_file, build_from_config, list_*           CORE
    rigs.py            RigInfo, SimRig, RealRig, RealFactories                         CORE
    preflight.py       PreflightItem, PreflightReport, sim_preflight, real_preflight   CORE
    snapshot.py        RunSnapshot, StoneInfo, ShotView, describe_action, format_event CORE
    sources.py         UrSnapshot, UrSource, AresLive, ares_live                       CORE
    run_controller.py  RunController, RunWorker, RunOptions, ConfirmRequest, enables  CORE
    overlay.py         ShotContext, make_shot_view                                     CAMERA
    twin_link.py       TwinLink, frame_from                                            TWIN
  views/
    __init__.py
    mauer_tab.py plan_view.py confirm_bar.py log_view.py                               CORE
    camera_view.py                                    stub by CORE, replaced by CAMERA
    ur_panel.py ares_panel.py feedback.py status_strip.py   stubs by CORE, replaced by STATUS
    twin_panel.py                                     stub by CORE, replaced by TWIN
robodk/
  twin_model.py        pure (no RoboDK import): StoneState, TwinFrame, TwinSettings, plan_ops    TWIN
  twin.py              RoboDK side: TwinScene, Twin                                             TWIN
tests/
  conftest.py          + Qt offscreen env, qapp fixture, no_lab_network fixture                 CORE
  hmi_fakes.py         amr fakes + Mauer fakes (section 12.1)                                    CORE
  test_hmi_*.py, test_sequencer_hooks.py (CORE); test_hmi_camera.py (CAMERA);
  test_hmi_status.py (STATUS); test_twin_model.py, test_twin_robodk.py (TWIN)
hmi.cmd                `py -m hmi %*` launcher like sim.cmd                                     INTEGRATE
```

**Not copied:** `.venv/`, `__pycache__/`, `.pytest_cache/`, `requirements*.txt`, `config.yaml`, `list_symbols.py`, `ads_diag.py`, `ads_portscan.py`. amr's `ui/main_window.py` is not copied as such; it becomes `hmi/main_window.py`.

**Import rules**
- `hmi` imports `mauer` with absolute imports. `mauer` never imports `hmi`.
- No module under `hmi/` imports any of the following at module level: `pyads`, `ids_peak`, `robodk`, `robolink`, `rdk_common`, `twin`. A test enforces this (section 12).
  - `pyads` is imported only inside `core/ads_link.py` factories and `amr/ads_worker.py`'s default factory.
  - `mauer.ur.link`, `mauer.ares.ads.AresAds`, `mauer.camera.open_camera` and `mauer.vision.{intrinsics,handeye}` are imported inside `RealFactories` defaults.
  - `robodk/twin.py` is imported inside `TwinLink.start()` after `sys.path.insert(0, str(REPO / "robodk"))`.
- `requirements.txt` gets `PySide6>=6.5,<7` (amr requirement; 6.11 is installed). PyYAML is not needed.

---

## 4. The amr_hmi copy: unchanged vs adapted

| amr file | Target | Change |
|---|---|---|
| `ads_worker.py` | `hmi/amr/ads_worker.py` | Imports only: `from . import plc_vars as pv`, `from .logic import halt_values`. The factory is injected by `core/ads_link.py`. |
| `plc_vars.py`, `frame.py` | `hmi/amr/` | None. |
| `logic.py` | `hmi/amr/logic.py` | Imports (`from .plc_vars import ...`, `from .ui import constants as C`). Two new arguments, sketched below. With their defaults, amr behaviour is unchanged. |
| `ui/constants.py`, `widgets.py`, `jog_panel.py`, `dashboard_widget.py`, `diagnostics_widget.py`, `battery_widget.py` | `hmi/amr/ui/` | Imports only (`from .. import frame as F`, `from ..logic import ...`, `from . import constants as C`). `ROBOT_LENGTH_M` / `ROBOT_WIDTH_M` (1.12 / 0.60 m) stay; a test pins them to `[ares] length` / `width`. |
| `ui/move_panel.py` | `hmi/amr/ui/` | Imports. New `set_run_lock(reason: str) -> None`: stores the reason, calls `_disarm()`, then `_refresh()`. `go_state()` passes `run_lock=self._run_lock` to `go_enabled`. |
| `ui/odom_map.py` | `hmi/amr/ui/` | Imports. New `OdomPanel.set_reset_lock(reason: str) -> None`. `reset_allowed` becomes `self._available and not self._move_active and not self._reset_lock`. The hint shows the reason. |
| `ui/control_widget.py` | `hmi/amr/ui/` | Imports. Three changes, listed below. |
| `ui/main_window.py` | `hmi/main_window.py` | Adapted (section 10.1). |
| `main.py` | `hmi/main.py` | Adapted: argparse (section 10.1), no YAML; `apply_dark_theme` verbatim. |
| `tests/conftest.py` | `tests/conftest.py` | Its Qt env line and `qapp` fixture are merged into the existing conftest. |
| `tests/fakes.py` | `tests/hmi_fakes.py` | Plus the Mauer fakes. |
| `tests/test_*.py` | `tests/test_hmi_{logic,frame,ads_worker,worker_thread,gui_smoke}.py` | Imports. The config comes from `station.toml` via `core.config.amr_cfg`. `test_frame` asserts `[hmi.frame]`. `gui_smoke` uses the new constructor and tab list. The lazy-import guard is extended to `hmi/**/*.py` (section 12). |

**`logic.py`: the two new arguments**
```python
jog_enabled(status, connected, run_lock: str = "")          # False when run_lock != ""
go_enabled(status, connected, if_version, param_error=None, awaiting_ack=False, run_lock: str = "")
# returns (False, run_lock) directly after the "not connected" check
```

**`ui/control_widget.py`: the three changes**
- New `set_run_lock(reason: str)`. It stores the reason, calls `release_jog()` when locking, calls `move.set_run_lock(reason)`, and shows a header banner "Mauer REAL run active - jog / GO locked" in the existing banner style.
- New `set_reset_lock(reason: str)`, which calls `odom.set_reset_lock`.
- `_apply_enables()` passes `run_lock` to `jog_enabled`.

**Behaviour that must not change**
- `AdsWorker` remains the only owner of its connection: a queue, ONE `write_list_by_name` per command including `nCommandId`.
- Safe defaults are written before the first heartbeat (`plc_vars.py:187-193`, unchanged).
- Commands from an older epoch are dropped on reconnect.
- HALT is priority 0 and writes `bCmdMoveAbort` plus all 6 jog bits FALSE, NOT `bCmdStop`. The pulse-generation reset after 300 ms stays.
- The legacy interface only on ADS error 1808; `InterfaceMismatch` without a reduced v2 mode.
- The heartbeat runs in the worker thread.
- Space/Esc means HALT, and every button has `NoFocus`.
- Jog is released on focus loss, tab change, disconnect and MANUAL exit, and always writes all 6 bits.
- GO needs a double click within 3 s and disarms on any change.
- Jog parameters are deferred, and Reset pose is locked, while `bMoveActive`.
- The frame flag pair stays.
- Standby stays separate from HALT.
- `pyads` is imported lazily.

---

## 5. Configuration

### 5.1 New sections in `config/station.toml`

**Append them at the END of the file.** This minimises merge conflicts with the main tree.

```toml
[hmi]
# Mauer HMI (hmi/, copy of MA 10_robot/hmi/amr_hmi v2, commit 5935c5b, 2026-10-07). ADS target, timeout and the
# relative-move defaults are NOT repeated here: hmi/core/config.py takes them from [ares_ads] (ams_net_id, host_ip,
# port, timeout_ms, speed_mms, rot_speed_degs, accel_mms2), the PLC limits from the mauer/ares/ads.py constants.
poll_interval_ms = 100          # CONFIRMED: amr_hmi value (status sum read)
heartbeat_interval_ms = 100     # CONFIRMED: amr_hmi value; must stay < 250 (AresAds HB_WINDOW_S, pattern A)
reconnect_interval_s = 5.0      # CONFIRMED: amr_hmi value
jog_speed_mms = 200.0           # CONFIRMED: amr_hmi value
jog_rot_speed_degs = 20.0       # CONFIRMED: amr_hmi value
default_speed_limit_mms = 500.0 # CONFIRMED: amr_hmi value (fSpeedLimit_mms)
jog_accel_mms2 = 1000.0         # CONFIRMED: amr_hmi value; written as fAccel_mms on MANUAL entry, also caps the move
                                # acceleration in the PLC (mauer/ares/ads.py:382-383) - keep >= [ares_ads] accel_mms2
accel_outside_manual_mms2 = 200.0  # CONFIRMED: amr_hmi value
watchdog_timeout_s = 2.0        # CONFIRMED: informative, FB_HMI_Interface T#2S (amr_hmi value)
ur_poll_hz = 10.0               # ASSUMPTION: UR panel refresh (URLink.state() is lock-protected, 125 Hz receiver)
camera_preview_scale = 0.25     # ASSUMPTION: 618 x 516 px preview of the 2472 x 2064 frame (draw 1 ms, 2026-10-07)
camera_live_hz = 1.0            # ASSUMPTION: REAL live view while no run is active (grab 123 ms, 2026-10-06)
sim_step_s = 0.3                # ASSUMPTION: SIM pacing, wait before every motion (unpaced a full sim takes 8 s)
confirm_arm_s = 0.5             # ASSUMPTION: Go of a step confirmation is enabled this long after it appears
resume_odom_tol_mm = 2.0        # ASSUMPTION: PLC odometry change since the run stopped that blocks a resume
                                # (= PLC minimum move fMinMove_mm)
resume_odom_tol_deg = 0.2       # ASSUMPTION: same for the heading (= fMinMove_deg)
log_view_max_lines = 5000       # ASSUMPTION
runs_dir = "data/runs"          # ASSUMPTION: HMI run logs go to <runs_dir>/<stamp>_hmi_<sim|real>/

[hmi.frame]
# Physical direction mapping of jog / relative move / odometry map (amr_hmi config.yaml frame). The two flags are
# confirmed TOGETHER (both true or both false); needs PLC >= v2.1 ([ares_ads] min_plc_build).
plus_y_is_left = true           # CONFIRMED: MA DECISIONS D21, direction test with PLC v2.1 28.09.2026
plus_omega_is_ccw = true        # CONFIRMED: same test
verified = true                 # CONFIRMED: same test

[hmi.move]
# Relative-move panel presets (operator values, amr_hmi config.yaml move).
default_distance_mm = 1000.0    # CONFIRMED: amr_hmi value
default_angle_deg = 90.0        # CONFIRMED: amr_hmi value
test_distance_mm = 100.0        # CONFIRMED: amr_hmi value ("Load test move", physical Left)
test_speed_mms = 50.0           # CONFIRMED: amr_hmi value

[hmi.twin]
# RoboDK digital twin (robodk/twin.py): an OWN RoboDK instance (rdk_common.connect(new_instance=True)); never the
# user's RoboDK (API 20500/20501), not robodk/simulate.py (20599), not the tests (20596-20598).
port = 20630                    # ASSUMPTION: first API port tried (>= 20630)
port_tries = 10                 # ASSUMPTION: ports port .. port + 9
rate_hz = 10.0                  # ASSUMPTION: tick measured 11 ms (visible window, 2026-10-07) -> ~11 % of one thread
socket_timeout_s = 5.0          # ASSUMPTION: API timeout after the build (connect() leaves 600 s)
build_timeout_s = 120.0         # ASSUMPTION: API timeout during build_station.build (3.6 s measured; STEP import
                                # if a mesh cache is missing)
visible = true                  # ASSUMPTION: normal window (a minimised RoboDK does not render, measured 2026-10-07)
ghost_wall = true               # ASSUMPTION: Wall_nominal shown as a transparent ghost
```

### 5.2 `hmi/core/config.py`

```python
def amr_cfg(cfg: Mapping) -> dict
    """The amr_hmi config dict {"ads", "plc", "hmi", "frame", "move"} from station.toml; KeyError names the missing
    table. Every amr consumer keeps its shape: AdsWorker cfg['ads'/'plc'/'hmi'], FrameConfig cfg['frame'],
    MoveLimits / MoveDefaults cfg['move'], JogPanel cfg['hmi']."""
def check_amr_cfg(cfg: Mapping) -> list[str]
    """Problems: [hmi] jog_accel_mms2 < [ares_ads] accel_mms2; heartbeat_interval_ms >= 1000 * ads.HB_WINDOW_S;
    frame flags differ; frame not verified; [hmi.twin] port < 20630 or port range containing 20500/20501."""
```

| amr key | Source |
|---|---|
| `ads.ams_net_id`, `ads.host_ip`, `ads.timeout_ms` | `[ares_ads]`, same keys |
| `ads.ads_port` | `[ares_ads] port` (the key name differs) |
| `plc.prefix`, `plc.to_plc_struct`, `plc.from_plc_struct` | `""`, `mauer.ares.ads.TO.rstrip(".")`, `FROM.rstrip(".")` |
| `hmi.*` (the 9 amr keys) | `[hmi]` |
| `frame.*` | `[hmi.frame]` |
| `move.default_distance_mm`, `default_angle_deg`, `test_distance_mm`, `test_speed_mms` | `[hmi.move]` |
| `move.default_speed_mms`, `default_rot_speed_degs`, `default_accel_mms2` | `[ares_ads] speed_mms`, `rot_speed_degs`, `accel_mms2` |
| `move.max_distance_mm`, `max_angle_deg`, `max_speed_mms`, `max_rot_speed_degs`, `max_accel_mms2`, `min_distance_mm`, `min_angle_deg` | `ads.MAX_DIST_MM`, `MAX_ANGLE_DEG`, `MAX_SPEED_MMS`, `MAX_ROT_SPEED_DEGS`, `MAX_ACCEL_MMS2`, `MIN_MOVE_MM`, `MIN_MOVE_DEG` (`mauer/ares/ads.py:101-108`) |

The amr widgets read the **main** config, without a variant. Variants never override `[hmi]` or `[ares_ads]`. Runs use the session config, which carries the job's variant.

**Consequence for existing jobs.** Adding `[hmi]` changes `mjob.config_sha256()`, which hashes the whole file (`job.py:831`). Every existing `data/jobs/*.json` then gets the `preflight_real` problem "config changed since the job was built". After the merge, rebuild the jobs once in the main tree: `py.exe tools/make_job.py` and `py.exe tools/make_job.py --variant c_acb`. INTEGRATE reports this. Keep later `[hmi]` edits rare for the same reason.

---

## 6. Resource ownership and threads

| Resource | Owner (creates / closes) | Thread that drives it | Other users |
|---|---|---|---|
| ADS connection #1 (heartbeat, MANUAL, HALT, jog, GO, odometry reset) | `AdsWorker`, created by `hmi/main.py` only with `--ares` | `ads-worker` QThread (amr, unchanged) | GUI through `worker.send/pulse/halt` (queue, priority 0 for HALT) |
| ADS connection #2 (relative moves of the run) | `RealRig` (`AresAds`) | `mauer-run` | `mauer-halt`: `abort()`, fallback only (`ads.py:726`, RLock) |
| UR (RTDE 30004, 30002 script, 29999 Dashboard) | `RealRig` (`URLink`) | `mauer-run` (`URRobot` blocks); `URLink` has its own RTDE receiver thread | GUI (`UrPanel`) and `mauer-twin` read `link.state()` (lock-protected); `mauer-halt`: `link.abort()` (up to ~2 s) |
| Camera (`IdsCamera` / `SynthCamera`) | `RealRig` / `SimRig` | `mauer-run` only (shots, manual grab when no run is active) | GUI gets `ShotView` previews through a signal |
| `SimWorld` | `SimRig` | `mauer-run` | GUI and twin read attributes only (rebound arrays, no mutation) |
| `Sequencer` + `RunLog` | `RunController` | `mauer-run` | GUI: `seq.pause()` (log write under the new RunLog lock); recovery calls are dispatched to `mauer-run` |
| RoboDK instance | `Twin` | `mauer-twin` | none |

**Threads**
- **GUI thread:** all widgets, `HmiContext`, `RunController`. Never blocks for more than about 20 ms. Never calls `run_block`, `AresAds`, the camera or RoboDK.
- **`ads-worker`** (QThread): amr `AdsWorker`.
- **`mauer-run`** (QThread): `RunWorker`. Loads sessions, opens and closes rigs, runs preflight, builds the `Sequencer` and runs `run()`, makes recovery calls and manual grabs, and runs the shot processor (detect plus preview, about 13 ms, in the same thread as `measure()`, as `detect.py:59-75` requires: "use them from one thread").
- **`mauer-halt`** (`threading.Thread`, daemon, short-lived): `URLink.abort()` and the `AresAds.abort()` fallback.
- **`mauer-twin`** (`threading.Thread`, daemon): RoboDK.

**Heartbeat under load.** The heartbeat is a Python QTimer in `ads-worker`. With a CPU-heavy run thread (SIM rendering 111 ms per frame, measurements) the GIL may delay it. A test asserts that the gap stays below `HB_WINDOW_S` = 0.25 s (section 12.2). The PLC watchdogs are 500 ms for a move and 2 s for the HMI.

---

## 7. Sequencer changes (`mauer/sequencer.py`)

`mauer/backends.py` is unchanged.

### Commit "Sequencer hooks for the HMI" (H1 to H4)

These four hooks are additive; behaviour is unchanged.

**H1: RunLog listeners and a write lock** (`RunLog`, lines 186-215)
```python
def add_listener(self, fn: Callable[[dict], None]) -> None
def remove_listener(self, fn: Callable[[dict], None]) -> None
def write(self, event, **data) -> dict
    # rec as today; json line + flush under self._lock (threading.Lock); then every listener gets rec in the
    # writer's thread; a listener exception is logged (log.warning) and swallowed - never reaches the run
```

**H2: frame tap.** New keyword-only parameter `Sequencer(..., on_shot: Callable[[np.ndarray, dict], None] | None = None)`.
- In `_measure.shoot`, the `shot` event write (line 724) becomes `rec = self.log.write("shot", ...)`.
- Then, if a tap is set: `self.on_shot(shot.frame.image, rec)` inside `try/except Exception` with `log.warning`.
- It runs in the run thread after `measure()`, so it cannot change the measurement.

**H3: `ares_cmd` event.** In `_ares_cmd` (line 475), BEFORE the backend call:
```python
self.log.write("ares_cmd", kind=kind, args=list(args), why=why, pose=self.pose_est, pose_src=self.pose_src)
```

**H4: `station_refilled` event.** After `self.station = SlotState.station(st)` (line 882):
```python
self.log.write("station_refilled", station=len(self.station))
```

### Commit "Sequencer: pause and abort wait until no stone is held; catch-all run_error" (H5, H6)

**H5: held-stone guard.** Behaviour changes ONLY while a stone is in the jaws.
- New attribute `self.held: dict | None = None`, shaped `{"from": "magazine"|"station", "slot": id, "kind": "full"|"half", "stone": list(key) | None, "unknown": bool}`.
- `_set_held(h)` sets `self.held = h` and writes `self.log.write("held", held=h)`.
- **`_place`** (line 973):
  - After `pick_magazine` returns and `magazine.take(mid)`: `_set_held({"from": "magazine", "slot": mid, "kind": t.kind, "stone": list(t.key), "unknown": False})`.
  - After `place_wall` returns: `placed.add(t.key)`, then `result.placed.append`, then `_set_held(None)`, then the unchanged `placed` event. Placed is updated before held is cleared, so a snapshot taken at the `held` event shows a consistent tool-to-wall transfer.
- **`_reload`**:
  - After `pick_station` and `station.take(sid)` (line 909): `_set_held({"from": "station", "slot": sid, "kind": kind, "stone": None, "unknown": False})`.
  - After `place_magazine` and `magazine.fill(mid, kind)` (line 912): `_set_held(None)`.
- **`_robot`, on `RobotError`:**
  - For `pick_magazine` / `pick_station`: `_set_held({... "slot": getattr(args[0], "id", None), "unknown": True})`, because the jaw state is unknown.
  - For `place_*`, if a stone is held: `_set_held({**self.held, "unknown": True})`.
- **`_confirm`** (line 434):
  ```python
  if self.paused and self.held is None:
      raise SequencerPaused("run paused", self.stop_k)
  if self.confirm is not None and not self.confirm(desc):
      if self.paused and self.held is None:       # pause pressed while the step waited for the operator
          raise SequencerPaused("run paused", self.stop_k)
      self.log.write("declined", what=desc)
      raise SequencerAborted(f"operator declined: {desc}", self.stop_k)
  ```
  While paused and holding, the place still runs; it is still confirmed in step mode. `_rotate`, `_translate` and `_interlock` keep their paused checks; no stone is held there because the arm must be parked.
- **`run()`**, before the `try` and after the paused check: if `self.held is not None`, raise `SequencerError("a stone may be in the jaws (<held>) - take it out / check the gripper, park the arm, then clear_held() and resume", start_stop)`.
- **`clear_held(source: str = "operator") -> None`**: writes `log.write("held_cleared", held=self.held, source=source)` and sets `self.held = None`.
- **The only existing test affected:** in `tests/test_sequencer.py::test_robot_block_error_stops_safely`, add `seq.clear_held()` after `w.robot.holding = None` (line 255). That is the operator action the guard now requires. A grep confirms no other test resumes with a stone held: the pause tests pause on ARES routes, and the step test declines an ARES move.

**H6: catch-all in `run()`.** After the `BackendError` branch (line 1079):
```python
except Exception as e:      # anything else (CameraError, a bug): the run ends in a defined state
    self.result.state, self.result.error = "error", f"{type(e).__name__}: {e}"
    self.log.write("run_error", error=str(e), stop=self.stop_k, kind=type(e).__name__)
    raise SequencerError(f"{type(e).__name__}: {e}", self.stop_k) from e
```
This fixes the confirmed `CameraError` escape, where `result.state` stayed "running" and no `run_error` was written. `tools/run_job.py` now receives a `SequencerError` instead of a traceback, which is intended.

---

## 8. RunController (`hmi/core/run_controller.py` and friends)

### 8.1 Data types

`hmi/core/session.py`
```python
@dataclass(frozen=True)
class JobSession:
    cfg: dict                 # mauer.config.load(config_path, variant)
    variant: str | None       # mauer.config.variant_of(cfg)
    job: Job
    path: Path | None         # job file; None = built from the config
    source: str               # "file" | "built" | "test"
    name: str                 # file stem, or "nominal_<shape><config.suffix(cfg)>"
class SessionError(ValueError): ...
def list_jobs() -> list[Path]                      # REPO/data/jobs/*.json, newest first
def list_variants() -> list[str]                   # config/variants/*.toml stems
def load_job_file(path, config_path=None) -> JobSession
    # variant = mjob.load(path, check=False).meta.get("config_variant") (as tools/run_job.py); cfg = config.load(
    # config_path, variant); job = mjob.load(path); SessionError when meta variant != variant_of(cfg) or JobError
def build_from_config(variant: str | None = None, config_path=None) -> JobSession
    # tools/make_job.py build_nominal(cfg) loaded by importlib (as tests/test_sequencer.py _tool); seconds -> worker
```

`hmi/core/rigs.py`
```python
@dataclass(frozen=True)
class RigInfo:
    mode: str                  # "sim" | "real"
    cfg: Mapping; job: Job
    robot: Any | None          # SimRobot | URRobot
    link: Any | None           # URLink (REAL)
    ads: Any | None            # AresAds (REAL)
    world: Any | None          # SimWorld (SIM)
    camera_kind: str | None    # "synth" | "ids"
@dataclass
class RealFactories:           # every default imports lazily; tests inject fakes
    link: Callable[[dict], Any]              # URLink.from_config(cfg)   (rig calls .start())
    ads: Callable[[dict], Any]               # AresAds(cfg["ares_ads"])  (rig calls .connect())
    camera: Callable[[dict], Any]            # mauer.camera.open_camera(cfg)
    robot: Callable[[Any, dict, Job], Any]   # URRobot(link, cfg, job)
    intrinsics: Callable[[dict], Any]        # intrinsics.load(repo_path([vision] intrinsics_file))
    handeye: Callable[[dict], np.ndarray]    # handeye.load(repo_path([vision] handeye_file)).T_flange_cam
class SimRig:   # SimWorld(cfg, job, scenario(opts.scenario), seed=opts.seed, start_stop=opts.start_stop,
                #          supersample=2, grasp_check=not opts.lenient_grasp)  (as tools/run_job.py sim_once)
class RealRig:  # open(): link.start() -> ads.connect() -> ads.check() -> camera -> robot (run_real order,
                # tools/run_job.py:228-283); each failure becomes a PreflightItem, the part stays None
                # close(): camera.close(), ads.close(), link.stop()  (seq.close() is done by the controller first)
```

`hmi/core/preflight.py`
```python
@dataclass(frozen=True)
class PreflightItem: source: str; text: str; blocking: bool   # source: "job/config"|"HMI"|"ARES"|"UR"|"camera"
@dataclass(frozen=True)
class PreflightReport:
    mode: str; items: tuple[PreflightItem, ...]; t: float
    @property
    def ok(self) -> bool: return not any(i.blocking for i in self.items)
def sim_preflight(cfg, job, config_path) -> PreflightReport         # preflight_real items, blocking=False (info)
def real_preflight(cfg, job, config_path, *, ares_enabled, ads_connected, ads_status, rig) -> PreflightReport
```

`real_preflight` uses NO override; every item blocks. It checks, in order:
1. `preflight_real(cfg, job, config_path=...)` (it includes the variant check).
2. HMI: `--ares` given; `AdsWorker` connected; interface v2; `eAmrState == 7` (MANUAL); not `bExtActive`.
3. ARES: the `AresAds` connect error, else `ads.check().problems`.
4. UR:
   - `link.start()` error;
   - `state()` is None or `state_age_s() > 0.5`;
   - `robot_mode != 7` ("power on + brake release on the pendant");
   - `safety_mode` not in `SAFETY_OK`;
   - `program_running`;
   - `controller_version[:2] == (3, 3)`: "PolyScope 3.3: get_inverse_kin_has_solution missing - every look / pick / place block fails (mauer/ur/script.py ik_guard needs a 3.3 variant; station.toml [ur] host comment)";
   - `URRobot` construction errors (payload, stone mass).
5. camera: the `open_camera` error.
6. Calibration loading errors.

`hmi/core/snapshot.py` (pure; built in the run thread)
```python
@dataclass(frozen=True)
class StoneInfo:
    key: tuple; label: str; leg: str | None; course: int; index: int; kind: str
    u_mm: float; z_top_mm: float; slot: str | None; i: int      # i = 1-based position in job.stones()
@dataclass(frozen=True)
class RunSnapshot:
    t: float; seq_state: str                  # RunResult.state: idle|running|done|paused|error|aborted
    n_stops: int; stop_k: int | None; stop_leg: str | None; stops_done: tuple[int, ...]
    n_stones: int; placed: frozenset          # stone keys (tuples)
    stone: StoneInfo | None                   # current_stone(job, stop_k, placed)
    action_kind: str; action: str             # describe_action(); kind: robot|ares|route|shot|measure|station|wait|idle
    at_station: bool; route: dict | None; reloads: int     # RouteProgress.to_dict()
    pose_est: Pose2D | None; pose_src: str; pose_status: str
    ares_cmd: dict | None                     # {"kind","args","why","pose": Pose2D,"odom": OdomPose|None} from the
                                              # 'ares_cmd' event until 'ares_move' / 'ares_error'
    T_wall_station: np.ndarray                # copy
    magazine: Mapping[str, str]; station: Mapping[str, str]   # filled slot -> kind (copies of SlotState)
    held: dict | None; pause_requested: bool
    ares_moves: int; corrections: int; warnings: tuple[str, ...]; error: str | None
    last_measurement: dict | None             # last camera 'wall_frame' / 'station_frame' record
    last_event: str; n_events: int; log_path: str
    @classmethod
    def initial(cls, session: JobSession, start_stop: int = 0) -> "RunSnapshot"   # before a sequencer exists
@dataclass(frozen=True)
class ShotView:
    t: float; look: str | None; parent: str | None    # look "live" for a manual grab
    preview: np.ndarray; scale: float; full_size: tuple[int, int]   # preview uint8 HxW or HxWx3 (BGR)
    rec: dict | None                          # the 'shot' event (boards: ok, n_corners, rms_px | reason)
    detections: dict | None = None            # CAMERA: {board: {"n": int, "ids": list, "pts": list}} (preview px)
    note: str = ""
ShotProcessor = Callable[[np.ndarray, dict | None, Mapping], ShotView]   # (image, shot rec, session cfg)
def default_shot_view(image, rec, cfg, scale: float) -> ShotView        # cv2.resize INTER_AREA only
def current_stone(job: Job, stop_k: int | None, placed: Collection) -> StoneInfo | None
    # first stone of job.stops[stop_k].stones not in placed (= the sequencer's own loop, sequencer.py run())
def describe_action(rec: dict) -> tuple[str, str] | None     # None = keep the previous action
def format_event(rec: dict) -> str                           # "HH:MM:SS.mmm  event  details", one line
def severity(rec: dict) -> str                               # "info" | "warning" | "error"
def category(rec: dict) -> str                               # "motion"|"vision"|"stones"|"run"|"warning"
```

`describe_action` mapping:

| Event | Action kind | Text |
|---|---|---|
| `robot` | robot | `"<action>: <what>"` |
| `ares_cmd` | ares | `"ARES <kind> <args> (<why>)"` |
| `ares_move` | ares | `"ARES <kind> ok / NOT ok: <summary>"` |
| `route` / `drive` / `resume_route` | route | — |
| `shot` | shot | `"image at look <look>: W0 ok 12 corners 0.01 px"` |
| camera `wall_frame` / `station_frame` | measure | — |
| `reload_start` | station | — |
| `station_empty` | wait | — |
| `station_refilled` | station | — |
| `run_*` | idle | — |

`severity`:
- error: `robot_error`, `ares_error`, `frame_jump`, `run_error`.
- warning: `warning`, `interlock`, `measurement_failed`, `declined`, `odometry_pose`, `run_paused`, `run_aborted`.

### 8.2 `RunOptions`, `ConfirmRequest`, `RunController`

```python
@dataclass
class RunOptions:
    mode: str                         # "sim" | "real"
    step: bool = False                # confirm every motion (changeable live: set_step)
    start_stop: int = 0
    stop_after: int | None = None     # inclusive
    camera_loop: bool = True
    save_images: bool = False
    scenario: str = "realistic"       # SIM: mauer.simworld.scenario
    seed: int = 1                     # SIM
    lenient_grasp: bool = False       # SIM
    sim_step_s: float | None = None   # SIM pacing; None = [hmi] sim_step_s (changeable live)
    log_dir: Path | None = None       # None = <runs_dir>/<YYYY-mm-dd_HHMMSS>_hmi_<mode>[_n] (never reused:
                                      # RunLog appends, sequencer.py:194)
@dataclass(frozen=True)
class ConfirmRequest:
    id: int; kind: str                # "motion" | "station_empty"
    text: str                         # the sequencer's description ("robot: ...", "ARES translate ...")
    holding: dict | None              # seq.held when asked
    pause_pending: bool; t: float

class RunController(QObject):
    state_changed = Signal(str, str)          # (state, detail)
    event = Signal(dict)                      # every RunLog record of the current sequencer, in order
    snapshot_changed = Signal(object)         # RunSnapshot after each run-thread record
    shot = Signal(object)                     # ShotView (sequencer shot or manual grab)
    confirm_requested = Signal(object)        # ConfirmRequest
    confirm_cleared = Signal(int)             # request id answered or released
    preflight_done = Signal(object)           # PreflightReport
    rig_changed = Signal(object)              # RigInfo | None
    session_loaded = Signal(object)           # JobSession
    message = Signal(str, str)                # (level "info"|"warning"|"error", text) -> status bar

    def __init__(self, hmi: Mapping, *, config_path: Path | None = None, runs_dir: Path,
                 ads_status_fn: Callable[[], dict | None] = lambda: None,
                 ads_connected_fn: Callable[[], bool] = lambda: False, ares_enabled: bool = False,
                 real_factories: RealFactories | None = None, parent: QObject | None = None)
    # properties (GUI thread): state, mode, session, opts, snapshot (latest RunSnapshot, atomic reference),
    #                          rig (RigInfo | None), preflight (PreflightReport | None), halted (bool), pending
    def can(self, action: str) -> tuple[bool, str]    # enable matrix 8.4 via the pure helper enables()
    # async (queued to mauer-run; result via signals)
    def load_file(self, path: Path) -> None
    def build_from_config(self, variant: str | None) -> None
    def prepare(self, opts: RunOptions) -> None       # rig + Sequencer + preflight -> state "ready"
    def recheck(self) -> None                          # REAL: preflight again without reconnecting
    def start(self) -> None
    def resume(self) -> None
    def confirm_pose(self) -> None; def set_pose(self, pose: Pose2D) -> None
    def apply_odometry(self) -> None                   # REAL: pose_est (+) odometry since the stop, via seq.set_pose
    def clear_held(self) -> None                       # seq.clear_held(); SIM also world.robot.holding = None
    def grab(self) -> None                             # REAL, no run active: one camera frame -> shot (rec None)
    def release(self) -> None                          # seq.close(), rig.close() -> state "loaded"
    # immediate, thread-safe (GUI thread)
    def set_session(self, session: JobSession) -> None # sync; tests and already built jobs
    def pause(self) -> None; def abort(self) -> None; def halt(self) -> None
    def answer_confirm(self, req_id: int, ok: bool) -> None
    def set_step(self, on: bool) -> None; def set_sim_step_s(self, s: float) -> None
    def set_shot_processor(self, fn: ShotProcessor | None) -> None
    def ur_source(self) -> "UrSource"                  # new UrSource on the current rig (section 9)
    def shutdown(self, timeout_s: float = 10.0) -> bool   # blocking, closeEvent only (section 10.1)
```

**Building the Sequencer** (in `mauer-run`), as in `tools/run_job.py` `sim_once` / `run_real`:
```python
seq = Sequencer(job, cfg, rig.robot, ares, camera, intr, T_flange_cam, log_dir=log_dir,
                confirm=self._confirm_cb, camera_loop=opts.camera_loop, save_images=opts.save_images,
                on_station_empty=self._station_empty_cb, on_shot=self._on_shot)
seq.log.add_listener(self._on_record)
```
Where the arguments come from:
- **SIM:** `ares=world.ares`, `camera=world.camera`, `intr=world.intr`, `T_flange_cam=world.T_flange_cam`.
- **REAL:** `ares=AdsAres(rig.ads)`, `camera=` the rig camera, `intr` and `T_flange_cam` from `RealFactories`.

A `Sequencer` is built only when every part exists. A REAL Start also requires `preflight.ok`; there is no override, as with `run_job --real`.

**The run** (`mauer-run`)
- `seq.run(opts.start_stop, opts.stop_after)`, wrapped in `try / except Exception`. A controller-level exception becomes state "error" with the message.
- After `run()` returns or raises:
  - final snapshot;
  - `<log_dir>/hmi_summary.json`: mode, job name and source, variant, opts, `asdict(seq.result)`, `world.summary()` in SIM, start and end times, `halted`;
  - REAL: record `odom_at_stop` from `ads_status_fn()`;
  - state = `seq.result.state` (done / paused / aborted / error).
- After an "error" or a HALT, `opts.step` is switched ON for the resume. The operator may switch it off.

**`_on_record(rec)`** (RunLog listener, in the writer's thread)
- Emits `event(rec)`.
- When called from the run thread: updates the action / `ares_cmd` / `last_measurement` state. At `ares_cmd` it stores `{pose, odom: ads_status_fn() odometry}`. It then builds a `RunSnapshot` (copies of the SlotStates, placed and held), stores it as `self._snapshot` (an atomic reference) and emits `snapshot_changed`.
- Records written from the GUI thread (`pause_requested`) only emit `event`.

**`_on_shot(image, rec)`** (run thread): `view = (processor or default_shot_view)(image, rec, session.cfg)` with `scale = [hmi] camera_preview_scale`; then `emit shot(view)`. Exceptions are caught; on failure it falls back to the default view.

### 8.3 Confirm protocol, pause, abort, HALT

```python
def _confirm_cb(self, desc: str) -> bool:                    # mauer-run (Sequencer.confirm)
    if self._halt.is_set():
        return False
    if self._abort_soft.is_set() and self._seq.held is None:  # soft abort: only with empty jaws
        return False
    if self._mode == "sim" and self._sim_step_s > 0 and self._halt.wait(self._sim_step_s):
        return False                                          # SIM pacing, interruptible by HALT
    if not self._step:
        return True
    return self._ask("motion", desc)

def _ask(self, kind: str, text: str) -> bool:                 # blocks the run thread, never the GUI
    req = ConfirmRequest(next(self._ids), kind, text, self._seq.held, self._seq.paused, time.time())
    with self._confirm_lock:
        self._pending, self._answer = req, None
        self._answer_evt.clear()
    self.confirm_requested.emit(req)
    while not self._answer_evt.wait(0.1):
        if self._halt.is_set():
            break
    with self._confirm_lock:
        ok = bool(self._answer) and not self._halt.is_set()
        self._pending = None
    self.confirm_cleared.emit(req.id)
    return ok

def _station_empty_cb(self) -> None:                          # Sequencer.on_station_empty
    if self._mode == "sim":
        self._world.refill_station(); return
    if not self._ask("station_empty", "Pick-up station empty or short of the next stone types: refill every "
                                      "slot, then press Refilled"):
        self._seq.log.write("declined", what="station refill")
        raise SequencerAborted("station refill declined", self._seq.stop_k)
```

**GUI-side controls**
- **`answer_confirm(id, ok)`**: ignored unless `id == pending.id`. Otherwise it sets the answer and the event.
- **`pause()`**: calls `seq.pause()`, which writes to the log under the lock. If a "motion" request is pending and `seq.held is None`, it releases the request with `False`; the sequencer then raises `SequencerPaused` thanks to the post-confirm check in H5. With a stone held, the request stays pending and the `ConfirmBar` shows "pause after this place".
- **`abort()`**: sets `_abort_soft`. A pending request is released only when nothing is held. With a stone held, Decline is the operator's explicit choice.
- **`halt()`**:
  1. `_halt.set()`;
  2. release the pending request with `False`;
  3. `halted = True`;
  4. REAL: start `mauer-halt`, which calls `rig.robot.abort()` (`URRobot.abort()` → `link.abort()`: stopl program plus Dashboard stop, at most about 2 s), and calls `rig.ads.abort()` (300 ms pulse) only when `not ads_connected_fn()`;
  5. `message(warning, ...)` with the helper's results.
  It returns at once. `_halt` and `_abort_soft` are cleared by the next `start()` / `resume()`.

| Trigger | ARES | UR | Sequencer | Result |
|---|---|---|---|---|
| HALT (button, Space, Esc; any tab, also while an HMI child window is active) | `AdsWorker.halt()`: one write `bCmdMoveAbort` TRUE plus 6 jog bits FALSE, reset after 300 ms; `AresAds.abort()` only if the worker is disconnected | `URLink.abort()` (REAL) | abort flag set, pending confirm released with False, step mode switched on | "error" (the motion failed) or "aborted" (at a confirm). A held stone means Jaws empty is needed before resume. |
| Pause | – | – | `seq.pause()`; acts at the next motion boundary with empty jaws | "paused", resumable |
| Abort | – | – | soft; the next confirm with empty jaws returns False | "aborted", resumable |
| Decline (ConfirmBar) | – | – | the confirm returns False now, even with a stone held | "aborted" |
| E-stop (ARES / UR hardware) | hardware | hardware | the sequencer sees the failure | "error" |

HALT is an operating function, not a safety function; the E-stops remain the safety function. The amr note text above HALT stays.

**Resume checks** (`mauer-run`, before `seq.run`). If one fails, the run is not started and a `message` explains why:
1. `seq.held is not None` → "jaws" (button **Jaws empty**).
2. `seq.pose_status != "ok"` → **Confirm pose** / **Set pose**.
3. REAL: `ads_status_fn()` is None or not connected → refused ("cannot verify ARES did not move").
4. REAL: `OdomPose(odom_at_stop).delta_to(now)` exceeds `resume_odom_tol_mm` / `_deg` → "ARES moved by (dx, dy, dθ) since the run stopped" → **Apply odometry** (`seq.set_pose(Pose2D.from_T(pose_est.T @ planar_T(dx, dy, radians(dθ))), "operator: odometry since the stop")`) or **Set pose**.
5. REAL: `recheck()`; any blocking item → refused.

Then: clear the flags, `seq.resume()` if `seq.paused`, and `seq.run(seq.stop_k if seq.stop_k is not None else opts.start_stop, opts.stop_after)`. This is the same `Sequencer` object; state lives only in memory (`sequencer.py` `resumed` logic).

### 8.4 Controller states and the enable matrix

**States:** `empty` → `loading` → `loaded` → `preparing` → `ready` → `running` ⇄ `pausing` / `aborting` → `paused` | `aborted` | `error` | `done` → `releasing` → `loaded`.

| Action | Allowed when |
|---|---|
| load, build, browse, mode and options | `empty` / `loaded` (no rig) |
| step, SIM speed | always (live) |
| prepare SIM | `loaded` |
| prepare REAL | `loaded` and `ares_enabled` (`--ares`) |
| start | `ready` and (SIM or `preflight.ok`) |
| pause | `running` |
| abort | `running` / `pausing` |
| resume | `paused` / `aborted` / `error` (the worker checks the rest, 8.3) |
| confirm pose | `paused` / `aborted` / `error` and `pose_status != "ok"` |
| set pose | `paused` / `aborted` / `error` |
| apply odometry | REAL, `paused` / `aborted` / `error`, odometry moved |
| jaws empty | `paused` / `aborted` / `error` and held |
| recheck | REAL and `ready` / `paused` / `aborted` / `error` |
| grab, live | REAL and `ready` / `paused` / `aborted` / `error` / `done` |
| release | `ready` / `paused` / `aborted` / `error` / `done` |
| HALT | always (no-op for the run when no rig exists) |

---

## 9. Read-only adapters (`hmi/core/sources.py`, CORE; used by STATUS and TWIN)

```python
@dataclass(frozen=True)
class UrSnapshot:
    source: str                         # "rtde" | "sim" | "none"
    t: float; age_s: float | None       # RTDE sample age (link.state_age_s())
    q_rad: tuple[float, ...] | None
    T_base_tcp_mm: np.ndarray | None    # URState.T_base_tcp_mm(); SIM: T_bf @ T_flange_tcp
    robot_mode: str | None; safety_mode: str | None; runtime_state: str | None   # link.ROBOT_MODE/... names
    safety_ok: bool | None; power_on: bool | None; program_running: bool | None
    do_open: bool | None; do_close: bool | None     # [ur] do_grip_open / do_grip_close: pulses, NOT a held state
    tool_voltage_v: int | None
    payload_kg: float | None; payload_cog_mm: list | None   # COMMANDED (URRobot.payloads / tool_kg), no read-back
    held_kind: str | None; parked: bool | None
    controller_version: tuple | None; rtde_error: str | None; missing_fields: tuple[str, ...]
class UrSource:
    def __init__(self, rig: RigInfo | None, job: Job | None)
    def snapshot(self) -> UrSnapshot
        # thread-safe; one instance per consumer thread. REAL: link.state() (lock-protected). SIM: world.robot.q,
        # else mauer.simworld.ik_near(world.robot.T_bf, self._q_prev) (SimRobot.q is None after Cartesian moves).
        # No rig: q = job.park_q_rad, source "none".
@dataclass(frozen=True)
class AresLive:
    pose: Pose2D | None                 # best current wall-frame pose for display
    src: str                            # "estimate" | "estimate+odometry" | "sim truth" | "start mark"
    status: str                         # seq.pose_status
    true_pose: Pose2D | None            # SIM: world.ares_true
    odom: OdomPose | None               # REAL: HMI ADS status (fPosX_m*1000, fPosY_m*1000, fPosTheta_deg); SIM: world.ares.odom
def ares_live(snap: RunSnapshot | None, ads_status: Mapping | None, world=None) -> AresLive
    # REAL during an ARES command (snap.ares_cmd with odom): pose = cmd pose (+) odom_cmd.delta_to(odom_now)
    # (body frame, as Sequencer._from_odometry); otherwise snap.pose_est. AdsAres forwards no progress callback
    # (backends.py:281-305), the HMI's own ADS status is the live source.
```

---

## 10. Main window and the CORE widgets

### 10.1 `hmi/main_window.py`: `MainWindow(ctx: HmiContext, worker, worker_thread: QThread | None = None)`

**Layout and display**
- **Window title:** `"Mauer HMI - <session name> - config <variant or 'main'> - <SIM|REAL|->"`, plus `" - ADS off"` without `--ares`.
- **Top area:** the amr `HaltButton` and note row, then the `ConfirmBar` (hidden unless a request is pending), then the `StatusStrip` (STATUS stub).
- **Tabs, in this exact order** (tests assert it): `Mauer`, `Camera`, `UR`, `ARES control` (the amr `ControlWidget`, attribute `self.control` kept), `Wall pose` (`AresPanel`), `Dashboard`, `Diagnostics`, `Battery`, `Run log`, `Twin`. `self.tabs.setFocusPolicy(Qt.NoFocus)` as in amr.
- **Status bar:** the amr labels plus `Mode`, `Run: <state>` and `controller.message` texts.

**Signal handling**
- **`_on_status(data)`:** the amr body, then `ctx.publish_ads_status(data)`, then `_apply_run_lock()`. `_update_visible_tab` is unchanged for the amr tabs; Mauer widgets subscribe to `ctx.ads_status` themselves and skip work while hidden.
- **`_on_connection`:** amr, then `ctx.publish_ads_connection(ok, msg)`.

**`trigger_halt()`**
```python
self.control.release_jog(send=False)   # amr: local state, no extra write
self._worker.halt()                     # amr: FIRST hardware action, unchanged
self._ctx.controller.halt()             # Mauer: never blocks
# amr status message ("HALT sent" / "HALT NOT sent: no ADS connection - use the E-stop") + run note
```

**`eventFilter`**
- HALT keys (Space/Esc) act when `_halt_scope()` is true: `QApplication.activeWindow()` is `self`, or a window whose `parentWidget()` chain reaches `self` (HMI child windows such as the non-native file dialog). They are always swallowed.
- Jog keys are unchanged: only while `self.isActiveWindow()`, on the ARES control tab and its Jog sub-tab.

**`_apply_run_lock()`** runs on `controller.state_changed`, `_on_status` and `_on_connection`:
- REAL and state in {`running`, `pausing`, `aborting`}: `control.set_run_lock("Mauer REAL run active - pause the run first")`.
- REAL and a sequencer exists (`ready` … `error`): `control.set_reset_lock("Mauer REAL run loaded - an odometry reset would break the resume check")`.
- Otherwise both are cleared.
- While a REAL run is paused, jog and GO are allowed (repositioning); the resume check (8.3) catches the movement.

**`closeEvent`**
- If the state is in {`preparing`, `running`, `pausing`, `aborting`, `releasing`}: `event.ignore()` and the status message "Stop the run first (Pause / Abort / HALT), then close". Closing the HMI stops the heartbeat; the PLC then aborts a move after 500 ms and drops MANUAL after 2 s.
- Otherwise:
  1. `_closing = True` and remove the event filter (amr);
  2. `ctx.run_shutdown_hooks()` (twin stop and similar; each guarded);
  3. `controller.shutdown(10 s)`: release a pending confirm, soft abort, join the run thread, `seq.close()`, then camera, `ads.close()`, `link.stop()`, then quit `mauer-run`;
  4. `worker.request_stop()` (amr; the heartbeat stops LAST);
  5. `worker_thread.quit()` / `wait(3000)`.

`hmi/core/context.py`
```python
class HmiContext(QObject):
    session_changed = Signal(object)          # JobSession | None (re-emitted RunController.session_loaded)
    ads_status = Signal(dict)                 # AdsWorker status (10 Hz), forwarded by MainWindow
    ads_connection = Signal(bool, str)
    twin_state_changed = Signal(str, str)
    def __init__(self, station_cfg: dict, *, config_path: Path | None = None, ares_enabled: bool = False,
                 controller=None, runs_dir: Path | None = None, parent=None)
        # creates RunController(station_cfg["hmi"], ..., ads_status_fn=lambda: self.last_ads_status,
        #                       ads_connected_fn=lambda: self.ads_connected) unless one is injected
    station_cfg: dict; hmi: dict; amr_cfg: dict; config_path; ares_enabled: bool; controller
    last_ads_status: dict | None              # replaced per poll, never mutated: readable from any thread
    ads_connected: bool; twin_state: tuple[str, str]
    session: JobSession | None                # property -> controller.session
    def publish_ads_status(self, d: dict) -> None; def publish_ads_connection(self, ok: bool, msg: str) -> None
    def set_twin_state(self, state: str, detail: str = "") -> None
    def add_shutdown_hook(self, fn: Callable[[], None]) -> None; def run_shutdown_hooks(self) -> None
```

`hmi/core/ads_link.py`
```python
def probing_factory(ads_cfg: Mapping) -> Callable[[], Any]
    # mauer.ares.ads.tcp_reachable(target_host(ams_net_id, host_ip), ADS_TCP_PORT 48898, TCP_PROBE_MAX_S) first
    # (pyads blocks ~20 s on an unreachable target, ads.py:57-61), then pyads.Connection (lazy import). Raises
    # OSError with the reason -> AdsWorker.connect_now reports "connect failed: ..." and retries.
def start_ads_worker(amr_cfg: dict) -> tuple[AdsWorker, QThread]   # thread "ads-worker", started
class NullAdsWorker(QObject):    # same signals and API as AdsWorker; never connects. start() emits
    # connection(False, "ADS off: start the HMI with --ares"); send/pulse/halt record and emit
    # command_error("ADS off: not sent"); is_connected False; request_stop() no-op
```

`hmi/main.py`
```
py.exe -m hmi [--job PATH] [--variant NAME] [--config PATH] [--ares] [--twin]
  --job      load this job at start (its own config variant, D-H11)
  --variant  preselect the variant for "Build from config"
  --ares     create the real AdsWorker (connects to [ares_ads] host: heartbeat, MANUAL, HALT, jog). Without it the
             HMI never opens an ADS connection; REAL mode is disabled
  --twin     start the RoboDK twin once a job is loaded
def build(argv: list[str] | None = None) -> tuple[MainWindow, HmiContext]   # no app.exec(); used by a test
def main(argv: list[str] | None = None) -> int
```
`main` checks `check_amr_cfg(station_cfg)` and shows any problems in the status bar. It never refuses to start; ADS is refused only if `--ares` is given and the checks fail.

### 10.2 Mauer tab (`hmi/views/mauer_tab.py`, `MauerTab(ctx, parent=None)`)

The left column is about 430 px wide; `PlanView` fills the right side.

**Job group**
- Combo of `list_jobs()` + **Load**.
- **Build from config** + variant combo ("main" + `list_variants()`).
- **Browse…** (non-native `QFileDialog`; enabled only per 8.4 and while no ARES `bMoveActive`).
- Summary: name, variant, stops, stones (full / half), magazine capacity, station slots, source, created, number of meta warnings.

**Run group**
- SIM / REAL radio. REAL is disabled without `--ares`, with a tooltip.
- SIM: scenario (none / e003 / slip / realistic), seed, lenient grasp.
- Common: stops from / to, camera loop, save images.
- Live: **Step mode**, **SIM s per motion**.

**Buttons** (all `NoFocus`; enabled from `controller.can`): **Prepare / Connect**, **Start**, **Pause**, **Resume**, **Abort**, **Release**.

**Progress** (from `snapshot_changed`)
- Run state (coloured, with a "HALTED" badge).
- Stop k/n (leg, a mm).
- Stone i/n (label, leg, course, kind, slot).
- Current action.
- Station trip: at station; route kind with `next_leg` / `n_legs`; reloads.
- ARES pose estimate: x mm, y mm, θ deg, source, status (green ok / orange odometry / red unknown).
- Held stone. Pause / abort pending. Last error.

**Recovery** (visible in `paused` / `aborted` / `error`)
- **Confirm pose**.
- **Set pose**: x mm, y mm, θ deg spin boxes, prefilled from `pose_est`, + **Apply**.
- **Apply odometry** (REAL).
- **Jaws empty**.
- **Re-check** (REAL).

**Preflight list**
- `QListWidget`, one item per `PreflightItem`: `"[source] text"`. Red means blocking, grey means info.
- Header: "REAL refused: n problems" / "REAL preflight ok" / "SIM (informative)".

**Feedback:** `RunFeedback(ctx)` (STATUS stub) at the bottom.

### 10.3 `PlanView(parent=None)` (`hmi/views/plan_view.py`, also used by STATUS)

```python
def plan_geometry(job: Job, cfg: Mapping) -> PlanGeometry    # pure, tested
    # stone footprints by key (T_wall_tcp position + yaw, t.length_mm or [brick] length, [brick] width),
    # stop poses (Stop.ares), routes (route / route_to_station / route_from_station waypoints), obstacles
    # (mauer.floor.job_obstacles when job.legs), station table (floor.station_table_poly), dock pose, bounds
class PlanView(QWidget):
    def set_session(self, session: JobSession | None) -> None
    def set_snapshot(self, snap: RunSnapshot | None) -> None
    def set_live(self, live: AresLive | None) -> None
    show_routes: bool; show_stops: bool
```
- QPainter in the wall frame (x right, y up), fitted to the bounds, with a 0.5 m grid.
- Planned stones: outline. Placed stones: filled, shaded by course. The current stone: highlighted.
- Stops: ghost ARES footprint (`floor.AresShape.footprint`) with the index. Routes: thin polylines. Obstacles: grey.
- ARES estimate: solid footprint, coloured by `pose_status`. SIM truth: dashed. A live REAL pose while a move runs.
- Repaint only through `update()`.

### 10.4 `ConfirmBar(ctx, parent=None)` (`hmi/views/confirm_bar.py`)

- A yellow frame with the request text in large type.
- **motion:** **Go** + **Decline**. **station_empty:** **Refilled** + **Abort run**.
- Go / Refilled are enabled `[hmi] confirm_arm_s` after the request appears, so a double click cannot carry over to the next step. Buttons are `NoFocus`; there are no keyboard shortcuts (Space/Esc are HALT).
- With a stone held, it shows: "Declining leaves the stone in the jaws - resume then needs 'Jaws empty'". With a pending pause / abort: "Pause / abort after this place".
- It hides itself on `confirm_cleared`.

### 10.5 `LogView(ctx, parent=None)` (Run log tab, `hmi/views/log_view.py`)

- A read-only `QPlainTextEdit` with `setMaximumBlockCount([hmi] log_view_max_lines)`, one `format_event` line per `controller.event`, coloured by `severity`.
- Category filter checkboxes; **Open log folder** (`QDesktopServices`); the log path label.

### 10.6 Feature stubs created by CORE

Each stub is a `QWidget` with a grey label "<Name>: implemented by the <FEATURE> step". The constructors are fixed:

```python
CameraView(ctx, parent=None)    # hmi/views/camera_view.py   -> CAMERA
UrPanel(ctx, parent=None)       # hmi/views/ur_panel.py      -> STATUS
AresPanel(ctx, parent=None)     # hmi/views/ares_panel.py    -> STATUS
RunFeedback(ctx, parent=None)   # hmi/views/feedback.py      -> STATUS
StatusStrip(ctx, parent=None)   # hmi/views/status_strip.py  -> STATUS
TwinPanel(ctx, parent=None)     # hmi/views/twin_panel.py    -> TWIN
```

**Rules for every feature widget**
- Update only from signals or timers.
- Never block the GUI thread for more than about 20 ms.
- No modal dialogs.
- Buttons are `NoFocus`.
- Handle `ctx.session is None` and every controller state.
- Never touch hardware objects except through `ctx.controller` (`ur_source`, `grab`, `rig`) and read-only state.

---

## 11. The three features

### 11.1 CAMERA (branch `hmi-camera`)

**Files:** `hmi/core/overlay.py` (new), `hmi/views/camera_view.py` (replaces the stub), `tests/test_hmi_camera.py`.

```python
@dataclass(frozen=True)
class ShotContext: specs: Mapping[str, BoardSpec]; border_px: float; scale: float; show_ids: bool
def shot_context(cfg: Mapping, hmi: Mapping, show_ids: bool = False) -> ShotContext
    # board_specs(cfg), [vision] border_px, [hmi] camera_preview_scale
def make_shot_view(image: np.ndarray, rec: dict | None, cfg: Mapping, sctx: ShotContext) -> ShotView
    # run thread: detect_boards(image, specs of rec["boards"] (all specs for a live grab), border_px);
    # preview = gray resize (INTER_AREA) -> BGR; corners green dots, corner ids (show_ids), per board
    # "name n corners rms px" (rec), boards with ok False listed in red with their reason
```

**`CameraView`**
- In `__init__` it calls `ctx.controller.set_shot_processor(...)`. The processor caches a `ShotContext` per `id(cfg)` and reads the "corner ids" flag.
- Image: a `QLabel` scaled with the aspect ratio kept. Conversion: `QImage(arr.data, w, h, 3 * w, QImage.Format_BGR888).copy()`; `QPixmap` in the GUI thread only.
- Info panel: look, parent, time; a board table (board, ok, corners, rms px, reason).
- Last fit from the `wall_frame` / `station_frame` camera events: rms mm, max mm, error vs nominal (mm, deg), jump (mm, deg), baseline mm.
- `measurement_failed` / `frame_jump` banner. A shot counter.
- **Grab** and **Live** (`[hmi] camera_live_hz`, QTimer → `controller.grab()`) only when `controller.can("grab")`. Live stops automatically when a run starts. Both are disabled in SIM: an extra `SynthCamera` grab changes the noise seeds (`simcam.py:82`).
- "Dead reckoning: no images" when `camera_loop` is False.

### 11.2 STATUS (branch `hmi-status`)

**Files:** `hmi/views/{ur_panel,ares_panel,feedback,status_strip}.py` (replace the stubs), `tests/test_hmi_status.py`.

**`UrPanel`**
- A QTimer at `[hmi] ur_poll_hz` reads `self._src.snapshot()`. `_src = ctx.controller.ur_source()`, renewed on `rig_changed`.
- Shows:
  - source and RTDE age (red if > 0.5 s), `rtde_error`, controller version, missing fields;
  - robot mode, safety mode (coloured), runtime state, power on, program running;
  - joints in deg; TCP x / y / z mm and rotation vector in deg;
  - gripper DO open / close with a note "pulses - not a held-state signal"; tool voltage;
  - payload, labelled "commanded (no read-back)";
  - held stone (from the snapshot); parked; the last robot action.
- SIM: "SIM - no RTDE" plus joints.

**`AresPanel` (Wall pose tab)**
- A `PlanView`, plus `ares_live(...)` from `snapshot_changed` and `ctx.ads_status`.
- Numbers: estimate x, y, θ with source and status; SIM truth and the estimate error (mm, deg); live odometry pose during a move; PLC odometry raw; the last ARES move summary (`ares_move`); a corrections counter; the station estimate vs nominal (`station_estimate`).
- A table of the camera fits per stop: stop, kind, why, rms mm, error vs nominal mm / deg, jump mm.

**`RunFeedback` (in the Mauer tab)**
- Accumulates from `controller.event`: placed n/N, reloads, ARES moves, corrections, warnings (a list), the measurement statistics, elapsed time and seconds per stone.
- SIM, once the run has ended: `rig.world.summary()["placement"]` (n, seated, horiz mean / p95 / max mm, pin max mm, by kind).
- "Open summary" opens `hmi_summary.json`.

**`StatusStrip` (global)** shows: run state (coloured) | mode | stop k/n | stone i/n label | short action | UR mode / safety | ARES state name + "move" if `bMoveActive` (`ctx.ads_status`) | twin state (`ctx.twin_state_changed`).

### 11.3 TWIN (branch `hmi-twin`)

**Files:** `robodk/twin_model.py`, `robodk/twin.py`, `hmi/core/twin_link.py`, `hmi/views/twin_panel.py` (replaces the stub), `tests/test_twin_model.py`, `tests/test_twin_robodk.py`.

**`robodk/twin_model.py`** (pure; imports only numpy and `mauer`)
```python
@dataclass(frozen=True)
class StoneState:
    magazine: Mapping[str, str]; station: Mapping[str, str]       # slot id -> kind
    placed: frozenset                                             # stone keys
    held: tuple[str, str | None, str] | None                      # (from "magazine"|"station"|"unknown", slot, kind)
    @classmethod
    def initial(cls, job) -> "StoneState"     # magazine initial_fill/initial_kinds, full station, nothing placed
    @classmethod
    def from_snapshot(cls, snap) -> "StoneState"
Loc = tuple[str, Any]                       # ("mag", sid) | ("station", sid) | ("wall", key) | ("tool", None)
@dataclass(frozen=True)
class Op: kind: str; src: Loc | None; dst: Loc | None; stone_kind: str      # "move" | "add" | "remove"
def plan_ops(old: StoneState, new: StoneState) -> list[Op]
    # removed / added locations paired by kind in this order: mag->tool, station->tool, tool->wall, tool->mag,
    # then mag->wall and station->mag (SIM states skipped between two ticks); unpaired = remove / add;
    # removes before adds
@dataclass(frozen=True)
class TwinFrame:
    q_rad: tuple[float, ...] | None; ares: Pose2D | None; ares_label: str
    T_wall_station: np.ndarray | None; stones: StoneState | None; caption: str = ""
@dataclass(frozen=True)
class TwinSettings:
    port: int = 20630; port_tries: int = 10; rate_hz: float = 10.0; socket_timeout_s: float = 5.0
    build_timeout_s: float = 120.0; visible: bool = True; ghost_wall: bool = True
    @classmethod
    def from_config(cls, cfg) -> "TwinSettings"   # [hmi.twin]; ValueError if port < 20630 or 20500/20501 in range
```

**`robodk/twin.py`** (flat imports and path setup as in `simulate.py:71-74`; imported only by `TwinLink.start()` and the robodk test)
```python
class TwinScene:                                  # twin thread only
    def __init__(self, RDK, cfg: dict, job, settings: TwinSettings)
    def build(self) -> None
        # build_station.build(RDK, cfg) (3.6 s, opens NO Cam2D); RDK.setCollisionActive(COLLISION_OFF);
        # f_wall.setPose(transl(0,0,0)) (world = wall frame, as simulate.py LSim.setup:523-527);
        # Wall_nominal GHOST colour (simulate.py:88) or hidden; RDK.Render(False); robot at park
    def apply(self, frame: TwinFrame) -> None
        # robot.setJoints(np.degrees(q).tolist()); f_ares.setPose(pose2d_T(ares)) (simulate.py:406, 2 lines
        # re-implemented - importing simulate.py pulls the planner); f_station.setPose(...) from T_wall_station
        # (z of the nominal kept); plan_ops(displayed, frame.stones) applied; ONE RDK.Render()
    def stone_count(self) -> dict[str, int]      # {"mag", "station", "wall", "tool"} for tests
class Twin:
    def __init__(self, cfg: dict, job, frame_fn: Callable[[], TwinFrame | None], settings: TwinSettings,
                 on_status: Callable[[str, str], None] = lambda s, d: None, *, connect=None, close=None)
    def start(self) -> None                       # returns at once; thread "mauer-twin"
    def stop(self, timeout_s: float = 15.0) -> None
    state: str          # "off" | "starting" | "running" | "lost" | "stopped"
    port: int | None; last_error: str | None; ticks: int; tick_ms: float   # median of the last 50
```

**Stone poses** (from `simulate.py` `LSim`):
- magazine: child of `f_ares` at `to_robodk(slot.T_ares_tcp) * rotx(PI) * T_tc_cad(cfg, kind)`;
- station: child of `f_station` at `to_robodk(slot.T_station_tcp) * rotx(PI) * T_tc_cad`;
- wall: child of `f_wall` at `to_robodk(task.T_wall_tcp) * rotx(PI) * T_tc_cad` (NOMINAL pose);
- held: `setParent(tool)` + `setPose(rdk_common.held_stone_pose_kind(cfg, kind))`. Use `setParent`, not `setParentStatic`; the latter's absolute pose is stale with render off.
- Mesh: `rdk_common.stone_mesh_path(cfg, kind)`, without `Sim`'s Scale. Colour: `build_station.STONE` / `HALF_STONE`, or `FAILED` for `held` with `"unknown"`. Name: `"Twin_<loc>"`.

**Thread loop**
```
for port in range(p, p + tries): try rdk_common.connect(new_instance=True, port=port, minimized=not visible)
                                 except RuntimeError: continue          # foreign RoboDK on that port
RDK.COM.settimeout(build_timeout_s); scene.build(); RDK.COM.settimeout(socket_timeout_s); state "running"
loop until stop: t0; frame = frame_fn(); if frame: scene.apply(frame); sleep(max(0, 1/rate - elapsed))
any exception -> state "lost" (on_status), leave the loop; finally close_instance(RDK) (exit code 3221225477
= 0xC0000005 is the known harmless exit crash) unless the user already closed RoboDK
```
- Frames are polled, so there is no backlog.
- A hung RoboDK costs at most `socket_timeout_s`.
- The twin never writes to the sequencer, the link or the controller.

**`hmi/core/twin_link.py`**
```python
def frame_from(snap: RunSnapshot | None, ur: UrSnapshot | None, live: AresLive | None, session) -> TwinFrame   # pure
    # SIM: ares = live.true_pose ("sim truth": consistent with the true joints); REAL: live.pose
    # ("estimate" / "estimate+odometry"); before a run: RunSnapshot.initial(session)
class TwinLink(QObject):
    status = Signal(str, str)                     # re-emits Twin.on_status into the GUI thread (queued)
    def __init__(self, ctx, parent=None)
    def start(self) -> None     # needs ctx.session; settings = TwinSettings.from_config(session.cfg); its own
                                # UrSource (twin-thread instance); lazy import of robodk/twin.py
    def stop(self, wait: bool = False) -> None    # stop() in a helper thread unless wait
    def frame(self) -> TwinFrame | None           # called in mauer-twin; reads controller.snapshot (atomic ref),
                                                  # UrSource.snapshot(), ctx.last_ads_status
```

**`TwinPanel`**
- A "RoboDK twin" checkbox, a status label (state, port, Hz, tick ms, error) and **Restart**.
- Registers `ctx.add_shutdown_hook(link.stop(wait=True))` and calls `ctx.set_twin_state(...)`.
- Restarts the twin on `ctx.session_changed`, so the variant and job stay consistent.
- `--twin` auto-starts it on the first session.
- REAL: wall stones are nominal and the ARES pose is the sequencer's estimate. The panel shows "twin = sequencer belief".

---

## 12. Tests

### 12.1 Infrastructure (CORE)

**`tests/conftest.py`**
- Before any Qt import: `os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")`.
- `qapp`: a session-scoped `QApplication`.
- `no_lab_network`: monkeypatches `socket.socket.connect` to raise `AssertionError` for any non-loopback address (127.0.0.1, ::1 and localhost are allowed: FakeUR, RoboDK API), and sets `sys.modules["pyads"] = None` and `sys.modules["ids_peak"] = None` so that `import pyads` raises.

**`tests/hmi_fakes.py`**
- The amr fakes: `FakeADSError`, `plc_symbols`, `FakeConnection`, `FakeWorker`, `Scheduler`.
- `wait_until(pred, timeout_s, qapp)`: a `processEvents` loop.
- `short_sim_session(n0=2, n1=1)`: `conftest.straight_config()` plus `make_job.build_nominal(cfg, length=10)` trimmed like `tests/test_sequencer.py::short_job`, cached with `lru_cache`. Returns a `JobSession(source="test")`.
- `make_ctx(qapp, tmp_path, controller=None, ares_enabled=False)` and `make_window(...)` (a `MainWindow` with a `FakeWorker`).
- `FakeRunController`: the same signals and attributes as `RunController`, settable `state`, `mode`, `snapshot`, `rig`; it records calls.
- `StubLink`: `state()` returns a `URState` from given values; it records `abort` / `stop`.
- `StubCamera`: `grab()` returns a `Frame`.

**Commands**
- From the worktree: `py.exe -m pytest -q tests`.
- HMI only: `py.exe -m pytest -q tests -k "hmi or twin_model or sequencer_hooks"`.
- RoboDK twin (opt-in): `py.exe -m pytest -m robodk tests/test_twin_robodk.py -o addopts="" -q`.
- Budget: the HMI suite should take under 60 s.

### 12.2 CORE tests

| File | Asserts |
|---|---|
| `test_hmi_logic.py`, `test_hmi_frame.py`, `test_hmi_ads_worker.py`, `test_hmi_worker_thread.py` | The amr tests, green on the package. `run_lock` cases for `jog_enabled` / `go_enabled`. `[hmi.frame]` true/true/verified and `[ares_ads] min_plc_build == "2.1"`. Lazy-import guard: no module in `hmi/**/*.py` imports `pyads`, `ids_peak`, `robodk`, `robolink`, `rdk_common` or `twin` at module level (AST, like amr `test_ads_worker.py:326-340`). |
| `test_hmi_gui_smoke.py` | The amr GUI tests on `MainWindow(ctx, FakeWorker())`; the exact tab list (10.1); the amr HALT key tests. |
| `test_hmi_config.py` | The `amr_cfg` mapping table (5.2); `check_amr_cfg(config.load()) == []`; `C.ROBOT_LENGTH_M * 1000 == [ares] length` and the same for width; every `[hmi*]` key has a status tag other than UNTAGGED (`mjob.config_status()`). |
| `test_sequencer_hooks.py` | H1: listener order and content, a listener exception does not break the run, parallel writes give no interleaved lines. H2: `on_shot` once per `shot` event, with that event's rec. H3: `ares_cmd` precedes every `ares_move`. H4: `station_refilled` after `station_empty`. H5: pause in step mode at "robot: place stone" → place done, then "paused", no stone lost (`len(world.records)` + magazine stones conserved) → resume → done; decline while held → `run()` refused until `clear_held()`; pick failure (`fail_on`) → held `unknown`. H6: a camera whose `grab` raises `CameraError` → state "error", a `run_error` kind `CameraError`, `SequencerError` raised. |
| `test_hmi_pattern_a.py` | `FakePlc(realtime=True, hmi_heartbeat=False)` (first fix `FakePlc._amr_state_machine`: READY + `bCmdManualMode` and not stop → MANUAL, the amr startup step "Manual"). `AdsWorker` in a QThread on `plc.factory("hmi")`; `worker.send({"bCmdManualMode": True})` → MANUAL. `AresAds(cfg["ares_ads"], connection_factory=plc.factory("client"))`: `preflight() == []`, `translate(100, 0)` ok; client writes ⊆ `ads.WRITABLE`. `worker.halt()` during `translate(1000, 0)` → `MoveOutcome` not ok (result 20). Heartbeat write gaps (`WriteRecord` conn "hmi") < 0.25 s while a SIM `RunController` run (`sim_step_s=0`) loads the CPU. |
| `test_hmi_run_controller.py` | SIM end-to-end, headless (threaded `RunController`, `short_sim_session`, scenario none, `sim_step_s=0`): prepare → `ready` + `preflight_done` (non-blocking) → start → `done`; events from `run_start` to `run_done`; `snapshot.placed` = 3 = `len(world.records)`; one `shot` signal per `shot` event (preview size = round(0.25 × camera size)); unique log dir under `tmp_path`; `hmi_summary.json`. Step mode: Go except the first "ARES" → Decline → "aborted" → resume → `done`, every stone placed once. Pause at "robot: place stone" → `paused` after the `placed` event. HALT on a pending request → released within 1 s, `halted`, step forced on, flags cleared by resume. Soft abort with a stone held → aborted after the place. Resume refused while held / pose unknown; `clear_held` / `set_pose` → `done`. REAL station-empty callback: Refilled returns; Abort writes `declined` and raises `SequencerAborted`. |
| `test_hmi_real_rig.py` | `RealRig` with `RealFactories` fakes: `StubLink` with `controller_version (3, 3, 3, 176)` → IK-guard blocker; `AresAds` on `FakePlc` + `HmiHeartbeat`; `StubCamera`; payload / mass items; `PreflightReport.ok` False → Start refused. Close order: `seq.close`, camera close, `ads.close`, `link.stop`. HALT helper: `link.abort` called, `ads.abort` only when the ADS worker is disconnected. Nothing outside loopback contacted (fixture). |
| `test_hmi_mauer_tab.py` | Through `MainWindow`: SIM session → Prepare → Start → `done`, progress shows stone 3/3. `ConfirmBar` Go disabled for `confirm_arm_s`, then enabled. Space while the bar is visible → `worker.of("halt")` and the request released. Space with an HMI child dialog active (monkeypatched `activeWindow`) → HALT. A fake REAL `running` state → GO / jog / Reset pose disabled with the reason; REAL `paused` → GO allowed, Reset pose locked. Close refused while `running` (`request_stop` not called). Title contains the variant name. `hmi.main.build([])` uses `NullAdsWorker`. |
| `test_hmi_snapshot.py` | `current_stone`, `describe_action` (table 8.1), `format_event` / `severity` / `category`, `RunSnapshot.initial`, `plan_geometry` bounds and counts, `enables()` matrix (8.4), `ares_live` odometry composition. |

### 12.3 Feature tests

**CAMERA** (`test_hmi_camera.py`)
- `make_shot_view` on a `SynthCamera` frame showing one board: detections for the rec's boards, preview size, ids drawn when `show_ids`, a missing board listed.
- `CameraView` updates from `FakeRunController.shot`.
- Grab / Live disabled in SIM and while `running`, enabled in REAL `ready`; Live stops on `running`.

**STATUS** (`test_hmi_status.py`)
- `UrPanel` with a stub `UrSource`: REAL texts, safety colour, joints in deg, stale RTDE in red; SIM source.
- `AresPanel`: live odometry pose during an `ares_cmd`, SIM truth error.
- `RunFeedback` after a real short SIM run shows placed and the SIM placement statistics.
- `StatusStrip` texts.

**TWIN**
- `test_twin_model.py` (no RoboDK): `plan_ops` for a pick from the magazine, a place on the wall, skipped states (magazine to wall), a station refill, station → tool → magazine, an unknown held stone, a kind mismatch; `TwinSettings.from_config` port rules; `frame_from` in SIM and REAL.
- `test_twin_robodk.py`: marker `robodk`, opt-in like `test_robodk_l.py`; skipped if `C:\RoboDK\bin\RoboDK.exe` is missing.
  - `Twin` with port 20630.. and `visible=False`; `connect` is wrapped to assert `port >= 20630`.
  - Feed frames for the initial state, a pick and a place. `stone_count` shows wall 1 and tool 0; the robot joints match `q`.
  - `stop()` leaves the process ended (return code 0 or 3221225477).
  - The user's RoboDK is never touched.

---

## 13. Implementation steps, branches, file ownership

### CORE (branch `hmi-core`, from `main` at `adb69cc` or later)

**Commits, in order:**
1. **C1** `hmi: copy MA amr_hmi v2 as package hmi.amr (relative imports); its tests run offscreen`. Includes `tests/conftest.py` (Qt env, `qapp`, `no_lab_network`), `tests/hmi_fakes.py` (amr fakes), `test_hmi_{logic,frame,ads_worker,worker_thread}.py` (cfg from a test dict = amr config.yaml values) and `requirements.txt` + PySide6.
2. **C2** `Config [hmi] in station.toml; hmi.core.config maps it and [ares_ads] to the amr_hmi config (no YAML)`. Tests switch to `amr_cfg(config.load())`; adds `test_hmi_config.py`.
3. **C3** `Sequencer hooks for the HMI: run-log listeners, frame tap, ares_cmd / station_refilled events` (H1 to H4 + tests).
4. **C4** `Sequencer: pause and abort wait until no stone is held; resume refused while the jaws may hold one; catch-all run_error` (H5, H6 + tests + the one `test_sequencer.py` line).
5. **C5** `FakePlc: READY + Manual request -> MANUAL; pattern A in one process (AdsWorker + AresAds) tested`.
6. **C6** `hmi.core: sessions, rigs, preflight, snapshots, sources, RunController; SIM end-to-end test` (`test_hmi_run_controller`, `real_rig`, `snapshot`).
7. **C7** `hmi: Mauer main window (tabs, ConfirmBar, HALT stops the run, run lock), Mauer tab, plan view, run log; feature stubs; py.exe -m hmi` (amr `ui/` run-lock edits, `test_hmi_gui_smoke`, `test_hmi_mauer_tab`).
8. **C8** `hmi/README.md` (usage, CLI, operating pattern A, HALT semantics, provenance).

**Exit criteria**
- `py.exe -m pytest -q tests` is green (except the opt-in markers).
- `py.exe -m hmi --job data/jobs/nominal_C.json` starts. Only with the copied asset and without `--ares`; it shows "ADS off".
- A SIM run of the short job completes in the GUI.

### FEATURES (three parallel branches from the `hmi-core` tip)

| Feature | Branch / worktree | Owns (creates or replaces) |
|---|---|---|
| CAMERA | `hmi-camera` | `hmi/core/overlay.py`, `hmi/views/camera_view.py`, `tests/test_hmi_camera.py` |
| STATUS | `hmi-status` | `hmi/views/ur_panel.py`, `hmi/views/ares_panel.py`, `hmi/views/feedback.py`, `hmi/views/status_strip.py`, `tests/test_hmi_status.py` |
| TWIN | `hmi-twin` | `robodk/twin_model.py`, `robodk/twin.py`, `hmi/core/twin_link.py`, `hmi/views/twin_panel.py`, `tests/test_twin_model.py`, `tests/test_twin_robodk.py` |

**There are NO shared file edits between features.** Features must not edit any CORE file: `hmi/main_window.py`, `hmi/core/{run_controller,snapshot,sources,context,session,rigs,preflight,config,ads_link}.py`, `hmi/views/{mauer_tab,plan_view,confirm_bar,log_view}.py`, `hmi/amr/**`, `config/station.toml`, `mauer/**`, `tests/conftest.py`, `tests/hmi_fakes.py`, `requirements.txt`, `README.md`, `docs/**`.
- Fakes a feature needs go into its own test file.
- If a CORE interface is missing, the feature adapts in its own files and lists the gap in its final report; INTEGRATE applies it.
- All `[hmi]` keys the features use (`camera_*`, `ur_poll_hz`, `[hmi.twin]`) already exist from C2.

### INTEGRATE (branch `hmi-integrate`, from the `hmi-core` tip)

1. Merge `hmi-camera`, `hmi-status` and `hmi-twin`. No conflicts are expected (disjoint files).
2. Apply the reported interface gaps.
3. Run the full suite, plus the opt-in `test_twin_robodk.py` (its own RoboDK instance on 20630+, closed afterwards).
4. Manual SIM smoke: `py.exe -m hmi --job data/jobs/nominal_C.json --twin`, SIM, scenario realistic, about 0.3 s per motion. Check: twin mirrors the arm, ARES and stones; Pause while a stone is held; Resume; HALT; close.
5. Add `hmi.cmd`.
6. Docs: `docs/ARCHITECTURE.md` gets an "HMI (hmi/)" section (modules, threads, ownership table 6, HALT table 8.3); `README.md` gets a short "Mauer HMI" paragraph.
7. Do NOT edit `docs/PLAN_2026-10-07_realtest.md` (the main session owns it). Report the B/C status and the job rebuild (5.2) instead.
8. Final checks: no `192.168` in the tests; lazy-import guard green; `git status` shows no assets.

---

## 14. Risks and open items

1. **Pattern A in ONE process is unproven on the robot.** E003 ran the AresAds equivalent as a separate process (`imu_cal_run.py`). The fake-PLC test covers the protocol and the GIL / heartbeat timing. Before the first wall run, the test plan must check with ARES jacked up: HMI with `--ares`, MANUAL, then a REAL step-mode move of 100 mm, then HALT. **Fallback, not implemented:** run `tools/run_job.py --real` as a subprocess and let the HMI monitor `run.jsonl`.
2. **PolyScope 3.3.3 has no `get_inverse_kin_has_solution`.** Every look / pick / place block fails until `mauer/ur/script.py` `ik_guard` gets a 3.3 variant. The HMI preflight shows this as a blocker; the fix is outside the HMI.
3. **Nominal jobs never pass `preflight_real`.** They carry `reach_check "kinematic"` (`sequencer.py:287`), so REAL Start stays disabled; there is no override by design. The test plan (track D) decides how a RoboDK-checked job is produced.
4. **`[hmi]` changes `config_sha256`.** Rebuild the jobs once after the merge (5.2).
5. **The held-stone guard (H5) changes one existing test.** That is intended: the sim confirmed the stone loss.
6. **RTDE went stale right after the IDS camera opened** (`link.py:559`). The rig keeps the `run_real` order (link first, camera last), and the UR panel shows the sample age.
7. **RoboDK** can crash with two Cam2D windows, exits when its last client disconnects (`-EXIT_LAST_COM`), and the user may close it. The twin opens no Cam2D, catches everything and never touches the run.
8. **In REAL, the twin and plan view show the sequencer's belief:** a nominal wall and the estimated ARES pose. This is labelled in the UI.
9. **An HMI ADS reconnect during a run writes the safe defaults.** MANUAL drops and the running move aborts (22). This is intended: the run ends with an ARES error, and the operator re-selects Manual before resuming.
10. **Run state is in memory only.** An HMI crash loses the resume ability; a fresh run of stop k assumes ARES at its start mark.

---

## 15. Implementation notes (CORE)

Where the CORE step (branch `hmi-core`) differs from the contract above, and why. Everything not listed here is
implemented as specified.

**Deviations**
1. `RunController.event` is `Signal(object)` (it still carries the record dict). PySide6 6.11 converts a
   `Signal(dict)` argument to a QVariantMap: a record with a non-string key arrives as `{}` ("Cannot copy-convert
   ... (dict)", checked 2026-10-07). `HmiContext.ads_status` stays `Signal(dict)` (amr status: string keys only).
2. `RunWorker` (thread `mauer-run`) is a daemon `threading.Thread` with a job queue, not a QThread: it needs no
   event loop, and a controller that is never shut down cannot crash the interpreter at exit. Signals emitted
   from it are queued into the GUI thread (PySide6 delivers bound-method AND lambda slots there - checked).
3. `RunController.can(action, mode=None)`: the optional `mode` is the mode the Mauer tab would prepare (radio
   buttons) - before Prepare the controller has no mode. `enables()` also knows "options", "sim_speed" and "halt".
4. Extra read-only members of `RunController`: `step`, `sim_step_s`, `ares_enabled`, `sequencer` (the current
   `Sequencer` for reading `held`, `pose_status`, `paused`), `log_dir`, `odom_moved()`. `ConfirmRequest.pause_pending`
   is also true for a pending soft abort.
5. `tests/fake_plc.py`: READY -> MANUAL on a RISING edge of the manual request. The PLC uses the level
   (FB_AMR_StateMachine.TcPOU:218), but `tests/test_ares_ads.py` forces READY (`amr_state = 6`) while the emulated
   HMI holds the request TRUE; a level rule would undo those tests. The pattern-A test gets the edge from the
   AdsWorker (safe defaults write FALSE, the Manual request TRUE).
6. **Additional sequencer change (H7)**, own commit: a direct move between stops (no route: straight / v1 jobs)
   now counts as a two-waypoint `RouteProgress` while it runs. Before, declining it in step mode (or a pause at its
   interlock, an ARES error) and resuming measured stop k from where ARES still stood - the re-aimed looks were
   unreachable (SIM: IK guard; REAL: an unchecked look pose). Found by the design's own step-mode test (8.2 / 12.2
   "Go except the first ARES -> Decline -> resume -> done"). The uninterrupted path is unchanged.
7. **RoboDK opt-in fix**, own commit: in `tests/test_robodk_l.py` and `tests/test_robodk_camera.py` the
   function-scoped autouse `_opt_in` skip ran AFTER the module fixture `rdk`, so a plain `pytest` run started a
   separate RoboDK (ports 20596 / 20598) before skipping. `_opt_in` is module-scoped now.
8. `closeEvent` refuses also while a job is loading (`BUSY` = loading, preparing, running, pausing, aborting,
   releasing); a second close of a closed window is accepted without a second shutdown.
9. `hmi/main.build()` starts the `NullAdsWorker` (it emits "ADS off"); `MainWindow` never starts a worker.
10. The heartbeat-under-load test runs the whole 10-stone straight test wall (34 stones, ~1.5 s): the 3-stone
    job finishes in well under a second at `sim_step_s = 0`.
11. Extra test file `tests/test_hmi_session.py` (job files load their own variant, refusals, build from config).

**What exists for the feature steps** (all in the GUI thread unless noted)
- `ctx` (`HmiContext`): `station_cfg`, `hmi` ([hmi] incl. the nested `frame` / `move` / `twin` tables), `amr_cfg`,
  `config_path`, `ares_enabled`, `start_twin` (`--twin`), `controller`, `last_ads_status` / `ads_connected`
  (readable from any thread), `session` (property), signals `session_changed(object)`, `ads_status(dict)`,
  `ads_connection(bool, str)`, `twin_state_changed(str, str)`; `set_twin_state(state, detail)`,
  `add_shutdown_hook(fn)`.
- `ctx.controller` (`RunController`): signals `state_changed(str, str)`, `event(object)`, `snapshot_changed(object)`,
  `shot(object)`, `confirm_requested(object)`, `confirm_cleared(int)`, `preflight_done(object)`,
  `rig_changed(object)`, `session_loaded(object)`, `message(str, str)`; properties `state`, `mode`, `session`, `opts`
  (`RunOptions`, e.g. `camera_loop`), `snapshot` (latest `RunSnapshot`, atomic reference - the twin thread may read
  it), `rig` (`RigInfo`: `mode`, `cfg`, `job`, `robot`, `link`, `ads`, `world`, `camera_kind`), `preflight`,
  `halted`, `pending`, `log_dir`; `can(action, mode=None)`; `grab()`; `set_shot_processor(fn)` (fn runs in
  `mauer-run`: `(image, shot record | None, session cfg) -> ShotView`); `ur_source()` (a new `UrSource` per consumer
  thread).
- `hmi.core.snapshot`: `RunSnapshot`, `StoneInfo`, `ShotView` (`full_size` = (width, height) px), `default_shot_view`,
  `describe_action`, `format_event`, `severity`, `category`, `boards_text`, `to_uint8`.
- `hmi.core.sources`: `UrSnapshot`, `UrSource`, `AresLive` (`src` also "none"), `ares_live(snap, ads_status, world)`,
  `odom_from_status`, `compose_odometry`.
- `hmi.views.plan_view`: `plan_geometry(job, cfg) -> PlanGeometry` (`stones` key -> polygon, `course`, `order`,
  `stops`, `routes` as (kind, waypoints), `obstacles` as (name, kind, polygon), `table`, `dock`, `ares`, `bounds`,
  `notes`); `PlanView.set_session / set_snapshot / set_live`.
- Tests: `tests/hmi_fakes.py` `FakeRunController` (`set_state(state, detail, mode=None)`, settable `sequencer`,
  `rig`, `snapshot`, `preflight`, `odom`; `of(name)` lists the recorded calls), `make_ctx`, `make_window`,
  `short_sim_session(n0, n1)`, `StubLink`, `StubCamera`, `wait_until`, `AMR_CFG`.
- Connect controller / context signals to bound methods of the widget (or lambdas): they are queued into the GUI
  thread; widgets never call hardware objects.
