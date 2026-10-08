"""Preflight lists of the HMI (docs/HMI_DESIGN.md section 8.1).

SIM: the real-run problems of mauer.sequencer.preflight_real, informative only. REAL: every item blocks, there is no
override (as tools/run_job.py --real): the job / config checks, the HMI's own ADS worker (pattern A: heartbeat,
MANUAL), the sequencer's AresAds preflight, the UR state over RTDE, the camera and the calibration files.

preflight_real reads config/station.toml from disk, the run uses the config loaded with the job (JobSession.cfg):
config_drift() adds the item that the two differ (review 2026-10-08).
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from mauer.ares.ads import AMR_STATE_NAMES, ST_MANUAL
from mauer.job import Job, config_sha256
from mauer.sequencer import preflight_real

SOURCES = ("job/config", "HMI", "ARES", "UR", "camera")
RTDE_MAX_AGE_S = 0.5                 # = URRobot.is_idle(): an older sample means the RTDE stream is stale
IK_GUARD_33 = ("PolyScope 3.3: get_inverse_kin_has_solution missing - every look / pick / place block fails "
               "(mauer/ur/script.py ik_guard needs a 3.3 variant; station.toml [ur] host comment)")


@dataclass(frozen=True)
class PreflightItem:
    source: str                      # "job/config" | "HMI" | "ARES" | "UR" | "camera"
    text: str
    blocking: bool


@dataclass(frozen=True)
class PreflightReport:
    mode: str                        # "sim" | "real"
    items: tuple[PreflightItem, ...]
    t: float                         # time.time() of the check

    @property
    def ok(self) -> bool:
        return not any(i.blocking for i in self.items)

    @property
    def n_blocking(self) -> int:
        return sum(i.blocking for i in self.items)

    def header(self) -> str:
        if self.mode == "sim":
            return f"SIM (informative): {len(self.items)} real-run problems"
        return "REAL preflight ok" if self.ok else f"REAL refused: {self.n_blocking} problems"


def config_drift(cfg: Mapping, config_path: str | Path | None, session_sha256: str | None) -> list[str]:
    """[problem] if the config on disk is no longer the one parsed into the session (JobSession.config_sha256); []
    for a session without a hash (tests)."""
    if not session_sha256:
        return []
    variant = cfg.get("_variant") or None
    try:
        disk = config_sha256(config_path, variant)
    except OSError as e:
        return [f"config not readable: {e}"]
    if disk == session_sha256:
        return []
    return ["config/station.toml" + (f" + variants/{variant}.toml" if variant else "") + " changed on disk since the "
            "job was loaded - the run would use the values loaded then: Release, load the job again"]


def sim_preflight(cfg: Mapping, job: Job, config_path: str | Path | None = None,
                  session_sha256: str | None = None) -> PreflightReport:
    """The real-run problems of the job and config, non-blocking (the simulation runs anyway)."""
    items = tuple(PreflightItem("job/config", p, False)
                  for p in [*config_drift(cfg, config_path, session_sha256),
                            *preflight_real(cfg, job, config_path=config_path)])
    return PreflightReport("sim", items, time.time())


def real_preflight(cfg: Mapping, job: Job, config_path: str | Path | None = None, *, ares_enabled: bool,
                   ads_connected: bool, ads_status: Mapping[str, Any] | None, rig,
                   session_sha256: str | None = None) -> PreflightReport:
    """Every reason not to start the REAL run, in the order of the design (8.1); `rig` is the opened RealRig (or None
    before Connect). Reads the rig's last AresAds check (RealRig.check_ads) - no ADS call here. session_sha256: the
    hash of the config the run uses (JobSession.config_sha256)."""
    items: list[PreflightItem] = []

    def add(source: str, text: str) -> None:
        items.append(PreflightItem(source, text, True))

    for p in config_drift(cfg, config_path, session_sha256):
        add("job/config", p)
    for p in preflight_real(cfg, job, config_path=config_path):
        add("job/config", p)
    # the HMI's own ADS worker (pattern A: it owns heartbeat, MANUAL and HALT)
    if not ares_enabled:
        add("HMI", "HMI started without --ares: no ADS worker (heartbeat, MANUAL, HALT) - restart with --ares")
    elif not ads_connected or not ads_status:
        add("HMI", "the HMI's ADS worker is not connected to the ARES PLC (ARES control tab)")
    else:
        if int(ads_status.get("nIfVersion", 0) or 0) < 2:
            add("HMI", "PLC interface is not v2 (no relative move) - load PLC build >= v2.1")
        state = int(ads_status.get("eAmrState", -1))
        if state != ST_MANUAL:
            add("HMI", f"AMR state {state} {AMR_STATE_NAMES.get(state, '?')}, needs 7 MANUAL MODE: ARES control tab, "
                       "Safety Run -> AMR Reset -> Start -> Manual")
        if ads_status.get("bExtActive"):
            add("HMI", "external control (C6030) active - stop the C6030 / ROS bridge")
    errors = getattr(rig, "errors", {}) if rig is not None else {}
    # ARES: the sequencer's own connection (move fields only)
    if rig is None:
        add("ARES", "not connected (Prepare / Connect in REAL mode)")
    elif rig.ads is None:
        add("ARES", f"AresAds not connected: {errors.get('ads', 'not opened')}")
    else:
        chk = rig.ads_check
        for p in (chk.problems if chk is not None else ["AresAds preflight not run"]):
            add("ARES", p)
    # UR over RTDE
    if rig is None:
        add("UR", "not connected (Prepare / Connect in REAL mode)")
    elif rig.link is None:
        add("UR", f"URLink not started: {errors.get('link', 'not opened')}")
    else:
        from mauer.ur.link import ROBOT_MODE, ROBOT_RUNNING, SAFETY_MODE, SAFETY_OK
        st, age = rig.link.state(), rig.link.state_age_s()
        if st is None or not age <= RTDE_MAX_AGE_S:
            add("UR", f"no current RTDE sample (age {age:.1f} s) - RTDE stream stale or stopped")
        else:
            if st.robot_mode != ROBOT_RUNNING:
                add("UR", f"robot mode {ROBOT_MODE.get(st.robot_mode, st.robot_mode)}, needs RUNNING: power on + "
                          "brake release on the pendant")
            if st.safety_mode not in SAFETY_OK:
                add("UR", f"safety mode {SAFETY_MODE.get(st.safety_mode, st.safety_mode)}: clear the stop on the "
                          "pendant")
            if st.program_running:
                add("UR", "a program is running on the controller - stop it on the pendant")
        v = getattr(rig.link, "controller_version", None)
        if v and tuple(v[:2]) == (3, 3):
            add("UR", IK_GUARD_33)
        if rig.robot is None:
            add("UR", f"URRobot not created: {errors.get('robot', 'no URLink')}")
    # camera and calibration
    if rig is not None:
        if rig.camera is None:
            add("camera", f"camera not open: {errors.get('camera', 'not opened')}")
        if rig.intr is None:
            add("camera", f"camera intrinsics not loaded: {errors.get('intrinsics', '?')}")
        if rig.T_flange_cam is None:
            add("camera", f"hand-eye calibration not loaded: {errors.get('handeye', '?')}")
    return PreflightReport("real", tuple(items), time.time())
