"""Rigs: the backends a run uses (docs/HMI_DESIGN.md sections 6 and 8.1).

SimRig = mauer.simworld (as tools/run_job.py sim_once). RealRig = URLink + AresAds + IDS camera + calibration (as
tools/run_job.py run_real, same order: link first, camera last - RTDE went stale right after the camera opened once,
mauer/ur/link.py). Nothing here connects on import or construction: RealRig.open() is called by the RunController
in the run thread after Prepare / Connect in REAL mode, never by tests (they inject RealFactories fakes).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable, Mapping

import numpy as np

from mauer import config as mconfig
from mauer.job import Job

log = logging.getLogger("hmi.rigs")


@dataclass(frozen=True)
class RigInfo:
    """What the GUI and the read-only adapters may look at (never call into the hardware objects except
    URLink.state() / state_age_s(), which are lock-protected)."""
    mode: str                  # "sim" | "real"
    cfg: Mapping
    job: Job
    robot: Any | None          # SimRobot | URRobot
    link: Any | None           # URLink (REAL)
    ads: Any | None            # AresAds (REAL)
    world: Any | None          # SimWorld (SIM)
    camera_kind: str | None    # "synth" | "ids"


# ── default factories (lazy imports: pyads, ids_peak and the RTDE client only load when REAL is used) ─────────────
def _link(cfg: dict) -> Any:
    from mauer.ur.link import URLink
    return URLink.from_config(cfg)


def _ads(cfg: dict) -> Any:
    from mauer.ares.ads import AresAds
    return AresAds(cfg["ares_ads"])


def _camera(cfg: dict) -> Any:
    from mauer.camera import open_camera
    return open_camera(cfg)


def _robot(link: Any, cfg: dict, job: Job) -> Any:
    """URRobot with the motion guard the real robot requires (as tools/run_job.py run_real)."""
    from mauer.backends import URRobot
    from mauer.motionguard import MotionGuard
    return URRobot(link, cfg, job, guard=MotionGuard(cfg, job.T_ares_base, job.park_q_rad, job.T_flange_tcp))


def _intrinsics(cfg: dict) -> Any:
    from mauer.vision import intrinsics
    return intrinsics.load(mconfig.repo_path(cfg["vision"]["intrinsics_file"]))


def _handeye(cfg: dict) -> np.ndarray:
    from mauer.vision import handeye
    return handeye.load(mconfig.repo_path(cfg["vision"]["handeye_file"])).T_flange_cam


@dataclass
class RealFactories:
    """How RealRig creates its parts; every default imports lazily, tests inject fakes."""
    link: Callable[[dict], Any] = _link                      # URLink.from_config(cfg) (the rig calls .start())
    ads: Callable[[dict], Any] = _ads                        # AresAds(cfg["ares_ads"]) (the rig calls .connect())
    camera: Callable[[dict], Any] = _camera                  # mauer.camera.open_camera(cfg) (opened)
    robot: Callable[[Any, dict, Job], Any] = _robot          # URRobot(link, cfg, job, guard=MotionGuard(...))
    intrinsics: Callable[[dict], Any] = _intrinsics          # [vision] intrinsics_file
    handeye: Callable[[dict], np.ndarray] = _handeye         # [vision] handeye_file -> T_flange_cam


def _err(e: BaseException) -> str:
    return f"{type(e).__name__}: {e}"


class SimRig:
    """The pure-Python simulated world (mauer.simworld), as tools/run_job.py sim_once."""
    mode = "sim"

    def __init__(self, cfg: dict, job: Job, opts) -> None:
        from mauer.simworld import SimWorld, scenario
        self.cfg, self.job = cfg, job
        self.world = SimWorld(cfg, job, scenario(opts.scenario), seed=int(opts.seed), start_stop=int(opts.start_stop),
                              supersample=2, grasp_check=not opts.lenient_grasp)
        self.robot, self.ares, self.camera = self.world.robot, self.world.ares, self.world.camera
        self.intr, self.T_flange_cam = self.world.intr, self.world.T_flange_cam
        self.link = None
        self.errors: dict[str, str] = {}

    @property
    def complete(self) -> bool:
        return True

    def info(self) -> RigInfo:
        return RigInfo("sim", self.cfg, self.job, self.robot, None, None, self.world, "synth")

    def close(self) -> None:
        try:
            self.camera.close()
        except Exception as e:          # noqa: BLE001 - closing is best effort
            log.warning("sim camera close: %s", e)


class RealRig:
    """URLink + AresAds + camera + calibration. open() never raises: each failure is kept in `errors` (part ->
    text) and that part stays None; real_preflight() turns them into blocking items."""
    mode = "real"

    def __init__(self, cfg: dict, job: Job, factories: RealFactories | None = None) -> None:
        self.cfg, self.job = cfg, job
        self.f = factories or RealFactories()
        self.link: Any = None
        self.ads: Any = None
        self.camera: Any = None
        self.robot: Any = None
        self.intr: Any = None
        self.T_flange_cam: np.ndarray | None = None
        self.ads_check: Any = None                 # mauer.ares.ads.Preflight of the last check_ads()
        self.errors: dict[str, str] = {}
        self.closed: list[str] = []                # close order (tests)

    @property
    def complete(self) -> bool:
        return all(x is not None for x in (self.link, self.ads, self.camera, self.robot, self.intr,
                                           self.T_flange_cam))

    def info(self) -> RigInfo:
        return RigInfo("real", self.cfg, self.job, self.robot, self.link, self.ads, None,
                       "ids" if self.camera is not None else None)

    def open(self) -> None:
        """link.start() -> ads.connect() -> ads.check() -> camera -> robot -> calibration (tools/run_job.py run_real
        order). Blocks for seconds (RTDE start, TCP probe, 250 ms heartbeat window): run thread only."""
        link = None
        try:
            link = self.f.link(self.cfg)
            link.start()
            self.link = link
        except Exception as e:          # noqa: BLE001 - every failure becomes a preflight item
            self.errors["link"] = _err(e)
            if link is not None:
                try:
                    link.stop()
                except Exception:       # noqa: BLE001
                    pass
        a = None
        try:
            a = self.f.ads(self.cfg)
            a.connect()
            self.ads = a
        except Exception as e:          # noqa: BLE001
            self.errors["ads"] = _err(e)
            if a is not None:
                try:
                    a.close()
                except Exception:       # noqa: BLE001
                    pass
        self.check_ads()
        try:
            self.camera = self.f.camera(self.cfg)
        except Exception as e:          # noqa: BLE001
            self.errors["camera"] = _err(e)
        if self.link is not None:
            try:
                self.robot = self.f.robot(self.link, self.cfg, self.job)
            except Exception as e:      # noqa: BLE001 - unknown payload / stone mass (URRobot refuses)
                self.errors["robot"] = _err(e)
        try:
            self.intr = self.f.intrinsics(self.cfg)
        except Exception as e:          # noqa: BLE001
            self.errors["intrinsics"] = _err(e)
        try:
            self.T_flange_cam = np.asarray(self.f.handeye(self.cfg), float)
        except Exception as e:          # noqa: BLE001
            self.errors["handeye"] = _err(e)

    def check_ads(self) -> None:
        """The AresAds preflight (two status reads 250 ms apart, no write)."""
        self.ads_check = None
        if self.ads is not None:
            try:
                self.ads_check = self.ads.check()
            except Exception as e:      # noqa: BLE001
                self.errors["ads"] = _err(e)

    def halt(self, ads_worker_connected: bool) -> str:
        """HALT helper (thread mauer-halt, D-H8 step 3): stop the robot program (URLink.abort: stopl program +
        Dashboard stop, up to ~2 s); the AresAds abort pulse only as a fallback when the HMI's ADS worker (which
        sent HALT first) is not connected. Never raises; returns what was done."""
        done = []
        try:
            if self.robot is not None:
                done.append(f"UR: {self.robot.abort()}")
            elif self.link is not None:
                done.append(f"UR: {self.link.abort()}")
        except Exception as e:          # noqa: BLE001
            done.append(f"UR abort failed: {e}")
        if not ads_worker_connected and self.ads is not None:
            try:
                self.ads.abort()
                done.append("ARES: bCmdMoveAbort pulse (AresAds, ADS worker not connected)")
            except Exception as e:      # noqa: BLE001
                done.append(f"ARES abort failed: {e} - use the E-stop")
        return "; ".join(done) or "nothing to stop"

    def close(self) -> None:
        """camera.close(), ads.close(), link.stop() (the controller closes the Sequencer first)."""
        for name, part, method in (("camera", self.camera, "close"), ("ads", self.ads, "close"),
                                   ("link", self.link, "stop")):
            if part is None:
                continue
            try:
                getattr(part, method)()
            except Exception as e:      # noqa: BLE001 - closing is best effort
                log.warning("%s %s: %s", name, method, e)
            self.closed.append(name)
        self.camera = self.ads = self.link = self.robot = None
