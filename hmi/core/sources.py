"""Read-only adapters for the status panels and the twin (docs/HMI_DESIGN.md section 9).

UrSource: the UR state of the current rig - REAL from URLink.state() (lock-protected, D-H3: no second RTDE client),
SIM from the simulated robot. ares_live(): the best current ARES pose in the wall frame for display - the
sequencer's estimate, followed by the HMI's own odometry while a REAL move runs (AdsAres forwards no progress).
Nothing here writes to hardware.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Mapping

import numpy as np

from mauer.ares.ads import OdomPose
from mauer.job import Job
from mauer.reference import Pose2D, planar_T


# ── UR ────────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class UrSnapshot:
    source: str                         # "rtde" | "sim" | "none"
    t: float
    age_s: float | None                 # RTDE sample age (link.state_age_s())
    q_rad: tuple[float, ...] | None
    T_base_tcp_mm: np.ndarray | None    # URState.T_base_tcp_mm(); SIM: T_bf @ T_flange_tcp
    robot_mode: str | None              # mauer.ur.link ROBOT_MODE / SAFETY_MODE / RUNTIME_STATE names
    safety_mode: str | None
    runtime_state: str | None
    safety_ok: bool | None
    power_on: bool | None
    program_running: bool | None
    do_open: bool | None                # [ur] do_grip_open / do_grip_close outputs: pulses, NOT a held state
    do_close: bool | None
    tool_voltage_v: int | None
    payload_kg: float | None            # COMMANDED (URRobot tool_kg), no read-back
    payload_cog_mm: list | None
    held_kind: str | None               # SIM: kind of the stone in the jaws; REAL: unknown (None)
    parked: bool | None
    controller_version: tuple | None
    rtde_error: str | None
    missing_fields: tuple[str, ...]


def _none_snapshot(source: str, q=None, **kw) -> UrSnapshot:
    base = dict(source=source, t=time.time(), age_s=None, q_rad=None if q is None else tuple(float(v) for v in q),
                T_base_tcp_mm=None, robot_mode=None, safety_mode=None, runtime_state=None, safety_ok=None,
                power_on=None, program_running=None, do_open=None, do_close=None, tool_voltage_v=None,
                payload_kg=None, payload_cog_mm=None, held_kind=None, parked=None, controller_version=None,
                rtde_error=None, missing_fields=())
    base.update(kw)
    return UrSnapshot(**base)


class UrSource:
    """UR state for ONE consumer thread (it keeps the last joints as the IK seed in SIM). rig: RigInfo or None."""

    def __init__(self, rig, job: Job | None) -> None:
        self.rig, self.job = rig, job
        self._q_prev = None if job is None else np.asarray(job.park_q_rad, float)

    def snapshot(self) -> UrSnapshot:
        rig = self.rig
        if rig is None:
            return _none_snapshot("none", self._q_prev, parked=True if self._q_prev is not None else None)
        if rig.mode == "sim":
            return self._sim(rig)
        return self._real(rig)

    def _sim(self, rig) -> UrSnapshot:
        from mauer.simworld import ik_near
        r = rig.world.robot
        q = r.q
        if q is None:                   # SimRobot.q is None after a Cartesian move: nominal IK near the last joints
            q = ik_near(np.asarray(r.T_bf, float), self._q_prev if self._q_prev is not None else r.park_q)
        if q is not None:
            self._q_prev = np.asarray(q, float)
        h = r.holding                   # read once: the run thread may set it to None meanwhile
        held = rig.world.kind_of.get(h[0], "full") if h is not None else None
        return _none_snapshot("sim", self._q_prev, T_base_tcp_mm=np.asarray(r.T_bf, float) @ r.T_flange_tcp,
                              robot_mode="RUNNING", safety_mode="NORMAL", safety_ok=True, power_on=True,
                              program_running=False, held_kind=held, parked=bool(r.parked))

    def _real(self, rig) -> UrSnapshot:
        from mauer.ur.link import ROBOT_MODE, RUNTIME_STATE, SAFETY_MODE, SAFETY_OK
        link, robot = rig.link, rig.robot
        if link is None:
            return _none_snapshot("rtde", rtde_error="URLink not started")
        st, age = link.state(), link.state_age_s()
        u = rig.cfg.get("ur", {}) if rig.cfg else {}
        payload = (float(robot.tool_kg), list(robot.tool_cog)) if robot is not None and hasattr(robot, "tool_kg") \
            else (None, None)
        common = dict(age_s=None if not math.isfinite(age) else float(age),
                      controller_version=getattr(link, "controller_version", None),
                      rtde_error=getattr(link, "rtde_error", None),
                      missing_fields=tuple(getattr(link, "missing_fields", ()) or ()),
                      payload_kg=payload[0], payload_cog_mm=payload[1])
        if st is None:
            return _none_snapshot("rtde", **common)
        parked = None
        if robot is not None and hasattr(robot, "park_q"):
            parked = bool(float(np.max(np.abs(np.asarray(st.actual_q, float) - robot.park_q))) <=
                          getattr(robot, "park_tol_rad", 0.01))
        q = np.asarray(st.actual_q, float)
        self._q_prev = q
        return _none_snapshot(
            "rtde", q, T_base_tcp_mm=st.T_base_tcp_mm(), robot_mode=ROBOT_MODE.get(st.robot_mode, str(st.robot_mode)),
            safety_mode=SAFETY_MODE.get(st.safety_mode, str(st.safety_mode)),
            runtime_state=RUNTIME_STATE.get(st.runtime_state, str(st.runtime_state)),
            safety_ok=st.safety_mode in SAFETY_OK, power_on=st.power_on, program_running=st.program_running,
            do_open=st.digital_out(u["do_grip_open"]) if "do_grip_open" in u else None,
            do_close=st.digital_out(u["do_grip_close"]) if "do_grip_close" in u else None,
            tool_voltage_v=st.tool_output_voltage, parked=parked, **common)


# ── ARES ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class AresLive:
    pose: Pose2D | None                 # best current wall-frame pose for display
    src: str                            # "estimate" | "estimate+odometry" | "sim truth" | "start mark" | "none"
    status: str                         # seq.pose_status
    true_pose: Pose2D | None            # SIM: world.ares_true
    odom: OdomPose | None               # REAL: HMI ADS status; SIM: world.ares.odom


def odom_from_status(ads_status: Mapping | None) -> OdomPose | None:
    """PLC odometry pose of an AdsWorker status dict (fPosX_m, fPosY_m in m -> mm, fPosTheta_deg)."""
    if not ads_status or "fPosX_m" not in ads_status:
        return None
    return OdomPose(float(ads_status["fPosX_m"]) * 1000.0, float(ads_status.get("fPosY_m", 0.0)) * 1000.0,
                    float(ads_status.get("fPosTheta_deg", 0.0)))


def compose_odometry(pose: Pose2D, before: OdomPose, after: OdomPose) -> Pose2D:
    """pose moved by the odometry displacement before -> after (body frame of `before`, as
    Sequencer._from_odometry): pose (+) delta."""
    dx, dy, dth = before.delta_to(after)
    return Pose2D.from_T(pose.T @ planar_T(dx, dy, math.radians(dth)))


def ares_live(snap, ads_status: Mapping | None, world=None) -> AresLive:
    """REAL during an ARES command (snap.ares_cmd with odometry): the command's start pose (+) the odometry since;
    otherwise the sequencer's estimate. SIM: also the true pose and the simulated odometry."""
    true_pose = getattr(world, "ares_true", None) if world is not None else None
    odom = world.ares.odom if world is not None else odom_from_status(ads_status)
    if snap is None:
        if true_pose is not None:
            return AresLive(true_pose, "sim truth", "ok", true_pose, odom)
        return AresLive(None, "none", "unknown", None, odom)
    cmd = snap.ares_cmd
    if world is None and cmd and cmd.get("odom") is not None and cmd.get("pose") is not None and odom is not None:
        return AresLive(compose_odometry(cmd["pose"], cmd["odom"], odom), "estimate+odometry", snap.pose_status,
                        None, odom)
    src = "start mark" if str(snap.pose_src).startswith(("assumed", "start mark")) else "estimate"
    return AresLive(snap.pose_est, src, snap.pose_status, true_pose, odom)
