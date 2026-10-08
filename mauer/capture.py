"""Robot-side image capture: the flange pose at the exposure, look-pose moves, boards in the UR base frame.

Used by the hand-eye workflow (tools/calib_handeye.py), the measurement tools (tools/measure_target.py) and the
sequencer. Conventions (docs/ARCHITECTURE.md): T_a_b = pose of frame b in frame a, 4x4 float64, mm and rad; UR poses
[m, rad rotation vector] only at the URScript/RTDE boundary.

    res = goto_look(link, cfg, T_base_flange=T, qnear_rad=q)      # one run_block: set_tcp + set_payload + movej
    shot = capture_shot(link, camera, settle_s=cfg["camera"]["settle_s"], max_qd=cfg["vision"]["max_qd_rad_s"])
    seen = boards_in_base(shot, specs, intr, cfg["vision"], T_flange_cam)   # {name: (BoardPose, T_base_board|None)}

Time stamps - what is used and how precise it is:
- `Frame.t_start` / `t_end` (mauer.camera): laptop time.time() just before the software trigger and just after the
  finished buffer arrived. The exposure lies inside; for the IDS GV-51F0CP the window also holds the ~45 ms GigE
  transfer of a Mono8 frame (ASSUMPTION from 5.1 MB at ~120 MB/s, mauer/camera/base.py).
- `URState.t_laptop` (mauer.ur.link): laptop time.time() when the RTDE thread received the sample (125 Hz, CB3). The
  controller state it carries is older by the network + thread latency (a few ms, not measured).
- capture_shot takes T_base_flange as the mean (geometry.average_T) of ALL RTDE samples received inside
  [t_start, t_end]; if none fell inside (window shorter than the 8 ms sample period, e.g. a replayed frame with a
  5 ms synthetic window), the nearest sample before t_start and the first after t_end, each at most `max_gap_s`
  (50 ms) away. The window is NOT narrowed to the exposure: the camera's `meta["system_timestamp_ns"]` (IDS
  host-correlated buffer time stamp) is recorded in the Shot (`t_camera`) but its reference event (exposure start or
  buffer completion) is not verified on hardware yet. Instead the robot must stand still over the whole window:
  `max_qd` (max |actual_qd| of the window samples) <= the limit, else the shot is retried. Worst case at the limit
  ([vision] max_qd_rad_s = 0.01 rad/s, ASSUMPTION): 0.01 rad/s x 50 ms window x 1 m lever = 0.5 mm drift across the
  window, i.e. <= 0.25 mm between the window mean and the exposure; after is_steady() + settle the joints are
  normally far below the limit (URSim: exactly 0).

Payload: every look block sets the payload from [ur] payload_tool_kg / payload_cog_mm. 0 kg = unknown (PLACEHOLDER):
goto_look refuses unless sim=True, which uses SIM_PAYLOAD_KG and logs a warning - URSim and simulations only.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from . import config
from . import geometry as g
from .camera.base import Camera, Frame
from .ur import script
from .vision.detect import BoardPose, measure
from .vision.intrinsics import Intrinsics
from .vision.targets import BoardSpec

log = logging.getLogger("mauer.capture")

MAX_GAP_S = 0.05            # nearest-sample fallback: at most 50 ms before t_start / after t_end (contract)
RETRY_WAIT_S = 0.5          # ASSUMPTION: wait before the next attempt when the arm was moving
SIM_PAYLOAD_KG = 1.0        # ASSUMPTION, URSim/simulation ONLY: valid set_payload while [ur] payload_tool_kg is unknown
MAX_STATE_AGE_S = 0.5       # RTDE sample older than this = link not streaming (same limit as URLink.run_block)
_SIM_PAYLOAD_WARNED: set[float] = set()


class CaptureError(RuntimeError):
    """No usable shot (robot moving, no RTDE samples for the frame, no RTDE state) or a refused look move.

    `attempts` lists the reason of every failed attempt; `shot` is the last shot taken (None if no frame matched),
    e.g. for diagnostics of a moving arm."""

    def __init__(self, msg: str, attempts: Sequence[str] = (), shot: "Shot | None" = None):
        super().__init__(msg)
        self.attempts = list(attempts)
        self.shot = shot


@dataclass
class Shot:
    """One image with the robot state during its exposure window (see the module docstring for the time stamps)."""
    frame: Frame
    T_base_flange: np.ndarray          # 4x4 [mm], mean of the RTDE flange poses matched to the frame
    q_rad: np.ndarray                  # (6,) [rad], mean of actual_q over the same samples
    tcp_pose_ur: list[float]           # active TCP [m, rad rotation vector], mean pose of the same samples
    max_qd: float                      # max |actual_qd| over the samples [rad/s]
    n_samples: int                     # RTDE samples used
    match: str = "window"              # "window" (samples inside [t_start, t_end]) or "nearest" (fallback)
    attempts: int = 1                  # grabs needed (1 = first try)
    spread_mm: float = 0.0             # max deviation of a matched flange pose from the mean [mm]
    spread_deg: float = 0.0            # same for the orientation [deg]
    t_pose: float = float("nan")       # mean receipt time of the matched samples (laptop time.time()) [s]
    t_camera: float | None = None      # meta system_timestamp_ns as time.time() seconds, if the camera gives it
    errors: list[str] = field(default_factory=list)   # reasons of the failed attempts before this one

    def dataset_fields(self) -> dict[str, Any]:
        """Keyword arguments for vision.dataset.Dataset.add(image, **fields) (the image is self.frame.image)."""
        m = self.frame.meta
        keep = {k: m[k] for k in ("camera", "exposure_us", "gain", "frame_id", "timestamp_ns", "system_timestamp_ns",
                                  "frame_id_gap", "kind", "T_base_flange_true") if k in m}
        return {"t_start": self.frame.t_start, "t_end": self.frame.t_end, "T_base_flange": self.T_base_flange,
                "q_rad": self.q_rad.tolist(), "tcp_pose_ur": list(self.tcp_pose_ur), "max_qd": self.max_qd,
                "n_samples": self.n_samples, "match": self.match, "attempts": self.attempts,
                "pose_spread_mm": self.spread_mm, "pose_spread_deg": self.spread_deg, "t_pose": self.t_pose,
                "frame_meta": keep}


# ── sample matching ───────────────────────────────────────────────────────────
def match_samples(link, t0: float, t1: float, max_gap_s: float = MAX_GAP_S) -> tuple[list, str]:
    """RTDE samples for the window [t0, t1] (laptop time): (samples, "window") if any were received inside,
    else ([last before t0, first after t1] - those within max_gap_s, "nearest"), ([], "none") if there is none.

    Waits up to max_gap_s for the first sample after the window (the RTDE thread may not have received it yet).
    `link` needs samples(t0, t1); wait_until(predicate, timeout_s) is used when present."""
    inside = list(link.samples(t0, t1))
    if inside:
        return inside, "window"
    before = [s for s in link.samples(t0 - max_gap_s, t0) if s.t_laptop < t0]
    after = [s for s in link.samples(t1, t1 + max_gap_s) if s.t_laptop > t1]
    if not after and hasattr(link, "wait_until"):
        # returns at once when a later sample is already there (also for an old replayed window)
        link.wait_until(lambda s: s.t_laptop > t1, max_gap_s)
        after = [s for s in link.samples(t1, t1 + max_gap_s) if s.t_laptop > t1]
    picked = ([before[-1]] if before else []) + ([after[0]] if after else [])
    return picked, ("nearest" if picked else "none")


def _flange(link, s) -> np.ndarray:
    return np.asarray(link.flange_T(s) if hasattr(link, "flange_T") else s.T_base_flange_mm(), float)


def _t_camera(meta: Mapping) -> float | None:
    ns = meta.get("system_timestamp_ns")
    try:
        return None if ns is None else float(ns) * 1e-9
    except (TypeError, ValueError):
        return None


def shot_from_samples(link, frame: Frame, samples: Sequence, match: str, attempts: int = 1) -> Shot:
    """Shot of a frame from its matched RTDE samples (mean flange/TCP pose, mean q, max |qd|, pose spread)."""
    if not samples:
        raise ValueError("shot_from_samples needs at least one sample")
    Tf = [_flange(link, s) for s in samples]
    T_mean = g.average_T(Tf)
    d = np.array([g.pose_delta(T_mean, T) for T in Tf])
    T_tcp = g.average_T([g.ur_to_T(s.actual_TCP_pose) for s in samples])
    q = np.mean([np.asarray(s.actual_q, float) for s in samples], axis=0)
    qd = max(float(np.max(np.abs(np.asarray(s.actual_qd, float)))) for s in samples)
    return Shot(frame=frame, T_base_flange=T_mean, q_rad=q, tcp_pose_ur=g.T_to_ur(T_tcp), max_qd=qd,
                n_samples=len(samples), match=match, attempts=attempts, spread_mm=float(d[:, 0].max()),
                spread_deg=float(d[:, 1].max()), t_pose=float(np.mean([s.t_laptop for s in samples])),
                t_camera=_t_camera(frame.meta))


# ── capture ───────────────────────────────────────────────────────────────────
def capture_shot(link, camera: Camera, settle_s: float = 0.0, max_qd: float = 0.01, retries: int = 2, *,
                 retry_wait_s: float = RETRY_WAIT_S, max_gap_s: float = MAX_GAP_S,
                 sleep: Callable[[float], None] = time.sleep) -> Shot:
    """Wait settle_s [s], check that the arm stands still (latest RTDE |actual_qd| <= max_qd [rad/s]), grab, match
    the RTDE samples of the exposure window and check them too. A moving arm or a frame without RTDE samples is
    retried up to `retries` times (after retry_wait_s); then CaptureError (with the reasons and the last shot).

    `link`: a mauer.ur.link.URLink (or anything with state(), samples(t0, t1), flange_T(sample); wait_until and
    state_age_s are used when present). `camera`: any mauer.camera.Camera whose Frame times are laptop time.time().
    """
    if settle_s > 0.0:
        sleep(float(settle_s))
    errors: list[str] = []
    last: Shot | None = None
    n_try = int(retries) + 1
    for attempt in range(1, n_try + 1):
        if attempt > 1 and retry_wait_s > 0.0:
            sleep(float(retry_wait_s))
        st = link.state()
        if st is None:
            raise CaptureError("no RTDE state - is the link started?", errors)
        age = float(link.state_age_s()) if hasattr(link, "state_age_s") else 0.0
        if age > MAX_STATE_AGE_S:
            errors.append(f"attempt {attempt}: RTDE state {age:.2f} s old (link not streaming)")
            continue
        qd_now = float(np.max(np.abs(np.asarray(st.actual_qd, float))))
        if qd_now > max_qd:
            errors.append(f"attempt {attempt}: arm moving before the grab (max |qd| {qd_now:.4f} > {max_qd} rad/s)")
            continue
        frame = camera.grab()
        samples, how = match_samples(link, frame.t_start, frame.t_end, max_gap_s)
        if not samples:
            errors.append(f"attempt {attempt}: no RTDE sample within {1000 * max_gap_s:.0f} ms of the frame window "
                          f"[{frame.t_start:.3f}, {frame.t_end:.3f}] (link stalled, or frame times not laptop "
                          "time.time() - e.g. a replayed recording)")
            continue
        last = shot_from_samples(link, frame, samples, how, attempt)
        last.errors = list(errors)
        if last.max_qd > max_qd:
            errors.append(f"attempt {attempt}: arm moved during the exposure window (max |qd| {last.max_qd:.4f} > "
                          f"{max_qd} rad/s, {last.n_samples} samples)")
            continue
        if errors:
            log.info("shot after %d attempts: %s", attempt, "; ".join(errors))
        return last
    raise CaptureError(f"no usable shot after {n_try} attempt(s): " + "; ".join(errors), errors, last)


# ── look moves ────────────────────────────────────────────────────────────────
def payload(cfg: Mapping, *, sim: bool = False) -> tuple[float, list[float]]:
    """(payload kg, CoG mm in the flange frame) for set_payload from [ur]. payload_tool_kg <= 0 means unknown
    (PLACEHOLDER): CaptureError for the real robot; with sim=True the logged fallback SIM_PAYLOAD_KG (or [ur]
    sim_payload_kg if present) - for URSim / simulation only."""
    u = cfg.get("ur", {})
    kg = float(u.get("payload_tool_kg", 0.0) or 0.0)
    cog = [float(v) for v in u.get("payload_cog_mm", [0.0, 0.0, 0.0])]
    if kg > 0.0:
        return kg, cog
    if not sim:
        raise CaptureError("[ur] payload_tool_kg is 0 (unknown PLACEHOLDER: gripper + camera + bracket) - refusing "
                           "to move the real robot with a wrong payload; weigh the tool and set it in "
                           "config/station.toml (sim=True / --sim only for URSim)")
    kg = float(u.get("sim_payload_kg", SIM_PAYLOAD_KG))
    if kg not in _SIM_PAYLOAD_WARNED:           # once per process and value, not once per block
        _SIM_PAYLOAD_WARNED.add(kg)
        log.warning("SIMULATION payload fallback: [ur] payload_tool_kg is unknown (0), using %.2f kg in every block "
                    "- never on the real robot", kg)
    return kg, cog


def look_block(cfg: Mapping, *, T_base_flange: np.ndarray | None = None, q_rad: Sequence[float] | None = None,
               qnear_rad: Sequence[float] | None = None, sim: bool = False,
               speeds: script.Speeds | None = None, reg_error: int = script.REG_ERROR) -> str:
    """URScript body of a look move: preamble (set_tcp = config T_flange_tcp, set_payload from [ur]) + an IK-guarded
    movej to T_base_flange (IK near qnear_rad, required here) or a movej to q_rad. Pure function."""
    if (T_base_flange is None) == (q_rad is None):
        raise ValueError("give exactly one of T_base_flange / q_rad")
    sp = speeds or script.Speeds.from_config(dict(cfg))
    T_ft = config.T_flange_tcp(dict(cfg))
    kg, cog = payload(cfg, sim=sim)
    lines = [script.preamble(T_ft, kg, cog)]
    if T_base_flange is not None:
        if qnear_rad is None:
            raise ValueError("look_block with T_base_flange needs qnear_rad")
        lines.append(script.look_pose(T_base_flange, qnear_rad, sp.a_joint, sp.v_joint, target="flange",
                                      T_flange_tcp=T_ft, reg_error=reg_error,
                                      ik_check=str(cfg["ur"].get("ik_check", "has_solution"))))
    else:
        lines.append(script.movej_q(q_rad, sp.a_joint, sp.v_joint))
    return "\n".join(lines)


def goto_look(link, cfg: Mapping, *, T_base_flange: np.ndarray | None = None, q_rad: Sequence[float] | None = None,
              qnear_rad: Sequence[float] | None = None, timeout_s: float = 60.0, sim: bool = False,
              speeds: script.Speeds | None = None, settle_s: float = 0.0, name: str = "look"):
    """Move to a look pose with ONE run_block: preamble (set_tcp from config T_flange_tcp, payload from [ur]) and
    script.look_pose (T_base_flange [mm], IK on the controller near qnear_rad - default: the current joints) or
    script.movej_q (q_rad). Speeds from [ur] unless `speeds` is given. Returns the link's BlockResult (ok False with
    error_code script.ERR_IK_UNREACHABLE when the pose has no IK solution - the arm did not move; the guard also opens
    a PolyScope popup, close it with Dashboard.close_popup()).

    Raises CaptureError when [ur] payload_tool_kg is unknown (0) and sim is False, or when qnear is needed but there
    is no RTDE state. The camera settle time belongs to capture_shot(settle_s=...)."""
    if T_base_flange is not None and qnear_rad is None:
        st = link.state()
        if st is None:
            raise CaptureError("goto_look: no RTDE state for qnear (link not started?)")
        qnear_rad = np.asarray(st.actual_q, float)
    reg = getattr(link, "reg_error", None)
    body = look_block(cfg, T_base_flange=T_base_flange, q_rad=q_rad, qnear_rad=qnear_rad, sim=sim, speeds=speeds,
                      reg_error=script.REG_ERROR if reg is None else int(reg))
    return link.run_block(body, name=name, timeout_s=timeout_s, settle_s=settle_s)


# ── boards ────────────────────────────────────────────────────────────────────
def boards_in_base(shot: Shot, specs: Mapping[str, BoardSpec] | Sequence[BoardSpec], intr: Intrinsics,
                   vcfg: Mapping, T_flange_cam: np.ndarray) -> dict[str, tuple[BoardPose, np.ndarray | None]]:
    """Every requested board: (BoardPose from vision.detect.measure, T_base_board [mm] or None).

    T_base_board = shot.T_base_flange @ T_flange_cam @ T_cam_board, only for poses that passed the [vision] checks
    (ok=True); boards not seen or rejected get None (the BoardPose says why)."""
    X = np.asarray(T_flange_cam, float)
    out: dict[str, tuple[BoardPose, np.ndarray | None]] = {}
    for name, pose in measure(shot.frame.image, specs, intr, vcfg).items():
        T = shot.T_base_flange @ X @ pose.T_cam_board if pose.ok and pose.T_cam_board is not None else None
        out[name] = (pose, T)
    return out
