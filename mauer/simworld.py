"""Pure-Python simulated world for the sequencer: ARES with drive errors and wheel slip, the UR5, the flange camera,
the wall and pick-up station boards, stones in the magazine, at the station and in the wall.

World frame = wall frame (docs/ARCHITECTURE.md). The simulation knows the TRUE poses; the sequencer only sees what the
real system would see: relative-move outcomes (odometry, which never sees slip), the robot's own flange pose and
rendered camera images. Everything the sequencer does with them (board detection, frame fit, closed-loop moves,
place poses relative to the measured frame) is the production code.

Model (all error parameters are ASSUMPTIONS with the source given at the field):
- ARES: planar true pose (x, y, theta) in the wall frame. A relative translation (body frame at the start pose) is
  executed with a scale error, scatter along the path, a heading change per run (giving a lateral offset of
  d * dtheta / 2 for a heading that turns evenly along the path), an extra lateral drift and, with some probability,
  a slip event (random direction and size, small heading jump). Odometry reports the commanded move exactly (the PLC
  closes its loop on odometry; slip is invisible to it - MA PLC README:127-128, mauer/ares/ads.py). Rotations get
  scatter and a loss after reversing the rotation direction (E003, mauer/ares/ads.py AresAds.rotate). Moves below the
  PLC minimum (2 mm / 0.2 deg) are refused like the real client (ads.check_move). ARES tilt on the casters is not
  modelled. Floor check (L jobs, mauer.floor.job_obstacles: legs, board plates, station table at its TRUE place): the
  area swept by every true move (hull of the start and end footprints; a rotation in 1 deg pieces) must not overlap
  an obstacle deeper than where the move started - otherwise a world violation "ARES footprint overlaps ..."
  (a start pose already on a plate is a note: the operator's start placement).
- UR5: true base = ARES pose @ T_ares_base (config) @ optional mount error. The arm reaches commanded Cartesian poses
  exactly (optional repeatability noise) and joint targets through the nominal UR5 DH forward kinematics (`ur5_fk`).
  With check_ik=True a target without any IK solution fails like the controller's IK guard. The robot reports its
  actual flange pose (encoders; kinematic calibration errors are not modelled).
- Camera: mauer.simcam.SynthCamera renders the boards from the TRUE flange pose with the TRUE T_flange_cam (the
  calibrated one used by the sequencer may differ by an injected hand-eye error). Boards far outside the view are
  culled before rendering (speed only). supersample >= 2: with 1 the renderer aliases and single-board tilt errors
  reach 0.6-0.7 deg (probe 2026-10-05, 12 views at 320 mm), with 2 they stay <= 0.05-0.15 deg like the research
  figure in station.toml [boards.ref] (tilt <= 0.06 deg).
- Stones: tracked from the magazine (ARES frame) / station (station frame) through the gripper to the wall. The jaws
  centre the stone across the jaws (TCP y) and square it to the jaws; the offset along the stone length and in height
  stays (ASSUMPTION: parallel ribbed jaws, mauer/config.py T_flange_tcp axes). At release the stone pose is compared
  with its nominal wall pose; it counts as seated when the horizontal error at both pins is within the pin capture
  range (README "capture range ~ 10 mm"; pins at +-half the pin spacing along the stone, pin spacing = bond offset
  100 mm, config [wall] bond_offset comment). Magazine holders have sockets as well ([deck] holder_z comment) and
  re-centre a stone the same way. Half stones ([half_brick], PLACEHOLDER): ONE pin pair centred along the stone,
  +-pin_across/2 across it; the jaws hold it along the stone only up to half the full-stone limit (ASSUMPTION: jaw
  contact scales with the stone length). Every stone keeps its type from the magazine / station to the wall; picking
  or placing a stone of another type than the sequencer expects is a world violation.

UR5 kinematics: UR's published nominal DH parameters (Universal Robots, "DH parameters for calculations of kinematics
and dynamics", universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-
and-dynamics/): d1 = 89.159, a2 = -425, a3 = -392.25, d4 = 109.15, d5 = 94.65, d6 = 82.3 mm, alpha = (pi/2, 0, 0,
pi/2, -pi/2, 0). FK gives the flange (tool0) in the controller base frame (= RoboDK UR5 base; the UR builder found
RoboDK's UR5 within 0.18-0.33 mm of these). `ur5_ik` is the closed form of K. Hawkins, "Analytic Inverse Kinematics
for the Universal Robots UR-5/UR-10 Arms" (2013), every solution verified by FK.
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Sequence

import numpy as np

from . import config as _config
from . import floor as _floor
from . import geometry as g
from .ares.ads import (MOVE_DONE, RES_OK, MoveOutcome, MoveRefused, OdomPose, check_move)
from .backends import RobotError, Shot
from .job import Job
from .reference import Pose2D, after_rotation, placements, wrap_angle
from .simcam import SynthCamera
from .vision import intrinsics as _intrinsics
from .vision.targets import board_specs

# ── UR5 kinematics ────────────────────────────────────────────────────────────
UR5_D = (89.159, 0.0, 0.0, 109.15, 94.65, 82.3)        # mm, UR nominal DH (module docstring)
UR5_A = (0.0, -425.0, -392.25, 0.0, 0.0, 0.0)          # mm
UR5_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
JOINT_W = (2.0, 2.0, 1.5, 1.0, 1.0, 0.5)               # joint weights of robodk/motion.py:23 (jdist)
GRASP_ACROSS_MM = 20.0                                 # ASSUMPTION, see SimRobot.__init__
GRASP_ALONG_MM = 50.0                                  # ASSUMPTION


def _dh(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def ur5_fk(q_rad: Sequence[float]) -> np.ndarray:
    """T_base_flange [mm] for joints q [rad] (nominal UR5 DH)."""
    q = np.asarray(q_rad, float).reshape(6)
    T = np.eye(4)
    for i in range(6):
        T = T @ _dh(q[i], UR5_D[i], UR5_A[i], UR5_ALPHA[i])
    return T


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ur5_ik(T_base_flange: np.ndarray, tol_mm: float = 1e-3) -> list[np.ndarray]:
    """All (up to 8) joint solutions [rad, wrapped to (-pi, pi]] of a flange pose [mm]; each verified by FK.
    Wrist singularity (sin q5 = 0): q6 = 0 is chosen."""
    T = np.asarray(T_base_flange, float)
    d1, d4, d5, d6 = UR5_D[0], UR5_D[3], UR5_D[4], UR5_D[5]
    a2, a3 = UR5_A[1], UR5_A[2]
    p05 = T @ np.array([0.0, 0.0, -d6, 1.0])
    r = math.hypot(p05[0], p05[1])
    if r < d4:
        return []
    psi, phi = math.atan2(p05[1], p05[0]), math.acos(min(1.0, d4 / r))
    T60 = g.inv(T)
    sols: list[np.ndarray] = []
    for s1 in (1.0, -1.0):
        t1 = psi + s1 * phi + math.pi / 2
        c5 = (T[0, 3] * math.sin(t1) - T[1, 3] * math.cos(t1) - d4) / d6
        if abs(c5) > 1.0 + 1e-9:
            continue
        b5 = math.acos(max(-1.0, min(1.0, c5)))
        for s5 in (1.0, -1.0):
            t5 = s5 * b5
            sn5 = math.sin(t5)
            t6 = 0.0 if abs(sn5) < 1e-9 else math.atan2(
                (-T60[1, 0] * math.sin(t1) + T60[1, 1] * math.cos(t1)) / sn5,
                (T60[0, 0] * math.sin(t1) - T60[0, 1] * math.cos(t1)) / sn5)
            T14 = g.inv(_dh(t1, d1, 0.0, UR5_ALPHA[0])) @ T @ g.inv(_dh(t5, d5, 0.0, UR5_ALPHA[4]) @
                                                                    _dh(t6, d6, 0.0, 0.0))
            p13 = (T14 @ np.array([0.0, -d4, 0.0, 1.0]))[:3]
            n = float(np.linalg.norm(p13))
            c3 = (n * n - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if abs(c3) > 1.0 + 1e-9 or n < 1e-9:
                continue
            b3 = math.acos(max(-1.0, min(1.0, c3)))
            for s3 in (1.0, -1.0):
                t3 = s3 * b3
                t2 = -math.atan2(p13[1], -p13[0]) + math.asin(max(-1.0, min(1.0, a3 * math.sin(t3) / n)))
                T34 = g.inv(_dh(t2, 0.0, a2, 0.0) @ _dh(t3, 0.0, a3, 0.0)) @ T14
                t4 = math.atan2(T34[1, 0], T34[0, 0])
                q = np.array([_wrap(v) for v in (t1, t2, t3, t4, t5, t6)])
                dp, da = g.pose_delta(ur5_fk(q), T)
                if dp < tol_mm and da < 1e-4 and not any(np.allclose(q, s, atol=1e-9) for s in sols):
                    sols.append(q)
    return sols


def in_family(q_rad: Sequence[float]) -> bool:
    """robodk/motion.py:36-41 family(): shoulder -135..+10 deg, elbow > 5 deg, wrist 2 < -5 deg (angles wrapped) -
    upper arm leaning towards the target, elbow up, wrist down (the only configuration the RoboDK planner uses)."""
    d = [math.degrees(_wrap(v)) for v in q_rad]
    return -135.0 < d[1] < 10.0 and d[2] > 5.0 and d[4] < -5.0


def joint_dist(a: Sequence[float], b: Sequence[float]) -> float:
    return float(sum(w * abs(x - y) for w, x, y in zip(JOINT_W, a, b)))


def _near_equiv(q: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """2 pi equivalent of q closest to ref per joint, inside the UR joint range +-2 pi."""
    out = q.copy()
    for i in range(6):
        cands = [q[i] + k * 2 * math.pi for k in (-2, -1, 0, 1, 2) if abs(q[i] + k * 2 * math.pi) <= 2 * math.pi]
        out[i] = min(cands, key=lambda v: abs(v - ref[i]))
    return out


def ik_near(T_base_flange: np.ndarray, q_ref: Sequence[float], family: bool = True) -> np.ndarray | None:
    """IK solution nearest q_ref (robodk/motion.py jdist weights), joint values chosen within +-2 pi next to q_ref;
    only the preferred family if `family`. None if there is none."""
    ref = np.asarray(q_ref, float).reshape(6)
    sols = [_near_equiv(s, ref) for s in ur5_ik(T_base_flange) if not family or in_family(s)]
    if not sols:
        return None
    return min(sols, key=lambda s: joint_dist(s, ref))


# ── error parameters ──────────────────────────────────────────────────────────
@dataclass
class DriveErrors:
    """ARES relative-move errors. Defaults = E003-like (MA calibration 01.10.2026, README); all ASSUMPTIONS for the
    sideways (+y) moves, which E003 did NOT calibrate (README 2026-10-02 afternoon)."""
    scale: float = -0.0003                 # E003: distance error -0.03 % (systematic, along the path)
    along_sigma_mm_per_3m: float = 0.75    # E003: scatter 0.5-1.0 mm per 3 m -> middle value
    heading_mean_deg: float = 0.11         # E003: ~0.11 deg heading change per straight run (sign ASSUMPTION: CCW)
    heading_sigma_deg: float = 0.06        # E003: sigma 0.06 deg
    lateral_sigma_mm_per_m: float = 2.0    # ASSUMPTION: README "lateral drift ~1 cm over 2-3 m" (incl. the heading
                                           # part) -> extra random lateral drift ~2 mm/m on top of d * dtheta / 2
    rot_sigma_deg: float = 0.14            # E003 rotation scatter (mauer/ares/ads.py AresAds.rotate docstring)
    rot_reversal_loss_deg: float = 1.2     # E003: ~1.2 deg loss after reversing the rotation direction (same source)
    rot_drift_sigma_mm: float = 2.0        # ASSUMPTION (no measurement): position drift during a rotation on the spot
    slip_prob: float = 0.2                 # ASSUMPTION: probability of a slip event per translation
    slip_max_mm: float = 30.0              # ASSUMPTION (task): slip up to ~30 mm, random direction, uniform size
    slip_max_deg: float = 0.3              # ASSUMPTION: heading jump of a slip event, uniform +-

    @classmethod
    def none(cls) -> "DriveErrors":
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class WorldErrors:
    """Everything injected into the simulated world (all ASSUMPTIONS). Defaults = no error."""
    drive: DriveErrors = field(default_factory=DriveErrors.none)
    start_sigma_mm: float = 0.0            # ARES start pose vs stop 0 (operator jogs to the start mark)
    start_sigma_deg: float = 0.0
    mount_xyz_mm: tuple = (0.0, 0.0, 0.0)  # true UR mount vs [ur5] (T_ares_base_true = T_ares_base @ error)
    mount_rpy_deg: tuple = (0.0, 0.0, 0.0)
    handeye_xyz_mm: tuple = (0.0, 0.0, 0.0)    # true T_flange_cam = calibrated @ error
    handeye_rpy_deg: tuple = (0.0, 0.0, 0.0)
    station_xyz_mm: tuple = (0.0, 0.0, 0.0)    # true station frame = nominal @ error
    station_rpy_deg: tuple = (0.0, 0.0, 0.0)
    repeat_mm: float = 0.0                 # robot pose noise (sigma per axis) of every reached pose
    repeat_deg: float = 0.0
    slot_sigma_mm: float = 0.0             # stone position scatter in the magazine / station holders

    @classmethod
    def realistic(cls) -> "WorldErrors":
        """E003-like drive errors with slip, plus ASSUMED errors of the other links in the chain."""
        return cls(drive=DriveErrors(),
                   start_sigma_mm=10.0, start_sigma_deg=0.3,     # README: absolute +-1-2 cm (start alignment)
                   mount_xyz_mm=(1.0, -0.5, 0.8), mount_rpy_deg=(0.05, -0.05, 0.1),   # ASSUMPTION: bolted mount
                   # hand-eye: research 25 poses ~0.07 mm / 0.009 deg (mauer/vision/handeye.py) -> x1.5-2 margin
                   handeye_xyz_mm=(0.10, -0.08, 0.12), handeye_rpy_deg=(0.015, -0.01, 0.02),
                   station_xyz_mm=(15.0, -10.0, 0.0), station_rpy_deg=(0.0, 0.0, 0.5),   # ASSUMPTION: placement
                   repeat_mm=0.03, repeat_deg=0.005,             # UR5 repeatability +-0.1 mm (datasheet) as ~3 sigma
                   slot_sigma_mm=1.0)                            # ASSUMPTION: holder sockets


# ── records ───────────────────────────────────────────────────────────────────
@dataclass
class PlacementRecord:
    """A stone released at the wall: true pose (TCP convention: top centre, x along the length, z down) vs nominal,
    errors in the wall frame axes [mm, deg]."""
    key: tuple
    T_wall_nominal: np.ndarray
    T_wall_release: np.ndarray
    dx_mm: float
    dy_mm: float
    dz_mm: float
    yaw_deg: float
    horiz_mm: float
    pin_mm: float
    seated: bool
    grasp_x_mm: float = 0.0
    grasp_z_mm: float = 0.0
    kind: str = "full"

    def to_dict(self) -> dict:
        return {"key": list(self.key), "kind": self.kind, "dx_mm": self.dx_mm, "dy_mm": self.dy_mm,
                "dz_mm": self.dz_mm, "yaw_deg": self.yaw_deg, "horiz_mm": self.horiz_mm, "pin_mm": self.pin_mm,
                "seated": self.seated, "grasp_x_mm": self.grasp_x_mm, "grasp_z_mm": self.grasp_z_mm}


def pose_errors(T_nom: np.ndarray, T_act: np.ndarray, pin_half_mm: float, height_mm: float,
                pins_xy: Sequence[Sequence[float]] | None = None) -> dict:
    """Errors of a stone pose in the axes of the parent frame: dx, dy, dz [mm], yaw [deg] about z, horizontal error of
    the top centre and the max horizontal error at the pins (default: +-pin_half_mm along the stone; pins_xy =
    explicit (along, across) pin positions in the TCP frame), at the stone bottom."""
    d = T_act[:3, 3] - T_nom[:3, 3]
    xa, xn = T_act[:3, 0], T_nom[:3, 0]
    yaw = math.degrees(math.atan2(xn[0] * xa[1] - xn[1] * xa[0], xn[0] * xa[0] + xn[1] * xa[1]))
    if pins_xy is None:
        pins_xy = ((pin_half_mm, 0.0), (-pin_half_mm, 0.0))
    pts = np.array([[float(x), float(y), height_mm] for x, y in pins_xy])
    pin = np.linalg.norm((g.apply(T_act, pts) - g.apply(T_nom, pts))[:, :2], axis=1)
    return {"dx_mm": float(d[0]), "dy_mm": float(d[1]), "dz_mm": float(d[2]), "yaw_deg": yaw,
            "horiz_mm": float(math.hypot(d[0], d[1])), "pin_mm": float(pin.max())}


def stats(values: Sequence[float]) -> dict:
    a = np.abs(np.asarray(values, float))
    if a.size == 0:
        return {"n": 0}
    return {"n": int(a.size), "mean": float(a.mean()), "rms": float(np.sqrt(np.mean(a ** 2))),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


# ── ARES ──────────────────────────────────────────────────────────────────────
@dataclass
class SimAresStatus:
    x_mm: float
    y_mm: float
    theta_deg: float
    move_active: bool = False
    amr_moving: bool = False

    @property
    def odom(self) -> OdomPose:
        return OdomPose(self.x_mm, self.y_mm, self.theta_deg)


class SimAres:
    """Simulated relative moves (see module docstring). The world's true pose changes; odometry follows the command."""

    def __init__(self, world: "SimWorld", seed: int = 0):
        self.world = world
        self.rng = np.random.default_rng(seed)
        self.odom = OdomPose(0.0, 0.0, 0.0)
        self.last_rot_sign = 0.0
        self.moves: list[dict] = []
        self.t_s = 0.0
        c = world.cfg.get("ares_ads", {})
        self.speed, self.accel = float(c.get("speed_mms", 150.0)), float(c.get("accel_mms2", 200.0))
        self.rot_speed = float(c.get("rot_speed_degs", 10.0))

    def _check(self, dx: float, dy: float, dth: float) -> None:
        reason = check_move(dx, dy, dth, self.speed, self.rot_speed, self.accel)
        if reason is not None:
            raise MoveRefused(f"move refused: {reason}")
        if not self.world.robot.is_parked():
            self.world.violations.append(f"ARES move ({dx:.1f}, {dy:.1f}, {dth:.2f}) while the robot is not parked")

    def _outcome(self, kind: str, dx: float, dy: float, dth: float, before: OdomPose, elapsed: float) -> MoveOutcome:
        return MoveOutcome(ok=True, cmd_id=len(self.moves), cmd_result=0, state=MOVE_DONE, result=RES_OK,
                           final_err_mm=0.0 if kind == "translate" else math.nan, text="simulated",
                           elapsed_s=elapsed, limited=False, odom_before=before, odom_after=self.odom, kind=kind,
                           dx_mm=dx, dy_mm=dy, dtheta_deg=dth, final_err_deg=0.0 if kind == "rotate" else math.nan)

    def translate(self, dx_mm: float, dy_mm: float) -> MoveOutcome:
        self._check(dx_mm, dy_mm, 0.0)
        e = self.world.errors.drive
        r = self.rng
        p = self.world.ares_true
        d = math.hypot(dx_mm, dy_mm)
        ex, ey = dx_mm / d, dy_mm / d
        dth = math.radians(r.normal(e.heading_mean_deg, e.heading_sigma_deg) if e.heading_sigma_deg > 0
                           else e.heading_mean_deg)
        along = d * (1.0 + e.scale) + (r.normal(0.0, e.along_sigma_mm_per_3m * d / 3000.0)
                                       if e.along_sigma_mm_per_3m > 0 else 0.0)
        lat = d * dth / 2.0 + (r.normal(0.0, e.lateral_sigma_mm_per_m * d / 1000.0)
                               if e.lateral_sigma_mm_per_m > 0 else 0.0)
        bx, by = ex * along - ey * lat, ey * along + ex * lat
        slip = None
        if e.slip_prob > 0 and r.random() < e.slip_prob:
            a, m = r.uniform(0.0, 2 * math.pi), r.uniform(0.0, e.slip_max_mm)
            sd = math.radians(r.uniform(-e.slip_max_deg, e.slip_max_deg))
            bx, by, dth = bx + m * math.cos(a), by + m * math.sin(a), dth + sd
            slip = {"mm": m, "dir_deg": math.degrees(a), "deg": math.degrees(sd)}
        n_tr = 1 + sum(m["kind"] == "translate" for m in self.moves)
        f = self.world.forced_slips.get(n_tr)
        if f is not None:                                  # test hook: extra body-frame error on translation n_tr
            bx, by, dth = bx + f[0], by + f[1], dth + math.radians(f[2])
            slip = {"forced": list(f)}
        c, s = math.cos(p.theta_rad), math.sin(p.theta_rad)
        self.world.ares_true = Pose2D(p.x_mm + c * bx - s * by, p.y_mm + s * bx + c * by,
                                      wrap_angle(p.theta_rad + dth))
        self.world.floor_check("translate", p, self.world.ares_true)
        before = self.odom
        t = math.radians(before.theta_deg)
        self.odom = OdomPose(before.x_mm + math.cos(t) * dx_mm - math.sin(t) * dy_mm,
                             before.y_mm + math.sin(t) * dx_mm + math.cos(t) * dy_mm, before.theta_deg)
        elapsed = d / self.speed + self.speed / self.accel          # trapezoid profile (ASSUMPTION)
        self.t_s += elapsed
        self.moves.append({"kind": "translate", "dx_mm": dx_mm, "dy_mm": dy_mm, "slip": slip,
                           "true_after": self.world.ares_true.to_dict()})
        return self._outcome("translate", dx_mm, dy_mm, 0.0, before, elapsed)

    def rotate(self, dtheta_deg: float) -> MoveOutcome:
        self._check(0.0, 0.0, dtheta_deg)
        e = self.world.errors.drive
        r = self.rng
        sign = math.copysign(1.0, dtheta_deg)
        act = dtheta_deg + (r.normal(0.0, e.rot_sigma_deg) if e.rot_sigma_deg > 0 else 0.0)
        if self.last_rot_sign and sign != self.last_rot_sign and e.rot_reversal_loss_deg > 0:
            act = sign * max(abs(act) - e.rot_reversal_loss_deg, 0.0)
        self.last_rot_sign = sign
        p0 = self.world.ares_true
        p = after_rotation(self.world.ares_true, math.radians(act))
        if e.rot_drift_sigma_mm > 0:
            p = Pose2D(p.x_mm + r.normal(0.0, e.rot_drift_sigma_mm), p.y_mm + r.normal(0.0, e.rot_drift_sigma_mm),
                       p.theta_rad)
        self.world.ares_true = p
        self.world.floor_check("rotate", p0, p, math.radians(act))
        before = self.odom
        self.odom = OdomPose(before.x_mm, before.y_mm, (before.theta_deg + dtheta_deg + 180.0) % 360.0 - 180.0)
        elapsed = abs(dtheta_deg) / self.rot_speed + 1.0
        self.t_s += elapsed
        self.moves.append({"kind": "rotate", "dtheta_deg": dtheta_deg, "actual_deg": act,
                           "true_after": p.to_dict()})
        return self._outcome("rotate", 0.0, 0.0, dtheta_deg, before, elapsed)

    def preflight(self) -> list[str]:
        return []

    def status(self) -> SimAresStatus:
        return SimAresStatus(self.odom.x_mm, self.odom.y_mm, self.odom.theta_deg)

    def idle(self) -> bool:
        return True

    def abort(self) -> None:
        return None


# ── robot ─────────────────────────────────────────────────────────────────────
class SimRobot:
    """Simulated UR5 + gripper. `fail_on` = {action: n} raises RobotError on the n-th call of that action (after the
    arm has left the park pose), for failure tests."""

    def __init__(self, world: "SimWorld", seed: int = 0, check_ik: bool = True,
                 fail_on: Mapping[str, int] | None = None, guard=None):
        self.world = world
        self.guard = guard                       # mauer.motionguard.MotionGuard (as URRobot) or None
        self.guard_vias = 0                      # detour vias the guard inserted (statistics)
        self.rng = np.random.default_rng(seed + 7)
        self.check_ik = check_ik
        self.fail_on = dict(fail_on or {})
        self.calls: Counter = Counter()
        self.park_q = np.asarray(world.job.park_q_rad, float)
        self.q: np.ndarray | None = self.park_q.copy()
        self.q_cmd = self.park_q.copy()          # commanded joints at the end of the last action (guard start)
        self.T_bf = ur5_fk(self.park_q)          # actual flange in the TRUE base frame [mm]
        self.parked = True
        self.holding: tuple[str, np.ndarray] | None = None
        self.T_flange_tcp = np.asarray(world.job.T_flange_tcp, float)
        self.capture_mm = float(world.capture_mm)
        # ASSUMPTIONS (EHPS-20-A stroke and jaw size not looked up): an open jaw lands on the stone when the stone is
        # more than GRASP_ACROSS_MM off across the jaws; along the stone the ribbed jaws still hold it up to
        # GRASP_ALONG_MM. Beyond either the simulation stops with a RobotError (the real robot has no sensor for it).
        self.grasp_across_mm, self.grasp_along_mm = GRASP_ACROSS_MM, GRASP_ALONG_MM

    # ── motion ───────────────────────────────────────────────────────────────
    def _noise(self) -> np.ndarray:
        e = self.world.errors
        if e.repeat_mm <= 0 and e.repeat_deg <= 0:
            return np.eye(4)
        return g.pose_xyz_rpy(self.rng.normal(0.0, e.repeat_mm, 3), self.rng.normal(0.0, e.repeat_deg, 3))

    def _call(self, action: str) -> None:
        self.calls[action] += 1
        self.world.n_blocks += 1
        if self.fail_on.get(action) == self.calls[action]:
            self.parked = False
            raise RobotError(f"{action}: simulated block failure (protective stop)", action=action)

    def _move_flange(self, T_base_flange: np.ndarray, what: str) -> None:
        T = np.asarray(T_base_flange, float)
        if self.check_ik and not ur5_ik(T):
            self.parked = False
            raise RobotError(f"{what}: no IK solution (controller IK guard)", action=what)
        self.T_bf = T @ self._noise()
        self.q = None
        self.parked = False

    def _move_tcp(self, T_base_tcp: np.ndarray, what: str) -> None:
        self._move_flange(np.asarray(T_base_tcp, float) @ g.inv(self.T_flange_tcp), what)

    def _held_kind(self) -> str | None:
        return None if self.holding is None else self.world.kind_of.get(self.holding[0], "full")

    def _guard(self, q_target, what: str, column=None) -> None:
        """mauer.motionguard check of the joint move q_cmd -> q_target (as URRobot._guarded); RobotError before the
        move if refused."""
        if self.guard is None or q_target is None:
            return
        v = self.guard.plan(self.q_cmd, q_target, self._held_kind(), (), column)
        if not v.ok:
            raise RobotError(f"{what}: refused by the motion guard: " + " | ".join(v.problems[-3:]), action=what)
        self.guard_vias += len(v.vias)

    def _guard_above(self, T_base_frame, T_frame_tcp, hint, what: str, approach_mm: float | None = None) -> None:
        if self.guard is None:
            return
        a = float(self.world.job.approach_mm if approach_mm is None else approach_mm)
        T = (np.asarray(T_base_frame, float) @ g.transl(0.0, 0.0, a)
             @ np.asarray(T_frame_tcp, float) @ g.inv(self.T_flange_tcp))
        q = ik_near(T, np.asarray(hint if hint is not None else self.q_cmd, float))
        if q is None:
            raise RobotError(f"{what}: no IK solution for the approach pose", action=what)
        column = np.asarray(T_base_frame, float) @ np.asarray(T_frame_tcp, float)      # descent column
        self._guard(q, what, column)
        self.q_cmd = q

    def T_world_tcp(self) -> np.ndarray:
        return self.world.T_wall_base_true() @ self.T_bf @ self.T_flange_tcp

    # ── Robot interface ──────────────────────────────────────────────────────
    def park(self):
        self._call("park")
        self._guard(self.park_q, "park")
        self.q_cmd = self.park_q.copy()
        self.q = self.park_q.copy()
        self.T_bf = ur5_fk(self.q)
        self.parked = True

    def goto_look(self, look):
        self._call("goto_look")
        if self.guard is not None:
            q_look = (np.asarray(look.q_rad, float) if look.q_rad is not None
                      else ik_near(np.asarray(look.T_base_flange, float),
                                   np.asarray(look.qnear_rad if look.qnear_rad is not None else self.q_cmd, float)))
            if q_look is None:
                raise RobotError(f"look {look.name}: no IK solution", action="goto_look")
            self._guard(q_look, f"look {look.name}")
            self.q_cmd = q_look
        if look.q_rad is not None:
            self.T_bf = ur5_fk(look.q_rad) @ self._noise()
            self.q, self.parked = np.asarray(look.q_rad, float), False
        else:
            self._move_flange(look.T_base_flange, f"look {look.name}")

    def shot(self, camera):
        self.world.n_shots += 1
        frame = camera.grab()
        return Shot(frame, self.T_bf.copy(), None if self.q is None else self.q.copy(), 0.0, 1,
                    {"sim": True})

    def _grip(self, stone_id: str, T_world_stone: np.ndarray, expected_kind: str | None = None) -> None:
        T_tcp_stone = g.inv(self.T_world_tcp()) @ T_world_stone
        off = T_tcp_stone[:3, 3]
        kind = self.world.kind_of.get(stone_id, "full")
        along = self.grasp_along_mm * self.world.length_of(kind) / self.world.length_of("full")
        if self.world.grasp_check and (abs(off[1]) > self.grasp_across_mm or abs(off[0]) > along):
            raise RobotError(f"gripper missed stone {stone_id} ({kind}): offset ({off[0]:.1f}, {off[1]:.1f}) mm",
                             action="grip")
        if expected_kind is not None and kind != expected_kind:
            self.world.violations.append(f"picked {kind} stone {stone_id} where a {expected_kind} stone was expected")
        self.holding = (stone_id, g.transl(off[0], 0.0, off[2]))   # jaws centre across, square the stone

    def _release(self) -> tuple[str, np.ndarray, np.ndarray]:
        if self.holding is None:
            raise RobotError("release: no stone in the gripper", action="release")
        sid, T_tcp_stone = self.holding
        self.holding = None
        return sid, self.T_world_tcp() @ T_tcp_stone, T_tcp_stone

    def pick_magazine(self, slot, T_base_ares, kind: str = "full", approach_mm: float | None = None):
        self._call("pick_magazine")
        self._guard_above(T_base_ares, slot.T_ares_tcp, slot.qnear_rad, f"pick magazine {slot.id}", approach_mm)
        self._move_tcp(np.asarray(T_base_ares, float) @ slot.T_ares_tcp, f"pick magazine {slot.id}")
        sid, T_ares_stone = self.world.mag_stones.pop(slot.id, (None, None))
        if sid is None:
            raise RobotError(f"magazine slot {slot.id} is empty (gripper closes on air)", action="pick_magazine")
        self._grip(sid, self.world.T_wall_ares_true() @ T_ares_stone, kind)

    def _release_above(self) -> float:
        return float(self.world.cfg.get("ur", {}).get("release_above_mm", 0.0))

    def place_wall(self, T_base_wall, stone):
        """As URRobot.place_wall: the jaws open [ur] release_above_mm above the place pose; the stone drops that far
        (world z) onto the pins / cones below."""
        from .backends import release_pose
        self._call("place_wall")
        rel = self._release_above()
        T_rel = release_pose(stone.T_wall_tcp, rel)
        self._guard_above(T_base_wall, T_rel, stone.qnear_rad, f"place {stone.key}")
        self._move_tcp(np.asarray(T_base_wall, float) @ T_rel, f"place {stone.key}")
        sid, T_wall_stone, T_tcp_stone = self._release()
        T_wall_stone = g.transl(0.0, 0.0, -rel) @ T_wall_stone                # the drop
        kind = self.world.kind_of.get(sid, "full")
        if kind != getattr(stone, "kind", "full"):
            self.world.violations.append(f"{kind} stone {sid} placed as {stone.key} ({stone.kind})")
        self.world.record_placement(stone, T_wall_stone, T_tcp_stone, kind)

    def pick_station(self, T_base_station, slot):
        self._call("pick_station")
        self._guard_above(T_base_station, slot.T_station_tcp, slot.qnear_rad, f"pick station {slot.id}")
        self._move_tcp(np.asarray(T_base_station, float) @ slot.T_station_tcp, f"pick station {slot.id}")
        sid, T_st_stone = self.world.station_stones.pop(slot.id, (None, None))
        if sid is None:
            raise RobotError(f"station slot {slot.id} is empty (gripper closes on air)", action="pick_station")
        self._grip(sid, self.world.T_wall_station_true @ T_st_stone, getattr(slot, "kind", "full"))

    def place_magazine(self, slot, T_base_ares, kind: str = "full", approach_mm: float | None = None):
        """As URRobot.place_magazine: the jaws open [ur] release_above_mm above the slot pose; the stone drops that
        far (world z) before the holder's cones / the pins below seat it."""
        from .backends import release_pose
        self._call("place_magazine")
        rel = self._release_above()
        T_rel = release_pose(slot.T_ares_tcp, rel)
        self._guard_above(T_base_ares, T_rel, slot.qnear_rad, f"place magazine {slot.id}", approach_mm)
        self._move_tcp(np.asarray(T_base_ares, float) @ T_rel, f"place magazine {slot.id}")
        sid, T_wall_stone, _ = self._release()
        T_wall_stone = g.transl(0.0, 0.0, -rel) @ T_wall_stone                # the drop
        true_kind = self.world.kind_of.get(sid, "full")
        if true_kind != kind:
            self.world.violations.append(f"{true_kind} stone {sid} put into magazine slot {slot.id} as {kind}")
        T_ares_stone = g.inv(self.world.T_wall_ares_true()) @ T_wall_stone
        err = pose_errors(slot.T_ares_tcp, T_ares_stone, self.world.pin_half_mm, self.world.height_mm,
                          self.world.pins_xy(true_kind))
        if err["pin_mm"] <= self.capture_mm:
            T_ares_stone = np.asarray(slot.T_ares_tcp, float).copy()      # holder sockets centre it
        else:
            self.world.notes.append(f"stone {sid} not seated in magazine slot {slot.id}: {err['pin_mm']:.1f} mm")
        self.world.mag_stones[slot.id] = (sid, T_ares_stone)
        self.world.n_reloaded += 1

    def dry_place(self, T_base_frame, stone, hover_mm: float, dwell_s: float):
        """Magazine dry run (mauer/magtest.py): the held stone to hover_mm above the place pose and back - it stays in
        the jaws (URRobot.dry_place)."""
        self._call("dry_place")
        T = g.transl(0.0, 0.0, float(hover_mm)) @ np.asarray(stone.T_wall_tcp, float)
        what = f"dry place {stone.key}"
        if self.holding is None:
            raise RobotError(f"{what}: no stone in the gripper", action="dry_place")
        self._guard_above(T_base_frame, T, stone.qnear_rad, what)
        self._move_tcp(np.asarray(T_base_frame, float) @ T, what)

    def is_parked(self) -> bool:
        return self.parked

    def is_idle(self) -> bool:
        return True

    def abort(self):
        return "simulated abort"


# ── world ─────────────────────────────────────────────────────────────────────
class SimWorld:
    """The simulated world for one job. Use world.robot, world.ares, world.camera, world.T_flange_cam (the CALIBRATED
    hand-eye the sequencer must use) and world.intr with mauer.sequencer.Sequencer; then world.summary()."""

    def __init__(self, cfg: Mapping, job: Job, errors: WorldErrors | None = None, *, seed: int = 0,
                 start_stop: int = 0, intr=None, T_flange_cam=None, supersample: int = 2, noise_sigma: float = 1.0,
                 blur_sigma_px: float = 0.6, check_ik: bool = True, fail_on: Mapping[str, int] | None = None,
                 capture_mm: float = 10.0, grasp_check: bool = True, guard: bool = False, standing=()):
        self.cfg, self.job = cfg, job
        self.errors = errors or WorldErrors()
        self.rng = np.random.default_rng(seed)
        e = self.errors
        b = cfg["brick"]
        self.height_mm = float(b["height"])
        # pin spacing = bond offset (station.toml [wall] bond_offset: "half a stone (100 mm) = pin spacing")
        self.pin_half_mm = 0.5 * float(cfg["wall"]["bond_offset"]) * (float(b["length"]) + float(b["head_joint"]))
        hb = cfg.get("half_brick", {}) or {}
        self.half_pin_across_mm = float(hb.get("pin_across", 53.0))       # [half_brick] pin_across (PLACEHOLDER)
        self._lengths = {"full": float(b["length"]), "half": float(hb.get("length", float(b["length"]) / 2.0))}
        self.kind_of: dict[str, str] = {}                                  # stone id -> "full" / "half"
        self.capture_mm = float(capture_mm)          # README: conical pins, capture range ~ 10 mm
        self.grasp_check = bool(grasp_check)         # False: never "miss" a stone (compare dead reckoning to the end)
        self.specs = board_specs(cfg)
        self.placements = placements(cfg)
        self.intr = intr if intr is not None else _intrinsics.nominal(cfg)
        self.T_flange_cam = np.asarray(T_flange_cam if T_flange_cam is not None else
                                       _config.T_flange_cam_nominal(dict(cfg)), float)
        self.T_flange_cam_true = self.T_flange_cam @ g.pose_xyz_rpy(e.handeye_xyz_mm, e.handeye_rpy_deg)
        self.T_ares_base_nominal = np.asarray(job.T_ares_base, float)
        self.T_ares_base_true = self.T_ares_base_nominal @ g.pose_xyz_rpy(e.mount_xyz_mm, e.mount_rpy_deg)
        self.T_wall_station_true = (np.asarray(job.station.T_wall_station, float)
                                    @ g.pose_xyz_rpy(e.station_xyz_mm, e.station_rpy_deg))
        self.ares_shape = _floor.AresShape.from_config(cfg) if "ares" in cfg else _floor.AresShape()
        self.floor: list = []                         # obstacles of the floor check (L jobs only)
        if job.legs:
            self.floor = _floor.job_obstacles(cfg, job.legs, self.T_wall_station_true,
                                              {n: sp.size_mm for n, sp in self.specs.items()})
        self.floor_hits: list[dict] = []
        self.floor_min_gap_mm = math.inf
        self.notes: list[str] = []
        p0 = job.stops[start_stop].ares
        self.ares_true = Pose2D(p0.x_mm + (self.rng.normal(0, e.start_sigma_mm) if e.start_sigma_mm else 0.0),
                                p0.y_mm + (self.rng.normal(0, e.start_sigma_mm) if e.start_sigma_mm else 0.0),
                                p0.theta_rad + (math.radians(self.rng.normal(0, e.start_sigma_deg))
                                                if e.start_sigma_deg else 0.0))
        self.ares_start = self.ares_true
        for o in self.floor:
            d = _floor.penetration(self.ares_shape.footprint(self.ares_true), o.poly)
            if d > 0:
                self.floor_hits.append({"move": 0, "kind": "start", "obstacle": o.name, "depth_mm": d})
                self.notes.append(f"start pose overlaps {o.name} by {d:.1f} mm (operator's start placement)")
        self.hidden_boards: set[str] = set()
        self.forced_slips: dict[int, tuple[float, float, float]] = {}   # translation no. (1-based) -> (dx, dy, dth_deg)
        self.violations: list[str] = []
        self.records: list[PlacementRecord] = []
        self.n_blocks = self.n_shots = self.n_reloaded = 0
        self._n_stone = 0
        # stones: magazine (ARES frame) and station (station frame), true poses
        self.mag_stones: dict[str, tuple[str, np.ndarray]] = {}
        standing = set(standing) | {t.key for st in job.stops[:start_stop] for t in st.stones}
        if standing:                                 # a later start: loaded for the next stones (job.restart_fill)
            from .job import restart_fill
            fill = restart_fill(job, standing)
        else:
            fill = [(sid, job.magazine.initial_kinds.get(sid, "full")) for sid in job.magazine.initial_fill]
        for sid, kind in fill:
            self.mag_stones[sid] = (self._new_stone("mag", kind), self._jitter(job.magazine.slot(sid).T_ares_tcp))
        self.station_stones: dict[str, tuple[str, np.ndarray]] = {}
        self.refill_station()
        mg = None
        if guard:                                # the real robot's motion guard (mauer.motionguard)
            from .motionguard import MotionGuard
            mg = MotionGuard(cfg, job.T_ares_base, job.park_q_rad, job.T_flange_tcp, approach_mm=job.approach_mm)
        self.robot = SimRobot(self, seed, check_ik, fail_on, mg)
        self.ares = SimAres(self, seed + 1)
        self.camera = SynthCamera(lambda: self.robot.T_bf, self.T_flange_cam_true, self.boards_for_camera, self.intr,
                                  supersample=supersample, noise_sigma=noise_sigma, blur_sigma_px=blur_sigma_px,
                                  seed=seed, distort=False)
        self.camera.open()

    # ── stones ───────────────────────────────────────────────────────────────
    def _new_stone(self, where: str, kind: str = "full") -> str:
        self._n_stone += 1
        sid = f"{where}{self._n_stone:03d}" + ("h" if kind == "half" else "")
        self.kind_of[sid] = kind
        return sid

    def length_of(self, kind: str) -> float:
        return self._lengths.get(kind, self._lengths["full"])

    def pins_xy(self, kind: str) -> list[tuple[float, float]]:
        """Pin positions (along, across) in the TCP frame: full stone +-pin_half along (model of this module), half
        stone one pair at the centre, +-pin_across/2 across ([half_brick], PLACEHOLDER)."""
        if kind == "half":
            return [(0.0, self.half_pin_across_mm / 2.0), (0.0, -self.half_pin_across_mm / 2.0)]
        return [(self.pin_half_mm, 0.0), (-self.pin_half_mm, 0.0)]

    def _jitter(self, T: np.ndarray) -> np.ndarray:
        s = self.errors.slot_sigma_mm
        T = np.asarray(T, float)
        if s <= 0:
            return T.copy()
        return g.transl(*self.rng.normal(0.0, s, 2), 0.0) @ T

    def refill_station(self) -> int:
        """Operator fills every empty station slot (usable slots of the job). Returns the number added."""
        n = 0
        for sid in self.job.station.take_order:
            if sid not in self.station_stones:
                slot = self.job.station.slot(sid)
                self.station_stones[sid] = (self._new_stone("st", slot.kind), self._jitter(slot.T_station_tcp))
                n += 1
        return n

    def record_placement(self, stone, T_wall_stone: np.ndarray, T_tcp_stone: np.ndarray,
                         kind: str | None = None) -> PlacementRecord:
        kind = kind or getattr(stone, "kind", "full")
        e = pose_errors(np.asarray(stone.T_wall_tcp, float), T_wall_stone, self.pin_half_mm, self.height_mm,
                        self.pins_xy(kind))
        rec = PlacementRecord(stone.key, np.asarray(stone.T_wall_tcp, float), T_wall_stone, e["dx_mm"], e["dy_mm"],
                              e["dz_mm"], e["yaw_deg"], e["horiz_mm"], e["pin_mm"], e["pin_mm"] <= self.capture_mm,
                              float(T_tcp_stone[0, 3]), float(T_tcp_stone[2, 3]), kind)
        self.records.append(rec)
        return rec

    # ── floor ────────────────────────────────────────────────────────────────
    def floor_check(self, kind: str, a: Pose2D, b: Pose2D, dtheta_rad: float | None = None) -> None:
        """True move a -> b (see module docstring): a violation for every obstacle the swept area enters deeper than
        the start footprint already was; tracks the smallest gap of an end footprint to any obstacle."""
        if not self.floor:
            return
        n = len(self.ares.moves) + 1
        sh = self.ares_shape
        pieces = ([_floor.swept_translation(sh, a, b)] if kind == "translate"
                  else _floor.swept_rotation(sh, a, b, dtheta_rad))
        fa = sh.footprint(a)
        for o in self.floor:
            d = max(_floor.penetration(pc, o.poly) for pc in pieces)
            if d > _floor.penetration(fa, o.poly) + 0.05:
                self.floor_hits.append({"move": n, "kind": kind, "obstacle": o.name, "depth_mm": d,
                                        "from": a.to_dict(), "to": b.to_dict()})
                self.violations.append(f"ARES footprint overlaps {o.name} by {d:.1f} mm during {kind} no. {n} "
                                       f"({a.describe()} -> {b.describe()})")
        fb = sh.footprint(b)
        self.floor_min_gap_mm = min(self.floor_min_gap_mm, min(_floor.poly_dist(fb, o.poly) for o in self.floor))

    # ── frames ───────────────────────────────────────────────────────────────
    def T_wall_ares_true(self) -> np.ndarray:
        return self.ares_true.T

    def T_wall_base_true(self) -> np.ndarray:
        return self.ares_true.T @ self.T_ares_base_true

    def board_poses_wall(self) -> list[tuple[Any, np.ndarray]]:
        """(spec, T_wall_board) of every wall / station board that exists in the world (not hidden)."""
        out = []
        for p in self.placements:
            if p.name in self.hidden_boards or p.name not in self.specs:
                continue
            if p.parent == "wall":
                out.append((self.specs[p.name], p.T_parent_board))
            elif p.parent == "station":
                out.append((self.specs[p.name], self.T_wall_station_true @ p.T_parent_board))
        return out

    def boards_for_camera(self) -> list[tuple[Any, np.ndarray]]:
        """Boards in the TRUE UR base frame for SynthCamera, culled to those whose centre is in front of the camera
        and within twice the image size around it (rendering cost only)."""
        T_base_wall = g.inv(self.T_wall_base_true())
        T_cam_base = g.inv(self.robot.T_bf @ self.T_flange_cam_true)
        K, W, H = self.intr.K, self.intr.width, self.intr.height
        out = []
        for spec, T_wb in self.board_poses_wall():
            T_bb = T_base_wall @ T_wb
            c = g.apply(T_cam_base @ T_bb, [[*spec.centre_mm, 0.0]])[0]
            if c[2] <= 10.0:
                continue
            u, v = K[0, 0] * c[0] / c[2] + K[0, 2], K[1, 1] * c[1] / c[2] + K[1, 2]
            if -W <= u <= 2 * W and -H <= v <= 2 * H:
                out.append((spec, T_bb))
        return out

    # ── results ──────────────────────────────────────────────────────────────
    def placement_stats(self) -> dict:
        r = self.records
        return {"n": len(r), "seated": int(sum(x.seated for x in r)),
                "by_kind": {k: {"n": sum(x.kind == k for x in r), "seated": sum(x.seated for x in r if x.kind == k),
                                "pin_max_mm": max((x.pin_mm for x in r if x.kind == k), default=None)}
                            for k in ("full", "half")},
                "horiz_mm": stats([x.horiz_mm for x in r]), "pin_mm": stats([x.pin_mm for x in r]),
                "dx_mm": stats([x.dx_mm for x in r]), "dy_mm": stats([x.dy_mm for x in r]),
                "dz_mm": stats([x.dz_mm for x in r]), "yaw_deg": stats([x.yaw_deg for x in r])}

    def summary(self) -> dict:
        moves = self.ares.moves
        return {"placement": self.placement_stats(),
                "ares": {"moves": len(moves), "translations": sum(m["kind"] == "translate" for m in moves),
                         "rotations": sum(m["kind"] == "rotate" for m in moves),
                         "slips": sum(1 for m in moves if m.get("slip")), "drive_time_s": self.ares.t_s,
                         "true_pose": self.ares_true.to_dict()},
                "robot": {"blocks": self.n_blocks, "shots": self.n_shots, "calls": dict(self.robot.calls)},
                "reloaded_stones": self.n_reloaded, "violations": list(self.violations), "notes": list(self.notes),
                "floor": {"checked": bool(self.floor), "hits": [h for h in self.floor_hits if h["kind"] != "start"],
                          "start_overlaps": [h for h in self.floor_hits if h["kind"] == "start"],
                          "min_gap_mm": None if not math.isfinite(self.floor_min_gap_mm) else self.floor_min_gap_mm}}


SCENARIOS = ("none", "e003", "slip", "realistic")


def scenario(name: str) -> WorldErrors:
    """Named error scenarios (tools/run_job.py --sim, tests):
    none       no error at all;
    e003       E003-like drive errors (DriveErrors defaults) without slip;
    slip       E003-like drive errors + slip events + start-pose error + the camera chain's own errors (hand-eye,
               robot repeatability) - everything the camera loop is meant to correct;
    realistic  `slip` + the errors OUTSIDE the camera loop: UR mount vs the deck magazine, stone scatter in the
               holders, station placement (WorldErrors.realistic). The magazine is referenced to the nominal mount,
               not measured, and the jaws do not centre along the stone, so these go 1:1 into the stone position
               along its length."""
    if name == "none":
        return WorldErrors()
    if name == "e003":
        return WorldErrors(drive=replace(DriveErrors(), slip_prob=0.0))
    if name == "slip":
        r = WorldErrors.realistic()
        return WorldErrors(drive=DriveErrors(), start_sigma_mm=r.start_sigma_mm, start_sigma_deg=r.start_sigma_deg,
                           handeye_xyz_mm=r.handeye_xyz_mm, handeye_rpy_deg=r.handeye_rpy_deg,
                           repeat_mm=r.repeat_mm, repeat_deg=r.repeat_deg)
    if name == "realistic":
        return WorldErrors.realistic()
    raise ValueError(f"unknown scenario {name!r} ({', '.join(SCENARIOS)})")
