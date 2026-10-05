"""Reference frames from measured ChArUco boards: the wall frame per stop, the pick-up station frame, the ARES pose.

Every reference board has a fixed, known placement in a parent frame (config/station.toml [[targets]]: parent "wall"
or "station"; the calibration board [boards.calib] on the ARES deck has parent "deck" = the ARES frame). The camera
measures the boards in the UR base frame (T_base_board = T_base_flange @ T_flange_cam @ T_cam_board,
docs/ARCHITECTURE.md). `fit_frame` finds the parent frame in the base frame, T_base_parent, as the least-squares
rigid fit (Kabsch, geometry.fit_rigid) of ALL chessboard corners of all observed boards of that parent: corner c of
board b is known in the parent frame (T_parent_board @ c) and measured in the base frame (T_base_board @ c).

- One board: the corners are related by an exact rigid transform, so the fit IS that board's 6D pose
  (T_base_parent = T_base_board @ inv(T_parent_board)), including its weak axes (tilt, depth along the optical axis).
- Two or more boards far apart: the heading about the parent z axis comes from the baseline between the boards
  (a lateral error e on boards L apart gives about e / L rad), no longer from the small extent of one board. A 0.1 deg
  heading error is 1.4 mm at u = +-800 mm (README "Camera / Open"), so the baseline is reported (`baseline_mm`).

Derived helpers: the ARES pose in the wall / station frame (T_parent_ares = inv(T_ares_base @ T_base_parent)), its
planar part (x, y, theta) as `Pose2D`, and the relative move between two planar poses in the ARES body frame (the
frame of the PLC relative move: +x forward, +y left, +theta CCW, mauer/ares/ads.py).

Conventions (docs/ARCHITECTURE.md): T_a_b = pose of frame b in frame a, 4x4 float64, mm and rad.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import combinations
from typing import Mapping, Sequence

import numpy as np

from . import config as _config
from . import geometry as g
from .vision.targets import BoardSpec, corners_obj

PARENTS = ("wall", "station", "deck")      # "deck" = the ARES frame (base_link), see module docstring


# ── board placements ──────────────────────────────────────────────────────────
@dataclass(frozen=True, eq=False)
class Placement:
    """Known pose of one board (frame: OpenCV CharucoBoard, origin top-left outer corner, z into the board) in its
    parent frame [mm]."""
    name: str
    parent: str
    T_parent_board: np.ndarray

    @property
    def T(self) -> np.ndarray:
        return self.T_parent_board


def placements(cfg: Mapping) -> list[Placement]:
    """All board placements from the config: the calib board ([boards.calib] xyz/rpy_deg, parent "deck") and every
    [[targets]] entry (parent "wall" / "station" / "deck"). A wall target with `leg = "<name>"` gives xyz/rpy_deg in
    that leg's frame ([[wall.legs]], config.leg_frames): T_wall_board = T_wall_leg @ pose(xyz, rpy). ValueError for an
    unknown parent or leg, a leg on a non-wall target, or a duplicate name."""
    out: list[Placement] = []
    c = cfg.get("boards", {}).get("calib", {})
    if "xyz" in c:
        out.append(Placement("calib", "deck", _config.pose(c)))
    legs = _config.leg_frames(dict(cfg))
    for t in cfg.get("targets", []):
        parent = str(t["parent"])
        if parent not in PARENTS:
            raise ValueError(f"target {t.get('name')!r}: parent {parent!r} not in {PARENTS}")
        T = g.pose_xyz_rpy(t["xyz"], t.get("rpy_deg", (0.0, 0.0, 0.0)))
        if "leg" in t:
            if parent != "wall":
                raise ValueError(f"target {t.get('name')!r}: leg {t['leg']!r} given for parent {parent!r}")
            if str(t["leg"]) not in legs:
                raise ValueError(f"target {t.get('name')!r}: unknown leg {t['leg']!r} (legs {sorted(legs)})")
            T = legs[str(t["leg"])] @ T
        out.append(Placement(str(t["name"]), parent, T))
    names = [p.name for p in out]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise ValueError(f"duplicate board names {dup}")
    return out


def by_parent(pl: Sequence[Placement], parent: str) -> list[Placement]:
    return [p for p in pl if p.parent == parent]


def board_centre(spec: BoardSpec, T_parent_board: np.ndarray) -> np.ndarray:
    """Board centre in the parent frame [mm] (3,)."""
    return g.apply(T_parent_board, [[*spec.centre_mm, 0.0]])[0]


# ── frame fit ─────────────────────────────────────────────────────────────────
@dataclass
class BoardResidual:
    """How well one board agrees with the fitted frame."""
    name: str
    n_corners: int
    rms_mm: float            # RMS distance fitted vs measured corner positions [mm]
    max_mm: float
    pos_mm: float            # board pose: fitted (T_base_parent @ T_parent_board) vs measured, origin distance [mm]
    ang_deg: float           # ... and rotation angle [deg]

    def to_dict(self) -> dict:
        return {"name": self.name, "n_corners": self.n_corners, "rms_mm": self.rms_mm, "max_mm": self.max_mm,
                "pos_mm": self.pos_mm, "ang_deg": self.ang_deg}


@dataclass
class FrameFit:
    """Result of fit_frame. rms_mm / max_mm: corner residuals over all boards (0 for a single board - one board
    cannot be checked against anything). baseline_mm: largest distance between two board centres (0 for one board),
    the lever for the heading; span_mm: largest distance between two corners used."""
    parent: str
    T_base_parent: np.ndarray
    residuals: dict[str, BoardResidual]
    rms_mm: float
    max_mm: float
    n_boards: int
    boards: list[str]
    baseline_mm: float
    span_mm: float
    n_points: int
    ignored: list[str] = field(default_factory=list)   # observed boards of another parent / without a placement

    def to_dict(self) -> dict:
        return {"parent": self.parent, "T_base_parent": self.T_base_parent.tolist(),
                "residuals": {k: r.to_dict() for k, r in self.residuals.items()}, "rms_mm": self.rms_mm,
                "max_mm": self.max_mm, "n_boards": self.n_boards, "boards": list(self.boards),
                "baseline_mm": self.baseline_mm, "span_mm": self.span_mm, "n_points": self.n_points,
                "ignored": list(self.ignored)}


def fit_frame(observed: Mapping[str, np.ndarray], placements: Sequence[Placement], specs: Mapping[str, BoardSpec],
              parent: str = "wall") -> FrameFit:
    """T_base_parent from measured board poses {name: T_base_board} (rigid Kabsch fit of all chessboard corners).

    Boards without a placement or of another parent are ignored (listed in FrameFit.ignored). ValueError when no
    observed board belongs to `parent`."""
    pl = {p.name: p for p in placements}
    use = sorted(n for n in observed if n in pl and pl[n].parent == parent)
    ignored = sorted(n for n in observed if n not in use)
    if not use:
        raise ValueError(f"no observed board of parent {parent!r} (observed: {sorted(observed)})")
    P_par, P_base, idx = [], [], []
    for n in use:
        if n not in specs:
            raise ValueError(f"board {n!r} has a placement but no BoardSpec")
        c = corners_obj(specs[n])
        P_par.append(g.apply(pl[n].T_parent_board, c))
        P_base.append(g.apply(np.asarray(observed[n], float), c))
        idx.append(len(c))
    A, B = np.vstack(P_par), np.vstack(P_base)
    T = g.fit_rigid(A, B)
    err = np.linalg.norm(g.apply(T, A) - B, axis=1)
    res: dict[str, BoardResidual] = {}
    k = 0
    for n, m in zip(use, idx):
        e = err[k:k + m]
        k += m
        d_pos, d_ang = g.pose_delta(T @ pl[n].T_parent_board, np.asarray(observed[n], float))
        res[n] = BoardResidual(n, int(m), float(np.sqrt(np.mean(e ** 2))), float(e.max()), d_pos, d_ang)
    centres = [board_centre(specs[n], pl[n].T_parent_board) for n in use]
    baseline = max((float(np.linalg.norm(a - b)) for a, b in combinations(centres, 2)), default=0.0)
    lo, hi = A.min(axis=0), A.max(axis=0)
    span = float(np.linalg.norm(hi - lo))           # bounding-box diagonal: >= any corner distance, cheap
    return FrameFit(parent, T, res, float(np.sqrt(np.mean(err ** 2))), float(err.max()), len(use), use, baseline,
                    span, int(len(A)), ignored)


# ── planar ARES pose ──────────────────────────────────────────────────────────
def wrap_angle(a: float) -> float:
    """Angle wrapped to (-pi, pi] [rad]."""
    w = (float(a) + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if w == -math.pi else w


@dataclass(frozen=True)
class Pose2D:
    """Planar pose of the ARES frame in a floor-level parent frame (wall / station): position [mm], heading [rad]
    (angle of the ARES x axis about the parent z axis)."""
    x_mm: float
    y_mm: float
    theta_rad: float

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)

    @property
    def T(self) -> np.ndarray:
        return planar_T(self.x_mm, self.y_mm, self.theta_rad)

    @classmethod
    def from_T(cls, T: np.ndarray) -> "Pose2D":
        """Planar part of a 4x4 pose: (x, y) and the heading of its x axis projected onto the parent xy plane."""
        T = np.asarray(T, float)
        return cls(float(T[0, 3]), float(T[1, 3]), float(math.atan2(T[1, 0], T[0, 0])))

    def to_dict(self) -> dict:
        return {"x_mm": float(self.x_mm), "y_mm": float(self.y_mm), "theta_rad": float(self.theta_rad)}

    @classmethod
    def from_dict(cls, d: Mapping) -> "Pose2D":
        return cls(float(d["x_mm"]), float(d["y_mm"]), float(d["theta_rad"]))

    def delta(self, other: "Pose2D") -> tuple[float, float]:
        """(position distance [mm], |heading difference| [deg]) to another pose."""
        return (math.hypot(other.x_mm - self.x_mm, other.y_mm - self.y_mm),
                abs(math.degrees(wrap_angle(other.theta_rad - self.theta_rad))))

    def describe(self) -> str:
        return f"({self.x_mm:.1f}, {self.y_mm:.1f}) mm, {self.theta_deg:.2f} deg"


def planar_T(x_mm: float, y_mm: float, theta_rad: float) -> np.ndarray:
    """transl(x, y, 0) @ rotz(theta)."""
    return g.transl(x_mm, y_mm, 0.0) @ g.rotz(theta_rad)


def T_parent_ares(T_base_parent: np.ndarray, T_ares_base: np.ndarray) -> np.ndarray:
    """ARES frame in the parent (wall / station) frame from the measured parent frame in the UR base frame:
    inv(T_ares_base @ T_base_parent)."""
    return g.inv(np.asarray(T_ares_base, float) @ np.asarray(T_base_parent, float))


def T_wall_ares(T_base_wall: np.ndarray, T_ares_base: np.ndarray) -> np.ndarray:
    """ARES frame in the wall frame (see T_parent_ares)."""
    return T_parent_ares(T_base_wall, T_ares_base)


def ares_pose(T_base_parent: np.ndarray, T_ares_base: np.ndarray) -> Pose2D:
    """Planar ARES pose (x, y, theta) in the parent frame."""
    return Pose2D.from_T(T_parent_ares(T_base_parent, T_ares_base))


def tilt_deg(T_parent_child: np.ndarray) -> float:
    """Angle between the child z axis and the parent z axis [deg] (ARES roll/pitch on its sprung casters)."""
    z = np.asarray(T_parent_child, float)[:3, 2]
    return float(np.degrees(np.arccos(np.clip(z[2], -1.0, 1.0))))


def T_base_parent_from(pose: Pose2D, T_ares_base: np.ndarray) -> np.ndarray:
    """Parent frame in the UR base frame for a planar ARES pose (no tilt): inv(T_ares_base) @ inv(pose.T)."""
    return g.inv(np.asarray(T_ares_base, float)) @ g.inv(pose.T)


def relative_move(p_from: Pose2D, p_to: Pose2D) -> tuple[float, float, float]:
    """(dx_mm, dy_mm, dtheta_rad): position of p_to in the ARES body frame at p_from (the frame of the PLC relative
    translation) and the heading change wrapped to (-pi, pi]."""
    dxw, dyw = p_to.x_mm - p_from.x_mm, p_to.y_mm - p_from.y_mm
    c, s = math.cos(p_from.theta_rad), math.sin(p_from.theta_rad)
    return c * dxw + s * dyw, -s * dxw + c * dyw, wrap_angle(p_to.theta_rad - p_from.theta_rad)


def after_translation(p: Pose2D, dx_mm: float, dy_mm: float) -> Pose2D:
    """Pose after an ideal body-frame translation (dx forward, dy left) from p."""
    c, s = math.cos(p.theta_rad), math.sin(p.theta_rad)
    return Pose2D(p.x_mm + c * dx_mm - s * dy_mm, p.y_mm + s * dx_mm + c * dy_mm, p.theta_rad)


def after_rotation(p: Pose2D, dtheta_rad: float) -> Pose2D:
    """Pose after an ideal rotation on the spot."""
    return Pose2D(p.x_mm, p.y_mm, wrap_angle(p.theta_rad + dtheta_rad))
