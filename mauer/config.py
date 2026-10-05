"""Config access: config/station.toml (all parameters with status tags) and the calibration files in calib/."""
from __future__ import annotations

import tomllib
from pathlib import Path

import numpy as np

from . import REPO
from .geometry import pose_xyz_rpy

STATION_TOML = REPO / "config" / "station.toml"


def load(path: str | Path | None = None) -> dict:
    """Station config as a plain dict (same content as robodk/rdk_common.load_config())."""
    with open(Path(path) if path else STATION_TOML, "rb") as f:
        return tomllib.load(f)


def repo_path(rel: str | Path) -> Path:
    """Absolute path for a repo-relative path from the config (absolute paths pass through)."""
    p = Path(rel)
    return p if p.is_absolute() else REPO / p


def pose(table: dict) -> np.ndarray:
    """4x4 pose from a config table with `xyz` [mm] and `rpy_deg` [deg] (see geometry.pose_xyz_rpy)."""
    return pose_xyz_rpy(table["xyz"], table.get("rpy_deg", (0.0, 0.0, 0.0)))


def leg_frames(cfg: dict) -> dict[str, np.ndarray]:
    """T_wall_leg of every [[wall.legs]] entry (xyz_in_wall [mm], rpy_in_wall_deg [deg]); {} for a straight wall.
    Same numbers as robodk/wallplan.py legs() (planar, pure Python)."""
    out = {}
    for d in (cfg.get("wall", {}) or {}).get("legs", []) or []:
        out[str(d["name"])] = pose_xyz_rpy(d.get("xyz_in_wall", (0.0, 0.0, 0.0)),
                                           d.get("rpy_in_wall_deg", (0.0, 0.0, 0.0)))
    return out


def T_flange_cam_nominal(cfg: dict) -> np.ndarray:
    """Camera frame (OpenCV: z optical axis, x right, y down in the image) in the flange frame, from the mount
    PLACEHOLDER in [camera.mount]. The calibrated value comes from calib/handeye.json (mauer.vision.handeye)."""
    return pose(cfg["camera"]["mount"])


def T_flange_tcp(cfg: dict) -> np.ndarray:
    """Gripper TCP in the flange frame – identical to robodk/rdk_common.tcp_pose(): z offset, then rotz(90°)."""
    from .geometry import rotz, transl
    return transl(0.0, 0.0, cfg["tool"]["tcp_z"]) @ rotz(np.pi / 2)


def T_ares_base(cfg: dict) -> np.ndarray:
    """UR5 base frame in the ARES frame – identical to robodk/rdk_common.ur5_base_pose()."""
    from .geometry import rotz, transl
    u = cfg["ur5"]
    return transl(u["mount_x"], u["mount_y"], u["mount_z"]) @ rotz(np.radians(u["mount_rz"]))
