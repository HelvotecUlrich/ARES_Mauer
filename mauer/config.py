"""Config access: config/station.toml (all parameters with status tags) and the calibration files in calib/."""
from __future__ import annotations

import tomllib
from pathlib import Path

import numpy as np

from . import REPO
from .geometry import pose_xyz_rpy

STATION_TOML = REPO / "config" / "station.toml"
VARIANTS = REPO / "config" / "variants"


def variant_path(name: str) -> Path:
    """config/variants/<name>.toml; FileNotFoundError listing the known variants if it does not exist."""
    p = VARIANTS / f"{name}.toml"
    if not p.is_file():
        known = sorted(q.stem for q in VARIANTS.glob("*.toml")) if VARIANTS.is_dir() else []
        raise FileNotFoundError(f"config variant {name!r} not found ({p}); known: {known}")
    return p


def merge(base: dict, over: dict) -> dict:
    """base with `over` laid over it: tables merged key by key (recursively), every other value - also an array or an
    array of tables such as [[wall.legs]] - replaced as a whole."""
    out = dict(base)
    for k, v in over.items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load(path: str | Path | None = None, variant: str | None = None) -> dict:
    """Station config as a plain dict (same content as robodk/rdk_common.load_config()). variant: the overlay
    config/variants/<variant>.toml merged over it (merge(); 2026-10-07: wall layouts kept side by side) - the result
    carries its name in cfg["_variant"] (variant_of, suffix)."""
    with open(Path(path) if path else STATION_TOML, "rb") as f:
        cfg = tomllib.load(f)
    if variant:
        with open(variant_path(variant), "rb") as f:
            cfg = merge(cfg, tomllib.load(f))
        cfg["_variant"] = str(variant)
    return cfg


def variant_of(cfg: dict) -> str | None:
    """Name of the config variant cfg was loaded with (None = config/station.toml alone)."""
    return cfg.get("_variant") or None


def suffix(cfg: dict) -> str:
    """'_<variant>' for output file names of a variant config, '' for the main config."""
    v = variant_of(cfg)
    return f"_{v}" if v else ""


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


def stack_top_z(cfg: dict, z0: float, layer: int, kind: str = "full") -> float:
    """Top of the stone in stack layer `layer` (1 = lowest) of stones standing on z0 (magazine, station): the stone
    heights plus [brick] bed_joint between them (the pins are 1 mm longer than the sockets are deep, 2026-10-06)."""
    b = cfg["brick"]
    h = float((cfg.get("half_brick") or {}).get("height", b["height"])) if kind == "half" else float(b["height"])
    return float(z0) + layer * h + (layer - 1) * float(b.get("bed_joint", 0.0))


def T_flange_tcp(cfg: dict) -> np.ndarray:
    """Gripper TCP in the flange frame – identical to robodk/rdk_common.tcp_pose(): z offset, then rotz(90°)."""
    from .geometry import rotz, transl
    return transl(0.0, 0.0, cfg["tool"]["tcp_z"]) @ rotz(np.pi / 2)


def T_ares_base(cfg: dict) -> np.ndarray:
    """UR5 base frame in the ARES frame – identical to robodk/rdk_common.ur5_base_pose()."""
    from .geometry import rotz, transl
    u = cfg["ur5"]
    return transl(u["mount_x"], u["mount_y"], u["mount_z"]) @ rotz(np.radians(u["mount_rz"]))
