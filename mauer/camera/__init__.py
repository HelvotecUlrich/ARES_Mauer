"""Cameras behind one interface (docs/ARCHITECTURE.md, `mauer.camera`).

    from mauer import config
    from mauer.camera import open_camera
    with open_camera(config.load()) as cam:              # IDS GV-51F0CP via IDS peak, settings from [camera]
        frame = cam.grab()                               # Frame: image uint8 (H, W), t_start, t_end [s], meta

- `ids.IdsCamera`: IDS peak (GigE Vision), software trigger, Mono8 – needs the IDS peak runtime (docs/CAMERA_SETUP.md).
- `files.FileCamera`: replays a folder of images (tests, recorded datasets; sidecar JSON keeps the time stamps).
- RoboDK simulated camera: robodk/sim_camera.py (same interface, not importable from here – mauer has no RoboDK).

ids_peak is imported lazily, so `import mauer.camera` works without it.
"""
from __future__ import annotations

from typing import Any

from .base import Camera, CameraError, Frame, focus_measure, image_stats
from .files import FileCamera, save_frame

KINDS = ("ids", "files")


def open_camera(cfg: dict | None = None, kind: str = "ids", **kw: Any) -> Camera:
    """Create and open a camera; usable as `with open_camera(cfg) as cam:`.

    kind "ids": IdsCamera.from_config(cfg, **kw) – kw overrides [camera] keys (serial, ip, exposure_us, gain,
    timeout_ms, packet_size, ...; None = keep the config value).
    kind "files": FileCamera(folder=kw["folder"], pattern=..., loop=...); exposure_us/gain default to [camera] so the
    replayed meta looks like the real camera's.
    cfg None = config/station.toml.
    """
    if kind not in KINDS:
        raise ValueError(f"unknown camera kind {kind!r}, expected one of {KINDS}")
    if cfg is None:
        from .. import config
        cfg = config.load()
    if kind == "ids":
        from .ids import IdsCamera
        cam: Camera = IdsCamera.from_config(cfg, **kw)
    else:
        if not kw.get("folder"):
            raise ValueError("open_camera(kind='files') needs folder=<image folder>")
        c = cfg.get("camera", {})
        kw = {k: v for k, v in kw.items() if v is not None}
        kw.setdefault("exposure_us", c.get("exposure_us"))
        kw.setdefault("gain", c.get("gain"))
        cam = FileCamera(**kw)
    cam.open()
    return cam


__all__ = ["Camera", "CameraError", "Frame", "FileCamera", "open_camera", "save_frame", "image_stats",
           "focus_measure", "KINDS"]
