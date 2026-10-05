"""Replay camera: grabs the images of a folder in sorted order, plus helpers to record frames for later replay.

`FileCamera(folder, pattern="*.png", loop=False)` returns the files sorted by name as grayscale uint8 (H, W) frames
(OpenCV IMREAD_GRAYSCALE, read via imdecode so non-ASCII Windows paths work). Time stamps:

- if a sidecar `<image>.json` written by `save_frame()` exists (tools/cam_check.py grab, recorded datasets), its
  recorded `t_start`, `t_end` and `meta` are replayed (meta["replayed"] = True);
- otherwise they are synthetic: t_start = clock() at grab(), t_end = t_start + exposure (meta["synthetic_time"] = True).

set_exposure_us / set_gain change nothing in the images; the requested values are only recorded in the meta
(meta["exposure_applied"] = False). Used for tests and for re-running the vision pipeline on recorded images.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .base import Camera, CameraError, Frame

SIDECAR_SUFFIX = ".json"


def _json_default(o: Any) -> Any:
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")


def read_gray(path: str | Path) -> np.ndarray:
    """Image file -> uint8 (H, W) (cv2.IMREAD_GRAYSCALE; 16-bit files are scaled to 8 bit by OpenCV)."""
    import cv2

    data = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE) if data.size else None
    if img is None:
        raise CameraError(f"cannot read image {path} (not an image OpenCV can decode?)")
    return img


def save_frame(frame: Frame, path: str | Path) -> tuple[Path, Path]:
    """Write the image (PNG, lossless; format from the suffix) and a JSON sidecar with t_start, t_end and meta.

    Returns (image path, sidecar path). The sidecar lets FileCamera replay the recorded time stamps.
    """
    import cv2

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(path.suffix or ".png", frame.image)
    if not ok:
        raise CameraError(f"cannot encode image for {path}")
    buf.tofile(str(path))
    side = path.with_name(path.name + SIDECAR_SUFFIX)
    side.write_text(json.dumps({"t_start": frame.t_start, "t_end": frame.t_end, "meta": frame.meta},
                               indent=1, default=_json_default), encoding="utf-8")
    return path, side


def load_sidecar(image_path: str | Path) -> dict | None:
    """The JSON sidecar of an image written by save_frame(), or None if there is none."""
    side = Path(image_path).with_name(Path(image_path).name + SIDECAR_SUFFIX)
    if not side.is_file():
        return None
    return json.loads(side.read_text(encoding="utf-8"))


class FileCamera(Camera):
    """Replays the images of a folder (sorted by file name) as if they came from a camera."""

    def __init__(self, folder: str | Path, pattern: str = "*.png", loop: bool = False,
                 exposure_us: float | None = None, gain: float | None = None, use_sidecar: bool = True,
                 clock: Callable[[], float] = time.time) -> None:
        self.folder = Path(folder)
        self.pattern = pattern
        self.loop = loop
        self.use_sidecar = use_sidecar
        self.clock = clock
        self._exposure_us = exposure_us
        self._gain = gain
        self._files: list[Path] = []
        self._index = 0                 # next file to return
        self._n_grabbed = 0             # running frame counter (frame_id)
        self._open = False

    # ── Camera interface ─────────────────────────────────────────────────────
    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        if self._open:
            return
        if not self.folder.is_dir():
            raise CameraError(f"FileCamera: folder {self.folder} does not exist")
        self._files = sorted((p for p in self.folder.glob(self.pattern) if p.is_file()), key=lambda p: p.name)
        if not self._files:
            raise CameraError(f"FileCamera: no files matching {self.pattern!r} in {self.folder}")
        self._index = 0
        self._n_grabbed = 0
        self._open = True

    def close(self) -> None:
        self._open = False

    def grab(self) -> Frame:
        if not self._open:
            raise CameraError("FileCamera: grab() before open()")
        if self._index >= len(self._files):
            if not self.loop:
                raise CameraError(f"FileCamera: all {len(self._files)} images of {self.folder} replayed "
                                  f"(loop=False)")
            self._index = 0
        path = self._files[self._index]
        self._index += 1
        img = read_gray(path)
        side = load_sidecar(path) if self.use_sidecar else None
        if side is not None:
            # recorded frame: keep its time stamps and meta (camera, frame_id, exposure_us, ... of the recording)
            meta: dict[str, Any] = dict(side.get("meta", {}))
            t_start, t_end = float(side["t_start"]), float(side["t_end"])
            meta["replayed"] = True
            meta.setdefault("camera", f"files:{self.folder.name}")
            meta.setdefault("frame_id", self._n_grabbed)
            if self._exposure_us is not None:
                meta["exposure_us_requested"] = self._exposure_us
        else:
            t_start = self.clock()
            t_end = t_start + (self._exposure_us or 0.0) * 1e-6
            meta = {"camera": f"files:{self.folder.name}", "frame_id": self._n_grabbed, "synthetic_time": True,
                    "exposure_us": self._exposure_us, "gain": self._gain}
        meta.update({"file": path.name, "index": self._index - 1, "exposure_applied": False})
        self._n_grabbed += 1
        return Frame(img, t_start, t_end, meta)

    def set_exposure_us(self, us: float) -> float:
        """No-op for the images; recorded in the meta of the following frames."""
        self._exposure_us = float(us)
        return self._exposure_us

    def set_gain(self, gain: float) -> float:
        """No-op for the images; recorded in the meta of the following frames."""
        self._gain = float(gain)
        return self._gain

    def info(self) -> dict[str, Any]:
        return {"kind": "files", "model": "FileCamera", "folder": str(self.folder), "pattern": self.pattern,
                "loop": self.loop, "n_images": len(self._files), "next_index": self._index, "open": self._open,
                "exposure_us": self._exposure_us, "gain": self._gain}

    # ── extras ───────────────────────────────────────────────────────────────
    def __len__(self) -> int:
        return len(self._files)

    @property
    def files(self) -> list[Path]:
        return list(self._files)
