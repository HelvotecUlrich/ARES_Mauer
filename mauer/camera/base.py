"""Camera interface shared by the real IDS camera, the file replay and the RoboDK simulated camera.

Contract (docs/ARCHITECTURE.md, `mauer.camera`):

- `Frame`: one mono image `image` (numpy uint8, shape (H, W)) with the laptop wall-clock times `t_start` (time.time()
  just before the trigger) and `t_end` (time.time() just after the finished buffer arrived) [s, Unix epoch], plus a
  `meta` dict (exposure_us, gain, frame_id, camera timestamp, ... – whatever the source knows).
  The exposure lies inside [t_start, t_end]; for the IDS camera t_end also contains the GigE transfer of the frame
  (~45 ms for Mono8, research_out/ids-camera.json, ASSUMPTION from 5.1 MB at ~120 MB/s). Robot poses for the image are
  taken from the RTDE history inside that window while the arm stands still ([camera] settle_s).
- `Camera`: open(), close(), grab() -> Frame, set_exposure_us(us) -> applied us, set_gain(g) -> applied gain,
  info() -> dict, context manager (`with cam:` opens if needed and always closes).
- `CameraError`: every failure a caller can act on (no runtime, no device, timeout, incomplete frame, end of a
  replay folder). Implementations translate vendor exceptions into it, with an actionable message.

Also here: small image-quality measures used by tools/cam_check.py (grey statistics, saturation, focus measure).
Units: exposure in µs, gain as a factor (1.0 = no gain), times in s.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any

import numpy as np


class CameraError(RuntimeError):
    """Camera failure with a message that says what to check (never a raw vendor/SWIG exception)."""


@dataclass
class Frame:
    """One grabbed image. `image` is uint8 (H, W) for Mono8 (the contract); unpacked Mono10/12 sources give uint16."""

    image: np.ndarray
    t_start: float                      # laptop time.time() just before the trigger [s]
    t_end: float                        # laptop time.time() after the finished buffer arrived [s]
    meta: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.image, np.ndarray) or self.image.ndim != 2:
            raise ValueError(f"Frame.image must be a 2-D mono numpy array, got "
                             f"{type(self.image).__name__} {getattr(self.image, 'shape', None)}")
        if self.t_end < self.t_start:
            raise ValueError(f"Frame t_end {self.t_end} < t_start {self.t_start}")

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    @property
    def t_mid(self) -> float:
        """Middle of the [t_start, t_end] window [s] – a first guess for the exposure time stamp."""
        return 0.5 * (self.t_start + self.t_end)

    @property
    def latency_s(self) -> float:
        """t_end - t_start [s]: trigger + exposure + readout + transfer (+ host overhead)."""
        return self.t_end - self.t_start


class Camera(abc.ABC):
    """Common camera interface. Subclasses implement the abstract methods; the context manager is provided here.

    `with cam:` calls open() if the camera is not open yet (so `with open_camera(...) as cam:` works too) and close()
    on exit, also after an exception. close() must be safe to call more than once.
    """

    @property
    @abc.abstractmethod
    def is_open(self) -> bool: ...

    @abc.abstractmethod
    def open(self) -> None:
        """Connect and get ready to grab; no-op if already open. Raises CameraError."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release everything; safe to call twice and after a failed open()."""

    @abc.abstractmethod
    def grab(self) -> Frame:
        """Take one image now (software trigger) and return it. Raises CameraError."""

    @abc.abstractmethod
    def set_exposure_us(self, us: float) -> float:
        """Set the exposure time [µs]; returns the value actually applied (clamped/rounded by the device)."""

    @abc.abstractmethod
    def set_gain(self, gain: float) -> float:
        """Set the gain factor (1.0 = no gain); returns the value actually applied."""

    @abc.abstractmethod
    def info(self) -> dict[str, Any]:
        """Identity and current settings (model, serial, firmware, exposure_us, gain, ...), JSON-serialisable."""

    def __enter__(self) -> "Camera":
        if not self.is_open:
            self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# ── image quality measures (tools/cam_check.py, sanity checks) ───────────────
def full_scale(img: np.ndarray) -> int:
    """Largest grey value of the image dtype (255 for uint8)."""
    return int(np.iinfo(img.dtype).max) if np.issubdtype(img.dtype, np.integer) else 1


def image_stats(img: np.ndarray) -> dict[str, float]:
    """Grey statistics: mean/std/min/max and the share of saturated (= full scale) and black (= 0) pixels [%]."""
    fs = full_scale(img)
    n = img.size
    return {
        "mean": float(img.mean()),
        "std": float(img.std()),
        "min": int(img.min()),
        "max": int(img.max()),
        "saturated_pct": 100.0 * float(np.count_nonzero(img >= fs)) / n,
        "black_pct": 100.0 * float(np.count_nonzero(img == 0)) / n,
    }


def center_roi(img: np.ndarray, frac: float = 0.25) -> tuple[slice, slice]:
    """Row/column slices of a centred ROI whose sides are `frac` of the image sides (0 < frac <= 1)."""
    if not 0.0 < frac <= 1.0:
        raise ValueError(f"ROI fraction must be in (0, 1], got {frac}")
    h, w = img.shape[:2]
    rh, rw = max(3, int(round(h * frac))), max(3, int(round(w * frac)))
    r0, c0 = (h - rh) // 2, (w - rw) // 2
    return slice(r0, r0 + rh), slice(c0, c0 + rw)


def focus_measure(img: np.ndarray, roi_frac: float = 0.25) -> float:
    """Variance of the Laplacian in the centre ROI (Pech-Pacheco et al. 2000, 'variance of Laplacian').

    Relative measure only: it depends on scene content, exposure and noise. Use it to compare while turning the focus
    ring with the camera and target standing still at the working distance – maximum = sharpest.
    """
    import cv2  # lazy: base.py stays importable without OpenCV

    rs, cs = center_roi(img, roi_frac)
    roi = np.ascontiguousarray(img[rs, cs])
    return float(cv2.Laplacian(roi, cv2.CV_64F, ksize=3).var())


def histogram(img: np.ndarray, bins: int = 256) -> np.ndarray:
    """Grey histogram over the full dtype range (counts, int64)."""
    fs = full_scale(img)
    h, _ = np.histogram(img, bins=bins, range=(0, fs + 1))
    return h
