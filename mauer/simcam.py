"""Synthetic camera for tests without hardware: renders the ChArUco boards the camera would see from a robot pose.

The flange pose comes from a callable (URSim via URLink, or the pure-Python world of mauer.simworld), the boards are
given in the robot base frame (a callable, because the base moves with ARES in the world). Images come from
mauer.vision.synth (pinhole + optional distortion, blur, noise) – no occlusion by stones, gripper or ARES; that is
what the RoboDK simulation (robodk/sim_camera.py) is for.
"""
from __future__ import annotations

import time
from typing import Callable, Sequence

import numpy as np

from . import geometry as g
from .camera.base import Camera, CameraError, Frame
from .vision.intrinsics import Intrinsics
from .vision.synth import render_boards
from .vision.targets import BoardSpec

BoardList = Sequence[tuple[BoardSpec, np.ndarray]]      # [(spec, T_base_board)]


class SynthCamera(Camera):
    """Camera whose images are rendered from `pose_source()` (true T_base_flange, mm) and `T_flange_cam` (true)."""

    def __init__(self, pose_source: Callable[[], np.ndarray], T_flange_cam: np.ndarray,
                 boards: Callable[[], BoardList] | BoardList, intr: Intrinsics, *, blur_sigma_px: float = 0.6,
                 noise_sigma: float = 1.0, supersample: int = 2, distort: bool = True, max_dist_mm: float = 2000.0,
                 seed: int = 0, clock: Callable[[], float] = time.time, exposure_us: float = 5000.0,
                 gain: float = 1.0) -> None:
        self.pose_source = pose_source
        self.T_flange_cam = np.asarray(T_flange_cam, float)
        self._boards = boards
        self.intr = intr
        self.blur_sigma_px, self.noise_sigma = blur_sigma_px, noise_sigma
        self.supersample, self.distort, self.max_dist_mm = supersample, distort, max_dist_mm
        self.seed, self.clock = seed, clock
        self.exposure_us, self.gain = float(exposure_us), float(gain)
        self._open = False
        self._n = 0

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def boards(self) -> BoardList:
        return list(self._boards() if callable(self._boards) else self._boards)

    def visible(self, T_base_cam: np.ndarray) -> list[tuple[BoardSpec, np.ndarray]]:
        """(spec, T_cam_board) of the boards in front of the camera with their printed side towards it."""
        out = []
        T_cam_base = g.inv(T_base_cam)
        for spec, T_base_board in self.boards():
            T = T_cam_base @ T_base_board
            bw, bh = spec.size_mm
            q = spec.square_mm
            quad = g.apply(T, np.array([[-q, -q, 0], [bw + q, -q, 0], [bw + q, bh + q, 0], [-q, bh + q, 0]]))
            if (quad[:, 2] <= 1.0).any() or quad[:, 2].min() > self.max_dist_mm:
                continue
            if T[2, 2] <= 0.0:                    # board z (into the board) must point away from the camera
                continue
            out.append((spec, T))
        return out

    def grab(self) -> Frame:
        if not self._open:
            raise CameraError("SynthCamera is not open")
        t_start = self.clock()
        T_base_flange = np.asarray(self.pose_source(), float)
        T_base_cam = T_base_flange @ self.T_flange_cam
        items = self.visible(T_base_cam)
        img = render_boards(items, self.intr, supersample=self.supersample, blur_sigma_px=self.blur_sigma_px,
                            noise_sigma=self.noise_sigma, distort=self.distort, seed=self.seed + self._n)
        self._n += 1
        t_end = self.clock()
        meta = {"kind": "synth", "exposure_us": self.exposure_us, "gain": self.gain, "frame_id": self._n,
                "boards_rendered": [s.name for s, _ in items], "T_base_flange_true": T_base_flange.tolist()}
        return Frame(img, t_start, max(t_end, t_start), meta)

    def set_exposure_us(self, us: float) -> float:
        self.exposure_us = float(us)
        return self.exposure_us

    def set_gain(self, gain: float) -> float:
        self.gain = float(gain)
        return self.gain

    def info(self) -> dict:
        return {"kind": "synth", "width": self.intr.width, "height": self.intr.height,
                "exposure_us": self.exposure_us, "gain": self.gain, "boards": [s.name for s, _ in self.boards()]}
