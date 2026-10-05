"""Vision for the eye-in-hand camera: ChArUco boards, detection, board pose, intrinsics, hand-eye calibration.

Free software only (OpenCV 4.14.0.94 pinned, reportlab for the printed targets; no HALCON). Conventions
(docs/ARCHITECTURE.md): T_a_b = pose of frame b in frame a, 4x4 float64, mm and rad; camera frame = OpenCV (z optical
axis, x image right, y image down); board frame = OpenCV 4.14 CharucoBoard (origin top-left outer corner, x right,
y down, z into the board).

Modules: targets (board specs from config/station.toml), detect (corners + pose), intrinsics (camera model, JSON,
calibration), handeye (calibrateHandEye, validation, look-pose planning), synth (synthetic images for tests),
printables (exact-scale PDF / RoboDK PNG), dataset (image + robot pose folders written by the capture tools).
"""
from __future__ import annotations

from .detect import BoardDetection, BoardPose, detect_boards, estimate_pose, measure
from .intrinsics import Intrinsics
from .targets import BoardSpec, board_specs, corners_obj, make_board

__all__ = ["BoardDetection", "BoardPose", "BoardSpec", "Intrinsics", "board_specs", "corners_obj", "detect_boards",
           "estimate_pose", "make_board", "measure"]
