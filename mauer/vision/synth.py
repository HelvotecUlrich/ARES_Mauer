"""Synthetic camera images of ChArUco boards with known ground truth (tests and tool self-checks, no hardware).

Adapted from the research renderer simcam.py (2026-10-05), which reproduced OpenCV's ChArUco corners to 0.026 px
mean / 0.093 px max on the GV-51F0CP geometry. Steps:
1. board texture = CharucoBoard.generateImage with an integer number of texture pixels per square and a white quiet
   zone of one square (cached per board);
2. ideal pinhole image at `supersample` x resolution via warpPerspective (homography K_ss [r1 r2 t] of the board
   plane), several boards composited in order;
3. INTER_AREA downsampling (box average over supersample x supersample pixels = finite pixel area);
4. lens distortion by remap: every output pixel samples the ideal image at its undistorted position
   (cv2.undistortPoints(..., P=K)), i.e. exactly the distortion model of cv2.projectPoints;
5. optional Gaussian blur (defocus / diffraction) and Gaussian noise (seeded).

Pixel convention: OpenCV, pixel centres at integer coordinates; a low-res pixel u covers the supersampled pixels
ss*u .. ss*u + ss - 1, so cx_ss = ss*cx + (ss - 1)/2. Lengths in mm. T_cam_board: board -> OpenCV camera frame.
"""
from __future__ import annotations

from functools import lru_cache
from typing import Sequence

import cv2
import numpy as np

from .. import geometry as g
from .intrinsics import Intrinsics
from .targets import BoardSpec, T_board_cam_looking_at, make_board

_DIST_MAPS: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}
_UNDIST_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-9)


@lru_cache(maxsize=16)
def board_texture(spec: BoardSpec, px_per_square: int) -> tuple[np.ndarray, float, float]:
    """(texture uint8, texture px per mm, texture pixel coordinate of the board origin) incl. one square of white
    quiet zone on every side. The board edge (0 mm) is the boundary between two texture pixels -> origin at
    px_per_square - 0.5 in pixel-centre coordinates."""
    n = int(px_per_square)
    size = ((spec.squares_x + 2) * n, (spec.squares_y + 2) * n)
    tex = make_board(spec).generateImage(size, marginSize=n, borderBits=1)
    return tex, n / spec.square_mm, n - 0.5


def _undistort_points(pts: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """Ideal pixel coordinates of distorted pixels; iterative (OpenCV 4.x undistortPointsIter, 5.x criteria=)."""
    if hasattr(cv2, "undistortPointsIter"):
        return cv2.undistortPointsIter(pts, K, D, None, K, _UNDIST_CRITERIA)
    return cv2.undistortPoints(pts, K, D, R=None, P=K, criteria=_UNDIST_CRITERIA)   # pragma: no cover (OpenCV 5)


def distortion_map(K: np.ndarray, D: np.ndarray, size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """remap() maps (float32 x, y) taking a distorted image from an ideal one with the same K (cached)."""
    key = (np.asarray(K, float).tobytes(), np.asarray(D, float).tobytes(), tuple(size))
    if key not in _DIST_MAPS:
        W, H = size
        u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
        pts = np.stack([u.ravel(), v.ravel()], axis=1).reshape(-1, 1, 2)
        und = _undistort_points(pts, np.asarray(K, float), np.asarray(D, float)).reshape(H, W, 2)
        if len(_DIST_MAPS) >= 4:
            _DIST_MAPS.pop(next(iter(_DIST_MAPS)))
        _DIST_MAPS[key] = (np.ascontiguousarray(und[..., 0]), np.ascontiguousarray(und[..., 1]))
    return _DIST_MAPS[key]


def render_boards(items: Sequence[tuple[BoardSpec, np.ndarray]], intr: Intrinsics,
                  size: tuple[int, int] | None = None, supersample: int = 3, blur_sigma_px: float = 0.0,
                  noise_sigma: float = 0.0, distort: bool = True, background: int = 255, seed: int = 0,
                  tex_px_per_mm: float = 40.0) -> np.ndarray:
    """uint8 (H, W) image of several boards [(spec, T_cam_board), ...]; later boards cover earlier ones.

    size = (W, H) [px], default (intr.width, intr.height). noise_sigma in grey values (DN), blur in px.
    tex_px_per_mm: texture resolution (40 px/mm >= 3 x the 13.7 px/mm the camera resolves at 320 mm).
    Raises ValueError when a board (incl. quiet zone) reaches behind the camera."""
    W, H = (intr.width, intr.height) if size is None else (int(size[0]), int(size[1]))
    ss = int(supersample)
    if ss < 1:
        raise ValueError("supersample must be >= 1")
    Kss = intr.K.copy()
    Kss[:2] *= ss
    Kss[0, 2] += (ss - 1) / 2.0
    Kss[1, 2] += (ss - 1) / 2.0
    canvas = np.full((H * ss, W * ss), int(background), np.uint8)
    for spec, T in items:
        T = np.asarray(T, float)
        n = max(int(round(spec.square_mm * tex_px_per_mm)), 8)
        tex, s, o = board_texture(spec, n)
        q = spec.square_mm                                                  # quiet zone = one square
        bw, bh = spec.size_mm
        quad_mm = np.array([[-q, -q, 0], [bw + q, -q, 0], [bw + q, bh + q, 0], [-q, bh + q, 0]])
        if (g.apply(T, quad_mm)[:, 2] <= 1e-6).any():
            raise ValueError(f"board {spec.name} reaches behind the camera")
        H_mm = Kss @ np.column_stack([T[:3, 0], T[:3, 1], T[:3, 3]])        # board plane (mm) -> ss image
        H_tex = H_mm @ np.linalg.inv(np.array([[s, 0.0, o], [0.0, s, o], [0.0, 0.0, 1.0]]))
        warped = cv2.warpPerspective(tex, H_tex, (W * ss, H * ss), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=int(background))
        if len(items) == 1:
            canvas = warped
            continue
        quad = cv2.perspectiveTransform(quad_mm[:, :2].reshape(-1, 1, 2), H_mm).reshape(-1, 2)
        mask = np.zeros_like(canvas)
        cv2.fillConvexPoly(mask, np.round(quad * 16).astype(np.int32), 255, cv2.LINE_8, shift=4)
        canvas[mask > 0] = warped[mask > 0]
    img = cv2.resize(canvas, (W, H), interpolation=cv2.INTER_AREA) if ss > 1 else canvas
    if distort and np.any(intr.D != 0):
        mx, my = distortion_map(intr.K, intr.D, (W, H))
        img = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=int(background))
    if blur_sigma_px <= 0 and noise_sigma <= 0:
        return img
    f = img.astype(np.float32)
    if blur_sigma_px > 0:
        f = cv2.GaussianBlur(f, (0, 0), float(blur_sigma_px))
    if noise_sigma > 0:
        f += np.random.default_rng(seed).normal(0.0, float(noise_sigma), f.shape).astype(np.float32)
    return np.clip(np.round(f), 0, 255).astype(np.uint8)


def render_board(spec: BoardSpec, intr: Intrinsics, T_cam_board: np.ndarray, size: tuple[int, int] | None = None,
                 supersample: int = 3, blur_sigma_px: float = 0.0, noise_sigma: float = 0.0, distort: bool = True,
                 background: int = 255, seed: int = 0, tex_px_per_mm: float = 40.0) -> np.ndarray:
    """uint8 (H, W) image of one board seen from T_cam_board (see render_boards)."""
    return render_boards([(spec, T_cam_board)], intr, size, supersample, blur_sigma_px, noise_sigma, distort,
                         background, seed, tex_px_per_mm)


def view(spec: BoardSpec, dist_mm: float, tilt_deg: float = 0.0, azimuth_deg: float = 0.0, roll_deg: float = 0.0,
         offset_mm: Sequence[float] = (0.0, 0.0)) -> np.ndarray:
    """T_cam_board of a camera aimed at the board centre + offset_mm (board x, y) from dist_mm
    (targets.T_board_cam_looking_at)."""
    cx, cy = spec.centre_mm
    return g.inv(T_board_cam_looking_at((cx + offset_mm[0], cy + offset_mm[1]), dist_mm, tilt_deg, azimuth_deg,
                                        roll_deg))


def project(spec_or_obj, intr: Intrinsics, T_cam_board: np.ndarray) -> np.ndarray:
    """Ground-truth image positions (N, 2) [px] of the board corners (BoardSpec) or of given (N, 3) points."""
    from .targets import corners_obj
    obj = corners_obj(spec_or_obj) if isinstance(spec_or_obj, BoardSpec) else np.asarray(spec_or_obj, float)
    rvec = cv2.Rodrigues(T_cam_board[:3, :3])[0]
    return cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, T_cam_board[:3, 3].reshape(3, 1), intr.K,
                             intr.D)[0].reshape(-1, 2)
