"""ChArUco detection and board pose (OpenCV 4.14, free software only).

Pipeline per image (research 2026-10-05, docs/ARCHITECTURE.md §mauer.vision):
1. Markers are detected ONCE per dictionary (cv2.aruco.ArucoDetector); markers with a corner closer than `border_px`
   to the image edge are dropped (a tag partly outside the image is still decoded with one wrong corner, which gave
   3-15 deg pose errors in the research).
2. Per board, only the markers of its id range are handed to its CharucoDetector.detectBoard (markers given -> no
   second detection), which interpolates and cornerSubPix-refines the chessboard corners; corners within
   `border_px` of the edge are dropped as well. board.matchImagePoints gives the object points (board frame, mm).
3. estimate_pose: solvePnP(SOLVEPNP_IPPE) + solvePnPRefineLM with the full K, D (the same distortion model as
   cv2.undistortPoints / projectPoints), then checks: enough corners, not collinear, finite, in front of the
   camera, RMS reprojection error <= max_reproj_px.

Pitfalls handled: OpenCV <= 4.13 shifts ChArUco corners by 0.5 px (pin 4.14.0.94); the research saw IPPE return
non-finite values for exactly fronto-parallel noise-free synthetic views - such a result falls back to
SOLVEPNP_SQPNP before the pose is rejected.
Units: pixels (OpenCV convention: pixel centres at integer coordinates), mm, rad. T_cam_board: 4x4, board -> camera.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import Iterable, Mapping

import cv2
import numpy as np

from .. import geometry as g
from .intrinsics import Intrinsics
from .targets import BoardSpec, dictionary, make_board


@dataclass
class BoardDetection:
    """Detected ChArUco corners of one board in one image."""
    board: str
    corner_ids: np.ndarray            # (N,) int32 ChArUco corner ids
    img_pts: np.ndarray               # (N, 2) float64 [px]
    obj_pts: np.ndarray               # (N, 3) float64 board frame [mm]
    marker_ids: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))   # markers used (M,)

    @property
    def n(self) -> int:
        return int(len(self.corner_ids))


@dataclass
class BoardPose:
    """Pose of a board in the camera frame. T_cam_board is filled whenever a pose was computed (also for ok=False
    after the RMS check, for diagnostics), None otherwise."""
    board: str
    T_cam_board: np.ndarray | None
    n_corners: int
    rms_px: float
    ok: bool
    reason: str = ""


# ── detectors (cached per process; use them from one thread) ─────────────────
@lru_cache(maxsize=None)
def _marker_detector(dict_name: str, border_px: int) -> cv2.aruco.ArucoDetector:
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # marker corners only seed the ChArUco corners
    p.minDistanceToBorder = max(int(border_px), 0)              # same border rule as below, applied while detecting
    return cv2.aruco.ArucoDetector(dictionary(dict_name), p)


@lru_cache(maxsize=None)
def _charuco(spec: BoardSpec) -> tuple[cv2.aruco.CharucoBoard, cv2.aruco.CharucoDetector]:
    board = make_board(spec)
    cp = cv2.aruco.CharucoParameters()
    cp.minMarkers = 2                  # a corner needs both adjacent markers (OpenCV default, stated explicitly)
    # cornerSubPix of the ChArUco corners with OpenCV's default termination (30 iterations / 0.1 px): 100 / 0.01 px
    # made no measurable difference on synthetic images (mean corner error 0.0255 px both, probe 2026-10-05)
    return board, cv2.aruco.CharucoDetector(board, cp, cv2.aruco.DetectorParameters())


def _as_gray(img: np.ndarray) -> np.ndarray:
    img = np.asarray(img)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.dtype != np.uint8:
        raise ValueError(f"expected a uint8 image, got {img.dtype}")
    return img


def _inside(pts: np.ndarray, w: int, h: int, border_px: float) -> np.ndarray:
    """Boolean mask of points at least border_px from the image edge (pixel centres 0 .. w-1)."""
    p = pts.reshape(-1, 2)
    return ((p[:, 0] >= border_px) & (p[:, 0] <= w - 1 - border_px)
            & (p[:, 1] >= border_px) & (p[:, 1] <= h - 1 - border_px))


def _spec_list(specs: Mapping[str, BoardSpec] | Iterable[BoardSpec]) -> list[BoardSpec]:
    return list(specs.values()) if isinstance(specs, Mapping) else list(specs)


def detect_markers(img: np.ndarray, dict_name: str, border_px: float = 10) -> tuple[list[np.ndarray], np.ndarray]:
    """All markers of one dictionary: (corners [(1, 4, 2) float32 per marker], ids (M,) int32), border-filtered."""
    gray = _as_gray(img)
    h, w = gray.shape
    corners, ids, _ = _marker_detector(dict_name, int(np.ceil(border_px))).detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return [], np.zeros(0, np.int32)
    keep = [i for i, c in enumerate(corners) if _inside(np.asarray(c), w, h, border_px).all()]
    return [corners[i] for i in keep], ids.ravel()[keep].astype(np.int32)


def detect_boards(img: np.ndarray, specs: Mapping[str, BoardSpec] | Iterable[BoardSpec],
                  border_px: float = 10) -> dict[str, BoardDetection]:
    """Detect every given board in a mono uint8 image (e.g. 2472 x 2064 Mono8).

    Returns {board name: BoardDetection} for every board with at least one of its markers in view (n may be 0 when
    all its corners were dropped at the border). Markers are found once per dictionary; each board only sees the
    markers of its own id range, so several boards with the same dictionary can share an image."""
    gray = _as_gray(img)
    h, w = gray.shape
    specs_l = _spec_list(specs)
    markers = {d: detect_markers(gray, d, border_px) for d in {s.dictionary for s in specs_l}}
    out: dict[str, BoardDetection] = {}
    for spec in specs_l:
        corners, ids = markers[spec.dictionary]
        sel = [i for i, mid in enumerate(ids) if spec.first_id <= mid <= spec.last_id]
        if not sel:
            continue
        m_corners = [corners[i] for i in sel]
        m_ids = ids[sel].reshape(-1, 1)
        board, det = _charuco(spec)
        ch_corners, ch_ids, _, _ = det.detectBoard(gray, markerCorners=m_corners, markerIds=m_ids)
        empty = BoardDetection(spec.name, np.zeros(0, np.int32), np.zeros((0, 2)), np.zeros((0, 3)),
                               m_ids.ravel().copy())
        if ch_ids is None or len(ch_ids) == 0:
            out[spec.name] = empty
            continue
        ch_corners = np.asarray(ch_corners, float).reshape(-1, 2)
        ch_ids = np.asarray(ch_ids).ravel().astype(np.int32)
        keep = _inside(ch_corners, w, h, border_px)
        if not keep.any():
            out[spec.name] = empty
            continue
        ch_corners, ch_ids = ch_corners[keep], ch_ids[keep]
        # matchImagePoints returns the object points in the order of the ids; the image points stay float64
        obj, _ = board.matchImagePoints(ch_corners.reshape(-1, 1, 2).astype(np.float32), ch_ids.reshape(-1, 1))
        obj = np.asarray(obj, float).reshape(-1, 3)
        if len(obj) != len(ch_ids):
            raise RuntimeError(f"{spec.name}: matchImagePoints returned {len(obj)} points for {len(ch_ids)} ids")
        out[spec.name] = BoardDetection(spec.name, ch_ids, ch_corners, obj, m_ids.ravel().copy())
    return out


# ── pose ──────────────────────────────────────────────────────────────────────
def _collinear(obj: np.ndarray) -> bool:
    """True when the object points (N, 3) lie (numerically) on one line - the pose about that line is undefined."""
    p = obj - obj.mean(axis=0)
    s = np.linalg.svd(p, compute_uv=False)
    return s[0] < 1e-9 or s[1] < 1e-3 * s[0]


def reprojection_errors(obj_pts: np.ndarray, img_pts: np.ndarray, T_cam_board: np.ndarray,
                        intr: Intrinsics) -> np.ndarray:
    """Per-corner reprojection error [px] of a pose (N,)."""
    rvec = cv2.Rodrigues(T_cam_board[:3, :3])[0]
    proj = cv2.projectPoints(np.asarray(obj_pts, float).reshape(-1, 1, 3), rvec, T_cam_board[:3, 3].reshape(3, 1),
                             intr.K, intr.D)[0].reshape(-1, 2)
    return np.linalg.norm(proj - np.asarray(img_pts, float).reshape(-1, 2), axis=1)


def estimate_pose(det: BoardDetection, intr: Intrinsics, min_corners: int = 8,
                  max_reproj_px: float = 1.0) -> BoardPose:
    """T_cam_board from the detected corners: SOLVEPNP_IPPE (planar) + solvePnPRefineLM, with plausibility checks.
    ok=False with a reason for: too few corners, collinear corners, non-finite result, board behind the camera,
    RMS reprojection error > max_reproj_px."""
    n = det.n
    need = max(int(min_corners), 4)                     # IPPE needs >= 4 points
    if n < need:
        return BoardPose(det.board, None, n, float("nan"), False, f"too few corners ({n} < {need})")
    obj = np.asarray(det.obj_pts, float).reshape(-1, 1, 3)
    imgp = np.asarray(det.img_pts, float).reshape(-1, 1, 2)
    if _collinear(obj.reshape(-1, 3)):
        return BoardPose(det.board, None, n, float("nan"), False, "degenerate: corners collinear")
    rvec = tvec = None
    for flag in (cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_SQPNP):
        try:
            ok, rv, tv = cv2.solvePnP(obj, imgp, intr.K, intr.D, flags=flag)
        except cv2.error:
            continue
        if ok and np.isfinite(rv).all() and np.isfinite(tv).all():
            rvec, tvec = rv, tv
            break
    if rvec is None:
        return BoardPose(det.board, None, n, float("nan"), False, "non-finite pose (solvePnP failed)")
    rvec, tvec = cv2.solvePnPRefineLM(obj, imgp, intr.K, intr.D, rvec, tvec,
                                      criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12))
    if not (np.isfinite(rvec).all() and np.isfinite(tvec).all()):
        return BoardPose(det.board, None, n, float("nan"), False, "non-finite pose after refinement")
    T = g.make_T(cv2.Rodrigues(rvec)[0], tvec.ravel())
    z = g.apply(T, obj.reshape(-1, 3))[:, 2]
    if (z <= 0).any():
        return BoardPose(det.board, T, n, float("nan"), False, "implausible: board behind the camera")
    err = reprojection_errors(obj, imgp, T, intr)
    rms = float(np.sqrt(np.mean(err ** 2)))
    if not rms <= max_reproj_px:
        return BoardPose(det.board, T, n, rms, False, f"reprojection RMS {rms:.3f} px > {max_reproj_px} px")
    return BoardPose(det.board, T, n, rms, True, "")


def measure(img: np.ndarray, specs: Mapping[str, BoardSpec] | Iterable[BoardSpec], intr: Intrinsics,
            vcfg: Mapping) -> dict[str, BoardPose]:
    """Detect + estimate the pose of every given board; vcfg = cfg["vision"] (min_corners, max_reproj_px,
    border_px). Every requested board gets an entry (ok=False, reason "not detected" when no marker was seen)."""
    border = float(vcfg.get("border_px", 10))
    dets = detect_boards(img, specs, border)
    out = {}
    for spec in _spec_list(specs):
        d = dets.get(spec.name)
        if d is None:
            out[spec.name] = BoardPose(spec.name, None, 0, float("nan"), False, "not detected")
        else:
            out[spec.name] = estimate_pose(d, intr, int(vcfg.get("min_corners", 8)),
                                           float(vcfg.get("max_reproj_px", 1.0)))
    return out


def draw_detections(img: np.ndarray, dets: Mapping[str, BoardDetection], scale: float = 1.0) -> np.ndarray:
    """BGR preview (optionally downscaled) with the detected corners and board names, for live views."""
    gray = _as_gray(img)
    if scale != 1.0:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for name, d in dets.items():
        if d.n == 0:
            continue
        # scaled pixel centre: (x + 0.5) * scale - 0.5
        p = (d.img_pts + 0.5) * scale - 0.5
        for x, y in p:
            cv2.circle(out, (int(round(x)), int(round(y))), 3, (0, 255, 0), -1, cv2.LINE_AA)
        x0, y0 = p.min(axis=0)
        cv2.putText(out, f"{name} ({d.n})", (int(x0), max(int(y0) - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 200, 255), 2, cv2.LINE_AA)
    return out
