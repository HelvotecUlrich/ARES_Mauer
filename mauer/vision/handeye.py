"""Eye-in-hand calibration T_flange_cam with cv2.calibrateHandEye, validation and look-pose planning.

Inputs per robot pose i (board fixed relative to the UR base, i.e. on the ARES deck so spring sway cancels out):
T_base_flange_i (RTDE / get_actual_tool_flange_pose, mm) and T_cam_board_i (detect.estimate_pose). OpenCV naming:
R/t_gripper2base = T_base_flange, R/t_target2cam = T_cam_board, output R/t_cam2gripper = T_flange_cam.

Research 2026-10-05 (vision-method.json, synthetic): all five methods are run; PARK is used (TSAI/HORAUD equivalent,
ANDREFF 2-3x worse); 25 poses with large rotations reach ~0.009 deg / 0.07 mm, small rotations are ~4x worse. The
`method` argument MUST be passed as a keyword - passed positionally it lands in an output slot and TSAI runs
silently. OpenCV forms motions from all pose pairs; TSAI drops pairs with a relative rotation below ~17 deg or above
~120 deg (calibration_handeye.cpp, OpenCV PR #24897). In-sample consistency (spread of T_base_board) does NOT reveal
a poor calibration, held-out poses do (holdout_check).

JSON (calib/handeye.json): {"T_flange_cam": 4x4, "method", "n_poses", "disagreement": {...},
"board_in_base_spread": {...}, "holdout": {...} | null, "motion": {...}, "warnings": [...], "opencv", "created",
"dataset", "intrinsics_file"}. Units: mm, deg in reports, rad internally.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from itertools import combinations
from pathlib import Path
from typing import Mapping, Sequence

import cv2
import numpy as np

from .. import geometry as g
from .intrinsics import Intrinsics, to_jsonable, now_iso
from .targets import BoardSpec, T_board_cam_looking_at, corners_obj

METHODS = ("TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS")
DEFAULT_METHOD = "PARK"
# Relative rotation between two poses that TSAI keeps (OpenCV calibration_handeye.cpp, research 2026-10-05).
REL_ROT_MIN_DEG = 17.0
REL_ROT_MAX_DEG = 120.0
# Warning thresholds - ASSUMPTIONS (no standard): share of useful pose pairs, rotation-axis diversity, agreement of
# the methods TSAI/PARK/HORAUD/DANIILIDIS (ANDREFF is known to be worse and only reported).
MIN_USEFUL_PAIR_FRACTION = 0.5
MIN_AXIS_EIG = 0.05
MAX_METHOD_DISAGREE_MM = 1.0
MAX_METHOD_DISAGREE_DEG = 0.1


@dataclass
class HandEyeResult:
    T_flange_cam: np.ndarray                     # chosen method
    method: str
    n_poses: int
    by_method: dict[str, np.ndarray]             # T_flange_cam of every method that succeeded
    disagreement: dict                           # pairwise / vs chosen (mm, deg)
    T_base_board: np.ndarray                     # mean of the in-sample board poses in the base frame
    board_in_base_spread: dict                   # geometry.spread (without "mean") + per-sample deviations
    motion: dict = field(default_factory=dict)   # relative rotations of the pose pairs
    warnings: list[str] = field(default_factory=list)
    meta: dict = field(default_factory=dict)     # opencv, created, dataset, intrinsics_file, holdout


def _check_lists(T_base_flange_list: Sequence[np.ndarray], T_cam_board_list: Sequence[np.ndarray]):
    A = [np.asarray(T, float).reshape(4, 4) for T in T_base_flange_list]
    B = [np.asarray(T, float).reshape(4, 4) for T in T_cam_board_list]
    if len(A) != len(B):
        raise ValueError(f"{len(A)} robot poses but {len(B)} board poses")
    for T in A + B:
        if not np.isfinite(T).all():
            raise ValueError("non-finite pose in the input")
    return A, B


def board_in_base(T_base_flange_list: Sequence[np.ndarray], T_flange_cam: np.ndarray,
                  T_cam_board_list: Sequence[np.ndarray]) -> list[np.ndarray]:
    """T_base_board = T_base_flange @ T_flange_cam @ T_cam_board for every sample."""
    A, B = _check_lists(T_base_flange_list, T_cam_board_list)
    return [Tf @ T_flange_cam @ Tc for Tf, Tc in zip(A, B)]


def motion_stats(T_base_flange_list: Sequence[np.ndarray]) -> dict:
    """Relative rotations of all pose pairs (what calibrateHandEye uses): angle stats [deg], share of pairs inside
    [REL_ROT_MIN_DEG, REL_ROT_MAX_DEG] and the eigenvalues of the rotation-axis scatter (all axes parallel ->
    second eigenvalue ~0 -> the hand-eye rotation is not observable about that axis)."""
    Rs = [np.asarray(T, float)[:3, :3] for T in T_base_flange_list]
    ang, axes = [], []
    for i, j in combinations(range(len(Rs)), 2):
        Rr = Rs[i].T @ Rs[j]
        a = g.angle_of(Rr)
        ang.append(np.degrees(a))
        v = g.R_to_rotvec(Rr)
        if np.linalg.norm(v) > 1e-9:
            axes.append(v / np.linalg.norm(v))
    if not ang:
        return {"n_pairs": 0}
    ang = np.array(ang)
    eig = [0.0, 0.0, 0.0]
    if axes:
        A = np.array(axes)
        eig = sorted(np.linalg.eigvalsh(A.T @ A / len(A)).tolist(), reverse=True)
    return {"n_pairs": int(len(ang)), "rel_rot_min_deg": float(ang.min()),
            "rel_rot_median_deg": float(np.median(ang)), "rel_rot_max_deg": float(ang.max()),
            "useful_pair_fraction": float(np.mean((ang >= REL_ROT_MIN_DEG) & (ang <= REL_ROT_MAX_DEG))),
            "axis_scatter_eig": [float(e) for e in eig]}


def solve(T_base_flange_list: Sequence[np.ndarray], T_cam_board_list: Sequence[np.ndarray],
          method: str = DEFAULT_METHOD) -> HandEyeResult:
    """Eye-in-hand calibration: all five OpenCV methods, the chosen one returned as T_flange_cam.

    Needs >= 3 poses with non-parallel rotation axes (research: 20-25 poses, tilt 15-30 deg in varied azimuths, roll
    up to +-90 deg about the optical axis). Warnings flag small relative rotations, near-parallel rotation axes and
    methods that disagree (bad data or a convention error)."""
    method = method.upper()
    if method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}")
    A, B = _check_lists(T_base_flange_list, T_cam_board_list)
    if len(A) < 3:
        raise ValueError(f"hand-eye calibration needs >= 3 poses, got {len(A)}")
    Rg = [T[:3, :3].copy() for T in A]
    tg = [T[:3, 3].reshape(3, 1).copy() for T in A]
    Rc = [T[:3, :3].copy() for T in B]
    tc = [T[:3, 3].reshape(3, 1).copy() for T in B]
    by, failed = {}, {}
    for m in METHODS:
        try:
            # method as KEYWORD: positionally it would be taken as the R_cam2gripper output and TSAI would run
            R, t = cv2.calibrateHandEye(Rg, tg, Rc, tc, method=getattr(cv2, "CALIB_HAND_EYE_" + m))
        except cv2.error as e:                  # pragma: no cover - degenerate input
            failed[m] = str(e).strip().splitlines()[-1]
            continue
        T = g.make_T(R, np.asarray(t).ravel())
        if not np.isfinite(T).all():
            failed[m] = "non-finite result"
            continue
        if np.array_equal(T, np.eye(4)):
            # TSAI without enough informative motions logs "Hand-eye calibration failed!" and returns R = I, t = 0
            # (seen with OpenCV 4.14 on synthetic small-rotation data) - an exact identity is never a real camera mount
            failed[m] = "no solution (identity returned: not enough informative motions)"
            continue
        by[m] = T
    if method not in by:
        raise RuntimeError(f"calibrateHandEye {method} failed: {failed.get(method)}")
    X = by[method]

    pair = {}
    for a, b in combinations(sorted(by), 2):
        pair[f"{a}-{b}"] = g.pose_delta(by[a], by[b])
    vs = {m: {"pos_mm": d[0], "ang_deg": d[1]} for m, d in ((m, g.pose_delta(X, T)) for m, T in by.items())}
    disagreement = {
        "vs_chosen": vs,
        "pairwise_max_pos_mm": max((d[0] for d in pair.values()), default=0.0),
        "pairwise_max_ang_deg": max((d[1] for d in pair.values()), default=0.0),
        "failed": failed,
        "T_flange_cam_by_method": {m: T.tolist() for m, T in by.items()},
    }

    Ts = board_in_base(A, X, B)
    sp = g.spread(Ts)
    mean = sp.pop("mean")
    dev = np.array([g.pose_delta(mean, T) for T in Ts])
    sp["per_sample_pos_mm"] = dev[:, 0].tolist()
    sp["per_sample_ang_deg"] = dev[:, 1].tolist()

    motion = motion_stats(A)
    warnings = []
    if len(A) < 15:
        warnings.append(f"only {len(A)} poses (research: 20-25)")
    if motion.get("useful_pair_fraction", 0.0) < MIN_USEFUL_PAIR_FRACTION:
        warnings.append(f"small relative rotations: only {100 * motion.get('useful_pair_fraction', 0.0):.0f} % of "
                        f"the pose pairs rotate {REL_ROT_MIN_DEG:g}-{REL_ROT_MAX_DEG:g} deg (median "
                        f"{motion.get('rel_rot_median_deg', 0.0):.1f} deg) - add tilt/roll variation")
    eig = motion.get("axis_scatter_eig", [0.0, 0.0, 0.0])
    if eig[1] < MIN_AXIS_EIG:
        warnings.append(f"rotation axes nearly parallel (axis scatter eigenvalues {np.round(eig, 3).tolist()}) - "
                        "rotate about different axes")
    for m, why in failed.items():
        warnings.append(f"{m} failed: {why}")
    for m in ("TSAI", "HORAUD", "DANIILIDIS", "PARK"):
        if m in vs and (vs[m]["pos_mm"] > MAX_METHOD_DISAGREE_MM or vs[m]["ang_deg"] > MAX_METHOD_DISAGREE_DEG):
            warnings.append(f"{m} differs from {method} by {vs[m]['pos_mm']:.3f} mm / {vs[m]['ang_deg']:.4f} deg - "
                            "check the data (pose pairing, units, frame conventions)")
    meta = {"opencv": cv2.__version__, "created": now_iso()}
    return HandEyeResult(X, method, len(A), by, disagreement, mean, sp, motion, warnings, meta)


def holdout_check(result: HandEyeResult, T_base_flange_holdout: Sequence[np.ndarray],
                  T_cam_board_holdout: Sequence[np.ndarray], spec: BoardSpec | None = None,
                  intr: Intrinsics | None = None) -> dict:
    """Validation with poses NOT used for the calibration: deviation of the predicted T_base_board from the in-sample
    mean (mm / deg). With spec and intr also the image-space check: board corners projected with the predicted pose
    (robot pose + T_flange_cam + in-sample board pose) vs with the measured pose, RMS / max [px]."""
    Ts = board_in_base(T_base_flange_holdout, result.T_flange_cam, T_cam_board_holdout)
    if not Ts:
        return {"n": 0}
    dev = np.array([g.pose_delta(result.T_base_board, T) for T in Ts])
    out = {"n": len(Ts), "pos_max_mm": float(dev[:, 0].max()), "pos_rms_mm": float(np.sqrt(np.mean(dev[:, 0] ** 2))),
           "ang_max_deg": float(dev[:, 1].max()), "ang_rms_deg": float(np.sqrt(np.mean(dev[:, 1] ** 2))),
           "per_sample_pos_mm": dev[:, 0].tolist(), "per_sample_ang_deg": dev[:, 1].tolist()}
    if spec is not None and intr is not None:
        obj = corners_obj(spec).reshape(-1, 1, 3)
        errs = []
        for Tf, Tc in zip(T_base_flange_holdout, T_cam_board_holdout):
            T_pred = g.inv(np.asarray(Tf, float) @ result.T_flange_cam) @ result.T_base_board
            p = [cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3].reshape(3, 1), intr.K,
                                   intr.D)[0].reshape(-1, 2) for T in (T_pred, np.asarray(Tc, float))]
            errs.append(np.linalg.norm(p[0] - p[1], axis=1))
        e = np.concatenate(errs)
        out["reproj_rms_px"] = float(np.sqrt(np.mean(e ** 2)))
        out["reproj_max_px"] = float(e.max())
    return out


def plan_poses(T_base_board: np.ndarray, T_flange_cam: np.ndarray, n: int,
               dist_mm: tuple[float, float] = (290.0, 355.0), tilt_deg: tuple[float, float] = (15.0, 30.0),
               roll_deg: tuple[float, float] = (-90.0, 90.0), seed: int = 0, spec: BoardSpec | None = None,
               centre_mm: Sequence[float] | None = None) -> list[np.ndarray]:
    """n flange poses T_base_flange whose camera (OpenCV frame) looks at the board centre - pure geometry, the caller
    checks reachability and collisions.

    The aim point is centre_mm (board frame), else the centre of `spec`, else the board origin. Azimuths of the
    tilt are stratified over 360 deg (random start, jitter +-1/4 sector), distance, tilt and roll about the optical
    axis are uniform in the given ranges (defaults = research recommendation inside the F5.6 depth of field)."""
    rng = np.random.default_rng(seed)
    if centre_mm is not None:
        c = tuple(float(v) for v in centre_mm)
    elif spec is not None:
        c = spec.centre_mm
    else:
        c = (0.0, 0.0)
    T_base_board = np.asarray(T_base_board, float)
    T_cam_flange = g.inv(np.asarray(T_flange_cam, float))
    az0 = rng.uniform(0.0, 360.0)
    out = []
    for i in range(int(n)):
        az = az0 + 360.0 * (i + rng.uniform(-0.25, 0.25)) / n
        T_board_cam = T_board_cam_looking_at(c, rng.uniform(*dist_mm), rng.uniform(*tilt_deg), az,
                                             rng.uniform(*roll_deg))
        out.append(T_base_board @ T_board_cam @ T_cam_flange)
    return out


# ── file ──────────────────────────────────────────────────────────────────────
def to_dict(result: HandEyeResult) -> dict:
    sp = dict(result.board_in_base_spread)
    sp["mean"] = result.T_base_board.tolist()
    m = result.meta
    return to_jsonable({
        "T_flange_cam": result.T_flange_cam.tolist(), "method": result.method, "n_poses": result.n_poses,
        "disagreement": result.disagreement, "board_in_base_spread": sp, "holdout": m.get("holdout"),
        "motion": result.motion, "warnings": result.warnings, "opencv": m.get("opencv", cv2.__version__),
        "created": m.get("created", now_iso()), "dataset": m.get("dataset", ""),
        "intrinsics_file": m.get("intrinsics_file", ""),
    })


def from_dict(d: Mapping) -> HandEyeResult:
    sp = dict(d.get("board_in_base_spread") or {})
    mean = np.asarray(sp.pop("mean", np.eye(4)), float)
    dis = dict(d.get("disagreement") or {})
    by = {k: np.asarray(v, float) for k, v in (dis.get("T_flange_cam_by_method") or {}).items()}
    X = np.asarray(d["T_flange_cam"], float)
    by.setdefault(d.get("method", DEFAULT_METHOD), X)
    meta = {k: d.get(k) for k in ("opencv", "created", "dataset", "intrinsics_file", "holdout")}
    return HandEyeResult(X, str(d.get("method", DEFAULT_METHOD)), int(d.get("n_poses", 0)), by, dis, mean, sp,
                         dict(d.get("motion") or {}), list(d.get("warnings") or []), meta)


def save(result: HandEyeResult, path: str | Path, holdout: dict | None = None, dataset: str = "",
         intrinsics_file: str = "") -> Path:
    """Write calib/handeye.json (parent folders created; overwrite protection is up to the caller)."""
    if holdout is not None:
        result.meta["holdout"] = holdout
    if dataset:
        result.meta["dataset"] = str(dataset)
    if intrinsics_file:
        result.meta["intrinsics_file"] = str(intrinsics_file)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_dict(result), indent=2) + "\n", encoding="utf-8")
    return path


def load(path: str | Path) -> HandEyeResult:
    return from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def format_result(result: HandEyeResult, holdout: dict | None = None) -> str:
    """Human-readable summary: all methods, disagreement, in-sample spread, motion, hold-out, warnings."""
    xyz, rpy = g.xyz_rpy(result.T_flange_cam)
    lines = [f"T_flange_cam ({result.method}, {result.n_poses} poses): xyz [{xyz[0]:.3f}, {xyz[1]:.3f}, "
             f"{xyz[2]:.3f}] mm, rpy [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}] deg"]
    for m, d in result.disagreement.get("vs_chosen", {}).items():
        x, r = g.xyz_rpy(result.by_method[m]) if m in result.by_method else (np.full(3, np.nan), np.full(3, np.nan))
        lines.append(f"  {m:<10s} xyz [{x[0]:9.3f}, {x[1]:9.3f}, {x[2]:9.3f}] rpy [{r[0]:9.4f}, {r[1]:9.4f}, "
                     f"{r[2]:9.4f}]  vs {result.method}: {d['pos_mm']:.3f} mm / {d['ang_deg']:.4f} deg")
    s = result.board_in_base_spread
    lines.append(f"board in base, in-sample (necessary, not sufficient): pos rms {s['pos_rms_mm']:.3f} / max "
                 f"{s['pos_max_mm']:.3f} mm, ang rms {s['ang_rms_deg']:.4f} / max {s['ang_max_deg']:.4f} deg")
    mo = result.motion
    if mo.get("n_pairs"):
        lines.append(f"pose pairs {mo['n_pairs']}: relative rotation {mo['rel_rot_min_deg']:.1f} / "
                     f"{mo['rel_rot_median_deg']:.1f} / {mo['rel_rot_max_deg']:.1f} deg (min/median/max), "
                     f"{100 * mo['useful_pair_fraction']:.0f} % in {REL_ROT_MIN_DEG:g}-{REL_ROT_MAX_DEG:g} deg")
    h = holdout if holdout is not None else result.meta.get("holdout")
    if h and h.get("n"):
        txt = (f"hold-out ({h['n']} poses): pos rms {h['pos_rms_mm']:.3f} / max {h['pos_max_mm']:.3f} mm, ang rms "
               f"{h['ang_rms_deg']:.4f} / max {h['ang_max_deg']:.4f} deg")
        if "reproj_rms_px" in h:
            txt += f", reprojection rms {h['reproj_rms_px']:.2f} / max {h['reproj_max_px']:.2f} px"
        lines.append(txt)
    for w in result.warnings:
        lines.append(f"WARNING: {w}")
    return "\n".join(lines)
