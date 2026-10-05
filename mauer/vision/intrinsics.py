"""Camera intrinsics: data class, JSON file (calib/camera_intrinsics.json), nominal pinhole model, ChArUco calibration.

Model: OpenCV pinhole + Brown-Conrady distortion D = [k1, k2, p1, p2, k3] (the model of cv2.projectPoints /
undistortPoints), pixel centres at integer coordinates. Calibration (research 2026-10-05, vision-method.json):
CharucoDetector.detectBoard -> board.matchImagePoints -> cv2.calibrateCameraExtended with CALIB_FIX_K3 (k3 overfits
on a 12 mm lens: -0.57 with 5 free parameters on synthetic data). Check the std deviations and per-view errors, not
only the RMS: with OpenCV <= 4.13 a 0.5 px ChArUco corner shift hid in cx/cy at an unchanged RMS.

JSON format:
{"K": 3x3, "D": [k1, k2, p1, p2, k3], "width", "height", "rms_px", "n_views",
 "std": {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"}, "per_view_rms_px": [...], "views": [...],
 "flags": [...], "opencv": cv2.__version__, "created": iso8601, "dataset": path, "board": BoardSpec as dict}
Never change focus, iris, binning or ROI after calibrating (new intrinsics AND a new hand-eye calibration).
"""
from __future__ import annotations

import datetime as _dt
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import cv2
import numpy as np

from .targets import BoardSpec

STD_NAMES = ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3")   # order of stdDeviationsIntrinsics
_FLAG_NAMES = ("CALIB_FIX_K3", "CALIB_FIX_ASPECT_RATIO", "CALIB_FIX_PRINCIPAL_POINT", "CALIB_ZERO_TANGENT_DIST",
               "CALIB_RATIONAL_MODEL", "CALIB_FIX_K1", "CALIB_FIX_K2", "CALIB_USE_INTRINSIC_GUESS")
_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-12)


def now_iso() -> str:
    """Local time with UTC offset, seconds resolution (ISO 8601)."""
    return _dt.datetime.now().astimezone().isoformat(timespec="seconds")


@dataclass
class Intrinsics:
    """K [px], D = [k1, k2, p1, p2, k3], image size [px], calibration RMS [px] and provenance in meta."""
    K: np.ndarray
    D: np.ndarray
    width: int
    height: int
    rms_px: float = float("nan")
    n_views: int = 0
    meta: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.K = np.asarray(self.K, float).reshape(3, 3)
        d = np.zeros(5)
        dd = np.asarray(self.D, float).ravel()
        if len(dd) > 5:
            raise ValueError("only the 5-parameter model [k1, k2, p1, p2, k3] is supported")
        d[:len(dd)] = dd
        self.D = d
        self.width, self.height = int(self.width), int(self.height)

    @property
    def fx(self) -> float:
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        return float(self.K[1, 2])

    @property
    def size(self) -> tuple[int, int]:
        """(width, height) [px] as OpenCV wants it."""
        return self.width, self.height

    def to_dict(self) -> dict:
        d = {"K": self.K.tolist(), "D": self.D.tolist(), "width": self.width, "height": self.height,
             "rms_px": None if not np.isfinite(self.rms_px) else float(self.rms_px), "n_views": int(self.n_views)}
        d.update(to_jsonable(self.meta))
        return d

    @classmethod
    def from_dict(cls, d: Mapping) -> "Intrinsics":
        core = {"K", "D", "width", "height", "rms_px", "n_views"}
        rms = d.get("rms_px")
        return cls(np.asarray(d["K"], float), np.asarray(d["D"], float), int(d["width"]), int(d["height"]),
                   float("nan") if rms is None else float(rms), int(d.get("n_views", 0)),
                   {k: v for k, v in d.items() if k not in core})


def to_jsonable(x):
    """Nested dicts/lists with numpy arrays and scalars -> plain JSON values (non-finite floats -> None)."""
    if isinstance(x, dict):
        return {str(k): to_jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return to_jsonable(x.tolist())
    if isinstance(x, np.generic):
        return to_jsonable(x.item())
    if isinstance(x, float) and not np.isfinite(x):
        return None
    return x


def save(intr: Intrinsics, path: str | Path) -> Path:
    """Write the JSON file (parent folders are created). Overwrite protection is up to the caller."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    d = intr.to_dict()
    d.setdefault("opencv", cv2.__version__)
    d.setdefault("created", now_iso())
    path.write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
    return path


def load(path: str | Path) -> Intrinsics:
    return Intrinsics.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def nominal(cfg: Mapping) -> Intrinsics:
    """Ideal pinhole from [camera] focal / pixel_um, zero distortion, principal point in the image centre
    cx = (W - 1) / 2 (pixel-centre convention) - the model of the RoboDK simulated camera, NOT of the real lens."""
    c = cfg["camera"]
    f_px = float(c["focal"]) / (float(c["pixel_um"]) * 1e-3)        # 12 mm / 2.74 um = 4379.56 px
    W, H = int(c["res_x"]), int(c["res_y"])
    K = np.array([[f_px, 0.0, (W - 1) / 2.0], [0.0, f_px, (H - 1) / 2.0], [0.0, 0.0, 1.0]])
    return Intrinsics(K, np.zeros(5), W, H, float("nan"), 0,
                      {"source": "nominal: [camera] focal / pixel_um, zero distortion (not calibrated)",
                       "created": now_iso()})


def flag_names(flags: int) -> list[str]:
    return [n for n in _FLAG_NAMES if flags & getattr(cv2, n)]


# ── calibration ───────────────────────────────────────────────────────────────
def calibrate(images: Iterable[np.ndarray], spec: BoardSpec, *, names: Sequence[str] | None = None,
              min_corners: int = 12, border_px: float = 10, flags: int = cv2.CALIB_FIX_K3,
              dataset: str = "", coverage_grid: tuple[int, int] = (12, 10)) -> tuple[Intrinsics, dict]:
    """Intrinsic calibration from ChArUco views of `spec` (uint8 mono images, all the same size).

    Views with fewer than `min_corners` corners are rejected (listed in the report). Returns (Intrinsics, report);
    report = {"n_images", "n_used", "rejected": [{"view", "reason"}], "per_view": [{"view", "n_corners",
    "rms_px"}], "rms_px", "std", "coverage" (fraction of a coverage_grid of image cells with corners),
    "warnings": [...]}. Raises ValueError with fewer than 3 usable views."""
    from .detect import detect_boards                       # local import: detect imports this module

    obj_all, img_all, used, rejected = [], [], [], []
    size = None
    gx, gy = coverage_grid
    cover = np.zeros((gy, gx), bool)
    n_images = 0
    for i, img in enumerate(images):
        n_images += 1
        name = names[i] if names is not None else str(i)
        img = np.asarray(img)
        h, w = img.shape[:2]
        if size is None:
            size = (w, h)
        elif size != (w, h):
            rejected.append({"view": name, "reason": f"image size {w}x{h} != {size[0]}x{size[1]}"})
            continue
        det = detect_boards(img, [spec], border_px).get(spec.name)
        n = 0 if det is None else det.n
        if n < min_corners:
            rejected.append({"view": name, "reason": f"{n} corners < {min_corners}"})
            continue
        obj_all.append(det.obj_pts.astype(np.float32).reshape(-1, 1, 3))
        img_all.append(det.img_pts.astype(np.float32).reshape(-1, 1, 2))
        used.append(name)
        cx = np.clip((det.img_pts[:, 0] / w * gx).astype(int), 0, gx - 1)
        cy = np.clip((det.img_pts[:, 1] / h * gy).astype(int), 0, gy - 1)
        cover[cy, cx] = True
    if len(obj_all) < 3:
        raise ValueError(f"only {len(obj_all)} usable views of {spec.name} (need >= 3, better 20-30); "
                         f"rejected: {rejected}")
    rms, K, D, _, _, std_in, _, per_view = cv2.calibrateCameraExtended(
        obj_all, img_all, size, None, None, flags=flags, criteria=_CRITERIA)
    std_in = np.asarray(std_in, float).ravel()
    per_view = np.asarray(per_view, float).ravel()
    std = {k: float(std_in[i]) for i, k in enumerate(STD_NAMES) if i < len(std_in)}
    report = {
        "n_images": n_images, "n_used": len(used), "rejected": rejected,
        "per_view": [{"view": v, "n_corners": int(len(o)), "rms_px": float(e)}
                     for v, o, e in zip(used, obj_all, per_view)],
        "rms_px": float(rms), "std": std, "coverage": float(cover.mean()), "warnings": [],
    }
    med = float(np.median(per_view))
    for pv in report["per_view"]:
        if pv["rms_px"] > max(3.0 * med, 0.3):          # ASSUMPTION: outlier rule, check the view by eye
            report["warnings"].append(f"view {pv['view']}: RMS {pv['rms_px']:.3f} px (median {med:.3f})")
    if report["coverage"] < 0.6:                          # ASSUMPTION: corners should reach most of the image
        report["warnings"].append(f"corners cover only {100 * report['coverage']:.0f} % of the image cells - "
                                  "add views with the board near the image corners/edges")
    if len(used) < 15:
        report["warnings"].append(f"only {len(used)} views (research: 20-30 at 290-355 mm, tilt 10-30 deg)")
    meta = {"std": std, "per_view_rms_px": per_view.tolist(), "views": used, "flags": flag_names(flags),
            "opencv": cv2.__version__, "created": now_iso(), "dataset": str(dataset), "board": spec.to_dict()}
    return Intrinsics(K, np.asarray(D).ravel()[:5], size[0], size[1], float(rms), len(used), meta), report


def format_report(intr: Intrinsics, report: Mapping) -> str:
    """Human-readable calibration report."""
    s = {k: report.get("std", {}).get(k, float("nan")) for k in STD_NAMES}
    lines = [
        f"views used {report['n_used']} of {report['n_images']}, RMS reprojection {report['rms_px']:.4f} px, "
        f"coverage {100 * report['coverage']:.0f} % of the image cells",
        f"fx {intr.fx:.2f} (std {s['fx']:.2f})  fy {intr.fy:.2f} (std {s['fy']:.2f}) px",
        f"cx {intr.cx:.2f} (std {s['cx']:.2f})  cy {intr.cy:.2f} (std {s['cy']:.2f}) px",
        "D [k1 k2 p1 p2 k3] = [" + ", ".join(f"{v:.5f}" for v in intr.D) + "]  std ["
        + ", ".join(f"{s[k]:.5f}" for k in ("k1", "k2", "p1", "p2", "k3")) + "]",
    ]
    for pv in report["per_view"]:
        lines.append(f"  view {pv['view']}: {pv['n_corners']:3d} corners, RMS {pv['rms_px']:.4f} px")
    for r in report["rejected"]:
        lines.append(f"  rejected {r['view']}: {r['reason']}")
    for w in report.get("warnings", []):
        lines.append(f"WARNING: {w}")
    return "\n".join(lines)
