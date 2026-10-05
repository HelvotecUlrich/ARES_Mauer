"""Intrinsic calibration of the eye-in-hand camera with the ChArUco board "calib" ([boards.calib]).

    py.exe tools/calib_intrinsics.py capture --dataset data/intr_2026-10-06            # IDS camera, live preview
    py.exe tools/calib_intrinsics.py capture --camera files --files data/some_folder   # replay (dry run)
    py.exe tools/calib_intrinsics.py solve data/intr_2026-10-06 [--out calib/camera_intrinsics.json] [--force]

capture: live preview (downscaled) with the detected corners and a coverage map of the views saved so far (cells
with corners turn green). SPACE saves the current image losslessly to the dataset (only with >= --min-corners
corners), q / ESC quits. Take 20-30 views at 290-355 mm, tilted 10-30 deg in different directions, board corners
reaching all image regions (research 2026-10-05). Focus and iris locked before; changing them later = recalibrate.

solve: offline, from a dataset folder (meta.json kind "intrinsics"): ChArUco corners -> cv2.calibrateCameraExtended
(CALIB_FIX_K3 unless --free-k3), prints RMS, std devs, per-view errors, rejected views and warnings, writes the
intrinsics JSON (refuses to overwrite an existing file without --force).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from mauer import config  # noqa: E402
from mauer.vision import intrinsics as intr_mod  # noqa: E402
from mauer.vision.dataset import Dataset  # noqa: E402
from mauer.vision.detect import detect_boards, draw_detections  # noqa: E402
from mauer.vision.targets import board_specs  # noqa: E402

COVER_GRID = (12, 10)        # coverage cells (x, y) over the image, same grid as intrinsics.calibrate


def _rel(p: Path) -> str:
    """Repo-relative path string when inside the repo (as stored in the calibration file), else absolute."""
    p = Path(p).resolve()
    try:
        return p.relative_to(REPO).as_posix()
    except ValueError:
        return str(p)


def cmd_solve(args: argparse.Namespace) -> int:
    cfg = config.load(args.config)
    vcfg = cfg.get("vision", {})
    ds = Dataset.load(args.dataset)
    if ds.kind != "intrinsics":
        print(f"note: dataset kind is {ds.kind!r}, using its images anyway", flush=True)
    board = args.board or ds.board or "calib"
    spec = board_specs(cfg)[board]
    out = config.repo_path(args.out or vcfg.get("intrinsics_file", "calib/camera_intrinsics.json"))
    if out.exists() and not args.force:
        print(f"refusing to overwrite {out} (use --force)", flush=True)
        return 2
    if len(ds) == 0:
        print(f"dataset {args.dataset} has no images", flush=True)
        return 1
    print(f"dataset {ds.path}: {len(ds)} images, board {spec.describe()}", flush=True)
    flags = 0 if args.free_k3 else cv2.CALIB_FIX_K3
    min_corners = args.min_corners or int(vcfg.get("calib_min_corners", 12))
    try:
        intr, report = intr_mod.calibrate((img for img, _ in ds), spec, names=[s["image"] for s in ds.samples],
                                          min_corners=min_corners, border_px=float(vcfg.get("border_px", 10)),
                                          flags=flags, dataset=_rel(ds.path))
    except ValueError as e:
        print(f"calibration failed: {e}", flush=True)
        return 1
    print(intr_mod.format_report(intr, report), flush=True)
    nom = intr_mod.nominal(cfg)
    print(f"vs nominal pinhole ([camera] focal / pixel_um): fx {100 * (intr.fx / nom.fx - 1):+.2f} %, "
          f"principal point offset ({intr.cx - nom.cx:+.1f}, {intr.cy - nom.cy:+.1f}) px", flush=True)
    if (intr.width, intr.height) != (nom.width, nom.height):
        print(f"WARNING: image size {intr.width}x{intr.height} differs from [camera] res "
              f"{nom.width}x{nom.height}", flush=True)
    intr_mod.save(intr, out)
    print(f"written {out}", flush=True)
    return 0


def _coverage_overlay(preview: np.ndarray, cover: np.ndarray) -> np.ndarray:
    """Tint the preview: cells with corners in a saved view green, empty cells red (alpha 0.18)."""
    h, w = preview.shape[:2]
    gy, gx = cover.shape
    tint = np.zeros_like(preview)
    for j in range(gy):
        for i in range(gx):
            colour = (0, 160, 0) if cover[j, i] > 0 else (0, 0, 160)
            tint[j * h // gy:(j + 1) * h // gy, i * w // gx:(i + 1) * w // gx] = colour
    return cv2.addWeighted(preview, 1.0, tint, 0.18, 0.0)


def cmd_capture(args: argparse.Namespace) -> int:
    try:
        from mauer.camera import CameraError, open_camera      # lazy: solve works without the camera package
    except ImportError as e:
        print(f"mauer.camera not available: {e}", flush=True)
        return 1
    cfg = config.load(args.config)
    vcfg = cfg.get("vision", {})
    spec = board_specs(cfg)[args.board]
    border = float(vcfg.get("border_px", 10))
    min_corners = args.min_corners or int(vcfg.get("calib_min_corners", 12))
    kw = {"folder": args.files, "loop": True} if args.camera == "files" else {}
    if args.camera == "files" and not args.files:
        print("--camera files needs --files <folder>", flush=True)
        return 2
    gx, gy = COVER_GRID
    cover = np.zeros((gy, gx), int)
    win = "calib_intrinsics (SPACE save, q quit)"
    try:
        with open_camera(cfg, kind=args.camera, **kw) as cam:
            # dataset created only once the camera is open (no empty folder after a camera error)
            ds = Dataset.create(args.dataset, "intrinsics", board=spec.name, notes=args.notes,
                                exist_ok=args.append)
            ds.update_meta(camera=cam.info())
            print(f"camera {cam.info()} -> dataset {ds.path} ({len(ds)} images so far)", flush=True)
            while True:
                fr = cam.grab()
                det = detect_boards(fr.image, [spec], border).get(spec.name)
                n = 0 if det is None else det.n
                view = draw_detections(fr.image, {spec.name: det} if det else {}, args.scale)
                view = _coverage_overlay(view, cover)
                colour = (0, 255, 0) if n >= min_corners else (0, 0, 255)
                cv2.putText(view, f"{n}/{spec.n_corners} corners | saved {len(ds)} | coverage "
                                  f"{100 * (cover > 0).mean():.0f} %", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            colour, 2, cv2.LINE_AA)
                cv2.imshow(win, view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    if n < min_corners:
                        print(f"not saved: {n} corners < {min_corners}", flush=True)
                        continue
                    simple = {k: v for k, v in (fr.meta or {}).items()
                              if isinstance(v, (int, float, str, bool)) or v is None}
                    s = ds.add(fr.image, t_start=fr.t_start, t_end=fr.t_end, n_corners=n, frame_meta=simple)
                    h, w = fr.image.shape[:2]
                    cx = np.clip((det.img_pts[:, 0] / w * gx).astype(int), 0, gx - 1)
                    cy = np.clip((det.img_pts[:, 1] / h * gy).astype(int), 0, gy - 1)
                    np.add.at(cover, (cy, cx), 1)
                    print(f"saved {s['image']} ({n} corners), {len(ds)} views", flush=True)
                elif key in (ord("q"), 27):
                    break
    except CameraError as e:
        print(f"camera error: {e}", flush=True)
        return 1
    finally:
        cv2.destroyAllWindows()
    print(f"{len(ds)} views in {ds.path}; next: py.exe tools/calib_intrinsics.py solve {ds.path}", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("solve", help="calibrate offline from a dataset folder")
    s.add_argument("dataset", help="dataset folder with meta.json")
    s.add_argument("--out", default=None, help="output JSON (default [vision] intrinsics_file)")
    s.add_argument("--force", action="store_true", help="overwrite an existing output file")
    s.add_argument("--board", default=None, help="board name (default: dataset meta, else 'calib')")
    s.add_argument("--min-corners", type=int, default=None, help="reject views with fewer corners (default 12)")
    s.add_argument("--free-k3", action="store_true", help="estimate k3 too (default CALIB_FIX_K3)")
    s.set_defaults(func=cmd_solve)
    c = sub.add_parser("capture", help="live capture of calibration views into a dataset")
    c.add_argument("--dataset", required=True, help="dataset folder to create (data/<name>)")
    c.add_argument("--append", action="store_true", help="add to an existing dataset")
    c.add_argument("--camera", choices=("ids", "files"), default="ids")
    c.add_argument("--files", default=None, help="image folder for --camera files")
    c.add_argument("--board", default="calib")
    c.add_argument("--min-corners", type=int, default=None, help="only save views with this many corners")
    c.add_argument("--scale", type=float, default=0.4, help="preview scale (0.4 -> 989 x 826 px)")
    c.add_argument("--notes", default="", help="free text stored in meta.json (focus/iris setting, light, ...)")
    c.set_defaults(func=cmd_capture)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
