"""Write the printable ChArUco boards: exact-scale vector PDFs and PNG textures for RoboDK, with a geometry check.

    py.exe tools/print_targets.py                       # all boards ("calib" + every [[targets]]) into targets/
    py.exe tools/print_targets.py --out targets --boards calib S0 --png-px-per-mm 20

Per board <name>: <name>.pdf (print at 100 %, see the text on the sheet) and <name>.png (one square of white
quiet zone; the PNG covers ((nx + 2) * square) x ((ny + 2) * square) mm, board origin at (square, square) mm from its
top-left corner - printed for RoboDK). Check without extra dependencies (no PDF rasteriser):
  1. the rectangles drawn into the PDF, rasterised (printables.rects_raster), are detected with all corners at the
     nominal positions (max deviation in mm);
  2. the PNG is detected with all corners at its nominal scale;
  3. a synthetic camera view (nominal pinhole from [camera], [camera] working_dist or farther so the board fits,
     20 deg tilt) is detected and the pose error against the ground truth is reported.
Exit code 1 if a check fails.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from mauer import config  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer.vision import intrinsics, printables, synth  # noqa: E402
from mauer.vision.dataset import read_image  # noqa: E402
from mauer.vision.detect import detect_boards, estimate_pose  # noqa: E402
from mauer.vision.targets import BoardSpec, board_specs  # noqa: E402

TOL_RASTER_MM = 0.05     # rasterised PDF rectangles at 20 px/mm: pixel quantisation 0.05 mm (detector sub-pixel)
TOL_PNG_PX = 0.1         # PNG corners vs nominal (generateImage at an integer px per square)
TOL_POSE = (0.1, 0.1)    # synthetic view: mm / deg, nominal pinhole without noise


def _corner_dev(img: np.ndarray, spec: BoardSpec, px_per_mm: float, all_specs) -> tuple[int, float]:
    """(corners found, max deviation [px]) of a board image with a one-square margin."""
    det = detect_boards(img, all_specs, border_px=2).get(spec.name)
    if det is None or det.n == 0:
        return 0, float("inf")
    expect = (det.obj_pts[:, :2] + spec.square_mm) * px_per_mm - 0.5
    return det.n, float(np.abs(det.img_pts - expect).max())


def check(spec: BoardSpec, png: Path, png_px_per_square: int, cfg: dict, all_specs) -> list[str]:
    """Run the three checks, print the numbers, return the failures."""
    fails = []
    n, dev = _corner_dev(printables.rects_raster(spec, 20.0), spec, 20.0, all_specs)
    print(f"    PDF geometry (raster 20 px/mm): {n}/{spec.n_corners} corners, max deviation {dev / 20.0:.4f} mm",
          flush=True)
    if n != spec.n_corners or dev / 20.0 > TOL_RASTER_MM:
        fails.append(f"{spec.name}: PDF geometry check failed")
    n, dev = _corner_dev(read_image(png), spec, png_px_per_square / spec.square_mm, all_specs)
    print(f"    PNG: {n}/{spec.n_corners} corners, max deviation {dev:.3f} px", flush=True)
    if n != spec.n_corners or dev > TOL_PNG_PX:
        fails.append(f"{spec.name}: PNG check failed")
    intr = intrinsics.nominal(cfg)
    bw, bh = spec.size_mm
    # distance at which the board incl. quiet zone fills <= 80 % of the shorter field of view
    fit = max(bw + 2 * spec.square_mm, bh + 2 * spec.square_mm) / (0.8 * min(intr.size)) * intr.fx
    dist = max(float(cfg["camera"].get("working_dist", 320.0)), fit)
    T = synth.view(spec, dist, tilt_deg=20.0, azimuth_deg=30.0, roll_deg=10.0)
    det = detect_boards(synth.render_board(spec, intr, T), all_specs).get(spec.name)
    if det is None:
        fails.append(f"{spec.name}: not detected in the synthetic view")
        return fails
    pose = estimate_pose(det, intr, 4, 1.0)
    dp, da = g.pose_delta(T, pose.T_cam_board) if pose.T_cam_board is not None else (np.inf, np.inf)
    print(f"    synthetic view at {dist:.0f} mm: {det.n}/{spec.n_corners} corners, pose error {dp:.4f} mm / "
          f"{da:.4f} deg ({pose.reason or 'ok'})", flush=True)
    if not pose.ok or det.n != spec.n_corners or dp > TOL_POSE[0] or da > TOL_POSE[1]:
        fails.append(f"{spec.name}: synthetic view check failed")
    return fails


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="targets", help="output folder (default targets/, relative to the repo)")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--boards", nargs="*", default=None, help="board names (default: calib + all [[targets]])")
    ap.add_argument("--png-px-per-mm", type=float, default=20.0,
                    help="PNG resolution, rounded to whole pixels per square (default 20)")
    ap.add_argument("--no-check", action="store_true", help="skip the geometry checks")
    ap.add_argument("--print-scale", type=float, default=None,
                    help="pre-scale the PDFs (default [plates] print_scale, else 1.0; 100/96 if the 100 mm bar "
                         "prints as 96 mm)")
    args = ap.parse_args(argv)

    cfg = config.load(args.config)
    specs = board_specs(cfg)
    names = args.boards or list(specs)
    unknown = [n for n in names if n not in specs]
    if unknown:
        print(f"unknown boards {unknown}; known: {list(specs)}", flush=True)
        return 2
    out = config.repo_path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fails: list[str] = []
    scale = args.print_scale if args.print_scale is not None else float(cfg.get("plates", {}).get("print_scale", 1.0))
    if scale != 1.0:
        print(f"PDFs pre-scaled x{scale:.4f} (printer compensation) - the 100 mm scale bar must print as 100 mm",
              flush=True)
    for name in names:
        spec = specs[name]
        pdf = printables.board_pdf(spec, out / f"{name}.pdf", scale=scale)
        png = out / f"{name}.png"
        w_mm, h_mm = printables.board_png(spec, png, args.png_px_per_mm)
        n_sq = int(round(spec.square_mm * args.png_px_per_mm))
        page = printables.page_for(spec)[0]
        print(f"{spec.describe()}\n    {pdf.name} ({page}), {png.name} {w_mm:g} x {h_mm:g} mm incl. quiet zone "
              f"({n_sq / spec.square_mm:.3f} px/mm)", flush=True)
        if not args.no_check:
            fails += check(spec, png, n_sq, cfg, specs)
    for f in fails:
        print(f"FAILED: {f}", flush=True)
    print(f"{len(names)} boards written to {out}" + ("" if fails else (" - checks ok" if not args.no_check else "")),
          flush=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
