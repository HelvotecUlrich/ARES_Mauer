"""Offline eye-in-hand calibration T_flange_cam from a "handeye" dataset (images + flange poses).

    py.exe tools/handeye_solve.py data/he_2026-10-06 [--intrinsics calib/camera_intrinsics.json]
                                  [--holdout-every 5] [--method PARK] [--out calib/handeye.json] [--force]
    py.exe tools/handeye_solve.py data/he_2026-10-06d --check calib/handeye.json      # validate, writes nothing

Per sample: board pose by ChArUco detection (IPPE + LM refine, the [vision] checks) with the calibrated intrinsics,
T_base_flange from the dataset (RTDE at the exposure). Samples without a robot pose, with the arm moving
(max_qd > --max-qd) or without a valid board pose are skipped and listed. Every k-th usable sample (--holdout-every)
is held out: the calibration runs on the rest and is validated on the held-out poses (research 2026-10-05: in-sample
consistency does not reveal a poor calibration, held-out poses do). Prints all five OpenCV methods, their
disagreement, the in-sample spread of the board in the base frame, the motion statistics, the hold-out check and
warnings; writes calib/handeye.json (refuses to overwrite without --force; --dry-run writes nothing).
--nominal-intrinsics uses the ideal pinhole from [camera] (RoboDK simulated camera only).

--check CALIB: no calibration - every usable sample of the dataset is measured with an existing T_flange_cam and
compared with the board pose stored in CALIB (handeye.holdout_check: base-frame deviation and reprojection) and with
the dataset's own mean board pose (spread). Views from another start pose than the calibration's, board untouched =
an independent validation (2026-10-06: did the camera adapter's contact with wrist 1 move the camera?).
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
from mauer.vision import handeye, intrinsics  # noqa: E402
from mauer.vision.dataset import Dataset  # noqa: E402
from mauer.vision.detect import measure  # noqa: E402
from mauer.vision.targets import board_specs  # noqa: E402


def _rel(p: Path) -> str:
    p = Path(p).resolve()
    try:
        return p.relative_to(REPO).as_posix()
    except ValueError:
        return str(p)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="dataset folder (meta.json kind 'handeye')")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--intrinsics", default=None, help="intrinsics JSON (default [vision] intrinsics_file)")
    ap.add_argument("--nominal-intrinsics", action="store_true", help="ideal pinhole from [camera] (simulation)")
    ap.add_argument("--board", default=None, help="board name (default: dataset meta, else 'calib')")
    ap.add_argument("--method", default=handeye.DEFAULT_METHOD, choices=handeye.METHODS)
    ap.add_argument("--holdout-every", type=int, default=None,
                    help="hold out every k-th usable sample for validation (0 = none; default [vision] "
                         "holdout_every, else 5)")
    ap.add_argument("--max-qd", type=float, default=None,
                    help="skip samples whose max |joint speed| during the exposure exceeds this [rad/s] (default "
                         "[vision] max_qd_rad_s, else 0.01 - ASSUMPTION: 0.01 rad/s x 10 ms x 1 m lever = 0.1 mm)")
    ap.add_argument("--out", default=None, help="output JSON (default [vision] handeye_file)")
    ap.add_argument("--force", action="store_true", help="overwrite an existing output file")
    ap.add_argument("--dry-run", action="store_true", help="solve and print, write nothing")
    ap.add_argument("--check", default=None, help="validate this existing calibration JSON on the dataset instead "
                                                  "of solving (writes nothing)")
    args = ap.parse_args(argv)

    cfg = config.load(args.config)
    vcfg = cfg.get("vision", {})
    holdout_every = int(vcfg.get("holdout_every", 5) if args.holdout_every is None else args.holdout_every)
    max_qd = float(vcfg.get("max_qd_rad_s", 0.01) if args.max_qd is None else args.max_qd)
    out = config.repo_path(args.out or vcfg.get("handeye_file", "calib/handeye.json"))
    if out.exists() and not (args.force or args.dry_run or args.check):
        print(f"refusing to overwrite {out} (use --force)", flush=True)
        return 2
    ds = Dataset.load(args.dataset)
    if ds.kind != "handeye":
        print(f"note: dataset kind is {ds.kind!r}", flush=True)
    spec = board_specs(cfg)[args.board or ds.board or "calib"]
    if args.nominal_intrinsics:
        intr, intr_file = intrinsics.nominal(cfg), "nominal ([camera] focal / pixel_um)"
    else:
        p = config.repo_path(args.intrinsics or vcfg.get("intrinsics_file", "calib/camera_intrinsics.json"))
        if not p.exists():
            print(f"intrinsics file {p} missing - run tools/calib_intrinsics.py solve first "
                  "(or --nominal-intrinsics for simulated images)", flush=True)
            return 1
        intr, intr_file = intrinsics.load(p), _rel(p)
    print(f"dataset {ds.path}: {len(ds)} samples, board {spec.describe()}, intrinsics {intr_file}", flush=True)

    used: list[tuple[str, np.ndarray, np.ndarray]] = []
    for img, s in ds:
        name = s["image"]
        T_bf = Dataset.T_base_flange(s)
        if T_bf is None:
            print(f"  {name}: skipped, no robot pose", flush=True)
            continue
        if s.get("max_qd") is not None and float(s["max_qd"]) > max_qd:
            print(f"  {name}: skipped, arm moving (max_qd {float(s['max_qd']):.4f} rad/s)", flush=True)
            continue
        if img.shape[:2] != (intr.height, intr.width):
            print(f"  {name}: skipped, image {img.shape[1]}x{img.shape[0]} != intrinsics "
                  f"{intr.width}x{intr.height}", flush=True)
            continue
        pose = measure(img, [spec], intr, vcfg)[spec.name]
        if not pose.ok:
            print(f"  {name}: skipped, {pose.reason}", flush=True)
            continue
        d = float(np.linalg.norm(pose.T_cam_board[:3, 3]))
        print(f"  {name}: {pose.n_corners} corners, RMS {pose.rms_px:.3f} px, board origin at {d:.1f} mm",
              flush=True)
        used.append((name, T_bf, pose.T_cam_board))

    if args.check:
        return check(args.check, used, spec, intr)
    k = max(holdout_every, 0)
    hold = [u for i, u in enumerate(used) if k > 0 and i % k == k - 1]
    cal = [u for i, u in enumerate(used) if not (k > 0 and i % k == k - 1)]
    print(f"{len(used)} usable samples: {len(cal)} for the calibration, {len(hold)} held out", flush=True)
    if len(cal) < 3:
        print("not enough samples for a hand-eye calibration (need >= 3, research: 20-25)", flush=True)
        return 1
    res = handeye.solve([u[1] for u in cal], [u[2] for u in cal], method=args.method)
    ho = handeye.holdout_check(res, [u[1] for u in hold], [u[2] for u in hold], spec, intr) if hold else None
    print(handeye.format_result(res, ho), flush=True)
    for (name, _, _), dp, da in zip(cal, res.board_in_base_spread["per_sample_pos_mm"],
                                    res.board_in_base_spread["per_sample_ang_deg"]):
        print(f"  in-sample {name}: board deviation {dp:.3f} mm / {da:.4f} deg", flush=True)
    if ho:
        for (name, _, _), dp, da in zip(hold, ho["per_sample_pos_mm"], ho["per_sample_ang_deg"]):
            print(f"  hold-out  {name}: board deviation {dp:.3f} mm / {da:.4f} deg", flush=True)
    dp, da = g.pose_delta(config.T_flange_cam_nominal(cfg), res.T_flange_cam)
    print(f"vs nominal mount [camera.mount] (PLACEHOLDER): {dp:.1f} mm / {da:.2f} deg "
          "(large values: check the bracket drawing or a convention error)", flush=True)
    if args.dry_run:
        print("dry run: nothing written", flush=True)
        return 0
    handeye.save(res, out, holdout=ho, dataset=_rel(ds.path), intrinsics_file=intr_file)
    print(f"written {out}", flush=True)
    return 0


def check(calib_path: str, used: list, spec, intr) -> int:
    """--check: the dataset's samples measured with an existing calibration (see the module docstring)."""
    res = handeye.load(config.repo_path(calib_path))
    if not used:
        print("no usable samples", flush=True)
        return 1
    ho = handeye.holdout_check(res, [u[1] for u in used], [u[2] for u in used], spec, intr)
    Ts = handeye.board_in_base([u[1] for u in used], res.T_flange_cam, [u[2] for u in used])
    sp = g.spread(Ts)
    off = g.pose_delta(res.T_base_board, sp["mean"])
    print(f"check of {calib_path} on {len(used)} samples:", flush=True)
    print(f"  vs its calibration board pose: pos rms {ho['pos_rms_mm']:.3f} / max {ho['pos_max_mm']:.3f} mm, ang rms "
          f"{ho['ang_rms_deg']:.4f} / max {ho['ang_max_deg']:.4f} deg"
          + (f", reprojection rms {ho['reproj_rms_px']:.2f} / max {ho['reproj_max_px']:.2f} px"
             if "reproj_rms_px" in ho else ""), flush=True)
    print(f"  mean of this dataset vs the calibration board pose: {off[0]:.3f} mm / {off[1]:.4f} deg (board untouched "
          "-> calibration error from other views)", flush=True)
    print(f"  spread around this dataset's own mean: pos rms {sp['pos_rms_mm']:.3f} / max {sp['pos_max_mm']:.3f} mm, "
          f"ang rms {sp['ang_rms_deg']:.4f} / max {sp['ang_max_deg']:.4f} deg", flush=True)
    for (name, _, _), dp, da in zip(used, ho["per_sample_pos_mm"], ho["per_sample_ang_deg"]):
        print(f"  {name}: {dp:.3f} mm / {da:.4f} deg from the calibration board pose", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
