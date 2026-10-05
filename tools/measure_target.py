"""Measure ChArUco boards in the UR base frame from the robot: repeatability / settle time, consistency over poses.

    py.exe tools/measure_target.py repeat --host IP [--n 10] [--boards calib,W0] [--settle-s 0,0.5,1,2 --move-mm 20]
    py.exe tools/measure_target.py poses --host IP --poses poses.json [--boards calib] [--report out.json]

Chain: T_base_board = T_base_flange (RTDE at the exposure, mauer.capture) @ T_flange_cam (--handeye, default
[vision] handeye_file; --nominal-mount = [camera.mount] PLACEHOLDER, planning/simulation only) @ T_cam_board (solvePnP
with --intrinsics / [vision] intrinsics_file; --nominal-intrinsics = ideal pinhole).

repeat: N shots from the CURRENT pose (no motion without --move-mm). Per visible board: mean T_base_board and its
  spread [mm, deg] (camera + detection noise, ARES sway), plus the spread of T_cam_board alone. --settle-s takes a
  list (e.g. 0,0.5,1,2): for each settle time the N shots are repeated. Settle times only mean something with
  --move-mm D: before every shot the TCP moves D mm up (base z) and back (movel, [ur] v_lin/a_lin), so the arm has
  just stopped when the settle time starts - the settle-time experiment for the sprung ARES ([camera] settle_s is a
  PLACEHOLDER: Heini thesis needed 2 s for a vibrating mount).
poses: look poses from a JSON file (format: tools/calib_handeye.py docstring; `calib_handeye.py plan --write`
  makes one around the deck calib board); the same board measured from every pose. The spread of T_base_board over
  the poses is THE validation of intrinsics + hand-eye (in-sample hand-eye consistency alone is not).

--ursim: URSim + mauer.simcam.SynthCamera rendering the calib board at its nominal deck pose with a ground-truth
T_flange_cam = nominal @ --inject (see tools/calib_handeye.py); results are also compared with that ground truth.
Real robot: --host required, typed confirmation before the first motion (--yes skips), refuses to move while
[ur] payload_tool_kg is 0 unless --sim. Ctrl-C aborts the running block.
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO, REPO / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402

import calib_handeye as ch  # noqa: E402
from mauer import config  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer.capture import CaptureError, boards_in_base, capture_shot, payload  # noqa: E402
from mauer.ur import script  # noqa: E402
from mauer.vision import handeye  # noqa: E402
from mauer.vision.dataset import Dataset  # noqa: E402
from mauer.vision.intrinsics import to_jsonable  # noqa: E402
from mauer.vision.targets import board_specs  # noqa: E402


def summarize(Ts: list[np.ndarray], T_true: np.ndarray | None = None) -> dict:
    """Mean pose (xyz mm, rpy deg), spread around it and, if given, the deviation of the mean from T_true."""
    if not Ts:
        return {"n": 0}
    sp = g.spread(Ts)
    mean = sp.pop("mean")
    xyz, rpy = g.xyz_rpy(mean)
    out = {"n": len(Ts), "mean": mean, "mean_xyz_mm": xyz, "mean_rpy_deg": rpy, **sp,
           "per_sample": [list(g.pose_delta(mean, T)) for T in Ts]}
    if T_true is not None:
        d = np.array([g.pose_delta(np.asarray(T_true, float), T) for T in Ts])
        out["truth_mean_delta"] = list(g.pose_delta(np.asarray(T_true, float), mean))
        out["truth_max_mm"], out["truth_max_deg"] = float(d[:, 0].max()), float(d[:, 1].max())
    return out


def fmt_summary(name: str, s: dict) -> str:
    if not s.get("n"):
        return f"{name}: not measured"
    txt = (f"{name}: n {s['n']}, mean xyz ({', '.join(f'{v:.2f}' for v in s['mean_xyz_mm'])}) mm, rpy "
           f"({', '.join(f'{v:.3f}' for v in s['mean_rpy_deg'])}) deg | spread pos rms {s['pos_rms_mm']:.3f} / max "
           f"{s['pos_max_mm']:.3f} mm, ang rms {s['ang_rms_deg']:.4f} / max {s['ang_max_deg']:.4f} deg")
    if "truth_mean_delta" in s:
        txt += (f" | vs ground truth: mean {s['truth_mean_delta'][0]:.3f} mm / {s['truth_mean_delta'][1]:.4f} deg, "
                f"max {s['truth_max_mm']:.3f} mm / {s['truth_max_deg']:.4f} deg")
    return txt


def select_specs(cfg: dict, text: str | None) -> dict:
    specs = board_specs(cfg)
    if not text or text == "all":
        return specs
    names = [n.strip() for n in text.split(",") if n.strip()]
    unknown = [n for n in names if n not in specs]
    if unknown:
        raise SystemExit(f"unknown board(s) {unknown}; known: {', '.join(specs)}")
    return {n: specs[n] for n in names}


def flange_cam(args, cfg: dict) -> tuple[np.ndarray, str]:
    if args.nominal_mount:
        return config.T_flange_cam_nominal(cfg), "nominal [camera.mount] (PLACEHOLDER)"
    p = config.repo_path(args.handeye or cfg.get("vision", {}).get("handeye_file", "calib/handeye.json"))
    if not p.exists():
        raise SystemExit(f"{p} missing - run tools/calib_handeye.py solve (or --nominal-mount for a rough check)")
    return handeye.load(p).T_flange_cam, str(p)


def truth_of(rig: ch.Rig, name: str) -> np.ndarray | None:
    return None if rig.truth is None else rig.truth["T_base_board"].get(name)


def write_report(path: str | None, doc: dict) -> None:
    if path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(to_jsonable(doc), indent=1) + "\n", encoding="utf-8")
        print(f"report written {p}", flush=True)


def open_dataset(path: str | None, rig: ch.Rig, notes: str) -> Dataset | None:
    if not path:
        return None
    return Dataset.create(config.repo_path(path), "measure", camera=rig.camera.info(), notes=notes)


# ── repeat ────────────────────────────────────────────────────────────────────
def bump_block(cfg: dict, T_base_tcp0: np.ndarray, move_mm: float, sim: bool, speeds: script.Speeds) -> str:
    """Preamble + movel move_mm up along base z and back to T_base_tcp0 (the arm has just stopped afterwards)."""
    kg, cog = payload(cfg, sim=sim)
    T_up = g.transl(0.0, 0.0, float(move_mm)) @ T_base_tcp0
    return "\n".join([script.preamble(config.T_flange_tcp(cfg), kg, cog), script.movel(T_up, speeds.a_lin,
                      speeds.v_lin), script.movel(T_base_tcp0, speeds.a_lin, speeds.v_lin)])


def cmd_repeat(args) -> int:
    cfg = config.load(args.config)
    specs = select_specs(cfg, args.boards)
    X, x_label = flange_cam(args, cfg)
    intr, intr_label = ch.load_intrinsics(args, cfg, args.ursim)
    settles = [float(v) for v in (args.settle_s or str(cfg.get("camera", {}).get("settle_s", 0.0))).split(",")]
    vcfg = cfg.get("vision", {})
    if args.move_mm > 0.0:
        refusal = ch.check_payload(cfg, args.sim or args.ursim)
        if refusal:
            print(refusal, flush=True)
            return 2
    elif len(settles) > 1:
        print("note: without --move-mm the arm stands still, so the settle times are not exercised", flush=True)
    print(f"T_flange_cam {x_label}, intrinsics {intr_label}, boards {', '.join(specs)}", flush=True)
    rig = ch.open_rig(args, cfg)
    report = {"kind": "repeat", "n": args.n, "move_mm": args.move_mm, "T_flange_cam": x_label,
              "intrinsics": intr_label, "settle": []}
    try:
        st = rig.link.state()
        T_tcp0 = rig.link.flange_T(st) @ config.T_flange_tcp(cfg)
        if args.move_mm > 0.0 and not ch.confirm_motion(
                args, f"{len(settles) * args.n} moves of {args.move_mm} mm up and back from the current pose"):
            print("not confirmed - nothing moved", flush=True)
            return 1
        ds = open_dataset(args.dataset, rig, "measure_target repeat")
        for s in settles:
            per_board: dict[str, list] = {n: [] for n in specs}
            per_cam: dict[str, list] = {n: [] for n in specs}
            flanges = []
            for k in range(args.n):
                if args.move_mm > 0.0:
                    r = rig.link.run_block(bump_block(cfg, T_tcp0, args.move_mm, rig.sim, rig.speeds), "bump",
                                           args.timeout_s)
                    if not r.ok:
                        print(f"bump move failed: {r.error} - stopping", flush=True)
                        return 1
                try:
                    shot = capture_shot(rig.link, rig.camera, settle_s=s, max_qd=ch.max_qd_value(args, cfg),
                                        retries=args.retries)
                except CaptureError as e:
                    print(f"settle {s:g} s, shot {k}: no usable image: {e}", flush=True)
                    continue
                flanges.append(shot.T_base_flange)
                seen = boards_in_base(shot, specs, intr, vcfg, X)
                for name, (bp, T) in seen.items():
                    if T is not None:
                        per_board[name].append(T)
                        per_cam[name].append(bp.T_cam_board)
                if ds is not None:
                    ds.add(shot.frame.image, **shot.dataset_fields(), settle_s=s, shot=k,
                           boards={n: {"ok": bp.ok, "n_corners": bp.n_corners, "T_base_board": T}
                                   for n, (bp, T) in seen.items()})
                parts = [f"{n} {bp.n_corners} corners" + ("" if T is not None else f" ({bp.reason})")
                         for n, (bp, T) in seen.items() if bp.n_corners or T is not None]
                print(f"settle {s:g} s, shot {k}: {', '.join(parts) or 'no board'}", flush=True)
            entry = {"settle_s": s, "boards": {}, "cam": {}, "flange": summarize(flanges)}
            print(f"── settle {s:g} s ──", flush=True)
            for name in specs:
                if per_board[name]:
                    entry["boards"][name] = summarize(per_board[name], truth_of(rig, name))
                    entry["cam"][name] = summarize(per_cam[name])
                    print("  " + fmt_summary(f"{name} in base", entry["boards"][name]), flush=True)
                    c = entry["cam"][name]
                    print(f"    T_cam_board alone: pos rms {c['pos_rms_mm']:.3f} / max {c['pos_max_mm']:.3f} mm, ang "
                          f"rms {c['ang_rms_deg']:.4f} / max {c['ang_max_deg']:.4f} deg", flush=True)
            if flanges and args.move_mm > 0.0:
                f = entry["flange"]
                print(f"  flange return repeatability (RTDE): pos max {f['pos_max_mm']:.3f} mm, ang max "
                      f"{f['ang_max_deg']:.4f} deg", flush=True)
            report["settle"].append(entry)
    except KeyboardInterrupt:
        print(f"interrupted - abort: {rig.link.abort()}", flush=True)
        return 130
    finally:
        rig.close()
    write_report(args.report, report)
    return 0 if any(e["boards"] for e in report["settle"]) else 1


# ── poses ─────────────────────────────────────────────────────────────────────
def cmd_poses(args) -> int:
    cfg = config.load(args.config)
    specs = select_specs(cfg, args.boards)
    X, x_label = flange_cam(args, cfg)
    intr, intr_label = ch.load_intrinsics(args, cfg, args.ursim)
    items = ch.load_poses_json(args.poses, cfg)
    print(f"{len(items)} poses from {args.poses}; T_flange_cam {x_label}, intrinsics {intr_label}, boards "
          f"{', '.join(specs)}", flush=True)
    refusal = ch.check_payload(cfg, args.sim or args.ursim)
    if refusal:
        print(refusal, flush=True)
        return 2
    rig = ch.open_rig(args, cfg)
    try:
        if not ch.confirm_motion(args, f"{len(items)} look poses from {args.poses}"):
            print("not confirmed - nothing moved", flush=True)
            return 1
        ds = open_dataset(args.dataset, rig, f"measure_target poses {args.poses}")
        try:
            meas = ch.measure_poses(rig, items, specs, intr, X, ch.settle_value(args, cfg), ch.max_qd_value(args, cfg),
                                    args.retries, args.timeout_s, ds)
        except KeyboardInterrupt:
            print(f"interrupted - abort: {rig.link.abort()}", flush=True)
            return 130
        truths = {n: truth_of(rig, n) for n in specs}
    finally:
        rig.close()
    per_board: dict[str, list] = {n: [] for n in specs}
    names: dict[str, list] = {n: [] for n in specs}
    for m in meas:
        if not m["ok"]:
            continue
        parts = []
        for name, (bp, T) in m["boards"].items():
            if T is not None:
                per_board[name].append(T)
                names[name].append(m["name"])
                parts.append(f"{name} {bp.n_corners} corners RMS {bp.rms_px:.3f} px")
        print(f"{m['name']}: " + (", ".join(parts) or "no board measured"), flush=True)
    report = {"kind": "poses", "poses_file": str(args.poses), "T_flange_cam": x_label, "intrinsics": intr_label,
              "n_poses": len(items), "n_reached": sum(1 for m in meas if m["ok"]),
              "unreachable": [m["name"] for m in meas if m.get("reason") == "unreachable"], "boards": {}}
    print("── consistency of T_base_board over the poses ──", flush=True)
    for name in specs:
        s = summarize(per_board[name], truths.get(name))
        if s["n"]:
            s["poses"] = names[name]
            report["boards"][name] = s
            print(fmt_summary(name, s), flush=True)
            for pn, (dp, da) in zip(names[name], s["per_sample"]):
                print(f"    {pn}: {dp:.3f} mm / {da:.4f} deg from the mean", flush=True)
    write_report(args.report, report)
    return 0 if any(s["n"] >= 2 for s in report["boards"].values()) else 1


# ── main ──────────────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, hlp in (("repeat", "N shots from the current pose (repeatability, settle time)"),
                      ("poses", "the same board from several look poses (hand-eye + intrinsics validation)")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("--boards", default="all", help="comma-separated board names or 'all'")
        p.add_argument("--handeye", default=None, help="T_flange_cam JSON (default [vision] handeye_file)")
        p.add_argument("--nominal-mount", action="store_true", help="use the [camera.mount] PLACEHOLDER instead")
        p.add_argument("--intrinsics", default=None)
        p.add_argument("--nominal-intrinsics", action="store_true")
        p.add_argument("--dataset", default=None, help="also store the images as a 'measure' dataset")
        p.add_argument("--report", default=None, help="write the results as JSON")
        ch.add_robot_args(p, "comma-separated settle times [s] to compare, e.g. 0,0.5,1,2 (default [camera] "
                             "settle_s)" if name == "repeat" else "wait before each image [s] (default [camera] "
                                                                  "settle_s)")
        if name == "repeat":
            p.add_argument("--n", type=int, default=10, help="shots per settle time")
            p.add_argument("--move-mm", type=float, default=0.0, help="move the TCP this far up and back before "
                                                                      "each shot [mm] (settle-time experiment)")
        else:
            p.add_argument("--poses", required=True, help="poses JSON (calib_handeye.py plan --write)")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("rtde").setLevel(logging.ERROR)
    return {"repeat": cmd_repeat, "poses": cmd_poses}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
