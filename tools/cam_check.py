"""Camera check for the IDS GV-51F0CP-M-GL (IDS peak): environment, test grabs, live focus/exposure view.

    py.exe tools/cam_check.py list                    # IDS peak versions, GenTL producers, cameras, firmware >= 3.31
    py.exe tools/cam_check.py grab -n 10              # 10 frames -> data/cam_check/<time>/ (PNG + JSON) with timing,
                                                      # grey mean/min/max, saturation %, focus measure
    py.exe tools/cam_check.py live                    # preview: focus measure, histogram, saturation; keys below
    py.exe tools/cam_check.py exposure 8000           # set exposure [us], print the applied value + a test frame
    ... --source files --folder <dir>                 # the same on recorded images (FileCamera), no camera needed

Camera settings come from config/station.toml [camera] (serial, ip, exposure_us, gain, ...); --serial/--ip/
--exposure-us/--gain override them for this run (--ip "" = take the first camera). Nothing is stored in the camera:
put values you want to keep into the config with a status tag and source.

`live` keys: + / - exposure x/÷ step, s save the full-resolution frame (PNG + JSON), z 1:1 centre zoom, r reset the
focus peak, q / Esc quit. Saturated pixels are drawn red, the green box is the focus ROI.

Focus / iris locking at the working distance (needs `live`; standard machine-vision practice, ASSUMPTION – no
IDS-specific procedure read): (1) camera on the flange, arm still, a high-contrast target (the calib ChArUco board)
at [camera] working_dist in the image centre; (2) open the iris fully (F2.8: shallowest depth of field, sharpest
optimum), set the exposure with +/- until no pixel saturates; (3) turn the focus ring through the maximum of the
focus measure (shown with its peak; z for a 1:1 view) and back to the peak; lock the focus screw; (4) stop down to the
working aperture (F5.6-8, [camera] comment), raise the exposure until the mean grey is back, lock the iris screw;
(5) save a reference frame (s). Any change of focus or iris afterwards invalidates the intrinsics and the hand-eye
calibration.

Exit codes: 0 ok, 1 camera problem (no camera, errors, old firmware), 2 setup problem (IDS peak runtime/package
missing) or CameraError.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from mauer import config  # noqa: E402
from mauer.camera import Camera, CameraError, open_camera, save_frame  # noqa: E402
from mauer.camera.base import center_roi, focus_measure, full_scale, histogram, image_stats  # noqa: E402

DEFAULT_OUT = REPO / "data" / "cam_check"


def _p(msg: str = "") -> None:
    print(msg, flush=True)


def _packet_size(s: str | None) -> int | str | None:
    if s is None or s in ("auto", "max"):
        return s
    return int(s)


def make_camera(args: argparse.Namespace) -> Camera:
    """Open the camera selected by the common options."""
    cfg = config.load(args.config)
    if args.source == "files":
        if not args.folder:
            raise CameraError("--source files needs --folder <image folder>")
        return open_camera(cfg, "files", folder=args.folder, pattern=args.pattern, loop=args.loop,
                           exposure_us=args.exposure_us, gain=args.gain)
    return open_camera(cfg, "ids", serial=args.serial, ip=args.ip, exposure_us=args.exposure_us, gain=args.gain,
                       timeout_ms=args.timeout_ms, packet_size=_packet_size(args.packet_size),
                       cti_path=args.cti, verbose=not args.quiet)


def _print_info(info: dict[str, Any]) -> None:
    keys = ("kind", "model", "serial", "firmware", "firmware_ok", "ip", "subnet", "mac", "width", "height",
            "pixel_format", "exposure_us", "gain", "packet_size_b", "packet_size_max_b", "temperature_c", "folder",
            "n_images", "ids_peak_api")
    _p("camera: " + ", ".join(f"{k} {info[k]}" for k in keys if info.get(k) is not None))


def _stats_line(i: int, frame, roi: float) -> tuple[str, dict[str, Any]]:
    st = image_stats(frame.image)
    foc = focus_measure(frame.image, roi)
    row = {"i": i, "frame_id": frame.meta.get("frame_id"), "t_start": frame.t_start, "t_end": frame.t_end,
           "latency_ms": 1e3 * frame.latency_s, "focus": foc, **st}
    line = (f"  {i:3d}  id {str(row['frame_id']):>6}  latency {row['latency_ms']:7.1f} ms  grey mean "
            f"{st['mean']:6.1f} min {st['min']:3d} max {st['max']:3d}  sat {st['saturated_pct']:7.3f} %  "
            f"focus {foc:9.1f}")
    return line, row


# ── list ─────────────────────────────────────────────────────────────────────
def cmd_list(args: argparse.Namespace) -> int:
    from mauer.camera import ids

    env = ids.environment(open_devices=not args.no_open, cti_path=args.cti or "",
                          log=lambda m: _p(f"  [ids] {m}"))
    if args.json:
        _p(json.dumps(env, indent=1, default=str))
    pk = env["packages"]
    _p(f"Python {env['python']}; ids_peak {pk['ids_peak']} (genericAPI {env['ids_peak_api']}), "
       f"ids_peak_ipl {pk['ids_peak_ipl']}, ids_peak_common {pk['ids_peak_common']}")
    _p(f"{env['gentl_var']}: {env['gentl_dirs'] or '(not set)'}")
    accepted = set(env["ids_ctis"])
    for c in env["gentl_ctis"]:
        _p(f"  producer {c}  {'(IDS)' if c in accepted else '(third-party - not usable by ids_peak)'}")
    _p(f"IDS producers found by ids_peak (EnvironmentInspector): {env['ids_ctis'] or 'none'}")
    _p(f"IDS producers in the install folder: {env['ids_install_ctis'] or 'none (IDS peak not installed?)'}")
    for s in env["systems"]:
        _p(f"system: {s['vendor']} {s['model']} {s['version']} [{s['tl_type']}] {s['cti']}")
    for i in env["interfaces"]:
        _p(f"interface: {i}")
    _p(f"IDS GigE Vision producer running: {'yes' if env['gev_ok'] else 'NO'}")
    for c, err in env["gev_errors"]:
        _p(f"  {c}: {err}")
    rc = 0
    fw_min = ".".join(map(str, ids.MIN_FIRMWARE))
    for d in env["devices"]:
        _p(f"device: {d['model']} serial {d['serial']} ip {d.get('ip')} mac {d.get('mac')} "
           f"[{d['tl_type']}] access {d['access_status']} openable {d['openable_control']}")
        if d["openable_control"] is False:
            rc = 1
            _p("  not openable: in use (IDS peak Cockpit, another script) or the camera IP is not in the subnet of "
               "the laptop NIC it is plugged into - give it a persistent IP in that subnet (ids_ipconfig / Cockpit "
               "IP configuration, [camera] ip), see docs/CAMERA_SETUP.md")
        if "firmware" in d:
            ok = d["firmware_ok"]
            verdict = "ok" if ok else ("TOO OLD" if ok is False else "format unknown - check in IDS peak Cockpit")
            _p(f"  firmware {d['firmware']} (needs >= {fw_min} for this model: {verdict}), subnet {d.get('subnet')},"
               f" user id {d.get('user_id')}, temperature {d.get('temperature_c')} C")
            if ok is False:
                rc = 1
        if "open_error" in d:
            _p(f"  could not open read-only: {d['open_error']}")
    if env["problem"]:
        _p(f"PROBLEM: {env['problem']}")
        return 2 if not env["gev_ok"] else 1
    return rc


# ── grab ─────────────────────────────────────────────────────────────────────
def cmd_grab(args: argparse.Namespace) -> int:
    out = Path(args.out) if args.out else DEFAULT_OUT / time.strftime("%Y-%m-%d_%H%M%S")
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    with make_camera(args) as cam:
        info = cam.info()
        _print_info(info)
        _p(f"grabbing {args.n} frame(s){'' if args.no_save else ' -> ' + str(out)}")
        t0 = time.time()
        for i in range(args.n):
            if i and args.interval_s > 0:
                time.sleep(args.interval_s)
            try:
                fr = cam.grab()
            except CameraError as e:
                errors.append(f"{i}: {e}")
                _p(f"  {i:3d}  ERROR {e}")
                continue
            line, row = _stats_line(i, fr, args.roi)
            if not args.no_save:
                row["file"] = save_frame(fr, out / f"img_{i:04d}.png")[0].name
            rows.append(row)
            _p(line)
        dt = time.time() - t0
    summary: dict[str, Any] = {"n_requested": args.n, "n_ok": len(rows), "n_errors": len(errors),
                               "duration_s": dt}
    if rows:
        lat = np.array([r["latency_ms"] for r in rows])
        summary.update({"latency_ms_mean": float(lat.mean()), "latency_ms_max": float(lat.max()),
                        "frames_per_s": len(rows) / dt if dt > 0 else None,
                        "grey_mean": float(np.mean([r["mean"] for r in rows])),
                        "saturated_pct_max": float(max(r["saturated_pct"] for r in rows)),
                        "focus_mean": float(np.mean([r["focus"] for r in rows]))})
        _p(f"summary: {len(rows)}/{args.n} ok, latency mean {summary['latency_ms_mean']:.1f} ms max "
           f"{summary['latency_ms_max']:.1f} ms, {summary['frames_per_s']:.2f} frames/s, grey mean "
           f"{summary['grey_mean']:.1f}, saturation max {summary['saturated_pct_max']:.3f} %")
    if errors:
        _p(f"{len(errors)} error(s); first: {errors[0]}")
    if not args.no_save and rows:
        out.mkdir(parents=True, exist_ok=True)
        (out / "cam_check_grab.json").write_text(json.dumps(
            {"info": info, "summary": summary, "frames": rows, "errors": errors,
             "args": {k: v for k, v in vars(args).items() if k != "func"}}, indent=1, default=str), encoding="utf-8")
        _p(f"wrote {len(rows)} frame(s) + cam_check_grab.json to {out}")
    return 0 if not errors else 1


# ── exposure ─────────────────────────────────────────────────────────────────
def cmd_exposure(args: argparse.Namespace) -> int:
    with make_camera(args) as cam:
        if args.value is not None:
            applied = cam.set_exposure_us(args.value)
            _p(f"exposure requested {args.value} us -> applied {applied} us")
        info = cam.info()
        rng = info.get("ranges", {}) or {}
        _p(f"exposure {info.get('exposure_us')} us (device range {rng.get('exposure_us')}), gain {info.get('gain')} "
           f"(range {rng.get('gain')})")
        if not args.no_grab:
            line, _ = _stats_line(0, cam.grab(), args.roi)
            _p(line)
    _p("Not stored in the camera: put the value into config/station.toml [camera] exposure_us (with status + source).")
    return 0


# ── live ─────────────────────────────────────────────────────────────────────
def render_live(img: np.ndarray, lines: list[str], roi_frac: float = 0.25, max_width: int = 1280,
                zoom: bool = False) -> np.ndarray:
    """Preview image (BGR uint8): downscaled (or 1:1 centre crop with zoom), saturated pixels red, focus ROI green,
    text lines top-left and a grey histogram bottom-right."""
    import cv2

    h, w = img.shape
    fs = full_scale(img)
    rs, cs = center_roi(img, roi_frac)
    if zoom:                                        # 1:1 centre crop, as large as the preview width allows
        cw, ch = min(w, max_width), min(h, int(max_width * 0.75))
        r0, c0 = (h - ch) // 2, (w - cw) // 2
        view = img[r0:r0 + ch, c0:c0 + cw]
        roi_box = (cs.start - c0, rs.start - r0, cs.stop - c0, rs.stop - r0)
    else:
        s = min(1.0, max_width / w)
        view = cv2.resize(img, (max(1, round(w * s)), max(1, round(h * s))), interpolation=cv2.INTER_AREA) \
            if s < 1.0 else img
        roi_box = tuple(round(v * s) for v in (cs.start, rs.start, cs.stop, rs.stop))
    v8 = view if view.dtype == np.uint8 else (view.astype(np.float64) * 255.0 / fs).astype(np.uint8)
    bgr = cv2.cvtColor(np.ascontiguousarray(v8), cv2.COLOR_GRAY2BGR)
    bgr[view >= fs] = (0, 0, 255)                   # saturated -> red
    cv2.rectangle(bgr, roi_box[:2], roi_box[2:], (0, 255, 0), 1)
    scale = max(0.4, bgr.shape[1] / 1800.0)
    lh = int(22 * scale / 0.6)
    box_w = int(max((cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] for t in lines), default=0)) + 12
    cv2.rectangle(bgr, (0, 0), (min(box_w, bgr.shape[1] - 1), lh * len(lines) + 8), (0, 0, 0), -1)
    for k, t in enumerate(lines):
        cv2.putText(bgr, t, (6, lh * (k + 1)), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    hist = histogram(img).astype(np.float64)
    hw, hh = 256, 80
    if bgr.shape[1] > hw + 10 and bgr.shape[0] > hh + lh * len(lines) + 20:
        panel = np.zeros((hh, hw, 3), np.uint8)
        hn = np.sqrt(hist / max(hist.max(), 1.0))   # sqrt: small bins stay visible
        for b in range(256):
            top = hh - 1 - int(hn[b] * (hh - 2))
            cv2.line(panel, (b, hh - 1), (b, top), (0, 0, 255) if b == 255 else (200, 200, 200), 1)
        bgr[-hh - 4:-4, -hw - 4:-4] = panel
    return bgr


def cmd_live(args: argparse.Namespace) -> int:
    import cv2

    out = Path(args.out) if args.out else DEFAULT_OUT / "live"
    win = "cam_check live (q quit)"
    with make_camera(args) as cam:
        info = cam.info()
        _print_info(info)
        cfg_cam = config.load(args.config).get("camera", {})
        exposure = info.get("exposure_us") or args.exposure_us or cfg_cam.get("exposure_us")
        if exposure is None:
            raise CameraError("unknown exposure time - pass --exposure-us")
        zoom, peak, fps, t_prev = False, 0.0, 0.0, time.time()
        _p("keys: +/- exposure, s save, z zoom, r reset focus peak, q quit")
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        except cv2.error as e:
            raise CameraError(f"no display for the live view: {e}") from e
        while True:
            try:
                fr = cam.grab()
            except CameraError as e:
                _p(f"grab: {e}")
                if args.source == "files":
                    break
                time.sleep(0.5)
                continue
            now = time.time()
            fps = 0.8 * fps + 0.2 / max(now - t_prev, 1e-6) if fps else 1.0 / max(now - t_prev, 1e-6)
            t_prev = now
            st = image_stats(fr.image)
            foc = focus_measure(fr.image, args.roi)
            peak = max(peak, foc)
            lines = [f"exposure {exposure:.0f} us  gain {fr.meta.get('gain')}  {fps:4.1f} fps  "
                     f"latency {1e3 * fr.latency_s:.0f} ms",
                     f"grey mean {st['mean']:.1f}  min {st['min']}  max {st['max']}  "
                     f"saturated {st['saturated_pct']:.3f} %",
                     f"focus {foc:.1f}  (peak {peak:.1f}, {100 * foc / peak if peak else 0:.0f} %)  "
                     f"ROI {100 * args.roi:.0f} %{'  ZOOM 1:1' if zoom else ''}",
                     "+/- exposure  s save  z zoom  r reset peak  q quit"]
            cv2.imshow(win, render_live(fr.image, lines, args.roi, args.max_width, zoom))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k in (ord("+"), ord("=")):
                exposure = cam.set_exposure_us(exposure * args.step)
                _p(f"exposure -> {exposure:.0f} us")
            elif k == ord("-"):
                exposure = cam.set_exposure_us(exposure / args.step)
                _p(f"exposure -> {exposure:.0f} us")
            elif k == ord("s"):
                path = save_frame(fr, out / f"live_{time.strftime('%Y-%m-%d_%H%M%S')}.png")[0]
                _p(f"saved {path} (focus {foc:.1f}, exposure {exposure:.0f} us)")
            elif k == ord("z"):
                zoom = not zoom
            elif k == ord("r"):
                peak = 0.0
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
        cv2.destroyAllWindows()
    return 0


# ── CLI ──────────────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="\n\n".join(__doc__.split("\n\n")[:2]),
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="Setup (IDS peak runtime, NIC, IP): docs/CAMERA_SETUP.md")
    common = argparse.ArgumentParser(add_help=False)
    g = common.add_argument_group("camera")
    g.add_argument("--config", default=None, help="station config (default config/station.toml)")
    g.add_argument("--source", choices=("ids", "files"), default="ids", help="IDS camera or a folder of images")
    g.add_argument("--folder", help="image folder for --source files")
    g.add_argument("--pattern", default="*.png", help="file pattern for --source files")
    g.add_argument("--loop", action="store_true", help="--source files: start again at the end")
    g.add_argument("--serial", default=None, help="camera serial (default [camera] serial)")
    g.add_argument("--ip", default=None, help='camera IP (default [camera] ip; "" = first camera found)')
    g.add_argument("--exposure-us", type=float, default=None, help="exposure time [us] (default [camera])")
    g.add_argument("--gain", type=float, default=None, help="gain factor, 1.0 = none (default [camera])")
    g.add_argument("--timeout-ms", type=int, default=None, help="grab timeout after the trigger [ms]")
    g.add_argument("--packet-size", default=None, help='GigE packet size: "auto", "max" or bytes')
    g.add_argument("--cti", default="", help="IDS GenTL producer .cti if it is not on GENICAM_GENTL64_PATH")
    g.add_argument("--roi", type=float, default=0.25, help="focus ROI side as a fraction of the image side")
    g.add_argument("--quiet", action="store_true", help="no IdsCamera progress messages")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list", help="IDS peak environment and cameras (no camera settings changed)")
    sp.add_argument("--cti", default="", help="IDS GenTL producer .cti to add")
    sp.add_argument("--no-open", action="store_true", help="do not open the cameras read-only for firmware/IP")
    sp.add_argument("--json", action="store_true", help="also print the raw result as JSON")
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("grab", parents=[common], help="grab N frames to a folder with timing and grey statistics")
    sp.add_argument("-n", type=int, default=5, help="number of frames")
    sp.add_argument("--interval-s", type=float, default=0.0, help="pause between frames [s]")
    sp.add_argument("--out", default=None,
                    help=f"output folder (default {DEFAULT_OUT.relative_to(REPO).as_posix()}/<time>)")
    sp.add_argument("--no-save", action="store_true", help="only print the statistics")
    sp.set_defaults(func=cmd_grab)

    sp = sub.add_parser("live", parents=[common], help="live preview with focus measure, histogram, saturation")
    sp.add_argument("--max-width", type=int, default=1280, help="preview width [px]")
    sp.add_argument("--step", type=float, default=1.25, help="exposure factor per +/- key")
    sp.add_argument("--out", default=None, help="folder for frames saved with s (default data/cam_check/live)")
    sp.set_defaults(func=cmd_live)

    sp = sub.add_parser("exposure", parents=[common], help="set the exposure time and print the applied value")
    sp.add_argument("value", type=float, nargs="?", default=None, help="exposure time [us] (omit: only print)")
    sp.add_argument("--no-grab", action="store_true", help="no test frame")
    sp.set_defaults(func=cmd_exposure)
    return ap


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")    # py.exe piped output is cp1252 on Windows
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except CameraError as e:
        _p(f"ERROR: {e}")
        return 2
    except KeyboardInterrupt:
        _p("interrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
