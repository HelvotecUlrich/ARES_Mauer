"""Run a job (mauer.job v1/v2) - in the pure-Python simulated world, on the real hardware, or just print the plan.

    py.exe tools/run_job.py data/jobs/nominal_C.json --dry-run
    py.exe tools/run_job.py --nominal L --sim [--scenario realistic] [--seed 1] [--compare] [--stops 0:1]
    py.exe tools/run_job.py --nominal 24 --sim ...           # straight wall (config without legs)
    py.exe tools/run_job.py data/jobs/nominal_L24.json --real [--step] [--stops 0]

--sim       mauer.simworld: ARES drive errors and slip (--scenario none | e003 | slip | realistic, mauer.simworld
            .scenario), simulated UR5 and flange camera (rendered ChArUco boards). Prints a table of the placement
            errors (stone pose at release vs nominal wall pose: horizontal, at the pins, dx along / dy across the
            wall, dz, yaw; seated = pin error within the capture range ~10 mm, README). --no-camera = pure dead
            reckoning (nominal moves and frames, no images); --compare runs both with the same seed. L jobs: the
            true ARES path is checked against legs, plates and table (FLOOR CONTACT lines, mauer.simworld).
            --lenient-grasp: the simulated gripper never misses a stone (lets dead reckoning run to the end).
--real      URLink ([ur]), AresAds ([ares_ads], amr_hmi v2 open, MANUAL), IDS camera ([camera]), calibration files
            ([vision] intrinsics_file / handeye_file). Refuses to start while safety-relevant values are
            PLACEHOLDER/unknown (mauer.sequencer.preflight_real lists every problem); there is no override for real
            runs (--i-know only applies to --sim --preflight). --step asks before every motion.
--dry-run   print the plan (stops, ARES moves and routes, looks, stones with type and magazine slot, predicted
            reloads with the stone types).
--nominal N build the job on the fly with tools/make_job.py build_nominal(length=N) instead of a job file;
            --nominal L builds the L of the config ([[wall.legs]]).

The run log (JSON lines, every measurement and decision) goes to --log-dir, default data/runs/<timestamp>/.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from mauer import config as mconfig  # noqa: E402
from mauer import job as mjob  # noqa: E402
from mauer.reference import relative_move  # noqa: E402
from mauer.floor import route_stats, segments  # noqa: E402
from mauer.sequencer import Sequencer, SequencerError, preflight_real  # noqa: E402


def parse_stops(text: str | None, n: int) -> tuple[int, int | None]:
    """"2" -> (2, 2), "1:3" -> (1, 3), "1:" -> (1, None), None -> (0, None)."""
    if not text:
        return 0, None
    if ":" in text:
        a, b = text.split(":", 1)
        return int(a or 0), (int(b) if b else None)
    k = int(text)
    return k, k


def load_job(args, cfg: dict) -> mjob.Job:
    if args.nominal:
        import make_job
        if str(args.nominal).upper() == "L":
            return make_job.build_l(cfg)
        return make_job.build_nominal(cfg, int(args.nominal))
    if not args.job:
        raise SystemExit("give a job file or --nominal LENGTH")
    return mjob.load(args.job)


# ── dry run ───────────────────────────────────────────────────────────────────
def print_route(route, indent: str = "      ") -> None:
    """Every leg of a route as the PLC relative move (body frame of the leg's start waypoint)."""
    segs, _ = segments(route)
    for i, sg in enumerate(segs):
        if sg.kind == "translate":
            dx, dy, _ = relative_move(sg.start, sg.end)
            print(f"{indent}{i}: translate dx {dx:+.1f} mm, dy {dy:+.1f} mm -> {sg.end.describe()}")
        else:
            print(f"{indent}{i}: rotate {math.degrees(sg.angle_rad):+.1f} deg at ({sg.start.x_mm:.0f}, "
                  f"{sg.start.y_mm:.0f}) mm")


def dry_run(job: mjob.Job) -> None:
    print(job.summary())
    mag = mjob.SlotState.magazine(job.magazine)
    full_station = mjob.SlotState.station(job.station)
    station = full_station.copy()
    n_st = len(job.station.take_order)
    reloads = refills = 0
    prev = None
    stones = job.stones()
    done = 0
    so = float((job.meta.get("route_check") or {}).get("arrival_standoff_mm", 0.0) or 0.0)
    last = (f"the last leg ends {so:g} mm before the stop, closed loop from the wall measurement there" if so > 0
            else "last leg closed loop")
    for s in job.stops:
        leg = f" (leg {s.leg}, a {s.a_mm:.0f} mm)" if s.leg else ""
        if prev is not None and s.route:
            st = route_stats(s.route)
            print(f"stop {s.index}{leg}: ARES route, {st['translations']} translations + {st['rotations']} "
                  f"rotation(s), {st['length_mm']:.0f} mm (wall frame waypoints, dead reckoning, {last}):")
            print_route(s.route)
        elif prev is not None:
            dx, dy, dth = relative_move(prev, s.ares)
            print(f"stop {s.index}{leg}: ARES translate dx {dx:+.1f} mm, dy {dy:+.1f} mm (body frame), "
                  f"rotate {math.degrees(dth):+.2f} deg")
        else:
            print(f"stop {s.index}{leg}: ARES at the start mark {s.ares.describe()} (wall frame)")
        if s.route_to_station:
            a, b = route_stats(s.route_to_station), route_stats(s.route_from_station)
            print(f"   station trip: {a['length_mm']:.0f} mm out ({len(s.route_to_station)} waypoints), "
                  f"{b['length_mm']:.0f} mm back ({len(s.route_from_station)} waypoints)")
        for lk in s.looks:
            q = "" if lk.qnear_rad is None else f", qnear {[round(math.degrees(v), 1) for v in lk.qnear_rad]} deg"
            print(f"   look {lk.name}: boards {lk.boards}{q}")
        for t in s.stones:
            if mag.empty():
                reloads += 1
                upcoming = [x.kind for x in stones[done:]]
                if mjob.reload_short(station, full_station, mag, upcoming):
                    station = full_station.copy()
                    refills += 1
                plan = mjob.reload_plan(mag, station, upcoming)
                for ssid, mid, kind in plan:
                    station.take(ssid)
                    mag.fill(mid, kind)
                kinds = {k: sum(1 for p in plan if p[2] == k) for k in ("full", "half")}
                print(f"   -- reload {reloads}: station -> magazine, {len(plan)} stones ({kinds['full']} full, "
                      f"{kinds['half']} half)")
            sid = mag.next_take(t.slot, kind=t.kind)
            mag.take(sid)
            done += 1
            print(f"   stone {t.label}: {t.kind:4s} u {t.u_mm:6.0f} mm, top {t.z_top_mm:5.0f} mm, slot {sid}"
                  + (" (flip)" if t.flip else ""))
        prev = s.ares
    n_half = sum(t.kind == "half" for t in stones)
    print(f"{job.n_stones} stones ({n_half} half), {len(job.stops)} stops, {reloads} reloads predicted, "
          f"{refills} station refills (magazine {job.magazine.capacity}, station {n_st} usable slots)")


# ── simulation ────────────────────────────────────────────────────────────────
def sim_once(cfg: dict, job: mjob.Job, args, camera_loop: bool, log_dir: Path | None) -> dict:
    from mauer.simworld import SimWorld, scenario
    world = SimWorld(cfg, job, scenario(args.scenario), seed=args.seed, start_stop=parse_stops(args.stops,
                     len(job.stops))[0], supersample=args.supersample, grasp_check=not args.lenient_grasp)
    seq = Sequencer(job, cfg, world.robot, world.ares, world.camera, world.intr, world.T_flange_cam,
                    log_dir=log_dir, camera_loop=camera_loop, save_images=args.save_images,
                    on_station_empty=world.refill_station)
    a, b = parse_stops(args.stops, len(job.stops))
    t0 = time.time()
    err = None
    try:
        seq.run(a, b)
    except SequencerError as e:
        err = f"{type(e).__name__}: {e}"
    finally:
        seq.close()
    out = world.summary()
    out.update({"camera_loop": camera_loop, "error": err, "state": seq.result.state, "wall_s": time.time() - t0,
                "corrections": seq.result.corrections, "reloads": seq.result.reloads,
                "ares_moves": seq.result.ares_moves, "log": seq.result.log_path,
                "station_estimate_obs": len(seq.station_obs)})
    return out


def _fmt(st: dict, key: str = "max") -> str:
    return "-" if not st or not st.get("n") else f"{st[key]:.2f}"


def print_table(rows: list[tuple[str, dict]]) -> None:
    head = (f"{'run':<26s} {'placed':>6s} {'seated':>6s} | {'horiz mean':>10s} {'p95':>6s} {'max':>6s} | "
            f"{'pin max':>7s} {'|dx| max':>8s} {'|dy| max':>8s} {'|dz| max':>8s} {'|yaw| max':>9s} | "
            f"{'moves':>5s} {'slips':>5s} {'corr':>4s} {'reload':>6s} {'shots':>5s}")
    print(head)
    print("-" * len(head))
    for name, r in rows:
        p = r["placement"]
        print(f"{name:<26s} {p['n']:6d} {p['seated']:6d} | {_fmt(p['horiz_mm'], 'mean'):>10s} "
              f"{_fmt(p['horiz_mm'], 'p95'):>6s} {_fmt(p['horiz_mm']):>6s} | {_fmt(p['pin_mm']):>7s} "
              f"{_fmt(p['dx_mm']):>8s} {_fmt(p['dy_mm']):>8s} {_fmt(p['dz_mm']):>8s} {_fmt(p['yaw_deg'], 'max'):>9s} | "
              f"{r['ares']['moves']:5d} {r['ares']['slips']:5d} {r['corrections']:4d} {r['reloads']:6d} "
              f"{r['robot']['shots']:5d}")
    for name, r in rows:
        bk = r["placement"].get("by_kind", {})
        if bk.get("half", {}).get("n"):
            print(f"{name}: full {bk['full']['seated']}/{bk['full']['n']} seated (pin max "
                  f"{bk['full']['pin_max_mm'] or 0:.2f} mm), half {bk['half']['seated']}/{bk['half']['n']} seated "
                  f"(pin max {bk['half']['pin_max_mm'] or 0:.2f} mm)")
    for name, r in rows:
        if r["error"]:
            print(f"{name}: stopped - {r['error']}")
        for v in r["violations"]:
            kind = "FLOOR CONTACT" if v.startswith("ARES footprint") else "INTERLOCK VIOLATION"
            print(f"{name}: {kind} - {v}")
    print("units: mm, deg; errors = stone pose at release vs nominal wall pose (wall frame axes: dx along, dy across)")


def run_sim(cfg: dict, job: mjob.Job, args) -> int:
    if args.preflight:
        problems = preflight_real(cfg, job, config_path=args.config)
        for p in problems:
            print(f"PREFLIGHT: {p}")
        if problems and not args.i_know:
            print("real-run preflight failed (simulation continues only with --i-know)")
            return 2
    modes = [True, False] if args.compare else [not args.no_camera]
    rows = []
    for loop in modes:
        log_dir = None
        if args.log_dir:
            log_dir = Path(args.log_dir) / ("camera" if loop else "dead_reckoning")
        r = sim_once(cfg, job, args, loop, log_dir)
        rows.append((f"{args.scenario} seed {args.seed} {'camera' if loop else 'dead reck.'}", r))
    print_table(rows)
    for name, r in rows:
        print(f"{name}: log {r['log']}, {r['wall_s']:.1f} s")
    if args.json_out:
        Path(args.json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_out).write_text(json.dumps({n: r for n, r in rows}, indent=1, default=str) + "\n")
    # with --compare the dead-reckoning run is the reference case: only the camera-loop run decides the exit code
    judged = [r for _, r in rows if r["camera_loop"] or not args.compare]
    return 0 if all(r["error"] is None and not r["violations"] for r in judged) else 1


# ── real ──────────────────────────────────────────────────────────────────────
def run_real(cfg: dict, job: mjob.Job, args) -> int:
    if args.i_know:
        print("--i-know is not accepted for --real: fix the listed problems in config/station.toml / calib/")
        return 2
    problems = preflight_real(cfg, job, config_path=args.config)
    if problems:
        print("real run refused - every problem:")
        for p in problems:
            print(f"  - {p}")
        return 2
    from mauer.ares import AresAds
    from mauer.backends import AdsAres, URRobot
    from mauer.camera import open_camera
    from mauer.ur.link import URLink
    from mauer.vision import handeye, intrinsics

    v = cfg["vision"]
    intr = intrinsics.load(mconfig.repo_path(v["intrinsics_file"]))
    X = handeye.load(mconfig.repo_path(v["handeye_file"])).T_flange_cam

    def confirm(desc: str) -> bool:
        return input(f"{desc} - go? [y/N] ").strip().lower() == "y"

    def station_empty() -> None:
        input("pick-up station empty: refill every slot, then press Enter ")

    link = URLink.from_config(cfg)
    ads = AresAds(cfg["ares_ads"])
    cam = None
    try:
        link.start()
        ads.connect()
        pf = ads.preflight()
        if pf:
            print("ARES not ready:\n  - " + "\n  - ".join(pf))
            return 2
        cam = open_camera(cfg)
        robot = URRobot(link, cfg, job)
        seq = Sequencer(job, cfg, robot, AdsAres(ads), cam, intr, X, log_dir=args.log_dir,
                        confirm=confirm if args.step else None, save_images=args.save_images,
                        on_station_empty=station_empty)
        a, b = parse_stops(args.stops, len(job.stops))
        try:
            seq.run(a, b)
        except SequencerError as e:
            print(f"STOPPED: {e}\nlog: {seq.result.log_path}")
            return 1
        finally:
            seq.close()
        print(f"done: {len(seq.result.placed)} stones, log {seq.result.log_path}")
        return 0
    finally:
        if cam is not None:
            cam.close()
        ads.close()
        link.stop()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("job", nargs="?", help="job JSON (tools/make_job.py or the RoboDK planner)")
    ap.add_argument("--nominal", default=None, metavar="LENGTH|L",
                    help="build a nominal job on the fly: a straight wall of LENGTH stones, or L = the config's L")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="pure-Python simulated world (mauer.simworld)")
    mode.add_argument("--real", action="store_true", help="real UR5 + ARES + IDS camera")
    mode.add_argument("--dry-run", action="store_true", help="print the plan only")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--stops", default=None, help="'k' (one stop), 'a:b' (a..b inclusive), 'a:' (a to the end)")
    ap.add_argument("--log-dir", default=None, help="run log folder (default data/runs/<timestamp>/)")
    ap.add_argument("--save-images", action="store_true", help="store every image in the run log folder")
    ap.add_argument("--step", action="store_true", help="--real: confirm every motion")
    ap.add_argument("--i-know", action="store_true", help="--sim --preflight only: continue despite problems")
    g = ap.add_argument_group("simulation")
    g.add_argument("--scenario", default="realistic", choices=("none", "e003", "slip", "realistic"),
                   help="injected errors (mauer.simworld.scenario)")
    g.add_argument("--seed", type=int, default=1)
    g.add_argument("--no-camera", action="store_true", help="dead reckoning only (no images, nominal frames)")
    g.add_argument("--compare", action="store_true", help="run with and without the camera loop")
    g.add_argument("--lenient-grasp", action="store_true", help="the simulated gripper never misses a stone")
    g.add_argument("--supersample", type=int, default=2, help="render supersampling (>= 2, see mauer.simworld)")
    g.add_argument("--preflight", action="store_true", help="also run the real-run preflight")
    g.add_argument("--json-out", default=None, help="write the summaries as JSON")
    g.add_argument("--quiet", action="store_true", help="no log warnings on the console")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.ERROR if args.quiet else logging.WARNING, format="%(levelname)s %(message)s")
    cfg = mconfig.load(args.config)
    try:
        job = load_job(args, cfg)
    except (ValueError, mjob.JobError) as e:
        print(f"FAILED to build/load the job: {e}", file=sys.stderr)
        return 1
    if args.dry_run:
        dry_run(job)
        return 0
    if args.sim:
        return run_sim(cfg, job, args)
    return run_real(cfg, job, args)


if __name__ == "__main__":
    raise SystemExit(main())
