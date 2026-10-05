"""Check the ADS link to ARES and, on request, drive one relative move.

  status   read only: TCP pre-check (port 48898), ADS connection, interface version, PLC build, AMR state, external
           control, HMI watchdog and heartbeat, move state, odometry pose, preflight verdict. Writes nothing.
  move     one relative move (--dx/--dy [mm] or --dtheta [deg], body frame at the start pose: +x forward, +y left,
           +theta CCW). Needs --yes and a passing preflight; prints the PLC outcome.

amr_hmi v2 must be open, connected and in MANUAL (operating pattern A): it supplies the heartbeat and HALT; this tool
writes only the move fields. Parameters: config/station.toml [ares_ads]. --fake runs against the simulated PLC of
tests/fake_plc.py (simulated time) for a dry run without ARES.

Examples (repo root, Windows Python):
  py.exe tools/ares_check.py status
  py.exe tools/ares_check.py move --dy 100 --yes
  py.exe tools/ares_check.py --fake move --dy 1400 --yes
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from mauer import config  # noqa: E402
from mauer.ares import AresAds, AresConnectionError, AresNotReady, AresStatus, MoveRefused, check_move  # noqa: E402
from mauer.ares.ads import ADS_TCP_PORT, DEFAULT_ROT_SPEED_DEGS, HB_WINDOW_S  # noqa: E402


def make_client(cfg: dict, fake: bool) -> AresAds:
    if not fake:
        return AresAds(cfg)
    sys.path.insert(0, str(REPO / "tests"))
    from fake_plc import FakePlc                          # test double, simulated time
    plc = FakePlc()
    plc.run(0.5)
    print("using the FAKE PLC (tests/fake_plc.py, simulated time) - nothing is sent to ARES", flush=True)
    return AresAds(cfg, connection_factory=plc.factory(), clock=plc.clock, sleep=plc.sleep)


def connect(ares: AresAds, fake: bool) -> bool:
    target = "fake PLC" if fake else f"{ares.ams_net_id}:{ares.port} (TCP {ares.host}:{ADS_TCP_PORT})"
    print(f"connecting to {target} ...", flush=True)
    try:
        ares.connect()
    except AresConnectionError as exc:
        print(f"FAILED: {exc}", flush=True)
        return False
    print("connected", flush=True)
    return True


def print_status(s: AresStatus, hb_delta: int | None) -> None:
    yes_no = {True: "yes", False: "no"}
    print(f"  interface      v{s.if_version}, build {s.build!r}")
    print(f"  AMR state      {s.amr_state} {s.amr_state_name} (PLC text {s.amr_state_text!r})"
          + ("" if s.amr_state == 7 else "  <- moves need 7 MANUAL MODE"))
    print(f"  ext control    {'ACTIVE (C6030 / ROS bridge)' if s.ext_active else 'inactive'}")
    hb = "?" if hb_delta is None else f"+{hb_delta} in {HB_WINDOW_S * 1000:.0f} ms"
    print(f"  HMI            watchdog {'ok' if s.hmi_watchdog_ok else 'EXPIRED'}, heartbeat {s.hmi_heartbeat} ({hb})")
    print(f"  safety         safety OK {yes_no[s.safety_ok]}, safety stop {yes_no[s.safety_stop_active]}, "
          f"drives enabled {yes_no[s.drives_enabled]}, fault {s.fault_text if s.fault_active else 'none'}")
    print(f"  move           {s.move_state_name}, result {s.move_result} ({s.move_result_text}), last id "
          f"{s.move_cmd_ack} verdict {s.move_cmd_result}, text {s.move_text!r}"
          + (", limited" if s.move_limited else ""))
    if s.move_target > 0.0:
        unit = "deg" if s.move_is_rotation else "mm"
        print(f"                 target {s.move_target:.1f} {unit}, done {s.move_done:.2f} {unit}, final error "
              f"{s.move_final_err:+.2f} {unit} (+ = short), {s.move_elapsed_s:.1f} s")
    print(f"  odometry       x = {s.x_mm:.1f} mm, y = {s.y_mm:.1f} mm, theta = {s.theta_deg:.2f} deg "
          f"(wheel odometry since reset, slip invisible), wheel path {s.odom_dist_mm / 1000.0:.3f} m")
    print(f"  bits           start {s.start_bit}, abort {s.abort_bit}, stop {s.stop_bit}, jog {s.jog_active}, "
          f"HMI accel {s.hmi_accel_mms2:.0f} mm/s2", flush=True)


def cmd_status(ares: AresAds) -> int:
    pf = ares.check()
    if pf.status is not None:
        print_status(pf.status, pf.heartbeat_delta)
    if pf.ok:
        print("preflight: OK - a relative move could start", flush=True)
        return 0
    print(f"preflight: {len(pf.problems)} problem(s):", flush=True)
    for p in pf.problems:
        print(f"  - {p}", flush=True)
    return 1


def plan_move(cfg: dict, args: argparse.Namespace) -> int | None:
    """Print the planned move and check it locally; exit code if the tool must stop here (no ADS traffic)."""
    parts = ([f"translation dx {args.dx:+g} mm, dy {args.dy:+g} mm"] if args.dx or args.dy or not args.dtheta else [])
    parts += [f"rotation {args.dtheta:+g} deg"] if args.dtheta else []
    print(f"move: {' + '.join(parts)} (body frame at the start pose: +x forward, +y left, +theta CCW)", flush=True)
    reason = check_move(args.dx, args.dy, args.dtheta,
                        cfg["speed_mms"] if args.speed is None else args.speed,
                        cfg.get("rot_speed_degs", DEFAULT_ROT_SPEED_DEGS) if args.rot_speed is None else args.rot_speed,
                        cfg["accel_mms2"] if args.accel is None else args.accel)
    if reason is not None:
        print(f"REFUSED (nothing sent): {reason}", flush=True)
        return 2
    if not args.yes:
        print("not moving: add --yes. ARES drives BY ITSELF - path clear, UR5 parked (the PLC has no arm interlock), "
              "E-stop at hand, amr_hmi open for HALT.", flush=True)
        return 2
    return None


def cmd_move(ares: AresAds, args: argparse.Namespace) -> int:
    last = {"state": None}

    def progress(s: AresStatus) -> None:
        if s.move_state != last["state"]:
            last["state"] = s.move_state
            unit = "deg" if s.move_is_rotation else "mm"
            print(f"  {s.move_state_name:<9} done {s.move_done:8.1f} {unit}  {s.move_text}", flush=True)
    try:
        o = ares.move(dx_mm=args.dx, dy_mm=args.dy, dtheta_deg=args.dtheta, speed_mms=args.speed,
                      rot_speed_degs=args.rot_speed, accel_mms2=args.accel, timeout_s=args.timeout,
                      progress=progress)
    except MoveRefused as exc:
        print(f"REFUSED (nothing sent): {exc}", flush=True)
        return 2
    except AresNotReady as exc:
        print(f"NOT READY (nothing sent): {exc}", flush=True)
        return 1
    except KeyboardInterrupt:
        print("Ctrl+C: abort sent (bCmdMoveAbort pulse) - check ARES and amr_hmi", flush=True)
        return 130
    print(o.summary(), flush=True)
    d = o.odom_delta()
    if d is not None:
        print(f"odometry delta in the start frame: dx {d[0]:+.1f} mm, dy {d[1]:+.1f} mm, dtheta {d[2]:+.3f} deg; "
              f"lateral {o.lat_err_mm:+.1f} mm, heading {o.head_err_deg:+.3f} deg (wheel odometry, slip invisible)",
              flush=True)
    return 0 if o.ok else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="station config (default config/station.toml)")
    ap.add_argument("--fake", action="store_true", help="dry run against tests/fake_plc.py (simulated time)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status", help="read-only connection and state check")
    mv = sub.add_parser("move", help="one relative move (needs --yes)")
    mv.add_argument("--dx", type=float, default=0.0, help="forward [mm]")
    mv.add_argument("--dy", type=float, default=0.0, help="left [mm]")
    mv.add_argument("--dtheta", type=float, default=0.0, help="rotation, + = CCW [deg] (not with --dx/--dy)")
    mv.add_argument("--speed", type=float, default=None, help="path speed [mm/s] (default [ares_ads] speed_mms)")
    mv.add_argument("--rot-speed", type=float, default=None, help="rotation rate [deg/s]")
    mv.add_argument("--accel", type=float, default=None, help="acceleration at the wheel [mm/s2]")
    mv.add_argument("--timeout", type=float, default=None,
                    help="client deadline [s] (default: PLC timeout + tAbortMax + margin)")
    mv.add_argument("--yes", action="store_true", help="really move ARES")
    args = ap.parse_args(argv)

    cfg = config.load(args.config)["ares_ads"]
    if args.cmd == "move":
        rc = plan_move(cfg, args)
        if rc is not None:
            return rc
    ares = make_client(cfg, args.fake)
    if not connect(ares, args.fake):
        return 2
    try:
        return cmd_status(ares) if args.cmd == "status" else cmd_move(ares, args)
    except AresConnectionError as exc:
        print(f"ADS ERROR: {exc}", flush=True)
        return 2
    finally:
        ares.close()


if __name__ == "__main__":
    sys.exit(main())
