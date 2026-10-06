"""UR5 CB3 link checks from the laptop (Dashboard 29999, RTDE 30004, URScript 30002) - day-1 checks on the robot.

    py.exe tools/ur_check.py info                     # versions, modes, RTDE fields, q, TCP, tool voltage (no motion)
    py.exe tools/ur_check.py ready                    # power on + brake release (the joints move slightly!)
    py.exe tools/ur_check.py tool-voltage 24          # set_tool_voltage(24) as a block, verified via RTDE
    py.exe tools/ur_check.py pose                     # T_base_tcp / T_base_flange in mm/deg and UR format
    py.exe tools/ur_check.py block my.script          # run URScript statements as a block with start/done markers
    py.exe tools/ur_check.py grip open|close          # gripper pulse on [ur] do_grip_*, the pulse read back via RTDE

Host: --host, else [ur].host from config/station.toml (currently an empty PLACEHOLDER). URSim: --host 127.0.0.1
(tests/ursim.py start). A block is its own `def` program: it stops any program running on the controller.
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from mauer import config, geometry as g  # noqa: E402
from mauer.ur import script  # noqa: E402
from mauer.ur.dashboard import Dashboard, DashboardError  # noqa: E402
from mauer.ur.link import (OPTIONAL_FIELDS, REQUIRED_FIELDS, ROBOT_MODE, RUNTIME_STATE, SAFETY_MODE,  # noqa: E402
                           URLink, probe_fields)


def fmt_T(T: np.ndarray) -> str:
    xyz, rpy = g.xyz_rpy(T)
    return ("xyz [mm] = (" + ", ".join(f"{v:9.3f}" for v in xyz) + ")  rpy [deg] = (" +
            ", ".join(f"{v:8.3f}" for v in rpy) + ")")


def ur_pose(T: np.ndarray) -> str:
    return g.ur_str(g.T_to_ur(T))


def open_link(a, cfg) -> URLink:
    return URLink.from_config(cfg, host=a.host).start()


def cmd_info(a, cfg) -> int:
    u = cfg["ur"]
    print(f"host {a.host}", flush=True)
    try:
        with Dashboard(a.host, u.get("dashboard_port", 29999), timeout_s=5.0) as d:
            print(f"dashboard: {d.banner}", flush=True)
            print(f"  PolyScope {d.polyscope_version()} | model {d.robot_model()} | serial {d.serial_number()}")
            print(f"  robotmode {d.robotmode()} | safetymode {d.safetymode()}", flush=True)
    except (OSError, DashboardError) as e:
        print(f"dashboard: FAILED ({e})", flush=True)
    regs = [f"output_int_register_{r}" for r in (u.get("reg_started", 24), u.get("reg_done", 25),
                                                 u.get("reg_error", script.REG_ERROR))]
    names = list(REQUIRED_FIELDS) + regs + list(OPTIONAL_FIELDS)
    avail = probe_fields(a.host, names, u.get("rtde_port", 30004))
    print("RTDE fields: " + ", ".join(f"{n} {'ok' if ok else 'MISSING'}" for n, ok in avail.items()), flush=True)
    with open_link(a, cfg) as ur:
        s = ur.state()
        print(f"RTDE: controller {ur.controller_version}, {len(ur.fields)} fields at {ur.frequency:.0f} Hz, "
              f"flange from {ur.tcp_source}", flush=True)
        print(f"  robot_mode {ROBOT_MODE.get(s.robot_mode)} | safety_mode {SAFETY_MODE.get(s.safety_mode)} | "
              f"runtime {RUNTIME_STATE.get(s.runtime_state)} | program running {s.program_running}")
        print("  q [deg] = (" + ", ".join(f"{v:8.3f}" for v in np.degrees(s.actual_q)) + ")")
        print(f"  TCP     {fmt_T(s.T_base_tcp_mm())}")
        if s.tcp_offset is not None:
            print(f"  tcp_offset {fmt_T(s.T_flange_tcp_mm())}")
        print(f"  tool output voltage {s.tool_output_voltage} V | digital outputs "
              f"{'n/a' if s.digital_outputs is None else format(s.digital_outputs & 0xFF, '08b')} (DO7..DO0)")
        print(f"  markers: started {s.reg_started}, done {s.reg_done}, error {s.reg_error}", flush=True)
    return 0


def cmd_ready(a, cfg) -> int:
    with Dashboard(a.host, cfg["ur"].get("dashboard_port", 29999), timeout_s=5.0) as d:
        print(f"robotmode {d.robotmode()}, safetymode {d.safetymode()} -> power on / brake release ...", flush=True)
        d.wait_ready(timeout_s=a.timeout)
        print(f"ready: robotmode {d.robotmode()}, safetymode {d.safetymode()}", flush=True)
    return 0


def cmd_tool_voltage(a, cfg) -> int:
    with open_link(a, cfg) as ur:
        before = ur.state().tool_output_voltage
        r = ur.run_block(script.tool_voltage(a.volt), f"tool_voltage_{a.volt}", timeout_s=10.0)
        if not r.ok:
            print(f"block failed: {r.error}", flush=True)
            return 1
        s = ur.wait_until(lambda x: x.tool_output_voltage == a.volt, 2.0)
        now = ur.state().tool_output_voltage
        print(f"tool output voltage {before} V -> {now} V ({'OK' if s is not None else 'NOT CONFIRMED'})", flush=True)
        if now is None:
            print("RTDE tool_output_voltage not available on this controller", flush=True)
        return 0 if s is not None else 1


def cmd_pose(a, cfg) -> int:
    with open_link(a, cfg) as ur:
        s = ur.state()
        T_tcp = s.T_base_tcp_mm()
        print("q [deg]       = (" + ", ".join(f"{v:8.3f}" for v in np.degrees(s.actual_q)) + ")")
        print(f"T_base_tcp    {fmt_T(T_tcp)}\n              {ur_pose(T_tcp)}")
        T_ft = ur.T_flange_tcp_current(s)
        if T_ft is None:
            print("T_base_flange: unknown (no RTDE tcp_offset and no set_tcp sent by this link)", flush=True)
            return 1
        T_fl = ur.flange_T(s)
        print(f"T_base_flange {fmt_T(T_fl)}\n              {ur_pose(T_fl)}")
        print(f"T_flange_tcp  {fmt_T(T_ft)}  (from {ur.tcp_source})", flush=True)
    return 0


def cmd_block(a, cfg) -> int:
    body = Path(a.file).read_text(encoding="ascii")
    with open_link(a, cfg) as ur:
        r = ur.run_block(body, a.name, timeout_s=a.timeout, settle_s=a.settle, interrupt=a.interrupt)
        if r.ok:
            print(f"block {r.name} #{r.block_id} done: start {r.start_latency_s:.3f} s, total {r.duration_s:.3f} s, "
                  f"error code {r.error_code}", flush=True)
            print(f"TCP {fmt_T(r.state.T_base_tcp_mm())}", flush=True)
            return 0
        print(f"block {r.name} #{r.block_id} FAILED: {r.error}\n--- program ---\n{r.program}", flush=True)
        return 1


def grip(ur, cfg, action: str) -> dict:
    """Gripper open/close as a block (script.gripper with [ur] do_grip_open/close, grip_pulse_s, grip_wait_s; no arm
    motion) and the output pulse as RTDE saw it: {"ok", "error", "do", "pulse_s" (None: never on), "other_on"}."""
    u = cfg["ur"]
    do_open, do_close = int(u["do_grip_open"]), int(u["do_grip_close"])
    this, other = (do_open, do_close) if action == "open" else (do_close, do_open)
    body = script.gripper(action, do_open, do_close, float(u["grip_pulse_s"]), float(u["grip_wait_s"]))
    t0 = time.time()
    r = ur.run_block(body, f"grip_{action}", timeout_s=10.0 + float(u["grip_pulse_s"]) + float(u["grip_wait_s"]))
    on = [x.t_laptop for x in ur.samples(t0) if x.digital_out(this)]
    return {"ok": r.ok, "error": r.error, "do": this, "pulse_s": (on[-1] - on[0]) if on else None,
            "other_on": any(x.digital_out(other) for x in ur.samples(t0 + 0.05))}


def cmd_grip(a, cfg) -> int:
    u = cfg["ur"]
    with open_link(a, cfg) as ur:
        res = grip(ur, cfg, a.action)
    if not res["ok"]:
        print(f"gripper {a.action} block FAILED: {res['error']}", flush=True)
        return 1
    seen = "never on (check the RTDE field actual_digital_output_bits)" if res["pulse_s"] is None else \
        f"on for {res['pulse_s']:.2f} s (commanded {float(u['grip_pulse_s']):g} s)"
    print(f"gripper {a.action}: DO{res['do']} {seen}; the other output {'ON' if res['other_on'] else 'off'}. "
          f"Did the jaws {a.action}?", flush=True)
    return 0 if res["pulse_s"] is not None and not res["other_on"] else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="robot IP (default: [ur].host)")
    ap.add_argument("--config", help="station.toml (default: config/station.toml)")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info", help="versions, modes, RTDE fields, q, TCP, tool voltage (no motion)")
    p = sub.add_parser("ready", help="power on + brake release (joints move slightly)")
    p.add_argument("--timeout", type=float, default=90.0, help="[s]")
    p = sub.add_parser("tool-voltage", help="set the tool connector voltage and verify it via RTDE")
    p.add_argument("volt", type=int, choices=[0, 12, 24])
    sub.add_parser("pose", help="current T_base_tcp / T_base_flange")
    p = sub.add_parser("block", help="run URScript statements (not indented, no def/end) as a block")
    p.add_argument("file")
    p.add_argument("--name", default="ur_check_block", help="program name (URScript identifier)")
    p.add_argument("--timeout", type=float, default=120.0, help="[s]")
    p.add_argument("--settle", type=float, default=0.0, help="extra wait after the done marker [s]")
    p.add_argument("--interrupt", action="store_true", help="stop a program that is already running")
    p = sub.add_parser("grip", help="gripper open/close pulse on [ur] do_grip_* (no arm motion), read back via RTDE")
    p.add_argument("action", choices=["open", "close"])
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.WARNING, format="%(name)s: %(message)s")
    if not a.verbose:
        logging.getLogger("rtde").setLevel(logging.CRITICAL)
    cfg = config.load(a.config)
    a.host = a.host or cfg["ur"].get("host", "")
    if not a.host:
        ap.error("[ur].host is an empty PLACEHOLDER in config/station.toml - pass --host <robot IP> "
                 "(URSim: --host 127.0.0.1)")
    return {"info": cmd_info, "ready": cmd_ready, "tool-voltage": cmd_tool_voltage, "pose": cmd_pose,
            "block": cmd_block, "grip": cmd_grip}[a.cmd](a, cfg)


if __name__ == "__main__":
    sys.exit(main())
