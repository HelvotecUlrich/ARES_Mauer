"""Report of an ARES tour (Samuel 2026-10-15: drive to every position of the job, look at the boards with the UR, no
stone) - or of any run log: how far ARES ended from where the sequencer predicted it after every move (the drive
error), how far from the nominal stop / dock, the frame fits and the corrections.

    py.exe tools/tour_report.py data/runs/<run folder> [--out <file.md>]

Writes <run folder>/tour_report.md (or --out) and prints it. Inputs: run.jsonl events "wall_frame" / "station_frame"
(source camera: measured, predicted, jump_mm / jump_deg = measured - predicted, err_nominal_mm / _deg, fit rms_mm,
tilt_deg), "ares_move" (kind, args, why, ok) and "run_start" (tour, laid). Dead-reckoning frames carry no measurement
and are counted only.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from mauer.sequencer import read_log  # noqa: E402


def _pose(p) -> str:
    if not p:
        return "-"
    return f"{p['x_mm']:.1f}, {p['y_mm']:.1f}, {math.degrees(p['theta_rad']):.2f}°"


def _stats(xs: list[float]) -> str:
    if not xs:
        return "-"
    return f"mean {sum(xs) / len(xs):.1f}, max {max(xs):.1f} (n {len(xs)})"


def report(run_dir: str | Path) -> str:
    run_dir = Path(run_dir)
    ev = read_log(run_dir / "run.jsonl")
    t0 = ev[0]["t"] if ev else 0.0
    start = next((e for e in ev if e["event"] == "run_start"), {})
    meas = [e for e in ev if e["event"] in ("wall_frame", "station_frame") and e.get("source") == "camera"]
    moves = [e for e in ev if e["event"] == "ares_move"]
    dead = sum(1 for e in ev if e["event"] in ("wall_frame", "station_frame") and e.get("source") != "camera")
    end = next((e for e in reversed(ev) if e["event"] in ("run_done", "run_paused", "run_error", "run_aborted")), None)
    out = [f"# ARES tour report: {run_dir.name}", ""]
    laid = start.get("laid") or {}
    out += [f"- tour (no stones): {bool(start.get('tour'))}; camera loop: {start.get('camera_loop')}; "
            f"stops {start.get('start_stop')}..{start.get('last_stop')}; as-laid gap {laid.get('piece_gap_mm', 0)} mm",
            f"- end: {end['event'] if end else 'no end record'}"
            + (f" ({end.get('error')})" if end and end.get("error") else "")
            + f", {(ev[-1]['t'] - t0) / 60.0:.1f} min, {len(moves)} ARES moves, {len(meas)} camera measurements"
            + (f", {dead} dead-reckoned frames" if dead else ""), ""]
    out += ["## Camera measurements", "",
            "jump = measured − predicted ARES pose (the drive error of the moves since the last measurement); "
            "nominal = measured − nominal stop / dock.", "",
            "| # | min | where | why | try | jump mm | jump ° | nominal mm | nominal ° | fit RMS mm | tilt ° | measured "
            "(x, y, θ) | out of reach |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, e in enumerate(meas, 1):
        where = f"stop {e.get('stop')}" + (" station" if e["event"] == "station_frame" else "")
        tilt = "-" if e.get("tilt_deg") is None else f"{e['tilt_deg']:.2f}"
        out.append(f"| {i} | {(e['t'] - t0) / 60.0:.1f} | {where} | {e.get('why', 'dock')} | {e.get('attempt', 0)} "
                   f"| {e.get('jump_mm', 0.0):.1f} | {e.get('jump_deg', 0.0):.2f} | {e.get('err_nominal_mm', 0.0):.1f} "
                   f"| {e.get('err_nominal_deg', 0.0):.2f} | {e.get('rms_mm', 0.0):.2f} | {tilt} "
                   f"| {_pose(e.get('measured'))} | {', '.join(e.get('unreachable') or []) or '-'} |")
    groups: dict[str, list] = {}
    for e in meas:
        if e["event"] == "station_frame":
            key = "at the station (after the route there)" if e.get("attempt", 0) == 0 else "station after a dock correction"
        elif e.get("attempt", 0) > 0:
            key = "wall after a correction move"
        else:
            key = {"after route": "wall after a route (leg change / stop move)",
                   "after reload": "wall after the return from the station",
                   "arrival": "wall at the start / a direct stop move"}.get(e.get("why"), f"wall ({e.get('why')})")
        groups.setdefault(key, []).append(e)
    out += ["", "## Drive error by situation", "",
            "A route (leg change, return from the station) ends the job's [sequencer] arrival_standoff_mm before its "
            "stop on purpose (40 mm until 2026-10-16, 0 since): its 'nominal' includes that standoff; the correction "
            "moves close it.", "",
            "| situation | jump mm | jump ° | nominal mm after |", "|---|---|---|---|"]
    for key, es in groups.items():
        out.append(f"| {key} | {_stats([e.get('jump_mm', 0.0) for e in es])} "
                   f"| {_stats([e.get('jump_deg', 0.0) for e in es])} "
                   f"| {_stats([e.get('err_nominal_mm', 0.0) for e in es])} |")
    out += ["", "## ARES moves", "",
            "odometry = the PLC's wheel odometry of the move (dx, dy mm, dθ °, body frame; logged since 2026-10-18), "
            "PLC lat / head = its lateral / heading error of a translation (PLC frame).", "",
            "| # | min | why | move | odometry | PLC lat mm / head ° | ok |", "|---|---|---|---|---|---|---|"]
    for e in moves:
        args = ", ".join(f"{a:+.1f}" for a in e.get("args", []))
        od = ", ".join(f"{a:+.1f}" if i < 2 else f"{a:+.2f}" for i, a in enumerate(e["odom"])) if e.get("odom") else "-"
        lh = (f"{e['lat_err_mm']:+.1f} / {e.get('head_err_deg', 0.0):+.2f}"
              if e["kind"] == "translate" and "lat_err_mm" in e else "-")
        out.append(f"| {e.get('n')} | {(e['t'] - t0) / 60.0:.1f} | {e.get('why', '')} | {e['kind']} ({args}) "
                   f"| {od} | {lh} | {'ok' if e.get('ok') else 'NOT ok: ' + str(e.get('summary', ''))} |")
    warns = [e["msg"] for e in ev if e["event"] == "warning"]
    if warns:
        out += ["", "## Warnings", ""] + [f"- {w}" for w in warns]
    return "\n".join(out) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir", help="run log folder (data/runs/<...>) with run.jsonl")
    ap.add_argument("--out", default=None, help="report file (default <run_dir>/tour_report.md)")
    a = ap.parse_args(argv)
    text = report(a.run_dir)
    out = Path(a.out) if a.out else Path(a.run_dir) / "tour_report.md"
    out.write_text(text, encoding="utf-8", newline="\n")
    sys.stdout.reconfigure(encoding="utf-8")
    print(text)
    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
