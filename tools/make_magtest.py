"""Magazine dry run on ARES as a job file (mauer/magtest.py, [magtest] in config/station.toml, docs/MAGTEST_DE.md).

    py.exe tools/make_magtest.py                  # -> data/jobs/magtest.json; then: hmi --job data\\jobs\\magtest.json
    py.exe tools/make_magtest.py --rounds 1       # there and back once (20 moves)
    py.exe tools/make_magtest.py --print          # plan only, write nothing

Built from the config alone (no reach table, no RoboDK, no camera): the magazine slots as tools/make_job.py builds
them (every position a full-stone holder, [magtest] layers), the walk of the two stones over [magtest] positions (the
upper stone of the stack to the next position, the lower one on top of it; there and back per round), and one front
pose per move on the centreline of the front leg ([[wall.legs]] side "front", its dist) at [magtest] front_u_mm x
front_courses (place height = the course top as robodk/wallplan.py), inside out and cycled over the moves. Every
front pose gets the IK and grasp yaw of make_job.holder_pose at its HOVER pose (hover_mm above the place pose, where
the dry place goes) and the approach above it; a pose without IK or with the tool closer than
mauer.armcheck.SELF_CLEARANCE_MM to the arm is dropped (listed). The motion guard checks every joint move again at
run time (URRobot on the robot, SimRobot in the HMI SIM run).
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from mauer import armcheck  # noqa: E402
from mauer import config as mconfig  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer import job as mjob  # noqa: E402
from mauer.config import STATION_TOML  # noqa: E402
from mauer.magtest import KIND  # noqa: E402
from mauer.reference import Pose2D  # noqa: E402

DEPENDS_PATTERNS = ("[ur5] mount_*", "[ares] deck_top_z", "[ares] controller_*", "[tool] tcp_z", "[tool] adapter_z", "[brick] height",
                    "[brick] bed_joint", "[brick] mass_kg", "[wall] base_z", "[deck] holder_z",
                    "[deck] magazine_rows_dx", "[deck] magazine_y", "[study] approach", "[ur] park_q_deg",
                    "[ur] payload_*", "[magtest] *")


def _make_job_tool():
    spec = importlib.util.spec_from_file_location("magtest_make_job", REPO / "tools" / "make_job.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def walk(positions: list[str], rounds: int) -> list[tuple[str, str]]:
    """(from, to) slot ids: the stack of two on positions[0]; per step the upper stone to layer 1 of the next
    position, the lower one onto it (layer 2); there and back per round."""
    path = list(positions) + list(positions)[-2::-1]
    out = []
    for _ in range(rounds):
        for a, b in zip(path[:-1], path[1:]):
            out += [(f"{a}l2", f"{b}l1"), (f"{a}l1", f"{b}l2")]
    return out


def build(cfg: dict, rounds: int | None = None, config_path: Path | None = None) -> mjob.Job:
    mt = cfg["magtest"]
    if int(mt["stones"]) != 2 or int(mt["layers"]) != 2:
        raise ValueError("[magtest] stones / layers: the walk is planned for 2 stones in layers 1 + 2")
    rounds = int(mt["rounds"] if rounds is None else rounds)
    if rounds < 1:
        raise ValueError(f"rounds must be >= 1, got {rounds}")
    fronts = [lg for lg in cfg["wall"].get("legs", []) if lg.get("side") == "front"]
    if len(fronts) != 1:
        raise ValueError(f"[[wall.legs]]: need exactly one leg with side 'front', got {len(fronts)}")
    front = fronts[0]
    dist, hover = float(front["dist"]), float(mt["hover_mm"])
    mk = _make_job_tool()
    c = copy.deepcopy(cfg)
    c["deck"]["half_positions"] = []                 # the test moves full stones only (every holder takes one)
    c["deck"]["magazine_layers"] = int(mt["layers"])
    ctx = mk._Ctx(c, dist, 1, mk.LOOK_MARGIN_MM)
    mag = ctx.magazine()
    positions = [str(p) for p in mt["positions"]]
    missing = [f"{p}l{lay}" for p in positions for lay in (1, 2) if f"{p}l{lay}" not in mag.take_order]
    if missing:
        raise ValueError(f"magazine slots without a usable pick (IK / tool clearance): {missing}; {ctx.warnings}")
    for sl in mag.slots:
        sl.kind = "full"                             # typed: the test moves full stones only
    start = [f"{positions[0]}l1", f"{positions[0]}l2"]
    mag.initial_fill, mag.initial_kinds = list(start), {s: "full" for s in start}

    b, w = cfg["brick"], cfg["wall"]
    targets, dropped = [], []
    for u in mt["front_u_mm"]:
        for k in mt["front_courses"]:
            z_top = float(w["base_z"]) + int(k) * (float(b["height"]) + float(b["bed_joint"])) + float(b["height"])
            T0 = g.transl(dist, float(u), z_top) @ g.rotz(math.pi / 2) @ g.rotx(math.pi)
            T_hover, q, ok, clear = mk.holder_pose(ctx, ctx.T_base_ares, g.transl(0.0, 0.0, hover) @ T0)
            if not ok or clear < armcheck.SELF_CLEARANCE_MM:
                dropped.append({"u_mm": float(u), "course": int(k), "ik": bool(ok), "clearance_mm": float(clear)})
                continue
            targets.append((float(u), int(k), z_top, g.transl(0.0, 0.0, -hover) @ T_hover, q))
    if not targets:
        raise ValueError(f"no front pose with IK and tool clearance (dropped {dropped})")

    stones, moves = [], []
    for i, (src, dst) in enumerate(walk(positions, rounds)):
        u, k, z_top, T, q = targets[i % len(targets)]
        stones.append(mjob.StoneTask(k, i, u, z_top, T, False, src, q.tolist(), [], None, "full", None))
        moves.append({"key": [k, i], "from": src, "to": dst})

    station = mjob.Station(np.eye(4), Pose2D(0.0, 0.0, 0.0), [], [], [], [])
    cfg_path = Path(config_path or STATION_TOML)
    variant = mconfig.variant_of(cfg)
    meta = {"kind": KIND, "shape": KIND, "warnings": list(ctx.warnings),
            KIND: {"moves": moves, "hover_mm": hover, "dwell_s": float(mt["dwell_s"]), "positions": positions,
                   "start_fill": start, "rounds": rounds, "front": {"leg": front.get("name"), "dist_mm": dist},
                   "targets": [[u, k] for u, k, *_ in targets], "dropped_targets": dropped,
                   "config": dict(mt)}}
    if variant:
        meta["config_variant"] = variant
    job = mjob.Job([mjob.Stop(0, 0.0, Pose2D(0.0, 0.0, 0.0), [], stones)], mag, station, ctx.T_flange_tcp,
                   ctx.T_ares_base, ctx.q_park.tolist(), ctx.approach, mjob.config_sha256(cfg_path, variant),
                   mjob.depends_on(cfg, (*DEPENDS_PATTERNS, f"[[wall.legs]] {front.get('name')}.dist"), cfg_path),
                   "tools/make_magtest.py (config only: no reach table, no RoboDK)", meta=meta)
    problems = mjob.validate(job)
    if problems:
        raise mjob.JobError(problems)
    return job


def summary(job: mjob.Job) -> str:
    m = job.meta[KIND]
    lines = [f"magazine dry run: {len(m['moves'])} moves ({m['rounds']} round(s) there and back over "
             f"{' '.join(m['positions'])}), start: 2 full stones stacked on {m['start_fill'][0]} + "
             f"{m['start_fill'][1]}",
             f"front leg {m['front']['leg']} at {m['front']['dist_mm']:g} mm: {len(m['targets'])} poses (u, course) "
             f"{', '.join(f'({u:+.0f}, {k})' for u, k in m['targets'])}; hover {m['hover_mm']:g} mm, hold "
             f"{m['dwell_s']:g} s, jaws stay closed"]
    if m["dropped_targets"]:
        lines.append("dropped front poses (no IK / tool too close to the arm): "
                     + ", ".join(f"({d['u_mm']:+.0f}, {d['course']})" for d in m["dropped_targets"]))
    lines += [f"warning: {x}" for x in job.meta.get("warnings", [])]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rounds", type=int, default=None, help="there-and-back rounds (default [magtest] rounds)")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--variant", default=None, help="config variant (config/variants/<VARIANT>.toml)")
    ap.add_argument("--out", default=None, help="output JSON (default data/jobs/magtest[_<variant>].json)")
    ap.add_argument("--print", action="store_true", help="print the plan only, write nothing")
    args = ap.parse_args(argv)
    cfg = mconfig.load(args.config, args.variant)
    try:
        job = build(cfg, args.rounds, Path(args.config) if args.config else None)
    except (ValueError, KeyError, mjob.JobError) as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1
    print(summary(job))
    if args.print:
        return 0
    out = Path(args.out) if args.out else REPO / "data" / "jobs" / f"magtest{mconfig.suffix(cfg)}.json"
    mjob.save(job, out)
    print(f"written {out}")
    print("depends on " + (", ".join(f"{d['key']} ({d['status']})" for d in job.depends_on) or "CONFIRMED values only"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
