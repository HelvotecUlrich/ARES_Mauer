"""Build a nominal job (mauer.job v1) without RoboDK: config + robodk/wallplan.py + results/reach_table.json.

    py.exe tools/make_job.py [--length 24] [--dist 740] [--out data/jobs/nominal_L24.json] [--max-looks 2]

What `build_nominal` does (pure Python; the RoboDK planner will later export the same format with collision-checked
joints and via points):
- wall plan exactly as robodk/simulate.py:238-256 (wallplan.layout / sequence / check_plan on the cached reach table,
  same cache key as simulate.reach_table, simulate.py:41-58) -> stops a_j and their stones;
- nominal ARES pose per stop in the wall frame: inv(wall_frame(dist) @ transl(-a, 0, 0)) (simulate.py:269,
  rdk_common.wall_frame);
- place poses: transl(u, 0, z_top) @ rotx(pi) (rdk_common.place_pose, simulate.py:302), no flip;
- look poses: the camera fronto-parallel above a board centre at [camera] working_dist with T_flange_cam_nominal
  ([camera.mount] PLACEHOLDER), flange orientation as for a place pose, rolled about the optical axis in 15 deg steps
  until the nominal UR5 kinematics (mauer.simworld.ik_near, robodk/motion.py family) has a solution; per stop the
  pair of reachable wall boards with the longest baseline (heading). Kinematic check only - no collisions, no
  occlusion (robodk/look_study.py does those) -> meta.reach_check = "kinematic";
- magazine: the slots of robodk/simulate.py MAG_ROWS / MAG_Y (simulate.py:32-38: rows at UR x - 653.6 / - 433.6 mm,
  y -205 / 0 / 205 mm, 3 layers; pick pose transl(x, y, deck + layer H) @ rotz(pi/2) @ rotx(pi), simulate.py:106-108,
  deck = [ares] deck_top_z + [deck] holder_z) unless [deck] magazine_* keys exist; emptied top layer first
  (simulate.py:132-149), filled bottom first; slots without a kinematic IK solution (pick or approach pose) dropped;
- pick-up station from [pickup_station]: station frame in the wall frame, docking pose, slot grid (stone length along
  the station x axis, TCP = stone top at table_z + [pickup_station] holder_z (default 0) + stone height), look poses
  for the station boards at the nominal dock; unreachable slots dropped (warning);
- park pose: [ur] park_q_deg if present, else the IK of the compact pose robodk/simulate.py:280 starts from (TCP
  300 mm ahead of the UR base at the transfer height over a full magazine, simulate.py:114-117, tool down, seed
  [0, -100, 52, -42, -90, 0] deg) - not collision-checked here;
- provenance: config sha256 and every PLACEHOLDER/ASSUMPTION/UNKNOWN config key it depends on.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
from itertools import combinations
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from mauer import config as mconfig  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer import job as mjob  # noqa: E402
from mauer.config import STATION_TOML  # noqa: E402
from mauer.reference import Pose2D, board_centre, placements  # noqa: E402
from mauer.simworld import ik_near  # noqa: E402
from mauer.vision.targets import board_specs  # noqa: E402

REACH_TABLE = REPO / "results" / "reach_table.json"
# robodk/simulate.py:36-37 (copied - simulate.py imports RoboDK): magazine rows (x relative to the UR base, layers)
# and stone positions along ARES y. Requested as [deck] magazine_rows_dx / magazine_layers / magazine_y.
SIM_MAG_ROWS = ((-653.6, 3), (-433.6, 3))
SIM_MAG_Y = (-205.0, 0.0, 205.0)
SIM_START_SEED_DEG = (0.0, -100.0, 52.0, -42.0, -90.0, 0.0)   # robodk/simulate.py:280
PARK_RADIUS_MM = 300.0              # robodk/motion.py:115 compact(): first radius tried
TRANSFER_CLEARANCE_MM = 60.0        # robodk/simulate.py:117 safe_z(): 60 mm over the stack incl. pins
ROLL_STEP_DEG = 15.0                # look-pose roll search step about the optical axis
LOOK_MARGIN_MM = 50.0               # ASSUMPTION: looks stay reachable for ARES errors up to the sequencer's jump limit

DEPENDS_PATTERNS = ("[ur5] mount_*", "[tool] tcp_z", "[brick] *", "[wall] *", "[deck] *", "[camera] working_dist",
                    "[camera] focal", "[camera] pixel_um", "[camera] res_*", "[camera.mount] *",
                    "[boards.ref] square_mm", "[boards.ref] marker_mm", "[[targets]] *.xyz", "[[targets]] *.rpy_deg",
                    "[pickup_station] *", "[study] approach", "[ur] park_q_deg")


# ── helpers ───────────────────────────────────────────────────────────────────
def load_wallplan():
    """robodk/wallplan.py (pure Python) imported by path - robodk/ is not a package and its other modules need
    RoboDK."""
    name = "mauer_wallplan"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, REPO / "robodk" / "wallplan.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                       # dataclasses resolve the module while the class is created
    spec.loader.exec_module(mod)
    return mod


def wall_frame_in_ares(cfg: dict, dist: float) -> np.ndarray:
    """Wall frame in the ARES frame for a wall centreline `dist` from the ARES centre - numpy copy of
    robodk/rdk_common.py:122-133 wall_frame (that module imports RoboDK)."""
    side = cfg["wall"]["side"]
    if side == "right":
        return g.transl(0, -dist, 0)
    if side == "left":
        return g.transl(0, dist, 0) @ g.rotz(math.pi)
    if side == "front":
        return g.transl(dist, 0, 0) @ g.rotz(math.pi / 2)
    if side == "rear":
        return g.transl(-dist, 0, 0) @ g.rotz(-math.pi / 2)
    raise ValueError(f"wall.side must be right/left/front/rear, not {side!r}")


def reach_key(cfg: dict, dist: float) -> str:
    """Cache key of robodk/simulate.py:43-45 reach_table (and look_study.reach_table_cached)."""
    key_src = json.dumps(["family-v2", cfg["ur5"], cfg["tool"], cfg["brick"], cfg["wall"]["base_z"],
                          cfg["wall"]["courses"], cfg["study"]["approach"], dist], sort_keys=True)
    return hashlib.sha1(key_src.encode()).hexdigest()[:12]


def load_reach_table(cfg: dict, dist: float, path: Path | None = None, allow_stale: bool = False) -> tuple[dict, str]:
    """{course: {u_rel: ok}} from results/reach_table.json; ValueError if missing or computed for another config
    (unless allow_stale)."""
    path = Path(path or REACH_TABLE)
    if not path.exists():
        raise ValueError(f"{path} missing - run py.exe robodk/simulate.py --plan-only once (RoboDK)")
    data = json.loads(path.read_text())
    key = reach_key(cfg, dist)
    if data.get("key") != key and not allow_stale:
        raise ValueError(f"{path} was computed for another configuration (key {data.get('key')} != {key}, dist "
                         f"{data.get('dist')}) - rerun py.exe robodk/simulate.py --plan-only --dist {dist:g}")
    return {int(k): {float(u): v for u, v in d.items()} for k, d in data["table"].items()}, str(data.get("key"))


def make_plan(cfg: dict, length: int, table: dict) -> tuple[list, list]:
    """(stones, plan) exactly as robodk/simulate.py:240-252."""
    wp = load_wallplan()

    def reach(k: int, u: float) -> bool:
        return table[k].get(round(round(u / 20.0) * 20.0, 3), False)

    lo = {k: min(u for u, ok in table[k].items() if ok) for k in table}
    stones = wp.layout(cfg, length)
    a0 = stones[0].u - lo[0]
    plan = wp.sequence(cfg, stones, reach, lo, a0)
    errors = wp.check_plan(cfg, stones, plan)
    if errors:
        raise ValueError("invalid plan: " + "; ".join(errors))
    return stones, plan


def preferred_flange_R(R_base_parent: np.ndarray, T_flange_tcp: np.ndarray) -> np.ndarray:
    """Flange orientation of a place/pick pose in a z-up parent frame: TCP = parent rotx(pi) (tool down, TCP x along
    the parent x)."""
    return R_base_parent @ g.rotx(math.pi)[:3, :3] @ g.inv(T_flange_tcp)[:3, :3]


def look_pose(T_base_board: np.ndarray, spec, T_flange_cam: np.ndarray, dist_mm: float, R_flange_pref: np.ndarray,
              roll_deg: float) -> np.ndarray:
    """T_base_flange of a camera fronto-parallel above the board centre at dist_mm (optical axis = board z), image x
    as close as possible to the camera x of the preferred flange orientation, then rolled by roll_deg."""
    c = g.apply(T_base_board, [[*spec.centre_mm, 0.0]])[0]
    z = T_base_board[:3, 2] / np.linalg.norm(T_base_board[:3, 2])
    x0 = (R_flange_pref @ T_flange_cam[:3, :3])[:, 0]
    x = x0 - (x0 @ z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.cross(z, [0.0, 1.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z]) @ g.rotz(math.radians(roll_deg))[:3, :3]
    T_base_cam = g.make_T(R, c - dist_mm * z)
    return T_base_cam @ g.inv(T_flange_cam)


def rolls(step: float = ROLL_STEP_DEG) -> list[float]:
    out = [0.0]
    k = 1
    while k * step <= 180.0 + 1e-9:
        out += [k * step, -k * step] if k * step < 180.0 - 1e-9 else [180.0]
        k += 1
    return out


def find_look(T_base_board, spec, T_flange_cam, dist_mm, R_pref, q_ref, T_parent_base: np.ndarray | None = None,
              margin_mm: float = 0.0) -> tuple[np.ndarray, np.ndarray, float] | None:
    """(T_base_flange, qnear, roll) of the first roll with a family IK solution, None if none. With margin_mm > 0 the
    look must also stay reachable when the sequencer re-aims it for ARES displaced by +-margin_mm along the parent
    x and y axes (T_parent_base = nominal UR base in the parent frame)."""
    for r in rolls():
        T = look_pose(T_base_board, spec, T_flange_cam, dist_mm, R_pref, r)
        q = ik_near(T, q_ref)
        if q is None:
            continue
        if margin_mm > 0 and T_parent_base is not None:
            shifted = [g.inv(g.transl(dx, dy, 0.0) @ T_parent_base) @ T_parent_base @ T
                       for dx, dy in ((margin_mm, 0), (-margin_mm, 0), (0, margin_mm), (0, -margin_mm))]
            if any(ik_near(Ts, q) is None for Ts in shifted):
                continue
        return T, q, r
    return None


def choose_boards(cands: dict[str, tuple], centres: dict[str, np.ndarray], n: int) -> list[str]:
    """Up to n reachable boards: the pair with the longest baseline first, then by distance from that pair."""
    names = sorted(cands)
    if len(names) <= 1 or n <= 1:
        return names[:max(n, 0)] if n > 1 else names[:1]
    a, b = max(combinations(names, 2), key=lambda p: float(np.linalg.norm(centres[p[0]] - centres[p[1]])))
    out = [a, b]
    rest = sorted((x for x in names if x not in out),
                  key=lambda x: -min(float(np.linalg.norm(centres[x] - centres[y])) for y in out))
    return (out + rest)[:n]


def park_q(cfg: dict, T_ares_base: np.ndarray, T_flange_tcp: np.ndarray, layers: int) -> tuple[np.ndarray, str]:
    """[ur] park_q_deg, else the IK of simulate.py's compact start pose (see module docstring)."""
    u = cfg.get("ur", {})
    if "park_q_deg" in u:
        return np.radians(np.asarray(u["park_q_deg"], float)), "[ur] park_q_deg"
    b = cfg["brick"]
    deck = cfg["ares"]["deck_top_z"] + cfg["deck"]["holder_z"]
    z_safe = deck + layers * b["height"] + b["height"] + b["pin_length"] + TRANSFER_CLEARANCE_MM
    seed = np.radians(SIM_START_SEED_DEG)
    from mauer.simworld import ur5_fk
    R = (ur5_fk(seed) @ T_flange_tcp)[:3, :3]
    yaw = math.atan2(R[1, 0], R[0, 0])
    T_ares_tcp = g.transl(T_ares_base[0, 3] + PARK_RADIUS_MM, T_ares_base[1, 3], z_safe) @ g.rotz(yaw) @ g.rotx(math.pi)
    T_bf = g.inv(T_ares_base) @ T_ares_tcp @ g.inv(T_flange_tcp)
    q = ik_near(T_bf, seed)
    if q is None:
        raise ValueError("no IK solution for the park pose - set [ur] park_q_deg")
    return q, (f"IK of robodk/simulate.py compact start pose (TCP {PARK_RADIUS_MM:g} mm ahead of the UR base at "
               f"z {z_safe:.1f} mm ARES, tool down, seed {list(SIM_START_SEED_DEG)} deg); not collision-checked")


# ── build ─────────────────────────────────────────────────────────────────────
def build_nominal(cfg: dict, length: int = 24, dist: float | None = None, *, reach_table_path: Path | None = None,
                  allow_stale_reach: bool = False, max_looks: int = 2, config_path: Path | None = None,
                  look_margin_mm: float = LOOK_MARGIN_MM) -> mjob.Job:
    """Nominal job for a wall of `length` stones in course 0 (see the module docstring)."""
    dist = float(cfg["wall"]["dist_nominal"] if dist is None else dist)
    warnings: list[str] = []
    table, key = load_reach_table(cfg, dist, reach_table_path, allow_stale_reach)
    stones, plan = make_plan(cfg, length, table)
    T_ares_base = mconfig.T_ares_base(cfg)
    T_flange_tcp = mconfig.T_flange_tcp(cfg)
    T_flange_cam = mconfig.T_flange_cam_nominal(cfg)
    T_base_ares = g.inv(T_ares_base)
    specs = board_specs(cfg)
    pl = placements(cfg)
    work = float(cfg["camera"]["working_dist"])
    b = cfg["brick"]
    H = float(b["height"])
    approach = float(cfg["study"]["approach"])
    deck_cfg = cfg["deck"]
    rows_dx = deck_cfg.get("magazine_rows_dx", [r[0] for r in SIM_MAG_ROWS])
    n_layers = int(deck_cfg.get("magazine_layers", max(n for _, n in SIM_MAG_ROWS)))
    mag_y = deck_cfg.get("magazine_y", list(SIM_MAG_Y))
    q_park, park_src = park_q(cfg, T_ares_base, T_flange_tcp, n_layers)
    W_ares = wall_frame_in_ares(cfg, dist)

    # stops
    stops: list[mjob.Stop] = []
    for k, (a, batch) in enumerate(plan):
        T_ares_wall = W_ares @ g.transl(-a, 0.0, 0.0)
        ares = Pose2D.from_T(g.inv(T_ares_wall))
        T_base_wall = T_base_ares @ T_ares_wall
        R_pref = preferred_flange_R(T_base_wall[:3, :3], T_flange_tcp)
        cands, centres = {}, {}
        for p in pl:
            if p.parent != "wall":
                continue
            r = find_look(T_base_wall @ p.T_parent_board, specs[p.name], T_flange_cam, work, R_pref, q_park,
                          g.inv(T_base_wall), look_margin_mm)
            if r is not None:
                cands[p.name] = r
                centres[p.name] = board_centre(specs[p.name], p.T_parent_board)
        chosen = choose_boards(cands, centres, max_looks)
        if not chosen:
            raise ValueError(f"stop {k} (a = {a:.0f} mm): no wall board has a look pose with an IK solution")
        if len(chosen) < 2:
            warnings.append(f"stop {k}: only board {chosen} reachable for a fronto-parallel look - heading from one "
                            "board only")
        looks = [mjob.Look(f"stop{k}-{n}", [n], cands[n][0], None, cands[n][1].tolist()) for n in chosen]
        tasks = []
        for s in batch:
            T_wt = g.transl(s.u, 0.0, s.z_top) @ g.rotx(math.pi)
            q = ik_near(T_base_wall @ T_wt @ g.inv(T_flange_tcp), q_park)
            flip = False
            if q is None:
                T_f = T_wt @ g.rotz(math.pi)
                q = ik_near(T_base_wall @ T_f @ g.inv(T_flange_tcp), q_park)
                if q is not None:
                    T_wt, flip = T_f, True
                else:
                    warnings.append(f"stop {k} stone {s.key}: no kinematic IK solution for the place pose")
            tasks.append(mjob.StoneTask(s.course, s.index, float(s.u), float(s.z_top), T_wt, flip, None,
                                        None if q is None else q.tolist()))
        stops.append(mjob.Stop(k, float(a), ares, looks, tasks))

    # magazine
    deck = cfg["ares"]["deck_top_z"] + deck_cfg["holder_z"]
    x0 = T_ares_base[0, 3]
    slots: list[mjob.MagazineSlot] = []
    for ri, dx in enumerate(rows_dx):
        for yi, y in enumerate(mag_y):
            for lay in range(1, n_layers + 1):
                T = g.transl(x0 + dx, y, deck + lay * H) @ g.rotz(math.pi / 2) @ g.rotx(math.pi)
                T_bf = T_base_ares @ T @ g.inv(T_flange_tcp)
                q = ik_near(T_bf, q_park)
                ok = q is not None and ik_near(g.transl(0, 0, approach) @ T_bf, q_park) is not None
                slots.append(mjob.MagazineSlot(f"r{ri}y{yi}l{lay}", T, lay, f"r{ri}y{yi}",
                                               None if q is None else q.tolist(), bool(ok)))
    usable = [s for s in slots if s.ik_ok]
    dropped = [s.id for s in slots if not s.ik_ok]
    if dropped:
        warnings.append(f"magazine slots without a kinematic IK solution (dropped): {dropped}")
    # a slot above an unusable one cannot be filled -> drop whole stacks above a gap
    for s in list(usable):
        below = [o for o in slots if o.stack == s.stack and o.layer < s.layer]
        if any(not o.ik_ok for o in below):
            usable.remove(s)
            warnings.append(f"magazine slot {s.id} dropped: a slot below it is unusable")
    take = [s.id for s in sorted(usable, key=lambda s: (-s.layer, float(np.sum(np.abs(np.asarray(s.qnear_rad) -
                                                                                        q_park))), s.id))]
    magazine = mjob.Magazine(slots, take, list(reversed(take)), list(take))

    # station
    ps = cfg["pickup_station"]
    T_wall_station = g.pose_xyz_rpy(ps["xyz_in_wall"], ps["rpy_in_wall_deg"])
    dock_T = g.pose_xyz_rpy(ps["ares_xyz"], ps["ares_rpy_deg"])
    dock = Pose2D.from_T(dock_T)
    T_base_station = T_base_ares @ g.inv(dock.T)
    z_top = float(ps["table_z"]) + float(ps.get("holder_z", 0.0)) + H
    ox, oy = ps["slot_origin"]
    st_slots = []
    for r in range(int(ps["slot_rows"])):
        for c in range(int(ps["slot_cols"])):
            T = g.transl(ox + c * ps["slot_pitch_x"], oy + r * ps["slot_pitch_y"], z_top) @ g.rotx(math.pi)
            T_bf = T_base_station @ T @ g.inv(T_flange_tcp)
            q = ik_near(T_bf, q_park)
            ok = q is not None and ik_near(g.transl(0, 0, approach) @ T_bf, q_park) is not None
            st_slots.append(mjob.StationSlot(f"s{r}{c}", T, None if q is None else q.tolist(), bool(ok)))
    st_take = [s.id for s in st_slots if s.ik_ok]
    if len(st_take) < len(st_slots):
        warnings.append(f"station slots without a kinematic IK solution at the nominal dock (not used): "
                        f"{[s.id for s in st_slots if not s.ik_ok]} - [pickup_station] layout is a PLACEHOLDER")
    if not st_take:
        warnings.append("no station slot reachable: reloads impossible with this [pickup_station] layout")
    R_pref = preferred_flange_R(T_base_station[:3, :3], T_flange_tcp)
    cands, centres = {}, {}
    st_boards = [p for p in pl if p.parent == "station"]
    for p in st_boards:
        r = find_look(T_base_station @ p.T_parent_board, specs[p.name], T_flange_cam, work, R_pref, q_park,
                      g.inv(T_base_station), look_margin_mm)
        if r is not None:
            cands[p.name] = r
            centres[p.name] = board_centre(specs[p.name], p.T_parent_board)
    chosen = choose_boards(cands, centres, max_looks)
    if not chosen:
        warnings.append("station: no board has a look pose with an IK solution at the nominal dock")
    st_looks = [mjob.Look(f"station-{n}", [n], cands[n][0], None, cands[n][1].tolist()) for n in chosen]
    station = mjob.Station(T_wall_station, dock, [p.name for p in st_boards], st_slots, st_take, st_looks)

    # planned magazine slot per stone (same rule as the sequencer: refill min(free slots, station slots))
    state = mjob.SlotState.magazine(magazine)
    for st in stops:
        for t in st.stones:
            if state.empty():
                n = min(state.n_free, len(st_take))
                for _ in range(n):
                    state.fill(state.next_fill())
            sid = state.next_take()
            if sid is None:
                warnings.append(f"stone {t.key}: magazine empty and no reload possible")
                break
            state.take(sid)
            t.slot = sid

    cfg_path = Path(config_path or STATION_TOML)
    meta = {"wall_dist_mm": dist, "length_stones": int(length), "n_stones": len(stones),
            "plan_a_mm": [float(a) for a, _ in plan], "look_source": "nominal", "look_dist_mm": work,
            "look_margin_mm": look_margin_mm,
            "reach_check": "kinematic",
            "reach_check_note": "nominal UR5 DH IK in the robodk/motion.py family only - no collision or occlusion "
                                "check (robodk/look_study.py, simulate.py)",
            "reach_table": {"path": "results/reach_table.json", "key": key}, "park_q_source": park_src,
            "magazine_source": ("[deck] magazine_* keys" if "magazine_rows_dx" in deck_cfg else
                                "robodk/simulate.py:36-37 MAG_ROWS / MAG_Y (copied)"),
            "warnings": warnings}
    job = mjob.Job(stops, magazine, station, T_flange_tcp, T_ares_base, q_park.tolist(), approach,
                   mjob.config_sha256(cfg_path), mjob.depends_on(cfg, DEPENDS_PATTERNS, cfg_path),
                   "tools/make_job.py build_nominal (config + robodk/wallplan.py + results/reach_table.json)",
                   meta=meta)
    problems = mjob.validate(job, boards=[p.name for p in pl])
    if problems:
        raise mjob.JobError(problems)
    return job


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--length", type=int, default=24, help="stones in course 0 (simulate.py --length; README: 24)")
    ap.add_argument("--dist", type=float, default=None, help="wall distance [mm] (default [wall] dist_nominal)")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--reach-table", default=None, help="reach table JSON (default results/reach_table.json)")
    ap.add_argument("--allow-stale-reach", action="store_true", help="use a reach table of another configuration")
    ap.add_argument("--max-looks", type=int, default=2, help="boards (look poses) per stop")
    ap.add_argument("--look-margin", type=float, default=LOOK_MARGIN_MM,
                    help="look poses must stay reachable for ARES errors up to this [mm] (kinematic check)")
    ap.add_argument("--out", default=None, help="output JSON (default data/jobs/nominal_L<length>.json)")
    ap.add_argument("--print", action="store_true", help="print the job summary only, write nothing")
    args = ap.parse_args(argv)
    cfg = mconfig.load(args.config)
    try:
        job = build_nominal(cfg, args.length, args.dist, reach_table_path=args.reach_table,
                            allow_stale_reach=args.allow_stale_reach, max_looks=args.max_looks,
                            config_path=args.config, look_margin_mm=args.look_margin)
    except (ValueError, mjob.JobError) as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1
    print(job.summary())
    if args.print:
        return 0
    out = Path(args.out) if args.out else REPO / "data" / "jobs" / f"nominal_L{args.length}.json"
    mjob.save(job, out)
    print(f"written {out}")
    print(f"depends on {len(job.depends_on)} non-CONFIRMED config keys, e.g. "
          + ", ".join(f"{d['key']} ({d['status']})" for d in job.depends_on[:6]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
