"""Build a nominal job (mauer.job v2) without RoboDK: config + robodk/wallplan.py + results/reach_table.json.

    py.exe tools/make_job.py                     # wall shape from the config: the L of [[wall.legs]] -> nominal_L.json
    py.exe tools/make_job.py --length 24 [--dist 740] [--out data/jobs/nominal_L24.json] [--max-looks 2]
                                                 # straight wall of 24 stones (configs without legs)

What `build_nominal` does (pure Python; the RoboDK planner will later export the same format with collision-checked
joints and via points):
- straight wall (--length N): wall plan exactly as robodk/simulate.py:238-256 (wallplan.layout / sequence /
  check_plan on the cached reach table, same cache key as simulate.reach_table, simulate.py:41-58) -> stops a_j and
  their stones;
- L ([wall] shape "L", [[wall.legs]]): every leg a rectangle with half stones at the ends of the odd courses
  (wallplan.plan_legs, reach margin [wall] reach_margin_mm), leg A completely, then leg B; stop poses per leg
  (heading differs per leg); board looks only for boards of the STOP'S OWN LEG (review 2026-10-05: a board of the
  other leg lies behind / across the built corner, and a fit across two legs pushes a leg-B pose error into the
  leg-A stones); ARES routes (mauer.floor.plan_route, [routes]) for EVERY move between stops (same leg: back off,
  along the leg, approach - real clearance instead of driving along the plates), for the leg change and for the
  station trips from every stop and back, each checked by mauer.floor.validate_route against the full legs, the
  floor plates ([[targets]] leg boards, spare blocks) and the station table - the build fails on any violation;
- nominal ARES pose per stop in the wall frame: inv(wall_frame(dist) @ transl(-a, 0, 0)) (simulate.py:269,
  rdk_common.wall_frame);
- place poses: transl(u, 0, z_top) @ rotx(pi) (rdk_common.place_pose, simulate.py:302), no flip;
- look poses: the camera fronto-parallel above a board centre at [camera] working_dist with T_flange_cam_nominal
  ([camera.mount] PLACEHOLDER), flange orientation as for a place pose, rolled about the optical axis in 15 deg steps
  until the nominal UR5 kinematics (mauer.simworld.ik_near, robodk/motion.py family) has a solution that is also
  free of the BUILT wall (mauer.armcheck: UR5 links, gripper and camera as capsules vs. the stones placed by the end
  of that stop - every measurement of the stop and a resumed run see at most those) - for ARES at the nominal stop,
  displaced by +-LOOK_MARGIN_MM and at the arrival standoff ([sequencer] arrival_standoff_mm, the first measurement
  after a route); per stop the pair of reachable boards with the longest baseline (heading). Coarse envelopes, no
  occlusion (robodk/simulate.py / look_study.py check those) -> meta.reach_check = "kinematic";
- magazine: the slots of robodk/simulate.py MAG_ROWS / MAG_Y (simulate.py:32-38: rows at UR x - 653.6 / - 433.6 mm,
  y -205 / 0 / 205 mm, 3 layers; pick pose transl(x, y, deck + layer H) @ rotz(pi/2) @ rotx(pi), simulate.py:106-108,
  deck = [ares] deck_top_z + [deck] holder_z) unless [deck] magazine_* keys exist; emptied top layer first
  (simulate.py:132-149), filled bottom first; slots without a kinematic IK solution (pick or approach pose) dropped;
- pick-up station from [pickup_station]: station frame in the wall frame, docking pose, stacked holders at the
  slots_xy / half_slots_xy positions (stone length along the station x axis, TCP = stone top at table_z +
  [pickup_station] holder_z (default 0) + layer x stone height), look poses for the station boards at the nominal
  dock clear of a full station and the table (mauer.armcheck.station_boxes); unreachable slots and the slots stacked
  on them dropped (warning), emptied top layer first;
- park pose: [ur] park_q_deg if present, else the IK of the compact pose robodk/simulate.py:280 starts from (TCP
  300 mm ahead of the UR base at the transfer height over a full magazine, simulate.py:114-117, tool down, seed
  [0, -100, 52, -42, -90, 0] deg) - not collision-checked here;
- stone types: full / half ([half_brick], PLACEHOLDER); the magazine is planned per type (mauer.job.reload_plan,
  the same rule as the sequencer) and the station has half-stone holders ([pickup_station] half_slots_xy);
- provenance: config sha256 and every PLACEHOLDER/ASSUMPTION/UNKNOWN config key it depends on.
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from itertools import combinations
from pathlib import Path
from typing import Sequence

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import numpy as np  # noqa: E402

from mauer import armcheck  # noqa: E402
from mauer import config as mconfig  # noqa: E402
from mauer import floor  # noqa: E402
from mauer import geometry as g  # noqa: E402
from mauer import job as mjob  # noqa: E402
from mauer import reach_cache  # noqa: E402
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
                    "[pickup_station] *", "[study] approach", "[ur] park_q_deg", "[half_brick] *",
                    "[[wall.legs]] *", "[routes] *", "[plates] block_*", "[plates] plate_*", "[ares] *",
                    "[sequencer] arrival_standoff_mm")


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
    """Cache key of the reach table of cfg at dist (mauer.reach_cache.key, shared with robodk/simulate.py and
    robodk/look_study.py). [brick] rib_mm is left out: it only places the corner of a wall of legs
    (wallplan.butt_corner) - the ribs are part of the stone mesh the reach study used."""
    return reach_cache.key(cfg, dist)


def load_reach_table(cfg: dict, dist: float, path: Path | None = None, allow_stale: bool = False) -> tuple[dict, str]:
    """({course: {u_rel: ok}}, key) from the reach-table cache (results/reach_table.json, mauer.reach_cache - one
    table per key, several wall distances side by side); ValueError if there is none for this configuration and
    distance (allow_stale: then the table cached for the same distance, else the only one)."""
    path = Path(path or REACH_TABLE)
    tables = reach_cache.read(path)
    if not tables:
        raise ValueError(f"{path} missing - run py.exe robodk/simulate.py --plan-only once (RoboDK)")
    key = reach_key(cfg, dist)
    if key in tables:
        return tables[key]["table"], key
    if allow_stale:
        same = [k for k, v in tables.items() if v["dist"] is not None and abs(v["dist"] - dist) < 1e-6]
        k = same[0] if same else (next(iter(tables)) if len(tables) == 1 else None)
        if k is not None:
            return tables[k]["table"], k
    raise ValueError(f"{path} was computed for another configuration (key {key} not among "
                     f"{sorted(f'{k} ({v["dist"]:g} mm)' for k, v in tables.items())}) - rerun "
                     f"py.exe robodk/simulate.py --plan-only --dist {dist:g}")


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
              margin_mm: float = 0.0, arm=None, offsets: Sequence[tuple[float, float]] = ()
              ) -> tuple[np.ndarray, np.ndarray, float] | None:
    """(T_base_flange, qnear, roll) of the first roll with a family IK solution, None if none. With margin_mm > 0 the
    look must also stay reachable when the sequencer re-aims it for ARES displaced by +-margin_mm along the parent
    x and y axes (T_parent_base = nominal UR base in the parent frame), also around every extra ARES displacement in
    `offsets` [(dx, dy) mm, parent frame] (the arrival standoff). arm = mauer.armcheck.ArmChecker (parent frame): every
    one of these configurations must be free of its boxes (the built wall)."""
    shifts: list[tuple[float, float]] = []
    if T_parent_base is not None:
        cross = [(margin_mm, 0.0), (-margin_mm, 0.0), (0.0, margin_mm), (0.0, -margin_mm)] if margin_mm > 0 else []
        for ox, oy in [(0.0, 0.0)] + [tuple(o) for o in offsets]:
            for dx, dy in [(0.0, 0.0)] + cross:
                p = (ox + dx, oy + dy)
                if p != (0.0, 0.0) and p not in shifts:
                    shifts.append(p)
    for r in rolls():
        T = look_pose(T_base_board, spec, T_flange_cam, dist_mm, R_pref, r)
        q = ik_near(T, q_ref)
        if q is None:
            continue
        if arm is not None and T_parent_base is not None and arm.hits(q, T_parent_base):
            continue
        ok = True
        for dx, dy in shifts:
            T_pb = g.transl(dx, dy, 0.0) @ T_parent_base
            qs = ik_near(g.inv(T_pb) @ T_parent_base @ T, q)
            if qs is None or (arm is not None and arm.hits(qs, T_pb)):
                ok = False
                break
        if ok:
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
def stop_pose(cfg: dict, dist: float, a: float, T_wall_leg: np.ndarray | None = None) -> Pose2D:
    """Nominal ARES pose in the wall frame for a stop at leg position a: inv(wall_frame(dist) @ transl(-a, 0, 0))
    in the leg frame (simulate.py:269, rdk_common.wall_frame), then T_wall_leg (identity for the straight wall)."""
    T = g.inv(wall_frame_in_ares(cfg, dist) @ g.transl(-a, 0.0, 0.0))
    return Pose2D.from_T(T if T_wall_leg is None else np.asarray(T_wall_leg, float) @ T)


class _Ctx:
    """Everything the stop / magazine / station builders share."""

    def __init__(self, cfg: dict, dist: float, max_looks: int, look_margin_mm: float):
        self.cfg, self.dist, self.max_looks, self.look_margin = cfg, dist, max_looks, look_margin_mm
        self.warnings: list[str] = []
        self.T_ares_base = mconfig.T_ares_base(cfg)
        self.T_flange_tcp = mconfig.T_flange_tcp(cfg)
        self.T_flange_cam = mconfig.T_flange_cam_nominal(cfg)
        self.T_base_ares = g.inv(self.T_ares_base)
        self.specs = board_specs(cfg)
        self.pl = placements(cfg)
        self.work = float(cfg["camera"]["working_dist"])
        self.H = float(cfg["brick"]["height"])
        self.approach = float(cfg["study"]["approach"])
        deck_cfg = cfg["deck"]
        self.rows_dx = deck_cfg.get("magazine_rows_dx", [r[0] for r in SIM_MAG_ROWS])
        self.n_layers = int(deck_cfg.get("magazine_layers", max(n for _, n in SIM_MAG_ROWS)))
        self.mag_y = deck_cfg.get("magazine_y", list(SIM_MAG_Y))
        self.q_park, self.park_src = park_q(cfg, self.T_ares_base, self.T_flange_tcp, self.n_layers)
        self.board_leg = {str(t["name"]): (str(t["leg"]) if "leg" in t else None)
                          for t in cfg.get("targets", []) if t.get("parent") == "wall"}
        self.standoff = arrival_standoff(cfg)


def arrival_standoff(cfg: dict) -> float:
    """[sequencer] arrival_standoff_mm (ASSUMPTION; 0 if missing): a route ends this far before its stop, the wall is
    measured there and the closed loop approaches the stop (mauer.sequencer)."""
    return float((cfg.get("sequencer", {}) or {}).get("arrival_standoff_mm", 0.0))


def standoff_offset(ares: Pose2D, standoff_mm: float) -> list[tuple[float, float]]:
    """ARES displacement (wall frame) of the arrival standoff: standoff_mm behind the stop (body -x)."""
    if standoff_mm <= 0:
        return []
    return [(-standoff_mm * math.cos(ares.theta_rad), -standoff_mm * math.sin(ares.theta_rad))]


def wall_look_candidates(ctx: _Ctx, ares: Pose2D, T_wall_leg: np.ndarray, leg: str | None = None,
                         built: Sequence = (), legs_by_name: dict | None = None,
                         standoff_mm: float | None = None) -> tuple[dict, dict]:
    """({board: (T_base_flange, q, roll)}, {board: centre}) of the wall boards with a usable look from ARES at `ares`:
    only boards of `leg` (None = every wall board, straight wall), family IK for the nominal pose, +-look margin and the
    arrival standoff, and none of these configurations hits the stones in `built` (mauer.armcheck)."""
    T_base_wall = ctx.T_base_ares @ g.inv(ares.T)
    R_pref = preferred_flange_R((T_base_wall @ T_wall_leg)[:3, :3], ctx.T_flange_tcp)
    arm = armcheck.ArmChecker(ctx.cfg, armcheck.stone_boxes(ctx.cfg, built, legs_by_name)) if built else None
    offs = standoff_offset(ares, ctx.standoff if standoff_mm is None else standoff_mm)
    cands, centres = {}, {}
    for p in ctx.pl:
        if p.parent != "wall" or (leg is not None and ctx.board_leg.get(p.name) != leg):
            continue
        r = find_look(T_base_wall @ p.T_parent_board, ctx.specs[p.name], ctx.T_flange_cam, ctx.work, R_pref,
                      ctx.q_park, g.inv(T_base_wall), ctx.look_margin, arm=arm, offsets=offs)
        if r is not None:
            cands[p.name] = r
            centres[p.name] = board_centre(ctx.specs[p.name], p.T_parent_board)
    return cands, centres


def _wall_looks(ctx: _Ctx, k: int, a: float, ares: Pose2D, T_wall_leg: np.ndarray, leg: str | None = None,
                built: Sequence = (), legs_by_name: dict | None = None,
                standoff_mm: float | None = None) -> list[mjob.Look]:
    """Look poses of stop k: the boards of wall_look_candidates; the max_looks boards with the longest baseline."""
    cands, centres = wall_look_candidates(ctx, ares, T_wall_leg, leg, built, legs_by_name, standoff_mm)
    chosen = choose_boards(cands, centres, ctx.max_looks)
    where = f"stop {k} (a = {a:.0f} mm{', leg ' + leg if leg else ''})"
    if not chosen:
        raise ValueError(f"{where}: no wall board has a look pose with an IK solution clear of the built wall")
    if len(chosen) < 2:
        ctx.warnings.append(f"{where}: only board {chosen} reachable for a fronto-parallel look - heading from one "
                            "board only")
    return [mjob.Look(f"stop{k}-{n}", [n], cands[n][0], None, cands[n][1].tolist()) for n in chosen]


def _stone_task(ctx: _Ctx, k: int, s, T_base_wall: np.ndarray, T_wall_leg: np.ndarray | None) -> mjob.StoneTask:
    """Place pose T_wall_leg @ transl(u, 0, z_top) @ rotx(pi) (rdk_common.place_pose), flipped by rotz(pi) only when
    the nominal kinematics has no solution otherwise."""
    T_wl = np.eye(4) if T_wall_leg is None else T_wall_leg
    T_wt = T_wl @ g.transl(s.u, 0.0, s.z_top) @ g.rotx(math.pi)
    q = ik_near(T_base_wall @ T_wt @ g.inv(ctx.T_flange_tcp), ctx.q_park)
    flip = False
    if q is None:
        T_f = T_wt @ g.rotz(math.pi)
        q = ik_near(T_base_wall @ T_f @ g.inv(ctx.T_flange_tcp), ctx.q_park)
        if q is not None:
            T_wt, flip = T_f, True
        else:
            ctx.warnings.append(f"stop {k} stone {s.key}: no kinematic IK solution for the place pose")
    leg = getattr(s, "leg", "") or None
    return mjob.StoneTask(s.course, s.index, float(s.u), float(s.z_top), T_wt, flip, None,
                          None if q is None else q.tolist(), [], leg, getattr(s, "kind", "full"),
                          float(s.length) if getattr(s, "length", 0.0) else None)


def _magazine(ctx: _Ctx) -> mjob.Magazine:
    cfg = ctx.cfg
    deck = cfg["ares"]["deck_top_z"] + cfg["deck"]["holder_z"]
    x0 = ctx.T_ares_base[0, 3]
    slots: list[mjob.MagazineSlot] = []
    for ri, dx in enumerate(ctx.rows_dx):
        for yi, y in enumerate(ctx.mag_y):
            for lay in range(1, ctx.n_layers + 1):
                T = g.transl(x0 + dx, y, deck + lay * ctx.H) @ g.rotz(math.pi / 2) @ g.rotx(math.pi)
                T_bf = ctx.T_base_ares @ T @ g.inv(ctx.T_flange_tcp)
                q = ik_near(T_bf, ctx.q_park)
                ok = q is not None and ik_near(g.transl(0, 0, ctx.approach) @ T_bf, ctx.q_park) is not None
                slots.append(mjob.MagazineSlot(f"r{ri}y{yi}l{lay}", T, lay, f"r{ri}y{yi}",
                                               None if q is None else q.tolist(), bool(ok)))
    usable = [s for s in slots if s.ik_ok]
    dropped = [s.id for s in slots if not s.ik_ok]
    if dropped:
        ctx.warnings.append(f"magazine slots without a kinematic IK solution (dropped): {dropped}")
    # a slot above an unusable one cannot be filled -> drop whole stacks above a gap
    for s in list(usable):
        below = [o for o in slots if o.stack == s.stack and o.layer < s.layer]
        if any(not o.ik_ok for o in below):
            usable.remove(s)
            ctx.warnings.append(f"magazine slot {s.id} dropped: a slot below it is unusable")
    take = [s.id for s in sorted(usable, key=lambda s: (-s.layer, float(np.sum(np.abs(np.asarray(s.qnear_rad) -
                                                                                        ctx.q_park))), s.id))]
    return mjob.Magazine(slots, take, list(reversed(take)), list(take))


def _station(ctx: _Ctx) -> mjob.Station:
    """Station frame, dock, the stacked holders for full and half stones ([pickup_station] slots_xy / slot_layers,
    half_slots_xy / half_slot_layers, PLACEHOLDER), look poses for the station boards at the nominal dock. Slots
    without a kinematic IK solution (pick or approach) are dropped, and with them every slot stacked on them; the
    station is emptied top layer first (like the magazine)."""
    cfg = ctx.cfg
    ps = cfg["pickup_station"]
    T_wall_station = g.pose_xyz_rpy(ps["xyz_in_wall"], ps["rpy_in_wall_deg"])
    dock = Pose2D.from_T(g.pose_xyz_rpy(ps["ares_xyz"], ps["ares_rpy_deg"]))
    T_base_station = ctx.T_base_ares @ g.inv(dock.T)
    st_slots: list[mjob.StationSlot] = []
    groups = [("s", "full", ctx.H, ps["slots_xy"], int(ps.get("slot_layers", 1)))]
    if ps.get("half_slots_xy"):
        hh = float(cfg.get("half_brick", {}).get("height", ctx.H))
        groups.append(("h", "half", hh, ps["half_slots_xy"], int(ps.get("half_slot_layers", 1))))
    z0 = float(ps["table_z"]) + float(ps.get("holder_z", 0.0))
    for prefix, kind, height, xys, layers in groups:
        for i, (x, y) in enumerate(xys):
            stack = f"{prefix}{i:02d}"
            for lay in range(1, layers + 1):
                T = g.transl(float(x), float(y), z0 + lay * height) @ g.rotx(math.pi)
                T_bf = T_base_station @ T @ g.inv(ctx.T_flange_tcp)
                q = ik_near(T_bf, ctx.q_park)
                ok = q is not None and ik_near(g.transl(0, 0, ctx.approach) @ T_bf, ctx.q_park) is not None
                st_slots.append(mjob.StationSlot(f"{stack}l{lay}", T, None if q is None else q.tolist(), bool(ok),
                                                 kind, lay, stack))
    usable = [s for s in st_slots if s.ik_ok]
    if len(usable) < len(st_slots):
        ctx.warnings.append(f"station slots without a kinematic IK solution at the nominal dock (not used): "
                            f"{[s.id for s in st_slots if not s.ik_ok]} - [pickup_station] layout is a PLACEHOLDER")
    for s in list(usable):                        # a stone cannot lie on an unusable (empty) holder
        if any(not o.ik_ok for o in st_slots if o.stack_id == s.stack_id and o.layer < s.layer):
            usable.remove(s)
            ctx.warnings.append(f"station slot {s.id} dropped: a slot below it is unusable")
    st_take = [s.id for s in sorted(usable, key=lambda s: (-s.layer, float(np.sum(np.abs(np.asarray(s.qnear_rad)
                                                                                         - ctx.q_park))), s.id))]
    if not st_take:
        ctx.warnings.append("no station slot reachable: reloads impossible with this [pickup_station] layout")
    R_pref = preferred_flange_R(T_base_station[:3, :3], ctx.T_flange_tcp)
    # looks clear of a FULL station (every usable holder filled - the state on arrival after a top-up) and the table
    full = [s for s in st_slots if s.id in st_take]
    arm = armcheck.ArmChecker(cfg, armcheck.station_boxes(cfg, full, floor.station_table_extent(cfg)))
    cands, centres = {}, {}
    st_boards = [p for p in ctx.pl if p.parent == "station"]
    for p in st_boards:
        r = find_look(T_base_station @ p.T_parent_board, ctx.specs[p.name], ctx.T_flange_cam, ctx.work, R_pref,
                      ctx.q_park, g.inv(T_base_station), ctx.look_margin, arm=arm)
        if r is not None:
            cands[p.name] = r
            centres[p.name] = board_centre(ctx.specs[p.name], p.T_parent_board)
    chosen = choose_boards(cands, centres, ctx.max_looks)
    if not chosen:
        ctx.warnings.append("station: no board has a look pose with an IK solution at the nominal dock")
    st_looks = [mjob.Look(f"station-{n}", [n], cands[n][0], None, cands[n][1].tolist()) for n in chosen]
    return mjob.Station(T_wall_station, dock, [p.name for p in st_boards], st_slots, st_take, st_looks)


def plan_slots(stops: list[mjob.Stop], magazine: mjob.Magazine, station: mjob.Station, warnings: list[str]) -> dict:
    """Stone types of the initially filled magazine slots and the planned slot of every stone - the sequencer's rule
    (mauer.job.reload_plan / reload_short: the empty magazine is refilled from the station with the types of the
    next stones; the operator tops the station up when it would bring fewer stones than a full one)."""
    stones = [t for st in stops for t in st.stones]
    kinds = [t.kind for t in stones]
    init = mjob.SlotState.magazine(magazine, kinds={})
    replay = init.copy()
    initial_kinds: dict[str, str] = {}
    for j in range(min(len(kinds), len(init))):
        sid = replay.next_take()
        replay.take(sid)
        initial_kinds[sid] = kinds[j]
    magazine.initial_fill = [s for s in magazine.initial_fill if s in initial_kinds]
    magazine.initial_kinds = initial_kinds
    mag = mjob.SlotState.magazine(magazine)
    full_station = mjob.SlotState.station(station)
    st = full_station.copy()
    reloads = refills = 0
    for j, t in enumerate(stones):
        if mag.empty():
            if mjob.reload_short(st, full_station, mag, kinds[j:]):
                st = full_station.copy()
                refills += 1
            pairs = mjob.reload_plan(mag, st, kinds[j:])
            if not pairs:
                warnings.append(f"stone {t.key}: magazine empty and no reload possible")
                break
            for ssid, mid, kind in pairs:
                st.take(ssid)
                mag.fill(mid, kind)
            reloads += 1
        sid = mag.next_take(kind=t.kind)
        if sid is None:
            warnings.append(f"stone {t.key}: no {t.kind} stone can be taken from the magazine")
            break
        mag.take(sid)
        t.slot = sid
    return {"reloads": reloads, "station_refills": refills}


def _common_meta(ctx: _Ctx, key: str, extra: dict) -> dict:
    return {"wall_dist_mm": ctx.dist, "look_source": "nominal", "look_dist_mm": ctx.work,
            "look_margin_mm": ctx.look_margin, "reach_check": "kinematic",
            "reach_check_note": "nominal UR5 DH IK in the robodk/motion.py family only - no collision or occlusion "
                                "check (robodk/look_study.py, simulate.py)",
            "reach_table": {"path": "results/reach_table.json", "key": key}, "park_q_source": ctx.park_src,
            "magazine_source": ("[deck] magazine_* keys" if "magazine_rows_dx" in ctx.cfg["deck"] else
                                "robodk/simulate.py:36-37 MAG_ROWS / MAG_Y (copied)"),
            **extra, "warnings": ctx.warnings}


def _finish(ctx: _Ctx, stops, magazine, station, key, meta_extra, config_path, legs=None) -> mjob.Job:
    slot_info = plan_slots(stops, magazine, station, ctx.warnings)
    cfg_path = Path(config_path or STATION_TOML)
    meta = _common_meta(ctx, key, {**meta_extra, "planned_reloads": slot_info["reloads"],
                                   "planned_station_refills": slot_info["station_refills"]})
    job = mjob.Job(stops, magazine, station, ctx.T_flange_tcp, ctx.T_ares_base, ctx.q_park.tolist(), ctx.approach,
                   mjob.config_sha256(cfg_path), mjob.depends_on(ctx.cfg, DEPENDS_PATTERNS, cfg_path),
                   "tools/make_job.py build_nominal (config + robodk/wallplan.py + results/reach_table.json)",
                   meta=meta, legs=legs or [])
    problems = mjob.validate(job, boards=[p.name for p in ctx.pl])
    if problems:
        raise mjob.JobError(problems)
    return job


def build_nominal(cfg: dict, length: int | None = None, dist: float | None = None, *,
                  reach_table_path: Path | None = None, allow_stale_reach: bool = False, max_looks: int = 2,
                  config_path: Path | None = None, look_margin_mm: float = LOOK_MARGIN_MM,
                  half_stones: bool | None = None) -> mjob.Job:
    """Nominal job (see the module docstring). length = stones in course 0 of a STRAIGHT wall; None = the shape of
    the config: the L of [[wall.legs]] (build_l), or a straight wall of 24 stones without legs."""
    if length is None and wallplan_is_l(cfg):
        return build_l(cfg, dist, reach_table_path=reach_table_path, allow_stale_reach=allow_stale_reach,
                       max_looks=max_looks, config_path=config_path, look_margin_mm=look_margin_mm,
                       half_stones=half_stones)
    length = 24 if length is None else int(length)
    dist = float(cfg["wall"]["dist_nominal"] if dist is None else dist)
    table, key = load_reach_table(cfg, dist, reach_table_path, allow_stale_reach)
    stones, plan = make_plan(cfg, length, table)
    ctx = _Ctx(cfg, dist, max_looks, look_margin_mm)
    stops: list[mjob.Stop] = []
    built: list = []
    for k, (a, batch) in enumerate(plan):
        ares = stop_pose(cfg, dist, a)
        T_base_wall = ctx.T_base_ares @ g.inv(ares.T)
        built += list(batch)
        looks = _wall_looks(ctx, k, a, ares, np.eye(4), None, built, None, standoff_mm=0.0)   # direct moves
        tasks = [_stone_task(ctx, k, s, T_base_wall, None) for s in batch]
        stops.append(mjob.Stop(k, float(a), ares, looks, tasks))
    return _finish(ctx, stops, _magazine(ctx), _station(ctx), key,
                   {"shape": "straight", "length_stones": int(length), "n_stones": len(stones),
                    "plan_a_mm": [float(a) for a, _ in plan]}, config_path)


def wallplan_is_l(cfg: dict) -> bool:
    return load_wallplan().is_l(cfg)


# ── L wall: floor, plates, routes ─────────────────────────────────────────────
def route_params(cfg: dict) -> dict:
    """[routes] (ASSUMPTIONS): clearance, back-off, weight of translations after the rotation, search grid; the
    arrival standoff of the sequencer ([sequencer] arrival_standoff_mm)."""
    r = cfg.get("routes", {}) or {}
    return {"clearance_mm": float(r.get("clearance_mm", 50.0)), "backoff_mm": float(r.get("backoff_mm", 100.0)),
            "arrival_standoff_mm": arrival_standoff(cfg),
            "after_rotation_weight": float(r.get("after_rotation_weight", 5.0)),
            "grid_mm": float(r.get("grid_mm", 100.0)), "search_mm": float(r.get("search_mm", 2500.0))}


def T_wall_station(cfg: dict) -> np.ndarray:
    ps = cfg["pickup_station"]
    return g.pose_xyz_rpy(ps["xyz_in_wall"], ps["rpy_in_wall_deg"])


def dock_in_wall(cfg: dict) -> Pose2D:
    ps = cfg["pickup_station"]
    return Pose2D.from_T(T_wall_station(cfg) @ g.pose_xyz_rpy(ps["ares_xyz"], ps["ares_rpy_deg"]))


def target_sites(cfg: dict, legs_: list) -> list:
    """floor.PlateSite of every [[targets]] wall board with a `leg` (ValueError if one is not on a block centre)."""
    specs = board_specs(cfg)
    by = {lg.name: lg for lg in legs_}
    out = []
    for t in cfg.get("targets", []):
        if t.get("parent") == "wall" and "leg" in t:
            out.append(floor.site_from_target(cfg, by[str(t["leg"])], t, specs[str(t["name"])].size_mm))
    return out


def floor_model(cfg: dict, legs_: list, sites: list, stop_poses: list[tuple[str, Pose2D]]) -> tuple[list, list[str]]:
    """(obstacles, problems): full leg footprints, plates + spare blocks of `sites`, the station table; problems =
    plates/blocks overlapping each other, a leg, ARES at a stop or the table, and ARES at a stop overlapping a leg or
    the table."""
    wp = load_wallplan()
    ares = floor.AresShape.from_config(cfg)
    leg_polys = {lg.name: wp.leg_footprint(cfg, lg) for lg in legs_}
    table = floor.station_table_poly(cfg, T_wall_station(cfg))
    fps = [(lab, ares.footprint(p)) for lab, p in stop_poses]
    problems = floor.check_sites(sites, leg_polys, fps, table)
    for lab, fp in fps:
        for name, lp in leg_polys.items():
            if floor.overlaps(fp, lp):
                problems.append(f"ARES at {lab} overlaps leg {name}")
        if floor.overlaps(fp, table):
            problems.append(f"ARES at {lab} overlaps the station table")
    return floor.obstacles(leg_polys, sites, table), problems


def plan_l_routes(cfg: dict, stops: list[mjob.Stop], obstacles: list, dock: Pose2D) -> dict:
    """Routes of an L job (mauer.floor.plan_route): every move between stops (same leg: back off, along the leg,
    approach - the plates end 10 mm before the ARES front at a stop, so driving along the stop line would keep no
    clearance against slip; other leg: the leg change), station trips from every stop and back. Returns
    {"problems": [...], "routes": {...stats}}; fills the routes into the stops."""
    rp = route_params(cfg)
    ares = floor.AresShape.from_config(cfg)
    kw = dict(after_rotation_weight=rp["after_rotation_weight"], grid_mm=rp["grid_mm"], search_mm=rp["search_mm"])
    problems: list[str] = []
    info: dict = {}

    def plan(a: Pose2D, b: Pose2D, name: str) -> list[Pose2D]:
        try:
            r = floor.plan_route(a, b, obstacles, ares, rp["clearance_mm"], rp["backoff_mm"], **kw)
        except floor.RouteError as e:
            problems.append(f"{name}: {e}")
            return []
        problems.extend(floor.validate_route(r.waypoints, obstacles, ares, rp["clearance_mm"], name=name))
        info[name] = {**r.stats, "waypoints": len(r.waypoints), "cost": r.cost}
        return r.waypoints

    for k, st in enumerate(stops):
        if k > 0:
            prev = stops[k - 1]
            what = "stop move" if prev.leg == st.leg else "leg change"
            st.route = plan(prev.ares, st.ares, f"{what} stop {k - 1} -> stop {k}")
        st.route_to_station = plan(st.ares, dock, f"stop {k} -> station")
        st.route_from_station = plan(dock, st.ares, f"station -> stop {k}")
    return {"problems": problems, "routes": info, **rp, "ares_radius_mm": ares.radius}


def build_l(cfg: dict, dist: float | None = None, *, reach_table_path: Path | None = None,
            allow_stale_reach: bool = False, max_looks: int = 2, config_path: Path | None = None,
            look_margin_mm: float = LOOK_MARGIN_MM, half_stones: bool | None = None) -> mjob.Job:
    """Nominal job for the L of [[wall.legs]]: leg after leg (config order), routes, typed stones (see the module
    docstring). ValueError / JobError on an invalid plan, plate layout or route."""
    wp = load_wallplan()
    legs_ = wp.legs(cfg)
    if not legs_:
        raise ValueError("build_l: no [[wall.legs]] in the config")
    dist = float(cfg["wall"]["dist_nominal"] if dist is None else dist)
    if half_stones is None:
        half_stones = bool(cfg["wall"].get("half_stones", True))
    margin = float(cfg["wall"].get("reach_margin_mm", 0.0))
    table, key = load_reach_table(cfg, dist, reach_table_path, allow_stale_reach)
    stones, plans = wp.plan_legs(cfg, legs_, table, half_stones, margin)
    ctx = _Ctx(cfg, dist, max_looks, look_margin_mm)
    for prev, lg in zip(legs_, legs_[1:]):
        exp = wp.butt_corner(cfg, prev, lg.n0, lg.name)
        if abs(exp.x - lg.x) > 1e-6 or abs(exp.y - lg.y) > 1e-6 or abs(exp.theta - lg.theta) > 1e-9:
            ctx.warnings.append(f"leg {lg.name} frame ({lg.x:.1f}, {lg.y:.1f}, {math.degrees(lg.theta):.1f} deg) is "
                                f"not the butt corner of leg {prev.name} ({exp.x:.1f}, {exp.y:.1f}, "
                                f"{math.degrees(exp.theta):.1f} deg)")
    T_legs = mconfig.leg_frames(cfg)
    by_name = {lg.name: lg for lg in legs_}
    stops: list[mjob.Stop] = []
    built: list = []                                # stones placed by the end of the stop (looks must clear them)
    for lg in legs_:
        T_wl = T_legs[lg.name]
        for a, batch in plans[lg.name]:
            k = len(stops)
            ares = stop_pose(cfg, dist, a, T_wl)
            T_base_wall = ctx.T_base_ares @ g.inv(ares.T)
            built += list(batch)
            looks = _wall_looks(ctx, k, a, ares, T_wl, lg.name, built, by_name)
            tasks = [_stone_task(ctx, k, s, T_base_wall, T_wl) for s in batch]
            stops.append(mjob.Stop(k, float(a), ares, looks, tasks, lg.name))
    magazine, station = _magazine(ctx), _station(ctx)
    sites = target_sites(cfg, legs_)
    obstacles, fproblems = floor_model(cfg, legs_, sites, [(f"stop {s.index}", s.ares) for s in stops])
    rinfo = plan_l_routes(cfg, stops, obstacles, station.dock_in_wall)
    problems = fproblems + rinfo.pop("problems")
    if problems:
        raise ValueError("L floor/route check failed:\n  - " + "\n  - ".join(problems))
    legs_meta = [{"name": lg.name, "n0": lg.n0, "T_wall_leg": T_legs[lg.name]} for lg in legs_]
    n_half = sum(s.kind == "half" for s in stones)
    extra = {"shape": "L", "legs": [{"name": lg.name, "n0": lg.n0, "plan_a_mm": [float(a) for a, _ in plans[lg.name]]}
                                    for lg in legs_],
             "n_stones": len(stones), "n_half_stones": n_half, "half_stones": bool(half_stones),
             "reach_margin_mm": margin, "plates": [{"board": s.board, "leg": s.leg, "k": s.k, "u_mm": s.u,
                                                    "spare_block": s.spare} for s in sites],
             "route_check": rinfo}
    return _finish(ctx, stops, magazine, station, key, extra, config_path, legs_meta)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--length", type=int, default=None,
                    help="straight wall of LENGTH stones in course 0 (simulate.py --length); default: the shape of "
                         "the config (the L of [[wall.legs]], else 24)")
    ap.add_argument("--no-half-stones", action="store_true", help="L: legs as trapezoids (full stones only)")
    ap.add_argument("--dist", type=float, default=None, help="wall distance [mm] (default [wall] dist_nominal)")
    ap.add_argument("--config", default=None, help="station.toml (default config/station.toml)")
    ap.add_argument("--reach-table", default=None, help="reach table JSON (default results/reach_table.json)")
    ap.add_argument("--allow-stale-reach", action="store_true", help="use a reach table of another configuration")
    ap.add_argument("--max-looks", type=int, default=2, help="boards (look poses) per stop")
    ap.add_argument("--look-margin", type=float, default=LOOK_MARGIN_MM,
                    help="look poses must stay reachable for ARES errors up to this [mm] (kinematic check)")
    ap.add_argument("--out", default=None, help="output JSON (default data/jobs/nominal_L<length>.json, the L: "
                                                "data/jobs/nominal_L.json)")
    ap.add_argument("--print", action="store_true", help="print the job summary only, write nothing")
    args = ap.parse_args(argv)
    cfg = mconfig.load(args.config)
    try:
        job = build_nominal(cfg, args.length, args.dist, reach_table_path=args.reach_table,
                            allow_stale_reach=args.allow_stale_reach, max_looks=args.max_looks,
                            config_path=args.config, look_margin_mm=args.look_margin,
                            half_stones=False if args.no_half_stones else None)
    except (ValueError, mjob.JobError) as e:
        print(f"FAILED: {e}", file=sys.stderr)
        return 1
    print(job.summary())
    if args.print:
        return 0
    name = "nominal_L.json" if job.legs else f"nominal_L{job.meta.get('length_stones', args.length)}.json"
    out = Path(args.out) if args.out else REPO / "data" / "jobs" / name
    mjob.save(job, out)
    print(f"written {out}")
    print(f"depends on {len(job.depends_on)} non-CONFIRMED config keys, e.g. "
          + ", ".join(f"{d['key']} ({d['status']})" for d in job.depends_on[:6]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
