"""Simulate the brick laying in RoboDK with a valid placement order and collision-checked paths.

Runs in its OWN RoboDK instance (rdk_common.connect(new_instance=True), API port 20599 by default, flags -NEWINSTANCE
-NOSPLASH -EXIT_LAST_COM): the station is built from config/station.toml (build_station.build) and the instance is
closed at the end; the user's RoboDK is never connected to.

Straight wall (`--length N`, or a config without [[wall.legs]]) - as before:
  1. reach table: which wall positions (relative to ARES) the UR5 can serve, per course (IK, vertical approach,
     collision-free against ARES with gripper and held stone) - cached in results/reach_table.json;
  2. wall plan (wallplan.py): trapezoid wall, stones only on complete supports, ARES stops and moves;
  3. execution: every pick and place is planned with motion.py (all segments tested against ARES, magazine,
     the wall built so far and the arm itself) and only then executed. Collision checking stays on.

Wall of legs (config [[wall.legs]]: the C of 2026-10-06, the L of 2026-10-05; default when legs exist) - runs the
planner's job:
  - job = tools/make_job.py build_nominal(cfg) (robodk/wallplan.py plan_legs on the reach table, one leg after another,
    ARES pose (x, y, theta) per stop in the wall frame, leg-change and station routes from mauer.floor, typed stones
    full/half with their magazine slot, reloads by mauer.job.reload_plan);
  - world = wall frame; ARES (frame "ARES base_link") is set to the stop pose and moved along the route waypoints
    (translation / rotation legs, leg change animated with the 90 deg turn); every route is sampled (100 mm / 5 deg)
    and checked in 3D (chassis, parked arm with gripper and camera, magazine stones against the built wall, the
    boards and the pick-up table);
  - stones: the target is the job's T_wall_tcp (T_wall_leg @ transl(u, 0, z_top) @ rotx(pi)), i.e. placed from the
    stone's leg frame; full stones with the CAD mesh, half stones with cad/stone_half_placeholder.stl
    (robodk/make_half_stone.py, PLACEHOLDER); the magazine holds both types in the job's slots;
  - station trips (the job's reload plan, mauer.job.reload_plan / reload_short as in the sequencer): the pick-up
    station starts full (every usable holder of the job, stacks of 2, PLACEHOLDER layout) and the operator tops it up
    before a trip when it would bring fewer stones than a full one; ARES drives the route to the dock, the station
    boards are checked and the planned looks executed like at the wall, then every planned stone is moved station
    holder -> magazine slot with collision-checked motion (top layer first), the arm parks and ARES drives back;
    --no-trips refills the magazine without driving or picking (fast check of the wall alone);
  - every motion is planned and collision-checked with motion.Planner (MoveJ_Test 1 deg / MoveL_Test 2 mm) exactly
    as for the straight wall; the camera body and the adapter plate (objects on the tool) are switched on against
    every stone (RoboDK does not check tool objects against static objects by default); the tool against the own arm
    additionally with the capsule model mauer.armcheck.self_clearance (RoboDK's meshes miss the contact of
    2026-10-06, motion.py docstring);
  - a stone whose motion cannot be planned is diagnosed (no IK = reach hole, or the colliding pairs of the vertical
    approach / transfer), then put in place without motion (red) so that the run continues with the planned wall
    state;
  - look poses: at every stop on arrival, after every station trip (the sequencer re-measures) and when the stop is
    complete, the boards are checked: the job's look pose first, then a grid (camera 250-400 mm from the board
    centre, optical axis <= 40 deg to the board normal, all 4 outer corners >= 50 px inside the 2472 x 2064 image),
    IK in motion.family(), collision-free (arm, gripper, camera, adapter), a collision-free transfer from the current
    (parked) pose, unoccluded (render with and without the occluders: < 0.2 % changed pixels on the board) and all
    ChArUco corners detected (mauer.vision.detect); the planned looks are executed on arrival and after each trip;
  - images results/l_top.png, l_corner.png, l_last_stop.png, l_camera_view.png, l_station.png (ARES at the dock on
    the first trip, station full); station saved as robodk/ARES_UR5_Mauer_L.rdk (never robodk/ARES_UR5_Mauer.rdk);
    report results/l_wall_sim.md.

Usage (Windows Python):
    py.exe robodk/simulate.py --animate             # the legs of the config incl. station trips (~25-35 min)
    py.exe robodk/simulate.py --animate --video results/l_wall_sim.mp4    # + time-lapse MP4 (robodk/timelapse.py)
    py.exe robodk/simulate.py --length 16           # straight wall of 16 stones
    py.exe robodk/simulate.py --plan-only           # reach table + plan only
    py.exe robodk/simulate.py --from-stop 2         # L: stops before 2 built without motion, simulate from stop 2

Not simulated: dynamics, jaw motion, tipping, pin engagement forces, ARES drive error (the RoboDK ARES stands exactly
at the nominal stop and dock), the operator's refills of the station (stones appear in the holders), the floor plates
(only the printed boards are objects).
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent, HERE.parent / "tools"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import wallplan  # noqa: E402
from build_station import HALF_STONE, STONE  # noqa: E402
from motion import (Planner, family, jdist, rank, set_held, set_released, set_static,  # noqa: E402
                    stones_in_station)
from rdk_common import (PI, REPO, STATION_NAME, T_TC_CAD, as_mat, box_points, close_instance, connect,  # noqa: E402
                        course_top_z, load_config, look_at, look_at_up, place_pose, project, snapshot_pinhole,
                        stone_dims, stone_mesh_path, stone_points, T_tc_cad, transl, ur5_base_pose, wall_frame)
from reach_study import Checker, frange  # noqa: E402
from robodk.robolink import (COLLISION_OFF, COLLISION_ON, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT,  # noqa: E402
                             ITEM_TYPE_ROBOT, ITEM_TYPE_STATION, ITEM_TYPE_TOOL)
from robodk.robomath import invH, rotx, rotz  # noqa: E402

GHOST = [0.72, 0.30, 0.20, 0.15]
FAILED = [0.90, 0.05, 0.05, 1.0]
FOOTPRINT = [0.20, 0.45, 0.85, 0.35]
# magazine rows: (x relative to the UR base, layers). Pitch 220 mm = stone 120 + room for the open jaws. No row
# closer to the UR: when the base turns, the UR5 upper arm (offset ~136 mm from the base axis) sweeps a circle of
# ~190 mm radius from shoulder height (deck + 89 mm) upwards - a stone row at x = UR - 214 mm (faces 153 mm from the
# axis) lies inside it.
MAG_ROWS = ((-653.6, 3), (-433.6, 3))
MAG_Y = (-205.0, 0.0, 205.0)       # stones end to end along ARES y (jaws do not reach past the stone ends)
MAG_LAYERS = max(n for _, n in MAG_ROWS)
L_RDK = REPO / "robodk" / f"{STATION_NAME}_L.rdk"
L_REPORT = REPO / "results" / "l_wall_sim.md"
LOOK_DISTS = (320.0, 250.0, 300.0, 350.0, 400.0)      # [mm]; 320 = [camera] working_dist first
LOOK_TILTS = (0.0, 15.0, 30.0, 40.0)                  # [deg] optical axis to board normal (<= 40)
OCCL_MAX = 0.002                                      # max. fraction of changed board pixels (occlusion)
DIFF_MIN = 40                                         # grey-level change that counts as "changed"


def reach_table(RDK, cfg: dict, dist: float, side: str | None = None) -> dict:
    """{course: {u_rel: ok}} for the wall at `dist` on `side` of ARES (default [wall] side), cached per configuration
    key, distance and side (mauer.reach_cache, results/reach_table.json; the other tables are kept). Grid u
    -1000..1000 mm in front / behind, -1400..1400 mm on a side (the UR5 sits 354 mm ahead of the ARES centre)."""
    from mauer import reach_cache
    side = str(side or cfg["wall"]["side"])
    table = reach_cache.get(cfg, dist, side=side)
    if table is not None:
        return table
    print(f"computing reach table ({side}, {dist:.0f} mm, about 1 min) ...", flush=True)
    c = copy.deepcopy(cfg)
    c["wall"]["side"] = side
    chk = Checker(RDK, c, True)
    span = 1400.0 if side in ("left", "right") else 1000.0
    us = frange(-span, span, 20.0)
    table = {k: {u: chk.check_both(place_pose(c, u, dist, course_top_z(c, k)))[0] for u in us}
             for k in range(c["wall"]["courses"])}
    chk.close()
    reach_cache.store(cfg, dist, table, side=side)
    return table


class Sim:
    def __init__(self, RDK, cfg: dict, dist: float):
        self.RDK, self.cfg, self.dist = RDK, cfg, dist
        self.robot = RDK.Item("UR5", ITEM_TYPE_ROBOT)
        self.tool = RDK.Item("Gripper_EHPS20", ITEM_TYPE_TOOL)
        self.f_ares = RDK.Item("ARES base_link", ITEM_TYPE_FRAME)
        self.f_wall = RDK.Item("Wall", ITEM_TYPE_FRAME)
        self.ares = RDK.Item("ARES_STEP_2026-09-23", ITEM_TYPE_OBJECT)
        self.station = [s for s in RDK.ItemList(ITEM_TYPE_STATION) if s.Name() == STATION_NAME][0]
        self.mesh = str(REPO / cfg["brick"]["mesh"])
        self.deck = cfg["ares"]["deck_top_z"] + cfg["deck"]["holder_z"]
        self.H = cfg["brick"]["height"]
        self.magazine: list = []
        self.n_new = 0
        self.planner = Planner(RDK, cfg, self.robot, self.tool, self.f_ares)
        self.cams = [c for c in (RDK.Item("Camera_body", ITEM_TYPE_OBJECT), RDK.Item("Camera_adapter_018660",
                                                                                    ITEM_TYPE_OBJECT)) if c.Valid()]

    def cam_pairs(self, stones: list, on: bool) -> None:
        """Camera body / adapter (objects on the tool) against stones: RoboDK leaves these pairs off by default."""
        if not stones or not self.cams:
            return
        a = [c for c in self.cams for _ in stones]
        b = [s for _ in self.cams for s in stones]
        self.RDK.setCollisionActivePairList([COLLISION_ON if on else COLLISION_OFF] * len(a), a, b,
                                            [0] * len(a), [0] * len(a))

    def arm_pairs(self, st, on: bool) -> None:
        """Gripper and all arm links against a stone (motion.set_held leaves gripper / wrist 3 off after a grip, so
        without this a placed stone is never checked against the gripper again); on = motion.set_released."""
        if on:
            set_released(self.RDK, st, self.robot, self.tool)
            return
        self.RDK.setCollisionActivePair(COLLISION_OFF, st, self.tool, 0, 0)
        for link in range(0, 7):
            self.RDK.setCollisionActivePair(COLLISION_OFF, st, self.robot, 0, link)

    def clear(self) -> None:
        for it in self.RDK.ItemList(ITEM_TYPE_OBJECT):
            if it.Name().startswith(("Stone_", "_held_stone", "Stones_", "Wall_plan", "ARES_at_")):
                it.Delete()
        ghost = self.RDK.Item("Wall_nominal", ITEM_TYPE_OBJECT)
        if ghost.Valid():
            ghost.setVisible(False)
        self.f_ares.setPose(transl(0, 0, 0))
        self.f_wall.setParentStatic(self.station)
        self.RDK.setCollisionActive(COLLISION_ON)

    def new_stone(self, parent, pose, name: str, kind: str = "full"):
        """Stone object (CAD mesh of its type) under `parent` at `pose` (CAD frame). Collision model with clearance:
        1 mm smaller per end face, ~0.9 mm per ribbed face (scaled about the stone centre), otherwise neighbours with
        0 mm head joint touch over the whole descent."""
        path = stone_mesh_path(self.cfg, kind) if kind != "full" else Path(self.mesh)
        if path is None:
            raise RuntimeError("half stone mesh missing - run py.exe robodk/make_half_stone.py")
        st = self.RDK.AddFile(str(path), parent)
        L, W, _ = stone_dims(self.cfg, kind)
        st.Scale([0.985, 1.0 - 2.0 / L, 1.0], transl(-W / 2, L / 2, 0))
        self.n_new += 1
        st.setName(f"Stone_{name}_{self.n_new:03d}")
        st.setPose(pose)
        st.setColor(STONE if kind == "full" else HALF_STONE)
        return st

    def stack_top(self) -> float:
        tops = [m[1].Pos()[2] for m in self.magazine]
        return max(tops) if tops else self.deck

    def fill_magazine(self, slots: list) -> None:
        self.magazine = []
        for layer, x, y in slots:
            p_tc = transl(x, y, self.deck + layer * self.H + (layer - 1) * self.cfg["brick"].get("bed_joint", 0.0)) \
                * rotz(PI / 2)
            st = self.new_stone(self.f_ares, p_tc * T_TC_CAD, "mag")
            self.magazine.append((st, p_tc * rotx(PI)))
        self.magazine.sort(key=lambda m: -m[1].Pos()[2])
        allst = stones_in_station(self.RDK)
        for st, _ in self.magazine:
            set_static(self.RDK, st, allst, self.ares)
        self.cam_pairs([m[0] for m in self.magazine], True)

    def safe_z(self, full: bool = False) -> float:
        """TCP height that carries a stone (incl. pins) 60 mm over the magazine (full: over a full magazine)."""
        top = self.deck + MAG_LAYERS * self.H if full else self.stack_top()
        return top + self.H + self.cfg["brick"]["pin_length"] + 60.0

    def go(self, moves) -> None:
        self.planner.execute(moves)

    def uncovered(self) -> list:
        """Magazine entries with no stone on top of them, highest first."""
        out = []
        for st, pick in self.magazine:
            x, y, z = pick.Pos()
            if not any(abs(p.Pos()[0] - x) < 1 and abs(p.Pos()[1] - y) < 1 and p.Pos()[2] > z + 1
                       for _, p in self.magazine):
                out.append((st, pick))
        return sorted(out, key=lambda m: -m[1].Pos()[2])

    def plan_pick(self):
        """Magazine stone with a collision-free path. The magazine is emptied layer by layer (only stones of the
        current top layer are eligible) so that no towers remain in the way of the arm; within the layer the
        stone with the first collision-free path (closest arm configuration) is taken."""
        P, r = self.planner, self.robot
        P.z_safe = self.safe_z()
        free = self.uncovered()
        if not free:
            return None
        z_top = free[0][1].Pos()[2]
        layers = [[m for m in free if abs(m[1].Pos()[2] - z_top) < 1]]
        layers.append([m for m in free if abs(m[1].Pos()[2] - z_top) >= 1])      # fallback only
        for group in layers:
            for st, pick in group:
                res = P.plan_to(r.Joints().list(), pick)
                if res:
                    return st, pick, res
        return None

    def pick_and_place(self, target_world) -> bool:
        P, r = self.planner, self.robot
        found = self.plan_pick()
        if not found:
            self.park()
            found = self.plan_pick()
        if not found:
            print("   no collision-free path to any magazine stone", flush=True)
            return False
        stone, pick, (moves, j_t, pose) = found
        self.go(moves)
        self.magazine = [m for m in self.magazine if m[0] != stone]
        self.cam_pairs([stone], False)
        set_held(self.RDK, stone, stones_in_station(self.RDK), self.ares, r, self.tool)
        stone.setParentStatic(self.tool)                                     # grip
        ret = P.plan_retreat(j_t, pose)          # up to the approach height; the transfer planner lifts further
        if not ret:
            print("   retreat from the magazine not collision-free", flush=True)
            return False
        self.go(ret[0])

        place = invH(self.f_ares.Pose()) * target_world
        P.z_safe = self.safe_z()
        res = P.plan_to(r.Joints().list(), place)
        if not res:
            self.park()
            res = P.plan_to(r.Joints().list(), place)
        if not res:
            print("   no collision-free path to the wall position", flush=True)
            return False
        moves, j_t, pose = res
        self.go(moves)
        stone.setParentStatic(self.f_wall)                                   # release
        stone.setName(stone.Name().replace("_mag_", "_wall_"))
        set_static(self.RDK, stone, stones_in_station(self.RDK), self.ares)
        self.cam_pairs([stone], True)
        ret = P.plan_retreat(j_t, pose)
        if not ret:
            print("   retreat from the wall not collision-free", flush=True)
            return False
        self.go(ret[0])
        self.arm_pairs(stone, True)                                          # jaws clear of the stone now
        return True

    def park(self, full: bool = False) -> None:
        """Compact pose above the UR base, reached on a collision-free path (full: high enough for a refill)."""
        P = self.planner
        P.z_safe = self.safe_z(full)
        j = self.robot.Joints().list()
        with P._planning():
            jp = P.compact(0.0, j)
        if jp is None:
            raise RuntimeError("park pose not reachable")
        path = P.transfer(j, jp)
        if path is None:
            raise RuntimeError("no collision-free path to the park pose")
        self.go([("J", q) for q in path] + [("J", jp)])

    def drive(self, delta_world: list, seconds: float = 3.0) -> None:
        start = self.f_ares.Pose()
        for i in range(1, 61):
            f = i / 60
            self.f_ares.setPose(transl(delta_world[0] * f, delta_world[1] * f, 0) * start)
            self.RDK.Render(True)
            time.sleep(seconds / 60)


# ── straight wall ─────────────────────────────────────────────────────────────
def run_straight(args, cfg: dict, RDK) -> int:
    """The straight-wall simulation (unchanged procedure, now in the own instance)."""
    mag_rows = [(cfg["ur5"]["mount_x"] + dx, n) for dx, n in MAG_ROWS]
    sim = Sim(RDK, cfg, args.dist)
    sim.clear()
    RDK.Render(False)
    table = reach_table(RDK, cfg, args.dist)

    def reach(k: int, u: float) -> bool:
        return table[k].get(round(round(u / 20.0) * 20.0, 3), False)

    lo = {k: min(u for u, ok in table[k].items() if ok) for k in table}
    print("reach per course (u relative to ARES): " + ", ".join(
        f"c{k} {lo[k]:.0f}..{max(u for u, ok in table[k].items() if ok):.0f} ({sum(table[k].values())} of {len(table[k])} ok)"
        for k in table), flush=True)
    stones = wallplan.layout(cfg, args.length)
    a0 = stones[0].u - lo[0]
    plan = wallplan.sequence(cfg, stones, reach, lo, a0)
    errors = wallplan.check_plan(cfg, stones, plan)
    if errors:
        raise SystemExit("invalid plan: " + "; ".join(errors))
    print(f"wall: {len(stones)} stones in {cfg['wall']['courses']} courses, {len(plan)} stops: "
          + ", ".join(f"{len(b)} stones" for _, b in plan)
          + "; ARES moves " + ", ".join(f"{(plan[i + 1][0] - plan[i][0]):.0f} mm" for i in range(len(plan) - 1)),
          flush=True)

    # magazine slots that the UR5 can serve (checked like the wall positions)
    chk = Checker(RDK, cfg, True)
    slots = [(lay, x, y) for x, n in mag_rows for lay in range(1, n + 1) for y in MAG_Y
             if chk.check_both(transl(x, y, sim.deck + lay * sim.H + (lay - 1) * cfg["brick"].get("bed_joint", 0.0)
                                      + 2.0) * rotz(PI / 2) * rotx(PI))[0]]
    chk.close()
    print(f"magazine: {len(slots)} reachable slots", flush=True)
    if args.plan_only:
        RDK.Render(True)
        return 0

    # wall frame: origin at the start of the wall, ARES centre at wall position a0
    sim.f_wall.setPose(wall_frame(cfg, args.dist) * transl(-a0, 0, 0))
    ghost = RDK.AddShape(as_mat([p for s in stones for p in stone_points(cfg, "wall", u=s.u, z_top=s.z_top)]))
    ghost.setParent(sim.f_wall)
    ghost.setName("Wall_plan")
    ghost.setColor(GHOST)
    for link in range(0, 8):                       # the plan ghost is not an obstacle
        RDK.setCollisionActivePair(0, sim.robot, ghost, link, 0)
    RDK.setCollisionActivePair(0, sim.tool, ghost, 0, 0)

    sim.fill_magazine(slots)
    sim.planner.z_safe = sim.safe_z()
    j_start = sim.planner.compact(0.0, [0, -100, 52, -42, -90, 0])   # collision-free start pose above the base
    if j_start is None:
        raise SystemExit("no collision-free start pose found")
    sim.robot.setJoints(j_start)
    RDK.Render(True)
    RDK.setSimulationSpeed(args.speed)
    sim.robot.setSpeed(400, 120, 1500, 600)        # mm/s, deg/s, mm/s2, deg/s2 (simulation only)
    travel = wall_frame(cfg, args.dist).VX()
    t0, placed, a_now = time.time(), 0, a0
    try:
        for j, (a, batch) in enumerate(plan):
            if a != a_now:
                sim.park()
                print(f"ARES drives {a - a_now:.0f} mm sideways", flush=True)
                sim.drive([travel[0] * (a - a_now), travel[1] * (a - a_now)])
                a_now = a
            print(f"stop {j + 1}: {len(batch)} stones", flush=True)
            for s in batch:
                if not sim.magazine:
                    sim.park(full=True)
                    print("   magazine empty -> reload (simulated)", flush=True)
                    sim.fill_magazine(slots)
                ok = sim.pick_and_place(sim.f_wall.Pose() * transl(s.u, 0, s.z_top) * rotx(PI))
                if not ok:
                    print(f"   FAILED at stone course {s.course}, u = {s.u:.0f} mm - stopping", flush=True)
                    return 1
                placed += 1
                print(f"   placed c{s.course} u={s.u:5.0f}  (collision tests so far: {sim.planner.tests})", flush=True)
        sim.park()
    finally:
        RDK.Render(True)
        RDK.setSimulationSpeed(1)
        print(f"done: {placed}/{len(stones)} stones, {sim.planner.tests} collision tests, "
              f"{time.time() - t0:.0f} s wall-clock", flush=True)
    return 0


# ── wall of legs (L, C) ───────────────────────────────────────────────────────
def _np():
    import numpy as np
    return np


def pose2d_T(p):
    """RoboDK Mat of a mauer.reference.Pose2D (ARES in the wall frame)."""
    return transl(p.x_mm, p.y_mm, 0) * rotz(p.theta_rad)


def interpolate(a, b, step_mm: float = 100.0, step_deg: float = 5.0) -> list:
    """Poses from a (excl.) to b (incl.): straight translation and/or rotation on the spot (shortest way)."""
    from mauer.reference import Pose2D, wrap_angle
    d = math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)
    dth = wrap_angle(b.theta_rad - a.theta_rad)
    n = max(1, int(math.ceil(max(d / step_mm, abs(math.degrees(dth)) / step_deg))))
    return [Pose2D(a.x_mm + (b.x_mm - a.x_mm) * i / n, a.y_mm + (b.y_mm - a.y_mm) * i / n,
                   a.theta_rad + dth * i / n) for i in range(1, n + 1)]


def magazine_events(job) -> tuple[list, dict]:
    """(initial magazine fill [(slot, kind)] bottom first, {global stone index: reload before it}) with a reload =
    {"pairs": [(station slot, magazine slot, kind), ...] in fill order, "refill": the operator tops the station up
    first} - the reload rule of tools/make_job.py plan_slots / the sequencer (mauer.job.reload_plan, reload_short);
    the station starts full."""
    from mauer import job as mjob
    mag = mjob.SlotState.magazine(job.magazine)
    init = sorted(mag.filled, key=lambda s: (mag.layer[s], s))
    initial = [(s, mag.kinds.get(s, "full")) for s in init]
    full_station = mjob.SlotState.station(job.station)
    st = full_station.copy()
    stones = job.stones()
    kinds = [t.kind for t in stones]
    events = {}
    for j, t in enumerate(stones):
        if mag.empty():
            refill = mjob.reload_short(st, full_station, mag, kinds[j:])
            if refill:
                st = full_station.copy()
            pairs = mjob.reload_plan(mag, st, kinds[j:])
            for ssid, mid, kind in pairs:
                st.take(ssid)
                mag.fill(mid, kind)
            events[j] = {"pairs": pairs, "refill": refill}
        sid = t.slot or mag.next_take(kind=t.kind)
        if sid is None or not mag.can_take(sid) or mag.kinds.get(sid) != t.kind:
            raise RuntimeError(f"job slot plan inconsistent at stone {t.label}: slot {sid}")
        mag.take(sid)
    return initial, events


def pair_label(item, link: int) -> str:
    name = item.Name()
    if item.Type() == ITEM_TYPE_ROBOT:
        return f"{name} link {link}"
    for pre, lab in (("Stone_wall_", "wall stone "), ("Stone_mag_", "magazine stone "),
                     ("Stone_station_", "station stone "), ("Board_", "board ")):
        if name.startswith(pre):
            rest = name[len(pre):]
            return lab + (rest.rsplit("_", 1)[0] if pre != "Board_" else rest)
    return {"ARES_STEP_2026-09-23": "ARES", "Gripper_EHPS20": "gripper", "Camera_body": "camera body",
            "Camera_adapter_018660": "camera adapter", "Pickup_table": "pick-up table"}.get(name, name)


class LSim(Sim):
    """The wall of legs from the job (see the module docstring)."""

    def __init__(self, RDK, cfg: dict, it: dict, job, args):
        super().__init__(RDK, cfg, args.dist)
        from mauer import config as mconfig
        from mauer import geometry as g
        from mauer.reference import placements
        from mauer.vision.intrinsics import nominal
        from mauer.vision.targets import board_specs
        self.g = g
        self.it, self.job, self.args = it, job, args
        self.cams = [it[k] for k in ("cam_body", "cam_adapter") if k in it]
        self.boards = it.get("boards", {})
        self.table = it.get("table")
        self.T_ab = ur5_base_pose(cfg)
        np = _np()
        self.np = np
        self.T_ab_np = np.asarray(job.T_ares_base, float)
        self.T_fc_np = mconfig.T_flange_cam_nominal(cfg)
        self.T_fc = g.to_robodk(self.T_fc_np)
        self.specs = board_specs(cfg)
        self.wall_boards = {p.name: p.T_parent_board for p in placements(cfg) if p.parent == "wall"}
        self.T_ws_np = np.asarray(job.station.T_wall_station, float)
        self.station_boards = {p.name: self.T_ws_np @ p.T_parent_board for p in placements(cfg)
                               if p.parent == "station"}
        self.board_T = {**self.wall_boards, **self.station_boards}          # wall frame
        self.intr = nominal(cfg)
        self.K = self.intr.K
        self.Wimg, self.Himg = int(cfg["camera"]["res_x"]), int(cfg["camera"]["res_y"])
        self.slot_T = {s.id: g.to_robodk(s.T_ares_tcp) for s in job.magazine.slots}
        self.mag: dict = {}                 # slot -> (item, kind)
        self.f_station = it.get("f_station")
        self.st_slot_T = {s.id: g.to_robodk(s.T_station_tcp) for s in job.station.slots}    # station frame
        self.st: dict = {}                  # station holder -> (item, kind)
        self.st_boxes: dict = {}            # station holder -> (lo, hi) solid box in the wall frame
        self.transfer_log: list = []        # station -> magazine moves
        self.trip_log: list = []
        self.wall_items: dict = {}          # stone key -> item
        self.wall_boxes: dict = {}          # stone key -> (lo, hi) solid box in the wall frame
        self.pose = None                    # ARES Pose2D in the wall frame
        self.stone_log: list = []
        self.route_log: list = []
        self.look_log: list = []
        self.notes: list = []
        self.cam = None
        self.grid = None
        self.j_home = None
        self.rec = None                     # timelapse.Recorder (--video)
        self.t0 = time.time()

    def caption(self, text: str) -> None:
        """Caption of the next time-lapse frames (--video)."""
        if self.rec is not None:
            self.rec.label = text
        self.motion_s = 0.0

    # ── setup ────────────────────────────────────────────────────────────────
    def setup(self) -> None:
        self.clear()
        self.f_wall.setPose(transl(0, 0, 0))                 # world = wall frame
        self.set_ares(self.job.stops[0].ares)
        self.RDK.setCollisionActivePair(COLLISION_OFF, self.robot, self.ares, 0, 0)

    def set_ares(self, p) -> None:
        self.f_ares.setPose(pose2d_T(p))
        self.pose = p

    def statics(self) -> list:
        return (list(self.wall_items.values()) + list(self.boards.values()) + ([self.table] if self.table else [])
                + [m[0] for m in self.st.values()])

    def route_pairs(self, on: bool) -> None:
        """ARES chassis and magazine stones against the built wall, the boards and the table - only while ARES
        drives (resting stones touch ARES / each other by design, see motion.set_static)."""
        movers = [self.ares] + [m[0] for m in self.mag.values()]
        statics = [s for s in self.statics() if s.Visible()]
        a = [m for m in movers for _ in statics]
        b = [s for _ in movers for s in statics]
        if a:
            self.RDK.setCollisionActivePairList([COLLISION_ON if on else COLLISION_OFF] * len(a), a, b,
                                                [0] * len(a), [0] * len(a))

    def pairs(self) -> list[str]:
        out = []
        for a, b, ia, ib in self.RDK.CollisionPairs():
            out.append(" / ".join(sorted((pair_label(a, ia), pair_label(b, ib)))))
        return sorted(set(out))

    # ── stones ───────────────────────────────────────────────────────────────
    def stack_top(self) -> float:
        tops = [self.slot_T[s].Pos()[2] for s in self.mag]
        return max(tops) if tops else self.deck

    def fill_slot(self, sid: str, kind: str) -> None:
        T = self.slot_T[sid]
        st = self.new_stone(self.f_ares, T * rotx(PI) * T_tc_cad(self.cfg, kind), f"mag_{sid}", kind)
        set_static(self.RDK, st, stones_in_station(self.RDK), self.ares)
        self.cam_pairs([st], True)
        self.mag[sid] = (st, kind)

    def table_pairs(self, stones: list, on: bool) -> None:
        """Stones against the pick-up table: off while they stand on it (contact by design), on while held."""
        if self.table is None or not stones:
            return
        n = len(stones)
        self.RDK.setCollisionActivePairList([COLLISION_ON if on else COLLISION_OFF] * n, stones, [self.table] * n,
                                            [0] * n, [0] * n)

    def fill_station(self, sids) -> None:
        """The operator puts stones into the given station holders (no motion), bottom layer first."""
        np, g = self.np, self.g
        stn = self.job.station
        for sid in sorted(sids, key=lambda i: (stn.slot(i).layer, i)):
            if sid in self.st:
                continue
            slot = stn.slot(sid)
            st = self.new_stone(self.f_station, self.st_slot_T[sid] * rotx(PI) * T_tc_cad(self.cfg, slot.kind),
                                f"station_{sid}", slot.kind)
            set_static(self.RDK, st, stones_in_station(self.RDK), self.ares)
            self.table_pairs([st], False)
            self.cam_pairs([st], True)
            self.st[sid] = (st, slot.kind)
            L, W, H = stone_dims(self.cfg, slot.kind)
            c = np.array([[x, y, z] for x in (-L / 2, L / 2) for y in (-W / 2, W / 2) for z in (0.0, H)])
            p = g.apply(self.T_ws_np @ slot.T_station_tcp, c)              # TCP z points into the stone
            self.st_boxes[sid] = (p.min(0), p.max(0))

    def station_safe_z(self) -> float:
        """Transfer height at the dock: a held stone (incl. pins) 60 mm over the magazine as it is and over the
        highest station holder (the station frame lies at floor level like the ARES frame)."""
        top = max(T.Pos()[2] for T in self.st_slot_T.values())
        return max(self.safe_z(), top + self.H + self.cfg["brick"]["pin_length"] + 60.0)

    def wall_pose(self, task):
        """CAD-frame pose (wall frame) of a placed stone of the job."""
        return self.g.to_robodk(task.T_wall_tcp) * rotx(PI) * T_tc_cad(self.cfg, task.kind)

    def put_in_wall(self, st, task, failed: bool = False) -> None:
        """Stone resting in the wall: no checks against ARES / other stones (contacts by design), camera objects on;
        gripper and wrist (off while it was held, motion.set_held) back on via arm_pairs after the retreat."""
        np = self.np
        if st.Parent().item != self.f_wall.item:
            # setParentStatic keeps the ABSOLUTE pose - with rendering off that pose of a new item is stale (identity)
            # until the station is updated (observed 2026-10-05: prebuilt stones jumped to the wall origin)
            self.RDK.Update()
            st.setParentStatic(self.f_wall)
        if failed:
            st.setPose(self.wall_pose(task))
            st.setColor(FAILED)
        st.setName(f"Stone_wall_{task.label}_{self.n_new:03d}")
        set_static(self.RDK, st, stones_in_station(self.RDK), self.ares)
        self.cam_pairs([st], True)
        self.wall_items[task.key] = st
        L, W, H = stone_dims(self.cfg, task.kind)
        c = np.array([[x, y, z] for x in (-L / 2, L / 2) for y in (-W / 2, W / 2) for z in (0.0, H)])
        p = self.g.apply(np.asarray(task.T_wall_tcp, float), c)            # TCP z points into the stone
        self.wall_boxes[task.key] = (p.min(0), p.max(0))
        if failed:
            self.arm_pairs(st, True)

    def prebuild(self, tasks: list) -> None:
        """Stones put in place without motion (--from-stop)."""
        for t in tasks:
            st = self.new_stone(self.f_wall, self.wall_pose(t), f"wall_{t.label}", t.kind)
            self.put_in_wall(st, t)

    # ── diagnosis ────────────────────────────────────────────────────────────
    def diagnose(self, target, j_from) -> tuple[str, str]:
        """(category, details) why plan_to(j_from, target) found no motion."""
        P = self.planner
        with P._planning():
            sols = P.ik_all(target)
            fam = rank(sols, j_from)
            if not sols:
                return "reach hole", "no IK solution for the TCP pose"
            if not fam:
                return "reach hole", f"{len(sols)} IK solutions, none in motion.family()"
            reasons = []
            for j_t, pose in fam[:4]:
                pre = transl(0, 0, P.engage) * pose
                app = transl(0, 0, P.approach) * pose
                j_pre, j_app = P.ik(pre, j_t), P.ik(app, j_t)
                if not (j_pre and j_app):
                    reasons.append("approach pose without IK")
                    continue
                if (max(abs(a - b) for a, b in zip(j_app, j_t)) > 60
                        or max(abs(a - b) for a, b in zip(j_pre, j_t)) > 30):
                    reasons.append("configuration change on the vertical approach")
                    continue
                if P.self_why(j_app):
                    reasons.append("approach pose (150 mm above): " + P.self_why(j_app))
                    continue
                if not P.self_free_j(j_app, j_t):
                    reasons.append("vertical approach: tool closer than the tool-vs-arm limit to the arm")
                    continue
                if not P.state_free(j_app):
                    reasons.append("approach pose (150 mm above) in collision: " + ", ".join(self.pairs()))
                    continue
                self.robot.setJoints(j_app)
                r = self.robot.MoveL_Test(j_app, pre, 2.0)
                if r != 0:
                    self.RDK.Update()
                    pr = self.pairs() if r > 0 else [f"MoveL_Test {r}"]
                    reasons.append("vertical approach in collision: " + ", ".join(pr))
                    continue
                r = self.robot.MoveJ_Test(j_from, j_app, 1.0)
                self.RDK.Update()
                pr = self.pairs() if r > 0 else [f"MoveJ_Test {r}"]
                reasons.append("transfer (direct, lift, compact, RRT) blocked; direct move: " + ", ".join(pr))
            text = "; ".join(dict.fromkeys(reasons))
        cat = "collision"
        if "camera" in text:
            cat = "camera/adapter collision"
        elif "wall stone A" in text and "wall stone B" not in text:
            cat = "collision with leg A"
        elif "wall stone" in text:
            cat = "collision with the wall"
        elif "configuration" in text or "without IK" in text:
            cat = "reach (approach)"
        return cat, text

    def diag_retreat(self) -> str:
        """Pairs at the first collision of the tested part of a vertical retreat from the current joints."""
        P = self.planner
        with P._planning():
            j = self.robot.Joints().list()
            pose = P.fk(j)
            pre = transl(0, 0, P.engage) * pose
            j_pre = P.ik(pre, j)
            if not j_pre:
                return "no IK 60 mm above"
            r = self.robot.MoveL_Test(j_pre, transl(0, 0, P.approach) * pose, 2.0)
            self.RDK.Update()
            return ", ".join(self.pairs()) if r > 0 else f"MoveL_Test {r}"

    def fail(self, rec: dict, phase: str, target, j_from, st=None, task=None) -> dict:
        cat, why = (self.diagnose(target, j_from) if target is not None
                    else ("collision", "retreat blocked: " + self.diag_retreat()))
        rec.update(ok=False, phase=phase, category=cat, why=why)
        print(f"   FAILED {rec['label']} ({phase}): {cat}: {why}", flush=True)
        if task is not None:
            if st is None:
                st = self.new_stone(self.f_wall, self.wall_pose(task), f"wall_{task.label}", task.kind)
            self.put_in_wall(st, task, failed=True)
        self.robot.setJoints(self.j_home)              # teleport back (failure: no motion)
        rec["teleported"] = True
        return rec

    # ── one stone ────────────────────────────────────────────────────────────
    def lay(self, task) -> dict:
        P, r = self.planner, self.robot
        rec = {"label": task.label, "leg": task.leg, "course": task.course, "index": task.index, "kind": task.kind,
               "slot": task.slot, "ok": False}
        st, kind = self.mag[task.slot]
        if kind != task.kind:
            raise RuntimeError(f"slot {task.slot} holds a {kind} stone, {task.label} needs {task.kind}")
        pick = self.slot_T[task.slot]
        t_m = time.time()
        P.z_safe = self.safe_z()
        res = P.plan_to(r.Joints().list(), pick)
        if not res:
            self.park_safe()
            res = P.plan_to(r.Joints().list(), pick)
        if not res:
            del self.mag[task.slot]
            return self.fail(rec, "pick", pick, r.Joints().list(), st, task)
        moves, j_t, pose = res
        self.go(moves)
        del self.mag[task.slot]
        self.cam_pairs([st], False)
        set_held(self.RDK, st, stones_in_station(self.RDK), self.ares, r, self.tool)
        self.RDK.Update()                                  # fresh absolute poses before re-parenting (see put_in_wall)
        st.setParentStatic(self.tool)                                         # grip
        ret = P.plan_retreat(j_t, pose)
        if not ret:
            return self.fail(rec, "retreat from the magazine", None, None, st, task)
        self.go(ret[0])
        place = invH(self.f_ares.Pose()) * self.f_wall.Pose() * self.g.to_robodk(task.T_wall_tcp)
        P.z_safe = self.safe_z()
        res = P.plan_to(r.Joints().list(), place)
        if not res:
            self.park_safe()
            res = P.plan_to(r.Joints().list(), place)
        if not res:
            return self.fail(rec, "place", place, r.Joints().list(), st, task)
        moves, j_t, pose = res
        self.go(moves)
        self.put_in_wall(st, task)                                            # release
        ret = P.plan_retreat(j_t, pose)
        if not ret:
            self.arm_pairs(st, True)
            return self.fail(rec, "retreat from the wall", None, None, None, None)
        self.go(ret[0])
        self.arm_pairs(st, True)                                              # jaws clear of the stone now
        rec["ok"] = True
        self.motion_s += time.time() - t_m
        return rec

    def park_safe(self, full: bool = False) -> None:
        """park(); if no collision-free path exists, note it and put the arm in the home pose without motion."""
        try:
            self.park(full)
        except RuntimeError as e:
            self.notes.append(f"park: {e} (arm put in the home pose without motion)")
            print(f"   PARK FAILED: {e}", flush=True)
            self.robot.setJoints(self.j_home)

    # ── station -> magazine ──────────────────────────────────────────────────
    def transfer(self, ssid: str, mid: str, kind: str) -> dict:
        """One stone station holder -> magazine slot with collision-checked motion (as lay(), ARES at the dock)."""
        P, r = self.planner, self.robot
        rec = {"station_slot": ssid, "slot": mid, "kind": kind, "ok": False}
        st, k_ = self.st[ssid]
        if k_ != kind:
            raise RuntimeError(f"station holder {ssid} holds a {k_} stone, the reload plan needs {kind}")
        t_m = time.time()
        pick = invH(self.f_ares.Pose()) * self.f_wall.Pose() * self.g.to_robodk(self.T_ws_np) * self.st_slot_T[ssid]
        P.z_safe = self.station_safe_z()
        res = P.plan_to(r.Joints().list(), pick)
        if not res:
            self.park_safe(full=True)
            res = P.plan_to(r.Joints().list(), pick)
        if not res:
            return self.transfer_fail(rec, "pick at the station", pick, r.Joints().list())
        moves, j_t, pose = res
        self.go(moves)
        del self.st[ssid]
        self.st_boxes.pop(ssid, None)
        self.cam_pairs([st], False)
        set_held(self.RDK, st, stones_in_station(self.RDK), self.ares, r, self.tool)
        self.table_pairs([st], True)
        self.RDK.Update()                                  # fresh absolute poses before re-parenting (see put_in_wall)
        st.setParentStatic(self.tool)                                         # grip
        ret = P.plan_retreat(j_t, pose)
        if not ret:
            return self.transfer_fail(rec, "retreat from the station", None, None, held=st)
        self.go(ret[0])
        place = self.slot_T[mid]
        P.z_safe = self.station_safe_z()
        res = P.plan_to(r.Joints().list(), place)
        if not res:
            self.park_safe(full=True)
            res = P.plan_to(r.Joints().list(), place)
        if not res:
            return self.transfer_fail(rec, "place in the magazine", place, r.Joints().list(), held=st)
        moves, j_t, pose = res
        self.go(moves)
        self.RDK.Update()
        st.setParentStatic(self.f_ares)                                       # release on the deck
        st.setName(st.Name().replace("Stone_station_", "Stone_mag_").replace(ssid, mid))
        set_static(self.RDK, st, stones_in_station(self.RDK), self.ares)
        self.cam_pairs([st], True)
        self.mag[mid] = (st, kind)
        ret = P.plan_retreat(j_t, pose)
        if not ret:
            self.arm_pairs(st, True)
            return self.transfer_fail(rec, "retreat from the magazine", None, None)
        self.go(ret[0])
        self.arm_pairs(st, True)                                              # jaws clear of the stone now
        rec["ok"] = True
        self.motion_s += time.time() - t_m
        return rec

    def transfer_fail(self, rec: dict, phase: str, target, j_from, held=None) -> dict:
        """Diagnose a failed transfer; the run goes on with the planned magazine: the stone is put into its magazine
        slot without motion (red), the arm back to the home pose."""
        cat, why = (self.diagnose(target, j_from) if target is not None
                    else ("collision", "retreat blocked: " + self.diag_retreat()))
        rec.update(ok=False, phase=phase, category=cat, why=why)
        print(f"   FAILED station {rec['station_slot']} -> magazine {rec['slot']} ({phase}): {cat}: {why}", flush=True)
        for item in [held] + ([self.st.pop(rec["station_slot"])[0]] if rec["station_slot"] in self.st else []):
            if item is not None and item.Valid():
                item.Delete()
        self.st_boxes.pop(rec["station_slot"], None)
        if rec["slot"] not in self.mag:
            self.fill_slot(rec["slot"], rec["kind"])
            self.mag[rec["slot"]][0].setColor(FAILED)
        self.robot.setJoints(self.j_home)
        rec["teleported"] = True
        return rec

    # ── routes ───────────────────────────────────────────────────────────────
    def drive_route(self, route: list, what: str, animate: bool) -> dict:
        """Move ARES along the waypoints (wall frame); every sample checked for collisions (route_pairs on)."""
        rec = {"what": what, "waypoints": len(route), "samples": 0, "hits": []}
        if len(route) < 2:
            return rec
        self.route_pairs(True)
        self.RDK.Render(animate)
        de = (what.replace("leg change ", "Schenkelwechsel ").replace("station", "Abholstation")
              .replace("stop ", "Halt ").replace("trip", "Fahrt"))
        self.caption(f"ARES fährt: {de}")
        try:
            for a, b in zip(route, route[1:]):
                for p in interpolate(a, b):
                    self.set_ares(p)
                    rec["samples"] += 1
                    if animate:
                        self.RDK.Render(True)
                        time.sleep(0.02)
                    else:
                        self.RDK.Update()
                    if self.rec is not None:
                        self.rec.frame()
                    if self.RDK.Collisions():
                        rec["hits"].append({"x": round(p.x_mm), "y": round(p.y_mm), "theta_deg": round(p.theta_deg, 1),
                                            "pairs": self.pairs()})
        finally:
            self.route_pairs(False)
        if rec["hits"]:
            print(f"   ROUTE {what}: {len(rec['hits'])} colliding samples, e.g. {rec['hits'][0]}", flush=True)
        self.route_log.append(rec)
        return rec

    def station_trip(self, k: int, n: int, stop, ev: dict, animate: bool, image: Path | None = None) -> dict:
        """Reload n from stop k like mauer.sequencer._reload: (operator top-up), route to the dock, station looks
        (planned ones executed), the planned stones station -> magazine, park, route back. ARES ends at the end of
        route_from_station (the stop); the caller re-checks the wall looks."""
        trip = {"trip": n, "stop": k, "refill": 0, "planned": len(ev["pairs"]), "moved": 0, "looks": {}}
        if ev["refill"]:
            missing = [sid for sid in self.job.station.take_order if sid not in self.st]
            self.fill_station(missing)
            trip["refill"] = len(missing)
            print(f"   operator tops up the station: {len(missing)} stones", flush=True)
        self.drive_route(stop.route_to_station, f"stop {k} -> station (trip {n})", animate=animate)
        self.set_ares(self.job.station.dock_in_wall)
        self.RDK.Render(False)
        planned = {lk.boards[0]: lk for lk in self.job.station.looks}
        self.caption(f"Fahrt {n}: Kamera misst die Tafeln der Abholstation")
        res = self.looks(k, f"station trip {n}", sorted(self.station_boards), planned, execute=True)
        trip["looks"] = {b: bool(r["ok"]) for b, r in res.items()}
        if image is not None:
            trip["image"] = station_image(self, image, n)
        for i, (ssid, mid, kind) in enumerate(ev["pairs"], 1):
            self.caption(f"Fahrt {n}: Abholstation → Magazin, Stein {i}/{len(ev['pairs'])} "
                         f"({'Halbstein' if kind == 'half' else 'Vollstein'})")
            rec = self.transfer(ssid, mid, kind)
            rec.update(trip=n, stop=k)
            self.transfer_log.append(rec)
            if rec["ok"]:
                trip["moved"] += 1
                print(f"   station {ssid:6s} -> magazine {mid}  ({kind}, collision tests so far: "
                      f"{self.planner.tests}, {(time.time() - self.t0) / 60:.1f} min)", flush=True)
        self.park_safe(full=True)
        self.drive_route(stop.route_from_station, f"station -> stop {k} (trip {n})", animate=animate)
        self.trip_log.append(trip)
        return trip

    # ── looks ────────────────────────────────────────────────────────────────
    def open_camera(self) -> None:
        from sim_camera import SimCamera
        self.RDK.Render(True)
        if self.cam is None:
            self.cam = SimCamera(self.RDK, self.cfg, self.it["cam_tool"], robot=self.robot,
                                 hide=[self.it["cam_body"]]).open()
        else:
            self.cam.open()

    def look_grid(self, name: str) -> list:
        if self.grid is None:
            import build_station as bs
            from look_study import view_grid
            spec = next(s for s in bs.board_layout(self.cfg) if s["name"] == name)
            self.grid = view_grid(spec, self.K, self.Wimg, self.Himg, LOOK_DISTS, LOOK_TILTS, 8, 8,
                                  float(self.cfg["camera"]["working_dist"]))
        return self.grid

    def view_metrics(self, T_ac, T_ab_) -> tuple[float, float, float]:
        """(distance to the board centre [mm], angle optical axis / board normal [deg], min corner margin [px])."""
        np, g = self.np, self.g
        bw, bh = self._board_wh
        c = g.apply(T_ab_, np.array([[bw / 2, bh / 2, 0.0]]))[0]
        d = float(np.linalg.norm(c - T_ac[:3, 3]))
        tilt = float(np.degrees(np.arccos(np.clip(T_ac[:3, 2] @ T_ab_[:3, 2], -1.0, 1.0))))
        P = g.apply(g.inv(T_ac) @ T_ab_, np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]], float))
        if np.any(P[:, 2] <= 0):
            return d, tilt, -1.0
        uv = (P @ self.K.T)[:, :2] / P[:, 2:3]
        m = np.minimum.reduce([uv[:, 0], self.Wimg - 1 - uv[:, 0], uv[:, 1], self.Himg - 1 - uv[:, 1]])
        return d, tilt, float(m.min())

    @property
    def _board_wh(self) -> tuple[float, float]:
        ref = self.cfg["boards"]["ref"]
        return ref["squares_x"] * ref["square_mm"], ref["squares_y"] * ref["square_mm"]

    def inside_solid(self, j, T_ac) -> str:
        """RoboDK's mesh check does not report a body completely inside another mesh (look_study.Study.inside):
        camera origin, flange and the flange -> camera line must stay outside the stones' and the ARES chassis' solid
        boxes (shrunk by 2 mm)."""
        np, g = self.np, self.g
        boxes = [((-self.cfg["ares"]["length"] / 2, -self.cfg["ares"]["width"] / 2, 0.0),
                  (self.cfg["ares"]["length"] / 2, self.cfg["ares"]["width"] / 2, self.cfg["ares"]["deck_top_z"]))]
        T_aw = g.inv(self.pose_T_np())
        for lo, hi in list(self.wall_boxes.values()) + list(self.st_boxes.values()):        # wall frame
            corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            p = g.apply(T_aw, corners)
            boxes.append((tuple(p.min(0)), tuple(p.max(0))))
        T_af = T_ac @ g.inv(self.T_fc_np)
        pts = [T_ac[:3, 3], T_af[:3, 3]] + [T_af[:3, 3] + (T_ac[:3, 3] - T_af[:3, 3]) * f for f in (0.25, 0.5, 0.75)]
        for lo, hi in boxes:
            lo, hi = np.array(lo) + 2.0, np.array(hi) - 2.0
            if any(np.all(p > lo) and np.all(p < hi) for p in pts):
                return "camera/flange inside a solid (stone or ARES chassis)"
        return ""

    def pose_T_np(self):
        from mauer.reference import planar_T
        return planar_T(self.pose.x_mm, self.pose.y_mm, self.pose.theta_rad)

    def render_check(self, name: str, T_ac, T_ab_) -> dict:
        """Render at the current joints: occlusion (render diff with the occluders hidden) and ChArUco detection."""
        import cv2
        from mauer.vision import detect
        np, g = self.np, self.g
        img = self.cam.grab_bgr()
        hide = [it for it in ([self.robot, self.tool, self.ares] + self.cams + list(self.wall_items.values())
                              + [m[0] for m in self.mag.values()] + [m[0] for m in self.st.values()]
                              + ([self.table] if self.table else []))
                if it.Visible()]
        for it in hide:
            it.setVisible(False)
        try:
            ref = self.cam.grab_bgr()
        finally:
            for it in hide:
                it.setVisible(True)
        bw, bh = self._board_wh
        P = g.apply(g.inv(T_ac) @ T_ab_, np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]], float))
        uv = (P @ self.K.T)[:, :2] / P[:, 2:3]
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, np.round(uv).astype(np.int32), 1)
        mask = cv2.erode(mask, np.ones((7, 7), np.uint8))
        diff = np.abs(img.astype(np.int16) - ref.astype(np.int16)).max(axis=2)
        occl = float((diff[mask > 0] > DIFF_MIN).mean()) if mask.any() else 1.0
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dets = detect.detect_boards(gray, {name: self.specs[name]}, self.cfg["vision"]["border_px"])
        n = dets[name].n if name in dets else 0
        err_mm = err_deg = float("nan")
        fix_mm = None
        if name in dets and n >= 4:
            bp = detect.estimate_pose(dets[name], self.intr, 4, 5.0)
            if bp.T_cam_board is not None:
                err_mm, err_deg = g.pose_delta(bp.T_cam_board, g.inv(T_ac) @ T_ab_)
            if not err_mm <= 0.2:                 # diagnose: the same solve with a normalised IPPE rotation vector
                o = dets[name].obj_pts.reshape(-1, 1, 3).astype(float)
                i = dets[name].img_pts.reshape(-1, 1, 2).astype(float)
                ok_, rv, tv = cv2.solvePnP(o, i, self.intr.K, self.intr.D, flags=cv2.SOLVEPNP_IPPE)
                if ok_:
                    rv = cv2.Rodrigues(cv2.Rodrigues(rv)[0])[0]
                    rv, tv = cv2.solvePnPRefineLM(o, i, self.intr.K, self.intr.D, rv, tv,
                                                  criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100,
                                                            1e-12))
                    fix_mm = g.pose_delta(g.make_T(cv2.Rodrigues(rv)[0], tv.ravel()), g.inv(T_ac) @ T_ab_)[0]
        need = self.specs[name].n_corners
        why = "occluded" if occl > OCCL_MAX else ("not detected" if n < need else "")
        return {"ok": not why, "why": why, "occl": occl, "corners": n, "need": need, "pnp_mm": err_mm,
                "pnp_deg": err_deg, "pnp_fix_mm": fix_mm, "img": img, "dets": dets}

    def check_board(self, name: str, look=None, max_ik: int = 80, max_transfer: int = 4,
                    max_render: int = 3) -> dict:
        """First valid look pose for board `name` at the current ARES pose and wall state (see module docstring)."""
        np, g, P = self.np, self.g, self.planner
        T_aw = g.inv(self.pose_T_np())
        T_ab_ = T_aw @ self.board_T[name]
        cands = []
        if look is not None and look.T_base_flange is not None:
            cands.append(("job", self.T_ab_np @ np.asarray(look.T_base_flange, float) @ self.T_fc_np))
        cands += [("grid", T_ab_ @ e[5]) for e in self.look_grid(name)]
        j0 = self.robot.Joints().list()
        res = {"board": name, "ok": False, "reason": "no IK in motion.family()", "n_ik": 0, "n_free": 0,
               "source": "", "coll": {}}
        n_tr = n_rd = 0
        P.z_safe = self.safe_z()
        def why(src: str, text: str) -> None:
            res["coll"][text] = res["coll"].get(text, 0) + 1
            if src == "job" and "job_reason" not in res:
                res["job_reason"] = text

        with P._planning():
            for src, T_ac in cands:
                if res["n_ik"] >= max_ik or n_tr >= max_transfer or n_rd >= max_render:
                    break
                d, tilt, margin = self.view_metrics(T_ac, T_ab_)
                if not (250.0 - 1e-6 <= d <= 400.0 + 1e-6 and tilt <= 40.0 + 1e-6 and margin >= 50.0):
                    if src == "job":
                        res["job_reason"] = f"outside the view limits (d {d:.0f} mm, tilt {tilt:.0f} deg, margin {margin:.0f} px)"
                    continue
                sols = self.robot.SolveIK_All(g.to_robodk(T_ac), tool=self.T_fc, reference=P.ref)
                js = []
                if sols.size(0) >= 6:
                    js = [[sols[r, c] for r in range(6)] for c in range(sols.size(1))]
                js = sorted((j for j in js if family(j)), key=lambda j: jdist(j, j0))[:2]
                if not js:
                    if src == "job":
                        res["job_reason"] = "no IK in motion.family()"
                    continue
                res["n_ik"] += 1
                for j in js:
                    if P.self_why(j):
                        why(src, P.self_why(j))
                        continue
                    if not P.state_free(j):
                        for p in self.pairs():
                            why(src, p)
                        continue
                    ins = self.inside_solid(j, T_ac)
                    if ins:
                        why(src, ins)
                        continue
                    res["n_free"] += 1
                    n_tr += 1
                    self.robot.setJoints(j0)
                    try:
                        path = P._transfer(j0, j)
                    except RuntimeError:
                        path = None
                    if path is None:
                        why(src, "no collision-free transfer from the current pose")
                        continue
                    n_rd += 1
                    self.robot.setJoints(j)
                    self.RDK.Update()
                    v = self.render_check(name, T_ac, T_ab_)
                    if not v["ok"] and src == "job":
                        res["job_reason"] = f"{v['why']} (corners {v['corners']}, occl {v['occl'] * 100:.2f} %)"
                    if v["ok"]:
                        res.update(ok=True, reason="ok", source=src, j=j, path=path, d=d, tilt=tilt, margin=margin,
                                   corners=v["corners"], occl=v["occl"], pnp_mm=v["pnp_mm"], pnp_deg=v["pnp_deg"],
                                   pnp_fix_mm=v["pnp_fix_mm"],
                                   T_ac=T_ac, img=v["img"], dets=v["dets"])
                        break
                    why("render", v["why"])
                if res["ok"]:
                    break
        if not res["ok"] and res["coll"]:
            top = sorted(res["coll"].items(), key=lambda kv: -kv[1])[:3]
            res["reason"] = "; ".join(f"{k} ({n})" for k, n in top)
        return res

    def looks(self, k: int, state: str, names: list, planned: dict, execute: bool) -> dict:
        """Check `names` at the current state; execute the planned ones (moves to the look pose and back)."""
        out = {}
        for name in names:
            r = self.check_board(name, planned.get(name))
            out[name] = r
            tag = "planned" if name in planned else "spare"
            print(f"   look {state} {name} ({tag}): {'ok' if r['ok'] else 'FAIL'} "
                  + (f"[{r['source']}, d {r['d']:.0f} mm, tilt {r['tilt']:.0f} deg, margin {r['margin']:.0f} px, "
                     f"{r['corners']} corners, PnP {r['pnp_mm']:.2f} mm]" if r["ok"] else r["reason"])
                  + (f" (job pose: {r['job_reason']})" if r.get("job_reason") and r.get("source") != "job" else ""),
                  flush=True)
            if r["ok"] and getattr(self.args, "save_looks", None):
                import cv2
                from mauer.vision import detect
                d = Path(self.args.save_looks)
                d.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(d / f"stop{k}_{state.replace(' ', '')}_{name}.png"),
                            detect.draw_detections(r["img"], r["dets"], scale=0.5))
            if execute and r["ok"] and name in planned:
                j_now = self.robot.Joints().list()
                with self.planner._planning():
                    path = self.planner._transfer(j_now, r["j"])
                if path is None:
                    self.park_safe()
                    with self.planner._planning():
                        path = self.planner._transfer(self.robot.Joints().list(), r["j"])
                if path is not None:
                    self.go([("J", q) for q in path] + [("J", r["j"])])
                    r["executed"] = True
                else:
                    r["executed"] = False
        self.look_log.append({"stop": k, "state": state, "planned": sorted(planned),
                              "res": {n: {kk: v for kk, v in r.items() if kk not in ("img", "dets", "path", "T_ac")}
                                      for n, r in out.items()}})
        return out

    # ── self-tests ───────────────────────────────────────────────────────────
    def self_test(self) -> dict:
        """Known collisions must be reported (gripper, camera body and ARES against a wall stone; held stone
        against a wall stone), and RoboDK's UR5 base / FK must equal the job's kinematics (mauer.simworld)."""
        np, g, P = self.np, self.g, self.planner
        out = {}
        s0 = self.job.stops[0]
        t0 = min((t for t in s0.stones if t.course == 0), key=lambda t: abs(t.u_mm - s0.a_mm))   # in front of ARES
        st = self.new_stone(self.f_wall, self.wall_pose(t0), "wall_selftest", t0.kind)
        set_static(self.RDK, st, stones_in_station(self.RDK), self.ares)
        self.cam_pairs([st], True)
        T_place = invH(self.f_ares.Pose()) * self.g.to_robodk(t0.T_wall_tcp)
        seed = self.j_home
        with P._planning():
            for name, T, tool in (("gripper TCP 60 mm into a wall stone", transl(0, 0, -60) * T_place, P.tcp),
                                  ("camera body through a wall stone top face",
                                   transl(0, 0, -10) * T_place * rotz(PI / 2), self.T_fc)):
                sols = self.robot.SolveIK_All(T, tool=tool, reference=P.ref)
                js = [[sols[r, c] for r in range(6)] for c in range(sols.size(1))] if sols.size(0) >= 6 else []
                js = sorted((j for j in js if family(j)), key=lambda j: jdist(j, seed))
                if not js:
                    out[name] = "no IK"
                    continue
                self.robot.setJoints(js[0])
                self.RDK.Update()
                out[name] = self.pairs()
            # ARES chassis into the stone (only with the route pairs on): towards the leg of stop 0 (straight ahead
            # for a leg in front of ARES, sideways for one on its side) until the panel facing the leg is 30 mm inside
            # the stone (a stone completely inside the hollow chassis shell would not be reported by the surface
            # check): gap ARES - wall face + 30 mm (250 mm in front at 840 mm, 250 mm on a side at 580 mm)
            p0 = self.pose
            from mauer.reference import Pose2D
            from make_job import leg_dist, leg_side
            lg0 = next((lg for lg in wallplan.legs(self.cfg) if lg.name == s0.leg), None)
            side = leg_side(self.cfg, lg0) if lg0 is not None else self.cfg["wall"]["side"]
            d0 = leg_dist(self.cfg, lg0, self.dist) if lg0 is not None else self.dist
            th = (math.atan2(-math.cos(lg0.theta), math.sin(lg0.theta)) if lg0 is not None    # leg -y (wall frame)
                  else p0.theta_rad)
            half = self.cfg["ares"]["width" if side in ("left", "right") else "length"] / 2
            d_in = d0 - half - self.cfg["brick"]["width"] / 2 + 30.0
            self.wall_items["_selftest"] = st
            self.route_pairs(True)
            self.set_ares(Pose2D(p0.x_mm + math.cos(th) * d_in, p0.y_mm + math.sin(th) * d_in, p0.theta_rad))
            self.robot.setJoints(self.j_home)
            self.RDK.Update()
            out[f"ARES chassis {d_in:.0f} mm towards leg {s0.leg} ({side}), 30 mm into the wall (route pairs on)"] = \
                self.pairs()
            self.route_pairs(False)
            del self.wall_items["_selftest"]
            self.set_ares(p0)
        st.Delete()
        # FK / base frame: RoboDK IK of the job's first look flange pose vs the job's qnear (nominal UR5 DH)
        lk = self.job.stops[0].looks[0]
        j = self.robot.SolveIK(g.to_robodk(np.asarray(lk.T_base_flange, float)), list(np.degrees(lk.qnear_rad)))
        jl = j.list()[:6] if j.size(0) >= 6 else []
        out["RoboDK IK of the job look vs job qnear [deg]"] = (
            round(max(abs(math.remainder(a - b, 360.0)) for a, b in zip(jl, np.degrees(lk.qnear_rad))), 4)
            if jl else "no IK")
        self.robot.setJoints(self.j_home)
        bad = [k for k, v in out.items() if k != "RoboDK IK of the job look vs job qnear [deg]" and not v]
        if bad:
            raise RuntimeError(f"collision self-test failed (not reported): {bad}: {out}")
        return out


def static_findings(cfg: dict) -> dict:
    """Facts computed from the meshes and the config (no RoboDK): rib height of the stone, the butt-corner gap of
    every next leg against the earlier leg's ribbed face (each corner), the full-stone mesh volume
    (orientation-independent)."""
    import make_half_stone as mh
    np = _np()
    full = mh.read_stl(REPO / cfg["brick"]["mesh"])
    V, F = mh.weld(full)
    raw = mh.volume(V, F) / 1000.0
    Fo = mh.orient(V, F)
    vol = mh.volume(V, Fo) / 1000.0
    flipped = sum(1 for a, b in zip(F.tolist(), Fo.tolist())
                  if tuple(a) not in {tuple(b), (b[1], b[2], b[0]), (b[2], b[0], b[1])})
    pts = full.reshape(-1, 3)
    W = float(cfg["brick"]["width"])
    rib = float(max(-pts[:, 0].min(), pts[:, 0].max() - W))
    legs = wallplan.legs(cfg)
    corners = []
    for A, B in zip(legs, legs[1:]):
        # the butting leg's flat end face against the ribbed face of the leg that runs through (wallplan.corner_of,
        # with the rib height measured on the mesh)
        c = wallplan.corner_of({**cfg, "brick": {**cfg["brick"], "rib_mm": rib}}, A, B)
        if c is None:
            raise ValueError(f"legs {A.name} -> {B.name}: not a butt corner (wallplan.corner_of)")
        corners.append({"prev": A.name, "next": B.name, "through": c["through"], "gap_mm": c["gap_mm"]})
    gap = min((c["gap_mm"] for c in corners), default=None)
    return {"rib_mm": rib, "corner_gap_mm": gap, "corners": corners, "volume_cm3": vol, "volume_signed_raw_cm3": raw,
            "flipped_faces": flipped, "faces": len(F)}


# ── images ────────────────────────────────────────────────────────────────────
def shape_of(cfg: dict) -> str:
    """ "C wall", "L wall", ... from [wall] shape (fallback: the number of legs)."""
    shape = str(cfg.get("wall", {}).get("shape", ""))
    return f"{shape} wall" if shape and shape != "straight" else f"wall of {len(wallplan.legs(cfg))} legs"


def station_image(sim: LSim, path: Path, trip: int) -> str | None:
    """ARES at the dock next to the loaded pick-up station, seen from the front right of the table. The snapshot
    closes every camera window, so the flange camera is re-opened afterwards."""
    import cv2
    from mauer.floor import station_table_extent
    np, g = sim.np, sim.g
    x_max, y_max = station_table_extent(sim.cfg)
    z_tab = float(sim.cfg["pickup_station"]["table_z"])
    w = lambda x, y, z: [float(c) for c in g.apply(sim.T_ws_np, np.array([[x, y, z]]))[0]]   # noqa: E731
    T_v = look_at(w(x_max + 1100.0, -1500.0, 1900.0), w(x_max / 2, -420.0, z_tab))
    ok, _ = snapshot_pinhole(sim.RDK, path, T_v, (1600, 1000), 55.0)
    sim.open_camera()
    if not ok:
        return None
    img = cv2.imread(str(path))
    n_full = sum(1 for _, kind in sim.st.values() if kind == "full")
    cv2.rectangle(img, (0, 0), (img.shape[1], 44), (255, 255, 255), -1)
    cv2.putText(img, f"pick-up station, trip {trip}: ARES at the dock, {len(sim.st)} stones on the table ({n_full} full "
                f"in stacks of 2, {len(sim.st) - n_full} half; PLACEHOLDER layout), boards S0 / S1", (14, 29),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), img)
    return path.name


def overview_view(cfg: dict, job) -> tuple[list, list]:
    """Eye and target (wall frame) of the time-lapse camera (--video): the legs and the station in view, looking from
    the open side of the C (-x) and from the side of leg A (-y), raised ~55 deg (chosen from 4 test views
    2026-10-07)."""
    np = _np()
    pts = []
    for lg in wallplan.legs(cfg):
        L = wallplan.leg_length(cfg, lg.n0)
        pts += [lg.to_wall(0.0, 0.0), lg.to_wall(L, 0.0)]
    T = np.asarray(job.station.T_wall_station, float)
    X, Y = (float(v) for v in cfg["pickup_station"]["table_size"])
    pts += [tuple((T @ np.array([x, y, 0.0, 1.0]))[:2]) for x in (0.0, X) for y in (0.0, Y)]
    P = np.array(pts, float)
    c = (P.min(axis=0) + P.max(axis=0)) / 2.0
    span = float(np.linalg.norm(P.max(axis=0) - P.min(axis=0)))
    d = np.array([-0.3, -0.6, 1.2])
    d = d / np.linalg.norm(d) * 1.0 * span
    return [float(c[0] + d[0]), float(c[1] + d[1]), float(d[2])], [float(c[0]), float(c[1]), 400.0]


def l_images(sim: LSim, look_shot: dict | None, res_dir: Path) -> list:
    """results/l_top.png, l_corner.png, l_last_stop.png (ARES at the stop of the last leg), l_camera_view.png."""
    import cv2
    np, g = sim.np, sim.g
    RDK, cfg, job = sim.RDK, sim.cfg, sim.job
    out = []
    # ARES footprints at every stop (translucent slabs); the ARES model stays at the last stop
    a, w = cfg["ares"]["length"], cfg["ares"]["width"]
    for st in job.stops:
        fp = RDK.AddShape(as_mat(box_points(a, w, 8.0, 0, 0, 4.0)))
        fp.setParent(sim.f_wall)
        fp.setPose(pose2d_T(st.ares))
        fp.setName(f"ARES_at_stop{st.index}")
        fp.setColor(FOOTPRINT)
        sim.RDK.setCollisionActivePair(COLLISION_OFF, sim.ares, fp)
    sim.robot.setJoints(sim.j_home)
    xs = [p[0] for b in sim.wall_boards.values() for p in [b[:3, 3]]] + [s.ares.x_mm for s in job.stops]
    ys = [p[1] for b in sim.wall_boards.values() for p in [b[:3, 3]]] + [s.ares.y_mm for s in job.stops]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    span = max(max(xs) - min(xs) + 1800, (max(ys) - min(ys) + 1800) * 1.6)
    size = (1800, 1125)
    hfov = 40.0
    hgt = span / 2 / math.tan(math.radians(hfov / 2))
    T_top = look_at_up([cx, cy, hgt], [cx, cy, 0.0], [0.0, 1.0, 0.0])
    path = res_dir / "l_top.png"
    ok, K = snapshot_pinhole(RDK, path, T_top, size, hfov)
    if ok:
        img = cv2.imread(str(path))
        lab = []
        for name, T in sim.wall_boards.items():
            c = g.apply(T, np.array([[40.0, 32.0, 0.0]]))[0]
            lab.append((name, c, (0, 90, 200)))
        for st in job.stops:
            lab.append((f"stop {st.index} ({st.leg})", [st.ares.x_mm, st.ares.y_mm, 10.0], (160, 60, 20)))
        for lg in wallplan.legs(cfg):
            x, y = lg.to_wall(wallplan.leg_length(cfg, lg.n0) / 2, -260.0)
            lab.append((f"leg {lg.name}", [x, y, 500.0], (20, 20, 20)))
        for text, p, col in lab:
            u, v, _ = project(K, T_top, [float(c) for c in p])
            cv2.putText(img, text, (int(u) + 6, int(v) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        for k, st in enumerate(job.stops):
            pts = [project(K, T_top, [w_.x_mm, w_.y_mm, 10.0])[:2] for w_ in ([job.stops[k - 1].ares] if k else [])
                   + list(st.route or ([job.stops[k - 1].ares, st.ares] if k else [st.ares]))]
            for p, q in zip(pts, pts[1:]):
                cv2.arrowedLine(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), (40, 140, 40), 2, cv2.LINE_AA,
                                tipLength=0.03)
        cv2.rectangle(img, (0, 0), (img.shape[1], 44), (255, 255, 255), -1)
        cv2.putText(img, f"{shape_of(cfg)} (RoboDK, finished; half stones light): ARES footprints at the stops (blue), "
                    "ARES model at the last stop; green: ARES moves between stops (leg change routes)", (14, 29),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.imwrite(str(path), img)
        out.append(path.name)
    # first corner with the half stones, seen from the outside (leg A far end / leg B start)
    legs = wallplan.legs(cfg)
    A = legs[0]
    xc, yc = A.to_wall(wallplan.leg_length(cfg, A.n0), 0.0)
    eye = [xc + 900.0, yc + 1500.0, 1300.0]                # outside the wall, from leg A's ARES side
    T_c = look_at(eye, [xc - 200.0, yc - 250.0, 220.0])
    if snapshot_pinhole(RDK, res_dir / "l_corner.png", T_c, (1600, 1000), 50.0)[0]:
        out.append("l_corner.png")
    # ARES at a stop of the last leg, arm at a look pose
    if look_shot is not None:
        sim.robot.setJoints(look_shot["j"])
        T_wa = sim.pose_T_np()
        p = lambda x, y, z: [float(c) for c in (T_wa @ np.array([x, y, z, 1.0]))[:3]]   # noqa: E731
        T_v = look_at(p(-1500.0, 1700.0, 1700.0), p(600.0, -100.0, 250.0))
        if snapshot_pinhole(RDK, res_dir / "l_last_stop.png", T_v, (1600, 1000), 50.0)[0]:
            out.append("l_last_stop.png")
        from mauer.vision import detect
        img = detect.draw_detections(look_shot["img"], look_shot["dets"], scale=0.5)
        cv2.rectangle(img, (0, 0), (img.shape[1], 40), (255, 255, 255), -1)
        cv2.putText(img, f"flange camera at stop {look_shot['stop']} (leg {look_shot['leg']}): board "
                    f"{look_shot['board']}, d {look_shot['d']:.0f} mm, tilt {look_shot['tilt']:.0f} deg, "
                    f"{look_shot['corners']} corners, PnP error {look_shot['pnp_mm']:.2f} mm",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(res_dir / "l_camera_view.png"), img)
        out.append("l_camera_view.png")
        sim.robot.setJoints(sim.j_home)
    return out


# ── report ────────────────────────────────────────────────────────────────────
def write_report(path: Path, cfg: dict, job, sim: LSim, info: dict) -> None:
    from mauer import armcheck
    from mauer import job as mjob
    L = []
    w = L.append
    stones = job.stones()
    by_leg = {}
    for r in sim.stone_log:
        d = by_leg.setdefault(r["leg"], {"planned": 0, "ok": 0, "half_ok": 0, "half": 0, "failed": []})
        d["planned"] += 1
        d["half"] += r["kind"] == "half"
        if r["ok"]:
            d["ok"] += 1
            d["half_ok"] += r["kind"] == "half"
        else:
            d["failed"].append(r)
    w(f"# {shape_of(cfg)} - RoboDK collision simulation")
    w("")
    argv = " ".join(Path(a).name if ("\\" in a or "/" in a) else a for a in info["argv"])   # no local paths
    w(f"Generated by `py.exe robodk/simulate.py {argv}` ({time.strftime('%Y-%m-%d %H:%M')}, "
      f"run time {info['runtime_s'] / 60:.1f} min, own RoboDK instance {info['robodk']}, OpenCV {info['opencv']}). "
      "Station saved as `robodk/ARES_UR5_Mauer_L.rdk` (not versioned). **Simulation results: they depend on the "
      "PLACEHOLDER / ASSUMPTION inputs listed below.**")
    w("")
    w("## Summary")
    w("")
    n_ok = sum(1 for r in sim.stone_log if r["ok"])
    w(f"- Stones: {n_ok} of {len(sim.stone_log)} simulated ({job.n_stones} planned) placed with collision-checked "
      "motion; per leg: " + ", ".join(f"{leg} {d['ok']}/{d['planned']} ({d['half_ok']} half)"
                                      for leg, d in by_leg.items()) + ".")
    for st in job.stops:
        es = [e for e in sim.look_log if e["stop"] == st.index]
        if not es:
            continue
        per = []
        for e in es:
            if e["state"] in ("arrival", "complete"):
                per.append(f"{e['state']} {sum(1 for r in e['res'].values() if r['ok'])} "
                           f"({', '.join(n for n, r in sorted(e['res'].items()) if r['ok'])})")
        pl = sorted({n for e in es for n in e["planned"]})
        pl_ok = {n: all(e["res"][n]["ok"] for e in es if n in e["res"]) for n in pl}
        w(f"- Stop {st.index} (leg {st.leg}): boards seen " + "; ".join(per) + "; planned "
          + ", ".join(f"{n} {'ok in every state' if v else 'NOT seen'}" for n, v in pl_ok.items()) + ".")
    hits = sum(len(r["hits"]) for r in sim.route_log)
    w(f"- Routes: {len(sim.route_log)} ARES moves (leg change, {info['reloads']} station trips there and back) "
      f"sampled in 3D, {hits} colliding samples.")
    if sim.trip_log:
        moved = sum(t["moved"] for t in sim.trip_log)
        planned_ = sum(t["planned"] for t in sim.trip_log)
        seen = sum(all(t["looks"].get(lk.boards[0], False) for lk in job.station.looks) for t in sim.trip_log)
        w(f"- Station trips: {len(sim.trip_log)} ({sum(1 for t in sim.trip_log if t['refill'])} operator top-ups "
          f"of the station before a trip), {moved} of {planned_} stones moved station -> magazine with "
          f"collision-checked motion; both planned station boards seen at {seen} of {len(sim.trip_log)} arrivals.")
    elif info["reloads"]:
        w(f"- Station trips: not simulated (--no-trips): the magazine was refilled {info['reloads']} times without "
          "driving or picking.")
    w("")
    w("## Inputs")
    w("")
    w(f"- Job: `tools/make_job.py` build_nominal (config + `robodk/wallplan.py` plan_legs + `results/reach_table.json` "
      f"key `{job.meta.get('reach_table', {}).get('key')}`), {len(job.stops)} stops, {job.n_stones} stones "
      f"({sum(t.kind == 'half' for t in stones)} half), magazine {job.magazine.capacity} slots, "
      f"{info['reloads']} reloads; plan = `results/l_wall_plan.md`.")
    stn = job.station
    used = [stn.slot(i) for i in stn.take_order]
    on_floor = float(cfg["pickup_station"]["table_z"]) <= 0.0
    where = "station area (floor)" if on_floor else "table"
    w(f"- Pick-up station (`[pickup_station]`, {'on the floor' if on_floor else 'table'}): {len(used)} usable holders - "
      f"{sum(s.kind == 'full' for s in used)} full stones in stacks of up to {max(s.layer for s in used)}, "
      f"{sum(s.kind == 'half' for s in used)} half stones; dock {stn.dock_in_wall.describe()} (wall frame), ARES "
      f"front {cfg['pickup_station']['ares_xyz'][1] * -1 - cfg['ares']['length'] / 2:.0f} mm from the {where}; boards "
      f"{', '.join(stn.boards)} {'in their windows on the floor' if on_floor else 'on the table top'}. The station "
      "starts full; the operator tops it up before a trip when it would bring fewer stones than a full one "
      "(`mauer.job.reload_short`, as in the sequencer).")
    for lg in job.legs:
        T = lg["T_wall_leg"]
        ml = next((d for d in job.meta.get("legs", []) if d.get("name") == lg["name"]), {})
        w(f"- Leg {lg['name']}: {lg['n0']} stones in course 0, frame ({T[0][3]:.0f}, {T[1][3]:.0f}) mm, "
          f"{math.degrees(math.atan2(T[1][0], T[0][0])):.0f} deg in the wall frame"
          + (f"; built on ARES's {ml['side']} side at {ml['dist_mm']:.0f} mm" if ml.get("side") else "") + ".")
    b_ = cfg["brick"]
    w(f"- Stones: full = `{b_['mesh']}`" + (f" (`{b_['mesh_cad']}` turned PINS UP)" if b_.get("pins_up") else " (CAD)")
      + "; half = `cad/stone_half_placeholder.stl` (`robodk/make_half_stone.py`: full stone clipped to 100 mm, "
      f"PLACEHOLDER); course pitch {b_['height'] + b_.get('bed_joint', 0.0):.0f} mm (bed joint "
      f"{b_.get('bed_joint', 0.0):g} mm). Collision models 1 mm shorter per end face and 0.9 mm narrower per ribbed "
      "face (clearance for the 0 mm head joints, as before).")
    w("- Motion: `motion.Planner` (IK in `motion.family()`, vertical approach 150 mm, MoveJ_Test 1 deg / MoveL_Test "
      "2 mm against ARES, magazine, wall, boards, table and the arm itself; the last 60 mm of a descent and the first "
      "60 mm of a retreat are contact phases and not tested). Camera body and adapter plate are checked against "
      "every stone (pairs switched on explicitly).")
    w("- Looks: job look pose first, else a grid (camera " + "/".join(f"{d:.0f}" for d in LOOK_DISTS)
      + " mm from the board centre, axis-to-normal " + "/".join(f"{t:.0f}" for t in LOOK_TILTS)
      + " deg, 8 azimuths, 8 rolls), 4 outer corners >= 50 px inside, IK in `motion.family()`, collision-free incl. "
      "camera/adapter, solid-volume check, collision-free transfer from the current pose, unoccluded (render diff "
      f"with all occluders hidden <= {OCCL_MAX * 100:.1f} % changed board pixels) and all 12 ChArUco corners "
      "detected; ideal pinhole render (RoboDK Cam2D), no blur/noise.")
    w("- Not simulated: dynamics, jaw motion, pin engagement, tipping; ARES drive error (ARES stands exactly at the "
      "nominal stop and dock, the routes are driven exactly); the operator's top-ups of the station (the stones "
      "appear in the holders); the floor plates (only the printed boards are objects); camera blur, noise, "
      "lighting.")
    w("")
    w("Depends on (non-CONFIRMED config values, from the job):")
    w("")
    w("| key | status | value |")
    w("|---|---|---|")
    for d in job.depends_on:
        w(f"| `{d['key']}` | {d['status']} | {json.dumps(d['value'])} |")
    w("")
    w("## Self-tests")
    w("")
    for k, v in info["self_test"].items():
        w(f"- {k}: {', '.join(v) if isinstance(v, list) else v}")
    w("")
    w("## Result per leg")
    w("")
    w("| leg | stones planned | placed with collision-checked motion | half stones placed | failed |")
    w("|---|---|---|---|---|")
    for leg, d in by_leg.items():
        w(f"| {leg} | {d['planned']} | {d['ok']} | {d['half_ok']} / {d['half']} | {len(d['failed'])} |")
    tot_ok = sum(d["ok"] for d in by_leg.values())
    w(f"| all | {len(sim.stone_log)} | {tot_ok} | {sum(d['half_ok'] for d in by_leg.values())} / "
      f"{sum(d['half'] for d in by_leg.values())} | {len(sim.stone_log) - tot_ok} |")
    w("")
    fails = [r for r in sim.stone_log if not r["ok"]]
    if fails:
        w("Failures (stone = leg, course, index, h = half; the stone was put in place without motion, red):")
        w("")
        w("| stone | stop | phase | category | details |")
        w("|---|---|---|---|---|")
        for r in fails:
            w(f"| {r['label']} | {r['stop']} | {r['phase']} | {r['category']} | {r['why']} |")
        w("")
    else:
        w("No failures: every stone was picked from its planned magazine slot and placed with a collision-checked "
          "motion.")
        w("")
    w(f"Collision tests: {sim.planner.tests} ({sim.planner.self_rejects} poses / moves refused by the tool-vs-arm "
      f"model, mauer.armcheck.self_clearance >= {armcheck.SELF_CLEARANCE_MM:g} mm); motion planning + execution "
      f"{sim.motion_s / 60:.1f} min.")
    w("")
    if sim.trip_log:
        w("## Station trips (ARES at the dock, collision-checked station -> magazine moves)")
        w("")
        w("Per trip: the operator's top-up (stones put into empty holders before the trip), the station boards at the "
          "dock (planned looks executed), the stones moved from the station holders (top layer first) into the "
          "magazine slots of the job's reload plan.")
        w("")
        w("| trip | from stop | operator top-up | station boards seen | stones moved | failed |")
        w("|---|---|---|---|---|---|")
        for t in sim.trip_log:
            fl = [r for r in sim.transfer_log if r.get("trip") == t["trip"] and not r["ok"]]
            w(f"| {t['trip']} | {t['stop']} | {t['refill'] or '-'} | "
              f"{', '.join(b for b, ok in sorted(t['looks'].items()) if ok) or '-'} | {t['moved']} / {t['planned']} | "
              + ("; ".join(f"{r['station_slot']} -> {r['slot']} ({r['phase']}: {r['category']})" for r in fl) or "-")
              + " |")
        w("")
    w("## ARES routes (3D check, 100 mm / 5 deg samples, parked arm, magazine as loaded)")
    w("")
    w("| route | waypoints | samples | colliding samples | pairs |")
    w("|---|---|---|---|---|")
    for r in sim.route_log:
        pr = sorted({p for h in r["hits"] for p in h["pairs"]})
        w(f"| {r['what']} | {r['waypoints']} | {r['samples']} | {len(r['hits'])} | {', '.join(pr) or '-'} |")
    w("")
    w("## Boards seen per stop (RoboDK look check)")
    w("")
    w("Planned = the job's two looks per stop (longest baseline). arrival = wall of the earlier stops, magazine full "
      "(all 8 boards checked); station trip n = the station boards with ARES at the dock (planned looks executed); "
      "after trip n = re-measurement after the n-th station trip of that stop (the planned boards only, executed); "
      "complete = the stop's stones placed (all 8 boards; worst case, the sequencer does not look there). Source "
      "job = the job's look pose, grid = first valid pose of the search grid.")
    w("")
    w("| stop | leg | state | planned boards ok | all boards seen (source, d mm / tilt deg) | not seen (reason) |")
    w("|---|---|---|---|---|---|")
    for e in sim.look_log:
        st = job.stops[e["stop"]]
        res = e["res"]
        pl = [n for n in e["planned"] if n in res]
        ok_pl = [n for n in pl if res[n]["ok"]]
        seen = [f"{n} ({res[n]['source']}, {res[n]['d']:.0f}/{res[n]['tilt']:.0f})" for n in sorted(res)
                if res[n]["ok"]]
        bad = [f"{n}: {res[n]['reason']}" for n in sorted(res) if not res[n]["ok"]]
        w(f"| {e['stop']} | {st.leg} | {e['state']} | {len(ok_pl)} / {len(pl)} ({', '.join(ok_pl) or '-'}) | "
          f"{len(seen)}: {', '.join(seen) or '-'} | {'; '.join(bad) or '-'} |")
    w("")
    pnp = [r["pnp_mm"] for e in sim.look_log for r in e["res"].values() if r["ok"] and r.get("pnp_mm") == r.get("pnp_mm")]
    if pnp:
        ok02 = sum(1 for x in pnp if x <= 0.2)
        w(f"PnP check of the rendered boards against the RoboDK truth (ideal render, nominal intrinsics, "
          f"`mauer.vision.detect`): {ok02} of {len(pnp)} views within 0.2 mm (median {sorted(pnp)[len(pnp) // 2]:.3f} "
          f"mm) - the board poses in RoboDK (incl. the legs after A, `mauer.reference.placements`) and the camera chain agree; "
          f"max {max(pnp):.2f} mm, see Problems.")
        w("")
    w("## Static findings (meshes and config)")
    w("")
    sf = info["static"]
    w(f"- Stone ribs on the long faces: {sf['rib_mm']:.2f} mm high (`cad/stone_full_2026-10-01.stl` bbox).")
    for c in sf.get("corners", []):
        run, butt = ((c["prev"], c["next"]) if c.get("through", "prev") == "prev" else (c["next"], c["prev"]))
        w(f"- Butt corner {c['prev']}-{c['next']} (leg {run} runs through): leg {butt}'s course-0 end face lies "
          f"{abs(c['gap_mm']):.2f} mm "
          + (f"INSIDE leg {run}'s rib envelope" if c["gap_mm"] < 0 else f"clear of leg {run}'s ribs")
          + ". The RoboDK collision models are shrunk (see Inputs) and do not see it.")
    w(f"- Full stone mesh volume: {sf['volume_cm3']:.0f} cm3 with consistent face orientation (ray casting gives the "
      f"same); the signed volume of the raw STL is {sf['volume_signed_raw_cm3']:.0f} cm3 because "
      + (f"{sf['flipped_faces']} of its {sf['faces']} faces are flipped" if sf["flipped_faces"] else "-")
      + ". cad/README.md and the [brick] mass_kg comment quote 2270 cm3.")
    w("")
    w("## Images")
    w("")
    for n in info["images"]:
        w(f"- `results/{n}`")
    w("")
    w("## Problems")
    w("")
    for p in info["problems"]:
        w(f"- {p}")
    w("")
    path.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")


def magazine_before(job, initial: list, events: dict, j_stop: int) -> list:
    """Magazine content [(slot, kind)] before the stone with global index j_stop (replay of the job's plan)."""
    content = dict(initial)
    for j, t in enumerate(job.stones()[:j_stop]):
        if j in events:
            content = {mid: kind for _, mid, kind in events[j]["pairs"]}
        content.pop(t.slot, None)
    return sorted(content.items(), key=lambda kv: (kv[0][-1], kv[0]))       # bottom layer first (id ends in l<n>)


def station_before(job, events: dict, j_stop: int) -> list:
    """Filled station holders before the stone with global index j_stop (replay; the station starts full)."""
    full = list(job.station.take_order)
    filled = set(full)
    for j in sorted(e for e in events if e < j_stop):
        if events[j]["refill"]:
            filled = set(full)
        filled -= {ssid for ssid, _, _ in events[j]["pairs"]}
    return [s for s in full if s in filled]


def run_l(args, cfg: dict, RDK, it: dict) -> int:
    from make_job import build_nominal
    t_start = time.time()
    from make_job import leg_dist, leg_side
    for lg in wallplan.legs(cfg):                   # every leg's reach table (side, distance) - computed if missing
        reach_table(RDK, cfg, leg_dist(cfg, lg, args.dist), leg_side(cfg, lg))
    job = build_nominal(cfg, dist=args.dist)
    print(job.summary(), flush=True)
    initial, events = magazine_events(job)
    if args.plan_only:
        return 0
    sim = LSim(RDK, cfg, it, job, args)
    sim.setup()
    RDK.Render(False)
    sim.planner.z_safe = sim.safe_z(full=True)
    sim.j_home = sim.planner.compact(0.0, [0, -100, 52, -42, -90, 0])
    if sim.j_home is None:
        raise SystemExit("no collision-free start pose found")
    sim.robot.setJoints(sim.j_home)
    info = {"argv": sys.argv[1:], "self_test": sim.self_test(), "static": static_findings(cfg)}
    print(f"self-test: {info['self_test']}", flush=True)
    print(f"static findings: {info['static']}", flush=True)
    stones = job.stones()
    first = {st.index: sum(len(s.stones) for s in job.stops[:st.index]) for st in job.stops}
    start = max(0, min(args.from_stop, len(job.stops) - 1))
    for st in job.stops[:start]:
        sim.prebuild(st.stones)                       # earlier stops: built without motion
    for sid, kind in magazine_before(job, initial, events, first[start]):
        sim.fill_slot(sid, kind)
    if args.trips:
        sim.fill_station(station_before(job, events, first[start]))          # the operator filled it before the run
    sim.open_camera()
    if args.video:
        from timelapse import Recorder
        eye, target = overview_view(cfg, job)
        sim.rec = Recorder(RDK, args.video, eye, target, fps=args.video_fps)
        sim.planner.on_move = sim.rec.frame
    RDK.Render(False)
    RDK.setSimulationSpeed(args.speed)
    sim.robot.setSpeed(400, 120, 1500, 600)
    all_boards = sorted(sim.wall_boards)
    look_shot = None
    reloads = sum(1 for j in events if j < first[start])
    prev = job.stops[start - 1] if start else None
    try:
        for st in job.stops[start:]:
            k = st.index
            planned = {lk.boards[0]: lk for lk in st.looks}
            print(f"stop {k}: leg {st.leg}, a = {st.a_mm:.0f} mm, ARES {st.ares.describe()} (wall), "
                  f"{len(st.stones)} stones", flush=True)
            if prev is not None:
                sim.park_safe()
                route = st.route or [prev.ares, st.ares]
                what = (f"leg change stop {prev.index} -> {k}" if st.route else f"stop {prev.index} -> {k}")
                sim.drive_route(route, what, animate=args.animate)
            sim.set_ares(st.ares)
            RDK.Render(False)
            sim.caption(f"Halt {k + 1}/{len(job.stops)}, Schenkel {st.leg}: Kamera misst die Wandtafeln")
            sim.looks(k, "arrival", all_boards, planned, execute=True)
            n_trip = 0
            for i, t in enumerate(st.stones):
                gidx = first[k] + i
                if gidx in events and gidx > 0:
                    n_trip += 1
                    reloads += 1
                    ev = events[gidx]
                    print(f"   magazine empty -> station trip {reloads} ({len(ev['pairs'])} stones, "
                          f"{sum(kd == 'half' for *_, kd in ev['pairs'])} half)", flush=True)
                    sim.park_safe(full=True)
                    here = sim.pose
                    if args.trips:
                        image = (REPO / "results" / "l_station.png"
                                 if not args.no_images and not sim.trip_log else None)
                        sim.station_trip(k, reloads, st, ev, animate=args.animate, image=image)
                    else:
                        for _, sid, kind in ev["pairs"]:                     # --no-trips: refilled without motion
                            sim.fill_slot(sid, kind)
                    sim.set_ares(here)
                    RDK.Render(False)
                    sim.caption(f"Halt {k + 1}/{len(job.stops)}, Schenkel {st.leg}: Kamera misst die Wandtafeln")
                    sim.looks(k, f"after trip {n_trip}", sorted(planned), planned, execute=True)
                sim.caption(f"Halt {k + 1}/{len(job.stops)}, Schenkel {st.leg}: Stein {gidx + 1}/{len(stones)} "
                            f"({'Halbstein' if t.kind == 'half' else 'Vollstein'}, Lage {t.course + 1})")
                rec = sim.lay(t)
                rec["stop"] = k
                sim.stone_log.append(rec)
                if rec["ok"]:
                    print(f"   placed {t.label:9s} slot {t.slot}  (collision tests so far: {sim.planner.tests}, "
                          f"{(time.time() - t_start) / 60:.1f} min)", flush=True)
                if args.max_stones and len(sim.stone_log) >= args.max_stones:
                    raise KeyboardInterrupt
            sim.park_safe()
            res = sim.looks(k, "complete", all_boards, planned, execute=False)
            if st.leg == job.stops[-1].leg and look_shot is None:
                for name in ("W7", "W6", "W5"):
                    r = res.get(name)
                    if r and r["ok"]:
                        look_shot = {**r, "stop": k, "leg": st.leg}
                        break
            prev = st
    except KeyboardInterrupt:
        print("stopped (--max-stones)", flush=True)
        sim.notes.append(f"run stopped after {len(sim.stone_log)} stones (--max-stones)")
    except Exception as e:  # noqa: BLE001 - keep the partial result
        import traceback
        traceback.print_exc()
        sim.notes.append(f"run aborted: {e!r}")
    finally:
        RDK.Render(True)
        RDK.setSimulationSpeed(1)
    if sim.rec is not None:
        if sim.rec.labels:
            sim.rec.label = "Mauer fertig"
            for _ in range(2 * sim.rec.fps):                 # 2 s on the finished wall
                sim.rec.frame()
        info["video"] = sim.rec.write(f"ARES_Mauer - RoboDK-Simulation {time.strftime('%d.%m.%Y')} "
                                      f"(Zeitraffer, ein Bild je Bewegung)")
        print(f"video: {info['video']}", flush=True)
    images = [t["image"] for t in sim.trip_log if t.get("image")]
    if not args.no_images:
        try:
            images += l_images(sim, look_shot, REPO / "results")
        except Exception as e:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            sim.notes.append(f"images failed: {e!r}")
    RDK.Save(str(args.out or L_RDK), sim.station)
    print(f"saved {args.out or L_RDK}", flush=True)
    import cv2
    info.update(runtime_s=time.time() - t_start, robodk=RDK.Version(), opencv=cv2.__version__, reloads=reloads,
                images=images, problems=problems_of(sim, info) + sim.notes)
    if not args.no_report:
        write_report(args.report or L_REPORT, cfg, job, sim, info)
        print(f"wrote {args.report or L_REPORT}", flush=True)
    if args.json:
        Path(args.json).write_text(json.dumps({"stones": sim.stone_log, "routes": sim.route_log,
                                               "looks": sim.look_log, "trips": sim.trip_log,
                                               "transfers": sim.transfer_log, "info": info}, default=str, indent=1),
                                   encoding="utf-8")
    n_ok = sum(r["ok"] for r in sim.stone_log)
    n_tr = sum(r["ok"] for r in sim.transfer_log)
    print(f"done: {n_ok}/{len(sim.stone_log)} stones placed with collision-checked motion "
          f"({job.n_stones} planned), {n_tr}/{len(sim.transfer_log)} moved station -> magazine in "
          f"{len(sim.trip_log)} station trips, {sim.planner.tests} collision tests, "
          f"{(time.time() - t_start) / 60:.1f} min", flush=True)
    return 0 if n_ok == job.n_stones and n_tr == len(sim.transfer_log) else 2


def problems_of(sim: LSim, info: dict) -> list:
    """Problems found in the run, as sentences for the report."""
    out = []
    fails = [r for r in sim.stone_log if not r["ok"]]
    for cat in sorted({r["category"] for r in fails}):
        rs = [r for r in fails if r["category"] == cat]
        out.append(f"{cat}: {len(rs)} stone(s) ({', '.join(r['label'] for r in rs[:12])}"
                   + (", ..." if len(rs) > 12 else "") + ") - see the failure table.")
    for r in sim.transfer_log:
        if not r["ok"]:
            out.append(f"station trip {r['trip']}: station holder {r['station_slot']} -> magazine {r['slot']} failed "
                       f"({r['phase']}): {r['category']}: {r['why']} (stone put into the magazine without motion).")
    for r in sim.route_log:
        if r["hits"]:
            out.append(f"route {r['what']}: {len(r['hits'])} colliding samples "
                       f"({', '.join(sorted({p for h in r['hits'] for p in h['pairs']}))}).")
    bad: dict = {}                     # (stop, board) -> [states] where a PLANNED board was not seen
    grid: dict = {}                    # (stop, board) -> [(state, job reason, d, tilt)] planned, job pose failed
    for e in sim.look_log:
        res = e["res"]
        n_ok = sum(1 for n in res if res[n]["ok"])
        for n in e["planned"]:
            if n not in res:
                continue
            if not res[n]["ok"]:
                bad.setdefault((e["stop"], n), []).append((e["state"], res[n]["reason"]))
            elif res[n]["source"] != "job":
                grid.setdefault((e["stop"], n), []).append((e["state"], res[n].get("job_reason", "?"), res[n]["d"],
                                                            res[n]["tilt"]))
        if len(res) > 2 and n_ok < 2:
            out.append(f"stop {e['stop']} {e['state']}: only {n_ok} board(s) seen - fewer than 2.")
    for (k, n), lst in bad.items():
        seen = sorted({b for e in sim.look_log if e["stop"] == k for b, r in e["res"].items() if r["ok"]})
        out.append(f"stop {k}: planned board {n} not seen at {', '.join(st for st, _ in lst)} ({lst[0][1]}); "
                   f"boards seen there: {', '.join(seen)}.")
    for (k, n), lst in grid.items():
        st, why, d, tilt = lst[0]
        out.append(f"stop {k}: the job's look pose for {n} collides at {', '.join(x[0] for x in lst)} ({why}); a grid "
                   f"pose works (d {d:.0f} mm, tilt {tilt:.0f} deg).")
    pnp = [(e["stop"], e["state"], n, r["pnp_mm"], r.get("pnp_fix_mm")) for e in sim.look_log
           for n, r in e["res"].items() if r["ok"] and r.get("pnp_mm", 0) > 0.2]
    if pnp:
        fix = [x[4] for x in pnp if x[4] is not None]
        out.append(f"PnP on the ideal render off by up to {max(x[3] for x in pnp):.2f} mm in {len(pnp)} view(s) ("
                   + ", ".join(sorted({f'stop {x[0]} {x[2]}' for x in pnp})) + ") although the detected corners lie "
                   "within 0.03 px of the truth: mauer.vision.detect.estimate_pose hands IPPE's rotation vector to "
                   "solvePnPRefineLM without normalising it (board at ~180 deg roll: |rvec| >> pi) and LM ends in a "
                   "wrong minimum (RMS ~0.3 px, still accepted)"
                   + (f"; with the rvec normalised (Rodrigues twice) the same views give <= {max(fix):.3f} mm"
                      if fix else "") + ".")
    sf = info["static"]
    for c in sf.get("corners", []):
        if c["gap_mm"] < 0:
            out.append(f"butt corner {c['prev']}-{c['next']}: leg {c['next']}'s first stone of every course overlaps "
                       f"leg {c['prev']}'s ribs by {-c['gap_mm']:.2f} mm (origin at the body face, the ribs stick out "
                       f"{sf['rib_mm']:.2f} mm) - the real stone cannot sit at its planned place.")
    kg = float(sim.cfg["brick"].get("mass_kg", 0.0) or 0.0)
    if kg <= 0.0 and abs(sf["volume_cm3"] - 2270.0) > 50.0:      # only while the stone is not weighed
        out.append(f"full stone volume: the mesh encloses {sf['volume_cm3']:.0f} cm3, not 2270 cm3 (the raw STL has "
                   "flipped faces) -> at ~2.3 g/cm3 about "
                   f"{sf['volume_cm3'] * 2.3 / 1000:.1f} kg, above the UR5 payload of 5 kg (to check with the real "
                   "stone / the STEP).")
    return out


def main(argv: list | None = None) -> int:
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], allow_abbrev=False)
    ap.add_argument("--length", type=int, default=None,
                    help="STRAIGHT wall of N stones in course 0 (default: the L of [[wall.legs]], else 16)")
    ap.add_argument("--speed", type=float, default=5.0, help="RoboDK simulation speed factor")
    ap.add_argument("--dist", type=float, default=cfg["wall"]["dist_nominal"])
    ap.add_argument("--plan-only", action="store_true", help="stop after planning (no motion)")
    ap.add_argument("--port", type=int, default=None, help="API port of the own RoboDK instance (default 20599)")
    ap.add_argument("--keep-open", action="store_true", help="leave the own RoboDK instance open at the end")
    ap.add_argument("--from-stop", type=int, default=0, help="L: build the earlier stops without motion")
    ap.add_argument("--max-stones", type=int, default=0, help="L: stop after N simulated stones (debug)")
    ap.add_argument("--animate", action="store_true", help="L: animate the ARES moves (stops and station trips)")
    ap.add_argument("--no-trips", dest="trips", action="store_false",
                    help="L: no station trips - the magazine is refilled without driving or picking")
    ap.add_argument("--no-images", action="store_true")
    ap.add_argument("--no-report", action="store_true")
    ap.add_argument("--report", type=Path, default=None, help=f"L report (default {L_REPORT.relative_to(REPO)})")
    ap.add_argument("--json", type=Path, default=None, help="L: raw results as JSON")
    ap.add_argument("--save-looks", type=Path, default=None, help="L: write every valid look image to this folder")
    ap.add_argument("--out", type=Path, default=None, help=f"L: station .rdk (default {L_RDK.relative_to(REPO)})")
    ap.add_argument("--video", type=Path, default=None,
                    help="L: time-lapse MP4 of the run (robodk/timelapse.py: a frame after every move)")
    ap.add_argument("--video-fps", type=int, default=30)
    args = ap.parse_args(argv)
    is_l = args.length is None and wallplan.is_l(cfg)
    if args.length is None:
        args.length = 16
    if args.out and args.out.resolve() == (REPO / "robodk" / f"{STATION_NAME}.rdk").resolve():
        raise SystemExit("--out must not overwrite the straight-wall station robodk/ARES_UR5_Mauer.rdk")

    import build_station
    RDK = connect(new_instance=True, port=args.port)
    try:
        print(f"RoboDK {RDK.Version()} (own instance, pid {RDK.NEW_INSTANCE.pid})", flush=True)
        print("building the station ...", flush=True)
        it = build_station.build(RDK, cfg)
        if is_l:
            return run_l(args, cfg, RDK, it)
        return run_straight(args, cfg, RDK)
    finally:
        if not args.keep_open:
            print(f"RoboDK instance closed (exit code {close_instance(RDK)})", flush=True)


if __name__ == "__main__":
    sys.exit(main())
