"""Simulate the brick laying in RoboDK with a valid placement order and collision-checked paths.

  1. reach table: which wall positions (relative to ARES) the UR5 can serve, per course (IK, vertical approach,
     collision-free against ARES with gripper and held stone) - cached in results/reach_table.json;
  2. wall plan (wallplan.py): trapezoid wall, stones only on complete supports, ARES stops and moves;
  3. execution: every pick and place is planned with motion.py (all segments tested against ARES, magazine,
     the wall built so far and the arm itself) and only then executed. Collision checking stays on.

Usage (Windows Python, station built with build_station.py and open in RoboDK):
    py.exe robodk/simulate.py [--length 16] [--speed 3] [--dist 740]

Not simulated: dynamics, jaw motion, tipping, pin engagement forces, ARES drive error.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import time

import wallplan
from build_station import STONE
from motion import Planner, set_held, set_static, stones_in_station
from rdk_common import (PI, REPO, T_TC_CAD, as_mat, connect, course_top_z, load_config, place_pose, stone_points,
                        transl, wall_frame)
from reach_study import Checker, frange
from robodk.robolink import (COLLISION_ON, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT, ITEM_TYPE_STATION,
                             ITEM_TYPE_TOOL)
from robodk.robomath import invH, rotx, rotz

GHOST = [0.72, 0.30, 0.20, 0.15]
# magazine rows: (x relative to the UR base, layers). Pitch 220 mm = stone 120 + room for the open jaws. No row
# closer to the UR: when the base turns, the UR5 upper arm (offset ~136 mm from the base axis) sweeps a circle of
# ~190 mm radius from shoulder height (deck + 89 mm) upwards - a stone row at x = UR - 214 mm (faces 153 mm from the
# axis) lies inside it.
MAG_ROWS = ((-653.6, 3), (-433.6, 3))
MAG_Y = (-205.0, 0.0, 205.0)       # stones end to end along ARES y (jaws do not reach past the stone ends)
MAG_LAYERS = max(n for _, n in MAG_ROWS)


def reach_table(RDK, cfg: dict, dist: float) -> dict:
    """{course: {u_rel: ok}} for the wall at `dist`, cached by the relevant configuration."""
    key_src = json.dumps(["family-v2", cfg["ur5"], cfg["tool"], cfg["brick"], cfg["wall"]["base_z"], cfg["wall"]["courses"],
                          cfg["study"]["approach"], dist], sort_keys=True)
    key = hashlib.sha1(key_src.encode()).hexdigest()[:12]
    path = REPO / "results" / "reach_table.json"
    if path.exists():
        data = json.loads(path.read_text())
        if data.get("key") == key:
            return {int(k): {float(u): v for u, v in d.items()} for k, d in data["table"].items()}
    print("computing reach table (about 1 min) ...", flush=True)
    chk = Checker(RDK, cfg, True)
    us = frange(-1000.0, 1000.0, 20.0)
    table = {k: {u: chk.check_both(place_pose(cfg, u, dist, course_top_z(cfg, k)))[0] for u in us}
             for k in range(cfg["wall"]["courses"])}
    chk.close()
    path.write_text(json.dumps({"key": key, "dist": dist, "table": table}))
    return table


class Sim:
    def __init__(self, RDK, cfg: dict, dist: float):
        self.RDK, self.cfg, self.dist = RDK, cfg, dist
        self.robot = RDK.Item("UR5", ITEM_TYPE_ROBOT)
        self.tool = RDK.Item("Gripper_EHPS20", ITEM_TYPE_TOOL)
        self.f_ares = RDK.Item("ARES base_link", ITEM_TYPE_FRAME)
        self.f_wall = RDK.Item("Wall", ITEM_TYPE_FRAME)
        self.ares = RDK.Item("ARES_STEP_2026-09-23", ITEM_TYPE_OBJECT)
        self.station = [s for s in RDK.ItemList(ITEM_TYPE_STATION) if s.Name() == "ARES_UR5_Mauer"][0]
        self.mesh = str(REPO / cfg["brick"]["mesh"])
        self.deck = cfg["ares"]["deck_top_z"] + cfg["deck"]["holder_z"]
        self.H = cfg["brick"]["height"]
        self.magazine: list = []
        self.n_new = 0
        self.planner = Planner(RDK, cfg, self.robot, self.tool, self.f_ares)

    def clear(self) -> None:
        for it in self.RDK.ItemList(ITEM_TYPE_OBJECT):
            if it.Name().startswith(("Stone_", "_held_stone", "Stones_", "Wall_plan")):
                it.Delete()
        ghost = self.RDK.Item("Wall_nominal", ITEM_TYPE_OBJECT)
        if ghost.Valid():
            ghost.setVisible(False)
        self.f_ares.setPose(transl(0, 0, 0))
        self.f_wall.setParentStatic(self.station)
        self.RDK.setCollisionActive(COLLISION_ON)

    def new_stone(self, parent, pose, name: str):
        st = self.RDK.AddFile(self.mesh, parent)
        # collision model with clearance: 1 mm smaller per end face and per ribbed face (about the stone centre),
        # otherwise neighbours with 0 mm head joint touch over the whole descent
        st.Scale([0.985, 0.99, 1.0], transl(-60, 100, 0))
        self.n_new += 1
        st.setName(f"Stone_{name}_{self.n_new:03d}")
        st.setPose(pose)
        st.setColor(STONE)
        return st

    def stack_top(self) -> float:
        tops = [m[1].Pos()[2] for m in self.magazine]
        return max(tops) if tops else self.deck

    def fill_magazine(self, slots: list) -> None:
        self.magazine = []
        for layer, x, y in slots:
            p_tc = transl(x, y, self.deck + layer * self.H) * rotz(PI / 2)
            st = self.new_stone(self.f_ares, p_tc * T_TC_CAD, "mag")
            self.magazine.append((st, p_tc * rotx(PI)))
        self.magazine.sort(key=lambda m: -m[1].Pos()[2])
        allst = stones_in_station(self.RDK)
        for st, _ in self.magazine:
            set_static(self.RDK, st, allst, self.ares)

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
        ret = P.plan_retreat(j_t, pose)
        if not ret:
            print("   retreat from the wall not collision-free", flush=True)
            return False
        self.go(ret[0])
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


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=int, default=16, help="stones in the first course")
    ap.add_argument("--speed", type=float, default=3.0, help="RoboDK simulation speed factor")
    ap.add_argument("--dist", type=float, default=cfg["wall"]["dist_nominal"])
    ap.add_argument("--plan-only", action="store_true", help="stop after planning (no motion)")
    args = ap.parse_args()
    mag_rows = [(cfg["ur5"]["mount_x"] + dx, n) for dx, n in MAG_ROWS]

    RDK = connect()
    if not any(st.Name() == "ARES_UR5_Mauer" for st in RDK.ItemList(ITEM_TYPE_STATION)):
        saved = REPO / "robodk" / "ARES_UR5_Mauer.rdk"
        if saved.exists():
            print("opening the saved station ...", flush=True)
            RDK.AddFile(str(saved))
        else:
            print("building the station (first run, about 1-2 min) ...", flush=True)
            import build_station
            build_station.main()
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
             if chk.check_both(transl(x, y, sim.deck + lay * sim.H + 2.0) * rotz(PI / 2) * rotx(PI))[0]]
    chk.close()
    print(f"magazine: {len(slots)} reachable slots", flush=True)
    if args.plan_only:
        RDK.Render(True)
        return

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
                    return
                placed += 1
                print(f"   placed c{s.course} u={s.u:5.0f}  (collision tests so far: {sim.planner.tests})", flush=True)
        sim.park()
    finally:
        RDK.Render(True)
        RDK.setSimulationSpeed(1)
        print(f"done: {placed}/{len(stones)} stones, {sim.planner.tests} collision tests, "
              f"{time.time() - t0:.0f} s wall-clock", flush=True)


if __name__ == "__main__":
    main()
