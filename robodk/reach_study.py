"""Reach study: which stone positions can the UR5 on ARES serve from one stop?

A position counts as reachable when the UR5 has an IK solution for the place pose (gripper vertical, stone along the
wall, gripper yaw 0° or 180°) AND the vertical linear approach from `approach` mm above to the place pose is feasible
and, if enabled, free of collisions with ARES and with itself (RoboDK MoveL_Test; the held stone is tool geometry).

Usage (Windows Python, station built with build_station.py and open in RoboDK):
    py.exe robodk/reach_study.py [--coarse] [--deck]

Output: results/reach_wall.csv, results/reach_deck.csv, results/reach_summary.md
"""
from __future__ import annotations

import csv
import sys
import time

from rdk_common import (PI, REPO, connect, course_shift, course_top_z, held_stone_pose, load_config, place_pose,
                        tcp_pose, transl, ur5_base_pose)
from robodk.robolink import COLLISION_OFF, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
from robodk.robomath import invH, rotx, rotz

from motion import family


def frange(a: float, b: float, step: float) -> list:
    n = int(round((b - a) / step))
    return [round(a + i * step, 3) for i in range(n + 1)]


class Checker:
    def __init__(self, RDK, cfg: dict, collisions: bool):
        self.RDK = RDK
        self.robot = RDK.Item("UR5", ITEM_TYPE_ROBOT)
        self.tool = RDK.Item("Gripper_EHPS20", ITEM_TYPE_TOOL)
        self.f_ares = RDK.Item("ARES base_link", ITEM_TYPE_FRAME)
        if not (self.robot.Valid() and self.tool.Valid() and self.f_ares.Valid()):
            raise RuntimeError("Station not found - run build_station.py first")
        self.f_ares.setPose(transl(0, 0, 0))
        self.tool_pose = tcp_pose(cfg)
        self.ref = invH(ur5_base_pose(cfg))       # ARES frame w.r.t. UR5 base
        self.approach = cfg["study"]["approach"]
        self.collisions = collisions
        self.robot.setPoseFrame(self.f_ares)
        self.robot.setPoseTool(self.tool)
        # held stone as an object on the TCP, so it is part of the collision check (not against the gripper/arm)
        old = RDK.Item("_held_stone", ITEM_TYPE_OBJECT)
        if old.Valid():
            old.Delete()
        self.stone = RDK.AddFile(str(REPO / cfg["brick"]["mesh"]), self.tool)
        self.stone.setName("_held_stone")
        self.stone.setPose(held_stone_pose())
        # (stays visible: RoboDK checks collisions only for visible objects)
        RDK.setCollisionActivePair(COLLISION_OFF, self.stone, self.tool, 0, 0)
        for link in range(0, 8):
            RDK.setCollisionActivePair(COLLISION_OFF, self.stone, self.robot, 0, link)

    def close(self) -> None:
        self.stone.Delete()

    def check(self, pose_place) -> tuple[bool, str]:
        """(reachable, reason) for one place/pick pose given in the ARES frame."""
        sols = self.robot.SolveIK_All(pose_place, tool=self.tool_pose, reference=self.ref)
        ncol = sols.size(1) if sols.size(0) >= 6 else 0
        if ncol == 0:
            return False, "ik"
        pose_app = transl(0, 0, self.approach) * pose_place
        reason = "approach"
        for c in range(ncol):
            j = [sols[r, c] for r in range(6)]
            if not family(j):
                continue                           # only the configuration used by the motion planner
            j_app = self.robot.SolveIK(pose_app, j, tool=self.tool_pose, reference=self.ref).list()
            if len(j_app) < 6 or max(abs(a - b) for a, b in zip(j_app, j)) > 60.0:
                continue                           # approach needs another configuration
            if not self.collisions:
                return True, "ok"
            r = self.robot.MoveL_Test(j_app, pose_place)
            if r == 0:
                return True, "ok"
            reason = "collision" if r > 0 else "linear"
        return False, reason

    def check_both(self, pose_place) -> tuple[bool, str]:
        r = self.check(pose_place)
        return r if r[0] else self.check(pose_place * rotz(PI))


def best_stop(ok: dict, cfg: dict, us: list) -> tuple[int, float]:
    """Max. stones per course per stop (running bond, stepped ends) and the window start s along the wall [mm]."""
    b = cfg["brick"]
    pitch = b["length"] + b["head_joint"]
    step = us[1] - us[0]

    def reach(k: int, u: float) -> bool:
        ui = round((u - us[0]) / step) * step + us[0]
        return ok.get((k, round(ui, 3)), False)

    best = (0, 0.0)
    for n in range(1, 15):
        for s in us:
            if all(reach(k, s + course_shift(cfg, k) + b["length"] / 2 + i * pitch)
                   for k in range(cfg["wall"]["courses"]) for i in range(n)):
                best = (n, s)
                break
        if best[0] < n:
            break
    return best


def wall_study(chk: Checker, cfg: dict, coarse: bool) -> list:
    st = cfg["study"]
    f = 2 if coarse else 1
    ds = frange(st["wall_dist_min"], st["wall_dist_max"], st["wall_dist_step"] * f)
    us = frange(st["u_min"], st["u_max"], st["u_step"] * f)
    rows, summary = [], []
    t0 = time.time()
    for d in ds:
        ok = {}
        for k in range(cfg["wall"]["courses"]):
            z = course_top_z(cfg, k)
            for u in us:
                r = chk.check_both(place_pose(cfg, u, d, z))
                ok[(k, u)] = r[0]
                rows.append({"wall_dist": d, "course": k, "u": u, "ok": int(r[0]), "reason": r[1]})
        n, s = best_stop(ok, cfg, us)
        ranges = []
        for k in range(cfg["wall"]["courses"]):
            good = [u for u in us if ok[(k, u)]]
            ranges.append((min(good), max(good)) if good else None)
        summary.append({"wall_dist": d, "n": n, "s": s, "ranges": ranges})
        print(f"d={d:6.0f}  stones/course/stop={n}  window start={s:7.1f}  "
              + "  ".join(f"c{k}:{r[0]:.0f}..{r[1]:.0f}" if r else f"c{k}:-" for k, r in enumerate(ranges))
              + f"  ({time.time() - t0:.0f} s)", flush=True)
    with open(REPO / "results" / "reach_wall.csv", "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    return summary


def deck_study(chk: Checker, cfg: dict) -> dict:
    """Pick poses on the deck (stones in a holder, long axis along ARES y, 1..layers): reachable map, 40 mm grid."""
    a, b, dk = cfg["ares"], cfg["brick"], cfg["deck"]
    xs = frange(-a["length"] / 2 + 60, a["length"] / 2 - 60, 40)
    ys = frange(-a["width"] / 2 + 60, a["width"] / 2 - 60, 40)
    layers = list(range(1, dk["layers"] + 1))
    maps, rows = {}, []
    for layer in layers:
        z = a["deck_top_z"] + dk["holder_z"] + layer * b["height"] + 2.0   # +2 mm: stone clear of the one below
        for y in ys:
            for x in xs:
                pose = transl(x, y, z) * rotz(PI / 2) * rotx(PI)            # stone length along ARES y
                r = chk.check_both(pose)
                maps[(layer, x, y)] = r[0]
                rows.append({"layer": layer, "x": x, "y": y, "ok": int(r[0]), "reason": r[1]})
    with open(REPO / "results" / "reach_deck.csv", "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    return {"xs": xs, "ys": ys, "layers": layers, "map": maps}


def write_summary(cfg: dict, wall: list, deck: dict | None, coarse: bool) -> None:
    b, w, u, t = cfg["brick"], cfg["wall"], cfg["ur5"], cfg["tool"]
    pitch = b["length"] + b["head_joint"]
    lines = ["# Reach study ARES + UR5", "",
             f"Generated by `robodk/reach_study.py` ({'coarse' if coarse else 'full'} grid). Inputs: `config/station.toml`.",
             "", f"Inputs: wall on the **{w['side']}** of ARES, {w['courses']} courses of the full stone "
             f"{b['length']:.0f} × {b['width']:.0f} × {b['height']:.0f} mm (CAD) in running bond on a {w['base_z']:.0f} mm "
             f"base plate (PLACEHOLDER); UR5 at x = {u['mount_x']} mm on the deck z = {u['mount_z']} mm (height "
             f"PLACEHOLDER); TCP {t['tcp_z']:.0f} mm (PLACEHOLDER), gripper envelope {t['size_x']:.0f} × {t['size_y']:.0f} mm.",
             "", "Criterion: IK solution, vertical linear approach "
             f"{cfg['study']['approach']:.0f} mm, collision-free against ARES (STEP) and the arm itself, held stone included.",
             "", "| wall dist. [mm] | stones/course/stop | ARES move per stop L [mm] | reachable u per course [mm] |",
             "|---:|---:|---:|---|"]
    for s in wall:
        rng = ", ".join(f"c{k}: {r[0]:.0f}…{r[1]:.0f}" if r else f"c{k}: –" for k, r in enumerate(s["ranges"]))
        lines.append(f"| {s['wall_dist']:.0f} | {s['n']} | {s['n'] * pitch:.0f} | {rng} |")
    if deck:
        nl = len(deck["layers"])
        lines += ["", f"## Pick area on the deck (top view, x forward →, rows = y from left (+) to right (−))", "",
                  f"`#` = reachable for all {nl} layers, `+` = only some layers, `.` = not reachable, `U` = UR5 base", ""]
        lines.append("```")
        for y in reversed(deck["ys"]):
            row = ""
            for x in deck["xs"]:
                if abs(x - u["mount_x"]) < 80 and abs(y - u["mount_y"]) < 80:
                    row += "U"
                    continue
                v = [deck["map"][(lay, x, y)] for lay in deck["layers"]]
                row += "#" if all(v) else "+" if any(v) else "."
            lines.append(f"y={y:6.0f} {row}")
        lines.append(f"         x = {deck['xs'][0]:.0f} … {deck['xs'][-1]:.0f} mm, step 40 mm")
        lines.append("```")
    (REPO / "results" / "reach_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg = load_config()
    coarse = "--coarse" in sys.argv
    RDK = connect()
    RDK.Render(False)
    chk = Checker(RDK, cfg, cfg["study"]["check_collisions"])
    try:
        wall = wall_study(chk, cfg, coarse)
        deck = deck_study(chk, cfg) if "--deck" in sys.argv else None
        write_summary(cfg, wall, deck, coarse)
    finally:
        chk.close()
        chk.robot.setJoints([0, -90, 0, -90, 0, 0])
        RDK.Render(True)
    print("Wrote results/reach_summary.md")


if __name__ == "__main__":
    main()
