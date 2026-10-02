"""Show the stones of one stop for a given wall distance (from results/reach_wall.csv) in the open station.

Usage (Windows Python, after build_station.py and reach_study.py):
    py.exe robodk/show_stop.py [wall_dist_mm] [--snapshot]

Green = stones placed from this stop (running bond, stepped ends), red = rest of the nominal wall. The arm is put
just above the place pose of the last stone of the top course.
"""
from __future__ import annotations

import csv
import sys

from build_station import STONE, exclude_from_collisions, wall_stones
from rdk_common import (REPO, as_mat, connect, course_shift, course_top_z, load_config, place_pose, snapshot,
                        stone_points, transl, ur5_base_pose, wall_frame)
from reach_study import best_stop
from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
from robodk.robomath import invH

GREEN = [0.25, 0.70, 0.30, 1.0]


def main() -> None:
    cfg = load_config()
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    d = float(args[0]) if args else cfg["wall"]["dist_nominal"]
    b, w = cfg["brick"], cfg["wall"]
    pitch = b["length"] + b["head_joint"]

    ok, us = {}, set()
    with open(REPO / "results" / "reach_wall.csv", newline="") as f:
        for r in csv.DictReader(f):
            if abs(float(r["wall_dist"]) - d) < 1e-6:
                ok[(int(r["course"]), float(r["u"]))] = r["ok"] == "1"
                us.add(float(r["u"]))
    if not ok:
        raise SystemExit(f"wall distance {d} not in reach_wall.csv")
    n, s = best_stop(ok, cfg, sorted(us))
    print(f"wall distance {d:.0f} mm: {n} stones per course per stop, ARES move per stop {n * pitch:.0f} mm, "
          f"window start {s:.0f} mm")

    RDK = connect()
    RDK.Render(False)
    f_wall = RDK.Item("Wall", ITEM_TYPE_FRAME)
    robot = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tool = RDK.Item("Gripper_EHPS20", ITEM_TYPE_TOOL)
    for name in ("Wall_nominal", "Stones_stop", "Stones_rest"):
        it = RDK.Item(name, ITEM_TYPE_OBJECT)
        if it.Valid():
            it.Delete()
    f_wall.setPose(wall_frame(cfg, d))

    stop, rest = [], []
    for uc, z_top, k in wall_stones(cfg, w["u_from"], w["u_to"]):
        i = round((uc - s - course_shift(cfg, k) - b["length"] / 2) / pitch)
        (stop if 0 <= i < n else rest).extend(stone_points(cfg, "wall", u=uc, z_top=z_top))
    for name, pts, col in (("Stones_stop", stop, GREEN), ("Stones_rest", rest, STONE)):
        obj = RDK.AddShape(as_mat(pts))
        obj.setParent(f_wall)
        obj.setName(name)
        obj.setColor(col)
        exclude_from_collisions(RDK, robot, tool, obj)

    # Arm just above the last stone of the top course (solution closest to an elbow-up seed)
    k = w["courses"] - 1
    u_last = s + course_shift(cfg, k) + b["length"] / 2 + (n - 1) * pitch
    pose = transl(0, 0, 60) * place_pose(cfg, u_last, d, course_top_z(cfg, k))
    j = robot.SolveIK(pose, [0, -90, 90, -90, -90, 0], tool=transl(0, 0, cfg["tool"]["tcp_z"]),
                      reference=invH(ur5_base_pose(cfg))).list()
    if len(j) >= 6:
        robot.setJoints(j[:6])
    RDK.Render(True)

    if "--snapshot" in sys.argv:
        img = REPO / "results" / f"stop_{w['side']}_d{d:.0f}.png"
        snapshot(RDK, img, eye=[d + 1900, -2300, 1700], target=[d * 0.6, 200, 250])
        print("Snapshot", img)


if __name__ == "__main__":
    main()
