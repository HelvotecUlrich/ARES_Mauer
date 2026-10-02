"""Build the RoboDK station ARES + UR5 + gripper + nominal wall from config/station.toml.

Usage (Windows Python, RoboDK is started if needed):
    py.exe robodk/build_station.py [--snapshot]

Rebuilds the station from scratch each run (an open station with the same name is closed first) and saves it to
robodk/ARES_UR5_Mauer.rdk (not versioned: contains the STEP mesh).
"""
from __future__ import annotations

import sys

from rdk_common import (REPO, STATION_NAME, as_mat, connect, course_shift, course_top_z, load_config,
                        snapshot, stone_points, tcp_pose, transl, ur5_base_pose, wall_frame)
from robodk.robolink import COLLISION_OFF, COLLISION_ON, ITEM_TYPE_STATION, PROJECTION_ALONG_NORMAL

GREY = [0.75, 0.75, 0.75, 1.0]
STONE = [0.72, 0.30, 0.20, 1.0]


def wall_stones(cfg: dict, u_from: float, u_to: float) -> list:
    """(u_centre, z_top, course) of all stones of the nominal wall between u_from and u_to."""
    b = cfg["brick"]
    pitch = b["length"] + b["head_joint"]
    out = []
    for k in range(cfg["wall"]["courses"]):
        first = course_shift(cfg, k) + b["length"] / 2
        i = int((u_from - first) // pitch)
        while first + i * pitch - b["length"] / 2 <= u_to:
            uc = first + i * pitch
            if uc + b["length"] / 2 >= u_from:
                out.append((uc, course_top_z(cfg, k), k))
            i += 1
    return out


def exclude_from_collisions(RDK, robot, tool, obj) -> None:
    for link in range(0, 8):
        RDK.setCollisionActivePair(COLLISION_OFF, robot, obj, link, 0)
    RDK.setCollisionActivePair(COLLISION_OFF, tool, obj, 0, 0)


def main() -> None:
    cfg = load_config()
    want_snapshot = "--snapshot" in sys.argv
    RDK = connect()
    RDK.Render(False)

    for st in RDK.ItemList(ITEM_TYPE_STATION):
        if st.Name() == STATION_NAME:
            st.Delete()
    station = RDK.AddStation(STATION_NAME)

    # ARES (STEP in base_link coordinates)
    f_ares = RDK.AddFrame("ARES base_link", station)
    # the STEP import (tessellation) is slow -> cache the mesh once next to the STEP (same coordinates)
    step = REPO / cfg["ares"]["step_file"]
    mesh = step.with_suffix(".stl")
    if mesh.exists() and mesh.stat().st_mtime > step.stat().st_mtime:
        ares = RDK.AddFile(str(mesh), f_ares)
    else:
        ares = RDK.AddFile(str(step), f_ares)
        if ares.Valid():
            ares.Save(str(mesh))
    if not ares.Valid():
        raise RuntimeError("ARES import failed")
    ares.setName("ARES_STEP_2026-09-23")
    ares.setColor(GREY)

    # Check the deck height at the mount position with vertical rays
    u = cfg["ur5"]
    rays = [[u["mount_x"] + dx, u["mount_y"] + dy, 2000.0, 0.0, 0.0, 1.0]   # RoboDK projects against the normal
            for dx, dy in ((0, 0), (-60, 0), (60, 0), (0, -60), (0, 60))]
    hits = RDK.ProjectPoints(rays, ares, PROJECTION_ALONG_NORMAL)
    print("Deck height at the UR5 mount (5 rays):", ", ".join(f"{h[2]:.1f}" for h in hits))

    # UR5 on its mount frame
    f_mount = RDK.AddFrame("UR5 mount", f_ares)
    f_mount.setPose(ur5_base_pose(cfg))
    robot = RDK.AddFile(str(REPO / u["robot_file"]), f_mount)
    if not robot.Valid():
        raise RuntimeError("UR5 import failed")
    robot.setName("UR5")
    robot.Parent().setPose(transl(0, 0, 0))   # the library file carries a 1000 mm base offset

    # Gripper (real geometry from Gino's station) with the TCP from the config
    t = cfg["tool"]
    tool = RDK.AddFile(str(REPO / t["tool_file"]), robot)
    if not tool.Valid():
        raise RuntimeError("gripper tool import failed")
    tool.setName("Gripper_EHPS20")
    tool.setPoseTool(tcp_pose(cfg))   # TCP x along the stone length, y = jaw closing direction
    robot.setPoseTool(tool)

    # Nominal wall with the real stone mesh (visual only, excluded from collision checks)
    w = cfg["wall"]
    f_wall = RDK.AddFrame("Wall", station)   # fixed in the world; ARES (base_link) moves between stops
    f_wall.setPose(wall_frame(cfg, w["dist_nominal"]))
    pts = []
    for uc, z_top, _ in wall_stones(cfg, w["u_from"], w["u_to"]):
        pts += stone_points(cfg, "wall", u=uc, z_top=z_top)
    wall = RDK.AddShape(as_mat(pts))      # AddShape cannot attach to a frame -> create, then re-parent
    wall.setParent(f_wall)
    wall.setName("Wall_nominal")
    wall.setColor(STONE)

    RDK.setCollisionActive(COLLISION_ON)
    RDK.setCollisionActivePair(COLLISION_OFF, robot, ares, 0, 0)   # UR5 base is bolted to the deck
    exclude_from_collisions(RDK, robot, tool, wall)

    robot.setJoints([0, -90, 0, -90, 0, 0])
    RDK.Render(True)
    out = REPO / "robodk" / f"{STATION_NAME}.rdk"
    RDK.Save(str(out), station)
    print("Saved", out)

    if want_snapshot:
        img = REPO / "results" / "station_view.png"
        ok = snapshot(RDK, img, eye=[2600, -2400, 1900], target=[400, 0, 300])
        print("Snapshot", img, "ok" if ok else "FAILED")


if __name__ == "__main__":
    main()
