"""Build the RoboDK station ARES + UR5 + gripper + flange camera + reference boards + nominal wall from
config/station.toml.

Usage (Windows Python):
    py.exe robodk/build_station.py [--snapshot]           # running RoboDK (started if needed), saves the .rdk below
    py.exe robodk/build_station.py --new-instance --out C:/temp/station.rdk   # own RoboDK on its own port

Rebuilds the station from scratch each run (an open station with the same name is closed first) and saves it to
robodk/ARES_UR5_Mauer.rdk, with [[wall.legs]] to robodk/ARES_UR5_Mauer_L.rdk (not versioned: contains the STEP mesh).

Camera ([camera], [camera.mount] = PLACEHOLDER): robot tool "Camera" with TCP = T_flange_cam (the simulated camera of
sim_camera.py attaches to it) and the object "Camera_body" (housing + lens, rdk_common.camera_body_points) attached
to the gripper. RoboDK does not check objects attached to a tool against static objects by default, so its collision
pairs are switched on explicitly (ARES, boards, pick-up table, arm links 0-5).

Boards ([boards.calib], [boards.ref], [[targets]]): ChArUco boards as flat textured objects from PNGs generated here
with OpenCV (cv2.aruco.CharucoBoard, marker ids first_id ..). Board frame = OpenCV 4.14 CharucoBoard frame: origin
at the top-left outer corner of the chessboard, x right, y down, z into the board. Wall boards hang in the "Wall"
frame, station boards in the "Pickup station" frame, the calibration board in the ARES frame (hidden: it is only on
the deck during the hand-eye calibration). A white quiet zone of one square around each board is an ASSUMPTION of
this model (print layout: tools/print_targets.py).

Wall: with [[wall.legs]] (config [wall] shape "L") the nominal L is drawn: every leg in its own frame ("Leg A",
"Leg B" under "Wall", T_wall_leg from xyz_in_wall / rpy_in_wall_deg), stones from robodk/wallplan.py layout_legs (leg
rectangles in running bond, half stones at both ends of the odd courses), full stones with the CAD mesh, half stones
with cad/stone_half_placeholder.stl (robodk/make_half_stone.py, PLACEHOLDER; the [half_brick] box if the file is
missing) in a lighter colour - one object "Wall_nominal" with two shapes. Leg boards (W5..W7 on leg B) come from
mauer.reference.placements. Without legs the straight wall [wall] u_from .. u_to as before.
"""
from __future__ import annotations

import argparse
import ast
import tempfile
from pathlib import Path

from rdk_common import (REPO, STATION_NAME, as_mat, box_points, camera_body_points, close_instance, connect,
                        course_shift, course_top_z, leg_frame, load_config, set_tool_object_collisions, snapshot,
                        stone_points, stone_points_kind, tcp_pose, transform_points, transl, ur5_base_pose, wall_frame)
from rdk_common import T_flange_cam as T_flange_cam_mat
from robodk.robolink import COLLISION_OFF, COLLISION_ON, ITEM_TYPE_STATION, PROJECTION_ALONG_NORMAL
from robodk.robomath import Mat, invH

GREY = [0.75, 0.75, 0.75, 1.0]
STONE = [0.72, 0.30, 0.20, 1.0]
HALF_STONE = [0.93, 0.62, 0.30, 1.0]   # half stones (PLACEHOLDER mesh) in a lighter colour
CAMERA = [0.15, 0.15, 0.18, 1.0]
ADAPTER = [0.85, 0.55, 0.10, 1.0]
TABLE = [0.55, 0.45, 0.30, 1.0]
BOARD_PX_PER_MM = 16.0             # texture resolution: 0.0625 mm/px, finer than the camera (0.073 mm/px at 320 mm)
TABLE_TOP_MM = 30.0                # ASSUMPTION: thickness of the modelled pick-up table top (no legs modelled)


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


def l_wall_stones(cfg: dict) -> list:
    """Stones of the nominal L (robodk/wallplan.py legs + layout_legs, [wall] half_stones), [] without legs."""
    import wallplan
    legs_ = wallplan.legs(cfg)
    if not legs_:
        return []
    return wallplan.layout_legs(cfg, legs_, bool(cfg["wall"].get("half_stones", True)))


def l_wall_points(cfg: dict, stones: list) -> tuple[list, list]:
    """(full, half) mesh vertices of the given wallplan stones in the WALL frame (leg frame -> T_wall_leg)."""
    full, half = [], []
    by_leg: dict = {}
    for s in stones:
        by_leg.setdefault(s.leg, []).append(s)
    for leg, ss in by_leg.items():
        T = leg_frame(cfg, leg or None)
        for s in ss:
            pts = stone_points_kind(cfg, "wall", u=s.u, z_top=s.z_top, kind=s.kind)
            (half if s.kind == "half" else full).extend(transform_points(T, pts))
    return full, half


def add_l_wall(RDK, cfg: dict, f_wall) -> tuple:
    """Nominal L: frames "Leg <name>" under the wall frame and one object "Wall_nominal" (shape 0 = full stones,
    shape 1 = half stones, if any). Returns (wall object, {leg name: frame}, stones)."""
    import wallplan
    frames = {}
    for lg in wallplan.legs(cfg):
        f = RDK.AddFrame(f"Leg {lg.name}", f_wall)
        f.setPose(leg_frame(cfg, lg.name))
        frames[lg.name] = f
    stones = l_wall_stones(cfg)
    full, half = l_wall_points(cfg, stones)
    shapes = [as_mat(full), STONE] + ([as_mat(half), HALF_STONE] if half else [])
    wall = RDK.AddShape(shapes)
    wall.setParent(f_wall)
    wall.setName("Wall_nominal")
    n_half = sum(s.kind == "half" for s in stones)
    print(f"nominal L: legs {', '.join(frames)}; {len(stones)} stones ({n_half} half)", flush=True)
    return wall, frames, stones


def exclude_from_collisions(RDK, robot, tool, obj) -> None:
    for link in range(0, 8):
        RDK.setCollisionActivePair(COLLISION_OFF, robot, obj, link, 0)
    RDK.setCollisionActivePair(COLLISION_OFF, tool, obj, 0, 0)


# ── boards ────────────────────────────────────────────────────────────────────
def board_layout(cfg: dict) -> list:
    """All boards of the config as dicts: name, parent ("deck" = ARES frame, "wall", "station"), geometry
    (squares_x, squares_y, square_mm, marker_mm, dictionary, first_id) and the board pose xyz [mm] / rpy_deg in the
    parent frame (a leg board of the L is converted from its leg frame to the wall frame, mauer.reference.placements).
    The calibration board is named "calib"; targets inherit the geometry of [boards.ref]."""
    keys = ("squares_x", "squares_y", "square_mm", "marker_mm", "dictionary")
    c = cfg["boards"]["calib"]
    out = [{"name": "calib", "parent": "deck", **{k: c[k] for k in keys}, "first_id": c["first_id"],
            "xyz": c["xyz"], "rpy_deg": c.get("rpy_deg", [0.0, 0.0, 0.0])}]
    ref = cfg["boards"]["ref"]
    from mauer.geometry import xyz_rpy
    from mauer.reference import placements
    T = {p.name: p.T_parent_board for p in placements(cfg)}         # leg boards: T_wall_leg @ pose (L wall)
    for t in cfg.get("targets", []):
        xyz, rpy = t["xyz"], t.get("rpy_deg", [0.0, 0.0, 0.0])
        if "leg" in t:                                             # [[targets]] leg = "B": xyz/rpy in the leg frame
            xyz, rpy = (v.tolist() for v in xyz_rpy(T[t["name"]]))
        out.append({"name": t["name"], "parent": t["parent"], **{k: t.get(k, ref[k]) for k in keys},
                    "first_id": t["first_id"], "xyz": xyz, "rpy_deg": rpy})
    return out


def charuco_board(spec: dict):
    """cv2.aruco.CharucoBoard of a board spec with the explicit marker ids first_id .. first_id + n - 1."""
    import cv2
    import numpy as np
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    size = (int(spec["squares_x"]), int(spec["squares_y"]))
    n = len(cv2.aruco.CharucoBoard(size, spec["square_mm"], spec["marker_mm"], d).getIds())
    ids = np.arange(spec["first_id"], spec["first_id"] + n, dtype=np.int32)
    return cv2.aruco.CharucoBoard(size, float(spec["square_mm"]), float(spec["marker_mm"]), d, ids)


def board_png(spec: dict, path: Path, px_per_mm: float = BOARD_PX_PER_MM, margin_squares: int = 1) -> dict:
    """Write the board as a PNG (whole pixels per square, white quiet zone of `margin_squares` squares) and return
    its geometry: path, mm_per_px, margin_mm, image size [px]."""
    import cv2
    sq_px = int(round(spec["square_mm"] * px_per_mm))
    m = margin_squares * sq_px
    w, h = int(spec["squares_x"]) * sq_px + 2 * m, int(spec["squares_y"]) * sq_px + 2 * m
    img = charuco_board(spec).generateImage((w, h), marginSize=m, borderBits=1)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), img):
        raise RuntimeError(f"could not write {path}")
    mm_per_px = spec["square_mm"] / sq_px
    return {"path": path, "mm_per_px": mm_per_px, "margin_mm": m * mm_per_px, "size_px": (w, h)}


def add_board(RDK, spec: dict, parent, T_parent_board: Mat, png_dir: Path | None = None):
    """Board as a flat textured object, child of `parent`, with its board frame at T_parent_board.

    RoboDK loads a PNG at 96 dpi as a flat object with its origin at the image corner, pixel (col, row) -> (+x, +y)
    and the readable side facing -z (research 2026-10-05) - exactly the OpenCV board frame shifted by the quiet zone,
    so only a scale and transl(-margin, -margin, 0) are needed."""
    png_dir = png_dir or Path(tempfile.gettempdir()) / "ares_mauer_boards"
    geo = board_png(spec, png_dir / f"board_{spec['name']}_{spec['dictionary']}_{spec['first_id']}.png")
    item = RDK.AddFile(str(geo["path"]))
    if not item.Valid():
        raise RuntimeError(f"board import failed: {geo['path']}")
    bb = item.setParam("BoundingBox")
    bb = ast.literal_eval(bb) if isinstance(bb, str) else bb
    size_mm = geo["size_px"][0] * geo["mm_per_px"]
    item.Scale(size_mm / float(bb["size"][0]))
    item.setParent(parent)
    item.setName(f"Board_{spec['name']}")
    item.setPose(T_parent_board * transl(-geo["margin_mm"], -geo["margin_mm"], 0))
    return item


def board_pose(spec: dict) -> Mat:
    """T_parent_board of a board spec (xyz [mm], rpy_deg as in mauer.geometry.pose_xyz_rpy)."""
    from mauer.geometry import pose_xyz_rpy, to_robodk
    return to_robodk(pose_xyz_rpy(spec["xyz"], spec["rpy_deg"]))


# ── pick-up station ───────────────────────────────────────────────────────────
def T_wall_station(cfg: dict) -> Mat:
    """Pick-up station frame in the wall frame, literally from [pickup_station] xyz_in_wall / rpy_in_wall_deg
    (z = 0: the frame lies at floor level; the table top is at z = table_z in it)."""
    from mauer.geometry import pose_xyz_rpy, to_robodk
    p = cfg["pickup_station"]
    return to_robodk(pose_xyz_rpy(p["xyz_in_wall"], p["rpy_in_wall_deg"]))


def T_station_ares(cfg: dict) -> Mat:
    """ARES base_link in the station frame when docked ([pickup_station] ares_xyz / ares_rpy_deg, PLACEHOLDER)."""
    from mauer.geometry import pose_xyz_rpy, to_robodk
    p = cfg["pickup_station"]
    return to_robodk(pose_xyz_rpy(p["ares_xyz"], p["ares_rpy_deg"]))


def table_extent(cfg: dict) -> tuple:
    """(x_max, y_max) of the modelled table top in the station frame, from the origin (front-left corner):
    mauer.floor.station_table_extent ([pickup_station] table_size, PLACEHOLDER) - the same table the routes avoid."""
    from mauer.floor import station_table_extent
    return station_table_extent(cfg)


def add_pickup_station(RDK, cfg: dict, f_wall):
    """Frame "Pickup station" (child of the wall frame) and its table top as a slab (PLACEHOLDER layout)."""
    f_station = RDK.AddFrame("Pickup station", f_wall)
    f_station.setPose(T_wall_station(cfg))
    x_max, y_max = table_extent(cfg)
    z = cfg["pickup_station"]["table_z"]
    table = RDK.AddShape(as_mat(box_points(x_max, y_max, TABLE_TOP_MM, x_max / 2, y_max / 2, z - TABLE_TOP_MM / 2)))
    table.setParent(f_station)
    table.setName("Pickup_table")
    table.setColor(TABLE)
    return f_station, table


# ── camera ────────────────────────────────────────────────────────────────────
def add_camera(RDK, cfg: dict, robot, tool, T_fc: Mat | None = None):
    """Flange camera: robot tool "Camera" (TCP = T_flange_cam) and the body object "Camera_body" attached to the
    gripper tool (objects under a tool are posed relative to its TCP). The gripper stays the active tool.
    Returns (camera tool, body)."""
    T_fc = T_fc if T_fc is not None else T_flange_cam_mat(cfg)
    cam_tool = robot.AddTool(T_fc, "Camera")
    robot.setPoseTool(tool)
    body = RDK.AddShape(as_mat(camera_body_points(cfg)))
    body.setParent(tool)
    body.setName("Camera_body")
    body.setPose(invH(tcp_pose(cfg)) * T_fc)
    body.setColor(CAMERA)
    return cam_tool, body


def add_camera_adapter(RDK, cfg: dict, tool):
    """Camera adapter plate ([camera.adapter] STEP, in the UR flange frame) attached to the gripper tool; objects under
    a tool are posed relative to its TCP, so the plate gets invH(TCP). The STEP mesh is cached as STL next to it."""
    step = REPO / cfg["camera"]["adapter"]["step_file"]
    mesh = step.with_suffix(".stl")
    if mesh.exists() and mesh.stat().st_mtime > step.stat().st_mtime:
        adapter = RDK.AddFile(str(mesh))
    else:
        adapter = RDK.AddFile(str(step))
        if adapter.Valid():
            adapter.Save(str(mesh))
    if not adapter.Valid():
        raise RuntimeError(f"camera adapter import failed: {step}")
    adapter.setParent(tool)
    adapter.setName("Camera_adapter_018660")
    adapter.setPose(invH(tcp_pose(cfg)))
    adapter.setColor(ADAPTER)
    return adapter


def set_camera_mount(cfg: dict, cam_tool, body, T_fc: Mat) -> None:
    """Move the camera (tool TCP and body) to another flange mount T_fc."""
    cam_tool.setPoseTool(T_fc)
    body.setPose(invH(tcp_pose(cfg)) * T_fc)


# ── station ───────────────────────────────────────────────────────────────────
def build(RDK, cfg: dict, camera: bool = True, boards: bool = True, pickup: bool = True) -> dict:
    """Build the station in RDK (replacing an open station of the same name) and return its items by role:
    station, f_ares, ares, f_mount, robot, tool, f_wall, wall, [legs {name: frame}, l_stones (L wall)],
    [cam_tool, cam_body, cam_adapter], [boards {name: item}], [f_station, table]."""
    RDK.Render(False)
    for st in RDK.ItemList(ITEM_TYPE_STATION):
        if st.Name() == STATION_NAME:
            st.Delete()
    station = RDK.AddStation(STATION_NAME)
    it = {"station": station}

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
    print("Deck height at the UR5 mount (5 rays):", ", ".join(f"{h[2]:.1f}" for h in hits), flush=True)

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
    # the camera adapter plate sits between flange and gripper: Gino's gripper geometry moves out by adapter_z
    tool.setGeometryPose(transl(0, 0, t.get("adapter_z", 0.0)) * tool.GeometryPose())
    tool.setPoseTool(tcp_pose(cfg))   # TCP x along the stone length, y = jaw closing direction
    robot.setPoseTool(tool)

    # Nominal wall with the real stone mesh (visual only, excluded from collision checks)
    w = cfg["wall"]
    f_wall = RDK.AddFrame("Wall", station)   # fixed in the world; ARES (base_link) moves between stops
    f_wall.setPose(wall_frame(cfg, w["dist_nominal"]))
    if w.get("legs"):                       # L wall ([[wall.legs]]): legs in their own frames, full + half stones
        wall, it["legs"], it["l_stones"] = add_l_wall(RDK, cfg, f_wall)
    else:
        pts = []
        for uc, z_top, _ in wall_stones(cfg, w["u_from"], w["u_to"]):
            pts += stone_points(cfg, "wall", u=uc, z_top=z_top)
        wall = RDK.AddShape(as_mat(pts))      # AddShape cannot attach to a frame -> create, then re-parent
        wall.setParent(f_wall)
        wall.setName("Wall_nominal")
        wall.setColor(STONE)
    it.update(f_ares=f_ares, ares=ares, f_mount=f_mount, robot=robot, tool=tool, f_wall=f_wall, wall=wall)

    obstacles = [ares]
    if pickup:
        it["f_station"], it["table"] = add_pickup_station(RDK, cfg, f_wall)
        obstacles.append(it["table"])
    if boards:
        it["boards"] = {}
        parents = {"deck": f_ares, "wall": f_wall, "station": it.get("f_station")}
        for spec in board_layout(cfg):
            parent = parents.get(spec["parent"])
            if parent is None:
                print(f"board {spec['name']}: parent {spec['parent']!r} not built - skipped", flush=True)
                continue
            b = add_board(RDK, spec, parent, board_pose(spec))
            if spec["name"] == "calib":
                b.setVisible(False)       # only on the deck during the hand-eye calibration (hidden = no collisions)
            it["boards"][spec["name"]] = b
            obstacles.append(b)
        print(f"boards: {', '.join(it['boards'])}", flush=True)
    if camera:
        it["cam_tool"], it["cam_body"] = add_camera(RDK, cfg, robot, tool)
        if "adapter" in cfg["camera"]:
            it["cam_adapter"] = add_camera_adapter(RDK, cfg, tool)

    RDK.setCollisionActive(COLLISION_ON)
    RDK.setCollisionActivePair(COLLISION_OFF, robot, ares, 0, 0)   # UR5 base is bolted to the deck
    exclude_from_collisions(RDK, robot, tool, wall)
    if camera:
        set_tool_object_collisions(RDK, it["cam_body"], robot, tool, obstacles)
        if "cam_adapter" in it:
            set_tool_object_collisions(RDK, it["cam_adapter"], robot, tool, obstacles)

    robot.setJoints([0, -90, 0, -90, 0, 0])
    RDK.Render(True)
    return it


def main(argv: list | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], allow_abbrev=False)
    ap.add_argument("--snapshot", action="store_true", help="render results/station_view.png")
    ap.add_argument("--new-instance", action="store_true",
                    help="start an own RoboDK on --port (the running one is not touched); closed at the end")
    ap.add_argument("--port", type=int, default=None, help="API port for --new-instance (default 20599)")
    ap.add_argument("--out", type=Path, default=None,
                    help=f".rdk to save (default robodk/{STATION_NAME}.rdk, the L: robodk/{STATION_NAME}_L.rdk; with "
                         "--new-instance only if given)")
    ap.add_argument("--no-camera", action="store_true", help="without flange camera")
    ap.add_argument("--no-boards", action="store_true", help="without reference boards and pick-up station")
    args, _ = ap.parse_known_args(argv)    # tolerant: simulate.py calls main() with its own sys.argv

    cfg = load_config()
    RDK = connect(new_instance=args.new_instance, port=args.port)
    try:
        it = build(RDK, cfg, camera=not args.no_camera, boards=not args.no_boards, pickup=not args.no_boards)
        suffix = "_L" if cfg["wall"].get("legs") else ""        # the L never overwrites the straight-wall station
        out = args.out or (None if args.new_instance else REPO / "robodk" / f"{STATION_NAME}{suffix}.rdk")
        if out is not None:
            RDK.Save(str(out), it["station"])
            print("Saved", out, flush=True)
        if args.snapshot:
            img = REPO / "results" / "station_view.png"
            ok = snapshot(RDK, img, eye=[2600, -2400, 1900], target=[400, 0, 300])
            print("Snapshot", img, "ok" if ok else "FAILED", flush=True)
    finally:
        if args.new_instance:
            close_instance(RDK)


if __name__ == "__main__":
    main()
