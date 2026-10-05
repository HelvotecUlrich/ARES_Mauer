"""Shared helpers: config loading, RoboDK connection, wall frame, stone mesh, flange camera, snapshots.

Run the scripts with Windows Python (RoboDK runs on Windows), e.g. from WSL:
    py.exe robodk/build_station.py
The RoboDK API is taken from the RoboDK installation (C:\\RoboDK\\Python), no pip install needed.

Frames
  ARES   base_link: floor, centre between the steering axes, x forward, y left, z up.
  wall   origin on the wall centreline (floor level), x along the wall = direction ARES travels between stops,
         y towards ARES, z up. A stone at wall position u has its top centre at (u, 0, z_top), long axis along x.
  tool   z along the flange axis (pointing down when placing), x along the stone length, y = jaw closing direction,
         TCP = top centre of the held stone.
  cam    OpenCV camera frame (z = optical axis, x = image right, y = image down, origin = projection centre); the
         RoboDK simulated camera uses the same axes (research 2026-10-05: patch at cam +x appears at image right).

Separate instance: connect(new_instance=True) starts an own RoboDK process on its own API port and refuses to work
with an instance that it did not start or that already holds a station named like the user's (STATION_NAME). Flags
(research 2026-10-05, RoboDK 6.0.0.26652): never -SKIPINI (drops the licence -> "Free (Limited)" refuses the 3rd
camera), never -HIDDEN (crashes on Cam2D_Add); the window is minimised instead, cameras still render.
"""
from __future__ import annotations

import struct
import sys
import time
import tomllib
from pathlib import Path

ROBODK_PY = Path(r"C:\RoboDK\Python")
if str(ROBODK_PY) not in sys.path:
    sys.path.insert(0, str(ROBODK_PY))

from robodk.robolink import (COLLISION_OFF, COLLISION_ON, ITEM_TYPE_STATION, WINDOWSTATE_MINIMIZED,  # noqa: E402
                             Robolink)
from robodk.robomath import Mat, rotx, rotz, transl  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:                      # mauer/ (pure numpy helpers) next to robodk/
    sys.path.insert(0, str(REPO))
STATION_NAME = "ARES_UR5_Mauer"
PI = 3.141592653589793
DEG = PI / 180.0
NEW_INSTANCE_PORT = 20599                          # own API port; the user's RoboDK listens on 20500/20501
NEW_INSTANCE_ARGS = ("-NEWINSTANCE", "-NOSPLASH", "-EXIT_LAST_COM")


def load_config() -> dict:
    with open(REPO / "config" / "station.toml", "rb") as f:
        return tomllib.load(f)


def connect(new_instance: bool = False, port: int | None = None, minimized: bool = True) -> Robolink:
    """Connect to RoboDK. Generous API timeout: large imports can take minutes.

    new_instance=False (default, unchanged behaviour): connect to the running RoboDK (or start one) - this is the
    user's instance; port selects a specific API port.
    new_instance=True: start a separate RoboDK on `port` (default NEW_INSTANCE_PORT). Raises RuntimeError without
    touching anything if the port already belongs to a RoboDK we did not start, or if the instance holds a station
    named like STATION_NAME (i.e. it could be the user's). Close it with close_instance()."""
    if not new_instance:
        RDK = Robolink() if port is None else Robolink(port=port)
    else:
        port = port or NEW_INSTANCE_PORT
        RDK = Robolink(port=port, args=list(NEW_INSTANCE_ARGS))
        if RDK.NEW_INSTANCE is None:              # connected to an already running RoboDK on that port
            try:
                RDK.Disconnect()
            except Exception:  # noqa: BLE001
                pass
            raise RuntimeError(f"a RoboDK that this script did not start already listens on port {port} - "
                               "refusing to use it (choose another --port)")
        names = [st.Name() for st in RDK.ItemList(ITEM_TYPE_STATION)]
        if any(n.startswith(STATION_NAME) for n in names):
            close_instance(RDK)                   # we started it, so we may close it - without touching the station
            raise RuntimeError(f"new RoboDK instance on port {port} already holds {names} - refusing to use it")
        if minimized:
            RDK.setWindowState(WINDOWSTATE_MINIMIZED)
    RDK.TIMEOUT = 600
    RDK.COM.settimeout(RDK.TIMEOUT)
    return RDK


def close_instance(RDK: Robolink, timeout_s: float = 30.0) -> int | None:
    """Close a RoboDK started by connect(new_instance=True) and return its exit code. Does nothing for an instance
    this process did not start (the user's). RoboDK 6.0 sometimes crashes on exit after collision tests with tool
    objects (research 2026-10-05, 0xc0000005) - harmless once the results are written."""
    proc = RDK.NEW_INSTANCE
    if proc is None:
        return None
    for call in (lambda: RDK.Cam2D_Close(0), RDK.CloseRoboDK):
        try:
            call()
        except Exception as e:  # noqa: BLE001 - closing must not mask the real result
            print(f"close_instance: {e}", flush=True)
    try:
        proc.wait(timeout=timeout_s)
    except Exception:  # noqa: BLE001
        proc.kill()
    return proc.returncode


def ur5_base_pose(cfg: dict) -> Mat:
    """UR5 base frame in the ARES frame."""
    u = cfg["ur5"]
    return transl(u["mount_x"], u["mount_y"], u["mount_z"]) * rotz(u["mount_rz"] * DEG)


def tcp_pose(cfg: dict) -> Mat:
    """TCP in the flange frame. The jaws of the Backengreifer close along the flange x axis, so the TCP frame is
    turned by 90° about z: TCP x (stone length) = flange y, TCP y (closing direction) = -flange x."""
    return transl(0, 0, cfg["tool"]["tcp_z"]) * rotz(PI / 2)


# Stone CAD frame -> "top centre" frame (origin top centre, x along the length, y across, z up)
T_TC_CAD = Mat([[0, 1, 0, 100], [-1, 0, 0, 60], [0, 0, 1, -120], [0, 0, 0, 1]])


def held_stone_pose() -> Mat:
    """Pose of a held stone object (CAD frame) relative to the TCP (RoboDK attaches tool objects to the TCP)."""
    return rotx(PI) * T_TC_CAD


def wall_frame(cfg: dict, dist: float) -> Mat:
    """Wall frame in the ARES frame for a wall centreline at distance `dist` from the ARES centre."""
    side = cfg["wall"]["side"]
    if side == "right":
        return transl(0, -dist, 0)
    if side == "left":
        return transl(0, dist, 0) * rotz(PI)
    if side == "front":
        return transl(dist, 0, 0) * rotz(PI / 2)
    if side == "rear":
        return transl(-dist, 0, 0) * rotz(-PI / 2)
    raise ValueError(f"wall.side must be right/left/front/rear, not {side!r}")


def course_top_z(cfg: dict, k: int) -> float:
    """Top of course k (0 = first course)."""
    b = cfg["brick"]
    return cfg["wall"]["base_z"] + k * (b["height"] + b["bed_joint"]) + b["height"]


def course_shift(cfg: dict, k: int) -> float:
    """Running-bond offset of course k along the wall."""
    b = cfg["brick"]
    return (k * cfg["wall"]["bond_offset"] % 1.0) * (b["length"] + b["head_joint"])


def place_pose(cfg: dict, u: float, dist: float, z_top: float, flip: bool = False) -> Mat:
    """TCP pose (ARES frame) for a stone with its top centre at wall position u, height z_top."""
    pose = wall_frame(cfg, dist) * transl(u, 0, z_top) * rotx(PI)
    if flip:  # symmetric gripper: may also grip turned by 180° about the vertical
        pose = pose * rotz(PI)
    return pose


# ── stone mesh ────────────────────────────────────────────────────────────────
def load_stl(path: Path) -> list:
    """Triangles [[x,y,z]*3] from a binary or ASCII STL (pure Python, no numpy on Windows)."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, line.split()[1:4])) for line in data.decode().splitlines()
             if line.strip().startswith("vertex")]
        return [v[i:i + 3] for i in range(0, len(v), 3)]
    n = struct.unpack("<I", data[80:84])[0]
    tris = []
    for i in range(n):
        f = struct.unpack("<12f", data[84 + 50 * i: 84 + 50 * i + 48])
        tris.append([list(f[3:6]), list(f[6:9]), list(f[9:12])])
    return tris


def stone_points(cfg: dict, frame: str, u: float = 0.0, z_top: float = 0.0, tcp_z: float = 0.0) -> list:
    """Stone mesh vertices (flat list, 3 per triangle) placed in the wall frame or in the tool (flange) frame.

    Stone CAD frame: x 0..width (ribbed faces), y -length..0, z 0..height (pins below z = 0).
    frame="wall": top centre at (u, 0, z_top), long axis along wall x.
    frame="tool": top centre at the TCP (0, 0, tcp_z) of the flange frame, z pointing into the stone.
    """
    b = cfg["brick"]
    L, W, H = b["length"], b["width"], b["height"]
    pts = []
    for tri in load_stl(REPO / b["mesh"]):
        for sx, sy, sz in tri:
            if frame == "wall":
                pts.append([u + sy + L / 2, W / 2 - sx, z_top - H + sz])   # proper rotation (no mirror)
            else:
                pts.append([sy + L / 2, sx - W / 2, tcp_z + H - sz])
    return pts


def box_points(sx: float, sy: float, sz: float, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0) -> list:
    """36 vertices (12 triangles) of an axis-aligned box centred at (cx, cy, cz)."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
         [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    pts = []
    for a, b_, c, d in quads:
        pts += [v[a], v[b_], v[c], v[a], v[c], v[d]]
    return pts


def as_mat(points: list) -> Mat:
    return Mat(points).tr()


def cylinder_points(r: float, z0: float, z1: float, n: int = 24) -> list:
    """Vertices (3 per triangle) of a closed n-gon prism of radius r along z from z0 to z1 (outward normals)."""
    import math
    ring = [(r * math.cos(2 * PI * i / n), r * math.sin(2 * PI * i / n)) for i in range(n)]
    pts = []
    for i in range(n):
        (xa, ya), (xb, yb) = ring[i], ring[(i + 1) % n]
        pts += [[xa, ya, z0], [xb, yb, z0], [xb, yb, z1], [xa, ya, z0], [xb, yb, z1], [xa, ya, z1]]
        pts += [[0, 0, z0], [xb, yb, z0], [xa, ya, z0], [0, 0, z1], [xa, ya, z1], [xb, yb, z1]]
    return pts


# ── flange camera ─────────────────────────────────────────────────────────────
# Camera body in the camera frame (origin = projection centre, z = optical axis):
#   housing 29 x 29 x 29 mm ([camera] body, IDS datasheet), lens Ø29 x 47.6 mm ([camera] lens, IDS shop listing),
#   lens length counted from the C-mount flange (ASSUMPTION). The sensor lies 17.526 mm behind the C-mount flange
#   (C-mount flange focal distance 0.69 in, standard value); the projection centre is taken f = 12 mm in front of
#   the sensor (thin lens focused at infinity, ASSUMPTION) -> the flange plane is 5.53 mm in front of the camera
#   origin. The real projection centre comes from the hand-eye calibration; this body is only for collisions and
#   pictures.
C_MOUNT_FFD = 17.526


def camera_body_points(cfg: dict, n: int = 24) -> list:
    """Housing cube + lens cylinder of the flange camera, triangles in the camera frame [mm]."""
    c = cfg["camera"]
    bx, by, bz = c["body"]
    lens_d, lens_l = 29.0, 47.6           # IDS-12M23-C1228: Ø29 x 47.6 mm ([camera] lens comment, IDS shop)
    z_flange = C_MOUNT_FFD - c["focal"]   # C-mount flange in front of the projection centre
    return (box_points(bx, by, bz, 0.0, 0.0, z_flange - bz / 2)
            + cylinder_points(lens_d / 2, z_flange, z_flange + lens_l, n))


def T_flange_cam(cfg: dict) -> Mat:
    """Nominal camera frame in the flange frame from [camera.mount] (PLACEHOLDER) as a RoboDK Mat
    (mauer.config.T_flange_cam_nominal converted, so both sides use the same rpy convention)."""
    from mauer.config import T_flange_cam_nominal
    from mauer.geometry import to_robodk
    return to_robodk(T_flange_cam_nominal(cfg))


def camera_params(cfg: dict, color: bool = True, flat_light: bool = False, far_mm: float = 3000.0) -> str:
    """Cam2D_Add/Cam2D_SetParams string for the flange camera: ideal pinhole with f/pixel from [camera]
    (research 2026-10-05: fx = fy = f/p, cx = (W-1)/2, cy = (H-1)/2, no distortion; FOV and PIXELSIZE must not both
    be given - the last one wins). COLOR/GRAYSCALE is always passed because the GRAYSCALE flag sticks. flat_light:
    ambient light only, objects render in their exact colours (for occlusion masks)."""
    c = cfg["camera"]
    w, h = int(c["res_x"]), int(c["res_y"])
    s = (f"FOCAL_LENGTH={c['focal']} PIXELSIZE={c['pixel_um']} SIZE={w}x{h} SNAPSHOT={w}x{h} "
         f"FAR_LENGTH={far_mm:.0f} BG_COLOR=white {'COLOR' if color else 'GRAYSCALE'}")
    if flat_light:
        s += " LIGHT_AMBIENT=white LIGHT_DIFFUSE=black LIGHT_SPECULAR=black"
    return s


def set_tool_object_collisions(RDK: Robolink, obj, robot, tool, obstacles: list, on: bool = True) -> None:
    """Collision pairs for an object attached to the gripper tool (camera body, held stone): RoboDK does NOT check
    such an object against static objects unless the pair is switched on explicitly (research 2026-10-05). Checked
    against `obstacles` and robot links 0-5; not against wrist 3 / flange (links 6, 7) and the gripper it is bolted
    to (same rule as motion.set_held)."""
    state = COLLISION_ON if on else COLLISION_OFF
    if obstacles:
        RDK.setCollisionActivePairList([state] * len(obstacles), [obj] * len(obstacles), list(obstacles),
                                       [0] * len(obstacles), [0] * len(obstacles))
    for link in range(0, 8):
        RDK.setCollisionActivePair(state if link <= 5 else COLLISION_OFF, obj, robot, 0, link)
    RDK.setCollisionActivePair(COLLISION_OFF, obj, tool, 0, 0)


# ── snapshots ─────────────────────────────────────────────────────────────────
def look_at(eye: list, target: list) -> Mat:
    """Camera pose (z forward, x right, y down) at eye looking at target."""
    def norm(v):
        n = sum(c * c for c in v) ** 0.5
        return [c / n for c in v]

    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]

    z = norm([t - e for t, e in zip(target, eye)])
    x = norm(cross(z, [0.0, 0.0, 1.0]))
    y = cross(z, x)
    return Mat([[x[0], y[0], z[0], eye[0]], [x[1], y[1], z[1], eye[1]], [x[2], y[2], z[2], eye[2]], [0, 0, 0, 1]])


def snapshot(RDK: Robolink, path: Path, eye: list, target: list, size: str = "1400x900") -> bool:
    """Render the station from a virtual camera into a PNG file."""
    frame = RDK.Item("_view", 3)
    if not frame.Valid():
        frame = RDK.AddFrame("_view")
        frame.setVisible(False)
    frame.setPose(look_at(eye, target))
    RDK.Cam2D_Close(0)                     # a left-over camera window can shrink the snapshot
    cam = RDK.Cam2D_Add(frame, f"FOCAL_LENGTH=6 FOV=45 FAR_LENGTH=20000 SIZE={size} SNAPSHOT={size} BG_COLOR=white")
    RDK.Render(True)
    time.sleep(1.0)                        # let the camera window render once at full size
    ok = RDK.Cam2D_Snapshot(str(path), cam)
    RDK.Cam2D_Close(cam)
    cam.Delete()                           # Cam2D_Close keeps the item; without Delete they pile up in the tree
    return bool(ok)
