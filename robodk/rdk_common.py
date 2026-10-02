"""Shared helpers: config loading, RoboDK connection, wall frame, stone mesh, snapshots.

Run the scripts with Windows Python (RoboDK runs on Windows), e.g. from WSL:
    py.exe robodk/build_station.py
The RoboDK API is taken from the RoboDK installation (C:\\RoboDK\\Python), no pip install needed.

Frames
  ARES   base_link: floor, centre between the steering axes, x forward, y left, z up.
  wall   origin on the wall centreline (floor level), x along the wall = direction ARES travels between stops,
         y towards ARES, z up. A stone at wall position u has its top centre at (u, 0, z_top), long axis along x.
  tool   z along the flange axis (pointing down when placing), x along the stone length, y = jaw closing direction,
         TCP = top centre of the held stone.
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

from robodk.robolink import Robolink  # noqa: E402
from robodk.robomath import Mat, rotx, rotz, transl  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
STATION_NAME = "ARES_UR5_Mauer"
PI = 3.141592653589793
DEG = PI / 180.0


def load_config() -> dict:
    with open(REPO / "config" / "station.toml", "rb") as f:
        return tomllib.load(f)


def connect() -> Robolink:
    """Connect to a running RoboDK or start it. Generous API timeout: large imports can take minutes."""
    RDK = Robolink()
    RDK.TIMEOUT = 600
    RDK.COM.settimeout(RDK.TIMEOUT)
    return RDK


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
    return bool(ok)
