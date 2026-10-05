"""Look-pose / visibility study: where can the reference boards go, and how should the flange camera be mounted, so
that the UR5 sees at least two boards from every stop while the wall grows from course 0 to the top course?

Usage (Windows Python; starts its OWN RoboDK on --port and closes it at the end - the user's RoboDK is not touched):
    py.exe robodk/look_study.py                  # full study -> results/look_study.md, look_study.csv, look_*.png
    py.exe robodk/look_study.py --quick          # coarse look grid, mount M0, placements A + F (smoke run)
    py.exe robodk/look_study.py --types A,F --mounts M0,M1

Method
  plan     wallplan.sequence on the cached reach table (results/reach_table.json, same cache key as simulate.py) for a
           wall of --length stones in course 0 -> stops a_j (ARES centre in wall coordinates) and their stones.
  scenes   per stop: "arrival" (stones of the earlier stops; stop 1 = no stones), "course0" (+ this stop's course-0
           stones), "complete" (+ all stones of this stop). Any state during the stop (e.g. the re-measurement after
           a magazine reload trip) lies between arrival and complete; fewer stones can only help, so a look pose that
           works at "complete" works for the whole stop. "empty" = no stones and no base plate (baseline, also used
           to prune: what fails there fails everywhere). Stones = CAD mesh; magazine full (simulate.py slots,
           3 x 2 x 3 = 18 stones, worst case); base plate as a box under course 0 (width = stone width, ASSUMPTION).
  boards   placement types (PLACEMENTS) x position u along the wall (grid relative to the stop, plus the configured
           [[targets]] x positions); [boards.ref] geometry; one board at a time, posts/brackets not modelled.
  looks    camera poses aimed at the board centre: distance d, angle between optical axis and board normal <= 40 deg
           (azimuth steps), roll about the optical axis (steps); all four outer board corners >= 50 px inside the
           2472 x 2064 image (ideal pinhole from [camera]).
  checks   (1) flange IK for T_ares_cam @ inv(T_flange_cam) with an arm configuration in motion.family();
           (2) RoboDK collision check of the static pose: arm, gripper (no stone held) and camera body against ARES,
           wall stones, magazine, base plate, the board itself (and the pick-up table);
           (3) render with the simulated flange camera (flat light, every obstacle in a saturated colour): no coloured
           pixel inside the projected board AND cv2.aruco CharucoDetector finds all chessboard corners.
           Candidates are tried cheapest first (fronto-parallel at [camera] working_dist) until one passes.
  result   visibility profile per placement type, camera mount, stop and wall state; regular layouts (pitch,
           offset) with >= 2 visible boards for every stop and state; margins (image slack in mm = how far the board
           may move before a corner enters the 50 px border band; number of collision-free alternatives).

Not checked: the transfer motion to the look pose (static pose only), posts/brackets of raised boards, camera
bracket and cable, ARES sway, lighting, stones in the pick-up station slots.
"""
from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import math
import pickle
import re
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import build_station as bs
import mauer.geometry as g
import wallplan
from motion import STEP_MM, family, wrap
from rdk_common import (NEW_INSTANCE_PORT, REPO, T_TC_CAD, as_mat, box_points, camera_body_points,
                        close_instance, connect,
                        course_top_z, load_config, load_stl, set_tool_object_collisions, snapshot, tcp_pose,
                        ur5_base_pose, wall_frame)
from robodk.robolink import COLLISION_OFF, COLLISION_ON
from robodk.robomath import invH
from sim_camera import SimCamera, intrinsics
from simulate import MAG_ROWS, MAG_Y

RESULTS = REPO / "results"
STATES = ("arrival", "course0", "complete")
MARGIN_PX = 50.0                     # task: all board corners >= 50 px inside the image
OCCL_MAX = 0.002                     # max. fraction of coloured (= occluded) pixels inside the projected board
CHROMA_MIN = 40                      # max-min over B, G, R [0..255] above which a pixel belongs to an obstacle
# colours for the occlusion masks (study instance only): every obstacle saturated, boards black/white
COL = {"ares": [0.25, 0.45, 0.85, 1], "robot": [0.95, 0.75, 0.10, 1], "tool": [0.20, 0.70, 0.20, 1],
       "plate": [0.60, 0.25, 0.75, 1], "table": [0.85, 0.55, 0.15, 1]}
LABELS = {"UR5": "arm", "Gripper_EHPS20": "gripper", "Camera_body": "camera", "ARES_STEP_2026-09-23": "ARES",
          "Study_stones": "wall", "Study_magazine": "magazine", "Study_plate": "base plate", "Pickup_table": "table"}


# ── study definitions ─────────────────────────────────────────────────────────
def mounts(cfg: dict) -> dict:
    """Camera mount candidates: key -> (description, xyz [mm], rpy [deg]) of T_flange_cam (flange frame: z out of
    the flange, x = jaw closing direction; the camera sits on the -x side beside the gripper)."""
    m = cfg["camera"]["mount"]
    xyz = [float(v) for v in m["xyz"]]
    return {
        "M0": (f"config PLACEHOLDER {tuple(xyz)}, rpy {tuple(m['rpy_deg'])}: optical axis along tool z", xyz,
               [float(v) for v in m["rpy_deg"]]),
        "M1": ("as M0, tilted 25 deg outward (optical axis towards flange -x, away from the gripper)", xyz,
               [0.0, -25.0, 0.0]),
        "M2": ("as M0, tilted 25 deg about flange x (optical axis towards flange -y)", xyz, [25.0, 0.0, 0.0]),
        "M3": ("lowered to the TCP plane: (-150, 0, tcp_z), axis along tool z",
               [xyz[0], xyz[1], float(cfg["tool"]["tcp_z"])], [0.0, 0.0, 0.0]),
    }


def placements(cfg: dict) -> dict:
    """Wall board placement types: key -> dict(title, y, z = board centre in the wall frame [mm], face = "up" (face
    up, board z down) or "ares" (vertical, readable side towards ARES), note). Positions derive from the config;
    clearances marked ASSUMPTION."""
    b, w = cfg["brick"], cfg["wall"]
    face = b["width"] / 2                                          # wall face (towards ARES) at y = +60
    front = w["dist_nominal"] - cfg["ares"]["length"] / 2         # ARES front edge at y = +180
    top = course_top_z(cfg, w["courses"] - 1)                     # top of the top course
    return {
        "A": dict(title="floor, face up, centred in the gap ARES front - wall face (config PLACEHOLDER)",
                  y=(face + front) / 2, z=3.0, face="up", note="= [[targets]] W0..W7"),
        "B": dict(title="floor, face up, behind the wall (far side), mirrored to A", y=-(face + front) / 2,
                  z=3.0, face="up", note=""),
        "C1": dict(title="vertical post in the gap, facing ARES, centre at course-1 height",
                   y=face + 50.0, z=course_top_z(cfg, 1) - b["height"] / 2, face="ares",
                   note="50 mm off the wall face (ASSUMPTION: outside the open jaws)"),
        "C2": dict(title="vertical post in the gap, facing ARES, centre 60 mm above the top course",
                   y=face + 50.0, z=top + 60.0, face="ares", note="50 mm off the wall face (ASSUMPTION)"),
        "C3": dict(title="vertical post behind the wall, facing ARES, centre 60 mm above the top course",
                   y=-face - 30.0, z=top + 60.0, face="ares", note="30 mm behind the far face (ASSUMPTION)"),
        "D": dict(title="face up on the base plate on the wall centreline (covered later by the wall)",
                  y=0.0, z=w["base_z"] + 3.0, face="up", note="must be removed before a stone covers it"),
        "F": dict(title="face up on a post behind the wall, 50 mm above the top course",
                  y=-face - 100.0, z=top + 50.0, face="up",
                  note="board centre 100 mm behind the far face (ASSUMPTION: clear of the open jaws)"),
        "G": dict(title="face up on a post in the gap, 50 mm above the top course",
                  y=(face + front) / 2, z=top + 50.0, face="up", note="post in the gap; jaw clearance not checked"),
    }


def R_face(face: str) -> np.ndarray:
    """Board orientation in the wall frame. up: x = wall x, y = -wall y, z = down (into the floor).
    ares: x = -wall x, y = down, z = -wall y (readable from ARES, i.e. looking along -wall y)."""
    if face == "up":
        return g.rotx(math.pi)
    T = np.eye(4)
    T[:3, :3] = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]])
    return T


def board_size(spec: dict) -> tuple[float, float]:
    return spec["squares_x"] * spec["square_mm"], spec["squares_y"] * spec["square_mm"]


def T_wall_board(spec: dict, p: dict, u: float) -> np.ndarray:
    """Board frame (origin = top-left outer corner) for a board of type p centred at wall position u."""
    w, h = board_size(spec)
    return g.transl(u, p["y"], p["z"]) @ R_face(p["face"]) @ g.transl(-w / 2, -h / 2, 0.0)


def T_ares_wall(cfg: dict, dist: float, a: float) -> np.ndarray:
    """Wall frame in the ARES frame with the ARES centre at wall position a
    (as simulate.py: Wall = wall_frame * transl(-a))."""
    return g.from_robodk(wall_frame(cfg, dist)) @ g.transl(-a, 0.0, 0.0)


# ── plan (offline) ────────────────────────────────────────────────────────────
def reach_table_cached(cfg: dict, dist: float) -> dict | None:
    """simulate.py's reach table from results/reach_table.json if its cache key matches the config (same key
    construction as simulate.reach_table), else None."""
    brick = {k: v for k, v in cfg["brick"].items() if k != "rib_mm"}     # rib_mm: L corner only, not the reach
    key_src = json.dumps(["family-v2", cfg["ur5"], cfg["tool"], brick, cfg["wall"]["base_z"],
                          cfg["wall"]["courses"], cfg["study"]["approach"], dist], sort_keys=True)
    key = hashlib.sha1(key_src.encode()).hexdigest()[:12]
    path = RESULTS / "reach_table.json"
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    if data.get("key") != key:
        return None
    return {int(k): {float(u): v for u, v in d.items()} for k, d in data["table"].items()}


def make_plan(cfg: dict, length: int, dist: float, table: dict) -> tuple[list, list]:
    """(stones, plan) exactly as simulate.main builds them."""
    def reach(k: int, u: float) -> bool:
        return table[k].get(round(round(u / 20.0) * 20.0, 3), False)

    lo = {k: min(u for u, ok in table[k].items() if ok) for k in table}
    stones = wallplan.layout(cfg, length)
    a0 = stones[0].u - lo[0]
    plan = wallplan.sequence(cfg, stones, reach, lo, a0)
    errors = wallplan.check_plan(cfg, stones, plan)
    if errors:
        raise SystemExit("invalid plan: " + "; ".join(errors))
    return stones, plan


@dataclass(frozen=True)
class Scene:
    name: str                 # "empty", "S2 complete", ...
    stop: int                 # 1-based stop index, 0 = empty
    state: str
    a: float                  # ARES centre in wall coordinates
    stones: tuple             # wallplan.Stone placed
    plate: bool


def scenes(plan: list) -> list:
    out = [Scene("empty", 0, "empty", plan[0][0], (), False)]
    done: list = []
    for j, (a, batch) in enumerate(plan, start=1):
        c0 = [s for s in batch if s.course == 0]
        for state, extra in (("arrival", []), ("course0", c0), ("complete", list(batch))):
            out.append(Scene(f"S{j} {state}", j, state, a, tuple(done + extra), True))
        done += list(batch)
    return out


# ── geometry helpers (numpy) ──────────────────────────────────────────────────
def stone_mesh_wall(cfg: dict, template: np.ndarray, stones) -> np.ndarray:
    """Stone CAD mesh vertices placed in the wall frame (same mapping as rdk_common.stone_points(frame="wall"))."""
    b = cfg["brick"]
    L, W, H = b["length"], b["width"], b["height"]
    out = []
    for s in stones:
        out.append(np.column_stack([s.u + template[:, 1] + L / 2, W / 2 - template[:, 0],
                                    s.z_top - H + template[:, 2]]))
    return np.vstack(out) if out else np.zeros((0, 3))


def magazine_slots(cfg: dict) -> list:
    """(x, y, z_top) of all simulate.py magazine slots (ARES frame), stone long axis along ARES y."""
    deck = cfg["ares"]["deck_top_z"] + cfg["deck"]["holder_z"]
    H = cfg["brick"]["height"]
    return [(cfg["ur5"]["mount_x"] + dx, y, deck + lay * H) for dx, n in MAG_ROWS for lay in range(1, n + 1)
            for y in MAG_Y]


def aabb_of(T: np.ndarray, lo, hi) -> np.ndarray:
    """Axis-aligned box (2, 3) around the box lo..hi transformed by T."""
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], float)
    p = g.apply(T, c)
    return np.array([p.min(0), p.max(0)])


def segments_hit(p0: np.ndarray, p1: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    """(S,) bool: segment p0[i] -> p1[i] intersects any of the boxes (B, 2, 3) (slab test)."""
    if len(boxes) == 0:
        return np.zeros(len(p0), bool)
    d = p1 - p0
    with np.errstate(divide="ignore", invalid="ignore"):
        inv_d = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
        t1 = (boxes[None, :, 0, :] - p0[:, None, :]) * inv_d[:, None, :]
        t2 = (boxes[None, :, 1, :] - p0[:, None, :]) * inv_d[:, None, :]
    tmin = np.minimum(t1, t2).max(axis=2)
    tmax = np.maximum(t1, t2).min(axis=2)
    return ((tmax >= np.maximum(tmin, 0.0)) & (tmin <= 1.0)).any(axis=1)


@dataclass
class Cand:
    cost: float
    d: float
    tilt: float
    az: float
    roll: float
    T_ac: np.ndarray          # camera in the ARES frame
    j: list                   # joints [deg]
    margin_px: float
    slack_mm: float
    clear_mm: float = -1.0    # largest camera shift (+-x, +-y in the ARES frame) that stays collision-free
    approach: str = ""        # free 150 mm retreat: "axis" (along the optical axis) or "vertical"


def view_grid(spec: dict, K: np.ndarray, W: int, H: int, dists, tilts, n_az: int, n_roll: int,
              work_mm: float) -> list:
    """Camera poses T_board_cam aimed at the board centre, with image margin and slack; only those with all outer
    corners >= MARGIN_PX inside the image. Returns [(cost, d, tilt, az, roll, T_board_cam, margin_px, slack_mm)]."""
    bw, bh = board_size(spec)
    c = np.array([bw / 2, bh / 2, 0.0])
    corners = np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]], float)
    f = K[0, 0]
    out = []
    for d in dists:
        for t in tilts:
            for az in ([0.0] if t == 0 else [360.0 * i / n_az for i in range(n_az)]):
                tr, ar = math.radians(t), math.radians(az)
                v = np.array([math.sin(tr) * math.cos(ar), math.sin(tr) * math.sin(ar), math.cos(tr)])
                x0 = np.array([1.0, 0, 0]) - v * v[0]
                x0 /= np.linalg.norm(x0)
                y0 = np.cross(v, x0)
                for k in range(n_roll):
                    r = 2 * math.pi * k / n_roll
                    xr = math.cos(r) * x0 + math.sin(r) * y0
                    T_bc = g.make_T(np.column_stack([xr, np.cross(v, xr), v]), c - d * v)
                    P = g.apply(g.inv(T_bc), corners)
                    uv = (P @ K.T)[:, :2] / P[:, 2:3]
                    m = np.minimum.reduce([uv[:, 0], W - 1 - uv[:, 0], uv[:, 1], H - 1 - uv[:, 1]])
                    if m.min() < MARGIN_PX:
                        continue
                    slack = float(((m - MARGIN_PX) * P[:, 2] / f).min())
                    cost = abs(d - work_mm) / 80.0 + t / 15.0
                    out.append((cost, d, t, az, math.degrees(r), T_bc, float(m.min()), slack))
    out.sort(key=lambda e: e[0])
    return out


# ── the study ─────────────────────────────────────────────────────────────────
class Study:
    def __init__(self, RDK, cfg: dict, args):
        self.RDK, self.cfg, self.args = RDK, cfg, args
        self.dist = args.dist
        t0 = time.time()
        it = bs.build(RDK, cfg, camera=True, boards=False, pickup=True)
        print(f"station built in {time.time() - t0:.1f} s", flush=True)
        self.it = it
        self.robot, self.tool, self.ares = it["robot"], it["tool"], it["ares"]
        self.f_ares, self.f_wall = it["f_ares"], it["f_wall"]
        self.cam_tool, self.body, self.table = it["cam_tool"], it["cam_body"], it["table"]
        it["wall"].setVisible(False)                       # nominal wall: replaced by the scene stones
        self.ares.setColor(COL["ares"])
        self.robot.setColor(COL["robot"])
        self.tool.setColor(COL["tool"])
        self.table.setColor(COL["table"])
        self.ref = invH(ur5_base_pose(cfg))                # ARES frame seen from the UR5 base
        self.T_ft = g.from_robodk(tcp_pose(cfg))           # gripper TCP in the flange frame
        self.robot.setPoseFrame(self.f_ares)               # MoveL_Test targets: ARES frame, gripper TCP
        self.robot.setPoseTool(self.tool)
        self.approach_mm = float(cfg["study"]["approach"])
        self.K, self.D, self.W, self.H = intrinsics(cfg)
        self.template = np.array([v for tri in load_stl(REPO / cfg["brick"]["mesh"]) for v in tri], float)
        self.stone_key = None
        self.stones = None
        b = cfg["brick"]
        # magazine (full) and base plate as static study objects
        mag = []
        self.mag_boxes = []
        T_tc = g.from_robodk(T_TC_CAD)
        for x, y, z in magazine_slots(cfg):
            T = g.transl(x, y, z) @ g.rotz(math.pi / 2) @ T_tc
            mag.append(g.apply(T, self.template))
            self.mag_boxes.append([[x - b["width"] / 2, y - b["length"] / 2, z - b["height"]],
                                   [x + b["width"] / 2, y + b["length"] / 2, z]])
        self.mag = self._shape(np.vstack(mag), "Study_magazine", self.f_ares, bs.STONE)
        self.mag_boxes = np.array(self.mag_boxes, float)
        n0 = args.length
        self.plate_len = n0 * (b["length"] + b["head_joint"])
        bz = cfg["wall"]["base_z"]
        self.plate = self.RDK.AddShape(as_mat(box_points(self.plate_len, b["width"], bz, self.plate_len / 2, 0,
                                                         bz / 2)))
        self.plate.setParent(self.f_wall)
        self.plate.setName("Study_plate")
        self.plate.setColor(COL["plate"])
        # study boards (one per geometry), moved around; ids of W0 / the calib board
        layout = bs.board_layout(cfg)
        self.specs = {"ref": next(s for s in layout if s["parent"] == "wall"),
                      "calib": next(s for s in layout if s["name"] == "calib")}
        self.board_items, self.detectors, self.n_corners = {}, {}, {}
        for key, spec in self.specs.items():
            item = bs.add_board(RDK, spec, self.f_ares, g.to_robodk(g.transl(0, 0, -5000)))
            item.setName(f"Board_study_{key}")
            item.setVisible(False)
            self.board_items[key] = item
            board = bs.charuco_board(spec)
            self.detectors[key] = (board, cv2.aruco.CharucoDetector(board))
            self.n_corners[key] = len(board.getChessboardCorners())
        self.statics = [self.ares, self.table, self.mag, self.plate, *self.board_items.values()]
        self._statics_off(self.statics)
        set_tool_object_collisions(RDK, self.body, self.robot, self.tool, self.statics)
        self.grids: dict = {}
        self.ik_cache: dict = {}
        self.mount = None
        self.T_fc = None
        self.home = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]          # arm straight up: free in every scene
        RDK.Render(True)                                   # the camera must be opened with rendering on
        self.cam = SimCamera(RDK, cfg, self.cam_tool, robot=self.robot, hide=[self.body]).open()
        self.cam.set_flat_light(True)
        RDK.Render(False)
        self.boxes = np.zeros((0, 2, 3))
        self.n_ik_calls = self.n_coll = self.n_render = 0
        # JointPoses convention: index 0 = robot base, either absolute (station = ARES frame) or relative
        T_ab = g.from_robodk(ur5_base_pose(cfg))
        P0 = g.from_robodk(self.robot.JointPoses(self.home)[0])
        self.T_links = np.eye(4) if np.allclose(P0, T_ab, atol=1e-3) else T_ab
        body = np.array(camera_body_points(cfg), float)
        lo_, hi_ = body.min(0), body.max(0)
        self.body_corners = np.array([[x, y, z] for x in (lo_[0], hi_[0]) for y in (lo_[1], hi_[1])
                                      for z in (lo_[2], hi_[2])])

    # ── RoboDK objects ───────────────────────────────────────────────────────
    def _shape(self, pts: np.ndarray, name: str, parent, color):
        tris = pts.reshape(-1, 3).tolist()
        item = self.RDK.AddShape(as_mat(tris))
        item.setParent(parent)
        item.setName(name)
        item.setColor(color)
        return item

    def _statics_off(self, items: list) -> None:
        """No collision checks between static objects (contacts by design, e.g. stones on the base plate)."""
        for i, a in enumerate(items):
            rest = items[i + 1:]
            if rest:
                self.RDK.setCollisionActivePairList([COLLISION_OFF] * len(rest), [a] * len(rest), rest,
                                                    [0] * len(rest), [0] * len(rest))

    def set_scene(self, sc: Scene | None, magazine: bool = True, T_aw: np.ndarray | None = None,
                  table: bool = True) -> None:
        """Wall frame at the stop (or T_aw), the stones of the scene, plate and magazine visibility, AABBs."""
        cfg = self.cfg
        T_aw = T_ares_wall(cfg, self.dist, sc.a) if T_aw is None else T_aw
        self.T_aw = T_aw
        self.f_wall.setPose(g.to_robodk(T_aw))
        stones = sc.stones if sc else ()
        key = tuple(s.key for s in stones)
        if key != self.stone_key:
            if self.stones is not None:
                self.stones.Delete()
                self.stones = None
            if stones:
                self.stones = self._shape(stone_mesh_wall(cfg, self.template, stones), "Study_stones", self.f_wall,
                                          bs.STONE)
                others = [o for o in self.statics]
                self.RDK.setCollisionActivePairList([COLLISION_OFF] * len(others), [self.stones] * len(others),
                                                    others, [0] * len(others), [0] * len(others))
                self.RDK.setCollisionActivePair(COLLISION_ON, self.body, self.stones, 0, 0)
            self.stone_key = key
        plate = bool(sc and sc.plate)
        self.plate.setVisible(plate)
        self.mag.setVisible(magazine)
        self.table.setVisible(table)
        # AABBs for the occlusion pre-filter (ARES frame)
        b = cfg["brick"]
        boxes = [[[-cfg["ares"]["length"] / 2, -cfg["ares"]["width"] / 2, 0.0],
                  [cfg["ares"]["length"] / 2, cfg["ares"]["width"] / 2, cfg["ares"]["deck_top_z"]]]]
        if magazine:
            boxes += self.mag_boxes.tolist()
        for s in stones:
            boxes.append(aabb_of(T_aw, [s.u - b["length"] / 2, -b["width"] / 2, s.z_top - b["height"]],
                                 [s.u + b["length"] / 2, b["width"] / 2, s.z_top]).tolist())
        if plate:
            boxes.append(aabb_of(T_aw, [0, -b["width"] / 2, 0], [self.plate_len, b["width"] / 2,
                                                                   cfg["wall"]["base_z"]]).tolist())
        if table:
            x_max, y_max = bs.table_extent(cfg)
            z = cfg["pickup_station"]["table_z"]
            T_as = T_aw @ g.from_robodk(bs.T_wall_station(cfg))
            boxes.append(aabb_of(T_as, [0, 0, z - bs.TABLE_TOP_MM], [x_max, y_max, z]).tolist())
        self.boxes = np.array(boxes, float)
        self.robot.setJoints(self.home)
        self.RDK.Update()
        n = self.RDK.Collisions()
        if n:
            raise RuntimeError(f"scene {sc.name if sc else '-'}: collision at the home pose: {self._pairs()}")

    def set_mount(self, key: str) -> None:
        desc, xyz, rpy = mounts(self.cfg)[key]
        self.mount = key
        self.T_fc = g.pose_xyz_rpy(xyz, rpy)
        bs.set_camera_mount(self.cfg, self.cam_tool, self.body, g.to_robodk(self.T_fc))
        self.T_fc_rdk = g.to_robodk(self.T_fc)

    def mount_interference(self) -> list:
        """Collision pairs of the camera body with the gripper mesh at the current mount (robot at home)."""
        self.RDK.setCollisionActivePair(COLLISION_ON, self.body, self.tool, 0, 0)
        self.robot.setJoints(self.home)
        self.RDK.Update()
        n = self.RDK.Collisions()
        pairs = self._pairs() if n else []
        self.RDK.setCollisionActivePair(COLLISION_OFF, self.body, self.tool, 0, 0)
        return pairs

    def place_board(self, key: str, T_ares_board: np.ndarray | None) -> None:
        for k, item in self.board_items.items():
            if k == key and T_ares_board is not None:
                sq = self.specs[k]["square_mm"]
                item.setPose(g.to_robodk(T_ares_board @ g.transl(-sq, -sq, 0.0)))
                item.setVisible(True)
            else:
                item.setVisible(False)

    def _pairs(self) -> list:
        out = []
        for a, b, ia, ib in self.RDK.CollisionPairs():
            la = LABELS.get(a.Name(), "board" if a.Name().startswith("Board") else a.Name())
            lb = LABELS.get(b.Name(), "board" if b.Name().startswith("Board") else b.Name())
            out.append("/".join(sorted((la, lb))))
        return out

    # ── candidates ───────────────────────────────────────────────────────────
    def grid(self, key: str) -> list:
        if key not in self.grids:
            a = self.args
            work = self.cfg["camera"]["working_dist"]
            self.grids[key] = view_grid(self.specs[key], self.K, self.W, self.H, a.dists, a.tilts, a.n_az,
                                        a.n_roll, work)
        return self.grids[key]

    def candidates(self, key: str, T_ares_board: np.ndarray) -> list:
        ck = (self.mount, key, tuple(np.round(T_ares_board, 3).ravel()))
        if ck in self.ik_cache:
            return self.ik_cache[ck]
        out = []
        for cost, d, t, az, roll, T_bc, margin, slack in self.grid(key):
            T_ac = T_ares_board @ T_bc
            sols = self.robot.SolveIK_All(g.to_robodk(T_ac), tool=self.T_fc_rdk, reference=self.ref)
            self.n_ik_calls += 1
            if sols.size(0) < 6:
                continue
            seen = set()
            for c in range(sols.size(1)):
                j = [wrap(sols[r, c]) for r in range(6)]
                if not family(j):
                    continue
                k = tuple(round(v, 1) for v in j)
                if k in seen:
                    continue
                seen.add(k)
                out.append(Cand(cost, d, t, az, roll, T_ac, j, margin, slack))
        out.sort(key=lambda c: c.cost)
        self.ik_cache[ck] = out
        return out

    # ── checks ───────────────────────────────────────────────────────────────
    def collide(self, j: list) -> list:
        self.n_coll += 1
        self.robot.setJoints(j)
        self.RDK.Update()
        return self._pairs() if self.RDK.Collisions() else []

    def inside(self, j: list, T_ac: np.ndarray, T_from: np.ndarray | None = None) -> str:
        """Solid-volume check that RoboDK's surface check misses: the ARES STEP is a hollow assembly and a body
        completely inside a mesh is not reported. Joint origins, link centre lines, flange -> TCP, flange -> camera
        (the bracket, not modelled) and the camera body corners must stay outside the solid boxes of the scene
        (ARES envelope up to the deck top, stones, magazine, base plate, table; shrunk by 2 mm). T_from: also the
        straight camera path T_from -> T_ac (approach). Returns "" or a label."""
        boxes = self.boxes.copy()
        boxes[:, 0, :] += 2.0
        boxes[:, 1, :] -= 2.0
        boxes = boxes[np.all(boxes[:, 1, :] > boxes[:, 0, :], axis=1)]
        links = [self.T_links @ g.from_robodk(P) for P in self.robot.JointPoses(j)]
        pts = [T[:3, 3] for T in links]
        T_af = T_ac @ g.inv(self.T_fc)
        flange, tcp, cam = T_af[:3, 3], (T_af @ self.T_ft)[:3, 3], T_ac[:3, 3]
        p0 = pts + [flange, flange]
        p1 = pts[1:] + [flange, tcp, cam]
        if T_from is not None:
            p0.append(T_from[:3, 3])
            p1.append(cam)
        hit = segments_hit(np.array(p0), np.array(p1), boxes)
        corners = g.apply(T_ac, self.body_corners)
        inside_pts = ((corners[:, None, :] > boxes[None, :, 0, :]) & (corners[:, None, :] < boxes[None, :, 1, :])
                      ).all(axis=2).any(axis=1)
        if hit.any() or inside_pts.any():
            return "solid volume (ARES/stones)"
        return ""

    def ik_near(self, T_ac: np.ndarray, seed: list, tool: np.ndarray | None = None) -> list | None:
        """IK solution next to seed (same family, no joint jump > 60 deg) for a camera (or tool) pose."""
        T_tool = self.T_fc_rdk if tool is None else g.to_robodk(tool)
        j = self.robot.SolveIK(g.to_robodk(T_ac), seed, tool=T_tool, reference=self.ref).list()
        self.n_ik_calls += 1
        if len(j) < 6:
            return None
        j = j[:6]
        if not family(j) or max(abs(a - b) for a, b in zip(j, seed)) > 60.0:
            return None
        return j

    def approach_ok(self, c: Cand) -> str:
        """The arm must get to the look pose: a 150 mm ([study] approach) linear retreat along the optical axis
        or vertically up must be collision-free (MoveL_Test, 2 mm steps). Returns "axis", "vertical" or ""."""
        h = self.approach_mm
        target = g.to_robodk(c.T_ac @ g.inv(self.T_fc) @ self.T_ft)       # gripper TCP pose (ARES frame)
        for name, T_up in (("axis", c.T_ac @ g.transl(0, 0, -h)), ("vertical", g.transl(0, 0, h) @ c.T_ac)):
            j_up = self.ik_near(T_up, c.j)
            if j_up is None or self.inside(j_up, T_up, T_from=None) or self.inside(c.j, c.T_ac, T_from=T_up):
                continue
            self.n_coll += 1
            if self.robot.MoveL_Test(j_up, target, STEP_MM) == 0:
                return name
        return ""

    def clearance(self, c: Cand, steps=(40.0, 30.0, 20.0, 10.0, 5.0)) -> float:
        """Largest shift d of the camera pose along +-x and +-y of the ARES frame (towards / away from the wall,
        along the wall) for which the arm stays collision-free; 0 if not even 5 mm. Also a measure of how much
        ARES may stop off its nominal position before the planned look pose collides."""
        for d in steps:
            ok = True
            for dx, dy in ((d, 0), (-d, 0), (0, d), (0, -d)):
                T_s = g.transl(dx, dy, 0) @ c.T_ac
                j = self.ik_near(T_s, c.j)
                if j is None or self.collide(j) or self.inside(j, T_s):
                    ok = False
                    break
            if ok:
                return d
        return 0.0

    def self_test(self, sc: "Scene") -> dict:
        """Known collisions must be reported: gripper TCP 60 mm inside a placed stone; camera origin 10 mm below a
        stone top, looking down (body straddles the top face). Raises if RoboDK does not see them. Note: RoboDK's
        mesh check does not report a body completely inside another mesh (observed 2026-10-05 with the camera origin
        40 mm inside a stone) - such look poses are still rejected by the render and the approach test."""
        out = {}
        stones = sorted(sc.stones, key=lambda s: (abs(s.u - sc.a), -s.course))
        for name, z_off, tool, expect in (("gripper TCP 60 mm into a stone", -60.0, self.T_ft, "gripper/wall"),
                                          ("camera body through a stone top face", -10.0, self.T_fc,
                                           "camera/wall")):
            res = None
            for s in stones[:6]:
                T = self.T_aw @ g.transl(s.u, 0.0, s.z_top + z_off) @ g.rotx(math.pi)
                for seed in ([0, -60, 120, -150, -90, 0], [180, -60, 120, -150, -90, 0], self.home):
                    j = self.robot.SolveIK(g.to_robodk(T), seed, tool=g.to_robodk(tool), reference=self.ref).list()
                    if len(j) >= 6:
                        res = self.collide(j[:6])
                        break
                if res is not None:
                    break
            out[name] = res
            if res is None:
                print(f"self-test '{name}': no IK, skipped", flush=True)
            elif expect not in res:
                raise RuntimeError(f"collision self-test failed: {name}: RoboDK reports {res}, expected {expect}")
        T_in = g.transl(300.0, 0.0, 200.0) @ g.roty(math.pi / 2)          # camera inside the ARES chassis
        out["camera inside the ARES chassis (solid check)"] = [self.inside(self.home, T_in)]
        if not out["camera inside the ARES chassis (solid check)"][0]:
            raise RuntimeError("solid-volume self-test failed: camera inside ARES not flagged")
        self.robot.setJoints(self.home)
        return out

    def blocked(self, c: Cand, key: str, T_ares_board: np.ndarray) -> bool:
        """Occlusion pre-filter (AABBs): rays camera -> 4 board corners + centre (only orders the renders)."""
        bw, bh = board_size(self.specs[key])
        pts = g.apply(T_ares_board, np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0], [bw / 2, bh / 2, 0]],
                                             float))
        p0 = np.repeat(c.T_ac[None, :3, 3], len(pts), 0)
        d = pts - p0
        n = np.linalg.norm(d, axis=1, keepdims=True)
        return bool(segments_hit(p0 + d / n * 60.0, pts - d / n * 2.0, self.boxes).any())

    def view(self, c: Cand, key: str, T_ares_board: np.ndarray, keep: bool = False) -> dict:
        """Render at the look pose and check occlusion + ChArUco detection (+ PnP error against the truth)."""
        self.n_render += 1
        self.robot.setJoints(c.j)
        self.RDK.Update()
        img = self.cam.grab_bgr()
        T_cb = g.inv(c.T_ac) @ T_ares_board
        bw, bh = board_size(self.specs[key])
        P = g.apply(T_cb, np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]], float))
        uv = (P @ self.K.T)[:, :2] / P[:, 2:3]
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, np.round(uv).astype(np.int32), 1)
        mask = cv2.erode(mask, np.ones((7, 7), np.uint8))
        chroma = img.max(axis=2).astype(np.int16) - img.min(axis=2).astype(np.int16)
        occl = float((chroma[mask > 0] > CHROMA_MIN).mean()) if mask.any() else 1.0
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        board, det = self.detectors[key]
        cc, ci, _, _ = det.detectBoard(gray)
        n = 0 if ci is None else len(ci)
        err_mm = err_deg = float("nan")
        if n >= 6:
            obj, imgp = board.matchImagePoints(cc, ci)
            ok, rv, tv = cv2.solvePnP(obj, imgp, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
            ok, rv, tv = cv2.solvePnP(obj, imgp, self.K, self.D, rv, tv, True, cv2.SOLVEPNP_ITERATIVE)
            err_mm, err_deg = g.pose_delta(g.make_T(cv2.Rodrigues(rv)[0], tv.ravel()), T_cb)
        why = "occluded" if occl > OCCL_MAX else ("not detected" if n < self.n_corners[key] else "")
        out = {"ok": not why, "why": why, "occl": occl, "corners": n, "pnp_mm": err_mm, "pnp_deg": err_deg}
        if keep:
            out["img"] = img
        return out

    def evaluate(self, key: str, T_ares_board: np.ndarray, count_free: bool = False) -> dict:
        """First valid look pose (cheapest first) for a board at T_ares_board in the current scene."""
        a = self.args
        cands = self.candidates(key, T_ares_board)
        res = {"ok": False, "n_ik": len(cands), "n_checked": 0, "n_free": 0, "n_render": 0, "reason": "no IK",
               "best": None, "view": None}
        if not cands:
            return res
        self.place_board(key, T_ares_board)
        coll, why, deferred = Counter(), Counter(), []
        limit = len(cands) if count_free else min(len(cands), a.max_checks)
        for c in cands[:limit]:
            res["n_checked"] += 1
            pairs = self.collide(c.j)
            if not pairs:
                ins = self.inside(c.j, c.T_ac)
                pairs = [ins] if ins else []
            if pairs:
                coll.update(set(pairs))
                continue
            res["n_free"] += 1
            if res["ok"]:
                continue
            c.approach = self.approach_ok(c)
            if not c.approach:
                why["no free approach"] += 1
                continue
            if self.blocked(c, key, T_ares_board):
                deferred.append(c)
                continue
            if res["n_render"] < a.max_renders:
                v = self.view(c, key, T_ares_board)
                res["n_render"] += 1
                if v["ok"]:
                    res.update(ok=True, best=c, view=v)
                    if not count_free:
                        break
                else:
                    why[v["why"]] += 1
        for c in deferred:                                   # pre-filter said blocked: try if budget is left
            if res["ok"] or res["n_render"] >= a.max_renders:
                break
            v = self.view(c, key, T_ares_board)
            res["n_render"] += 1
            if v["ok"]:
                res.update(ok=True, best=c, view=v)
            else:
                why[v["why"]] += 1
        if res["ok"]:
            res["reason"] = "ok"
            res["best"].clear_mm = self.clearance(res["best"])
        elif res["n_free"] == 0:
            top = ", ".join(f"{k} {n}" for k, n in coll.most_common(2))
            res["reason"] = f"collision ({top})" + ("" if res["n_checked"] == len(cands) else " [budget]")
        elif deferred and not why:
            res["reason"] = "occluded (pre-filter)"
        else:
            res["reason"] = ", ".join(k for k, _ in why.most_common()) or "occluded"
        res["coll"] = coll
        return res


# ── layouts ───────────────────────────────────────────────────────────────────
def good(r: dict | None, u: float, robust: bool, reach_mm: float) -> bool:
    """Board counts for a layout: visible; robust = also >= 10 mm clearance and inside the stone reach."""
    if not r or not r.get("ok"):
        return False
    return not robust or (r["best"].clear_mm >= 10.0 and abs(u) <= reach_mm)


def best_layouts(vis: dict, stops: list, grid: list, step: float, pitches, reach_mm: float) -> list:
    """Regular layouts (a board every `pitch` mm, offset relative to stop 1, positions on the evaluated grid). For each
    pitch the offset with the most robust boards in the worst stop/state, then the most visible ones.
    Returns [dict(pitch, offset, min_r, min_v, worst, counts_v, counts_r, baseline)] (baseline = smallest distance
    between the outermost robust boards of a stop/state [mm])."""
    gset = set(grid)
    a1 = stops[0]
    out = []
    for p in pitches:
        best = None
        for o in np.arange(0.0, p, step):
            cv, cr, base = {}, {}, {}
            for j, a in enumerate(stops, start=1):
                us = [u for u in (round(o + k * p - (a - a1), 1) for k in range(-10, 30)) if u in gset]
                for st in STATES:
                    rv = [u for u in us if good(vis.get((j, st, u)), u, False, reach_mm)]
                    rr = [u for u in us if good(vis.get((j, st, u)), u, True, reach_mm)]
                    cv[(j, st)], cr[(j, st)] = len(rv), len(rr)
                    base[(j, st)] = (max(rr) - min(rr)) if len(rr) >= 2 else 0.0
            key = (min(cr.values()), min(cv.values()), sum(cv.values()), -o)
            if best is None or key > best[0]:
                worst = min(cr, key=lambda k: (cr[k], cv[k]))
                best = (key, {"pitch": p, "offset": float(o), "min_r": key[0], "min_v": key[1],
                              "worst": f"S{worst[0]} {worst[1]}", "counts_v": cv, "counts_r": cr,
                              "baseline": min(base.values())})
        out.append(best[1])
    return out


def recommend(lay: dict, mount_keys: list):
    """(type, mount, layout) with >= 2 robust boards in every stop and state; preferred: a spare (>= 3 visible),
    then the largest pitch, then more robust boards, then the mount listed first (M0 = PLACEHOLDER).
    Fallback without a robust layout: >= 2 visible boards, largest pitch."""
    best = None
    for (tk, mk), ls in lay.items():
        pref = -mount_keys.index(mk) if mk in mount_keys else 0
        for l in ls:
            if l["min_r"] >= 2:
                score = (1, l["min_v"] >= 3, l["pitch"], l["min_r"], l["min_v"], pref)
            elif l["min_v"] >= 2:
                score = (0, False, l["pitch"], l["min_v"], 0, pref)
            else:
                continue
            if best is None or score > best[0]:
                best = (score, (tk, mk, l))
    return best[1] if best else None


def status_of(text: str, section: str, key: str) -> str:
    """Status tag of a config key from the inline comment in station.toml (CONFIRMED/ASSUMPTION/PLACEHOLDER/UNKNOWN)."""
    sec = None
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("["):
            sec = s.strip("[]")
            continue
        if sec == section and re.match(rf"{re.escape(key)}\s*=", s):
            m = re.search(r"#\s*.*?(CONFIRMED|ASSUMPTION|PLACEHOLDER|UNKNOWN)", s)
            return m.group(1) if m else "see section comment"
    return "?"


# ── main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], allow_abbrev=False)
    ap.add_argument("--port", type=int, default=NEW_INSTANCE_PORT, help="API port of the own RoboDK instance")
    ap.add_argument("--length", type=int, default=24, help="stones in course 0 (README result: 24)")
    ap.add_argument("--dist", type=float, default=cfg["wall"]["dist_nominal"])
    ap.add_argument("--types", default="A,B,C1,C2,C3,D,F,G", help="placement types (see placements())")
    ap.add_argument("--mounts", default="M0,M1,M2,M3", help="camera mounts (see mounts())")
    ap.add_argument("--range", type=float, default=1200.0, help="board positions |u_rel| <= range [mm]")
    ap.add_argument("--step", type=float, default=100.0, help="grid step of the board positions [mm]")
    ap.add_argument("--max-checks", type=int, default=400, help="collision checks per board and scene")
    ap.add_argument("--max-renders", type=int, default=3, help="renders per board and scene")
    ap.add_argument("--quick", action="store_true", help="coarse look grid, M0, placements A + F")
    ap.add_argument("--no-snapshots", action="store_true")
    ap.add_argument("--out", default="look_study", help="basename of the outputs in results/")
    ap.add_argument("--save-rdk", type=Path, default=None, help="save the study station there (never the repo .rdk)")
    ap.add_argument("--dump", type=Path, default=None, help="pickle all results there (for --report-from)")
    ap.add_argument("--report-from", type=Path, default=None,
                    help="only rewrite results/<out>.md/.csv from a --dump pickle (no RoboDK)")
    args = ap.parse_args()
    if args.report_from:
        with open(args.report_from, "rb") as fh:
            d = pickle.load(fh)
        write_csv(RESULTS / f"{args.out}.csv", d["rows"])
        write_report(RESULTS / f"{args.out}.md", d["cfg"], d["args"], d["plan"], d["all_scenes"], d["grid_all"],
                     d["conf_u"], d["profiles"], d["extra"], d["P"], d["M"], d["types"], d["mount_keys"], d["runtime"])
        print(f"rewrote results/{args.out}.md and .csv from {args.report_from}", flush=True)
        return
    if args.quick:
        args.dists, args.tilts, args.n_az, args.n_roll = (280.0, 360.0), (0.0, 20.0, 40.0), 4, 4
        if args.types == ap.get_default("types"):
            args.types = "A,F"
        if args.mounts == ap.get_default("mounts"):
            args.mounts = "M0"
        args.step = max(args.step, 200.0)
    else:
        args.dists, args.tilts, args.n_az, args.n_roll = (250.0, 300.0, 350.0, 400.0), (0.0, 15.0, 30.0, 40.0), 8, 8
    if args.save_rdk and args.save_rdk.resolve() == (REPO / "robodk" / "ARES_UR5_Mauer.rdk").resolve():
        raise SystemExit("--save-rdk must not overwrite the repo station")
    types = [t for t in args.types.split(",") if t]
    mount_keys = [m for m in args.mounts.split(",") if m]
    P = placements(cfg)
    M = mounts(cfg)

    t_start = time.time()
    table = reach_table_cached(cfg, args.dist)
    if table is None:
        raise SystemExit("results/reach_table.json does not match the config - run simulate.py --plan-only first")
    stones, plan = make_plan(cfg, args.length, args.dist, table)
    stops = [a for a, _ in plan]
    print(f"plan: {len(stones)} stones, stops at a = {stops} ({[len(b) for _, b in plan]} stones)", flush=True)
    all_scenes = scenes(plan)
    step = args.step
    grid = [round(float(u), 1) for u in np.arange(-args.range, args.range + 1e-6, step)]
    # configured target x positions relative to the stops (same offset for every stop if the pitches match)
    # straight-wall study: boards along the wall frame x = leg A of an L (leg B boards are not in this study)
    first_leg = (cfg["wall"].get("legs") or [{"name": None}])[0]["name"]
    targets = [t for t in cfg.get("targets", []) if t["parent"] == "wall" and t.get("leg") in (None, first_leg)]
    ref = next(s for s in bs.board_layout(cfg) if s["parent"] == "wall")
    bw, bh = board_size(ref)
    conf_u = sorted({round(t["xyz"][0] + bw / 2 - a, 1) for t in targets for a in stops
                     if abs(t["xyz"][0] + bw / 2 - a) <= args.range})
    grid_all = sorted(set(grid) | set(conf_u))
    print(f"board positions per stop: {len(grid_all)} (grid {step:.0f} mm + configured {conf_u})", flush=True)

    RDK = connect(new_instance=True, port=args.port)
    rows, profiles, extra = [], {}, {}
    try:
        print(f"RoboDK {RDK.Version()} (own instance, pid {RDK.NEW_INSTANCE.pid}, port {args.port})", flush=True)
        S = Study(RDK, cfg, args)
        mount_check = {}
        for mk in mount_keys:
            S.set_mount(mk)
            mount_check[mk] = S.mount_interference()
            print(f"mount {mk}: camera body vs gripper mesh: {mount_check[mk] or 'free'}", flush=True)
        extra["mount_check"] = mount_check
        S.set_mount(mount_keys[0])
        sc_test = next(s for s in all_scenes if s.stop == 1 and s.state == "complete")
        S.set_scene(sc_test)
        extra["self_test"] = S.self_test(sc_test)
        print(f"collision self-test ({sc_test.name}): {extra['self_test']}", flush=True)

        # ── wall boards: visibility profiles ─────────────────────────────────
        for tk in types:
            p = P[tk]
            for mk in mount_keys:
                t0 = time.time()
                S.set_mount(mk)
                vis = {}
                base = {}
                S.set_scene(all_scenes[0])
                for u in grid_all:
                    T_ab = S.T_aw @ T_wall_board(ref, p, all_scenes[0].a + u)
                    base[u] = S.evaluate("ref", T_ab)
                    vis[(0, "empty", u)] = base[u]
                for j, a in enumerate(stops, start=1):
                    sc_by = {sc.state: sc for sc in all_scenes if sc.stop == j}
                    for st in ("complete", "course0", "arrival"):
                        sc = sc_by[st]
                        S.set_scene(sc)
                        for u in grid_all:
                            if (j, st, u) in vis:
                                continue
                            if not base[u]["ok"]:
                                vis[(j, st, u)] = {**base[u], "pruned": True}
                                continue
                            T_wb = T_wall_board(ref, p, a + u)
                            cover = covering(cfg, ref, T_wb, sc.stones)
                            if cover is not None:
                                vis[(j, st, u)] = {"ok": False, "reason": f"covered by stone c{cover.course} "
                                                   f"u={cover.u:.0f}", "n_ik": base[u]["n_ik"], "n_checked": 0,
                                                   "n_free": 0, "n_render": 0, "best": None, "view": None,
                                                   "covered": True}
                                continue
                            r = S.evaluate("ref", S.T_aw @ T_wb)
                            vis[(j, st, u)] = r
                            if r["ok"]:                      # valid with fewer stones too (same pose)
                                for st2 in STATES[:STATES.index(st)]:
                                    if covering(cfg, ref, T_wb, sc_by[st2].stones) is None:
                                        vis.setdefault((j, st2, u), {**r, "propagated": st})
                profiles[(tk, mk)] = vis
                n_ok = {st: sum(1 for (j, s, u), r in vis.items() if s == st and r["ok"]) for st in
                        ("empty",) + STATES}
                print(f"[{tk}/{mk}] {time.time() - t0:5.0f} s  visible positions: {n_ok}  "
                      f"(IK calls {S.n_ik_calls}, collision checks {S.n_coll}, renders {S.n_render})", flush=True)
                for (j, st, u), r in sorted(vis.items(), key=lambda kv: (kv[0][0], STATES.index(kv[0][1])
                                                                          if kv[0][1] in STATES else -1, kv[0][2])):
                    rows.append(csv_row(tk, mk, j, st, u, (stops[j - 1] if j else stops[0]) + u, r))

        # ── layouts ──────────────────────────────────────────────────────────
        pitches = [p for p in (1400.0, 1000.0, 800.0, 700.0, 600.0, 500.0, 400.0, 300.0) if p % step == 0]
        reach0 = max(abs(u) for u, ok in table[0].items() if ok)          # stone reach of course 0
        lay = {k: best_layouts(v, stops, grid, step, pitches, reach0) for k, v in profiles.items()}
        extra["reach0"] = reach0
        conf = {}
        for k, v in profiles.items():                       # configured target x positions
            cnt = {}
            for j, a in enumerate(stops, start=1):
                for st in STATES:
                    cnt[(j, st)] = sum(1 for t in targets
                                       if vis_ok(v, j, st, round(t["xyz"][0] + bw / 2 - a, 1)))
            conf[k] = cnt
        extra["layouts"], extra["configured"] = lay, conf

        # ── recommended combination: robustness at "complete" with all candidates checked ──
        rec = recommend(lay, mount_keys)
        extra["rec_robust"] = bool(rec and rec[2]["min_r"] >= 2)
        extra["recommended"] = rec
        if rec:
            tk, mk, L_ = rec
            pitch, off = L_["pitch"], L_["offset"]
            S.set_mount(mk)
            detail = []
            for j, a in enumerate(stops, start=1):
                sc = next(s for s in all_scenes if s.stop == j and s.state == "complete")
                S.set_scene(sc)
                for k in range(-10, 30):
                    u = round(off + k * pitch - (a - stops[0]), 1)
                    if u not in grid or not good(profiles[(tk, mk)].get((j, "complete", u)), u, False, reach0):
                        continue
                    T_wb = T_wall_board(ref, P[tk], a + u)
                    r = S.evaluate("ref", S.T_aw @ T_wb, count_free=True)
                    detail.append((j, u, a + u, r))
                    print(f"   rec {tk}/{mk} S{j} u_rel={u:+.0f}: ok={r['ok']} free {r['n_free']}/{r['n_ik']}",
                          flush=True)
            extra["rec_detail"] = detail

        # ── deck board near the front edge (E), calib board, pick-up station ──
        S.set_mount("M0")
        extra["E"] = deck_front(S, all_scenes)
        extra["calib"] = calib_board(S, all_scenes[0])
        extra["station"] = station_boards(S, all_scenes[0])

        # ── snapshots ────────────────────────────────────────────────────────
        if not args.no_snapshots:
            extra["snapshots"] = snapshots(S, args, P, profiles, all_scenes, stops, ref, extra)
        if args.save_rdk:
            RDK.Save(str(args.save_rdk), S.it["station"])
            print("saved", args.save_rdk, flush=True)
        extra["counts"] = {"ik": S.n_ik_calls, "coll": S.n_coll, "render": S.n_render}
    finally:
        rc = close_instance(RDK)
        print(f"RoboDK instance closed (exit code {rc})", flush=True)

    runtime = time.time() - t_start
    if args.dump:
        with open(args.dump, "wb") as fh:
            pickle.dump({"cfg": cfg, "args": args, "plan": plan, "all_scenes": all_scenes, "grid_all": grid_all,
                         "conf_u": conf_u, "profiles": profiles, "extra": extra, "P": P, "M": M, "types": types,
                         "mount_keys": mount_keys, "runtime": runtime, "rows": rows}, fh)
    write_csv(RESULTS / f"{args.out}.csv", rows)
    write_report(RESULTS / f"{args.out}.md", cfg, args, plan, all_scenes, grid_all, conf_u, profiles, extra, P, M,
                 types, mount_keys, runtime)
    print(f"wrote results/{args.out}.md and .csv ({runtime / 60:.1f} min)", flush=True)


def vis_ok(vis: dict, j: int, st: str, u: float) -> bool:
    return bool(vis.get((j, st, u), {}).get("ok"))


def covering(cfg: dict, spec: dict, T_wb: np.ndarray, stones) -> object | None:
    """The first stone (in the given order) whose body overlaps the board plate (board inside the wall)."""
    b = cfg["brick"]
    bw, bh = board_size(spec)
    box = aabb_of(T_wb, [-spec["square_mm"], -spec["square_mm"], 0.0],          # 3 mm plate behind the print
                  [bw + spec["square_mm"], bh + spec["square_mm"], 3.0])
    for s in stones:
        lo = np.array([s.u - b["length"] / 2, -b["width"] / 2, s.z_top - b["height"]])
        hi = np.array([s.u + b["length"] / 2, b["width"] / 2, s.z_top])
        if np.all(box[1] > lo + 1e-6) and np.all(box[0] < hi - 1e-6):
            return s
    return None


def deck_front(S: Study, all_scenes: list) -> list:
    """(E) face-up board on the deck near the front edge, ARES frame (references ARES only, not the wall)."""
    cfg = S.cfg
    spec = S.specs["ref"]
    bw, bh = board_size(spec)
    out = []
    x_c = cfg["ares"]["length"] / 2 - bh / 2 - spec["square_mm"] - 5.0     # plate 5 mm inside the front edge
    worst = next(s for s in all_scenes if s.stop == 2 and s.state == "complete")
    for y_c in (0.0, 200.0, -200.0):
        # long board side along ARES y, short side (incl. quiet zone) inside the front edge
        T_ab = (g.transl(x_c, y_c, cfg["ares"]["deck_top_z"] + 3.0) @ g.rotz(math.pi / 2) @ g.rotx(math.pi)
                @ g.transl(-bw / 2, -bh / 2, 0))
        for sc in (all_scenes[0], worst):
            S.set_scene(sc)
            r = S.evaluate("ref", T_ab, count_free=True)
            out.append({"x": x_c, "y": y_c, "scene": sc.name, **r})
            print(f"   E deck board ({x_c:.0f}, {y_c:+.0f}) {sc.name}: {r['reason']} free {r['n_free']}/{r['n_ik']}",
                  flush=True)
    return out


def calib_board(S: Study, empty: Scene) -> dict:
    """Calibration board on the deck ([boards.calib], magazine removed): all look poses checked for collisions,
    a spread sample of the collision-free ones rendered."""
    cfg = S.cfg
    S.set_scene(empty, magazine=False)
    spec = S.specs["calib"]
    T_ab = g.from_robodk(bs.board_pose(spec))
    cands = S.candidates("calib", T_ab)
    S.place_board("calib", T_ab)
    free = [c for c in cands if not S.collide(c.j) and not S.inside(c.j, c.T_ac)]
    sample = free[:: max(1, len(free) // 12)][:12]
    views = [S.view(c, "calib", T_ab) for c in sample]
    tilts = Counter(int(c.tilt) for c in free)
    hp = cfg.get("vision", {}).get("handeye_plan", {})
    in_plan = [c for c in free if hp and hp["dist_mm"][0] - 1e-6 <= c.d <= hp["dist_mm"][1] + 1e-6
               and hp["tilt_deg"][0] - 1e-6 <= c.tilt <= hp["tilt_deg"][1] + 1e-6]
    out = {"n_grid": len(S.grid("calib")), "n_ik": len(cands), "n_free": len(free),
           "tilts": dict(sorted(tilts.items())),
           "n_sample": len(sample), "n_sample_ok": sum(v["ok"] for v in views),
           "pnp_mm_max": max((v["pnp_mm"] for v in views if v["ok"]), default=float("nan")),
           "dists": sorted({c.d for c in free}), "free": free, "views": views, "plan": hp,
           "n_in_plan": len(in_plan)}
    print(f"   calib board: IK {len(cands)}, collision-free {len(free)}, rendered ok {out['n_sample_ok']}/"
          f"{len(sample)}", flush=True)
    S.place_board("calib", None)
    return out


def station_boards(S: Study, empty: Scene) -> list:
    """S0/S1 at the pick-up station with ARES docked (PLACEHOLDER layout): literally as configured (station frame at
    floor level -> boards on the floor) and on the table top (z + table_z, as the target comment intends)."""
    cfg = S.cfg
    T_wa = g.from_robodk(bs.T_wall_station(cfg)) @ g.from_robodk(bs.T_station_ares(cfg))
    T_aw = g.inv(T_wa)
    S.set_scene(empty, magazine=False, T_aw=T_aw, table=True)
    out = []
    z_tab = cfg["pickup_station"]["table_z"]
    for t in cfg.get("targets", []):
        if t["parent"] != "station":
            continue
        for variant, dz in (("as configured (floor)", 0.0), ("on the table top", z_tab)):
            T_sb = g.transl(0, 0, dz) @ g.pose_xyz_rpy(t["xyz"], t.get("rpy_deg", (0, 0, 0)))
            T_ab = T_aw @ g.from_robodk(bs.T_wall_station(cfg)) @ T_sb
            r = S.evaluate("ref", T_ab, count_free=True)
            c = g.apply(T_ab, np.array([[board_size(S.specs["ref"])[0] / 2, board_size(S.specs["ref"])[1] / 2, 0]]))
            out.append({"name": t["name"], "variant": variant, "centre_ares": c[0], **r})
            print(f"   station {t['name']} {variant}: {r['reason']} free {r['n_free']}/{r['n_ik']}", flush=True)
    S.place_board("ref", None)
    return out


def snapshots(S: Study, args, P: dict, profiles: dict, all_scenes: list, stops: list, ref: dict, extra: dict) -> list:
    """Overview + camera view for: the recommended combination at its worst scene, the PLACEHOLDER (A/M0) at the
    worst scene of stop 2, the deck calibration board."""
    out = []
    shots = []
    rec = extra.get("recommended")
    if rec:
        shots.append(("rec", rec[0], rec[1], 2, "complete"))
    if ("A", "M0") in profiles:
        shots.append(("A", "A", "M0", 2, "complete"))
        shots.append(("A_arrival", "A", "M0", 2, "arrival"))
    for tag, tk, mk, j, st in shots:
        vis = profiles[(tk, mk)]
        S.set_mount(mk)
        sc = next(s for s in all_scenes if s.stop == j and s.state == st)
        S.set_scene(sc)
        ok_u = [u for (jj, ss, u), r in vis.items() if jj == j and ss == st and r["ok"]]
        any_u = ok_u or [u for (jj, ss, u), r in vis.items() if jj == j and ss == "arrival" and r["ok"]]
        if not any_u:
            continue
        u = min(any_u, key=abs)
        T_wb = T_wall_board(ref, P[tk], stops[j - 1] + u)
        T_ab = S.T_aw @ T_wb
        r = S.evaluate("ref", T_ab)
        c = r["best"]
        if c is None:                                         # not visible here: show the best collision-free try
            cands = [c for c in S.candidates("ref", T_ab)[:args.max_checks]
                     if not S.collide(c.j) and not S.inside(c.j, c.T_ac)]
            c = cands[0] if cands else None
        if c is None:
            continue
        S.robot.setJoints(c.j)
        S.RDK.Update()
        cam_img = S.view(c, "ref", T_ab, keep=True)["img"]
        name = f"look_{tag}_{tk}_{mk}_S{j}_{st}"
        cv2.imwrite(str(RESULTS / f"{name}_cam.png"), cv2.resize(cam_img, (S.W // 2, S.H // 2),
                                                                   interpolation=cv2.INTER_AREA))
        tgt = g.apply(T_ab, np.array([[board_size(ref)[0] / 2, board_size(ref)[1] / 2, 0]]))[0]
        eye = [tgt[0] - 500.0, tgt[1] - 1900.0, tgt[2] + 1100.0]
        S.robot.setJoints(c.j)
        S.RDK.Render(True)                                    # cameras only render if opened with rendering on
        snapshot(S.RDK, RESULTS / f"{name}.png", eye=eye, target=[tgt[0] - 250, tgt[1], tgt[2] + 150])
        S.RDK.Render(True)
        S.cam.open()                                          # snapshot() closed all camera windows
        S.cam.set_flat_light(True)
        S.RDK.Render(False)
        out.append((name, tk, mk, sc.name, u, r["ok"]))
        print(f"   snapshot results/{name}.png (+ _cam.png), board ok={r['ok']}", flush=True)
    # calibration board
    cal = extra.get("calib")
    if cal and cal["free"]:
        S.set_mount("M0")
        S.set_scene(all_scenes[0], magazine=False)
        T_ab = g.from_robodk(bs.board_pose(S.specs["calib"]))
        S.place_board("calib", T_ab)
        c = cal["free"][0]
        S.robot.setJoints(c.j)
        S.RDK.Update()
        img = S.view(c, "calib", T_ab, keep=True)["img"]
        cv2.imwrite(str(RESULTS / "look_calib_cam.png"), cv2.resize(img, (S.W // 2, S.H // 2)))
        S.robot.setJoints(c.j)
        S.RDK.Render(True)
        snapshot(S.RDK, RESULTS / "look_calib.png", eye=[-900, -1800, 1500], target=[100, 0, 450])
        S.RDK.Render(True)
        S.cam.open()
        S.cam.set_flat_light(True)
        S.RDK.Render(False)
        S.place_board("calib", None)
        out.append(("look_calib", "calib", "M0", "deck", 0.0, True))
    return out


# ── outputs ───────────────────────────────────────────────────────────────────
CSV_FIELDS = ["type", "mount", "stop", "state", "u_rel", "u_wall", "ok", "reason", "d_mm", "tilt_deg", "az_deg",
              "roll_deg", "margin_px", "slack_mm", "clear_mm", "approach", "n_ik", "n_checked", "n_free", "n_render",
              "corners", "occl",
              "pnp_err_mm", "joints_deg", "propagated_from"]


def csv_row(tk, mk, j, st, u, u_wall, r) -> dict:
    c, v = r.get("best"), r.get("view") or {}
    return {"type": tk, "mount": mk, "stop": j, "state": st, "u_rel": u, "u_wall": round(u_wall, 1),
            "ok": int(bool(r["ok"])), "reason": r["reason"],
            "d_mm": c.d if c else "", "tilt_deg": c.tilt if c else "", "az_deg": round(c.az, 1) if c else "",
            "roll_deg": round(c.roll, 1) if c else "", "margin_px": round(c.margin_px, 1) if c else "",
            "slack_mm": round(c.slack_mm, 1) if c else "", "clear_mm": c.clear_mm if c else "",
            "approach": c.approach if c else "", "n_ik": r.get("n_ik", ""),
            "n_checked": r.get("n_checked", ""), "n_free": r.get("n_free", ""), "n_render": r.get("n_render", ""),
            "corners": v.get("corners", ""), "occl": round(v["occl"], 5) if "occl" in v else "",
            "pnp_err_mm": round(v["pnp_mm"], 3) if "pnp_mm" in v else "",
            "joints_deg": " ".join(f"{x:.1f}" for x in c.j) if c else "", "propagated_from": r.get("propagated", "")}


def write_csv(path: Path, rows: list) -> None:
    with open(path, "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        wr.writeheader()
        wr.writerows(rows)


def symbol(r: dict | None) -> str:
    if not r:
        return " "
    if r["ok"]:
        cl = r["best"].clear_mm
        return "#" if cl >= 30 else "+" if cl >= 20 else "=" if cl >= 10 else "-" if cl >= 5 else "!"
    if r.get("covered"):
        return "x"
    reason = r["reason"]
    if reason.startswith("no IK"):
        return "."
    if reason.startswith("collision"):
        return "c"
    return "o"


def write_report(path: Path, cfg: dict, args, plan, all_scenes, grid_all, conf_u, profiles, extra, P, M, types,
                 mount_keys, runtime) -> None:
    text = (REPO / "config" / "station.toml").read_text(encoding="utf-8")
    stops = [a for a, _ in plan]
    ref = next(s for s in bs.board_layout(cfg) if s["parent"] == "wall")
    bw, bh = board_size(ref)
    L = []
    w = L.append
    w("# Look-pose / visibility study: reference boards for the flange camera")
    w("")
    w(f"Generated by `robodk/look_study.py` ({time.strftime('%Y-%m-%d %H:%M')}, run time {runtime / 60:.1f} min, "
      f"{'quick grid' if args.quick else 'full grid'}; own RoboDK instance, OpenCV {cv2.__version__}). "
      f"Per-position data: `results/{args.out}.csv`.")
    w("")
    w("**All numbers are simulation results that depend on PLACEHOLDER/ASSUMPTION inputs (list below).**")
    w("")
    w("## Inputs")
    w("")
    deps = [("ur5", "mount_z"), ("ur5", "mount_rz"), ("ur5", "mount_y"), ("tool", "tcp_z"), ("wall", "courses"),
            ("wall", "base_z"), ("wall", "dist_nominal"), ("deck", "holder_z"), ("camera", "working_dist"),
            ("boards.ref", "square_mm"), ("boards.ref", "marker_mm"), ("boards.calib", "xyz"),
            ("pickup_station", "xyz_in_wall"), ("pickup_station", "table_z"), ("pickup_station", "ares_xyz")]
    w("| config key | value | status |")
    w("|---|---|---|")
    for sec, key in deps:
        d = cfg
        for part in sec.split("."):
            d = d[part]
        w(f"| `[{sec}] {key}` | {d[key]} | {status_of(text, sec, key)} |")
    w(f"| `[camera.mount]` | xyz {cfg['camera']['mount']['xyz']}, rpy {cfg['camera']['mount']['rpy_deg']} | "
      "PLACEHOLDER (section comment) |")
    w(f"| `[[targets]]` W0..W7, S0, S1 | see config | PLACEHOLDER (section comment) |")
    w("")
    w(f"- Wall plan (offline, `wallplan.sequence` on `results/reach_table.json`, as `simulate.py --length "
      f"{args.length}`): {sum(len(b) for _, b in plan)} stones, {len(plan)} stops at a = "
      + ", ".join(f"{a:.0f}" for a in stops) + " mm (ARES centre in wall coordinates, wall start u = 0) with "
      + ", ".join(str(len(b)) for _, b in plan) + " stones.")
    w(f"- Camera: ideal pinhole {cfg['camera']['res_x']} x {cfg['camera']['res_y']} px, f = {cfg['camera']['focal']} "
      f"mm, pixel {cfg['camera']['pixel_um']} µm (RoboDK Cam2D, verified 2026-10-05); body 29 mm cube + Ø29 x 47.6 mm "
      "lens, projection centre 5.5 mm behind the C-mount flange (ASSUMPTION, `rdk_common.camera_body_points`).")
    w(f"- Boards: [boards.ref] {ref['squares_x']} x {ref['squares_y']} squares of {ref['square_mm']} mm "
      f"({bw:.0f} x {bh:.0f} mm, {ref['dictionary']}), white quiet zone of one square (ASSUMPTION); "
      f"[boards.calib] {cfg['boards']['calib']['squares_x']} x {cfg['boards']['calib']['squares_y']} x "
      f"{cfg['boards']['calib']['square_mm']} mm.")
    w(f"- Look poses: d = {', '.join(f'{d:.0f}' for d in args.dists)} mm, axis-to-normal angle "
      f"{', '.join(f'{t:.0f}' for t in args.tilts)} deg, {args.n_az} azimuths, {args.n_roll} camera rolls; "
      f"corners >= {MARGIN_PX:.0f} px inside; arm in `motion.family()`; static collision check (no stone held); "
      f"render: <= {OCCL_MAX * 100:.1f} % coloured pixels on the board and all "
      f"{(ref['squares_x'] - 1) * (ref['squares_y'] - 1)} ChArUco corners detected. Budget per board and scene: "
      f"{args.max_checks} collision checks, {args.max_renders} renders.")
    w("- Scenes: magazine full (18 stones, worst case), base plate under course 0 (stone width, ASSUMPTION). "
      "\"empty\" = no stones, no base plate. A pose valid at \"complete\" is valid for every state of that stop.")
    w("")
    stt = extra.get("self_test") or {}
    w("Collision self-test (known collisions must be reported): " + "; ".join(
        f"{k}: {', '.join(sorted(set(v))) if v else ('no IK' if v is None else 'NOT detected')}"
        for k, v in stt.items()))
    w("")
    w("## Camera mounts")
    w("")
    w("| mount | T_flange_cam | camera body vs gripper mesh |")
    w("|---|---|---|")
    for mk in mount_keys:
        desc, xyz, rpy = M[mk]
        mc = extra.get("mount_check", {}).get(mk)
        w(f"| {mk} | {desc}; xyz {xyz}, rpy {rpy} | {'free' if not mc else 'INTERFERES: ' + ', '.join(mc)} |")
    w("")
    w("## Board placement types (wall frame: x along the wall, y towards ARES, z up; board centre)")
    w("")
    w("| type | placement | centre y, z [mm] | note |")
    w("|---|---|---|---|")
    for tk in types:
        p = P[tk]
        w(f"| {tk} | {p['title']} | {p['y']:.0f}, {p['z']:.0f} | {p['note']} |")
    w("")
    w("## Layouts with >= 2 boards per stop and state")
    w("")
    w("Regular layouts: boards every `pitch` mm, offset relative to stop 1; count = visible boards in the worst "
      "stop/state. Configured = the [[targets]] x positions (W0..W7, every 700 mm from u = 0).")
    w("")
    w(f"A board counts as **visible** if a valid look pose exists (IK in the family, collision-free static pose, "
      f"free 150 mm approach, board unoccluded and fully detected), and as **robust** if, in addition, its look "
      f"pose keeps >= 10 mm clearance and the board lies inside the stone reach of course 0 (|u_rel| <= "
      f"{extra.get('reach0', 0):.0f} mm). Layouts are ranked by robust boards in the worst stop/state, then visible.")
    w("")
    w("| type | mount | largest pitch with >= 2 robust | ... and >= 3 visible (spare) | pitch [mm] -> worst robust / "
      "visible | configured targets: min visible (worst) |")
    w("|---|---|---|---|---|---|")
    for (tk, mk), ls in extra.get("layouts", {}).items():
        r2 = [l for l in ls if l["min_r"] >= 2]
        r3 = [l for l in r2 if l["min_v"] >= 3]
        c2 = max(r2, key=lambda l: l["pitch"]) if r2 else None
        c3 = max(r3, key=lambda l: l["pitch"]) if r3 else None
        cnt = extra["configured"][(tk, mk)]
        wc = min(cnt, key=cnt.get)
        p2 = f"{c2['pitch']:.0f}" if c2 else "none"
        p3 = f"{c3['pitch']:.0f}" if c3 else "none"
        w(f"| {tk} | {mk} | {p2} | {p3} | "
          + ", ".join(f"{l['pitch']:.0f}: {l['min_r']}/{l['min_v']}" for l in ls)
          + f" | {cnt[wc]} (S{wc[0]} {wc[1]}) |")
    w("")
    rec = extra.get("recommended")
    if rec:
        tk, mk, L_ = rec
        pitch, off = L_["pitch"], L_["offset"]
        u1 = stops[0] + off
        w(("" if extra.get("rec_robust") else "(no layout with >= 2 robust boards - visible-only criterion) ")
          + f"**Recommended: type {tk} ({P[tk]['title']}) with mount {mk}, a board every {pitch:.0f} mm** "
          f"(board centres at wall u = {(u1 % pitch):.0f} + k x {pitch:.0f} mm): in every stop and state at least "
          f"{L_['min_r']} robust and {L_['min_v']} visible boards (worst: {L_['worst']}); smallest baseline between "
          f"the outer robust boards {L_['baseline']:.0f} mm.")
        w("")
        w("| stop | a [mm] | " + " | ".join(f"{st} robust / visible" for st in STATES) + " |")
        w("|---|---|" + "---|" * len(STATES))
        for j, a in enumerate(stops, start=1):
            w(f"| S{j} | {a:.0f} | " + " | ".join(f"{L_['counts_r'][(j, st)]} / {L_['counts_v'][(j, st)]}"
                                                 for st in STATES) + " |")
        w("")
        det = extra.get("rec_detail", [])
        if det:
            w("Margins of these boards at \"complete\" (all look-pose candidates checked for collisions):")
            w("")
            w("| stop | board u (wall) | u rel. to ARES | look pose d / tilt | image slack [mm] | clearance [mm] | "
              "approach | collision-free poses / IK-feasible | PnP error vs truth [mm] |")
            w("|---|---|---|---|---|---|---|---|---|")
            for j, u, uw, r in det:
                c, v = r["best"], r["view"] or {}
                w(f"| S{j} | {uw:.0f} | {u:+.0f} | " + (f"{c.d:.0f} mm / {c.tilt:.0f} deg | {c.slack_mm:.1f} | "
                                                          f">= {c.clear_mm:.0f} | {c.approach} | "
                                                          if c else "- | - | - | - | ")
                  + f"{r['n_free']} / {r['n_ik']} | " + (f"{v['pnp_mm']:.3f}" if v else "-") + " |")
            w("")
            us = sorted({round(uw, 1) for _, _, uw, _ in det})
            w("Suggested `[[targets]]` (board origin = top-left outer corner, wall frame) for this layout:")
            w("")
            w("```toml")
            for i, uw in enumerate(us):
                T = T_wall_board(ref, P[tk], uw)
                xyz, rpy = g.xyz_rpy(T)
                rpy = [0.0 if abs(v) < 1e-9 else v for v in rpy]
                w(f"[[targets]]\nname = \"W{i}\"\nparent = \"wall\"\nfirst_id = {30 + 10 * i}\n"
                  f"xyz = [{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]   # PLACEHOLDER: look_study "
                  f"{time.strftime('%Y-%m-%d')} type {tk}\nrpy_deg = [{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}]")
            w("```")
            w("")
    else:
        w("**No evaluated combination gives >= 2 visible boards in every stop and state.**")
        w("")
    w("## Visibility profiles")
    w("")
    w("One row per scene, one column per board position u relative to the ARES centre (left = towards the wall "
      f"start), {args.step:.0f} mm grid from {grid_all[0]:+.0f} to {grid_all[-1]:+.0f} mm "
      f"(configured target positions {conf_u} included, marked `'`; `|` = ARES centre). Visible with a "
      "collision-free 150 mm approach and clearance `#` >= 30 mm, `+` >= 20 mm, `=` >= 10 mm, `-` >= 5 mm, "
      "`!` < 5 mm; "
      "`c` every IK-feasible look pose collides, `o` collision-free poses exist but no free approach / board "
      "occluded / not detected, `.` no look pose with IK in the allowed family, `x` board position occupied by a "
      "stone. Clearance = largest shift of the camera pose (+-x, +-y in the ARES frame) that stays collision-free; "
      "it also bounds the ARES stop error the planned look pose tolerates.")
    w("")
    header_u = [u for u in grid_all]
    for (tk, mk), vis in profiles.items():
        w(f"### {tk} / {mk}")
        w("")
        w("```")
        w("scene         " + "".join("|" if abs(u) < 1e-6 else ("'" if u in conf_u else " ") for u in header_u))
        w(f"{'empty':13s} " + "".join(symbol(vis.get((0, 'empty', u))) for u in header_u))
        for j in range(1, len(stops) + 1):
            for st in STATES:
                w(f"{f'S{j} {st}':13s} " + "".join(symbol(vis.get((j, st, u))) for u in header_u))
        w("```")
        reasons = Counter(r["reason"] for (j, st, u), r in vis.items() if not r["ok"] and st != "empty"
                          and not r.get("pruned"))
        if reasons:
            w("Most frequent failure reasons (stop scenes): " + "; ".join(f"{k} ({n})" for k, n in
                                                                        reasons.most_common(4)))
        w("")
    d_keys = [k for k in profiles if k[0] == "D"]
    if d_keys:
        vis = profiles[d_keys[0]]
        w(f"## D: boards on the base plate - when are they covered? ({d_keys[0][1]})")
        w("")
        w("Boards on the base plate are only usable until the first stone of the stop covers them; they have to be "
          "removed before (by hand or with a tool the gripper does not have). Board positions visible at arrival and "
          "the stone of the stop (placement order of `wallplan.sequence`) that covers them first:")
        w("")
        w("| stop | stones in the stop | visible at arrival: u_rel [mm] -> covered by stone # |")
        w("|---|---|---|")
        for j, (a, batch) in enumerate(plan, start=1):
            items = []
            for u in grid_all:
                if not vis_ok(vis, j, "arrival", u):
                    continue
                T_wb = T_wall_board(ref, P["D"], a + u)
                k = next((i for i, st_ in enumerate(batch, start=1) if covering(cfg, ref, T_wb, [st_])), None)
                items.append(f"{u:+.0f} -> {k if k else 'never'}")
            w(f"| S{j} | {len(batch)} | {', '.join(items) if items else 'none'} |")
        w("")
    w("## Deck board near the front edge (E)")
    w("")
    w("References only ARES/the UR5 mount, not the wall - it cannot replace wall boards "
      "(useful as a sway/mount check).")
    w("")
    w("| centre (ARES x, y) [mm] | scene | result | clearance [mm] | collision-free / IK-feasible |")
    w("|---|---|---|---|---|")
    for e in extra.get("E", []):
        cl = f">= {e['best'].clear_mm:.0f}" if e.get("best") else "-"
        w(f"| {e['x']:.0f}, {e['y']:+.0f} | {e['scene']} | {e['reason']} | {cl} | {e['n_free']} / {e['n_ik']} |")
    w("")
    cal = extra.get("calib")
    if cal:
        w("## Calibration board on the deck ([boards.calib], magazine removed)")
        w("")
        w(f"Look-pose grid (board fully in view with {MARGIN_PX:.0f} px margin): {cal['n_grid']} poses, "
          f"{cal['n_ik']} with IK in the family, **{cal['n_free']} collision-free** (by axis-to-normal angle "
          f"[deg]: {cal['tilts']}; distances {cal['dists']} mm). Rendered sample: {cal['n_sample_ok']}/"
          f"{cal['n_sample']} fully detected, PnP error <= {cal['pnp_mm_max']:.3f} mm. "
          + (f"Inside the planned hand-eye ranges ([vision.handeye_plan] d {cal['plan']['dist_mm']} mm, tilt "
             f"{cal['plan']['tilt_deg']} deg): {cal['n_in_plan']} collision-free poses "
             f"(n_poses = {cal['plan']['n_poses']}). " if cal.get("plan") else "")
          + f"The board at "
          f"{cfg['boards']['calib']['xyz']} overlaps the magazine row at x = "
          f"{cfg['ur5']['mount_x'] + MAG_ROWS[1][0]:.0f} mm: the hand-eye calibration needs that part of the deck "
          "cleared.")
        w("")
    st = extra.get("station")
    if st:
        w("## Pick-up station boards (ARES docked at the PLACEHOLDER pose)")
        w("")
        w("The config is ambiguous: `xyz_in_wall` and `ares_xyz` have z = 0 (station frame at floor level), while the "
          "targets S0/S1 (z = 3 mm) are described as \"face up on the station table\" and docs/ARCHITECTURE.md puts "
          "the station origin on the table top. Both readings were checked.")
        w("")
        w("| board | variant | centre in ARES frame [mm] | result | look pose d / tilt | clearance [mm] | "
          "collision-free / IK-feasible |")
        w("|---|---|---|---|---|---|---|")
        for e in st:
            c, b_ = e["centre_ares"], e.get("best")
            w(f"| {e['name']} | {e['variant']} | ({c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f}) | {e['reason']} | "
              + (f"{b_.d:.0f} / {b_.tilt:.0f} | >= {b_.clear_mm:.0f}" if b_ else "- | -")
              + f" | {e['n_free']} / {e['n_ik']} |")
        w("")
    shots = extra.get("snapshots") or []
    if shots:
        w("## Snapshots (results/*.png, not versioned; analysis colours)")
        w("")
        for name, tk, mk, sc, u, ok in shots:
            w(f"- `results/{name}.png` + `{name}_cam.png`: {tk}/{mk}, {sc}, board at u_rel {u:+.0f} mm, "
              f"{'visible' if ok else 'NOT visible (best collision-free try)'}")
        w("")
    w("## Notes on the simulation (RoboDK 6.0.0.26652, observed in this study)")
    w("")
    w("- Cam2D cameras render only if opened while rendering is on (`RDK.Render(True)`); afterwards snapshots also "
      "follow robot moves with rendering off.")
    w("- A textured object (board PNG) added after the first camera was opened renders black in every camera: all "
      "boards are created first.")
    w("- RoboDK's collision check does not report a body completely inside another mesh (camera origin 40 mm inside a "
      "stone: only the gripper was reported); bodies crossing a surface are reported. The ARES STEP is a hollow "
      "assembly, so an arm inside the chassis would pass unnoticed - the study therefore adds a solid-volume check "
      "(ARES envelope 1120 x 600 mm up to the deck top, stones, magazine, base plate, table as boxes) for the joint "
      "origins, link centre lines, flange -> TCP, flange -> camera (bracket line) and the camera body corners.")
    w("- Pick-up station: the table is modelled as a 30 mm top slab without legs or apron (ASSUMPTION), so looks "
      "under the table top are possible in the model; the \"as configured (floor)\" results depend on that.")
    w("- The modelled lens reaches 53 mm in front of the projection centre; it is hidden during each snapshot "
      "(otherwise the camera sees its own lens) and shown again for the collision checks.")
    w("")
    cnt = extra.get("counts", {})
    w("## Run")
    w("")
    w(f"`py.exe robodk/look_study.py {'--quick ' if args.quick else ''}--types {','.join(types)} --mounts "
      f"{','.join(mount_keys)}`: {runtime / 60:.1f} min, {cnt.get('ik', 0)} IK calls, {cnt.get('coll', 0)} "
      f"collision checks, {cnt.get('render', 0)} renders (2472 x 2064). RoboDK in its own instance (port "
      f"{args.port}), closed at the end; the user's station was not touched.")
    w("")
    w("Not checked: transfer motions to the look poses (static poses only), posts/brackets of raised boards, the "
      "camera bracket and cable, ARES sway on its casters, lighting/exposure, stones in the station slots, "
      "the safety scanner fields of ARES (posts in the gap).")
    path.write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
