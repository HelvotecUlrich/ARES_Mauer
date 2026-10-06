"""Coarse arm-vs-wall collision check (pure numpy) for look poses planned without RoboDK (tools/make_job.py,
tools/plan_layout.py): the UR5, the gripper and the flange camera as capsules, the BUILT wall as boxes.

Why: a look pose with a family IK solution can still put the wrist or the gripper into stones that are already placed
(review 2026-10-05: the stop-2 look at W4 had wrist 1 / wrist 2 / the flange inside the finished leg A). This check is
a planning filter, not a substitute for RoboDK's collision check (robodk/simulate.py LSim.check_board, look_study.py):
the shapes are envelopes with ASSUMED radii, the stones are boxes.

Arm model (UR5 CB3, joint angles q -> nominal DH frames, mauer.simworld): the DH chain collapses the lateral link
offsets into d4; the real upper arm runs `SHOULDER_OFFSET_MM` along the joint-2 axis from the DH line, the forearm
`SHOULDER_OFFSET_MM - ELBOW_OFFSET_MM` (ROS-Industrial ur_description ur5: shoulder_offset 135.85 mm, elbow_offset
119.7 mm, wrist_1_length 93.0 mm - consistent with the UR DH value d4 = 135.85 - 119.7 + 93.0 = 109.15 mm).
Capsules: base column, shoulder housing, upper arm, elbow housing, forearm, wrist 1, wrist 2, wrist 3 (radii
`LINK_RADII_MM`, ASSUMPTION from the joint housing sizes, UR5 outline drawing not looked up).
Tool (flange frame, z along the flange axis): gripper body on the axis down to the TCP, two open jaws at +-`JAW_X_MM`
along the flange x axis (the jaws close along flange x, config [tool]) reaching `JAW_BELOW_TCP_MM` below the TCP
(ASSUMPTION: jaw 26.7 x 60 x 68 mm, cad/README.md, partly below the held stone's top), the camera body + lens on the
optical axis of [camera.mount] and the adapter arm (cad/camera_adapter_018660_A_1.stp bbox x -168..31.5,
y -31.3..31.3, z -24..12, config [camera.adapter]).

Obstacles: `Box` (centre, rotation, half extents) in the parent frame (wall). `stone_boxes` builds them from
robodk/wallplan.py stones and their legs: length along the leg, width + 2 x [brick] rib_mm across (ribs on the long
faces), height [brick] height, plus the base plate under course 0 ([wall] base_z).

Units mm, rad. Frames: docs/ARCHITECTURE.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

import numpy as np

from . import geometry as g
from .simworld import UR5_A, UR5_ALPHA, UR5_D, _dh

SHOULDER_OFFSET_MM = 135.85       # ur_description ur5 shoulder_offset (see module docstring)
ELBOW_OFFSET_MM = 119.7           # ur_description ur5 elbow_offset
LINK_RADII_MM = {"base": 50.0, "shoulder": 50.0, "upper_arm": 50.0, "elbow": 45.0, "forearm": 40.0,
                 "wrist1": 40.0, "wrist2": 40.0, "wrist3": 40.0}      # ASSUMPTION (module docstring)
GRIPPER_BODY_RADIUS_MM = 45.0     # ASSUMPTION: EHPS-20-A body + adapter around the flange axis
JAW_X_MM = 75.0                   # ASSUMPTION: open jaw centre along flange x (stone 120 / 2 + jaw 26.7 / 2 + stroke)
JAW_RADIUS_MM = 30.0              # ASSUMPTION: jaw 60 mm wide (cad/README.md) -> +-30 mm
JAW_TOP_ABOVE_TCP_MM = 60.0       # ASSUMPTION: jaw top 60 mm above the TCP (jaw height 68 mm)
JAW_BELOW_TCP_MM = 45.0           # ASSUMPTION: jaw tip 45 mm below the TCP (= below the held stone's top)
CAMERA_RADIUS_MM = 25.0           # IDS housing 29 mm cube (half diagonal 20.5) / lens diameter 29 mm ([camera] body)
CAMERA_Z_MM = (-20.0, 60.0)       # flange z: camera housing behind the C-mount flange .. lens front ([camera.mount])
ADAPTER_RADIUS_MM = 36.0          # hypot(31.3, 18): adapter arm y +-31.3, z -24..12
ADAPTER_Z_MM = -6.0
CLEARANCE_MM = 10.0               # ASSUMPTION: extra distance beyond the envelopes
STEP_MM = 10.0                    # capsule sampling step


@dataclass(frozen=True)
class Box:
    name: str
    centre: np.ndarray            # (3,) parent frame
    R: np.ndarray                 # (3, 3) box axes in the parent frame
    half: np.ndarray              # (3,) half extents


@dataclass(frozen=True)
class Capsule:
    name: str
    a: np.ndarray
    b: np.ndarray
    r: float


def tool_capsules(cfg: Mapping) -> list[tuple[str, np.ndarray, np.ndarray, float]]:
    """Gripper and camera envelope in the FLANGE frame: (name, a, b, radius)."""
    tcp = float(cfg["tool"]["tcp_z"])
    cam = cfg["camera"]["mount"]["xyz"]
    cx, cy = float(cam[0]), float(cam[1])
    top = tcp - JAW_TOP_ABOVE_TCP_MM
    out = [("gripper", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, top]), GRIPPER_BODY_RADIUS_MM),
           ("jaw bar", np.array([-JAW_X_MM, 0.0, top]), np.array([JAW_X_MM, 0.0, top]), JAW_RADIUS_MM)]
    for s in (1.0, -1.0):
        out.append((f"jaw {'+' if s > 0 else '-'}x", np.array([s * JAW_X_MM, 0.0, top]),
                    np.array([s * JAW_X_MM, 0.0, tcp + JAW_BELOW_TCP_MM]), JAW_RADIUS_MM))
    out.append(("camera", np.array([cx, cy, CAMERA_Z_MM[0]]), np.array([cx, cy, CAMERA_Z_MM[1]]), CAMERA_RADIUS_MM))
    out.append(("camera adapter", np.array([cx, cy, ADAPTER_Z_MM]), np.array([0.0, 0.0, ADAPTER_Z_MM]),
                ADAPTER_RADIUS_MM))
    return out


def arm_capsules(q_rad: Sequence[float], T_parent_base: np.ndarray, tool: Sequence | None = None) -> list[Capsule]:
    """Capsules of the UR5 at joints q (and of the tool, flange-frame list of tool_capsules) in the parent frame."""
    T = np.asarray(T_parent_base, float).copy()
    fr = [T.copy()]
    for i in range(6):
        T = T @ _dh(float(q_rad[i]), UR5_D[i], UR5_A[i], UR5_ALPHA[i])
        fr.append(T.copy())
    p = [F[:3, 3] for F in fr]
    z1 = fr[1][:3, 2]                                            # joint-2 axis (parallel to joints 3 and 4)
    s, e = SHOULDER_OFFSET_MM * z1, (SHOULDER_OFFSET_MM - ELBOW_OFFSET_MM) * z1
    R = LINK_RADII_MM
    caps = [Capsule("base", p[0], p[1], R["base"]), Capsule("shoulder", p[1], p[1] + s, R["shoulder"]),
            Capsule("upper arm", p[1] + s, p[2] + s, R["upper_arm"]), Capsule("elbow", p[2] + s, p[2] + e, R["elbow"]),
            Capsule("forearm", p[2] + e, p[3] + e, R["forearm"]), Capsule("wrist 1", p[3] + e, p[4], R["wrist1"]),
            Capsule("wrist 2", p[4], p[5], R["wrist2"]), Capsule("wrist 3", p[5], p[6], R["wrist3"])]
    if tool:
        F = fr[6]
        for name, a, b, r in tool:
            caps.append(Capsule(name, g.apply(F, [a])[0], g.apply(F, [b])[0], float(r)))
    return caps


def _samples(caps: Sequence[Capsule], step: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pts, rad, idx = [], [], []
    for i, c in enumerate(caps):
        n = max(1, int(math.ceil(float(np.linalg.norm(c.b - c.a)) / step)))
        t = np.linspace(0.0, 1.0, n + 1)[:, None]
        pts.append(c.a + t * (c.b - c.a))
        rad.append(np.full(n + 1, c.r))
        idx.append(np.full(n + 1, i))
    return np.vstack(pts), np.concatenate(rad), np.concatenate(idx)


def box_distance(P: np.ndarray, boxes: Sequence[Box]) -> np.ndarray:
    """Signed distance of points P (N, 3) to every box (M, N): > 0 outside, < 0 inside."""
    if not boxes:
        return np.zeros((0, len(P)))
    C = np.stack([b.centre for b in boxes])
    Rs = np.stack([b.R for b in boxes])
    H = np.stack([b.half for b in boxes])
    d = np.einsum("mji,mnj->mni", Rs, P[None, :, :] - C[:, None, :])          # points in the box frames
    q = np.abs(d) - H[:, None, :]
    out = np.linalg.norm(np.maximum(q, 0.0), axis=2)
    return out + np.minimum(q.max(axis=2), 0.0)


def collisions(caps: Sequence[Capsule], boxes: Sequence[Box], clearance_mm: float = CLEARANCE_MM,
               step_mm: float = STEP_MM) -> list[tuple[str, str, float]]:
    """[(capsule, box, penetration mm)] for every capsule that comes closer than its radius + clearance to a box
    (deepest per pair), deepest first. Capsules are sampled every step_mm (an error < step / 2 on thin corners)."""
    if not boxes or not caps:
        return []
    P, r, idx = _samples(caps, step_mm)
    D = box_distance(P, boxes) - (r + clearance_mm)[None, :]                 # (M, N), < 0 = hit
    hits: dict[tuple[int, int], float] = {}
    for m, n in zip(*np.nonzero(D < 0.0)):
        k = (int(idx[n]), int(m))
        hits[k] = max(hits.get(k, 0.0), float(-D[m, n]))
    return sorted(((caps[i].name, boxes[m].name, d) for (i, m), d in hits.items()), key=lambda h: -h[2])


def stone_boxes(cfg: Mapping, stones: Iterable, legs: Mapping[str, object] | None = None,
                base_plates: bool = True) -> list[Box]:
    """Boxes (wall frame) of wallplan stones (u, z_top, length, leg) - width incl. the ribs of the long faces
    ([brick] rib_mm) - and, if base_plates, the [wall] base_z plate under course 0 of every leg that has a stone.
    legs: {name: wallplan.Leg} (to_wall, theta); a stone with leg "" (straight wall) uses the wall frame."""
    b = cfg["brick"]
    L, W, H = float(b["length"]), float(b["width"]), float(b["height"])
    rib = float(b.get("rib_mm", 0.0))
    out, spans = [], {}
    for s in stones:
        lg = (legs or {}).get(getattr(s, "leg", "") or "")
        th = float(getattr(lg, "theta", 0.0)) if lg is not None else 0.0
        ln = float(getattr(s, "length", 0.0) or L)
        x, y = lg.to_wall(s.u, 0.0) if lg is not None else (s.u, 0.0)
        out.append(Box(getattr(s, "label", str(s)), np.array([x, y, s.z_top - H / 2.0]), g.rotz(th)[:3, :3],
                       np.array([ln / 2.0, W / 2.0 + rib, H / 2.0])))
        key = getattr(s, "leg", "") or ""
        lo, hi = spans.get(key, (math.inf, -math.inf))
        spans[key] = (min(lo, s.u - ln / 2.0), max(hi, s.u + ln / 2.0))
    bz = float(cfg["wall"].get("base_z", 0.0))
    if base_plates and bz > 0:
        for key, (lo, hi) in spans.items():
            lg = (legs or {}).get(key)
            th = float(getattr(lg, "theta", 0.0)) if lg is not None else 0.0
            u = (lo + hi) / 2.0
            x, y = lg.to_wall(u, 0.0) if lg is not None else (u, 0.0)
            out.append(Box(f"base plate {key or 'wall'}", np.array([x, y, bz / 2.0]), g.rotz(th)[:3, :3],
                           np.array([(hi - lo) / 2.0, W / 2.0, bz / 2.0])))
    return out


def station_boxes(cfg: Mapping, slots: Iterable, table: tuple[float, float] | None = None) -> list[Box]:
    """Boxes (station frame) of the stones in the given station holders (T_station_tcp = top centre, TCP z into the
    stone, stone length along TCP x; width incl. the ribs) and, if `table` = (x_max, y_max) is given, the table top
    ([pickup_station] table_z, 30 mm slab like robodk/build_station.py)."""
    out = []
    rib = float(cfg["brick"].get("rib_mm", 0.0))
    for s in slots:
        dims = cfg["half_brick"] if getattr(s, "kind", "full") == "half" else cfg["brick"]
        L, W, H = float(dims["length"]), float(dims["width"]), float(dims["height"])
        T = np.asarray(s.T_station_tcp, float)
        out.append(Box(f"station stone {s.id}", g.apply(T, [[0.0, 0.0, H / 2.0]])[0], T[:3, :3],
                       np.array([L / 2.0, W / 2.0 + rib, H / 2.0])))
    if table is not None:
        z = float(cfg["pickup_station"]["table_z"])
        out.append(Box("pick-up table", np.array([table[0] / 2.0, table[1] / 2.0, z - 15.0]), np.eye(3),
                       np.array([table[0] / 2.0, table[1] / 2.0, 15.0])))
    return out


class ArmChecker:
    """Collision test of UR5 configurations against a fixed set of boxes (parent frame), with the tool envelope of
    the config. `hits(q, T_parent_base)` -> collisions(...) list (empty = free)."""

    def __init__(self, cfg: Mapping, boxes: Sequence[Box], clearance_mm: float = CLEARANCE_MM):
        self.tool = tool_capsules(cfg)
        self.boxes = list(boxes)
        self.clearance = float(clearance_mm)

    def hits(self, q_rad: Sequence[float], T_parent_base: np.ndarray) -> list[tuple[str, str, float]]:
        if not self.boxes:
            return []
        caps = arm_capsules(q_rad, T_parent_base, self.tool)
        base = np.asarray(T_parent_base, float)[:3, 3]
        # UR5 reach 850 mm + tool ~200 mm: boxes farther than that cannot be touched (cost only)
        near = [b for b in self.boxes
                if float(np.linalg.norm(b.centre[:2] - base[:2]) - np.linalg.norm(b.half[:2])) < 1300.0]
        return collisions(caps, near, self.clearance)
