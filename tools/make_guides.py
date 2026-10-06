"""Floor guides of the first course and of the floor pick-up station: laser-cut MDF pieces (DXF for the Trotec,
800 x 600 mm sheets), 3D-printed socket blocks (STL for the Bambu Lab X1E + OpenSCAD source) and the floor map (PDF
on A3 at a fixed scale + a coordinate table) to measure out the room.

    py.exe tools/make_guides.py [--out targets/guides] [--no-map]

Design (config [guides], Samuel 2026-10-06: MDF strip + printed sockets, carpet tape, V-tabs for the existing board
plates, station on the floor in one row):
- Per wall leg a 4 mm MDF strip ([plates] mdf_t), [guides] strip_width wide, centred on the leg's wall line, from
  u = 0 to n0 x pitch. Per first-course stone two peg holes (the block's pegs) and four relief holes under the pins.
  The ARES-side edge (leg v = +strip_width/2, the face the existing wall plates were cut for) carries a V-tab at every
  inner stone joint u = pitch k (k = 1 .. n0-1); one tab in a plate's V-notch plus the edge fixes a plate completely.
- Pieces: a strip is cut [guides] cut_offset after a stone joint (the tab stays whole) into pieces of at most
  segment_max; consecutive pieces join with a dovetail on the centre line (tail on the downstream piece, socket on the
  upstream one, wall order A -> B -> C). The last corner_arm_stones stones of a leg and the first of the next form
  one L-piece across the corner (the 3 mm butt-joint gap filled), so the corner offset is cut, not measured.
- Socket block (printed, [wall] base_z - mdf_t high): four conical sockets through (two for a half stone), cone =
  pin cone of the stone CAD + [guides] socket_clearance; two pegs underneath into the strip's peg holes. The STL is
  oriented for printing (top face on the bed, pegs up: no supports; the cone walls overhang 24 deg from vertical).
- Station: one MDF piece set in the station frame ([pickup_station]: x along the row, y away from ARES), cut between
  the stacks into pieces <= segment_max; holes for every stack's block at row_y, closed windows for the plain S0/S1
  plates ([[targets]] parent "station") with [guides] window_clearance.
- Floor map: everything in the wall frame, shown in a map frame with the origin O at the OUTER START CORNER of the
  first leg's strip (x along the first leg, y towards ARES); pieces, course-0 stones, board plates, the station with
  its stacks, ARES at every stop and at the dock with its routes (tools/make_job.py build_l) and rotation circles.

Outputs (in --out): laser_sheet_<n>.dxf (layers CUT red / ENGRAVE blue, as tools/make_plates.py; sheet border on
layer SHEET green, not to be cut), laser_test.dxf (test pieces: one stone, a dovetail pair, an S-plate window),
laser_sheets.svg / .png (previews), socket_block_full.stl / socket_block_half.stl, socket_blocks.scad, floor_map.pdf,
floor_plan.md (coordinates, diagonals, part list, room size).
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO, REPO / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import make_plates as mp  # noqa: E402

from mauer import config, floor  # noqa: E402
from mauer.reference import Pose2D  # noqa: E402

Pt = tuple[float, float]
SHEET = "SHEET"
mp.ACI.setdefault(SHEET, 3)                 # green: sheet border, reference only (not cut, not engraved)
N_SEG = 72                                  # facets of a cone / peg in the STL


# ── parameters ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class GuideParams:
    pitch: float                # stone length = first-course pitch [mm]
    strip_w: float
    mdf_t: float
    block_h: float              # [wall] base_z - mdf_t
    block_l: float
    block_w: float
    half_l: float
    pin_along: float
    pin_across: float
    r_base: float
    r_tip: float
    pin_len: float
    clearance: float
    peg_d: float
    peg_len: float
    peg_hole_d: float
    peg_u: float
    half_peg_u: float
    relief_d: float
    vtab: tuple[float, float]
    dovetail: tuple[float, float, float]
    joint_c: float
    cut_offset: float
    arm: int
    seg_max: float
    sheet: tuple[float, float]
    sheet_margin: float
    window_c: float

    @property
    def h(self) -> float:
        return self.strip_w / 2.0


def params(cfg: dict) -> GuideParams:
    gd, pl = cfg["guides"], cfg["plates"]
    mdf = float(pl["mdf_t"])
    p = GuideParams(
        pitch=float(cfg["brick"]["length"]), strip_w=float(gd["strip_width"]), mdf_t=mdf,
        block_h=float(cfg["wall"]["base_z"]) - mdf, block_l=float(gd["block_length"]), block_w=float(gd["block_width"]),
        half_l=float(gd["half_block_length"]), pin_along=float(gd["pin_along"]), pin_across=float(gd["pin_across"]),
        r_base=float(gd["pin_r_base"]), r_tip=float(gd["pin_r_tip"]), pin_len=float(cfg["brick"]["pin_length"]),
        clearance=float(gd["socket_clearance"]), peg_d=float(gd["peg_d"]), peg_len=float(gd["peg_len"]),
        peg_hole_d=float(gd["peg_hole_d"]), peg_u=float(gd["peg_u"]), half_peg_u=float(gd["half_peg_u"]),
        relief_d=float(gd["relief_d"]), vtab=tuple(float(v) for v in gd["vtab"]),
        dovetail=tuple(float(v) for v in gd["dovetail"]), joint_c=float(gd["joint_clearance"]),
        cut_offset=float(gd["cut_offset"]), arm=int(gd["corner_arm_stones"]), seg_max=float(gd["segment_max"]),
        sheet=tuple(float(v) for v in gd["sheet"]), sheet_margin=float(gd["sheet_margin"]),
        window_c=float(gd["window_clearance"]))
    if p.block_h <= p.peg_len or p.block_h >= p.pin_len:
        raise ValueError(f"[guides]: block height {p.block_h} mm must lie between the peg ({p.peg_len}) and the pin "
                         f"length ({p.pin_len}) - the pin tips end in the strip's relief holes")
    if cone_r(p, p.block_h) * 2.0 >= p.relief_d:
        raise ValueError("[guides] relief_d smaller than the cone where it leaves the block")
    if abs(float(pl["block_length"]) - p.pitch) > 1e-9:
        raise ValueError("[plates] block_length must equal the stone pitch (the plates' notches sit on its joints)")
    return p


# ── socket blocks ─────────────────────────────────────────────────────────────
def cone_r(p: GuideParams, depth: float) -> float:
    """Socket radius at `depth` below the block top: the pin cone (r_base at the stone's underside, r_tip pin_len
    below) + clearance."""
    return p.r_base - (p.r_base - p.r_tip) * depth / p.pin_len + p.clearance


def block_size(p: GuideParams, kind: str) -> tuple[float, float]:
    return (p.block_l if kind == "full" else p.half_l), p.block_w


def pin_xy(p: GuideParams, kind: str) -> list[Pt]:
    """Pin centres in the block / stone frame (u along the stone, v across, origin = stone centre)."""
    a, b = p.pin_along / 2.0, p.pin_across / 2.0
    if kind == "full":
        return [(su * a, sv * b) for su in (-1.0, 1.0) for sv in (-1.0, 1.0)]
    return [(0.0, -b), (0.0, b)]


def peg_xy(p: GuideParams, kind: str) -> list[Pt]:
    d = p.peg_u if kind == "full" else p.half_peg_u
    return [(-d, 0.0), (d, 0.0)]


def _ray_hit(c: Pt, ang: float, rect: tuple[float, float, float, float]) -> Pt:
    x0, y0, x1, y1 = rect
    dx, dy = math.cos(ang), math.sin(ang)
    ts = []
    if dx > 1e-12:
        ts.append((x1 - c[0]) / dx)
    if dx < -1e-12:
        ts.append((x0 - c[0]) / dx)
    if dy > 1e-12:
        ts.append((y1 - c[1]) / dy)
    if dy < -1e-12:
        ts.append((y0 - c[1]) / dy)
    t = min(ts)
    return c[0] + t * dx, c[1] + t * dy


def _on_rect(q: Pt, rect: tuple[float, float, float, float], tol: float = 1e-9) -> bool:
    x0, y0, x1, y1 = rect
    inside = x0 - tol <= q[0] <= x1 + tol and y0 - tol <= q[1] <= y1 + tol
    edge = min(abs(q[0] - x0), abs(q[0] - x1), abs(q[1] - y0), abs(q[1] - y1)) <= tol
    return inside and edge


def _zip_ring(inner: list[int], outer: list[int], outer_ang: list[float], n: int) -> list[tuple[int, int, int]]:
    """Triangles (CCW seen from +z) between a hole ring (n vertices at angles 2 pi i / n) and the cell boundary
    (vertices sorted by angle in [0, 2 pi), outer[0] at angle 0)."""
    m = len(outer)
    tris = []
    i = j = 0
    while i < n or j < m:
        ai = 2.0 * math.pi * (i + 1) / n
        aj = outer_ang[j + 1] if j + 1 < m else 2.0 * math.pi
        if i < n and (j >= m or ai <= aj + 1e-12):
            tris.append((inner[i], outer[j % m], inner[(i + 1) % n]))
            i += 1
        else:
            tris.append((inner[i % n], outer[j], outer[(j + 1) % m]))
            j += 1
    return tris


def block_body(p: GuideParams, kind: str, n: int = N_SEG) -> tuple[np.ndarray, np.ndarray]:
    """Closed mesh (vertices, triangles) of the block body WITHOUT pegs, use orientation: bottom z = 0 on the MDF,
    top z = block_h, sockets through. Outward normals (CCW)."""
    L, W = block_size(p, kind)
    H = p.block_h
    pins = pin_xy(p, kind)
    r_top, r_bot = cone_r(p, 0.0), cone_r(p, H)
    xs, ys = sorted({q[0] for q in pins}), sorted({q[1] for q in pins})

    def splits(vals, lo, hi):
        return [lo] + [(a + b) / 2.0 for a, b in zip(vals, vals[1:])] + [hi]

    xb, yb = splits(xs, -L / 2, L / 2), splits(ys, -W / 2, W / 2)
    cells = [((xb[i], yb[j], xb[i + 1], yb[j + 1]), (xs[i], ys[j])) for i in range(len(xs)) for j in range(len(ys))]
    for rect, c in cells:
        if not (rect[0] < c[0] - r_top and c[0] + r_top < rect[2] and rect[1] < c[1] - r_top and c[1] + r_top < rect[3]):
            raise ValueError(f"[guides] {kind} block {L} x {W} mm too small for the sockets (r {r_top:.2f} mm)")
    angs = [2.0 * math.pi * k / n for k in range(n)]
    hits = [[_ray_hit(c, a, rect) for a in angs] for rect, c in cells]
    all_pts = [q for hs in hits for q in hs] + [(x, y) for rect, _ in cells for x in (rect[0], rect[2])
                                                for y in (rect[1], rect[3])]
    V: list[tuple[float, float, float]] = []
    idx: dict = {}

    def vid(x: float, y: float, z: float) -> int:
        k = (round(x, 7), round(y, 7), round(z, 7))
        if k not in idx:
            idx[k] = len(V)
            V.append((x, y, z))
        return idx[k]

    T: list[tuple[int, int, int]] = []
    for (rect, c) in cells:
        bnd = {}
        for q in all_pts:
            if _on_rect(q, rect):
                bnd[(round(q[0], 7), round(q[1], 7))] = q
        pts = sorted(bnd.values(), key=lambda q: math.atan2(q[1] - c[1], q[0] - c[0]) % (2.0 * math.pi))
        ang = [math.atan2(q[1] - c[1], q[0] - c[0]) % (2.0 * math.pi) for q in pts]
        if ang[0] > 1e-9:
            raise AssertionError("boundary must start at the ray hit of angle 0")
        for z, r, up in ((H, r_top, True), (0.0, r_bot, False)):
            inner = [vid(c[0] + r * math.cos(a), c[1] + r * math.sin(a), z) for a in angs]
            outer = [vid(q[0], q[1], z) for q in pts]
            for t in _zip_ring(inner, outer, ang, n):
                T.append(t if up else (t[0], t[2], t[1]))
        for k in range(n):                              # socket wall, normal towards the axis (into the hole)
            a, b = angs[k], angs[(k + 1) % n]
            bk, bk1 = vid(c[0] + r_bot * math.cos(a), c[1] + r_bot * math.sin(a), 0.0), \
                vid(c[0] + r_bot * math.cos(b), c[1] + r_bot * math.sin(b), 0.0)
            tk, tk1 = vid(c[0] + r_top * math.cos(a), c[1] + r_top * math.sin(a), H), \
                vid(c[0] + r_top * math.cos(b), c[1] + r_top * math.sin(b), H)
            T += [(bk, tk, tk1), (bk, tk1, bk1)]
    outer_pts = {}
    for q in all_pts:
        if abs(abs(q[0]) - L / 2) < 1e-9 or abs(abs(q[1]) - W / 2) < 1e-9:
            outer_pts[(round(q[0], 7), round(q[1], 7))] = q

    def perim(q: Pt) -> float:
        x, y = q
        if abs(y + W / 2) < 1e-9 and x < L / 2 - 1e-9:
            return x + L / 2
        if abs(x - L / 2) < 1e-9 and y < W / 2 - 1e-9:
            return L + y + W / 2
        if abs(y - W / 2) < 1e-9 and x > -L / 2 + 1e-9:
            return L + W + L / 2 - x
        return 2 * L + W + W / 2 - y

    ring = sorted(outer_pts.values(), key=perim)
    for a, b in zip(ring, ring[1:] + ring[:1]):         # outer walls, normal outwards
        ab, bb, bt, at = vid(a[0], a[1], 0.0), vid(b[0], b[1], 0.0), vid(b[0], b[1], H), vid(a[0], a[1], H)
        T += [(ab, bb, bt), (ab, bt, at)]
    return np.asarray(V, float), np.asarray(T, int)


def cylinder(c: Pt, r: float, z0: float, z1: float, n: int = N_SEG) -> tuple[np.ndarray, np.ndarray]:
    """Closed cylinder (outward normals)."""
    angs = [2.0 * math.pi * k / n for k in range(n)]
    V = [(c[0] + r * math.cos(a), c[1] + r * math.sin(a), z) for z in (z0, z1) for a in angs]
    V += [(c[0], c[1], z0), (c[0], c[1], z1)]
    cb, ct = 2 * n, 2 * n + 1
    T = []
    for k in range(n):
        k1 = (k + 1) % n
        T += [(k, k1, n + k1), (k, n + k1, n + k), (cb, k1, k), (ct, n + k, n + k1)]
    return np.asarray(V, float), np.asarray(T, int)


def block_mesh(p: GuideParams, kind: str, print_orientation: bool = True) -> list[tuple[np.ndarray, np.ndarray]]:
    """Body + pegs (separate closed shells; the pegs reach 0.5 mm into the body - the slicer unites them). Print
    orientation: turned 180 deg about x (top face on the bed, pegs up)."""
    shells = [block_body(p, kind)]
    for q in peg_xy(p, kind):
        shells.append(cylinder(q, p.peg_d / 2.0, -p.peg_len, 0.5))
    if print_orientation:
        out = []
        for V, T in shells:
            W = V.copy()
            W[:, 1], W[:, 2] = -V[:, 1], p.block_h - V[:, 2]
            out.append((W, T))
        shells = out
    return shells


def write_stl(path: Path, shells: list[tuple[np.ndarray, np.ndarray]], name: str) -> None:
    tris = [V[t] for V, T in shells for t in T]
    with open(path, "wb") as f:
        f.write(name.encode("ascii")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for tri in tris:
            nrm = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            ln = float(np.linalg.norm(nrm))
            nrm = nrm / ln if ln > 0 else nrm
            f.write(struct.pack("<12fH", *nrm, *tri[0], *tri[1], *tri[2], 0))


def write_scad(path: Path, p: GuideParams) -> None:
    def blk(kind: str) -> str:
        L, W = block_size(p, kind)
        pins = ", ".join(f"[{u:.3f}, {v:.3f}]" for u, v in pin_xy(p, kind))
        pegs = ", ".join(f"[{u:.3f}, {v:.3f}]" for u, v in peg_xy(p, kind))
        return (f"module socket_block_{kind}() {{\n"
                f"    difference() {{\n        translate([{-L / 2:.3f}, {-W / 2:.3f}, 0]) cube([{L:.3f}, {W:.3f}, H]);\n"
                f"        for (q = [{pins}]) translate([q[0], q[1], -0.01]) cylinder(h = H + 0.02, r1 = R_BOT, "
                f"r2 = R_TOP);\n    }}\n"
                f"    for (q = [{pegs}]) translate([q[0], q[1], -PEG_LEN]) cylinder(h = PEG_LEN + 0.5, d = PEG_D);\n}}\n")
    path.write_text(
        "// Socket blocks of the ARES_Mauer floor guides - generated by tools/make_guides.py from config/station.toml\n"
        "// [guides] (edit the config and regenerate rather than this file). Use orientation: bottom on the MDF strip,\n"
        "// pegs down; PRINT upside down (top face on the bed, pegs up). Units mm.\n"
        f"$fn = 96;\nH = {p.block_h:.3f};          // [wall] base_z - [plates] mdf_t\n"
        f"R_TOP = {cone_r(p, 0.0):.3f};      // pin cone at the stone's underside + clearance\n"
        f"R_BOT = {cone_r(p, p.block_h):.3f};      // the same cone at the block bottom\n"
        f"PEG_D = {p.peg_d:.3f};\nPEG_LEN = {p.peg_len:.3f};\n\n" + blk("full") + "\n" + blk("half") +
        "\nsocket_block_full();\ntranslate([0, 140, 0]) socket_block_half();\n", encoding="ascii")


# ── laser-cut pieces ──────────────────────────────────────────────────────────
@dataclass
class Piece:
    """A flat MDF piece in its local frame (a leg frame, or the station frame), placed in the wall frame by pose."""
    name: str
    pose: tuple[float, float, float]            # local frame in the wall frame (x, y, theta [rad])
    outline: list[Pt]                           # CCW, with tabs / dovetails
    holes: list[tuple[Pt, float]] = field(default_factory=list)      # (centre, diameter)
    windows: list[list[Pt]] = field(default_factory=list)
    blocks: list[tuple[str, Pt, float, str]] = field(default_factory=list)   # (label, centre, angle, kind)
    texts: list[tuple[Pt, float, str]] = field(default_factory=list)
    notes: str = ""

    def to_wall(self, q: Pt) -> Pt:
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        return x + c * q[0] - s * q[1], y + s * q[0] + c * q[1]

    def bbox(self) -> tuple[float, float, float, float]:
        a = np.asarray(self.outline)
        return float(a[:, 0].min()), float(a[:, 1].min()), float(a[:, 0].max()), float(a[:, 1].max())


def feature(p: GuideParams, kind: str, s: float) -> list[tuple[float, float]]:
    """(along, outward) points of an edge feature centred at s: V-tab, dovetail tail (male, outward) or socket
    (female, inward, + joint clearance)."""
    if kind == "tab":
        w, hh = p.vtab
        return [(s - w / 2, 0.0), (s, hh), (s + w / 2, 0.0)]
    w, d, f = p.dovetail
    if kind == "tail":
        return [(s - w / 2, 0.0), (s - w / 2 - f, d), (s + w / 2 + f, d), (s + w / 2, 0.0)]
    if kind == "socket":
        c = p.joint_c
        return [(s - w / 2 - c, 0.0), (s - w / 2 - f - c, -(d + c)), (s + w / 2 + f + c, -(d + c)), (s + w / 2 + c, 0.0)]
    raise ValueError(kind)


def decorate(p: GuideParams, poly: list[Pt], feats: dict[int, list[tuple[float, str]]]) -> list[Pt]:
    """Outline of a CCW polygon with features on its edges ({edge index: [(distance from the edge start, kind)]})."""
    out: list[Pt] = []
    for e, (a, b) in enumerate(zip(poly, poly[1:] + poly[:1])):
        a_, b_ = np.asarray(a, float), np.asarray(b, float)
        L = float(np.linalg.norm(b_ - a_))
        d = (b_ - a_) / L
        nrm = np.array([d[1], -d[0]])                    # outward for CCW
        out.append(tuple(a_))
        for s, kind in sorted(feats.get(e, [])):
            for al, o in feature(p, kind, s):
                if not 0.0 < al < L:
                    raise ValueError(f"edge feature {kind} at {s:.1f} mm does not fit the edge ({L:.1f} mm)")
                out.append(tuple(a_ + d * al + nrm * o))
    return out


def stone_holes(p: GuideParams, kind: str, centre: Pt, ang: float) -> list[tuple[Pt, float]]:
    c, s = math.cos(ang), math.sin(ang)

    def at(q: Pt) -> Pt:
        return centre[0] + c * q[0] - s * q[1], centre[1] + s * q[0] + c * q[1]
    return [(at(q), p.relief_d) for q in pin_xy(p, kind)] + [(at(q), p.peg_hole_d) for q in peg_xy(p, kind)]


def _split(items: list[int], kmax: int) -> list[list[int]]:
    if not items:
        return []
    n = math.ceil(len(items) / kmax)
    base, extra = divmod(len(items), n)
    out, i = [], 0
    for k in range(n):
        m = base + (1 if k < extra else 0)
        out.append(items[i:i + m])
        i += m
    return out


def corner_geometry(a, b, h: float, L: float) -> float:
    """v of leg b's origin in leg a's frame (a's length L) for a butt corner turning towards a's +v (ARES side, the
    inside of the C/L): b starts at u = L - h beside a's ARES-side face. ValueError for any other corner."""
    u, v = a.from_wall(b.x, b.y)
    dth = (b.theta - a.theta + math.pi) % (2 * math.pi) - math.pi
    if abs(dth - math.pi / 2) > 1e-6 or abs(u - (L - h)) > 1e-6 or v < h - 1e-6 or v > h + 20.0:
        raise ValueError(f"corner {a.name}->{b.name}: only butt corners turning towards the ARES side are supported "
                         f"(+90 deg, {b.name} starting at u = L - {h:g} beside {a.name}'s ARES-side face; got "
                         f"{math.degrees(dth):.1f} deg, u {u:.1f}, v {v:.1f} mm)")
    return v


def leg_pieces(cfg: dict, p: GuideParams, legs: list, sites: list) -> list[Piece]:
    """The MDF pieces of all legs in wall order (see the module docstring)."""
    h, P, off = p.h, p.pitch, p.cut_offset
    kmax = int((p.seg_max - p.dovetail[1] - off) // P)
    plate_at = {}
    for s in sites:
        plate_at[(s.leg, s.k)] = s.board
    pieces: list[Piece] = []
    nl = len(legs)
    for i, lg in enumerate(legs):
        L = lg.n0 * P
        first = p.arm if i > 0 else 0
        last = p.arm if i < nl - 1 else 0
        if first + last > lg.n0:
            raise ValueError(f"leg {lg.name}: {lg.n0} stones are fewer than the corner arms ({first} + {last})")
        for grp in _split(list(range(first, lg.n0 - last)), kmax):
            a, b = grp[0], grp[-1] + 1
            u0 = 0.0 if a == 0 else a * P + off
            u1 = L if b == lg.n0 else b * P + off
            poly = [(u0, -h), (u1, -h), (u1, h), (u0, h)]
            feats: dict[int, list] = {2: [(u1 - k * P, "tab") for k in range(1, lg.n0) if u0 < k * P < u1]}
            if not (i == 0 and a == 0):
                feats[3] = [(h, "tail")]
            if not (i == nl - 1 and b == lg.n0):
                feats[1] = [(h, "socket")]
            pc = Piece(f"{lg.name}{a}-{lg.name}{b - 1}", (lg.x, lg.y, lg.theta), decorate(p, poly, feats))
            _stones(p, pc, lg.name, range(a, b), lambda q: q, 0.0, plate_at)
            pieces.append(pc)
        if i < nl - 1:
            nb = legs[i + 1]
            v0 = corner_geometry(lg, nb, h, L)
            u_cut = (lg.n0 - p.arm) * P + off
            arm_end = nb.n0 * P if (i + 1 == nl - 1 and p.arm == nb.n0) else p.arm * P + off
            v_end = v0 + arm_end
            poly = [(u_cut, -h), (L, -h), (L, v_end), (L - 2 * h, v_end), (L - 2 * h, h), (u_cut, h)]
            feats = {5: [(h, "tail")],
                     4: [((L - 2 * h) - k * P, "tab") for k in range(1, lg.n0)
                         if u_cut < k * P < L - 2 * h - p.vtab[0]],
                     3: [(v_end - (v0 + k * P), "tab") for k in range(1, nb.n0) if 0 < k * P < arm_end - off / 2]}
            if not (i + 1 == nl - 1 and arm_end >= nb.n0 * P):
                feats[2] = [(h, "socket")]
            pc = Piece(f"{lg.name}{lg.n0 - p.arm}-{nb.name}{p.arm - 1}", (lg.x, lg.y, lg.theta), decorate(p, poly, feats),
                       notes=f"corner {lg.name}/{nb.name}")
            _stones(p, pc, lg.name, range(lg.n0 - p.arm, lg.n0), lambda q: q, 0.0, plate_at)

            def to_a(q: Pt, L=L, v0=v0) -> Pt:                 # leg nb frame -> leg lg frame
                return L - h - q[1], v0 + q[0]
            _stones(p, pc, nb.name, range(0, p.arm), to_a, math.pi / 2, plate_at)
            pieces.append(pc)
    return pieces


def _stones(p: GuideParams, pc: Piece, leg: str, ks, to_local, ang: float, plate_at: dict) -> None:
    """Holes, engraved block footprints and labels of first-course stones ks of `leg` on piece pc (to_local maps the
    leg's (u, v) into the piece frame, ang = the leg's rotation in the piece frame)."""
    for k in ks:
        c = to_local((k * p.pitch + p.pitch / 2.0, 0.0))
        pc.holes += stone_holes(p, "full", c, ang)
        pc.blocks.append((f"{leg}{k}", c, ang, "full"))
        if (leg, k) in plate_at:
            q = to_local((k * p.pitch + p.pitch / 2.0, p.h - 9.0))
            pc.texts.append((q, 5.0, f"plate {plate_at[(leg, k)]} ->"))


def station_pieces(cfg: dict, p: GuideParams, plates: list[tuple[str, Pt, tuple[float, float]]]) -> list[Piece]:
    """MDF pieces of the floor station in the station frame (see the module docstring); plates = (name, plate centre,
    plate size) of the station boards."""
    ps = cfg["pickup_station"]
    T = cfg["pickup_station"]["xyz_in_wall"]
    th = math.radians(float(ps["rpy_in_wall_deg"][2]))
    X, Y = (float(v) for v in ps["table_size"])
    ry = float(ps["row_y"])
    hb = cfg.get("half_brick", cfg["brick"])
    stacks = [(f"s{i:02d}", float(x), "full", float(cfg["brick"]["length"])) for i, (x, _) in enumerate(ps["slots_xy"])]
    stacks += [(f"h{i:02d}", float(x), "half", float(hb["length"])) for i, (x, _) in enumerate(ps.get("half_slots_xy", []))]
    for _, (x, y) in enumerate(list(ps["slots_xy"]) + list(ps.get("half_slots_xy", []))):
        if abs(float(y) - ry) > 1e-6:
            raise ValueError("[pickup_station]: every stack must stand on row_y (one row)")
    stacks.sort(key=lambda s: s[1])
    # cut candidates: middle of the gaps between neighbouring stacks, away from the blocks
    cands = []
    for a, b in zip(stacks, stacks[1:]):
        lo = a[1] + block_size(p, a[2])[0] / 2.0
        hi = b[1] - block_size(p, b[2])[0] / 2.0
        if hi - lo > 2 * (p.dovetail[1] + 5.0):
            cands.append((a[1] + a[3] / 2.0 + b[1] - b[3] / 2.0) / 2.0)
    cuts, start = [], 0.0
    while X - start > p.seg_max - p.dovetail[1]:
        ok = [c for c in cands if start + 50.0 < c <= start + p.seg_max - p.dovetail[1]]
        if not ok:
            raise ValueError("[pickup_station]: no cut between the stacks keeps the pieces <= segment_max")
        cuts.append(max(ok))
        start = cuts[-1]
    win = []
    for name, c, (w, hh) in plates:
        hw, hh2 = w / 2 + p.window_c, hh / 2 + p.window_c
        win.append((name, c, [(c[0] - hw, c[1] - hh2), (c[0] + hw, c[1] - hh2), (c[0] + hw, c[1] + hh2),
                              (c[0] - hw, c[1] + hh2)]))
    joint_ys = [ry] + sorted({round(c[1], 3) for _, c, _ in plates})
    xs = [0.0] + cuts + [X]
    out = []
    for i, (x0, x1) in enumerate(zip(xs, xs[1:])):
        poly = [(x0, 0.0), (x1, 0.0), (x1, Y), (x0, Y)]
        feats: dict[int, list] = {}
        if i < len(xs) - 2:
            feats[1] = [(y, "socket") for y in joint_ys]
        if i > 0:
            feats[3] = [(Y - y, "tail") for y in joint_ys]
        pc = Piece(f"station{i + 1}", (float(T[0]), float(T[1]), th), decorate(p, poly, feats), notes="station")
        for name, x, kind, _ in stacks:
            if x0 < x < x1:
                pc.holes += stone_holes(p, kind, (x, ry), 0.0)
                pc.blocks.append((name, (x, ry), 0.0, kind))
        for name, c, w in win:
            if x0 < c[0] < x1:
                pc.windows.append(w)
                pc.texts.append(((c[0] - 12.0, c[1] - 3.0), 8.0, name))
        out.append(pc)
    return out


def test_pieces(p: GuideParams, plate_wh: tuple[float, float]) -> list[Piece]:
    """Test cut: one stone with a V-tab and a dovetail socket, and a mating piece with the tail and an S-plate
    window (block pegs, plate notch, dovetail and window fit)."""
    h, P = p.h, p.pitch
    t1 = Piece("test1", (0.0, 0.0, 0.0), decorate(p, [(0.0, -h), (P + p.cut_offset, -h), (P + p.cut_offset, h),
                                                          (0.0, h)], {1: [(h, "socket")], 2: [(p.cut_offset, "tab")]}))
    t1.holes += stone_holes(p, "full", (P / 2.0, 0.0), 0.0)
    t1.blocks.append(("test", (P / 2.0, 0.0), 0.0, "full"))
    x0 = P + p.cut_offset
    w, hh = plate_wh[0] + 2 * (p.window_c + 15.0), plate_wh[1] + 2 * (p.window_c + 14.0)
    t2 = Piece("test2", (0.0, 0.0, 0.0), decorate(p, [(x0, -h), (x0 + w, -h), (x0 + w, -h + hh), (x0, -h + hh)],
                                                    {3: [(hh - h, "tail")]}))
    c = (x0 + w / 2.0, -h + hh / 2.0)
    hw, hh2 = plate_wh[0] / 2 + p.window_c, plate_wh[1] / 2 + p.window_c
    t2.windows.append([(c[0] - hw, c[1] - hh2), (c[0] + hw, c[1] - hh2), (c[0] + hw, c[1] + hh2), (c[0] - hw, c[1] + hh2)])
    return [t1, t2]


# ── sheets ────────────────────────────────────────────────────────────────────
def _local_shape(pc: Piece, rot: bool) -> tuple[list[Pt], float, float, float, float]:
    pts = pc.outline if not rot else [(-y, x) for x, y in pc.outline]
    a = np.asarray(pts)
    return pts, float(a[:, 0].min()), float(a[:, 1].min()), float(a[:, 0].max()), float(a[:, 1].max())


def nest(pieces: list[Piece], p: GuideParams, gap: float = 6.0) -> list[list[tuple[Piece, bool, Pt]]]:
    """Shelf packing on sheets of p.sheet minus margins: [(piece, rotated 90 deg, offset of its local frame)]."""
    W, H = p.sheet[0] - 2 * p.sheet_margin, p.sheet[1] - 2 * p.sheet_margin
    items = []
    for pc in pieces:
        opts = []
        for rot in (False, True):
            _, x0, y0, x1, y1 = _local_shape(pc, rot)
            if x1 - x0 <= W and y1 - y0 <= H:
                opts.append((y1 - y0, x1 - x0, rot, x0, y0))
        if not opts:
            raise ValueError(f"piece {pc.name} does not fit the {p.sheet[0]:.0f} x {p.sheet[1]:.0f} mm sheet")
        items.append((pc, sorted(opts)))
    items.sort(key=lambda it: -it[1][0][0])
    sheets: list[dict] = []
    for pc, opts in items:
        placed = False
        for sh in sheets:
            for hgt, wid, rot, x0, y0 in opts:
                for shelf in sh["shelves"]:
                    if hgt <= shelf[1] + 1e-9 and shelf[2] + wid <= W + 1e-9:
                        sh["items"].append((pc, rot, (p.sheet_margin + shelf[2] - x0, p.sheet_margin + shelf[0] - y0)))
                        shelf[2] += wid + gap
                        placed = True
                        break
                if placed:
                    break
                if sh["y"] + hgt <= H + 1e-9:
                    sh["shelves"].append([sh["y"], hgt, wid + gap])
                    sh["items"].append((pc, rot, (p.sheet_margin - x0, p.sheet_margin + sh["y"] - y0)))
                    sh["y"] += hgt + gap
                    placed = True
                    break
            if placed:
                break
        if not placed:
            hgt, wid, rot, x0, y0 = opts[0]
            sheets.append({"shelves": [[0.0, hgt, wid + gap]], "y": hgt + gap,
                           "items": [(pc, rot, (p.sheet_margin - x0, p.sheet_margin - y0))]})
    return [sh["items"] for sh in sheets]


def _place(q: Pt, rot: bool, off: Pt) -> Pt:
    x, y = (q if not rot else (-q[1], q[0]))
    return x + off[0], y + off[1]


def sheet_dxf(items: list[tuple[Piece, bool, Pt]], p: GuideParams, label: str) -> mp.Dxf:
    d = mp.Dxf()
    d.poly(SHEET, [(0.0, 0.0), (p.sheet[0], 0.0), (p.sheet[0], p.sheet[1]), (0.0, p.sheet[1])])
    d.text(SHEET, (2.0, p.sheet[1] - 6.0), 4.0, label)
    for pc, rot, off in items:
        d.poly(mp.CUT, [_place(q, rot, off) for q in pc.outline])
        for c, dia in pc.holes:
            d.circle(mp.CUT, _place(c, rot, off), dia / 2.0)
        for w in pc.windows:
            d.poly(mp.CUT, [_place(q, rot, off) for q in w])
        for name, c, ang, kind in pc.blocks:
            L, W = block_size(p, kind)
            ca, sa = math.cos(ang), math.sin(ang)
            corners = [(c[0] + ca * u - sa * v, c[1] + sa * u + ca * v) for u, v in
                       ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2))]
            d.poly(mp.ENGRAVE, [_place(q, rot, off) for q in corners])
            d.text(mp.ENGRAVE, _place((c[0] - 6.0, c[1] - 2.5), rot, off), 5.0, name)
        for q, hgt, s in pc.texts:
            d.text(mp.ENGRAVE, _place(q, rot, off), hgt, s)
        x0, y0, x1, y1 = pc.bbox()
        d.text(mp.ENGRAVE, _place((x0 + 6.0, y0 + 4.0), rot, off), 6.0, pc.name + (f" ({pc.notes})" if pc.notes else ""))
    return d


def write_svg(sheets: list[list[tuple[Piece, bool, Pt]]], p: GuideParams, path: Path) -> None:
    W, H = p.sheet
    gap = 20.0
    total_h = len(sheets) * (H + gap)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}mm" height="{total_h:.0f}mm" '
             f'viewBox="0 0 {W:.0f} {total_h:.0f}"><g fill="none" stroke-width="0.6">']
    for i, items in enumerate(sheets):
        oy = i * (H + gap)

        def tr(q: Pt) -> str:
            return f"{q[0]:.2f},{oy + H - q[1]:.2f}"
        parts.append(f'<rect x="0" y="{oy:.1f}" width="{W:.0f}" height="{H:.0f}" stroke="green"/>')
        parts.append(f'<text x="4" y="{oy + 12:.1f}" font-size="10" fill="green">sheet {i + 1}</text>')
        for pc, rot, off in items:
            parts.append('<polygon stroke="red" points="' + " ".join(tr(_place(q, rot, off)) for q in pc.outline) + '"/>')
            for c, dia in pc.holes:
                x, y = _place(c, rot, off)
                parts.append(f'<circle cx="{x:.2f}" cy="{oy + H - y:.2f}" r="{dia / 2:.2f}" stroke="red"/>')
            for w in pc.windows:
                parts.append('<polygon stroke="red" points="' + " ".join(tr(_place(q, rot, off)) for q in w) + '"/>')
            x0, y0, _, _ = pc.bbox()
            x, y = _place((x0 + 6.0, y0 + 6.0), rot, off)
            parts.append(f'<text x="{x:.1f}" y="{oy + H - y:.1f}" font-size="9" fill="blue">{pc.name}</text>')
    parts.append("</g></svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def write_png(sheets: list[list[tuple[Piece, bool, Pt]]], p: GuideParams, path: Path, px_per_mm: float = 1.5) -> None:
    """Preview of the sheets (cut red, engraved blue, sheet border green), stacked vertically."""
    import cv2
    W, H = p.sheet
    gap = 20.0
    img = np.full((int(len(sheets) * (H + gap) * px_per_mm), int(W * px_per_mm), 3), 255, np.uint8)
    for i, items in enumerate(sheets):
        oy = i * (H + gap)

        def P(q: Pt) -> tuple[int, int]:
            return int(round(q[0] * px_per_mm)), int(round((oy + H - q[1]) * px_per_mm))
        cv2.rectangle(img, P((0.0, H)), P((W, 0.0)), (0, 160, 0), 1)
        cv2.putText(img, f"sheet {i + 1}", P((4.0, H - 14.0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 160, 0), 1, cv2.LINE_AA)
        for pc, rot, off in items:
            for poly_ in [pc.outline] + pc.windows:
                q = np.array([P(_place(x, rot, off)) for x in poly_], np.int32)
                cv2.polylines(img, [q], True, (0, 0, 220), 1, cv2.LINE_AA)
            for c, dia in pc.holes:
                cv2.circle(img, P(_place(c, rot, off)), max(1, int(dia / 2 * px_per_mm)), (0, 0, 220), 1, cv2.LINE_AA)
            for name, c, ang, kind in pc.blocks:
                L, Wb = block_size(p, kind)
                ca, sa = math.cos(ang), math.sin(ang)
                q = np.array([P(_place((c[0] + ca * u - sa * v, c[1] + sa * u + ca * v), rot, off))
                              for u, v in ((-L / 2, -Wb / 2), (L / 2, -Wb / 2), (L / 2, Wb / 2), (-L / 2, Wb / 2))],
                             np.int32)
                cv2.polylines(img, [q], True, (200, 80, 0), 1, cv2.LINE_AA)
            x0, y0, _, _ = pc.bbox()
            cv2.putText(img, pc.name, P(_place((x0 + 8.0, y0 + 8.0), rot, off)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 80, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


# ── floor map ─────────────────────────────────────────────────────────────────
@dataclass
class FloorModel:
    pieces: list[Piece]                       # leg + station pieces
    stones: list[tuple[str, list[Pt]]]        # course-0 stones and station stacks (wall frame polygons)
    plates: list[tuple[str, list[Pt]]]        # W plates and S plates (wall frame)
    ares: list[tuple[str, Pose2D]]            # stops + dock
    routes: list[list[Pose2D]]
    rot_circles: list[tuple[Pt, float]]
    origin: Pt                                # O in the wall frame
    x_axis: float                             # map x axis heading in the wall frame [rad]
    ares_shape: object = None
    marks: list[tuple[str, Pt]] = field(default_factory=list)   # marking points (wall frame)
    warnings: list[str] = field(default_factory=list)

    def to_map(self, q: Pt) -> Pt:
        c, s = math.cos(-self.x_axis), math.sin(-self.x_axis)
        x, y = q[0] - self.origin[0], q[1] - self.origin[1]
        return c * x - s * y, s * x + c * y

    def to_wall_map(self, q_map: Pt) -> Pt:
        c, s = math.cos(self.x_axis), math.sin(self.x_axis)
        return self.origin[0] + c * q_map[0] - s * q_map[1], self.origin[1] + s * q_map[0] + c * q_map[1]

    def footprints(self) -> list[list[Pt]]:
        out = [self.ares_shape.footprint(a) for _, a in self.ares]
        for r in self.routes:
            out += [self.ares_shape.footprint(w) for w in r]
        return out

    def bbox(self, with_ares: bool) -> tuple[float, float, float, float]:
        pts = [self.to_map(pc.to_wall(q)) for pc in self.pieces for q in pc.outline]
        pts += [self.to_map(q) for _, poly in self.stones + self.plates for q in poly]
        if with_ares:
            pts += [self.to_map(q) for fp in self.footprints() for q in fp]
            for c, r in self.rot_circles:
                m = self.to_map(c)
                pts += [(m[0] - r, m[1] - r), (m[0] + r, m[1] + r)]
        a = np.asarray(pts)
        return float(a[:, 0].min()), float(a[:, 1].min()), float(a[:, 0].max()), float(a[:, 1].max())


def floor_model(cfg: dict, p: GuideParams, legs: list, sites: list, st_pieces: list[Piece], leg_pcs: list[Piece],
                job=None) -> FloorModel:
    from mauer.reference import placements
    lg0 = legs[0]
    O = lg0.to_wall(0.0, -p.h)
    m = FloorModel(leg_pcs + st_pieces, [], [], [], [], [], O, lg0.theta, floor.AresShape.from_config(cfg))
    b, hb = cfg["brick"], cfg.get("half_brick", cfg["brick"])
    wr = float(b["width"]) / 2.0 + float(b.get("rib_mm", 0.0))
    for lg in legs:
        for k in range(lg.n0):
            u0, u1 = k * p.pitch, (k + 1) * p.pitch
            m.stones.append((f"{lg.name}{k}", [lg.to_wall(u, v) for u, v in ((u0, -wr), (u1, -wr), (u1, wr), (u0, wr))]))
        m.marks += [(f"{lg.name} start, outer corner", lg.to_wall(0.0, -p.h)),
                    (f"{lg.name} start, ARES-side corner", lg.to_wall(0.0, p.h)),
                    (f"{lg.name} end, outer corner", lg.to_wall(lg.n0 * p.pitch, -p.h)),
                    (f"{lg.name} end, ARES-side corner", lg.to_wall(lg.n0 * p.pitch, p.h))]
    for s in sites:
        m.plates.append((s.board, list(s.plate)))
    ps = cfg["pickup_station"]
    st = Piece("station", (float(ps["xyz_in_wall"][0]), float(ps["xyz_in_wall"][1]),
                           math.radians(float(ps["rpy_in_wall_deg"][2]))), [])
    ry = float(ps["row_y"])
    for name, xys, length in (("s", ps["slots_xy"], float(b["length"])), ("h", ps.get("half_slots_xy", []),
                                                                         float(hb["length"]))):
        for i, (x, _) in enumerate(xys):
            m.stones.append((f"{name}{i:02d}", [st.to_wall(q) for q in ((x - length / 2, ry - wr), (x + length / 2, ry - wr),
                                                                       (x + length / 2, ry + wr), (x - length / 2, ry + wr))]))
    X, Y = (float(v) for v in ps["table_size"])
    m.marks += [("station, front-left corner (ARES side)", st.to_wall((0.0, 0.0))),
                ("station, front-right corner (ARES side)", st.to_wall((X, 0.0))),
                ("station, back-right corner", st.to_wall((X, Y))),
                ("station, back-left corner", st.to_wall((0.0, Y)))]
    for pl in placements(cfg):
        if pl.parent == "station":
            for name, c, (w, hh) in _station_plates(cfg):
                if name == pl.name:
                    m.plates.append((name, [st.to_wall(q) for q in ((c[0] - w / 2, c[1] - hh / 2), (c[0] + w / 2, c[1] - hh / 2),
                                                                    (c[0] + w / 2, c[1] + hh / 2), (c[0] - w / 2, c[1] + hh / 2))]))
    dock = Pose2D.from_T(_T(st.pose) @ _pose_T(ps["ares_xyz"], ps["ares_rpy_deg"]))
    m.ares.append(("dock", dock))
    m.marks.append(("ARES base_link at the dock", (dock.x_mm, dock.y_mm)))
    if job is not None:
        for s in job.stops:
            m.ares.append((f"stop {s.index}", s.ares))
            for r in (s.route, s.route_to_station, s.route_from_station):
                if r:
                    m.routes.append(list(r))
        for r in m.routes:
            for a, bb in zip(r, r[1:]):
                if math.hypot(a.x_mm - bb.x_mm, a.y_mm - bb.y_mm) < 1.0 and abs(a.theta_rad - bb.theta_rad) > 1e-4:
                    m.rot_circles.append(((a.x_mm, a.y_mm), m.ares_shape.radius))
    else:
        m.warnings.append("no job: ARES stops and routes not drawn")
    return m


def _T(pose: tuple[float, float, float]) -> np.ndarray:
    from mauer import geometry as g
    return g.pose_xyz_rpy([pose[0], pose[1], 0.0], [0.0, 0.0, math.degrees(pose[2])])


def _pose_T(xyz, rpy) -> np.ndarray:
    from mauer import geometry as g
    return g.pose_xyz_rpy(xyz, rpy)


def _station_plates(cfg: dict) -> list[tuple[str, Pt, tuple[float, float]]]:
    """(name, plate centre, plate size) of the plain station plates in the station frame: the board lies face up
    (rpy 180, 0, 0) on its plain plate, plate centre = board centre - 4 mm in station y (tools/make_plates.py
    plain_plate: the label strip)."""
    from mauer.vision.targets import board_specs
    specs = board_specs(cfg)
    out = []
    for t in cfg.get("targets", []):
        if t.get("parent") != "station":
            continue
        if [round(float(v), 6) for v in t.get("rpy_deg", (0, 0, 0))] != [180.0, 0.0, 0.0]:
            raise ValueError(f"station board {t['name']}: only face-up boards (rpy 180, 0, 0) are supported")
        sp = specs[str(t["name"])]
        pl = mp.plain_plate(cfg, sp)
        bw, bh = sp.size_mm
        c = (float(t["xyz"][0]) + bw / 2.0, float(t["xyz"][1]) - bh / 2.0 - 4.0)
        out.append((str(t["name"]), c, (pl.w, pl.h)))
    return out


def _scale(bb: tuple[float, float, float, float], area_mm: tuple[float, float]) -> int:
    w, h = bb[2] - bb[0], bb[3] - bb[1]
    for s in (10, 20, 25, 30, 40, 50):
        if w / s <= area_mm[0] and h / s <= area_mm[1]:
            return s
    return 100


def write_map_pdf(path: Path, m: FloorModel, cfg: dict, p: GuideParams, info: dict) -> int:
    """Page 1: top view at a fixed scale on A3 (print at 100 %); page 2: marking points, diagonals, parts."""
    from reportlab.lib.pagesizes import A3, landscape
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    pw, ph = landscape(A3)
    margin = 12 * mm
    area = ((pw - 2 * margin) / mm, (ph - 2 * margin - 22 * mm) / mm)
    bb = m.bbox(with_ares=True)
    sc = _scale((bb[0] - 100, bb[1] - 100, bb[2] + 100, bb[3] + 100), area)
    cx, cy = (bb[0] + bb[2]) / 2.0, (bb[1] + bb[3]) / 2.0
    ox, oy = pw / 2.0, margin + 22 * mm + (ph - 2 * margin - 22 * mm) / 2.0

    def P(q_wall: Pt) -> tuple[float, float]:
        x, y = m.to_map(q_wall)
        return ox + (x - cx) / sc * mm, oy + (y - cy) / sc * mm

    def Pm(q_map: Pt) -> tuple[float, float]:
        return ox + (q_map[0] - cx) / sc * mm, oy + (q_map[1] - cy) / sc * mm

    c = canvas.Canvas(str(path), pagesize=(pw, ph))
    c.setTitle("ARES_Mauer floor map")
    # grid every 500 mm in map coordinates
    c.setStrokeColorRGB(0.88, 0.88, 0.88)
    c.setLineWidth(0.3)
    g0x, g0y = math.floor((bb[0] - 100) / 500) * 500, math.floor((bb[1] - 100) / 500) * 500
    gx = g0x
    while gx <= bb[2] + 100:
        c.line(*Pm((gx, bb[1] - 100)), *Pm((gx, bb[3] + 100)))
        c.setFillColorRGB(0.55, 0.55, 0.55)
        c.setFont("Helvetica", 5)
        c.drawString(*Pm((gx + 10, bb[1] - 90)), f"{gx:.0f}")
        gx += 500
    gy = g0y
    while gy <= bb[3] + 100:
        c.line(*Pm((bb[0] - 100, gy)), *Pm((bb[2] + 100, gy)))
        c.drawString(*Pm((bb[0] - 95, gy + 10)), f"{gy:.0f}")
        gy += 500

    def poly(pts, stroke=(0, 0, 0), fill=None, lw=0.6, dash=None):
        path_ = c.beginPath()
        q = [P(x) for x in pts]
        path_.moveTo(*q[0])
        for x in q[1:]:
            path_.lineTo(*x)
        path_.close()
        c.setStrokeColorRGB(*stroke)
        c.setLineWidth(lw)
        c.setDash(*(dash or ()))
        if fill:
            c.setFillColorRGB(*fill)
        c.drawPath(path_, stroke=1, fill=1 if fill else 0)
        c.setDash()

    # ARES footprints along the routes, rotation circles, stops, dock
    for fp in m.footprints():
        poly(fp, stroke=(0.75, 0.82, 0.95), lw=0.3)
    c.setStrokeColorRGB(0.6, 0.7, 0.95)
    for ctr, r in m.rot_circles:
        x, y = P(ctr)
        c.setDash(2, 2)
        c.circle(x, y, r / sc * mm, stroke=1, fill=0)
        c.setDash()
    for name, a in m.ares:
        poly(m.ares_shape.footprint(a), stroke=(0.1, 0.3, 0.8), lw=0.8, dash=(3, 2))
        x, y = P((a.x_mm, a.y_mm))
        c.setFillColorRGB(0.1, 0.3, 0.8)
        c.circle(x, y, 1.2, stroke=0, fill=1)
        c.setFont("Helvetica", 6)
        c.drawString(x + 3, y + 3, name)
    for r in m.routes:
        c.setStrokeColorRGB(0.1, 0.3, 0.8)
        c.setLineWidth(0.4)
        for a, b in zip(r, r[1:]):
            c.line(*P((a.x_mm, a.y_mm)), *P((b.x_mm, b.y_mm)))
    # guides, stones, plates
    for pc in m.pieces:
        poly([pc.to_wall(q) for q in pc.outline], stroke=(0.45, 0.25, 0.05), fill=(0.93, 0.85, 0.70), lw=0.5)
        for w in pc.windows:
            poly([pc.to_wall(q) for q in w], stroke=(0.45, 0.25, 0.05), fill=(1, 1, 1), lw=0.4)
    for name, pts in m.stones:
        poly(pts, stroke=(0.4, 0.4, 0.4), lw=0.3, dash=(1, 1))
    for name, pts in m.plates:
        poly(pts, stroke=(0.0, 0.5, 0.0), fill=(0.85, 0.95, 0.85), lw=0.5)
        a = np.mean(np.asarray(pts), axis=0)
        x, y = P((float(a[0]), float(a[1])))
        c.setFillColorRGB(0.0, 0.4, 0.0)
        c.setFont("Helvetica-Bold", 6)
        c.drawCentredString(x, y - 2, name)
    for pc in m.pieces:
        x0, y0, x1, y1 = pc.bbox()
        q = pc.to_wall(((x0 + x1) / 2, (y0 + y1) / 2))
        x, y = P(q)
        c.setFillColorRGB(0.45, 0.25, 0.05)
        c.setFont("Helvetica", 5)
        c.drawCentredString(x, y - 12, pc.name)
    # origin and axes
    o = P(m.origin)
    c.setStrokeColorRGB(0.8, 0, 0)
    c.setFillColorRGB(0.8, 0, 0)
    c.setLineWidth(1.0)
    ex = P(m.to_wall_map((500.0, 0.0)))
    ey = P(m.to_wall_map((0.0, 500.0)))
    c.line(*o, *ex)
    c.line(*o, *ey)
    c.circle(*o, 2.0, stroke=1, fill=1)
    c.setFont("Helvetica-Bold", 8)
    c.drawString(o[0] - 10, o[1] - 10, "O")
    c.drawString(ex[0] + 2, ex[1] - 3, "x")
    c.drawString(ey[0] - 3, ey[1] + 3, "y")
    # overall dimensions
    for with_ares, col, lab in ((False, (0.45, 0.25, 0.05), "guides + station"), (True, (0.1, 0.3, 0.8),
                                                                              "incl. ARES moves")):
        b0 = m.bbox(with_ares)
        c.setStrokeColorRGB(*col)
        c.setLineWidth(0.5)
        c.setDash(4, 2)
        x0, y0 = Pm((b0[0], b0[1]))
        x1, y1 = Pm((b0[2], b0[3]))
        c.rect(x0, y0, x1 - x0, y1 - y0, stroke=1, fill=0)
        c.setDash()
        c.setFillColorRGB(*col)
        c.setFont("Helvetica", 7)
        c.drawString(x0 + 2, y1 + 2 if not with_ares else y0 - 8,
                     f"{lab}: {b0[2] - b0[0]:.0f} x {b0[3] - b0[1]:.0f} mm (x {b0[0]:.0f}..{b0[2]:.0f}, "
                     f"y {b0[1]:.0f}..{b0[3]:.0f})")
    # scale bar + title
    c.setStrokeColorRGB(0, 0, 0)
    c.setFillColorRGB(0, 0, 0)
    sx, sy = margin, margin + 6 * mm
    c.setLineWidth(1.0)
    c.line(sx, sy, sx + 1000 / sc * mm, sy)
    for k in range(0, 1001, 250):
        c.line(sx + k / sc * mm, sy - 1.5 * mm, sx + k / sc * mm, sy + 1.5 * mm)
    c.setFont("Helvetica", 7)
    c.drawString(sx, sy + 2.5 * mm, "1 m")
    c.setFont("Helvetica-Bold", 11)
    c.drawString(margin, ph - margin - 4 * mm, f"ARES_Mauer floor map - {info['title']}")
    c.setFont("Helvetica", 8)
    c.drawString(margin, ph - margin - 9 * mm,
                 f"scale 1:{sc} on A3 - print at 100 % (check the 1 m bar). Map frame: origin O = outer start corner of "
                 f"leg {info['first_leg']}'s guide strip, x along that leg, y towards ARES; mm. Grid 500 mm.")
    c.drawString(margin, ph - margin - 13 * mm, info["sub"])
    c.setFont("Helvetica", 6)
    c.drawString(margin + 45 * mm, margin + 4 * mm,
                 "brown: MDF guide pieces (names = first-last stone) and the station piece with the S windows; grey "
                 "dashed: first-course stones and station stacks; green: board plates; blue: ARES at the stops / dock "
                 "(dashed), its footprints along the routes and rotation circles")
    c.showPage()
    # page 2: marking points
    c.setFont("Helvetica-Bold", 11)
    c.drawString(margin, ph - margin - 4 * mm, "Marking points (map frame, mm) and checks")
    c.setFont("Helvetica", 8)
    y = ph - margin - 12 * mm
    for line in info["page2"]:
        if y < margin:
            c.showPage()
            c.setFont("Helvetica", 8)
            y = ph - margin
        c.drawString(margin, y, line)
        y -= 4.2 * mm
    c.showPage()
    c.save()
    return sc


# ── main ──────────────────────────────────────────────────────────────────────
def build(cfg: dict, with_job: bool = True) -> dict:
    """Everything make_guides writes, as data (also used by the tests)."""
    import make_job as mj
    wp = mj.load_wallplan()
    p = params(cfg)
    legs = wp.legs(cfg)
    if not legs:
        raise ValueError("make_guides needs a wall of legs ([[wall.legs]])")
    sites = mj.target_sites(cfg, legs)
    for s in sites:
        if s.k < 0 or s.k >= next(lg.n0 for lg in legs if lg.name == s.leg):
            raise ValueError(f"plate {s.board} on a spare block - the guides have no spare blocks")
    leg_pcs = leg_pieces(cfg, p, legs, sites)
    st_plates = _station_plates(cfg)
    st_pcs = station_pieces(cfg, p, st_plates)
    job = None
    warn = []
    if with_job:
        try:
            job = mj.build_l(cfg)
        except Exception as e:  # the map is still useful without the routes
            warn.append(f"job not built ({e}) - ARES stops and routes missing on the map")
    fm = floor_model(cfg, p, legs, sites, st_pcs, leg_pcs, job)
    fm.warnings += warn
    plate_wh = st_plates[0][2] if st_plates else (140.0, 132.0)
    return {"params": p, "legs": legs, "sites": sites, "leg_pieces": leg_pcs, "station_pieces": st_pcs,
            "test_pieces": test_pieces(p, plate_wh), "floor": fm, "job": job}


def plan_md(cfg: dict, data: dict, sheets: list, sc: int) -> list[str]:
    p, fm = data["params"], data["floor"]
    lines = []
    lines.append("Marking points: lay out leg A's outer edge first (O -> A end), then place the others by x / y; check "
                 "the diagonals.")
    for name, q in fm.marks:
        x, y = fm.to_map(q)
        lines.append(f"  {name:45s} x {x:8.1f}   y {y:8.1f}")
    for name, a in fm.ares:
        x, y = fm.to_map((a.x_mm, a.y_mm))
        lines.append(f"  ARES {name:40s} x {x:8.1f}   y {y:8.1f}   heading {math.degrees(a.theta_rad - fm.x_axis):6.1f} deg")
    lines.append("")
    lines.append("Diagonals (tape measure, mm):")
    pts = dict(fm.marks)
    names = [n for n, _ in fm.marks]
    o = names[0]
    for n in names[1:]:
        if "outer corner" in n or "station" in n:
            a, b = pts[o], pts[n]
            lines.append(f"  {o} -> {n}: {math.hypot(a[0] - b[0], a[1] - b[1]):.0f}")
    b0, b1 = fm.bbox(False), fm.bbox(True)
    lines.append("")
    lines.append(f"Guides + station: {b0[2] - b0[0]:.0f} x {b0[3] - b0[1]:.0f} mm; including every ARES position, route "
                 f"and rotation circle: {b1[2] - b1[0]:.0f} x {b1[3] - b1[1]:.0f} mm "
                 f"(x {b1[0]:.0f}..{b1[2]:.0f}, y {b1[1]:.0f}..{b1[3]:.0f} in the map frame).")
    n_full = sum(1 for pc in data["leg_pieces"] + data["station_pieces"] for b in pc.blocks if b[3] == "full")
    n_half = sum(1 for pc in data["station_pieces"] for b in pc.blocks if b[3] == "half")
    lines.append("")
    lines.append(f"Parts: {len(sheets)} MDF sheets {p.sheet[0]:.0f} x {p.sheet[1]:.0f} x {p.mdf_t:.0f} mm "
                 f"({len(data['leg_pieces'])} guide pieces, {len(data['station_pieces'])} station pieces), "
                 f"{n_full} full + {n_half} half socket blocks (print 1-2 spares each), carpet tape.")
    for fx in fm.warnings:
        lines.append(f"WARNING: {fx}")
    return lines


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=str(REPO / "targets" / "guides"))
    ap.add_argument("--no-map", action="store_true", help="skip the floor map (no job / routes needed)")
    a = ap.parse_args(argv)
    cfg = config.load(a.config)
    data = build(cfg, with_job=not a.no_map)
    p = data["params"]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sheets = nest(data["leg_pieces"] + data["station_pieces"], p)
    for i, items in enumerate(sheets):
        sheet_dxf(items, p, f"ARES_Mauer guides sheet {i + 1}/{len(sheets)}").write(out / f"laser_sheet_{i + 1}.dxf")
    test = nest(data["test_pieces"], p)
    sheet_dxf(test[0], p, "ARES_Mauer guides TEST").write(out / "laser_test.dxf")
    write_svg(sheets + test, p, out / "laser_sheets.svg")
    write_png(sheets + test, p, out / "laser_sheets.png")
    for kind in ("full", "half"):
        write_stl(out / f"socket_block_{kind}.stl", block_mesh(p, kind), f"ARES_Mauer socket block {kind}")
    write_scad(out / "socket_blocks.scad", p)
    print(f"{len(sheets)} laser sheets + test sheet, socket blocks full / half (STL, SCAD) -> {out}", flush=True)
    for i, items in enumerate(sheets):
        print(f"  sheet {i + 1}: " + ", ".join(pc.name for pc, _, _ in items), flush=True)
    if not a.no_map:
        fm = data["floor"]
        legs = data["legs"]
        info = {"title": "C wall " + " / ".join(f"{lg.name} {lg.n0}" for lg in legs) + " stones, floor station",
                "first_leg": legs[0].name,
                "sub": f"[wall] base_z {cfg['wall']['base_z']} mm, guides [guides], station [pickup_station] - "
                       "generated by tools/make_guides.py from config/station.toml",
                "page2": []}
        info["page2"] = plan_md(cfg, data, sheets, 0)
        sc = write_map_pdf(out / "floor_map.pdf", fm, cfg, p, info)
        md = ["# ARES_Mauer floor plan", "", f"Generated by `tools/make_guides.py` from `config/station.toml`. Map "
              f"`floor_map.pdf` at 1:{sc} on A3 (print at 100 %). Map frame: origin O = outer start corner of leg "
              f"{legs[0].name}'s guide strip, x along leg {legs[0].name}, y towards ARES (mm).", "", "```"]
        md += info["page2"] + ["```", "", "Laser sheets: " + "; ".join(
            f"{i + 1}: " + ", ".join(pc.name for pc, _, _ in items) for i, items in enumerate(sheets))]
        (out / "floor_plan.md").write_text("\n".join(md) + "\n", encoding="utf-8")
        print(f"floor map 1:{sc} -> {out / 'floor_map.pdf'}, {out / 'floor_plan.md'}", flush=True)
        for line in info["page2"]:
            print("  " + line, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
