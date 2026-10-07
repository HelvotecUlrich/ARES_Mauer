"""Floor guides of the first course and of the floor pick-up station: laser-cut MDF pieces (DXF for the Trotec,
800 x 600 mm sheets), 3D-printed locating cones (STL for the Bambu Lab X1E + OpenSCAD source) and the floor map (PDF
on A3 at a fixed scale + a coordinate table) to measure out the room.

    py.exe tools/make_guides.py [--out targets/guides] [--no-map]

Design (config [guides], Samuel 2026-10-06: stones PINS UP ([brick] pins_up), MDF strip + printed locating cones,
carpet tape, V-tabs for the existing board plates, station on the floor in one row):
- Per wall leg a 4 mm MDF strip ([plates] mdf_t), [guides] strip_width wide, centred on the leg's wall line, from
  u = 0 to n0 x pitch. Every first-course stone stands directly on the strip ([wall] base_z = mdf_t); two peg holes
  per stone take the pegs of two locating cones in diagonally opposite sockets of its underside (a half stone: one
  cone in each of its two sockets).
  The ARES-side edge (leg v = +strip_width/2, the face the existing wall plates were cut for) carries a V-tab at every
  inner stone joint u = pitch k (k = 1 .. n0-1); one tab in a plate's V-notch plus the edge fixes a plate completely.
- Pieces: a strip is cut [guides] cut_offset after a stone joint (the tab stays whole) into pieces of at most
  segment_max; consecutive pieces join with a dovetail on the centre line (tail on the downstream piece, socket on the
  upstream one, wall order A -> B -> C). The last corner_arm_stones stones of a leg and the first of the next form
  one L-piece across the corner (the 3 mm butt-joint gap filled), so the corner offset is cut, not measured - either
  leg may run through the corner (wallplan.butt_corner through; the C since 2026-10-07: A and C run through).
- Locating cone (printed): the stone's socket cone (STEP: r socket_r_mouth at the face -> socket_r_bottom at
  socket_depth) minus [guides] socket_clearance, locator_h high, with a peg into the strip's peg hole. The STL is
  oriented for printing (narrow top on the bed, peg up: no supports; the wall overhangs 25 deg from vertical).
- Station: one MDF piece set in the station frame ([pickup_station]: x along the row, y away from ARES), cut between
  the stacks into pieces <= segment_max; holes for every stack's block at row_y, closed windows for the plain S0/S1
  plates ([[targets]] parent "station") with [guides] window_clearance.
- Floor map: everything in the wall frame, shown in a map frame with the origin O at the OUTER START CORNER of the
  first leg's strip (x along the first leg, y towards ARES); pieces, course-0 stones, board plates, the station with
  its stacks, ARES at every stop and at the dock with its routes (tools/make_job.py build_l) and rotation circles.

Outputs (in --out): laser_sheet_<n>.dxf (layers CUT red / ENGRAVE blue, as tools/make_plates.py; sheet border on
layer SHEET green, not to be cut), laser_test.dxf (test pieces: one stone, a dovetail pair, an S-plate window),
laser_sheets.svg / .png (previews), locating_cone.stl, locating_cone.scad, floor_map.pdf, floor_plan.md
(coordinates, diagonals, part list, room size).
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
    stone_l: float
    stone_w: float
    half_l: float
    pin_along: float            # socket (= pin) centres along the stone
    pin_across: float           # ... and across
    r_mouth: float              # socket cone (STEP)
    r_bottom: float
    socket_depth: float
    locator_h: float
    clearance: float
    peg_d: float
    peg_len: float
    peg_hole_d: float
    vtab: tuple[float, float]
    dovetail: tuple[float, float, float]
    joint_c: float
    kerf: float                 # laser kerf: cut outlines drawn kerf/2 larger, holes / windows kerf/2 smaller
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
    gd, pl, b = cfg["guides"], cfg["plates"], cfg["brick"]
    mdf = float(pl["mdf_t"])
    hb = cfg.get("half_brick") or b
    p = GuideParams(
        pitch=float(b["length"]), strip_w=float(gd["strip_width"]), mdf_t=mdf, stone_l=float(b["length"]),
        stone_w=float(b["width"]), half_l=float(hb.get("length", b["length"] / 2)), pin_along=float(gd["pin_along"]),
        pin_across=float(gd["pin_across"]), r_mouth=float(gd["socket_r_mouth"]), r_bottom=float(gd["socket_r_bottom"]),
        socket_depth=float(gd["socket_depth"]), locator_h=float(gd["locator_h"]),
        clearance=float(gd["socket_clearance"]), peg_d=float(gd["peg_d"]), peg_len=float(gd["peg_len"]),
        peg_hole_d=float(gd["peg_hole_d"]), vtab=tuple(float(v) for v in gd["vtab"]),
        dovetail=tuple(float(v) for v in gd["dovetail"]), joint_c=float(gd["joint_clearance"]),
        kerf=float(gd.get("kerf_mm", 0.0)),
        cut_offset=float(gd["cut_offset"]), arm=int(gd["corner_arm_stones"]), seg_max=float(gd["segment_max"]),
        sheet=tuple(float(v) for v in gd["sheet"]), sheet_margin=float(gd["sheet_margin"]),
        window_c=float(gd["window_clearance"]))
    if not b.get("pins_up"):
        raise ValueError("make_guides is designed for [brick] pins_up = true (locating cones in the sockets)")
    if abs(float(cfg["wall"]["base_z"]) - mdf) > 1e-9:
        raise ValueError("[wall] base_z must equal [plates] mdf_t: the first course stands on the MDF strip")
    if not 0.0 < p.locator_h < p.socket_depth or p.peg_len >= mdf:
        raise ValueError("[guides]: locator_h must be inside the socket and the peg shorter than the MDF is thick")
    if 2 * cone_r(p, p.locator_h) <= p.peg_d + 4.0:
        raise ValueError("[guides]: the locating cone's top is too narrow around its peg")
    if abs(float(pl["block_length"]) - p.pitch) > 1e-9:
        raise ValueError("[plates] block_length must equal the stone pitch (the plates' notches sit on its joints)")
    return p


# ── socket blocks ─────────────────────────────────────────────────────────────
def cone_r(p: GuideParams, height: float) -> float:
    """Locating cone radius at `height` above the strip = the stone's socket cone at that depth above its underside
    (STEP) minus the clearance."""
    return p.r_mouth - (p.r_mouth - p.r_bottom) * height / p.socket_depth - p.clearance


def stone_size(p: GuideParams, kind: str) -> tuple[float, float]:
    return (p.stone_l if kind == "full" else p.half_l), p.stone_w


def socket_xy(p: GuideParams, kind: str) -> list[Pt]:
    """All socket centres in the stone frame (u along the stone, v across, origin = stone centre)."""
    a, b = p.pin_along / 2.0, p.pin_across / 2.0
    if kind == "full":
        return [(su * a, sv * b) for su in (-1.0, 1.0) for sv in (-1.0, 1.0)]
    return [(0.0, -b), (0.0, b)]


def locator_xy(p: GuideParams, kind: str) -> list[Pt]:
    """The sockets that get a locating cone: two diagonally opposite ones (a half stone: its two)."""
    a, b = p.pin_along / 2.0, p.pin_across / 2.0
    return [(-a, -b), (a, b)] if kind == "full" else [(0.0, -b), (0.0, b)]


def frustum(c: Pt, r0: float, r1: float, z0: float, z1: float, n: int = N_SEG) -> tuple[np.ndarray, np.ndarray]:
    """Closed frustum (outward normals): radius r0 at z0, r1 at z1."""
    angs = [2.0 * math.pi * k / n for k in range(n)]
    V = [(c[0] + r * math.cos(a), c[1] + r * math.sin(a), z) for z, r in ((z0, r0), (z1, r1)) for a in angs]
    V += [(c[0], c[1], z0), (c[0], c[1], z1)]
    cb, ct = 2 * n, 2 * n + 1
    T = []
    for k in range(n):
        k1 = (k + 1) % n
        T += [(k, k1, n + k1), (k, n + k1, n + k), (cb, k1, k), (ct, n + k, n + k1)]
    return np.asarray(V, float), np.asarray(T, int)


def locator_mesh(p: GuideParams, print_orientation: bool = True) -> list[tuple[np.ndarray, np.ndarray]]:
    """Cone (base on the strip at z 0) + peg (into the peg hole, reaching 0.5 mm into the cone - the slicer unites
    the shells). Print orientation: turned 180 deg about x (narrow top on the bed, peg up)."""
    shells = [frustum((0.0, 0.0), cone_r(p, 0.0), cone_r(p, p.locator_h), 0.0, p.locator_h),
              frustum((0.0, 0.0), p.peg_d / 2.0, p.peg_d / 2.0, -p.peg_len, 0.5)]
    if print_orientation:
        out = []
        for V, T in shells:
            W = V.copy()
            W[:, 1], W[:, 2] = -V[:, 1], p.locator_h - V[:, 2]
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
    path.write_text(
        "// Locating cone of the ARES_Mauer floor guides - generated by tools/make_guides.py from config/station.toml\n"
        "// [guides] (edit the config and regenerate rather than this file). Use orientation: base on the MDF strip,\n"
        "// peg down into the peg hole; PRINT upside down (narrow top on the bed, peg up). Units mm.\n"
        f"$fn = 96;\nH = {p.locator_h:.3f};          // [guides] locator_h\n"
        f"R_BASE = {cone_r(p, 0.0):.3f};    // socket cone at the stone's underside - clearance\n"
        f"R_TOP = {cone_r(p, p.locator_h):.3f};     // the same cone at H\n"
        f"PEG_D = {p.peg_d:.3f};\nPEG_LEN = {p.peg_len:.3f};\n\n"
        "cylinder(h = H, r1 = R_BASE, r2 = R_TOP);\n"
        "translate([0, 0, -PEG_LEN]) cylinder(h = PEG_LEN + 0.5, d = PEG_D);\n", encoding="ascii")


# ── laser-cut pieces ──────────────────────────────────────────────────────────
@dataclass
class Piece:
    """A flat MDF piece in its local frame (a leg frame, or the station frame), placed in the wall frame by pose."""
    name: str
    pose: tuple[float, float, float]            # local frame in the wall frame (x, y, theta [rad])
    outline: list[Pt]                           # CCW, with tabs / dovetails
    holes: list[tuple[Pt, float]] = field(default_factory=list)      # (centre, diameter)
    windows: list[list[Pt]] = field(default_factory=list)
    blocks: list[tuple[str, Pt, float, str]] = field(default_factory=list)   # stones: (label, centre, angle, kind)
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
    """Peg holes of the locating cones of a stone at `centre`, turned by `ang` (rad) in the piece frame."""
    c, s = math.cos(ang), math.sin(ang)
    return [((centre[0] + c * q[0] - s * q[1], centre[1] + s * q[0] + c * q[1]), p.peg_hole_d)
            for q in locator_xy(p, kind)]


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


def corner_geometry(a, b, h: float, L: float) -> tuple[str, float]:
    """Butt corner a -> b turning towards a's +v (ARES side, the inside of the C/L), b's origin in a's frame (a's
    length L): ("prev", v0) - a runs through, b starts at u = L - h beside a's ARES-side face at v = v0; or ("next",
    u0) - b runs through (2026-10-07), its centre line at u = u0 beyond a's end face, starting flush with a's outer
    face (v = -h). ValueError for any other corner."""
    u, v = a.from_wall(b.x, b.y)
    dth = (b.theta - a.theta + math.pi) % (2 * math.pi) - math.pi
    if abs(dth - math.pi / 2) <= 1e-6:
        if abs(u - (L - h)) <= 1e-6 and h - 1e-6 <= v <= h + 20.0:
            return "prev", v
        if abs(v + h) <= 1e-6 and L + h - 1e-6 <= u <= L + h + 20.0:
            return "next", u
    raise ValueError(f"corner {a.name}->{b.name}: only butt corners turning towards the ARES side are supported "
                     f"(+90 deg; {b.name} starting at u = L - {h:g} beside {a.name}'s ARES-side face, or running "
                     f"through beyond {a.name}'s end; got {math.degrees(dth):.1f} deg, u {u:.1f}, v {v:.1f} mm)")


def course0(p: GuideParams, lg) -> list[tuple[float, float, str]]:
    """First-course stones of a leg on its strip: (u0, u1, kind) - floor(n0) full stones from u = 0, and for a leg of
    x.5 stones a half stone at the end (robodk/wallplan.layout_leg, 2026-10-07: A of 5 1/2 stones)."""
    P = p.pitch
    nf = int(math.floor(lg.n0 + 1e-9))
    out = [(k * P, (k + 1) * P, "full") for k in range(nf)]
    if lg.n0 - nf > 1e-9:
        out.append((nf * P, lg.n0 * P, "half"))
    return out


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
        st = course0(p, lg)
        n = len(st)
        J = [u0 for u0, _, _ in st] + [st[-1][1]]          # course-0 joints, J[0] = 0, J[n] = leg length
        L = J[-1]
        first = p.arm if i > 0 else 0
        last = p.arm if i < nl - 1 else 0
        if first + last > n:
            raise ValueError(f"leg {lg.name}: {n} stones are fewer than the corner arms ({first} + {last})")
        for grp in _split(list(range(first, n - last)), kmax):
            a, b = grp[0], grp[-1] + 1
            u0 = 0.0 if a == 0 else J[a] + off
            u1 = L if b == n else J[b] + off
            poly = [(u0, -h), (u1, -h), (u1, h), (u0, h)]
            feats: dict[int, list] = {2: [(u1 - J[k], "tab") for k in range(1, n) if u0 < J[k] < u1]}
            if not (i == 0 and a == 0):
                feats[3] = [(h, "tail")]
            if not (i == nl - 1 and b == n):
                feats[1] = [(h, "socket")]
            pc = Piece(f"{lg.name}{a}-{lg.name}{b - 1}", (lg.x, lg.y, lg.theta), decorate(p, poly, feats))
            _stones(p, pc, lg.name, st, range(a, b), lambda q: q, 0.0, plate_at)
            pieces.append(pc)
        if i < nl - 1:
            nb = legs[i + 1]
            stb = course0(p, nb)
            Jb = [u0 for u0, _, _ in stb] + [stb[-1][1]]
            nbn = len(stb)
            kind, c0 = corner_geometry(lg, nb, h, L)
            u_cut = J[n - p.arm] + off
            arm_end = Jb[-1] if (i + 1 == nl - 1 and p.arm == nbn) else Jb[p.arm] + off
            if kind == "prev":                         # lg runs through, nb starts beside its ARES-side face
                v0 = c0
                v_end = v0 + arm_end
                poly = [(u_cut, -h), (L, -h), (L, v_end), (L - 2 * h, v_end), (L - 2 * h, h), (u_cut, h)]
                feats = {5: [(h, "tail")],
                         4: [((L - 2 * h) - J[k], "tab") for k in range(1, n)
                             if u_cut < J[k] < L - 2 * h - p.vtab[0]],
                         3: [(v_end - (v0 + Jb[k]), "tab") for k in range(1, nbn) if 0 < Jb[k] < arm_end - off / 2]}

                def to_a(q: Pt, L=L, v0=v0) -> Pt:             # leg nb frame -> leg lg frame
                    return L - h - q[1], v0 + q[0]
            else:                                      # nb runs through beyond lg's end (the gap is filled)
                u0 = c0
                v_end = -h + arm_end
                poly = [(u_cut, -h), (u0 + h, -h), (u0 + h, v_end), (u0 - h, v_end), (u0 - h, h), (u_cut, h)]
                feats = {5: [(h, "tail")],
                         4: [((u0 - h) - J[k], "tab") for k in range(1, n) if u_cut < J[k] < L],
                         3: [(arm_end - Jb[k], "tab") for k in range(1, nbn)
                             if 2 * h + p.vtab[0] < Jb[k] < arm_end - off / 2]}

                def to_a(q: Pt, u0=u0) -> Pt:                  # leg nb frame -> leg lg frame
                    return u0 - q[1], -h + q[0]
            if not (i + 1 == nl - 1 and arm_end >= Jb[-1]):
                feats[2] = [(h, "socket")]
            pc = Piece(f"{lg.name}{n - p.arm}-{nb.name}{p.arm - 1}", (lg.x, lg.y, lg.theta), decorate(p, poly, feats),
                       notes=f"corner {lg.name}/{nb.name} ({(lg if kind == 'prev' else nb).name} runs through)")
            _stones(p, pc, lg.name, st, range(n - p.arm, n), lambda q: q, 0.0, plate_at)
            _stones(p, pc, nb.name, stb, range(0, p.arm), to_a, math.pi / 2, plate_at)
            pieces.append(pc)
    return pieces


def _stones(p: GuideParams, pc: Piece, leg: str, st: list, ks, to_local, ang: float, plate_at: dict) -> None:
    """Holes, engraved block footprints and labels of the first-course stones ks (indices into st = course0 of
    `leg`) on piece pc (to_local maps the leg's (u, v) into the piece frame, ang = the leg's rotation in the piece
    frame)."""
    for k in ks:
        u0, u1, kind = st[k]
        c = to_local(((u0 + u1) / 2.0, 0.0))
        pc.holes += stone_holes(p, kind, c, ang)
        pc.blocks.append((f"{leg}{k}", c, ang, kind))
        if (leg, k) in plate_at:
            q = to_local(((u0 + u1) / 2.0, p.h - 9.0))
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
    reach = p.pin_along / 2.0 + cone_r(p, 0.0) + 5.0                    # the cones of a stone stay this far inside
    for a, b in zip(stacks, stacks[1:]):
        lo = a[1] + (reach if a[2] == "full" else cone_r(p, 0.0) + 5.0)
        hi = b[1] - (reach if b[2] == "full" else cone_r(p, 0.0) + 5.0)
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


def offset_polygon(poly: list[Pt], d: float) -> list[Pt]:
    """Polygon offset by d along the outward normals (CCW polygon: d > 0 grows it, d < 0 shrinks it), mitred corners -
    the laser kerf compensation (d = +-kerf/2 is a fraction of a millimetre, far below any edge length)."""
    n = len(poly)
    P = [np.asarray(q, float) for q in poly]
    out = []
    for i in range(n):
        a, b, c = P[i - 1], P[i], P[(i + 1) % n]
        e1, e2 = (b - a) / np.linalg.norm(b - a), (c - b) / np.linalg.norm(c - b)
        n1, n2 = np.array([e1[1], -e1[0]]), np.array([e2[1], -e2[0]])
        out.append(tuple(b + d * (n1 + n2) / (1.0 + float(n1 @ n2))))
    return out


def cut_geometry(pc: "Piece", p: GuideParams) -> tuple[list[Pt], list[tuple[Pt, float]], list[list[Pt]]]:
    """What the laser follows: the outline kerf/2 outside, holes and windows kerf/2 inside the nominal geometry."""
    k = p.kerf / 2.0
    return (offset_polygon(pc.outline, k), [(c, dia - p.kerf) for c, dia in pc.holes],
            [offset_polygon(w, -k) for w in pc.windows])


def _place(q: Pt, rot: bool, off: Pt) -> Pt:
    x, y = (q if not rot else (-q[1], q[0]))
    return x + off[0], y + off[1]


def sheet_dxf(items: list[tuple[Piece, bool, Pt]], p: GuideParams, label: str) -> mp.Dxf:
    d = mp.Dxf()
    d.poly(SHEET, [(0.0, 0.0), (p.sheet[0], 0.0), (p.sheet[0], p.sheet[1]), (0.0, p.sheet[1])])
    d.text(SHEET, (2.0, p.sheet[1] - 6.0), 4.0, label + (f" - CUT lines kerf-compensated for {p.kerf:g} mm" if p.kerf
                                                       else ""))
    for pc, rot, off in items:
        outline, holes, windows = cut_geometry(pc, p)
        d.poly(mp.CUT, [_place(q, rot, off) for q in outline])
        for c, dia in holes:
            d.circle(mp.CUT, _place(c, rot, off), dia / 2.0)
        for w in windows:
            d.poly(mp.CUT, [_place(q, rot, off) for q in w])
        for name, c, ang, kind in pc.blocks:
            L, W = stone_size(p, kind)
            ca, sa = math.cos(ang), math.sin(ang)
            corners = [(c[0] + ca * u - sa * v, c[1] + sa * u + ca * v) for u, v in
                       ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2))]
            d.poly(mp.ENGRAVE, [_place(q, rot, off) for q in corners])
            for u, v in locator_xy(p, kind):
                d.circle(mp.ENGRAVE, _place((c[0] + ca * u - sa * v, c[1] + sa * u + ca * v), rot, off), cone_r(p, 0.0))
            d.text(mp.ENGRAVE, _place((c[0] - 6.0, c[1] - 2.5), rot, off), 5.0, name, rot_deg=90.0 if rot else 0.0)
        for q, hgt, s in pc.texts:
            d.text(mp.ENGRAVE, _place(q, rot, off), hgt, s, rot_deg=90.0 if rot else 0.0)
        anchor, ang = piece_label_anchor(pc, p)
        d.text(mp.ENGRAVE, _place(anchor, rot, off), 6.0, pc.name + (f" ({pc.notes})" if pc.notes else ""),
               rot_deg=math.degrees(ang) + (90.0 if rot else 0.0))
    return d


def piece_label_anchor(pc: Piece, p: GuideParams) -> tuple[Pt, float]:
    """Start and direction (rad, piece frame) of the engraved piece name: inside the lower-left corner of the first
    stone block (behind the piece joint cut_offset after the stone joint), along the block (review 2026-10-07: from the bbox corner the name of a corner L-piece placed turned
    on the sheet lay entirely outside the piece); a piece without blocks: bbox corner."""
    if not pc.blocks:
        x0, y0, _, _ = pc.bbox()
        return (x0 + 6.0, y0 + 4.0), 0.0
    _, c, ang, kind = pc.blocks[0]
    L, W = stone_size(p, kind)
    u, v = -L / 2.0 + p.cut_offset + 6.0, -W / 2.0 + 4.0    # pieces start cut_offset after a stone joint
    ca, sa = math.cos(ang), math.sin(ang)
    return (c[0] + ca * u - sa * v, c[1] + sa * u + ca * v), ang


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
                L, Wb = stone_size(p, kind)
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
        for k, (u0, u1, _) in enumerate(course0(p, lg)):
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
        lg = next(lg for lg in legs if lg.name == s.leg)
        if s.k < 0 or s.k >= sum(kind == "full" for _, _, kind in course0(p, lg)):
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
        x, y = (round(v, 1) + 0.0 for v in fm.to_map(q))              # + 0.0: no "-0.0"
        lines.append(f"  {name:45s} x {x:8.1f}   y {y:8.1f}")
    for name, a in fm.ares:
        x, y = (round(v, 1) + 0.0 for v in fm.to_map((a.x_mm, a.y_mm)))
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
    n_cones = sum(len(pc.holes) for pc in data["leg_pieces"] + data["station_pieces"])
    lines.append("")
    lines.append(f"Parts: {len(sheets)} MDF sheets {p.sheet[0]:.0f} x {p.sheet[1]:.0f} x {p.mdf_t:.0f} mm "
                 f"({len(data['leg_pieces'])} guide pieces, {len(data['station_pieces'])} station pieces), "
                 f"{n_cones} locating cones (2 per stone on the guides; print a few spares), carpet tape.")
    for fx in fm.warnings:
        lines.append(f"WARNING: {fx}")
    return lines


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--variant", default=None, help="config variant config/variants/<VARIANT>.toml")
    ap.add_argument("--out", default=None, help="default targets/guides[_<variant>]")
    ap.add_argument("--no-map", action="store_true", help="skip the floor map (no job / routes needed)")
    a = ap.parse_args(argv)
    cfg = config.load(a.config, a.variant)
    a.out = a.out or str(REPO / "targets" / f"guides{config.suffix(cfg)}")
    data = build(cfg, with_job=not a.no_map)
    p = data["params"]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    v = config.variant_of(cfg)
    tag = f"ARES_Mauer guides{' ' + v if v else ''} " + "/".join(f"{lg.name}{lg.n0:g}" for lg in data["legs"])
    src = "config/station.toml" + (f" + config/variants/{v}.toml" if v else "")
    sheets = nest(data["leg_pieces"] + data["station_pieces"], p)
    for i, items in enumerate(sheets):
        sheet_dxf(items, p, f"{tag} sheet {i + 1}/{len(sheets)}").write(out / f"laser_sheet_{i + 1}.dxf")
    test = nest(data["test_pieces"], p)
    sheet_dxf(test[0], p, f"{tag} TEST").write(out / "laser_test.dxf")
    write_svg(sheets + test, p, out / "laser_sheets.svg")
    write_png(sheets + test, p, out / "laser_sheets.png")
    for old in ("socket_block_full.stl", "socket_block_half.stl", "socket_blocks.scad"):   # the pins-down design
        (out / old).unlink(missing_ok=True)
    write_stl(out / "locating_cone.stl", locator_mesh(p), "ARES_Mauer locating cone")
    write_scad(out / "locating_cone.scad", p)
    print(f"{len(sheets)} laser sheets + test sheet, locating cone (STL, SCAD) -> {out}", flush=True)
    for i, items in enumerate(sheets):
        print(f"  sheet {i + 1}: " + ", ".join(pc.name for pc, _, _ in items), flush=True)
    if not a.no_map:
        fm = data["floor"]
        legs = data["legs"]
        info = {"title": "C wall " + " / ".join(f"{lg.name} {lg.n0:g}" for lg in legs) + " stones, floor station"
                         + (f" (config variant {v})" if v else ""),
                "first_leg": legs[0].name,
                "sub": f"stones pins up, first course on the 4 mm MDF ([wall] base_z {cfg['wall']['base_z']} mm), "
                       "course pitch 121 mm; guides [guides], station [pickup_station] - generated by "
                       f"tools/make_guides.py from {src}",
                "page2": []}
        info["page2"] = plan_md(cfg, data, sheets, 0)
        sc = write_map_pdf(out / "floor_map.pdf", fm, cfg, p, info)
        md = ["# ARES_Mauer floor plan" + (f" (config variant {v})" if v else ""), "",
              f"Generated by `tools/make_guides.py` from `{src}`. Map "
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
