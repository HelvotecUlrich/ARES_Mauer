"""Plates on the ARES deck (2026-10-08): 4 mm MDF with the peg holes of the locating cones of every magazine holder,
screwed to the deck with M6 / slot nuts 20 mm from the ARES outer edge. Three variants ([deck_plate]):

    magazine   full ARES width (600), split_x .. front_x: all magazine stones, dovetail SOCKETS on the rear edge (now)
    rear       deck rear edge .. split_x, R 80 corners, dovetail TAILS (later, maybe a control cabinet on it)
    onepiece   the first design: rear edge .. front_x, inset_mm inside the deck edge so it fits the 800 x 600 bed

    py.exe tools/make_deck_plate.py        # -> targets/deck_plate/deck_plate_<variant>.dxf + deck_plates.png

Everything comes from config/station.toml: the deck ([ares] length, width, deck_corner_r), [deck_plate], the holders
([ur5] mount_x + [deck] magazine_rows_dx, magazine_y; stones with their long axis along ARES y as tools/make_job.py
_magazine), the cones ([guides] pin_along / pin_across: 4 per holder at the sockets of a full stone; the half-stone
holder ([deck] half_positions) takes two halves end to end on the same 4 cones), the dovetail ([guides] dovetail,
joint_clearance, as tools/make_guides.py) and the holes ([guides] peg_hole_d, kerf_mm: cut circles kerf smaller,
outlines kerf/2 outside). Drawn in plate coordinates (ARES x, y shifted so each plate starts at 0, 0); layers CUT red /
ENGRAVE blue (tools/make_plates.py). Engraved: stone footprints, holder names, ARES centreline, ARES / HSLU.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import make_plates as mp  # noqa: E402

from mauer import config  # noqa: E402

Pt = tuple[float, float]
ARC_SEG_DEG = 3.0           # polyline segment of the rounded corners
CAP_W = 0.96                # Arial Bold advance / cap height of ARES, HSLU (PIL arialbd.ttf: 0.96-0.97)
LOGO_H, LOGO2_H = 50.0, 14.0  # cap height of "ARES" between the stone rows and of "HSLU" left / right of it [mm]
COVER_H = 40.0              # cap height of "ARES" / "HSLU" on the front covers [mm]
WEB_MM = 3.0                # minimum MDF between a hole and an edge (check)


@dataclass
class Plate:
    name: str
    outline: list[Pt]                       # nominal, counter-clockwise, ARES frame
    cones: list[Pt] = field(default_factory=list)
    bolts: list[Pt] = field(default_factory=list)
    holders: list[tuple[str, Pt]] = field(default_factory=list)
    texts: list[tuple[Pt, float, str]] = field(default_factory=list)   # left baseline, cap height, text (rot 90)
    lines: list[tuple[Pt, Pt]] = field(default_factory=list)


def arc(cx: float, cy: float, r: float, a0: float, a1: float) -> list[Pt]:
    n = max(2, int(round(abs(a1 - a0) / ARC_SEG_DEG)))
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def dovetail(x: float, yc: float, gd: dict, female: bool, downward: bool) -> list[Pt]:
    """The four points of one dovetail on the joint line x = const, widening towards +x (into the magazine plate):
    neck width n at the joint, n + 2 flare at depth d. female: the socket, joint_clearance wider per side."""
    n, d, f = (float(v) for v in gd["dovetail"])
    c = float(gd["joint_clearance"]) if female else 0.0
    lo, hi = yc - n / 2 - c, yc + n / 2 + c
    pts = [(x, lo), (x + d, lo - f), (x + d, hi + f), (x, hi)]
    return pts[::-1] if downward else pts


def centred(text: str, h: float, cx: float, cy: float) -> tuple[Pt, float, str]:
    """A word read along +y (rot 90, letters' top towards -x) centred on (cx, cy): its left baseline point."""
    return ((cx + h / 2, cy - CAP_W * h * len(text) / 2), h, text)


def plates(cfg: dict) -> list[Plate]:
    a, dp, dk, gd, u = cfg["ares"], cfg["deck_plate"], cfg["deck"], cfg["guides"], cfg["ur5"]
    L, W, R = float(a["length"]), float(a["width"]), float(a["deck_corner_r"])
    xr, xs, xf, yh = -L / 2, float(dp["split_x"]), float(dp["front_x"]), W / 2
    e = float(dp["bolt_edge_mm"])
    a_u, a_v = float(gd["pin_along"]) / 2, float(gd["pin_across"]) / 2
    half = {str(p) for p in dk.get("half_positions", [])}
    holders, cones = [], []
    for ri, dx in enumerate(dk["magazine_rows_dx"]):
        for yi, y in enumerate(dk["magazine_y"]):
            x = float(u["mount_x"]) + float(dx)
            holders.append((f"r{ri}y{yi}" + (" half" if f"r{ri}y{yi}" in half else ""), (x, float(y))))
            cones += [(x + sv * a_v, float(y) + su * a_u) for su in (-1, 1) for sv in (-1, 1)]   # long axis along y
    ys = sorted(float(y) for y in dp["dovetail_y"])
    side = lambda xs_: [(float(x), s * (yh - e)) for x in xs_ for s in (-1, 1)]            # noqa: E731
    rear_bolts = [(xr + e, float(y)) for y in dp["bolt_rear_y"]]

    # magazine plate: rectangle, sockets on the rear edge (walked downwards, from +y to -y)
    mag = [(xs, -yh), (xf, -yh), (xf, yh), (xs, yh)]
    for yc in reversed(ys):
        mag += dovetail(xs, yc, gd, female=True, downward=True)
    rows = sorted(float(u["mount_x"]) + float(dx) for dx in dk["magazine_rows_dx"])
    x_gap = (rows[0] + rows[-1]) / 2                  # between the two stone rows (Samuel 2026-10-08: ARES + HSLU)
    w_logo = CAP_W * LOGO_H * 4
    logo = [centred("ARES", LOGO_H, x_gap, 0.0)] + [centred("HSLU", LOGO2_H, x_gap, s_ * (w_logo / 2 + 70.0))
                                                     for s_ in (-1, 1)]
    p_mag = Plate("magazine", mag, cones, side(dp["bolt_side_x"]), holders, logo,
                  [((xs + 30.0, 0.0), (x_gap - LOGO_H / 2 - 8.0, 0.0)), ((x_gap + LOGO_H / 2 + 8.0, 0.0),
                                                                         (xf - 15.0, 0.0))])

    # rear plate: rounded rear corners, tails on the front edge (walked upwards)
    rear = [(xs, -yh)]
    for yc in ys:
        rear += dovetail(xs, yc, gd, female=False, downward=False)
    rear += [(xs, yh)] + arc(xr + R, yh - R, R, 90.0, 180.0) + arc(xr + R, -yh + R, R, 180.0, 270.0)
    p_rear = Plate("rear", rear, [], side(dp["rear_bolt_side_x"]) + rear_bolts, [], [],
                   [((xr + 40.0, 0.0), (xs - 15.0, 0.0))])

    # onepiece: the first design (inset to fit the bed)
    ins = float(dp["inset_mm"])
    one = [(xf, -yh + ins), (xf, yh - ins)] + arc(xr + R, yh - R, R - ins, 90.0, 180.0) \
        + arc(xr + R, -yh + R, R - ins, 180.0, 270.0)
    p_one = Plate("onepiece", one, cones, side(dp["onepiece_bolt_side_x"]) + rear_bolts, holders, [],
                  [((xr + 25.0, 0.0), (xf - 15.0, 0.0))])
    # front covers around the UR's aluminium plate, split at y = 0 (each one goes in beside the mounted UR)
    xb, xfr = L / 2 - float(dp["cover_depth"]), L / 2
    hp = float(dp["ur_plate"]) / 2 + float(dp["ur_plate_gap"])
    ux = float(u["mount_x"])
    right = ([(xb, 0.0), (xb, -yh)] + arc(xfr - R, -yh + R, R, 270.0, 360.0)
             + [(xfr, 0.0), (ux + hp, 0.0), (ux + hp, -hp), (ux - hp, -hp), (ux - hp, 0.0)])
    left = [(x, -y) for x, y in reversed(right)]
    covers = []
    for name, poly, s_ in (("cover_left", left, 1.0), ("cover_right", right, -1.0)):
        bolts = [(xfr - e, s_ * abs(float(y))) for y in dp["cover_bolt_front_y"][:1]] \
            + [(float(x), s_ * (yh - e)) for x in dp["cover_bolt_side_x"]]
        word = "ARES" if s_ > 0 else "HSLU"               # Samuel 2026-10-08: ARES left, HSLU right, read from the
        y_word = s_ * (hp + 5.0 + (yh - 35.0)) / 2        # front; centred beside the cut-out, clear of the side holes
        covers.append(Plate(name, poly, [], bolts, [],
                            [centred(word, COVER_H, ux, y_word)],
                            []))
    return [p_mag, p_rear, p_one] + covers


# ── geometry helpers ──────────────────────────────────────────────────────────
def offset(poly: list[Pt], d: float) -> list[Pt]:
    """Counter-clockwise polygon moved d outward (miter), for the kerf compensation of the outline."""
    out = []
    n = len(poly)
    for i in range(n):
        (ax, ay), (bx, by), (cx, cy) = poly[i - 1], poly[i], poly[(i + 1) % n]
        e1, e2 = (bx - ax, by - ay), (cx - bx, cy - by)
        l1, l2 = math.hypot(*e1), math.hypot(*e2)
        n1, n2 = (e1[1] / l1, -e1[0] / l1), (e2[1] / l2, -e2[0] / l2)
        k = d / (1.0 + n1[0] * n2[0] + n1[1] * n2[1])
        out.append((bx + k * (n1[0] + n2[0]), by + k * (n1[1] + n2[1])))
    return out


def inside(poly: list[Pt], p: Pt) -> bool:
    x, y, c = p[0], p[1], False
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            c = not c
    return c


def edge_dist(poly: list[Pt], p: Pt) -> float:
    best = math.inf
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy))
    return best


def area(poly: list[Pt]) -> float:
    return 0.5 * sum(ax * by - bx * ay for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]))


def problems(pl: Plate, cfg: dict) -> list[str]:
    """Every hole inside its plate with WEB_MM of MDF to the edge; bolts clear of the stones and the cones; the
    outline counter-clockwise; the plate fits the laser bed ([guides] sheet, sheet_margin) in one orientation."""
    gd = cfg["guides"]
    out = [] if area(pl.outline) > 0 else [f"{pl.name}: outline not counter-clockwise"]
    sl, sw = float(cfg["brick"]["length"]), float(cfg["brick"]["width"])
    cone_r = float(gd["socket_r_mouth"]) - float(gd["socket_clearance"])
    for kind, pts, d in (("cone", pl.cones, float(gd["peg_hole_d"])),
                         ("bolt", pl.bolts, float(cfg["deck_plate"]["bolt_hole_d"]))):
        for q in pts:
            if not inside(pl.outline, q) or edge_dist(pl.outline, q) < d / 2 + WEB_MM:
                out.append(f"{pl.name}: {kind} hole at ({q[0]:.1f}, {q[1]:.1f}) outside / closer than {WEB_MM} mm "
                           "to the edge")
    for x, y in pl.bolts:
        for _, (hx, hy) in pl.holders:
            if abs(x - hx) < sw / 2 + 10.0 and abs(y - hy) < sl / 2 + 10.0:
                out.append(f"{pl.name}: bolt hole at ({x:.1f}, {y:.1f}) under a stone")
        if any(math.hypot(x - qx, y - qy) < cone_r + 10.0 for qx, qy in pl.cones):
            out.append(f"{pl.name}: bolt hole at ({x:.1f}, {y:.1f}) under a cone")
    a, dp = cfg["ares"], cfg["deck_plate"]
    xt = float(a["length"]) / 2 - float(a["deck_corner_r"]) - float(dp["corner_bolt_clear"])   # side holes |x| <= xt
    yt = float(a["width"]) / 2 - float(a["deck_corner_r"]) - float(dp["corner_bolt_clear"])    # front / rear |y| <= yt
    e = float(dp["bolt_edge_mm"])
    for x, y in pl.bolts:
        on_side = abs(abs(y) - (float(a["width"]) / 2 - e)) < 1e-6
        on_end = abs(abs(x) - (float(a["length"]) / 2 - e)) < 1e-6
        if (on_side and abs(x) > xt + 1e-6) or (on_end and abs(y) > yt + 1e-6):
            out.append(f"{pl.name}: bolt hole at ({x:.1f}, {y:.1f}) closer than {dp['corner_bolt_clear']} mm to a "
                       "corner radius (corner connector)")
    xs, ys = [q[0] for q in pl.outline], [q[1] for q in pl.outline]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    bw, bh = (float(v) - 2 * float(gd.get("sheet_margin", 0.0)) for v in gd["sheet"])
    if not ((w <= bw and h <= bh) or (w <= bh and h <= bw)):
        out.append(f"{pl.name}: {w:.0f} x {h:.0f} mm does not fit the bed {bw:.0f} x {bh:.0f} (sheet - margins)")
    return out


# ── output ────────────────────────────────────────────────────────────────────
def dxf(pl: Plate, cfg: dict) -> mp.Dxf:
    """Cut / engrave with the plate's lower-left corner at (0, 0) (laser software places files by their extents)."""
    gd = cfg["guides"]
    k = float(gd.get("kerf_mm", 0.0))
    ox, oy = -min(q[0] for q in pl.outline) + k, -min(q[1] for q in pl.outline) + k
    T = lambda q: (q[0] + ox, q[1] + oy)                                       # noqa: E731
    d = mp.Dxf()
    d.poly(mp.CUT, [T(q) for q in offset(pl.outline, k / 2)])
    for c in pl.cones:
        d.circle(mp.CUT, T(c), (float(gd["peg_hole_d"]) - k) / 2)
    for c in pl.bolts:
        d.circle(mp.CUT, T(c), (float(cfg["deck_plate"]["bolt_hole_d"]) - k) / 2)
    sl, sw = float(cfg["brick"]["length"]), float(cfg["brick"]["width"])
    ym = max(q[1] for q in pl.outline) - 2.0          # the stones overhang the sides: engrave inside only
    for name, (x, y) in pl.holders:
        y0, y1 = max(y - sl / 2, -ym), min(y + sl / 2, ym)
        d.poly(mp.ENGRAVE, [T(q) for q in ((x - sw / 2, y0), (x + sw / 2, y0), (x + sw / 2, y1), (x - sw / 2, y1))])
        d.text(mp.ENGRAVE, T((x - 4.0, y - 3.0 * len(name))), 6.0, name, rot_deg=90.0)
    for a, b in pl.lines:
        d.line(mp.ENGRAVE, T(a), T(b))
    for q, h, s in pl.texts:
        d.text(mp.ENGRAVE, T(q), h, s, rot_deg=90.0)
    return d


def preview(pls: list[Plate], cfg: dict, path: Path, px_mm: float = 1.0) -> None:
    """Top: magazine + rear as mounted on the deck (grey); bottom: onepiece."""
    from PIL import Image, ImageDraw
    a = cfg["ares"]
    L, W, R = float(a["length"]), float(a["width"]), float(a["deck_corner_r"])
    gap = 60.0
    x0, y_top = -L / 2 - 20, W / 2 + 20
    img = Image.new("RGB", (int((L + 40) * px_mm), int((2 * (W + 40) + gap) * px_mm)), "white")
    dr = ImageDraw.Draw(img)
    sl, sw = float(cfg["brick"]["length"]), float(cfg["brick"]["width"])
    pd, bd = float(cfg["guides"]["peg_hole_d"]), float(cfg["deck_plate"]["bolt_hole_d"])
    deck = [(L / 2, -W / 2), (L / 2, W / 2)] + arc(-L / 2 + R, W / 2 - R, R, 90, 180) \
        + arc(-L / 2 + R, -W / 2 + R, R, 180, 270)
    for row, group in enumerate(([p for p in pls if p.name != "onepiece"], [p for p in pls if p.name == "onepiece"])):
        dy = row * (W + 40 + gap)
        P = lambda x, y: ((x - x0) * px_mm, (y_top - y + dy) * px_mm)          # noqa: E731
        dr.line([P(*q) for q in deck + deck[:1]], fill=(170, 170, 170), width=1)
        dr.text(P(L / 2 - 230, -W / 2 + 15), "ARES deck (grey)   FRONT ->", fill=(120, 120, 120))
        for pl in group:
            dr.line([P(*q) for q in pl.outline + pl.outline[:1]], fill="red", width=2)
            ym = max(q[1] for q in pl.outline) - 2.0
            for name, (x, y) in pl.holders:
                dr.rectangle([P(x - sw / 2, min(y + sl / 2, ym)), P(x + sw / 2, max(y - sl / 2, -ym))],
                             outline="blue")
                dr.text(P(x - 12, y + 5), name, fill="blue")
            for (x, y), dd in [(c, pd) for c in pl.cones] + [(c, bd) for c in pl.bolts]:
                dr.ellipse([P(x - dd / 2, y + dd / 2), P(x + dd / 2, y - dd / 2)], outline="red", width=2)
            xs = [q[0] for q in pl.outline]
            dr.text(P((min(xs) + max(xs)) / 2 - 25, 25), pl.name, fill="black")
            for (tx, ty), h, txt in pl.texts:
                if h < 10.0:
                    continue
                from PIL import ImageFont
                try:
                    font = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", int(h * 1.4 * px_mm))
                except OSError:
                    font = ImageFont.load_default()
                im = Image.new("RGBA", (int(CAP_W * h * len(txt) * px_mm * 1.3), int(h * 1.6 * px_mm)), (0, 0, 0, 0))
                ImageDraw.Draw(im).text((0, 0), txt, fill=(0, 0, 255, 255), font=font)
                im = im.rotate(90, expand=True)                 # reading along +y = up in the picture
                bx, by = P(tx, ty)
                img.paste(im, (int(bx - h * 1.25 * px_mm), int(by - im.size[1])), im)
    img.save(path)


def main() -> int:
    cfg = config.load()
    pls = plates(cfg)
    out = ROOT / "targets" / "deck_plate"
    out.mkdir(parents=True, exist_ok=True)
    for old in ("deck_plate.dxf", "deck_plate.png"):    # the first single plate (2026-10-08) = "onepiece" now
        (out / old).unlink(missing_ok=True)
    bad = []
    for pl in pls:
        bad += problems(pl, cfg)
        dxf(pl, cfg).write(out / f"deck_plate_{pl.name}.dxf")
        xs, ys = [q[0] for q in pl.outline], [q[1] for q in pl.outline]
        print(f"written deck_plate_{pl.name}.dxf: x {min(xs):g}..{max(xs):g}, y {min(ys):g}..{max(ys):g} "
              f"({max(xs) - min(xs):g} x {max(ys) - min(ys):g} mm), {len(pl.cones)} peg holes, {len(pl.bolts)} M6")
    preview(pls, cfg, out / "deck_plates.png")
    for b in bad:
        print("PROBLEM:", b)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
