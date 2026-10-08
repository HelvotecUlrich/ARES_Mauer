"""Magazine plate on the ARES deck (2026-10-08): one 4 mm MDF plate behind the UR with the peg holes of the locating
cones of every magazine holder, screwed to the deck with M6 / slot nuts 20 mm from the ARES outer edge.

    py.exe tools/make_deck_plate.py        # -> targets/deck_plate/deck_plate.dxf + deck_plate.png (preview)

Everything comes from config/station.toml: the deck outline ([ares] length, width, deck_corner_r), the plate
([deck_plate]), the holders ([ur5] mount_x + [deck] magazine_rows_dx, magazine_y; stones with their long axis along
ARES y as tools/make_job.py _magazine), the cones ([guides] pin_along / pin_across: 4 per holder at the sockets of a
full stone; a half-stone holder ([deck] half_positions) takes two halves end to end on the same 4 cones) and the
holes ([guides] peg_hole_d, kerf_mm: cut circles kerf smaller, outline kerf/2 outside, as tools/make_guides.py).
Drawing in the ARES frame (x forward, y left; DXF x = ARES x, DXF y = ARES y), origin = base_link. Layers CUT red /
ENGRAVE blue (tools/make_plates.py). Engraved: stone footprints, holder names, FRONT arrow, ARES centreline, UR axis.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import make_plates as mp  # noqa: E402

from mauer import config  # noqa: E402

Pt = tuple[float, float]
ARC_SEG_DEG = 3.0           # polyline segment of the rounded corners


def rounded_rect(x0: float, x1: float, y0: float, y1: float, r_rear: float) -> list[Pt]:
    """Outline with rounded REAR corners (x = x0) of radius r_rear, square front corners (x = x1), counter-clockwise."""
    pts: list[Pt] = [(x1, y0), (x1, y1)]
    n = max(2, int(round(90.0 / ARC_SEG_DEG)))
    for cx, cy, a0 in ((x0 + r_rear, y1 - r_rear, 90.0), (x0 + r_rear, y0 + r_rear, 180.0)):
        for i in range(n + 1):
            a = math.radians(a0 + 90.0 * i / n)
            pts.append((cx + r_rear * math.cos(a), cy + r_rear * math.sin(a)))
    return pts


def plate(cfg: dict) -> dict:
    a, dp, dk, gd, u = cfg["ares"], cfg["deck_plate"], cfg["deck"], cfg["guides"], cfg["ur5"]
    L, W, R = float(a["length"]), float(a["width"]), float(a["deck_corner_r"])
    ins, kerf = float(dp["inset_mm"]), float(gd.get("kerf_mm", 0.0))
    x_rear, x_front, y_half = -L / 2 + ins, float(dp["front_x"]), W / 2 - ins
    outline = rounded_rect(x_rear, x_front, -y_half, y_half, R - ins)
    holders, cones = [], []
    a_u, a_v = float(gd["pin_along"]) / 2, float(gd["pin_across"]) / 2
    half = {str(p) for p in dk.get("half_positions", [])}
    for ri, dx in enumerate(dk["magazine_rows_dx"]):
        for yi, y in enumerate(dk["magazine_y"]):
            x = float(u["mount_x"]) + float(dx)
            name = f"r{ri}y{yi}" + (" half" if f"r{ri}y{yi}" in half else "")
            holders.append((name, (x, float(y))))
            cones += [(x + sv * a_v, float(y) + su * a_u) for su in (-1, 1) for sv in (-1, 1)]   # long axis along y
    e = float(dp["bolt_edge_mm"])
    bolts = [(-L / 2 + e, float(y)) for y in dp["bolt_rear_y"]]
    bolts += [(float(x), s * (W / 2 - e)) for x in dp["bolt_side_x"] for s in (-1, 1)]
    return dict(outline=outline, holders=holders, cones=cones, bolts=bolts, kerf=kerf,
                peg_d=float(gd["peg_hole_d"]), bolt_d=float(dp["bolt_hole_d"]), x_rear=x_rear, x_front=x_front,
                y_half=y_half, mount_x=float(u["mount_x"]), stone=(float(cfg["brick"]["length"]),
                                                                 float(cfg["brick"]["width"])))


def problems(pl: dict, cfg: dict) -> list[str]:
    """Geometry checks: every hole inside the plate with an edge web, bolts clear of the stones and the cones."""
    out = []
    web = 3.0
    cone_r = float(cfg["guides"]["socket_r_mouth"]) - float(cfg["guides"]["socket_clearance"])
    sl, sw = pl["stone"]
    for kind, pts, d in (("cone", pl["cones"], pl["peg_d"]), ("bolt", pl["bolts"], pl["bolt_d"])):
        for x, y in pts:
            if not (pl["x_rear"] + d / 2 + web <= x <= pl["x_front"] - d / 2 - web
                    and abs(y) <= pl["y_half"] - d / 2 - web):
                out.append(f"{kind} hole at ({x:.1f}, {y:.1f}) closer than {web} mm to the plate edge / outside")
    r_corner = float(cfg["ares"]["deck_corner_r"]) - float(cfg["deck_plate"]["inset_mm"])
    cx = pl["x_rear"] + r_corner
    for x, y in pl["bolts"]:
        cy = math.copysign(pl["y_half"] - r_corner, y)
        if x < cx and abs(y) > abs(cy) and math.hypot(x - cx, y - cy) > r_corner - pl["bolt_d"] / 2 - web:
            out.append(f"bolt hole at ({x:.1f}, {y:.1f}) outside the rounded corner")
        for _, (hx, hy) in pl["holders"]:
            if abs(x - hx) < sw / 2 + 10.0 and abs(y - hy) < sl / 2 + 10.0:
                out.append(f"bolt hole at ({x:.1f}, {y:.1f}) under a stone (holder at {hx:.1f}, {hy:.1f})")
        for qx, qy in pl["cones"]:
            if math.hypot(x - qx, y - qy) < cone_r + 10.0:
                out.append(f"bolt hole at ({x:.1f}, {y:.1f}) under a cone")
    return out


def offset(poly: list[Pt], d: float) -> list[Pt]:
    """Polygon (counter-clockwise) moved d outward (miter), for the kerf compensation of the outline."""
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


def dxf(pl: dict) -> mp.Dxf:
    """Cut / engrave in plate coordinates: ARES (x, y) shifted so the plate starts at (0, 0) - laser software places
    files by their extents; the engraved centreline / FRONT arrow / UR note keep the orientation on ARES."""
    d = mp.Dxf()
    k = pl["kerf"]
    ox, oy = -pl["x_rear"] + k, pl["y_half"] + k
    T = lambda q: (q[0] + ox, q[1] + oy)                                       # noqa: E731
    d.poly(mp.CUT, [T(q) for q in offset(pl["outline"], k / 2)])
    for c in pl["cones"]:
        d.circle(mp.CUT, T(c), (pl["peg_d"] - k) / 2)
    for c in pl["bolts"]:
        d.circle(mp.CUT, T(c), (pl["bolt_d"] - k) / 2)
    sl, sw = pl["stone"]
    ym = pl["y_half"] - 2.0                     # the stones overhang the plate at the sides: engrave inside only
    for name, (x, y) in pl["holders"]:
        y0, y1 = max(y - sl / 2, -ym), min(y + sl / 2, ym)
        d.poly(mp.ENGRAVE, [T(q) for q in ((x - sw / 2, y0), (x + sw / 2, y0), (x + sw / 2, y1), (x - sw / 2, y1))])
        d.text(mp.ENGRAVE, T((x - 4.0, y - 3.0 * len(name))), 6.0, name, rot_deg=90.0)
    xr, xf = pl["x_rear"], pl["x_front"]
    d.line(mp.ENGRAVE, T((xr + 15.0, 0.0)), T((xf - 15.0, 0.0)))               # ARES centreline (y = 0)
    d.line(mp.ENGRAVE, T((xf - 15.0, 0.0)), T((xf - 30.0, 8.0)))               # FRONT arrow head
    d.line(mp.ENGRAVE, T((xf - 15.0, 0.0)), T((xf - 30.0, -8.0)))
    d.text(mp.ENGRAVE, T((xf - 35.0, -100.0)), 7.0, "FRONT", rot_deg=90.0)
    d.text(mp.ENGRAVE, T((xr + 20.0, -200.0)), 5.0,
           f"ARES deck plate - UR axis {pl['mount_x'] - xf:.0f} mm ahead of the front edge", rot_deg=90.0)
    return d


def preview(pl: dict, path: Path, px_mm: float = 1.5) -> None:
    from PIL import Image, ImageDraw
    x0, x1, yh = pl["x_rear"] - 20, pl["x_front"] + 20, pl["y_half"] + 20
    W, H = int((x1 - x0) * px_mm), int(2 * yh * px_mm)
    img = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(img)
    P = lambda x, y: ((x - x0) * px_mm, (yh - y) * px_mm)                     # noqa: E731
    dr.line([P(*q) for q in pl["outline"] + pl["outline"][:1]], fill="red", width=2)
    sl, sw = pl["stone"]
    ym = pl["y_half"] - 2.0
    for name, (x, y) in pl["holders"]:
        dr.rectangle([P(x - sw / 2, min(y + sl / 2, ym)), P(x + sw / 2, max(y - sl / 2, -ym))], outline="blue")
        dr.text(P(x - 10, y + 5), name, fill="blue")
    for (x, y), d in [(c, pl["peg_d"]) for c in pl["cones"]] + [(c, pl["bolt_d"]) for c in pl["bolts"]]:
        dr.ellipse([P(x - d / 2, y + d / 2), P(x + d / 2, y - d / 2)], outline="red", width=2)
    dr.line([P(pl["x_rear"] + 15, 0), P(pl["x_front"] - 15, 0)], fill="blue")
    dr.text(P(pl["x_front"] - 70, 15), "FRONT ->", fill="blue")
    img.save(path)


def main() -> int:
    cfg = config.load()
    pl = plate(cfg)
    bad = problems(pl, cfg)
    for b in bad:
        print("PROBLEM:", b)
    out = ROOT / "targets" / "deck_plate"
    out.mkdir(parents=True, exist_ok=True)
    dxf(pl).write(out / "deck_plate.dxf")
    preview(pl, out / "deck_plate.png")
    print(f"written {out / 'deck_plate.dxf'}: plate x {pl['x_rear']:g}..{pl['x_front']:g}, y +-{pl['y_half']:g} "
          f"({pl['x_front'] - pl['x_rear']:g} x {2 * pl['y_half']:g} mm), {len(pl['cones'])} peg holes "
          f"d {pl['peg_d']:g}, {len(pl['bolts'])} M6 holes d {pl['bolt_d']:g} (finished; kerf {pl['kerf']:g} comp.)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
