"""Laser-cut MDF carrier plates for the printed ChArUco boards: DXF (R12, mm) + a PNG preview.

    py.exe tools/make_plates.py [--out targets]

Per board <name>_plate.dxf plus plates_all.dxf (all plates on one sheet, 10 mm apart) and plates_preview.png.
Layers: CUT (red, colour 1) = outline, notches, holes; ENGRAVE (blue, colour 5) = where to glue the print (board outline,
paper quiet zone, origin crosshair and x/y arrows exactly like on the PDF), labels. Engrave on the top face.

Wall plates ([[targets]] parent "wall"): the plate lies face up on the floor between the ARES front and the wall. Its
wall-side edge (top edge in the DXF) rests against the side faces of the female-pin base blocks of course 0; the two
V-notches in that edge line up with the joints of one block ([plates] block_length, PLACEHOLDER) -> the board position
in the wall frame is fixed by the blocks, no survey needed. DXF view = from above with the wall at the top
(X = -(u - u_centre), Y = -(y - y_centre) in wall coordinates), so the print lies rotated by 180 deg relative to
reading direction: align its crosshair and arrows with the engraved ones.
Station and calib plates: plain plates (station layout unknown), print glued on the engraved marks.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from mauer import config, geometry as g  # noqa: E402
from mauer.vision.printables import trim_margin_mm  # noqa: E402
from mauer.vision.targets import board_specs  # noqa: E402

CUT, ENGRAVE = "CUT", "ENGRAVE"
ACI = {CUT: 1, ENGRAVE: 5}        # AutoCAD colour index 1 = pure red (255, 0, 0), 5 = pure blue (0, 0, 255)


class Dxf:
    """Minimal DXF R12 writer: LINE, CIRCLE, TEXT on two layers (units mm). Every entity carries its colour
    explicitly (code 62), not only by layer, because laser software often maps colours per entity."""

    def __init__(self) -> None:
        self.ents: list[str] = []

    def line(self, layer: str, a, b) -> None:
        self.ents.append(f"0\nLINE\n8\n{layer}\n62\n{ACI[layer]}\n10\n{a[0]:.4f}\n20\n{a[1]:.4f}\n30\n0.0\n"
                         f"11\n{b[0]:.4f}\n21\n{b[1]:.4f}\n31\n0.0\n")

    def poly(self, layer: str, pts, closed: bool = True) -> None:
        pts = list(pts)
        for a, b in zip(pts, pts[1:] + (pts[:1] if closed else [])):
            self.line(layer, a, b)

    def circle(self, layer: str, c, r: float) -> None:
        self.ents.append(f"0\nCIRCLE\n8\n{layer}\n62\n{ACI[layer]}\n10\n{c[0]:.4f}\n20\n{c[1]:.4f}\n30\n0.0\n40\n{r:.4f}\n")

    def text(self, layer: str, p, h: float, s: str) -> None:
        self.ents.append(f"0\nTEXT\n8\n{layer}\n62\n{ACI[layer]}\n10\n{p[0]:.4f}\n20\n{p[1]:.4f}\n30\n0.0\n40\n{h:.4f}\n1\n{s}\n")

    def write(self, path: Path) -> None:
        layers = "".join(f"0\nLAYER\n2\n{n}\n70\n0\n62\n{c}\n6\nCONTINUOUS\n" for n, c in ACI.items())
        path.write_text("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n0\nENDSEC\n"
                        f"0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n2\n{layers}0\nENDTAB\n0\nENDSEC\n"
                        "0\nSECTION\n2\nENTITIES\n" + "".join(self.ents) + "0\nENDSEC\n0\nEOF\n", encoding="ascii")


class Plate:
    """Plate geometry in plate coordinates (mm, DXF X right / Y up, origin = plate centre)."""

    def __init__(self, name: str, w: float, h: float) -> None:
        self.name, self.w, self.h = name, w, h
        self.cut_polys: list[list] = []
        self.cut_circles: list[tuple] = []
        self.eng_lines: list[tuple] = []
        self.texts: list[tuple] = []

    def emit(self, dxf: Dxf, off=(0.0, 0.0)) -> None:
        o = np.asarray(off, float)
        for pts in self.cut_polys:
            dxf.poly(CUT, [tuple(np.asarray(p) + o) for p in pts])
        for c, r in self.cut_circles:
            dxf.circle(CUT, tuple(np.asarray(c) + o), r)
        for a, b in self.eng_lines:
            dxf.line(ENGRAVE, tuple(np.asarray(a) + o), tuple(np.asarray(b) + o))
        for p, hgt, s in self.texts:
            dxf.text(ENGRAVE, tuple(np.asarray(p) + o), hgt, s)


def board_marks(plate: Plate, spec, to_plate, q: float) -> None:
    """Engrave board outline, paper cut-out (board + margin q, as cut along the PDF's dashed line), origin crosshair and
    x/y arrows (as on the PDF). to_plate maps board coordinates (mm, x right / y down on the print) to plate
    coordinates."""
    bw, bh = spec.size_mm
    arm = min(12.0, q - 1.0)
    P = lambda x, y: tuple(to_plate(x, y))                                     # noqa: E731
    rect = lambda x0, y0, x1, y1: [P(x0, y0), P(x1, y0), P(x1, y1), P(x0, y1)]  # noqa: E731
    for pts in (rect(0, 0, bw, bh), rect(-q, -q, bw + q, bh + q)):
        plate.eng_lines += list(zip(pts, pts[1:] + pts[:1]))
    plate.eng_lines += [(P(-arm, 0.0), P(-1.5, 0.0)), (P(0.0, -arm), P(0.0, -1.5))]     # crosshair outside the corner
    for (dx, dy), lab in (((1, 0), "x"), ((0, 1), "y")):                          # arrows along the board edges
        a, b = P(0.0, 0.0), P(28.0 * dx, 28.0 * dy)
        plate.eng_lines.append((a, b))
        tip = np.asarray(b)
        d = (tip - np.asarray(a)) / 28.0
        n = np.array([-d[1], d[0]])
        plate.eng_lines += [(tuple(tip), tuple(tip - 3.0 * d + 1.5 * n)), (tuple(tip), tuple(tip - 3.0 * d - 1.5 * n))]
        plate.texts.append((tuple(tip + 5.0 * d - np.array([1.0, 1.5])), 3.0, lab))


def wall_plate(cfg: dict, spec, T_wall_board: np.ndarray) -> Plate:
    pc = cfg["plates"]
    face_y = pc["block_width"] / 2.0
    L, D = pc["plate_length"], pc["plate_depth"]
    centre = g.apply(T_wall_board, [[*spec.centre_mm, 0.0]])[0]
    u_c, y_c = float(centre[0]), face_y + D / 2.0
    if abs(float(centre[1]) - y_c) > 0.5:
        raise ValueError(f"{spec.name}: board centre y = {centre[1]:.1f} mm, plate centre y = {y_c:.1f} mm - "
                         "[[targets]] and [plates] disagree")
    to_plate_w = lambda u, y: np.array([-(u - u_c), -(y - y_c)])                 # noqa: E731  wall -> DXF top view

    def to_plate(x, y):                                                          # board -> wall -> plate
        p = g.apply(T_wall_board, [[x, y, 0.0]])[0]
        return to_plate_w(p[0], p[1])

    pl = Plate(spec.name, L, D)
    # outline with V-notches in the wall-side (top) edge at the block joints inside the plate
    bl, nw, nd = pc["block_length"], pc["notch_width"], pc["notch_depth"]
    joints = [k * bl for k in range(int(np.floor((u_c - L / 2) / bl)) - 1, int(np.ceil((u_c + L / 2) / bl)) + 2)
              if abs(k * bl - u_c) < L / 2 - nw]
    if len(joints) < 2:
        print(f"WARNING {spec.name}: only {len(joints)} block joint(s) under the plate", flush=True)
    xs = sorted(float(to_plate_w(j, 0.0)[0]) for j in joints)
    top = [(L / 2, D / 2)]
    for x in sorted(xs, reverse=True):
        top += [(x + nw / 2, D / 2), (x, D / 2 - nd), (x - nw / 2, D / 2)]
    top += [(-L / 2, D / 2)]
    pl.cut_polys.append(top + [(-L / 2, -D / 2), (L / 2, -D / 2)])
    m = trim_margin_mm(spec, pc.get("print_scale", 1.0))
    qz = spec.size_mm[0] / 2 + m                                                 # half cut-out width along X
    hx = (qz + L / 2) / 2
    for sx in (-1, 1):
        pl.cut_circles.append(((sx * hx, -8.0), pc["finger_hole_d"] / 2))
    board_marks(pl, spec, to_plate, m)
    # labels outside the paper quiet zone (|X| > qz): joint marks under the notches, name and orientation beside
    for x in xs:
        pl.texts.append(((x - 4.5 if x < 0 else x - 10.5, D / 2 - nd - 5.0), 3.0, "joint"))
    side = -L / 2 + 4.0
    pl.texts.append(((side, D / 2 - 18.0), 3.5, "WALL ^"))
    pl.texts.append(((side, -D / 2 + 26.0), 3.5, f"{spec.name}"))
    pl.texts.append(((side, -D / 2 + 20.0), 3.0, f"ids {spec.first_id}-{spec.last_id}"))
    pl.texts.append(((side, -D / 2 + 14.0), 3.0, f"u = {u_c:.0f} mm"))
    pl.texts.append(((side, -D / 2 + 5.0), 3.5, "ARES v"))
    return pl


def plain_plate(cfg: dict, spec) -> Plate:
    m = cfg["plates"]["margin"]
    bw, bh = spec.size_mm
    q = trim_margin_mm(spec, cfg["plates"].get("print_scale", 1.0))
    W, H = bw + 2 * q + 2 * m, bh + 2 * q + 2 * m + 8.0                          # + 8 mm for the label
    pl = Plate(spec.name, W, H)
    pl.cut_polys.append([(-W / 2, -H / 2), (W / 2, -H / 2), (W / 2, H / 2), (-W / 2, H / 2)])
    to_plate = lambda x, y: np.array([x - bw / 2, -(y - bh / 2) + 4.0])         # noqa: E731  print as read
    board_marks(pl, spec, to_plate, q)
    pl.texts.append(((-W / 2 + 4.0, -H / 2 + 4.0), 4.0, f"{spec.name} ids {spec.first_id}-{spec.last_id}"))
    return pl


def write_svg(plates: list[tuple[Plate, tuple]], path: Path, line_w: float = 0.3) -> None:
    """SVG (mm) for laser software that raster-engraves blue: cuts = red hairlines (#FF0000, 0.01 mm, no fill),
    engrave marks = blue FILLED strips line_w wide (#0000FF) - a raster engrave pass only fills areas, a hairline has
    none (Trotec Speedy 2026-10-05: DXF text engraved, blue lines not). Text as blue filled glyphs."""
    xmin = min(o[0] - p.w / 2 for p, o in plates) - 5
    xmax = max(o[0] + p.w / 2 for p, o in plates) + 5
    ymin = min(o[1] - p.h / 2 for p, o in plates) - 5
    ymax = max(o[1] + p.h / 2 for p, o in plates) + 5
    W, H = xmax - xmin, ymax - ymin
    S = lambda p: (p[0] - xmin, ymax - p[1])                                     # noqa: E731  DXF (y up) -> SVG
    f = lambda v: f"{v:.3f}"                                                     # noqa: E731
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{f(W)}mm" height="{f(H)}mm" viewBox="0 0 {f(W)} {f(H)}">']
    for pl, o in plates:
        o = np.asarray(o, float)
        for pts in pl.cut_polys:
            q = " ".join(f"{f(x)},{f(y)}" for x, y in (S(np.asarray(p) + o) for p in pts))
            out.append(f'<polygon points="{q}" fill="none" stroke="#FF0000" stroke-width="0.01"/>')
        for c, r in pl.cut_circles:
            x, y = S(np.asarray(c) + o)
            out.append(f'<circle cx="{f(x)}" cy="{f(y)}" r="{f(r)}" fill="none" stroke="#FF0000" stroke-width="0.01"/>')
        for a, b in pl.eng_lines:
            a, b = np.asarray(a, float) + o, np.asarray(b, float) + o
            d = b - a
            n = np.linalg.norm(d)
            if n < 1e-9:
                continue
            d /= n
            nn = np.array([-d[1], d[0]]) * line_w / 2
            a, b = a - d * line_w / 2, b + d * line_w / 2                       # square ends close the corners
            q = " ".join(f"{f(x)},{f(y)}" for x, y in (S(p) for p in (a + nn, b + nn, b - nn, a - nn)))
            out.append(f'<polygon points="{q}" fill="#0000FF" stroke="none"/>')
        for p, h, txt in pl.texts:
            x, y = S(np.asarray(p, float) + o)
            out.append(f'<text x="{f(x)}" y="{f(y)}" font-family="Arial" font-size="{f(h * 1.4)}" '
                       f'fill="#0000FF">{txt.replace("&", "&amp;").replace("<", "&lt;")}</text>')
    out.append("</svg>\n")
    path.write_text("\n".join(out), encoding="utf-8")


def preview(plates: list[tuple[Plate, tuple]], path: Path, px_per_mm: float = 4.0) -> None:
    import cv2
    xmax = max(o[0] + p.w / 2 for p, o in plates) + 10
    ymin = min(o[1] - p.h / 2 for p, o in plates) - 10
    ymax = max(o[1] + p.h / 2 for p, o in plates) + 10
    xmin = min(o[0] - p.w / 2 for p, o in plates) - 10
    W, H = int((xmax - xmin) * px_per_mm), int((ymax - ymin) * px_per_mm)
    img = np.full((H, W, 3), 255, np.uint8)
    P = lambda p: (int(round((p[0] - xmin) * px_per_mm)), int(round((ymax - p[1]) * px_per_mm)))  # noqa: E731
    for pl, o in plates:
        o = np.asarray(o)
        for pts in pl.cut_polys:
            q = [P(np.asarray(p) + o) for p in pts]
            for a, b in zip(q, q[1:] + q[:1]):
                cv2.line(img, a, b, (0, 0, 220), 2, cv2.LINE_AA)
        for c, r in pl.cut_circles:
            cv2.circle(img, P(np.asarray(c) + o), int(r * px_per_mm), (0, 0, 220), 2, cv2.LINE_AA)
        for a, b in pl.eng_lines:
            cv2.line(img, P(np.asarray(a) + o), P(np.asarray(b) + o), (200, 80, 0), 1, cv2.LINE_AA)
        for p, h, s in pl.texts:
            cv2.putText(img, s, P(np.asarray(p) + o), cv2.FONT_HERSHEY_SIMPLEX, h * px_per_mm / 22.0, (200, 80, 0), 1,
                        cv2.LINE_AA)
    cv2.imwrite(str(path), img)


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=REPO / "targets", help="output folder (default targets/)")
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args(argv)
    cfg = config.load(args.config)
    specs = board_specs(cfg)
    out = args.out if args.out.is_absolute() else REPO / args.out
    out.mkdir(parents=True, exist_ok=True)

    plates = []
    for t in cfg["targets"]:
        spec = specs[t["name"]]
        plates.append(wall_plate(cfg, spec, config.pose(t)) if t["parent"] == "wall" else plain_plate(cfg, spec))
    plates.append(plain_plate(cfg, specs["calib"]))

    for pl in plates:
        d = Dxf()
        pl.emit(d)
        d.write(out / f"{pl.name}_plate.dxf")
        write_svg([(pl, (0.0, 0.0))], out / f"{pl.name}_plate.svg")
        print(f"{pl.name}_plate.dxf  {pl.w:.0f} x {pl.h:.0f} mm", flush=True)

    # one sheet: rows of plates, 10 mm apart, max 600 mm wide
    gap, x, y, row_h, placed, sheet = 10.0, 0.0, 0.0, 0.0, [], Dxf()
    for pl in plates:
        if x + pl.w > 600.0 and x > 0:
            x, y, row_h = 0.0, y - row_h - gap, 0.0
        o = (x + pl.w / 2, y - pl.h / 2)
        pl.emit(sheet, o)
        placed.append((pl, o))
        x += pl.w + gap
        row_h = max(row_h, pl.h)
    sheet.write(out / "plates_all.dxf")
    write_svg(placed, out / "plates_all.svg")
    preview(placed, out / "plates_preview.png")
    print(f"plates_all.dxf + .svg ({len(plates)} plates), plates_preview.png -> {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
