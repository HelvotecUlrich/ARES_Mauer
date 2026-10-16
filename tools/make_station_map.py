"""Station and magazine plan (German, PDF): the slot names the software uses, to write them onto the pick-up station
and to load the ARES magazine by the stone list.

    py.exe tools/make_station_map.py [--job data/jobs/nominal_C_robodk.json] [--out results/stationsplan.pdf] [--final]

Everything comes from the job (tools/make_job.py: job.station / job.magazine slots with their ids, stacks, layers,
types and positions; the dock pose) and config/station.toml (stone sizes [brick] / [half_brick], station area
[pickup_station] table_size, the S0 / S1 plates [[targets]] parent "station" as tools/make_guides.py cuts their
windows, ARES outline [ares], UR plate [deck_plate] ur_plate, magazine plate [deck_plate] split_x / front_x):
- page 1: pick-up station from above, ARES docked at the bottom (its UR end towards the station), every stack with
  its name (s00.. full stones, h00.. half stones), type and the layers the job uses; station frame axes;
- page 2: ARES deck from above with every magazine stack (r<row>y<position>[a|b]), its layers and the start fill;
- page 3: both as tables with every slot id (stack + l<layer>, l1 = bottom), the unused layers and the naming.
The slot id the sequencer logs and the HMI shows is <stack>l<layer>, e.g. s01l5 = stack s01, layer 5 (the top one),
r1y0al1 = magazine row 1, position y0, half-stone stack a, layer 1. tools/make_stone_list.py puts the same pages into
the stone list (which names these slots for every stone).
"""
from __future__ import annotations

import argparse
import math
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO, REPO / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from mauer import config as mconfig  # noqa: E402
from mauer import job as mjob  # noqa: E402

PT_PER_MM = 72.0 / 25.4
KIND_DE = {"full": "Voll", "half": "Halb", "": "Voll/Halb"}


# ── data ─────────────────────────────────────────────────────────────────────
def _layers_txt(layers: list[int]) -> str:
    """[1, 2, 3] -> 'l1–l3', [2] -> 'l2', [] -> '–'."""
    if not layers:
        return "–"
    ls = sorted(layers)
    if ls == list(range(ls[0], ls[-1] + 1)) and len(ls) > 1:
        return f"l{ls[0]}–l{ls[-1]}"
    return ", ".join(f"l{v}" for v in ls)


def station_stacks(job: mjob.Job) -> list[dict]:
    """Every stack of the pick-up station: {"stack", "kind", "x_mm", "y_mm" (station frame, stone centre), "row"
    (1 = nearest to ARES), "used" [layers in the job's take order], "unused" [layers the UR cannot reach - left
    empty], "ids" [slot ids of the used layers, bottom first], "axis" (unit vector of the stone length)}, sorted by
    row and x."""
    st = job.station
    take = set(st.take_order)
    ys = sorted({round(float(s.T_station_tcp[1, 3]), 1) for s in st.slots})
    out = []
    for sid in sorted({s.stack_id for s in st.slots}):
        sl = sorted((s for s in st.slots if s.stack_id == sid), key=lambda s: s.layer)
        T = np.asarray(sl[0].T_station_tcp, float)
        out.append({"stack": sid, "kind": sl[0].kind, "x_mm": float(T[0, 3]), "y_mm": float(T[1, 3]),
                    "row": ys.index(round(float(T[1, 3]), 1)) + 1, "axis": (float(T[0, 0]), float(T[1, 0])),
                    "used": [s.layer for s in sl if s.id in take], "unused": [s.layer for s in sl if s.id not in take],
                    "ids": [s.id for s in sl if s.id in take]})
    return sorted(out, key=lambda s: (s["row"], s["x_mm"]))


def magazine_stacks(job: mjob.Job) -> list[dict]:
    """Every magazine stack on ARES: {"stack", "kind", "x_mm", "y_mm" (ARES frame of this repo: x towards the UR end),
    "used", "unused", "ids", "start" [layers filled at the start, job.magazine.initial_fill], "start_kinds" {layer:
    kind}, "axis"}, sorted by stack name."""
    m = job.magazine
    take, init = set(m.take_order), set(m.initial_fill)
    out = []
    for sid in sorted({s.stack for s in m.slots}):
        sl = sorted((s for s in m.slots if s.stack == sid), key=lambda s: s.layer)
        T = np.asarray(sl[0].T_ares_tcp, float)
        out.append({"stack": sid, "kind": sl[0].kind, "x_mm": float(T[0, 3]), "y_mm": float(T[1, 3]),
                    "axis": (float(T[0, 0]), float(T[1, 0])),
                    "used": [s.layer for s in sl if s.id in take], "unused": [s.layer for s in sl if s.id not in take],
                    "ids": [s.id for s in sl if s.id in take], "start": [s.layer for s in sl if s.id in init],
                    "start_kinds": {s.layer: m.initial_kinds.get(s.id, "full") for s in sl if s.id in init}})
    return out


def stone_size(cfg: dict, kind: str) -> tuple[float, float]:
    """(length, width) of a stone of `kind` ([brick] / [half_brick])."""
    b = cfg["half_brick"] if kind == "half" and "half_brick" in cfg else cfg["brick"]
    return float(b["length"]), float(b["width"])


def station_plates(cfg: dict) -> list[tuple[str, tuple[float, float], tuple[float, float]]]:
    """(name, plate centre, plate size) of the S0 / S1 plates in the station frame, as tools/make_guides.py cuts their
    windows."""
    import make_guides
    return make_guides._station_plates(cfg)


# ── drawing helpers (reportlab graphics, vector) ──────────────────────────────
def _clip(pts: list, box: tuple[float, float, float, float]) -> list:
    """Sutherland-Hodgman: polygon `pts` clipped to the axis-parallel box (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = box
    edges = [(lambda p: p[0] >= x0, lambda a, b: _cut_x(a, b, x0)),
             (lambda p: p[0] <= x1, lambda a, b: _cut_x(a, b, x1)),
             (lambda p: p[1] >= y0, lambda a, b: _cut_y(a, b, y0)),
             (lambda p: p[1] <= y1, lambda a, b: _cut_y(a, b, y1))]
    out = list(pts)
    for inside, cut in edges:
        src, out = out, []
        for i, cur in enumerate(src):
            prev = src[i - 1]
            if inside(cur):
                if not inside(prev):
                    out.append(cut(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cut(prev, cur))
        if not out:
            break
    return out


def _cut_x(a, b, x):
    t = (x - a[0]) / (b[0] - a[0])
    return (x, a[1] + t * (b[1] - a[1]))


def _cut_y(a, b, y):
    t = (y - a[1]) / (b[1] - a[1])
    return (a[0] + t * (b[0] - a[0]), y)


def _rounded_rect(L: float, W: float, r: float, n: int = 6) -> list[tuple[float, float]]:
    """Outline of an L x W rectangle centred at the origin with corner radius r."""
    pts = []
    for cx, cy, a0 in ((L / 2 - r, W / 2 - r, 0), (-L / 2 + r, W / 2 - r, 90), (-L / 2 + r, -W / 2 + r, 180),
                       (L / 2 - r, -W / 2 + r, 270)):
        for i in range(n + 1):
            a = math.radians(a0 + 90.0 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _rect(cx: float, cy: float, L: float, W: float, axis=(1.0, 0.0)) -> list[tuple[float, float]]:
    """Corners of an L x W rectangle centred at (cx, cy), its length along `axis`."""
    ax, ay = axis
    n = math.hypot(ax, ay) or 1.0
    ax, ay = ax / n, ay / n
    return [(cx + u * ax - v * ay, cy + u * ay + v * ax) for u, v in
            ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2))]


def _tf(T: np.ndarray, pts) -> list[tuple[float, float]]:
    """2-D points through the 4x4 transform T."""
    return [(float(T[0, 0] * x + T[0, 1] * y + T[0, 3]), float(T[1, 0] * x + T[1, 1] * y + T[1, 3])) for x, y in pts]


class View:
    """World (mm, a top view: x right, y up) -> reportlab Drawing (pt), the box (x0, y0, x1, y1) fitted into
    max_w x max_h points."""

    def __init__(self, box: tuple[float, float, float, float], max_w: float, max_h: float):
        from reportlab.graphics.shapes import Drawing
        self.box = box
        x0, y0, x1, y1 = box
        self.s = min(max_w / (x1 - x0), max_h / (y1 - y0))           # pt per mm of the world
        self.d = Drawing((x1 - x0) * self.s, (y1 - y0) * self.s)

    @property
    def scale_n(self) -> float:
        """The drawing scale 1 : n on paper printed at 100 %."""
        return PT_PER_MM / self.s

    def p(self, x: float, y: float) -> tuple[float, float]:
        return ((x - self.box[0]) * self.s, (y - self.box[1]) * self.s)

    def poly(self, pts, *, fill=None, stroke=None, width: float = 0.8, dash=None, clip: bool = True) -> None:
        from reportlab.graphics.shapes import Polygon
        from reportlab.lib import colors
        pts = _clip(pts, self.box) if clip else list(pts)
        if len(pts) < 3:
            return
        flat = [c for q in pts for c in self.p(*q)]
        self.d.add(Polygon(flat, fillColor=fill, strokeColor=colors.black if stroke is None else stroke,
                           strokeWidth=width, strokeDashArray=dash))

    def line(self, a, b, *, color=None, width: float = 0.8, dash=None) -> None:
        from reportlab.graphics.shapes import Line
        from reportlab.lib import colors
        self.d.add(Line(*self.p(*a), *self.p(*b), strokeColor=color or colors.black, strokeWidth=width,
                        strokeDashArray=dash))

    def arrow(self, a, b, *, color=None, width: float = 1.2, head_pt: float = 6.0) -> None:
        from reportlab.graphics.shapes import Polygon
        from reportlab.lib import colors
        color = color or colors.black
        self.line(a, b, color=color, width=width)
        (ax, ay), (bx, by) = self.p(*a), self.p(*b)
        L = math.hypot(bx - ax, by - ay) or 1.0
        ux, uy = (bx - ax) / L, (by - ay) / L
        h, w = head_pt, head_pt * 0.45
        self.d.add(Polygon([bx, by, bx - h * ux - w * uy, by - h * uy + w * ux, bx - h * ux + w * uy,
                            by - h * uy - w * ux], fillColor=color, strokeColor=color, strokeWidth=0.5))

    def text(self, x: float, y: float, s: str, size: float = 8.0, *, bold: bool = False, anchor: str = "middle",
             color=None, angle: float = 0.0) -> None:
        """Text at world (x, y); y is the middle of the capitals (centred vertically)."""
        from reportlab.graphics.shapes import Group, String
        from reportlab.lib import colors
        px, py = self.p(x, y)
        st = String(0, -0.35 * size, s, fontName="Helvetica-Bold" if bold else "Helvetica", fontSize=size,
                    fillColor=color or colors.black, textAnchor=anchor)
        g = Group(st)
        g.translate(px, py)
        if angle:
            g.rotate(angle)
        self.d.add(g)

    def lines(self, x: float, y: float, rows: list[tuple[str, float, bool]], *, color=None) -> None:
        """Several centred lines (text, size, bold) as a block centred at world (x, y)."""
        gap = 0.25
        h = sum(r[1] for r in rows) * (1 + gap) - rows[-1][1] * gap
        top = self.p(x, y)[1] + h / 2
        for s, size, bold in rows:
            yy = top - size / 2
            self.text(x, (yy / self.s) + self.box[1], s, size, bold=bold, color=color)
            top -= size * (1 + gap)


def _colors():
    from reportlab.lib import colors
    return {"full": colors.Color(0.80, 0.88, 0.97), "half": colors.Color(0.99, 0.86, 0.70),
            "unused": colors.Color(0.92, 0.92, 0.92), "mdf": colors.Color(0.96, 0.93, 0.86),
            "plate": colors.Color(0.85, 0.95, 0.85), "ares": colors.Color(0.90, 0.90, 0.90),
            "x": colors.Color(0.80, 0.10, 0.10), "y": colors.Color(0.05, 0.55, 0.10),
            "grey": colors.Color(0.4, 0.4, 0.4)}


def _ares_outline(cfg: dict) -> list[tuple[float, float]]:
    a = cfg["ares"]
    return _rounded_rect(float(a["length"]), float(a["width"]), float(a.get("deck_corner_r", 0.0)))


def _fit(text: str, bold: bool, width_pt: float, size_max: float) -> float:
    """Largest font size <= size_max at which `text` fits width_pt."""
    from reportlab.pdfbase.pdfmetrics import stringWidth
    w1 = stringWidth(text, "Helvetica-Bold" if bold else "Helvetica", 1.0) or 1.0
    return min(size_max, width_pt / w1)


def _stack_label(v: View, s: dict, L: float, W: float, extra: str | None = None) -> None:
    """Name (big), type, layers (and `extra`) inside a stack's footprint, sized to fit it."""
    w_pt, h_pt = L * v.s, W * v.s
    if abs(s["axis"][0]) < abs(s["axis"][1]):          # length across the page: the footprint is L tall, W wide
        w_pt, h_pt = h_pt, w_pt
    w_in = w_pt - 6.0
    n = len(s["used"])
    kind = KIND_DE.get(s["kind"], s["kind"])
    small_rows = ([f"{kind} · {n} Lage{'n' if n != 1 else ''}", _layers_txt(s["used"])] if n else ["leer lassen"])
    if n and _fit(small_rows[0], False, w_in, 8.0) < 6.5:          # narrow (half stone): the short form
        small_rows = [kind, _layers_txt(s["used"])]
    if extra:
        small_rows.append(extra)
    small = max(5.0, min(_fit(r, False, w_in, 8.0) for r in small_rows))
    big = _fit(s["stack"], True, w_in, 17.0)
    big = max(7.0, min(big, (h_pt - 6.0 - small * 1.25 * len(small_rows)) / 1.25))
    v.lines(s["x_mm"], s["y_mm"], [(s["stack"], big, True)] + [(r, small, False) for r in small_rows])


# ── page 1: the station from above ───────────────────────────────────────────
def station_drawing(job: mjob.Job, cfg: dict, max_w: float, max_h: float) -> View:
    """Top view of the pick-up station (station frame: x along the rows, y away from ARES), ARES docked below it."""
    c = _colors()
    ps = cfg["pickup_station"]
    X, Y = (float(v) for v in ps["table_size"])
    T_dock = np.asarray(job.station.dock.T, float)                     # station <- ARES (this repo's ARES frame)
    ur = _tf(T_dock, [tuple(np.asarray(job.T_ares_base, float)[:2, 3])])[0]
    ur_half = float(cfg.get("deck_plate", {}).get("ur_plate", 180.0)) / 2.0
    y_cut = min(0.0, ur[1] - ur_half - 40.0)
    v = View((-190.0, y_cut, X + 30.0, Y + 30.0), max_w, max_h)
    # ARES at the dock (only its UR end is in view, cut off at y_cut)
    ares = _clip(_tf(T_dock, _ares_outline(cfg)), v.box)
    v.poly(ares, fill=c["ares"], stroke=c["grey"])
    xs = [q[0] for q in ares] or [0.0]
    v.line((min(xs) - 40.0, y_cut + 1.0), (max(xs) + 40.0, y_cut + 1.0), color=c["grey"], dash=[4, 3], width=0.8)
    plate = _rect(0.0, 0.0, 2 * ur_half, 2 * ur_half)
    v.poly(_tf(T_dock @ np.asarray(job.T_ares_base, float), plate), fill=c["unused"], stroke=c["grey"])
    v.text(ur[0], ur[1], "UR", 10, bold=True)
    dock = job.station.dock
    a = cfg["ares"]
    edge = _tf(T_dock, [(float(a["length"]) / 2.0, 0.0)])[0]
    x_txt = max(xs) + 30.0
    y_mid = (edge[1] + y_cut) / 2.0
    v.text(x_txt, y_mid + 40.0, "ARES angedockt,", 9, bold=True, anchor="start")
    v.text(x_txt, y_mid + 5.0, "UR-Ende zur Station", 9, bold=True, anchor="start")
    v.text(x_txt, y_mid - 35.0, f"ARES-Mitte bei x {_de(dock.x_mm)} / y {_de(dock.y_mm)} mm", 7, anchor="start",
           color=c["grey"])
    v.arrow((min(xs) + 50.0, y_cut + 40.0), (min(xs) + 50.0, edge[1] - 15.0), color=c["grey"], width=1.0)
    v.text(min(xs) + 70.0, (y_cut + edge[1]) / 2.0, "ARES-x", 7, anchor="start", color=c["grey"])
    # station area (MDF piece) and the S0 / S1 plates
    v.poly([(0.0, 0.0), (X, 0.0), (X, Y), (0.0, Y)], fill=c["mdf"], width=1.0)
    for name, (cx, cy), (w, h) in station_plates(cfg):
        v.poly(_rect(cx, cy, w, h), fill=c["plate"], stroke=c["y"], width=0.8)
        v.lines(cx, cy, [(name, 11, True), ("Code-Platte", 6.5, False)], color=c["y"])
    # stacks
    stacks = station_stacks(job)
    for s in stacks:
        L, W = stone_size(cfg, s["kind"])
        v.poly(_rect(s["x_mm"], s["y_mm"], L, W, s["axis"]), fill=c[s["kind"]] if s["used"] else c["unused"],
               width=1.0)
        _stack_label(v, s, L, W)
    # rows
    for r in sorted({s["row"] for s in stacks}):
        y = float(np.mean([s["y_mm"] for s in stacks if s["row"] == r]))
        v.lines(-95.0, y, [(f"Reihe {r}", 9, True), ("an ARES" if r == 1 else "hinten", 7, False)])
    # axes of the station frame at its origin
    v.arrow((0.0, 0.0), (160.0, 0.0), color=c["x"], width=1.4)
    v.arrow((0.0, 0.0), (0.0, 160.0), color=c["y"], width=1.4)
    v.text(175.0, -22.0, "x", 9, bold=True, color=c["x"])
    v.text(-22.0, 175.0, "y", 9, bold=True, color=c["y"])
    v.text(-14.0, -22.0, "0", 8, bold=True, anchor="end")
    return v


# ── page 2: the magazine on ARES from above ──────────────────────────────────
def magazine_drawing(job: mjob.Job, cfg: dict, max_w: float, max_h: float) -> View:
    """Top view of the ARES deck in this repo's ARES frame (x towards the UR end, to the right; y up) with the
    magazine stacks."""
    c = _colors()
    a = cfg["ares"]
    L, W = float(a["length"]), float(a["width"])
    v = View((-L / 2 - 170.0, -W / 2 - 40.0, L / 2 + 30.0, W / 2 + 40.0), max_w, max_h)
    v.poly(_ares_outline(cfg), fill=c["ares"], stroke=c["grey"], width=1.0)
    dp = cfg.get("deck_plate", {})
    if "split_x" in dp and "front_x" in dp:                               # MDF magazine plate (tools/make_deck_plate)
        x0, x1 = float(dp["split_x"]), float(dp["front_x"])
        v.poly(_clip(_ares_outline(cfg), (x0, -W, x1, W)), fill=c["mdf"], stroke=c["grey"], width=0.6)
    T_ab = np.asarray(job.T_ares_base, float)
    ur_half = float(dp.get("ur_plate", 180.0)) / 2.0
    v.poly(_tf(T_ab, _rect(0.0, 0.0, 2 * ur_half, 2 * ur_half)), fill=c["unused"], stroke=c["grey"])
    v.text(T_ab[0, 3], T_ab[1, 3], "UR", 12, bold=True)
    v.lines(-L / 2 - 85.0, 0.0, [("Controller-", 8, True), ("Ende", 8, True)], color=c["grey"])
    for s in magazine_stacks(job):
        Ls, Ws = stone_size(cfg, s["kind"] or "full")
        v.poly(_rect(s["x_mm"], s["y_mm"], Ls, Ws, s["axis"]), fill=c[s["kind"] or "full"] if s["used"] else
               c["unused"], width=1.0)
        start = f"Start: {_layers_txt(s['start'])}" if s["start"] else "Start: leer"
        _stack_label(v, s, Ls, Ws, start)
    # axes at the ARES origin (base_link: centre between the steering axes)
    v.arrow((0.0, 0.0), (130.0, 0.0), color=c["x"], width=1.4)
    v.arrow((0.0, 0.0), (0.0, 130.0), color=c["y"], width=1.4)
    v.text(145.0, -18.0, "x", 9, bold=True, color=c["x"])
    v.text(16.0, 140.0, "y", 9, bold=True, color=c["y"])
    x_lbl = float(dp["front_x"]) + 12.0 if "front_x" in dp else 10.0      # beside the magazine plate's edge
    v.text(max(10.0, x_lbl), -25.0, "ARES-Mitte", 7, anchor="start")
    return v


def _de(x: float, nd: int = 1) -> str:
    """German decimal comma, trailing zeros dropped (ASCII minus: the drawings use WinAnsi fonts)."""
    s = f"{float(x):.{nd}f}".replace(".", ",")
    return s.rstrip("0").rstrip(",") if "," in s else s


# ── pages ────────────────────────────────────────────────────────────────────
def story(job: mjob.Job, cfg: dict, frame_w: float, frame_h: float, *, landscape_template: str | None = None,
          portrait_template: str | None = None, number: str = "") -> list:
    """Flowables of the three plan pages for a document whose landscape frame is frame_w x frame_h points. With
    page template names (a document with portrait and landscape pages, tools/make_stone_list.py) the drawings go on
    landscape pages and the tables on a portrait page; `number` numbers the headings <number>.1 - .3."""
    from reportlab.lib import colors
    from reportlab.lib.styles import getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.platypus import CondPageBreak, NextPageTemplate, PageBreak, Paragraph, Spacer, Table, TableStyle

    ss = getSampleStyleSheet()
    H2, P = ss["Heading2"], ss["BodyText"]
    small = ss["BodyText"].clone("small_plan", fontSize=8, leading=10)
    cell = ss["BodyText"].clone("cell_plan", fontSize=8, leading=9.5)
    pre = (lambda i: f"{number}.{i} ") if number else (lambda i: "")    # noqa: E731
    st = job.station
    dock = st.dock
    ps = cfg["pickup_station"]
    out: list = []
    if landscape_template:
        out += [NextPageTemplate(landscape_template), PageBreak()]
    budget_h = frame_h - 40 * mm                               # heading + intro, legend
    v = station_drawing(job, cfg, frame_w, budget_h)
    stacks = station_stacks(job)
    n = {k: sum(len(s["used"]) for s in stacks if s["kind"] == k) for k in ("full", "half")}
    out += [Paragraph(f"{pre(1)}Abholstation – Platznummern zum Anschreiben", H2),
            Paragraph("Blick von oben, ARES unten (so wie man von ARES aus auf die Station schaut). Jedes Feld ist ein "
                      "Stapel mit seinem Namen aus Software und HMI; ein Stationsplatz heißt <b>Stapel + Lage</b>, "
                      "z. B. <font name='Courier'>s01l5</font> = Stapel s01, Lage 5 (l1 liegt auf der MDF-Platte). "
                      f"Belegt: {n['full']} Voll + {n['half']} Halb = {n['full'] + n['half']} Steine.", small),
            Spacer(1, 2 * mm), v.d, Spacer(1, 2 * mm),
            Paragraph(f"Stationsrahmen: Ursprung 0 = vordere linke Ecke der Stationsfläche (ARES-Seite), <font "
                      f"color='#cc1a1a'><b>x</b></font> entlang der Reihen (von ARES aus nach rechts), <font "
                      f"color='#0d8c1a'><b>y</b></font> von ARES weg. ARES dockt mit der Mitte (base_link) bei x "
                      f"{_de(dock.x_mm)} / y {_de(dock.y_mm)} mm, ARES-x (zum UR) um "
                      f"{_de(math.degrees(dock.theta_rad), 1)}° gegen die Stations-x-Achse gedreht"
                      f"{' (zeigt in +y, zur Station)' if abs(math.degrees(dock.theta_rad) - 90.0) < 0.5 else ''}. "
                      "Stationsfläche "
                      f"{_de(ps['table_size'][0], 0)} × {_de(ps['table_size'][1], 0)} mm. Maßstab ≈ 1:"
                      f"{v.scale_n:.0f} bei 100 % Druck (nicht zum Messen, Maße: Tabelle und Bodenplan). "
                      "<b>Jeden Stapel nur bis zur angegebenen Lage füllen</b> – ein Stein darüber ist der Software "
                      "unbekannt und liegt den "
                      "Greifbacken im Weg.", small), PageBreak()]
    v2 = magazine_drawing(job, cfg, frame_w, budget_h)
    mags = magazine_stacks(job)
    off = [f"{s['stack']}l{k}" for s in mags for k in s["unused"]]
    out += [Paragraph(f"{pre(2)}Magazin auf ARES – Platznummern", H2),
            Paragraph("Blick von oben auf das Deck, UR rechts, Controller-Ende links. Ein Magazinplatz heißt "
                      "<b>r</b>(Reihe) <b>y</b>(Position quer) [<b>a</b>/<b>b</b> = Hälfte des Halbsteinhalters] "
                      "<b>l</b>(Lage), z. B. <font name='Courier'>r1y0al1</font> = Reihe 1, Position y0, Halbstein a, "
                      "Lage 1 (unten). „Start“ = Lagen, die vor dem Lauf von Hand gefüllt werden "
                      "(Steinliste: Startfüllung).", small),
            Spacer(1, 2 * mm), v2.d, Spacer(1, 2 * mm),
            Paragraph("ARES-Rahmen dieses Repos (config [ares] frame_x_points_to): Ursprung = Mitte zwischen den "
                      "Lenkachsen, <font color='#cc1a1a'><b>x</b></font> zum UR-Ende, <font color='#0d8c1a'><b>y</b>"
                      "</font> quer (y0 bei -y, y2 bei +y). Reihe r0 liegt weiter vom UR weg, r1 näher am UR. "
                      f"Maßstab ≈ 1:{v2.scale_n:.0f} bei 100 % Druck. "
                      + (f"Nicht erreichbar, frei lassen: {', '.join(off)}." if off else ""), small)]
    if portrait_template:
        out += [NextPageTemplate(portrait_template)]
    out += [PageBreak(), Paragraph(f"{pre(3)}Platzkennungen als Liste", H2),
            Paragraph("Kennung = Stapel + „l“ + Lage, Lage 1 = unten. Die Software räumt die Station und das "
                      "Magazin von oben ab und füllt das Magazin von unten. x / y = Steinmitte in mm (Station: "
                      "Stationsrahmen; Magazin: ARES-Rahmen). Positionen sind Planwerte der Konfiguration "
                      "([pickup_station] slots_xy / half_slots_xy, [deck] magazine_rows_dx / magazine_y: ANNAHME).",
                      small), Spacer(1, 2 * mm)]
    grid = TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.grey), ("BACKGROUND", (0, 0), (-1, 0), colors.lightgrey),
                       ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"), ("FONTSIZE", (0, 0), (-1, -1), 8),
                       ("VALIGN", (0, 0), (-1, -1), "MIDDLE"), ("TOPPADDING", (0, 0), (-1, -1), 2),
                       ("BOTTOMPADDING", (0, 0), (-1, -1), 2)])
    rows = [["Stapel", "Reihe", "x [mm]", "y [mm]", "Typ", "Lagen", "Plätze (von unten)", "leer lassen"]]
    for s in stacks:
        rows.append([s["stack"], f"{s['row']} ({'an ARES' if s['row'] == 1 else 'hinten'})", _de(s["x_mm"]),
                     _de(s["y_mm"]), KIND_DE[s["kind"]], len(s["used"]), Paragraph(", ".join(s["ids"]) or "–", cell),
                     _layers_txt(s["unused"]) if s["unused"] else "–"])
    t = Table(rows, colWidths=[w * mm for w in (14, 20, 15, 15, 11, 12, 75, 18)], repeatRows=1)
    t.setStyle(grid)
    out += [Paragraph("<b>Abholstation</b>", P), t, Spacer(1, 4 * mm), CondPageBreak(60 * mm)]
    rows = [["Stapel", "x [mm]", "y [mm]", "Typ", "Lagen", "Plätze (von unten)", "Start", "frei lassen"]]
    for s in mags:
        rows.append([s["stack"], _de(s["x_mm"]), _de(s["y_mm"]), KIND_DE.get(s["kind"], s["kind"]), len(s["used"]),
                     Paragraph(", ".join(s["ids"]) or "–", cell),
                     ", ".join(f"l{k} {KIND_DE[s['start_kinds'][k]]}" for k in s["start"]) or "–",
                     _layers_txt(s["unused"]) if s["unused"] else "–"])
    t = Table(rows, colWidths=[w * mm for w in (16, 15, 15, 11, 12, 58, 33, 20)], repeatRows=1)
    t.setStyle(grid)
    out += [Paragraph("<b>Magazin auf ARES</b>", P), t]
    return out


def write_pdf(path: Path, job: mjob.Job, cfg: dict, job_path: Path) -> None:
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.units import mm
    from reportlab.platypus import BaseDocTemplate, Frame, PageTemplate

    def footer(canv, doc):
        canv.saveState()
        canv.setFont("Helvetica", 7)
        w = canv._pagesize[0]
        canv.drawString(12 * mm, 6 * mm, f"ARES_Mauer – Stationsplan – {job_path.as_posix()} (Job vom "
                                         f"{datetime.fromisoformat(job.created):%d.%m.%Y %H:%M}), erzeugt "
                                         f"{datetime.now():%d.%m.%Y %H:%M} mit tools/make_station_map.py")
        canv.drawRightString(w - 12 * mm, 6 * mm, f"Seite {doc.page}")
        canv.restoreState()

    m = 12 * mm
    qw, qh = landscape(A4)
    pw, ph = A4
    quer = PageTemplate("quer", [Frame(m, m, qw - 2 * m, qh - 2 * m, id="q")], onPage=footer, pagesize=(qw, qh))
    hoch = PageTemplate("hoch", [Frame(15 * mm, 15 * mm, pw - 30 * mm, ph - 30 * mm, id="h")], onPage=footer,
                        pagesize=(pw, ph))
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = BaseDocTemplate(str(path), pagesize=(qw, qh), leftMargin=m, rightMargin=m, topMargin=m, bottomMargin=m,
                          title="Stationsplan", author="ARES_Mauer")
    doc.addPageTemplates([quer, hoch])
    doc.build(story(job, cfg, qw - 2 * m - 12, qh - 2 * m - 12, portrait_template="hoch"))


# ── job loading (shared with tools/make_stone_list.py) ───────────────────────
def load_checked(job_path: Path | None, config_path: Path | None, variant: str | None) -> tuple:
    """(job, cfg, job path relative to the repo) - refuses a job built with another variant or an older config."""
    cfg = mconfig.load(config_path, variant)
    sfx = mconfig.suffix(cfg)
    job_path = job_path or REPO / "data" / "jobs" / f"nominal_{cfg['wall'].get('shape') or 'legs'}{sfx}.json"
    job = mjob.load(job_path)
    if (job.meta.get("config_variant") or None) != mconfig.variant_of(cfg):
        raise SystemExit(f"{job_path} was built with config variant {job.meta.get('config_variant')!r}, not "
                         f"{mconfig.variant_of(cfg)!r} - pass the matching --variant / --job")
    if job.config_sha256 and job.config_sha256 != mjob.config_sha256(config_path, mconfig.variant_of(cfg)):
        raise SystemExit(f"{job_path} was built from an older config (sha256 differs) - rebuild it with "
                         "tools/make_job.py" + (f" --variant {mconfig.variant_of(cfg)}" if mconfig.variant_of(cfg)
                                                else ""))
    rel = job_path.resolve().relative_to(REPO) if job_path.resolve().is_relative_to(REPO) else job_path
    return job, cfg, Path(rel)


def copy_final(path: Path) -> Path:
    """Copy `path` into final/ (the folder of the layout "final", final/README.md)."""
    dst = REPO / "final" / path.name
    shutil.copyfile(path, dst)
    return dst


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--job", type=Path, default=None,
                    help="job file (default data/jobs/nominal_<[wall] shape>[_<variant>].json)")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--variant", default=None, help="config variant config/variants/<VARIANT>.toml")
    ap.add_argument("--out", type=Path, default=None, help="default results/stationsplan[_<variant>].pdf")
    ap.add_argument("--final", action="store_true", help="also copy the PDF into final/")
    a = ap.parse_args(argv)
    job, cfg, rel = load_checked(a.job, a.config, a.variant)
    out = a.out or REPO / "results" / f"stationsplan{mconfig.suffix(cfg)}.pdf"
    write_pdf(out, job, cfg, rel)
    msg = f"written {out}: {len(station_stacks(job))} station stacks, {len(magazine_stacks(job))} magazine stacks"
    if a.final:
        msg += f"; copied to {copy_final(out)}"
    print(msg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
