"""Stone list (German, PDF): what the wall of a job needs, how the stones get onto ARES and where every stone of the
pick-up station and of the magazine ends up in the wall.

    py.exe tools/make_stone_list.py [--job data/jobs/nominal_C.json] [--out results/steinliste.pdf]

Everything comes from the job (tools/make_job.py) and config/station.toml:
- stones per leg, course and type (full / half) and their total;
- the loading plan by the sequencer's rule (mauer.job.reload_plan / reload_short, the same loop as
  tools/make_job.py plan_slots): the start fill of the magazine and of the pick-up station, and per station trip the
  operator's top-up before it and the stones moved station -> magazine. The sequencer assumes the operator fills
  EVERY empty station holder (mauer.sequencer._reload: SlotState.station(...) = full station after the refill), so
  the stones put out exceed the wall by what is left in the station at the end;
- the trace of every stone through that loop (2026-10-16, Samuel: "welcher Stein in der Station schlussendlich wo an
  der Mauer ist"): magazine start fill or station stack / layer (and the station fill it was put out with) -> station
  trip -> magazine slot -> laying number, leg, course, u. The sequencer takes the same slots
  (mauer.sequencer._reload / _place: reload_plan, magazine.next_take(t.slot, kind)) as long as the run follows the
  plan; a resumed run, a stone declared placed by hand or a `slot_changed` log record shifts it from there on;
- drawings: the magazine on the ARES deck (start fill), the pick-up station from above (MDF piece and windows of
  tools/make_guides.py, stack names as engraved, per layer the laying number) and the wall unfolded, seen from
  outside (per stone its laying number and where it comes from);
- every stone in laying order (stop, leg, course, centre along the leg, type, magazine slot, origin);
- masses ([brick] / [half_brick] mass_kg) and the status tags of the config values the list depends on (CLAUDE.md).
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO, REPO / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from mauer import config as mconfig  # noqa: E402
from mauer import job as mjob  # noqa: E402

KINDS = ("full", "half")
STATUS_DE = {"CONFIRMED": "BESTÄTIGT", "ASSUMPTION": "ANNAHME", "PLACEHOLDER": "PLATZHALTER", "UNKNOWN": "UNBEKANNT",
             "UNTAGGED": "OHNE STATUS"}
KIND_DE = {"full": "Voll", "half": "Halb"}
OPENING_DE = {"window": "Fenster", "door": "Tür"}
SIDE_DE = {"front": "vor ARES", "left": "links von ARES", "right": "rechts von ARES"}
DEPENDS = ["[wall] courses", "[[wall.legs]] {leg}.n0", "[brick] length", "[brick] width", "[brick] height",
           "[half_brick] length", "[brick] mass_kg", "[half_brick] mass_kg", "[deck] magazine_rows_dx",
           "[deck] magazine_y", "[deck] magazine_layers", "[deck] half_positions", "[pickup_station] slots_xy",
           "[pickup_station] slot_layers", "[pickup_station] half_slots_xy", "[pickup_station] half_slot_layers"]
# fill colour per station trip in the drawings (0 = magazine start fill); trips beyond the list repeat 1..
TRIP_RGB = [(0.84, 0.84, 0.84), (0.70, 0.83, 0.97), (0.99, 0.80, 0.58), (0.73, 0.91, 0.70), (0.85, 0.76, 0.95),
            (0.98, 0.93, 0.55), (0.96, 0.74, 0.80)]


def counts(job: mjob.Job) -> dict:
    """{(leg, course, kind): n} of the job's stones (leg "" for a straight wall)."""
    return dict(Counter((t.leg or "", t.course, t.kind) for t in job.stones()))


def loading_plan(job: mjob.Job) -> dict:
    """Start fill and station trips by the sequencer's rule (tools/make_job.py plan_slots), and the way of every stone:
    {"magazine": {kind: n}, "station": {kind: n}, "trips": [{"stop", "leg", "topup": {kind: n}, "moved": {kind: n}}],
     "left": {kind: n} in the station at the end,
     "stones": [{"nr" (laying order from 1), "stop", "leg", "course", "index", "u_mm", "length_mm", "kind", "label",
                 "mag" (magazine slot it is taken from), "src" (station slot, None = magazine start fill), "trip"
                 (station trip that brought it, 0 = start fill), "fill" (station fill it was put out with: 0 = start,
                 k = the top-up before trip k; None for the magazine start fill)}, ...] in laying order,
     "left_slots": {station slot: fill} of the stones left in the station}."""
    mag = mjob.SlotState.magazine(job.magazine)
    full = mjob.SlotState.station(job.station)
    st = full.copy()
    in_mag = {sid: {"src": None, "trip": 0, "fill": None} for sid in mag.filled}     # origin of a magazine stone
    fill_of = {sid: 0 for sid in st.filled}                                          # station slot -> its fill
    out = {"magazine": {k: mag.count(k) for k in KINDS}, "station": {k: st.count(k) for k in KINDS}, "trips": [],
           "stones": []}
    for stop in job.stops:
        upcoming_all = [t.kind for s in job.stops[stop.index:] for t in s.stones]
        for i, t in enumerate(stop.stones):
            if mag.empty():
                trip = len(out["trips"]) + 1
                upcoming = upcoming_all[i:]
                topup = {k: 0 for k in KINDS}
                if mjob.reload_short(st, full, mag, upcoming):
                    topup = {k: full.count(k) - st.count(k) for k in KINDS}
                    fill_of.update({sid: trip for sid in full.filled - st.filled})
                    st = full.copy()
                pairs = mjob.reload_plan(mag, st, upcoming)
                if not pairs:
                    raise ValueError(f"stone {t.label}: magazine empty and no reload possible")
                for ssid, mid, kind in pairs:
                    st.take(ssid)
                    mag.fill(mid, kind)
                    in_mag[mid] = {"src": ssid, "trip": trip, "fill": fill_of.pop(ssid)}
                out["trips"].append({"stop": stop.index, "leg": stop.leg or "", "topup": topup,
                                     "moved": {k: sum(1 for *_, x in pairs if x == k) for k in KINDS}})
            sid = mag.next_take(t.slot, kind=t.kind)
            if sid is None:
                raise ValueError(f"stone {t.label}: no {t.kind} stone in the magazine")
            mag.take(sid)
            out["stones"].append({"nr": len(out["stones"]) + 1, "stop": stop.index, "leg": t.leg or "",
                                  "course": t.course, "index": t.index, "u_mm": float(t.u_mm),
                                  "length_mm": None if t.length_mm is None else float(t.length_mm), "kind": t.kind,
                                  "label": t.label, "mag": sid, **in_mag.pop(sid)})
    out["left"] = {k: st.count(k) for k in KINDS}
    out["left_slots"] = {sid: fill_of[sid] for sid in st.filled}
    return out


def station_stacks(job: mjob.Job) -> list[dict]:
    """Every stack of the pick-up station (name as engraved on the MDF piece, tools/make_guides.py) with its row (1 =
    near ARES), centre (station frame, mm), stone type and the number of layers the UR reaches (the job's take order;
    2026-10-09: 5 / 4 / 2 - a stone above them is unknown to the software and in the jaws' way), sorted by row and x."""
    st = job.station
    rows = sorted({round(float(s.T_station_tcp[1, 3]), 1) for s in st.slots})
    out = []
    for sid in sorted({s.stack_id for s in st.slots}):
        sl = [s for s in st.slots if s.stack_id == sid]
        base = min(sl, key=lambda s: s.layer)
        y = round(float(base.T_station_tcp[1, 3]), 1)
        out.append({"stack": sid, "row": rows.index(y) + 1, "x_mm": float(base.T_station_tcp[0, 3]), "y_mm": y,
                    "kind": base.kind, "layers": sum(1 for s in sl if s.id in st.take_order)})
    return sorted(out, key=lambda s: (s["row"], s["x_mm"]))


def station_use(job: mjob.Job, plan: dict) -> dict[str, list[dict]]:
    """{station slot: [stone records of loading_plan()["stones"] it supplies, in laying order]} - one stone per slot,
    more when the operator tops the station up between trips; slots holding a stone that is left over get a record
    {"nr": None, "fill": f}."""
    use: dict[str, list[dict]] = {sid: [] for sid in job.station.take_order}
    for r in plan["stones"]:
        if r["src"] is not None:
            use[r["src"]].append(r)
    for sid, f in plan["left_slots"].items():
        use[sid].append({"nr": None, "fill": f})
    return use


def magazine_start(job: mjob.Job, plan: dict) -> dict[str, dict]:
    """{magazine slot: stone record} of the magazine's start fill."""
    return {r["mag"]: r for r in plan["stones"] if r["src"] is None}


def slot_txt(sid: str) -> str:
    """Short slot name for the drawings: 's01l5' -> 's01·5', 'r1y0al1' -> 'r1y0a·1' (stack · layer)."""
    base, sep, layer = sid.rpartition("l")
    return f"{base}·{layer}" if sep and base and layer.isdigit() else sid


def origin_de(r: dict, job: mjob.Job | None = None) -> str:
    """Where a stone comes from, in words: 'Magazin r1y2 Lage 3 (Start)' / 'Station s01 Lage 5, Fahrt 1'."""
    if r["src"] is None:
        if job is not None:
            ms = job.magazine.slot(r["mag"])
            return f"Magazin {ms.stack} Lage {ms.layer} (Start)"
        return f"Magazin {slot_txt(r['mag'])} (Start)"
    if job is not None:
        ss = job.station.slot(r["src"])
        txt = f"Station {ss.stack_id} Lage {ss.layer}, Fahrt {r['trip']}"
    else:
        txt = f"Station {slot_txt(r['src'])}, Fahrt {r['trip']}"
    return txt + (f" (nachgelegt vor Fahrt {r['fill']})" if r.get("fill") else "")


def trip_rgb(trip: int) -> tuple[float, float, float]:
    return TRIP_RGB[0] if trip == 0 else TRIP_RGB[1 + (trip - 1) % (len(TRIP_RGB) - 1)]


def _de(x: float, nd: int = 1) -> str:
    return f"{x:.{nd}f}".replace(".", ",")


def _num(x: float, nd: int = 2) -> str:
    """Up to nd decimals, trailing zeros dropped: 99.5 -> '99,5', 120.0 -> '120', 1.25 -> '1,25' (a fixed :.0f / :.1f
    showed the 99.5 mm half stone as 100 and 1.25 kg as 1,2)."""
    return _de(float(x), nd).rstrip("0").rstrip(",")


def _n(d: dict) -> str:
    """'12 (10 Voll, 2 Halb)' from {kind: n}."""
    parts = [f"{d[k]} {KIND_DE[k]}" for k in KINDS if d.get(k)]
    return f"{sum(d.values())}" + (f" ({', '.join(parts)})" if parts else "")


# ── drawings (reportlab canvas; dimensions in drawing mm, origin lower left) ────────────────────────────────────────
def _sketch(w_mm: float, h_mm: float, paint):
    """A flowable of w x h mm that calls paint(canvas, mm) - the canvas origin at its lower-left corner."""
    from reportlab.lib.units import mm
    from reportlab.platypus import Flowable

    class Sketch(Flowable):
        def wrap(self, aw, ah):
            return w_mm * mm, h_mm * mm

        def draw(self):
            paint(self.canv, mm)

    return Sketch()


def _text(c, s: str, x: float, y: float, size: float, bold: bool = False, maxw: float | None = None,
          align: str = "c") -> None:
    """Draw s at (x, y) [pt]; shrink the font (down to 3.5 pt) until it fits maxw [pt]."""
    from reportlab.pdfbase.pdfmetrics import stringWidth
    font = "Helvetica-Bold" if bold else "Helvetica"
    while maxw is not None and size > 3.5 and stringWidth(s, font, size) > maxw:
        size -= 0.25
    c.setFont(font, size)
    {"c": c.drawCentredString, "l": c.drawString, "r": c.drawRightString}[align](x, y, s)


def _legend(c, M: float, x: float, y: float, plan: dict, maxw: float) -> None:
    """Colour legend of the trips in one line from (x, y) [mm]."""
    items = [(0, "Magazin, Startfüllung")] + [
        (i, f"Fahrt {i} (ab Halt {tr['stop']}{', ' + tr['leg'] if tr['leg'] else ''})")
        for i, tr in enumerate(plan["trips"], 1)]
    from reportlab.pdfbase.pdfmetrics import stringWidth
    cx = x
    for trip, txt in items:
        w = 4.5 + stringWidth(txt, "Helvetica", 7) / M + 5
        if cx + w > x + maxw:
            cx, y = x, y - 4.5
        c.setFillColorRGB(*trip_rgb(trip))
        c.setStrokeColorRGB(0.3, 0.3, 0.3)
        c.rect(cx * M, (y - 0.6) * M, 3 * M, 3 * M, fill=1, stroke=1)
        c.setFillColorRGB(0, 0, 0)
        _text(c, txt, (cx + 4) * M, y * M, 7, align="l")
        cx += w


def station_sketch(job: mjob.Job, cfg: dict, plan: dict, scale: float = 7.0, width_mm: float = 265.0):
    """The pick-up station from above, seen from ARES (station frame: x to the right from the left edge of the MDF
    piece, y away from ARES): the MDF pieces with their dovetail cut and the S windows (tools/make_guides.py), every
    stack with its engraved name, layer count and per layer (top first) the laying number of the stone it supplies,
    coloured by station trip; x of every stack centre; ARES at the dock below."""
    import make_guides as mg

    ps = cfg["pickup_station"]
    X, Y = (float(v) for v in ps["table_size"])
    p = mg.params(cfg)
    pieces = mg.station_pieces(cfg, p, mg._station_plates(cfg))
    stacks = station_stacks(job)
    use = station_use(job, plan)
    by_stack = {s["stack"]: sorted((sl for sl in job.station.slots if sl.stack_id == s["stack"]
                                    and sl.id in job.station.take_order), key=lambda sl: -sl.layer) for s in stacks}
    L = {"full": float(cfg["brick"]["length"]), "half": float(cfg.get("half_brick", cfg["brick"])["length"])}
    W = float(cfg["brick"]["width"])
    left, bottom, top = 28.0, 34.0, 11.0
    h_mm = bottom + Y / scale + top
    ares = cfg["ares"]
    dock_x = float(ps["ares_xyz"][0])
    ares_front = float(ps["ares_xyz"][1]) + float(ares["length"]) / 2.0     # ARES front (UR end) in the station frame

    def paint(c, M):
        P = lambda x, y: ((left + x / scale) * M, (bottom + y / scale) * M)  # noqa: E731
        # ARES at the dock (its UR end, cut off by the drawing)
        y0 = ares_front - 13.0 * scale
        c.setStrokeColorRGB(0.1, 0.3, 0.8)
        c.setFillColorRGB(0.88, 0.92, 1.0)
        c.setDash(3, 2)
        a, b = P(dock_x - float(ares["width"]) / 2, y0), P(dock_x + float(ares["width"]) / 2, ares_front)
        c.rect(a[0], a[1], b[0] - a[0], b[1] - a[1], fill=1, stroke=1)
        c.setDash()
        c.setFillColorRGB(0.1, 0.3, 0.8)
        _text(c, f"ARES beim Andocken (UR-Ende zur Station, {_num(-ares_front)} mm vor der Platte)",
              (a[0] + b[0]) / 2, (a[1] + b[1]) / 2 - 2, 6.5)
        # MDF pieces
        c.setStrokeColorRGB(0.45, 0.25, 0.05)
        c.setFillColorRGB(0.96, 0.91, 0.82)
        c.setLineWidth(0.8)
        for pc in pieces:
            path = c.beginPath()
            q = [P(*v) for v in pc.outline]
            path.moveTo(*q[0])
            for v in q[1:]:
                path.lineTo(*v)
            path.close()
            c.drawPath(path, fill=1, stroke=1)
        c.setLineWidth(0.5)
        for pc in pieces:
            for w in pc.windows:
                a, b = P(*w[0]), P(*w[2])
                c.setFillColorRGB(1, 1, 1)
                c.setStrokeColorRGB(0.1, 0.5, 0.1)
                c.rect(a[0], a[1], b[0] - a[0], b[1] - a[1], fill=1, stroke=1)
            for (tx, ty), _, s in pc.texts:
                c.setFillColorRGB(0.1, 0.5, 0.1)
                cx = (left + (tx + 12.0) / scale) * M
                _text(c, f"{s} (Code)", cx, P(0, ty)[1], 6.5, bold=True)
        # stacks
        for s in stacks:
            ln, kind = L[s["kind"]], s["kind"]
            a, b = P(s["x_mm"] - ln / 2, s["y_mm"] - W / 2), P(s["x_mm"] + ln / 2, s["y_mm"] + W / 2)
            bw, bh = b[0] - a[0], b[1] - a[1]
            c.setStrokeColorRGB(0, 0, 0)
            c.setFillColorRGB(1, 1, 1) if s["layers"] else c.setFillColorRGB(0.8, 0.8, 0.8)
            c.rect(a[0], a[1], bw, bh, fill=1, stroke=1)
            c.setFillColorRGB(0, 0, 0)
            head = f"{s['stack']} · {s['layers']}" if s["layers"] else f"{s['stack']} leer"
            _text(c, head, a[0] + bw / 2, b[1] - 2.6 * M, 6.5, bold=True, maxw=bw - 1.0 * M)
            lines = []
            for sl in by_stack[s["stack"]]:
                recs = use.get(sl.id, [])
                nrs = "/".join("übrig" if r["nr"] is None else str(r["nr"]) for r in recs) or "–"
                r0 = next((r for r in recs if r["nr"] is not None), None)
                txt = f"{sl.layer} → Nr {nrs}" if r0 is not None else f"{sl.layer} → {nrs}"
                if kind == "full" and r0 is not None and len(recs) == 1:
                    txt += f" · {r0['leg']} L{r0['course'] + 1}" if r0["leg"] else f" · L{r0['course'] + 1}"
                lines.append((txt, r0["trip"] if r0 is not None else None))
            lh = min(2.3 * M, (bh - 3.6 * M) / max(1, len(lines)))
            for j, (txt, trip) in enumerate(lines):
                y = b[1] - 3.6 * M - (j + 1) * lh + 0.5
                if trip is not None:
                    c.setFillColorRGB(*trip_rgb(trip))
                    c.setStrokeColorRGB(0.3, 0.3, 0.3)
                    c.rect(a[0] + 0.8 * M, y - 0.2 * M, 1.8 * M, 1.8 * M, fill=1, stroke=1)
                c.setFillColorRGB(0, 0, 0)
                _text(c, txt, a[0] + 3.0 * M, y, 5.8, maxw=bw - 3.6 * M, align="l")
            # x of the centre: row 1 below the piece, the other rows above it
            c.setFillColorRGB(0.25, 0.25, 0.25)
            if s["row"] == 1:
                _text(c, _num(s["x_mm"], 1), (a[0] + b[0]) / 2, P(0, 0)[1] - 3.0 * M, 5.5)
            else:
                _text(c, _num(s["x_mm"], 1), (a[0] + b[0]) / 2, P(0, Y)[1] + 1.2 * M, 5.5)
        # row labels, x origin, overall size
        c.setFillColorRGB(0, 0, 0)
        rows = sorted({(s["row"], s["y_mm"]) for s in stacks})
        for row, y in rows:
            yy = P(0, y)[1]
            _text(c, f"Reihe {row}", 1.0 * M, yy + 1.0 * M, 7.5, bold=True, align="l")
            _text(c, "vorne, an ARES" if row == 1 else "hinten", 1.0 * M, yy - 2.2 * M, 6.5, align="l")
        _text(c, "x [mm] Reihe 1:", 1.0 * M, P(0, 0)[1] - 3.0 * M, 5.5, align="l")
        if len(rows) > 1:
            _text(c, "x [mm] Reihe 2:", 1.0 * M, P(0, Y)[1] + 1.2 * M, 5.5, align="l")
        c.setStrokeColorRGB(0, 0, 0)
        ya = P(0, Y)[1] + 5.5 * M                                     # overall length above the piece
        c.line(P(0, 0)[0], ya, P(X, 0)[0], ya)
        for xx in (0.0, X):
            c.line(P(xx, 0)[0], ya - 1.0 * M, P(xx, 0)[0], ya + 1.0 * M)
        c.setFillColorRGB(1, 1, 1)
        tw = 70.0 * M
        c.rect(P(X / 2, 0)[0] - tw / 2, ya - 1.0 * M, tw, 2.6 * M, fill=1, stroke=0)
        c.setFillColorRGB(0, 0, 0)
        _text(c, f"{_num(X)} mm, x = 0 an der linken Kante", P(X / 2, 0)[0], ya - 0.6 * M, 6)
        _legend(c, M, left, 1.0, plan, width_mm - left)

    return _sketch(width_mm, h_mm, paint)


def magazine_sketch(job: mjob.Job, cfg: dict, plan: dict, scale: float = 8.0, width_mm: float = 178.0):
    """The ARES deck from above (this repo's ARES frame: x to the right towards the UR end = vehicle rear, y up; the
    vehicle drives to the left, its left side is at the bottom) with every magazine holder (stack name) and per layer
    of the start fill the laying number of the stone."""
    ares = cfg["ares"]
    Lx, Wy = float(ares["length"]), float(ares["width"])
    R = float(ares.get("deck_corner_r", 0.0))
    L = {"full": float(cfg["brick"]["length"]), "half": float(cfg.get("half_brick", cfg["brick"])["length"])}
    W = float(cfg["brick"]["width"])
    start = magazine_start(job, plan)
    stacks: dict[str, list] = {}
    for sl in job.magazine.slots:
        if sl.id in job.magazine.take_order:
            stacks.setdefault(sl.stack, []).append(sl)
    left, bottom = (width_mm - Lx / scale) / 2, 12.0
    h_mm = bottom + Wy / scale + 12.0
    ur = (float(cfg["ur5"]["mount_x"]), float(cfg["ur5"].get("mount_y", 0.0)))

    def paint(c, M):
        P = lambda x, y: ((left + (x + Lx / 2) / scale) * M, (bottom + (y + Wy / 2) / scale) * M)  # noqa: E731
        a, b = P(-Lx / 2, -Wy / 2), P(Lx / 2, Wy / 2)
        c.setStrokeColorRGB(0.4, 0.4, 0.4)
        c.setFillColorRGB(0.93, 0.93, 0.93)
        c.roundRect(a[0], a[1], b[0] - a[0], b[1] - a[1], R / scale * M, fill=1, stroke=1)
        c.setFillColorRGB(0.4, 0.4, 0.4)
        _text(c, "UR5", *P(*ur), 7, bold=True)
        c.circle(*P(*ur), 75.0 / scale * M, fill=0, stroke=1)
        for sid, sl in sorted(stacks.items()):
            base = min(sl, key=lambda s: s.layer)
            x, y = float(base.T_ares_tcp[0, 3]), float(base.T_ares_tcp[1, 3])
            ln = L["half" if base.kind == "half" else "full"]
            a, b = P(x - W / 2, y - ln / 2), P(x + W / 2, y + ln / 2)       # stone length along ARES y
            bw, bh = b[0] - a[0], b[1] - a[1]
            c.setStrokeColorRGB(0, 0, 0)
            c.setFillColorRGB(1, 1, 1)
            c.rect(a[0], a[1], bw, bh, fill=1, stroke=1)
            c.setFillColorRGB(0, 0, 0)
            _text(c, sid, a[0] + bw / 2, b[1] - 2.4 * M, 6, bold=True, maxw=bw - 0.6 * M)
            lines = []
            for s in sorted(sl, key=lambda s: -s.layer):
                r = start.get(s.id)
                lines.append((f"{s.layer} → Nr {r['nr']}" if r else f"{s.layer} → leer", r))
            lh = min(2.5 * M, (bh - 3.2 * M) / max(1, len(lines)))
            for j, (txt, r) in enumerate(lines):
                y_ = b[1] - 3.2 * M - (j + 1) * lh + 0.5
                if r is not None:
                    c.setFillColorRGB(*trip_rgb(0))
                    c.setStrokeColorRGB(0.3, 0.3, 0.3)
                    c.rect(a[0] + 0.5 * M, y_ - 0.2 * M, 1.6 * M, 1.6 * M, fill=1, stroke=1)
                c.setFillColorRGB(0, 0, 0) if r is not None else c.setFillColorRGB(0.5, 0.5, 0.5)
                _text(c, txt, a[0] + 2.4 * M, y_, 6.0, maxw=bw - 2.7 * M, align="l")
        c.setFillColorRGB(0, 0, 0)
        _text(c, "← Fahrtrichtung (Fahrzeugfront)", P(-Lx / 2, 0)[0], P(0, Wy / 2)[1] + 2.5 * M, 7, align="l")
        _text(c, "UR-Ende = Fahrzeugheck →", P(Lx / 2, 0)[0], P(0, Wy / 2)[1] + 2.5 * M, 7, align="r")
        _text(c, "links in Fahrtrichtung (y0)", P(0, 0)[0], P(0, -Wy / 2)[1] - 3.5 * M, 7)
        _text(c, "rechts in Fahrtrichtung (y2)", P(0, 0)[0], P(0, Wy / 2)[1] + 2.5 * M, 7)

    return _sketch(width_mm, h_mm, paint)


def _leg_ends(job: mjob.Job, plan: dict) -> dict[str, tuple[str, str]]:
    """{leg: (text at u = 0, text at the far end)}: 'Ecke zu B' when another leg passes within 300 mm of that end,
    else 'freies Ende'."""
    legs = {lg["name"]: np.asarray(lg["T_wall_leg"], float) for lg in job.legs}
    length = {}
    for r in plan["stones"]:
        ln = r["length_mm"] or 0.0
        length[r["leg"]] = max(length.get(r["leg"], 0.0), r["u_mm"] + ln / 2)
    seg = {n: (T[:2, 3], T[:2, 3] + T[:2, 0] * length.get(n, 0.0)) for n, T in legs.items()}

    def near(pt, other):
        a, b = seg[other]
        d = b - a
        t = 0.0 if not d.any() else float(np.clip(np.dot(pt - a, d) / np.dot(d, d), 0.0, 1.0))
        return float(np.linalg.norm(a + t * d - pt))

    out = {}
    for n in legs:
        ends = []
        for pt in seg[n]:
            o = [m for m in legs if m != n and near(pt, m) < 300.0]
            ends.append(f"Ecke zu {o[0]}" if o else "freies Ende")
        out[n] = (ends[0], ends[1])
    return out


def wall_sketch(job: mjob.Job, cfg: dict, plan: dict, width_mm: float = 265.0, height_mm: float = 150.0):
    """The wall unfolded, every leg seen from OUTSIDE (the legs' +y points towards ARES inside the C: u grows to the
    right), the legs packed into rows (longest first); per stone its laying number and where it comes from (station
    stack · layer, or M + magazine slot for the start fill), coloured by station trip; openings dashed."""
    b, hb = cfg["brick"], cfg.get("half_brick", cfg["brick"])
    pitch_z = float(b["height"]) + float(b.get("bed_joint", 0.0))
    n_courses = max(r["course"] for r in plan["stones"]) + 1
    legs_cfg = {lg["name"]: lg for lg in cfg["wall"].get("legs", [])}
    order = [lg["name"] for lg in job.legs] or [""]
    length = {}
    for r in plan["stones"]:
        ln = r["length_mm"] or float((hb if r["kind"] == "half" else b)["length"])
        length[r["leg"]] = max(length.get(r["leg"], 0.0), r["u_mm"] + ln / 2)
    ends = _leg_ends(job, plan) if job.legs else {}
    gap, head, foot = 12.0, 7.0, 7.0                                   # drawing mm
    for scale in (8.0, 10.0, 12.5, 15.0, 20.0, 25.0):
        rows: list[list[str]] = []
        for n in sorted(order, key=lambda n: -length.get(n, 0.0)):
            for row in rows:
                if sum(length[m] / scale + gap for m in row) + length[n] / scale <= width_mm - 8.0:
                    row.append(n)
                    break
            else:
                rows.append([n])
        h_row = head + n_courses * pitch_z / scale + foot
        if len(rows) * h_row <= height_mm:
            break
    rows = [sorted(row, key=order.index) for row in rows]
    h_mm = len(rows) * h_row

    def paint(c, M):
        for ri, row in enumerate(rows):
            y_floor = h_mm - (ri + 1) * h_row + foot
            x0 = 8.0
            for n in row:
                lw = length[n] / scale
                P = lambda u, z: ((x0 + u / scale) * M, (y_floor + z / scale) * M)  # noqa: E731
                lc = legs_cfg.get(n, {})
                title = (f"Schenkel {n}" + (f" – {SIDE_DE.get(lc.get('side'), lc.get('side'))}" if lc.get("side")
                                            else "") if n else "Wand") + ", von außen"
                c.setFillColorRGB(0, 0, 0)
                _text(c, title, P(0, 0)[0], P(0, n_courses * pitch_z)[1] + 1.8 * M, 7.5, bold=True, align="l",
                      maxw=lw * M)
                c.setStrokeColorRGB(0, 0, 0)
                c.setLineWidth(1.0)
                c.line((x0 - gap / 2) * M, P(0, 0)[1], (x0 + lw + gap / 2) * M, P(0, 0)[1])
                c.setLineWidth(0.5)
                if n in ends:
                    _text(c, ends[n][0], P(0, 0)[0], P(0, 0)[1] - 3.0 * M, 6, align="l")
                    _text(c, ends[n][1], P(length[n], 0)[0], P(0, 0)[1] - 3.0 * M, 6, align="r")
                    _text(c, "u = 0", P(0, 0)[0], P(0, 0)[1] - 5.6 * M, 5, align="l")
                for o in lc.get("openings", []):
                    k0, k1 = (int(v) for v in o["courses"])
                    a = P(float(o["u_from"]), k0 * pitch_z)
                    bb = P(float(o["u_to"]), k1 * pitch_z + float(b["height"]))
                    c.setDash(2, 2)
                    c.setStrokeColorRGB(0.5, 0.5, 0.5)
                    c.rect(a[0], a[1], bb[0] - a[0], bb[1] - a[1], fill=0, stroke=1)
                    c.setDash()
                    c.setFillColorRGB(0.4, 0.4, 0.4)
                    _text(c, OPENING_DE.get(o.get("name", ""), o.get("name", "")), (a[0] + bb[0]) / 2,
                          (a[1] + bb[1]) / 2, 7)
                for r in plan["stones"]:
                    if r["leg"] != n:
                        continue
                    ln = r["length_mm"] or float((hb if r["kind"] == "half" else b)["length"])
                    a = P(r["u_mm"] - ln / 2, r["course"] * pitch_z)
                    bb = P(r["u_mm"] + ln / 2, r["course"] * pitch_z + float(b["height"]))
                    bw, bh = bb[0] - a[0], bb[1] - a[1]
                    c.setFillColorRGB(*trip_rgb(r["trip"]))
                    c.setStrokeColorRGB(0, 0, 0)
                    c.rect(a[0], a[1], bw, bh, fill=1, stroke=1)
                    c.setFillColorRGB(0, 0, 0)
                    src = slot_txt(r["src"]) if r["src"] is not None else "M " + slot_txt(r["mag"])
                    fs = min(7.0, bh / M * 0.45 * 2.83)
                    _text(c, f"{r['nr']}", a[0] + bw / 2, a[1] + bh * 0.55, fs, bold=True, maxw=bw - 0.6 * M)
                    _text(c, src, a[0] + bw / 2, a[1] + bh * 0.18, fs * 0.8, maxw=bw - 0.6 * M)
                c.setFillColorRGB(0.3, 0.3, 0.3)
                if x0 <= 8.0 + 1e-6:
                    for k in range(n_courses):
                        _text(c, f"L{k + 1}", P(0, 0)[0] - 1.0 * M, P(0, (k + 0.4) * pitch_z)[1], 5.5, align="r")
                x0 += lw + gap

    return _sketch(width_mm, h_mm, paint), scale


def write_pdf(path: Path, job: mjob.Job, cfg: dict, job_path: Path, cfg_path: Path | None = None) -> dict:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.styles import getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.platypus import (BaseDocTemplate, Frame, KeepTogether, NextPageTemplate, PageBreak, PageTemplate,
                                    Paragraph, Spacer, Table, TableStyle)

    c = counts(job)
    plan = loading_plan(job)
    legs = sorted({lg for lg, _, _ in c})
    courses = sorted({k for _, k, _ in c})
    status = mjob.config_status(cfg_path, mconfig.variant_of(cfg))
    tag = lambda key: STATUS_DE.get(status.get(key, {}).get("status", "UNTAGGED"), "?")  # noqa: E731
    b, hb = cfg["brick"], cfg["half_brick"]
    n_kind = {k: sum(v for (_, _, kk), v in c.items() if kk == k) for k in KINDS}

    ss = getSampleStyleSheet()
    H1, H2, P = ss["Title"], ss["Heading2"], ss["BodyText"]
    small = ss["BodyText"].clone("small", fontSize=8, leading=10)
    grid = TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.grey), ("BACKGROUND", (0, 0), (-1, 0), colors.lightgrey),
                       ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"), ("FONTSIZE", (0, 0), (-1, -1), 9),
                       ("ALIGN", (1, 0), (-1, -1), "CENTER"), ("VALIGN", (0, 0), (-1, -1), "MIDDLE")])
    total_row = [("FONTNAME", (0, -1), (-1, -1), "Helvetica-Bold"), ("LINEABOVE", (0, -1), (-1, -1), 1.0, colors.black)]

    def table(rows, widths, extra=()):
        t = Table(rows, colWidths=[w * mm for w in widths], repeatRows=1)
        t.setStyle(TableStyle(list(grid.getCommands()) + list(extra)))
        return t

    def n0_de(v) -> str:                         # 5.5 -> "5½" (a leg ending with a half stone)
        f = float(v)
        return f"{int(f)}½" if abs(f - int(f) - 0.5) < 1e-9 else f"{f:g}"

    def trip_bg(rows_recs, col0: int = 0):       # background of the first column = trip colour
        return [("BACKGROUND", (col0, i), (col0, i), colors.Color(*trip_rgb(r["trip"])))
                for i, r in enumerate(rows_recs, 1) if r is not None and r.get("nr") is not None]

    leg_txt = " / ".join(f"{lg} {n0_de(next(x['n0'] for x in cfg['wall']['legs'] if x['name'] == lg))}"
                         for lg in legs if lg) if any(legs) else "gerade Wand"
    variant = mconfig.variant_of(cfg)
    src = "config/station.toml" + (f" + config/variants/{variant}.toml" if variant else "")
    story = [Paragraph(f"Steinliste – {cfg['wall'].get('shape', '')}-Mauer"
                       + (f" (Variante {variant})" if variant else ""), H1),
             Paragraph(f"Schenkel {leg_txt} Steine in der untersten Lage, {len(courses)} Lagen. Erzeugt am "
                       f"{datetime.now():%d.%m.%Y %H:%M} mit <font name='Courier'>tools/make_stone_list.py</font> "
                       f"aus <font name='Courier'>{job_path.as_posix()}</font> (Job vom "
                       f"{datetime.fromisoformat(job.created):%d.%m.%Y %H:%M}). Planungsstand: die Zahlen hängen von "
                       "den Werten in Abschnitt 8 ab; ANNAHME / PLATZHALTER sind nicht gemessen.", P),
             Spacer(1, 4 * mm),
             Paragraph("1. Bedarf für die Mauer", H2)]
    rows = [["Schenkel", "Vollsteine", "Halbsteine", "Summe"]]
    for lg in legs:
        f = sum(v for (l_, _, k), v in c.items() if l_ == lg and k == "full")
        h = sum(v for (l_, _, k), v in c.items() if l_ == lg and k == "half")
        rows.append([lg or "Wand", f, h, f + h])
    rows.append(["Gesamt", n_kind["full"], n_kind["half"], sum(n_kind.values())])
    story += [table(rows, (35, 30, 30, 30), total_row), Spacer(1, 3 * mm)]
    mass_f = float(b.get("mass_kg", 0.0) or 0.0)
    mass_h = float(hb.get("mass_kg", 0.0) or 0.0)
    m_line = (f"Masse: Vollstein {_num(mass_f)} kg ({tag('[brick] mass_kg')}) → {n_kind['full']} Vollsteine ≈ "
              f"{_de(n_kind['full'] * mass_f, 0)} kg; " if mass_f else "Masse Vollstein: unbekannt; ")
    m_line += (f"Halbstein {_num(mass_h)} kg ({tag('[half_brick] mass_kg')})." if mass_h else
               f"Halbstein: Masse {tag('[half_brick] mass_kg').lower()} (nicht mitgerechnet).")
    story += [Paragraph(f"Maße (L × B × H): Vollstein {_num(b['length'])} × {_num(b['width'])} × {_num(b['height'])} mm "
                        f"({tag('[brick] length')}); Halbstein {_num(hb['length'])} × {_num(hb['width'])} × "
                        f"{_num(hb['height'])} mm (Länge {tag('[half_brick] length')}). {m_line}", P),
              Spacer(1, 3 * mm), Paragraph("2. Je Lage (Lage 1 = unterste, steht auf den Bodenführungen)", H2)]
    rows = [["Lage"] + [f"{lg or 'Wand'} {KIND_DE[k]}" for lg in legs for k in KINDS] + ["Summe"]]
    for k_ in courses:
        vals = [c.get((lg, k_, k), 0) for lg in legs for k in KINDS]
        rows.append([f"Lage {k_ + 1}"] + vals + [sum(vals)])
    rows.append(["Gesamt"] + [sum(c.get((lg, k_, k), 0) for k_ in courses) for lg in legs for k in KINDS]
                + [sum(c.values())])
    story += [table(rows, [22] + [20] * (2 * len(legs)) + [20], total_row), Spacer(1, 3 * mm)]

    put = {k: plan["magazine"][k] + plan["station"][k] + sum(tr["topup"][k] for tr in plan["trips"]) for k in KINDS}
    story += [Paragraph("3. Beladeplan", H2),
              Paragraph("ARES setzt das Magazin leer und fährt dann zur Abholstation; dort lädt der UR5 die nächsten "
                        "Steine in Setzreihenfolge um, so viele wie ins Magazin passen (je Typ eigene Plätze) und in "
                        "der Station liegen.", P), Spacer(1, 1 * mm),
              Paragraph(f"<b>Start:</b> Magazin auf ARES {_n(plan['magazine'])}, Abholstation voll "
                        f"{_n(plan['station'])} → zu Beginn {sum(plan['magazine'].values()) + sum(plan['station'].values())}"
                        " Steine bereitlegen.", P), Spacer(1, 2 * mm),
              KeepTogether([Paragraph("<b>Magazin auf ARES füllen</b> (Draufsicht, Maßstab 1:8; je Halter von oben "
                                      "nach unten: Lage → Nr. des Steins in der Setzreihenfolge, Abschnitt 7):", P),
                            Spacer(1, 1 * mm), magazine_sketch(job, cfg, plan)]), Spacer(1, 2 * mm)]
    stacks = station_stacks(job)
    per_row = []                                 # one table per row of the station, side by side
    for row in sorted({s["row"] for s in stacks}):
        rows = [[f"Reihe {row} ({'vorne, an ARES' if row == 1 else 'hinten'})", "", "", ""],
                ["Stapel (Gravur)", "x [mm]", "Typ", "Lagen"]]
        rows += [[s["stack"], f"{s['x_mm']:.1f}", KIND_DE[s["kind"]], s["layers"] if s["layers"] else "leer lassen"]
                 for s in stacks if s["row"] == row]
        per_row.append(table(rows, (26, 18, 14, 22), [("SPAN", (0, 0), (-1, 0)),
                                                     ("FONTNAME", (0, 1), (-1, 1), "Helvetica-Bold"),
                                                     ("BACKGROUND", (0, 1), (-1, 1), colors.lightgrey)]))
    st_tab = Table([per_row], colWidths=[85 * mm] * len(per_row))
    st_tab.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0)]))
    story += [Paragraph("<b>Abholstation füllen:</b> jeder Stapel genau so hoch wie angegeben (Reihe 1 = an ARES; "
                        "x ab der linken Kante der MDF-Platte wie im Bodenplan; Zeichnung in Abschnitt 4). Ein Stein "
                        "darüber ist der Software unbekannt und liegt den Greifbacken im Weg.", P), Spacer(1, 1 * mm),
              st_tab, Spacer(1, 2 * mm)]
    rows = [["Fahrt", "ab Halt (Schenkel)", "vorher in die Station nachlegen", "Station → Magazin"]]
    for i, tr in enumerate(plan["trips"], 1):
        rows.append([i, f"{tr['stop']} ({tr['leg'] or '-'})", _n(tr["topup"]) if any(tr["topup"].values()) else "–",
                     _n(tr["moved"])])
    story += [table(rows, (15, 35, 60, 50),
                    [("BACKGROUND", (0, i), (0, i), colors.Color(*trip_rgb(i))) for i in range(1, len(rows))]),
              Spacer(1, 2 * mm),
              Paragraph(f"Die Software geht davon aus, dass beim Nachlegen <b>jede leere Halterung</b> der Station "
                        f"gefüllt wird. Damit werden insgesamt <b>{_n(put)}</b> Steine bereitgelegt; am Ende bleiben "
                        f"{_n(plan['left'])} unbenutzt in der Station. Die Mauer selbst braucht "
                        f"{_n(n_kind)}.", P)]

    # 4. station drawing, 5. wall drawing (landscape)
    use = station_use(job, plan)
    story += [NextPageTemplate("quer"), PageBreak(),
              Paragraph("4. Zeichnung Abholstation (Draufsicht, Maßstab 1:7)", H2),
              Paragraph("Von ARES aus gesehen: unten die Seite, an der ARES andockt, x nach rechts ab der linken Kante "
                        "der MDF-Platte. Je Stapel: <b>Name (wie graviert) · Anzahl Lagen</b>, darunter je Lage von "
                        "oben nach unten <b>Lage → Nr.</b> des Steins in der Setzreihenfolge"
                        + (" und Schenkel / Mauerlage" if any(legs) else "") + "; das Farbfeld zeigt die "
                        "Stationsfahrt, mit der der Stein ins Magazin kommt. Dieselbe Nr. steht in der Mauer "
                        "(Abschnitt 5) und in den Tabellen (6, 7).", small),
              Spacer(1, 2 * mm), station_sketch(job, cfg, plan)]
    wall, wscale = wall_sketch(job, cfg, plan)
    story += [PageBreak(),
              Paragraph(f"5. Mauer abgewickelt – wo jeder Stein hinkommt (Ansicht von außen, Maßstab 1:{wscale:g})",
                        H2),
              Paragraph("Jeder Stein: <b>Nr.</b> in der Setzreihenfolge, darunter seine Herkunft: <b>Stapel · Lage</b> "
                        "in der Abholstation (z. B. s01·5 = Stapel s01, Lage 5) oder <b>M</b> + Magazinplatz für die "
                        "Startfüllung. Farbe = Stationsfahrt (Legende). Ansicht von außen, u wächst nach rechts; "
                        "L1 = unterste Lage.", small),
              Spacer(1, 3 * mm), wall, Spacer(1, 2 * mm)]
    legend = _sketch(265.0, 9.0, lambda cv, M: _legend(cv, M, 0.0, 5.0, plan, 265.0))
    story += [legend, NextPageTemplate("hoch"), PageBreak()]

    # 6. station -> wall table
    st = job.station
    has_topup = any(any(tr["topup"].values()) for tr in plan["trips"])
    head = ["Stapel", "Reihe", "Lage"] + (["Füllung"] if has_topup else []) + \
        ["Fahrt", "Magazin", "Nr.", "Schenkel", "Mauerlage", "u [mm]", "Kennung"]
    rows, recs = [head], []
    for s in stacks:
        for sl in sorted((x for x in st.slots if x.stack_id == s["stack"] and x.id in st.take_order),
                         key=lambda x: -x.layer):
            for r in use.get(sl.id, []):
                fill = ("Start" if not r["fill"] else f"vor Fahrt {r['fill']}") if has_topup else None
                if r["nr"] is None:
                    rows.append([s["stack"], s["row"], sl.layer] + ([fill] if has_topup else [])
                                + ["–", "–", "–", "bleibt übrig", "", "", ""])
                    recs.append(None)
                    continue
                rows.append([s["stack"], s["row"], sl.layer] + ([fill] if has_topup else [])
                            + [r["trip"], slot_txt(r["mag"]), r["nr"], r["leg"] or "–", r["course"] + 1,
                               f"{r['u_mm']:.0f}", r["label"]])
                recs.append(r)
    widths = [16, 13, 12] + ([20] if has_topup else []) + [13, 20, 12, 19, 20, 16, 20]
    story += [Paragraph("6. Zuordnung Abholstation → Mauer", H2),
              Paragraph("Je Stapel von der obersten Lage nach unten (in dieser Reihenfolge nimmt der UR5 sie). "
                        "Nr. = Setzreihenfolge (Abschnitt 7), u = Mitte des Steins entlang des Schenkels ab dessen "
                        "Anfang (Bodenkarte), Mauerlage 1 = unterste. Magazin = Platz auf ARES, über den der Stein "
                        "läuft (Halter · Lage).", P), Spacer(1, 2 * mm),
              table(rows, widths, trip_bg(recs, col0=3 + has_topup)), Spacer(1, 3 * mm)]
    ms = magazine_start(job, plan)
    rows, recs = [["Magazinplatz", "Lage", "Nr.", "Schenkel", "Mauerlage", "u [mm]", "Typ", "Kennung"]], []
    for sid in sorted(ms, key=lambda s: (job.magazine.slot(s).stack, -job.magazine.slot(s).layer)):
        r, sl = ms[sid], job.magazine.slot(sid)
        rows.append([sl.stack, sl.layer, r["nr"], r["leg"] or "–", r["course"] + 1, f"{r['u_mm']:.0f}",
                     KIND_DE[r["kind"]], r["label"]])
        recs.append(r)
    story += [Paragraph("Magazin-Startfüllung → Mauer", ss["Heading3"]),
              table(rows, (26, 14, 14, 22, 20, 18, 14, 24), trip_bg(recs, col0=2)), Spacer(1, 2 * mm),
              Paragraph("<b>Gilt, solange der Lauf nach Plan geht.</b> Der Sequencer nimmt dieselben Plätze "
                        "(gleiche Regel wie hier). Wird ein Lauf an einer späteren Stelle wieder aufgenommen, ein "
                        "Stein von Hand als gesetzt gemeldet oder weicht ein Magazinplatz ab (Eintrag "
                        "<font name='Courier'>slot_changed</font> im Lauf-Log), verschiebt sich die Zuordnung ab dort.",
                        P)]

    # 7. laying order with origin, 8. dependencies
    story += [PageBreak(), Paragraph("7. Setzreihenfolge (alle Steine) mit Herkunft", H2),
              Paragraph("u = Mitte des Steins entlang des Schenkels ab dessen Anfang (Bodenkarte). Kennung und "
                        "Magazinplatz wie in der Software (Halter · Lage). Herkunft: Stapel und Lage in der "
                        "Abholstation und die Fahrt, mit der der Stein ins Magazin kommt, oder Startfüllung des "
                        "Magazins.", P), Spacer(1, 2 * mm)]
    rows = [["Nr.", "Halt", "Schenkel", "Lage", "u [mm]", "Typ", "Kennung", "Magazin", "Herkunft"]]
    for r in plan["stones"]:
        rows.append([r["nr"], r["stop"], r["leg"] or "–", r["course"] + 1, f"{r['u_mm']:.0f}", KIND_DE[r["kind"]],
                     r["label"], slot_txt(r["mag"]), Paragraph(origin_de(r, job), small)])
    story += [table(rows, (11, 11, 16, 11, 15, 12, 20, 18, 66), trip_bg(plan["stones"])),
              PageBreak(), Paragraph(f"8. Abhängigkeiten ({src})", H2)]
    rows = [["Wert", "Status", "Inhalt"]]
    for key in DEPENDS:
        keys = [key.format(leg=lg) for lg in legs if lg] if "{leg}" in key else [key]
        for k in keys:
            sec, _, name = k.partition("] ")
            if k.startswith("[[wall.legs]]"):
                lg, attr = name.split(".")
                val = next(x[attr] for x in cfg["wall"]["legs"] if x["name"] == lg)
            else:
                val = cfg.get(sec.strip("["), {}).get(name, "–")
            rows.append([Paragraph(k, small), tag(k), Paragraph(str(val), small)])
    story += [table(rows, (55, 28, 95))]

    def footer(width):
        def draw(canv, doc):
            canv.saveState()
            canv.setFont("Helvetica", 7)
            canv.drawString(15 * mm, 8 * mm, f"ARES_Mauer – Steinliste – {job_path.name}")
            canv.drawRightString(width - 15 * mm, 8 * mm, f"Seite {doc.page}")
            canv.restoreState()
        return draw

    path.parent.mkdir(parents=True, exist_ok=True)
    pw, ph = A4
    qw, qh = landscape(A4)
    doc = BaseDocTemplate(str(path), pagesize=A4, title="Steinliste", author="ARES_Mauer")
    pad = dict(leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
    doc.addPageTemplates([
        PageTemplate("hoch", [Frame(15 * mm, 15 * mm, pw - 30 * mm, ph - 30 * mm, **pad)], onPage=footer(pw),
                     pagesize=A4),
        PageTemplate("quer", [Frame(15 * mm, 15 * mm, qw - 30 * mm, qh - 30 * mm, **pad)], onPage=footer(qw),
                     pagesize=landscape(A4))])
    doc.build(story)
    return {"counts": n_kind, "plan": plan, "put_out": put, "stacks": stacks}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--job", type=Path, default=None,
                    help="job file (default data/jobs/nominal_<[wall] shape>[_<variant>].json)")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--variant", default=None, help="config variant config/variants/<VARIANT>.toml")
    ap.add_argument("--out", type=Path, default=None, help="default results/steinliste[_<variant>].pdf")
    a = ap.parse_args(argv)
    cfg = mconfig.load(a.config, a.variant)
    sfx = mconfig.suffix(cfg)
    a.job = a.job or REPO / "data" / "jobs" / f"nominal_{cfg['wall'].get('shape') or 'legs'}{sfx}.json"
    a.out = a.out or REPO / "results" / f"steinliste{sfx}.pdf"
    job = mjob.load(a.job)
    if (job.meta.get("config_variant") or None) != mconfig.variant_of(cfg):
        raise SystemExit(f"{a.job} was built with config variant {job.meta.get('config_variant')!r}, not "
                         f"{mconfig.variant_of(cfg)!r} - pass the matching --variant / --job")
    if job.config_sha256 and job.config_sha256 != mjob.config_sha256(a.config, mconfig.variant_of(cfg)):
        raise SystemExit(f"{a.job} was built from an older config (sha256 differs) - rebuild it with "
                         "tools/make_job.py" + (f" --variant {mconfig.variant_of(cfg)}" if mconfig.variant_of(cfg)
                                                else ""))
    job_path = a.job.resolve().relative_to(REPO) if a.job.resolve().is_relative_to(REPO) else a.job
    info = write_pdf(a.out, job, cfg, Path(job_path), a.config)
    print(f"written {a.out}: {_n(info['counts'])} stones in the wall, {len(info['plan']['trips'])} station trips, "
          f"{_n(info['put_out'])} put out (left in the station: {_n(info['plan']['left'])})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
