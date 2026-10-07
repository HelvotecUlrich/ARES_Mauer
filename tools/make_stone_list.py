"""Stone list (German, PDF): what the wall of a job needs and how the stones get onto ARES.

    py.exe tools/make_stone_list.py [--job data/jobs/nominal_C.json] [--out results/steinliste.pdf]

Everything comes from the job (tools/make_job.py) and config/station.toml:
- stones per leg, course and type (full / half) and their total;
- the loading plan by the sequencer's rule (mauer.job.reload_plan / reload_short, the same loop as
  tools/make_job.py plan_slots): the start fill of the magazine and of the pick-up station, and per station trip the
  operator's top-up before it and the stones moved station -> magazine. The sequencer assumes the operator fills
  EVERY empty station holder (mauer.sequencer._reload: SlotState.station(...) = full station after the refill), so
  the stones put out exceed the wall by what is left in the station at the end;
- every stone in laying order (stop, leg, course, centre along the leg, type, magazine slot);
- masses ([brick] / [half_brick] mass_kg) and the status tags of the config values the list depends on (CLAUDE.md).
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from mauer import config as mconfig  # noqa: E402
from mauer import job as mjob  # noqa: E402

KINDS = ("full", "half")
STATUS_DE = {"CONFIRMED": "BESTÄTIGT", "ASSUMPTION": "ANNAHME", "PLACEHOLDER": "PLATZHALTER", "UNKNOWN": "UNBEKANNT",
             "UNTAGGED": "OHNE STATUS"}
KIND_DE = {"full": "Voll", "half": "Halb"}
DEPENDS = ["[wall] courses", "[[wall.legs]] {leg}.n0", "[brick] length", "[brick] width", "[brick] height",
           "[half_brick] length", "[brick] mass_kg", "[half_brick] mass_kg", "[deck] magazine_rows_dx",
           "[deck] magazine_y", "[deck] magazine_layers", "[deck] half_positions", "[pickup_station] slots_xy",
           "[pickup_station] slot_layers", "[pickup_station] half_slots_xy", "[pickup_station] half_slot_layers"]


def counts(job: mjob.Job) -> dict:
    """{(leg, course, kind): n} of the job's stones (leg "" for a straight wall)."""
    return dict(Counter((t.leg or "", t.course, t.kind) for t in job.stones()))


def loading_plan(job: mjob.Job) -> dict:
    """Start fill and station trips by the sequencer's rule (tools/make_job.py plan_slots):
    {"magazine": {kind: n}, "station": {kind: n}, "trips": [{"stop", "leg", "topup": {kind: n}, "moved": {kind: n}}],
     "left": {kind: n} in the station at the end}."""
    mag = mjob.SlotState.magazine(job.magazine)
    full = mjob.SlotState.station(job.station)
    st = full.copy()
    out = {"magazine": {k: mag.count(k) for k in KINDS}, "station": {k: st.count(k) for k in KINDS}, "trips": []}
    for stop in job.stops:
        upcoming_all = [t.kind for s in job.stops[stop.index:] for t in s.stones]
        for i, t in enumerate(stop.stones):
            if mag.empty():
                upcoming = upcoming_all[i:]
                topup = {k: 0 for k in KINDS}
                if mjob.reload_short(st, full, mag, upcoming):
                    topup = {k: full.count(k) - st.count(k) for k in KINDS}
                    st = full.copy()
                pairs = mjob.reload_plan(mag, st, upcoming)
                if not pairs:
                    raise ValueError(f"stone {t.label}: magazine empty and no reload possible")
                for ssid, mid, kind in pairs:
                    st.take(ssid)
                    mag.fill(mid, kind)
                out["trips"].append({"stop": stop.index, "leg": stop.leg or "", "topup": topup,
                                     "moved": {k: sum(1 for *_, x in pairs if x == k) for k in KINDS}})
            sid = mag.next_take(t.slot, kind=t.kind)
            if sid is None:
                raise ValueError(f"stone {t.label}: no {t.kind} stone in the magazine")
            mag.take(sid)
    out["left"] = {k: st.count(k) for k in KINDS}
    return out


def _de(x: float, nd: int = 1) -> str:
    return f"{x:.{nd}f}".replace(".", ",")


def _n(d: dict) -> str:
    """'12 (10 Voll, 2 Halb)' from {kind: n}."""
    parts = [f"{d[k]} {KIND_DE[k]}" for k in KINDS if d.get(k)]
    return f"{sum(d.values())}" + (f" ({', '.join(parts)})" if parts else "")


def write_pdf(path: Path, job: mjob.Job, cfg: dict, job_path: Path, cfg_path: Path | None = None) -> dict:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.platypus import PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

    c = counts(job)
    plan = loading_plan(job)
    legs = sorted({lg for lg, _, _ in c})
    courses = sorted({k for _, k, _ in c})
    status = mjob.config_status(cfg_path)
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

    leg_txt = " / ".join(f"{lg} {next(int(x['n0']) for x in cfg['wall']['legs'] if x['name'] == lg)}"
                         for lg in legs if lg) if any(legs) else "gerade Wand"
    story = [Paragraph(f"Steinliste – {cfg['wall'].get('shape', '')}-Mauer", H1),
             Paragraph(f"Schenkel {leg_txt} Steine in der untersten Lage, {len(courses)} Lagen. Erzeugt am "
                       f"{datetime.now():%d.%m.%Y %H:%M} mit <font name='Courier'>tools/make_stone_list.py</font> "
                       f"aus <font name='Courier'>{job_path.as_posix()}</font> (Job vom "
                       f"{datetime.fromisoformat(job.created):%d.%m.%Y %H:%M}). Planungsstand: die Zahlen hängen von den Werten auf Seite 2 ab; "
                       "ANNAHME / PLATZHALTER sind nicht gemessen.", P),
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
    m_line = (f"Masse: Vollstein {_de(mass_f)} kg ({tag('[brick] mass_kg')}) → {n_kind['full']} Vollsteine ≈ "
              f"{_de(n_kind['full'] * mass_f, 0)} kg; " if mass_f else "Masse Vollstein: unbekannt; ")
    m_line += (f"Halbstein {_de(mass_h)} kg ({tag('[half_brick] mass_kg')})." if mass_h else
               f"Halbstein: Masse {tag('[half_brick] mass_kg').lower()} (nicht mitgerechnet).")
    story += [Paragraph(f"Maße (L × B × H): Vollstein {b['length']:.0f} × {b['width']:.0f} × {b['height']:.0f} mm "
                        f"({tag('[brick] length')}); Halbstein {hb['length']:.0f} × {hb['width']:.0f} × "
                        f"{hb['height']:.0f} mm (Länge {tag('[half_brick] length')}). {m_line}", P),
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
                        " Steine bereitlegen.", P), Spacer(1, 2 * mm)]
    rows = [["Fahrt", "ab Halt (Schenkel)", "vorher in die Station nachlegen", "Station → Magazin"]]
    for i, tr in enumerate(plan["trips"], 1):
        rows.append([i, f"{tr['stop']} ({tr['leg'] or '-'})", _n(tr["topup"]) if any(tr["topup"].values()) else "–",
                     _n(tr["moved"])])
    story += [table(rows, (15, 35, 60, 50)), Spacer(1, 2 * mm),
              Paragraph(f"Die Software geht davon aus, dass beim Nachlegen <b>jede leere Halterung</b> der Station "
                        f"gefüllt wird. Damit werden insgesamt <b>{_n(put)}</b> Steine bereitgelegt; am Ende bleiben "
                        f"{_n(plan['left'])} unbenutzt in der Station. Die Mauer selbst braucht "
                        f"{_n(n_kind)}.", P),
              PageBreak(),
              Paragraph("4. Abhängigkeiten (config/station.toml)", H2)]
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
    story += [table(rows, (55, 28, 95)), Spacer(1, 4 * mm),
              Paragraph("5. Setzreihenfolge (alle Steine)", H2),
              Paragraph("u = Mitte des Steins entlang des Schenkels ab dessen Anfang (Bodenkarte). Kennung und "
                        "Magazinplatz wie in der Software (Lage/Position dort ab 0 gezählt).", P), Spacer(1, 2 * mm)]
    rows = [["Nr.", "Halt", "Schenkel", "Lage", "u [mm]", "Typ", "Kennung", "Magazinplatz"]]
    n = 0
    for stop in job.stops:
        for t in stop.stones:
            n += 1
            rows.append([n, stop.index, t.leg or "–", t.course + 1, f"{t.u_mm:.0f}", KIND_DE[t.kind], t.label,
                         t.slot or "–"])
    story.append(table(rows, (12, 12, 18, 12, 18, 14, 24, 28)))

    def footer(canv, doc):
        canv.saveState()
        canv.setFont("Helvetica", 7)
        canv.drawString(15 * mm, 8 * mm, f"ARES_Mauer – Steinliste – {job_path.name}")
        canv.drawRightString(195 * mm, 8 * mm, f"Seite {doc.page}")
        canv.restoreState()

    path.parent.mkdir(parents=True, exist_ok=True)
    SimpleDocTemplate(str(path), pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm, topMargin=15 * mm,
                      bottomMargin=15 * mm, title="Steinliste", author="ARES_Mauer").build(
        story, onFirstPage=footer, onLaterPages=footer)
    return {"counts": n_kind, "plan": plan, "put_out": put}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--job", type=Path, default=REPO / "data" / "jobs" / "nominal_C.json")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=REPO / "results" / "steinliste.pdf")
    a = ap.parse_args(argv)
    job = mjob.load(a.job)
    cfg = mconfig.load(a.config)
    job_path = a.job.resolve().relative_to(REPO) if a.job.resolve().is_relative_to(REPO) else a.job
    info = write_pdf(a.out, job, cfg, Path(job_path), a.config)
    print(f"written {a.out}: {_n(info['counts'])} stones in the wall, {len(info['plan']['trips'])} station trips, "
          f"{_n(info['put_out'])} put out (left in the station: {_n(info['plan']['left'])})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
