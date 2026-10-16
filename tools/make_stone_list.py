"""Stone list (German, PDF): what the wall of a job needs, how the stones get onto ARES and where each one goes.

    py.exe tools/make_stone_list.py [--job data/jobs/nominal_C.json] [--out results/steinliste.pdf] [--final]
                                    [--check-log data/runs/<run>/run.jsonl]

Everything comes from the job (tools/make_job.py) and config/station.toml:
- stones per leg, course and type (full / half) and their total;
- the loading plan by the sequencer's rule, replayed with the sequencer's own functions (stone_flow): the magazine
  starts with job.magazine.initial_fill; every stone is taken by SlotState.next_take(stone.slot, kind) as in
  mauer.sequencer.Sequencer._place; the empty magazine is refilled by mauer.job.reload_plan, after a station top-up
  when mauer.job.reload_short says so. The sequencer assumes the operator fills EVERY empty station holder
  (Sequencer._reload: SlotState.station(...) = full station after the refill), so the stones put out exceed the wall
  by what is left in the station at the end;
- the station and magazine plan with every slot name (tools/make_station_map.py, the pages of
  results/stationsplan.pdf);
- the start fill of the magazine (magazine slot -> wall stone), every station trip (station slot -> magazine slot ->
  wall stone, in transfer order) and every stone in laying order (stop, leg, course, position, centre along the leg,
  type, magazine slot, where it came from);
- masses ([brick] / [half_brick] mass_kg) and the status tags of the config values the list depends on (CLAUDE.md).
--check-log compares the list with the run log of a run from the job's first stone (mauer.sequencer RunLog run.jsonl,
real run or ARES tour): every placed / tour_skip stone with its magazine slot, the start fill, every reload_start plan,
reload_transfer count and station refill. Mismatches are printed, listed in the PDF and give exit code 1.
--final also copies the PDF into final/.
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO, REPO / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import make_station_map as msm  # noqa: E402

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


def stone_flow(job: mjob.Job) -> dict:
    """Where every stone comes from and goes to, by the sequencer's rule (module docstring; the same functions as
    mauer.sequencer Sequencer._place / _reload and tools/make_job.py plan_slots):
    {"stones": [rec, ...] in laying order, rec = {"n" (1-based), "stop" (index into job.stops, as the run log), "leg",
       "course", "index", "u_mm", "kind", "label", "key", "mag" (magazine slot it is taken from), "trip" (None = start
       fill, else the station trip that brought it, 1-based), "station" (its station slot, None for the start fill)};
     "start": [{"mag", "kind", "stone": rec}] the start fill of the magazine in fill order (bottom layer first);
     "trips": [{"n", "stop", "leg", "first" (n of the first stone placed after it), "refill" [station slots the operator
       fills before the trip = every empty holder], "topup" {kind: n}, "moves": [{"station", "mag", "kind", "stone":
       rec}] in transfer order}];
     "station": {kind: n} of the full station, "left": {kind: n} and "left_slots" still in the station at the end}.
    ValueError when the job cannot be laid this way (a stone without a slot, an impossible reload)."""
    mag = mjob.SlotState.magazine(job.magazine)
    full = mjob.SlotState.station(job.station)
    st = full.copy()
    start = [{"mag": sid, "kind": mag.kinds.get(sid, "full"), "stone": None}
             for sid in job.magazine.fill_order if sid in mag.filled]
    src: dict[str, dict] = {s["mag"]: s for s in start}         # magazine slot -> the entry that filled it
    kinds = [t.kind for t in job.stones()]
    recs: list[dict] = []
    trips: list[dict] = []
    for k, stop in enumerate(job.stops):
        for t in stop.stones:
            j = len(recs)
            if mag.empty():
                trip = {"n": len(trips) + 1, "stop": k, "leg": stop.leg or "", "first": j + 1, "refill": [],
                        "topup": {kd: 0 for kd in KINDS}, "moves": []}
                if st.empty() or mjob.reload_short(st, full, mag, kinds[j:]):
                    trip["refill"] = [s for s in full.take_order if s not in st.filled]
                    trip["topup"] = {kd: full.count(kd) - st.count(kd) for kd in KINDS}
                    st = full.copy()
                pairs = mjob.reload_plan(mag, st, kinds[j:])
                if not pairs:
                    raise ValueError(f"stone {t.label}: magazine empty and no reload possible")
                for ssid, mid, kind in pairs:
                    st.take(ssid)
                    mag.fill(mid, kind)
                    mv = {"station": ssid, "mag": mid, "kind": kind, "stone": None, "trip": trip["n"]}
                    trip["moves"].append(mv)
                    src[mid] = mv
                trips.append(trip)
            sid = mag.next_take(t.slot, kind=t.kind)
            if sid is None:
                raise ValueError(f"stone {t.label}: no {t.kind} stone can be taken from the magazine")
            mag.take(sid)
            how = src.pop(sid)
            rec = {"n": j + 1, "stop": k, "leg": t.leg or "", "course": t.course, "index": t.index,
                   "u_mm": float(t.u_mm), "kind": t.kind, "label": t.label, "key": tuple(t.key), "mag": sid,
                   "trip": how.get("trip"), "station": how.get("station")}
            how["stone"] = rec
            recs.append(rec)
    return {"stones": recs, "start": start, "trips": trips, "station": {kd: full.count(kd) for kd in KINDS},
            "left": {kd: st.count(kd) for kd in KINDS}, "left_slots": [s for s in full.take_order if s in st.filled]}


def loading_plan(job: mjob.Job, flow: dict | None = None) -> dict:
    """Counts of the start fill and the station trips (stone_flow):
    {"magazine": {kind: n}, "station": {kind: n}, "trips": [{"stop", "leg", "topup": {kind: n}, "moved": {kind: n}}],
     "left": {kind: n} in the station at the end}."""
    f = flow or stone_flow(job)
    return {"magazine": {k: sum(1 for s in f["start"] if s["kind"] == k) for k in KINDS}, "station": f["station"],
            "trips": [{"stop": tr["stop"], "leg": tr["leg"], "topup": tr["topup"],
                       "moved": {k: sum(1 for m in tr["moves"] if m["kind"] == k) for k in KINDS}}
                      for tr in f["trips"]],
            "left": f["left"]}


def compare_log(flow: dict, events: list[dict]) -> list[str]:
    """Differences between stone_flow and a run log (mauer.sequencer.read_log) of a run from the job's first stone, in
    German for the operator; [] = the log took every stone from the listed magazine slot and moved every listed stone
    station -> magazine. Checks: run_start magazine count = start fill, the placed / tour_skip stones in order (stop,
    stone, magazine slot), per station trip the reload_start (stop, plan [station slot, magazine slot, kind]), its
    reload_transfer count and whether the station was refilled before it."""
    bad: list[str] = []
    starts = [e for e in events if e.get("event") == "run_start"]
    if not starts:
        return ["kein run_start im Log"]
    later = [e["event"] for e in events if e.get("event") in ("restart_fill", "prebuilt", "declared_placed")]
    if starts[0].get("start_stop", 0) != 0 or later:
        bad.append(f"Lauf beginnt nicht mit dem ersten Stein des Jobs (start_stop {starts[0].get('start_stop')}"
                   + (f", {', '.join(sorted(set(later)))}" if later else "") + ") – Abgleich nur ab Jobbeginn")
    elif starts[0].get("magazine") != len(flow["start"]):
        bad.append(f"Startfüllung: Log {starts[0].get('magazine')} Steine im Magazin, Liste {len(flow['start'])}")
    got = [(e.get("stop"), tuple(e.get("stone") or ()), e.get("slot")) for e in events
           if e.get("event") in ("placed", "tour_skip")]
    want = [(r["stop"], r["key"], r["mag"]) for r in flow["stones"]]
    for i, (g, w) in enumerate(zip(got, want), 1):
        if g != w:
            bad.append(f"Stein Nr. {i}: Log Halt {g[0]}, {list(g[1])} aus {g[2]} – Liste Halt {w[0]}, {list(w[1])} "
                       f"aus {w[2]}")
    if len(got) != len(want):
        bad.append(f"Log: {len(got)} Steine, Liste: {len(want)}")
    trips: list[dict] = []
    refilled = False
    for e in events:
        ev = e.get("event")
        if ev == "station_refilled":
            refilled = True
        elif ev == "reload_start":
            rec = {"stop": e.get("stop"), "plan": [tuple(x) for x in e.get("plan") or []], "moved": None,
                   "refilled": refilled}
            refilled = False
            if e.get("resumed") and trips and trips[-1]["moved"] is None:      # the interrupted trip, continued
                rec["refilled"] = rec["refilled"] or trips[-1]["refilled"]
                trips[-1] = rec
            else:
                trips.append(rec)
        elif ev == "reload_transfer" and trips:
            trips[-1]["moved"] = e.get("moved")
    for tr, lg in zip(flow["trips"], trips):
        plan = [(m["station"], m["mag"], m["kind"]) for m in tr["moves"]]
        if lg["stop"] != tr["stop"]:
            bad.append(f"Fahrt {tr['n']}: Log ab Halt {lg['stop']}, Liste ab Halt {tr['stop']}")
        if lg["plan"] != plan:
            diff = [f"{i}. Log {list(a) if a else '–'} / Liste {list(b) if b else '–'}"
                    for i, (a, b) in enumerate(_zip_long(lg["plan"], plan), 1) if a != b]
            bad.append(f"Fahrt {tr['n']}: Umladeplan weicht ab: " + "; ".join(diff[:6]) + (" …" if len(diff) > 6
                                                                                            else ""))
        if lg["moved"] is None:
            bad.append(f"Fahrt {tr['n']}: kein reload_transfer im Log (Lauf abgebrochen?)")
        elif lg["moved"] != len(plan):
            bad.append(f"Fahrt {tr['n']}: Log {lg['moved']} umgeladen, Liste {len(plan)}")
        if lg["refilled"] != bool(tr["refill"]):
            bad.append(f"Fahrt {tr['n']}: Station nachgefüllt laut Log {'ja' if lg['refilled'] else 'nein'}, "
                       f"laut Liste {'ja' if tr['refill'] else 'nein'}")
    if len(trips) != len(flow["trips"]):
        bad.append(f"Log: {len(trips)} Stationsfahrten, Liste: {len(flow['trips'])}")
    return bad


def _zip_long(a: list, b: list) -> list[tuple]:
    n = max(len(a), len(b))
    return [(a[i] if i < len(a) else None, b[i] if i < len(b) else None) for i in range(n)]


def check_logs(flow: dict, paths: list[Path]) -> list[dict]:
    """[{"path", "what" (German description of the run), "n_stones", "n_trips", "problems"}] per run log."""
    from mauer.sequencer import read_log
    out = []
    for p in paths:
        ev = read_log(p)
        st = next((e for e in ev if e.get("event") == "run_start"), {})
        when = f" vom {datetime.fromtimestamp(float(st['t'])):%d.%m.%Y %H:%M}" if "t" in st else ""
        out.append({"path": p, "what": ("ARES-Tour" if st.get("tour") else "Lauf") + when,
                    "n_stones": sum(e.get("event") in ("placed", "tour_skip") for e in ev),
                    "n_trips": sum(e.get("event") == "reload_start" and not e.get("resumed") for e in ev),
                    "problems": compare_log(flow, ev)})
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


def _stone_cells(r: dict | None) -> list:
    """Kennung, Schenkel, Lage, Pos., u, Halt, Setz-Nr. of a stone record (1-based course and position)."""
    if r is None:
        return ["–"] * 7
    return [r["label"], r["leg"] or "–", r["course"] + 1, r["index"] + 1, f"{r['u_mm']:.0f}", r["stop"], r["n"]]


def write_pdf(path: Path, job: mjob.Job, cfg: dict, job_path: Path, cfg_path: Path | None = None,
              checks: list[dict] | None = None) -> dict:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.styles import getSampleStyleSheet
    from reportlab.lib.units import mm
    from reportlab.platypus import (BaseDocTemplate, CondPageBreak, Frame, KeepTogether, PageBreak, PageTemplate,
                                    Paragraph, Spacer, Table, TableStyle)

    c = counts(job)
    flow = stone_flow(job)
    plan = loading_plan(job, flow)
    legs = sorted({lg for lg, _, _ in c})
    courses = sorted({k for _, k, _ in c})
    status = mjob.config_status(cfg_path, mconfig.variant_of(cfg))
    tag = lambda key: STATUS_DE.get(status.get(key, {}).get("status", "UNTAGGED"), "?")  # noqa: E731
    b, hb = cfg["brick"], cfg["half_brick"]
    n_kind = {k: sum(v for (_, _, kk), v in c.items() if kk == k) for k in KINDS}

    ss = getSampleStyleSheet()
    H1, H2, H3, P = ss["Title"], ss["Heading2"], ss["Heading3"], ss["BodyText"]
    small = ss["BodyText"].clone("small", fontSize=8, leading=10)
    grid = TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.grey), ("BACKGROUND", (0, 0), (-1, 0), colors.lightgrey),
                       ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"), ("FONTSIZE", (0, 0), (-1, -1), 9),
                       ("ALIGN", (1, 0), (-1, -1), "CENTER"), ("VALIGN", (0, 0), (-1, -1), "MIDDLE")])
    dense = [("FONTSIZE", (0, 0), (-1, -1), 8), ("TOPPADDING", (0, 0), (-1, -1), 1.5),
             ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5), ("ALIGN", (0, 0), (-1, -1), "CENTER")]
    total_row = [("FONTNAME", (0, -1), (-1, -1), "Helvetica-Bold"), ("LINEABOVE", (0, -1), (-1, -1), 1.0, colors.black)]
    half_bg = colors.Color(0.99, 0.92, 0.82)

    def table(rows, widths, extra=()):
        t = Table(rows, colWidths=[w * mm for w in widths], repeatRows=1)
        t.setStyle(TableStyle(list(grid.getCommands()) + list(extra)))
        return t

    def half_rows(rows, kind_col: int, first: int = 1) -> list:
        """Background of the half-stone rows (kind in column kind_col)."""
        return [("BACKGROUND", (0, i), (-1, i), half_bg) for i, r in enumerate(rows) if i >= first
                and r[kind_col] == KIND_DE["half"]]

    def n0_de(v) -> str:                         # 5.5 -> "5½" (a leg ending with a half stone)
        f = float(v)
        return f"{int(f)}½" if abs(f - int(f) - 0.5) < 1e-9 else f"{f:g}"

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
                       "den Werten in Abschnitt 8 ab; ANNAHME / PLATZHALTER sind nicht gemessen.", P)]
    for chk in checks or []:
        ok = not chk["problems"]
        story.append(Paragraph(
            f"<b>Abgleich mit dem Lauf-Log</b> <font name='Courier'>{chk['path'].as_posix()}</font> ({chk['what']}): "
            + (f"<font color='#0d7a1a'><b>stimmt</b></font> – alle {chk['n_stones']} Steine kamen aus dem hier "
               f"genannten Magazinplatz, alle {chk['n_trips']} Stationsfahrten luden genau die hier genannten "
               "Stationsplätze in die genannten Magazinplätze um." if ok else
               f"<font color='#c00000'><b>{len(chk['problems'])} Abweichung(en)</b></font>: "
               + "; ".join(chk["problems"][:12]) + (" …" if len(chk["problems"]) > 12 else "")), small))
    story += [Spacer(1, 4 * mm), Paragraph("1. Bedarf für die Mauer", H2)]
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
                        "der Station liegen. Welcher Stein wo liegt und wohin er kommt: Abschnitte 5 – 7, die "
                        "Platznamen: Abschnitt 4.", P), Spacer(1, 1 * mm),
              Paragraph(f"<b>Start:</b> Magazin auf ARES {_n(plan['magazine'])}, Abholstation voll "
                        f"{_n(plan['station'])} → zu Beginn {sum(plan['magazine'].values()) + sum(plan['station'].values())}"
                        " Steine bereitlegen.", P), Spacer(1, 2 * mm)]
    stacks = station_stacks(job)
    rows = [["Stapel (Gravur)", "Reihe", "x [mm]", "Typ", "Lagen"]]
    rows += [[s["stack"], f"{s['row']} ({'vorne' if s['row'] == 1 else 'hinten'})", f"{s['x_mm']:.1f}",
              KIND_DE[s["kind"]], s["layers"] if s["layers"] else "leer lassen"] for s in stacks]
    story += [Paragraph("<b>Abholstation füllen:</b> jeder Stapel genau so hoch wie angegeben (Reihe 1 = an ARES; "
                        "x ab der linken Kante der MDF-Platte wie im Bodenplan). Ein Stein darüber ist der Software "
                        "unbekannt und liegt den Greifbacken im Weg.", P), Spacer(1, 1 * mm),
              table(rows, (35, 30, 22, 18, 22)), Spacer(1, 3 * mm),
              Paragraph("<b>Stationsfahrten</b> (Einzelheiten: Abschnitt 6):", P), Spacer(1, 1 * mm)]
    rows = [["Fahrt", "ab Halt (Schenkel)", "vor Stein Nr.", "vorher in die Station nachlegen", "Station → Magazin"]]
    for tr, tp in zip(flow["trips"], plan["trips"]):
        rows.append([tr["n"], f"{tr['stop']} ({tr['leg'] or '-'})", tr["first"],
                     _n(tp["topup"]) if any(tp["topup"].values()) else "–", _n(tp["moved"])])
    story += [table(rows, (14, 32, 24, 55, 45)), Spacer(1, 2 * mm),
              Paragraph(f"Die Software geht davon aus, dass beim Nachlegen <b>jede leere Halterung</b> der Station "
                        f"gefüllt wird. Damit werden insgesamt <b>{_n(put)}</b> Steine bereitgelegt; am Ende bleiben "
                        f"{_n(plan['left'])} unbenutzt in der Station"
                        + (f" ({', '.join(flow['left_slots'])})" if flow["left_slots"] else "")
                        + f". Die Mauer selbst braucht {_n(n_kind)}.", P)]

    # 4. slot names: the plan pages of tools/make_station_map.py (landscape drawings, portrait tables)
    qw, qh = landscape(A4)
    m_q = 12 * mm
    story += msm.story(job, cfg, qw - 2 * m_q - 12, qh - 2 * m_q - 12, landscape_template="quer",
                       portrait_template="hoch", number="4")

    # 5. start fill of the magazine
    head = ["Nr.", "Magazinplatz", "Typ", "Stein (Kennung)", "Schenkel", "Lage", "Pos.", "u [mm]", "Halt", "Setz-Nr."]
    rows = [head]
    for i, s in enumerate(flow["start"], 1):
        rows.append([i, s["mag"], KIND_DE.get(s["kind"], s["kind"])] + _stone_cells(s["stone"]))
    story += [CondPageBreak(90 * mm), Paragraph("5. Startfüllung des Magazins (vor dem Lauf, von Hand)", H2),
              Paragraph(f"{len(flow['start'])} Steine, in dieser Reihenfolge einlegen (unterste Lage zuerst). Rechts "
                        "steht, welcher Stein der Mauer daraus wird. Lage und Pos. ab 1 gezählt (Pos. = Stein Nr. in "
                        "seiner Lage ab Schenkelanfang); u = Steinmitte entlang des Schenkels ab dessen Anfang. Die "
                        "Kennung zählt wie die Software ab 0: <font name='Courier'>Bc1i0h</font> = Schenkel B, Lage "
                        "c1 (= 2), Pos. i0 (= 1), h = Halbstein.", small), Spacer(1, 1.5 * mm),
              table(rows, (10, 25, 12, 25, 17, 12, 12, 16, 12, 16), dense + half_rows(rows, 2)), Spacer(1, 3 * mm)]

    # 6. station trips
    story += [CondPageBreak(60 * mm), Paragraph("6. Stationsfahrten: Stationsplatz → Magazinplatz → Stein", H2),
              Paragraph("Je Fahrt in der Reihenfolge, in der der UR umlädt (oberste Lagen zuerst). Die Steine werden "
                        "danach in Setzreihenfolge gesetzt (Spalte Setz-Nr., Abschnitt 7).", small),
              Spacer(1, 1.5 * mm)]
    head = ["Nr.", "Stationsplatz", "Magazinplatz", "Typ", "Stein (Kennung)", "Schenkel", "Lage", "Pos.", "u [mm]",
            "Halt", "Setz-Nr."]
    for tr in flow["trips"]:
        rows = [head]
        for i, mv in enumerate(tr["moves"], 1):
            rows.append([i, mv["station"], mv["mag"], KIND_DE.get(mv["kind"], mv["kind"])] + _stone_cells(mv["stone"]))
        moved = {k: sum(1 for m in tr["moves"] if m["kind"] == k) for k in KINDS}
        last = tr["first"] + len(tr["moves"]) - 1
        txt = (f"<b>Fahrt {tr['n']}</b> – ARES fährt von Halt {tr['stop']} (Schenkel {tr['leg'] or '-'}) zur Station, "
               f"vor Stein Nr. {tr['first']}; {_n(moved)} Steine für die Setz-Nr. {tr['first']} – {last}.")
        if tr["refill"]:
            txt += (f" <b>Vorher nachlegen</b> ({_n(tr['topup'])}): " + ", ".join(tr["refill"]) + ".")
        story += [KeepTogether([Paragraph(txt, small), Spacer(1, 1 * mm),
                                table(rows, (9, 22, 22, 11, 24, 16, 11, 11, 15, 11, 15),
                                      dense + half_rows(rows, 3))]), Spacer(1, 3 * mm)]
    if not flow["trips"]:
        story.append(Paragraph("Keine Stationsfahrt: das Magazin reicht für die ganze Mauer.", small))

    # 7. laying order
    story += [PageBreak(), Paragraph("7. Setzreihenfolge (alle Steine)", H2),
              Paragraph("Kennung und Magazinplatz wie in der Software (Kennung ab 0 gezählt, Lage / Pos. hier ab 1). "
                        "„Herkunft“: Start = Startfüllung des Magazins, sonst die Fahrt und der Stationsplatz, aus "
                        "dem der Stein kam. Graue Zeilen: hier fährt ARES zur Station.", small), Spacer(1, 1.5 * mm)]
    rows = [["Nr.", "Halt", "Schenkel", "Lage", "Pos.", "u [mm]", "Typ", "Kennung", "Magazinplatz", "Herkunft"]]
    extra = []
    first = {tr["first"]: tr for tr in flow["trips"]}
    for r in flow["stones"]:
        if r["n"] in first:
            tr = first[r["n"]]
            extra += [("SPAN", (0, len(rows)), (-1, len(rows))), ("BACKGROUND", (0, len(rows)), (-1, len(rows)),
                                                                  colors.Color(0.85, 0.85, 0.85)),
                      ("FONTNAME", (0, len(rows)), (-1, len(rows)), "Helvetica-Bold")]
            rows.append([f"Stationsfahrt {tr['n']} ab Halt {tr['stop']}: {len(tr['moves'])} Steine umladen"
                         + (" (vorher Station nachfüllen)" if tr["refill"] else "")] + [""] * 9)
        rows.append([r["n"], r["stop"], r["leg"] or "–", r["course"] + 1, r["index"] + 1, f"{r['u_mm']:.0f}",
                     KIND_DE[r["kind"]], r["label"], r["mag"],
                     "Start" if r["trip"] is None else f"Fahrt {r['trip']}: {r['station']}"])
    extra += [("BACKGROUND", (0, i), (-1, i), half_bg) for i, r in enumerate(rows) if i and r[6] == KIND_DE["half"]]
    story.append(table(rows, (11, 11, 16, 11, 11, 15, 12, 22, 24, 34), dense + extra))

    # 8. dependencies
    story += [CondPageBreak(80 * mm), Spacer(1, 4 * mm), Paragraph(f"8. Abhängigkeiten ({src})", H2)]
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
    story.append(table(rows, (55, 28, 95)))
    if any(chk["problems"] for chk in checks or []):
        story += [Spacer(1, 4 * mm), Paragraph("Abweichungen zum Lauf-Log", H3)]
        for chk in checks:
            for p in chk["problems"]:
                story.append(Paragraph(f"<font name='Courier'>{chk['path'].name}</font>: {p}", small))

    def footer(canv, doc):
        canv.saveState()
        canv.setFont("Helvetica", 7)
        w = canv._pagesize[0]
        canv.drawString(12 * mm, 6 * mm, f"ARES_Mauer – Steinliste – {job_path.name}")
        canv.drawRightString(w - 12 * mm, 6 * mm, f"Seite {doc.page}")
        canv.restoreState()

    pw, ph = A4
    hoch = PageTemplate("hoch", [Frame(15 * mm, 15 * mm, pw - 30 * mm, ph - 30 * mm, id="h")], onPage=footer,
                        pagesize=(pw, ph))
    quer = PageTemplate("quer", [Frame(m_q, m_q, qw - 2 * m_q, qh - 2 * m_q, id="q")], onPage=footer,
                        pagesize=(qw, qh))
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = BaseDocTemplate(str(path), pagesize=A4, title="Steinliste", author="ARES_Mauer")
    doc.addPageTemplates([hoch, quer])
    doc.build(story)
    return {"counts": n_kind, "plan": plan, "put_out": put, "stacks": stacks, "flow": flow}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--job", type=Path, default=None,
                    help="job file (default data/jobs/nominal_<[wall] shape>[_<variant>].json)")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--variant", default=None, help="config variant config/variants/<VARIANT>.toml")
    ap.add_argument("--out", type=Path, default=None, help="default results/steinliste[_<variant>].pdf")
    ap.add_argument("--check-log", type=Path, action="append", default=[], metavar="RUN_JSONL",
                    help="compare with the run log of a run from the job's first stone (repeatable)")
    ap.add_argument("--final", action="store_true", help="also copy the PDF into final/")
    a = ap.parse_args(argv)
    job, cfg, job_path = msm.load_checked(a.job, a.config, a.variant)
    a.out = a.out or REPO / "results" / f"steinliste{mconfig.suffix(cfg)}.pdf"
    checks = check_logs(stone_flow(job), a.check_log) if a.check_log else None
    for chk in checks or []:
        chk["path"] = chk["path"].resolve().relative_to(REPO) if chk["path"].resolve().is_relative_to(REPO) \
            else chk["path"]
    info = write_pdf(a.out, job, cfg, job_path, a.config, checks)
    print(f"written {a.out}: {_n(info['counts'])} stones in the wall, {len(info['plan']['trips'])} station trips, "
          f"{_n(info['put_out'])} put out (left in the station: {_n(info['plan']['left'])})")
    for chk in checks or []:
        print(f"check {chk['path']}: " + ("match - " + f"{chk['n_stones']} stones, {chk['n_trips']} station trips"
                                         if not chk["problems"] else f"{len(chk['problems'])} mismatch(es):"))
        for p in chk["problems"]:
            print(f"  - {p}")
    if a.final:
        print(f"copied to {msm.copy_final(a.out)}")
    return 1 if any(chk["problems"] for chk in checks or []) else 0


if __name__ == "__main__":
    raise SystemExit(main())
