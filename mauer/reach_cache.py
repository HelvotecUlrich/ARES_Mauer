"""Cache of the RoboDK reach tables (results/reach_table.json), shared by robodk/simulate.py (computes them),
robodk/look_study.py and tools/make_job.py (read them).

A reach table is {course: {u_rel: ok}}: can the UR5 place a full stone of that course at wall position u_rel
(relative to the ARES centre, 20 mm grid) from a stop at wall distance `dist` with the wall on `side` of ARES (front,
left, right, rear; robodk/simulate.py reach_table). Its cache key covers everything the table depends on: the UR5
mount, the tool, the stone (without [brick] rib_mm - the ribs only place the corner of a wall of legs, they are part of
the stone mesh the check uses), [wall] base_z / courses, [study] approach, the wall distance and the side (the side
enters the key only when it is not "front", so the front tables cached before 2026-10-06 keep their keys).

File layout (version 2, 2026-10-06 - one table per key, so several wall distances / sides live side by side):
    {"version": 2, "tables": {key: {"dist": mm, "side": "front" | "left" | ..., "table": {course: {u_rel: ok}}}}}
Version 1 (one table: {"key", "dist", "table"}) is still read.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Mapping

from . import REPO

PATH = REPO / "results" / "reach_table.json"


def key(cfg: Mapping, dist: float, side: str | None = None) -> str:
    """Cache key of the reach table for cfg at wall distance dist [mm], wall on `side` of ARES (default [wall] side)."""
    side = str(side or cfg["wall"].get("side", "front"))
    brick = {k: v for k, v in cfg["brick"].items() if k != "rib_mm"}
    parts = ["family-v2", cfg["ur5"], cfg["tool"], brick, cfg["wall"]["base_z"], cfg["wall"]["courses"],
             cfg["study"]["approach"], dist] + ([] if side == "front" else [side])
    src = json.dumps(parts, sort_keys=True)
    return hashlib.sha1(src.encode()).hexdigest()[:12]


def _parse(table: Mapping) -> dict:
    return {int(k): {float(u): bool(v) for u, v in d.items()} for k, d in table.items()}


def read(path: str | Path | None = None) -> dict[str, dict]:
    """{key: {"dist": mm, "table": {course: {u_rel: ok}}}} of every cached table ({} if the file is missing)."""
    p = Path(path or PATH)
    if not p.exists():
        return {}
    data = json.loads(p.read_text())
    if "tables" in data:
        items = data["tables"].items()
    else:                                              # version 1: a single table
        items = [(str(data.get("key")), {"dist": data.get("dist"), "table": data["table"]})]
    return {str(k): {"dist": None if v.get("dist") is None else float(v["dist"]), "side": str(v.get("side", "front")),
                     "table": _parse(v["table"])} for k, v in items}


def get(cfg: Mapping, dist: float, path: str | Path | None = None, side: str | None = None) -> dict | None:
    """The cached table for cfg at dist (wall on `side`), None if there is none for this key."""
    e = read(path).get(key(cfg, dist, side))
    return None if e is None else e["table"]


def store(cfg: Mapping, dist: float, table: Mapping, path: str | Path | None = None, side: str | None = None) -> str:
    """Add (or replace) the table of cfg at dist (wall on `side`); the other cached tables are kept. Returns its key."""
    p = Path(path or PATH)
    tables = {k: {"dist": v["dist"], "side": v["side"], "table": v["table"]} for k, v in read(p).items()}
    side = str(side or cfg["wall"].get("side", "front"))
    k = key(cfg, dist, side)
    tables[k] = {"dist": float(dist), "side": side, "table": {int(c): {float(u): bool(v) for u, v in d.items()}
                                                               for c, d in table.items()}}
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({"version": 2, "tables": tables}))
    return k
