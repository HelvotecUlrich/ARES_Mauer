"""Cache of the RoboDK reach tables (results/reach_table.json), shared by robodk/simulate.py (computes them),
robodk/look_study.py and tools/make_job.py (read them).

A reach table is {course: {u_rel: ok}}: can the UR5 place a full stone of that course at wall position u_rel
(relative to the ARES centre, 20 mm grid) from a stop at wall distance `dist` (robodk/simulate.py reach_table). Its
cache key covers everything the table depends on: the UR5 mount, the tool, the stone (without [brick] rib_mm - the ribs
only place the corner of a wall of legs, they are part of the stone mesh the check uses), [wall] base_z / courses,
[study] approach and the wall distance.

File layout (version 2, 2026-10-06 - one table per key, so several wall distances live side by side):
    {"version": 2, "tables": {key: {"dist": mm, "table": {course: {u_rel: ok}}}}}
Version 1 (one table: {"key", "dist", "table"}) is still read.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Mapping

from . import REPO

PATH = REPO / "results" / "reach_table.json"


def key(cfg: Mapping, dist: float) -> str:
    """Cache key of the reach table for cfg at wall distance dist [mm]."""
    brick = {k: v for k, v in cfg["brick"].items() if k != "rib_mm"}
    src = json.dumps(["family-v2", cfg["ur5"], cfg["tool"], brick, cfg["wall"]["base_z"], cfg["wall"]["courses"],
                      cfg["study"]["approach"], dist], sort_keys=True)
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
    return {str(k): {"dist": None if v.get("dist") is None else float(v["dist"]), "table": _parse(v["table"])}
            for k, v in items}


def get(cfg: Mapping, dist: float, path: str | Path | None = None) -> dict | None:
    """The cached table for cfg at dist, None if there is none for this key."""
    e = read(path).get(key(cfg, dist))
    return None if e is None else e["table"]


def store(cfg: Mapping, dist: float, table: Mapping, path: str | Path | None = None) -> str:
    """Add (or replace) the table of cfg at dist; the other cached tables are kept. Returns its key."""
    p = Path(path or PATH)
    tables = {k: {"dist": v["dist"], "table": v["table"]} for k, v in read(p).items()}
    k = key(cfg, dist)
    tables[k] = {"dist": float(dist), "table": {int(c): {float(u): bool(v) for u, v in d.items()}
                                                for c, d in table.items()}}
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({"version": 2, "tables": tables}))
    return k
