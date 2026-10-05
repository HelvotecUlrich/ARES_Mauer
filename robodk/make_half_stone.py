"""Half stone mesh (PLACEHOLDER): the full stone mesh clipped to half length and closed.

    py.exe robodk/make_half_stone.py            # writes cad/stone_half_placeholder.stl

The CAD of the CURRENT half stone is not available ([half_brick] in config/station.toml: Gino's archive half stone is
the old design). Until it is measured, the half stone is modelled from the full stone cad/stone_full_2026-10-01.stl
(CAD frame: x 0..width (ribbed long faces), y -length..0, z 0..height, pins below z = 0, sockets on top): the part
y in [-[half_brick] length, 0] is kept and the cut face at y = -length is closed with a planar cap (ear clipping of
the section polygon, which contains the rib profile). Result, same CAD frame convention: x 0..120 (ribs to -1.69 /
121.69), y -100..0, z -23.5..120, ONE pin pair below and ONE socket pair on top at y = -49.75 (the full stone's end
pair; 0.25 mm off the half stone's centre at y = -50 - inside the PLACEHOLDER "pin pair centred" of [half_brick]
pin_across). Height, width, pins and ribs = full stone.

Checks before writing: the input is a closed mesh, the output is closed and consistently oriented (every edge shared
by exactly two triangles in opposite directions), positive volume, the bounding box as expected. Pure Python + numpy,
no RoboDK. The STL is not versioned (cad/*.stl in .gitignore): rerun this script after a checkout.
"""
from __future__ import annotations

import argparse
import struct
import sys
import tomllib
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "cad" / "stone_half_placeholder.stl"
ROUND = 4                                    # vertex welding: 0.1 µm


# ── STL io ────────────────────────────────────────────────────────────────────
def read_stl(path: Path) -> np.ndarray:
    """(N, 3, 3) triangles from a binary or ASCII STL."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode().splitlines() if ln.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return np.frombuffer(data[84:84 + 50 * n], dtype=rec)["v"].astype(float)


def write_stl(path: Path, tris: np.ndarray, name: str) -> None:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 0, n / np.maximum(ln, 1e-30), 0.0)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = n, tris
    header = name.encode()[:80].ljust(80, b"\0")
    path.write_bytes(header + struct.pack("<I", len(tris)) + rec.tobytes())


# ── mesh topology ─────────────────────────────────────────────────────────────
def weld(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(vertices (M, 3), faces (N, 3) int) with vertices merged after rounding to ROUND decimals."""
    pts = np.round(tris.reshape(-1, 3), ROUND)
    V, inv = np.unique(pts, axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    keep = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 2] != F[:, 0])       # drop degenerate faces
    return V, F[keep]


def edge_problems(F: np.ndarray) -> list[str]:
    """[] if every undirected edge is used by exactly two faces in opposite directions (closed, oriented)."""
    und, dirc = defaultdict(int), defaultdict(int)
    for a, b, c in F:
        for e in ((a, b), (b, c), (c, a)):
            und[tuple(sorted(e))] += 1
            dirc[e] += 1
    out = []
    bad = sum(1 for n in und.values() if n != 2)
    if bad:
        out.append(f"{bad} edges not shared by exactly two faces (open or non-manifold mesh)")
    dup = sum(1 for n in dirc.values() if n > 1)
    if dup:
        out.append(f"{dup} directed edges used twice (inconsistent orientation)")
    return out


def orient(V: np.ndarray, F: np.ndarray) -> np.ndarray:
    """Consistent winding over every connected component (BFS over shared edges), outward (positive volume)."""
    F = F.copy()
    adj = defaultdict(list)
    for i, (a, b, c) in enumerate(F):
        for e in ((a, b), (b, c), (c, a)):
            adj[tuple(sorted(e))].append(i)
    seen = np.zeros(len(F), bool)
    for start in range(len(F)):
        if seen[start]:
            continue
        comp, q = [start], deque([start])
        seen[start] = True
        while q:
            i = q.popleft()
            a, b, c = F[i]
            for u, v in ((a, b), (b, c), (c, a)):
                for j in adj[tuple(sorted((u, v)))]:
                    if j == i or seen[j]:
                        continue
                    x, y, z = F[j]
                    if (u, v) in ((x, y), (y, z), (z, x)):          # same direction -> flip j
                        F[j] = [x, z, y]
                    seen[j] = True
                    comp.append(j)
                    q.append(j)
        if volume(V, F[comp]) < 0:
            F[comp] = F[comp][:, [0, 2, 1]]
    return F


def volume(V: np.ndarray, F: np.ndarray) -> float:
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


# ── clipping ──────────────────────────────────────────────────────────────────
def clip_keep_above(V: np.ndarray, F: np.ndarray, axis: int, c: float) -> tuple[list, list]:
    """Clip an oriented closed mesh to the half space p[axis] >= c. Returns (triangles [(3, 3) arrays], cut segments
    [(p, q)]) where each cut segment runs along the cut in the direction of the face it came from."""
    tris, segs = [], []
    for f in F:
        P = V[f]
        d = P[:, axis] - c
        if np.any(np.abs(d) < 1e-9):
            raise ValueError(f"a vertex lies on the cut plane {axis}={c} - choose another cut")
        inside = d > 0
        n_in = int(inside.sum())
        if n_in == 3:
            tris.append(P)
            continue
        if n_in == 0:
            continue
        # rotate so that the lone vertex (inside for n_in == 1, outside for n_in == 2) comes first, keeping the winding
        lone = int(np.flatnonzero(inside if n_in == 1 else ~inside)[0])
        A, B, C = P[lone], P[(lone + 1) % 3], P[(lone + 2) % 3]
        dA, dB, dC = d[lone], d[(lone + 1) % 3], d[(lone + 2) % 3]
        AB = A + (B - A) * (dA / (dA - dB))
        AC = A + (C - A) * (dA / (dA - dC))
        if n_in == 1:                 # keep A, AB, AC; the face boundary runs A -> AB -> (cut) -> AC -> A
            tris.append(np.array([A, AB, AC]))
            segs.append((AB, AC))
        else:                         # keep the quad AB, B, C, AC; boundary ... C -> AC -> (cut) -> AB -> B
            tris.append(np.array([AB, B, C]))
            tris.append(np.array([AB, C, AC]))
            segs.append((AC, AB))
    return tris, segs


def loops(segs: list) -> list[np.ndarray]:
    """Chain directed cut segments into closed loops (points snapped to ROUND decimals)."""
    key = lambda p: tuple(np.round(p, ROUND))          # noqa: E731
    nxt, pos = {}, {}
    for p, q in segs:
        kp, kq = key(p), key(q)
        if kp == kq:
            continue
        if kp in nxt:
            raise ValueError("cut section is not a set of simple loops (branching)")
        nxt[kp] = kq
        pos[kp], pos[kq] = p, q
    out, used = [], set()
    for start in list(nxt):
        if start in used:
            continue
        loop, k = [], start
        while k not in used:
            used.add(k)
            loop.append(pos[k])
            if k not in nxt:
                raise ValueError("cut section loop is open")
            k = nxt[k]
        if k != start:
            raise ValueError("cut section loop does not close on itself")
        out.append(np.array(loop))
    return out


def ear_clip(poly2: np.ndarray, eps: float = 1e-9) -> list[tuple[int, int, int]]:
    """Triangulate a simple polygon (2D, counter-clockwise) by ear clipping; returns index triples (CCW). Collinear
    vertices (long straight edges of the section with many points) are KEPT - dropping them would leave T-junctions
    against the side faces - they are only never used as an ear tip."""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    idx = list(range(len(poly2)))
    out = []
    while len(idx) > 3:
        n = len(idx)
        for i in range(n):
            ia, ib, ic = idx[(i - 1) % n], idx[i], idx[(i + 1) % n]
            a, b, c = poly2[ia], poly2[ib], poly2[ic]
            if cross(a, b, c) <= eps:                       # reflex or collinear tip: not an ear
                continue
            if any(cross(a, b, poly2[j]) >= -eps and cross(b, c, poly2[j]) >= -eps and cross(c, a, poly2[j]) >= -eps
                   for j in idx if j not in (ia, ib, ic)):
                continue
            out.append((ia, ib, ic))
            idx.pop(i)
            break
        else:
            raise ValueError("ear clipping failed (polygon not simple?)")
    out.append(tuple(idx))
    return out


def cap(loops_: list, axis: int, c: float, outward: np.ndarray) -> list:
    """Planar cap triangles for the cut loops at p[axis] = c, facing `outward`. Only simple loops without holes."""
    other = [i for i in range(3) if i != axis]
    tris = []
    for L in loops_:
        P2 = L[:, other]
        area = 0.5 * float(np.sum(P2[:, 0] * np.roll(P2[:, 1], -1) - np.roll(P2[:, 0], -1) * P2[:, 1]))
        order = np.arange(len(L)) if area > 0 else np.arange(len(L))[::-1]
        for ia, ib, ic in ear_clip(P2[order]):
            T = np.array([L[order[ia]], L[order[ib]], L[order[ic]]])
            T[:, axis] = c
            n = np.cross(T[1] - T[0], T[2] - T[0])
            if float(n @ outward) < 0:
                T = T[[0, 2, 1]]
            tris.append(T)
    return tris


# ── main ──────────────────────────────────────────────────────────────────────
def make_half(full: np.ndarray, length: float) -> tuple[np.ndarray, dict]:
    """Half stone triangles (CAD frame of the full stone, y -length..0) and a report."""
    V, F = weld(full)
    probs = edge_problems(F)
    rep = {"input_faces": len(F), "input_problems": probs}
    if any("not shared" in p for p in probs):
        raise ValueError(f"input mesh is not closed: {probs}")
    F = orient(V, F)
    rep["input_volume_cm3"] = volume(V, F) / 1000.0
    kept, segs = clip_keep_above(V, F, axis=1, c=-length)
    lp = loops(segs)
    caps = cap(lp, axis=1, c=-length, outward=np.array([0.0, -1.0, 0.0]))
    tris = np.array(kept + caps, float)
    V2, F2 = weld(tris)
    rep.update(n_loops=len(lp), cap_faces=len(caps), output_faces=len(F2), output_problems=edge_problems(F2),
               output_volume_cm3=volume(V2, F2) / 1000.0,
               bbox_min=V2.min(0).round(3).tolist(), bbox_max=V2.max(0).round(3).tolist())
    return V2[F2], rep


def main(argv: list | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args(argv)
    with open(REPO / "config" / "station.toml", "rb") as fh:
        cfg = tomllib.load(fh)
    b, hb = cfg["brick"], cfg.get("half_brick", {})
    length = float(hb.get("length", b["length"] / 2.0))
    for key in ("width", "height", "pin_length"):
        if abs(float(hb.get(key, b[key])) - float(b[key])) > 1e-9:
            raise SystemExit(f"[half_brick] {key} differs from [brick] {key}: the clipped full stone does not fit")
    full = read_stl(REPO / b["mesh"])
    tris, rep = make_half(full, length)
    print(f"input {b['mesh']}: {rep['input_faces']} faces, {rep['input_volume_cm3']:.1f} cm3"
          + (f" (repaired: {'; '.join(rep['input_problems'])})" if rep["input_problems"] else ""))
    print(f"half stone: {rep['output_faces']} faces ({rep['cap_faces']} cap faces, "
          f"{rep['n_loops']} section loop(s)), volume {rep['output_volume_cm3']:.1f} cm3, bbox {rep['bbox_min']} .. "
          f"{rep['bbox_max']}")
    if rep["output_problems"]:
        raise SystemExit(f"output mesh invalid: {rep['output_problems']}")
    if abs(rep["bbox_min"][1] + length) > 1e-3 or abs(rep["bbox_max"][1]) > 1e-3:
        raise SystemExit(f"output length wrong: y {rep['bbox_min'][1]} .. {rep['bbox_max'][1]}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_stl(args.out, tris, f"stone_half_placeholder clipped from {Path(b['mesh']).name} y>={-length:g}")
    print(f"written {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
