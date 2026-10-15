"""As-laid floor guides (Samuel 2026-10-15): the wall's guide pieces (tools/make_guides.py leg_pieces) lie on the floor
with [laid] piece_gap_mm at every dovetail joint, so that the stones fit. The first piece (with the map origin O)
stays put and every joint pushes the pieces after it that much further along its leg, in wall order ([[wall.legs]]
A -> B -> C; a corner L-piece carries the next leg). Measured 2026-10-15: W2 -> W5 1209.0 mm instead of 1206 (three
joints between them; run 2026-10-15_155402_hmi_real stopped with a frame fit RMS of 1.50 mm).

A runtime correction like the measured ARES pose, not a planning input: the RoboDK-verified job keeps the design
positions and the sequencer moves the wall boards (frame fit) and the wall place poses by these shifts. planning_bytes
(mauer.job) skips [laid], so a changed gap needs neither a job rebuild nor a RoboDK run. Shifts are wall-frame
(dx, dy) [mm]:
- a board: the shift of the piece carrying its plate's V-tab;
- a first-course stone: the shift of the piece under its centre (its cones are on one piece, make_guides._stones);
- a stone of a higher course: the mean shift of the stones below it whose pins it takes (overlapping footprints, as
  robodk/wallplan.supports), so a stone over a widened joint goes halfway.
The cuts are those of make_guides (cut_offset after a block joint, pieces of at most segment_max, corner_arm_stones
per corner arm); tests/test_laid.py checks that both agree.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from . import config as _config

EPS = 1e-6


def gap_mm(cfg: Mapping) -> float:
    """[laid] piece_gap_mm (0 without the table: the pieces butt as designed)."""
    return float((cfg.get("laid") or {}).get("piece_gap_mm", 0.0) or 0.0)


def _split(n_items: int, kmax: int) -> list[int]:
    """Sizes of the groups make_guides._split cuts n_items consecutive blocks into (at most kmax each, balanced)."""
    if n_items <= 0:
        return []
    n = math.ceil(n_items / kmax)
    base, extra = divmod(n_items, n)
    return [base + (1 if k < extra else 0) for k in range(n)]


def _legs(cfg: Mapping) -> list[Mapping]:
    return list((cfg.get("wall", {}) or {}).get("legs", []) or [])


def _pitch(cfg: Mapping) -> float:
    b = cfg["brick"]
    return float(b["length"]) + float(b.get("head_joint", 0.0))


def _board_u(cfg: Mapping, t: Mapping) -> tuple[int, float]:
    """(block k, block centre u) of a wall board on a plate site: the board origin (top-left corner) lies
    block_length / 2 - board width / 2 before the block centre; k as mauer.floor.site_from_target."""
    bl = float(cfg["plates"]["block_length"])
    P = _pitch(cfg)
    x = float(t["xyz"][0])
    k = int(math.floor((x + 1e-6) / P))         # origin u_c - board_w / 2 lies inside block k (u_c - 100 < x < u_c)
    return k, k * P + bl / 2.0


def leg_cuts(cfg: Mapping) -> dict[str, list[float]]:
    """{leg: [u of every dovetail joint along that leg]} (leg frame, mm), wall order - the cuts of
    make_guides.leg_pieces: groups of at most kmax blocks between the corner arms, a joint cut_offset after the block
    joint where a piece starts (not at the first leg's start), one before each corner L-piece."""
    gd, b = cfg["guides"], cfg["brick"]
    P, off = _pitch(cfg), float(gd["cut_offset"])
    S = float(b["length"])
    half_l = float((cfg.get("half_brick") or {}).get("length", S / 2.0))
    kmax = int((float(gd["segment_max"]) - float(gd["dovetail"][1]) - off) // P)
    arm = int(gd["corner_arm_stones"])
    legs = _legs(cfg)
    nl = len(legs)
    out: dict[str, list[float]] = {}
    for i, lg in enumerate(legs):
        name, n0 = str(lg["name"]), float(lg["n0"])
        nf = int(math.floor(n0 + 1e-9))
        half = n0 - nf > 1e-9
        n = nf + (1 if half else 0)
        end = nf * P + half_l if half else (nf - 1) * P + S
        spare = [_board_u(cfg, t)[0] for t in cfg.get("targets", [])
                 if t.get("parent") == "wall" and str(t.get("leg")) == name and _board_u(cfg, t)[0] >= nf]
        if spare:
            n += max(0, max(spare) + 1 - int(math.ceil(end / P - 1e-9)))
        first = arm if i > 0 else 0
        last = arm if i < nl - 1 else 0
        cuts, a = [], first
        for m in _split(n - last - first, kmax):
            if not (i == 0 and a == 0):
                cuts.append(a * P + off)
            a += m
        if i < nl - 1:
            cuts.append((n - arm) * P + off)
        out[name] = sorted(set(cuts))
    return out


class LaidShifts:
    """Wall-frame shifts of the as-laid guide pieces (module docstring) for one config."""

    def __init__(self, cfg: Mapping):
        self.cfg = cfg
        self.gap = gap_mm(cfg)
        self.cuts = leg_cuts(cfg) if self.gap and _legs(cfg) else {}
        frames = _config.leg_frames(dict(cfg))
        self.T_leg = frames
        self.start: dict[str, np.ndarray] = {}
        s = np.zeros(2)
        for lg in _legs(cfg):
            name = str(lg["name"])
            self.start[name] = s.copy()
            s = s + self.gap * len(self.cuts.get(name, [])) * self.e(name)

    def e(self, leg: str) -> np.ndarray:
        """Unit vector along the leg (its frame x) in the wall frame."""
        return np.asarray(self.T_leg[leg][:2, 0], float)

    def at(self, leg: str | None, u: float) -> np.ndarray:
        """Shift (dx, dy) [mm] of the piece under leg-frame u of `leg` (zero without legs or gap)."""
        if not self.gap or leg is None or leg not in self.start:
            return np.zeros(2)
        n = sum(1 for c in self.cuts.get(leg, []) if c < u - EPS)
        return self.start[leg] + self.gap * n * self.e(leg)

    def board(self, t: Mapping) -> np.ndarray:
        """Shift of a [[targets]] wall board with `leg` (its plate's V-tab, make_guides.tabs_of; zero otherwise)."""
        leg = t.get("leg")
        if not self.gap or leg is None or t.get("parent") != "wall":
            return np.zeros(2)
        leg = str(leg)
        bl = float(self.cfg["plates"]["block_length"])
        _, u_c = _board_u(self.cfg, t)
        L = self._strip_end(leg)
        tab = u_c + bl / 2.0
        if not tab < L - float(self.cfg["guides"]["vtab"][0]):
            tab = u_c - bl / 2.0
        return self.at(leg, tab)

    def _strip_end(self, leg: str) -> float:
        """u of the end of the leg's strip (its last block, spare blocks included)."""
        P = _pitch(self.cfg)
        S = float(self.cfg["brick"]["length"])
        lg = next(d for d in _legs(self.cfg) if str(d["name"]) == leg)
        n0 = float(lg["n0"])
        nf = int(math.floor(n0 + 1e-9))
        half_l = float((self.cfg.get("half_brick") or {}).get("length", S / 2.0))
        end = nf * P + half_l if n0 - nf > 1e-9 else (nf - 1) * P + S
        ks = [_board_u(self.cfg, t)[0] for t in self.cfg.get("targets", [])
              if t.get("parent") == "wall" and str(t.get("leg")) == leg]
        k_last = max(ks) if ks else -1
        return max(end, k_last * P + S)

    def stones(self, stones: Sequence) -> dict:
        """{stone key: shift (dx, dy)} for stones with .leg, .course, .u (leg frame centre), .key and .length
        (0 = [brick] length): course 0 from the piece under the centre, higher courses the mean of their supports."""
        S = float(self.cfg["brick"]["length"])
        by_course: dict[int, list] = {}
        for s in stones:
            by_course.setdefault(int(s.course), []).append(s)
        out: dict = {}
        for c in sorted(by_course):
            for s in by_course[c]:
                if not self.gap or not s.leg:
                    out[s.key] = np.zeros(2)
                    continue
                sup = [b for b in by_course.get(c - 1, []) if b.leg == s.leg
                       and abs(b.u - s.u) < ((b.length or S) + (s.length or S)) / 2.0 - EPS] if c > 0 else []
                out[s.key] = (np.mean([out[b.key] for b in sup], axis=0) if sup else self.at(s.leg, s.u))
        return out


class _TaskView:
    """A job StoneTask as LaidShifts.stones sees it (leg-frame centre u, length)."""

    def __init__(self, t):
        self.leg, self.course, self.u, self.key = t.leg, int(t.course), float(t.u_mm), t.key
        self.length = float(t.length_mm or 0.0)


def shift_job(job, cfg: Mapping):
    """A copy of `job` whose wall stones' T_wall_tcp are moved by their as-laid shift (the job itself stays as
    verified); `job` unchanged without a gap. The shift is a pure wall-frame translation (dx, dy)."""
    import copy

    ls = LaidShifts(cfg)
    if not ls.gap or not ls.start:
        return job
    out = copy.deepcopy(job)
    tasks = out.stones()
    sh = ls.stones([_TaskView(t) for t in tasks])
    for t in tasks:
        d = sh.get(t.key)
        if d is None or not np.any(d):
            continue
        T = np.array(t.T_wall_tcp, float)
        T[0, 3] += float(d[0])
        T[1, 3] += float(d[1])
        t.T_wall_tcp = T
    return out


def summary(cfg: Mapping) -> dict:
    """{"piece_gap_mm", "cuts" {leg: [u]}, "boards" {name: [dx, dy]}} for the run log."""
    ls = LaidShifts(cfg)
    return {"piece_gap_mm": ls.gap, "cuts": ls.cuts,
            "boards": {str(t["name"]): [round(float(v), 3) for v in ls.board(t)]
                       for t in cfg.get("targets", []) if t.get("parent") == "wall" and "leg" in t}}
