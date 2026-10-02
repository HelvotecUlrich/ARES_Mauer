"""Wall layout and placement sequence (pure Python, no RoboDK) for interlocking stones in running bond.

Rule: a stone can only be placed when every stone of the course below that lies under it is already placed (the
pins make it impossible to slide a stone in under another one later). With full stones only, both wall ends become
slopes: course k starts and ends half a stone (k x bond offset) later/earlier than course 0 - a trapezoid.

Rule 2: a stone is never placed "in the open": a course-0 stone must sit next to an already placed course-0 stone
(only the very first stone of the wall is free); stones of higher courses sit on their complete supports.

Sequencing: ARES stops at wall positions a_j (wall coordinate of the ARES centre). At each stop the wall is continued
where it ends: of all reachable stones that satisfy both rules, the one closest to the start of the wall is placed
next (lower course first on a tie). This grows the wall as a staircase - course 0 one stone ahead, the courses above
following on top. Then ARES moves on by a whole number of stone lengths, as far as possible while every unfinished
stone stays reachable; the next stop first completes the slope left behind, then continues.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class Stone:
    course: int
    index: int
    u: float          # centre along the wall [mm] (wall frame x, origin = start of course 0)
    z_top: float      # top of the stone [mm]

    @property
    def key(self) -> tuple:
        return (self.course, self.index)


def layout(cfg: dict, n0: int) -> list:
    """Trapezoid wall: course 0 has n0 stones, every course above one stone less (bond offset 0.5)."""
    b, w = cfg["brick"], cfg["wall"]
    pitch = b["length"] + b["head_joint"]
    shift = w["bond_offset"] * pitch
    stones = []
    for k in range(w["courses"]):
        z_top = w["base_z"] + k * (b["height"] + b["bed_joint"]) + b["height"]
        n_k = int((n0 * pitch - 2 * k * shift) / pitch + 1e-9)      # both ends slope back by `shift` per course
        for i in range(n_k):
            stones.append(Stone(k, i, k * shift + b["length"] / 2 + i * pitch, z_top))
    return stones


def supports(stone: Stone, by_course: dict, length: float) -> list:
    """Stones of the course below that overlap the footprint of `stone`."""
    if stone.course == 0:
        return []
    return [s for s in by_course.get(stone.course - 1, []) if abs(s.u - stone.u) < length - 1e-6]


def sequence(cfg: dict, stones: list, reach: Callable[[int, float], bool], reach_lo: dict,
             a0: float, max_stops: int = 50) -> list:
    """Greedy plan: [(a_j, [Stone, ...]), ...]. `reach(course, u_rel)`, `reach_lo[course]` = smallest reachable
    u_rel of that course; a0 = first ARES position."""
    b = cfg["brick"]
    L = b["length"]
    pitch = b["length"] + b["head_joint"]
    by_course: dict = {}
    for s in stones:
        by_course.setdefault(s.course, []).append(s)
    placed: set = set()
    plan, a = [], a0
    for _ in range(max_stops):
        batch = []
        while True:
            cand = [s for s in stones if s.key not in placed and reach(s.course, s.u - a)
                    and all(p.key in placed for p in supports(s, by_course, L))
                    and (s.course > 0 or not placed or any(abs(t.u - s.u - pitch) < 1e-6 or abs(s.u - t.u - pitch) < 1e-6
                                                           for t in by_course[0] if t.key in placed))]
            if not cand:
                break
            nxt = min(cand, key=lambda s: (s.u, s.course))          # continue where the wall ends
            placed.add(nxt.key)
            batch.append(nxt)
        plan.append((a, batch))
        open_ = [s for s in stones if s.key not in placed]
        if not open_:
            return plan
        # advance by m stone pitches: every unfinished stone must stay at or ahead of the trailing reach edge
        m = 0
        while all(s.u - (a + (m + 1) * pitch) >= reach_lo[s.course] - 1e-6 for s in open_):
            m += 1
        if m == 0:
            raise RuntimeError(f"stuck at a = {a:.0f}: {len(open_)} stones left that cannot be reached/supported")
        a += m * pitch
    raise RuntimeError("too many stops")


def check_plan(cfg: dict, stones: list, plan: list) -> list:
    """Independent check of a plan: every stone once, supports before the stone, never a stone under a placed one."""
    L = cfg["brick"]["length"]
    by_course: dict = {}
    for s in stones:
        by_course.setdefault(s.course, []).append(s)
    errors, placed = [], set()
    order = [s for _, batch in plan for s in batch]
    if sorted(s.key for s in order) != sorted(s.key for s in stones):
        errors.append("not every stone placed exactly once")
    for s in order:
        for p in supports(s, by_course, L):
            if p.key not in placed:
                errors.append(f"stone {s.key} placed before its support {p.key}")
        above = [t for t in by_course.get(s.course + 1, []) if abs(t.u - s.u) < L - 1e-6]
        for t in above:
            if t.key in placed:
                errors.append(f"stone {s.key} would have to go under the already placed {t.key}")
        if s.course == 0 and placed and not any(abs(abs(t.u - s.u) - (L + cfg["brick"]["head_joint"])) < 1e-6
                                                for t in by_course[0] if t.key in placed):
            errors.append(f"stone {s.key} placed in the open (no neighbour in course 0)")
        placed.add(s.key)
    return errors
