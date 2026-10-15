"""mauer.laid (as-laid guide pieces, 1 mm gap at every dovetail joint, Samuel 2026-10-15): the cuts agree with
tools/make_guides.py, the boards and stones move with their pieces, and [laid] is no planning input."""
import copy
import re
import sys

import numpy as np
import pytest

from mauer import REPO, config, laid
from mauer.job import planning_bytes

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_guides as mg  # noqa: E402


def _with_gap(cfg, gap=1.0):
    c = copy.deepcopy(cfg)
    c["laid"] = {"piece_gap_mm": gap}
    return c


def _cuts_from_piece_names(cfg) -> dict:
    """{leg: [u of the joint where a piece starts]} from make_guides' piece names ("B2-B4": starts at block 2 of B)."""
    P = float(cfg["brick"]["length"]) + float(cfg["brick"].get("head_joint", 0.0))
    off = float(cfg["guides"]["cut_offset"])
    first_leg = str(cfg["wall"]["legs"][0]["name"])
    out = {str(lg["name"]): [] for lg in cfg["wall"]["legs"]}
    for pc in mg.build(cfg, with_job=False)["leg_pieces"]:
        m = re.match(r"([A-Z]+)(\d+)-", pc.name)
        leg, a = m.group(1), int(m.group(2))
        if not (leg == first_leg and a == 0):
            out[leg].append(a * P + off)
    return {k: sorted(v) for k, v in out.items()}


@pytest.mark.parametrize("variant", [None, "c_acb", "c_a55"])
def test_cuts_are_the_joints_of_make_guides(variant):
    cfg = config.load(variant=variant)
    got, want = laid.leg_cuts(cfg), _cuts_from_piece_names(cfg)
    assert got.keys() == want.keys()
    for leg in want:
        assert got[leg] == pytest.approx(want[leg]), leg


def test_configured_c_has_six_joints():
    cfg = config.load()
    cuts = laid.leg_cuts(cfg)
    assert {k: len(v) for k, v in cuts.items()} == {"A": 1, "B": 3, "C": 2}


def _boards(cfg):
    return {str(t["name"]): t for t in cfg["targets"] if t.get("parent") == "wall"}


def test_boards_follow_their_pieces():
    cfg = _with_gap(config.load())
    ls = laid.LaidShifts(cfg)
    b = _boards(cfg)
    eA, eB, eC = ls.e("A"), ls.e("B"), ls.e("C")
    assert ls.board(b["W0"]) == pytest.approx([0.0, 0.0])                   # A0-A2: the fixed piece
    assert ls.board(b["W1"]) == pytest.approx(eA)                           # A3-B1
    assert ls.board(b["W2"]) == pytest.approx(eA)                           # A3-B1 (B's first blocks)
    assert ls.board(b["W5"]) == pytest.approx(eA + 3 * eB)                  # B7-C1
    assert ls.board(b["W6"]) == pytest.approx(eA + 3 * eB + eC)             # C2-C4
    assert ls.board(b["W7"]) == pytest.approx(eA + 3 * eB + 2 * eC)         # C5-C6
    # what the camera saw 2026-10-15: W5 3 mm further from W2 along B
    assert ls.board(b["W5"]) - ls.board(b["W2"]) == pytest.approx(3.0 * eB)


def test_no_gap_no_shift():
    cfg = config.load()
    c0 = copy.deepcopy(cfg)
    c0.pop("laid", None)
    ls = laid.LaidShifts(c0)
    assert all(np.allclose(ls.board(t), 0.0) for t in _boards(c0).values())


class _St:
    def __init__(self, leg, course, u, length=0.0, index=0):
        self.leg, self.course, self.u, self.length, self.index = leg, course, u, length, index

    @property
    def key(self):
        return (self.leg, self.course, self.index)


def test_upper_stone_over_a_widened_joint_goes_halfway():
    cfg = _with_gap(config.load())
    ls = laid.LaidShifts(cfg)
    P = 201.0
    # course 0 of B: block 1 (A3-B1) and block 2 (B2-B4, behind the joint at 2 P + 20)
    s0 = _St("B", 0, 1 * P + 100.0, index=1)
    s1 = _St("B", 0, 2 * P + 100.0, index=2)
    up = _St("B", 1, 2 * P + 100.0 - P / 2.0, index=1)                      # over the joint between them
    sh = ls.stones([s0, s1, up])
    assert sh[s1.key] - sh[s0.key] == pytest.approx(ls.e("B"))
    assert sh[up.key] == pytest.approx((sh[s0.key] + sh[s1.key]) / 2.0)


def test_laid_table_is_no_planning_input():
    base = b"[brick]\nlength = 200.0\n"
    assert planning_bytes(base + b"[laid]\npiece_gap_mm = 1.0\n") == planning_bytes(base)
    assert planning_bytes(base + b"[robodk_accept]\ncode_commit = \"abc\"\n") == planning_bytes(base)
