"""tools/plan_layout.py for walls of any number of legs (the C of 2026-10-06 and the L of 2026-10-05): candidate
parsing and the leg frames of with_legs (butt corners), no planning (the evaluation runs in --evaluate)."""
import copy
import importlib.util
import math

import pytest

from conftest import l_config
from mauer import REPO, config


def _tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


pl = _tool("plan_layout")


def test_parse_legs():
    assert pl.parse_legs("10,7,5") == [(10, 7, 5)]
    assert pl.parse_legs("9-10, 5-6") == [(9, 5), (9, 6), (10, 5), (10, 6)]
    assert pl.legs_label({"n0": [10, 7, 5]}) == "A 10, B 7, C 5"


def test_with_legs_c_is_two_butt_corners():
    """Corners away from ARES's side (ARES outside the C) - the configured C has ARES inside (next test)."""
    cfg = copy.deepcopy(config.load())
    cfg["wall"]["ares_inside"] = False
    c = pl.with_legs(cfg, (10, 7, 5))
    legs = c["wall"]["legs"]
    assert [lg["name"] for lg in legs] == ["A", "B", "C"]
    assert [lg["rpy_in_wall_deg"][2] for lg in legs] == pytest.approx([0.0, -90.0, 180.0])   # C runs back along A
    W, rib, gap = cfg["brick"]["width"], cfg["brick"]["rib_mm"], cfg["wall"]["corner_gap_mm"]
    off = W / 2 + rib + gap                               # a leg starts at the inside face of the one before
    assert legs[1]["xyz_in_wall"][:2] == pytest.approx([10 * 200 - W / 2, -off])
    assert legs[2]["xyz_in_wall"][:2] == pytest.approx([10 * 200 - W / 2 - off, -off - 7 * 200 + W / 2])
    # over the outer faces (bodies): from A's ARES-side face (y = W/2) to C's (B runs through, C's outer face is
    # flush with B's end): 7 x 200 + W/2 + rib + gap + W/2 -> 1.52 m ("1.5 across")
    across = W / 2 - (legs[1]["xyz_in_wall"][1] - 7 * 200)
    assert across == pytest.approx(1522.69, abs=0.01)
    wp = pl.mj.load_wallplan()
    lg = wp.legs(c)
    assert wp.check_legs(c, lg, wp.layout_legs(c, lg)) == []     # no stone of one leg in another (over the ribs)


def test_with_legs_reproduces_the_l_and_shape_names():
    lc = l_config()
    c = pl.with_legs(lc, (12, 6))
    for a, b in zip(c["wall"]["legs"], lc["wall"]["legs"]):
        assert a["name"] == b["name"] and a["n0"] == b["n0"]
        assert a["xyz_in_wall"] == pytest.approx(b["xyz_in_wall"], abs=0.005)
        assert math.isclose(a["rpy_in_wall_deg"][2], b["rpy_in_wall_deg"][2])
    assert pl.shape_name(lc) == "L wall"


def test_with_legs_inside_c_turns_towards_ares_and_keeps_sides():
    cfg = config.load()
    assert cfg["wall"]["ares_inside"] is True
    c = pl.with_legs(cfg, (10, 7, 5))
    legs = c["wall"]["legs"]
    assert [lg["rpy_in_wall_deg"][2] for lg in legs] == pytest.approx([0.0, 90.0, 180.0])     # C back above A
    assert legs[1]["xyz_in_wall"][1] > 0 and legs[2]["xyz_in_wall"][1] > legs[1]["xyz_in_wall"][1]
    assert [(lg["side"], lg["dist"]) for lg in legs] == [("right", 580.0), ("front", 840.0), ("left", 580.0)]
    for a, b in zip(legs, cfg["wall"]["legs"]):                              # the config's C is exactly this
        assert a["xyz_in_wall"] == pytest.approx(b["xyz_in_wall"], abs=0.005)
