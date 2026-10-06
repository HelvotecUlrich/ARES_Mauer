"""tools/make_plates.py with the L layout: the plate of a leg board is built in its LEG frame (wall at the top of the
DXF view, same notches and engraving as on leg A), labelled with the leg; print_targets is untouched (board names,
ids and sizes only). robodk/rdk_common.py leg helpers agree with the numpy poses (pure helpers, no RoboDK call)."""
import importlib.util
import math
import sys

import numpy as np
import pytest

from conftest import l_config
from mauer import REPO, config
from mauer import geometry as g
from mauer.reference import placements
from mauer.vision.targets import board_specs


def _tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


make_plates = _tool("make_plates")


@pytest.fixture(scope="module")
def cfg():
    return l_config()                                    # the L: W0 / W5 on block 0 of legs A / B


def test_leg_board_plates(cfg):
    specs = board_specs(cfg)
    walls = {t["name"]: t for t in cfg["targets"] if t["parent"] == "wall"}
    plates = {n: make_plates.wall_plate(cfg, specs[n], config.pose(t), t.get("leg")) for n, t in walls.items()}
    a, b = plates["W0"], plates["W5"]                    # same block index (k = 0) on legs A and B
    assert a.cut_polys == b.cut_polys and a.eng_lines == b.eng_lines   # identical plate in its leg frame
    texts = {n: [t[2] for t in p.texts] for n, p in plates.items()}
    assert "B: u = 100 mm" in texts["W5"] and "A: u = 100 mm" in texts["W0"] and "WALL ^" in texts["W5"]
    for p in plates.values():                            # two notches on the block joints, wall at the top
        top = [pt for pt in p.cut_polys[0] if pt[1] < p.h / 2 - 1e-6 and abs(pt[1] - (p.h / 2 - 4.0)) < 1e-6]
        assert len(top) == 2 and sorted(x for x, _ in top) == pytest.approx([-100.0, 100.0])


def test_plate_rotation_on_the_floor_comes_from_the_leg(cfg):
    pl = {p.name: p for p in placements(cfg)}
    T_legs = config.leg_frames(cfg)
    for t in cfg["targets"]:
        if t["parent"] != "wall":
            continue
        T = pl[t["name"]].T_parent_board
        assert np.allclose(T, T_legs[t["leg"]] @ g.pose_xyz_rpy(t["xyz"], t["rpy_deg"]))
        heading = math.degrees(math.atan2(T[1, 0], T[0, 0]))
        assert heading == pytest.approx(0.0 if t["leg"] == "A" else -90.0, abs=1e-9)
        assert T[2, 2] == pytest.approx(-1.0)          # face up (board z into the board = down)


def test_print_targets_ignores_the_placement():
    src = (REPO / "tools" / "print_targets.py").read_text(encoding="utf-8")
    assert "xyz" not in src and "placements" not in src and '"leg"' not in src


def test_rdk_common_leg_helpers_match_numpy(cfg):
    sys.path.insert(0, r"C:\RoboDK\Python")             # RoboDK API (rdk_common adds it too); no RoboDK started
    pytest.importorskip("robodk")
    sys.path.insert(0, str(REPO / "robodk"))
    try:
        import rdk_common as rc
    except Exception as e:                              # noqa: BLE001 - RoboDK API not installed here
        pytest.skip(f"robodk API not importable: {e}")
    make_job = _tool("make_job")
    T_legs = config.leg_frames(cfg)
    for leg, a in (("A", 1780.0), ("B", 600.0)):
        M = g.from_robodk(rc.place_pose_leg(cfg, 300.0, 740.0, 260.0, leg, a))
        T_ares_wall = g.inv(make_job.stop_pose(cfg, 740.0, a, T_legs[leg]).T)
        T = T_ares_wall @ T_legs[leg] @ g.transl(300.0, 0.0, 260.0) @ g.rotx(math.pi)
        assert np.allclose(M, T, atol=1e-9)
        assert np.allclose(g.from_robodk(rc.leg_frame(cfg, leg)), T_legs[leg], atol=1e-12)
    assert np.allclose(g.from_robodk(rc.wall_frame_at(cfg, 740.0)), g.from_robodk(rc.wall_frame(cfg, 740.0)))
    pts = rc.half_stone_points(cfg, "wall", 50.0, 260.0)
    xs = [p[0] for p in pts]
    assert (min(xs), max(xs)) == pytest.approx((0.0, 100.0))
