"""The UR CB3 control box on ARES (Samuel 2026-10-08): at the end opposite the UR, ~250 mm beyond the chassis and
~200 mm over the deck. It is part of the arm's collision boxes (motion guard, look checks) and of the ARES footprint
(routes, rotations, floor checks)."""
from __future__ import annotations

import math

import pytest

from mauer import armcheck, config
from mauer import floor
from mauer.reference import Pose2D


@pytest.fixture(scope="module")
def cfg():
    return config.load()


def test_the_controller_box_on_ares(cfg):
    lo, hi = config.ares_controller_box(cfg)
    a = cfg["ares"]
    end = -float(a["length"]) / 2.0                                    # the end opposite the UR (this repo's -x)
    assert lo[0] == pytest.approx(end - a["controller_out_mm"]) == pytest.approx(-810.0)
    assert hi[0] == pytest.approx(lo[0] + a["controller_size_mm"][0])            # 268 mm deep (UR5 manual)
    assert hi[2] == pytest.approx(a["deck_top_z"] + a["controller_top_mm"]) == pytest.approx(533.6)
    assert hi[2] - lo[2] == pytest.approx(a["controller_size_mm"][2])            # 418 mm high
    assert lo[1] <= -a["width"] / 2.0 and hi[1] >= a["width"] / 2.0               # lateral position unknown: full width
    boxes = {b.name: b for b in armcheck.ares_boxes(cfg)}
    box = boxes["UR controller"]
    assert box.centre[0] - box.half[0] == pytest.approx(lo[0]) and box.centre[2] + box.half[2] == pytest.approx(hi[2])


def test_the_footprint_and_the_rotation_circle_include_the_controller(cfg):
    sh = floor.AresShape.from_config(cfg)
    xs = [p[0] for p in sh.footprint(Pose2D(0.0, 0.0, 0.0))]
    assert min(xs) == pytest.approx(-810.0) and max(xs) == pytest.approx(560.0)
    assert sh.radius == pytest.approx(math.hypot(810.0, 300.0))
    turned = [p[0] for p in sh.footprint(Pose2D(0.0, 0.0, math.pi))]
    assert max(turned) == pytest.approx(810.0)                         # the box turns with ARES
    bare = floor.AresShape(1120.0, 600.0)
    assert bare.radius == pytest.approx(math.hypot(560.0, 300.0))      # without a controller: as before


def test_the_reach_key_follows_the_ares_layout(cfg):
    """RoboDK's reach study checks the arm against the ARES object - turned for the UR at the rear, with the control
    box (2026-10-08): both enter the cache key, a stale table cannot be reused after they change."""
    import copy

    from mauer import reach_cache
    k = reach_cache.key(cfg, 840.0)
    c = copy.deepcopy(cfg)
    c["ares"]["controller_out_mm"] = 300.0
    assert reach_cache.key(c, 840.0) != k
    c = copy.deepcopy(cfg)
    c["ares"]["frame_x_points_to"] = "front"
    assert reach_cache.key(c, 840.0) != k
    c = copy.deepcopy(cfg)
    c["ares"]["step_file"] = "other.stp"                  # not part of the layout key (the mesh name only)
    assert reach_cache.key(c, 840.0) == k
