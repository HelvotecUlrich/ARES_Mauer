"""Half stones are gripped [half_brick] grasp_above_top_mm higher (Samuel 2026-10-09: the pins of the half stone stand
in the middle, under the gripper - "nicht allzuweit runter", 20 mm): the TCP poses of the magazine, the station and the
wall, and every model that rebuilds the held / standing stone from a TCP pose (collision boxes, motion guard, payload
CoG, magazine-test approach, RoboDK stone frame)."""
import importlib.util
import math
import sys

import numpy as np
import pytest

from mauer import REPO, armcheck, config
from mauer import geometry as g
from mauer.backends import stone_payload
from mauer.motionguard import MotionGuard

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def job(cfg):
    return make_job.build_nominal(cfg)


def test_grasp_offset_from_the_config(cfg):
    assert config.grasp_above_top_mm(cfg, "full") == 0.0
    assert config.grasp_above_top_mm(cfg, "") == 0.0
    assert config.grasp_above_top_mm(cfg, "half") == pytest.approx(float(cfg["half_brick"]["grasp_above_top_mm"]))
    assert cfg["half_brick"]["grasp_above_top_mm"] == 20.0


def test_half_stone_tcp_poses_sit_above_the_stone_top(cfg, job):
    off = config.grasp_above_top_mm(cfg, "half")
    kinds = {t.kind for t in job.stones()}
    assert kinds == {"full", "half"}
    for t in job.stones():                                         # wall: z_top_mm stays the stone top
        assert t.T_wall_tcp[2, 3] == pytest.approx(t.z_top_mm + (off if t.kind == "half" else 0.0)), t.label
    ps = cfg["pickup_station"]
    z0 = float(ps["table_z"]) + float(ps["holder_z"])
    for s in job.station.slots:
        top = config.stack_top_z(cfg, z0, s.layer, s.kind)
        assert s.T_station_tcp[2, 3] == pytest.approx(top + (off if s.kind == "half" else 0.0)), s.id
    deck = float(cfg["ares"]["deck_top_z"]) + float(cfg["deck"]["holder_z"])
    halves = [s for s in job.magazine.slots if s.kind == "half"]
    assert halves
    for s in job.magazine.slots:
        top = config.stack_top_z(cfg, deck, s.layer, s.kind or "full")
        assert s.T_ares_tcp[2, 3] == pytest.approx(top + (off if s.kind == "half" else 0.0)), s.id


def test_stone_box_and_held_stone_hang_below_the_raised_tcp(cfg):
    off = config.grasp_above_top_mm(cfg, "half")
    up = float(cfg["brick"]["pin_length"])
    for kind, H, o in (("full", float(cfg["brick"]["height"]), 0.0),
                       ("half", float(cfg["half_brick"]["height"]), off)):
        box = armcheck._stone_box(cfg, "s", np.eye(4), kind)                # TCP frame: z into the stone
        assert box.centre[2] - box.half[2] == pytest.approx(o - up)          # pin tips
        assert box.centre[2] + box.half[2] == pytest.approx(o + H)           # bottom face
        mg = MotionGuard(cfg, np.eye(4), [0.0] * 6, config.T_flange_tcp(cfg))
        T_tcp_flange = g.inv(config.T_flange_tcp(cfg))
        z = [g.apply(T_tcp_flange, [p])[0][2] for _, a, b, _ in mg._held_capsules(kind) for p in (a, b)]
        assert min(z) == pytest.approx(o) and max(z) == pytest.approx(o + H)


def test_payload_cog_of_a_half_stone_is_lower(cfg):
    T = config.T_flange_tcp(cfg)
    H = float(cfg["half_brick"]["height"])
    off = config.grasp_above_top_mm(cfg, "half")
    m, cog = stone_payload(0.0, [0.0, 0.0, 0.0], 1.5, T, H, grasp_mm=off)
    assert m == pytest.approx(1.5)
    assert cog[2] == pytest.approx(float(cfg["tool"]["tcp_z"]) + off + H / 2.0)


def test_robodk_stone_frame_of_a_half_stone(cfg):
    """robodk/rdk_common.T_tc_cad: CAD frame of the stone in the TCP-related top-centre frame (z up) - the half stone's
    CAD origin (bottom) lies grasp_above_top_mm deeper (pure matrix, no RoboDK needed)."""
    spec = importlib.util.spec_from_file_location("rdk_common_under_test", REPO / "robodk" / "rdk_common.py")
    try:
        rc = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = rc
        spec.loader.exec_module(rc)
    except ImportError as e:                                      # the RoboDK API is only on Windows
        pytest.skip(f"robodk API not importable: {e}")
    off = config.grasp_above_top_mm(cfg, "half")
    full, half = rc.T_tc_cad(cfg, "full"), rc.T_tc_cad(cfg, "half")
    assert full[2, 3] == pytest.approx(-float(cfg["brick"]["height"]))
    assert half[2, 3] == pytest.approx(-float(cfg["half_brick"]["height"]) - off)


def test_magazine_approach_counts_the_lower_hanging_half_stone(cfg, job):
    from mauer import magtest
    full = next(s for s in job.magazine.slots if s.kind != "half")
    others = [s.id for s in job.magazine.slots if s.stack != full.stack and s.layer == 1]
    kinds = {k: job.magazine.slot(k).kind or "full" for k in others}
    z = float(full.T_ares_tcp[2, 3])
    a_full = magtest.magazine_approach_mm(cfg, job, others, kinds, full.id, "full", z, 1.0)
    a_half = magtest.magazine_approach_mm(cfg, job, others, kinds, full.id, "half", z, 1.0)
    H, Hh = float(cfg["brick"]["height"]), float(cfg["half_brick"]["height"])
    assert a_half - a_full == pytest.approx(config.grasp_above_top_mm(cfg, "half") + Hh - H)


def test_transfer_heights_carry_the_lower_hanging_half_stone(cfg):
    mg = MotionGuard(cfg, config.T_ares_base(cfg), [0.0] * 6, config.T_flange_tcp(cfg))
    b, dk, a = cfg["brick"], cfg["deck"], cfg["ares"]
    layers = int(dk.get("magazine_layers", dk.get("layers", 1)))
    top = (float(a["deck_top_z"]) + float(dk["holder_z"]) + layers * float(b["height"])
           + (layers - 1) * float(b.get("bed_joint", 0.0)))
    held = max(float(b["height"]), float(cfg["half_brick"]["height"]) + config.grasp_above_top_mm(cfg, "half"))
    assert mg.z_safe_base == pytest.approx(top + held + float(b["pin_length"]) + 60.0
                                           - float(config.T_ares_base(cfg)[2, 3]))
    assert math.isfinite(mg.z_safe_base)
