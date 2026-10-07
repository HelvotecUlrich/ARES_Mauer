"""Direction / sign mapping (spec 5.4): physical operator direction -> PLC frame, jog bit choice.

Copied 2026-10-07 from MA 10_robot/hmi/amr_hmi/tests/test_frame.py (commit 5935c5b); the shipped config is
config/station.toml [hmi.frame] (hmi.core.config.amr_cfg) instead of config.yaml."""

from __future__ import annotations

import math

import pytest

from hmi.amr import frame as F
from hmi.core.config import amr_cfg
from mauer import config

pytestmark = pytest.mark.usefixtures("no_lab_network")

DOC = F.FrameConfig(plus_y_is_left=True, plus_omega_is_ccw=True)       # PLC documentation
FLIP = F.FrameConfig(plus_y_is_left=False, plus_omega_is_ccw=False)    # both inverted on the robot
S = math.sqrt(0.5)


@pytest.mark.parametrize("direction, doc, flip", [
    (F.FORWARD,    (1, 0),   (1, 0)),
    (F.BACK,       (-1, 0),  (-1, 0)),
    (F.LEFT,       (0, 1),   (0, -1)),
    (F.RIGHT,      (0, -1),  (0, 1)),
    (F.FWD_LEFT,   (S, S),   (S, -S)),
    (F.FWD_RIGHT,  (S, -S),  (S, S)),
    (F.BACK_LEFT,  (-S, S),  (-S, -S)),
    (F.BACK_RIGHT, (-S, -S), (-S, S)),
])
def test_translation_unit_all_directions_both_configs(direction, doc, flip):
    assert F.translation_unit(direction, DOC) == pytest.approx(doc)
    assert F.translation_unit(direction, FLIP) == pytest.approx(flip)


@pytest.mark.parametrize("direction", F.TRANSLATIONS)
def test_translation_vector_magnitude(direction):
    dx, dy = F.translation_vector(direction, 1000.0, DOC)
    assert math.hypot(dx, dy) == pytest.approx(1000.0)
    # negative distance input is treated as magnitude
    assert F.translation_vector(direction, -1000.0, DOC) == pytest.approx((dx, dy))


def test_translation_vector_no_negative_zero():
    dx, dy = F.translation_vector(F.LEFT, 100.0, DOC)
    assert dx == 0.0 and math.copysign(1.0, dx) == 1.0
    assert dy == 100.0


def test_x_is_never_affected_by_config():
    for d in F.TRANSLATIONS:
        assert F.translation_unit(d, DOC)[0] == F.translation_unit(d, FLIP)[0]


@pytest.mark.parametrize("cfg, ccw, cw", [(DOC, 90.0, -90.0), (FLIP, -90.0, 90.0)])
def test_rotation_angle(cfg, ccw, cw):
    assert F.rotation_angle(F.CCW, 90.0, cfg) == ccw
    assert F.rotation_angle(F.CW, 90.0, cfg) == cw
    assert F.rotation_angle(F.CCW, -90.0, cfg) == ccw  # magnitude


def test_unknown_directions_raise():
    with pytest.raises(ValueError):
        F.translation_unit("up", DOC)
    with pytest.raises(ValueError):
        F.rotation_angle(F.LEFT, 10, DOC)
    with pytest.raises(ValueError):
        F.jog_field(F.FWD_LEFT, DOC)


@pytest.mark.parametrize("cfg, expected", [
    (DOC, {F.FORWARD: "bCmdJogFwd", F.BACK: "bCmdJogBwd",
           # PLC v2.1: bCmdJogLeft -> +vy; +Y = left -> physical left uses JogLeft
           F.LEFT: "bCmdJogLeft", F.RIGHT: "bCmdJogRight",
           F.CCW: "bCmdJogRotLeft", F.CW: "bCmdJogRotRight"}),
    (FLIP, {F.FORWARD: "bCmdJogFwd", F.BACK: "bCmdJogBwd",
            F.LEFT: "bCmdJogRight", F.RIGHT: "bCmdJogLeft",
            F.CCW: "bCmdJogRotRight", F.CW: "bCmdJogRotLeft"}),
])
def test_jog_bit_choice(cfg, expected):
    for direction, bit in expected.items():
        assert F.jog_field(direction, cfg) == bit


@pytest.mark.parametrize("cfg", [DOC, FLIP, F.FrameConfig(True, False), F.FrameConfig(False, True)])
def test_jog_and_move_use_the_same_plc_sign(cfg):
    """The jog bit chosen for a physical direction produces the same PLC sign as the relative move."""
    for d in (F.LEFT, F.RIGHT):
        _, uy = F.translation_unit(d, cfg)
        assert F.PLC_JOG_VY_SIGN[F.jog_field(d, cfg)] == uy
    for d in (F.CCW, F.CW):
        sign = math.copysign(1, F.rotation_angle(d, 1.0, cfg))
        assert F.PLC_JOG_OMEGA_SIGN[F.jog_field(d, cfg)] == sign


def test_from_config_defaults_and_values():
    # D21 (28.09.2026, PLC v2.1): default plus_y_is_left = true, plus_omega_is_ccw = true
    assert F.FrameConfig.from_config({}) == F.FrameConfig(True, True, False)
    assert F.FrameConfig.from_config({"frame": None}) == F.FrameConfig(True, True, False)
    cfg = {"frame": {"plus_y_is_left": True, "plus_omega_is_ccw": False, "verified": True}}
    assert F.FrameConfig.from_config(cfg) == F.FrameConfig(True, False, True)


def test_code_default_follows_config_yaml_default():
    """FrameConfig(), from_config() without keys and the shipped config give the same mapping."""
    shipped = amr_cfg(config.load())
    assert shipped["frame"]["plus_y_is_left"] is True
    assert shipped["frame"]["plus_omega_is_ccw"] is True
    assert shipped["frame"]["verified"] is True        # direction test with PLC v2.1 passed 28.09.2026 (D21)
    d, sh = F.FrameConfig(), F.FrameConfig.from_config(shipped)
    assert d == F.FrameConfig.from_config({})
    assert (d.plus_y_is_left, d.plus_omega_is_ccw) == (sh.plus_y_is_left, sh.plus_omega_is_ccw)
    assert d.verified is False                          # code default stays conservative
    assert config.load()["ares_ads"]["min_plc_build"] == "2.1"     # the mapping needs PLC >= v2.1 (D21)


def test_default_jog_bits_plc_v21():
    """PLC v2.1 (D21): 'Left' -> bCmdJogLeft (PLC +vy), 'CCW' -> bCmdJogRotLeft (PLC +omega)."""
    d = F.FrameConfig()
    assert F.jog_field(F.LEFT, d) == "bCmdJogLeft" and F.jog_field(F.RIGHT, d) == "bCmdJogRight"
    assert F.jog_field(F.CCW, d) == "bCmdJogRotLeft" and F.jog_field(F.CW, d) == "bCmdJogRotRight"
    assert F.translation_vector(F.LEFT, 100.0, d) == (0.0, 100.0)        # relative move: same sign as jog


def test_mapping_texts_are_neutral():
    d = F.FrameConfig()
    assert F.plus_y_text(d) == "PLC +Y (= left per config, unverified)"      # default since D21 (PLC v2.1)
    assert F.plus_omega_text(d) == "PLC +omega (= CCW per config, unverified)"
    v = F.FrameConfig(True, False, True)
    assert F.plus_y_text(v) == "PLC +Y (= left per config, verified)"
    assert F.plus_omega_text(v) == "PLC +omega (= CW per config, verified)"


@pytest.mark.parametrize("y_left, ccw", [(True, True), (False, True), (True, False), (False, False)])
def test_physical_pose_for_the_map(y_left, ccw):
    cfg = F.FrameConfig(y_left, ccw)
    fwd, left, head = F.physical_pose(1.5, 0.4, 30.0, cfg)
    assert fwd == 1.5                                          # X never affected
    assert left == pytest.approx(0.4 if y_left else -0.4)       # screen up = physical left
    assert head == pytest.approx(30.0 if ccw else -30.0)        # screen heading = physical CCW
    assert F.physical_pose(0.0, 0.0, 0.0, cfg) == (0.0, 0.0, 0.0)
    # a physical Left test move ends up drawn on the left (up) side, whatever the config says about PLC +Y
    _, dy = F.translation_vector(F.LEFT, 100.0, cfg)
    assert F.physical_pose(0.0, dy / 1000.0, 0.0, cfg)[1] == pytest.approx(0.1)
    # a physical CCW rotation is drawn CCW
    th = F.rotation_angle(F.CCW, 90.0, cfg)
    assert F.physical_pose(0.0, 0.0, th, cfg)[2] == pytest.approx(90.0)


def test_describe():
    assert F.describe_translation(0, 100, F.FrameConfig()) == "+Y 100 mm (left)"      # default config (D21)
    assert F.describe_translation(1000, 0, DOC) == "+X 1000 mm (forward)"
    assert F.describe_translation(0, 100, DOC) == "+Y 100 mm (left)"
    assert F.describe_translation(0, 100, FLIP) == "+Y 100 mm (right)"
    txt = F.describe_translation(707.1, -707.1, DOC)
    assert txt.startswith("+X 707 mm -Y 707 mm = 1000 mm") and "forward-right" in txt
    assert F.describe_rotation(90, DOC) == "+90.0 deg (CCW)"
    assert F.describe_rotation(90, FLIP) == "+90.0 deg (CW)"
    assert F.describe_rotation(-45, DOC) == "-45.0 deg (CW)"
