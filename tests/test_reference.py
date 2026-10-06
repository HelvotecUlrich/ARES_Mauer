"""Frame fit from measured reference boards (mauer.reference) and the planar ARES pose helpers.

Board poses are built from the config placements ([[targets]], PLACEHOLDER layout) and an arbitrary true wall frame
in the UR base frame; measurement errors are injected per board (pose error) or per corner (noise), seeded.
"""
import math

import numpy as np
import pytest

from mauer import config
from mauer import geometry as g
from mauer.reference import (PARENTS, Pose2D, after_rotation, after_translation, ares_pose, board_centre, fit_frame,
                             placements, relative_move, T_base_parent_from, T_wall_ares, tilt_deg, wrap_angle)
from mauer.vision.targets import board_specs, corners_obj


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def specs(cfg):
    return board_specs(cfg)


@pytest.fixture(scope="module")
def pl(cfg):
    return placements(cfg)


@pytest.fixture(scope="module")
def T_base_wall(cfg):
    """True wall frame in the UR base frame: ARES at stop 0 of the front wall (wall frame ARES pose (920, 740, -90)),
    plus a small ARES tilt."""
    T_wall_ares_true = g.transl(920.0, 740.0, 0.0) @ g.rotz(-math.pi / 2) @ g.pose_xyz_rpy([0, 0, 0], [0.2, -0.1, 0])
    return g.inv(config.T_ares_base(cfg)) @ g.inv(T_wall_ares_true)


def _pl(pl, name):
    return next(p for p in pl if p.name == name)


def test_placements_from_config(cfg, pl):
    names = [p.name for p in pl]
    assert names[0] == "calib" and _pl(pl, "calib").parent == "deck"
    assert {p.name for p in pl if p.parent == "wall"} == {f"W{i}" for i in range(8)}
    assert {p.name for p in pl if p.parent == "station"} == {"S0", "S1"}
    assert all(p.parent in PARENTS for p in pl)
    t = cfg["targets"][0]
    assert np.allclose(_pl(pl, t["name"]).T_parent_board, g.pose_xyz_rpy(t["xyz"], t["rpy_deg"]))


def test_unknown_parent_rejected(cfg):
    bad = dict(cfg)
    bad["targets"] = [dict(cfg["targets"][0], parent="roof")]
    with pytest.raises(ValueError, match="roof"):
        placements(bad)


def test_single_board_is_its_6d_pose(pl, specs, T_base_wall):
    p = _pl(pl, "W1")
    err = g.pose_xyz_rpy([0.4, -0.3, 1.2], [0.3, -0.2, 0.05])          # a (bad) measurement of this board
    T_meas = T_base_wall @ p.T_parent_board @ err
    fit = fit_frame({"W1": T_meas}, pl, specs, "wall")
    assert np.allclose(fit.T_base_parent, T_meas @ g.inv(p.T_parent_board), atol=1e-9)
    assert fit.n_boards == 1 and fit.boards == ["W1"] and fit.baseline_mm == 0.0
    assert fit.rms_mm < 1e-9 and fit.residuals["W1"].pos_mm < 1e-9
    assert fit.n_points == specs["W1"].n_corners


def _heading_err_deg(T_fit, T_true):
    D = g.inv(T_true) @ T_fit
    return abs(math.degrees(math.atan2(D[1, 0], D[0, 0])))


def test_two_boards_far_apart_give_the_heading(pl, specs, T_base_wall):
    """Each board measured with a 0.1 deg yaw error and 0.05 mm lateral error (opposite signs): one board gives the
    full 0.1 deg heading error, two boards far apart (W0, W4 of the configured layout, > 1.5 m apart) only
    ~(0.1 mm / baseline) rad."""
    errs = {"W0": g.pose_xyz_rpy([0.0, 0.05, 0.0], [0, 0, 0.1]), "W4": g.pose_xyz_rpy([0.0, -0.05, 0.0], [0, 0, -0.1])}
    obs = {n: T_base_wall @ _pl(pl, n).T_parent_board @ e for n, e in errs.items()}
    one = fit_frame({"W0": obs["W0"]}, pl, specs, "wall")
    two = fit_frame(obs, pl, specs, "wall")
    assert _heading_err_deg(one.T_base_parent, T_base_wall) == pytest.approx(0.1, abs=1e-6)
    assert _heading_err_deg(two.T_base_parent, T_base_wall) < 0.01
    c = {n: board_centre(specs[n], _pl(pl, n).T_parent_board) for n in ("W0", "W4")}   # layout from the config
    assert two.baseline_mm == pytest.approx(float(np.linalg.norm(c["W4"] - c["W0"])), abs=1e-6)
    assert two.n_boards == 2 and two.rms_mm > 0.0 and two.baseline_mm > 1500.0


def test_noisy_corners(pl, specs, T_base_wall):
    """Board poses from corners with 0.1 mm noise (rigid fit per board, like a PnP result): the frame comes out within
    a few hundredths of a mm at the boards; the residual RMS reflects the noise."""
    rng = np.random.default_rng(3)
    obs = {}
    for n in ("W1", "W2"):
        c = corners_obj(specs[n])
        true = g.apply(T_base_wall @ _pl(pl, n).T_parent_board, c)
        obs[n] = g.fit_rigid(c, true + rng.normal(0.0, 0.1, true.shape))
    fit = fit_frame(obs, pl, specs, "wall")
    for n in ("W1", "W2"):
        T_pb = _pl(pl, n).T_parent_board
        d_mm, d_deg = g.pose_delta(fit.T_base_parent @ T_pb, T_base_wall @ T_pb)
        assert d_mm < 0.1 and d_deg < 0.05
    assert _heading_err_deg(fit.T_base_parent, T_base_wall) < 0.02     # ~0.03 mm centroid noise over 700 mm
    assert 0.0 < fit.rms_mm < 0.1


def test_moved_board_shows_in_the_residuals(pl, specs, T_base_wall):
    obs = {n: T_base_wall @ _pl(pl, n).T_parent_board for n in ("W1", "W2")}
    obs["W2"] = obs["W2"] @ g.transl(3.0, 0.0, 0.0)                    # W2 knocked 3 mm along its x axis
    fit = fit_frame(obs, pl, specs, "wall")
    assert fit.rms_mm == pytest.approx(1.5, abs=0.05)
    assert fit.residuals["W1"].pos_mm == pytest.approx(1.5, abs=0.1)
    assert fit.residuals["W2"].pos_mm == pytest.approx(1.5, abs=0.1)


def test_wrong_parent_boards_are_ignored(pl, specs, T_base_wall):
    T_base_station = g.transl(-2000.0, 300.0, -330.0) @ g.rotz(1.0)
    obs = {"W1": T_base_wall @ _pl(pl, "W1").T_parent_board,
           "S0": T_base_station @ _pl(pl, "S0").T_parent_board,
           "calib": g.transl(0, 0, 5) @ _pl(pl, "calib").T_parent_board,
           "X9": np.eye(4)}                                                  # no placement at all
    fit = fit_frame(obs, pl, specs, "wall")
    assert fit.boards == ["W1"] and fit.ignored == ["S0", "X9", "calib"]
    assert np.allclose(fit.T_base_parent, T_base_wall, atol=1e-9)
    st = fit_frame(obs, pl, specs, "station")
    assert st.boards == ["S0"] and np.allclose(st.T_base_parent, T_base_station, atol=1e-9)
    with pytest.raises(ValueError, match="no observed board of parent 'wall'"):
        fit_frame({"S0": obs["S0"]}, pl, specs, "wall")


def test_station_frame_from_both_station_boards(pl, specs):
    T_base_station = g.transl(-1500.0, 800.0, -330.0) @ g.rotz(math.radians(178.0))
    obs = {n: T_base_station @ _pl(pl, n).T_parent_board for n in ("S0", "S1")}
    fit = fit_frame(obs, pl, specs, "station")
    assert np.allclose(fit.T_base_parent, T_base_station, atol=1e-9)
    c0, c1 = (board_centre(specs[n], _pl(pl, n).T_parent_board) for n in ("S0", "S1"))
    assert fit.baseline_mm == pytest.approx(float(np.linalg.norm(c1 - c0)), abs=1e-6)


def test_ares_pose_in_the_wall_frame(cfg, T_base_wall):
    T_ab = config.T_ares_base(cfg)
    p = ares_pose(T_base_wall, T_ab)
    assert (p.x_mm, p.y_mm) == pytest.approx((920.0, 740.0), abs=1e-9)
    assert p.theta_deg == pytest.approx(-90.0, abs=1e-6)
    assert tilt_deg(T_wall_ares(T_base_wall, T_ab)) == pytest.approx(math.hypot(0.2, 0.1), abs=1e-3)
    flat = T_base_parent_from(p, T_ab)
    assert ares_pose(flat, T_ab).delta(p) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_relative_move_conventions():
    """Front wall: ARES faces the wall (theta -90 deg in the wall frame), the next stop 1400 mm further along the
    wall is a pure sideways move to the LEFT (+y body, README 'ARES moves sideways (+y)')."""
    a, b = Pose2D(920.0, 740.0, -math.pi / 2), Pose2D(2320.0, 740.0, -math.pi / 2)
    dx, dy, dth = relative_move(a, b)
    assert (dx, dy, dth) == pytest.approx((0.0, 1400.0, 0.0), abs=1e-9)
    assert after_translation(a, dx, dy).delta(b) == pytest.approx((0.0, 0.0), abs=1e-9)
    c = Pose2D(-1100.0, 700.0, math.pi / 2)
    dx, dy, dth = relative_move(a, c)
    assert after_rotation(after_translation(a, dx, dy), dth).delta(c) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert abs(dth) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi) and wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert Pose2D.from_dict(c.to_dict()) == c


def test_leg_placements(cfg, pl):
    """Wall boards with `leg`: xyz/rpy in the leg frame, composed with [[wall.legs]] (T_wall_leg @ pose)."""
    T_legs = config.leg_frames(cfg)
    for t in cfg["targets"]:
        if "leg" in t:
            assert np.allclose(_pl(pl, t["name"]).T_parent_board,
                               T_legs[t["leg"]] @ g.pose_xyz_rpy(t["xyz"], t["rpy_deg"]))
    bad = dict(cfg)
    bad["targets"] = [dict(cfg["targets"][0], leg="Z")]
    with pytest.raises(ValueError, match="unknown leg"):
        placements(bad)
    bad["targets"] = [dict(cfg["targets"][-1], leg="A")]                     # a station board with a leg
    with pytest.raises(ValueError, match="leg 'A' given for parent 'station'"):
        placements(bad)
