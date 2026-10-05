"""ChArUco detection and board pose (mauer.vision.detect) on synthetic images with known ground truth.

Camera: GV-51F0CP geometry from config (2472 x 2064 px, f = 12 mm / 2.74 um = 4379.6 px). ASSUMPTION (synthetic, the
real lens is not calibrated yet): principal point off-centre (1240.3, 1027.8) px, fy/fx = 1.0002 and a modest
distortion D = [-0.09, 0.18, 0.0004, -0.0003, 0] (the research values, vision-method.json). "Nominal" images: blur
sigma 0.8 px, noise 1.5 DN. The pose is solved with the true K, D, so the errors are those of detection + PnP.
"""
import numpy as np
import pytest

from mauer import config
from mauer import geometry as g
from mauer.vision import detect, intrinsics, synth, targets

NOMINAL = dict(blur_sigma_px=0.8, noise_sigma=1.5)
# (tilt, azimuth, roll [deg], aim offset [mm]) at 320 mm: fronto-parallel, moderate and strong tilt, varied roll
VIEWS = [(0, 0, 0, (0, 0)), (0, 0, 5, (20, -15)), (20, 30, -40, (-25, 10)), (20, 210, 80, (15, 25)),
         (35, 120, 10, (-10, -20)), (35, 300, -85, (25, 0))]


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def specs(cfg):
    return targets.board_specs(cfg)


@pytest.fixture(scope="module")
def intr(cfg):
    nom = intrinsics.nominal(cfg)
    K = nom.K.copy()
    K[0, 2], K[1, 2], K[1, 1] = 1240.3, 1027.8, K[1, 1] * 1.0002      # ASSUMPTION, see module docstring
    return intrinsics.Intrinsics(K, [-0.09, 0.18, 0.0004, -0.0003, 0.0], nom.width, nom.height)


def _errors(T_true, T_est):
    """position error [mm] in the board plane (x, y) and along its normal, total rotation and in-plane yaw [deg]."""
    D = g.inv(T_true) @ T_est
    yaw = np.degrees(np.arctan2(D[1, 0], D[0, 0]))
    return np.linalg.norm(D[:2, 3]), abs(D[2, 3]), np.degrees(g.angle_of(D[:3, :3])), abs(yaw)


@pytest.mark.parametrize("name, lat_mm, normal_mm, rot_deg, yaw_deg", [
    # 11x8 board (70 corners over 150 x 105 mm): observed max over 3 seeds 0.007 mm / 0.004 deg -> 4x margin
    ("calib", 0.03, 0.03, 0.02, 0.01),
    # 5x4 reference board (12 corners over 64 x 48 mm): tilt about an in-plane axis is weakly observed in a
    # fronto-parallel view; observed max 0.04 mm / 0.051 deg (research Monte Carlo, 40 views: tilt max 0.075 deg)
    # -> 0.12 deg; position and the in-plane yaw (heading of face-up boards) stay far below 0.1 mm / 0.01 deg
    ("W0", 0.05, 0.1, 0.12, 0.01),
])
def test_pose_accuracy_at_320mm(specs, intr, name, lat_mm, normal_mm, rot_deg, yaw_deg):
    sp = specs[name]
    worst = np.zeros(4)
    corner_err = []
    for k, (tilt, az, roll, off) in enumerate(VIEWS):
        T = synth.view(sp, 320.0, tilt, az, roll, offset_mm=off if name != "calib" else (0, 0))
        img = synth.render_board(sp, intr, T, seed=k, **NOMINAL)
        assert img.shape == (2064, 2472) and img.dtype == np.uint8
        dets = detect.detect_boards(img, specs, border_px=10)
        assert set(dets) == {name}                                  # no other board's ids are reported
        d = dets[name]
        assert d.n >= 0.85 * sp.n_corners
        assert np.all((d.marker_ids >= sp.first_id) & (d.marker_ids <= sp.last_id))
        corner_err.append(np.linalg.norm(synth.project(d.obj_pts, intr, T) - d.img_pts, axis=1))
        p = detect.estimate_pose(d, intr, min_corners=8, max_reproj_px=1.0)
        assert p.ok, p.reason
        assert p.rms_px < 0.15
        worst = np.maximum(worst, _errors(T, p.T_cam_board))
    e = np.concatenate(corner_err)
    assert e.mean() < 0.05 and e.max() < 0.2          # research: 0.026 px mean, 0.093 px max (OpenCV 4.14)
    assert worst[0] < lat_mm and worst[1] < normal_mm and worst[2] < rot_deg and worst[3] < yaw_deg, worst


def test_two_boards_same_dictionary_in_one_image(specs, intr):
    """W0 (ids 30..39) and W1 (ids 40..49) side by side: each board sees only its own markers."""
    w0, w1 = specs["W0"], specs["W1"]
    # W1 origin 100 mm left of W0's in the same plane (20 mm gap, quiet zones do not cover the other board); the
    # camera aims at the middle of the pair (W0 x = -10 mm) from 360 mm (field ~203 mm wide)
    T0 = synth.view(w0, 360.0, 10, 0, 5, offset_mm=(-50.0, 0.0))
    T1 = T0 @ g.transl(-100.0, 0.0, 0.0)
    img = synth.render_boards([(w0, T0), (w1, T1)], intr, seed=3, **NOMINAL)
    poses = detect.measure(img, specs, intr, config.load()["vision"])
    assert {n for n, p in poses.items() if p.ok} == {"W0", "W1"}
    assert poses["S0"].reason == "not detected"
    for n, T in (("W0", T0), ("W1", T1)):
        dp, da = g.pose_delta(T, poses[n].T_cam_board)
        assert dp < 0.1 and da < 0.12
    d = detect.detect_boards(img, [w0, w1])
    assert set(d["W0"].marker_ids) <= set(range(30, 40)) and set(d["W1"].marker_ids) <= set(range(40, 50))


def test_border_corners_dropped(specs, intr):
    """Board partly outside the image: no corner within border_px, the remaining corners are not corrupted."""
    sp = specs["calib"]
    border = 10
    T = synth.view(sp, 300.0, 10, 0, 0, offset_mm=(-75.0, 0.0))     # left part of the board outside the image
    img = synth.render_board(sp, intr, T, seed=5, **NOMINAL)
    d = detect.detect_boards(img, [sp], border_px=border)[sp.name]
    assert 8 <= d.n < sp.n_corners
    W, H = intr.size
    p = d.img_pts
    assert p[:, 0].min() >= border and p[:, 0].max() <= W - 1 - border
    assert p[:, 1].min() >= border and p[:, 1].max() <= H - 1 - border
    err = np.linalg.norm(synth.project(d.obj_pts, intr, T) - p, axis=1)
    assert err.max() < 0.2
    pose = detect.estimate_pose(d, intr, 8, 1.0)
    assert pose.ok and g.pose_delta(T, pose.T_cam_board)[0] < 0.05
    # a wider border keeps fewer corners
    assert detect.detect_boards(img, [sp], border_px=200)[sp.name].n < d.n


def test_reference_board_mostly_out_of_view(specs, intr):
    """Only a corner of the small board in view -> too few corners -> ok=False with a reason."""
    sp = specs["S0"]
    T = synth.view(sp, 320.0, 0, 0, 0, offset_mm=(-118.0, -98.0))
    img = synth.render_board(sp, intr, T, seed=6, **NOMINAL)
    p = detect.measure(img, [sp], intr, {"min_corners": 8, "max_reproj_px": 1.0, "border_px": 10})["S0"]
    assert not p.ok and ("too few" in p.reason or p.reason == "not detected")


def test_rejections(specs, intr):
    sp = specs["W0"]
    obj = targets.corners_obj(sp)
    T = synth.view(sp, 320.0, 20, 45, 0)
    img_pts = synth.project(obj, intr, T)
    ids = np.arange(sp.n_corners, dtype=np.int32)
    good = detect.BoardDetection("W0", ids, img_pts, obj)
    assert detect.estimate_pose(good, intr, 8, 1.0).ok
    few = detect.BoardDetection("W0", ids[:6], img_pts[:6], obj[:6])
    assert "too few" in detect.estimate_pose(few, intr, 8, 1.0).reason
    row = slice(0, sp.squares_x - 1)                                      # one row of corners: collinear
    line = detect.BoardDetection("W0", ids[row], img_pts[row], obj[row])
    r = detect.estimate_pose(line, intr, 4, 1.0)
    assert not r.ok and "collinear" in r.reason
    noisy = detect.BoardDetection("W0", ids, img_pts + np.random.default_rng(0).normal(0, 3.0, img_pts.shape), obj)
    r = detect.estimate_pose(noisy, intr, 8, 1.0)
    assert not r.ok and "reprojection" in r.reason and r.T_cam_board is not None


def test_non_finite_ippe_falls_back(specs, intr, monkeypatch):
    """The research saw IPPE return NaN for exactly fronto-parallel noise-free views: then SQPNP is used, and a pose
    that stays non-finite is rejected."""
    sp = specs["calib"]
    obj = targets.corners_obj(sp)
    T = synth.view(sp, 320.0)
    det = detect.BoardDetection("calib", np.arange(sp.n_corners, dtype=np.int32), synth.project(obj, intr, T), obj)
    p = detect.estimate_pose(det, intr, 8, 1.0)                     # no fallback needed here
    assert p.ok and g.pose_delta(T, p.T_cam_board)[0] < 1e-6
    real = detect.cv2.solvePnPGeneric
    nan = (1, (np.full((3, 1), np.nan),), (np.full((3, 1), np.nan),), None)
    monkeypatch.setattr(detect.cv2, "solvePnPGeneric",
                        lambda *a, flags, **k: nan if flags == detect.cv2.SOLVEPNP_IPPE else real(*a, flags=flags, **k))
    p = detect.estimate_pose(det, intr, 8, 1.0)
    assert p.ok and g.pose_delta(T, p.T_cam_board)[0] < 1e-6          # SQPNP result, refined
    monkeypatch.setattr(detect.cv2, "solvePnPGeneric", lambda *a, **k: nan)
    p = detect.estimate_pose(det, intr, 8, 1.0)
    assert not p.ok and "non-finite" in p.reason


def test_ippe_rotation_vector_is_normalised_before_the_refinement(specs, intr, monkeypatch):
    """RoboDK 2026-10-05: IPPE gave |rvec| = 6.5e7 rad (an equivalent of the right rotation) for a board at ~180 deg
    roll and the LM refinement started there settled 0.78 mm off. The start vector is normalised (|rvec| <= pi) and
    both IPPE solutions are refined - an inflated start gives the same pose as the normal one."""
    sp = specs["W5"]
    R = (g.rotz(np.radians(179.0)) @ g.rotx(np.radians(4.0)))[:3, :3]
    T = g.make_T(R, np.array([3.0, 2.0, 320.0]) - R @ np.array([*sp.centre_mm, 0.0]))
    img = synth.render_board(sp, intr, T, supersample=2)              # rendered corners: LM depends on its start
    det = detect.detect_boards(img, [sp], 10)["W5"]
    good = detect.estimate_pose(det, intr, 8, 1.0)
    assert good.ok and g.pose_delta(T, good.T_cam_board)[0] < 0.2
    rv = detect.cv2.Rodrigues(good.T_cam_board[:3, :3])[0]
    th = float(np.linalg.norm(rv))
    big = rv / th * (th + 2.0 * np.pi * 1e7)                            # the same rotation, |rvec| ~ 6.3e7 rad
    assert np.allclose(detect.normalised_rvec(big), rv, atol=1e-6)
    real = detect.cv2.solvePnPGeneric
    monkeypatch.setattr(detect.cv2, "solvePnPGeneric",
                        lambda *a, **k: (lambda r: (r[0], tuple(rv_ / np.linalg.norm(rv_) * (np.linalg.norm(rv_) + 2.0
                                                                 * np.pi * 1e7) for rv_ in r[1]), r[2], r[3]))(
                            real(*a, **k)))
    p = detect.estimate_pose(det, intr, 8, 1.0)
    d_mm, d_deg = g.pose_delta(good.T_cam_board, p.T_cam_board)
    assert p.ok and d_mm < 1e-6 and d_deg < 1e-6                     # same start rotation -> same result


def test_nothing_in_view(specs, intr):
    blank = np.full((intr.height, intr.width), 200, np.uint8)
    assert detect.detect_boards(blank, specs) == {}
    wrong = synth.render_board(specs["W0"], intr, synth.view(specs["W0"], 320.0), seed=1, **NOMINAL)
    assert detect.detect_boards(wrong, [specs["W1"]]) == {}               # W1 ids 40..49 are not on W0


def test_coarse_pose_of_a_partly_visible_board(specs, intr, cfg):
    """A board shifted so that most of it is outside the image: too few ChArUco corners for a pose (measure), but
    the markers in view give a rough pose (detect.coarse_poses) whose POSITION re-aims the camera (a few mm; its
    orientation from one or two markers can be off by degrees and is not used)."""
    sp = specs["W0"]
    cx, cy = sp.centre_mm
    w_img = intr.width * cfg["camera"]["pixel_um"] / 1000.0 * 320.0 / cfg["camera"]["focal"]   # field width at 320 mm
    for shift in (w_img / 2.0 - 20.0, w_img / 2.0 + 10.0):    # 6 ChArUco corners in view / only 2 markers
        T = g.transl(-cx + shift, -cy, 320.0)                  # board centre near / beyond the right image edge
        img = synth.render_board(sp, intr, T, supersample=2)
        pose = detect.measure(img, [sp], intr, cfg["vision"])["W0"]
        rough = detect.coarse_poses(img, [sp], intr, cfg["vision"])
        assert not pose.ok
        assert "W0" in rough
        c = [[cx, cy, 0.0]]                                   # the board centre: only its position is used
        d_mm = float(np.linalg.norm(g.apply(rough["W0"].T_cam_board, c)[0] - g.apply(T, c)[0]))
        assert d_mm < 5.0
    full = synth.render_board(sp, intr, g.transl(-cx, -cy, 320.0), supersample=2)
    assert detect.coarse_poses(full, [sp], intr, cfg["vision"]) == {}         # fully visible: not a coarse case
