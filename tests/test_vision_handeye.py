"""Eye-in-hand calibration (mauer.vision.handeye) and tools/handeye_solve.py with a known ground truth.

Geometry: calib board on the ARES deck as in [boards.calib] (PLACEHOLDER pose), UR5 base from [ur5]; true
T_flange_cam = [camera.mount] (PLACEHOLDER) with an ASSUMED mounting error of (1.2, -0.8, 2.0) mm / (1.5, -2.0, 0.7)
deg. Look poses from handeye.plan_poses. Robot pose noise 0.1 mm / 0.01 deg per axis (ASSUMPTION; UR5 repeatability
is +-0.1 mm). Camera ASSUMPTION as in the other vision tests: off-centre principal point, D = [-0.09, 0.18, 4e-4,
-3e-4, 0]. Board poses come either from projected corners + 0.05 px noise (fast; the research found the same errors
as with rendered images) or from rendered images + detection.
"""
import importlib.util
import json

import numpy as np
import pytest

from mauer import REPO, config
from mauer import geometry as g
from mauer.vision import detect, handeye, intrinsics, synth, targets
from mauer.vision.dataset import Dataset

ROBOT_NOISE = (0.1, 0.01)            # mm, deg per axis (ASSUMPTION)


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def board(cfg):
    return targets.board_specs(cfg)["calib"]


@pytest.fixture(scope="module")
def intr(cfg):
    nom = intrinsics.nominal(cfg)
    K = nom.K.copy()
    K[0, 2], K[1, 2], K[1, 1] = 1240.3, 1027.8, K[1, 1] * 1.0002
    return intrinsics.Intrinsics(K, [-0.09, 0.18, 0.0004, -0.0003, 0.0], nom.width, nom.height)


@pytest.fixture(scope="module")
def T_base_board(cfg):
    return g.inv(config.T_ares_base(cfg)) @ config.pose(cfg["boards"]["calib"])


@pytest.fixture(scope="module")
def X(cfg):
    return config.T_flange_cam_nominal(cfg) @ g.pose_xyz_rpy([1.2, -0.8, 2.0], [1.5, -2.0, 0.7])


def _perturb(T, rng, t_mm, r_deg):
    return T @ g.pose_xyz_rpy(rng.normal(0, t_mm, 3), rng.normal(0, r_deg, 3))


def _measure_fast(board, intr, T_cam_board, rng, px=0.05):
    obj = targets.corners_obj(board)
    p = synth.project(obj, intr, T_cam_board)
    vis = (p[:, 0] > 10) & (p[:, 0] < intr.width - 11) & (p[:, 1] > 10) & (p[:, 1] < intr.height - 11)
    det = detect.BoardDetection(board.name, np.flatnonzero(vis).astype(np.int32),
                                p[vis] + rng.normal(0, px, (int(vis.sum()), 2)), obj[vis])
    pose = detect.estimate_pose(det, intr, 8, 1.0)
    assert pose.ok, pose.reason
    return pose.T_cam_board


def _samples(board, intr, T_base_board, X, n, seed, **plan):
    rng = np.random.default_rng(seed)
    A, B = [], []
    for T in handeye.plan_poses(T_base_board, X, n, seed=seed, spec=board, **plan):
        B.append(_measure_fast(board, intr, g.inv(T @ X) @ T_base_board, rng))
        A.append(_perturb(T, rng, *ROBOT_NOISE))
    return A, B


def test_plan_poses_look_at_board_centre(board, T_base_board, X):
    poses = handeye.plan_poses(T_base_board, X, 24, seed=3, spec=board)
    assert len(poses) == 24
    c = g.apply(T_base_board, [[*board.centre_mm, 0.0]])[0]
    n_into = T_base_board[:3, 2]                                   # board +z (into the board)
    az = []
    for T in poses:
        cam = T @ X
        v = c - cam[:3, 3]
        d = np.linalg.norm(v)
        assert 290.0 <= d <= 355.0
        assert np.allclose(v / d, cam[:3, 2], atol=1e-9)            # optical axis through the board centre
        tilt = np.degrees(np.arccos(cam[:3, 2] @ n_into))
        assert 15.0 - 1e-9 <= tilt <= 30.0 + 1e-9
        tb = g.inv(T_base_board)[:3, :3] @ cam[:3, 2]                # viewing direction in the board frame
        az.append(np.degrees(np.arctan2(tb[1], tb[0])) % 360.0)
    assert len(set((np.array(az) // 90).astype(int))) == 4         # tilt directions in all four quadrants
    again = handeye.plan_poses(T_base_board, X, 24, seed=3, spec=board)
    assert all(np.array_equal(a, b) for a, b in zip(poses, again))  # deterministic


def test_solve_recovers_mount(board, intr, T_base_board, X):
    A, B = _samples(board, intr, T_base_board, X, 20, seed=0)
    res = handeye.solve(A, B)
    assert res.method == "PARK" and res.n_poses == 20 and set(res.by_method) == set(handeye.METHODS)
    dp, da = g.pose_delta(X, res.T_flange_cam)
    # 10 seeds (probe 2026-10-05): mean 0.059 mm / 0.007 deg, max 0.088 mm / 0.013 deg -> bounds ~3x the max
    assert dp < 0.3 and da < 0.04
    vs = res.disagreement["vs_chosen"]
    for m in ("TSAI", "HORAUD"):                                   # research: equivalent to PARK
        assert vs[m]["pos_mm"] < 0.1 and vs[m]["ang_deg"] < 0.01
    assert not res.warnings, res.warnings
    assert res.motion["useful_pair_fraction"] > 0.5
    assert g.pose_delta(T_base_board, res.T_base_board)[0] < 0.5
    assert res.board_in_base_spread["pos_rms_mm"] < 0.5           # robot noise 0.1 mm + lever arm
    # hold-out poses from another plan: consistent with the in-sample board pose
    Ah, Bh = _samples(board, intr, T_base_board, X, 6, seed=1)
    ho = handeye.holdout_check(res, Ah, Bh, board, intr)
    assert ho["n"] == 6 and ho["pos_rms_mm"] < 0.6 and ho["ang_max_deg"] < 0.1
    assert 0.0 < ho["reproj_rms_px"] < 10.0


def test_wrong_pairing_is_visible(board, intr, T_base_board, X):
    """Shuffled robot poses (a pairing error) give a huge in-sample spread."""
    A, B = _samples(board, intr, T_base_board, X, 12, seed=4)
    res = handeye.solve(A[1:] + A[:1], B)
    assert res.board_in_base_spread["pos_rms_mm"] > 5.0


def test_small_rotations_are_flagged(board, intr, T_base_board, X):
    A, B = _samples(board, intr, T_base_board, X, 20, seed=2, tilt_deg=(2.0, 6.0), roll_deg=(-5.0, 5.0))
    res = handeye.solve(A, B)
    assert any("small relative rotations" in w for w in res.warnings)
    assert res.motion["useful_pair_fraction"] < 0.5
    # TSAI drops pairs below ~17 deg; OpenCV then returns an identity, which solve() reports as failed
    assert "TSAI" in res.disagreement["failed"]


def test_input_checks(board, intr, T_base_board, X):
    A, B = _samples(board, intr, T_base_board, X, 4, seed=5)
    with pytest.raises(ValueError):
        handeye.solve(A, B[:3])
    with pytest.raises(ValueError):
        handeye.solve(A[:2], B[:2])
    with pytest.raises(ValueError):
        handeye.solve(A, B, method="FOO")


@pytest.fixture(scope="module")
def rendered(board, intr, T_base_board, X):
    """12 rendered views (blur 0.8 px, noise 1.5 DN) with exact robot poses."""
    poses = handeye.plan_poses(T_base_board, X, 12, seed=1, spec=board)
    imgs = [synth.render_board(board, intr, g.inv(T @ X) @ T_base_board, blur_sigma_px=0.8, noise_sigma=1.5,
                               seed=i) for i, T in enumerate(poses)]
    return poses, imgs


def test_rendered_chain(rendered, board, intr, X, cfg):
    poses, imgs = rendered
    B = []
    for img in imgs:
        p = detect.measure(img, [board], intr, cfg["vision"])[board.name]
        assert p.ok, p.reason
        B.append(p.T_cam_board)
    res = handeye.solve(poses, B)
    dp, da = g.pose_delta(X, res.T_flange_cam)
    assert dp < 0.05 and da < 0.01                                  # observed 0.003 mm / 0.0005 deg


def test_save_load_roundtrip(tmp_path, board, intr, T_base_board, X):
    A, B = _samples(board, intr, T_base_board, X, 15, seed=6)
    res = handeye.solve(A, B)
    ho = handeye.holdout_check(res, A[:3], B[:3])
    p = handeye.save(res, tmp_path / "handeye.json", holdout=ho, dataset="data/x", intrinsics_file="calib/i.json")
    d = json.loads(p.read_text(encoding="utf-8"))
    assert {"T_flange_cam", "method", "n_poses", "disagreement", "board_in_base_spread", "holdout", "opencv",
            "created", "dataset", "intrinsics_file"} <= set(d)
    back = handeye.load(p)
    assert np.allclose(back.T_flange_cam, res.T_flange_cam) and back.method == "PARK" and back.n_poses == 15
    assert np.allclose(back.T_base_board, res.T_base_board) and set(back.by_method) == set(res.by_method)
    assert back.meta["holdout"]["n"] == 3 and back.meta["dataset"] == "data/x"
    assert "PARK" in handeye.format_result(back)


def test_tool_handeye_solve(tmp_path, rendered, intr, X):
    poses, imgs = rendered
    ds = Dataset.create(tmp_path / "he", "handeye", board="calib")
    for T, img in zip(poses, imgs):
        ds.add(img, t_start=0.0, t_end=0.01, T_base_flange=T, max_qd=0.0)
    ds.add(imgs[0])                                                 # no robot pose -> skipped
    ifile = intrinsics.save(intr, tmp_path / "intr.json")
    spec = importlib.util.spec_from_file_location("handeye_solve", REPO / "tools" / "handeye_solve.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    out = tmp_path / "handeye.json"
    argv = [str(ds.path), "--intrinsics", str(ifile), "--holdout-every", "4", "--out", str(out)]
    assert tool.main(argv) == 0
    res = handeye.load(out)
    assert res.n_poses == 9 and res.meta["holdout"]["n"] == 3      # 12 usable, every 4th held out
    assert g.pose_delta(X, res.T_flange_cam)[0] < 0.05
    assert res.meta["holdout"]["pos_max_mm"] < 0.1
    assert tool.main(argv) == 2                                     # refuses to overwrite
    assert tool.main(argv + ["--force"]) == 0
