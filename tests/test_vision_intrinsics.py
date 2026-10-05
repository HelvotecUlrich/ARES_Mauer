"""Intrinsic calibration (mauer.vision.intrinsics), dataset folders (mauer.vision.dataset) and tools/calib_intrinsics.py
on synthetic ChArUco views with known ground truth.

ASSUMPTION (synthetic camera, the real lens is not calibrated yet): fx = 12 mm / 2.74 um = 4379.6 px from config,
fy/fx = 1.0002, principal point (1240.3, 1027.8) px, D = [-0.09, 0.18, 0.0004, -0.0003, 0] (research values).
Views: 18 of the 11x8 calib board at 290-355 mm, tilt 10-35 deg, random azimuth/roll, blur 0.8 px, noise 1.5 DN.
"""
import importlib.util
import json

import cv2
import numpy as np
import pytest

from mauer import REPO, config
from mauer.vision import intrinsics, synth, targets
from mauer.vision.dataset import Dataset


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(name, REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def board(cfg):
    return targets.board_specs(cfg)["calib"]


@pytest.fixture(scope="module")
def true_intr(cfg):
    nom = intrinsics.nominal(cfg)
    K = nom.K.copy()
    K[0, 2], K[1, 2], K[1, 1] = 1240.3, 1027.8, K[1, 1] * 1.0002
    return intrinsics.Intrinsics(K, [-0.09, 0.18, 0.0004, -0.0003, 0.0], nom.width, nom.height)


@pytest.fixture(scope="module")
def images(board, true_intr):
    rng = np.random.default_rng(0)
    out = []
    for i in range(18):
        T = synth.view(board, rng.uniform(290, 355), rng.uniform(10, 35) if i else 0.0, rng.uniform(0, 360),
                       rng.uniform(-180, 180), offset_mm=(rng.uniform(-40, 40), rng.uniform(-30, 30)))
        out.append(synth.render_board(board, true_intr, T, blur_sigma_px=0.8, noise_sigma=1.5, seed=i))
    return out


def test_nominal(cfg):
    n = intrinsics.nominal(cfg)
    assert n.fx == pytest.approx(12.0 / 2.74e-3) and n.fy == n.fx                     # 4379.56 px
    assert (n.cx, n.cy) == ((2472 - 1) / 2, (2064 - 1) / 2) and not n.D.any()
    assert n.size == (2472, 2064)


def test_calibrate_recovers_intrinsics(images, board, true_intr):
    blank = np.full_like(images[0], 255)
    partial = synth.render_board(board, true_intr, synth.view(board, 320.0, offset_mm=(-150.0, -110.0)))
    est, rep = intrinsics.calibrate(images + [blank, partial], board, min_corners=12)
    assert rep["n_used"] == 18 and rep["n_images"] == 20
    assert {r["view"] for r in rep["rejected"]} == {"18", "19"}
    # observed over 3 seeds (probe 2026-10-05): |dfx| <= 0.13 px (std 0.14), |dc| <= 0.25 px, RMS 0.03 px
    # -> bounds 1 px (0.023 % of fx; ~7 std) for fx, fy, cx, cy
    for a, b in ((est.fx, true_intr.fx), (est.fy, true_intr.fy), (est.cx, true_intr.cx), (est.cy, true_intr.cy)):
        assert abs(a - b) < 1.0
    assert rep["rms_px"] < 0.06 and rep["std"]["fx"] < 0.5 and rep["std"]["cx"] < 0.5
    assert est.D[4] == 0.0                                                             # CALIB_FIX_K3
    assert len(rep["per_view"]) == 18 and max(v["rms_px"] for v in rep["per_view"]) < 0.1
    assert rep["coverage"] > 0.6 and not any("RMS" in w for w in rep["warnings"])
    # the whole model: true rays reprojected with the estimate deviate < 1 px over the image (observed <= 0.34 px)
    gx, gy = np.meshgrid(np.linspace(0, 2471, 25), np.linspace(0, 2063, 21))
    pts = np.stack([gx.ravel(), gy.ravel()], 1).reshape(-1, 1, 2).astype(np.float64)
    rays = cv2.undistortPointsIter(pts, true_intr.K, true_intr.D, None, None,
                                   (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)).reshape(-1, 2)
    rep_pts = cv2.projectPoints(np.column_stack([rays, np.ones(len(rays))]), np.zeros(3), np.zeros(3), est.K,
                                est.D)[0].reshape(-1, 2)
    assert np.linalg.norm(rep_pts - pts.reshape(-1, 2), axis=1).max() < 1.0
    assert est.meta["board"] == board.to_dict() and est.meta["opencv"] == cv2.__version__
    assert est.meta["flags"] == ["CALIB_FIX_K3"] and len(est.meta["per_view_rms_px"]) == 18


def test_calibrate_needs_views(images, board):
    with pytest.raises(ValueError, match="usable views"):
        intrinsics.calibrate(images[:2], board)


def test_save_load_roundtrip(tmp_path, true_intr, board):
    intr = intrinsics.Intrinsics(true_intr.K, true_intr.D, 2472, 2064, 0.03, 18,
                                 {"std": {"fx": 0.1}, "per_view_rms_px": [0.03] * 18, "dataset": "data/x",
                                  "board": board.to_dict()})
    p = intrinsics.save(intr, tmp_path / "calib" / "camera_intrinsics.json")
    d = json.loads(p.read_text(encoding="utf-8"))
    assert {"K", "D", "width", "height", "rms_px", "n_views", "std", "per_view_rms_px", "opencv", "created",
            "dataset", "board"} <= set(d)
    assert len(d["D"]) == 5 and np.array(d["K"]).shape == (3, 3)
    back = intrinsics.load(p)
    assert np.array_equal(back.K, intr.K) and np.array_equal(back.D, intr.D)
    assert (back.width, back.height, back.rms_px, back.n_views) == (2472, 2064, 0.03, 18)
    assert targets.BoardSpec.from_dict(back.meta["board"]) == board


def test_dataset_roundtrip(tmp_path, images):
    ds = Dataset.create(tmp_path / "ds", "handeye", camera={"model": "synthetic", "res": np.array([2472, 2064])},
                        board="calib", notes="test")
    T = np.eye(4)
    T[:3, 3] = (400.0, -100.0, 300.0)
    s0 = ds.add(images[0], t_start=10.0, t_end=10.05, T_base_flange=T, q_rad=[0.1] * 6,
                tcp_pose_ur=[0.4, -0.1, 0.3, 0.0, 3.14, 0.0], max_qd=1e-4, extra_field="kept")
    ds.add(images[1])                                                       # no robot pose
    assert s0["image"] == "img_000.png" and len(ds) == 2
    with pytest.raises(FileExistsError):
        Dataset.create(tmp_path / "ds", "handeye")
    back = Dataset.load(tmp_path / "ds")
    assert back.kind == "handeye" and back.board == "calib" and back.meta["camera"]["res"] == [2472, 2064]
    items = list(back)
    assert np.array_equal(items[0][0], images[0]) and np.array_equal(items[1][0], images[1])   # lossless
    assert np.array_equal(Dataset.T_base_flange(items[0][1]), T) and Dataset.T_base_flange(items[1][1]) is None
    assert items[0][1]["extra_field"] == "kept" and items[0][1]["max_qd"] == 1e-4
    assert items[1][1]["q_rad"] is None and items[1][1]["t_start"] is None
    meta = json.loads((tmp_path / "ds" / "meta.json").read_text(encoding="utf-8"))
    assert set(meta) >= {"kind", "created", "camera", "board", "notes", "samples"}
    with pytest.raises(ValueError):
        Dataset.create(tmp_path / "other", "nonsense")


def test_tool_solve(tmp_path, images):
    ds = Dataset.create(tmp_path / "intr", "intrinsics", board="calib")
    for img in images[:10]:
        ds.add(img)
    tool = _load_tool("calib_intrinsics")
    out = tmp_path / "camera_intrinsics.json"
    assert tool.main(["solve", str(ds.path), "--out", str(out)]) == 0
    est = intrinsics.load(out)
    assert est.n_views == 10 and abs(est.fx - 12.0 / 2.74e-3) < 3.0
    assert est.meta["views"][0] == "img_000.png"
    assert tool.main(["solve", str(ds.path), "--out", str(out)]) == 2                   # no overwrite
    assert tool.main(["solve", str(ds.path), "--out", str(out), "--force"]) == 0


def test_tool_capture_with_file_camera(tmp_path, images, monkeypatch):
    """capture loop with the FileCamera (mauer.camera) and a scripted key sequence: SPACE, SPACE, q."""
    pytest.importorskip("mauer.camera")
    from mauer.vision.dataset import write_png
    src = tmp_path / "replay"
    src.mkdir()
    for i, img in enumerate(images[:3]):
        write_png(src / f"frame_{i:02d}.png", img)
    keys = iter([ord(" "), ord(" "), ord("q")])
    monkeypatch.setattr(cv2, "imshow", lambda *a, **k: None)
    monkeypatch.setattr(cv2, "waitKey", lambda *a, **k: next(keys))
    monkeypatch.setattr(cv2, "destroyAllWindows", lambda *a, **k: None)
    tool = _load_tool("calib_intrinsics")
    out = tmp_path / "captured"
    assert tool.main(["capture", "--camera", "files", "--files", str(src), "--dataset", str(out)]) == 0
    ds = Dataset.load(out)
    assert ds.kind == "intrinsics" and ds.board == "calib" and len(ds) == 2
    assert ds.meta["camera"]["kind"] == "files"
    for (img, s), ref in zip(ds, images[:2]):
        assert np.array_equal(img, ref) and s["n_corners"] >= 12 and s["t_end"] >= s["t_start"]
