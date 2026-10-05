"""Hand-eye calibration end to end against URSim CB3 3.15.8 (Docker) with the synthetic camera.

Opt-in like tests/test_ur_ursim.py:   py.exe -m pytest -m ursim tests/test_capture_ursim.py   (or MAUER_URSIM=1).
Skipped when the Docker CLI/daemon or the image universalrobots/ursim_cb3:3.15.8 is missing (never pulled). The
container ursim_mauer is started for this module and stopped at the end if this module started it (the image stays).

World: calib board at its nominal deck pose ([boards.calib], PLACEHOLDER) in the UR base frame via [ur5] mount;
ground-truth T_flange_cam = [camera.mount] nominal (PLACEHOLDER) @ INJECT (a test value: a few mm / deg mounting
error); mauer.simcam.SynthCamera renders the board from the URSim flange pose (RTDE) with the nominal intrinsics
(blur 0.6 px, noise 1 DN). The tools run through their CLI entry points (tools/calib_handeye.py capture / solve /
verify / plan --check, tools/measure_target.py poses / repeat), each with its own URLink to URSim.

Tolerance of the calibrated T_flange_cam (0.1 mm / 0.02 deg) - what limits the result here:
- robot pose: the SynthCamera renders from link.flange_T() (latest RTDE sample) and capture_shot averages the RTDE
  samples of the frame window; URSim reports exact doubles of its own nominal FK and actual_qd = 0 at rest, so both
  are the same pose (checked in test_capture_shot_rtde to < 1e-6 mm). URScript pose literals are rounded to 0.1 um /
  1e-9 rad (geometry.ur_str) - only the commanded pose, which the result does not use.
- image: render + ChArUco detection noise, ~0.03 px per corner (mauer/vision/synth.py research) -> board pose a few
  um at 300 mm (offline probe: <= 0.005 mm / 0.003 deg per view), averaged over ~15 poses by calibrateHandEye.
  Observed: offline loop (tests/test_capture.py, 3 plans) 0.002-0.005 mm / <= 0.0011 deg; this module on URSim
  (2026-10-05, 20 planned, 19 reachable, 16 used + 3 held out) 0.0025 mm / 0.0008 deg. 0.1 mm / 0.02 deg leaves
  > 20x margin for other plan / noise seeds while still failing on any pairing or timing error (a frame matched to
  a pose 1 s off moves by > 10 mm). The ~0.3 mm / 0.05 deg first proposed would only be needed with real robot
  noise (UR5 +-0.1 mm; tests/test_vision_handeye.py: 0.06 mm mean with 0.1 mm / 0.01 deg pose noise).
"""
import json
import logging
import os
import sys
import threading
import time

import numpy as np
import pytest

import ursim
from mauer import REPO, config
from mauer import geometry as g
from mauer.capture import CaptureError, capture_shot, goto_look
from mauer.simcam import SynthCamera
from mauer.ur import script as s
from mauer.ur.link import URLink
from mauer.vision import handeye, intrinsics, targets
from mauer.vision.dataset import Dataset

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import calib_handeye as ch  # noqa: E402
import measure_target as mt  # noqa: E402

pytestmark = pytest.mark.ursim
logging.getLogger("rtde").setLevel(logging.CRITICAL)

INJECT = "1.5,-2.0,2.5,1.0,-1.5,0.8"        # dx, dy, dz [mm], rx, ry, rz [deg] - test value
N_POSES, SEED = 20, 0
TOL_MM, TOL_DEG = 0.1, 0.02
RESULTS: dict = {}                           # numbers for the report (printed at the end with -s)


@pytest.fixture(scope="module")
def sim(request):
    if "ursim" not in (request.config.getoption("markexpr") or "") and os.environ.get("MAUER_URSIM") != "1":
        pytest.skip("URSim tests are opt-in: py.exe -m pytest -m ursim tests/test_capture_ursim.py")
    ok, why = ursim.available()
    if not ok:
        pytest.skip(f"URSim not available: {why}")
    started = False
    try:
        started = ursim.start(boot_timeout_s=240.0)
        yield ursim.HOST
    finally:
        if started:                      # a container someone else started keeps running
            ursim.stop()
        print("\nURSim capture results: " + json.dumps(RESULTS, indent=1, default=float), flush=True)


@pytest.fixture(scope="module")
def link(sim):
    t0 = time.time()
    while True:   # RTDE may come up a little after the Dashboard
        try:
            ur = URLink.from_config(config.load(), host=sim).start()
            break
        except Exception:
            if time.time() - t0 > 30.0:
                raise
            time.sleep(1.0)
    yield ur
    ur.stop()


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def work(tmp_path_factory):
    return tmp_path_factory.mktemp("ursim_capture")


@pytest.fixture(scope="module")
def X_true(cfg):
    return config.T_flange_cam_nominal(cfg) @ ch.parse_inject(INJECT)


def _home(link, cfg):
    r = goto_look(link, cfg, q_rad=ch.Q_REF, sim=True, speeds=ch.URSIM_SPEEDS, timeout_s=30.0, name="home")
    assert r.ok, r.error


def test_capture_shot_rtde(link, cfg, X_true):
    """One look pose: the flange pose of the shot is the pose the image was rendered from."""
    _home(link, cfg)
    p = ch.make_plan(cfg, N_POSES, SEED)[0]
    r = goto_look(link, cfg, T_base_flange=p.T_base_flange, qnear_rad=p.qnear_rad, sim=True, speeds=ch.URSIM_SPEEDS)
    assert r.ok, r.error
    spec = targets.board_specs(cfg)["calib"]
    cam = SynthCamera(lambda: link.flange_T(), X_true, [(spec, ch.calib_board_in_base(cfg))], intrinsics.nominal(cfg))
    with cam:
        shot = capture_shot(link, cam, settle_s=0.0, max_qd=0.01, retries=0)
    d_true = g.pose_delta(np.asarray(shot.frame.meta["T_base_flange_true"]), shot.T_base_flange)
    d_plan = g.pose_delta(p.T_base_flange, shot.T_base_flange)
    RESULTS["shot"] = {"n_samples": shot.n_samples, "window_ms": 1000 * shot.frame.latency_s, "max_qd": shot.max_qd,
                       "vs_render_pose_mm": d_true[0], "reached_vs_plan_mm": d_plan[0],
                       "reached_vs_plan_deg": d_plan[1], "q_vs_nominal_ik_deg": float(np.degrees(
                           np.abs([ch._wrap(a) for a in shot.q_rad - p.qnear_rad]).max()))}
    assert shot.match == "window" and shot.n_samples >= 3 and shot.max_qd < 1e-6
    assert d_true[0] < 1e-6 and d_true[1] < 1e-6
    assert d_plan[0] < 0.1 and d_plan[1] < 0.01                     # controller IK reaches the planned flange pose
    assert RESULTS["shot"]["q_vs_nominal_ik_deg"] < 0.01            # same branch as the nominal qnear


def test_moving_robot_is_rejected(link, cfg):
    _home(link, cfg)
    far = list(ch.Q_REF)
    far[5] += 1.0                                                    # wrist 3 only, 0.2 rad/s
    th = threading.Thread(target=lambda: link.run_block(s.movej_q(far, 0.5, 0.2), "slow", 30))
    th.start()
    try:
        link.wait_until(lambda x: np.abs(x.actual_qd).max() > 0.1, 5.0)
        with pytest.raises(CaptureError, match="arm moving before the grab"):
            capture_shot(link, _StubCam(), settle_s=0.0, max_qd=0.01, retries=1, retry_wait_s=0.2)
    finally:
        th.join(30.0)
    shot = capture_shot(link, _StubCam(), settle_s=0.0, max_qd=0.01, retries=0)     # at rest again
    assert shot.max_qd < 1e-6


class _StubCam:
    def grab(self):
        from mauer.camera.base import Frame
        t0 = time.time()
        time.sleep(0.05)
        return Frame(np.zeros((8, 8), np.uint8), t0, time.time(), {})


def test_plan_check_on_controller(sim, cfg, capsys):
    assert ch.main(["plan", "--n", str(N_POSES), "--seed", str(SEED), "--check", "--ursim"]) == 0
    out = capsys.readouterr().out
    nominal = sum(p.qnear_rad is not None for p in ch.make_plan(cfg, N_POSES, SEED))
    n_ctrl = int(out.split("controller IK: ")[1].split("/")[0])
    RESULTS["plan_check"] = {"n_planned": N_POSES, "nominal_ik": nominal, "controller_ik": n_ctrl}
    assert n_ctrl == nominal


def test_handeye_capture_and_solve(sim, cfg, work, X_true, capsys):
    ds_path = work / "he"
    rc = ch.main(["capture", "--ursim", "--yes", "--dataset", str(ds_path), "--n", str(N_POSES), "--seed",
                  str(SEED), "--inject", INJECT, "--settle-s", "0"])
    out = capsys.readouterr().out
    assert rc == 0, out
    ds = Dataset.load(ds_path)
    unreachable = ds.meta["unreachable"]
    RESULTS["capture"] = {"planned": N_POSES, "captured": len(ds), "unreachable": unreachable,
                          "corners_min": min(x["corners"] for x in ds.samples),
                          "samples_per_shot_min": min(x["n_samples"] for x in ds.samples),
                          "reached_vs_plan_max_mm": max(g.pose_delta(np.asarray(x["T_base_flange_planned"]),
                                                                     Dataset.T_base_flange(x))[0]
                                                        for x in ds.samples)}
    assert len(ds) + len(unreachable) == N_POSES and len(ds) >= 15
    assert all(x["max_qd"] < 1e-6 and x["match"] == "window" for x in ds.samples)
    assert np.allclose(ds.meta["truth"]["T_flange_cam"], X_true)
    # resume: nothing left to capture (unreachable poses are retried and rejected again, no other motion)
    assert ch.main(["capture", "--ursim", "--yes", "--dataset", str(ds_path), "--n", str(N_POSES), "--seed",
                    str(SEED), "--inject", INJECT, "--settle-s", "0", "--resume"]) == 0
    assert len(Dataset.load(ds_path)) == len(ds)
    capsys.readouterr()

    he = work / "handeye.json"
    assert ch.main(["solve", str(ds_path), "--out", str(he)]) == 0
    out = capsys.readouterr().out
    assert "vs ground truth" in out
    res = handeye.load(he)
    dp, da = g.pose_delta(X_true, res.T_flange_cam)
    ho = res.meta["holdout"]
    RESULTS["solve"] = {"n_poses": res.n_poses, "err_mm": dp, "err_deg": da, "holdout_n": ho["n"],
                        "holdout_pos_max_mm": ho["pos_max_mm"], "holdout_ang_max_deg": ho["ang_max_deg"],
                        "holdout_reproj_rms_px": ho["reproj_rms_px"], "warnings": res.warnings,
                        "board_vs_truth_mm": g.pose_delta(ch.calib_board_in_base(cfg), res.T_base_board)[0],
                        "methods_max_disagree_mm": res.disagreement["pairwise_max_pos_mm"]}
    assert dp < TOL_MM and da < TOL_DEG, (dp, da)
    assert ho["n"] >= 3 and ho["pos_max_mm"] < TOL_MM


def test_verify_on_new_poses(sim, work, capsys):
    he = work / "handeye.json"
    rc = ch.main(["verify", "--ursim", "--yes", "--handeye", str(he), "--n", "5", "--inject", INJECT,
                  "--settle-s", "0", "--tol-mm", str(TOL_MM), "--tol-deg", str(TOL_DEG)])
    out = capsys.readouterr().out
    RESULTS["verify"] = [ln for ln in out.splitlines() if "verification poses" in ln or "ground truth" in ln
                         or ln.startswith(("PASS", "FAIL"))]
    assert rc == 0, out


def test_measure_target_poses_consistency(sim, cfg, work, capsys):
    poses = work / "poses.json"
    assert ch.main(["plan", "--n", "6", "--seed", "2000", "--write", str(poses)]) == 0
    rep = work / "poses_report.json"
    rc = mt.main(["poses", "--ursim", "--yes", "--poses", str(poses), "--handeye", str(work / "handeye.json"),
                  "--boards", "calib", "--inject", INJECT, "--settle-s", "0", "--report", str(rep)])
    out = capsys.readouterr().out
    assert rc == 0, out
    r = json.loads(rep.read_text(encoding="utf-8"))
    b = r["boards"]["calib"]
    RESULTS["measure_poses"] = {"n_poses": r["n_poses"], "n_reached": r["n_reached"], "unreachable": r["unreachable"],
                                "n_measured": b["n"], "pos_max_mm": b["pos_max_mm"], "ang_max_deg": b["ang_max_deg"],
                                "truth_max_mm": b["truth_max_mm"], "truth_max_deg": b["truth_max_deg"]}
    assert b["n"] >= 4 and b["pos_max_mm"] < TOL_MM and b["ang_max_deg"] < TOL_DEG
    assert b["truth_max_mm"] < 2 * TOL_MM


def test_measure_target_repeat_with_bump(sim, work, capsys):
    """repeat from the current (last look) pose with a 20 mm up-and-back move before every shot."""
    rep = work / "repeat.json"
    rc = mt.main(["repeat", "--ursim", "--yes", "--n", "3", "--move-mm", "20", "--settle-s", "0,0.2", "--boards",
                  "calib", "--handeye", str(work / "handeye.json"), "--inject", INJECT, "--report", str(rep)])
    out = capsys.readouterr().out
    assert rc == 0, out
    r = json.loads(rep.read_text(encoding="utf-8"))
    RESULTS["repeat"] = [{"settle_s": e["settle_s"], "pos_max_mm": e["boards"]["calib"]["pos_max_mm"],
                          "flange_return_max_mm": e["flange"]["pos_max_mm"]} for e in r["settle"]]
    for e in r["settle"]:
        assert e["boards"]["calib"]["n"] == 3 and e["boards"]["calib"]["pos_max_mm"] < TOL_MM
