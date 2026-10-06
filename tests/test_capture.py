"""mauer.capture and the robot-side tools (tools/calib_handeye.py, tools/measure_target.py) without a robot.

A fake link replays RTDE samples at 125 Hz on a simulated clock (World): the flange pose, q and qd come from a motion
function of time, so sample matching, the nearest-sample fallback and the moving-robot retry are deterministic.
Cameras: SynthCamera (rendered boards, clock = the world clock) and FileCamera (replayed sidecar time stamps).
The full capture -> dataset -> solve loop of calib_handeye runs against a fake rig whose run_block "moves" instantly
and rejects poses without a nominal UR5 IK solution like the controller's IK guard (error code 1).
"""
import copy
import logging
import math
import re
import sys

import numpy as np
import pytest

from mauer import REPO, capture, config
from mauer import geometry as g
from mauer.camera.base import Camera, Frame
from mauer.camera.files import FileCamera, save_frame
from mauer.capture import CaptureError, boards_in_base, capture_shot, goto_look, match_samples
from mauer.simcam import SynthCamera
from mauer.ur import script
from mauer.ur.link import BlockResult, URState
from mauer.vision import intrinsics, targets
from mauer.vision.dataset import Dataset

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import calib_handeye as ch  # noqa: E402
import measure_target as mt  # noqa: E402

T0 = 1000.0                 # world time of sample k = 0 [s]
DT = 1.0 / 125.0            # RTDE period [s]
INJECT = g.pose_xyz_rpy([1.5, -2.0, 2.5], [1.0, -1.5, 0.8])    # mounting error of the "true" camera (test value)


@pytest.fixture(scope="module")
def cfg():
    c = config.load()
    # The reachability numbers below (23 of 25 poses, p19/p23 unreachable) belong to this PLACEHOLDER board pose;
    # pin it so that a config change does not silently change the test scenario.
    c["boards"]["calib"]["xyz"] = [-82.5, 60.0, 336.6]
    c["boards"]["calib"]["rpy_deg"] = [180.0, 0.0, 0.0]
    c["camera"]["mount"] = {"xyz": [-150.0, 0.0, 60.0], "rpy_deg": [0.0, 0.0, 0.0]}   # mount of that scenario
    c["ur"]["payload_tool_kg"] = 0.0      # unknown payload: refusal + sim fallback (the real 1.68 kg is set 2026-10-06)
    return c


def config_file_without_payload(tmp_path) -> str:
    """station.toml with payload_tool_kg = 0 (unknown) for the CLI refusal tests."""
    text = (REPO / "config" / "station.toml").read_text(encoding="utf-8")
    new = re.sub(r"(?m)^payload_tool_kg = [0-9.]+", "payload_tool_kg = 0.0", text)
    assert new != text
    p = tmp_path / "station_no_payload.toml"
    p.write_text(new, encoding="utf-8")
    return str(p)


@pytest.fixture(scope="module")
def T_ft(cfg):
    return config.T_flange_tcp(cfg)


class World:
    """Simulated clock + RTDE sample grid t_k = T0 + k * DT; motion(t) -> (T_base_flange, q, qd)."""

    def __init__(self, motion, t=T0 + 1.0, gaps=()):
        self.t, self.motion, self.gaps = float(t), motion, list(gaps)

    def has(self, t: float) -> bool:
        return not any(a <= t <= b for a, b in self.gaps)

    def times(self, t0: float, t1: float) -> list[float]:
        t1 = min(t1, self.t)
        k0, k1 = math.ceil((t0 - T0) / DT - 1e-9), math.floor((t1 - T0) / DT + 1e-9)
        return [T0 + k * DT for k in range(max(k0, 0), k1 + 1) if self.has(T0 + k * DT)]

    def sleep(self, s: float) -> None:
        self.t += float(s)

    def ticker(self, step_s: float):
        """Clock that advances the world by step_s per call (a camera grab spans one step)."""
        def clock():
            t = self.t
            self.t += step_s
            return t
        return clock


class FakeLink:
    """The parts of URLink that mauer.capture uses, on the World clock."""
    reg_error = script.REG_ERROR
    host = "fake"

    def __init__(self, world: World, T_ft: np.ndarray):
        self.world, self.T_ft = world, T_ft
        self.blocks: list[str] = []
        self.popups = 0

    def sample(self, t: float) -> URState:
        T, q, qd = self.world.motion(t)
        return URState(t_laptop=t, timestamp=t - T0, actual_q=np.asarray(q, float), actual_qd=np.asarray(qd, float),
                       actual_TCP_pose=np.array(g.T_to_ur(T @ self.T_ft)), robot_mode=7, safety_mode=1,
                       runtime_state=1, robot_status_bits=1, reg_started=0, reg_done=0, reg_error=0,
                       tcp_offset=np.array(g.T_to_ur(self.T_ft)))

    def samples(self, t0, t1=None):
        return [self.sample(t) for t in self.world.times(t0, float("inf") if t1 is None else t1)]

    def state(self):
        ts = self.world.times(self.world.t - 10.0, self.world.t)
        return self.sample(ts[-1]) if ts else None

    def state_age_s(self) -> float:
        s = self.state()
        return float("inf") if s is None else self.world.t - s.t_laptop

    def wait_until(self, pred, timeout_s):
        end = self.world.t + timeout_s
        while True:
            s = self.state()
            if s is not None and pred(s):
                return s
            if self.world.t >= end:
                return None
            self.world.t = min(end, self.world.t + DT / 4)

    def flange_T(self, s=None):
        return (s or self.state()).T_base_flange_mm()

    def close_popup(self):
        self.popups += 1


class StubCamera(Camera):
    """Grey frames; each grab spans `duration_s` of world time starting at `offset_s` after now."""

    def __init__(self, world: World, duration_s: float = 0.05, offset_s: float = 0.0):
        self.world, self.duration_s, self.offset_s, self._open = world, duration_s, offset_s, True

    is_open = property(lambda self: self._open)

    def open(self):
        self._open = True

    def close(self):
        self._open = False

    def grab(self) -> Frame:
        self.world.t += self.offset_s
        t0 = self.world.t
        self.world.t += self.duration_s
        return Frame(np.full((48, 64), 128, np.uint8), t0, self.world.t, {"camera": "stub"})

    def set_exposure_us(self, us):
        return us

    def set_gain(self, gain):
        return gain

    def info(self):
        return {"kind": "stub"}


def still(T, q=np.zeros(6)):
    return lambda t: (T, q, np.zeros(6))


def drifting(T, v_mm_s=0.1, qd=1e-4):
    """Slow drift along base x (below the stillness limit) - the window mean must be the mean of the samples."""
    return lambda t: (g.transl(v_mm_s * (t - T0), 0, 0) @ T, np.zeros(6), np.full(6, qd))


T_LOOK = g.pose_xyz_rpy([-350.0, 20.0, 380.0], [180.0, 0.0, 30.0])


# ── sample matching ───────────────────────────────────────────────────────────
def test_window_mean_of_samples_inside(T_ft):
    w = World(drifting(T_LOOK), t=T0 + 0.0011)            # just after sample k = 0
    link = FakeLink(w, T_ft)
    shot = capture_shot(link, StubCamera(w, 0.05), settle_s=0.0, max_qd=0.01, retries=0)
    times = [T0 + k * DT for k in range(1, 7)]               # 1000.008 .. 1000.048 lie in [1000.0011, 1000.0511]
    assert shot.match == "window" and shot.n_samples == len(times) and shot.attempts == 1
    expect = g.transl(0.1 * (np.mean(times) - T0), 0, 0) @ T_LOOK
    assert g.pose_delta(expect, shot.T_base_flange)[0] < 1e-9
    assert shot.max_qd == pytest.approx(1e-4) and shot.t_pose == pytest.approx(np.mean(times))
    assert shot.spread_mm == pytest.approx(0.1 * (times[-1] - times[0]) / 2, abs=1e-9)
    assert g.pose_delta(g.ur_to_T(shot.tcp_pose_ur), shot.T_base_flange @ T_ft)[0] < 1e-6


def test_nearest_sample_fallback(T_ft):
    w = World(drifting(T_LOOK), t=T0 + 0.0011)
    link = FakeLink(w, T_ft)
    shot = capture_shot(link, StubCamera(w, 0.004), retries=0)        # [1000.0011, 1000.0051]: no sample inside
    assert shot.match == "nearest" and shot.n_samples == 2
    expect = g.transl(0.1 * (DT / 2), 0, 0) @ T_LOOK                  # mean of samples k = 0 and k = 1
    assert g.pose_delta(expect, shot.T_base_flange)[0] < 1e-9


def test_nearest_only_within_50_ms(T_ft):
    w = World(still(T_LOOK), t=T0 + 1.0, gaps=[(T0 + 0.9, T0 + 1.2)])   # RTDE silent around the frame
    link = FakeLink(w, T_ft)
    picked, how = match_samples(link, T0 + 1.0, T0 + 1.004)
    assert picked == [] and how == "none"
    w.gaps = [(T0 + 0.97, T0 + 1.04)]                                   # before 30 ms away, after 36 ms away
    w.t = T0 + 1.004
    picked, how = match_samples(link, T0 + 1.0, T0 + 1.004)
    assert how == "nearest" and len(picked) == 2
    assert T0 + 1.0 - picked[0].t_laptop < 0.05 and picked[1].t_laptop - (T0 + 1.004) < 0.05


def test_no_samples_raises_after_retries(T_ft):
    w = World(still(T_LOOK), t=T0 + 1.0, gaps=[(T0 + 0.9, T0 + 5.0)])
    link = FakeLink(w, T_ft)
    link.state_age_s = lambda: 0.0                           # pretend the link is streaming
    with pytest.raises(CaptureError) as e:
        capture_shot(link, StubCamera(w, 0.004), retries=1, retry_wait_s=0.1, sleep=w.sleep)
    assert len(e.value.attempts) == 2 and all("no RTDE sample within 50 ms" in a for a in e.value.attempts)


def test_stale_link_and_no_state(T_ft):
    w = World(still(T_LOOK), t=T0 + 1.0, gaps=[(T0 + 0.2, T0 + 5.0)])   # last sample 0.8 s old
    with pytest.raises(CaptureError, match="old"):
        capture_shot(FakeLink(w, T_ft), StubCamera(w), retries=0)
    w2 = World(still(T_LOOK), t=T0 - 1.0)                               # before the first sample
    with pytest.raises(CaptureError, match="no RTDE state"):
        capture_shot(FakeLink(w2, T_ft), StubCamera(w2), retries=0)


# ── moving robot ──────────────────────────────────────────────────────────────
def moving_until(t_stop: float, qd: float = 0.05):
    def motion(t):
        a = qd * (min(t, t_stop) - T0)
        return T_LOOK @ g.rotz(a), np.full(6, a), np.full(6, qd if t < t_stop else 0.0)
    return motion


def test_moving_robot_retry_then_ok(T_ft):
    t_start = T0 + 1.0
    w = World(moving_until(t_start + 0.8), t=t_start)
    link = FakeLink(w, T_ft)
    with pytest.raises(CaptureError, match="arm moving before the grab"):
        capture_shot(link, StubCamera(w), retries=0)
    w.t = t_start
    shot = capture_shot(link, StubCamera(w), max_qd=0.01, retries=2, retry_wait_s=0.5, sleep=w.sleep)
    assert shot.attempts == 3 and len(shot.errors) == 2 and shot.max_qd == 0.0     # moving at +0, +0.5; still at +1.0


def test_motion_inside_window_is_rejected(T_ft):
    t_start = T0 + 1.0

    def motion(t):                                            # still, a joint moves 30-70 ms later, still again
        moving = t_start + 0.03 <= t < t_start + 0.07
        return T_LOOK, np.zeros(6), np.full(6, 0.2 if moving else 0.0)

    w = World(motion, t=t_start)
    link = FakeLink(w, T_ft)
    with pytest.raises(CaptureError, match="moved during the exposure window") as e:
        capture_shot(link, StubCamera(w, 0.05), retries=0)
    assert e.value.shot is not None and e.value.shot.max_qd == pytest.approx(0.2)
    w.t = t_start
    shot = capture_shot(link, StubCamera(w, 0.05), retries=1, retry_wait_s=0.2, sleep=w.sleep)
    assert shot.attempts == 2 and shot.max_qd == 0.0


# ── boards in base ────────────────────────────────────────────────────────────
def _board_below(spec, T_base_cam, dist=320.0):
    c = np.array([*spec.centre_mm, 0.0])
    return g.transl(*(T_base_cam[:3, 3] + [0, 0, -dist])) @ g.rotx(np.pi) @ g.transl(*-c)


def test_boards_in_base_synth_matches_truth(cfg, T_ft):
    specs = targets.board_specs(cfg)
    intr = intrinsics.nominal(cfg)
    X_true = config.T_flange_cam_nominal(cfg) @ INJECT
    T_bf = g.pose_xyz_rpy([-200.0, 150.0, 450.0], [180.0, 0.0, 20.0])
    T_bb = _board_below(specs["W0"], T_bf @ X_true)
    w = World(still(T_bf), t=T0 + 2.0)
    link = FakeLink(w, T_ft)
    cam = SynthCamera(lambda: link.flange_T(), X_true, [(specs["W0"], T_bb)], intr, clock=w.ticker(0.03))
    cam.open()
    shot = capture_shot(link, cam, retries=0)
    assert shot.match == "window" and shot.n_samples >= 3 and g.pose_delta(shot.T_base_flange, T_bf)[0] < 1e-9
    seen = boards_in_base(shot, {n: specs[n] for n in ("W0", "W1", "calib")}, intr, cfg["vision"], X_true)
    assert set(seen) == {"W0", "W1", "calib"}
    bp, T = seen["W0"]
    assert bp.ok and T is not None
    d_mm, d_deg = g.pose_delta(T, T_bb)
    assert d_mm < 0.2 and d_deg < 0.1                         # rendered + detected (test_simcam: same bounds)
    assert seen["W1"][1] is None and not seen["W1"][0].ok     # not in the image
    # a wrong hand-eye shows up 1:1 in the base frame
    bp2, T2 = boards_in_base(shot, [specs["W0"]], intr, cfg["vision"], config.T_flange_cam_nominal(cfg))["W0"]
    assert g.pose_delta(T2, T_bb)[0] > 2.0


def test_file_camera_replay_matches_recorded_times(cfg, T_ft, tmp_path):
    w = World(drifting(T_LOOK, v_mm_s=1.0, qd=0.002), t=T0 + 10.0)
    link = FakeLink(w, T_ft)
    img = np.full((48, 64), 90, np.uint8)
    save_frame(Frame(img, T0 + 2.0011, T0 + 2.0461, {"exposure_us": 5000.0}), tmp_path / "f0.png")
    save_frame(Frame(img, T0 + 3.0011, T0 + 3.0031, {}), tmp_path / "f1.png")       # 2 ms: nearest samples
    cam = FileCamera(tmp_path)
    cam.open()
    a = capture_shot(link, cam, retries=0)
    b = capture_shot(link, cam, retries=0)
    times = [2.008, 2.016, 2.024, 2.032, 2.040]               # grid samples inside [2.0011, 2.0461] (after T0)
    assert a.match == "window" and a.n_samples == len(times) and b.match == "nearest" and b.n_samples == 2
    assert g.pose_delta(a.T_base_flange, g.transl(1.0 * np.mean(times), 0, 0) @ T_LOOK)[0] < 1e-6
    assert a.frame.meta["replayed"] and a.dataset_fields()["frame_meta"]["exposure_us"] == 5000.0


def test_shot_dataset_roundtrip(T_ft, tmp_path):
    w = World(still(T_LOOK, q=np.arange(6) * 0.1), t=T0 + 1.0)
    shot = capture_shot(FakeLink(w, T_ft), StubCamera(w), retries=0)
    ds = Dataset.create(tmp_path / "m", "measure")
    s = ds.add(shot.frame.image, **shot.dataset_fields(), pose_name="x")
    back = Dataset.load(tmp_path / "m")
    assert np.allclose(Dataset.T_base_flange(back.samples[0]), T_LOOK) and s["q_rad"] == pytest.approx(
        list(np.arange(6) * 0.1))
    assert back.samples[0]["n_samples"] == shot.n_samples and back.samples[0]["match"] == "window"


# ── look moves ────────────────────────────────────────────────────────────────
class BlockLink(FakeLink):
    def run_block(self, body, name="b", timeout_s=60.0, settle_s=0.0):
        self.blocks.append(body)
        return BlockResult(ok=True, block_id=len(self.blocks), name=name)


def test_goto_look_payload_and_body(cfg, T_ft, caplog):
    assert float(cfg["ur"]["payload_tool_kg"]) == 0.0        # PLACEHOLDER in station.toml (test relies on it)
    w = World(still(T_LOOK, q=np.array(ch.Q_REF)), t=T0 + 1.0)
    link = BlockLink(w, T_ft)
    with pytest.raises(CaptureError, match="payload_tool_kg is 0"):
        goto_look(link, cfg, T_base_flange=T_LOOK)
    assert link.blocks == []                                  # nothing sent
    capture._SIM_PAYLOAD_WARNED.clear()
    with caplog.at_level(logging.WARNING, logger="mauer.capture"):
        r = goto_look(link, cfg, T_base_flange=T_LOOK, sim=True)
        goto_look(link, cfg, T_base_flange=T_LOOK, sim=True)
    assert r.ok and caplog.text.count("SIMULATION payload fallback") == 1          # logged once, not per block
    body = link.blocks[-1]
    assert script.set_tcp(T_ft) in body and "set_payload(1.0, [0.0, 0.0, 0.06])" in body
    assert f"look = {script.pose(T_LOOK @ T_ft)}" in body                         # flange target -> TCP pose
    assert f"qnear={script.q_list(ch.Q_REF)}" in body                           # default qnear = current joints
    assert "get_inverse_kin_has_solution(look" in body and "v=0.5" in body      # [ur] v_joint
    cfg2 = {**cfg, "ur": {**cfg["ur"], "payload_tool_kg": 2.5}}
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="mauer.capture"):
        goto_look(link, cfg2, q_rad=[0.1] * 6, speeds=ch.URSIM_SPEEDS)
    assert "fallback" not in caplog.text
    assert "set_payload(2.5" in link.blocks[-1] and "movej([0.1" in link.blocks[-1] and "v=1.0" in link.blocks[-1]
    with pytest.raises(ValueError):
        goto_look(link, cfg2, T_base_flange=T_LOOK, q_rad=[0.0] * 6)


# ── tools: nominal kinematics, plan, poses JSON ──────────────────────────────
def test_ur5_ik_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(100):
        q = rng.uniform(-np.pi, np.pi, 6)
        sols = ch.ur5_ik(ch.ur5_fk(q)[-1])
        assert 1 <= len(sols) <= 8
        assert any(np.allclose([ch._wrap(a) for a in s - q], 0.0, atol=1e-6) for s in sols)
    # HOME branch is chosen for a pose reached from HOME
    home = np.array(ch.Q_REF)
    q = ch.choose_branch(ch.ur5_ik(ch.ur5_fk(home)[-1]), ch.q_reference(ch.ur5_fk(home)[-1][:2, 3]))
    assert np.allclose(q, home, atol=1e-9)
    assert ch.ur5_ik(g.transl(0.0, 0.0, 300.0) @ g.rotx(np.pi)) == []            # wrist centre on the base axis


def test_plan_and_poses_json(cfg, tmp_path):
    plan = ch.make_plan(cfg, 25, seed=0)
    again = ch.make_plan(cfg, 25, seed=0)
    assert all(np.array_equal(a.T_base_flange, b.T_base_flange) for a, b in zip(plan, again))
    for p in plan:
        assert 290.0 - 1e-6 <= p.dist_mm <= 355.0 + 1e-6 and 15.0 - 1e-6 <= p.tilt_deg <= 30.0 + 1e-6
        if p.qnear_rad is not None:
            assert g.pose_delta(ch.ur5_fk(p.qnear_rad)[-1], p.T_base_flange)[0] < 1e-6
    n_ik = sum(p.qnear_rad is not None for p in plan)
    assert n_ik == 23                     # current PLACEHOLDER board/mount: 2 poses put the wrist on the base axis
    path = ch.write_poses_json(tmp_path / "poses.json", plan[:3], "test")
    items = ch.load_poses_json(path, cfg)
    assert [i["name"] for i in items] == ["p00", "p01", "p02"]
    assert np.allclose(items[1]["T_base_flange"], plan[1].T_base_flange) and items[1]["q_rad"] is None
    (tmp_path / "alt.json").write_text('[{"name": "a", "xyz": [-300, 0, 400], "rpy_deg": [180, 0, 0]}, '
                                       '{"name": "b", "q_rad": [0, -1.57, 1.57, -1.57, -1.57, 0]}]')
    alt = ch.load_poses_json(tmp_path / "alt.json", cfg)
    assert alt[0]["qnear_rad"] is not None and alt[1]["T_base_flange"] is None


def test_plan_cli_and_refusals(cfg, tmp_path, capsys):
    out = tmp_path / "p.json"
    assert ch.main(["plan", "--n", "6", "--seed", "3", "--write", str(out)]) == 0
    assert len(ch.load_poses_json(out, cfg)) == 6
    # payload 0 (unknown) -> refused before connecting, no dataset created
    ds = tmp_path / "he"
    no_payload = config_file_without_payload(tmp_path)
    assert ch.main(["--config", no_payload, "capture", "--dataset", str(ds), "--host", "192.0.2.1"]) == 2
    assert not ds.exists() and "refusing to move the real robot" in capsys.readouterr().out
    assert mt.main(["--config", no_payload, "poses", "--poses", str(out), "--host", "192.0.2.1", "--nominal-mount",
                    "--nominal-intrinsics"]) == 2
    no_host = copy.deepcopy(cfg)
    no_host["ur"]["host"] = ""                              # the config's host is set since 2026-10-06
    with pytest.raises(SystemExit, match="host"):
        ch.resolve_host(ch.build_parser().parse_args(["verify"]), no_host)
    assert ch.resolve_host(ch.build_parser().parse_args(["verify", "--ursim"]), cfg) == ch.URSIM_HOST


# ── tools: full capture -> solve loop against a fake rig ─────────────────────
class RigLink(FakeLink):
    """run_block 'moves' instantly (1 s of world time); targets without a nominal IK solution fail like the
    controller's IK guard (stopped without done marker, error code 1)."""

    def __init__(self, world, T_ft):
        super().__init__(world, T_ft)
        self.segments = [(T0, T_LOOK, np.array(ch.Q_REF))]
        world.motion = self.motion

    def motion(self, t):
        T, q = next((T, q) for t0, T, q in reversed(self.segments) if t0 <= t)
        return T, q, np.zeros(6)

    def info(self):
        return {"host": self.host}

    def run_block(self, body, name="b", timeout_s=60.0, settle_s=0.0):
        self.blocks.append(body)
        m = re.search(r"^look = (p\[[^\]]*\])", body, re.M)
        T_bf = g.ur_to_T([float(v) for v in m.group(1)[2:-1].split(",")]) @ g.inv(self.T_ft)
        sols = ch.ur5_ik(T_bf)
        if not sols:
            return BlockResult(ok=False, block_id=len(self.blocks), name=name, error_code=script.ERR_IK_UNREACHABLE,
                               error="program stopped without the done marker (error code 1)")
        self.world.t += 1.0
        self.segments.append((self.world.t - 0.9, T_bf, sols[0]))
        return BlockResult(ok=True, block_id=len(self.blocks), name=name, error_code=0)


def test_capture_loop_and_solve_offline(cfg, T_ft, tmp_path, capsys):
    import handeye_solve
    spec = targets.board_specs(cfg)["calib"]
    X_true = config.T_flange_cam_nominal(cfg) @ INJECT
    T_bb = ch.calib_board_in_base(cfg)
    w = World(None, t=T0 + 1.0)
    link = RigLink(w, T_ft)
    cam = SynthCamera(lambda: link.flange_T(), X_true, [(spec, T_bb)], intrinsics.nominal(cfg),
                      clock=w.ticker(0.03))
    cam.open()
    rig = ch.Rig(cfg, link, cam, sim=True, ursim=True, speeds=ch.URSIM_SPEEDS,
                 truth={"T_flange_cam": X_true, "T_base_board": {"calib": T_bb}, "intrinsics": "nominal"})
    plan = ch.make_plan(cfg, 25, seed=0)
    ds = Dataset.create(tmp_path / "he", "handeye", board="calib")
    out = ch.capture_poses(rig, plan, ds, spec, 0.0, 0.01, 0, 10.0)
    assert out["unreachable"] == ["p19", "p23"] and link.popups == 2 and not out["failed"]
    assert len(out["captured"]) == 23 == len(ds)
    assert all(s["corners"] >= 40 and s["match"] == "window" for s in ds.samples)
    assert "set_payload(1.0" in link.blocks[0]                 # sim payload fallback in every look block
    # resume: nothing left to do for the captured poses
    done = {s["pose_index"] for s in ds.samples}
    again = ch.capture_poses(rig, plan, ds, spec, 0.0, 0.01, 0, 10.0, done)
    assert again["captured"] == [] and len(again["skipped_done"]) == 23 and again["unreachable"] == ["p19", "p23"]
    he = tmp_path / "handeye.json"
    assert handeye_solve.main([str(ds.path), "--nominal-intrinsics", "--out", str(he)]) == 0
    from mauer.vision import handeye
    res = handeye.load(he)
    dp, da = g.pose_delta(X_true, res.T_flange_cam)
    assert dp < 0.05 and da < 0.01, (dp, da)                    # exact robot poses: only render/detection noise
    assert res.meta["holdout"]["n"] == 4 and res.meta["holdout"]["pos_max_mm"] < 0.05
    # consistency over new poses with the calibrated mount (the measure_target 'poses' computation)
    rig.camera.seed = 100
    items = [{"name": p.name, "T_base_flange": p.T_base_flange, "qnear_rad": p.qnear_rad}
             for p in ch.make_plan(cfg, 6, seed=1000, prefix="v")]
    meas = ch.measure_poses(rig, items, {"calib": spec}, intrinsics.nominal(cfg), res.T_flange_cam, 0.0, 0.01, 0,
                            10.0)
    Ts = [m["boards"]["calib"][1] for m in meas if m["ok"]]
    s = mt.summarize(Ts, T_bb)
    assert s["n"] >= 5 and s["pos_max_mm"] < 0.05 and s["truth_max_mm"] < 0.1
