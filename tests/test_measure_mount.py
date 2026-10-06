"""tools/measure_target.py mount: the camera-mount check of the first table test (2026-10-06) against a fake robot
(pure translations, joint angles fixed) and mauer.simcam.SynthCamera rendering the calib board flat on the table.
A camera mounted as [camera.mount] passes; an adapter turned on the flange by 90 or 180 deg is caught and named."""
import json
import re
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from mauer import REPO, config
from mauer import geometry as g
from mauer.simcam import SynthCamera
from mauer.simworld import ur5_fk
from mauer.ur import script
from mauer.vision import intrinsics
from mauer.vision.targets import board_specs

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import calib_handeye as ch  # noqa: E402
import measure_target as mt  # noqa: E402

Q0 = np.radians([-68.403, -90.443, -79.437, -101.303, 88.829, 22.829])  # the UR5 on the table, 2026-10-06
T_FT = g.transl(0.0, 0.0, 200.0)                                        # its pendant TCP (from FK vs RTDE)


class MountLink:
    """URLink stand-in: movel blocks move the TCP (joint angles stay at Q0 - R_base_flange from ur5_fk), set_tcp
    changes the active TCP and stays (PolyScope 3.3, probe 2026-10-06); no tcp_offset field, so flange_T only works
    after this link sent a set_tcp."""

    def __init__(self):
        self.F = ur5_fk(Q0)                       # flange
        self.T_ft = T_FT                          # active TCP: the pendant's
        self.tracked = None
        self.blocks: list[str] = []

    @property
    def T_tcp(self):
        return self.F @ self.T_ft

    def state(self):
        T = self.T_tcp.copy()
        return SimpleNamespace(actual_q=Q0.copy(), actual_qd=np.zeros(6), T_base_tcp_mm=lambda: T)

    def wait_until(self, pred, timeout_s):
        s = self.state()
        return s if pred(s) else None

    def run_block(self, body, name, timeout_s):
        script.block_program(name, body, 1, 20, 21, 22)          # the real link validates name + body first
        self.blocks.append(body)
        m = re.search(r"set_tcp\(p\[([^\]]*)\]", body)
        if m:
            self.T_ft = self.tracked = g.ur_to_T([float(v) for v in m.group(1).split(",")])
        m = re.search(r"movel\(p\[([^\]]*)\]", body)
        if m:
            self.F = g.ur_to_T([float(v) for v in m.group(1).split(",")]) @ g.inv(self.T_ft)
        return SimpleNamespace(ok=True, error="")

    def flange(self):
        return self.F.copy()

    tcp_source = "tracked set_tcp"

    def flange_T(self, s=None):
        if self.tracked is None:                  # PolyScope 3.3 before this link sent a set_tcp
            raise ValueError("sample has no tcp_offset (RTDE field missing): pass T_flange_tcp")
        return (s or self.state()).T_base_tcp_mm() @ g.inv(self.tracked)

    def info(self):
        return {"host": "fake"}

    def abort(self):
        return "aborted"

    def stop(self):
        pass


def fake_rig(cfg, monkeypatch, T_flange_cam_true):
    """Board flat on the table (face up), centred 400 mm below the TRUE camera; open_rig patched to return the rig."""
    spec = board_specs(cfg)["calib"]
    link = MountLink()
    c = (link.flange() @ T_flange_cam_true)[:3, 3]
    w, h = spec.squares_x * spec.square_mm, spec.squares_y * spec.square_mm
    T_board = g.transl(c[0], c[1], c[2] - 400.0) @ g.rotx(np.pi) @ g.transl(-w / 2, -h / 2, 0.0)
    link.T_board = T_board
    cam = SynthCamera(link.flange, T_flange_cam_true, [(spec, T_board)], intrinsics.nominal(cfg), supersample=1)
    cam.open()
    rig = ch.Rig(cfg=cfg, link=link, camera=cam, sim=True, ursim=False, speeds=script.Speeds.from_config(cfg))
    monkeypatch.setattr(ch, "open_rig", lambda args, cfg_: rig)
    return link


def run_mount(cfg, monkeypatch, tmp_path, T_flange_cam_true):
    """The mount check with --yes; returns (exit code, report, link)."""
    link = fake_rig(cfg, monkeypatch, T_flange_cam_true)
    rep = tmp_path / "mount.json"
    rc = mt.main(["mount", "--boards", "calib", "--nominal-intrinsics", "--yes", "--settle-s", "0",
                  "--report", str(rep)])
    return rc, json.loads(rep.read_text()), link


def test_rotation_from_pairs():
    R = g.rotz(0.3)[:3, :3] @ g.rotx(-1.1)[:3, :3]
    a = [np.array([10.0, 0, 0]), np.array([0, 10.0, 0])]
    assert np.allclose(mt.rotation_from_pairs(a, [R @ v for v in a]), R, atol=1e-9)
    assert mt.axis_name([0.02, -0.999, 0.0]).startswith("-y")


def test_mount_as_configured_passes(monkeypatch, tmp_path, capsys):
    cfg = config.load()
    X = config.T_flange_cam_nominal(cfg)
    rc, rep, link = run_mount(cfg, monkeypatch, tmp_path, X)
    out = capsys.readouterr().out
    assert rc == 0 and "MOUNT OK" in out, out
    assert rep["verdict"]["angle_deg"] < 1.0
    assert rep["verdict"]["image_down"].startswith("+y") and rep["verdict"]["optical_axis"].startswith("+z")
    assert rep["tilt_nominal_deg"] < 1.0                                  # flat board, optical axis = flange z
    assert len(link.blocks) == 4 and np.allclose(link.T_tcp, ur5_fk(Q0) @ T_FT)   # back at the start
    for b in link.blocks:                                                 # neither TCP nor payload is touched
        assert "set_tcp" not in b and "set_payload" not in b and "a=0.2, v=0.02" in b


@pytest.mark.parametrize("turn_deg, right", [(90.0, "+y"), (180.0, "-x")])
def test_turned_adapter_is_caught(monkeypatch, tmp_path, capsys, turn_deg, right):
    cfg = config.load()
    X_true = g.rotz(np.radians(turn_deg)) @ config.T_flange_cam_nominal(cfg)    # adapter turned on the flange
    rc, rep, _ = run_mount(cfg, monkeypatch, tmp_path, X_true)
    out = capsys.readouterr().out
    assert rc == 3 and "MOUNT DIFFERS" in out, out
    assert rep["verdict"]["angle_deg"] == pytest.approx(turn_deg, abs=1.0)
    assert rep["verdict"]["image_right"].startswith(right)


def test_no_motion_without_move(monkeypatch, tmp_path, capsys):
    cfg = config.load()
    link = fake_rig(cfg, monkeypatch, config.T_flange_cam_nominal(cfg))
    assert mt.main(["mount", "--boards", "calib", "--nominal-intrinsics", "--move-mm", "0", "--settle-s", "0"]) == 0
    assert link.blocks == [] and "using calib" in capsys.readouterr().out


def test_locate_and_plan_around_a_table_board(monkeypatch, tmp_path, capsys):
    """calib_handeye locate (no motion) finds the board on the table; --board-pose aims the plan at it."""
    cfg = config.load()
    link = fake_rig(cfg, monkeypatch, config.T_flange_cam_nominal(cfg))
    out = tmp_path / "board.json"
    assert ch.main(["locate", "--nominal-intrinsics", "--write", str(out)]) == 0
    assert "nominal UR5 DH" in capsys.readouterr().out and link.blocks == []
    T = np.asarray(json.loads(out.read_text())["T_base_board"])
    dt, dr = g.pose_delta(T, link.T_board)
    assert dt < 1.0 and dr < 0.2, (dt, dr)
    c2 = ch.apply_board_pose(config.load(), out)
    assert np.allclose(ch.calib_board_in_base(c2), T) and ch.deck_z_in_base(c2) == pytest.approx(T[2, 3])
    poses = ch.make_plan(c2, 8, seed=0)
    assert all(289.0 <= p.dist_mm <= 356.0 and 14.0 <= p.tilt_deg <= 31.0 for p in poses), \
        [(p.dist_mm, p.tilt_deg) for p in poses]
    assert ch.make_plan(config.load(), 8, seed=0)[0].T_base_flange.tolist() != poses[0].T_base_flange.tolist()


def _table_cfg(link):
    c = config.load()
    c["vision"]["handeye_plan"]["board_pose"] = {"T_base_board": link.T_board.tolist(), "file": "test"}
    return c


def test_plan_stays_in_the_robots_arm_configuration(monkeypatch):
    """2026-10-06: the deck convention (elbow up, base turned to the board) put every look pose in the other branch
    than the robot on the table (q1 -68 deg, q3 -79 deg) - the first move would have swept the base ~180 deg."""
    cfg = config.load()
    link = fake_rig(cfg, monkeypatch, config.T_flange_cam_nominal(cfg))
    c = _table_cfg(link)
    deck = ch.make_plan(c, 10, seed=0)
    msg = ch.start_jump_refusal(Q0, deck)
    assert msg and "--q-ref-deg=-68.4,-90.4,-79.4" in msg
    table = ch.make_plan(c, 10, seed=0, q_ref=Q0)
    assert all(p.qnear_rad is not None for p in table) and ch.start_jump_refusal(Q0, table) is None
    for p in table:                                   # same branch: base, shoulder, elbow stay near the start
        assert np.max(np.abs(np.degrees(p.qnear_rad[:3] - Q0[:3]))) < 60.0, (p.name, np.degrees(p.qnear_rad))
        assert np.allclose(ur5_fk(p.qnear_rad), p.T_base_flange, atol=1e-3)
        assert abs(np.degrees(p.qnear_rad[5] - Q0[5])) < 110.0, (p.name, np.degrees(p.qnear_rad))   # cables
    assert ch.plan_settings(c, 10, 0, Q0)["q_ref_deg"] == pytest.approx(np.degrees(Q0).round(3).tolist())


def test_capture_refuses_an_arm_reconfiguration(monkeypatch, tmp_path, capsys):
    cfg = config.load()
    link = fake_rig(cfg, monkeypatch, config.T_flange_cam_nominal(cfg))
    bp = tmp_path / "board.json"
    bp.write_text(json.dumps({"T_base_board": link.T_board.tolist()}))
    assert ch.main(["capture", "--dataset", str(tmp_path / "he"), "--board-pose", str(bp), "--n", "6"]) == 2
    assert "refusing to move" in capsys.readouterr().out and link.blocks == []


class _Shot:
    """capture_shot stand-in: the image now and the true flange pose (the fake arm stands still)."""

    def __init__(self, link, camera):
        self.frame = camera.grab()
        self.T_base_flange = link.flange()

    def dataset_fields(self):
        return {"t_start": self.frame.t_start, "t_end": self.frame.t_end, "T_base_flange": self.T_base_flange,
                "q_rad": Q0.tolist(), "tcp_pose_ur": None, "max_qd": 0.0}


def test_orbit_relative_to_the_start_and_solve(monkeypatch, tmp_path, capsys):
    """orbit (2026-10-06, the simple way on the table): views relative to the start pose, straight-line moves only,
    back to the start; the hand-eye solve on its images recovers a camera mounted 2-3 mm / <1 deg off the design."""
    from mauer.vision import handeye
    cfg = config.load()
    X_true = config.T_flange_cam_nominal(cfg) @ g.pose_xyz_rpy([2.0, -3.0, 1.0], [0.5, -0.3, 0.8])
    link = fake_rig(cfg, monkeypatch, X_true)
    monkeypatch.setattr(ch, "capture_shot", lambda lk, cam, **kw: _Shot(lk, cam))
    start = link.T_tcp.copy()
    ds = tmp_path / "he"
    assert ch.main(["orbit", "--dataset", str(ds), "--yes", "--settle-s", "0"]) == 0, capsys.readouterr().out
    out = capsys.readouterr().out
    assert "25 views kept" in out and "25 images" in out, out
    assert all("movej" not in b and "get_inverse_kin" not in b and "set_tcp" in b for b in link.blocks)
    assert g.pose_delta(link.F @ config.T_flange_tcp(cfg), start @ g.inv(T_FT) @ config.T_flange_tcp(cfg))[0] < 1e-6
    he = tmp_path / "he.json"
    assert ch.main(["solve", str(ds), "--nominal-intrinsics", "--out", str(he)]) == 0
    dt, dr = g.pose_delta(handeye.load(he).T_flange_cam, X_true)
    assert dt < 1.0 and dr < 0.1, (dt, dr)


def test_orbit_views_are_small_and_snake():
    v = ch.orbit_views()
    assert len(v) == 25 and v[0] == (0.0, 0.0, 0.0, 0.0)
    assert max(t for _, t, _, _ in v) == 25.0 and max(abs(r) for _, _, r, _ in v) == ch.ORBIT_ROLL_DEG
    rolls = [r for _, _, r, _ in v[1:]]
    assert all(abs(a - b) <= ch.ORBIT_ROLL_DEG for a, b in zip(rolls, rolls[1:]))     # no -40 -> +40 jumps
