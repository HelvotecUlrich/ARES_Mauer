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
    n = int(re.search(r"(\d+) views kept", out).group(1))
    assert 20 <= n <= 25 and f"{n} images" in out, out           # this start is the touching one of 2026-10-06:
    dropped = re.search(r"dropped: (.*)", out)                    # its dropped views are tool-vs-arm ones
    assert dropped is None or all(" vs " in d for d in dropped.group(1).split("; ")), out
    assert all("movej" not in b and "get_inverse_kin" not in b and "set_tcp" in b for b in link.blocks)
    assert g.pose_delta(link.F @ config.T_flange_tcp(cfg), start @ g.inv(T_FT) @ config.T_flange_tcp(cfg))[0] < 1e-6
    he = tmp_path / "he.json"
    assert ch.main(["solve", str(ds), "--nominal-intrinsics", "--out", str(he)]) == 0
    dt, dr = g.pose_delta(handeye.load(he).T_flange_cam, X_true)
    assert dt < 1.0 and dr < 0.1, (dt, dr)
    import handeye_solve                                       # --check: validate the saved calibration, no solve
    capsys.readouterr()
    assert handeye_solve.main([str(ds), "--nominal-intrinsics", "--check", str(he)]) == 0
    out = capsys.readouterr().out
    assert float(re.search(r"mean of this dataset vs the calibration board pose: ([0-9.]+) mm", out).group(1)) < 0.5


def test_orbit_views_are_small_and_snake():
    v = ch.orbit_views()
    assert len(v) == 25 and v[0] == (0.0, 0.0, 0.0, 0.0)
    assert max(t for _, t, _, _ in v) == 25.0 and max(abs(r) for _, _, r, _ in v) == ch.ORBIT_ROLL_DEG
    rolls = [r for _, _, r, _ in v[1:]]
    assert all(abs(a - b) <= ch.ORBIT_ROLL_DEG for a, b in zip(rolls, rolls[1:]))     # no -40 -> +40 jumps


def test_orbit_drops_the_view_that_touched_the_arm():
    """2026-10-06, first orbit on the table: the camera adapter touched wrist 1 (end of the forearm) on the way to o18
    - the user hit the e-stop; o00..o17 ran clear. With the recorded start the self-collision check drops o18 and keeps
    everything before it."""
    from mauer import armcheck
    cfg = config.load()
    q0 = np.radians([-69.3583, -88.6686, -97.7776, -83.5734, 90.0611, 20.7653])     # data/he_2026-10-06 meta
    spec = board_specs(cfg)["calib"]
    T_cam_board0 = g.transl(-spec.centre_mm[0], -spec.centre_mm[1], 346.86)            # board centre on the axis
    kept, dropped, geo = ch.orbit_plan(cfg, ur5_fk(q0), q0, T_cam_board0, spec, ares=False)    # on the lab table
    names = [v["name"] for v in kept]
    assert names[:18] == [f"o{i:02d}" for i in range(18)], names
    assert "o18" not in names and any(d.startswith("o18: camera adapter vs forearm") for d in dropped), dropped
    assert all(v["self_mm"] >= armcheck.SELF_CLEARANCE_MM for v in kept)
    assert armcheck.self_clearance(q0, cfg)[0] > armcheck.SELF_CLEARANCE_MM


def test_self_clearance_sees_the_camera_folded_onto_the_forearm():
    from mauer import armcheck
    cfg = config.load()
    q = np.radians([-69.0, -89.0, -98.0, -84.0, 90.0, 21.0])
    ok = armcheck.self_clearance(q, cfg)[0]
    worst = min(armcheck.self_clearance(np.r_[q[:5], q[5] + np.radians(a)], cfg)[0] for a in range(0, 360, 10))
    assert ok > 15.0 and worst < ok                              # turning wrist 3 brings the adapter to the arm


def _deck_start(cfg, x_ares: float, y_ares: float, dist_mm: float = 320.0):
    """Camera looking straight down at a calib board centred at (x, y) on the ARES deck from dist_mm: (q0, T_cam_board0,
    spec). The board frame: z into the board (down), as OpenCV."""
    from mauer import armcheck
    from mauer.simworld import ik_near
    spec = board_specs(cfg)["calib"]
    X = config.T_flange_cam_nominal(cfg)
    T_ab = config.T_ares_base(cfg)
    deck = float(cfg["ares"]["deck_top_z"]) + 4.0
    park = np.asarray(np.radians(cfg["ur"]["park_q_deg"]), float)
    best = None
    for yaw in range(0, 360, 15):                 # the camera's yaw about the vertical with the most tool clearance
        T_ares_cam = g.transl(x_ares, y_ares, deck + dist_mm) @ g.rotx(np.pi) @ g.rotz(np.radians(yaw))  # axis down
        q = ik_near(g.inv(T_ab) @ T_ares_cam @ g.inv(X), park)
        if q is not None and (best is None or armcheck.self_clearance(q, cfg)[0] > best[0]):
            best = (armcheck.self_clearance(q, cfg)[0], q)
    assert best is not None and best[0] > 40.0, best
    q0 = best[1]
    T_cam_board0 = g.transl(-spec.centre_mm[0], -spec.centre_mm[1], dist_mm)
    return q0, T_cam_board0, spec


def test_orbit_on_ares_checks_the_views_against_ares_and_the_controller():
    """Calibration on ARES (T4, 2026-10-09): every view and every straight-line move is checked against the ARES
    boxes (chassis, UR control box at the far end, magazine empty) - a box next to the board drops the views that
    tilt into it. Found on the way: 320 mm over the deck the 25 deg tilt o20 brings a gripper jaw to 9 mm from the
    deck (the old check saw camera, TCP and flange only) - dropped now."""
    from mauer import armcheck
    cfg = config.load()
    q0, T_cb, spec = _deck_start(cfg, -100.0, 0.0)
    kept, dropped, _ = ch.orbit_plan(cfg, ur5_fk(q0), q0, T_cb, spec)
    assert len(kept) >= 20, dropped
    assert any(d.startswith("o20: jaw") and "ARES chassis" in d for d in dropped), dropped
    no_ares, _, _ = ch.orbit_plan(cfg, ur5_fk(q0), q0, T_cb, spec, ares=False)
    assert "o20" in [v["name"] for v in no_ares]                     # the table check alone would have run it
    pillar = armcheck.Box("test pillar", np.array([-100.0, 230.0, 600.0]), np.eye(3), np.array([60.0, 60.0, 300.0]))
    boxes = armcheck.ares_boxes(cfg) + [pillar]
    kept2, dropped2, _ = ch.orbit_plan(cfg, ur5_fk(q0), q0, T_cb, spec, boxes=boxes)
    assert len(kept2) < len(kept) and any("test pillar" in d for d in dropped2), dropped2
    checker = armcheck.ArmChecker(cfg, boxes, on_ares=config.T_ares_base(cfg))
    assert all(checker.hits(v["q_nominal"], None) == [] for v in kept2)
