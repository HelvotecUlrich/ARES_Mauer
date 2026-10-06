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
    """URLink stand-in: movel blocks translate the TCP, the joint angles stay at Q0 (R_base_flange from ur5_fk)."""

    def __init__(self):
        self.T_tcp = ur5_fk(Q0) @ T_FT
        self.blocks: list[str] = []

    def state(self):
        T = self.T_tcp.copy()
        return SimpleNamespace(actual_q=Q0.copy(), actual_qd=np.zeros(6), T_base_tcp_mm=lambda: T)

    def wait_until(self, pred, timeout_s):
        s = self.state()
        return s if pred(s) else None

    def run_block(self, body, name, timeout_s):
        self.blocks.append(body)
        p = re.search(r"movel\(p\[([^\]]*)\]", body).group(1)
        self.T_tcp = g.ur_to_T([float(v) for v in p.split(",")])
        return SimpleNamespace(ok=True, error="")

    def flange(self):
        return self.T_tcp @ g.inv(T_FT)

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
