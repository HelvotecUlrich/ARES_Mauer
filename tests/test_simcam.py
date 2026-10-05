import numpy as np

from mauer import config, geometry as g
from mauer.simcam import SynthCamera
from mauer.vision import detect, intrinsics, targets


def test_synthcamera_board_pose_matches_truth():
    cfg = config.load()
    specs = targets.board_specs(cfg)
    spec = specs["W0"]
    intr = intrinsics.nominal(cfg)
    T_flange_cam = g.pose_xyz_rpy([-150, 0, 60], [0, 0, 0])
    # board face up 320 mm below the camera: board z (into the board) points down = along the camera z
    T_base_cam = g.pose_xyz_rpy([400, 100, 700], [180, 0, 0])
    c = np.array([*spec.centre_mm, 0.0])
    T_base_board = g.transl(*(T_base_cam[:3, 3] + [0, 0, -320])) @ g.rotx(np.pi) @ g.transl(*-c)
    T_base_flange = T_base_cam @ g.inv(T_flange_cam)
    cam = SynthCamera(lambda: T_base_flange, T_flange_cam, [(spec, T_base_board)], intr, noise_sigma=0.5)
    with cam:
        fr = cam.grab()
    assert fr.image.shape == (intr.height, intr.width) and fr.meta["boards_rendered"] == ["W0"]
    pose = detect.measure(fr.image, [spec], intr, cfg["vision"])["W0"]
    assert pose.ok, pose.reason
    d, a = g.pose_delta(T_base_cam @ pose.T_cam_board, T_base_board)
    assert d < 0.2 and a < 0.1


def test_synthcamera_skips_back_facing_and_behind():
    cfg = config.load()
    spec = targets.board_specs(cfg)["W1"]
    intr = intrinsics.nominal(cfg)
    T_base_cam = g.pose_xyz_rpy([0, 0, 0], [180, 0, 0])            # looking down (-z of base)
    above = g.transl(-40, -32, 300)                                  # board above the camera: behind it
    facing_away = g.transl(-40, -32, -320)                           # board z = base z points towards the camera
    cam = SynthCamera(lambda: T_base_cam, np.eye(4), [(spec, above), (spec, facing_away)], intr)
    assert cam.visible(T_base_cam) == []
