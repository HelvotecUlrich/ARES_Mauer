"""RoboDK simulated flange camera (robodk/sim_camera.py) and the board objects of robodk/build_station.py.

Opens its OWN RoboDK instance on port 20598 (rdk_common.connect(new_instance=True)) and closes it at the end; the
user's RoboDK is never touched. Skipped without a RoboDK installation. Checks: the separate-instance guard, the
camera parameters from config/station.toml (snapshot shape, grayscale frame), a ChArUco board PNG rendered at a
known pose is detected by cv2.aruco.CharucoDetector with a pose error < 1 mm, and the camera on the robot tool
"Camera" (T_flange_cam) gives the board in the UR5 base frame within 1 mm.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.robodk


@pytest.fixture(autouse=True, scope="module")    # module scope: skips BEFORE the module fixture rdk
def _opt_in(request):                               # starts a RoboDK instance
    """Opt-in like the URSim tests: these start a separate RoboDK instance."""
    import os
    if "robodk" not in (request.config.getoption("markexpr") or "") and os.environ.get("MAUER_ROBODK") != "1":
        pytest.skip("RoboDK tests are opt-in: py.exe -m pytest -m robodk tests/test_robodk_camera.py")

REPO = Path(__file__).resolve().parent.parent
if not Path(r"C:\RoboDK\bin\RoboDK.exe").exists():
    pytest.skip("RoboDK not installed", allow_module_level=True)
cv2 = pytest.importorskip("cv2")
sys.path.insert(0, str(REPO / "robodk"))

import build_station as bs  # noqa: E402
import mauer.geometry as g  # noqa: E402
import rdk_common as rc  # noqa: E402
from sim_camera import SimCamera, intrinsics  # noqa: E402

PORT = 20598                      # not the study port (20599) and not the user's (20500/20501)


@pytest.fixture(scope="module")
def cfg():
    return rc.load_config()


def ref_spec(cfg) -> dict:
    return next(s for s in bs.board_layout(cfg) if s["parent"] == "wall")


@pytest.fixture(scope="module")
def rdk(cfg):
    RDK = rc.connect(new_instance=True, port=PORT)
    RDK.AddStation("test_robodk_camera")
    RDK.Render(True)              # cameras must be opened with rendering on
    # textured boards must exist before the first camera is opened in the instance (else they render black)
    RDK.test_board = bs.add_board(RDK, ref_spec(cfg), RDK.ActiveStation(), g.to_robodk(g.transl(0, 0, -5000)))
    yield RDK
    rc.close_instance(RDK)


def place(item, cfg, T_w_board: np.ndarray) -> None:
    """Move the board object so that its board frame is at T_w_board (quiet zone of one square, see add_board)."""
    sq = ref_spec(cfg)["square_mm"]
    item.setPose(g.to_robodk(T_w_board @ g.transl(-sq, -sq, 0.0)))


def look(d_mm: float, tilt_deg: float, spec: dict) -> np.ndarray:
    """Camera pose in the board frame: d mm from the board centre, optical axis tilted about the board x axis."""
    w, h = spec["squares_x"] * spec["square_mm"], spec["squares_y"] * spec["square_mm"]
    t = np.radians(tilt_deg)
    v = np.array([0.0, np.sin(t), np.cos(t)])               # viewing direction (board z = into the board)
    x = np.array([1.0, 0.0, 0.0])
    R = np.column_stack([x, np.cross(v, x), v])
    return g.make_T(R, np.array([w / 2, h / 2, 0.0]) - d_mm * v)


def board_pose(img: np.ndarray, spec: dict, K: np.ndarray, D: np.ndarray) -> tuple[np.ndarray, int]:
    board = bs.charuco_board(spec)
    cc, ci, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(img)
    assert ci is not None and len(ci) >= 6, "board not detected"
    obj, imgp = board.matchImagePoints(cc, ci)
    _, rv, tv = cv2.solvePnP(obj, imgp, K, D, flags=cv2.SOLVEPNP_IPPE)
    _, rv, tv = cv2.solvePnP(obj, imgp, K, D, rv, tv, True, cv2.SOLVEPNP_ITERATIVE)
    return g.make_T(cv2.Rodrigues(rv)[0], tv.ravel()), len(ci)


def test_guard_refuses_an_instance_it_did_not_start(rdk):
    with pytest.raises(RuntimeError, match="did not start"):
        rc.connect(new_instance=True, port=PORT)            # our fixture instance already listens there


def test_snapshot_shape_and_frame(rdk, cfg):
    f = rdk.AddFrame("cam_frame_shape")
    f.setPose(g.to_robodk(g.transl(0, 0, 500) @ g.rotx(np.pi)))
    with SimCamera(rdk, cfg, f) as cam:
        frame = cam.grab()
        info = cam.info()
    assert frame.image.shape == (cfg["camera"]["res_y"], cfg["camera"]["res_x"]) == (2064, 2472)
    assert frame.image.dtype == np.uint8
    assert frame.t_start <= frame.t_end
    K, D, w, h = intrinsics(cfg)
    assert K[0, 0] == pytest.approx(cfg["camera"]["focal"] / (cfg["camera"]["pixel_um"] * 1e-3))
    assert (K[0, 2], K[1, 2]) == ((w - 1) / 2, (h - 1) / 2)
    assert info["width"] == w and info["height"] == h
    f.Delete()


def _windows(pid: int, prefix: str) -> list:
    """Visible top-level windows of process pid whose title starts with prefix (Win32)."""
    import ctypes
    from ctypes import wintypes
    u32 = ctypes.windll.user32
    out = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def cb(h, _):
        p = wintypes.DWORD()
        u32.GetWindowThreadProcessId(h, ctypes.byref(p))
        if p.value == pid and u32.IsWindowVisible(h):
            n = u32.GetWindowTextLengthW(h)
            buf = ctypes.create_unicode_buffer(n + 1)
            u32.GetWindowTextW(h, buf, n + 1)
            if buf.value.startswith(prefix):
                out.append(h)
        return True
    u32.EnumWindows(cb, 0)
    return out


def _restore_and_close(rdk, title: str) -> None:
    """What the user did on 2026-10-06: restore the camera window and close it with its X (Win32 messages)."""
    import ctypes
    import time
    wins = _windows(rdk.NEW_INSTANCE.pid, title)
    assert wins, f"camera window {title!r} not found"
    for h in wins:
        ctypes.windll.user32.ShowWindow(h, 9)                  # SW_RESTORE
    time.sleep(1.0)
    for h in wins:
        ctypes.windll.user32.PostMessageW(h, 0x0010, 0, 0)     # WM_CLOSE
    time.sleep(1.5)


def test_snapshot_survives_a_closed_camera_window(rdk, cfg):
    """A closed camera window made RoboDK return 160 x 133 snapshots (run 2026-10-06 aborted with 'bad snapshot:
    (133, 160, 3)'). The window now opens MINIMIZED, where closing it does no harm; a camera opened the old way
    (normal window) reproduces the failure and grab re-opens it."""
    f = rdk.AddFrame("cam_frame_close")
    f.setPose(g.to_robodk(g.transl(0, 0, 500) @ g.rotx(np.pi)))
    with SimCamera(rdk, cfg, f) as cam:
        _restore_and_close(rdk, "cam_frame_close - Size")
        assert cam._snapshot() is not None                     # minimised window: closing it is harmless
    f2 = rdk.AddFrame("cam_frame_close2")
    f2.setPose(f.Pose())
    cam = SimCamera(rdk, cfg, f2)
    cam._params = lambda flat: rc.camera_params(cfg, True, flat, cam.far_mm)     # old: normal window
    cam.open()
    try:
        _restore_and_close(rdk, "cam_frame_close2 - Size")
        assert cam._snapshot() is None                         # the failure is reproduced: a shrunken snapshot
        assert cam._last_shape is not None and cam._last_shape[:2] != (2064, 2472)  # (160 x 133 / 144 x 120 seen)
        del cam._params                                         # re-open with the module's parameters
        assert cam.grab().image.shape == (2064, 2472)
    finally:
        cam.close()
    f.Delete()
    f2.Delete()


def test_board_png_pose_error_below_1mm(rdk, cfg):
    spec = ref_spec(cfg)
    T_w_board = g.transl(100.0, -50.0, 3.0) @ g.rotz(np.radians(20)) @ g.rotx(np.pi)   # face up, turned
    place(rdk.test_board, cfg, T_w_board)
    K, D, _, _ = intrinsics(cfg)
    f = rdk.AddFrame("cam_frame_board")
    with SimCamera(rdk, cfg, f) as cam:
        for d, tilt in ((320.0, 0.0), (300.0, 25.0)):
            T_board_cam = look(d, tilt, spec)
            f.setPose(g.to_robodk(T_w_board @ T_board_cam))
            T_cb, n = board_pose(cam.grab().image, spec, K, D)
            dt, dr = g.pose_delta(T_cb, g.inv(T_board_cam))
            assert n == (spec["squares_x"] - 1) * (spec["squares_y"] - 1)
            assert dt < 1.0, f"d={d} tilt={tilt}: {dt:.3f} mm"
            assert dr < 0.2, f"d={d} tilt={tilt}: {dr:.3f} deg"
    f.Delete()


def test_flange_camera_gives_board_in_base_frame(rdk, cfg):
    robot_file, tool_file = REPO / cfg["ur5"]["robot_file"], REPO / cfg["tool"]["tool_file"]
    if not (robot_file.exists() and tool_file.exists()):
        pytest.skip("UR5 library / gripper tool file not present (not versioned)")
    robot = rdk.AddFile(str(robot_file))
    robot.Parent().setPose(g.to_robodk(np.eye(4)))           # the library file carries a base offset
    tool = rdk.AddFile(str(tool_file), robot)
    tool.setPoseTool(rc.tcp_pose(cfg))
    robot.setPoseTool(tool)
    cam_tool, body = bs.add_camera(rdk, cfg, robot, tool)
    T_fc = g.from_robodk(rc.T_flange_cam(cfg))
    spec = ref_spec(cfg)
    T_w_base = g.from_robodk(robot.Parent().PoseAbs())
    T_w_board = g.transl(450.0, 150.0, -200.0) @ g.rotx(np.pi)                          # face up below the arm
    place(rdk.test_board, cfg, T_w_board)
    T_base_cam = g.inv(T_w_base) @ T_w_board @ look(320.0, 15.0, spec)
    j = robot.SolveIK(g.to_robodk(T_base_cam), [0, -90, 90, -90, -90, 0], tool=g.to_robodk(T_fc)).list()
    assert len(j) >= 6, "no IK for the test look pose"
    robot.setJoints(j[:6])
    K, D, _, _ = intrinsics(cfg)
    with SimCamera(rdk, cfg, cam_tool, robot=robot) as cam:
        assert cam.hide == [body]                              # lens hidden in its own view
        frame = cam.grab()
    T_cb, _ = board_pose(frame.image, spec, K, D)
    T_base_board = np.asarray(frame.meta["T_base_flange"]) @ T_fc @ T_cb
    dt, dr = g.pose_delta(T_base_board, g.inv(T_w_base) @ T_w_board)
    assert dt < 1.0, f"{dt:.3f} mm"
    assert dr < 0.2, f"{dr:.3f} deg"
    assert body.Visible()                                      # shown again after the snapshot
    robot.Delete()
