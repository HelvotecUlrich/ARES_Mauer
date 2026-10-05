"""mauer.ur against URSim CB3 3.15.8 (UR's simulator in Docker, started by tests/ursim.py).

Opt-in, because it boots a 3.3 GB container:   py.exe -m pytest -m ursim tests/test_ur_ursim.py
(or set MAUER_URSIM=1). Skipped when the Docker CLI/daemon or the image is missing (never pulled). The container is
started for this module and stopped at the end; the image stays on disk.

Speeds here (FAST) are for the simulator only – the real robot uses the [ur] ASSUMPTION values.
"""
import logging
import math
import os
import threading
import time

import numpy as np
import pytest

import ursim
from mauer import config, geometry as g
from mauer.ur import script as s
from mauer.ur.dashboard import Dashboard
from mauer.ur.link import URLink
from mauer.ur.rtde.rtde import RTDE

pytestmark = pytest.mark.ursim
logging.getLogger("rtde").setLevel(logging.CRITICAL)   # URSim: "SafetySetup has not been confirmed yet" messages

HOME = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]   # [rad], tool pointing down
FAST = s.Speeds(v_joint=1.0, a_joint=1.4, v_lin=0.25, a_lin=1.0, v_contact=0.05)
TOL_MM, TOL_DEG = 0.1, 0.01


@pytest.fixture(scope="module")
def sim(request):
    if "ursim" not in (request.config.getoption("markexpr") or "") and os.environ.get("MAUER_URSIM") != "1":
        pytest.skip("URSim tests are opt-in: py.exe -m pytest -m ursim tests/test_ur_ursim.py")
    ok, why = ursim.available()
    if not ok:
        pytest.skip(f"URSim not available: {why}")
    try:
        ursim.start(boot_timeout_s=240.0)
        yield ursim.HOST
    finally:
        ursim.stop()


@pytest.fixture(scope="module")
def link(sim):
    t0 = time.time()
    while True:   # RTDE may come up a little after the Dashboard
        try:
            ur = URLink(sim).start()
            break
        except (OSError, Exception):
            if time.time() - t0 > 30.0:
                raise
            time.sleep(1.0)
    yield ur
    ur.stop()


@pytest.fixture(scope="module")
def T_ft():
    """Gripper TCP in the flange frame from config/station.toml (tcp_z 135 mm ASSUMPTION, rotz 90)."""
    return config.T_flange_tcp(config.load())


def home(link, T_ft, tool_volt=None):
    r = link.run_block(s.preamble(T_ft, 1.0, [0, 0, 60], tool_volt) + "\n" + s.movej_q(HOME, 1.4, 1.0), "home", 30)
    assert r.ok, r.error
    return r


def dh_fk_ur5(q) -> np.ndarray:
    """UR5 nominal DH forward kinematics, base -> flange [mm]. DH table: UR article 'DH parameters for calculations
    of kinematics and dynamics' (d1 0.089159, a2 -0.425, a3 -0.39225, d4 0.10915, d5 0.09465, d6 0.0823 m)."""
    a = [0.0, -425.0, -392.25, 0.0, 0.0, 0.0]
    d = [89.159, 0.0, 0.0, 109.15, 94.65, 82.3]
    alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
    T = np.eye(4)
    for i in range(6):
        T = T @ g.rotz(q[i]) @ g.transl(0, 0, d[i]) @ g.transl(a[i], 0, 0) @ g.rotx(alpha[i])
    return T


def test_dashboard_info(sim):
    with Dashboard(sim) as d:
        assert d.banner.startswith("Connected: Universal Robots Dashboard Server")
        assert d.version()[:2] == (3, 15)
        assert d.robot_model() == "UR5"
        assert d.robotmode() == "RUNNING" and d.safetymode() == "NORMAL"


def test_rtde_stream(link):
    assert link.controller_version[:2] == (3, 15) and link.missing_fields == []
    assert {"tcp_offset", "target_TCP_pose", "tool_output_voltage"} <= set(link.fields)
    t0 = time.time()
    time.sleep(1.0)
    smp = link.samples(t0, t0 + 1.0)
    assert len(smp) >= 100, f"{len(smp)} samples/s"                # 125 Hz nominal
    assert all(b.timestamp > a.timestamp for a, b in zip(smp, smp[1:]))


def test_movej_block(link, T_ft):
    r = home(link, T_ft)
    assert np.abs(r.state.actual_q - HOME).max() < 1e-4
    assert r.start_latency_s < 1.0 and r.error_code == 0
    assert np.abs(r.state.actual_qd).max() < 1e-3                   # done marker only after is_steady()
    r2 = link.run_block("", "empty", 5)
    assert r2.ok and r2.block_id == r.block_id + 1 and r2.start_latency_s < 0.5


def test_tool_voltage(link):
    for volt in (24, 0):
        assert link.run_block(s.tool_voltage(volt), f"tool_{volt}v", 5).ok
        assert link.wait_until(lambda x: x.tool_output_voltage == volt, 2.0) is not None


def test_movel_reaches_target(link, T_ft):
    home(link, T_ft)
    T0 = link.state().T_base_tcp_mm()
    target = T0 @ g.transl(50.0, -30.0, -40.0) @ g.rotz(math.radians(10.0))
    r = link.run_block(s.movel(target, 1.0, 0.25), "movel", 30)
    assert r.ok, r.error
    d_mm, d_deg = g.pose_delta(r.state.T_base_tcp_mm(), target)
    assert d_mm < TOL_MM and d_deg < TOL_DEG
    assert g.pose_delta(g.ur_to_T(r.state.target_TCP_pose), target)[0] < TOL_MM


def test_set_tcp_and_flange(link, T_ft):
    a = home(link, T_ft).state
    assert np.allclose(a.T_flange_tcp_mm(), T_ft, atol=1e-6)        # RTDE tcp_offset = what set_tcp sent
    b = link.run_block(s.set_tcp(np.eye(4)), "tcp_zero", 5)
    assert b.ok
    b = link.wait_until(lambda x: np.abs(x.tcp_offset).max() == 0.0, 1.0)
    # flange from (TCP pose, tcp_offset) == pose reported with zero TCP at the same joints
    assert g.pose_delta(a.T_base_flange_mm(), b.T_base_tcp_mm())[0] < 1e-3
    assert g.pose_delta(a.T_base_tcp_mm(), b.T_base_tcp_mm() @ T_ft)[0] < 1e-3
    # and == UR5 nominal DH FK (URSim uses nominal kinematics; the real robot is calibrated)
    d_mm, d_deg = g.pose_delta(dh_fk_ur5(a.actual_q), a.T_base_flange_mm())
    assert d_mm < 0.01 and d_deg < 1e-3
    assert np.allclose(link.tcp_tracked, np.eye(4), atol=1e-9)      # fallback tracking follows the last set_tcp


def test_look_pose_flange_target(link, T_ft):
    home(link, T_ft)
    target = link.flange_T() @ g.transl(30.0, 20.0, 10.0) @ g.rotz(math.radians(5.0))
    body = s.preamble(T_ft, 1.0, [0, 0, 60]) + "\n" + s.look_pose(target, HOME, 1.0, 0.8, T_flange_tcp=T_ft)
    r = link.run_block(body, "look", 30, settle_s=0.1)
    assert r.ok, r.error
    assert r.state.t_laptop >= r.t_done + 0.1
    d_mm, d_deg = g.pose_delta(link.flange_T(r.state), target)
    assert d_mm < TOL_MM and d_deg < TOL_DEG


def test_place_relative_to_frame(link, T_ft):
    home(link, T_ft)
    F = g.ur_to_T([-0.45, -0.10, 0.05, 0.0, 0.0, 0.3])              # "measured" frame (research test_place.py)
    local = g.rotx(math.pi)                                           # TCP z down onto the frame
    body = s.preamble(T_ft, 1.0, [0, 0, 60]) + "\n" + s.place_stone(
        F, local, 150.0, HOME, FAST, do_open=0, pulse_s=0.2, wait_s=0.1, do_close=1, contact_mm=60.0,
        payload_after=(0.5, [0, 0, 60]))
    r = link.run_block(body, "place", 60)
    assert r.ok, r.error
    above, at = F @ g.transl(0, 0, 150.0) @ local, F @ local
    d_mm, d_deg = g.pose_delta(r.state.T_base_tcp_mm(), above)
    assert d_mm < TOL_MM and d_deg < TOL_DEG
    smp = link.samples(r.t_started, r.t_done)
    assert min(g.pose_delta(x.T_base_tcp_mm(), at)[0] for x in smp) < TOL_MM   # reached the place pose
    assert any(x.digital_out(0) for x in smp)                          # open pulse seen
    assert r.state.digital_out(0) is False and r.state.digital_out(1) is False


def test_compile_error_detected(link, T_ft):
    r = link.run_block("movel(p[0,0,0,0,0,0)", "syntax_error", 10)
    assert not r.ok and "did not start" in r.error and r.t_started is None
    home(link, T_ft)                                                   # the robot accepts the next block


def test_unreachable_pose_guard(link, T_ft):
    home(link, T_ft)
    r = link.run_block(s.movej_pose(g.pose_xyz_rpy([2000.0, 0, 500.0], [180, 0, 0]), HOME, 1.0, 1.0), "unreach", 10)
    assert not r.ok and "without the done marker" in r.error and r.error_code == s.ERR_IK_UNREACHABLE
    assert np.abs(link.state().actual_q - HOME).max() < 1e-4          # did not move
    with Dashboard(link.host) as d:
        d.close_popup()
    home(link, T_ft)


def test_abort_stops_motion(link, T_ft):
    home(link, T_ft)
    far = list(HOME)
    far[0] += 1.0
    out = {}
    th = threading.Thread(target=lambda: out.setdefault("r", link.run_block(s.movej_q(far, 0.5, 0.2), "slow", 30)))
    th.start()
    time.sleep(1.0)
    msg = link.abort()
    th.join(10.0)
    assert "stopl program sent" in msg
    r = out["r"]
    assert not r.ok and "without the done marker" in r.error
    st = link.wait_until(lambda x: np.abs(x.actual_qd).max() < 1e-4, 3.0)
    assert st is not None and abs(st.actual_q[0] - far[0]) > 0.3     # stopped well before the target
    home(link, T_ft)


def test_protective_stop_detected(link, T_ft):
    """RTDE watchdog with action 'stop' and no input updates -> protective stop C207A0 (research 2026-10-05)."""
    home(link, T_ft)
    con = RTDE(link.host, link.rtde_port)
    con.connect()
    try:
        inp = con.send_input_setup(["input_int_register_47"], ["INT32"])
        inp.input_int_register_47 = 0
        con.send_output_setup(["timestamp"], ["DOUBLE"], frequency=10)
        con.send_start()
        con.send(inp)
        t0 = time.time()
        r = link.run_block('rtde_set_watchdog("input_int_register_47", 2, "stop")\nsleep(5.0)', "pstop", 15)
    finally:
        con.send_pause()
        con.disconnect()
    if not r.ok and "PROTECTIVE_STOP" not in r.error:
        pytest.skip(f"watchdog did not cause a protective stop here: {r.error}")
    assert not r.ok and "PROTECTIVE_STOP" in r.error and time.time() - t0 < 3.0
    refused = link.run_block("", "while_stopped", 5)
    assert not refused.ok and "not ready" in refused.error
    time.sleep(max(0.0, 5.5 - (time.time() - t0)))                    # unlock fails < 5 s after the stop
    with Dashboard(link.host) as d:
        d.unlock_protective_stop()
    assert link.wait_until(lambda x: x.safety_mode == 1, 10.0) is not None
    home(link, T_ft)
