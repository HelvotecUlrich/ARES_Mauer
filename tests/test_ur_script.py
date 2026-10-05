"""mauer.ur.script: URScript text generation (no robot). Units: inputs mm/rad, URScript m/rad."""
import re

import numpy as np
import pytest

from mauer import config, geometry as g
from mauer.ur import script as s

Q = [0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]
SP = s.Speeds(v_joint=0.5, a_joint=0.6, v_lin=0.1, a_lin=0.3, v_contact=0.02)
_POSE = re.compile(r"p\[([^\]]*)\]")


def poses_in(text: str) -> list[np.ndarray]:
    """All p[...] literals of a text as 4x4 [mm]."""
    return [g.ur_to_T([float(v) for v in m.split(",")]) for m in _POSE.findall(text)]


def test_num_formatting():
    assert s.num(0.5) == "0.5"
    assert s.num(24) == "24.0"
    assert s.num(-0.0) == "0.0"
    assert s.num(-1e-12) == "0.0"
    assert s.num(1e-9) == "0.000000001"
    assert s.num(-0.001) == "-0.001"
    assert "e" not in s.num(1.234567e-7)
    with pytest.raises(ValueError):
        s.num(float("nan"))


def test_pose_literal_roundtrip_mm_to_m():
    rng = np.random.default_rng(1)
    for _ in range(50):
        T = g.pose_xyz_rpy(rng.uniform(-900, 900, 3), rng.uniform(-180, 180, 3))
        lit = s.pose(T)
        vals = [float(v) for v in _POSE.fullmatch(lit).group(1).split(",")]
        assert np.all(np.abs(vals[:3]) <= 0.9 + 1e-9)          # metres on the wire
        d_mm, d_deg = g.pose_delta(g.ur_to_T(vals), T)
        # 9 decimals: 1 nm, 1e-9 rad; pose_delta's arccos adds ~1e-6 deg round-off near zero
        assert d_mm < 1e-5 and d_deg < 1e-5
    # near 180 deg the rotation vector is not unique, the pose must still round-trip
    T = g.pose_xyz_rpy([400, -100, 300], [180, 0, 0])
    assert g.pose_delta(poses_in(s.pose(T))[0], T)[1] < 1e-5


def test_pose_rejects_non_rigid():
    with pytest.raises(ValueError):
        s.pose(np.diag([2.0, 1.0, 1.0, 1.0]))
    with pytest.raises(ValueError):
        s.pose(np.eye(3))


def test_set_tcp_and_payload_units():
    cfg = config.load()
    T_ft = config.T_flange_tcp(cfg)
    txt = s.set_tcp(T_ft)
    assert txt.startswith("set_tcp(p[") and "0.135000000" in txt and "1.570796327" in txt
    assert np.allclose(poses_in(txt)[0], T_ft, atol=1e-6)
    assert s.set_payload(1.25, [10.0, 0.0, 60.0]) == "set_payload(1.25, [0.01, 0.0, 0.06])"
    with pytest.raises(ValueError):
        s.set_payload(-1.0, [0, 0, 0])
    assert s.tool_voltage(24) == "set_tool_voltage(24)"
    with pytest.raises(ValueError):
        s.tool_voltage(5)
    pre = s.preamble(T_ft, 1.0, [0, 0, 60], tool_volt=24).splitlines()
    assert [ln.split("(")[0] for ln in pre] == ["set_tcp", "set_payload", "set_tool_voltage"]


def test_moves():
    assert s.movej_q(Q, 1.4, 1.05) == ("movej([0.000000000, -1.570796327, 1.570796327, -1.570796327, "
                                        "-1.570796327, 0.000000000], a=1.4, v=1.05)")
    T = g.pose_xyz_rpy([400, 100, 300], [180, 0, 0])
    ml = s.movel(T, 0.3, 0.1)
    assert ml.startswith("movel(p[0.400000000, 0.100000000, 0.300000000,") and ml.endswith("a=0.3, v=0.1)")
    with pytest.raises(ValueError):
        s.movel(T, 0.3, 0.0)                                     # speeds must be > 0
    with pytest.raises(ValueError):
        s.movej_q(Q[:5], 1, 1)


def test_movej_pose_has_ik_guard_and_qnear():
    T = g.pose_xyz_rpy([400, 100, 300], [180, 0, 0])
    txt = s.movej_pose(T, Q, 0.5, 0.4, var="tgt")
    lines = txt.splitlines()
    assert lines[0].startswith("tgt = p[")
    assert lines[1].startswith("if not get_inverse_kin_has_solution(tgt, qnear=[")
    assert "  halt" in lines and "halt()" not in txt              # halt is a keyword, halt() does not compile
    assert f"write_output_integer_register({s.REG_ERROR}, {s.ERR_IK_UNREACHABLE})" in txt
    assert lines[-1].startswith("movej(get_inverse_kin(tgt, qnear=[0.000000000, -1.570796327")
    assert lines[-1].endswith("a=0.5, v=0.4)")
    assert txt.count("if not") == 1 and lines[-2] == "end"
    with pytest.raises(ValueError):
        s.movej_pose(T, Q, 0.5, 0.4, var="end")                  # keyword as variable


def test_look_pose_flange_to_tcp():
    cfg = config.load()
    T_ft = config.T_flange_tcp(cfg)
    T_bf = g.pose_xyz_rpy([-450, -100, 450], [180, 0, 30])
    txt = s.look_pose(T_bf, Q, 0.5, 0.5, T_flange_tcp=T_ft)
    T_sent = poses_in(txt.splitlines()[0])[0]
    assert g.pose_delta(T_sent, T_bf @ T_ft)[0] < 1e-6           # TCP pose = flange @ T_flange_tcp
    assert g.pose_delta(poses_in(s.look_pose(T_bf @ T_ft, Q, 0.5, 0.5, target="tcp"))[0], T_bf @ T_ft)[0] < 1e-6
    with pytest.raises(ValueError):
        s.look_pose(T_bf, Q, 0.5, 0.5)                           # flange target needs the TCP


def test_gripper_pulse_and_level():
    txt = s.gripper("close", do_open=0, do_close=1, pulse_s=0.5, wait_s=1.0)
    calls = [ln for ln in txt.splitlines() if not ln.startswith("#")]
    assert calls == ["set_standard_digital_out(0, False)", "set_standard_digital_out(1, True)", "sleep(0.5)",
                     "set_standard_digital_out(1, False)", "sleep(1.0)"]
    lvl = s.gripper("open", 0, 1, pulse_s=0.0, wait_s=0.2)
    assert "set_standard_digital_out(0, True)" in lvl and "set_standard_digital_out(0, False)" not in lvl
    assert "set_digital_out(" not in txt                            # deprecated in 3.15
    with pytest.raises(ValueError):
        s.gripper("open", 8, 1, 0.5, 1.0)
    with pytest.raises(ValueError):
        s.gripper("open", 1, 1, 0.5, 1.0)
    with pytest.raises(ValueError):
        s.gripper("grab", 0, 1, 0.5, 1.0)


def test_place_stone_sequence_and_poses():
    F = g.ur_to_T([-0.45, -0.10, 0.05, 0.0, 0.0, 0.3])           # frame measured by the camera
    local = g.rotx(np.pi)                                          # TCP z down at the place pose
    txt = s.place_stone(F, local, 150.0, Q, SP, do_open=0, pulse_s=0.5, wait_s=1.0, do_close=1, contact_mm=60.0,
                        payload_after=(0.8, [0, 0, 60]))
    lines = [ln for ln in txt.splitlines() if not ln.startswith(("#", " ", "if ", "end"))]
    assert lines[0].startswith("pl_F = p[") and np.allclose(poses_in(lines[0])[0], F, atol=1e-6)
    assigned = {ln.split(" =")[0]: poses_in(ln)[0] for ln in lines[1:4]}
    assert g.pose_delta(assigned["pl_at"], local)[0] < 1e-6
    assert g.pose_delta(assigned["pl_above"], g.transl(0, 0, 150) @ local)[0] < 1e-6   # along frame z
    assert g.pose_delta(assigned["pl_pre"], g.transl(0, 0, 60) @ local)[0] < 1e-6
    assert all("pose_trans(pl_F, p[" in ln for ln in lines[1:4])
    motion = [ln for ln in lines if ln.startswith(("movej", "movel", "set_standard", "sleep", "set_payload"))]
    assert motion == [
        "movej(get_inverse_kin(pl_above, qnear=[0.000000000, -1.570796327, 1.570796327, -1.570796327, "
        "-1.570796327, 0.000000000]), a=0.6, v=0.5)",
        "movel(pl_pre, a=0.3, v=0.1)", "movel(pl_at, a=0.3, v=0.02)",
        "set_standard_digital_out(1, False)", "set_standard_digital_out(0, True)", "sleep(0.5)",
        "set_standard_digital_out(0, False)", "sleep(1.0)", "set_payload(0.8, [0.0, 0.0, 0.06])",
        "movel(pl_pre, a=0.3, v=0.02)", "movel(pl_above, a=0.3, v=0.1)"]
    assert txt.count("get_inverse_kin_has_solution(") == 2           # above and at are guarded
    # without contact_mm the whole approach runs at contact speed
    one = s.place_stone(F, local, 150.0, Q, SP, 0, 0.5, 1.0)
    assert "pl_pre" not in one and "movel(pl_at, a=0.3, v=0.02)" in one and "movel(pl_above, a=0.3, v=0.02)" in one


def test_pick_stone_closes_and_sets_payload():
    F = g.pose_xyz_rpy([300, 200, 400], [0, 0, 90])
    txt = s.pick_stone(F, g.rotx(np.pi), 120.0, Q, SP, do_close=1, pulse_s=0.5, wait_s=1.0, do_open=0,
                       contact_mm=60.0, open_first=True, payload_after=(5.0, [0, 0, 120]))
    i_open = txt.index("set_standard_digital_out(0, True)")
    i_movej = txt.index("movej(get_inverse_kin(pk_above")
    i_close = txt.index("set_standard_digital_out(1, True)")
    i_payload = txt.index("set_payload(5.0, [0.0, 0.0, 0.12])")
    i_at = txt.index("movel(pk_at")
    assert i_open < i_movej < i_at < i_close < i_payload            # open first, close at the stone, then payload
    with pytest.raises(ValueError):
        s.pick_stone(F, g.rotx(np.pi), 120.0, Q, SP, 1, 0.5, 1.0, open_first=True)


def test_block_program_wrapper():
    body = s.movej_q(Q, 1.0, 1.0) + "\n" + s.ik_guard("x", Q)
    prog = s.block_program("look_1", body, 4711)
    lines = prog.splitlines()
    assert lines[0] == "def look_1():" and lines[-1] == "end" and prog.endswith("end\n")
    assert all(ln.startswith(" ") for ln in lines[1:-1] if ln)     # everything indented (SM l.115-117)
    assert lines[1] == f"  write_output_integer_register({s.REG_ERROR}, 0)"
    assert lines[2] == f"  write_output_integer_register({s.REG_STARTED}, 4711)"
    assert lines[-5:-1] == ["  while not is_steady():", "    sync()", "  end",
                            f"  write_output_integer_register({s.REG_DONE}, 4711)"]
    assert "    halt" in lines                                     # nested indentation kept
    no_err = s.block_program("b", "", 1, reg_error=None)
    assert "write_output_integer_register(26" not in no_err and no_err.count("write_output_integer_register") == 2
    for bad in (dict(name="1abc"), dict(name="def"), dict(body="textmsg(\"°\")"), dict(reg_started=48),
                dict(reg_done=24)):
        kw = dict(name="b", body="", block_id=1, reg_started=24, reg_done=25)
        kw.update(bad)
        with pytest.raises(ValueError):
            s.block_program(**kw)


def test_parse_set_tcp_and_abort():
    cfg = config.load()
    T_ft = config.T_flange_tcp(cfg)
    prog = s.block_program("b", s.set_tcp(np.eye(4)) + "\n" + s.preamble(T_ft, 1.0, [0, 0, 60]), 3)
    assert np.allclose(s.parse_set_tcp(prog), T_ft, atol=1e-6)    # the last set_tcp wins
    assert s.parse_set_tcp("movej([0,0,0,0,0,0])") is None
    assert s.abort_program() == "def mauer_abort():\n  stopl(1.2)\nend\n"


def test_speeds_from_config():
    sp = s.Speeds.from_config(config.load())
    assert sp.v_joint > 0 and sp.v_contact <= sp.v_lin            # [ur] ASSUMPTION values
