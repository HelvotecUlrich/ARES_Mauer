"""URScript text for the UR5 CB3 (PolyScope 3.15) – pure functions, unit-testable without a robot.

Inputs follow docs/ARCHITECTURE.md: poses are 4x4 numpy matrices in mm (`T_a_b`), joint angles in rad. Speeds and
accelerations are passed in URScript units (joint moves rad/s and rad/s², linear moves m/s and m/s²), masses in kg.
Poses are converted to URScript `p[x, y, z (m), rx, ry, rz (rad rotation vector)]` only here, with
`mauer.geometry.T_to_ur` / `ur_str` (0.1 µm, 1e-9 rad).

Every function returns URScript statements, one per line and NOT indented (nested blocks indented by two spaces).
`block_program()` (used by `mauer.ur.link.URLink.run_block`) wraps them in a `def` program with start/done markers.

URScript references: "SM p.P l.L" = URScript Manual CB-Series 3.15.4
(reference/2026-10-02_Gino/UR5/scriptManual_3.15.4.pdf) page P, line L of its pdftotext (research scratch
scriptManual_3.15.4.txt, 2026-10-05). Calls marked "URSim" were run against URSim CB3 3.15.8 (research 2026-10-05,
research_out/ur5-interface.json, and tests/test_ur_ursim.py).

Program rules (SM p.4 l.105-119): a program sent to port 30002 starts with `def`/`sec` in column 1, every other line
is indented, the last line is `end` in column 1, and the text is ASCII only (SM l.108-109). `#` comment lines (SM p.11
l.491) are accepted over 30002 (URSim, place block).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ..geometry import T_to_ur, transl, ur_str, ur_to_T

# Output integer registers 24..47 belong to external RTDE clients (SM p.111 l.4613-4623; 0..23 = fieldbus/PLC).
# PolyScope 3.3 (the lab's UR5) has only 0..23: [ur] reg_* = 20..22 there (probe 2026-10-06).
REG_STARTED = 24        # [ur] reg_started: block id, first statement of a block
REG_DONE = 25           # [ur] reg_done: block id, last statement (after is_steady)
REG_ERROR = 26          # ASSUMPTION: error code register (0 = none), reset at the start of every block

# Error codes a block writes into REG_ERROR before it halts.
ERR_NONE = 0
ERR_IK_UNREACHABLE = 1  # get_inverse_kin_has_solution() was False / get_inverse_kin() stopped for a target pose

# How a block checks that its target poses have an IK solution before the first move ([ur] ik_check):
# "has_solution": get_inverse_kin_has_solution + halt (tested on URSim CB3 3.15.8);
# "get_inverse_kin": PolyScope 3.3.3 (the lab's UR5) has no get_inverse_kin_has_solution (compile error, probe
# 2026-10-06), but get_inverse_kin stops the program with a runtime error for an unreachable pose (no motion).
IK_CHECKS = ("has_solution", "get_inverse_kin")

ABORT_DECEL_MPS2 = 1.2  # ASSUMPTION: stopl deceleration [m/s²] for abort (= the UR movel default a, SM p.27 l.1138)

_IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
# URScript keywords that must not be used as program/variable names (SM p.6-7, flow of control, l.240-285).
_KEYWORDS = {"def", "sec", "end", "if", "elif", "else", "while", "for", "return", "halt", "thread", "run", "kill",
             "join", "and", "or", "not", "xor", "True", "False", "None", "global", "local", "break", "continue"}


# ── formatting ────────────────────────────────────────────────────────────────
def num(x: float) -> str:
    """Finite float as a URScript literal without exponent (9 decimals, trailing zeros trimmed): 0.5, 24.0, -0.001."""
    x = float(x)
    if not np.isfinite(x):
        raise ValueError(f"URScript number must be finite, got {x}")
    s = f"{x:.9f}".rstrip("0")
    s = s + "0" if s.endswith(".") else s
    return "0.0" if s in ("-0.0", "0.0") else s


def q_list(q_rad: Sequence[float]) -> str:
    """Joint list literal [q1, ..., q6] in rad (1e-9 rad)."""
    q = np.asarray(q_rad, float).reshape(-1)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError(f"need 6 finite joint angles [rad], got {q_rad!r}")
    return "[" + ", ".join(f"{v:.9f}" for v in q) + "]"


def _check_T(T: np.ndarray, what: str = "pose") -> np.ndarray:
    T = np.asarray(T, float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"{what}: need a finite 4x4 matrix [mm]")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0.0 or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError(f"{what}: not a rigid transform")
    return T


def pose(T_mm: np.ndarray) -> str:
    """URScript pose literal p[x, y, z (m), rx, ry, rz (rad)] from a 4x4 pose in mm."""
    return ur_str(T_to_ur(_check_T(T_mm)))


def _speed(name: str, x: float) -> str:
    if not (np.isfinite(x) and x > 0.0):
        raise ValueError(f"{name} must be > 0, got {x}")
    return num(x)


def _ident(name: str, what: str = "name") -> str:
    if not _IDENT.match(name) or name in _KEYWORDS:
        raise ValueError(f"{what} {name!r} is not a valid URScript identifier")
    return name


def _do(n: int) -> int:
    if int(n) != n or not 0 <= int(n) <= 7:
        raise ValueError(f"standard digital output must be 0..7 (SM p.100 l.4164-4170), got {n}")
    return int(n)


def _ascii(text: str, what: str) -> str:
    try:
        text.encode("ascii")
    except UnicodeEncodeError as e:
        raise ValueError(f"{what} must be ASCII (URScript over 30002, SM l.108-109): {e}") from None
    return text


def indent(text: str, n: int = 2) -> str:
    """Indent every non-empty line by n spaces (relative indentation is kept)."""
    pad = " " * n
    return "\n".join(pad + ln if ln.strip() else "" for ln in text.splitlines())


# ── motion parameters ────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Speeds:
    """Speeds/accelerations for generated moves, URScript units."""
    v_joint: float      # rad/s   movej (leading axis, SM p.26 l.1092)
    a_joint: float      # rad/s²
    v_lin: float        # m/s     free movel (approach/retract above the contact zone, SM p.27 l.1138)
    a_lin: float        # m/s²
    v_contact: float    # m/s     movel in the contact zone (last approach to / first retract from the stone)

    @classmethod
    def from_config(cls, cfg: dict) -> "Speeds":
        """From [ur] v_joint, a_joint, v_lin, a_lin, v_contact (all ASSUMPTION first-run values)."""
        u = cfg["ur"]
        return cls(float(u["v_joint"]), float(u["a_joint"]), float(u["v_lin"]), float(u["a_lin"]),
                   float(u["v_contact"]))

    def scaled(self, f: float) -> "Speeds":
        """Every speed and acceleration times f (0 < f <= 1): the HMI's REAL speed, for a pendant without the
        speed slider in use (2026-10-09)."""
        if not (math.isfinite(f) and 0.0 < f <= 1.0):
            raise ValueError(f"speed factor must be in (0, 1], got {f}")
        return Speeds(self.v_joint * f, self.a_joint * f, self.v_lin * f, self.a_lin * f, self.v_contact * f)


# ── single statements ────────────────────────────────────────────────────────
def set_tcp(T_flange_tcp: np.ndarray) -> str:
    """Active TCP = flange -> TCP transform (SM p.50 l.2122). URSim: stays active in later programs (not relied on)."""
    return f"set_tcp({pose(T_flange_tcp)})"


def set_payload(mass_kg: float, cog_mm: Sequence[float]) -> str:
    """Payload mass [kg] and centre of gravity [mm, flange frame] -> set_payload(m, [x, y, z] m) (SM p.49 l.2063;
    the cog argument is always given – omitting it is deprecated and ties the CoG to the TCP)."""
    if not (np.isfinite(mass_kg) and mass_kg >= 0.0):
        raise ValueError(f"payload mass must be >= 0 kg, got {mass_kg}")
    c = np.asarray(cog_mm, float).reshape(-1)
    if c.shape != (3,) or not np.all(np.isfinite(c)):
        raise ValueError(f"cog_mm needs 3 finite values, got {cog_mm!r}")
    return f"set_payload({num(mass_kg)}, [{', '.join(num(v / 1000.0) for v in c)}])"


def tool_voltage(volt: int = 24) -> str:
    """Tool connector supply 0, 12 or 24 V (SM p.101 l.4199-4206). Max 600 mA (UR5 manual, [camera] power)."""
    if volt not in (0, 12, 24):
        raise ValueError(f"tool voltage must be 0, 12 or 24 V, got {volt}")
    return f"set_tool_voltage({int(volt)})"


def preamble(T_flange_tcp: np.ndarray, payload_kg: float, cog_mm: Sequence[float], tool_volt: int | None = None) -> str:
    """set_tcp + set_payload (+ set_tool_voltage) – put at the start of every block (research recommendation)."""
    lines = [set_tcp(T_flange_tcp), set_payload(payload_kg, cog_mm)]
    if tool_volt is not None:
        lines.append(tool_voltage(tool_volt))
    return "\n".join(lines)


def movej_q(q_rad: Sequence[float], a: float, v: float) -> str:
    """Joint move to q [rad] with a [rad/s²], v [rad/s] (SM p.26 l.1092)."""
    return f"movej({q_list(q_rad)}, a={_speed('a', a)}, v={_speed('v', v)})"


def movel(T_base_tcp: np.ndarray, a: float, v: float) -> str:
    """Linear tool move to T_base_tcp [mm] with a [m/s²], v [m/s] (SM p.27 l.1138)."""
    return f"movel({pose(T_base_tcp)}, a={_speed('a', a)}, v={_speed('v', v)})"


def ik_guard(var: str, qnear_rad: Sequence[float], reg_error: int = REG_ERROR,
             code: int = ERR_IK_UNREACHABLE, ik_check: str = "has_solution") -> str:
    """Stop the program with an error code if pose variable `var` has no IK solution near qnear.

    ik_check "has_solution": get_inverse_kin_has_solution (SM p.43 l.1777-1791) avoids the runtime exception of
    get_inverse_kin (SM p.42 l.1740-1745); textmsg -> PolyScope Log tab (SM p.56 l.2344), popup non-blocking (SM p.48
    l.2014), error code into an RTDE output register (SM p.111 l.4613), then the `halt` keyword ends the program (SM
    p.7 l.279; `halt()` with parentheses is a compile error – URSim).

    ik_check "get_inverse_kin" (PolyScope 3.3, no get_inverse_kin_has_solution): the error code is written first,
    `<var>_q = get_inverse_kin(var, qnear=...)` stops the program with a runtime error if there is no solution (probe
    2026-10-06 on the UR5; the message is in the PolyScope Log tab), the code is cleared after it. The qnear keyword
    is the one the movej lines use. Check it on the robot with tools/calib_handeye.py plan --check (a block that does
    not move).

    Either way the link sees "stopped without done marker" + the code, and nothing has moved when the guards stand
    before the first move of the block.
    """
    _ident(var, "variable")
    if ik_check not in IK_CHECKS:
        raise ValueError(f"ik_check must be one of {IK_CHECKS}, got {ik_check!r}")
    if ik_check == "get_inverse_kin":
        _ident(f"{var}_q", "variable")
        return "\n".join([
            f"write_output_integer_register({int(reg_error)}, {int(code)})",
            f"{var}_q = get_inverse_kin({var}, qnear={q_list(qnear_rad)})",
            f"write_output_integer_register({int(reg_error)}, {ERR_NONE})",
        ])
    return "\n".join([
        f"if not get_inverse_kin_has_solution({var}, qnear={q_list(qnear_rad)}):",
        f'  textmsg("mauer: no IK solution for {var} = ", {var})',
        f'  popup("mauer: {var} not reachable (get_inverse_kin_has_solution)", title="mauer", error=True)',
        f"  write_output_integer_register({int(reg_error)}, {int(code)})",
        "  halt",
        "end",
    ])


def movej_pose(T_base_tcp: np.ndarray, qnear_rad: Sequence[float], a: float, v: float, var: str = "target",
               reg_error: int = REG_ERROR, ik_check: str = "has_solution") -> str:
    """Joint move to a Cartesian TCP pose: IK on the controller (calibrated kinematics) with the branch nearest
    qnear (e.g. the RoboDK plan), guarded by ik_guard (ik_check: see there).

    get_inverse_kin(x, qnear=...) SM p.42 l.1740 (URSim: keyword qnear works), movej SM p.26 l.1092.
    """
    _ident(var, "variable")
    q = q_list(qnear_rad)
    return "\n".join([
        f"{var} = {pose(T_base_tcp)}",
        ik_guard(var, qnear_rad, reg_error, ik_check=ik_check),
        f"movej(get_inverse_kin({var}, qnear={q}), a={_speed('a', a)}, v={_speed('v', v)})",
    ])


def look_pose(T_base_target: np.ndarray, qnear_rad: Sequence[float], a: float, v: float, *, target: str = "flange",
              T_flange_tcp: np.ndarray | None = None, var: str = "look", reg_error: int = REG_ERROR,
              ik_check: str = "has_solution") -> str:
    """Go to an image pose with a joint move (IK near qnear).

    target = "flange": T_base_target is T_base_flange (e.g. from vision.handeye.plan_poses); converted to the TCP
    pose with T_flange_tcp, which must be the TCP active in the block (the block's set_tcp). target = "tcp":
    T_base_target is T_base_tcp. The block wrapper waits for is_steady(); the extra camera settle time (ARES springs)
    is run_block(settle_s=...).
    """
    if target == "flange":
        if T_flange_tcp is None:
            raise ValueError("look_pose(target='flange') needs T_flange_tcp (the TCP set in the same block)")
        T_base_tcp = _check_T(T_base_target, "T_base_flange") @ _check_T(T_flange_tcp, "T_flange_tcp")
    elif target == "tcp":
        T_base_tcp = T_base_target
    else:
        raise ValueError(f"target must be 'flange' or 'tcp', got {target!r}")
    return movej_pose(T_base_tcp, qnear_rad, a, v, var=var, reg_error=reg_error, ik_check=ik_check)


def gripper(action: str, do_open: int | None, do_close: int | None, pulse_s: float, wait_s: float) -> str:
    """Open/close the jaws with standard digital outputs (set_standard_digital_out SM p.100 l.4164; sleep SM p.50
    l.2134). Wiring [ur] do_grip_*: DO0 = open, DO1 = close, edge-triggered (UR5 test 2026-10-06).

    pulse_s > 0: the other output off, this output on for pulse_s, off again, then wait_s for the stroke.
    pulse_s == 0: level mode – the other output off, this output stays on, then wait_s.
    Outputs keep their state when a program stops (UR5 manual, research 2026-10-05).
    """
    if action not in ("open", "close"):
        raise ValueError(f"action must be 'open' or 'close', got {action!r}")
    this, other = (do_open, do_close) if action == "open" else (do_close, do_open)
    if this is None:
        raise ValueError(f"gripper {action}: digital output number missing")
    this = _do(this)
    if other is not None and _do(other) == this:
        raise ValueError("do_open and do_close must differ")
    if not (np.isfinite(pulse_s) and pulse_s >= 0.0 and np.isfinite(wait_s) and wait_s >= 0.0):
        raise ValueError("pulse_s and wait_s must be >= 0 s")
    lines = [f"# gripper {action} (DO{this})"]
    if other is not None:
        lines.append(f"set_standard_digital_out({_do(other)}, False)")
    lines.append(f"set_standard_digital_out({this}, True)")
    if pulse_s > 0.0:
        lines += [f"sleep({num(pulse_s)})", f"set_standard_digital_out({this}, False)"]
    if wait_s > 0.0:
        lines.append(f"sleep({num(wait_s)})")
    return "\n".join(lines)


# ── pick / place relative to a measured frame ────────────────────────────────
def _contact_move(prefix: str, T_base_frame: np.ndarray, T_frame_tcp: np.ndarray, approach_mm: float,
                  qnear_rad: Sequence[float], speeds: Speeds, contact_mm: float | None,
                  grip_lines: str, payload_after: tuple[float, Sequence[float]] | None, reg_error: int,
                  label: str, ik_check: str = "has_solution") -> str:
    """Shared body of place_stone / pick_stone: above -> (pre) -> at -> grip -> (payload) -> (pre) -> above.

    All target poses are formed on the controller with pose_trans(F, local) (SM p.70 l.2899: T_world->to =
    T_world->from * T_from->to), F = T_base_frame measured by the camera, local = T_frame_tcp shifted along the
    FRAME z axis (wall/station frame: z up) by approach_mm / contact_mm.
    """
    _check_T(T_base_frame, "T_base_frame")
    _check_T(T_frame_tcp, "T_frame_tcp")
    if not (np.isfinite(approach_mm) and approach_mm > 0.0):
        raise ValueError(f"approach_mm must be > 0, got {approach_mm}")
    if contact_mm is not None and not (np.isfinite(contact_mm) and contact_mm > 0.0):
        raise ValueError(f"contact_mm must be > 0 or None, got {contact_mm}")
    two_stage = contact_mm is not None and contact_mm < approach_mm
    F, at, pre, above = f"{prefix}_F", f"{prefix}_at", f"{prefix}_pre", f"{prefix}_above"
    a_l, v_free, v_c = _speed("a_lin", speeds.a_lin), _speed("v_lin", speeds.v_lin), _speed("v_contact",
                                                                                           speeds.v_contact)
    lines = [
        f"# {label}: TCP relative to the measured frame, approach {num(approach_mm)} mm along frame z",
        f"{F} = {pose(T_base_frame)}",
        f"{at} = pose_trans({F}, {pose(T_frame_tcp)})",
        f"{above} = pose_trans({F}, {pose(transl(0.0, 0.0, approach_mm) @ T_frame_tcp)})",
    ]
    if two_stage:
        lines.append(f"{pre} = pose_trans({F}, {pose(transl(0.0, 0.0, contact_mm) @ T_frame_tcp)})")
    lines += [
        ik_guard(above, qnear_rad, reg_error, ik_check=ik_check),
        ik_guard(at, qnear_rad, reg_error, ik_check=ik_check),
        f"movej(get_inverse_kin({above}, qnear={q_list(qnear_rad)}), a={_speed('a_joint', speeds.a_joint)}, "
        f"v={_speed('v_joint', speeds.v_joint)})",
    ]
    if two_stage:
        lines.append(f"movel({pre}, a={a_l}, v={v_free})")
    lines += [f"movel({at}, a={a_l}, v={v_c})", grip_lines]
    if payload_after is not None:
        lines.append(set_payload(payload_after[0], payload_after[1]))
    if two_stage:
        lines += [f"movel({pre}, a={a_l}, v={v_c})", f"movel({above}, a={a_l}, v={v_free})"]
    else:
        lines.append(f"movel({above}, a={a_l}, v={v_c})")
    return "\n".join(lines)


def place_stone(T_base_frame: np.ndarray, T_frame_tcp_place: np.ndarray, approach_mm: float,
                qnear_rad: Sequence[float], speeds: Speeds, do_open: int, pulse_s: float, wait_s: float, *,
                do_close: int | None = None, contact_mm: float | None = None,
                payload_after: tuple[float, Sequence[float]] | None = None, reg_error: int = REG_ERROR,
                ik_check: str = "has_solution") -> str:
    """Place the held stone at T_frame_tcp_place relative to the measured frame T_base_frame (both mm).

    Sequence: movej (IK near qnear) to `approach_mm` above along frame z -> movel down (v_lin to `contact_mm` above,
    then v_contact; whole approach at v_contact if contact_mm is None) -> open pulse on do_open (+ do_close off)
    -> wait -> optional set_payload(payload_after = (kg, cog_mm), the payload without the stone) -> retract the
    same way. Tested shape: URSim place block (research test_place.py, tests/test_ur_ursim.py).
    """
    grip = gripper("open", do_open, do_close, pulse_s, wait_s)
    return _contact_move("pl", T_base_frame, T_frame_tcp_place, approach_mm, qnear_rad, speeds, contact_mm, grip,
                         payload_after, reg_error, "place_stone", ik_check)


def dry_place_stone(T_base_frame: np.ndarray, T_frame_tcp_hover: np.ndarray, approach_mm: float,
                    qnear_rad: Sequence[float], speeds: Speeds, dwell_s: float, *, contact_mm: float | None = None,
                    reg_error: int = REG_ERROR, ik_check: str = "has_solution") -> str:
    """A place WITHOUT letting go (magazine dry run, mauer/magtest.py): the held stone goes down to the hover pose
    T_frame_tcp_hover (the place pose raised along the frame z) like place_stone goes to the place pose, stays there
    dwell_s and comes back up. No gripper output and no payload change - the stone stays in the jaws."""
    if not (np.isfinite(dwell_s) and dwell_s >= 0.0):
        raise ValueError(f"dwell_s must be >= 0, got {dwell_s}")
    return _contact_move("dp", T_base_frame, T_frame_tcp_hover, approach_mm, qnear_rad, speeds, contact_mm,
                         f"sleep({num(dwell_s)})", None, reg_error, "dry_place_stone", ik_check)


def pick_stone(T_base_frame: np.ndarray, T_frame_tcp_pick: np.ndarray, approach_mm: float,
               qnear_rad: Sequence[float], speeds: Speeds, do_close: int, pulse_s: float, wait_s: float, *,
               do_open: int | None = None, contact_mm: float | None = None, open_first: bool = False,
               payload_after: tuple[float, Sequence[float]] | None = None, reg_error: int = REG_ERROR,
               ik_check: str = "has_solution") -> str:
    """Pick a stone at T_frame_tcp_pick relative to T_base_frame – mirror of place_stone with a close pulse.

    payload_after = (kg, cog_mm) with the stone (set right after closing, before lifting). open_first=True opens
    the jaws first (needs do_open); otherwise they must already be open (place_stone leaves them open).
    """
    grip = gripper("close", do_open, do_close, pulse_s, wait_s)
    body = _contact_move("pk", T_base_frame, T_frame_tcp_pick, approach_mm, qnear_rad, speeds, contact_mm, grip,
                         payload_after, reg_error, "pick_stone", ik_check)
    if open_first:
        if do_open is None:
            raise ValueError("open_first needs do_open")
        # open before the joint move to the approach pose (no stone in the jaws yet)
        lines = body.split("\n")
        i = next(k for k, ln in enumerate(lines) if ln.startswith("movej("))
        lines.insert(i, gripper("open", do_open, do_close, pulse_s, wait_s))
        body = "\n".join(lines)
    return body


# ── programs ──────────────────────────────────────────────────────────────────
def block_program(name: str, body: str, block_id: int, reg_started: int = REG_STARTED, reg_done: int = REG_DONE,
                  reg_error: int | None = REG_ERROR) -> str:
    """Complete `def` program around a block body with RTDE markers:

        def <name>():
          write_output_integer_register(reg_error, 0)      # error code reset (if reg_error is not None)
          write_output_integer_register(reg_started, id)
          <body>
          while not is_steady():                            # SM p.47 l.1966, sync SM p.55 l.2333
            sync()
          end
          write_output_integer_register(reg_done, id)       # arm at rest
        end

    Output registers keep their values across programs (URSim), so ids must change from block to block.
    """
    _ident(name, "program name")
    for r in (reg_started, reg_done) + (() if reg_error is None else (reg_error,)):
        if not 0 <= int(r) <= 47:
            raise ValueError(f"output integer register must be 0..47 (SM p.111 l.4619), got {r}")
    if len({reg_started, reg_done, reg_error} - {None}) != 2 + (reg_error is not None):
        raise ValueError("reg_started, reg_done and reg_error must differ")
    if not -2**31 <= int(block_id) < 2**31:
        raise ValueError("block id must fit int32")
    body = _ascii(body.replace("\r\n", "\n").replace("\r", "\n").rstrip(), "block body")
    lines = [f"def {name}():"]
    if reg_error is not None:
        lines.append(f"  write_output_integer_register({int(reg_error)}, {ERR_NONE})")
    lines.append(f"  write_output_integer_register({int(reg_started)}, {int(block_id)})")
    if body.strip():
        lines.append(indent(body, 2))
    lines += ["  while not is_steady():", "    sync()", "  end",
              f"  write_output_integer_register({int(reg_done)}, {int(block_id)})", "end", ""]
    return "\n".join(lines)


def abort_program(a: float = ABORT_DECEL_MPS2) -> str:
    """Program that interrupts the running one (a new `def` program stops it – URSim) and decelerates the tool
    with stopl(a [m/s²]) (SM p.36 l.1487)."""
    return f"def mauer_abort():\n  stopl({_speed('a', a)})\nend\n"


_SET_TCP = re.compile(r"set_tcp\(\s*p\[([^\]]*)\]\s*\)")


def parse_set_tcp(program: str) -> np.ndarray | None:
    """T_flange_tcp [mm] of the LAST set_tcp(p[...]) literal in a program text, None if there is none (used by the
    link to track the active TCP when RTDE tcp_offset is not available)."""
    m = None
    for m in _SET_TCP.finditer(program):
        pass
    if m is None:
        return None
    vals = [float(v) for v in m.group(1).split(",")]
    if len(vals) != 6:
        return None
    return ur_to_T(vals)
