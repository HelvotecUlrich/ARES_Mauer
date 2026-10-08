"""
frame.py - Mapping of physical operator directions to PLC-frame signs (spec 5.4, rev. 2 section 9).

The PLC code is documented as "+X forward, +Y left, +omega CCW" (GVL_AMR comments, right-handed IK). The
assignment lives in the HMI config:

    frame:
      plus_y_is_left: true      # PLC +Y is physically left
      plus_omega_is_ccw: true   # PLC +omega is physically counter-clockwise (seen from above)
      verified: false           # false -> warning banner + test-move button

28.09.2026 (Runbook R5, DECISIONS D21): with PLC v2 the robot had +Y = right but +omega = CCW (mirrored kinematics:
"Left" module at the rear, drive angle CW +). PLC v2.1 corrects the kinematics, so +Y = left and +omega = CCW;
the defaults follow v2.1. verified stays false until the direction test has been repeated with v2.1.
FrameConfig() and FrameConfig.from_config() use the same defaults as config.yaml.

Jog and relative move use the same mapping. For jog the PLC v2.1 maps (FB_HMI_Interface):
    bCmdJogLeft -> +vy, bCmdJogRight -> -vy, bCmdJogRotLeft -> +omega, bCmdJogRotRight -> -omega
so the HMI picks the bit that produces the wanted PLC sign. PLC v2 (2026-09-25) mapped bCmdJogLeft -> -vy:
this HMI needs PLC v2.1 for correct jog directions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Tuple

# Physical operator directions (seen from above, relative to the robot's front)
FORWARD = "forward"
BACK = "back"
LEFT = "left"
RIGHT = "right"
FWD_LEFT = "fwd_left"
FWD_RIGHT = "fwd_right"
BACK_LEFT = "back_left"
BACK_RIGHT = "back_right"
CCW = "ccw"
CW = "cw"

TRANSLATIONS = (FORWARD, BACK, LEFT, RIGHT, FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT)
ROTATIONS = (CCW, CW)

# (forward component, left component) of each physical translation direction, unit length
_PHYS_UNIT = {
    FORWARD:    (1.0, 0.0),
    BACK:       (-1.0, 0.0),
    LEFT:       (0.0, 1.0),
    RIGHT:      (0.0, -1.0),
    FWD_LEFT:   (math.sqrt(0.5), math.sqrt(0.5)),
    FWD_RIGHT:  (math.sqrt(0.5), -math.sqrt(0.5)),
    BACK_LEFT:  (-math.sqrt(0.5), math.sqrt(0.5)),
    BACK_RIGHT: (-math.sqrt(0.5), -math.sqrt(0.5)),
}

LABELS = {
    FORWARD: "Forward", BACK: "Back", LEFT: "Left", RIGHT: "Right",
    FWD_LEFT: "Fwd-Left", FWD_RIGHT: "Fwd-Right", BACK_LEFT: "Back-Left", BACK_RIGHT: "Back-Right",
    CCW: "CCW", CW: "CW",
}

# Sign each PLC jog bit produces (FB_HMI_Interface, PLC v2.1 2026-09-28; v2 had JogLeft -> -vy)
PLC_JOG_VX_SIGN = {"bCmdJogFwd": 1, "bCmdJogBwd": -1}
PLC_JOG_VY_SIGN = {"bCmdJogLeft": 1, "bCmdJogRight": -1}
PLC_JOG_OMEGA_SIGN = {"bCmdJogRotLeft": 1, "bCmdJogRotRight": -1}

# Defaults of the direction mapping (spec section 9); config.yaml ships the same values.
DEFAULT_PLUS_Y_IS_LEFT = True      # PLC v2.1: +Y = left (D21); physical "Left" -> bCmdJogLeft -> PLC +vy
DEFAULT_PLUS_OMEGA_IS_CCW = True   # +omega = CCW (observed 28.09.2026): "CCW" -> bCmdJogRotLeft -> PLC +omega


@dataclass(frozen=True)
class FrameConfig:
    plus_y_is_left: bool = DEFAULT_PLUS_Y_IS_LEFT
    plus_omega_is_ccw: bool = DEFAULT_PLUS_OMEGA_IS_CCW
    verified: bool = False

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "FrameConfig":
        """Build from the full config dict (section 'frame'); missing keys -> the defaults above."""
        sec = (cfg or {}).get("frame") or {}
        return cls(
            plus_y_is_left=bool(sec.get("plus_y_is_left", DEFAULT_PLUS_Y_IS_LEFT)),
            plus_omega_is_ccw=bool(sec.get("plus_omega_is_ccw", DEFAULT_PLUS_OMEGA_IS_CCW)),
            verified=bool(sec.get("verified", False)),
        )

    @property
    def left_sign(self) -> int:
        """PLC Y sign of physical 'left'."""
        return 1 if self.plus_y_is_left else -1

    @property
    def ccw_sign(self) -> int:
        """PLC omega/theta sign of physical 'counter-clockwise'."""
        return 1 if self.plus_omega_is_ccw else -1

    @property
    def status_word(self) -> str:
        """'verified' or 'unverified' (config frame.verified) - appended to every mapping text."""
        return "verified" if self.verified else "unverified"


def translation_unit(direction: str, frame: FrameConfig) -> Tuple[float, float]:
    """Unit vector (ux, uy) in the PLC frame for a physical translation direction."""
    if direction not in _PHYS_UNIT:
        raise ValueError(f"unknown translation direction: {direction!r}")
    fwd, left = _PHYS_UNIT[direction]
    return fwd, left * frame.left_sign


def translation_vector(direction: str, distance_mm: float, frame: FrameConfig) -> Tuple[float, float]:
    """(dx, dy) in mm in the PLC frame; |(dx, dy)| = |distance_mm|."""
    ux, uy = translation_unit(direction, frame)
    d = abs(float(distance_mm))
    return _clean(ux * d), _clean(uy * d)


def rotation_angle(direction: str, angle_deg: float, frame: FrameConfig) -> float:
    """Signed PLC theta [deg] for a physical rotation direction and an angle magnitude."""
    if direction == CCW:
        sign = frame.ccw_sign
    elif direction == CW:
        sign = -frame.ccw_sign
    else:
        raise ValueError(f"unknown rotation direction: {direction!r}")
    return _clean(sign * abs(float(angle_deg)))


def jog_field(direction: str, frame: FrameConfig) -> str:
    """PLC jog bit that moves the robot in the given physical direction (forward/back/left/right/ccw/cw)."""
    if direction == FORWARD:
        return "bCmdJogFwd"
    if direction == BACK:
        return "bCmdJogBwd"
    if direction in (LEFT, RIGHT):
        want = frame.left_sign if direction == LEFT else -frame.left_sign
        return next(f for f, s in PLC_JOG_VY_SIGN.items() if s == want)
    if direction in (CCW, CW):
        want = frame.ccw_sign if direction == CCW else -frame.ccw_sign
        return next(f for f, s in PLC_JOG_OMEGA_SIGN.items() if s == want)
    raise ValueError(f"no jog bit for direction {direction!r}")


def physical_y_name(sign: float, frame: FrameConfig) -> str:
    """Physical side ('left'/'right') of a PLC Y component with the given sign."""
    return "left" if (sign > 0) == frame.plus_y_is_left else "right"


def physical_rot_name(sign: float, frame: FrameConfig) -> str:
    """Physical sense ('CCW'/'CW') of a PLC theta with the given sign."""
    return "CCW" if (sign > 0) == frame.plus_omega_is_ccw else "CW"


def plus_y_text(frame: FrameConfig) -> str:
    """Neutral text of the Y mapping, e.g. 'PLC +Y (= right per config, unverified)'."""
    return f"PLC +Y (= {physical_y_name(1.0, frame)} per config, {frame.status_word})"


def plus_omega_text(frame: FrameConfig) -> str:
    """Neutral text of the rotation mapping, e.g. 'PLC +omega (= CCW per config, unverified)'."""
    return f"PLC +omega (= {physical_rot_name(1.0, frame)} per config, {frame.status_word})"


def physical_pose(x: float, y: float, theta_deg: float, frame: FrameConfig) -> Tuple[float, float, float]:
    """PLC odometry pose -> physically oriented map pose (forward, left, CCW heading), for the odometry map.

    Screen right = PLC +X (forward at the odometry origin), screen up = physical left:
        left = (+1 if plus_y_is_left else -1) * y,   heading_ccw = (+1 if plus_omega_is_ccw else -1) * theta
    X is never affected by the config. Units are passed through (m / deg).
    """
    return float(x), _clean(frame.left_sign * float(y)), _clean(frame.ccw_sign * float(theta_deg))


def describe_translation(dx_mm: float, dy_mm: float, frame: FrameConfig) -> str:
    """Plain text of a translation, e.g. '+X 1000 mm (forward)' or '+X 707 mm +Y 707 mm (forward-left)'."""
    parts = []
    phys = []
    if abs(dx_mm) >= 0.05:
        parts.append(f"{'+' if dx_mm > 0 else '-'}X {abs(dx_mm):.0f} mm")
        phys.append("forward" if dx_mm > 0 else "back")
    if abs(dy_mm) >= 0.05:
        parts.append(f"{'+' if dy_mm > 0 else '-'}Y {abs(dy_mm):.0f} mm")
        phys.append(physical_y_name(dy_mm, frame))
    if not parts:
        return "no translation (0 mm)"
    dist = math.hypot(dx_mm, dy_mm)
    txt = " ".join(parts)
    if len(parts) == 2:
        txt += f" = {dist:.0f} mm"
    return f"{txt} ({'-'.join(phys)})"


def describe_rotation(theta_deg: float, frame: FrameConfig) -> str:
    """Plain text of a rotation, e.g. '+90.0 deg (CCW)'."""
    if abs(theta_deg) < 1e-6:
        return "no rotation (0 deg)"
    return f"{theta_deg:+.1f} deg ({physical_rot_name(theta_deg, frame)})"


def _clean(v: float) -> float:
    """Remove -0.0 and float noise below 1e-9."""
    return 0.0 if abs(v) < 1e-9 else v
