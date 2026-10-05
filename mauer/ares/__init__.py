"""ARES mobile platform: relative moves over ADS (CX9240 PLC v2.x, HMI interface v2, operating pattern A).

`AresAds` (mauer.ares.ads) commands one relative move at a time - a translation (dx, dy) or a rotation dtheta in the
ARES body frame at the start pose (+x forward, +y left, +theta CCW) - and reports the PLC result and the wheel-odometry
pose before and after. amr_hmi v2 must stay open: it owns the heartbeat, MANUAL and HALT. Tests use the pyads-like
fake PLC in tests/fake_plc.py; tools/ares_check.py is the command-line check.
"""
from __future__ import annotations

from .ads import (AresAds, AresConnectionError, AresError, AresNotReady, AresStatus, MoveOutcome, MoveRefused,
                  OdomPose, Preflight, check_move, next_move_id, parse_build_version, plc_timeout_s, tcp_reachable)

__all__ = ["AresAds", "AresConnectionError", "AresError", "AresNotReady", "AresStatus", "MoveOutcome", "MoveRefused",
           "OdomPose", "Preflight", "check_move", "next_move_id", "parse_build_version", "plc_timeout_s",
           "tcp_reachable"]
