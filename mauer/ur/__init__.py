"""UR5 CB3 (PolyScope 3.15) link from the laptop – free software only.

- `script`: pure functions that return URScript text (set_tcp, set_payload, tool voltage, movej/movel, IK-guarded
  Cartesian joint moves, pick/place relative to a camera-measured frame with pose_trans, gripper pulses) and the
  block wrapper with RTDE start/done markers. Unit-testable without a robot.
- `link`: `URLink` – RTDE state stream (125 Hz, laptop-time-stamped history) + one persistent port-30002 socket that
  runs one `def` program per atomic step and detects compile errors, protective stops and stopped-without-done.
- `dashboard`: `Dashboard` – port 29999 (power on, brake release, modes, unlock protective stop, stop, version).
- `rtde/`: official UR RTDE Python client 2.8.4 (BSD-3), vendored with one import changed (rtde/README.md).

Conventions (frames, mm/rad, T_a_b naming): docs/ARCHITECTURE.md. Parameters: config/station.toml [ur].
Integration tests against URSim CB3 3.15.8 (Docker): tests/test_ur_ursim.py; checks on the real robot:
tools/ur_check.py.
"""
