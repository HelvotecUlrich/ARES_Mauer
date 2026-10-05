"""ARES_Mauer runtime package: camera, vision, UR5 and ARES interfaces, referencing and the sequencer.

Pure Python (numpy, OpenCV, pyads, IDS peak) – nothing in here imports RoboDK. The RoboDK glue (station, simulation,
simulated camera) lives in robodk/. Conventions (frames, units, pose naming): docs/ARCHITECTURE.md.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
