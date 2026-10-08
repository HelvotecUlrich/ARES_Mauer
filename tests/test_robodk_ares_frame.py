"""RoboDK station on the real ARES layout (2026-10-08): the UR sits at the vehicle REAR - this repo's ARES frame (+x at
the UR end) is base_link turned by 180 deg, so the ARES STEP (base_link coordinates) is turned by 180 deg in the
"ARES base_link" frame - and the UR control box at the far end is part of the ARES object (every ARES collision pair).

Opt-in (own RoboDK instance on port 20594, closed at the end):
    py.exe -m pytest -m robodk tests/test_robodk_ares_frame.py -o addopts="" -q      (or MAUER_ROBODK=1)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.robodk


@pytest.fixture(autouse=True, scope="module")
def _opt_in(request):
    import os
    if "robodk" not in (request.config.getoption("markexpr") or "") and os.environ.get("MAUER_ROBODK") != "1":
        pytest.skip("RoboDK tests are opt-in: py.exe -m pytest -m robodk tests/test_robodk_ares_frame.py")


REPO = Path(__file__).resolve().parent.parent
if not Path(r"C:\RoboDK\bin\RoboDK.exe").exists():
    pytest.skip("RoboDK not installed", allow_module_level=True)
if str(REPO / "robodk") not in sys.path:
    sys.path.insert(0, str(REPO / "robodk"))

import build_station as bs  # noqa: E402
import rdk_common as rc  # noqa: E402

PORT = 20594


@pytest.fixture(scope="module")
def built():
    from mauer import config
    cfg = config.load()
    RDK = rc.connect(new_instance=True, port=PORT)
    try:
        it = bs.build(RDK, cfg, camera=False, boards=False, pickup=False)
        yield RDK, cfg, it
    finally:
        rc.close_instance(RDK)


def test_the_step_is_turned_and_the_controller_is_part_of_ares(built):
    from robodk.robolink import PROJECTION_ALONG_NORMAL
    RDK, cfg, it = built
    ares = it["ares"]
    R = np.array(ares.Pose().Rows())[:3, :3]
    assert np.allclose(R, [[-1, 0, 0], [0, -1, 0], [0, 0, 1]], atol=1e-9)     # rotz(180 deg): frame_x_points_to rear
    top = float(cfg["ares"]["deck_top_z"]) + float(cfg["ares"]["controller_top_mm"])
    # a vertical ray onto the box beyond the chassis end (-x, outside the deck) hits its top; one beyond the UR end
    # (+x, outside the deck) hits nothing of ARES at that height
    rays = [[-700.0, 0.0, 2000.0, 0.0, 0.0, 1.0], [700.0, 0.0, 2000.0, 0.0, 0.0, 1.0]]
    hits = RDK.ProjectPoints(rays, ares, PROJECTION_ALONG_NORMAL)
    z_ctl = hits[0][2]
    assert z_ctl == pytest.approx(top, abs=1.0), hits
    assert not (abs(hits[1][2] - top) < 1.0), hits
