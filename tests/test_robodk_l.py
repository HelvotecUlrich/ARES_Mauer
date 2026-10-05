"""RoboDK model of the L wall: half stone mesh, the nominal L in build_station, and a short collision-checked run of
leg B's first stones next to the finished leg A (robodk/simulate.py LSim).

Opt-in like tests/test_robodk_camera.py (they start a separate RoboDK instance on port 20596 and close it at the end;
the user's RoboDK is never touched):
    py.exe -m pytest -m robodk tests/test_robodk_l.py -o addopts="" -q      (or MAUER_ROBODK=1)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.robodk


@pytest.fixture(autouse=True)
def _opt_in(request):
    """Opt-in: these start a separate RoboDK instance."""
    import os
    if "robodk" not in (request.config.getoption("markexpr") or "") and os.environ.get("MAUER_ROBODK") != "1":
        pytest.skip("RoboDK tests are opt-in: py.exe -m pytest -m robodk tests/test_robodk_l.py")


REPO = Path(__file__).resolve().parent.parent
if not Path(r"C:\RoboDK\bin\RoboDK.exe").exists():
    pytest.skip("RoboDK not installed", allow_module_level=True)
for p in (REPO / "robodk", REPO / "tools"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import build_station as bs  # noqa: E402
import make_half_stone as mh  # noqa: E402
import mauer.geometry as g  # noqa: E402
import rdk_common as rc  # noqa: E402
import wallplan  # noqa: E402

PORT = 20596                      # not the simulation (20599), not test_robodk_camera (20598), not the user's (2050x)


@pytest.fixture(scope="module")
def cfg():
    return rc.load_config()


@pytest.fixture(scope="module")
def rdk():
    RDK = rc.connect(new_instance=True, port=PORT)
    yield RDK
    rc.close_instance(RDK)


# ── half stone mesh (no RoboDK call) ──────────────────────────────────────────
def test_half_stone_mesh_closed_and_half_of_the_full_stone(cfg):
    full = mh.read_stl(REPO / cfg["brick"]["mesh"])
    tris, rep = mh.make_half(full, float(cfg["half_brick"]["length"]))
    assert rep["output_problems"] == []                       # closed, consistently oriented
    assert rep["n_loops"] == 1
    assert rep["output_volume_cm3"] == pytest.approx(rep["input_volume_cm3"] / 2, rel=2e-3)
    lo, hi = np.array(rep["bbox_min"]), np.array(rep["bbox_max"])
    assert lo[1] == pytest.approx(-cfg["half_brick"]["length"], abs=1e-3) and hi[1] == pytest.approx(0.0, abs=1e-3)
    assert lo[2] == pytest.approx(-cfg["brick"]["pin_length"], abs=1e-3)
    assert hi[2] == pytest.approx(cfg["brick"]["height"], abs=1e-3)
    pins = tris.reshape(-1, 3)
    pins = pins[pins[:, 2] < -1.0]                             # one pin pair below
    assert len(np.unique(np.round(pins[:, 0] > 60.0))) == 2      # both sides of the width
    assert pins[:, 1].min() > -100.0 and pins[:, 1].max() < 0.0


# ── station ───────────────────────────────────────────────────────────────────
def test_build_station_draws_the_l_with_leg_boards(rdk, cfg):
    it = bs.build(rdk, cfg)
    it["f_wall"].setPose(rc.transl(0, 0, 0))
    rdk.Update()
    stones = it["l_stones"]
    assert len(stones) == 76 and sum(s.kind == "half" for s in stones) == 8
    assert set(it["legs"]) == {"A", "B"}
    for name, f in it["legs"].items():
        assert np.allclose(g.from_robodk(f.Pose()), g.from_robodk(rc.leg_frame(cfg, name)), atol=1e-6)
    bb = it["wall"].setParam("BoundingBox")
    bb = eval(bb) if isinstance(bb, str) else bb                # noqa: S307 - RoboDK returns a dict literal
    rib = 1.69
    assert bb["min"][0] == pytest.approx(0.0, abs=0.01)
    assert bb["max"][0] == pytest.approx(2400.0 + rib, abs=0.05)          # leg B's outer ribs
    assert bb["min"][1] == pytest.approx(-60.0 - 1200.0, abs=0.01)        # leg B's far end
    assert bb["max"][1] == pytest.approx(60.0 + rib, abs=0.05)            # leg A's ARES-side ribs
    assert bb["max"][2] == pytest.approx(500.0, abs=0.01)                 # 4 courses on the 20 mm base
    from mauer.reference import placements
    T = {p.name: p.T_parent_board for p in placements(cfg) if p.parent == "wall"}
    sq = cfg["boards"]["ref"]["square_mm"]
    for name, T_wb in T.items():
        item = it["boards"][name]
        assert item.Parent().Name() == "Wall"
        assert np.allclose(g.from_robodk(item.Pose()), T_wb @ g.transl(-sq, -sq, 0.0), atol=1e-3), name


# ── simulation: leg B next to the finished leg A ──────────────────────────────
def test_leg_b_first_stones_collision_checked_and_boards_seen(rdk, cfg):
    import simulate as S
    from make_job import build_nominal

    class Args:
        dist = float(cfg["wall"]["dist_nominal"])

    it = bs.build(rdk, cfg)
    job = build_nominal(cfg)
    sim = S.LSim(rdk, cfg, it, job, Args())
    sim.setup()
    rdk.Render(False)
    sim.planner.z_safe = sim.safe_z(full=True)
    sim.j_home = sim.planner.compact(0.0, [0, -100, 52, -42, -90, 0])
    assert sim.j_home is not None
    sim.robot.setJoints(sim.j_home)
    st_res = sim.self_test()                                   # raises if a known collision is not reported
    assert st_res["RoboDK IK of the job look vs job qnear [deg]"] < 0.1
    k = next(i for i, s in enumerate(job.stops) if s.leg == "B")
    for st in job.stops[:k]:
        sim.prebuild(st.stones)                                 # leg A complete
    initial, events = S.magazine_events(job)
    first = sum(len(s.stones) for s in job.stops[:k])
    for sid, kind in S.magazine_before(job, initial, events, first):
        sim.fill_slot(sid, kind)
    stop = job.stops[k]
    sim.set_ares(stop.ares)
    sim.open_camera()
    rdk.Render(False)
    # the B-side boards are seen from the leg-B stop with leg A finished
    seen = [n for n in ("W5", "W6", "W7") if sim.check_board(n, None)["ok"]]
    assert len(seen) >= 2, seen
    # leg B's first stones (corner stones of courses 0 and 1, next to leg A's end) with collision-checked motion
    if first in events:
        for sid, kind in events[first]:
            sim.fill_slot(sid, kind)
    for t in stop.stones[:4]:
        rec = sim.lay(t)
        assert rec["ok"], rec
    assert {t.label for t in stop.stones[:4]} >= {"Bc0i0", "Bc1i0h"}
