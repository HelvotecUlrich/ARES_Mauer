"""RoboDK model of the L wall: half stone mesh, the nominal L in build_station, a short collision-checked run of
leg B's first stones next to the finished leg A and a station trip with stones moved station -> magazine
(robodk/simulate.py LSim).

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

PORT = 20596                      # not the simulation (20599), not test_robodk_camera (20598), not the user's (2050x);
                                  # the station-trip test uses PORT - 1


@pytest.fixture(scope="module")
def cfg():
    from conftest import l_config
    return l_config()                                     # the L of 2026-10-05 (the config is the C)


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
    B = next(lg for lg in cfg["wall"]["legs"] if lg["name"] == "B")
    assert bb["min"][1] == pytest.approx(B["xyz_in_wall"][1] - B["n0"] * 200.0, abs=0.01)   # leg B's far end
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
        for _, sid, kind in events[first]["pairs"]:
            sim.fill_slot(sid, kind)
    for t in stop.stones[:4]:
        rec = sim.lay(t)
        assert rec["ok"], rec
    assert {t.label for t in stop.stones[:4]} >= {"Bc0i0", "Bc1i0h"}


# ── station trip: stones station -> magazine ─────────────────────────────────
@pytest.fixture()
def rdk_fresh():
    """An instance of its own: textured boards added after the first camera of an instance render black
    (robodk/sim_camera.py), and the leg-B test above already opened one in the module instance."""
    RDK = rc.connect(new_instance=True, port=PORT - 1)
    yield RDK
    rc.close_instance(RDK)


def test_station_trip_moves_stones_into_the_magazine(rdk_fresh, cfg, tmp_path):
    """The first reload of the job, shortened: route to the dock, station boards seen, a top-layer stone, the stone
    below it and a half stone moved station -> magazine with collision-checked motion, route back - no colliding
    route sample, every stone exactly in its magazine slot."""
    import simulate as S
    from make_job import build_nominal

    class Args:
        dist = float(cfg["wall"]["dist_nominal"])

    rdk = rdk_fresh
    it = bs.build(rdk, cfg)
    job = build_nominal(cfg)
    sim = S.LSim(rdk, cfg, it, job, Args())
    sim.setup()
    rdk.Render(False)
    sim.planner.z_safe = sim.safe_z(full=True)
    sim.j_home = sim.planner.compact(0.0, [0, -100, 52, -42, -90, 0])
    sim.robot.setJoints(sim.j_home)
    sim.fill_station(job.station.take_order)
    assert len(sim.st) == len(job.station.take_order) == 20
    rdk.Update()
    assert sim.pairs() == []                                   # stones rest on the table and on each other
    initial, events = S.magazine_events(job)
    j1 = min(events)
    k = next(st.index for st in job.stops if j1 < sum(len(s.stones) for s in job.stops[:st.index + 1]))
    assert S.magazine_before(job, initial, events, j1) == []   # the magazine is empty before the first trip
    pairs = events[j1]["pairs"]
    top = pairs[0][0]
    assert job.station.slot(top).layer == 2
    below = top[:-1] + "1"
    half = next(sid for sid in job.station.take_order if job.station.slot(sid).kind == "half")
    ev = {"pairs": [(top, pairs[0][1], "full"), (below, pairs[1][1], "full"), (half, pairs[2][1], "half")],
          "refill": False}
    stop = job.stops[k]
    sim.set_ares(stop.ares)
    sim.open_camera()
    rdk.Render(False)
    trip = sim.station_trip(k, 1, stop, ev, animate=False, image=tmp_path / "l_station.png")
    assert trip["moved"] == 3, sim.transfer_log
    assert trip["image"] == "l_station.png" and (tmp_path / "l_station.png").stat().st_size > 10000
    assert trip["looks"] == {"S0": True, "S1": True}
    st_looks = [e for e in sim.look_log if e["state"] == "station trip 1"]
    assert st_looks and all(r["source"] == "job" for r in st_looks[0]["res"].values())   # the job's own looks
    assert [r["hits"] for r in sim.route_log] == [[], []]
    for ssid, mid, kind in ev["pairs"]:
        assert ssid not in sim.st and sim.mag[mid][1] == kind
        item = sim.mag[mid][0]
        assert item.Parent().Name() == "ARES base_link" and item.Name().startswith(f"Stone_mag_{mid}_")
        want = g.from_robodk(sim.slot_T[mid] * rc.rotx(np.pi) * rc.T_tc_cad(cfg, kind))
        assert np.allclose(g.from_robodk(item.Pose()), want, atol=1e-3), mid
    assert len(sim.st) == 17 and sim.pose.delta(stop.ares)[0] < 1e-6       # ARES back at the stop


# ── the C of the config (2026-10-06, ARES inside) ───────────────────────────────────────────────────────────────────
def test_c_self_test_and_first_stones_of_leg_c(rdk_fresh):
    """The C as configured (ARES inside, A on its right and C on its left at 580 mm, B in front at 840 mm): the
    collision self-test (ARES driven sideways into leg A is reported), legs A and B built without motion, the leg-C
    boards seen from the C stop and C's first stones next to B (the second corner) placed with collision-checked
    motion."""
    import simulate as S
    from make_job import build_nominal
    cfg = rc.load_config()
    assert cfg["wall"]["shape"] == "C" and cfg["wall"]["dist_nominal"] == 840.0

    class Args:
        dist = float(cfg["wall"]["dist_nominal"])

    rdk = rdk_fresh
    it = bs.build(rdk, cfg)
    job = build_nominal(cfg)
    sim = S.LSim(rdk, cfg, it, job, Args())
    sim.setup()
    rdk.Render(False)
    sim.planner.z_safe = sim.safe_z(full=True)
    sim.j_home = sim.planner.compact(0.0, [0, -100, 52, -42, -90, 0])
    sim.robot.setJoints(sim.j_home)
    st_res = sim.self_test()                                   # raises if a known collision is not reported
    assert any(k.startswith("ARES chassis 250 mm towards leg") and "30 mm into the wall" in k and v
               for k, v in st_res.items())
    k = next(i for i, s in enumerate(job.stops) if s.leg == "C")
    for st in job.stops[:k]:
        sim.prebuild(st.stones)                                 # legs A and B complete
    initial, events = S.magazine_events(job)
    first = sum(len(s.stones) for s in job.stops[:k])
    for sid, kind in S.magazine_before(job, initial, events, first):
        sim.fill_slot(sid, kind)
    stop = job.stops[k]
    sim.set_ares(stop.ares)
    sim.open_camera()
    rdk.Render(False)
    planned = {lk.boards[0]: lk for lk in stop.looks}
    seen = [n for n in sorted(planned) if sim.check_board(n, planned[n])["ok"]]
    assert seen == sorted(planned) == ["W6", "W7"], seen
    for i, t in enumerate(stop.stones[:4]):
        for _, sid, kind in events.get(first + i, {}).get("pairs", []):     # a reload before this stone (no trip)
            sim.fill_slot(sid, kind)
        rec = sim.lay(t)
        assert rec["ok"], rec
    assert {t.label for t in stop.stones[:4]} >= {"Cc0i0", "Cc1i0h"}
