"""tools/make_stone_list.py: the loading plan follows the job's reload rule and accounts for every stone; the stone
flow (station slot -> magazine slot -> wall stone) is what the sequencer does; tools/make_station_map.py names every
slot."""
import copy
import sys
from collections import Counter

import pytest

from mauer import REPO, config

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402
import make_station_map as msm  # noqa: E402
import make_stone_list as msl  # noqa: E402

REAL_TOUR = REPO / "data" / "runs" / "2026-10-16_163318_hmi_real" / "run.jsonl"   # ARES tour on the real rig
REAL_JOB = REPO / "data" / "jobs" / "nominal_C_robodk.json"                          # the job that tour ran


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def job(cfg):
    return make_job.build_nominal(cfg)            # slow (minutes): built once for the module


def test_loading_plan_accounts_for_every_stone(cfg, job, tmp_path):
    plan = msl.loading_plan(job)
    n = Counter(t.kind for t in job.stones())
    assert len(plan["trips"]) == job.meta["planned_reloads"]
    assert sum(any(tr["topup"].values()) for tr in plan["trips"]) == job.meta["planned_station_refills"]
    for k in msl.KINDS:                      # put out (start + top-ups) = into the wall + left in the station
        put = plan["magazine"][k] + plan["station"][k] + sum(tr["topup"][k] for tr in plan["trips"])
        assert put - plan["left"][k] == n[k]
        assert sum(tr["moved"][k] for tr in plan["trips"]) + plan["magazine"][k] == n[k]
    out = tmp_path / "liste.pdf"
    info = msl.write_pdf(out, job, cfg, tmp_path / "job.json")
    assert out.read_bytes().startswith(b"%PDF") and info["counts"] == dict(n)


def test_station_stacks_tell_the_operator_how_high_to_fill(job):
    """2026-10-09: stacks of 5 / 4 / 2 layers (the UR reaches less at the ends and in row 2) - the list names every
    stack (as engraved on the MDF piece) with its row, position and the number of layers to fill; a stone above them
    is unknown to the software and in the jaws' way."""
    stacks = msl.station_stacks(job)
    st = job.station
    assert {s["stack"] for s in stacks} == {sl.stack_id for sl in st.slots}
    for s in stacks:
        mine = [sl for sl in st.slots if sl.stack_id == s["stack"] and sl.id in st.take_order]
        assert s["layers"] == len(mine) and s["kind"] == st.slot(f"{s['stack']}l1").kind
    full = msl.mjob.SlotState.station(st)
    for k in msl.KINDS:
        assert sum(s["layers"] for s in stacks if s["kind"] == k) == full.count(k)
    assert {s["row"] for s in stacks} == {1, 2}
    assert max(s["layers"] for s in stacks) == 5 and min(s["layers"] for s in stacks) == 2


def test_numbers_keep_their_decimals():
    """2026-10-15: :.0f / :.1f printed the 99.5 mm half stone as 100 and 1.25 kg as 1,2 on the operator's list."""
    assert [msl._num(x) for x in (99.5, 120.0, 1.25, 3.0, 0.0)] == ["99,5", "120", "1,25", "3", "0"]


def _check_flow(job, flow, planned: bool = True):
    """Every wall stone once, in job order (from the slot the job planned, `planned`); types match the slots; every
    stone that went station -> magazine is used before the next trip; a station slot gives one stone per fill."""
    stones = job.stones()
    assert [r["key"] for r in flow["stones"]] == [tuple(t.key) for t in stones]
    assert [r["n"] for r in flow["stones"]] == list(range(1, len(stones) + 1))
    if planned:                                                   # else the sequencer logs slot_changed
        assert [r["mag"] for r in flow["stones"]] == [t.slot for t in stones]
    for r in flow["stones"]:
        ms = job.magazine.slot(r["mag"])
        assert not ms.kind or ms.kind == r["kind"]
        if r["station"] is not None:
            assert job.station.slot(r["station"]).kind == r["kind"]
    assert sorted(s["stone"]["n"] for s in flow["start"]) == [r["n"] for r in flow["stones"] if r["trip"] is None]
    assert [s["mag"] for s in flow["start"]] == [m for m in job.magazine.fill_order if m in job.magazine.initial_fill]
    assert all(s["kind"] == s["stone"]["kind"] for s in flow["start"])
    taken = set()
    for tr in flow["trips"]:
        if tr["refill"]:
            taken = set()
        got = [m["stone"]["n"] for m in tr["moves"]]
        assert sorted(got) == [r["n"] for r in flow["stones"] if r["trip"] == tr["n"]]
        assert sorted(got) == list(range(tr["first"], tr["first"] + len(got)))      # used before the next trip
        for m in tr["moves"]:
            assert m["stone"]["station"] == m["station"] and m["stone"]["mag"] == m["mag"]
            assert m["station"] not in taken
            taken.add(m["station"])
    assert len(flow["start"]) + sum(len(tr["moves"]) for tr in flow["trips"]) == len(stones)


def _tour_log(cfg, job, folder):
    """Run log of an ARES tour of `job` in the simulation (the real sequencer's slot bookkeeping, no camera)."""
    from mauer.sequencer import Sequencer, read_log
    from mauer.simworld import SimWorld, scenario
    w = SimWorld(cfg, job, scenario("none"), seed=1)
    seq = Sequencer(job, cfg, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=folder, camera_loop=False,
                    on_station_empty=w.refill_station, tour=True)
    res = seq.run()
    assert res.state == "done", res.error
    return read_log(seq.log.path)


def test_stone_flow_is_what_the_sequencer_does(cfg, job, tmp_path):
    """Samuel 2026-10-16: the list says for every stone which station slot and magazine slot it passes and where it
    goes in the wall - the run log of the sequencer (tour, simulated) moves exactly those stones; with a station of 2
    layers the operator tops it up before some trips (every empty holder) and the list names those holders."""
    flow = msl.stone_flow(job)
    _check_flow(job, flow)
    ev = _tour_log(cfg, job, tmp_path / "full")
    assert msl.compare_log(flow, ev) == []
    # the check is not vacuous: a stone from another slot, a changed transfer and a missing trip are reported
    bad = copy.deepcopy(ev)
    sk = next(e for e in bad if e["event"] == "tour_skip")
    sk["slot"] = "r9y9l9"
    rs = next(e for e in bad if e["event"] == "reload_start")
    rs["plan"][0][0] = "s99l1"
    i = max(i for i, e in enumerate(bad) if e["event"] == "reload_start")
    del bad[i]
    probs = msl.compare_log(flow, bad)
    assert any("Stein Nr. 1:" in p and "r9y9l9" in p for p in probs)
    assert any(p.startswith("Fahrt 1: Umladeplan") and "s99l1" in p for p in probs)
    assert any("Stationsfahrten" in p for p in probs)
    # a station of 2 layers: top-ups before some trips
    small = copy.deepcopy(job)
    small.station.take_order = [s for s in small.station.take_order if small.station.slot(s).layer <= 2]
    flow2 = msl.stone_flow(small)
    _check_flow(small, flow2, planned=False)
    assert any(tr["refill"] for tr in flow2["trips"])
    assert msl.compare_log(flow2, _tour_log(cfg, small, tmp_path / "small")) == []
    plan = msl.loading_plan(small)
    for tr, tp in zip(flow2["trips"], plan["trips"]):
        assert len(tr["refill"]) == sum(tp["topup"].values())


@pytest.mark.skipif(not (REAL_TOUR.exists() and REAL_JOB.exists()), reason="real tour log / job not on this machine")
def test_stone_flow_matches_the_real_tour_2026_10_16():
    """The ARES tour on the real rig (2026-10-16 16:33, job nominal_C_robodk.json): every stone was taken from the
    listed magazine slot, every trip moved the listed station slots into the listed magazine slots."""
    from mauer import job as mjob
    from mauer.sequencer import read_log
    flow = msl.stone_flow(mjob.load(REAL_JOB))
    assert msl.compare_log(flow, read_log(REAL_TOUR)) == []


def test_station_map_names_every_used_slot_once(cfg, job, tmp_path):
    """tools/make_station_map.py: every slot the job uses appears once under its stack (bottom first), the unused
    layers are named, the start fill is the job's; the PDF is written."""
    st, mag = job.station, job.magazine
    stacks = msm.station_stacks(job)
    ids = [i for s in stacks for i in s["ids"]]
    assert sorted(ids) == sorted(st.take_order) and len(ids) == len(set(ids))
    for s in stacks:
        assert s["ids"] == [f"{s['stack']}l{k}" for k in s["used"]] and s["used"] == sorted(s["used"])
        assert set(s["used"]) | set(s["unused"]) == {sl.layer for sl in st.slots if sl.stack_id == s["stack"]}
    mags = msm.magazine_stacks(job)
    assert sorted(i for s in mags for i in s["ids"]) == sorted(mag.take_order)
    assert sorted(f"{s['stack']}l{k}" for s in mags for k in s["start"]) == sorted(mag.initial_fill)
    out = tmp_path / "plan.pdf"
    msm.write_pdf(out, job, cfg, tmp_path / "job.json")
    assert out.read_bytes().startswith(b"%PDF")
