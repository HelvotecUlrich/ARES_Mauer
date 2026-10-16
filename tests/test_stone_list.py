"""tools/make_stone_list.py: the loading plan follows the job's reload rule and accounts for every stone."""
import sys
from collections import Counter

from mauer import REPO, config

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402
import make_stone_list as msl  # noqa: E402


def test_loading_plan_accounts_for_every_stone(tmp_path):
    cfg = config.load()
    job = make_job.build_nominal(cfg)
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


def test_station_stacks_tell_the_operator_how_high_to_fill():
    """2026-10-09: stacks of 5 / 4 / 2 layers (the UR reaches less at the ends and in row 2) - the list names every
    stack (as engraved on the MDF piece) with its row, position and the number of layers to fill; a stone above them
    is unknown to the software and in the jaws' way."""
    cfg = config.load()
    job = make_job.build_nominal(cfg)
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


def test_trace_follows_every_stone_from_station_to_wall():
    """2026-10-16 (Samuel: "welcher Stein in der Station schlussendlich wo an der Mauer ist"): every wall stone is
    traced to the magazine start fill or to one station slot (stack, layer, trip), of its own type, and every station
    slot the job fills supplies exactly one stone or is left over (no top-up in the main C)."""
    cfg = config.load()
    job = make_job.build_nominal(cfg)
    plan = msl.loading_plan(job)
    recs = plan["stones"]
    assert [r["nr"] for r in recs] == list(range(1, job.n_stones + 1))
    assert [r["label"] for r in recs] == [t.label for t in job.stones()]
    start = msl.magazine_start(job, plan)
    assert set(start) == set(job.magazine.initial_fill)
    assert all(r["kind"] == job.magazine.initial_kinds.get(sid, "full") for sid, r in start.items())
    from_st = [r for r in recs if r["src"] is not None]
    assert len({(r["src"], r["fill"]) for r in from_st}) == len(from_st)
    assert all(job.station.slot(r["src"]).kind == r["kind"] for r in from_st)
    for i, tr in enumerate(plan["trips"], 1):
        assert Counter(r["kind"] for r in from_st if r["trip"] == i) == Counter(
            {k: n for k, n in tr["moved"].items() if n})
    use = msl.station_use(job, plan)
    assert job.meta["planned_station_refills"] == 0
    assert all(len(v) == 1 for v in use.values()) and set(use) == set(job.station.take_order)
    assert sum(v[0]["nr"] is None for v in use.values()) == sum(plan["left"].values())
    assert msl.slot_txt("s01l5") == "s01·5" and msl.slot_txt("r1y0al1") == "r1y0a·1"
    assert msl.origin_de(from_st[0], job).startswith(f"Station {job.station.slot(from_st[0]['src']).stack_id} Lage")


def test_trace_is_what_the_sequencer_does(tmp_path):
    """The trace replays mauer.sequencer: an ARES tour (no stone handled, the slot counts run on) of the variant with a
    station top-up takes every stone from the traced magazine slot, filled from the traced station slot on the traced
    trip."""
    from mauer.sequencer import Sequencer, read_log
    from mauer.simworld import SimWorld, scenario
    cfg = config.load(None, "c_acb")
    job = make_job.build_nominal(cfg)
    plan = msl.loading_plan(job)
    assert any(any(tr["topup"].values()) for tr in plan["trips"])
    w = SimWorld(cfg, job, scenario("none"), seed=1)
    seq = Sequencer(job, cfg, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=tmp_path / "tour",
                    on_station_empty=w.refill_station, tour=True)
    assert seq.run().state == "done"
    origin = {sid: None for sid in job.magazine.initial_fill}
    trip, seen = 0, {}
    for e in read_log(seq.log.path):
        if e["event"] == "reload_start":
            trip += 1
            origin.update({mid: (ssid, trip) for ssid, mid, _ in e["plan"]})
        elif e["event"] == "tour_skip":
            seen[tuple(e["stone"])] = (e["slot"], origin.pop(e["slot"]))
    assert len(seen) == len(plan["stones"])
    for r, t in zip(plan["stones"], job.stones()):
        assert seen[t.key] == (r["mag"], None if r["src"] is None else (r["src"], r["trip"])), r["label"]
    assert any(r["fill"] for r in plan["stones"] if r["src"] is not None)        # a stone of the top-up is laid
    out = tmp_path / "liste_c_acb.pdf"                       # the list with the top-up column and refilled slots
    msl.write_pdf(out, job, cfg, tmp_path / "job.json")
    assert out.read_bytes().startswith(b"%PDF")


def test_numbers_keep_their_decimals():
    """2026-10-15: :.0f / :.1f printed the 99.5 mm half stone as 100 and 1.25 kg as 1,2 on the operator's list."""
    assert [msl._num(x) for x in (99.5, 120.0, 1.25, 3.0, 0.0)] == ["99,5", "120", "1,25", "3", "0"]
