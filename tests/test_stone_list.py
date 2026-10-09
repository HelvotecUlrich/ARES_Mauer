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
