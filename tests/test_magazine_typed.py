"""Typed magazine slots (2026-10-07: every deck holder has 4 locating cones and takes one full stone or two half
stones; [deck] half_positions hold the half stones): mauer.job MagazineSlot.kind, SlotState fixed kinds, fill_plan /
reload_plan / reload_short with a capacity per stone type, the job file round trip and validation; tools/make_job.py
builds the half slots and turns every holder's grasp so that the tool stays clear of the arm (holder_pose)."""
import copy
import math
import sys

import numpy as np
import pytest

from mauer import REPO, config
from mauer import job as mjob

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402

I4 = np.eye(4)


def _mag(kinds_by_stack):
    """Magazine {stack: (kind, layers)}; take order top layer first."""
    slots = [mjob.MagazineSlot(f"{st}l{lay}", I4, lay, st, kind=k) for st, (k, n) in kinds_by_stack.items()
             for lay in range(1, n + 1)]
    take = [s.id for s in sorted(slots, key=lambda s: (-s.layer, s.id))]
    return mjob.Magazine(slots, take, list(reversed(take)), [])


def _station(n_full, n_half):
    slots = [mjob.StationSlot(f"s{i:02d}l1", I4, kind="full") for i in range(n_full)]
    slots += [mjob.StationSlot(f"h{i:02d}l1", I4, kind="half") for i in range(n_half)]
    return mjob.Station(I4, mjob.Pose2D(0, 0, 0), [], slots, [s.id for s in slots], [])


def test_reload_fills_each_type_into_its_own_slots_up_to_its_capacity():
    mag = mjob.SlotState.magazine(_mag({"r0": ("full", 2), "h0": ("half", 2)}), filled=[])
    st = mjob.SlotState.station(_station(4, 2))
    upcoming = ["full", "half", "full", "full", "half", "half", "full"]
    pairs = mjob.reload_plan(mag, st, upcoming)
    # the 3rd full stone does not fit the 2 full slots -> the magazine takes the first 3 stones only
    assert sorted((mid, k) for _, mid, k in pairs) == [("h0l1", "half"), ("r0l1", "full"), ("r0l2", "full")]
    assert all(ss.startswith("s" if k == "full" else "h") for ss, _, k in pairs)
    fill_idx = [mag.fill_order.index(mid) for _, mid, _ in pairs]
    assert fill_idx == sorted(fill_idx)                                       # bottom first
    for ss, mid, k in pairs:
        mag.fill(mid, k)
    for k in upcoming[:3]:                                                    # the sequencer's takes meet the stones
        sid = mag.next_take(kind=k)
        assert sid is not None
        mag.take(sid)
    assert mag.empty()
    with pytest.raises(ValueError, match="holds half stones"):                # a full stone never into a half slot
        mjob.SlotState.magazine(_mag({"h0": ("half", 1)}), filled=[]).fill("h0l1", "full")


def test_reload_short_counts_the_typed_capacity():
    mag = mjob.SlotState.magazine(_mag({"r0": ("full", 2), "h0": ("half", 2)}), filled=[])
    full = mjob.SlotState.station(_station(4, 2))
    one_half = full.copy()
    one_half.take("h01l1")
    assert not mjob.reload_short(one_half, full, mag, ["full", "full", "full"])     # no half needed
    assert mjob.reload_short(one_half, full, mag, ["half", "half", "full"])         # 1 instead of 2 halves


def test_untyped_magazine_keeps_the_old_rule():
    mag = mjob.SlotState.magazine(_mag({"r0": ("", 2)}), filled=[])
    pairs = mjob.reload_plan(mag, mjob.SlotState.station(_station(2, 2)), ["half", "full", "full"])
    assert [k for _, _, k in pairs] == ["full", "half"]                       # fill order; taken top first


def test_job_round_trip_and_validation_of_slot_kinds(tmp_path):
    cfg = config.load()
    job = make_job.build_nominal(cfg)
    kinds = {s.id: s.kind for s in job.magazine.slots}
    assert set(kinds.values()) == {"full", "half"}
    back = mjob.load(mjob.save(job, tmp_path / "j.json"))
    assert {s.id: s.kind for s in back.magazine.slots} == kinds
    bad = copy.deepcopy(job)
    sid = next(s for s in bad.magazine.initial_fill if kinds[s] == "full")
    bad.magazine.initial_kinds[sid] = "half"
    bad.magazine.slots[0].kind = "quarter"
    text = "\n".join(mjob.validate(bad))
    assert f"magazine initial_kinds: slot {sid} holds full stones, not half" in text
    assert "kind 'quarter'" in text


@pytest.fixture(scope="module")
def nominal():
    cfg = config.load()
    return cfg, make_job.build_nominal(cfg)


def test_make_job_puts_two_half_stones_per_layer_into_a_half_position(nominal):
    cfg, job = nominal
    dk = cfg["deck"]
    x0 = config.T_ares_base(cfg)[0, 3]
    half = [s for s in job.magazine.slots if s.kind == "half"]
    assert len(half) == len(dk["half_positions"]) * 2 * dk["magazine_layers"] and all(s.ik_ok for s in half)
    off = cfg["half_brick"]["length"] / 2
    for s in half:
        pos = s.stack[:-1]                                                   # r<row>y<column> + a / b
        ri, yi = int(pos[1:pos.index("y")]), int(pos[pos.index("y") + 1:])
        assert pos in dk["half_positions"] and s.stack[-1] in "ab"
        dy = -off if s.stack[-1] == "a" else off
        assert np.allclose(s.T_ares_tcp[:2, 3], [x0 + dk["magazine_rows_dx"][ri], dk["magazine_y"][yi] + dy])
        assert np.allclose(abs(s.T_ares_tcp[:3, 0]), [0, 1, 0], atol=1e-9)   # long axis across ARES (along y)
        assert off + cfg["half_brick"]["length"] / 2 == cfg["brick"]["length"] / 2      # on a full stone's footprint
    full_stacks = {s.stack for s in job.magazine.slots if s.kind == "full"}
    assert not any(st.startswith(tuple(dk["half_positions"])) for st in full_stacks)
    kinds = {x.id: x.kind for x in job.magazine.slots}
    assert job.magazine.initial_kinds and all(job.magazine.initial_kinds[s] == kinds[s]
                                              for s in job.magazine.initial_fill)
    for t in job.stones():                                                   # every stone from a slot of its type
        assert kinds[t.slot] == t.kind


def test_holders_are_gripped_with_the_tool_clear_of_the_arm(nominal):
    """2026-10-07: with the grasp yaw of simulate.py (stone axis +y) the magazine picks had the camera adapter 0.3 mm
    from wrist 1 (r1y2: -10 mm into the forearm) and the station picks -0.3 mm; holder_pose turns them by 180 deg."""
    from mauer import armcheck
    from mauer import geometry as g
    from mauer.simworld import ik_near
    cfg, job = nominal
    T_base_ares = g.inv(job.T_ares_base)
    poses = [(s.id, T_base_ares @ s.T_ares_tcp, s.qnear_rad) for s in job.magazine.slots if s.ik_ok]
    T_base_station = T_base_ares @ g.inv(job.station.dock.T)
    poses += [(s.id, T_base_station @ s.T_station_tcp, s.qnear_rad) for s in job.station.slots if s.ik_ok]
    worst_turned = math.inf
    for sid, T, q in poses:
        assert armcheck.self_clearance(q, cfg)[0] >= armcheck.SELF_CLEARANCE_MM, sid
        qt = ik_near(T @ g.rotz(math.pi) @ g.inv(job.T_flange_tcp), np.asarray(job.park_q_rad))
        if qt is not None:
            worst_turned = min(worst_turned, armcheck.self_clearance(qt, cfg)[0])
    assert worst_turned < armcheck.SELF_CLEARANCE_MM                         # the other yaw would be too close
    looks = [lk for st in job.stops for lk in st.looks] + list(job.station.looks)
    assert all(armcheck.self_clearance(lk.qnear_rad, cfg)[0] >= armcheck.SELF_CLEARANCE_MM for lk in looks)


def test_self_checker_flags_the_camera_folded_onto_the_forearm():
    from mauer import armcheck
    cfg = config.load()
    chk = armcheck.SelfChecker(cfg)
    q = np.radians([-69.0, -89.0, -98.0, -84.0, 90.0, 21.0])
    assert chk.hits(q, I4) == []
    hit = chk.hits(np.r_[q[:4], q[4] + np.radians(60), q[5] + np.radians(270)], I4)
    assert len(hit) == 1 and hit[0][0] in ("camera", "camera adapter") and hit[0][2] > armcheck.SELF_CLEARANCE_MM
