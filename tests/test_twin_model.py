"""RoboDK twin, pure part (robodk/twin_model.py) - no RoboDK: stone-state diffs (picks, places, station trips,
states skipped between two ticks, a held stone of unknown state, a kind change), the two-phase application, the
[hmi.twin] port rules, the start view."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

from hmi.core.snapshot import RunSnapshot
from hmi_fakes import short_sim_session
from mauer import config

pytestmark = pytest.mark.usefixtures("no_lab_network")

ROBODK_DIR = Path(__file__).resolve().parent.parent / "robodk"
if str(ROBODK_DIR) not in sys.path:
    sys.path.insert(0, str(ROBODK_DIR))
import twin_model as tm  # noqa: E402  (pure: numpy and mauer, no RoboDK)

TOOL = tm.TOOL
SS = tm.StoneState


def st(mag=None, station=None, placed=(), held=None) -> "tm.StoneState":
    return SS(dict(mag or {}), dict(station or {}), frozenset(placed), held)


def moves(ops) -> list:
    return [(o.src, o.dst) for o in ops if o.kind == "move"]


# ── plan_ops ──────────────────────────────────────────────────────────────────
def test_pick_from_the_magazine_moves_the_stone_into_the_jaws():
    old = st({"m1": "full", "m2": "half"}, {"s1": "full"})
    new = st({"m2": "half"}, {"s1": "full"}, held=("magazine", "m1", "full"))
    assert tm.plan_ops(old, new) == [tm.Op("move", ("mag", "m1"), TOOL, "full")]


def test_place_moves_the_held_stone_into_the_wall():
    old = st({"m2": "half"}, held=("magazine", "m1", "full"))
    new = st({"m2": "half"}, placed=[("A", 0, 0)])
    ops = tm.plan_ops(old, new, {("A", 0, 0): "full"})
    assert ops == [tm.Op("move", TOOL, ("wall", ("A", 0, 0)), "full")]


def test_states_skipped_between_two_ticks_magazine_to_wall():
    """Picked and placed between two ticks: the magazine stone goes straight into the wall."""
    old = st({"m1": "full", "m2": "full"})
    new = st({"m2": "full"}, placed=[(0, 0)])
    assert moves(tm.plan_ops(old, new)) == [(("mag", "m1"), ("wall", (0, 0)))]


def test_station_refill_adds_stones():
    old = st({}, {"s1": "full"})
    new = st({}, {"s1": "full", "s2": "full", "s3": "half"})
    ops = tm.plan_ops(old, new)
    assert [(o.kind, o.dst, o.stone_kind) for o in ops] == [("add", ("station", "s2"), "full"),
                                                             ("add", ("station", "s3"), "half")]


def test_station_to_tool_to_magazine_and_a_whole_transfer_skipped():
    s0 = st({}, {"s1": "half", "s2": "full"})
    s1 = st({}, {"s2": "full"}, held=("station", "s1", "half"))
    s2 = st({"m4": "half"}, {"s2": "full"})
    assert tm.plan_ops(s0, s1) == [tm.Op("move", ("station", "s1"), TOOL, "half")]
    assert tm.plan_ops(s1, s2) == [tm.Op("move", TOOL, ("mag", "m4"), "half")]
    assert moves(tm.plan_ops(s0, s2)) == [(("station", "s1"), ("mag", "m4"))]       # station -> magazine


def test_held_stone_of_unknown_state_is_drawn_failed():
    """A failed pick: the magazine slot still counts as filled, the jaws may hold the stone - both are shown, the
    one in the jaws as failed; a failed place turns the held stone failed (object replaced)."""
    old = st({"m1": "full"})
    new = st({"m1": "full"}, held=tm.held_tuple({"from": "magazine", "slot": "m1", "kind": "full", "unknown": True}))
    assert new.held == ("unknown", "m1", "full")
    assert tm.plan_ops(old, new) == [tm.Op("add", None, TOOL, "full", True)]
    held = st({}, held=("magazine", "m1", "full"))
    failed = st({}, held=("unknown", "m1", "full"))
    assert [(o.kind, o.failed) for o in tm.plan_ops(held, failed)] == [("remove", False), ("add", True)]
    assert tm.plan_ops(failed, st({})) == [tm.Op("remove", TOOL, None, "full", True)]        # clear_held


def test_kind_mismatch_is_not_paired_and_removes_come_before_adds():
    old = st({}, held=("magazine", "m1", "full"))
    new = st({}, placed=[("B", 1, 0)])
    ops = tm.plan_ops(old, new, {("B", 1, 0): "half"})
    assert [(o.kind, o.src, o.dst, o.stone_kind) for o in ops] == [("remove", TOOL, None, "full"),
                                                                    ("add", None, ("wall", ("B", 1, 0)), "half")]


def test_apply_ops_two_phase_place_and_next_pick_in_one_tick():
    """Held full stone placed and the half stone of m2 picked between two ticks: the moves overlap at the jaws."""
    old = st({"m2": "half"}, held=("magazine", "m1", "full"))
    new = st({}, placed=[("A", 0, 1)], held=("magazine", "m2", "half"))
    ops = tm.plan_ops(old, new, {("A", 0, 1): "full"})
    assert sorted(moves(ops), key=str) == sorted([(("mag", "m2"), TOOL), (TOOL, ("wall", ("A", 0, 1)))], key=str)
    state = {("mag", "m2"): "stone-m2", TOOL: "stone-m1"}
    placed = []
    out = tm.apply_ops(state, ops, place=lambda obj, op: placed.append((obj, op.dst)))
    assert out == {TOOL: "stone-m2", ("wall", ("A", 0, 1)): "stone-m1"}
    assert sorted(placed, key=str) == sorted([("stone-m2", TOOL), ("stone-m1", ("wall", ("A", 0, 1)))], key=str)


def test_apply_ops_creates_missing_sources_and_drops_removed_objects():
    dropped = []
    ops = [tm.Op("move", ("mag", "x"), ("wall", (0, 0)), "full"), tm.Op("remove", ("mag", "y"), None, "full"),
           tm.Op("add", None, ("station", "s"), "half")]
    out = tm.apply_ops({("mag", "y"): "Y"}, ops, make=lambda op: f"new {op.dst[0]}", drop=dropped.append)
    assert out == {("wall", (0, 0)): "new wall", ("station", "s"): "new station"} and dropped == ["Y"]


def test_initial_state_and_snapshot_agree():
    s = short_sim_session()
    a, b = SS.initial(s.job), SS.from_snapshot(RunSnapshot.initial(s))
    assert a == b and a.counts() == {"mag": len(s.job.magazine.initial_fill), "station": len(s.job.station.slots),
                                     "wall": 0, "tool": 0}
    assert tm.plan_ops(a, b) == []
    assert all(o.kind == "add" for o in tm.plan_ops(st(), a)) and len(tm.plan_ops(st(), a)) == sum(a.counts().values())


# ── settings ──────────────────────────────────────────────────────────────────
def test_settings_from_the_station_config():
    t = tm.TwinSettings.from_config(config.load())
    assert t.port >= 20630 and 20500 not in t.ports and 20501 not in t.ports
    assert t.rate_hz == pytest.approx(10.0) and t.visible and t.ghost_wall


@pytest.mark.parametrize("twin", [{"port": 20500}, {"port": 20599}, {"port": 20630, "port_tries": 0},
                                  {"port": 20640, "rate_hz": 0.0}])
def test_settings_refuse_the_users_ports_and_nonsense(twin):
    with pytest.raises(ValueError):
        tm.TwinSettings.from_config({"hmi": {"twin": twin}})


def test_default_view_looks_at_the_scene_from_above():
    s = short_sim_session()
    x0, y0, x1, y1 = tm.scene_bounds(s.job)
    eye, target = tm.default_view(s.job)
    assert x0 <= target[0] <= x1 and y0 <= target[1] <= y1
    assert eye[2] > 1000.0 and eye[1] < target[1]
