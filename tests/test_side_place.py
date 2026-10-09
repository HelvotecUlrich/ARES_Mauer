"""Wall stones are set from the side (Samuel 2026-10-09: "seitlich und von oben anfahren, so 3 cm über den Pins des
unteren Steines und dann seitlich zum Stein nebendran hinfahren" - straight from above the small joint could hit the
neighbour, from the side it is at most pushed a little): a stone with exactly ONE placed neighbour in its course (any
leg, also across a corner) comes down [ur] place_side_mm (20) beside its place pose, away from that neighbour, to
config.side_lift_mm above it (the held stone's bottom [ur] place_side_above_pins_mm = 30 mm above the pin tips of the
course below), moves sideways to the place pose and goes down. Without a neighbour or between two: straight down."""
import copy
import math
import sys
import types

import numpy as np
import pytest

from mauer import REPO, config
from mauer import geometry as g
from mauer import job as mjob
from mauer.backends import release_pose

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def job(cfg):
    return make_job.build_nominal(cfg)


def _inside(q, P) -> bool:
    n = len(P)
    s = [(P[(i + 1) % n][0] - P[i][0]) * (q[1] - P[i][1]) - (P[(i + 1) % n][1] - P[i][1]) * (q[0] - P[i][0])
         for i in range(n)]
    return all(v > 0 for v in s) or all(v < 0 for v in s)


def _touching(cfg, job) -> dict:
    """Independent of wallplan: {stone key: set of its ends (-1 / +1 along its leg's u) touching a stone of the same
    course placed before it} - probed 5 mm beyond the end face on the stone's centre line, inside the footprint
    (over the ribs) of the earlier stones."""
    frames = config.leg_frames(cfg)
    W = float(cfg["brick"]["width"]) + 2.0 * float(cfg["brick"]["rib_mm"])
    placed, out = [], {}
    for t in job.stones():
        T = frames[t.leg]
        L = float(t.length_mm)
        to_w = lambda u, v: (T @ np.array([u, v, 0.0, 1.0]))[:2]       # noqa: E731
        ends = {e: to_w(t.u_mm + e * (L / 2.0 + 5.0), 0.0) for e in (-1, 1)}
        out[t.key] = {e for e, q in ends.items() if any(_inside(q, P) for k, P in placed if k == t.course)}
        placed.append((t.course, [to_w(t.u_mm + du, dv) for du, dv in
                                  ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2))]))
    return out


def test_side_parameters_from_the_config(cfg):
    u, b = cfg["ur"], cfg["brick"]
    assert u["place_side_mm"] == 20.0 and u["place_side_above_pins_mm"] == 30.0
    assert config.side_lift_mm(cfg) == pytest.approx(float(b["pin_length"]) - float(b["bed_joint"]) + 30.0)


def test_every_stone_with_one_placed_neighbour_comes_from_the_other_side(cfg, job):
    frames = config.leg_frames(cfg)
    touch = _touching(cfg, job)
    side = [t for t in job.stones() if t.side_mm]
    assert len(side) > len(job.stones()) // 2
    for t in job.stones():
        ends = touch[t.key]
        if len(ends) != 1:
            assert t.side_mm == 0.0, (t.label, ends)
            continue
        assert abs(t.side_mm) == pytest.approx(float(cfg["ur"]["place_side_mm"])), t.label
        d = (t.T_wall_tcp @ np.array([t.side_mm, 0.0, 0.0, 1.0]))[:3] - t.T_wall_tcp[:3, 3]
        u_dir = frames[t.leg][:3, 0]
        assert abs(d[2]) < 1e-9 and abs(abs(float(d @ u_dir)) - abs(t.side_mm)) < 1e-6   # along the leg, level
        (e,) = ends
        assert np.sign(float(d @ u_dir)) == -e, (t.label, e)                        # away from the neighbour


def test_closures_and_first_stones_come_straight_down():
    """Variant c_acb: B closes between A and C - its stones with a neighbour on both sides have no side."""
    c = config.load(variant="c_acb")
    j = make_job.build_nominal(c)
    touch = _touching(c, j)
    both = [t for t in j.stones() if len(touch[t.key]) == 2]
    assert both and all(t.side_mm == 0.0 for t in both)
    assert all(t.side_mm == 0.0 for t in j.stones() if not touch[t.key])


def test_side_mm_round_trip_and_old_jobs(job, tmp_path):
    p = tmp_path / "j.json"
    mjob.save(job, p)
    back = mjob.load(p)
    assert [t.side_mm for t in back.stones()] == [t.side_mm for t in job.stones()]
    d = mjob.to_dict(job)
    for st in d["stops"]:
        for t in st["stones"]:
            t.pop("side_mm", None)
    old = mjob.from_dict(d)
    assert all(t.side_mm == 0.0 for t in old.stones())                          # old jobs: straight down


def test_ur_robot_places_from_the_side(cfg, job):
    """URRobot.place_wall: the guarded joint move ends above the side point (the descent column moves with it), the
    script comes down there, moves sideways at side_lift_mm - release_above_mm above the release pose."""
    from mauer.backends import URRobot
    from mauer.motionguard import MotionGuard
    from mauer.ur.link import BlockResult

    class Link:
        def __init__(self, q):
            self.blocks = []
            self.st = types.SimpleNamespace(actual_q=np.asarray(q, float), actual_qd=np.zeros(6),
                                            program_running=False, runtime_state=1)

        def state(self):
            return self.st

        def state_age_s(self):
            return 0.0

        def run_block(self, body, name="b", timeout_s=0.0, settle_s=0.0):
            self.blocks.append((name, body))
            return BlockResult(ok=True, block_id=len(self.blocks), name=name, error="")

    c = copy.deepcopy(cfg)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"], c["half_brick"]["mass_kg"] = 1.5, 3.0, 1.5
    guard = MotionGuard(c, job.T_ares_base, job.park_q_rad, job.T_flange_tcp, approach_mm=job.approach_mm)
    cols = []
    guard.plan = lambda q_from, q_to, holding, vias=(), column=None: (
        cols.append(column) or types.SimpleNamespace(ok=True, vias=[], problems=[]))
    link = Link(job.park_q_rad)
    r = URRobot(link, c, job, guard=guard)
    stone = next(t for t in job.stones() if t.side_mm)
    T_base_wall = g.inv(job.T_ares_base) @ g.inv(job.stops[0].ares.T)
    r.place_wall(T_base_wall, stone)
    _, body = link.blocks[-1]
    rel = float(c["ur"]["release_above_mm"])
    T_rel = release_pose(stone.T_wall_tcp, rel)
    assert np.allclose(cols[-1], T_base_wall @ T_rel @ g.transl(stone.side_mm, 0.0, 0.0))
    assert "movej(get_inverse_kin(pl_sabove" in body and "movel(pl_low" in body
    low = config.side_lift_mm(c) - rel
    exp = g.transl(0.0, 0.0, low) @ T_rel @ g.transl(stone.side_mm, 0.0, 0.0)
    from test_ur_script import poses_in                                          # noqa: E402
    line = next(ln for ln in body.splitlines() if ln.startswith("pl_slow = pose_trans(pl_F"))
    assert g.pose_delta(poses_in(line)[0], exp)[0] < 1e-6


def test_sim_robot_comes_down_beside_the_place_pose(cfg, job):
    """SimRobot.place_wall asks the motion guard for the same approach as URRobot (above the side point) and ends at
    the release pose."""
    from mauer.simworld import SimRobot
    stone = next(t for t in job.stones() if t.side_mm)
    calls = []
    r = SimRobot.__new__(SimRobot)
    r._call = lambda name: None
    r._release_above = lambda: 10.0
    r._guard_above = lambda T_bf, T_ft, hint, what, approach_mm=None: calls.append(("guard", np.asarray(T_ft)))
    r._move_tcp = lambda T, what: calls.append(("move", np.asarray(T)))
    r._release = lambda: ("s1", np.eye(4), np.eye(4))
    r.world = types.SimpleNamespace(cfg=cfg, kind_of={"s1": stone.kind}, violations=[],
                                    record_placement=lambda *a: calls.append(("record",)))
    SimRobot.place_wall(r, np.eye(4), stone)
    T_rel = release_pose(stone.T_wall_tcp, 10.0)
    assert calls[0][0] == "guard" and np.allclose(calls[0][1], T_rel @ g.transl(stone.side_mm, 0.0, 0.0))
    assert calls[-2][0] == "move" and np.allclose(calls[-2][1], T_rel)
