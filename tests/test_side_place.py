"""Wall stones are set from the side (Samuel 2026-10-09: "seitlich und von oben anfahren, so 3 cm über den Pins des
unteren Steines und dann seitlich zum Stein nebendran hinfahren" - straight from above the small joint could hit the
neighbour, from the side it is at most pushed a little): a stone with exactly ONE placed neighbour in its course (any
leg, also across a corner) comes down [ur] place_side_mm (20) beside its place pose, away from that neighbour, to
config.side_lift_mm above it (the held stone's bottom [ur] place_side_above_pins_mm = 30 mm above the pin tips of the
course below), moves sideways towards the place pose and goes down. Without a neighbour or between two: straight down.
Samuel 2026-10-16 ("die Position wo der Stein losgelassen wird soll immer noch 5mm seitlicher Abstand von der
Endposition haben, da wir sonst Stein an Stein streifen"): a stone set from the side is let go [ur] release_side_mm (5)
short of its place pose, on the side it comes from, [ur] release_above_mm (20) above it; the side point stays, the
sideways move is 15 mm. The stone drops onto the cones / pins and centres itself."""
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
    assert u["release_side_mm"] == 5.0 and config.release_side_mm(cfg) == 5.0
    assert config.release_side_mm({"ur": {}}) == 0.0                               # older configs: at the place pose


def test_side_release_split():
    """(shift, rest) along the TCP x: let go `shift` beside the place pose towards the side point, which stays."""
    assert config.side_release_split(20.0, 5.0) == (5.0, 15.0)
    assert config.side_release_split(-20.0, 5.0) == (-5.0, -15.0)
    assert config.side_release_split(0.0, 5.0) == (0.0, 0.0)                       # straight down: at the place pose
    assert config.side_release_split(20.0, 0.0) == (0.0, 20.0)
    for side, rel in ((20.0, 20.0), (-20.0, 25.0), (20.0, -1.0), (20.0, float("nan"))):
        with pytest.raises(ValueError):
            config.side_release_split(side, rel)


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


class _Link:
    def __init__(self, q):
        self.blocks = []
        self.st = types.SimpleNamespace(actual_q=np.asarray(q, float), actual_qd=np.zeros(6),
                                        program_running=False, runtime_state=1)

    def state(self):
        return self.st

    def state_age_s(self):
        return 0.0

    def run_block(self, body, name="b", timeout_s=0.0, settle_s=0.0):
        from mauer.ur.link import BlockResult
        self.blocks.append((name, body))
        return BlockResult(ok=True, block_id=len(self.blocks), name=name, error="")


def _ur_robot(cfg, job):
    """URRobot on a fake link with a guard that records the descent columns it is asked for."""
    from mauer.backends import URRobot
    from mauer.motionguard import MotionGuard
    c = copy.deepcopy(cfg)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"], c["half_brick"]["mass_kg"] = 1.5, 3.0, 1.5
    guard = MotionGuard(c, job.T_ares_base, job.park_q_rad, job.T_flange_tcp, approach_mm=job.approach_mm)
    cols = []
    guard.plan = lambda q_from, q_to, holding, vias=(), column=None, *a, **k: (
        cols.append(column) or types.SimpleNamespace(ok=True, vias=[], problems=[], blend_mm=[]))
    link = _Link(job.park_q_rad)
    return URRobot(link, c, job, guard=guard), link, cols, c


def _script_pose(body: str, name: str) -> np.ndarray:
    """The local pose (in the frame F) of `name = pose_trans(pl_F, p[...])` of a place script [mm]."""
    from test_ur_script import poses_in                                          # noqa: E402
    line = next(ln for ln in body.splitlines() if ln.startswith(f"{name} = pose_trans(pl_F"))
    return poses_in(line)[0]


def test_ur_robot_places_from_the_side(cfg, job):
    """URRobot.place_wall: the guarded joint move ends above the side point (the descent column moves with it), the
    script comes down there, moves sideways at side_lift_mm - release_above_mm above the release pose - and lets go
    [ur] release_side_mm short of the place pose on the side it comes from (Samuel 2026-10-16), release_above_mm
    above it."""
    r, link, cols, c = _ur_robot(cfg, job)
    stone = next(t for t in job.stones() if t.side_mm)
    T_base_wall = g.inv(job.T_ares_base) @ g.inv(job.stops[0].ares.T)
    r.place_wall(T_base_wall, stone)
    _, body = link.blocks[-1]
    rel, rs = float(c["ur"]["release_above_mm"]), float(c["ur"]["release_side_mm"])
    T_rel = release_pose(stone.T_wall_tcp, rel)
    T_side = T_rel @ g.transl(stone.side_mm, 0.0, 0.0)                            # side point: where it was
    assert np.allclose(cols[-1], T_base_wall @ T_side)
    assert "movej(get_inverse_kin(pl_sabove" in body and "movel(pl_low" in body
    low = config.side_lift_mm(c) - rel
    assert g.pose_delta(_script_pose(body, "pl_slow"), g.transl(0.0, 0.0, low) @ T_side)[0] < 1e-6
    at = _script_pose(body, "pl_at")                                             # where the jaws open
    d = at[:3, 3] - np.asarray(stone.T_wall_tcp)[:3, 3]
    toward_side = (T_side[:3, 3] - T_rel[:3, 3]) / abs(stone.side_mm)            # unit vector to the side point
    assert d[2] == pytest.approx(rel, abs=1e-6)                                  # 20 mm above (wall z)
    assert float(d @ toward_side) == pytest.approx(rs, abs=1e-6)                 # 5 mm towards the side it came from
    assert np.linalg.norm(d[:2] - float(d @ toward_side) * toward_side[:2]) < 1e-6
    assert np.allclose(at[:3, :3], np.asarray(stone.T_wall_tcp)[:3, :3])          # same orientation
    lo = _script_pose(body, "pl_low")                                            # sideways: 15 mm
    assert np.linalg.norm(_script_pose(body, "pl_slow")[:3, 3] - lo[:3, 3]) == pytest.approx(
        abs(stone.side_mm) - rs, abs=1e-6)
    assert np.allclose(lo[:3, 3], at[:3, 3] + np.array([0.0, 0.0, low]))


def test_ur_robot_straight_down_stones_let_go_at_the_place_pose(cfg, job):
    """Without a side (no neighbour or two): straight down, the jaws open release_above_mm above the place pose -
    no lateral offset."""
    r, link, _, c = _ur_robot(cfg, job)
    stone = next(t for t in job.stones() if not t.side_mm)
    r.place_wall(g.inv(job.T_ares_base) @ g.inv(job.stops[0].ares.T), stone)
    _, body = link.blocks[-1]
    assert "pl_sabove" not in body
    exp = release_pose(stone.T_wall_tcp, float(c["ur"]["release_above_mm"]))
    assert g.pose_delta(_script_pose(body, "pl_at"), exp)[0] < 1e-6


def test_place_wall_poses_are_the_poses_the_script_checks(cfg, job):
    """backends.place_wall_poses (the sequencer's reach check of a measured stop, 2026-10-16) lists exactly the poses
    whose IK the place script guards (the contact pose aside, it lies on a vertical between two of them)."""
    from mauer.backends import place_wall_poses
    r, link, _, c = _ur_robot(cfg, job)
    T_base_wall = g.inv(job.T_ares_base) @ g.inv(job.stops[0].ares.T)
    side = next(t for t in job.stones() if t.side_mm)
    straight = next(t for t in job.stones() if not t.side_mm)
    for stone, names in ((side, ("pl_above", "pl_at", "pl_sabove", "pl_slow", "pl_low")),
                         (straight, ("pl_above", "pl_at"))):
        r.place_wall(T_base_wall, stone)
        _, body = link.blocks[-1]
        got = place_wall_poses(c, stone, job.approach_mm)
        assert len(got) == len(names)
        for P, n in zip(got, names):
            assert g.pose_delta(P, _script_pose(body, n))[0] < 1e-6, n


def test_ur_robot_magazine_put_down_has_no_lateral_offset(cfg, job):
    r, link, _, c = _ur_robot(cfg, job)
    T_base_ares = g.inv(np.asarray(job.T_ares_base))
    slot = job.magazine.slot("r0y1l1")
    r.place_magazine(slot, T_base_ares, "full")
    _, body = link.blocks[-1]
    exp = release_pose(slot.T_ares_tcp, float(c["ur"]["release_above_mm"]))
    assert g.pose_delta(_script_pose(body, "pl_at"), exp)[0] < 1e-6


def test_ur_robot_refuses_a_release_side_beyond_the_side(cfg, job):
    from mauer.backends import URRobot
    c = copy.deepcopy(cfg)
    c["ur"]["release_side_mm"] = float(c["ur"]["place_side_mm"])                 # would let go AT the side point
    with pytest.raises(ValueError, match="release_side_mm"):
        URRobot(_Link(job.park_q_rad), c, job)
    c["ur"]["release_side_mm"] = -1.0
    with pytest.raises(ValueError, match="release_side_mm"):
        URRobot(_Link(job.park_q_rad), c, job)


def _sim_robot(cfg, stone, landing):
    """SimRobot.place_wall on stubs: `landing(T_release)` = the stone pose in the wall frame when the jaws open."""
    from mauer.simworld import SimRobot
    calls, recs = [], []
    r = SimRobot.__new__(SimRobot)
    r._call = lambda name: None
    r._release_above = lambda: 20.0
    r.capture_mm = 10.0
    r._guard_above = lambda T_bf, T_ft, hint, what, approach_mm=None: calls.append(("guard", np.asarray(T_ft)))
    r._move_tcp = lambda T, what: calls.append(("move", np.asarray(T)))
    r._release = lambda: ("s1", landing(calls[-1][1]), np.eye(4))
    pins = [(60.0, 0.0), (-60.0, 0.0)]
    r.world = types.SimpleNamespace(
        cfg=cfg, kind_of={"s1": stone.kind}, violations=[], pin_half_mm=60.0, height_mm=120.0,
        pins_xy=lambda kind: pins,
        record_placement=lambda st, T, T_ts, kind: recs.append(np.asarray(T, float)))
    return SimRobot.place_wall, r, calls, recs


def test_sim_robot_comes_down_beside_the_place_pose(cfg, job):
    """SimRobot.place_wall asks the motion guard for the same approach as URRobot (above the side point) and ends at
    the release pose, release_side_mm short of the place pose on the approach side."""
    stone = next(t for t in job.stones() if t.side_mm)
    place, r, calls, recs = _sim_robot(cfg, stone, lambda T_base_tcp: T_base_tcp)
    place(r, np.eye(4), stone)
    T_rel = release_pose(stone.T_wall_tcp, 20.0)
    s = float(np.sign(stone.side_mm)) * float(cfg["ur"]["release_side_mm"])
    assert calls[0][0] == "guard" and np.allclose(calls[0][1], T_rel @ g.transl(stone.side_mm, 0.0, 0.0))
    assert calls[-1][0] == "move" and np.allclose(calls[-1][1], T_rel @ g.transl(s, 0.0, 0.0))


def test_sim_cones_take_the_deliberate_side_offset_out(cfg, job):
    """SIM model (2026-10-16): a stone let go release_side_mm beside its place pose drops onto the conical pins of
    the course below; landing within their capture (capture_mm) they pull it back by that deliberate offset - the
    rest of the error (ARES, camera, arm) stays in the statistics as before. Beyond the capture it lies where it fell
    (not seated)."""
    stone = next(t for t in job.stones() if t.side_mm)
    place, r, _, recs = _sim_robot(cfg, stone, lambda T_base_tcp: T_base_tcp)          # exact arm, frames
    place(r, np.eye(4), stone)
    assert g.pose_delta(recs[-1], stone.T_wall_tcp)[0] < 1e-6                             # seated on its place pose
    e = np.asarray(stone.T_wall_tcp)[:3, 1] * 2.0                                         # a 2 mm error across
    place, r, _, recs = _sim_robot(cfg, stone, lambda T: g.transl(*e) @ T)
    place(r, np.eye(4), stone)
    assert g.pose_delta(recs[-1], g.transl(*e) @ np.asarray(stone.T_wall_tcp))[0] < 1e-6                           # the 2 mm stay
    toward = np.asarray(stone.T_wall_tcp)[:3, 0] * float(np.sign(stone.side_mm))
    far = toward * 6.0                                                                    # 5 + 6 = 11 > capture 10
    place, r, _, recs = _sim_robot(cfg, stone, lambda T: g.transl(*far) @ T)
    place(r, np.eye(4), stone)
    exp = g.transl(*(far + toward * float(cfg["ur"]["release_side_mm"]))) @ np.asarray(stone.T_wall_tcp)
    assert g.pose_delta(recs[-1], exp)[0] < 1e-6                                          # where it fell
