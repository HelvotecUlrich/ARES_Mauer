"""mauer.motionguard (2026-10-07): the capsule-model check of every UR5 joint move before it is sent - park pose,
self collision, the held stone, the descent column of butt joints, contacts at the start, detours - and the guarded
sequencer run in mauer.simworld (the real backend URRobot refuses to start without it)."""
import copy
import math
import sys

import numpy as np
import pytest

from mauer import REPO, config
from mauer import geometry as g
from mauer.motionguard import GuardWorld, MotionGuard
from mauer.simworld import ik_near

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def job(cfg):
    return make_job.build_nominal(cfg)


def _guard(cfg, job):
    return MotionGuard(cfg, job.T_ares_base, job.park_q_rad, job.T_flange_tcp)


def _full_magazine(job):
    return [s for s in job.magazine.slots if s.id in job.magazine.take_order]


def test_the_park_pose_is_free_and_the_old_one_was_not(cfg, job):
    """[ur] park_q_deg of 2026-10-07: tool 40 mm from the arm, clear of ARES and a full magazine; the value before
    (same TCP pose, folded IK branch, never checked) had the jaws inside the forearm."""
    mg = _guard(cfg, job)
    mg.set_world(GuardWorld(magazine=_full_magazine(job)))
    assert mg.state_problems(job.park_q_rad) == [] and mg.state_problems(job.park_q_rad, "full") == []
    old = np.radians([21.34, -127.66, 20.7, 16.96, -90.0, 21.34])
    assert any("jaw" in p and "forearm" in p for p in mg.state_problems(old))


def test_a_target_in_self_collision_is_refused_before_anything_moves(cfg, job):
    mg = _guard(cfg, job)
    q = np.radians([-69.0, -89.0, -98.0, -84.0, 90.0, 21.0])
    folded = np.r_[q[:4], q[4] + math.radians(60), q[5] + math.radians(270)]        # camera folded onto the forearm
    v = mg.plan(q, folded)
    assert not v.ok and v.problems[0].startswith("target:") and "forearm" in v.problems[0]


def _first_b_stone(cfg, job):
    """World and approach joints for the first stone of leg B (butts against the finished leg A)."""
    k = next(i for i, s in enumerate(job.stops) if s.leg == "B")
    stop, t = job.stops[k], job.stops[k].stones[0]
    assert (t.leg, t.course, t.index) == ("B", 0, 0)
    T_base_wall = g.inv(np.asarray(job.T_ares_base)) @ g.inv(stop.ares.T)
    placed = [s for st in job.stops[:k] for s in st.stones]
    world = GuardWorld(magazine=[], wall_stones=placed, legs=job.legs, T_base_wall=T_base_wall)
    T_above = (T_base_wall @ g.transl(0.0, 0.0, job.approach_mm) @ t.T_wall_tcp @ g.inv(job.T_flange_tcp))
    q_above = ik_near(T_above, np.asarray(t.qnear_rad))
    column = (T_base_wall @ t.T_wall_tcp)[:2, 3]
    return world, q_above, column


def test_the_held_stone_may_touch_its_butt_joint_only_in_the_descent_column(cfg, job):
    """Main config: B's first stone is placed against A's ARES-side face (1 mm + rib); at its approach pose the held
    stone already hangs beside A's higher courses. In its descent column touching is allowed, overlapping is not."""
    mg = _guard(cfg, job)
    world, q_above, column = _first_b_stone(cfg, job)
    mg.set_world(world)
    assert mg.plan(job.park_q_rad, q_above, "full", column=column).ok
    v = mg.plan(job.park_q_rad, q_above, "full")
    assert not v.ok and "held stone" in v.problems[0] and "(wall)" in v.problems[0]
    from mauer.simworld import ur5_fk
    T_ft = np.asarray(job.T_flange_tcp)
    T_tcp = ur5_fk(q_above) @ T_ft
    T_bw = world.T_base_wall
    p_wall = g.apply(g.inv(T_bw), [T_tcp[:3, 3]])[0]
    p_new = g.apply(T_bw, [p_wall + np.array([0.0, -30.0, 0.0])])[0]       # 30 mm towards leg A's face (wall -y)
    T_new = T_tcp.copy()
    T_new[:3, 3] = p_new
    q_in = ik_near(T_new @ g.inv(T_ft), q_above)
    assert q_in is not None
    assert mg.state_problems(q_in, "full") != []
    v = mg.plan(job.park_q_rad, q_in, "full", column=p_new[:2])
    assert not v.ok                                              # overlap is refused even in the column


def _pick_start(cfg, job, mg, sid):
    """Arm at the approach pose above magazine slot sid with its stone in the jaws: the stones above it are gone (the
    stacks are emptied from the top), the other stacks full."""
    full = _full_magazine(job)
    slot = job.magazine.slot(sid)
    mg.set_world(GuardWorld(magazine=[o for o in full if o.id != sid and not (o.stack == slot.stack
                                                                               and o.layer > slot.layer)]))
    T = (g.inv(np.asarray(job.T_ares_base)) @ g.transl(0.0, 0.0, job.approach_mm) @ slot.T_ares_tcp
         @ g.inv(job.T_flange_tcp))
    return ik_near(T, np.asarray(slot.qnear_rad)), slot.kind or "full"


def test_contacts_at_the_start_may_stay_but_not_deepen(cfg, job):
    """A stone lifted out of a stack hangs 5 mm beside a higher neighbour stack (205 mm columns, 200 mm stones; a
    stack layout that the top-layer-first take order only leaves after an interruption). The arm stands there: a move
    that does not get closer may start with that contact (r1y2l1); the direct joint move from r1y1l1 to the park pose
    would swing the stone 21 mm into the neighbour - refused (the backend without the guard would send it)."""
    mg = _guard(cfg, job)
    park = np.asarray(job.park_q_rad)
    q, kind = _pick_start(cfg, job, mg, "r1y2l1")
    assert mg.state_problems(q, kind) != []
    assert mg.path_problems([q, park], kind, from_start=True) == []
    assert mg.path_problems([q, park], kind, from_start=False) != []
    q, kind = _pick_start(cfg, job, mg, "r1y1l1")
    p = mg.path_problems([q, park], kind, from_start=True)
    assert p and "held stone" in p[0]


def test_a_blocked_direct_move_gets_a_detour_that_is_clear(cfg, job):
    """The first look after the first station trip of the main C: the direct joint move from the park pose swings the
    jaws over the built leg A; the guard finds a detour and every point of it is clear."""
    mg = _guard(cfg, job)
    stop = job.stops[0]
    T_base_wall = g.inv(np.asarray(job.T_ares_base)) @ g.inv(stop.ares.T)
    mg.set_world(GuardWorld(magazine=_full_magazine(job), wall_stones=stop.stones[:14], legs=job.legs,
                            T_base_wall=T_base_wall))
    lk = stop.looks[0]
    q_look = np.asarray(lk.q_rad if lk.q_rad is not None else lk.qnear_rad, float)
    v = mg.plan(job.park_q_rad, q_look)
    assert v.ok
    assert mg.path_problems([np.asarray(job.park_q_rad), *v.vias, q_look]) == []


@pytest.mark.parametrize("variant", [None, "c_acb"], ids=lambda v: v or "main")
def test_guarded_sequencer_builds_the_first_stop(variant, tmp_path):
    """mauer.simworld with the guard (as URRobot on the real robot): the first stop of the C incl. its station trip,
    realistic ARES errors - every move planned by the guard, nothing refused, every stone seated."""
    from mauer.sequencer import Sequencer
    from mauer.simworld import SimWorld, scenario
    c = config.load(variant=variant)
    j = make_job.build_nominal(c)
    w = SimWorld(c, j, scenario("realistic"), seed=1, guard=True)
    seq = Sequencer(j, c, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=tmp_path / "run",
                    on_station_empty=w.refill_station)
    res = seq.run(0, 0)
    p = w.placement_stats()
    assert res.state == "done" and p["n"] == p["seated"] == len(j.stops[0].stones)
    assert w.robot.guard_vias > 0 and w.violations == []


def test_simworld_and_urrobot_use_the_same_park_pose(cfg, job):
    from mauer.simworld import SimWorld
    w = SimWorld(cfg, job, guard=True)
    assert np.allclose(w.robot.park_q, np.radians(cfg["ur"]["park_q_deg"]))
    c = copy.deepcopy(cfg)
    assert c["ur"]["park_q_deg"][0] == pytest.approx(161.83)
