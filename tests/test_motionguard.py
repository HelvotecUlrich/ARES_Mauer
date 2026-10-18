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
    return MotionGuard(cfg, job.T_ares_base, job.park_q_rad, job.T_flange_tcp, approach_mm=job.approach_mm)


def _full_magazine(job):
    return [s for s in job.magazine.slots if s.id in job.magazine.take_order]


def test_the_park_pose_is_free_and_the_old_one_was_not(cfg, job):
    """[ur] park_q_deg of 2026-10-07: tool 40 mm from the arm, clear of ARES and a full magazine; the value before
    (TCP 50 mm closer to the base, folded IK branch, never checked) had the jaws inside the forearm."""
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


def _corner_stone(cfg, job):
    """World and approach joints for leg A's course-0 stone at the corner with B (the C of 2026-10-08: B runs through
    both corners and is built first; A is built from that corner, its first stone butts against B's ARES-side face,
    1 mm + rib). Before 2026-10-08: B's first stone against the finished leg A."""
    k = next(i for i, s in enumerate(job.stops) if s.leg == "A")
    stop = job.stops[k]
    t = max((s for s in stop.stones if s.course == 0), key=lambda s: s.index)
    assert (t.leg, t.course, t.index) == ("A", 0, 4) and t is stop.stones[0]      # set first: from the corner
    T_base_wall = g.inv(np.asarray(job.T_ares_base)) @ g.inv(stop.ares.T)
    placed = [s for st in job.stops[:k] for s in st.stones]
    world = GuardWorld(magazine=[], wall_stones=placed, legs=job.legs, T_base_wall=T_base_wall)
    T_above = (T_base_wall @ g.transl(0.0, 0.0, job.approach_mm) @ t.T_wall_tcp @ g.inv(job.T_flange_tcp))
    q_above = ik_near(T_above, np.asarray(t.qnear_rad))
    column = T_base_wall @ t.T_wall_tcp                         # the target TCP pose
    return world, q_above, column


def test_the_held_stone_may_touch_its_butt_joint_only_in_the_descent_column(cfg, job):
    """Main config (the C of 2026-10-08): A's corner stone is placed against B's ARES-side face (1 mm + rib); B is
    complete, so at its approach pose the held stone already hangs beside B's higher courses. In its descent column
    touching is allowed, overlapping is not."""
    mg = _guard(cfg, job)
    world, q_above, column = _corner_stone(cfg, job)
    mg.set_world(world)
    assert mg.plan(job.park_q_rad, q_above, "full", column=column).ok
    v = mg.plan(job.park_q_rad, q_above, "full")
    assert not v.ok and "held stone" in v.problems[0] and "(wall)" in v.problems[0]
    from mauer.simworld import ur5_fk
    T_ft = np.asarray(job.T_flange_tcp)
    T_tcp = ur5_fk(q_above) @ T_ft
    T_bw = world.T_base_wall
    p_wall = g.apply(g.inv(T_bw), [T_tcp[:3, 3]])[0]
    p_new = g.apply(T_bw, [p_wall + np.array([30.0, 0.0, 0.0])])[0]        # 30 mm towards leg B's face (wall +x)
    T_new = T_tcp.copy()
    T_new[:3, 3] = p_new
    q_in = ik_near(T_new @ g.inv(T_ft), q_above)
    assert q_in is not None
    assert mg.state_problems(q_in, "full") != []
    v = mg.plan(job.park_q_rad, q_in, "full", column=T_new)
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
    assert c["ur"]["park_q_deg"][0] == pytest.approx(71.83)     # 161.83 until [ur5] mount_rz 0 -> 90 (2026-10-08)


def test_held_stone_edges_floor_and_column_window(cfg, job):
    """Review 2026-10-07: the capsule grid missed the stone corners by 14.6 mm (now the 12 edges are segments), the
    floor was no obstacle, and the descent-column exemption had no height limit."""
    from mauer import armcheck
    from mauer.simworld import ur5_fk
    mg = _guard(cfg, job)
    caps = mg._held_capsules("full")
    T_tf = g.inv(np.asarray(job.T_flange_tcp))
    pts = np.array([g.apply(T_tf, [c[1], c[2]]) for c in caps]).reshape(-1, 3)        # TCP frame
    L, W, H = cfg["brick"]["length"], cfg["brick"]["width"] + 2 * cfg["brick"]["rib_mm"], cfg["brick"]["height"]
    for corner in [(sx * L / 2, sy * W / 2, z) for sx in (-1, 1) for sy in (-1, 1) for z in (0.0, H)]:
        assert float(np.min(np.linalg.norm(pts - np.array(corner), axis=1))) < 1e-6, corner
    # the floor: the park joints with the shoulder lowered until the TCP is below the floor
    mg.set_world(GuardWorld())
    q = np.asarray(job.park_q_rad, float).copy()
    T_ares = np.asarray(job.T_ares_base)
    for d in np.radians(np.arange(0.0, 120.0, 5.0)):
        qq = q.copy()
        qq[1] += d
        if (T_ares @ ur5_fk(qq) @ np.asarray(job.T_flange_tcp))[2, 3] < -20.0:
            assert any("floor" in p for p in mg.state_problems(qq)), np.degrees(qq)
            break
    else:
        pytest.skip("no pose below the floor found")
    # the column: 300 mm above a butt-joint place the held stone is no longer exempt
    world, q_above, column = _corner_stone(cfg, job)
    mg.set_world(world)
    mg._column = column
    assert mg._in_column(q_above)
    high = mg.lift(q_above, 300.0)
    assert high is not None and not mg._in_column(high)
    mg._column = None


def test_a_run_started_later_knows_the_wall_that_stands(cfg, job, tmp_path):
    """Review 2026-10-07: a fresh Sequencer started at stop k > 0 had an empty guard world. Now the stones of the
    earlier stops count as built (and stones declared from an earlier run log are skipped and built)."""
    from mauer.sequencer import Sequencer
    from mauer.simworld import SimWorld, scenario
    first_b = job.stops[1].stones[:2]
    w = SimWorld(cfg, job, scenario("none"), seed=1, start_stop=1, guard=True, standing=[t.key for t in first_b])
    seq = Sequencer(job, cfg, w.robot, w.ares, w.camera, w.intr, w.T_flange_cam, log_dir=tmp_path / "run",
                    on_station_empty=w.refill_station)
    seen = []
    orig = w.robot.guard.set_world
    w.robot.guard.set_world = lambda world: (seen.append(len(world.wall_stones)), orig(world))
    with pytest.raises(ValueError, match="not stones of this job"):
        seq.declare_placed([("Z", 0, 0)])
    seq.declare_placed([t.key for t in first_b], source="test")
    res = seq.run(1, 1)
    n_a = len(job.stops[0].stones)
    assert res.state == "done" and min(seen) >= n_a + len(first_b)
    assert not any(k in res.placed for k in (t.key for t in first_b))              # declared stones are skipped


def test_the_shortest_free_detour_wins(cfg, job, monkeypatch):
    """Samuel 2026-10-09 ("keine unnötigen Wege"): when the direct move is blocked the guard takes the free detour
    with the least joint travel (motionguard.travel_deg ~ movej time), not the park pose because it comes first in
    the list (world sim of the C: 43 of 134 picks / places went through the park pose, +20 % joint travel). The job's
    own vias stay first, park_last still puts the park detours behind the others."""
    from mauer import motionguard as mgm
    mg = _guard(cfg, job)
    q_from = np.asarray(job.magazine.slot(job.magazine.take_order[0]).qnear_rad, float)
    q_to = np.asarray(next(t for t in job.stones() if t.qnear_rad).qnear_rad, float)
    monkeypatch.setattr(mg, "state_problems", lambda q, holding=None, **k: [])
    monkeypatch.setattr(mg, "path_problems", lambda qs, holding=None, **k: ["blocked"] if len(qs) == 2 else [])
    cands = mg.candidates(q_from, q_to)
    assert [] in cands and any(any(np.allclose(v, mg.park_q) for v in c) for c in cands)
    travel = [mgm.travel_deg([q_from, *c, q_to]) for c in cands]
    assert travel == sorted(travel)                                            # shortest first
    v = mg.plan(q_from, q_to, "full")
    assert v.ok and mgm.travel_deg([q_from, *v.vias, q_to]) == pytest.approx(min(t for c, t in zip(cands, travel) if c))
    via = [np.radians([10.0, -80.0, 60.0, -70.0, -90.0, 0.0])]                 # the job's vias: still tried first
    assert all(np.allclose(a, b) for a, b in zip(mg.candidates(q_from, q_to, via)[0], via))
    mg.park_last = True
    pk = [any(np.allclose(v, mg.park_q) for v in c) for c in mg.candidates(q_from, q_to)]
    assert pk == sorted(pk)                                                    # every park detour behind the others


# ── look moves (2026-10-18) ──────────────────────────────────────────────────
def _tcp_of(job, q):
    from mauer.simworld import ur5_fk
    return (ur5_fk(np.asarray(q, float)) @ np.asarray(job.T_flange_tcp))[:3, 3]


def _blends_do_not_overlap(job, path, radii):
    """Every blend sphere stays within its two moves and never reaches the neighbouring one (the UR would skip it)."""
    P = [_tcp_of(job, q) for q in path]
    r = [0.0, *radii, 0.0]
    return all(r[i] + r[i + 1] < float(np.linalg.norm(P[i + 1] - P[i])) + 1e-9 for i in range(len(P) - 1))


@pytest.fixture(scope="module")
def look_moves(cfg, job):
    moves, warnings = make_job.look_moves(cfg, job)
    return moves, warnings


def test_every_look_move_leaves_a_code_upwards_and_is_blended(cfg, job, look_moves):
    """Samuel 2026-10-18 ("die Bewegungen zu den Codes und weg von den Codes ... am besten nach oben wegfahren, sodass
    es sinnvolle und effiziente Bewegungen gibt"): every move of the job's measurements - park -> look -> look -> park
    at every stop and at the station, in the worst-case world - is free, leaves a look upwards and comes down onto the
    next one from above (the via next to a look >= 50 mm higher, straight above it or slanted towards the UR base by
    at most motionguard.LOOK_SLANT - a code at the foot of the high wall), its vias are blended (the arm stops only at
    the looks and the park pose) and no two blends overlap."""
    from mauer.motionguard import LOOK_SLANT
    moves, warnings = look_moves
    n_looks = sum(len(s.looks) for s in job.stops) + len(job.station.looks)
    assert warnings == [] and len(moves) == n_looks + len(job.stops) + 1          # into every look, + back to park
    mg = _guard(cfg, job)
    worlds = {w: world for w, _, world in make_job.look_worlds(cfg, job)}
    for m in moves:
        path = [np.asarray(q) for q in m["path"]]
        mg.set_world(worlds[m["where"]])
        assert mg.path_problems(path) == [], (m["where"], m["from"], m["to"])
        assert len(m["blend_mm"]) == len(path) - 2 and all(r > 0.0 for r in m["blend_mm"]), m
        assert _blends_do_not_overlap(job, path, m["blend_mm"])
        for end, via, is_look in ((path[0], path[1], m["from"] != "park"), (path[-1], path[-2], m["to"] != "park")):
            if is_look:
                a, b = _tcp_of(job, end), _tcp_of(job, via)
                assert b[2] > a[2] + 50.0, (m["where"], m["from"], m["to"])
                assert float(np.hypot(*(b - a)[:2])) <= max(LOOK_SLANT) * (b[2] - a[2]) + 1.0, (m["from"], m["to"])


def test_look_moves_are_faster_than_the_stop_and_go_detours(cfg, job, look_moves):
    """Run 2026-10-16_182116: the look and park moves took 577 of the tour's 847 s; the two station looks were 19..22 s
    apart (a 5-via detour that stops at every via). The blended look moves of the station take less than 2/3 of the
    joint-move time the guard's detours (unblended, as before) take in the same world."""
    from mauer.motionguard import move_time_s
    moves, _ = look_moves
    mg = _guard(cfg, job)
    world = next(w for name, _, w in make_job.look_worlds(cfg, job) if name == "station")
    mg.set_world(world)
    v, a = float(cfg["ur"]["v_joint"]), float(cfg["ur"]["a_joint"])
    new = old = 0.0
    for m in (m for m in moves if m["where"] == "station"):
        q0, q1 = np.asarray(m["path"][0]), np.asarray(m["path"][-1])
        before = mg.plan(q0, q1, None, vias_first=False)
        assert before.ok and before.blend_mm == []
        old += move_time_s([q0, *before.vias, q1], v, a)
        new += m["time_s"]
    assert new < 2.0 / 3.0 * old, (new, old)


def test_blend_radii_stay_inside_short_moves(cfg, job):
    """A short move between two vias (a lift step) gets a small radius: BLEND_FRAC of it; the ends are exact."""
    from mauer import motionguard as mgm
    mg = _guard(cfg, job)
    q0 = np.asarray(job.park_q_rad, float)
    assert mgm.blend_radii([q0, q0, q0], job.T_flange_tcp) == [0.0]       # zero-length moves: no blend
    lk = job.stops[0].looks[0]
    q = ik_near(np.asarray(lk.T_base_flange), np.asarray(lk.qnear_rad))
    up1, up2 = mg.lift(q, 40.0), mg.lift(q, 400.0)
    r = mgm.blend_radii([q, up1, up2], job.T_flange_tcp)
    assert r[0] == pytest.approx(mgm.BLEND_FRAC * 40.0, rel=1e-3)
    r = mgm.blend_radii([q, up2, q], job.T_flange_tcp)
    assert r[0] == pytest.approx(mgm.BLEND_MM)


def test_ur_robot_sends_blended_look_moves_and_parks_upwards(cfg, job):
    """URRobot: a look move's vias go out as movej with the guard's blend radius (r in m), after the controller's IK
    check of the look pose and before the final move to it; the park after a look asks the guard for a look move (up
    first), a park after anything else does not."""
    import types
    from mauer.backends import URRobot
    c = copy.deepcopy(cfg)
    c["ur"]["payload_tool_kg"], c["brick"]["mass_kg"], c["half_brick"]["mass_kg"] = 1.5, 3.0, 1.5
    mg = _guard(c, job)
    calls = []
    q_via = np.radians([40.0, -80.0, 120.0, -130.0, -90.0, 110.0])

    def plan(q_from, q_to, holding=None, vias=(), column=None, vias_first=True, look=False):
        calls.append(look)
        return types.SimpleNamespace(ok=True, vias=[q_via, q_via + 0.1] if look else [], problems=[],
                                     blend_mm=[45.0, 22.5] if look else [])
    mg.plan = plan

    class _State:
        actual_q = list(job.park_q_rad)
        actual_qd = [0.0] * 6
        program_running = False

    class _Link:
        reg_error = 26
        blocks = []

        def state(self):
            return _State()

        def state_age_s(self):
            return 0.0

        def run_block(self, body, name, timeout_s, settle_s=0.0):
            self.blocks.append((name, body))
            return types.SimpleNamespace(ok=True, error=None)

    link = _Link()
    rob = URRobot(link, c, job, guard=mg)
    rob.goto_look(job.stops[0].looks[0])
    name, body = link.blocks[-1]
    lines = body.splitlines()
    moves = [i for i, ln in enumerate(lines) if ln.startswith("movej(")]
    guard_line = next(i for i, ln in enumerate(lines) if "get_inverse_kin" in ln and not ln.startswith("movej("))
    assert name == "mauer_look" and guard_line < moves[0] and len(moves) == 3
    assert lines[moves[0]].endswith(", r=0.045)") and lines[moves[1]].endswith(", r=0.0225)")
    assert "get_inverse_kin(look" in lines[moves[2]] and ", r=" not in lines[moves[2]]
    rob.park()
    assert calls == [True, True] and ", r=" in link.blocks[-1][1]
    rob.park()                                                   # from the park pose: no look move
    assert calls[-1] is False
