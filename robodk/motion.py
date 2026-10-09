"""Collision-checked motion planning for pick and place with the UR5 in RoboDK.

Every segment is tested against the current station (ARES, magazine, the wall built so far, the arm itself; the held
stone is part of the robot) with MoveJ_Test / MoveL_Test before it is executed. Only the last `engage` mm of a
descent (jaws around the stone, pins entering the sockets) and the first `engage` mm of the retreat are contact
phases by design and not tested.

Tool against the own arm: RoboDK checks the gripper, camera body and camera adapter against the UR5 links, but its
meshes report the first contact only about 45 mm deeper than the capsule model mauer.armcheck.self_clearance (probe
2026-10-07: no RoboDK collision at model -17 mm), and the real camera adapter touched wrist 1 on 2026-10-06 at model
-3.7 mm. Every pose and every move (contact phases included) is therefore also checked with that model: tool >=
armcheck.SELF_CLEARANCE_MM from the links, sampled every SELF_STEP_DEG along joint moves.

Strategy for a move to a target with vertical approach:
  1. all IK solutions of the target (both gripper orientations), sorted by joint distance from the current joints;
  2. per solution: approach point above, pre-engage point; the vertical line approach -> pre-engage must be free;
  3. transfer current -> approach: direct joint move, else via safe waypoints (lift to the safe height, rotate the
     base at that height, compact pose near the base); the first collision-free variant in cost order wins.
"""
from __future__ import annotations

import math
import random

from rdk_common import PI, tcp_pose, transl, ur5_base_pose
from mauer import armcheck
from robodk.robolink import COLLISION_OFF, COLLISION_ON, ITEM_TYPE_OBJECT
from robodk.robomath import invH, rotx, rotz

W = (2.0, 2.0, 1.5, 1.0, 1.0, 0.5)         # joint weights for the path cost (base and shoulder move most mass)
STEP_DEG = 1.0                              # collision test resolution of joint moves (RoboDK default 4° missed a
STEP_MM = 2.0                               # forearm/stone contact) and of linear moves
SELF_STEP_DEG = 2.0                         # sampling of the tool-vs-arm model along joint moves (module docstring)


def jdist(a, b) -> float:
    return sum(w * abs(x - y) for w, x, y in zip(W, a, b))


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def family(j) -> bool:
    """Preferred UR configuration for pick and place (angles wrapped to ±180°): upper arm leaning towards the target
    (shoulder -135..+10°, never reaching back over the top of the robot), elbow up, wrist pointing down. Staying in
    this family keeps joint moves from swinging the gripper through the forearm, and moves between magazine and
    wall become a turn of the base joint."""
    return -135.0 < wrap(j[1]) < 10.0 and wrap(j[2]) > 5.0 and wrap(j[4]) < -5.0


def rank(sols, ref) -> list:
    """IK solutions of the preferred family, by joint distance to ref (other configurations are not used)."""
    return sorted((s for s in sols if family(s[0])), key=lambda s: jdist(s[0], ref))


class Planner:
    def __init__(self, RDK, cfg: dict, robot, tool, f_ares, engage: float = 60.0):
        self.RDK, self.cfg, self.robot, self.tool, self.f_ares = RDK, cfg, robot, tool, f_ares
        self.tcp = tcp_pose(cfg)
        self.ref = invH(ur5_base_pose(cfg))
        self.base_xy = (cfg["ur5"]["mount_x"], cfg["ur5"]["mount_y"])
        self.engage = engage
        self.approach = cfg["study"]["approach"]
        self.z_safe = 900.0                  # TCP height for transfers over the magazine (updated by the caller)
        self.tests = 0
        self.self_rejects = 0                # poses / moves refused by the tool-vs-arm model only
        self.on_move = None                  # called after every executed move (simulate.py --video: a frame)
        robot.setPoseFrame(f_ares)
        robot.setPoseTool(tool)

    # ── kinematics ───────────────────────────────────────────────────────────
    def ik(self, pose, seed):
        j = self.robot.SolveIK(pose, seed, tool=self.tcp, reference=self.ref).list()
        return j[:6] if len(j) >= 6 else None

    def ik_all(self, pose) -> list:
        out = []
        for p in (pose, pose * rotz(PI)):
            sols = self.robot.SolveIK_All(p, tool=self.tcp, reference=self.ref)
            if sols.size(0) >= 6:
                for c in range(sols.size(1)):
                    out.append(([sols[r, c] for r in range(6)], p))
        return out

    def fk(self, j):
        """TCP pose in the ARES frame for joints j."""
        return invH(self.ref) * self.robot.SolveFK(j) * self.tcp

    # ── collision tests (robot is put back afterwards) ───────────────────────
    def self_ok(self, j) -> bool:
        """Tool >= armcheck.SELF_CLEARANCE_MM from the arm's own links at joints j [deg] (capsule model)."""
        return armcheck.self_clearance([math.radians(a) for a in j], self.cfg)[0] >= armcheck.SELF_CLEARANCE_MM

    def self_why(self, j) -> str:
        """'' if self_ok(j), else the closest tool part / link of the capsule model (for diagnoses)."""
        d, part, link = armcheck.self_clearance([math.radians(a) for a in j], self.cfg)
        return "" if d >= armcheck.SELF_CLEARANCE_MM else f"{part} vs {link} {d:.0f} mm (tool-vs-arm model)"

    def self_free_j(self, j1, j2) -> bool:
        """self_ok along the joint move j1 -> j2 (every SELF_STEP_DEG); short linear moves are taken as joint moves."""
        n = max(1, math.ceil(max(abs(a - b) for a, b in zip(j1, j2)) / SELF_STEP_DEG))
        ok = all(self.self_ok([a + (b - a) * i / n for a, b in zip(j1, j2)]) for i in range(n + 1))
        self.self_rejects += not ok
        return ok

    def free_j(self, j1, j2) -> bool:
        self.tests += 1
        if not self.self_free_j(j1, j2):
            return False
        r = self.robot.MoveJ_Test(j1, j2, STEP_DEG)
        return r == 0

    def free_l(self, j1, pose) -> bool:
        self.tests += 1
        j2 = self.ik(pose, j1)
        if j2 is not None and not self.self_free_j(j1, j2):
            return False
        r = self.robot.MoveL_Test(j1, pose, STEP_MM)
        return r == 0

    def lifted(self, j, z_target: float):
        """Joints of the pose of j lifted vertically to z_target (or the highest reachable below), else None."""
        p = self.fk(j)
        z0 = p.Pos()[2]
        h = z_target - z0
        while h > 0:
            jj = self.ik(transl(0, 0, h) * p, j)
            if jj and max(abs(a - b) for a, b in zip(jj, j)) < 120:
                return jj
            h -= 50.0
        return None

    def state_free(self, j) -> bool:
        """True if the arm (with gripper and held stone) is collision-free at joints j (RoboDK and self_ok)."""
        if not self.self_ok(j):
            self.self_rejects += 1
            return False
        self.robot.setJoints(j)
        self.RDK.Update()                    # needed while rendering is off, else the link poses are stale
        return self.RDK.Collisions() == 0

    def compact(self, theta_deg: float, seed):
        """Compact, collision-free transfer pose near the UR base in direction theta: TCP at z_safe, tool down,
        tool yaw as in seed. Among all arm configurations the one closest to seed is taken."""
        R = self.fk(seed)
        yaw = math.atan2(R[1, 0], R[0, 0])
        for r in (300.0, 250.0, 350.0, 200.0, 400.0):
            x = self.base_xy[0] + r * math.cos(math.radians(theta_deg))
            y = self.base_xy[1] + r * math.sin(math.radians(theta_deg))
            pose = transl(x, y, self.z_safe) * rotz(yaw) * rotx(PI)
            for j, _ in rank(self.ik_all(pose), seed):
                if self.state_free(j):
                    return j
        return None

    # ── planning ─────────────────────────────────────────────────────────────
    def transfer(self, j_from, j_to) -> list | None:
        """Collision-free joint path j_from -> j_to (list of intermediate joints, possibly empty)."""
        with self._planning():
            return self._transfer(j_from, j_to)

    def _transfer(self, j_from, j_to) -> list | None:
        if not self.state_free(j_from):
            pairs = [(p[0].Name(), p[2], p[1].Name(), p[3]) for p in self.RDK.CollisionPairs()]
            raise RuntimeError(f"start of the transfer is already in collision: {pairs}")
        cands = [[]]
        up_from = self.lifted(j_from, self.z_safe)
        up_to = self.lifted(j_to, self.z_safe)
        if up_from:
            cands.append([up_from])
            if up_to:
                cands.append([up_from, up_to])
            rot = list(up_from)
            rot[0] = j_to[0]
            cands.append([up_from, rot])
            if up_to:
                cands.append([up_from, rot, up_to])
        comp_from = self.compact(self._theta(j_from), j_from)
        comp_to = self.compact(self._theta(j_to), j_to)
        if comp_from and comp_to:
            cands.append([comp_from, comp_to])
            if up_from:
                cands.append([up_from, comp_from, comp_to])
        cands.sort(key=lambda path: sum(jdist(a, b) for a, b in zip([j_from] + path, path + [j_to])))
        for path in cands:
            seq = [j_from] + path + [j_to]
            if all(self.free_j(a, b) for a, b in zip(seq, seq[1:])):
                return path
        if self.state_free(j_to):
            return self.rrt(j_from, j_to)                     # general fallback
        return None

    # ── general fallback: RRT-Connect in joint space ─────────────────────────
    def rrt(self, j_start, j_goal, max_iter: int = 300, step: float = 20.0, seed: int = 1):
        """Joint-space RRT-Connect: every node is checked with state_free, every edge with MoveJ_Test (1°).
        Samples stay in the preferred configuration family and near the start/goal joint ranges.
        Returns the list of intermediate joints (shortcut) or None."""
        rnd = random.Random(seed)
        margin = (150.0, 60.0, 60.0, 120.0, 30.0, 120.0)
        lo = [max(-360.0, min(a, b) - m) for a, b, m in zip(j_start, j_goal, margin)]
        hi = [min(360.0, max(a, b) + m) for a, b, m in zip(j_start, j_goal, margin)]

        def sample():
            for _ in range(200):
                q = [rnd.uniform(l, h) for l, h in zip(lo, hi)]
                if family(q):
                    return q
            return [rnd.uniform(l, h) for l, h in zip(lo, hi)]

        def steer(a, b):
            d = max(abs(x - y) for x, y in zip(a, b))
            if d <= step:
                return list(b)
            f = step / d
            return [x + f * (y - x) for x, y in zip(a, b)]

        def nearest(tree, q):
            return min(range(len(tree)), key=lambda i: jdist(tree[i][0], q))

        def extend(tree, q):
            i = nearest(tree, q)
            qn = steer(tree[i][0], q)
            if family(qn) and self.state_free(qn) and self.free_j(tree[i][0], qn):
                tree.append((qn, i))
                return len(tree) - 1
            return None

        def path_to(tree, i):
            out = []
            while i is not None:
                out.append(tree[i][0])
                i = tree[i][1]
            return out[::-1]

        ta, tb = [(list(j_start), None)], [(list(j_goal), None)]
        for it in range(max_iter):
            ia = extend(ta, sample())
            if ia is not None:
                qa = ta[ia][0]
                while True:                                    # greedy connect of the other tree
                    ib = extend(tb, qa)
                    if ib is None:
                        break
                    if max(abs(x - y) for x, y in zip(tb[ib][0], qa)) < 1e-6:
                        p1, p2 = path_to(ta, ia), path_to(tb, ib)
                        if p1[0] != list(j_start):
                            p1, p2 = p2, p1
                        full = p1 + p2[::-1][1:]
                        return self._shortcut(full)[1:-1]
            ta, tb = tb, ta
        return None

    def _shortcut(self, path: list) -> list:
        """Remove waypoints where a direct collision-free joint move exists (greedy, from the start)."""
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.free_j(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def _theta(self, j) -> float:
        """Direction (deg, ARES frame) from the UR base to the TCP of joints j."""
        p = self.fk(j).Pos()
        return math.degrees(math.atan2(p[1] - self.base_xy[1], p[0] - self.base_xy[0]))

    def _planning(self):
        """Context: no rendering, robot joints restored afterwards (the collision tests move the robot)."""
        planner = self

        class _Ctx:
            def __enter__(self_):
                self_.j = planner.robot.Joints()
                planner.RDK.Render(False)

            def __exit__(self_, *exc):
                planner.robot.setJoints(self_.j)
                return False
        return _Ctx()

    def plan_to(self, j_from, target, approach: float | None = None, side=None, low: float | None = None):
        with self._planning():
            return self._plan_to(j_from, target, approach, side, low)

    def _plan_to(self, j_from, target, approach: float | None = None, side=None, low: float | None = None):
        """Moves from j_from to the target pose (ARES frame) with a vertical approach:
        returns (moves, j_target) with moves = [("J", joints) | ("L", pose) | ("E", pose)] ("E" = engage, untested).
        side = (dx, dy, dz) mm in the ARES frame, low mm (a wall stone set from the side, Samuel 2026-10-09): the
        approach comes down at the target shifted by `side` (tested down to the engage height), then - untested
        contact phase - to `low` above the shifted target, sideways to `low` above the target and down."""
        app_h = approach or self.approach
        S = transl(*side) if side is not None else None
        sols = rank(self.ik_all(target), j_from)
        for j_t, pose in sols:
            pre = transl(0, 0, self.engage) * pose
            app = transl(0, 0, app_h) * pose
            tail = [("E", pose)]
            chain = [j_t]
            if S is not None:                                  # from the side: pre / app over the side point
                lo_t = transl(0, 0, low) * pose
                lo_s = S * lo_t
                pre, app = S * pre, S * app
                j_lo_s, j_lo_t = self.ik(lo_s, j_t), self.ik(lo_t, j_t)
                if not (j_lo_s and j_lo_t):
                    continue
                tail = [("E", lo_s), ("E", lo_t), ("E", pose)]
                chain = [j_lo_s, j_lo_t, j_t]
            j_pre = self.ik(pre, j_t)
            j_app = self.ik(app, j_t)
            if not (j_pre and j_app):
                continue
            if max(abs(a - b) for a, b in zip(j_app, j_t)) > 60 or max(abs(a - b) for a, b in zip(j_pre, j_t)) > 30:
                continue                                       # configuration change on the vertical line
            if not self.state_free(j_app) or not self.free_l(j_app, pre) or \
                    not all(self.self_free_j(a, b) for a, b in zip([j_pre] + chain, chain)):
                continue                                       # (the engage part: the tool-vs-arm model only)
            path = self._transfer(j_from, j_app)
            if path is None:
                continue
            moves = [("J", j) for j in path] + [("J", j_app), ("L", pre)] + tail
            return moves, j_t, pose
        return None

    def plan_retreat(self, j_at, pose, height: float | None = None):
        """Vertical retreat from pose: engage part untested, rest tested. Returns (moves, j_end)."""
        with self._planning():
            return self._plan_retreat(j_at, pose, height)

    def _plan_retreat(self, j_at, pose, height: float | None = None):
        h = height or self.approach
        pre = transl(0, 0, self.engage) * pose
        app = transl(0, 0, h) * pose
        j_pre = self.ik(pre, j_at)
        j_app = self.ik(app, j_at)
        if not (j_pre and j_app) or not self.self_free_j(j_at, j_pre) or not self.free_l(j_pre, app):
            return None
        return [("E", pre), ("L", app)], j_app

    # ── execution ────────────────────────────────────────────────────────────
    def execute(self, moves) -> None:
        self.RDK.Render(True)
        for kind, target in moves:
            if kind == "J":
                self.robot.MoveJ(target)
            elif kind == "L":
                self.robot.MoveL(target)
            else:                                              # contact phase: collision check paused
                self.RDK.setCollisionActive(COLLISION_OFF)
                self.robot.MoveL(target)
                self.RDK.setCollisionActive(COLLISION_ON)
            if self.on_move is not None:
                self.on_move()


def set_static(RDK, stone, others: list, ares) -> None:
    """A resting stone: no collision checks against ARES and other stones (contacts by design)."""
    items = [ares] + [o for o in others if o != stone]
    RDK.setCollisionActivePairList([COLLISION_OFF] * len(items), [stone] * len(items), items,
                                   [0] * len(items), [0] * len(items))


def set_held(RDK, stone, others: list, ares, robot, tool) -> None:
    """A held stone: checked against ARES, other stones and the arm (links 0-5), not against wrist 3 / gripper."""
    items = [ares] + [o for o in others if o != stone]
    RDK.setCollisionActivePairList([COLLISION_ON] * len(items), [stone] * len(items), items,
                                   [0] * len(items), [0] * len(items))
    for link in range(0, 8):
        RDK.setCollisionActivePair(COLLISION_ON if link <= 5 else COLLISION_OFF, stone, robot, 0, link)
    RDK.setCollisionActivePair(COLLISION_OFF, stone, tool, 0, 0)


def set_released(RDK, stone, robot, tool) -> None:
    """A released stone after the retreat (jaws clear of it): the gripper (tool) and robot links 0-6 are checked
    against it again. set_held switches the gripper / wrist 3 off for the held stone and nothing switched them back
    on, so a placed stone was never checked against the gripper (RoboDK run 2026-10-05; robodk/simulate.py
    Sim.arm_pairs did this as a workaround)."""
    RDK.setCollisionActivePair(COLLISION_ON, stone, tool, 0, 0)
    for link in range(0, 7):
        RDK.setCollisionActivePair(COLLISION_ON, stone, robot, 0, link)


def stones_in_station(RDK) -> list:
    return [o for o in RDK.ItemList(ITEM_TYPE_OBJECT) if o.Name().startswith("Stone_")]
