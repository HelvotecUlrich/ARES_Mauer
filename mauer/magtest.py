"""Magazine dry run on ARES (Samuel 2026-10-08, [magtest] in config/station.toml, operator sheet docs/MAGTEST_DE.md).

ARES stands still - no ADS move, no camera, no wall. Two full stones are moved through the magazine: every MOVE picks
one stone from a magazine slot, takes it forward over a place pose of the front leg, lowers it to [magtest] hover_mm
above the place height, holds dwell_s and comes back up WITHOUT opening the jaws (URRobot.dry_place), then puts it
down in the next slot (a real open, [ur] release_above_mm above the slot pose). Picks and put-downs approach the
slot straight from above, from a height where the held stone clears every other magazine stack by [magtest]
approach_clear_mm (magazine_approach_mm); on the way the motion guard keeps the held stone [magtest] stone_clear_mm
from every stone and tries the detours without the park pose first (Samuel 2026-10-09: with the job's 150 mm approach
the held stone hung 6.5 mm above the neighbour's pins and the guard sent the arm round the park pose). The plan comes
from tools/make_magtest.py as a job file (mauer.job, meta "kind" "magtest"): one stop with ARES at the origin (wall frame = ARES frame), one StoneTask per move (its place pose
T_wall_tcp, its pick slot `slot`), meta["magtest"]["moves"] = [{"key", "from", "to"}, ...] in run order.

    is_magtest(job)                      the job is a magazine dry run
    preflight(cfg, job) -> [problems]    reasons not to run it on the real robot (UR values only; ARES and the camera
                                         take no part); warnings(cfg, job) -> [notes] shown but not blocking
    MagazineTest(Sequencer)              the runner: same constructor and hooks as the Sequencer (step confirmations,
                                         pause, run log, held stone, motion guard world), so the Mauer HMI runs it
                                         like a wall job (hmi/core/run_controller.py)

Pause and Abort act between moves (empty jaws). A robot error inside a move leaves `held` set: the operator takes
the stone out, puts it back on the pick slot of that move ("from"), parks the arm and confirms empty jaws
(clear_held) - the magazine is then as before the move and the run repeats it.
"""
from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from . import geometry as g
from .config import grasp_above_top_mm
from .job import Job
from .sequencer import RunResult, Sequencer, SequencerError, SequencerPaused

KIND = "magtest"

# config keys whose wrong value moves the arm without any correction in this test (no camera): a PLACEHOLDER blocks
BLOCK_IF_PLACEHOLDER = ("[ur5] mount_x", "[ur5] mount_y", "[ur5] mount_z", "[ur5] mount_rz", "[ares] deck_top_z",
                        "[deck] holder_z", "[tool] tcp_z")
WARN_IF_PLACEHOLDER = {"[ur] payload_cog_mm": "the UR payload model (its collision detection) - keep the speed low"}


def pendant_check(cfg: Mapping) -> tuple[str, str]:
    """(X, Y): where the TCP moves on the pendant (Move tab, feature Base) when the shown X / Y grows, seen in the ARES
    driving direction - FORWARD / BACKWARD / LEFT / RIGHT - from [ur5] mount_rz and [ares] frame_x_points_to (this
    repo's +x is the vehicle rear when "rear"). The start checklist asks the operator to confirm it."""
    rz = np.radians(float(cfg["ur5"]["mount_rz"]))
    flip = -1.0 if str((cfg.get("ares") or {}).get("frame_x_points_to", "front")) == "rear" else 1.0
    out = []
    for v in ((np.cos(rz), np.sin(rz)), (-np.sin(rz), np.cos(rz))):         # base +X, +Y in this repo's ARES frame
        x, y = flip * v[0], flip * v[1]                                       # in the vehicle frame (base_link)
        names = [n for n, ok in (("FORWARD", x > 0.98), ("BACKWARD", x < -0.98), ("LEFT", y > 0.98),
                                 ("RIGHT", y < -0.98)) if ok]
        out.append(names[0] if names else f"between the vehicle axes ({np.degrees(np.arctan2(y, x)):.0f} deg from "
                                         "forward towards left)")
    return out[0], out[1]


def magazine_approach_mm(cfg: Mapping, job: Job, filled, kinds: Mapping[str, str], sid: str, kind: str,
                         z_tcp: float, approach_mm: float) -> float:
    """Approach / retract height [mm] above the TCP pose of magazine slot `sid` (TCP z_tcp in the ARES frame: the slot
    pose, or the release pose of a put-down) so that a held stone of `kind` hanging there clears the top (pins
    included) of every stone in the OTHER stacks (`filled` slot ids, their types in `kinds`) by [magtest]
    approach_clear_mm;
    at least approach_mm (the job's). The held stone's top face lies config.grasp_above_top_mm below the TCP (a half
    stone is held 20 mm higher, 2026-10-09), its body below that, its pins above it."""
    b, hb = cfg["brick"], cfg.get("half_brick", {}) or {}
    clear = float(cfg.get("magtest", {}).get("approach_clear_mm", 0.0))
    height = {"full": float(b["height"]), "half": float(hb.get("height", b["height"]))}
    pins = {"full": float(b.get("pin_length", 0.0)), "half": float(hb.get("pin_length", b.get("pin_length", 0.0)))}
    stack = job.magazine.slot(sid).stack
    tops = [float(s.T_ares_tcp[2, 3]) + pins.get(kinds.get(k, "full"), pins["full"])
            for k in filled for s in [job.magazine.slot(k)] if s.stack != stack]
    if not tops:
        return float(approach_mm)
    bottom_at_slot = (float(z_tcp) - grasp_above_top_mm(dict(cfg), kind)      # held stone bottom with the TCP at z_tcp
                      - height.get(kind, height["full"]))
    return max(float(approach_mm), max(tops) + clear - bottom_at_slot)


def is_magtest(job: Job | None) -> bool:
    return job is not None and (job.meta or {}).get("kind") == KIND


def _status(config_path: str | Path | None, variant: str | None) -> dict:
    from .job import config_status
    try:
        return config_status(config_path, variant)
    except OSError:
        return {}


def preflight(cfg: Mapping, job: Job, *, config_path: str | Path | None = None) -> list[str]:
    """Every reason not to run the dry run on the real robot (empty = ok). The UR link, the payload and the values
    the magazine and front poses are computed from; nothing about ARES moves, boards, calibration or RoboDK."""
    from .job import config_sha256
    p: list[str] = []
    if not is_magtest(job):
        return [f"not a magazine dry run (meta kind {job.meta.get('kind')!r})"]
    u, b = cfg.get("ur", {}), cfg.get("brick", {})
    if not str(u.get("host", "")).strip():
        p.append("[ur] host is empty (PLACEHOLDER) - set the UR5 IP")
    tool = float(u.get("payload_tool_kg", 0.0) or 0.0)
    stone = float(b.get("mass_kg", 0.0) or 0.0)
    if tool <= 0.0:
        p.append("[ur] payload_tool_kg <= 0 (PLACEHOLDER) - weigh gripper + adapter + camera")
    if stone <= 0.0:
        p.append("[brick] mass_kg <= 0 (UNKNOWN) - weigh a stone")
    if tool > 0.0 and stone > 0.0 and tool + stone > 5.0:
        p.append(f"payload {tool + stone:.2f} kg (tool + stone) exceeds the UR5 rated payload 5 kg")
    for key in ("do_grip_open", "do_grip_close"):
        if u.get(key) is None:
            p.append(f"[ur] {key} missing")
    variant = job.meta.get("config_variant") or None
    if (cfg.get("_variant") or None) != variant:
        p.append(f"config variant {cfg.get('_variant')!r} loaded, the test was built with {variant!r}")
    status = _status(config_path, variant)
    for key in BLOCK_IF_PLACEHOLDER:
        st = status.get(key, {}).get("status")
        if st in ("PLACEHOLDER", "UNKNOWN"):
            p.append(f"{key} {st} - measure it (moves the magazine and front poses 1:1, no camera correction)")
    try:
        if job.config_sha256 and config_sha256(config_path, variant) != job.config_sha256:
            p.append("config/station.toml changed since the test was built - rebuild it: py.exe tools/make_magtest.py")
    except OSError as e:
        p.append(f"config not readable: {e}")
    if dict(cfg.get("magtest", {})) != job.meta["magtest"].get("config"):
        p.append("[magtest] changed since the test was built - rebuild it: py.exe tools/make_magtest.py")
    return p


def warnings(cfg: Mapping, job: Job, *, config_path: str | Path | None = None) -> list[str]:
    """Not blocking, shown in the preflight list: PLACEHOLDER values that do not move the arm."""
    status = _status(config_path, job.meta.get("config_variant") or None)
    return [f"{key} {status[key]['status']}: {what}" for key, what in WARN_IF_PLACEHOLDER.items()
            if status.get(key, {}).get("status") in ("PLACEHOLDER", "UNKNOWN")]


class MagazineTest(Sequencer):
    """The dry run (module docstring) on the Sequencer's machinery. run() ignores the stop arguments: it continues
    with the first move not done yet. `placed` holds the keys of the moves done (the HMI shows them as progress); the
    motion guard sees the magazine only - nothing stands in front of ARES."""

    def __init__(self, job: Job, cfg: Mapping, robot, ares, camera, intr, T_flange_cam,
                 log_dir: str | Path | None = None, confirm: Callable[[str], bool] | None = None,
                 now: Callable[[], float] = time.time, **kw: Any):
        if not is_magtest(job):
            raise ValueError("MagazineTest needs a magtest job (tools/make_magtest.py)")
        kw["camera_loop"] = False                    # nothing is measured
        super().__init__(job, cfg, robot, ares, camera, intr, np.eye(4) if T_flange_cam is None else T_flange_cam,
                         log_dir, confirm, now, **kw)
        m = job.meta[KIND]
        self.hover_mm, self.dwell_s = float(m["hover_mm"]), float(m["dwell_s"])
        self.moves = {tuple(mv["key"]): mv for mv in m["moves"]}
        self.n_moves = len(m["moves"])
        self.move_from: str | None = None            # pick slot of the move in progress (clear_held puts it back)
        guard = getattr(robot, "guard", None)
        if guard is not None:                        # Samuel 2026-10-09: no park pose between front and magazine
            guard.park_last = True                   # unless nothing else is clear, and the held stone keeps
            guard.stone_clear_mm = float(cfg.get("magtest", {}).get("stone_clear_mm", 0.0))   # its distance

    def _guard_world(self):
        return replace(super()._guard_world(), wall_stones=[])

    def clear_held(self, source: str = "operator") -> None:
        """Operator: the jaws are empty and the stone stands on the pick slot of the interrupted move again (taken out
        of the jaws or out of the put-down slot) - the magazine is as before that move, which runs again."""
        src = self.move_from
        super().clear_held(source)
        if src is not None and src not in self.magazine.filled:
            self.magazine.fill(src, "full")
            self.log.write("magtest_stone_returned", slot=src, source=source)
        self.move_from = None

    def _move(self, i: int, t) -> None:
        mv = self.moves[t.key]
        src, dst = mv["from"], mv["to"]
        tag = f"move {i}/{self.n_moves}"
        if src not in self.magazine.filled or not self.magazine.can_take(src):
            raise SequencerError(f"{tag}: magazine slot {src} is empty or covered - the magazine is not as planned "
                                 f"(filled: {sorted(self.magazine.filled)})", 0, t.key)
        if not self.magazine.can_fill(dst):
            raise SequencerError(f"{tag}: magazine slot {dst} cannot take a stone (taken, or nothing below it)", 0,
                                 t.key)
        mag, filled, kinds = self.job.magazine, self.magazine.filled, self.magazine.kinds
        rel = float(self.cfg["ur"].get("release_above_mm", 0.0))
        a_pick = magazine_approach_mm(self.cfg, self.job, filled - {src}, kinds, src, "full",
                                      float(mag.slot(src).T_ares_tcp[2, 3]), self.job.approach_mm)
        a_put = magazine_approach_mm(self.cfg, self.job, filled - {src}, kinds, dst, "full",
                                     float(mag.slot(dst).T_ares_tcp[2, 3]) + rel, self.job.approach_mm)
        self.log.write("magtest_move", move=i, n_moves=self.n_moves, stone=t.key, slot_from=src, slot_to=dst,
                       u_mm=t.u_mm, course=t.course, approach_pick_mm=a_pick, approach_put_mm=a_put,
                       release_above_mm=rel)
        self.move_from = src
        self._robot("pick_magazine", f"{tag}: pick magazine slot {src} (from {a_pick:.0f} mm above)", mag.slot(src),
                    self.T_base_ares, "full", approach_mm=a_pick)
        self.magazine.take(src)
        self._set_held({"from": "magazine", "slot": src, "kind": "full", "stone": list(t.key), "unknown": False})
        self._robot("dry_place", f"{tag}: dry place in front (u {t.u_mm:+.0f} mm, course {t.course}): down to "
                                 f"{self.hover_mm:g} mm above, hold {self.dwell_s:g} s, jaws stay CLOSED",
                    self.T_base_wall, t, self.hover_mm, self.dwell_s)
        self.log.write("dry_placed", move=i, stone=t.key, u_mm=t.u_mm, course=t.course, hover_mm=self.hover_mm,
                       T_base_tcp=self.T_base_wall @ t.T_wall_tcp)
        self._robot("place_magazine", f"{tag}: put down in magazine slot {dst} (from {a_put:.0f} mm above, let go "
                                      f"{rel:g} mm above the slot)", mag.slot(dst), self.T_base_ares, "full",
                    approach_mm=a_put)
        self.magazine.fill(dst, "full")
        self._set_held(None)
        self.move_from = None
        self.placed.add(t.key)
        self.result.placed.append(t.key)
        self.log.write("magtest_move_done", move=i, stone=t.key, slot_from=src, slot_to=dst,
                       done=len(self.placed), n_moves=self.n_moves)

    def run(self, start_stop: int = 0, stop_after: int | None = None) -> RunResult:
        if self.paused:
            raise SequencerPaused("run is paused - resume() first", 0)
        if self.held is not None:
            raise SequencerError(f"a stone may be in the jaws ({self.held}) - take it out, put it back on magazine "
                                 f"slot {self.move_from or self.held.get('slot')}, park the arm, then clear_held() "
                                 "('Jaws empty') and resume", 0)
        stop = self.job.stops[0]
        self.result.state, self.result.error = "running", None
        self.log.write("run_start", kind=KIND, moves_done=len(self.placed), n_moves=self.n_moves,
                       hover_mm=self.hover_mm, dwell_s=self.dwell_s, magazine=sorted(self.magazine.filled),
                       job_source=self.job.source, config_sha256=self.job.config_sha256)
        self.stop_k = 0
        self.pose_est, self.pose_src, self.pose_status = stop.ares, "ARES stands (magazine dry run)", "ok"
        self.T_base_wall = self.T_base_ares @ g.inv(stop.ares.T)     # ARES at the origin: wall frame = ARES frame

        def body() -> RunResult:
            self._park()
            for i, t in enumerate(stop.stones, 1):
                if t.key not in self.placed:
                    self._move(i, t)
            self._park()
            self.result.stops_done = [0]
            self.result.state = "done"
            self.log.write("run_done", placed=len(self.placed), magazine=sorted(self.magazine.filled))
            return self.result
        return self._ending(body)
