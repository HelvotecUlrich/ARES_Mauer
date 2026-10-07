"""Backends the sequencer talks to - one interface for the real hardware and the simulation (mauer.simworld).

    Robot   park(), goto_look(look), shot(camera) -> Shot, pick_magazine(slot, T_base_ares, kind="full"),
            place_wall(T_base_wall, stone), pick_station(T_base_station, slot), place_magazine(slot, T_base_ares,
            kind="full"), is_parked(), is_idle(), abort(). kind = stone type "full" / "half" (stone.kind, slot.kind
            for the station holders) - it sets the payload. Every motion is one atomic robot program; a failed one raises RobotError (the arm is then NOT
            parked - the sequencer never moves ARES in that state).
    Ares    translate(dx_mm, dy_mm) / rotate(dtheta_deg) -> mauer.ares.MoveOutcome, preflight() -> [problems],
            status(), idle() -> bool, abort(). Body frame at the start pose: +x forward, +y left, +theta CCW.
    Camera  mauer.camera.Camera (grab() -> Frame); shots are taken through Robot.shot so that the flange pose of the
            exposure comes with the image.

Real implementations:
- `URRobot`: mauer.ur.link.URLink + mauer.ur.script. Every block starts with set_tcp + set_payload (research
  recommendation, mauer/ur/script.py preamble); picks/places with script.pick_stone / place_stone relative to the
  measured frame (pose_trans on the controller), contact speed over the last [ur] contact_mm, gripper pulses on the
  [ur] do_grip_* outputs (DO0 open, DO1 close, confirmed 2026-10-06). IK branch hints (qnear) come from the job,
  else from the nominal UR5 kinematics (mauer.simworld.ik_near) seeded with the current joints - the controller
  solves the real IK.
  Shots: mauer.capture.capture_shot (flange pose = mean of the RTDE samples of the exposure window).
- `AdsAres`: thin adapter around mauer.ares.AresAds (pattern A, amr_hmi owns heartbeat/MANUAL/HALT).

Conventions: docs/ARCHITECTURE.md (T_a_b, mm, rad). Payload: [ur] payload_tool_kg / payload_cog_mm (PLACEHOLDER 0 =
unknown -> URRobot refuses unless sim=True), stone [brick] mass_kg / half stone [half_brick] mass_kg (UNKNOWN 0) with
the centre of gravity half a stone height below the TCP (TCP = top centre of the held stone, z into the stone, the
same for both types).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable

import numpy as np

from . import geometry as g
from .camera.base import Frame

log = logging.getLogger("mauer.backends")


# ── errors ────────────────────────────────────────────────────────────────────
class BackendError(RuntimeError):
    """Base class of backend failures."""


class RobotError(BackendError):
    """A robot program failed (protective stop, IK guard, timeout, stopped without done marker, ...). The arm is
    wherever it stopped - not parked. `result` is the backend's own record (URLink BlockResult) if any."""

    def __init__(self, msg: str, result: Any = None, action: str = ""):
        super().__init__(msg)
        self.result = result
        self.action = action


class ShotError(BackendError):
    """No usable image (camera failure, arm moving during the exposure, no robot pose for the frame)."""


class AresError(BackendError):
    """ARES move failed or was refused (the MoveOutcome / reason is attached)."""

    def __init__(self, msg: str, outcome: Any = None):
        super().__init__(msg)
        self.outcome = outcome


# ── data ──────────────────────────────────────────────────────────────────────
@dataclass
class Shot:
    """Image + flange pose at the exposure. Same fields as mauer.capture.Shot (which the real robot returns); the
    sequencer only uses `frame` and `T_base_flange`."""
    frame: Frame
    T_base_flange: np.ndarray
    q_rad: np.ndarray | None = None
    max_qd: float = 0.0
    n_samples: int = 1
    meta: dict = field(default_factory=dict)


# ── protocols ─────────────────────────────────────────────────────────────────
@runtime_checkable
class Robot(Protocol):
    def park(self) -> Any: ...
    def goto_look(self, look) -> Any: ...
    def shot(self, camera) -> Any: ...
    def pick_magazine(self, slot, T_base_ares: np.ndarray, kind: str = "full") -> Any: ...
    def place_wall(self, T_base_wall: np.ndarray, stone) -> Any: ...
    def pick_station(self, T_base_station: np.ndarray, slot) -> Any: ...
    def place_magazine(self, slot, T_base_ares: np.ndarray, kind: str = "full") -> Any: ...
    def is_parked(self) -> bool: ...
    def is_idle(self) -> bool: ...
    def abort(self) -> Any: ...


@runtime_checkable
class Ares(Protocol):
    def translate(self, dx_mm: float, dy_mm: float) -> Any: ...
    def rotate(self, dtheta_deg: float) -> Any: ...
    def preflight(self) -> list[str]: ...
    def status(self) -> Any: ...
    def idle(self) -> bool: ...
    def abort(self) -> Any: ...


@runtime_checkable
class CameraLike(Protocol):
    def grab(self) -> Frame: ...


# ── payload ───────────────────────────────────────────────────────────────────
def stone_payload(tool_kg: float, tool_cog_mm: Sequence[float], stone_kg: float, T_flange_tcp: np.ndarray,
                  stone_height_mm: float) -> tuple[float, list[float]]:
    """(mass kg, CoG mm in the flange frame) of tool + held stone. Stone CoG = TCP + H/2 along TCP z (TCP = top
    centre of the stone, z into the stone; ASSUMPTION: homogeneous stone, CoG at its geometric centre)."""
    c_stone = g.apply(np.asarray(T_flange_tcp, float), [[0.0, 0.0, stone_height_mm / 2.0]])[0]
    m = float(tool_kg) + float(stone_kg)
    if m <= 0.0:
        return 0.0, [float(v) for v in tool_cog_mm]
    c = (float(tool_kg) * np.asarray(tool_cog_mm, float) + float(stone_kg) * c_stone) / m
    return m, c.tolist()


# ── real robot ────────────────────────────────────────────────────────────────
class URRobot:
    """Robot backend on a started mauer.ur.link.URLink. `job` provides T_flange_tcp, park_q_rad and approach_mm.

    sim=True allows the unknown payload (URSim / simulation only; mauer.capture SIM_PAYLOAD_KG fallback) and is
    never used for the real robot (tools/run_job.py --real refuses it).

    guard (mauer.motionguard.MotionGuard, REQUIRED unless sim, 2026-10-07): every joint move to a look, a park or
    the approach pose of a pick / place is checked from the actual joints first; a blocked move gets the guard's
    detour vias, a move without a clear path is refused (RobotError) before anything moves. The sequencer sets the
    guard's world (magazine, built wall, station) before every action."""

    def __init__(self, link, cfg: Mapping, job, *, sim: bool = False, timeout_s: float = 180.0,
                 park_tol_rad: float = 0.01, idle_qd_rad_s: float = 0.01, guard=None):
        from .ur import script                                   # local: keeps `import mauer.backends` light
        self.script = script
        self.link, self.cfg, self.job, self.sim = link, cfg, job, sim
        u, b = cfg["ur"], cfg["brick"]
        self.speeds = script.Speeds.from_config(dict(cfg))
        self.T_flange_tcp = np.asarray(job.T_flange_tcp, float)
        self.park_q = np.asarray(job.park_q_rad, float)
        self.approach_mm = float(job.approach_mm)
        self.contact_mm = float(u.get("contact_mm", 60.0))
        self.do_open, self.do_close = int(u["do_grip_open"]), int(u["do_grip_close"])
        self.pulse_s, self.wait_s = float(u["grip_pulse_s"]), float(u["grip_wait_s"])
        self.timeout_s = float(timeout_s)
        self.park_tol_rad, self.idle_qd = float(park_tol_rad), float(idle_qd_rad_s)
        self.settle_s = float(cfg.get("camera", {}).get("settle_s", 0.0))
        self.max_qd = float(cfg.get("vision", {}).get("max_qd_rad_s", 0.01))
        kg = float(u.get("payload_tool_kg", 0.0) or 0.0)
        if kg <= 0.0:
            if not sim:
                raise ValueError("[ur] payload_tool_kg is 0 (unknown PLACEHOLDER) - refusing to drive the real robot")
            from .capture import SIM_PAYLOAD_KG
            kg = float(u.get("sim_payload_kg", SIM_PAYLOAD_KG))
            log.warning("SIMULATION payload fallback %.2f kg (URSim only)", kg)
        self.tool_kg, self.tool_cog = kg, [float(v) for v in u.get("payload_cog_mm", [0.0, 0.0, 0.0])]
        stone_kg = float(b.get("mass_kg", 0.0) or 0.0)
        if stone_kg <= 0.0 and not sim:
            raise ValueError("[brick] mass_kg is 0 (UNKNOWN) - refusing to drive the real robot")
        self.with_stone = stone_payload(self.tool_kg, self.tool_cog, stone_kg, self.T_flange_tcp, float(b["height"]))
        self.payloads = {"full": self.with_stone}
        if any(getattr(t, "kind", "full") == "half" for t in job.stones()):
            hb = cfg.get("half_brick", {}) or {}
            half_kg = float(hb.get("mass_kg", 0.0) or 0.0)
            if half_kg <= 0.0 and not sim:
                raise ValueError("[half_brick] mass_kg is 0 (UNKNOWN) - refusing to drive the real robot")
            self.payloads["half"] = stone_payload(self.tool_kg, self.tool_cog, half_kg, self.T_flange_tcp,
                                                  float(hb.get("height", b["height"])))
        self.held_kind = "full"
        if guard is None and not sim:
            raise ValueError("the real robot needs the motion guard (mauer.motionguard.MotionGuard) - refusing")
        self.guard = guard
        self.holding: str | None = None          # kind of the stone in the jaws (for the guard), None = empty

    # ── helpers ──────────────────────────────────────────────────────────────
    def _q_now(self) -> np.ndarray:
        st = self.link.state()
        if st is None:
            raise RobotError("no RTDE state (link not started?)")
        return np.asarray(st.actual_q, float)

    def _qnear(self, hint, T_base_tcp: np.ndarray) -> np.ndarray:
        """IK branch hint: the job's, else the nominal-kinematics solution nearest the current joints (preferred
        family), else the current joints (the controller's IK guard then reports an unreachable pose)."""
        if hint is not None:
            return np.asarray(hint, float)
        q0 = self._q_now()
        from .simworld import ik_near                           # nominal UR5 DH kinematics (pure numpy)
        q = ik_near(T_base_tcp @ g.inv(self.T_flange_tcp), q0)
        return q0 if q is None else q

    def _preamble(self, holding: bool) -> str:
        kg, cog = self.payloads[self.held_kind] if holding else (self.tool_kg, self.tool_cog)
        return self.script.preamble(self.T_flange_tcp, kg, cog)

    def _vias(self, vias) -> list[str]:
        return [self.script.movej_q(q, self.speeds.a_joint, self.speeds.v_joint) for q in (vias or [])]

    def _guarded(self, q_target, name: str, vias=None, column=None) -> list[str]:
        """movej lines of the checked path actual joints -> (vias) -> q_target (mauer.motionguard; without a guard:
        the given vias unchecked - sim only). column: target TCP (x, y) of a place (MotionGuard.plan). RobotError if
        the guard finds no clear path."""
        if self.guard is None:
            return self._vias(vias)
        v = self.guard.plan(self._q_now(), q_target, self.holding, vias or (), column)
        if not v.ok:
            raise RobotError(f"{name}: refused by the motion guard (no clear path in the capsule model): "
                             + " | ".join(v.problems[-3:]), action=name)
        return self._vias(v.vias)

    def _q_above(self, T_base_frame: np.ndarray, T_frame_tcp: np.ndarray, q_hint, name: str) -> np.ndarray:
        """Joints of the approach pose (approach_mm along the frame z above the TCP pose), as the script's
        get_inverse_kin(above, qnear) - nominal kinematics near the hint."""
        from .simworld import ik_near
        T = T_base_frame @ g.transl(0.0, 0.0, self.approach_mm) @ np.asarray(T_frame_tcp, float) @ \
            g.inv(self.T_flange_tcp)
        q = ik_near(T, np.asarray(q_hint, float))
        if q is None:
            raise RobotError(f"{name}: no IK solution for the approach pose", action=name)
        return q

    def _run(self, body: str, name: str, settle_s: float = 0.0):
        res = self.link.run_block(body, name=name, timeout_s=self.timeout_s, settle_s=settle_s)
        if not res.ok:
            raise RobotError(f"{name}: {res.error}", res, name)
        return res

    # ── Robot interface ──────────────────────────────────────────────────────
    def park(self):
        body = "\n".join([self._preamble(self.holding is not None), *self._guarded(self.park_q, "mauer_park"),
                          self.script.movej_q(self.park_q, self.speeds.a_joint, self.speeds.v_joint)])
        return self._run(body, "mauer_park")

    def goto_look(self, look):
        lines = [self._preamble(False)]
        if look.q_rad is not None:
            lines += self._guarded(np.asarray(look.q_rad, float), "mauer_look")
            lines.append(self.script.movej_q(look.q_rad, self.speeds.a_joint, self.speeds.v_joint))
        else:
            T = np.asarray(look.T_base_flange, float)
            q = self._qnear(look.qnear_rad, T @ self.T_flange_tcp)
            if self.guard is not None:
                from .simworld import ik_near
                q_look = ik_near(T, np.asarray(q, float))
                if q_look is None:
                    raise RobotError("mauer_look: no IK solution for the look pose", action="mauer_look")
                lines += self._guarded(q_look, "mauer_look")
                q = q_look                           # the script's IK near the checked joints
            lines.append(self.script.look_pose(T, q, self.speeds.a_joint, self.speeds.v_joint, target="flange",
                                               T_flange_tcp=self.T_flange_tcp))
        return self._run("\n".join(lines), "mauer_look")

    def shot(self, camera):
        from .capture import CaptureError, capture_shot
        try:
            return capture_shot(self.link, camera, settle_s=self.settle_s, max_qd=self.max_qd)
        except CaptureError as e:
            raise ShotError(str(e)) from e

    def _pick(self, T_base_frame: np.ndarray, T_frame_tcp: np.ndarray, hint, vias, name: str, kind: str = "full"):
        if kind not in self.payloads:
            raise RobotError(f"{name}: no payload for stone type {kind!r}", action=name)
        self.held_kind = kind
        T_base_frame = np.asarray(T_base_frame, float)
        q = self._qnear(hint, T_base_frame @ np.asarray(T_frame_tcp, float))
        column = (T_base_frame @ np.asarray(T_frame_tcp, float))[:2, 3]
        if self.guard is not None:                   # the script's IK near the checked joints = the checked path
            q = self._q_above(T_base_frame, T_frame_tcp, q, name)
            path = self._guarded(q, name, vias, column)
        else:
            path = self._vias(vias)
        body = "\n".join([self._preamble(False), *path,
                          self.script.pick_stone(T_base_frame, T_frame_tcp, self.approach_mm, q, self.speeds,
                                                 self.do_close, self.pulse_s, self.wait_s, do_open=self.do_open,
                                                 contact_mm=self.contact_mm, open_first=True,
                                                 payload_after=self.payloads[kind])])
        res = self._run(body, name)
        self.holding = kind
        return res

    def _place(self, T_base_frame: np.ndarray, T_frame_tcp: np.ndarray, hint, vias, name: str):
        T_base_frame = np.asarray(T_base_frame, float)
        q = self._qnear(hint, T_base_frame @ np.asarray(T_frame_tcp, float))
        column = (T_base_frame @ np.asarray(T_frame_tcp, float))[:2, 3]
        if self.guard is not None:                   # the script's IK near the checked joints = the checked path
            q = self._q_above(T_base_frame, T_frame_tcp, q, name)
            path = self._guarded(q, name, vias, column)
        else:
            path = self._vias(vias)
        body = "\n".join([self._preamble(True), *path,
                          self.script.place_stone(T_base_frame, T_frame_tcp, self.approach_mm, q, self.speeds,
                                                  self.do_open, self.pulse_s, self.wait_s, do_close=self.do_close,
                                                  contact_mm=self.contact_mm,
                                                  payload_after=(self.tool_kg, self.tool_cog))])
        res = self._run(body, name)
        self.holding = None
        return res

    def pick_magazine(self, slot, T_base_ares, kind: str = "full"):
        return self._pick(T_base_ares, slot.T_ares_tcp, slot.qnear_rad, None, "mauer_pick_mag", kind)

    def place_wall(self, T_base_wall, stone):
        self.held_kind = getattr(stone, "kind", self.held_kind)
        return self._place(T_base_wall, stone.T_wall_tcp, stone.qnear_rad, stone.via_q_rad, "mauer_place_wall")

    def pick_station(self, T_base_station, slot):
        return self._pick(T_base_station, slot.T_station_tcp, slot.qnear_rad, None, "mauer_pick_station",
                          getattr(slot, "kind", "full"))

    def place_magazine(self, slot, T_base_ares, kind: str = "full"):
        self.held_kind = kind
        return self._place(T_base_ares, slot.T_ares_tcp, slot.qnear_rad, None, "mauer_place_mag")

    def is_parked(self) -> bool:
        st = self.link.state()
        return st is not None and float(np.max(np.abs(np.asarray(st.actual_q, float) - self.park_q))) <= \
            self.park_tol_rad

    def is_idle(self) -> bool:
        from .ur.link import RUNTIME_STOPPED
        st = self.link.state()
        if st is None or self.link.state_age_s() > 0.5:
            return False
        return (not st.program_running and st.runtime_state == RUNTIME_STOPPED
                and float(np.max(np.abs(np.asarray(st.actual_qd, float)))) <= self.idle_qd)

    def abort(self):
        return self.link.abort()


# ── real ARES ─────────────────────────────────────────────────────────────────
class AdsAres:
    """Ares backend over mauer.ares.AresAds (connected). translate/rotate return the MoveOutcome; the PLC's own
    checks (MoveRefused, AresNotReady, AresConnectionError) propagate."""

    def __init__(self, ads):
        self.ads = ads

    def translate(self, dx_mm: float, dy_mm: float):
        return self.ads.translate(dx_mm, dy_mm)

    def rotate(self, dtheta_deg: float):
        return self.ads.rotate(dtheta_deg)

    def preflight(self) -> list[str]:
        return self.ads.preflight()

    def status(self):
        return self.ads.status()

    def idle(self) -> bool:
        s = self.ads.status()
        return not s.move_active and not s.amr_moving

    def abort(self):
        return self.ads.abort()
