"""RoboDK digital twin of a Mauer run (docs/HMI_DESIGN.md section 11.3): a passive mirror of the UR joints, the ARES
pose (wall frame), the pick-up station frame and the stones (magazine, station, wall, jaws) in an OWN RoboDK instance.

Twin(cfg, job, frame_fn, settings).start() runs everything in the daemon thread "mauer-twin":
  - rdk_common.connect(new_instance=True) on the first free API port of [hmi.twin] port .. port + port_tries - 1
    (>= 20630; never the user's RoboDK on 20500 / 20501: a port held by a RoboDK this thread did not start is
    skipped untouched);
  - build_station.build(RDK, cfg) (3.8 s measured 2026-10-08 with the C; no Cam2D is opened), collisions OFF, world =
    wall frame (f_wall at the identity, as simulate.py LSim.setup), Wall_nominal as a transparent ghost, the item
    tree, the axes of the reference frames and the tool names hidden (the frame names stay), a view of the whole
    scene;
  - a loop at [hmi.twin] rate_hz: frame_fn() -> TwinScene.apply(): joints, ARES frame, station frame, stone objects
    (twin_model.plan_ops on the stone state on screen and the new one), ONE Render - only when something changed
    (render 11.5 ms with a visible window, 0.3 ms minimised: measured 2026-10-08).
The twin never calls into the run: frame_fn only reads immutable snapshots (hmi/core/twin_link.py), the run never
waits for the twin. Any RoboDK error (the user closed RoboDK, a hung API: socket_timeout_s) ends the twin in state
"lost"; the instance is closed at the end unless the user already closed it (exit code 3221225477 = 0xC0000005 is
RoboDK 6.0's known harmless crash on exit). States: off -> starting -> running -> stopped | lost.

Stone objects are named "Twin_<location>" and posed as in simulate.py LSim (CAD frame = T_tcp * rotx(pi) * T_tc_cad):
magazine stones under "ARES base_link" (slot T_ares_tcp), station stones under "Pickup station" (slot
T_station_tcp), wall stones under "Wall" at their NOMINAL T_wall_tcp, a held stone under the gripper tool
(rdk_common.held_stone_pose_kind; setParent, not setParentStatic: the absolute pose of a new item is stale with
rendering off). Meshes without simulate.py's collision scaling (the twin checks nothing).

Imported only by hmi/core/twin_link.py (lazily) and tests/test_twin_robodk.py; paths as in simulate.py.
"""
from __future__ import annotations

import logging
import queue
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import build_station  # noqa: E402
import rdk_common as rc  # noqa: E402
from build_station import HALF_STONE, STONE  # noqa: E402
from robodk.robolink import (COLLISION_OFF, FLAG_ROBODK_ALL, FLAG_ROBODK_REFERENCES_VISIBLE,  # noqa: E402
                             FLAG_ROBODK_TREE_VISIBLE, ITEM_TYPE_TOOL, WINDOWSTATE_MAXIMIZED, WINDOWSTATE_MINIMIZED,
                             WINDOWSTATE_NORMAL)
from robodk.robomath import invH, rotx, rotz, transl  # noqa: E402
from twin_model import (MIN_PORT, USER_PORTS, Loc, Op, StoneState, TwinFrame, TwinSettings,  # noqa: E402
                        apply_ops, default_view, plan_ops, wall_kinds)

from mauer.geometry import to_robodk  # noqa: E402

log = logging.getLogger("mauer.twin")

GHOST = [0.72, 0.30, 0.20, 0.15]       # = simulate.py GHOST (not imported: simulate.py pulls the planner)
FAILED = [0.90, 0.05, 0.05, 1.0]       # = simulate.py FAILED: a held stone whose jaw state is unknown
# twin window: no item tree, no reference frames (axes and labels of ~20 frames hide the robot); navigation stays
UI_FLAGS = FLAG_ROBODK_ALL & ~FLAG_ROBODK_TREE_VISIBLE & ~FLAG_ROBODK_REFERENCES_VISIBLE
Q_EPS_RAD = 1e-5                       # joint change that is drawn
POSE_EPS_MM = 1e-3                     # ARES / station pose change that is drawn
STATS_N = 50                           # ticks in the median of tick_ms / lag_ms / rate_hz


def pose2d_T(p):
    """RoboDK Mat of a mauer.reference.Pose2D (ARES in the wall frame) - simulate.py pose2d_T."""
    return transl(p.x_mm, p.y_mm, 0) * rotz(p.theta_rad)


def view_pose(eye: list, target: list):
    """RDK.setViewPose argument for a view from eye to target [mm, world = wall frame]: the world in the view frame,
    the view looking along its -z (OpenGL) - rotx(pi) * inv(look_at) (checked 2026-10-08: without the rotx the view
    shows nothing)."""
    return rotx(rc.PI) * invH(rc.look_at(eye, target))


class TwinScene:
    """The RoboDK station of the twin and the stone objects on it. Twin thread only."""

    def __init__(self, RDK, cfg: dict, job, settings: TwinSettings) -> None:
        self.RDK, self.cfg, self.job, self.settings = RDK, cfg, job, settings
        self.it: dict = {}
        self.items: dict[Loc, Any] = {}                 # location -> stone item
        self.shown = StoneState({}, {}, frozenset(), None)
        self.wall_kinds = wall_kinds(job)
        self._tasks = {tuple(t.key): t for t in job.stones()}
        self._mag_T = {s.id: to_robodk(s.T_ares_tcp) for s in job.magazine.slots}
        self._st_T = {s.id: to_robodk(s.T_station_tcp) for s in job.station.slots}
        self._q: np.ndarray | None = None
        self._ares = None
        self._T_ws: np.ndarray | None = None
        self._z_station = 0.0
        self._caption: str | None = None
        self.n_created = 0
        self.skipped: list[str] = []                    # operations on locations the job does not know

    # ── station ──────────────────────────────────────────────────────────────
    def build(self) -> None:
        RDK = self.RDK
        self.it = it = build_station.build(RDK, self.cfg)        # renders at its end
        RDK.Render(False)                                        # from here: one Render per changed tick
        RDK.setCollisionActive(COLLISION_OFF)
        it["f_wall"].setPose(transl(0, 0, 0))                    # world = wall frame (simulate.py LSim.setup)
        if self.settings.ghost_wall:
            it["wall"].setColor(GHOST)
        else:
            it["wall"].setVisible(False)
        if it.get("f_station") is not None:
            self._z_station = float(it["f_station"].Pose().Pos()[2])
        self._q = np.asarray(self.job.park_q_rad, float)
        it["robot"].setJoints(np.degrees(self._q).tolist())
        RDK.setFlagsRoboDK(UI_FLAGS)
        for tool in RDK.ItemList(ITEM_TYPE_TOOL):        # their TCP names (gripper, camera) cover the wrist
            tool.setVisible(True, False)
        self.set_view(*default_view(self.job))
        RDK.Render()

    def set_view(self, eye: list, target: list) -> None:
        self.RDK.setViewPose(view_pose(eye, target))

    # ── stones ───────────────────────────────────────────────────────────────
    def _known(self, loc: Loc | None) -> bool:
        if loc is None:
            return True
        kind, ident = loc
        if kind == "mag":
            return ident in self._mag_T
        if kind == "station":
            return ident in self._st_T and self.it.get("f_station") is not None
        if kind == "wall":
            return tuple(ident) in self._tasks
        return kind == "tool"

    def _pose(self, loc: Loc, kind: str):
        """(parent item, pose of the stone's CAD frame in it) of a location."""
        tc = rotx(rc.PI) * rc.T_tc_cad(self.cfg, kind)
        where, ident = loc
        if where == "mag":
            return self.it["f_ares"], self._mag_T[ident] * tc
        if where == "station":
            return self.it["f_station"], self._st_T[ident] * tc
        if where == "wall":
            return self.it["f_wall"], to_robodk(self._tasks[tuple(ident)].T_wall_tcp) * tc
        return self.it["tool"], rc.held_stone_pose_kind(self.cfg, kind)

    def _place(self, item, op: Op) -> None:
        parent, pose = self._pose(op.dst, op.stone_kind)
        item.setParent(parent)
        item.setPose(pose)
        item.setColor(FAILED if op.failed else (HALF_STONE if op.stone_kind == "half" else STONE))
        where, ident = op.dst
        label = "tool" if where == "tool" else f"{where}_{'_'.join(map(str, ident)) if where == 'wall' else ident}"
        item.setName(f"Twin_{label}")

    def _make(self, op: Op):
        path = rc.stone_mesh_path(self.cfg, op.stone_kind)
        if path is None:                                  # no half stone mesh: the [half_brick] box (CAD frame)
            L, W, H = rc.stone_dims(self.cfg, op.stone_kind)
            item = self.RDK.AddShape(rc.as_mat(rc.box_points(W, L, H, W / 2, -L / 2, H / 2)))
        else:
            item = self.RDK.AddFile(str(path), self.it["f_wall"])
        if not item.Valid():
            raise RuntimeError(f"stone object could not be created ({path})")
        self.n_created += 1
        self._place(item, op)
        return item

    @staticmethod
    def _drop(item) -> None:
        item.Delete()

    def update_stones(self, new: StoneState) -> int:
        """Bring the stone objects from self.shown to `new`; returns the number of operations."""
        ops = []
        for op in plan_ops(self.shown, new, self.wall_kinds):
            if self._known(op.dst):
                ops.append(op)
            elif op.kind == "move":                       # unknown target: the object goes
                ops.append(Op("remove", op.src, None, op.stone_kind))
                self.skipped.append(f"{op.dst}")
            else:
                self.skipped.append(f"{op.dst}")
        self.items = apply_ops(self.items, ops, self._make, self._place, self._drop)
        self.shown = new
        return len(ops)

    def stone_count(self) -> dict[str, int]:
        """{"mag", "station", "wall", "tool"}: stone objects per place (tests)."""
        out = {"mag": 0, "station": 0, "wall": 0, "tool": 0}
        for where, _ in self.items:
            out[where] += 1
        return out

    # ── one tick ─────────────────────────────────────────────────────────────
    def apply(self, frame: TwinFrame) -> bool:
        """Show `frame`; True if anything changed (then the scene was rendered once)."""
        it, changed = self.it, False
        if frame.q_rad is not None:
            q = np.asarray(frame.q_rad, float)
            if self._q is None or q.shape != self._q.shape or float(np.max(np.abs(q - self._q))) > Q_EPS_RAD:
                it["robot"].setJoints(np.degrees(q).tolist())
                self._q, changed = q, True
        a = frame.ares
        if a is not None and (self._ares is None or abs(a.x_mm - self._ares.x_mm) > POSE_EPS_MM
                              or abs(a.y_mm - self._ares.y_mm) > POSE_EPS_MM
                              or abs(a.theta_rad - self._ares.theta_rad) > 1e-6):
            it["f_ares"].setPose(pose2d_T(a))
            self._ares, changed = a, True
        T = frame.T_wall_station
        if T is not None and it.get("f_station") is not None:
            T = np.array(T, float)
            T[2, 3] = self._z_station                    # the estimate is planar: z of the nominal station frame
            if self._T_ws is None or float(np.max(np.abs(T - self._T_ws))) > 1e-6:
                it["f_station"].setPose(to_robodk(T))
                self._T_ws, changed = T, True
        if frame.stones is not None and frame.stones != self.shown:
            changed = self.update_stones(frame.stones) > 0 or changed
        if frame.caption != self._caption:
            self.RDK.ShowMessage(frame.caption, False)  # status bar of the RoboDK window
            self._caption = frame.caption
        if changed:
            self.RDK.Render()
        return changed

    def snapshot(self, path: Path, eye: list | None = None, target: list | None = None,
                 window_state: int | None = None, restore_state: int | None = None, tries: int = 6) -> bool:
        """PNG of RoboDK's 3D view (Cam2D_Snapshot of the main view, no Cam2D). The view must be rendered: a
        minimised window gives the stale picture of its last visible state, and the first picture after the window
        comes back can be blank white (both checked 2026-10-08) - pass window_state (e.g. WINDOWSTATE_MAXIMIZED) to
        show it for the snapshot and restore_state to put it back; a blank picture is taken again (up to tries)."""
        RDK = self.RDK
        if eye is not None and target is not None:
            self.set_view(eye, target)
        if window_state is not None:
            RDK.setWindowState(window_state)
            time.sleep(0.5)                              # the window is up before it is rendered
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        ok = False
        try:
            for _ in range(max(1, tries)):
                RDK.Render()
                time.sleep(0.5)                          # let the window paint once at its new size
                ok = bool(RDK.Cam2D_Snapshot(str(path), None)) and not blank_image(Path(path))
                if ok:
                    break
        finally:
            if restore_state is not None:
                RDK.setWindowState(restore_state)
        return ok


def blank_image(path: Path) -> bool:
    """True if the PNG is missing or one flat colour (a view that was not painted)."""
    import cv2
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    return img is None or float(img.std()) < 1.0


@dataclass
class _Snap:
    path: Path
    eye: list | None
    target: list | None
    show: bool
    done: Callable[[bool, str], None]


class Twin:
    """The twin thread (see the module docstring). on_status(state, detail) is called in that thread on every state
    change; connect / close replace rdk_common.connect / close_instance (tests)."""

    def __init__(self, cfg: dict, job, frame_fn: Callable[[], TwinFrame | None], settings: TwinSettings,
                 on_status: Callable[[str, str], None] = lambda s, d: None, *, connect=None, close=None) -> None:
        self.cfg, self.job, self.frame_fn, self.settings = cfg, job, frame_fn, settings
        self.on_status = on_status
        self._connect = connect or rc.connect
        self._close = close or rc.close_instance
        self.state = "off"
        self.detail = ""
        self.port: int | None = None
        self.last_error: str | None = None
        self.ticks = 0
        self.renders = 0
        self.frame_errors = 0
        self.tick_ms = 0.0                  # median duration of the last STATS_N rendered ticks
        self.lag_ms = 0.0                   # median data age when rendered (frame.t -> Render done)
        self.rate_hz = 0.0                  # loop rate over the last STATS_N ticks
        self.exit_code: int | None = None
        self.proc = None                    # RoboDK process (subprocess.Popen) of this twin
        self.scene: TwinScene | None = None
        self._tick_ms: deque = deque(maxlen=STATS_N)
        self._lag_ms: deque = deque(maxlen=STATS_N)
        self._starts: deque = deque(maxlen=STATS_N)
        self._requests: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ── control (any thread) ─────────────────────────────────────────────────
    def start(self) -> None:
        """Start the twin thread; returns at once."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._set("starting", f"RoboDK on port {self.settings.port}..{self.settings.ports[-1]}")
        self._thread = threading.Thread(target=self._run, name="mauer-twin", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 15.0) -> bool:
        """Stop the loop, close the RoboDK instance, wait up to timeout_s; True if the thread ended."""
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout_s)
        return t is None or not t.is_alive()

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def request_snapshot(self, path: Path, eye: list | None = None, target: list | None = None, show: bool = False,
                         done: Callable[[bool, str], None] = lambda ok, path: None) -> None:
        """PNG of the twin view at the next tick (twin thread); done(ok, path) is called there. show: put the window
        up (maximised) for the snapshot - needed when it is minimised (visible = false)."""
        self._requests.put(_Snap(Path(path), eye, target, show, done))

    def stats(self) -> dict:
        return {"state": self.state, "detail": self.detail, "port": self.port, "rate_hz": self.rate_hz,
                "tick_ms": self.tick_ms, "lag_ms": self.lag_ms, "ticks": self.ticks, "renders": self.renders,
                "frame_errors": self.frame_errors, "last_error": self.last_error}

    # ── twin thread ──────────────────────────────────────────────────────────
    def _set(self, state: str, detail: str = "") -> None:
        self.state, self.detail = state, detail
        try:
            self.on_status(state, detail)
        except Exception as e:      # noqa: BLE001 - a status consumer must not stop the twin
            log.warning("twin status callback failed: %s", e)

    def _timeout(self, RDK, seconds: float) -> None:
        RDK.TIMEOUT = float(seconds)                       # robolink resets COM to TIMEOUT after long calls
        RDK.COM.settimeout(float(seconds))

    def _open(self):
        """(RDK, port) on the first port where a NEW instance starts; None if stopped meanwhile."""
        errors = []
        for port in self.settings.ports:
            if self._stop.is_set():
                return None
            if port < MIN_PORT or port in USER_PORTS:     # TwinSettings refuses these already
                raise RuntimeError(f"refusing RoboDK API port {port}")
            try:
                # rdk_common.connect starts RoboDK without the Qt platform variables (rdk_common.qt_free_env)
                RDK = self._connect(new_instance=True, port=port, minimized=not self.settings.visible)
            except RuntimeError as e:                     # a RoboDK this twin did not start: skipped untouched
                errors.append(f"{port}: {e}")
                log.info("twin: port %d not used: %s", port, e)
                continue
            return RDK, port
        raise RuntimeError(f"no free RoboDK API port in {self.settings.port}..{self.settings.ports[-1]}"
                           + (f" ({errors[-1]})" if errors else ""))

    def _closed_by_user(self) -> bool:
        return self.proc is not None and self.proc.poll() is not None

    def _run(self) -> None:
        RDK = None
        try:
            got = self._open()
            if got is None:
                return
            RDK, self.port = got
            self.proc = getattr(RDK, "NEW_INSTANCE", None)
            self._set("starting", f"building the station (port {self.port})")
            self._timeout(RDK, self.settings.build_timeout_s)
            scene = TwinScene(RDK, self.cfg, self.job, self.settings)
            scene.build()
            self.scene = scene
            self._timeout(RDK, self.settings.socket_timeout_s)
            if not self._stop.is_set():
                self._set("running", f"port {self.port}")
                self._loop(scene)
        except Exception as e:      # noqa: BLE001 - whatever RoboDK does, the HMI and the run go on
            self.last_error = f"{type(e).__name__}: {e}"
            if not self._stop.is_set():
                why = "RoboDK was closed" if self._closed_by_user() else self.last_error
                log.warning("twin lost: %s", why)
                self._set("lost", why)
        finally:
            self.exit_code = self._shutdown(RDK)
            if self.state != "lost":
                self._set("stopped", "" if self.exit_code is None else f"RoboDK exit code {self.exit_code}")

    def _loop(self, scene: TwinScene) -> None:
        period = 1.0 / float(self.settings.rate_hz)
        while not self._stop.is_set():
            t0 = time.monotonic()
            self._starts.append(t0)
            if self._closed_by_user():
                raise ConnectionError("RoboDK was closed")
            self._serve(scene)
            try:
                frame = self.frame_fn()
            except Exception as e:  # noqa: BLE001 - a bad frame skips the tick, the twin goes on
                self.frame_errors += 1
                self.last_error = f"frame: {type(e).__name__}: {e}"
                if self.frame_errors <= 3:
                    log.warning("twin frame failed: %s", e)
                frame = None
            if frame is not None and scene.apply(frame):
                self.renders += 1
                self._tick_ms.append((time.monotonic() - t0) * 1000.0)
                self.tick_ms = float(statistics.median(self._tick_ms))
                if frame.t:
                    self._lag_ms.append(max(0.0, (time.time() - frame.t) * 1000.0))
                    self.lag_ms = float(statistics.median(self._lag_ms))
            self.ticks += 1
            if len(self._starts) > 1 and self._starts[-1] > self._starts[0]:
                self.rate_hz = (len(self._starts) - 1) / (self._starts[-1] - self._starts[0])
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def _serve(self, scene: TwinScene) -> None:
        while True:
            try:
                req = self._requests.get_nowait()
            except queue.Empty:
                return
            ok = False
            try:
                restore = None
                if req.show:
                    restore = WINDOWSTATE_NORMAL if self.settings.visible else WINDOWSTATE_MINIMIZED
                ok = scene.snapshot(req.path, req.eye, req.target, WINDOWSTATE_MAXIMIZED if req.show else None,
                                    restore)
            finally:
                try:
                    req.done(ok, str(req.path))
                except Exception as e:  # noqa: BLE001
                    log.warning("twin snapshot callback failed: %s", e)

    def _shutdown(self, RDK) -> int | None:
        if RDK is None:
            return None
        if self._closed_by_user():                         # nothing to close; drop the socket
            try:
                RDK.COM.close()
            except Exception:  # noqa: BLE001
                pass
            return self.proc.returncode
        try:
            return self._close(RDK)
        except Exception as e:      # noqa: BLE001 - closing must not raise into the HMI
            log.warning("twin: closing RoboDK failed: %s", e)
            if self.proc is not None and self.proc.poll() is None:
                self.proc.kill()
            return None

