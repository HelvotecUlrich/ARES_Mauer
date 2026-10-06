"""Laptop <-> UR5 CB3 link: RTDE state stream + one-shot URScript blocks with start/done markers.

Architecture (research 2026-10-05, tested against URSim CB3 3.15.8, research_out/ur5-interface.json):

- RTDE (port 30004, official UR client vendored in mauer/ur/rtde, BSD-3): one output recipe at up to 125 Hz
  (CB3 maximum) in a background thread. Every sample is stamped with the laptop `time.time()` on receipt and kept
  in a time-limited history, so the robot pose during a camera exposure (`Frame.t_start..t_end`, also time.time())
  can be taken from `samples(t0, t1)`. `RTDE.receive()` returns the newest package and drops older buffered ones,
  so a slow consumer loses samples, never gets stale ones.
- Port 30002 (secondary client interface): ONE persistent socket, drained by a background thread (the controller
  streams state on it at 10 Hz). Each atomic step (look pose, pick, place, gripper) is its own `def` program from
  `script.block_program`: it writes its id into output_int_register 24 as the first statement and, after
  `while not is_steady(): sync() end`, into register 25 as the last. Persistent socket: start latency ~35 ms in
  URSim against 0.3-0.6 s with a new connection per program.
- A `def` program sent on 30002 stops whatever program runs (URSim; UR ROS2 driver docs); a `sec` program does not.
  Output registers keep their values across programs, so block ids increase from the larger of the two marker
  registers found at start.
- Errors detected by `run_block`: no start marker within `start_timeout_s` (compile/syntax error – the reason is
  only in the PolyScope Log tab), safety_mode not NORMAL/REDUCED during the block (protective stop, e-stop),
  runtime_state STOPPED without the done marker (runtime error, `halt`, IK guard, stop from pendant/Dashboard),
  timeout (then the program is aborted). The Dashboard's programState is useless here (it says STOPPED while a
  30002 program runs) – the link reads RTDE runtime_state and robot_status_bits.
- No RTDE watchdog: `rtde_set_watchdog(..., "stop")` raised protective stop C207A0 when the laptop went quiet
  (URSim) – a dead laptop would stop the arm mid-motion with a stone in the jaws. A crashed laptop only lets the
  current block finish.

Flange pose: `T_base_flange = T_base_tcp @ inv(T_flange_tcp)`. T_flange_tcp comes from the RTDE field `tcp_offset`
("Transformation from the output flange coordinate system to the TCP", RTDE guide; present on URSim CB3 3.15.8).
If a controller lacks `tcp_offset`, the link falls back to the last `set_tcp(p[...])` it sent in a started block
(`tcp_tracked`); `URLink.flange_T()` says which one it used via `tcp_source`.

Units: as delivered by RTDE (m, rad, rotation vectors, rad/s, V) in `URState`; 4x4 helpers return mm.
Enum values: UR Client Library datatypes.h (robot_mode, safety_mode), RTDE guide (runtime_state, status bits).
"""
from __future__ import annotations

import collections
import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..geometry import inv, ur_to_T
from . import script
from .dashboard import Dashboard, DashboardError
from .rtde.rtde import RTDE

log = logging.getLogger("mauer.ur")

ROBOT_MODE = {-1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING", 3: "POWER_OFF",
              4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING", 8: "UPDATING_FIRMWARE"}
SAFETY_MODE = {1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY", 5: "SAFEGUARD_STOP",
               6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP", 8: "VIOLATION", 9: "FAULT",
               10: "VALIDATE_JOINT_ID", 11: "UNDEFINED"}
RUNTIME_STATE = {0: "STOPPING", 1: "STOPPED", 2: "PLAYING", 3: "PAUSING", 4: "PAUSED", 5: "RESUMING"}
ROBOT_RUNNING = 7
SAFETY_OK = (1, 2)            # NORMAL, REDUCED (reduced limits, still allowed to move)
RUNTIME_STOPPED = 1
BIT_POWER_ON, BIT_PROGRAM_RUNNING = 0x1, 0x2   # robot_status_bits (RTDE guide: bits 0-3)

FRESH_STATE_WAIT_S = 2.0   # a block waits this long for a current RTDE sample before it fails (stale stream)

# RTDE output recipe: name -> wire type. Required fields exist on every CB3 >= 3.9 (registers 24..47 since 3.9.0).
REQUIRED_FIELDS = {
    "timestamp": "DOUBLE",              # controller time since start [s]
    "actual_q": "VECTOR6D",             # [rad]
    "actual_qd": "VECTOR6D",            # [rad/s]
    "actual_TCP_pose": "VECTOR6D",      # [m, rotation vector rad], base frame, active TCP
    "robot_mode": "INT32",
    "safety_mode": "INT32",
    "runtime_state": "UINT32",
    "robot_status_bits": "UINT32",
}
# Probed at start; missing ones are dropped (URSim CB3 3.15.8 has all of them).
OPTIONAL_FIELDS = {
    "target_TCP_pose": "VECTOR6D",      # [m, rotvec]
    "tcp_offset": "VECTOR6D",           # flange -> TCP [m, rotvec]
    "tool_output_voltage": "INT32",     # [V] – camera supply check
    "actual_digital_output_bits": "UINT64",   # bits 0-7 standard DO (gripper)
}


class URLinkError(RuntimeError):
    """Link setup / connection problem (not a block failure – those come back as BlockResult)."""


class URBlockError(RuntimeError):
    """Raised by BlockResult.raise_for_error()."""

    def __init__(self, result: "BlockResult"):
        super().__init__(f"block {result.name} #{result.block_id}: {result.error}")
        self.result = result


def _ro(a) -> np.ndarray | None:
    if a is None:
        return None
    a = np.array(a, float)
    a.setflags(write=False)
    return a


@dataclass(frozen=True)
class URState:
    """One RTDE sample. Units as delivered: m, rad, rotation vectors, rad/s, V. Optional fields are None when the
    controller does not offer them."""
    t_laptop: float                         # laptop time.time() on receipt [s]
    timestamp: float                        # controller time since start [s]
    actual_q: np.ndarray                    # (6,) [rad]
    actual_qd: np.ndarray                   # (6,) [rad/s]
    actual_TCP_pose: np.ndarray             # (6,) [m, rad rotvec]
    robot_mode: int
    safety_mode: int
    runtime_state: int
    robot_status_bits: int
    reg_started: int                        # output_int_register <reg_started>
    reg_done: int                           # output_int_register <reg_done>
    reg_error: int | None = None            # output_int_register <reg_error>
    target_TCP_pose: np.ndarray | None = None
    tcp_offset: np.ndarray | None = None    # (6,) flange -> TCP [m, rad rotvec]
    tool_output_voltage: int | None = None  # [V]
    digital_outputs: int | None = None      # actual_digital_output_bits

    @property
    def power_on(self) -> bool:
        return bool(self.robot_status_bits & BIT_POWER_ON)

    @property
    def program_running(self) -> bool:
        return bool(self.robot_status_bits & BIT_PROGRAM_RUNNING)

    def digital_out(self, n: int) -> bool | None:
        """Standard digital output n (0..7), None if actual_digital_output_bits is not in the recipe."""
        return None if self.digital_outputs is None else bool((self.digital_outputs >> int(n)) & 1)

    def T_base_tcp_mm(self) -> np.ndarray:
        """Active TCP in the UR base frame, 4x4 [mm]."""
        return ur_to_T(self.actual_TCP_pose)

    def T_flange_tcp_mm(self) -> np.ndarray | None:
        """Active TCP offset from RTDE tcp_offset, 4x4 [mm], None if not available."""
        return None if self.tcp_offset is None else ur_to_T(self.tcp_offset)

    def T_base_flange_mm(self, T_flange_tcp: np.ndarray | None = None) -> np.ndarray:
        """Flange in the UR base frame [mm] = T_base_tcp @ inv(T_flange_tcp). Uses the sample's own tcp_offset;
        `T_flange_tcp` [mm] is only the fallback for controllers without that field."""
        T_ft = self.T_flange_tcp_mm()
        if T_ft is None:
            if T_flange_tcp is None:
                raise ValueError("sample has no tcp_offset (RTDE field missing): pass T_flange_tcp")
            T_ft = np.asarray(T_flange_tcp, float)
        return self.T_base_tcp_mm() @ inv(T_ft)

    def describe(self) -> str:
        return (f"robot_mode {ROBOT_MODE.get(self.robot_mode, self.robot_mode)}, "
                f"safety_mode {SAFETY_MODE.get(self.safety_mode, self.safety_mode)}, "
                f"runtime {RUNTIME_STATE.get(self.runtime_state, self.runtime_state)}, "
                f"program_running {self.program_running}")


def flange_T(sample: URState, T_flange_tcp: np.ndarray | None = None) -> np.ndarray:
    """T_base_flange [mm] of an RTDE sample (see URState.T_base_flange_mm)."""
    return sample.T_base_flange_mm(T_flange_tcp)


@dataclass
class BlockResult:
    """Outcome of URLink.run_block. Times are laptop time.time() [s]: t_sent = program sent, t_started / t_done =
    receipt of the first RTDE sample showing the start / done marker."""
    ok: bool
    block_id: int
    name: str
    t_sent: float | None = None
    t_started: float | None = None
    t_done: float | None = None
    error: str | None = None
    error_code: int | None = None     # output_int_register reg_error after the block (script.ERR_*), if read
    state: URState | None = None      # last sample (after settle_s when ok, at the failure otherwise)
    program: str = field(default="", repr=False)

    @property
    def start_latency_s(self) -> float | None:
        return None if self.t_started is None or self.t_sent is None else self.t_started - self.t_sent

    @property
    def duration_s(self) -> float | None:
        return None if self.t_done is None or self.t_sent is None else self.t_done - self.t_sent

    def raise_for_error(self) -> "BlockResult":
        if not self.ok:
            raise URBlockError(self)
        return self


def _open_rtde(host: str, port: int, fields: dict[str, str], frequency: float) -> tuple[RTDE, tuple]:
    """Connected, started RTDE connection with the given output recipe. Raises ValueError if a field is unknown
    to the controller (client: 'Unknown data type: NOT_FOUND') or claimed elsewhere (IN_USE)."""
    con = RTDE(host, port)
    con.connect()   # vendored client: 1 s socket timeout, negotiates protocol v2, else v1 (PolyScope 3.3)
    try:
        version = con.get_controller_version()
        if not con.send_output_setup(list(fields), list(fields.values()), frequency=frequency):
            raise URLinkError("RTDE output setup failed (type mismatch, see the 'rtde' log)")
        if not con.send_start():
            raise URLinkError("RTDE start failed")
    except BaseException:
        con.disconnect()
        raise
    return con, version


def probe_fields(host: str, names: list[str], port: int = 30004) -> dict[str, bool]:
    """Which RTDE output fields the controller offers (one short connection per field)."""
    out: dict[str, bool] = {}
    for n in names:
        con = RTDE(host, port)
        con.connect()
        try:
            con.send_output_setup([n], [], frequency=10)
            out[n] = True
        except ValueError:
            out[n] = False
        finally:
            con.disconnect()
    return out


class URLink:
    """RTDE state + URScript blocks for one UR controller. Use as a context manager or start()/stop().

    stop() only closes the connections – it does NOT stop the robot; use abort() for that.
    """

    def __init__(self, host: str, rtde_port: int = 30004, script_port: int = 30002, frequency: float = 125.0,
                 reg_started: int = script.REG_STARTED, reg_done: int = script.REG_DONE, history_s: float = 30.0,
                 *, reg_error: int | None = script.REG_ERROR, dashboard_port: int = 29999,
                 start_timeout_s: float = 3.0, connect_timeout_s: float = 5.0):
        if not host:
            raise ValueError("URLink: empty host ([ur].host in config/station.toml is a PLACEHOLDER)")
        self.host = host
        self.rtde_port, self.script_port, self.dashboard_port = int(rtde_port), int(script_port), int(dashboard_port)
        self.frequency = float(frequency)
        self.reg_started, self.reg_done = int(reg_started), int(reg_done)
        self.reg_error = None if reg_error is None else int(reg_error)
        self.history_s = float(history_s)
        self.start_timeout_s = float(start_timeout_s)
        self.connect_timeout_s = float(connect_timeout_s)

        self.controller_version: tuple | None = None
        self.fields: dict[str, str] = {}
        self.missing_fields: list[str] = []
        self.tcp_tracked: np.ndarray | None = None   # T_flange_tcp [mm] of the last set_tcp sent in a started block
        self.rtde_error: str | None = None           # last RTDE connection problem (None = streaming)

        self._con: RTDE | None = None
        self._cond = threading.Condition()
        self._latest: URState | None = None
        self._history: collections.deque[URState] = collections.deque()
        self._running = False
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._sock_dead = False
        self._send_lock = threading.Lock()
        self._block_lock = threading.Lock()
        self._next_id = 1

    @classmethod
    def from_config(cls, cfg: dict, host: str | None = None, **kw) -> "URLink":
        """From [ur] (host, ports, rtde_hz, reg_started/reg_done, block_start_timeout_s; reg_error optional)."""
        u = cfg["ur"]
        host = host or u.get("host", "")
        if not host:
            raise ValueError("[ur].host is an empty PLACEHOLDER in config/station.toml - set the robot IP or pass a "
                             "host (URSim: 127.0.0.1)")
        args = dict(rtde_port=u.get("rtde_port", 30004), script_port=u.get("script_port", 30002),
                    frequency=u.get("rtde_hz", 125.0), reg_started=u.get("reg_started", script.REG_STARTED),
                    reg_done=u.get("reg_done", script.REG_DONE), reg_error=u.get("reg_error", script.REG_ERROR),
                    dashboard_port=u.get("dashboard_port", 29999),
                    start_timeout_s=u.get("block_start_timeout_s", 3.0))
        args.update(kw)
        return cls(host, **args)

    # ── lifecycle ────────────────────────────────────────────────────────────
    def _recipe(self) -> dict[str, str]:
        f = dict(REQUIRED_FIELDS)
        f[f"output_int_register_{self.reg_started}"] = "INT32"
        f[f"output_int_register_{self.reg_done}"] = "INT32"
        if self.reg_error is not None:
            f[f"output_int_register_{self.reg_error}"] = "INT32"
        f.update(OPTIONAL_FIELDS)
        return f

    def start(self) -> "URLink":
        """Connect RTDE (probing optional fields), start the receiver thread, wait for the first sample, open the
        persistent 30002 socket."""
        if self._running:
            return self
        fields = self._recipe()
        try:
            con, version = _open_rtde(self.host, self.rtde_port, fields, self.frequency)
        except ValueError as e:
            avail = probe_fields(self.host, list(fields), self.rtde_port)
            missing = [n for n, ok in avail.items() if not ok]
            required = [n for n in missing if n in REQUIRED_FIELDS or n.startswith("output_int_register_")
                        and n != f"output_int_register_{self.reg_error}"]
            if required:
                raise URLinkError(f"RTDE fields not available on this controller: {required} ({e})") from e
            fields = {n: t for n, t in fields.items() if n not in missing}
            self.missing_fields = missing
            log.warning("RTDE fields not available, continuing without: %s", missing)
            con, version = _open_rtde(self.host, self.rtde_port, fields, self.frequency)
        self._con, self.controller_version, self.fields = con, version, fields
        self._running = True
        self._thread = threading.Thread(target=self._rtde_loop, name="ur-rtde", daemon=True)
        self._thread.start()
        try:
            first = self.wait_until(lambda s: True, timeout_s=3.0)
            if first is None:
                raise URLinkError(f"no RTDE sample from {self.host}:{self.rtde_port} within 3 s")
            self._next_id = self._id_after(max(first.reg_started, first.reg_done, 0))
            self._open_script()
        except BaseException:
            self.stop()
            raise
        log.info("URLink %s: controller %s, %d RTDE fields at %.0f Hz", self.host, version, len(fields),
                 self.frequency)
        return self

    def stop(self) -> None:
        """Close RTDE and the 30002 socket (does not stop a running robot program – see abort())."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._con is not None:
            try:
                self._con.send_pause()
            except Exception:  # connection already gone
                pass
            self._con.disconnect()
            self._con = None
        self._close_script()

    close = stop

    def __enter__(self) -> "URLink":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # ── RTDE state ───────────────────────────────────────────────────────────
    def _to_state(self, o, t: float) -> URState:
        g = lambda n: getattr(o, n, None)  # noqa: E731
        reg_err = g(f"output_int_register_{self.reg_error}") if self.reg_error is not None else None
        tv, do = g("tool_output_voltage"), g("actual_digital_output_bits")
        return URState(
            t_laptop=t, timestamp=float(o.timestamp), actual_q=_ro(o.actual_q), actual_qd=_ro(o.actual_qd),
            actual_TCP_pose=_ro(o.actual_TCP_pose), robot_mode=int(o.robot_mode), safety_mode=int(o.safety_mode),
            runtime_state=int(o.runtime_state), robot_status_bits=int(o.robot_status_bits),
            reg_started=int(g(f"output_int_register_{self.reg_started}")),
            reg_done=int(g(f"output_int_register_{self.reg_done}")),
            reg_error=None if reg_err is None else int(reg_err),
            target_TCP_pose=_ro(g("target_TCP_pose")), tcp_offset=_ro(g("tcp_offset")),
            tool_output_voltage=None if tv is None else int(tv), digital_outputs=None if do is None else int(do))

    def _rtde_loop(self) -> None:
        con = self._con
        while self._running:
            try:
                obj = con.receive()   # newest package, None after 1 s without data
            except Exception as e:    # RTDEException/OSError (connection lost), struct errors on garbage
                if not self._running:
                    break
                self.rtde_error = f"{type(e).__name__}: {e}"
                log.warning("RTDE connection lost (%s) - reconnecting", self.rtde_error)
                try:
                    con.disconnect()
                except OSError:
                    pass
                con = self._rtde_reconnect()
                if con is None:
                    break
                continue
            if obj is None:
                continue
            st = self._to_state(obj, time.time())
            with self._cond:
                self._latest = st
                self._history.append(st)
                cutoff = st.t_laptop - self.history_s
                while self._history and self._history[0].t_laptop < cutoff:
                    self._history.popleft()
                self._cond.notify_all()

    def _rtde_reconnect(self) -> RTDE | None:
        while self._running:
            time.sleep(1.0)
            try:
                con, _ = _open_rtde(self.host, self.rtde_port, self.fields, self.frequency)
            except Exception as e:  # keep trying until stop()
                self.rtde_error = f"reconnect: {type(e).__name__}: {e}"
                continue
            self._con, self.rtde_error = con, None
            log.info("RTDE reconnected")
            return con
        return None

    def state(self) -> URState | None:
        """Latest RTDE sample (None before the first one)."""
        with self._cond:
            return self._latest

    def state_age_s(self) -> float:
        """Laptop time since the latest sample was received [s] (inf before the first)."""
        s = self.state()
        return float("inf") if s is None else time.time() - s.t_laptop

    def samples(self, t0: float, t1: float | None = None) -> list[URState]:
        """Samples received in [t0, t1] (laptop time.time(); t1 None = now), oldest first, from the last
        `history_s` seconds."""
        t1 = float("inf") if t1 is None else t1
        with self._cond:
            return [s for s in self._history if t0 <= s.t_laptop <= t1]

    def wait_until(self, predicate: Callable[[URState], bool], timeout_s: float) -> URState | None:
        """Block until predicate(latest sample) is True; returns that sample, or None after timeout_s. The predicate
        runs under the state lock – keep it cheap."""
        deadline = time.time() + timeout_s
        with self._cond:
            while True:
                s = self._latest
                if s is not None and predicate(s):
                    return s
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    return None
                self._cond.wait(min(remaining, 0.1))

    def _first_time(self, predicate: Callable[[URState], bool], t_from: float) -> float | None:
        with self._cond:
            for s in self._history:
                if s.t_laptop >= t_from and predicate(s):
                    return s.t_laptop
        return None

    # ── flange / TCP ─────────────────────────────────────────────────────────
    @property
    def tcp_source(self) -> str:
        """'rtde tcp_offset' or 'tracked set_tcp' – where flange_T takes T_flange_tcp from."""
        return "rtde tcp_offset" if "tcp_offset" in self.fields else "tracked set_tcp"

    def T_flange_tcp_current(self, state: URState | None = None) -> np.ndarray | None:
        """Active TCP offset [mm]: RTDE tcp_offset if available, else the last set_tcp this link sent."""
        s = state or self.state()
        if s is not None and s.tcp_offset is not None:
            return s.T_flange_tcp_mm()
        return None if self.tcp_tracked is None else self.tcp_tracked.copy()

    def flange_T(self, state: URState | None = None) -> np.ndarray:
        """T_base_flange [mm] of a sample (default: latest)."""
        s = state or self.state()
        if s is None:
            raise URLinkError("no RTDE sample yet")
        return s.T_base_flange_mm(self.tcp_tracked)

    # ── port 30002 ───────────────────────────────────────────────────────────
    def _open_script(self) -> None:
        s = socket.create_connection((self.host, self.script_port), timeout=self.connect_timeout_s)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(1.0)
        self._sock, self._sock_dead = s, False
        threading.Thread(target=self._drain_loop, args=(s,), name="ur-30002-drain", daemon=True).start()

    def _close_script(self) -> None:
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def _drain_loop(self, s: socket.socket) -> None:
        """Read away the 10 Hz state stream on 30002 (else the controller's send buffer fills)."""
        while self._running and self._sock is s:
            try:
                if not s.recv(65536):
                    break
            except socket.timeout:
                continue
            except OSError:
                break
        if self._sock is s:
            self._sock_dead = True

    def send_program(self, text: str) -> None:
        """Send URScript text on the persistent 30002 socket (reconnects once if it broke). A `def` program
        stops a running program; prefer run_block, which adds markers and error detection."""
        data = (text if text.endswith("\n") else text + "\n").encode("ascii")
        with self._send_lock:
            for attempt in (0, 1):
                try:
                    if self._sock is None or self._sock_dead:
                        self._close_script()
                        self._open_script()
                    self._sock.sendall(data)
                    return
                except OSError as e:
                    self._sock_dead = True
                    if attempt:
                        raise URLinkError(f"cannot send to {self.host}:{self.script_port}: {e}") from e

    # ── blocks ───────────────────────────────────────────────────────────────
    @staticmethod
    def _id_after(i: int) -> int:
        return 1 if i >= 2**31 - 2 or i < 0 else i + 1

    def _new_id(self, s: URState) -> int:
        bid = self._next_id
        while bid in (s.reg_started, s.reg_done):
            bid = self._id_after(bid)
        self._next_id = self._id_after(bid)
        return bid

    def run_block(self, body: str, name: str = "mauer_block", timeout_s: float = 120.0, settle_s: float = 0.0, *,
                  start_timeout_s: float | None = None, interrupt: bool = False,
                  abort_on_timeout: bool = True) -> BlockResult:
        """Run URScript statements `body` (not indented, see mauer.ur.script) as program `name` and wait until it
        has finished with the arm at rest (is_steady) plus `settle_s` [s].

        Returns a BlockResult (never raises for robot-side failures; use .raise_for_error()). Refuses to start when
        the robot is not RUNNING / safety not NORMAL or REDUCED, or when another program is running and
        interrupt=False. On timeout the program is aborted (abort_on_timeout). Raises ValueError for an invalid
        name or a non-ASCII body (caller error, nothing sent).
        """
        script.block_program(name, body, 1, self.reg_started, self.reg_done, self.reg_error)   # validate early
        start_timeout_s = self.start_timeout_s if start_timeout_s is None else float(start_timeout_s)
        with self._block_lock:
            return self._run_block(body, name, float(timeout_s), float(settle_s), start_timeout_s, interrupt,
                                   abort_on_timeout)

    def _run_block(self, body, name, timeout_s, settle_s, start_timeout_s, interrupt, abort_on_timeout):
        res = BlockResult(ok=False, block_id=0, name=name)

        def fail(msg: str, s: URState | None = None) -> BlockResult:
            res.error, res.state = msg, s or self.state()
            if res.state is not None and res.state.reg_error is not None and res.t_started is not None:
                res.error_code = res.state.reg_error
                if res.error_code:
                    res.error += f" (error code {res.error_code})"
            log.error("block %s #%d failed: %s", name, res.block_id, res.error)
            return res

        s0 = self.state()
        if s0 is None or self.state_age_s() > 0.5:          # e.g. a reconnect, or right after the IDS camera opened
            s0 = self.wait_until(lambda x: time.time() - x.t_laptop <= 0.5, FRESH_STATE_WAIT_S)   # (UR5, 2026-10-06)
        if s0 is None or self.state_age_s() > 0.5:
            return fail(f"no current RTDE state (age {self.state_age_s():.2f} s; {self.rtde_error or 'no data'})")
        if s0.robot_mode != ROBOT_RUNNING:
            return fail(f"robot not ready: robot_mode {ROBOT_MODE.get(s0.robot_mode, s0.robot_mode)} - power on and "
                        "release the brakes (Dashboard / tools/ur_check.py ready)", s0)
        if s0.safety_mode not in SAFETY_OK:
            return fail(f"robot not ready: safety_mode {SAFETY_MODE.get(s0.safety_mode, s0.safety_mode)}", s0)
        if s0.program_running or s0.runtime_state != RUNTIME_STOPPED:
            # our previous block ends right after its done marker – give it a moment
            s1 = self.wait_until(lambda s: s.runtime_state == RUNTIME_STOPPED and not s.program_running, 0.5)
            if s1 is None and not interrupt:
                return fail("another program is running on the controller (" + self.state().describe() +
                            "); pass interrupt=True to stop it")
        s0 = self.state()
        res.block_id = bid = self._new_id(s0)
        res.program = script.block_program(name, body, bid, self.reg_started, self.reg_done, self.reg_error)
        res.t_sent = time.time()
        try:
            self.send_program(res.program)
        except URLinkError as e:
            return fail(str(e), s0)
        log.debug("block %s #%d sent", name, bid)

        # ── start marker ──
        s = self.wait_until(lambda s: s.reg_started == bid or s.reg_done == bid or s.safety_mode not in SAFETY_OK,
                            start_timeout_s)
        if s is None:
            return fail(f"did not start within {start_timeout_s:.1f} s - compile/syntax error? The reason is only "
                        "in the PolyScope Log tab (program text in BlockResult.program)")
        if s.reg_started != bid and s.reg_done != bid:
            return fail(f"safety_mode {SAFETY_MODE.get(s.safety_mode, s.safety_mode)} before the block started", s)
        res.t_started = self._first_time(lambda x: x.reg_started == bid or x.reg_done == bid, res.t_sent) or s.t_laptop
        T_tcp = script.parse_set_tcp(body)
        if T_tcp is not None:
            self.tcp_tracked = T_tcp

        # ── done marker / failure ──
        deadline = res.t_sent + timeout_s
        while True:
            s = self.wait_until(lambda s: s.reg_done == bid or s.safety_mode not in SAFETY_OK or
                                (s.runtime_state == RUNTIME_STOPPED and not s.program_running),
                                max(0.0, deadline - time.time()))
            if s is None:
                last = self.state()
                if abort_on_timeout:
                    self.abort()
                return fail(f"timeout after {timeout_s:.1f} s ({last.describe()})"
                            + ("; program aborted" if abort_on_timeout else ""), last)
            if s.reg_done == bid:
                break
            if s.safety_mode not in SAFETY_OK:
                return fail(f"safety_mode {SAFETY_MODE.get(s.safety_mode, s.safety_mode)} during the block "
                            "(protective stop / emergency stop) - inspect, then Dashboard unlock protective stop", s)
            # stopped: the done marker may still be in flight
            s2 = self.wait_until(lambda x: x.reg_done == bid or x.safety_mode not in SAFETY_OK, 0.25)
            if s2 is not None and s2.reg_done == bid:
                s = s2
                break
            s = s2 or self.state()
            if s.safety_mode not in SAFETY_OK:
                return fail(f"safety_mode {SAFETY_MODE.get(s.safety_mode, s.safety_mode)} during the block", s)
            return fail("program stopped without the done marker (runtime error, halt/IK guard, or stopped from "
                        "the pendant/Dashboard - see the PolyScope Log tab)", s)
        res.t_done = self._first_time(lambda x: x.reg_done == bid, res.t_sent) or s.t_laptop
        state = s
        if settle_s > 0.0:
            t_end = res.t_done + settle_s
            time.sleep(max(0.0, t_end - time.time()))
            # a sample received after the settle time, not one from just before it
            state = self.wait_until(lambda x: x.t_laptop >= t_end, 1.0) or self.state()
        res.ok, res.state = True, state
        if res.state.reg_error is not None:
            res.error_code = res.state.reg_error
        log.debug("block %s #%d done: start %.3f s, total %.3f s", name, bid, res.start_latency_s, res.duration_s)
        return res

    def abort(self, dashboard: bool = True) -> str:
        """Stop the running program: send a stopl program on 30002 (interrupts the running program at once), then
        (dashboard=True) Dashboard 'stop'. Returns what was done. Does not raise."""
        done = []
        try:
            self.send_program(script.abort_program())
            done.append("stopl program sent")
        except (URLinkError, OSError) as e:
            done.append(f"stopl program failed: {e}")
        if dashboard:
            try:
                with Dashboard(self.host, self.dashboard_port, timeout_s=2.0) as d:
                    done.append(f"dashboard: {d.stop()}")
            except (OSError, DashboardError) as e:
                done.append(f"dashboard stop failed: {e}")
        msg = "; ".join(done)
        log.warning("abort: %s", msg)
        return msg

    def info(self) -> dict:
        """Connection summary for logs and tools/ur_check.py."""
        return {"host": self.host, "controller_version": self.controller_version, "frequency_hz": self.frequency,
                "fields": list(self.fields), "missing_fields": list(self.missing_fields),
                "tcp_source": self.tcp_source, "registers": {"started": self.reg_started, "done": self.reg_done,
                                                             "error": self.reg_error}}
