"""mauer.ur.link / mauer.ur.dashboard against a fake UR controller on localhost (no robot, no Docker).

The fake speaks the real wire protocols, so the vendored RTDE client, the persistent 30002 socket with its drain
thread and the Dashboard client run unchanged:
- RTDE (port 0 = ephemeral): protocol/version handshake, output recipe ('NOT_FOUND' for fields marked missing),
  data packages at ~125 Hz. protocol=1 speaks RTDE protocol version 1 only, like PolyScope 3.3 (the lab's UR5,
  2026-10-06): no output frequency in the setup request, no recipe id in the setup reply and the data packages.
- 30002: collects `def ... end` programs; a minimal interpreter executes write_output_integer_register, sleep,
  set_tcp, set_standard_digital_out, halt and the test directive '# FAKE_PSTOP' (protective stop); a line with
  unbalanced brackets = compile error (never runs). A new program interrupts the running one. It also streams junk
  at ~400 kB/s that the link must drain.
- Dashboard: banner + the CB3 replies used by mauer.ur.dashboard.
"""
import re
import socket
import struct
import threading
import time

import numpy as np
import pytest

from mauer import geometry as g
from mauer.ur import script as s
from mauer.ur.dashboard import Dashboard, DashboardError
from mauer.ur.link import ROBOT_MODE, URLink, URLinkError, flange_T

WIRE = {"DOUBLE": "d", "VECTOR6D": "6d", "INT32": "i", "UINT32": "I", "UINT64": "Q"}
TYPES = {"timestamp": "DOUBLE", "actual_q": "VECTOR6D", "actual_qd": "VECTOR6D", "actual_TCP_pose": "VECTOR6D",
         "target_TCP_pose": "VECTOR6D", "tcp_offset": "VECTOR6D", "robot_mode": "INT32", "safety_mode": "INT32",
         "runtime_state": "UINT32", "robot_status_bits": "UINT32", "tool_output_voltage": "INT32",
         "actual_digital_output_bits": "UINT64"}


class FakeUR:
    def __init__(self, missing=(), regs=None, protocol=2, version=(3, 15, 8, 0)):
        self.t0 = time.time()
        self.protocol, self.version = protocol, tuple(version)
        self.lock = threading.Lock()
        self.stop_ev = threading.Event()
        self.missing = set(missing)
        self.q = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        self.tcp_pose = [0.1, -0.2, 0.3, 0.0, 3.0, 0.0]
        self.tcp_offset = [0.0] * 6
        self.robot_mode, self.safety_mode, self.runtime, self.bits = 7, 1, 1, 0x1
        self.regs = dict(regs or {})
        self.dout, self.voltage = 0, 0
        self.programs: list[str] = []
        self.dash_cmds: list[str] = []
        self.junk_sent = self.send_timeouts = 0
        self.runner: threading.Thread | None = None
        self.interrupt = threading.Event()
        self.rtde_conns: list[socket.socket] = []
        self.threads: list[threading.Thread] = []
        self.ports = {}
        for name, handler in (("rtde", self._rtde_conn), ("script", self._script_conn), ("dash", self._dash_conn)):
            srv = socket.create_server(("127.0.0.1", 0))
            srv.settimeout(0.05)
            self.ports[name] = srv.getsockname()[1]
            self._spawn(self._accept, srv, handler)

    def _spawn(self, fn, *args):
        t = threading.Thread(target=fn, args=args, daemon=True)
        t.start()
        self.threads.append(t)

    def close(self):
        self.stop_ev.set()
        self.interrupt.set()
        for t in self.threads:
            t.join(timeout=2.0)

    def _accept(self, srv, handler):
        while not self.stop_ev.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            self._spawn(handler, conn)
        srv.close()

    # ── RTDE ──
    def value(self, name):
        if name == "timestamp":
            return time.time() - self.t0
        m = re.fullmatch(r"output_int_register_(\d+)", name)
        if m:
            return self.regs.get(int(m.group(1)), 0)
        return {"actual_q": self.q, "actual_qd": [0.0] * 6, "actual_TCP_pose": self.tcp_pose,
                "target_TCP_pose": self.tcp_pose, "tcp_offset": self.tcp_offset, "robot_mode": self.robot_mode,
                "safety_mode": self.safety_mode, "runtime_state": self.runtime, "robot_status_bits": self.bits,
                "tool_output_voltage": self.voltage, "actual_digital_output_bits": self.dout}[name]

    def type_of(self, name):
        if name in self.missing:
            return "NOT_FOUND"
        if re.fullmatch(r"output_int_register_(\d+)", name) and int(name.rsplit("_", 1)[1]) <= 47:
            return "INT32"
        return TYPES.get(name, "NOT_FOUND")

    def _rtde_conn(self, conn):
        self.rtde_conns.append(conn)
        conn.settimeout(0.004)
        buf, recipe, streaming, t_next, proto = b"", None, False, 0.0, 1

        def send(cmd, payload=b""):
            conn.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)

        try:
            while not self.stop_ev.is_set():
                try:
                    data = conn.recv(4096)
                    if not data:
                        break
                    buf += data
                except socket.timeout:
                    pass
                while len(buf) >= 3:
                    size, cmd = struct.unpack_from(">HB", buf)
                    if len(buf) < size:
                        break
                    payload, buf = buf[3:size], buf[size:]
                    if cmd == ord("V"):
                        want = struct.unpack(">H", payload)[0]
                        if want <= self.protocol:
                            proto = want
                        send(cmd, b"\x01" if want <= self.protocol else b"\x00")
                    elif cmd == ord("v"):
                        send(cmd, struct.pack(">IIII", *self.version))
                    elif cmd == ord("O"):
                        names = (payload[8:] if proto >= 2 else payload).decode().split(",")
                        types = [self.type_of(n) for n in names]
                        recipe = None if "NOT_FOUND" in types else (names, types)
                        send(cmd, (bytes([1]) if proto >= 2 else b"") + ",".join(types).encode())
                    elif cmd == ord("S"):
                        streaming = recipe is not None
                        send(cmd, b"\x01" if streaming else b"\x00")
                    elif cmd == ord("P"):
                        streaming = False
                        send(cmd, b"\x01")
                if streaming and time.time() >= t_next:
                    t_next = time.time() + 0.008
                    names, types = recipe
                    with self.lock:
                        vals = []
                        for n, t in zip(names, types):
                            v = self.value(n)
                            vals += list(v) if t == "VECTOR6D" else [v]
                    rid = [1] if proto >= 2 else []
                    fmt = ">" + "B" * len(rid) + "".join(WIRE[t] for t in types)
                    send(ord("U"), struct.pack(fmt, *rid, *vals))
        except OSError:
            pass
        conn.close()

    def kick_rtde(self):
        for c in list(self.rtde_conns):
            try:
                c.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    # ── 30002 ──
    def _script_conn(self, conn):
        conn.settimeout(0.01)
        buf, t_junk = "", time.time()
        try:
            while not self.stop_ev.is_set():
                try:
                    data = conn.recv(65536)
                    if not data:
                        break
                    buf += data.decode("ascii")
                except socket.timeout:
                    pass
                while (m := re.search(r"^(def|sec) \w+\(\):\n.*?^end\n", buf, re.S | re.M)):
                    buf = buf[m.end():]
                    self.run_program(m.group(0))
                if time.time() >= t_junk:   # the controller streams state on 30002 – the client must drain it
                    t_junk += 0.01
                    try:
                        conn.send(b"\x00" * 4096)
                        self.junk_sent += 4096
                    except socket.timeout:
                        self.send_timeouts += 1
        except OSError:
            pass
        conn.close()

    def run_program(self, prog):
        self.programs.append(prog)
        if self.runner is not None and self.runner.is_alive():
            self.interrupt.set()
            self.runner.join()
        if any(ln.count("(") != ln.count(")") or ln.count("[") != ln.count("]") for ln in prog.splitlines()):
            return   # compile error: never runs
        self.interrupt = threading.Event()
        self.runner = threading.Thread(target=self._exec, args=(prog, self.interrupt), daemon=True)
        self.runner.start()

    def _exec(self, prog, ev):
        with self.lock:
            self.runtime, self.bits = 2, self.bits | 0x2
        for raw in prog.splitlines()[1:-1]:
            ln = raw.strip()
            if ev.is_set():
                break
            if m := re.fullmatch(r"write_output_integer_register\((\d+), (-?\d+)\)", ln):
                with self.lock:
                    self.regs[int(m.group(1))] = int(m.group(2))
            elif m := re.fullmatch(r"sleep\(([\d.]+)\)", ln):
                if ev.wait(float(m.group(1))):
                    break
            elif m := re.fullmatch(r"set_tcp\(p\[(.*)\]\)", ln):
                with self.lock:
                    self.tcp_offset = [float(v) for v in m.group(1).split(",")]
            elif m := re.fullmatch(r"set_standard_digital_out\((\d), (True|False)\)", ln):
                with self.lock:
                    bit = 1 << int(m.group(1))
                    self.dout = self.dout | bit if m.group(2) == "True" else self.dout & ~bit
            elif ln == "halt":
                break
            elif ln == "# FAKE_PSTOP":
                with self.lock:
                    self.safety_mode = 3
                break
        with self.lock:
            self.runtime, self.bits = 1, self.bits & ~0x2

    # ── Dashboard ──
    def _dash_conn(self, conn):
        conn.sendall(b"Connected: Universal Robots Dashboard Server\n")
        conn.settimeout(0.05)
        buf = b""
        try:
            while not self.stop_ev.is_set():
                try:
                    data = conn.recv(4096)
                    if not data:
                        break
                    buf += data
                except socket.timeout:
                    continue
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode().strip()
                    self.dash_cmds.append(cmd)
                    conn.sendall((self.dash_reply(cmd) + "\n").encode())
        except OSError:
            pass
        conn.close()

    def _later(self, dt, **kw):
        def go():
            time.sleep(dt)
            with self.lock:
                for k, v in kw.items():
                    setattr(self, k, v)
        threading.Thread(target=go, daemon=True).start()

    def dash_reply(self, cmd):
        if cmd == "robotmode":
            return f"Robotmode: {ROBOT_MODE[self.robot_mode]}"
        if cmd == "safetymode":
            return "Safetymode: " + {1: "NORMAL", 3: "PROTECTIVE_STOP", 7: "ROBOT_EMERGENCY_STOP"}[self.safety_mode]
        if cmd == "power on":
            self._later(0.1, robot_mode=5)
            return "Powering on"
        if cmd == "brake release":
            self._later(0.1, robot_mode=7)
            return "Brake releasing"
        if cmd == "stop":
            self.interrupt.set()
            return "Stopped"
        if cmd == "unlock protective stop":
            self.safety_mode = 1
            return "Protective stop releasing"
        return {"PolyscopeVersion": "URSoftware 3.15.8.106339 (Jun 20 2022)", "get robot model": "UR5",
                "close popup": "closing popup", "close safety popup": "closing safety popup"}.get(
            cmd, f"could not understand: '{cmd}'")


@pytest.fixture
def fake():
    f = FakeUR(regs={24: 41, 25: 40})
    yield f
    f.close()


def make_link(f: FakeUR, **kw) -> URLink:
    return URLink("127.0.0.1", rtde_port=f.ports["rtde"], script_port=f.ports["script"],
                  dashboard_port=f.ports["dash"], **kw).start()


def test_state_stream_and_history(fake):
    with make_link(fake, history_s=0.5) as ur:
        assert ur.controller_version == (3, 15, 8, 0) and ur.missing_fields == []
        t0 = time.time()
        time.sleep(0.6)
        st = ur.state()
        assert st.actual_q.shape == (6,) and st.robot_mode == 7 and st.reg_started == 41 and st.reg_done == 40
        assert not st.actual_q.flags.writeable
        assert ur.state_age_s() < 0.1
        smp = ur.samples(t0)
        assert len(smp) > 30                                         # ~125 Hz, 0.5 s history
        assert all(b.t_laptop >= a.t_laptop and b.timestamp >= a.timestamp for a, b in zip(smp, smp[1:]))
        assert smp[0].t_laptop >= time.time() - 0.5 - 0.05          # pruned to history_s
        mid = smp[len(smp) // 2].t_laptop
        assert all(x.t_laptop <= mid for x in ur.samples(t0, mid))
        assert ur.wait_until(lambda x: x.timestamp > st.timestamp + 0.05, 1.0) is not None
        assert ur.wait_until(lambda x: False, 0.05) is None
        assert set(ur.info()["fields"]) >= {"tcp_offset", "output_int_register_24", "output_int_register_26"}


def test_blocks_ids_markers_and_times(fake):
    with make_link(fake) as ur:
        ids = []
        for k in range(3):
            r = ur.run_block("sleep(0.05)", f"blk{k}", timeout_s=5.0)
            assert r.ok, r.error
            ids.append(r.block_id)
            assert r.t_sent <= r.t_started <= r.t_done and r.duration_s >= 0.05
            assert r.program == s.block_program(f"blk{k}", "sleep(0.05)", r.block_id)
            assert r.error_code == 0 and r.state.reg_done == r.block_id
        assert ids == [42, 43, 44]                                   # after the larger marker found at start
        assert len(fake.programs) == 3
        r = ur.run_block("", "settle", timeout_s=5.0, settle_s=0.2)
        assert r.ok and r.state.t_laptop >= r.t_done + 0.2
        # the 30002 state stream was drained the whole time
        assert fake.junk_sent > 100_000 and fake.send_timeouts == 0


def test_compile_error_then_recovers(fake):
    with make_link(fake) as ur:
        r = ur.run_block("movel(p[0,0,0,0,0,0)", "bad", timeout_s=5.0, start_timeout_s=0.3)
        assert not r.ok and "did not start" in r.error and "PolyScope Log" in r.error
        assert r.t_started is None and "movel(p[0,0,0,0,0,0)" in r.program
        assert ur.run_block("sleep(0.01)", "good", timeout_s=5.0).ok
        n = len(fake.programs)
        for name, body in (("1bad", ""), ("halt", ""), ("ok", 'textmsg("°")')):
            with pytest.raises(ValueError):
                ur.run_block(body, name)                             # caller error: raised, nothing sent
        assert len(fake.programs) == n


def test_stopped_without_done_reports_error_code(fake):
    with make_link(fake) as ur:
        r = ur.run_block(f"write_output_integer_register({s.REG_ERROR}, {s.ERR_IK_UNREACHABLE})\nhalt", "halted",
                         timeout_s=5.0)
        assert not r.ok and "without the done marker" in r.error
        assert r.error_code == s.ERR_IK_UNREACHABLE and r.t_started is not None and r.t_done is None
        assert ur.run_block("", "next", timeout_s=5.0).error_code == 0   # reset at every block start


def test_protective_stop_detected_and_blocks_refused(fake):
    with make_link(fake) as ur:
        r = ur.run_block("sleep(0.05)\n# FAKE_PSTOP\nsleep(1.0)", "pstop", timeout_s=5.0)
        assert not r.ok and "PROTECTIVE_STOP" in r.error and r.duration_s is None
        n = len(fake.programs)
        r2 = ur.run_block("", "after", timeout_s=5.0)
        assert not r2.ok and "not ready" in r2.error and len(fake.programs) == n   # nothing sent


def test_not_ready_and_running_program(fake):
    with make_link(fake) as ur:
        fake.robot_mode = 3
        time.sleep(0.05)
        r = ur.run_block("", "x", timeout_s=1.0)
        assert not r.ok and "POWER_OFF" in r.error and not fake.programs
        fake.robot_mode = 7
        fake.run_program("def other():\n  sleep(3.0)\nend\n")      # e.g. started from the pendant
        time.sleep(0.05)
        r = ur.run_block("", "x", timeout_s=1.0)
        assert not r.ok and "another program is running" in r.error
        assert ur.run_block("", "x", timeout_s=1.0, interrupt=True).ok


def test_timeout_aborts(fake):
    with make_link(fake) as ur:
        t0 = time.time()
        r = ur.run_block("sleep(5.0)", "slow", timeout_s=0.3)
        assert not r.ok and "timeout" in r.error and "aborted" in r.error and time.time() - t0 < 2.0
        assert fake.programs[-1] == s.abort_program() and "stop" in fake.dash_cmds


def test_flange_from_tcp_offset(fake):
    T_ft = g.transl(0, 0, 135) @ g.rotz(np.pi / 2)
    T_bf = g.pose_xyz_rpy([-450, -100, 430], [180, 0, 25])
    fake.tcp_offset, fake.tcp_pose = g.T_to_ur(T_ft), g.T_to_ur(T_bf @ T_ft)
    with make_link(fake) as ur:
        st = ur.state()
        assert ur.tcp_source == "rtde tcp_offset"
        assert g.pose_delta(flange_T(st), T_bf)[0] < 1e-6
        assert g.pose_delta(ur.flange_T(), T_bf)[0] < 1e-6
        assert g.pose_delta(st.T_base_tcp_mm(), T_bf @ T_ft)[0] < 1e-6


def test_missing_optional_fields_and_tracked_tcp():
    f = FakeUR(missing={"tcp_offset", "target_TCP_pose"})
    try:
        T_ft = g.transl(0, 0, 135) @ g.rotz(np.pi / 2)
        T_bf = g.pose_xyz_rpy([-450, -100, 430], [180, 0, 25])
        f.tcp_pose = g.T_to_ur(T_bf @ T_ft)
        with make_link(f) as ur:
            assert sorted(ur.missing_fields) == ["target_TCP_pose", "tcp_offset"]
            st = ur.state()
            assert st.tcp_offset is None and st.target_TCP_pose is None and ur.tcp_source == "tracked set_tcp"
            with pytest.raises(ValueError):
                st.T_base_flange_mm()
            assert ur.run_block(s.set_tcp(T_ft) + "\nsleep(0.01)", "tcp", timeout_s=5.0).ok
            assert np.allclose(ur.tcp_tracked, T_ft, atol=1e-6)
            assert g.pose_delta(ur.flange_T(), T_bf)[0] < 1e-6
    finally:
        f.close()


def test_missing_required_field_raises():
    f = FakeUR(missing={"runtime_state"})
    try:
        with pytest.raises(URLinkError, match="runtime_state"):
            make_link(f)
    finally:
        f.close()


def test_rtde_protocol_v1_polyscope_3_3():
    """PolyScope 3.3.3 (the lab's UR5, 2026-10-06) refuses RTDE protocol v2; the client falls back to v1 (no output
    frequency, no recipe ids). Before the fix every field came back MISSING ('Unknown data type: OT_FOUND')."""
    f = FakeUR(regs={24: 41, 25: 40}, protocol=1, version=(3, 3, 3, 292))
    try:
        with make_link(f) as ur:
            assert ur.controller_version == (3, 3, 3, 292) and ur.missing_fields == []
            assert ur.wait_until(lambda x: x.reg_started == 41, 2.0) is not None
            st = ur.state()
            assert np.allclose(st.actual_q, f.q) and st.robot_mode == 7 and st.reg_done == 40
            assert g.pose_delta(st.T_base_tcp_mm(), g.ur_to_T(f.tcp_pose))[0] < 1e-6
            assert ur.run_block("", "v1_block", timeout_s=2.0).ok
    finally:
        f.close()


def test_rtde_reconnects(fake):
    with make_link(fake) as ur:
        fake.kick_rtde()
        assert ur.wait_until(lambda x: False, 0.3) is None
        t_kick = time.time()
        assert ur.wait_until(lambda x: x.t_laptop > t_kick + 0.1, 5.0) is not None   # stream is back
        assert ur.run_block("", "after_reconnect", timeout_s=2.0).ok


def test_digital_outputs_in_state(fake):
    with make_link(fake) as ur:
        r = ur.run_block(s.gripper("close", 0, 1, pulse_s=0.0, wait_s=0.05), "grip", timeout_s=5.0)
        assert r.ok and r.state.digital_out(1) is True and r.state.digital_out(0) is False


def test_dashboard_client(fake):
    fake.robot_mode = 3
    with Dashboard("127.0.0.1", fake.ports["dash"], timeout_s=2.0) as d:
        assert d.banner.startswith("Connected: Universal Robots Dashboard Server")
        assert d.robotmode() == "POWER_OFF" and d.safetymode() == "NORMAL"
        assert d.version() == (3, 15, 8) and d.robot_model() == "UR5"
        d.wait_ready(timeout_s=5.0, poll_s=0.05)                     # power on -> IDLE -> brake release -> RUNNING
        assert d.robotmode() == "RUNNING"
        assert fake.dash_cmds.count("power on") == 1 and fake.dash_cmds.count("brake release") == 1
        with pytest.raises(DashboardError, match="could not understand"):
            d._expect("bogus", "ok")
        fake.safety_mode = 3
        with pytest.raises(DashboardError, match="protective stop"):
            d.wait_ready(timeout_s=1.0, poll_s=0.05)
        d.wait_ready(timeout_s=2.0, poll_s=0.05, unlock=True)
        assert d.safetymode() == "NORMAL"
        fake.safety_mode = 7
        with pytest.raises(DashboardError, match="operator"):
            d.wait_ready(timeout_s=1.0, poll_s=0.05)
        assert d.stop() == "Stopped" and d.close_popup() == "closing popup"
    with pytest.raises(ValueError):
        Dashboard("")


def test_link_from_config_requires_host():
    with pytest.raises(ValueError, match="PLACEHOLDER"):
        URLink.from_config({"ur": {"host": ""}})
    ur = URLink.from_config({"ur": {"host": "10.0.0.2", "reg_started": 30, "reg_done": 31,
                                    "block_start_timeout_s": 2.0}})
    assert (ur.host, ur.reg_started, ur.reg_done, ur.reg_error, ur.start_timeout_s) == ("10.0.0.2", 30, 31, 26, 2.0)
