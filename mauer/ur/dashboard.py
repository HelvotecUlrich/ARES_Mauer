"""UR Dashboard Server client (CB3, port 29999): power, brakes, protective stop, mode and version queries.

Protocol (UR "Dashboard Server CB-Series, port 29999", support article 15690, research scratch
Dashboard_Server_CB-Series.txt): ASCII, one command per line, one reply line, banner line on connect
("Connected: Universal Robots Dashboard Server"). Commands used here and the PolyScope version they need:
power on/off, brake release, safetymode (3.0); unlock protective stop, close safety popup (3.1); get robot model,
get serial number (3.12); robotmode, close popup (1.6); stop, PolyscopeVersion, programState (1.4/1.8).
Replies seen on URSim CB3 3.15.8 (research 2026-10-05): 'Robotmode: RUNNING', 'Safetymode: NORMAL',
'Protective stop releasing', 'URSoftware 3.15.8.106339 (Jun 20 2022)', 'UR5'.

programState answers 'STOPPED' even while a program sent over port 30002 runs (URSim) – track socket programs with
RTDE runtime_state (mauer.ur.link), not with this class. safetymode lags RTDE safety_mode: ~0.6 s after a protective
stop it still answered NORMAL, and right after 'unlock protective stop' still PROTECTIVE_STOP (URSim 3.15.8,
2026-10-05) – take safety decisions from RTDE.
"""
from __future__ import annotations

import re
import socket
import time

# CB3 robotmode / safetymode reply words (Dashboard doc lines 60-70 and 137-147).
ROBOTMODES = ("NO_CONTROLLER", "DISCONNECTED", "CONFIRM_SAFETY", "BOOTING", "POWER_OFF", "POWER_ON", "IDLE",
              "BACKDRIVE", "RUNNING")
SAFETY_OK = ("NORMAL", "REDUCED")
# Needs the operator at the pendant / safety hardware – never cleared automatically.
SAFETY_OPERATOR = ("SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP", "VIOLATION", "FAULT")


class DashboardError(RuntimeError):
    """Unexpected or failure reply from the Dashboard Server."""


class Dashboard:
    """Blocking Dashboard client. Connects in the constructor (with retries for up to `wait_s`, e.g. while URSim
    boots); use as a context manager or call close()."""

    def __init__(self, host: str, port: int = 29999, timeout_s: float = 10.0, wait_s: float = 0.0):
        if not host:
            raise ValueError("Dashboard: empty host ([ur].host in config/station.toml is a PLACEHOLDER)")
        self.host, self.port, self.timeout_s = host, int(port), float(timeout_s)
        self._sock: socket.socket | None = None
        self._file = None
        self.banner = ""
        deadline = time.time() + float(wait_s)
        while True:
            try:
                self._connect()
                break
            except OSError:
                self.close()
                if time.time() >= deadline:
                    raise
                time.sleep(1.0)

    def _connect(self) -> None:
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self._file = self._sock.makefile("rwb", buffering=0)
        self.banner = self._readline()
        if not self.banner:
            raise ConnectionError("Dashboard: empty banner (server not ready)")

    def close(self) -> None:
        for obj in (self._file, self._sock):
            try:
                if obj is not None:
                    obj.close()
            except OSError:
                pass
        self._file = self._sock = None

    def __enter__(self) -> "Dashboard":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _readline(self) -> str:
        line = self._file.readline()
        if not line:
            raise ConnectionError("Dashboard: connection closed by the robot")
        return line.decode("latin-1").strip()

    # ── raw ──────────────────────────────────────────────────────────────────
    def send(self, cmd: str) -> str:
        """Send one command, return the reply line (stripped)."""
        if self._file is None:
            raise ConnectionError("Dashboard: not connected")
        if "\n" in cmd or "\r" in cmd:
            raise ValueError("Dashboard command must be a single line")
        self._file.write((cmd + "\n").encode("ascii"))
        return self._readline()

    def _expect(self, cmd: str, *ok_prefixes: str) -> str:
        reply = self.send(cmd)
        if not any(reply.lower().startswith(p.lower()) for p in ok_prefixes):
            raise DashboardError(f"Dashboard '{cmd}' -> '{reply}'")
        return reply

    def _value(self, cmd: str, key: str) -> str:
        reply = self.send(cmd)
        m = re.match(rf"\s*{key}\s*:\s*(\S+)", reply, re.IGNORECASE)
        if not m:
            raise DashboardError(f"Dashboard '{cmd}' -> unexpected reply '{reply}'")
        return m.group(1).upper()

    # ── commands ─────────────────────────────────────────────────────────────
    def power_on(self) -> str:
        return self._expect("power on", "Powering on")

    def power_off(self) -> str:
        return self._expect("power off", "Powering off")

    def brake_release(self) -> str:
        """Release the brakes – the joints move slightly; only with a clear workspace."""
        return self._expect("brake release", "Brake releasing")

    def robotmode(self) -> str:
        """'RUNNING', 'POWER_OFF', 'IDLE', ... (CB3 text reply)."""
        return self._value("robotmode", "Robotmode")

    def safetymode(self) -> str:
        """'NORMAL', 'PROTECTIVE_STOP', ..."""
        return self._value("safetymode", "Safetymode")

    def unlock_protective_stop(self) -> str:
        """Fails less than 5 s after the stop ('Cannot unlock protective stop until 5s after occurrence...').
        Inspect the cause first."""
        return self._expect("unlock protective stop", "Protective stop releasing")

    def close_safety_popup(self) -> str:
        return self._expect("close safety popup", "closing safety popup")

    def close_popup(self) -> str:
        return self._expect("close popup", "closing popup")

    def stop(self) -> str:
        """Stop the running program (also a program sent over 30002 – URSim)."""
        return self._expect("stop", "Stopped")

    def program_state(self) -> str:
        """Raw programState reply – NOT valid for programs sent over port 30002 (see module docstring)."""
        return self.send("programState")

    def polyscope_version(self) -> str:
        """e.g. 'URSoftware 3.15.8.106339 (Jun 20 2022)'."""
        return self.send("PolyscopeVersion")

    def version(self) -> tuple[int, int, int]:
        """(major, minor, bugfix) parsed from polyscope_version()."""
        reply = self.polyscope_version()
        m = re.search(r"(\d+)\.(\d+)\.(\d+)", reply)
        if not m:
            raise DashboardError(f"PolyscopeVersion -> unexpected reply '{reply}'")
        return int(m.group(1)), int(m.group(2)), int(m.group(3))

    def robot_model(self) -> str:
        """'UR3' / 'UR5' / 'UR10' (needs PolyScope >= 3.12)."""
        return self.send("get robot model")

    def serial_number(self) -> str:
        """Robot serial number (needs PolyScope >= 3.12)."""
        return self.send("get serial number")

    def wait_ready(self, timeout_s: float = 90.0, poll_s: float = 0.5, unlock: bool = False) -> None:
        """Bring the arm to robotmode RUNNING with safetymode NORMAL/REDUCED: power on, then brake release.

        The brake release moves the joints slightly – call it only with a clear workspace. A protective stop is
        unlocked only with unlock=True (after inspecting its cause); safeguard/emergency stops, violations, faults
        and CONFIRM_SAFETY need the operator and raise DashboardError.
        """
        deadline = time.time() + timeout_s
        t_power = t_brake = t_unlock = -1e9
        while True:
            mode, safety = self.robotmode(), self.safetymode()
            if mode == "RUNNING" and safety in SAFETY_OK:
                return
            if safety in SAFETY_OPERATOR:
                raise DashboardError(f"safetymode {safety}: needs the operator (pendant / safety circuit)")
            if mode == "CONFIRM_SAFETY":
                raise DashboardError("robotmode CONFIRM_SAFETY: confirm the safety configuration on the pendant")
            now = time.time()
            if safety == "PROTECTIVE_STOP":
                if not unlock:
                    raise DashboardError("protective stop: inspect the cause, then unlock (wait_ready(unlock=True))")
                if now - t_unlock > 6.0:
                    t_unlock = now
                    try:
                        self.unlock_protective_stop()
                    except DashboardError:
                        pass  # < 5 s after the stop: retried after 6 s
            elif mode == "POWER_OFF" and now - t_power > 10.0:
                t_power = now
                self.power_on()
            elif mode == "IDLE" and now - t_brake > 10.0:
                t_brake = now
                self.brake_release()
            if now > deadline:
                raise DashboardError(f"not ready after {timeout_s:.0f} s: robotmode {mode}, safetymode {safety}")
            time.sleep(poll_s)
