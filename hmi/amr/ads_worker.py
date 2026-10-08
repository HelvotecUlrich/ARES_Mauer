"""
ads_worker.py - ADS communication in its own thread (spec 6.1).

AdsWorker is a QObject that is moved to a QThread and is the ONLY owner of the pyads connection:
  * poll timer (hmi.poll_interval_ms, default 100 ms): one sum-read (read_list_by_name) of stFromPlc
  * heartbeat timer (hmi.heartbeat_interval_ms, default 100 ms): nHeartbeat += 1
  * command queue: send()/pulse()/halt() may be called from any thread; each logical command is ONE
    write_list_by_name including nCommandId; pulses are reset (FALSE) 300 ms later by the worker itself
  * reconnect inside the worker (hmi.reconnect_interval_s); after every (re)connect and BEFORE the first
    heartbeat the safe defaults are written (all jog bits, horn, move start/abort, mode requests FALSE)
  * interface detection: GVL_HMI.stFromPlc.nIfVersion readable -> v2 lists; legacy lists (no move) ONLY if the
    symbol does not exist ("symbol not found", ADS 1808). Any other detection error (timeout, connection) fails the
    connect and the reconnect timer retries - a v2 PLC is never run as legacy because of a transient error.
  * v2 symbol check on connect: every stFromPlc / stToPlc field of the v2 lists must exist. A v2 PLC without some
    of them (e.g. an older v2 build without eMoveCmdResult / bMoveLimited, spec section 9) is NOT run with a
    reduced list: the connect fails with "PLC build does not match this HMI: ... missing: <fields>", nothing is
    written, and the reconnect timer retries (connects as soon as the matching build runs). Without this check the
    first poll would fail with pyads "symbol not found (1808)" and the HMI would loop connect/disconnect.
  * ads.timeout_ms is applied with Connection.set_timeout()

pyads is imported lazily, so the module (and the tests) work without pyads / without a PLC.
"""

from __future__ import annotations

import itertools
import logging
import queue
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, Optional, Tuple

from PySide6.QtCore import QObject, Qt, QTimer, Signal, Slot

from . import plc_vars as pv
from .logic import halt_values

log = logging.getLogger(__name__)

PULSE_MS = 300
PRIO_HALT = 0
PRIO_NORMAL = 1
ADSERR_SYMBOL_NOT_FOUND = 1808    # pyads.ADSError.err_code "symbol not found"
MISSING_NAMES_SHOWN = 4


def is_symbol_not_found(exc: BaseException) -> bool:
    """True if a pyads error means 'symbol not found' (ADS 1808), not a connection / timeout problem."""
    return getattr(exc, "err_code", None) == ADSERR_SYMBOL_NOT_FOUND or "symbol not found" in str(exc).lower()


class InterfaceMismatch(Exception):
    """The PLC reports interface v2 but lacks symbols of this HMI's v2 lists (different PLC build)."""

    def __init__(self, missing: Iterable[str]) -> None:
        self.missing = list(missing)
        shown = ", ".join(self.missing[:MISSING_NAMES_SHOWN])
        more = f" (+{len(self.missing) - MISSING_NAMES_SHOWN} more)" if len(self.missing) > MISSING_NAMES_SHOWN else ""
        super().__init__(f"{pv.MISMATCH_TEXT}: interface v2 but missing {shown}{more} - "
                         "load the matching PLC build v2")


@dataclass
class _Cmd:
    values: Dict[str, Any]
    label: str
    epoch: int
    pulse_fields: Tuple[str, ...] = ()
    pulse_ms: int = PULSE_MS
    pulse_gen: Dict[str, int] = field(default_factory=dict)   # only for pulse resets


def _default_connection_factory(ads_cfg: Dict[str, Any]) -> Callable[[], Any]:
    """Factory creating a pyads.Connection (pyads imported only here)."""
    def make() -> Any:
        import pyads  # noqa: WPS433 - lazy import, not needed for tests
        host = ads_cfg.get("host_ip") or None
        if host:
            return pyads.Connection(ads_cfg["ams_net_id"], int(ads_cfg["ads_port"]), host)
        return pyads.Connection(ads_cfg["ams_net_id"], int(ads_cfg["ads_port"]))
    return make


class AdsWorker(QObject):
    """ADS owner thread object. Signals are delivered to the GUI thread (queued)."""

    status = Signal(dict)              # stFromPlc fields (missing -> type default)
    connection = Signal(bool, str)     # connected, message
    interface = Signal(int, str)       # interface version (1 legacy / 2), PLC build text
    command_error = Signal(str)        # write not done / failed (operator feedback)

    _wake = Signal()
    _stop_req = Signal()

    def __init__(self, cfg: Dict[str, Any],
                 connection_factory: Optional[Callable[[], Any]] = None,
                 scheduler: Optional[Callable[[int, Callable[[], None]], None]] = None) -> None:
        super().__init__()
        ads = cfg.get("ads", {}) or {}
        plc = cfg.get("plc", {}) or {}
        hmi = cfg.get("hmi", {}) or {}
        self._timeout_ms = int(ads.get("timeout_ms", 1000))
        self._from_prefix = str(plc.get("from_plc_struct", "GVL_HMI.stFromPlc")) + "."
        self._to_prefix = str(plc.get("to_plc_struct", "GVL_HMI.stToPlc")) + "."
        self._poll_ms = int(hmi.get("poll_interval_ms", hmi.get("update_interval_ms", 100)))
        self._hb_ms = int(hmi.get("heartbeat_interval_ms", 100))
        self._reconnect_ms = int(float(hmi.get("reconnect_interval_s", 5)) * 1000)
        self._factory = connection_factory or _default_connection_factory(ads)
        self._schedule = scheduler or (lambda ms, fn: QTimer.singleShot(ms, fn))

        self._plc: Any = None
        self._connected = False          # read from other threads (atomic bool)
        self._hb_enabled = False
        self._epoch = 0
        self._if_version = pv.IF_LEGACY
        self._plc_build = ""
        self._from_vars: Dict[str, str] = pv.from_plc_vars(pv.IF_LEGACY)
        self._to_vars: Dict[str, str] = pv.to_plc_vars(pv.IF_LEGACY)
        self._from_symbols: list = []
        self._hb = 0
        self._cmd_id = 0
        self._pulse_gen: Dict[str, int] = {}
        self._queue: "queue.PriorityQueue[Tuple[int, int, _Cmd]]" = queue.PriorityQueue()
        self._seq = itertools.count()
        self._lock = threading.Lock()    # guards _epoch/_connected snapshot for enqueue
        self._timers: list = []
        self._stopped = False
        self._worker_tid: Optional[int] = None   # set in start() (worker thread)

        self._wake.connect(self.process_pending)
        # emitted only from a foreign thread (see request_stop) -> returns after _shutdown ran
        self._stop_req.connect(self._shutdown, Qt.BlockingQueuedConnection)

    # ── properties (thread-safe reads) ────────────────────────────────────────
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def if_version(self) -> int:
        return self._if_version

    # ── API for the GUI thread ────────────────────────────────────────────────
    def send(self, values: Dict[str, Any], label: str = "") -> None:
        """Queue ONE write of the given stToPlc fields (+ nCommandId)."""
        self._enqueue(_Cmd(dict(values), label or ",".join(values), self._epoch), PRIO_NORMAL)

    def pulse(self, values: Dict[str, Any], pulse_fields: Iterable[str], label: str = "",
              ms: int = PULSE_MS) -> None:
        """ONE write of values (pulse fields TRUE); the pulse fields are written FALSE after ms."""
        vals = dict(values)
        pf = tuple(pulse_fields)
        for f in pf:
            vals.setdefault(f, True)
        self._enqueue(_Cmd(vals, label or ",".join(pf), self._epoch, pf, int(ms)), PRIO_NORMAL)

    def halt(self) -> None:
        """HALT (spec 6.2): priority write bCmdMoveAbort TRUE + all jog bits FALSE; abort FALSE after 300 ms."""
        vals = halt_values(self._if_version)
        pf = ("bCmdMoveAbort",) if "bCmdMoveAbort" in vals else ()
        self._enqueue(_Cmd(vals, "HALT", self._epoch, pf, PULSE_MS), PRIO_HALT)

    def request_stop(self) -> None:
        """Stop timers, release jog bits and close the connection (blocks until done)."""
        if self._worker_tid is None or self._worker_tid == threading.get_ident():
            self._shutdown()           # not started in a thread (tests) or called from the worker itself
        else:
            self._stop_req.emit()      # BlockingQueuedConnection

    # ── worker-thread slots ───────────────────────────────────────────────────
    @Slot()
    def start(self) -> None:
        """Create the timers in the worker thread and connect immediately."""
        self._worker_tid = threading.get_ident()
        poll = QTimer(self)
        poll.setInterval(self._poll_ms)
        poll.timeout.connect(self.poll_once)
        hb = QTimer(self)
        hb.setInterval(self._hb_ms)
        hb.timeout.connect(self.heartbeat_once)
        rc = QTimer(self)
        rc.setInterval(self._reconnect_ms)
        rc.timeout.connect(self._reconnect_tick)
        self._timers = [poll, hb, rc]
        for t in self._timers:
            t.start()
        self.connect_now()

    def _reconnect_tick(self) -> None:
        if not self._connected and not self._stopped:
            self.connect_now()

    def connect_now(self) -> bool:
        """(Re)connect: open, set timeout, detect interface, write safe defaults, then enable heartbeat."""
        self._close()
        plc: Any = None
        try:
            plc = self._factory()
            plc.open()
            try:
                plc.set_timeout(self._timeout_ms)
            except Exception as exc:       # pragma: no cover - depends on the ADS router
                log.warning("set_timeout(%s) failed: %s", self._timeout_ms, exc)
            version, build = self._detect_interface(plc)
            if version >= pv.IF_V2:
                self._check_symbols(plc, version)      # raises InterfaceMismatch
            self._plc = plc
            self._set_version(version, build)
            with self._lock:
                self._epoch += 1           # commands queued before this connect are dropped
            self._write_raw(pv.safe_default_values(version), "safe defaults")
            self._connected = True
            self._hb_enabled = True
        except InterfaceMismatch as exc:
            self._discard(plc)
            msg = str(exc)
            log.error("%s (symbols: %s)", msg, exc.missing)
            self.connection.emit(False, msg)
            return False
        except Exception as exc:
            self._discard(plc)
            msg = f"connect failed: {exc}"
            log.warning(msg)
            self.connection.emit(False, msg)
            return False
        log.info("ADS connected, interface v%d (%s)", self._if_version, self._plc_build or "-")
        self.connection.emit(True, "connected")
        self.interface.emit(self._if_version, self._plc_build)
        self.heartbeat_once()
        self.process_pending()
        return True

    def poll_once(self) -> Optional[Dict[str, Any]]:
        """One sum-read of all FROM fields; emits status. Read error -> disconnect."""
        if not self._connected or self._plc is None:
            return None
        try:
            raw = self._plc.read_list_by_name(self._from_symbols, cache_symbol_info=True)
        except Exception as exc:
            self._lost(f"read failed: {exc}")
            return None
        data: Dict[str, Any] = {}
        for fld, typ in self._from_vars.items():
            val = raw.get(self._from_prefix + fld)
            data[fld] = pv.TYPE_DEFAULTS[typ] if val is None else val
        self.status.emit(data)
        return data

    def heartbeat_once(self) -> bool:
        if not (self._connected and self._hb_enabled) or self._plc is None:
            return False
        self._hb = (self._hb + 1) & 0xFFFF
        try:
            self._write_raw({"nHeartbeat": self._hb}, "heartbeat", with_cmd_id=False)
            return True
        except Exception as exc:
            self._lost(f"heartbeat write failed: {exc}")
            return False

    @Slot()
    def process_pending(self) -> None:
        """Drain the command queue (worker thread)."""
        while True:
            try:
                _prio, _seq, cmd = self._queue.get_nowait()
            except queue.Empty:
                return
            if not self._connected or cmd.epoch != self._epoch:
                if not cmd.pulse_gen:      # silently drop stale pulse resets (safe defaults cover them)
                    self.command_error.emit(f"{cmd.label}: not sent (not connected)")
                continue
            if cmd.pulse_gen and any(self._pulse_gen.get(f) != g for f, g in cmd.pulse_gen.items()):
                continue                   # a newer pulse of this field owns the reset
            values = {k: v for k, v in cmd.values.items() if k in self._to_vars}
            dropped = sorted(set(cmd.values) - set(values))
            if dropped:
                log.warning("%s: fields not in PLC interface v%d: %s", cmd.label, self._if_version, dropped)
            if not values:
                self.command_error.emit(f"{cmd.label}: not available on this PLC (interface v{self._if_version})")
                continue
            try:
                self._write_raw(values, cmd.label)
            except _WriteRejected as exc:
                self.command_error.emit(f"{cmd.label}: {exc}")
                continue
            except Exception as exc:
                self.command_error.emit(f"{cmd.label}: write failed ({exc})")
                self._lost(f"write failed: {exc}")
                return
            pf = tuple(f for f in cmd.pulse_fields if f in values)
            if pf:
                gen = {}
                for f in pf:
                    self._pulse_gen[f] = self._pulse_gen.get(f, 0) + 1
                    gen[f] = self._pulse_gen[f]
                reset = _Cmd({f: False for f in pf}, cmd.label + " (reset)", cmd.epoch, (), 0, gen)
                self._schedule(cmd.pulse_ms, lambda r=reset: self._enqueue_and_process(r))

    # ── internals ─────────────────────────────────────────────────────────────
    def _enqueue(self, cmd: _Cmd, prio: int) -> None:
        self._queue.put((prio, next(self._seq), cmd))
        self._wake.emit()

    def _enqueue_and_process(self, cmd: _Cmd) -> None:
        self._queue.put((PRIO_NORMAL, next(self._seq), cmd))
        self.process_pending()

    def _detect_interface(self, plc: Any) -> Tuple[int, str]:
        """nIfVersion readable -> v2 (build text read too); legacy ONLY if nIfVersion does not exist (ADS 1808).

        Any other error of the detection read (timeout, connection, unexpected value) is re-raised, so the connect
        fails and the reconnect timer retries. A v2 PLC must never be run with the legacy lists because of a
        transient error: no move support, no v2 safe defaults, and HALT would not send bCmdMoveAbort.
        """
        ifv = self._from_prefix + "nIfVersion"
        build = self._from_prefix + "sPlcBuild"
        try:
            raw = plc.read_list_by_name([ifv, build], cache_symbol_info=True)
        except Exception as exc:
            if not is_symbol_not_found(exc):
                raise
            log.info("nIfVersion not found (%s) - checking legacy interface", exc)
        else:
            version = int(raw.get(ifv) or 0)
            return max(version, pv.IF_V2), str(raw.get(build) or "")
        # raises if the connection itself does not work -> connect fails
        plc.read_list_by_name([self._from_prefix + "nPlcHeartbeat"], cache_symbol_info=True)
        return pv.IF_LEGACY, "legacy PLC (no nIfVersion)"

    def _check_symbols(self, plc: Any, version: int) -> None:
        """One sum-read of all FROM + TO fields of the interface; missing symbols -> InterfaceMismatch.

        Only a 'symbol not found' error is resolved symbol by symbol (to name the missing fields); any other error
        (timeout, connection) is re-raised at once, so a dead connection never causes one ADS call per symbol.
        """
        names = ([self._from_prefix + f for f in pv.from_plc_vars(version)]
                 + [self._to_prefix + f for f in pv.to_plc_vars(version)])
        try:
            plc.read_list_by_name(names, cache_symbol_info=True)
            return
        except Exception as exc:
            if not is_symbol_not_found(exc):
                raise
            first_error = exc
        missing = []
        for name in names:
            try:
                plc.read_list_by_name([name], cache_symbol_info=True)
            except Exception as exc:
                if not is_symbol_not_found(exc):
                    raise
                missing.append(name.split(".", 1)[-1])        # "stFromPlc.eMoveCmdResult"
        if not missing:
            raise first_error
        raise InterfaceMismatch(missing)

    def _set_version(self, version: int, build: str) -> None:
        self._if_version = version
        self._plc_build = build
        self._from_vars = pv.from_plc_vars(version)
        self._to_vars = pv.to_plc_vars(version)
        self._from_symbols = [self._from_prefix + f for f in self._from_vars]

    def _write_raw(self, values: Dict[str, Any], label: str, with_cmd_id: bool = True) -> None:
        """ONE write_list_by_name. Raises on ADS call failure, _WriteRejected on per-symbol errors."""
        if self._plc is None:
            raise RuntimeError("no connection")
        payload = dict(values)
        if with_cmd_id:
            self._cmd_id = (self._cmd_id + 1) & 0xFFFF
            payload["nCommandId"] = self._cmd_id
        symbols = {self._to_prefix + k: v for k, v in payload.items()}
        result = self._plc.write_list_by_name(symbols, cache_symbol_info=True)
        errors = {k: v for k, v in (result or {}).items() if str(v).lower() != "no error"}
        if errors:
            raise _WriteRejected(f"PLC returned errors {errors}")
        log.debug("wrote %s: %s", label, payload)

    def _lost(self, reason: str) -> None:
        """Connection lost: drop queue (epoch), stop heartbeat, close; reconnect timer takes over."""
        log.warning("ADS connection lost: %s", reason)
        self._close()
        self.connection.emit(False, reason)

    def _discard(self, plc: Any) -> None:
        """Failed connect: close the half-open connection too (it is not yet self._plc), then the usual close."""
        if plc is not None and plc is not self._plc:
            try:
                plc.close()
            except Exception:
                pass
        self._close()

    def _close(self) -> None:
        was = self._connected
        self._connected = False
        self._hb_enabled = False
        with self._lock:
            if was:
                self._epoch += 1
        plc, self._plc = self._plc, None
        if plc is not None:
            try:
                plc.close()
            except Exception:
                pass

    @Slot()
    def _shutdown(self) -> None:
        self._stopped = True
        for t in self._timers:
            t.stop()
        if self._connected and self._plc is not None:
            try:   # release motion bits; heartbeat stops -> PLC watchdog does the rest
                vals = {f: False for f in pv.JOG_FIELDS}
                vals["bCmdHorn"] = False
                self._write_raw(vals, "shutdown")
            except Exception as exc:
                log.warning("shutdown write failed: %s", exc)
        self._close()


class _WriteRejected(Exception):
    """write_list_by_name returned per-symbol errors (connection itself is fine)."""
