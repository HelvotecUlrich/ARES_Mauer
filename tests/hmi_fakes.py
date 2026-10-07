"""Test doubles of the Mauer HMI tests (no network, no pyads, no hardware).

amr_hmi part: copied 2026-10-07 from MA 10_robot/hmi/amr_hmi/tests/fakes.py (commit 5935c5b) - fake pyads
connection, fake ADS worker, scheduler. AMR_CFG = the values of the amr config.yaml (MA commit 5935c5b) in the amr
config shape; its ADS address is the TEST-NET-1 address 192.0.2.10 (RFC 5737) - the tests never name the lab one.
"""

from __future__ import annotations

import functools
from typing import Any, Callable, Dict, List, Optional, Tuple

from PySide6.QtCore import QObject, Signal

from hmi.amr import plc_vars as pv

# amr_hmi config.yaml (MA commit 5935c5b) as the dict MainWindow / AdsWorker / the panels read
AMR_CFG: Dict[str, Any] = {
    "ads": {"ams_net_id": "192.0.2.10.1.1", "ads_port": 851, "host_ip": "192.0.2.10", "timeout_ms": 1000},
    "plc": {"prefix": "", "to_plc_struct": "GVL_HMI.stToPlc", "from_plc_struct": "GVL_HMI.stFromPlc"},
    "hmi": {"poll_interval_ms": 100, "heartbeat_interval_ms": 100, "reconnect_interval_s": 5, "jog_speed_mms": 200.0,
            "jog_rot_speed_degs": 20.0, "default_speed_limit_mms": 500.0, "jog_accel_mms2": 1000.0,
            "accel_outside_manual_mms2": 200.0, "watchdog_timeout_s": 2.0},
    "frame": {"plus_y_is_left": True, "plus_omega_is_ccw": True, "verified": True},
    "move": {"default_distance_mm": 1000.0, "default_angle_deg": 90.0, "default_speed_mms": 150.0,
             "default_rot_speed_degs": 10.0, "default_accel_mms2": 200.0, "test_distance_mm": 100.0,
             "test_speed_mms": 50.0, "max_distance_mm": 20000.0, "max_angle_deg": 720.0, "max_speed_mms": 500.0,
             "max_rot_speed_degs": 45.0, "max_accel_mms2": 1000.0, "min_distance_mm": 2.0, "min_angle_deg": 0.2},
}

FROM = "GVL_HMI.stFromPlc."
TO = "GVL_HMI.stToPlc."


class FakeADSError(Exception):
    """Stands in for pyads.ADSError (e.g. 1808 symbol not found, 1861 timeout); err_code like pyads."""

    def __init__(self, msg: str, err_code: Optional[int] = None) -> None:
        super().__init__(msg)
        self.err_code = err_code


def plc_symbols(version: int, **overrides: Any) -> Dict[str, Any]:
    """Symbol table of a fake PLC with the given interface version (all values = type defaults)."""
    sym: Dict[str, Any] = {}
    for f, t in pv.from_plc_vars(version).items():
        sym[FROM + f] = pv.TYPE_DEFAULTS[t]
    for f, t in pv.to_plc_vars(version).items():
        sym[TO + f] = pv.TYPE_DEFAULTS[t]
    if version >= pv.IF_V2:
        sym[FROM + "nIfVersion"] = 2
        sym[FROM + "sPlcBuild"] = "ARES CX9240 v2 2026-09-25"
    for k, v in overrides.items():
        sym[k] = v
    return sym


class FakeConnection:
    """Minimal pyads.Connection replacement recording every call in a shared log."""

    def __init__(self, symbols: Dict[str, Any], log: List[Tuple], fail_open: bool = False) -> None:
        self.symbols = symbols
        self.log = log
        self.fail_open = fail_open
        self.fail_reads = False
        self.fail_writes = False
        self.timeout_ms: Optional[int] = None
        self.is_open = False

    def open(self) -> None:
        self.log.append(("open",))
        if self.fail_open:
            raise FakeADSError("target machine not found")
        self.is_open = True

    def close(self) -> None:
        self.log.append(("close",))
        self.is_open = False

    def set_timeout(self, ms: int) -> None:
        self.log.append(("set_timeout", ms))
        self.timeout_ms = ms

    def read_list_by_name(self, names: List[str], cache_symbol_info: bool = True, **_: Any) -> Dict[str, Any]:
        self.log.append(("read", tuple(names)))
        if self.fail_reads:
            raise FakeADSError("timeout elapsed (1861)", 1861)
        missing = [n for n in names if n not in self.symbols]
        if missing:
            # like pyads (adsGetSymbolInfo): the error does not name the symbol
            raise FakeADSError("ADSError: symbol not found (1808). ", 1808)
        return {n: self.symbols[n] for n in names}

    def write_list_by_name(self, values: Dict[str, Any], cache_symbol_info: bool = True,
                           **_: Any) -> Dict[str, str]:
        self.log.append(("write", dict(values)))
        if self.fail_writes:
            raise FakeADSError("timeout elapsed (1861)", 1861)
        missing = [n for n in values if n not in self.symbols]
        if missing:
            raise FakeADSError("ADSError: symbol not found (1808). ", 1808)
        self.symbols.update(values)
        return {k: "no error" for k in values}


class FakeWorker(QObject):
    """Stands in for AdsWorker in GUI tests: same signals and API, records calls."""

    status = Signal(dict)
    connection = Signal(bool, str)
    interface = Signal(int, str)
    command_error = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.calls: List[Tuple[str, Any]] = []
        self.connected = True
        self.version = pv.IF_V2

    @property
    def is_connected(self) -> bool:
        return self.connected

    @property
    def if_version(self) -> int:
        return self.version

    def send(self, values: Dict[str, Any], label: str = "") -> None:
        self.calls.append(("send", dict(values)))

    def pulse(self, values: Dict[str, Any], pulse_fields, label: str = "", ms: int = 300) -> None:
        self.calls.append(("pulse", (dict(values), tuple(pulse_fields))))

    def halt(self) -> None:
        self.calls.append(("halt", None))

    def request_stop(self) -> None:
        self.calls.append(("stop", None))

    def start(self) -> None:
        self.calls.append(("start", None))

    def of(self, kind: str) -> List[Any]:
        return [p for k, p in self.calls if k == kind]


class Scheduler:
    """Captures scheduled callbacks instead of starting QTimers."""

    def __init__(self) -> None:
        self.pending: List[Tuple[int, Callable[[], None]]] = []

    def __call__(self, ms: int, fn: Callable[[], None]) -> None:
        self.pending.append((ms, fn))

    def run_all(self) -> None:
        todo, self.pending = self.pending, []
        for _ms, fn in todo:
            fn()


# ── Mauer test jobs ───────────────────────────────────────────────────────────
def make_job_tool():
    """tools/make_job.py as a module (as tests/test_sequencer.py _tool)."""
    import importlib.util

    from mauer import REPO
    spec = importlib.util.spec_from_file_location("tool_make_job", REPO / "tools" / "make_job.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@functools.lru_cache(maxsize=None)
def straight_job10():
    """(cfg, job): the straight-wall test config (conftest.straight_config) and its nominal job of 10 stones in
    course 0 (2 stops), built once per session. Shared - never mutate them."""
    from conftest import straight_config
    cfg = straight_config()
    return cfg, make_job_tool().build_nominal(cfg, length=10)


def short_job(job, n0=6, n1=3, fill=None):
    """First n0 stones of stop 0 and n1 of stop 1; fill = stones in the magazine at the start (the first `fill`
    stones, each in a slot of its type) - as tests/test_sequencer.py short_job."""
    import copy

    from mauer import job as mjob
    j = copy.deepcopy(job)
    j.stops = j.stops[:2]
    j.stops[0].stones = j.stops[0].stones[:n0]
    j.stops[1].stones = j.stops[1].stones[:n1]
    if fill is not None:
        kinds = [t.kind for st in j.stops for t in st.stones][:fill]
        plan = dict(mjob.fill_plan(mjob.SlotState.magazine(j.magazine, filled=[], kinds={}), kinds))
        j.magazine.initial_fill = [sid for sid in j.magazine.take_order if sid in plan]
        j.magazine.initial_kinds = plan
    return j
