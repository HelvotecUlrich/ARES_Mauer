"""The HMI's own ADS connection #1 (pattern A, D-H2): the amr AdsWorker in the QThread "ads-worker" - heartbeat,
MANUAL, HALT, jog, GO, odometry reset - created ONLY by hmi/main.py with --ares (D-H5). Without --ares the window
gets a NullAdsWorker that never connects.

probing_factory(): a TCP probe of the AMS router port first (pyads blocks ~20 s per connect to an unreachable
target, mauer/ares/ads.py TCP_PROBE_MAX_S), then pyads.Connection (imported here, lazily).
"""
from __future__ import annotations

from typing import Any, Callable, Mapping

from PySide6.QtCore import QObject, QThread, Signal

from mauer.ares import ads as mads

from ..amr.ads_worker import AdsWorker

ADS_OFF = "ADS off: start the HMI with --ares"


def probing_factory(ads_cfg: Mapping) -> Callable[[], Any]:
    """Connection factory for AdsWorker (amr cfg['ads']): TCP probe of host:48898, then pyads.Connection. Raises
    OSError with the reason -> AdsWorker.connect_now reports 'connect failed: ...' and retries."""
    def make() -> Any:
        host = mads.target_host(str(ads_cfg["ams_net_id"]), str(ads_cfg.get("host_ip") or "") or None)
        reason = mads.tcp_reachable(host, mads.ADS_TCP_PORT, mads.TCP_PROBE_MAX_S)
        if reason is not None:
            raise OSError(f"ARES PLC not reachable: TCP {host}:{mads.ADS_TCP_PORT} ({reason})")
        import pyads                     # lazy: tests and SIM never import it
        if ads_cfg.get("host_ip"):
            return pyads.Connection(ads_cfg["ams_net_id"], int(ads_cfg["ads_port"]), ads_cfg["host_ip"])
        return pyads.Connection(ads_cfg["ams_net_id"], int(ads_cfg["ads_port"]))
    return make


def start_ads_worker(amr_cfg: dict) -> tuple[AdsWorker, QThread]:
    """The amr AdsWorker on the probing factory in its started thread "ads-worker" (it connects at once)."""
    worker = AdsWorker(amr_cfg, connection_factory=probing_factory(amr_cfg["ads"]))
    thread = QThread()
    thread.setObjectName("ads-worker")
    worker.moveToThread(thread)
    thread.started.connect(worker.start)
    thread.start()
    return worker, thread


class NullAdsWorker(QObject):
    """AdsWorker stand-in without --ares: same signals and API, never connects; commands are recorded and refused."""

    status = Signal(dict)
    connection = Signal(bool, str)
    interface = Signal(int, str)
    command_error = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.calls: list[tuple[str, Any]] = []

    @property
    def is_connected(self) -> bool:
        return False

    @property
    def if_version(self) -> int:
        return 0

    def start(self) -> None:
        self.connection.emit(False, ADS_OFF)

    def send(self, values: dict, label: str = "") -> None:
        self.calls.append(("send", dict(values)))
        self.command_error.emit(f"{label or ','.join(values)}: ADS off: not sent")

    def pulse(self, values: dict, pulse_fields, label: str = "", ms: int = 300) -> None:
        self.calls.append(("pulse", (dict(values), tuple(pulse_fields))))
        self.command_error.emit(f"{label or ','.join(pulse_fields)}: ADS off: not sent")

    def halt(self) -> None:
        self.calls.append(("halt", None))
        self.command_error.emit("HALT: ADS off: not sent")

    def request_stop(self) -> None:
        self.calls.append(("stop", None))
