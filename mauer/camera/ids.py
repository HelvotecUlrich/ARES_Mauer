r"""IDS GV-51F0CP-M-GL Rev.2.2 (uEye+ GigE Vision) through IDS peak: software-triggered single frames as numpy.

Free stack, no HALCON: IDS peak runtime (GenTL producer for GigE Vision, free download from IDS) + the PyPI wheels
ids_peak / ids_peak_ipl (requirements.txt). Research: scratchpad research_out/ids-camera.json (2026-10-05).

Why the runtime is needed: the ids_peak wheel bundles the genericAPI but no GenTL producer (.cti), and it refuses
third-party producers – ProducerLibrary.Open(Hikrobot MvProducerGEV.cti) -> "PEAK_RETURN_CODE_NOT_AVAILABLE |
Provided producerLibrary is not supported" (tested 2026-10-05). Without IDS peak, DeviceManager.Update() succeeds with
zero systems (tested 2026-10-05, ids_peak 1.17.0, only the Hikrobot MVS path on GENICAM_GENTL64_PATH); open() turns
that into a CameraError that says what to install. With IDS peak installed but its kernel GigE driver not running,
the GEVK producer loads but TLOpen fails ("Opening driver file '\\.\ids_gevcore' failed", seen 2026-10-05 while the
installer ran); open() explains that too (gev_diagnosis).

Acquisition (IDS manual "Using the software trigger", IDS peak 26.06 / uEye+ firmware 3.94,
https://www.1stvision.com/cameras/IDS/IDS-manuals/en/operate-software-trigger.html): UserSetSelector Default +
UserSetLoad, PixelFormat Mono8, ExposureAuto/GainAuto Off, ExposureTime [µs], GainSelector AnalogAll + Gain,
AcquisitionMode Continuous, TriggerSelector ExposureStart, TriggerMode On, TriggerSource Software; buffers of
PayloadSize, TLParamsLocked = 1, DataStream.StartAcquisition, AcquisitionStart. grab() = TriggerSoftware +
WaitForFinishedBuffer(timeout) + copy to numpy (H, W) + QueueBuffer. One acquisition runs from open() to close().
Node names: IDS camera feature reference 3.94 (1stVision mirror, pages named after the node, read 2026-10-05);
ids_peak API names: ids_peak 1.17.0 wheel (ids_peak/ids_peak.py). Nodes that may be missing are checked first.

Verification status (2026-10-05): every ids_peak call used here exists in the 1.17.0 API. Run for real on this laptop:
the no-runtime path (the GEVK-driver error was seen with a manual probe; its message path runs on the fake), and –
after IDS peak 26.06.2 was installed – producer discovery
and device enumeration with the real camera attached (descriptor, DeviceSelector/DeviceID/GevDeviceIPAddress lookup;
"GV-51FxCP-M", not openable: its IP 192.168.0.1 is outside the NIC subnets). configure/grab/close run only against the
fake ids_peak in tests/test_camera.py; everything that opens a real device is UNTESTED – marked "UNTESTED on
hardware" below.
"""
from __future__ import annotations

import glob
import os
import re
import sys
import threading
import time
from contextlib import contextmanager
from importlib import metadata
from pathlib import Path
from typing import Any, Callable, Iterator, Sequence

from .base import Camera, CameraError, Frame

SETUP_DOC = "docs/CAMERA_SETUP.md"
# GV-51F0CP-M-GL is supported from IDS Vision firmware 3.31 (https://www.machine-vision-shop.com/all-products/cameras/
# ids-gv-51f0cp-m-gl, research_out/ids-camera.json); current manual: uEye+ firmware 3.94.
MIN_FIRMWARE = (3, 31)
# GenTL producers are found through this variable (ids_peak.py DeviceManager.Update / EnvironmentInspector docs).
GENTL_ENV = "GENICAM_GENTL64_PATH" if sys.maxsize > 2**32 else "GENICAM_GENTL32_PATH"
# IDS peak install root. CONFIRMED on this laptop (IDS peak 26.06.2-1190 installed 2026-10-05): producers at
# ids_gevgentl\{32,64}\ids_gevgentlk.cti (GigE Vision, kernel "GEVK", TLType "GEV") and ids_u3vgentl\{32,64}\
# ids_u3vgentlk.cti. The installer adds the ...\64 folders to the machine-wide GENICAM_GENTL64_PATH, but processes
# whose environment predates the install (e.g. py.exe started from an older WSL session) do not see them – so the
# install folder is searched when no GigE producer is found on GENTL_ENV.
IDS_INSTALL_ROOTS = (r"C:\Program Files\IDS\ids_peak",)
GEV_TL = "GEV"                  # System.TLType() of the IDS GigE Vision producer (seen 2026-10-05)
STANDARD_PACKET_B = 1500        # Ethernet without jumbo frames; IDS peak readme recommends ~9000 B packets
UNPACKED_MONO = ("Mono8", "Mono10", "Mono12", "Mono16")   # to_numpy_array gives (H, W); packed formats give 1-D

MSG_NO_PACKAGE = ("Python package ids_peak not importable ({err}) - install it with "
                  "'py.exe -m pip install ids_peak ids_peak_ipl' and the IDS peak runtime, see " + SETUP_DOC)
MSG_NO_CAMERA = ("no camera found - check power (UR tool output 24 V), cable, NIC IP subnet (laptop NIC and camera in "
                 "the same subnet, [camera] ip in config/station.toml; 'ids_ipconfig -L' lists cameras), see "
                 + SETUP_DOC)


def no_runtime_message(foreign_ctis: Sequence[str] = ()) -> str:
    """Actionable message for 'no IDS GenTL producer'."""
    s = ("IDS peak runtime not installed - no IDS GenTL producer found. Install IDS peak with the GigE Vision "
         "transport layer, see " + SETUP_DOC + ".")
    if foreign_ctis:
        names = ", ".join(sorted({Path(c).name for c in foreign_ctis}))
        s += (f" (Only third-party producers on {GENTL_ENV}: {names} - ids_peak does not accept them.)")
    return s


def producer_error_message(errors: Sequence[tuple[str, str]]) -> str:
    """Actionable message for 'IDS GigE producer installed but it does not start' (errors: (cti, error text))."""
    detail = "; ".join(f"{Path(c).name}: {err}" for c, err in errors)
    s = f"IDS peak is installed but its GigE Vision producer does not start ({detail})."
    if "ids_gevcore" in detail:
        # seen 2026-10-05 while the IDS peak installer was still running: TLOpen -> "Opening driver file
        # '\\.\ids_gevcore' failed" (kernel transport layer GEVK without its driver)
        s += (" The kernel GigE transport layer (GEVK) needs its driver ids_gevcore: let the IDS peak installation "
              "finish (here the producer worked right after it, no reboot); if it persists, reboot or reinstall "
              "IDS peak with the GigE Vision transport layer 'Socket (GEV)'.")
    return s + " See " + SETUP_DOC + "."


# ── library lifetime (Library.Initialize/Close once per process, reference counted) ──
_LIB_LOCK = threading.RLock()
_LIB_REFS: dict[int, list] = {}            # id(ids_peak module) -> [module, refs, set of added .cti paths]


def _import_peak() -> Any:
    """Lazy import of ids_peak.ids_peak (so the rest of mauer works without it); errors -> CameraError."""
    try:
        from ids_peak import ids_peak as p
    except ImportError as e:
        raise CameraError(MSG_NO_PACKAGE.format(err=e)) from e
    except OSError as e:                    # DLL load failure on Windows
        raise CameraError(MSG_NO_PACKAGE.format(err=f"DLL load failed: {e}")) from e
    return p


def _lib_acquire(p: Any) -> None:
    with _LIB_LOCK:
        entry = _LIB_REFS.get(id(p))
        if entry is None:
            try:
                p.Library.Initialize()
            except Exception as e:
                raise CameraError(f"IDS peak library initialisation failed: {type(e).__name__}: {e}") from e
            entry = _LIB_REFS[id(p)] = [p, 0, set()]
        entry[1] += 1


def _lib_release(p: Any) -> None:
    with _LIB_LOCK:
        entry = _LIB_REFS.get(id(p))
        if entry is None:
            return
        entry[1] -= 1
        if entry[1] <= 0:
            del _LIB_REFS[id(p)]
            try:
                p.Library.Close()
            except Exception as e:      # nothing a caller can do about it
                print(f"[ids] warning: Library.Close failed: {type(e).__name__}: {e}", flush=True)


def library_refs() -> int:
    """Number of open references to the IDS peak library (all modules) – for tests and diagnostics."""
    with _LIB_LOCK:
        return sum(e[1] for e in _LIB_REFS.values())


@contextmanager
def ids_library() -> Iterator[Any]:
    """`with ids_library() as p:` – ids_peak.ids_peak module with the library initialised for the block."""
    p = _import_peak()
    _lib_acquire(p)
    try:
        yield p
    finally:
        _lib_release(p)


# ── small helpers ─────────────────────────────────────────────────────────────
def _is_exc(e: BaseException, *names: str) -> bool:
    """True if e is an instance of an exception class with one of these names (ids_peak.exceptions.*)."""
    return any(c.__name__ in names for c in type(e).__mro__)


def _call(fn: Callable, *args: Any, default: Any = None) -> Any:
    """fn(*args), or default on any exception – only for optional, informative reads."""
    try:
        return fn(*args)
    except Exception:
        return default


def ip_str(v: int | None) -> str:
    """GigE Vision IPv4 integer (first octet in the high byte) -> dotted string."""
    return "" if v is None else ".".join(str((int(v) >> s) & 0xFF) for s in (24, 16, 8, 0))


def mac_str(v: int | None) -> str:
    return "" if v is None else ":".join(f"{(int(v) >> s) & 0xFF:02X}" for s in range(40, -8, -8))


def parse_version(s: str | None) -> tuple[int, int] | None:
    """First 'major.minor' in a version string ('3.94.0.0' -> (3, 94)); None if there is none.
    The exact DeviceFirmwareVersion format of this camera is UNVERIFIED."""
    m = re.search(r"(\d+)\.(\d+)", s or "")
    return (int(m.group(1)), int(m.group(2))) if m else None


def firmware_ok(s: str | None) -> bool | None:
    """firmware >= MIN_FIRMWARE (3.31)? None if the string cannot be parsed (check in IDS peak Cockpit)."""
    v = parse_version(s)
    return None if v is None else v >= MIN_FIRMWARE


def package_versions() -> dict[str, str | None]:
    out = {}
    for name in ("ids_peak", "ids_peak_ipl", "ids_peak_common"):
        try:
            out[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            out[name] = None
    return out


def gentl_dirs() -> list[str]:
    return [d for d in os.environ.get(GENTL_ENV, "").split(os.pathsep) if d.strip()]


def ctis_in(dirs: Sequence[str]) -> list[str]:
    return sorted({c for d in dirs for c in glob.glob(os.path.join(d, "*.cti"))})


def ids_install_ctis(roots: Sequence[str] | None = None, this_arch: bool = True) -> list[str]:
    """IDS producers (*.cti) below the IDS peak install root(s) (default IDS_INSTALL_ROOTS), GigE ('gev') first.
    Empty if not installed. this_arch: only those in a '64' (64-bit Python) or '32' folder – the other cannot load."""
    roots = IDS_INSTALL_ROOTS if roots is None else roots
    found = {c for r in roots if os.path.isdir(r) for c in glob.glob(os.path.join(r, "**", "*.cti"), recursive=True)}
    if this_arch:
        arch = "64" if sys.maxsize > 2**32 else "32"
        found = {c for c in found if arch in Path(c).parent.parts[-1:]}
    return sorted(found, key=lambda c: (0 if "gev" in Path(c).name.lower() else 1, c))


def _is_gev_cti(c: str) -> bool:
    return "gev" in Path(c).name.lower()


def probe_producer(p: Any, cti: str) -> str | None:
    """Open a producer and its system outside the DeviceManager; None if that works, else the error text.
    Used only to explain why the DeviceManager found no GigE system (it skips failing producers silently)."""
    try:
        lib = p.ProducerLibrary.Open(cti)
        system = lib.System().OpenSystem()
        del system, lib
        return None
    except Exception as e:
        return f"{type(e).__name__}: {e}"


# ── node access (GenICam node map of the remote device / interface) ──────────
def _node(nm: Any, name: str) -> Any:
    """The node if the map has it and it is available (readable or writeable), else None."""
    try:
        if not nm.HasNode(name):
            return None
        n = nm.FindNode(name)
        return n if n is not None and n.IsAvailable() else None
    except Exception:
        return None


def _read(nm: Any, name: str, default: Any = None) -> Any:
    """Value of a node (enum -> symbolic string), or default if missing/unreadable."""
    n = _node(nm, name)
    if n is None or not _call(n.IsReadable, default=False):
        return default
    try:
        if hasattr(n, "CurrentEntry"):
            return n.CurrentEntry().SymbolicValue()
        return n.Value()
    except Exception:
        return default


def _writeable(nm: Any, name: str) -> Any:
    n = _node(nm, name)
    return n if n is not None and _call(n.IsWriteable, default=False) else None


def _set_enum(nm: Any, name: str, value: str, required: bool = True) -> bool:
    """Set an enumeration node; True if it holds `value` afterwards. Missing node/entry: CameraError or False."""
    if _read(nm, name) == value:
        return True
    n = _writeable(nm, name)
    if n is None:
        if required:
            raise CameraError(f"camera node {name} is not available/writeable - cannot set it to {value!r}")
        return False
    if not n.HasEntry(value):
        if required:
            entries = _call(lambda: [e.SymbolicValue() for e in n.Entries()], default=[])
            raise CameraError(f"camera node {name} has no entry {value!r} (entries: {entries})")
        return False
    n.SetCurrentEntry(value)
    return True


def _set_number(nm: Any, name: str, value: float, integer: bool = False,
                log: Callable[[str], None] | None = None) -> float | int:
    """Set a float/integer node clamped to [Minimum, Maximum] (integers aligned to Increment); returns the read-back."""
    n = _writeable(nm, name)
    if n is None:
        raise CameraError(f"camera node {name} is not available/writeable")
    lo, hi = n.Minimum(), n.Maximum()
    v = min(max(value, lo), hi)
    if integer:
        inc = int(_call(n.Increment, default=1) or 1)
        v = int(lo + ((int(v) - int(lo)) // inc) * inc)
    if log is not None and v != value:
        log(f"{name} {value} outside/not on the device grid [{lo}, {hi}] -> {v}")
    n.SetValue(int(v) if integer else float(v))
    return n.Value()


def _execute(nm: Any, name: str, wait_ms: int | None = None) -> None:
    n = _node(nm, name)
    if n is None:
        raise CameraError(f"camera command {name} is not available")
    n.Execute()
    if wait_ms is not None:
        n.WaitUntilDone(wait_ms)


def _iface_lookup(desc: Any, names: Sequence[str]) -> dict[str, Any] | None:
    """Values of interface-module nodes (GenTL SFNC 'DeviceEnumeration') for this descriptor, or None if unknown.

    IDS transport layer feature reference: DeviceSelector (0-based), DeviceID[DeviceSelector],
    GevDeviceIPAddress[DeviceSelector], GevDeviceMACAddress[DeviceSelector] in the interface node map.
    Matching DeviceID to DeviceDescriptor.ID() (both the GenTL device ID) worked on the real camera 2026-10-05
    (IP and MAC read before opening, IDS peak 26.06.2 GEVK producer).
    """
    try:
        nm = desc.ParentInterface().NodeMaps()[0]
        sel = nm.FindNode("DeviceSelector")
        want = desc.ID()
        lo, hi = int(sel.Minimum()), int(sel.Maximum())
    except Exception:
        return None
    for i in range(lo, hi + 1):
        try:
            sel.SetValue(i)
            if nm.FindNode("DeviceID").Value() == want:
                return {k: _read(nm, k) for k in names}
        except Exception:
            continue
    return None


def descriptor_ip(desc: Any) -> str | None:
    """Camera IP from the interface node map before opening ('' never; None = unknown)."""
    v = _iface_lookup(desc, ["GevDeviceIPAddress"])
    return ip_str(v["GevDeviceIPAddress"]) if v and v["GevDeviceIPAddress"] is not None else None


def describe(desc: Any) -> str:
    ip = descriptor_ip(desc)
    return (f"{_call(desc.ModelName, default='?')} serial {_call(desc.SerialNumber, default='?')}"
            f" ip {ip or '?'}")


def _has_gev(dm: Any) -> bool:
    return any(_call(s.TLType) == GEV_TL for s in dm.Systems())


def _device_manager(p: Any, cti_paths: Sequence[str] = (), search_install: bool = True,
                    log: Callable[[str], None] = print) -> Any:
    """DeviceManager after Update(); adds explicit and (if no GigE system was found) installed IDS producers."""
    dm = p.DeviceManager.Instance()
    added: set = _LIB_REFS[id(p)][2]

    def add(ctis: Sequence[str]) -> bool:
        new = [c for c in ctis if c and c not in added]
        for c in new:
            try:
                dm.AddProducerLibrary(c)
                added.add(c)
            except Exception as e:
                log(f"cannot add GenTL producer {c}: {type(e).__name__}: {e}")
        return bool(new)

    def update() -> None:
        try:
            dm.Update()
        except Exception as e:
            # NotFoundException: GENTL_ENV missing/empty; CTILoadingException: a producer on it failed to load.
            # Either way: scan only the producers added by hand (if any). Otherwise Systems() stays empty.
            if not _is_exc(e, "NotFoundException", "CTILoadingException"):
                raise CameraError(f"IDS peak DeviceManager.Update failed: {type(e).__name__}: {e}") from e
            if _is_exc(e, "CTILoadingException"):
                log(f"a GenTL producer on {GENTL_ENV} failed to load: {e}")
            policy = getattr(p.DeviceManager, "UpdatePolicy_DontScanEnvironmentForProducerLibraries", None)
            if added and policy is not None:
                dm.Update(policy)

    add(cti_paths)
    update()
    if not _has_gev(dm) and search_install:
        new = [c for c in ids_install_ctis() if c not in added]
        if add(new):
            log(f"no IDS GigE producer on {GENTL_ENV}; adding the installed ones: {new}")
            update()
    return dm


def gev_diagnosis(p: Any, dm: Any, cti_paths: Sequence[str] = (), search_install: bool = True) -> tuple[str, list]:
    """Why there is no GigE Vision system: (actionable message, [(cti, error)]); ('', []) if there is one."""
    if _has_gev(dm):
        return "", []
    ids_ctis = list(_call(p.EnvironmentInspector.CollectCTIPaths, default=[]) or [])
    cands = [c for c in dict.fromkeys([*cti_paths, *ids_ctis, *(ids_install_ctis() if search_install else [])])
             if c and _is_gev_cti(c)]
    if not cands:
        return no_runtime_message(ctis_in(gentl_dirs())), []
    errors = [(c, probe_producer(p, c) or "opens here, but the DeviceManager found no GigE system") for c in cands]
    return producer_error_message(errors), errors


class IdsCamera(Camera):
    """IDS uEye+ GigE Vision camera via IDS peak, software trigger, one acquisition from open() to close().

    serial / ip select the camera ('' = do not filter; both empty = first camera found). exposure_us / gain None =
    keep the value of the Default user set. packet_size: "auto" keeps the GigE packet size the GenTL producer
    negotiated on open (IDS peak readme: it uses the largest size the network supports), "max" forces the camera
    maximum (only if the NIC jumbo frame size is at least that), or a size in bytes. cti_path: IDS GenTL producer(s)
    to add if they are not on GENICAM_GENTL64_PATH.
    """

    def __init__(self, serial: str = "", ip: str = "", pixel_format: str = "Mono8",
                 exposure_us: float | None = None, gain: float | None = None, timeout_ms: int = 3000,
                 n_buffers: int = 3, packet_size: int | str = "auto", load_default_userset: bool = True,
                 cti_path: str | Sequence[str] = "", search_install: bool = True, cmd_timeout_ms: int = 2000,
                 verbose: bool = True) -> None:
        if pixel_format not in UNPACKED_MONO:
            raise ValueError(f"pixel_format {pixel_format!r} not supported (unpacked mono only: {UNPACKED_MONO}; "
                             f"the Frame contract is Mono8 -> uint8)")
        if not (packet_size in ("auto", "max") or (isinstance(packet_size, int) and packet_size > 0)):
            raise ValueError(f"packet_size must be 'auto', 'max' or bytes > 0, got {packet_size!r}")
        self.serial = str(serial or "")
        self.ip = str(ip or "")
        self.pixel_format = pixel_format
        self.exposure_us = exposure_us
        self.gain = gain
        self.timeout_ms = int(timeout_ms)
        self.n_buffers = int(n_buffers)
        self.packet_size = packet_size
        self.load_default_userset = load_default_userset
        self.cti_paths = [cti_path] if isinstance(cti_path, str) else list(cti_path)
        self.search_install = search_install
        self.cmd_timeout_ms = int(cmd_timeout_ms)
        self.verbose = verbose
        self._reset_handles()

    @classmethod
    def from_config(cls, cfg: dict | None = None, **overrides: Any) -> "IdsCamera":
        """From [camera] of config/station.toml; keyword overrides that are None are ignored.
        Keys not in the config yet (timeout_ms, packet_size, n_buffers, cti_path) fall back to the defaults above."""
        if cfg is None:
            from .. import config
            cfg = config.load()
        c = cfg.get("camera", {})
        kw: dict[str, Any] = {k: c[k] for k in ("serial", "ip", "pixel_format", "exposure_us", "gain", "timeout_ms",
                                                 "n_buffers", "packet_size", "load_default_userset", "cti_path")
                              if k in c}
        kw.update({k: v for k, v in overrides.items() if v is not None})
        return cls(**kw)

    def _reset_handles(self) -> None:
        self._p: Any = None
        self._lib_held = False
        self._desc: Any = None
        self._dev: Any = None
        self._nm: Any = None
        self._ds: Any = None
        self._trigger: Any = None
        self._tl_locked = False
        self._ds_running = False
        self._dev_running = False
        self._open = False
        self._exposure_applied: float | None = None
        self._gain_applied: float | None = None
        self._packet_size_b: int | None = None
        self._last_frame_id: int | None = None
        self._step = ""

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[IdsCamera] {msg}", flush=True)

    # ── open / configure ─────────────────────────────────────────────────────
    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        if self._open:
            return
        p = _import_peak()
        _lib_acquire(p)
        self._p, self._lib_held = p, True
        try:
            self._step = "find device"
            dm = _device_manager(p, self.cti_paths, self.search_install, self._log)
            self._desc, self._dev = self._select(p, dm)
            if self._dev is None:
                self._step = "open device"
                self._dev = self._open_device(p, self._desc)
            self._step = "remote node map"
            self._nm = self._dev.RemoteDevice().NodeMaps()[0]
            self._configure()
            self._start()
            self._open = True
            self._log(f"open: {describe(self._desc)}, {self.pixel_format}, exposure {self._exposure_applied} us, "
                      f"gain {self._gain_applied}, packet size {self._packet_size_b} B")
        except CameraError:
            self.close()
            raise
        except Exception as e:
            step = self._step
            self.close()
            raise CameraError(f"IDS camera: '{step}' failed: {type(e).__name__}: {e}") from e

    def _select(self, p: Any, dm: Any) -> tuple[Any, Any]:
        """Device descriptor matching serial/ip (and the device if it had to be opened to read its IP)."""
        devs = list(dm.Devices())
        if not devs:
            diag, _ = gev_diagnosis(p, dm, self.cti_paths, self.search_install)
            raise CameraError(diag or MSG_NO_CAMERA)
        cands = devs
        if self.serial:
            cands = [d for d in cands if str(_call(d.SerialNumber, default="")) == self.serial]
        if self.ip and cands:
            ips = [(d, descriptor_ip(d)) for d in cands]
            matched = [d for d, ip in ips if ip == self.ip]
            if not matched:
                # interface node map did not tell: open the unknown ones and read GevCurrentIPAddress
                for d, ip in ips:
                    if ip is None and _call(d.IsOpenable, p.DeviceAccessType_Control, default=False):
                        dev = self._open_device(p, d)       # UNTESTED on hardware
                        if ip_str(_read(dev.RemoteDevice().NodeMaps()[0], "GevCurrentIPAddress")) == self.ip:
                            return d, dev
                        del dev
            cands = matched
        if not cands:
            crit = " and ".join(s for s in (self.serial and f"serial {self.serial}", self.ip and f"IP {self.ip}") if s)
            found = "; ".join(describe(d) for d in devs)
            raise CameraError(f"no IDS camera with {crit} ([camera] serial/ip in config/station.toml); found: {found}."
                              f" Set the persistent IP with ids_ipconfig or pass serial/ip='' to take the first camera,"
                              f" see {SETUP_DOC}")
        if len(cands) > 1:
            self._log(f"{len(cands)} cameras match, using the first: {describe(cands[0])}")
        return cands[0], None

    def _open_device(self, p: Any, desc: Any) -> Any:
        if not _call(desc.IsOpenable, p.DeviceAccessType_Control, default=True):
            raise CameraError(f"camera {describe(desc)} found but cannot be opened for control - it is in use (IDS peak"
                              f" Cockpit, another script; after a crash wait for the GigE Vision heartbeat timeout) or"
                              f" its IP is not in the subnet of the laptop NIC (ids_ipconfig), see {SETUP_DOC}")
        try:
            return desc.OpenDevice(p.DeviceAccessType_Control)
        except Exception as e:
            if _is_exc(e, "BadAccessException"):
                raise CameraError(f"camera {describe(desc)}: access denied ({e}) - close IDS peak Cockpit or other "
                                  f"programs using it") from e
            raise

    def _configure(self) -> None:
        """Default user set, Mono8, manual exposure/gain, software trigger, packet size. UNTESTED on hardware."""
        nm = self._nm
        self._step = "configure"
        packet_before = _read(nm, "GevSCPSPacketSize")
        if self.load_default_userset and _node(nm, "UserSetSelector") is not None \
                and _node(nm, "UserSetLoad") is not None:
            self._step = "load user set Default"
            _set_enum(nm, "UserSetSelector", "Default")
            _execute(nm, "UserSetLoad", wait_ms=self.cmd_timeout_ms)
        self._step = "PixelFormat"
        _set_enum(nm, "PixelFormat", self.pixel_format)
        self._step = "exposure"
        _set_enum(nm, "ExposureMode", "Timed", required=False)     # ExposureTime applies in Timed mode
        _set_enum(nm, "ExposureAuto", "Off", required=False)
        if self.exposure_us is not None:
            self._exposure_applied = _set_number(nm, "ExposureTime", float(self.exposure_us), log=self._log)
        else:
            self._exposure_applied = _read(nm, "ExposureTime")
        self._step = "gain"
        if not _set_enum(nm, "GainSelector", "AnalogAll", required=False):
            self._log(f"GainSelector AnalogAll not available (current: {_read(nm, 'GainSelector')})")
        _set_enum(nm, "GainAuto", "Off", required=False)
        if self.gain is not None:
            self._gain_applied = _set_number(nm, "Gain", float(self.gain), log=self._log)
        else:
            self._gain_applied = _read(nm, "Gain")
        self._step = "software trigger"
        _set_enum(nm, "AcquisitionMode", "Continuous", required=False)
        _set_enum(nm, "TriggerSelector", "ExposureStart")
        _set_enum(nm, "TriggerMode", "On")
        _set_enum(nm, "TriggerSource", "Software")
        self._step = "packet size"
        self._packet_size_b = self._configure_packet_size(packet_before)

    def _configure_packet_size(self, before: int | None) -> int | None:
        """GevSCPSPacketSize (IDS feature reference: GigE stream packet size [B], read-only during acquisition)."""
        nm = self._nm
        n = _node(nm, "GevSCPSPacketSize")
        if n is None:
            return None                                         # not a GigE Vision device
        cur = _read(nm, "GevSCPSPacketSize")
        writeable = _writeable(nm, "GevSCPSPacketSize") is not None
        if self.packet_size == "auto":
            # keep what the GenTL negotiated at open; restore it if the Default user set lowered it
            if before is not None and cur is not None and cur < before and writeable:
                _set_number(nm, "GevSCPSPacketSize", before, integer=True, log=self._log)
        elif writeable:
            target = n.Maximum() if self.packet_size == "max" else int(self.packet_size)
            _set_number(nm, "GevSCPSPacketSize", target, integer=True, log=self._log)
        else:
            self._log(f"GevSCPSPacketSize not writeable, keeping {cur} B")
        size = _read(nm, "GevSCPSPacketSize")
        if size is not None and size <= STANDARD_PACKET_B:
            self._log(f"WARNING: GigE packet size {size} B (no jumbo frames) - a 5.1 MB Mono8 frame needs ~3500 "
                      f"packets; enable jumbo frames (~9000 B, IDS peak readme) on the camera NIC, see {SETUP_DOC}")
        return None if size is None else int(size)

    def _start(self) -> None:
        """Buffers, TLParamsLocked, DataStream + device acquisition start. UNTESTED on hardware."""
        nm = self._nm
        self._step = "open data stream"
        streams = self._dev.DataStreams()
        if not streams:
            raise CameraError("camera has no data stream")
        self._ds = streams[0].OpenDataStream()
        self._step = "allocate buffers"
        payload = _read(nm, "PayloadSize")
        if not payload:
            raise CameraError("camera node PayloadSize not readable - cannot size the buffers")
        n = max(self.n_buffers, int(self._ds.NumBuffersAnnouncedMinRequired()))
        self._ds.AddAcquisitionBuffers(int(payload), n)       # ids_peak >= 1.17: allocate + announce + queue
        self._step = "start acquisition"
        tl = _writeable(nm, "TLParamsLocked")
        if tl is not None:
            tl.SetValue(1)              # payload must not change while buffers are announced (ids_peak.py docs)
            self._tl_locked = True
        self._ds.StartAcquisition()
        self._ds_running = True
        _execute(nm, "AcquisitionStart", wait_ms=self.cmd_timeout_ms)
        self._dev_running = True
        self._trigger = _node(nm, "TriggerSoftware")
        if self._trigger is None:
            raise CameraError("camera command TriggerSoftware not available")

    # ── grab ─────────────────────────────────────────────────────────────────
    def _drain(self) -> int:
        """Requeue frames that are already waiting (late frame after a timeout) so grab() returns a fresh one."""
        n = 0
        while n < 64:
            pending = _call(self._ds.NumBuffersAwaitDelivery, default=0)
            if not pending:
                break
            try:
                buf = self._ds.WaitForFinishedBuffer(0)
            except Exception:
                break
            self._note_frame_id(_call(buf.FrameID))
            self._ds.QueueBuffer(buf)
            n += 1
        if n:
            self._log(f"dropped {n} stale frame(s) before the trigger")
        return n

    def _note_frame_id(self, frame_id: int | None) -> int:
        """Remember the id of every received buffer; returns how many ids were skipped since the previous one
        (frames lost in transport or triggers the camera did not answer with a frame)."""
        gap = 0
        if frame_id is not None:
            if self._last_frame_id is not None and frame_id > self._last_frame_id + 1:
                gap = int(frame_id) - int(self._last_frame_id) - 1
            self._last_frame_id = int(frame_id)
        return gap

    def grab(self) -> Frame:
        """TriggerSoftware -> WaitForFinishedBuffer -> numpy copy -> QueueBuffer. UNTESTED on hardware."""
        if not self._open:
            raise CameraError("IdsCamera: grab() before open()")
        stale = self._drain()
        t_start = time.time()
        try:
            self._trigger.Execute()
            buf = self._ds.WaitForFinishedBuffer(self.timeout_ms)
        except Exception as e:
            if _is_exc(e, "TimeoutException"):
                raise CameraError(
                    f"no frame within {self.timeout_ms} ms after the software trigger (exposure "
                    f"{self._exposure_applied} us) - packet loss? check jumbo frames / NIC receive buffers / "
                    f"firewall for py.exe / cable, see {SETUP_DOC}") from e
            raise CameraError(f"grab failed: {type(e).__name__}: {e}") from e
        t_end = time.time()
        try:
            frame_id = _call(buf.FrameID)
            gap = self._note_frame_id(frame_id)
            if buf.IsIncomplete():
                raise CameraError("incomplete frame (packet loss: jumbo frames, NIC receive buffers, cable, "
                                  f"bandwidth), see {SETUP_DOC}")
            img = buf.ToImageView().to_numpy_array(copy=True)
            if img.ndim != 2:
                raise CameraError(f"unexpected image shape {img.shape} for {self.pixel_format}")
            meta: dict[str, Any] = {
                "camera": f"ids:{_call(self._desc.SerialNumber, default='?')}",
                "exposure_us": self._exposure_applied, "gain": self._gain_applied,
                "pixel_format": self.pixel_format, "frame_id": frame_id,
                "timestamp_ns": _call(buf.Timestamp_ns),                 # device clock, unknown epoch
                "system_timestamp_ns": _call(buf.SystemTimestamp_ns),    # IDS host-correlated Unix ns (typ. 1-2 ms)
                "stale_dropped": stale,
            }
            if gap:
                meta["frame_id_gap"] = gap
        finally:
            try:
                self._ds.QueueBuffer(buf)
            except Exception as e:
                self._log(f"warning: QueueBuffer failed: {type(e).__name__}: {e}")
        return Frame(img, t_start, t_end, meta)

    # ── settings ─────────────────────────────────────────────────────────────
    def set_exposure_us(self, us: float) -> float:
        """Exposure time [µs] (IDS: ExposureTime, Float, µs). Before open(): stored for open()."""
        if not self._open:
            self.exposure_us = float(us)
            return self.exposure_us
        try:
            self._exposure_applied = float(_set_number(self._nm, "ExposureTime", float(us), log=self._log))
        except CameraError:
            raise
        except Exception as e:
            raise CameraError(f"setting ExposureTime {us} us failed: {type(e).__name__}: {e}") from e
        return self._exposure_applied

    def set_gain(self, gain: float) -> float:
        """Gain factor (IDS: Gain[GainSelector=AnalogAll], 1.0 = no gain). Before open(): stored for open()."""
        if not self._open:
            self.gain = float(gain)
            return self.gain
        try:
            _set_enum(self._nm, "GainSelector", "AnalogAll", required=False)
            self._gain_applied = float(_set_number(self._nm, "Gain", float(gain), log=self._log))
        except CameraError:
            raise
        except Exception as e:
            raise CameraError(f"setting Gain {gain} failed: {type(e).__name__}: {e}") from e
        return self._gain_applied

    def ranges(self) -> dict[str, tuple[float, float] | None]:
        """Device limits of exposure [µs] and gain (None if unknown / not open)."""
        out: dict[str, tuple[float, float] | None] = {"exposure_us": None, "gain": None}
        if self._open:
            for key, name in (("exposure_us", "ExposureTime"), ("gain", "Gain")):
                n = _node(self._nm, name)
                if n is not None:
                    out[key] = (_call(n.Minimum), _call(n.Maximum))
        return out

    def info(self) -> dict[str, Any]:
        d: dict[str, Any] = {"kind": "ids", "open": self._open, "serial_requested": self.serial,
                             "ip_requested": self.ip, "pixel_format": self.pixel_format,
                             "exposure_us": self._exposure_applied if self._open else self.exposure_us,
                             "gain": self._gain_applied if self._open else self.gain,
                             "packet_size_mode": self.packet_size, "timeout_ms": self.timeout_ms,
                             "packages": package_versions()}
        p = self._p
        if p is not None:
            d["ids_peak_api"] = _call(lambda: p.Library.Version().ToString())
        if not self._open:
            return d
        nm, desc = self._nm, self._desc
        fw = _read(nm, "DeviceFirmwareVersion")
        pkt = _node(nm, "GevSCPSPacketSize")
        d.update({
            "model": _read(nm, "DeviceModelName") or _call(desc.ModelName),
            "vendor": _read(nm, "DeviceVendorName") or _call(desc.VendorName),
            "serial": _read(nm, "DeviceSerialNumber") or _call(desc.SerialNumber),
            "user_id": _read(nm, "DeviceUserID"),
            "firmware": fw, "firmware_ok": firmware_ok(fw), "firmware_min": ".".join(map(str, MIN_FIRMWARE)),
            "tl_type": _call(desc.TLType),
            "producer_cti": _call(lambda: desc.ParentInterface().ParentSystem().CTIFullPath()),
            "ip": ip_str(_read(nm, "GevCurrentIPAddress")) or None,
            "subnet": ip_str(_read(nm, "GevCurrentSubnetMask")) or None,
            "mac": mac_str(_read(nm, "GevMACAddress")) or None,
            "width": _read(nm, "Width"), "height": _read(nm, "Height"),
            "pixel_format": _read(nm, "PixelFormat"),
            "exposure_us": _read(nm, "ExposureTime"), "gain": _read(nm, "Gain"),
            "trigger": {k: _read(nm, k) for k in ("TriggerSelector", "TriggerMode", "TriggerSource")},
            "packet_size_b": _read(nm, "GevSCPSPacketSize"),
            "packet_size_max_b": _call(pkt.Maximum) if pkt is not None else None,
            "payload_b": _read(nm, "PayloadSize"),
            "temperature_c": _read(nm, "DeviceTemperature"),
            "ranges": self.ranges(),
        })
        return d

    # ── close ────────────────────────────────────────────────────────────────
    def close(self) -> None:
        """Stop acquisition, revoke buffers, unlock TL params, release device and library; never raises."""
        errors: list[str] = []
        p = self._p

        def attempt(what: str, fn: Callable, *args: Any) -> None:
            try:
                fn(*args)
            except Exception as e:
                errors.append(f"{what}: {type(e).__name__}: {e}")

        nm, ds = self._nm, self._ds
        if self._dev_running and nm is not None:
            attempt("AcquisitionStop", _execute, nm, "AcquisitionStop", self.cmd_timeout_ms)
        if self._ds_running and ds is not None:
            attempt("StopAcquisition", ds.StopAcquisition, p.AcquisitionStopMode_Default)
        if ds is not None:
            attempt("FlushAndRevokeAllBuffers", ds.FlushAndRevokeAllBuffers)
        if self._tl_locked and nm is not None:
            attempt("TLParamsLocked=0", lambda: nm.FindNode("TLParamsLocked").SetValue(0))
        lib_held = self._lib_held
        self._reset_handles()               # drop data stream / device / node map before Library.Close
        if lib_held and p is not None:
            _lib_release(p)
        for err in errors:
            self._log(f"warning during close: {err}")


# ── environment check (tools/cam_check.py list) ──────────────────────────────
def _access_status_name(p: Any, v: Any) -> str:
    names = {getattr(p, n): n.split("_", 1)[1] for n in dir(p) if n.startswith("DeviceAccessStatus_")}
    return names.get(v, str(v))


def device_info(p: Any, desc: Any, open_device: bool = True) -> dict[str, Any]:
    """What a descriptor tells without opening, plus (open_device) firmware/IP/temperature read with read-only
    access. The read-only open is UNTESTED on hardware."""
    iface = _iface_lookup(desc, ["GevDeviceIPAddress", "GevDeviceMACAddress"]) or {}
    d: dict[str, Any] = {
        "model": _call(desc.ModelName), "serial": _call(desc.SerialNumber), "vendor": _call(desc.VendorName),
        "display_name": _call(desc.DisplayName), "tl_type": _call(desc.TLType), "id": _call(desc.ID),
        "version": _call(desc.Version), "access_status": _access_status_name(p, _call(desc.AccessStatus)),
        "openable_control": _call(desc.IsOpenable, p.DeviceAccessType_Control),
        "ip": ip_str(iface.get("GevDeviceIPAddress")) or None,
        "mac": mac_str(iface.get("GevDeviceMACAddress")) or None,
    }
    if open_device:
        try:
            if not _call(desc.IsOpenable, p.DeviceAccessType_ReadOnly, default=False):
                raise CameraError("not openable read-only (in use or IP not in the NIC subnet)")
            dev = desc.OpenDevice(p.DeviceAccessType_ReadOnly)
            nm = dev.RemoteDevice().NodeMaps()[0]
            fw = _read(nm, "DeviceFirmwareVersion")
            d.update({"firmware": fw, "firmware_ok": firmware_ok(fw),
                      "ip": ip_str(_read(nm, "GevCurrentIPAddress")) or d["ip"],
                      "subnet": ip_str(_read(nm, "GevCurrentSubnetMask")) or None,
                      "mac": mac_str(_read(nm, "GevMACAddress")) or d["mac"],
                      "user_id": _read(nm, "DeviceUserID"), "temperature_c": _read(nm, "DeviceTemperature")})
            del nm, dev
        except Exception as e:
            d["open_error"] = f"{type(e).__name__}: {e}"
    return d


def environment(open_devices: bool = True, cti_path: str | Sequence[str] = "", search_install: bool = True,
                log: Callable[[str], None] = print) -> dict[str, Any]:
    """IDS peak environment check: versions, GenTL producers, systems, devices. `problem` holds the actionable
    message when something is missing (ok = False); never raises CameraError."""
    dirs = gentl_dirs()
    env: dict[str, Any] = {"python": sys.version.split()[0], "packages": package_versions(), "gentl_var": GENTL_ENV,
                           "gentl_dirs": dirs, "gentl_ctis": ctis_in(dirs), "ids_install_ctis": ids_install_ctis(),
                           "ids_peak_api": None, "ids_ctis": [], "systems": [], "interfaces": [], "devices": [],
                           "gev_ok": False, "gev_errors": [], "ok": False, "problem": None}
    try:
        with ids_library() as p:
            env["ids_peak_api"] = _call(lambda: p.Library.Version().ToString())
            env["ids_ctis"] = list(_call(p.EnvironmentInspector.CollectCTIPaths, default=[]) or [])
            ctis = [cti_path] if isinstance(cti_path, str) else list(cti_path)
            dm = _device_manager(p, ctis, search_install, log)
            env["systems"] = [{"vendor": _call(s.VendorName), "model": _call(s.ModelName),
                               "version": _call(s.Version), "tl_type": _call(s.TLType),
                               "cti": _call(s.CTIFullPath)} for s in dm.Systems()]
            env["interfaces"] = [_call(i.DisplayName) for i in dm.Interfaces()]
            env["devices"] = [device_info(p, d, open_devices) for d in dm.Devices()]
            diag, env["gev_errors"] = gev_diagnosis(p, dm, ctis, search_install)
            env["gev_ok"] = not diag
            if env["devices"]:
                env["ok"] = True
            else:
                env["problem"] = diag or MSG_NO_CAMERA
    except CameraError as e:
        env["problem"] = str(e)
    return env


__all__ = ["IdsCamera", "environment", "device_info", "gev_diagnosis", "probe_producer", "ids_library",
           "library_refs", "firmware_ok", "parse_version", "ip_str", "mac_str", "no_runtime_message",
           "producer_error_message", "MIN_FIRMWARE", "MSG_NO_CAMERA", "SETUP_DOC"]
