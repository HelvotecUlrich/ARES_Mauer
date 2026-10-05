"""Tests for mauer.camera: Frame/Camera contract, FileCamera replay, open_camera, IdsCamera and tools/cam_check.py.

IdsCamera runs against a fake `ids_peak` package (monkeypatched into sys.modules) that mirrors the names of the ids_peak
1.17.0 API used by IdsCamera (Library, DeviceManager, DeviceDescriptor, Device, RemoteDevice, NodeMap, nodes,
DataStream, Buffer, ImageView, EnvironmentInspector, ProducerLibrary) and emulates a software-triggered camera. The
fake's numbers (sensor size, packet sizes, firmware string, ranges) are test values, not camera facts. One test also
runs the real ids_peak (if installed) without a usable producer in a subprocess and checks the actionable message.
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

from mauer import config
from mauer.camera import Camera, CameraError, FileCamera, Frame, open_camera, save_frame
from mauer.camera import ids
from mauer.camera.base import center_roi, focus_measure, histogram, image_stats
from mauer.camera.files import load_sidecar

REPO = Path(__file__).resolve().parent.parent


# ── fake ids_peak ─────────────────────────────────────────────────────────────
class _PeakException(Exception):
    pass


class TimeoutException(_PeakException):
    pass


class NotFoundException(_PeakException):
    pass


class BadAccessException(_PeakException):
    pass


class InvalidArgumentException(_PeakException):
    pass


class OutOfRangeException(_PeakException):
    pass


class FakeNode:
    def __init__(self, name: str, log: list, writeable=True, readable=True, available=True):
        self._name, self._log = name, log
        self._w, self._r, self._a = writeable, readable, available

    @staticmethod
    def _eval(v):
        return v() if callable(v) else v

    def Name(self):
        return self._name

    def IsAvailable(self):
        return self._eval(self._a)

    def IsReadable(self):
        return self._eval(self._r)

    def IsWriteable(self):
        return self._eval(self._w)

    def _check_write(self):
        if not self.IsWriteable():
            raise BadAccessException(f"{self._name} not writeable")


class FakeEntry:
    def __init__(self, v):
        self.v = v

    def SymbolicValue(self):
        return self.v

    def IsAvailable(self):
        return True


class FakeEnum(FakeNode):
    def __init__(self, name, log, entries, value, **kw):
        super().__init__(name, log, **kw)
        self.entries, self.value = list(entries), value

    def CurrentEntry(self):
        return FakeEntry(self.value)

    def SetCurrentEntry(self, v):
        self._check_write()
        if v not in self.entries:
            raise InvalidArgumentException(f"{self._name}: no entry {v}")
        self.value = v
        self._log.append(("set", self._name, v))

    def HasEntry(self, v):
        return v in self.entries

    def Entries(self):
        return [FakeEntry(e) for e in self.entries]


class FakeNumber(FakeNode):
    def __init__(self, name, log, value, lo, hi, inc=None, **kw):
        super().__init__(name, log, **kw)
        self.value, self.lo, self.hi, self.inc = value, lo, hi, inc

    def Value(self):
        return self._eval(self.value)

    def SetValue(self, v):
        self._check_write()
        if not self.lo <= v <= self.hi:
            raise OutOfRangeException(f"{self._name}: {v} not in [{self.lo}, {self.hi}]")
        if self.inc and (v - self.lo) % self.inc:
            raise InvalidArgumentException(f"{self._name}: {v} not on the increment grid")
        self.value = v
        self._log.append(("set", self._name, v))

    def Minimum(self):
        return self.lo

    def Maximum(self):
        return self.hi

    def Increment(self):
        return self.inc or 1


class FakeString(FakeNode):
    def __init__(self, name, log, value, **kw):
        super().__init__(name, log, writeable=False, **kw)
        self.value = value

    def Value(self):
        return self._eval(self.value)


class FakeCommand(FakeNode):
    def __init__(self, name, log, action=None, **kw):
        super().__init__(name, log, **kw)
        self.action = action

    def Execute(self):
        self._check_write()
        self._log.append(("exec", self._name))
        if self.action:
            self.action()

    def WaitUntilDone(self, timeout_ms=500):
        self._log.append(("wait", self._name, timeout_ms))

    def IsDone(self):
        return True


class FakeNodeMap:
    def __init__(self, nodes: dict):
        self.nodes = nodes

    def HasNode(self, name):
        return name in self.nodes

    def FindNode(self, name):
        if name not in self.nodes:
            raise NotFoundException(f"node {name} not found")
        return self.nodes[name]

    def TryFindNode(self, name):
        return self.nodes.get(name)


class FakeImageView:
    def __init__(self, img):
        self.img = img

    def to_numpy_array(self, copy=True):
        return self.img.copy() if copy else self.img


class FakeBuffer:
    def __init__(self, idx, size, ds):
        self.idx, self.size, self.ds = idx, size, ds
        self.image = None
        self.frame_id = 0
        self.incomplete = False

    def IsIncomplete(self):
        return self.incomplete

    def ToImageView(self):
        return FakeImageView(self.image)

    def FrameID(self):
        return self.frame_id

    def Timestamp_ns(self):
        return self.frame_id * 1_000_000

    def SystemTimestamp_ns(self):
        return int(time.time() * 1e9)

    def Width(self):
        return self.image.shape[1]

    def Height(self):
        return self.image.shape[0]

    def ParentDataStream(self):
        return self.ds


class FakeDataStream:
    def __init__(self, dev):
        self.dev, self.log = dev, dev.log
        self.announced, self.input, self.output = [], [], []
        self.started = False

    def NumBuffersAnnouncedMinRequired(self):
        return 1

    def AddAcquisitionBuffers(self, size, count):
        assert size == self.dev.payload(), "buffer size must equal PayloadSize"
        for _ in range(count):
            b = FakeBuffer(len(self.announced), size, self)
            self.announced.append(b)
            self.input.append(b)
        self.log.append(("buffers", size, count))

    def StartAcquisition(self, *args):
        if self.started:
            raise BadAccessException("acquisition already running")
        self.started = True
        self.log.append(("ds_start",))

    def StopAcquisition(self, *args):
        self.started = False
        self.log.append(("ds_stop",) + args)

    def FlushAndRevokeAllBuffers(self):
        self.announced, self.input, self.output = [], [], []
        self.log.append(("revoke",))

    def NumBuffersAwaitDelivery(self):
        return len(self.output)

    def WaitForFinishedBuffer(self, timeout_ms):
        self.log.append(("wait_buffer", timeout_ms))
        if not self.output:
            raise TimeoutException("Wait for finished buffer timed out")
        return self.output.pop(0)

    def QueueBuffer(self, buf):
        if buf not in self.announced:
            raise InvalidArgumentException("buffer not announced")
        self.input.append(buf)
        self.log.append(("queue", buf.idx))

    def KillWait(self):
        pass


class FakeDataStreamDescriptor:
    def __init__(self, dev):
        self.dev = dev

    def OpenDataStream(self):
        self.dev.stream = FakeDataStream(self.dev)
        self.dev.log.append(("open_stream",))
        return self.dev.stream


class FakeRemoteDevice:
    def __init__(self, nm):
        self.nm = nm

    def NodeMaps(self):
        return [self.nm]


class FakeDevice:
    """Simulated GigE camera: node map with the IDS node names, software trigger fills one buffer per trigger."""

    def __init__(self, log, serial="4108000001", ip="192.168.50.10", width=64, height=48, gain_selector=True,
                 userset_resets_packet=False, temperature=True):
        self.log, self.serial, self.ip_int = log, serial, ids_ip(ip)
        self.width, self.height = width, height
        self.stream: FakeDataStream | None = None
        self.acquiring = False
        self.frames = 0
        self.drop_triggers = 0           # simulate lost triggers -> timeout
        self.incomplete_next = 0         # simulate packet loss
        self.trigger_times: list[float] = []
        self.userset_resets_packet = userset_resets_packet
        L = log
        idle = lambda: not self.acquiring                                  # noqa: E731
        unlocked = lambda: not self.acquiring and self.nodes["TLParamsLocked"].value == 0  # noqa: E731
        n = {
            "DeviceModelName": FakeString("DeviceModelName", L, "GV-51F0CP-M-GL"),
            "DeviceVendorName": FakeString("DeviceVendorName", L, "IDS Imaging Development Systems GmbH"),
            "DeviceSerialNumber": FakeString("DeviceSerialNumber", L, serial),
            "DeviceFirmwareVersion": FakeString("DeviceFirmwareVersion", L, "3.94.0.0"),   # fake format
            "DeviceUserID": FakeString("DeviceUserID", L, "ares_cam"),
            "GevCurrentIPAddress": FakeNumber("GevCurrentIPAddress", L, self.ip_int, 0, 2**32 - 1, writeable=False),
            "GevCurrentSubnetMask": FakeNumber("GevCurrentSubnetMask", L, ids_ip("255.255.255.0"), 0, 2**32 - 1,
                                               writeable=False),
            "GevMACAddress": FakeNumber("GevMACAddress", L, 0x000C_DF12_3456, 0, 2**48 - 1, writeable=False),
            "Width": FakeNumber("Width", L, width, 16, width, writeable=unlocked),
            "Height": FakeNumber("Height", L, height, 16, height, writeable=unlocked),
            "PayloadSize": FakeNumber("PayloadSize", L, self.payload, 0, 2**31, writeable=False),
            "UserSetSelector": FakeEnum("UserSetSelector", L, ["Default", "UserSet0", "UserSet1"], "UserSet0"),
            "UserSetLoad": FakeCommand("UserSetLoad", L, action=self._load_default, writeable=idle),
            "PixelFormat": FakeEnum("PixelFormat", L, ["Mono8", "Mono10", "Mono12", "Mono12p"], "Mono12",
                                    writeable=unlocked),
            "ExposureMode": FakeEnum("ExposureMode", L, ["Timed", "TriggerControlled"], "Timed"),
            "ExposureAuto": FakeEnum("ExposureAuto", L, ["Off", "Once", "Continuous"], "Continuous"),
            "ExposureTime": FakeNumber("ExposureTime", L, 10000.0, 20.0, 2_000_000.0),
            "GainAuto": FakeEnum("GainAuto", L, ["Off", "Once", "Continuous"], "Continuous"),
            "Gain": FakeNumber("Gain", L, 2.0, 1.0, 15.9),
            "AcquisitionMode": FakeEnum("AcquisitionMode", L, ["SingleFrame", "MultiFrame", "Continuous"],
                                        "SingleFrame", writeable=idle),
            "TriggerSelector": FakeEnum("TriggerSelector", L, ["AcquisitionStart", "ExposureStart", "FrameStart"],
                                        "FrameStart", writeable=idle),
            "TriggerMode": FakeEnum("TriggerMode", L, ["Off", "On"], "Off", writeable=idle),
            "TriggerSource": FakeEnum("TriggerSource", L, ["Line0", "Software"], "Line0", writeable=idle),
            "TriggerSoftware": FakeCommand("TriggerSoftware", L, action=self._trigger),
            "AcquisitionStart": FakeCommand("AcquisitionStart", L, action=self._acq_start),
            "AcquisitionStop": FakeCommand("AcquisitionStop", L, action=self._acq_stop),
            "TLParamsLocked": FakeNumber("TLParamsLocked", L, 0, 0, 1),
            "GevSCPSPacketSize": FakeNumber("GevSCPSPacketSize", L, 9000, 576, 16000, inc=4, writeable=idle),
        }
        if gain_selector:
            n["GainSelector"] = FakeEnum("GainSelector", L, ["AnalogAll", "DigitalAll"], "DigitalAll")
        if temperature:
            n["DeviceTemperature"] = FakeNumber("DeviceTemperature", L, 41.5, -50.0, 150.0, writeable=False)
        self.nodes = n
        self.nm = FakeNodeMap(n)

    def payload(self):
        bpp = 1 if self.nodes["PixelFormat"].value == "Mono8" else 2
        return self.width * self.height * bpp

    def _load_default(self):
        n = self.nodes
        n["ExposureTime"].value, n["Gain"].value = 15000.0, 1.0
        n["TriggerMode"].value, n["PixelFormat"].value = "Off", "Mono8"
        if self.userset_resets_packet:
            n["GevSCPSPacketSize"].value = 1500

    def _acq_start(self):
        self.acquiring = True

    def _acq_stop(self):
        self.acquiring = False

    def expected_image(self, k: int) -> np.ndarray:
        yy, xx = np.mgrid[0:self.height, 0:self.width]
        return ((xx * 3 + yy * 5 + k * 7) % 256).astype(np.uint8)

    def _trigger(self):
        self.trigger_times.append(time.time())
        n = self.nodes
        if not (self.acquiring and n["TriggerMode"].value == "On" and n["TriggerSource"].value == "Software"
                and n["TriggerSelector"].value == "ExposureStart"):
            self.log.append(("trigger_ignored",))
            return
        if self.drop_triggers:
            self.drop_triggers -= 1
            return
        ds = self.stream
        if ds is None or not ds.started or not ds.input:
            self.log.append(("underrun",))
            return
        buf = ds.input.pop(0)
        self.frames += 1
        buf.frame_id = self.frames
        buf.image = self.expected_image(self.frames)
        buf.incomplete = self.incomplete_next > 0
        self.incomplete_next = max(0, self.incomplete_next - 1)
        ds.output.append(buf)

    # ids_peak.Device API
    def RemoteDevice(self):
        return FakeRemoteDevice(self.nm)

    def DataStreams(self):
        return [FakeDataStreamDescriptor(self)]


def ids_ip(s: str) -> int:
    a = [int(x) for x in s.split(".")]
    return (a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3]


class FakeSystem:
    def __init__(self, tl="GEV"):
        self.tl = tl

    def VendorName(self):
        return "IDS Imaging Development Systems GmbH"

    def ModelName(self):
        return f"IDS GenICam Producer ({self.tl}K)"

    def Version(self):
        return "3.0.0"

    def TLType(self):
        return self.tl

    def CTIFullPath(self):
        return rf"C:\fake\ids_{self.tl.lower()}gentlk.cti"

    def DisplayName(self):
        return self.ModelName()


class FakeInterface:
    def __init__(self, fake):
        self.fake, L = fake, fake.log
        self.sel = FakeNumber("DeviceSelector", L, 0, 0, 0)
        self.nm = FakeNodeMap({
            "DeviceSelector": self.sel,
            "DeviceID": FakeString("DeviceID", L, lambda: f"dev-{fake.devices[self.sel.value].serial}"),
            "GevDeviceIPAddress": FakeNumber("GevDeviceIPAddress", L, lambda: fake.devices[self.sel.value].ip_int,
                                             0, 2**32 - 1, writeable=False),
            "GevDeviceMACAddress": FakeNumber("GevDeviceMACAddress", L, 0x000C_DF12_3456, 0, 2**48 - 1,
                                              writeable=False),
        })

    def NodeMaps(self):
        self.sel.hi = max(0, len(self.fake.devices) - 1)
        return [self.nm]

    def DisplayName(self):
        return "IDS GigE Vision (fake NIC)"

    def ParentSystem(self):
        return FakeSystem()


class FakeDescriptor:
    def __init__(self, fake, dev: FakeDevice):
        self.fake, self.dev = fake, dev

    def ID(self):
        return f"dev-{self.dev.serial}"

    def SerialNumber(self):
        return self.dev.serial

    def ModelName(self):
        return "GV-51F0CP-M-GL"

    def VendorName(self):
        return "IDS Imaging Development Systems GmbH"

    def DisplayName(self):
        return f"GV-51F0CP-M-GL ({self.dev.serial})"

    def TLType(self):
        return "GEV"

    def Version(self):
        return "fake"

    def AccessStatus(self):
        return 1

    def IsOpenable(self, access=3):
        return not self.fake.busy

    def OpenDevice(self, access):
        if self.fake.busy:
            raise BadAccessException("device in use")
        self.fake.log.append(("open_device", self.dev.serial, access))
        self.fake.opened.append(access)
        return self.dev

    def ParentInterface(self):
        return self.fake.interface


class FakeIds:
    """State of one fake ids_peak installation: module objects, devices, call log, library counters."""

    def __init__(self, n_devices=1, systems=("GEV",), **dev_kw):
        self.log: list = []
        self.devices = [FakeDevice(self.log, serial=f"41080000{i:02d}", ip=f"192.168.50.{10 + i}", **dev_kw)
                        for i in range(n_devices)]
        self.systems = list(systems)
        self.busy = False
        self.opened: list = []
        self.init_calls = 0
        self.close_calls = 0
        self.added_ctis: list[str] = []
        self.interface = FakeInterface(self)
        self.mod, self.pkg = self._build()

    def _build(self):
        fake = self
        m = types.ModuleType("ids_peak.ids_peak")
        m.DeviceAccessType_ReadOnly, m.DeviceAccessType_Control, m.DeviceAccessType_Exclusive = 2, 3, 4
        m.DeviceAccessStatus_ReadWrite, m.DeviceAccessStatus_Busy = 1, 4
        m.AcquisitionStopMode_Default = 0

        class Version:
            def ToString(self):
                return "1.17.0-fake"

        class Library:
            @staticmethod
            def Initialize():
                fake.init_calls += 1

            @staticmethod
            def Close():
                fake.close_calls += 1

            @staticmethod
            def Version():
                return Version()

        class DeviceManager:
            UpdatePolicy_ScanEnvironmentForProducerLibraries = 0
            UpdatePolicy_DontScanEnvironmentForProducerLibraries = 1
            _inst = None

            @staticmethod
            def Instance():
                if DeviceManager._inst is None:
                    DeviceManager._inst = DeviceManager()
                return DeviceManager._inst

            def AddProducerLibrary(self, path):
                fake.added_ctis.append(path)

            def Update(self, *args):
                fake.log.append(("update",) + args)

            def Systems(self):
                return [FakeSystem(t) for t in fake.systems]

            def Interfaces(self):
                return [fake.interface] if fake.systems else []

            def Devices(self):
                return [FakeDescriptor(fake, d) for d in fake.devices] if "GEV" in fake.systems else []

        class EnvironmentInspector:
            @staticmethod
            def CollectCTIPaths():
                return [rf"C:\fake\ids_{t.lower()}gentlk.cti" for t in fake.systems]

        class ProducerLibrary:
            @staticmethod
            def Open(path):
                class _Sys:
                    def OpenSystem(self):
                        raise _PeakException("TLOpen: Opening driver file '\\\\.\\ids_gevcore' failed!")

                class _Lib:
                    def System(self):
                        return _Sys()
                return _Lib()

        m.Library, m.DeviceManager = Library, DeviceManager
        m.EnvironmentInspector, m.ProducerLibrary = EnvironmentInspector, ProducerLibrary
        pkg = types.ModuleType("ids_peak")
        pkg.ids_peak = m
        pkg.TimeoutException = TimeoutException
        return m, pkg

    def calls(self, kind: str) -> list:
        return [c for c in self.log if c[0] == kind]

    def index(self, entry: tuple) -> int:
        return self.log.index(entry)


@pytest.fixture
def fake_ids(monkeypatch):
    """Factory: fake_ids(**kw) installs a fresh fake ids_peak and returns its FakeIds state."""
    def make(**kw) -> FakeIds:
        f = FakeIds(**kw)
        monkeypatch.setitem(sys.modules, "ids_peak", f.pkg)
        monkeypatch.setitem(sys.modules, "ids_peak.ids_peak", f.mod)
        monkeypatch.setattr(ids, "IDS_INSTALL_ROOTS", ())       # never touch a real IDS install from fake tests
        return f
    yield make
    assert ids.library_refs() == 0, "IDS peak library reference leaked"


def _cam(**kw) -> ids.IdsCamera:
    kw.setdefault("verbose", False)
    kw.setdefault("search_install", False)
    return ids.IdsCamera(**kw)


# ── Frame / quality helpers ───────────────────────────────────────────────────
def test_frame_contract():
    img = np.zeros((4, 6), np.uint8)
    f = Frame(img, 10.0, 10.5, {"exposure_us": 100.0})
    assert (f.height, f.width) == (4, 6)
    assert f.t_mid == pytest.approx(10.25) and f.latency_s == pytest.approx(0.5)
    with pytest.raises(ValueError):
        Frame(np.zeros((4, 6, 3), np.uint8), 0.0, 1.0)
    with pytest.raises(ValueError):
        Frame(img, 2.0, 1.0)


def test_image_stats_focus_histogram():
    img = np.full((100, 200), 100, np.uint8)
    img[:10, :] = 255                       # 10 % saturated
    img[-5:, :] = 0                         # 5 % black
    st = image_stats(img)
    assert st["saturated_pct"] == pytest.approx(10.0) and st["black_pct"] == pytest.approx(5.0)
    assert (st["min"], st["max"]) == (0, 255)
    assert histogram(img).sum() == img.size and histogram(img)[255] == 2000
    rs, cs = center_roi(img, 0.5)
    assert (rs.stop - rs.start, cs.stop - cs.start) == (50, 100)
    rng = np.random.default_rng(1)
    sharp = (rng.random((240, 320)) > 0.5).astype(np.uint8) * 255
    blurred = cv2.GaussianBlur(sharp, (0, 0), 3.0)
    assert focus_measure(sharp) > 10 * focus_measure(blurred)


# ── FileCamera ────────────────────────────────────────────────────────────────
def _write_images(folder: Path, n: int = 3, shape=(48, 64)) -> list[np.ndarray]:
    rng = np.random.default_rng(42)
    imgs = [rng.integers(0, 256, shape, dtype=np.uint8) for _ in range(n)]
    folder.mkdir(parents=True, exist_ok=True)
    for i, im in reversed(list(enumerate(imgs))):            # write out of order: sorting must not depend on it
        assert cv2.imwrite(str(folder / f"img_{i:03d}.png"), im)
    return imgs


def test_file_camera_roundtrip(tmp_path):
    imgs = _write_images(tmp_path / "set")
    (tmp_path / "set" / "notes.txt").write_text("not an image")      # does not match the pattern
    cam = FileCamera(tmp_path / "set", clock=lambda: 1000.0, exposure_us=5000.0)
    assert not cam.is_open
    with cam:
        assert cam.is_open and len(cam) == 3
        frames = [cam.grab() for _ in range(3)]
        for k, (f, im) in enumerate(zip(frames, imgs)):
            assert f.image.dtype == np.uint8 and f.image.shape == im.shape
            assert np.array_equal(f.image, im)
            assert f.meta["file"] == f"img_{k:03d}.png" and f.meta["frame_id"] == k
            assert f.meta["synthetic_time"] and f.t_start == 1000.0 and f.t_end == pytest.approx(1000.005)
        with pytest.raises(CameraError, match="replayed"):
            cam.grab()
        assert cam.set_exposure_us(1234.0) == 1234.0 and cam.set_gain(2.0) == 2.0
        assert cam.info()["exposure_us"] == 1234.0 and cam.info()["n_images"] == 3
    assert not cam.is_open


def test_file_camera_loop_color_and_errors(tmp_path):
    imgs = _write_images(tmp_path / "set", n=2)
    color = np.dstack([imgs[0], imgs[0], imgs[0]])
    cv2.imwrite(str(tmp_path / "set" / "img_999.png"), color)          # colour file -> grayscale
    with FileCamera(tmp_path / "set", loop=True) as cam:
        got = [cam.grab() for _ in range(5)]
    assert [f.meta["index"] for f in got] == [0, 1, 2, 0, 1]
    assert got[2].image.ndim == 2 and np.array_equal(got[2].image, imgs[0])
    assert got[4].meta["frame_id"] == 4
    with pytest.raises(CameraError, match="does not exist"):
        FileCamera(tmp_path / "missing").open()
    with pytest.raises(CameraError, match="no files"):
        FileCamera(tmp_path / "set", pattern="*.tif").open()
    with pytest.raises(CameraError, match="before open"):
        FileCamera(tmp_path / "set").grab()


def test_save_frame_sidecar_replay(tmp_path):
    img = np.arange(48 * 64, dtype=np.uint32).reshape(48, 64).astype(np.uint8)
    f = Frame(img, 1700000000.25, 1700000000.30, {"exposure_us": 4000.0, "gain": np.float64(1.5),
                                                  "frame_id": np.int64(17), "camera": "ids:4108"})
    png, side = save_frame(f, tmp_path / "rec" / "a.png")
    assert png.is_file() and side.name == "a.png.json"
    assert load_sidecar(png)["meta"]["frame_id"] == 17
    with FileCamera(tmp_path / "rec") as cam:
        g = cam.grab()
    assert np.array_equal(g.image, img)
    assert (g.t_start, g.t_end) == (f.t_start, f.t_end)
    assert g.meta["replayed"] and g.meta["camera"] == "ids:4108" and g.meta["frame_id"] == 17
    assert g.meta["gain"] == 1.5


# ── open_camera factory ───────────────────────────────────────────────────────
def test_open_camera_files(tmp_path):
    _write_images(tmp_path / "set", n=2)
    cfg = config.load()
    cam = open_camera(cfg, "files", folder=tmp_path / "set")
    try:
        assert isinstance(cam, FileCamera) and isinstance(cam, Camera) and cam.is_open
        f = cam.grab()
        assert f.meta["exposure_us"] == cfg["camera"]["exposure_us"]       # defaults from [camera]
    finally:
        cam.close()
    with open_camera(cfg, "files", folder=tmp_path / "set", exposure_us=777.0) as cam2:
        assert cam2.grab().meta["exposure_us"] == 777.0
    with pytest.raises(ValueError, match="folder"):
        open_camera(cfg, "files")
    with pytest.raises(ValueError, match="unknown camera kind"):
        open_camera(cfg, "robodk")


def test_open_camera_ids_uses_config(fake_ids):
    f = fake_ids()
    cfg = config.load()
    cfg["camera"]["serial"] = str(f.devices[0].serial)     # the real camera's serial is in the config
    cam = open_camera(cfg, "ids", verbose=False, search_install=False)
    try:
        assert isinstance(cam, ids.IdsCamera) and cam.is_open
        assert cam.ip == cfg["camera"]["ip"] and cam.serial == cfg["camera"]["serial"]
        dev = f.devices[0]
        assert dev.nodes["ExposureTime"].value == cfg["camera"]["exposure_us"]
        assert dev.nodes["Gain"].value == cfg["camera"]["gain"]
        assert dev.nodes["PixelFormat"].value == cfg["camera"]["pixel_format"]
    finally:
        cam.close()


# ── IdsCamera without a usable IDS runtime (real ids_peak) ────────────────────
def test_ids_no_package(monkeypatch):
    monkeypatch.setitem(sys.modules, "ids_peak", None)            # import -> ImportError
    with pytest.raises(CameraError, match="pip install ids_peak"):
        _cam().open()
    env = ids.environment(open_devices=False)
    assert not env["ok"] and "ids_peak" in env["problem"]
    assert ids.library_refs() == 0


_REAL_NO_RUNTIME = r"""
import sys
sys.path.insert(0, sys.argv[1])
from mauer.camera import CameraError
from mauer.camera.ids import IdsCamera, library_refs
try:
    IdsCamera(serial="", ip="", verbose=False, search_install=False).open()
    print("OPENED")
except CameraError as e:
    print("CAMERAERROR", e)
except BaseException as e:
    print("RAW", type(e).__module__, type(e).__name__, e)
print("REFS", library_refs())
"""


def test_ids_real_without_runtime_gives_actionable_error(tmp_path):
    """Real ids_peak, no IDS producer reachable (GenTL path = empty folder, install search off): CameraError with the
    'runtime not installed' message – in a subprocess so the real DeviceManager starts clean."""
    pytest.importorskip("ids_peak")
    env = {**os.environ, ids.GENTL_ENV: str(tmp_path)}
    r = subprocess.run([sys.executable, "-c", _REAL_NO_RUNTIME, str(REPO)], env=env, capture_output=True,
                       text=True, timeout=60)
    out = r.stdout + r.stderr
    assert "CAMERAERROR" in out, out
    assert "IDS peak runtime not installed" in out and ids.SETUP_DOC in out
    assert "RAW" not in out and "REFS 0" in out


def test_ids_real_state_never_raw(tmp_path):
    """Whatever the real installation state on this machine: open() raises CameraError with one of the actionable
    messages. Skipped if a camera could actually be opened (the test must never reconfigure a real camera); a camera
    that is attached but not openable (in use / IP outside the NIC subnet) fails before OpenDevice."""
    pytest.importorskip("ids_peak")
    env = ids.environment(open_devices=False, log=lambda m: None)
    if any(d["openable_control"] is not False for d in env["devices"]):
        pytest.skip("an openable IDS camera is attached - this test must not configure a real camera")
    with pytest.raises(CameraError) as ei:
        ids.IdsCamera(serial="", ip="", verbose=False).open()
    msg = str(ei.value)
    assert any(s in msg for s in ("IDS peak runtime not installed", "producer does not start", "no camera found",
                                  "ids_peak not importable", "cannot be opened for control")), msg
    assert ids.library_refs() == 0


# ── IdsCamera against the fake ids_peak ───────────────────────────────────────
def test_ids_configure_and_trigger_sequence(fake_ids):
    f = fake_ids()
    cam = _cam(exposure_us=8000.0, gain=2.5, timeout_ms=1500)
    cam.open()
    try:
        dev, log = f.devices[0], f.log
        n = dev.nodes
        assert n["PixelFormat"].value == "Mono8"
        assert n["ExposureAuto"].value == "Off" and n["GainAuto"].value == "Off"
        assert n["GainSelector"].value == "AnalogAll"
        assert n["ExposureTime"].value == 8000.0 and n["Gain"].value == 2.5
        assert n["AcquisitionMode"].value == "Continuous"
        assert (n["TriggerSelector"].value, n["TriggerMode"].value, n["TriggerSource"].value) == \
            ("ExposureStart", "On", "Software")
        # order: user set first, selector before mode/source, settings before buffers, lock before start
        i_load = f.index(("exec", "UserSetLoad"))
        assert f.index(("set", "UserSetSelector", "Default")) < i_load
        assert i_load < f.index(("set", "ExposureTime", 8000.0)) < f.index(("buffers", 64 * 48, 3))
        assert f.index(("set", "GainSelector", "AnalogAll")) < f.index(("set", "Gain", 2.5))
        assert f.index(("set", "TriggerSelector", "ExposureStart")) < f.index(("set", "TriggerMode", "On"))
        assert f.index(("set", "TriggerMode", "On")) < f.index(("buffers", 64 * 48, 3))
        assert f.index(("buffers", 64 * 48, 3)) < f.index(("set", "TLParamsLocked", 1)) < f.index(("ds_start",))
        assert f.index(("ds_start",)) < f.index(("exec", "AcquisitionStart"))
        assert ("wait", "AcquisitionStart", cam.cmd_timeout_ms) in log
        assert f.opened == [f.mod.DeviceAccessType_Control]
        assert dev.acquiring and f.init_calls == 1

        n0 = len(f.log)
        fr = cam.grab()
        assert f.log[n0:].count(("exec", "TriggerSoftware")) == 1
        assert fr.image.dtype == np.uint8 and fr.image.shape == (48, 64)
        assert np.array_equal(fr.image, dev.expected_image(1))
        assert fr.t_start <= dev.trigger_times[-1] <= fr.t_end
        assert fr.meta["frame_id"] == 1 and fr.meta["timestamp_ns"] == 1_000_000
        assert fr.meta["exposure_us"] == 8000.0 and fr.meta["gain"] == 2.5
        assert fr.meta["pixel_format"] == "Mono8" and fr.meta["camera"] == "ids:4108000000"
        assert ("wait_buffer", 1500) in f.log
        # buffer requeued: all announced buffers back in the input pool
        assert len(dev.stream.input) == 3 and not dev.stream.output
        fr2 = cam.grab()
        assert fr2.meta["frame_id"] == 2 and "frame_id_gap" not in fr2.meta
        assert np.array_equal(fr2.image, dev.expected_image(2))
        assert len(f.calls("queue")) == 2
    finally:
        cam.close()
    assert not f.devices[0].acquiring and f.close_calls == 1


def test_ids_close_sequence_and_idempotent(fake_ids):
    f = fake_ids()
    cam = _cam()
    cam.open()
    stream = f.devices[0].stream
    cam.close()
    i_stop = f.index(("exec", "AcquisitionStop"))
    i_ds_stop = f.index(("ds_stop", f.mod.AcquisitionStopMode_Default))
    i_revoke = f.index(("revoke",))
    i_unlock = f.index(("set", "TLParamsLocked", 0))
    assert i_stop < i_ds_stop < i_revoke < i_unlock
    assert not stream.started and not stream.announced
    assert f.init_calls == 1 and f.close_calls == 1
    cam.close()                                                 # second close: no-op
    assert f.close_calls == 1 and not cam.is_open
    with pytest.raises(CameraError, match="before open"):
        cam.grab()


def test_ids_library_refcount_two_cameras(fake_ids):
    f = fake_ids(n_devices=2)
    a = _cam(serial="4108000000")
    b = _cam(serial="4108000001")
    with a, b:
        assert f.init_calls == 1 and ids.library_refs() == 2
        assert a.grab().meta["camera"] == "ids:4108000000"
        assert b.grab().meta["camera"] == "ids:4108000001"
    assert f.init_calls == 1 and f.close_calls == 1 and ids.library_refs() == 0


def test_ids_timeout_incomplete_stale(fake_ids):
    f = fake_ids()
    with _cam(timeout_ms=50) as cam:
        dev = f.devices[0]
        dev.drop_triggers = 1
        with pytest.raises(CameraError, match="no frame within 50 ms"):
            cam.grab()
        dev.incomplete_next = 1
        with pytest.raises(CameraError, match="incomplete frame"):
            cam.grab()
        assert len(dev.stream.input) == 3                       # incomplete buffer was requeued
        # a late frame waiting in the output queue is dropped, the next grab returns a fresh one
        dev.nodes["TriggerSoftware"].Execute()                  # simulated late frame (frame 2)
        fr = cam.grab()
        assert fr.meta["stale_dropped"] == 1 and fr.meta["frame_id"] == 3
        assert "frame_id_gap" not in fr.meta                    # frames 1 (incomplete) and 2 (stale) were received
        assert np.array_equal(fr.image, dev.expected_image(3))
        assert len(dev.stream.input) == 3
        dev.frames += 2                                         # two frames lost in transport
        fr = cam.grab()
        assert fr.meta["frame_id"] == 6 and fr.meta["frame_id_gap"] == 2


def test_ids_select_by_serial_and_ip(fake_ids):
    f = fake_ids(n_devices=3)
    with _cam(ip="192.168.50.12") as cam:
        assert cam.info()["serial"] == "4108000002" and cam.info()["ip"] == "192.168.50.12"
    with _cam(serial="4108000001") as cam:
        assert cam.grab().meta["camera"] == "ids:4108000001"
    with _cam(serial="4108000001", ip="192.168.50.11") as cam:
        assert cam.info()["serial"] == "4108000001"
    with pytest.raises(CameraError) as ei:
        _cam(serial="999").open()
    assert "no IDS camera with serial 999" in str(ei.value) and "4108000000" in str(ei.value)
    with pytest.raises(CameraError, match="IP 10.0.0.1"):
        _cam(ip="10.0.0.1").open()
    assert ids.library_refs() == 0


def test_ids_no_producer_no_device_busy(fake_ids, monkeypatch):
    f = fake_ids(systems=())
    monkeypatch.setattr(f.mod.EnvironmentInspector, "CollectCTIPaths", staticmethod(lambda: []))
    with pytest.raises(CameraError, match="IDS peak runtime not installed") as ei:
        _cam().open()
    assert ids.SETUP_DOC in str(ei.value)
    # producer installed but its system does not open (seen with GEVK while the installer ran)
    f = fake_ids(systems=("U3V",))
    monkeypatch.setattr(f.mod.EnvironmentInspector, "CollectCTIPaths",
                        staticmethod(lambda: [r"C:\fake\ids_gevgentlk.cti"]))
    with pytest.raises(CameraError, match="producer does not start") as ei:
        _cam().open()
    assert "ids_gevcore" in str(ei.value) and "Socket (GEV)" in str(ei.value)
    f = fake_ids(n_devices=0)
    with pytest.raises(CameraError, match="no camera found - check power"):
        _cam().open()
    f = fake_ids()
    f.busy = True
    with pytest.raises(CameraError, match="cannot be opened for control"):
        _cam().open()
    assert f.init_calls == 1 and f.close_calls == 1


def test_ids_packet_size_modes(fake_ids):
    f = fake_ids(userset_resets_packet=True)
    with _cam() as cam:                                          # auto: keep the negotiated 9000 B
        assert f.devices[0].nodes["GevSCPSPacketSize"].value == 9000
        assert cam.info()["packet_size_b"] == 9000 and cam.info()["packet_size_max_b"] == 16000
    f = fake_ids()
    with _cam(packet_size="max") as cam:
        assert cam.info()["packet_size_b"] == 16000
    f = fake_ids()
    with _cam(packet_size=8001) as cam:                          # aligned to the increment grid (576 + 4 k)
        assert cam.info()["packet_size_b"] == 8000
    with pytest.raises(ValueError):
        _cam(packet_size="huge")


def test_ids_settings_info_and_optional_nodes(fake_ids):
    f = fake_ids(gain_selector=False, temperature=False)
    cam = _cam()
    assert cam.set_exposure_us(3000.0) == 3000.0                 # before open: stored
    with cam:
        n = f.devices[0].nodes
        assert n["ExposureTime"].value == 3000.0
        assert cam.set_exposure_us(5.0) == 20.0                  # clamped to the device minimum
        assert cam.set_exposure_us(12345.0) == 12345.0
        assert cam.set_gain(100.0) == 15.9 and cam.set_gain(1.0) == 1.0
        assert cam.grab().meta["exposure_us"] == 12345.0
        info = cam.info()
        assert info["model"] == "GV-51F0CP-M-GL" and info["firmware"] == "3.94.0.0" and info["firmware_ok"]
        assert info["ip"] == "192.168.50.10" and info["subnet"] == "255.255.255.0"
        assert info["mac"] == "00:0C:DF:12:34:56"
        assert info["temperature_c"] is None and info["ids_peak_api"] == "1.17.0-fake"
        assert info["trigger"] == {"TriggerSelector": "ExposureStart", "TriggerMode": "On",
                                   "TriggerSource": "Software"}
        assert info["ranges"]["exposure_us"] == (20.0, 2_000_000.0)
        json.dumps(info)                                         # JSON-serialisable
    with pytest.raises(ValueError, match="pixel_format"):
        _cam(pixel_format="Mono12p")


def test_ids_open_failure_releases_everything(fake_ids):
    f = fake_ids()
    del f.devices[0].nodes["TriggerSoftware"]
    with pytest.raises(CameraError, match="TriggerSoftware"):
        _cam().open()
    assert not f.devices[0].acquiring and f.close_calls == 1
    assert ("revoke",) in f.log and ("set", "TLParamsLocked", 0) in f.log


def test_firmware_and_format_helpers():
    assert ids.firmware_ok("3.94.0.0") and ids.firmware_ok("3.31") and not ids.firmware_ok("3.30.1")
    assert ids.firmware_ok("IDS 4.2") and ids.firmware_ok("unknown") is None
    assert ids.ip_str(ids_ip("192.168.50.10")) == "192.168.50.10"
    assert ids.mac_str(0x000CDF123456) == "00:0C:DF:12:34:56"
    assert "Socket (GEV)" in ids.producer_error_message([("x.cti", "Opening driver file '\\\\.\\ids_gevcore'")])


def test_environment_with_fake(fake_ids):
    fake_ids(n_devices=1)
    env = ids.environment(open_devices=True, log=lambda m: None)
    assert env["ok"] and env["gev_ok"] and env["problem"] is None
    d = env["devices"][0]
    assert d["serial"] == "4108000000" and d["ip"] == "192.168.50.10" and d["firmware_ok"]
    assert d["mac"] == "00:0C:DF:12:34:56" and "open_error" not in d


# ── tools/cam_check.py ────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def cam_check():
    spec = importlib.util.spec_from_file_location("cam_check", REPO / "tools" / "cam_check.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_cam_check_grab_files(cam_check, tmp_path, capsys):
    _write_images(tmp_path / "set", n=3)
    out = tmp_path / "out"
    rc = cam_check.main(["grab", "--source", "files", "--folder", str(tmp_path / "set"), "-n", "3",
                         "--out", str(out)])
    assert rc == 0
    assert sorted(p.name for p in out.glob("img_*.png")) == ["img_0000.png", "img_0001.png", "img_0002.png"]
    summary = json.loads((out / "cam_check_grab.json").read_text(encoding="utf-8"))
    assert summary["summary"]["n_ok"] == 3 and len(summary["frames"]) == 3
    assert "saturation max" in capsys.readouterr().out
    rc = cam_check.main(["grab", "--source", "files", "--folder", str(tmp_path / "set"), "-n", "4", "--no-save"])
    assert rc == 1                                               # 4th frame: folder exhausted -> error counted


def test_cam_check_ids_fake(cam_check, fake_ids, tmp_path, capsys):
    fake_ids()
    assert cam_check.main(["list"]) == 0
    out = capsys.readouterr().out
    assert "GV-51F0CP-M-GL serial 4108000000 ip 192.168.50.10" in out and "needs >= 3.31" in out
    any_cam = ["--serial", "", "--ip", ""]                        # config holds the real camera's serial
    assert cam_check.main(["grab", *any_cam, "-n", "2", "--quiet", "--out", str(tmp_path / "g")]) == 0
    assert len(list((tmp_path / "g").glob("img_*.png.json"))) == 2
    assert cam_check.main(["exposure", "7000", *any_cam, "--quiet"]) == 0
    assert "applied 7000.0 us" in capsys.readouterr().out


def test_cam_check_errors_are_messages(cam_check, fake_ids, capsys):
    fake_ids(systems=())
    assert cam_check.main(["grab", "--quiet"]) == 2
    out = capsys.readouterr().out
    assert "ERROR: IDS peak runtime not installed" in out and "Traceback" not in out
    assert cam_check.main(["list"]) == 2
    assert cam_check.main(["grab", "--source", "files"]) == 2  # no --folder
    assert "--folder" in capsys.readouterr().out


def test_render_live(cam_check):
    rng = np.random.default_rng(3)
    img = rng.integers(0, 256, (2064, 2472), dtype=np.uint8)
    view = cam_check.render_live(img, ["line 1", "line 2"], roi_frac=0.25, max_width=1000)
    assert view.dtype == np.uint8 and view.shape[2] == 3 and view.shape[1] == 1000
    zoom = cam_check.render_live(img, ["z"], max_width=800, zoom=True)
    assert zoom.shape[:2] == (600, 800)
