"""RoboDK simulated flange camera implementing the mauer.camera Camera contract (docs/ARCHITECTURE.md).

    with SimCamera(RDK, cfg, cam_tool, robot=robot) as cam:
        frame = cam.grab()            # Frame(image uint8 (H, W) grayscale, t_start, t_end, meta)

The camera is a RoboDK Cam2D attached to the robot tool "Camera" (build_station.add_camera: TCP = T_flange_cam), so
it follows the arm. Verified behaviour (research 2026-10-05, RoboDK 6.0.0.26652): ideal pinhole with
fx = fy = focal / pixel, cx = (W - 1) / 2, cy = (H - 1) / 2 and no distortion (fit over 378 corners: 4380.0 vs
4379.6 px nominal, RMS 0.36 px); camera frame = OpenCV frame; Cam2D_Snapshot('', cam) returns PNG bytes in
0.11-0.19 s; the first snapshot after Cam2D_Add can be empty or small -> warm-up with a shape check.

Pitfall (observed 2026-10-05): a textured object (board PNG via AddFile) that is added to the instance AFTER the
first Cam2D camera was opened renders black in every camera - add all boards before opening a camera (the study
and build_station.build do). Open cameras with RDK.Render(True); snapshots then also work with rendering off.

Pitfall (probe 2026-10-06, RoboDK 6.0.0.26652): closing the camera WINDOW (its X) leaves the item valid, but every
later Cam2D_Snapshot returns a 160 x 133 image; re-opening the item (Cam2D_Add(item, params, cam)) restores the full
size. Minimising or resizing the window has no effect, the flag NO_TASKBAR shrinks every snapshot the same way. The
camera window is therefore opened MINIMIZED (a 2472 x 2064 window covers the screen) and grab_bgr re-opens the
camera once when a snapshot has the wrong size.

The render is ideal: no blur, noise, distortion, vignetting or exposure effects - set_exposure_us / set_gain are
only recorded in the frame meta. Use intrinsics() as the "calibrated" camera for simulated images.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from rdk_common import camera_params

try:                                            # the real contract lives in mauer.camera (written in parallel)
    from mauer.camera.base import Frame
except ImportError:
    try:
        from mauer.camera import Frame
    except ImportError:
        @dataclass
        class Frame:
            """Fallback of mauer.camera Frame: image uint8 (H, W), laptop time.time() around the exposure."""
            image: np.ndarray
            t_start: float
            t_end: float
            meta: dict = field(default_factory=dict)


def intrinsics(cfg: dict) -> tuple[np.ndarray, np.ndarray, int, int]:
    """(K 3x3, D (5,), width, height) of the simulated camera: ideal pinhole from [camera] focal / pixel_um."""
    c = cfg["camera"]
    w, h = int(c["res_x"]), int(c["res_y"])
    f_px = c["focal"] / (c["pixel_um"] * 1e-3)
    K = np.array([[f_px, 0.0, (w - 1) / 2.0], [0.0, f_px, (h - 1) / 2.0], [0.0, 0.0, 1.0]])
    return K, np.zeros(5), w, h


class SimCamera:
    """Simulated IDS camera on the UR5 flange, duck-typed like mauer.camera.base.Camera (open, close, is_open, grab,
    set_exposure_us, set_gain, info, context manager). `attach_to` is the item the camera sits on (the "Camera"
    robot tool, or any frame); `robot` (optional) adds the joints and T_base_flange [mm] at the time of the grab to
    Frame.meta.

    `hide`: items made invisible during each snapshot. The modelled lens reaches 53 mm in front of the projection
    centre, so the camera would otherwise render its own lens; default: the object "Camera_body" when the camera
    sits on the tool "Camera" (build_station.add_camera). Hidden items are also excluded from RoboDK's collision
    check, so they are shown again right after the snapshot."""

    def __init__(self, RDK, cfg: dict, attach_to, robot=None, far_mm: float = 3000.0, warmup_s: float = 0.3,
                 hide: list | None = None):
        self.RDK, self.cfg, self.attach_to, self.robot = RDK, cfg, attach_to, robot
        self.far_mm, self.warmup_s = far_mm, warmup_s
        self.K, self.D, self.width, self.height = intrinsics(cfg)
        self.cam = None
        self.exposure_us = float(cfg["camera"].get("exposure_us", 0.0))
        self.gain = float(cfg["camera"].get("gain", 1.0))
        self._flat = False
        self._last_shape = None
        if hide is None:
            hide = []
            if attach_to.Name() == "Camera":
                body = RDK.Item("Camera_body")
                if body.Valid():
                    hide = [body]
        self.hide = list(hide)

    # ── Camera protocol ──────────────────────────────────────────────────────
    def _params(self, flat: bool) -> str:
        return camera_params(self.cfg, True, flat, self.far_mm) + " MINIMIZED"

    def open(self) -> "SimCamera":
        if self.cam is None:
            self.cam = self.RDK.Cam2D_Add(self.attach_to, self._params(False))
            if not self.cam.Valid():
                raise RuntimeError("Cam2D_Add failed")
        else:                                   # re-open the same item (e.g. after Cam2D_Close(0) elsewhere)
            self.RDK.Cam2D_Add(self.attach_to, self._params(self._flat), self.cam)
        time.sleep(self.warmup_s)
        for _ in range(5):                      # first snapshot after Cam2D_Add is unreliable
            if self._snapshot() is not None:
                break
            time.sleep(0.3)
        return self

    def close(self) -> None:
        if self.cam is not None:
            try:
                self.RDK.Cam2D_Close(self.cam)
                self.cam.Delete()
            finally:
                self.cam = None

    def grab(self) -> Frame:
        """Grayscale frame (mono IMX547). The image is the render of the current station state."""
        t0 = time.time()
        bgr = self.grab_bgr()
        t1 = time.time()
        # luminance weights on the BGR decode (RoboDK's own GRAYSCALE flag averages R, G, B instead)
        import cv2
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        return Frame(image=gray, t_start=t0, t_end=t1, meta=self._meta())

    @property
    def is_open(self) -> bool:
        return self.cam is not None

    def set_exposure_us(self, us: float) -> float:
        """Recorded in Frame.meta only (no effect on the render); returns the applied value."""
        self.exposure_us = float(us)
        return self.exposure_us

    def set_gain(self, g: float) -> float:
        """Recorded in Frame.meta only (no effect on the render); returns the applied value."""
        self.gain = float(g)
        return self.gain

    def info(self) -> dict:
        c = self.cfg["camera"]
        return {"source": "robodk", "model": f"RoboDK Cam2D simulating {c['model']} + {c['lens']}",
                "width": self.width, "height": self.height, "K": self.K.tolist(), "D": self.D.tolist(),
                "exposure_us": self.exposure_us, "gain": self.gain}

    def __enter__(self) -> "SimCamera":
        return self.open()

    def __exit__(self, *exc) -> bool:
        self.close()
        return False

    # ── extras for the simulation ────────────────────────────────────────────
    def set_flat_light(self, flat: bool) -> None:
        """Flat light: ambient only, every object renders in its exact colour (occlusion masks)."""
        if self.cam is None:
            raise RuntimeError("camera not open")
        if flat != self._flat:
            self.RDK.Cam2D_SetParams(self._params(flat), self.cam)
            self._flat = flat

    def grab_bgr(self) -> np.ndarray:
        """Colour render (H, W, 3) BGR uint8 of the current station state. A snapshot of the wrong size (camera
        window closed by the user, see the module docstring) re-opens the camera once."""
        if self.cam is None:
            raise RuntimeError("camera not open")
        img = self._snapshot()
        if img is None:
            self.open()                         # Cam2D_Add on the existing item + warm-up
            img = self._snapshot()
        if img is None:
            raise RuntimeError(f"bad snapshot: {self._last_shape} (also after re-opening the camera)")
        return img

    def _snapshot(self) -> np.ndarray | None:
        """One Cam2D snapshot as BGR, None if it is empty or not width x height (shape kept in _last_shape)."""
        import cv2
        shown = [it for it in self.hide if it.Visible()]
        for it in shown:
            it.setVisible(False)
        try:
            data = self.RDK.Cam2D_Snapshot("", self.cam)
        finally:
            for it in shown:
                it.setVisible(True)
        img = None
        if isinstance(data, (bytes, bytearray)) and len(data) > 0:
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        self._last_shape = None if img is None else img.shape
        if img is None or img.shape[:2] != (self.height, self.width):
            return None
        return img

    def _meta(self) -> dict:
        meta = {"source": "robodk", "exposure_us": self.exposure_us, "gain": self.gain}
        if self.robot is not None:
            from mauer.geometry import from_robodk
            j = self.robot.Joints().list()
            meta["joints_deg"] = j[:6]
            meta["T_base_flange"] = from_robodk(self.robot.SolveFK(j)).tolist()
        return meta
