"""Time-lapse video of a RoboDK run (robodk/simulate.py --video): a fixed overview camera in the station, one frame
after every executed robot move and every ARES route sample, a caption per frame (what the run is doing), written as
MP4 at the end.

The camera is a Cam2D view like rdk_common.snapshot, its window opened MINIMIZED like sim_camera.SimCamera.
rdk_common.snapshot / snapshot_pinhole close every camera window (Cam2D_Close(0)); after that Cam2D_Snapshot on the
closed view still succeeds but renders 160 x 133 px (sim_camera.py, probe 2026-10-06; first full run 2026-10-07:
every frame after the station image pixelated). With the view open next to the flange camera for the whole run,
RoboDK crashed twice (0xC000041D, after ~1700 and ~520 frames, during planning) while runs without the view never
did: the view is therefore open only for its snapshot - re-opened (Cam2D_Add(item, params, cam), as
SimCamera.open), snapshot until it has full size, closed again. Frames are PNGs in <video>_frames/ until write() encodes
them with OpenCV "mp4v" (MPEG-4; "avc1" needs the OpenH264 DLL, without it OpenCV wrote 7x larger files) and
deletes them. The caption is drawn with PIL (umlauts; OpenCV's Hershey fonts are ASCII only).
"""
from __future__ import annotations

import shutil
import time
from pathlib import Path

from rdk_common import look_at

FONT = Path(r"C:\Windows\Fonts\arial.ttf")


class Recorder:
    def __init__(self, RDK, path: Path, eye: list, target: list, size: tuple = (1280, 720), fps: int = 30,
                 hfov_deg: float = 45.0):
        self.RDK, self.path, self.size, self.fps, self.hfov = RDK, Path(path), tuple(size), int(fps), hfov_deg
        self.dir = self.path.parent / f"{self.path.stem}_frames"
        shutil.rmtree(self.dir, ignore_errors=True)
        self.dir.mkdir(parents=True)
        self.view = RDK.AddFrame("_timelapse_view")
        self.view.setVisible(False)
        self.view.setPose(look_at(eye, target))
        self.cam = None
        self.label = ""
        self.labels: list[str] = []
        self.failed = 0
        self.reopens = 0

    def _open(self) -> None:
        w, h = self.size
        params = (f"FOCAL_LENGTH=6 FOV={self.hfov:g} FAR_LENGTH=20000 SIZE={w}x{h} SNAPSHOT={w}x{h} BG_COLOR=white "
                  "MINIMIZED")
        if self.cam is None:
            self.cam = self.RDK.Cam2D_Add(self.view, params)
        else:                                             # re-open the same item
            self.RDK.Cam2D_Add(self.view, params, self.cam)
            self.reopens += 1
        self.RDK.Render(True)

    def _full_size(self, p: Path) -> bool:
        from PIL import Image
        try:
            with Image.open(p) as im:
                return tuple(im.size) == self.size
        except OSError:
            return False

    def frame(self) -> None:
        """One frame of the current scene with the current caption (the view open only for it)."""
        p = self.dir / f"f{len(self.labels):05d}.png"
        self._open()
        ok = False
        for i in range(8):                                # the first snapshot after opening can be small / empty
            time.sleep(0.15 if i else 0.05)
            if self.RDK.Cam2D_Snapshot(str(p), self.cam) and self._full_size(p):
                ok = True
                break
        self.RDK.Cam2D_Close(self.cam)
        if ok:
            self.labels.append(self.label)
        else:
            self.failed += 1

    def write(self, title: str = "") -> dict:
        """Encode the frames (caption bar at the bottom, title top left) and delete them; {path, frames, seconds}."""
        import cv2
        import numpy as np
        from PIL import Image, ImageDraw, ImageFont
        if self.cam is not None:
            try:
                self.RDK.Cam2D_Close(self.cam)
                self.cam.Delete()
            except Exception:  # noqa: BLE001
                pass
            self.cam = None
        self.view.Delete()
        w, h = self.size
        font = ImageFont.truetype(str(FONT), 22) if FONT.exists() else ImageFont.load_default()
        small = ImageFont.truetype(str(FONT), 16) if FONT.exists() else font
        writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
        n = 0
        for i, label in enumerate(self.labels):
            img = Image.open(self.dir / f"f{i:05d}.png").convert("RGB").resize((w, h))
            d = ImageDraw.Draw(img)
            d.rectangle([0, h - 40, w, h], fill=(30, 30, 30))
            d.text((14, h - 33), label, fill=(255, 255, 255), font=font)
            if title:
                d.text((12, 10), title, fill=(60, 60, 60), font=small)
            writer.write(cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR))
            n += 1
        writer.release()
        shutil.rmtree(self.dir, ignore_errors=True)
        return {"path": str(self.path), "frames": n, "seconds": n / self.fps, "failed_frames": self.failed,
                "camera_reopens": self.reopens}
