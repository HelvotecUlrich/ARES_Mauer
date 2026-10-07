"""Time-lapse video of a RoboDK run (robodk/simulate.py --video): a fixed overview camera in the station, one frame
after every executed robot move and every ARES route sample, a caption per frame (what the run is doing), written as
MP4 at the end.

The camera is a Cam2D view like rdk_common.snapshot (renders with the RoboDK window minimised). rdk_common.snapshot /
snapshot_pinhole close every camera window (Cam2D_Close(0)); a failed frame therefore reopens the view once and
retries. Frames are PNGs in <video>_frames/ until write() encodes them (OpenCV, H.264 "avc1", else "mp4v") and
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

    def _open(self) -> None:
        if self.cam is not None:
            try:
                self.RDK.Cam2D_Close(self.cam)
                self.cam.Delete()
            except Exception:  # noqa: BLE001 - already closed by Cam2D_Close(0)
                pass
        w, h = self.size
        self.cam = self.RDK.Cam2D_Add(self.view, f"FOCAL_LENGTH=6 FOV={self.hfov:g} FAR_LENGTH=20000 SIZE={w}x{h} "
                                                 f"SNAPSHOT={w}x{h} BG_COLOR=white")
        self.RDK.Render(True)
        time.sleep(1.0)                                   # first render at full size (rdk_common.snapshot)

    def frame(self) -> None:
        """One frame of the current scene with the current caption."""
        p = self.dir / f"f{len(self.labels):05d}.png"
        ok = self.cam is not None and self.RDK.Cam2D_Snapshot(str(p), self.cam)
        if not ok:
            self._open()
            ok = self.RDK.Cam2D_Snapshot(str(p), self.cam)
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
        writer = None
        for cc in ("avc1", "mp4v"):
            writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*cc), self.fps, (w, h))
            if writer.isOpened():
                break
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
        return {"path": str(self.path), "frames": n, "seconds": n / self.fps, "failed_frames": self.failed}
