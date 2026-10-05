"""Camera dataset folders (data/<name>/): lossless PNG images plus meta.json with the robot state per image.

Written by the capture tools (tools/calib_intrinsics.py capture, the robot-side hand-eye / measurement capture) and
read by the offline solvers. data/ is not versioned (large); the calibration files in calib/ name the dataset.

meta.json:
{"kind": "intrinsics" | "handeye" | "measure", "created": iso8601, "camera": {...camera info...}, "board": name,
 "notes": str,
 "samples": [{"image": "img_000.png", "t_start", "t_end",            # laptop time.time() around the exposure [s]
              "T_base_flange": 4x4 list [mm] | null,                  # flange pose at the exposure (RTDE)
              "q_rad": [6] | null, "tcp_pose_ur": [6] | null,          # joints [rad], UR pose [m, rotation vector]
              "max_qd": float | null, ...}]}                          # max |joint speed| during the exposure [rad/s]
Extra per-sample fields are kept. meta.json is rewritten atomically after every add().
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Iterator, Mapping

import cv2
import numpy as np

from .intrinsics import to_jsonable, now_iso

KINDS = ("intrinsics", "handeye", "measure")
META = "meta.json"


def write_png(path: str | Path, image: np.ndarray) -> None:
    """Lossless PNG; works with non-ASCII Windows paths (cv2.imwrite does not)."""
    ok, buf = cv2.imencode(".png", np.asarray(image), [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise IOError(f"PNG encoding failed for {path}")
    Path(path).write_bytes(buf.tobytes())


def read_image(path: str | Path) -> np.ndarray:
    """Image file as stored (mono PNG -> uint8 (H, W)); IOError if unreadable."""
    data = np.frombuffer(Path(path).read_bytes(), np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise IOError(f"cannot decode image {path}")
    return img


class Dataset:
    """A dataset folder. Create with Dataset.create(), open with Dataset.load(); iterate over (image, sample)."""

    def __init__(self, path: str | Path, meta: dict):
        self.path = Path(path)
        self.meta = meta

    # ── construction ──────────────────────────────────────────────────────────
    @classmethod
    def create(cls, path: str | Path, kind: str, camera: Mapping | None = None, board: str | None = None,
               notes: str = "", exist_ok: bool = False) -> "Dataset":
        """New empty dataset folder. Refuses an existing meta.json unless exist_ok (then it is loaded)."""
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {KINDS}")
        path = Path(path)
        if (path / META).exists():
            if not exist_ok:
                raise FileExistsError(f"dataset already exists: {path}")
            ds = cls.load(path)
            if ds.kind != kind:
                raise ValueError(f"existing dataset {path} is of kind {ds.kind!r}, not {kind!r}")
            return ds
        path.mkdir(parents=True, exist_ok=True)
        ds = cls(path, {"kind": kind, "created": now_iso(), "camera": to_jsonable(dict(camera or {})),
                        "board": board, "notes": notes, "samples": []})
        ds._write_meta()
        return ds

    @classmethod
    def load(cls, path: str | Path) -> "Dataset":
        path = Path(path)
        meta = json.loads((path / META).read_text(encoding="utf-8"))
        if meta.get("kind") not in KINDS:
            raise ValueError(f"{path / META}: unknown kind {meta.get('kind')!r}")
        meta.setdefault("samples", [])
        return cls(path, meta)

    # ── content ───────────────────────────────────────────────────────────────
    @property
    def kind(self) -> str:
        return self.meta["kind"]

    @property
    def board(self) -> str | None:
        return self.meta.get("board")

    @property
    def samples(self) -> list[dict]:
        return self.meta["samples"]

    def __len__(self) -> int:
        return len(self.samples)

    def image(self, i: int) -> np.ndarray:
        return read_image(self.path / self.samples[i]["image"])

    def __iter__(self) -> Iterator[tuple[np.ndarray, dict]]:
        for i, s in enumerate(self.samples):
            yield self.image(i), s

    @staticmethod
    def T_base_flange(sample: Mapping) -> np.ndarray | None:
        """4x4 [mm] of a sample, None when the sample has no robot pose."""
        T = sample.get("T_base_flange")
        return None if T is None else np.asarray(T, float).reshape(4, 4)

    def add(self, image: np.ndarray, t_start: float | None = None, t_end: float | None = None,
            T_base_flange: np.ndarray | None = None, q_rad=None, tcp_pose_ur=None, max_qd: float | None = None,
            **extra) -> dict:
        """Store an image losslessly as img_NNN.png and append its sample record (meta.json rewritten)."""
        i = len(self.samples)
        name = f"img_{i:03d}.png"
        while (self.path / name).exists():           # never overwrite an image (e.g. after a crash)
            i += 1
            name = f"img_{i:03d}.png"
        write_png(self.path / name, image)
        sample = {"image": name, "t_start": t_start, "t_end": t_end,
                  "T_base_flange": None if T_base_flange is None else np.asarray(T_base_flange, float).reshape(4, 4),
                  "q_rad": q_rad, "tcp_pose_ur": tcp_pose_ur, "max_qd": max_qd}
        sample.update(extra)
        sample = to_jsonable(sample)
        self.samples.append(sample)
        self._write_meta()
        return sample

    def update_meta(self, **fields) -> None:
        """Set top-level meta fields (e.g. camera=cam.info(), notes=...) and rewrite meta.json."""
        if "samples" in fields or ("kind" in fields and fields["kind"] not in KINDS):
            raise ValueError("samples cannot be replaced / unknown kind")
        self.meta.update(to_jsonable(fields))
        self._write_meta()

    def _write_meta(self) -> None:
        tmp = self.path / (META + ".tmp")
        tmp.write_text(json.dumps(to_jsonable(self.meta), indent=1) + "\n", encoding="utf-8")
        os.replace(tmp, self.path / META)

    def __repr__(self) -> str:
        return f"Dataset({str(self.path)!r}, kind={self.kind!r}, board={self.board!r}, samples={len(self)})"
