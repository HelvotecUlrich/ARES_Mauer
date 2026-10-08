"""Camera overlay (docs/HMI_DESIGN.md section 11.1) - pure, no Qt.

make_shot_view turns one camera image into the display copy the camera view shows: a downscaled preview with the
ChArUco corners found by mauer.vision.detect (the detector the sequencer measured with), coloured by the result the
sequencer logged for the board in the 'shot' record (accepted / rejected), board names at the corners and a legend
with every board of the look. A manual grab (no record) shows every board in view ("seen").

It runs in the run thread "mauer-run" through the controller's shot processor: the detectors are cached per process
and must be used from one thread (detect.py), the sequencer's measure() runs in the same thread, and the frame tap is
called after measure(), so the measurement is never changed. Re-detecting costs ~11 ms per sequencer shot (2472 x 2064,
the look's boards) and ~20 ms for a manual grab (all boards), measured 2026-10-08 with py.exe on this laptop.

Pixel convention as detect.draw_detections: preview px = (image px + 0.5) * (preview size / image size) - 0.5.
write_snapshot stores the shown image (and the full camera image) with a JSON sidecar; it may run in any thread.
"""
from __future__ import annotations

import json
import re
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np

from mauer.vision.detect import detect_boards
from mauer.vision.targets import BoardSpec, board_specs

from .snapshot import ShotView, to_uint8

# board result -> overlay colour (BGR)
STATUS_BGR = {"accepted": (60, 220, 60), "seen": (230, 200, 40), "partial": (0, 170, 255),
              "rejected": (0, 140, 255), "not seen": (80, 80, 255)}
STATUSES = tuple(STATUS_BGR)
FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass(frozen=True)
class ShotContext:
    specs: Mapping[str, BoardSpec]      # board_specs(cfg)
    border_px: float                    # [vision] border_px (as the measurement)
    scale: float                        # [hmi] camera_preview_scale
    show_ids: bool                      # draw the ChArUco corner ids
    min_corners: int = 8                # [vision] min_corners (a manual grab: enough corners for a pose?)


def shot_context(cfg: Mapping, hmi: Mapping, show_ids: bool = False) -> ShotContext:
    v = cfg.get("vision", {})
    return ShotContext(board_specs(cfg), float(v.get("border_px", 10)), float(hmi.get("camera_preview_scale", 0.25)),
                       bool(show_ids), int(v.get("min_corners", 8)))


@dataclass(frozen=True)
class BoardResult:
    """One board of a shot as the operator sees it."""
    board: str
    status: str             # sequencer shot: accepted | rejected | not seen; manual grab: seen | partial
    n_corners: int          # corners of the measurement (shot) or found in the image (grab)
    rms_px: float | None    # RMS reprojection error of the accepted board pose [px]
    reason: str             # why rejected / not seen / partial

    def text(self) -> str:
        if self.status == "accepted":
            rms = f" {self.rms_px:.2f} px" if self.rms_px is not None else ""
            return f"{self.board} accepted {self.n_corners} corners{rms}"
        if self.status == "seen":
            return f"{self.board} seen {self.n_corners} corners"
        return f"{self.board} {self.status}" + (f": {self.reason}" if self.reason else "")


@dataclass(frozen=True, eq=False)
class CameraShot(ShotView):
    """ShotView of the camera view: plus the full image (snapshot), the clean preview (re-drawn when the corner-id
    switch changes) and the board results."""
    image: np.ndarray | None = None     # full camera image, uint8 (the frame's own array, not copied)
    base: np.ndarray | None = None      # grey preview without the overlay
    boards: tuple[BoardResult, ...] = ()
    header: str = ""                    # first legend line
    ids_drawn: bool = False             # the preview shows the corner ids
    detect_ms: float = 0.0              # detection + drawing time in the run thread [ms]


def _float(v: Any) -> float | None:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if np.isfinite(f) else None


def detections_preview(dets: Mapping, sx: float, sy: float) -> dict:
    """{board: {"n", "ids", "pts" (preview px), "markers"}} from detect_boards() results."""
    out = {}
    for name, d in dets.items():
        p = np.asarray(d.img_pts, float).reshape(-1, 2)
        p = np.column_stack(((p[:, 0] + 0.5) * sx - 0.5, (p[:, 1] + 0.5) * sy - 0.5))
        out[name] = {"n": int(d.n), "ids": [int(i) for i in d.corner_ids], "pts": np.round(p, 2).tolist(),
                     "markers": int(len(d.marker_ids))}
    return out


def board_results(rec: Mapping | None, detections: Mapping | None, min_corners: int = 8) -> tuple[BoardResult, ...]:
    """The boards of a 'shot' record in its order (accepted / rejected / not seen), or for a manual grab (rec None)
    every board found in the image (seen: >= min_corners corners, else partial)."""
    dets = detections or {}
    if rec is not None and rec.get("boards") is not None:
        out = []
        for b, r in rec["boards"].items():
            n = int(r.get("n_corners") or 0)
            if r.get("ok"):
                out.append(BoardResult(b, "accepted", n, _float(r.get("rms_px")), ""))
            else:
                seen = n > 0 or int(dets.get(b, {}).get("n", 0)) > 0
                out.append(BoardResult(b, "rejected" if seen else "not seen", n, None, str(r.get("reason", ""))))
        return tuple(out)
    out = []
    for b, d in sorted(dets.items()):
        n = int(d.get("n", 0))
        if n >= min_corners:
            out.append(BoardResult(b, "seen", n, None, ""))
        else:
            why = f"{n} corners < {min_corners}, no pose" if n else f"{d.get('markers', 0)} markers, no corners"
            out.append(BoardResult(b, "partial", n, None, why))
    return tuple(out)


def header_text(rec: Mapping | None, t: float) -> str:
    stamp = time.strftime("%H:%M:%S", time.localtime(t))
    if rec is None:
        return f"manual grab  {stamp}"
    return f"look {rec.get('look', '?')} ({rec.get('parent', '?')})  {stamp}"


def _outlined(img: np.ndarray, text: str, org: tuple[int, int], fs: float, col, thick: int = 1) -> None:
    """Text with a dark outline: readable on the white board margin and on the black squares."""
    cv2.putText(img, text, org, FONT, fs, (20, 20, 20), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, fs, col, thick, cv2.LINE_AA)


def render_overlay(base: np.ndarray, detections: Mapping | None, results: tuple[BoardResult, ...], header: str,
                   show_ids: bool) -> np.ndarray:
    """BGR preview: corners as dots (ids optional) and the board name at each board, coloured by its result; a
    legend (header + one line per board) top left on a darkened box."""
    out = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR) if base.ndim == 2 else base.copy()
    h, w = out.shape[:2]
    fs = min(max(w / 1500.0, 0.35), 0.8)                # 0.41 for the 618 px preview of a 2472 px image
    status = {r.board: r.status for r in results}
    for name, d in (detections or {}).items():
        if not d.get("n"):
            continue
        col = STATUS_BGR.get(status.get(name, "seen"), STATUS_BGR["seen"])
        pts = np.asarray(d["pts"], float).reshape(-1, 2)
        for (x, y), cid in zip(pts, d["ids"]):
            c = (int(round(x)), int(round(y)))
            cv2.circle(out, c, 4, (20, 20, 20), -1, cv2.LINE_AA)
            cv2.circle(out, c, 3, col, -1, cv2.LINE_AA)
            if show_ids:
                _outlined(out, str(cid), (c[0] + 5, c[1] - 5), fs * 0.8, col)
        x0, y0 = pts.min(axis=0)
        _outlined(out, f"{name} ({d['n']})", (int(x0), max(int(y0) - 10, 14)), fs * 1.2, col, 2)
    lines = [(header, (225, 225, 225))] + [(r.text(), STATUS_BGR.get(r.status, (225, 225, 225))) for r in results]
    lines = [(t if len(t) <= 70 else t[:67] + "...", c) for t, c in lines if t]
    if lines:
        (_, th), base_px = cv2.getTextSize("Ag", FONT, fs, 1)
        line_h = th + base_px + 4
        tw = max(cv2.getTextSize(t, FONT, fs, 1)[0][0] for t, _ in lines)
        x1, y1 = min(w, tw + 12), min(h, line_h * len(lines) + 6)
        out[:y1, :x1] = (out[:y1, :x1] * 0.3).astype(np.uint8)
        for i, (t, c) in enumerate(lines):
            cv2.putText(out, t, (6, line_h * (i + 1)), FONT, fs, c, 1, cv2.LINE_AA)
    return out


def make_shot_view(image: np.ndarray, rec: Mapping | None, cfg: Mapping, sctx: ShotContext) -> CameraShot:
    """Run thread: detect the boards of the record (all boards for a manual grab), downscale, draw. A failing
    detection leaves the preview without corners and says why in `note`."""
    t0 = time.perf_counter()
    img = to_uint8(np.asarray(image))
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = img.shape[:2]
    names = list(rec["boards"]) if rec is not None and rec.get("boards") is not None else list(sctx.specs)
    specs = [sctx.specs[b] for b in names if b in sctx.specs]
    note = ""
    try:
        dets = detect_boards(img, specs, sctx.border_px) if specs else {}
    except Exception as e:          # noqa: BLE001 - the image is still shown
        dets, note = {}, f"detection failed: {type(e).__name__}: {e}"
    size = (max(1, int(round(w * sctx.scale))), max(1, int(round(h * sctx.scale))))
    base = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    detections = detections_preview(dets, size[0] / w, size[1] / h)
    results = board_results(rec, detections, sctx.min_corners)
    t = float(rec.get("t", time.time())) if rec is not None else time.time()
    header = header_text(rec, t)
    preview = render_overlay(base, detections, results, header, sctx.show_ids)
    return CameraShot(t=t, look=rec.get("look") if rec is not None else "live",
                      parent=rec.get("parent") if rec is not None else None, preview=preview,
                      scale=float(sctx.scale), full_size=(int(w), int(h)), rec=dict(rec) if rec is not None else None,
                      detections=detections, note=note, image=img, base=base, boards=results, header=header,
                      ids_drawn=bool(sctx.show_ids), detect_ms=(time.perf_counter() - t0) * 1000.0)


def redraw(view: CameraShot, show_ids: bool) -> CameraShot:
    """The same shot drawn again (corner-id switch) - GUI thread, ~1 ms, no detection."""
    if view.base is None or view.ids_drawn == bool(show_ids):
        return view
    return replace(view, preview=render_overlay(view.base, view.detections, view.boards, view.header, show_ids),
                   ids_drawn=bool(show_ids))


class OverlayProcessor:
    """The camera view's ShotProcessor (called in the run thread): make_shot_view with a ShotContext cached per
    session config (the cache holds the config, so the identity check cannot see a recycled id). show_ids is set
    from the GUI thread and read per shot."""

    def __init__(self, hmi: Mapping) -> None:
        self.hmi = dict(hmi)
        self.show_ids = False
        self._cache: tuple[Mapping, ShotContext] | None = None

    def context(self, cfg: Mapping) -> ShotContext:
        c = self._cache
        if c is None or c[0] is not cfg:
            c = (cfg, shot_context(cfg, self.hmi))
            self._cache = c
        return c[1]

    def __call__(self, image: np.ndarray, rec: dict | None, cfg: Mapping) -> CameraShot:
        return make_shot_view(image, rec, cfg, replace(self.context(cfg), show_ids=bool(self.show_ids)))


# ── snapshots ─────────────────────────────────────────────────────────────────
def _jsonable(x: Any) -> Any:
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, np.generic):
        return x.item()
    return str(x)


def write_snapshot(folder: Path, view: ShotView, fit: Mapping | None = None) -> list[Path]:
    """<folder>/<YYYYmmdd_HHMMSS_mmm>_<look>_view.png (the shown preview with its overlay), ..._full.png (the full
    camera image, when the view kept it) and ....json (shot record, board results, detections, the last frame
    fit). File IO only - any thread. Returns the written paths."""
    from mauer.vision.dataset import write_png
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    look = re.sub(r"[^A-Za-z0-9_+-]+", "_", str(view.look or "image")).strip("_") or "image"
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(view.t)) + f"_{int((view.t % 1.0) * 1000):03d}"
    stem, n = f"{stamp}_{look}", 1
    while (folder / f"{stem}_view.png").exists():
        n += 1
        stem = f"{stamp}_{look}_{n}"
    paths = [folder / f"{stem}_view.png"]
    write_png(paths[0], view.preview)
    full = getattr(view, "image", None)
    if full is not None:
        paths.append(folder / f"{stem}_full.png")
        write_png(paths[-1], full)
    meta = {"saved": time.time(), "t": view.t, "look": view.look, "parent": view.parent,
            "full_size_px": list(view.full_size), "preview_scale": view.scale, "shot": view.rec,
            "boards": [asdict(r) for r in getattr(view, "boards", ())], "detections_preview_px": view.detections,
            "note": view.note, "last_fit": dict(fit) if fit else None, "files": [p.name for p in paths]}
    paths.append(folder / f"{stem}.json")
    paths[-1].write_text(json.dumps(meta, indent=1, default=_jsonable), encoding="utf-8")
    return paths
