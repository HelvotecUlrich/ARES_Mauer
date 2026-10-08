"""Camera overlay (docs/HMI_DESIGN.md section 11.1) on synthetic images: the ChArUco corners drawn on the preview
with the sequencer's board results, manual grabs, the corner-id redraw and the snapshot files."""
from __future__ import annotations

import json
import time

import numpy as np
import pytest

from hmi.core.overlay import (
    STATUS_BGR, CameraShot, OverlayProcessor, board_results, make_shot_view, redraw, shot_context, write_snapshot,
)
from mauer import config
from mauer.vision import intrinsics, synth, targets
from mauer.vision.dataset import read_image

pytestmark = pytest.mark.usefixtures("no_lab_network")

SCALE = 0.25                     # [hmi] camera_preview_scale


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def frame(cfg):
    """A full-size synthetic image (2472 x 2064) of board W0 at 320 mm, tilted, and its true corner positions [px]
    (index = ChArUco corner id)."""
    sp = targets.board_specs(cfg)["W0"]
    intr = intrinsics.nominal(cfg)
    T = synth.view(sp, 320.0, 15, 30, 10, offset_mm=(10, -5))
    img = synth.render_board(sp, intr, T, seed=1, blur_sigma_px=0.8, noise_sigma=1.5)
    return img, synth.project(sp, intr, T)


def shot_rec(look: str = "L0", **boards) -> dict:
    """A 'shot' record as the sequencer writes it (the boards of the look with ok / n_corners / rms_px / reason)."""
    return {"t": time.time(), "event": "shot", "parent": "wall", "look": look, "image": None, "boards": boards}


W0_OK = {"ok": True, "n_corners": 12, "rms_px": 0.031}
W1_MISSING = {"ok": False, "reason": "not detected", "n_corners": 0}


def sctx(cfg, show_ids=False):
    return shot_context(cfg, cfg["hmi"], show_ids)


def colour_at(img: np.ndarray, xy) -> np.ndarray:
    x, y = (int(round(v)) for v in xy)
    return img[y, x].astype(int)


# ── overlay (pure) ────────────────────────────────────────────────────────────
def test_shot_view_draws_the_measured_corners(cfg, frame):
    img, truth = frame
    v = make_shot_view(img, shot_rec(W0=W0_OK, W1=W1_MISSING), cfg, sctx(cfg))
    h, w = img.shape
    assert isinstance(v, CameraShot) and v.full_size == (w, h) and v.scale == SCALE
    assert v.preview.shape == (round(SCALE * h), round(SCALE * w), 3) and v.preview.dtype == np.uint8
    assert v.base.shape == v.preview.shape[:2] and v.image is not None and v.image.shape == img.shape
    assert v.look == "L0" and v.parent == "wall" and v.rec["boards"]["W0"]["ok"] and v.note == ""
    # only the record's boards are detected; W1 is not in view
    assert set(v.detections) == {"W0"}
    d = v.detections["W0"]
    assert d["n"] == 12 == len(d["ids"]) == len(d["pts"])
    exp = (truth[d["ids"]] + 0.5) * np.array([v.preview.shape[1] / w, v.preview.shape[0] / h]) - 0.5
    assert np.abs(np.asarray(d["pts"]) - exp).max() < 0.1        # preview px
    assert [(r.board, r.status) for r in v.boards] == [("W0", "accepted"), ("W1", "not seen")]
    assert v.boards[0].rms_px == pytest.approx(0.031) and v.boards[1].reason == "not detected"
    # corners drawn in the colour of the result
    assert np.abs(colour_at(v.preview, d["pts"][5]) - STATUS_BGR["accepted"]).max() < 40
    assert v.header.startswith("look L0 (wall)") and 0 < v.detect_ms < 2000


def test_rejected_board_is_drawn_as_rejected(cfg, frame):
    img, _ = frame
    v = make_shot_view(img, shot_rec(W0={"ok": False, "reason": "reprojection RMS 1.40 px > 1 px",
                                         "n_corners": 12}), cfg, sctx(cfg))
    r = v.boards[0]
    assert (r.board, r.status, r.n_corners) == ("W0", "rejected", 12) and "RMS" in r.reason
    assert "rejected" in r.text()
    assert np.abs(colour_at(v.preview, v.detections["W0"]["pts"][5]) - STATUS_BGR["rejected"]).max() < 40


def test_manual_grab_looks_for_every_board_and_redraws_ids(cfg, frame):
    img, _ = frame
    v = make_shot_view(img, None, cfg, sctx(cfg))
    assert v.rec is None and v.look == "live" and v.header.startswith("manual grab")
    assert [(r.board, r.status, r.n_corners) for r in v.boards] == [("W0", "seen", 12)]
    with_ids = redraw(v, True)
    assert with_ids.ids_drawn and int((with_ids.preview != v.preview).any(axis=2).sum()) > 50
    assert redraw(with_ids, True) is with_ids
    assert np.array_equal(redraw(with_ids, False).preview, v.preview)      # the same drawing without the ids
    drawn = make_shot_view(img, None, cfg, sctx(cfg, show_ids=True))
    assert drawn.ids_drawn and np.array_equal(drawn.base, v.base)


def test_board_results_of_a_manual_grab():
    dets = {"W2": {"n": 0, "markers": 2}, "W1": {"n": 3, "markers": 3}, "S0": {"n": 12, "markers": 6}}
    res = board_results(None, dets, min_corners=8)
    assert [(r.board, r.status) for r in res] == [("S0", "seen"), ("W1", "partial"), ("W2", "partial")]
    assert "3 corners < 8" in res[1].reason and "2 markers" in res[2].reason


def test_sixteen_bit_image_and_processor_cache(cfg, frame):
    img, _ = frame
    proc = OverlayProcessor(cfg["hmi"])
    assert proc.context(cfg) is proc.context(cfg)
    assert proc.context(dict(cfg)) is not proc.context(cfg)            # another config object, another context
    v = proc((img.astype(np.uint16) * 16), shot_rec(W0=W0_OK), cfg)      # unpacked Mono12
    assert v.image.dtype == np.uint8 and v.detections["W0"]["n"] == 12
    proc.show_ids = True
    assert proc(img, shot_rec(W0=W0_OK), cfg).ids_drawn


def test_write_snapshot(cfg, frame, tmp_path):
    img, _ = frame
    v = make_shot_view(img, shot_rec("L1 retry", W0=W0_OK), cfg, sctx(cfg))
    fit = {"event": "wall_frame", "source": "camera", "rms_mm": 0.12}
    paths = write_snapshot(tmp_path / "snaps", v, fit)
    assert paths[0].name.endswith("_L1_retry_view.png") and paths[1].name.endswith("_L1_retry_full.png")
    assert paths[2].name.endswith("_L1_retry.json")
    assert read_image(paths[0]).shape == v.preview.shape
    assert np.array_equal(read_image(paths[1]), img)
    meta = json.loads(paths[2].read_text(encoding="utf-8"))
    assert meta["look"] == "L1 retry" and meta["shot"]["boards"]["W0"]["ok"] and meta["last_fit"] == fit
    assert meta["boards"][0]["status"] == "accepted" and meta["detections_preview_px"]["W0"]["n"] == 12
    again = write_snapshot(tmp_path / "snaps", v)                        # same image again: a new name
    assert again[0] != paths[0] and again[0].exists()
