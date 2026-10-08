"""Camera view and overlay (docs/HMI_DESIGN.md section 11.1), offscreen with synthetic images: the ChArUco corners
drawn on the preview with the sequencer's board results, the display limit, Grab / Live only in REAL with a rig and
no run, the frame-fit panel from run-log records, the snapshot into the run log folder, and a SIM run through the
real RunController whose shots are processed in the run thread and arrive with detections."""
from __future__ import annotations

import json
import threading
import time
from dataclasses import replace

import numpy as np
import pytest

from hmi.core.overlay import (
    STATUS_BGR, CameraShot, OverlayProcessor, board_results, make_shot_view, redraw, shot_context, write_snapshot,
)
from hmi.core.run_controller import RunOptions
from hmi.views.camera_view import LOOP_OFF, SNAP_DIR, CameraView, fit_rows, measurement_banner
from hmi_fakes import FakeRunController, make_ctx, make_window, short_sim_session, wait_until
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


def test_fit_rows_and_banners():
    rec = {"event": "wall_frame", "source": "camera", "kind": "wall", "stop": 3, "why": "after route", "attempt": 1,
           "boards": ["W2", "W3"], "rms_mm": 0.123, "max_mm": 0.2, "baseline_mm": 600.0, "err_nominal_mm": 4.56,
           "err_nominal_deg": 0.31, "jump_mm": 12.3, "jump_deg": 0.5, "measured": {"x_mm": 10, "y_mm": -840,
                                                                                    "theta_rad": 0.01},
           "fit": {"residuals": {"W2": {"rms_mm": 0.1, "n_corners": 12}, "W3": {"rms_mm": None, "n_corners": 9}}}}
    rows = dict(fit_rows(rec))
    assert rows["Result"] == "ACCEPTED" and rows["Frame"] == "wall stop 3 (after route), attempt 1"
    assert rows["Fit RMS / max"] == "0.12 / 0.20 mm" and rows["Error vs nominal"] == "4.6 mm / 0.31 deg"
    assert rows["Jump vs prediction"] == "12.3 mm / 0.50 deg" and rows["Measured ARES"] == "(10, -840) mm +0.57 deg"
    assert rows["Per board"] == "W2 0.10 mm (12 c), W3 - mm (9 c)"
    assert dict(fit_rows({"event": "station_frame", "source": "dead_reckoning"}))["Result"].startswith("dead")
    assert measurement_banner({"event": "measurement_failed", "msg": "stop 0 wall: board(s) not measured"}) == \
        ("error", "Measurement REJECTED: stop 0 wall: board(s) not measured")
    lvl, text = measurement_banner({"event": "frame_jump", "what": "stop 2 wall", "d_mm": 61.0, "d_deg": 0.4,
                                    "measured": {"x_mm": 0, "y_mm": 0, "theta_rad": 0}, "predicted": None})
    assert lvl == "error" and text.startswith("Frame REJECTED (stop 2 wall)") and "61.0 mm" in text
    assert measurement_banner({"event": "coarse_aim", "board": "W1", "n_corners": 4})[0] == "warning"
    assert measurement_banner({"event": "board_search"})[0] == "warning"
    assert measurement_banner({"event": "shot"}) is None


# ── CameraView on a FakeRunController ─────────────────────────────────────────
@pytest.fixture
def cam(qapp, tmp_path, cfg):
    """(CameraView, FakeRunController, ctx): shown, display limit 2 Hz (wide timing margins)."""
    fake = FakeRunController(cfg["hmi"], ares_enabled=True)
    ctx = make_ctx(qapp, tmp_path, controller=fake, ares_enabled=True)
    ctx.hmi["camera_display_hz"] = 2.0
    v = CameraView(ctx)
    v.resize(1100, 700)
    v.show()
    qapp.processEvents()
    yield v, fake, ctx
    v._stop_live("test end")
    v.close()


@pytest.fixture(scope="module")
def shot_view(cfg, frame):
    return make_shot_view(frame[0], shot_rec(W0=W0_OK, W1=W1_MISSING), cfg, sctx(cfg))


def pause_events(qapp, s: float) -> None:
    end = time.monotonic() + s
    while time.monotonic() < end:
        qapp.processEvents()
        time.sleep(0.005)


def test_view_installs_the_overlay_processor(cam):
    v, fake, _ = cam
    proc = fake.of("set_shot_processor")[-1][0]
    assert isinstance(proc, OverlayProcessor) and proc is v._proc
    v.ids.setChecked(True)
    assert proc.show_ids


def test_shots_are_rate_limited_and_shown(cam, qapp, shot_view):
    v, fake, _ = cam
    for i in range(10):                                   # a burst: only the newest one is painted
        fake.shot.emit(replace(shot_view, look=f"L{i}"))
    assert wait_until(lambda: v.n_shown == 1, 2.0, qapp)
    assert v.n_shots == 10 and v._view.look == "L9" and v.image.source() is not None
    assert v.info["Look"].text() == "L9 (wall)" and "10 received, 1 shown" in v.info["Shots"].text()
    rows = [(v.table.item(i, 0).text(), v.table.item(i, 1).text()) for i in range(v.table.rowCount())]
    assert rows == [("W0", "accepted"), ("W1", "not seen")] and v.table.item(0, 3).text() == "0.03"
    fake.shot.emit(replace(shot_view, look="L10"))
    pause_events(qapp, 0.15)                              # limit 2 Hz: not before 0.5 s after the last paint
    assert v.n_shown == 1
    assert wait_until(lambda: v.n_shown == 2, 2.0, qapp) and v._view.look == "L10"


def test_hidden_view_paints_when_shown(cam, qapp, shot_view):
    v, fake, _ = cam
    v.hide()
    fake.shot.emit(shot_view)
    pause_events(qapp, 0.1)
    assert v.n_shots == 1 and v.n_shown == 0
    v.show()
    assert wait_until(lambda: v.n_shown == 1, 2.0, qapp)


def test_corner_id_switch_redraws_the_shown_image(cam, qapp, shot_view):
    v, fake, _ = cam
    fake.shot.emit(shot_view)
    assert wait_until(lambda: v.n_shown == 1, 2.0, qapp)
    before = v._view.preview
    v.ids.setChecked(True)
    assert v._view.ids_drawn and not np.array_equal(v._view.preview, before)


def test_grab_and_live_only_in_real_with_a_rig(cam, qapp, shot_view):
    v, fake, _ = cam
    fake.set_state("ready", mode="sim")
    assert not v.grab_btn.isEnabled() and not v.live.isEnabled()
    v.live.setChecked(True)                               # programmatic: refused, nothing grabbed
    assert not v.live.isChecked() and not v.live_active and fake.of("grab") == []
    fake.set_state("ready", mode="real")
    assert v.grab_btn.isEnabled() and v.live.isEnabled()
    v.live.setChecked(True)
    assert v.live_active and len(fake.of("grab")) == 1
    v._live_tick()                                        # the grab is still in flight: no second one
    assert len(fake.of("grab")) == 1
    fake.shot.emit(replace(shot_view, rec=None, look="live"))
    v._live_tick()
    assert len(fake.of("grab")) == 2
    fake.set_state("running")                             # a run starts: Live stops, Grab is locked
    assert not v.live.isChecked() and not v.live_active and not v.grab_btn.isEnabled()
    assert "Live stopped" in v.snap_status.text()
    fake.set_state("paused")
    v.live.setChecked(True)
    assert v.live_active
    fake.message.emit("error", "grab failed: CameraError: no frame within 3000 ms")
    assert not v.live_active and "grab failed" in v.snap_status.text()


def test_fit_panel_and_banner_from_records(cam):
    v, fake, _ = cam
    fit = {"event": "wall_frame", "source": "camera", "stop": 0, "why": "start", "attempt": 0, "boards": ["W0"],
           "rms_mm": 0.0, "max_mm": 0.0, "baseline_mm": 0.0, "err_nominal_mm": 2.1, "err_nominal_deg": 0.05,
           "jump_mm": 1.0, "jump_deg": 0.01, "measured": {"x_mm": 0.0, "y_mm": 0.0, "theta_rad": 0.0}}
    fake.event.emit(fit)
    assert v.fit["Result"].text() == "ACCEPTED" and v.fit["Error vs nominal"].text() == "2.1 mm / 0.05 deg"
    assert v.banner_text() == ""
    fake.event.emit({"event": "coarse_aim", "board": "W1", "n_corners": 4})
    assert v.banner_text().startswith("Coarse aim")
    fake.event.emit({"event": "measurement_failed", "msg": "stop 1 wall: board(s) not measured: W1"})
    assert v.fit["Result"].text() == "REJECTED" and v.banner_text().startswith("Measurement REJECTED")
    assert v.fit["Fits"].text().startswith("1 accepted, 1 rejected")
    fake.event.emit({"event": "frame_jump", "what": "stop 1 wall", "d_mm": 80.0, "d_deg": 1.0})
    assert v.banner_text().startswith("Frame REJECTED") and v.n_rejected == 2
    fake.event.emit(fit)
    assert v.banner_text() == "" and v.fit["Result"].text() == "ACCEPTED" and v.n_fits == 2
    fake.rig_changed.emit(object())                       # a new rig = a new run log
    assert v.n_fits == v.n_rejected == 0 and v.fit["Result"].text() == "-"


def test_dead_reckoning_note(cam):
    v, fake, _ = cam
    assert v.loop_note.isHidden()
    fake.opts = RunOptions("sim", camera_loop=False)
    fake.set_state("ready", mode="sim")
    assert not v.loop_note.isHidden() and v.loop_note.text() == LOOP_OFF


def test_snapshot_into_the_run_log_folder(cam, qapp, shot_view, tmp_path):
    v, fake, _ = cam
    fake.shot.emit(shot_view)                             # no run log folder (nothing prepared)
    assert wait_until(lambda: v.n_shown == 1, 2.0, qapp)
    assert not v.snap_btn.isEnabled()
    fake.log_dir = tmp_path / "runs" / "2026-10-08_120000_hmi_sim"
    fake.shot.emit(shot_view)
    assert wait_until(lambda: v.n_shown == 2, 3.0, qapp) and v.snap_btn.isEnabled()
    fake.event.emit({"event": "wall_frame", "source": "camera", "stop": 0, "rms_mm": 0.1})
    v.snap_btn.click()
    assert wait_until(lambda: v.snap_status.text().startswith("snapshot saved"), 10.0, qapp), v.snap_status.text()
    folder = fake.log_dir / SNAP_DIR
    names = sorted(p.name for p in folder.iterdir())
    assert len(names) == 3 and any(n.endswith("_view.png") for n in names) and any(n.endswith("_full.png")
                                                                                  for n in names)
    meta = json.loads(next(folder.glob("*.json")).read_text(encoding="utf-8"))
    assert meta["last_fit"]["rms_mm"] == 0.1 and meta["boards"][0]["board"] == "W0"
    assert read_image(next(folder.glob("*_view.png"))).shape == shot_view.preview.shape
    assert v.open_btn.isEnabled()


# ── a SIM run through the real RunController ──────────────────────────────────
def test_sim_run_shots_arrive_with_detections(qapp, tmp_path):
    w, fw, ctx = make_window(qapp, tmp_path)
    c = ctx.controller
    try:
        w.show()
        w.tabs.setCurrentWidget(w.camera)
        cam = w.camera
        threads: list[str] = []
        proc = cam._proc
        c.set_shot_processor(lambda img, rec, cfg: (threads.append(threading.current_thread().name),
                                                     proc(img, rec, cfg))[1])
        events, states = [], []
        c.event.connect(events.append)
        c.state_changed.connect(lambda s, d: states.append(s))
        c.set_session(short_sim_session())
        c.prepare(RunOptions("sim", scenario="none", sim_step_s=0.0))
        assert wait_until(lambda: states and states[-1] == "ready", 30.0, qapp), states
        assert cam.source.text() == "camera: SIM camera (synthetic render)" and not cam.grab_btn.isEnabled()
        c.start()
        assert wait_until(lambda: states and states[-1] == "done", 60.0, qapp), states
        n_shot = sum(e["event"] == "shot" for e in events)
        assert n_shot > 0 and wait_until(lambda: cam.n_shots == n_shot, 5.0, qapp)
        assert set(threads) == {"mauer-run"}                   # detection in the run thread only (D-H4, detect.py)
        assert wait_until(lambda: cam._pending is None and cam.n_shown >= 1, 5.0, qapp)
        view = cam._view
        assert isinstance(view, CameraShot) and view.rec is not None and view.detections
        assert any(r.status == "accepted" for r in view.boards)
        assert cam.fit["Result"].text() == "ACCEPTED" and cam.n_fits >= 1
        assert cam._view_dir == c.log_dir and cam.snap_btn.isEnabled()
    finally:
        if c.state in ("running", "pausing", "aborting"):
            c.halt()
            wait_until(lambda: c.state not in ("running", "pausing", "aborting"), 30.0, qapp)
        w.close()
