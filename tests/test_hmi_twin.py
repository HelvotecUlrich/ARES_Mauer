"""Twin tab and TwinLink without RoboDK (a fake twin in place of robodk/twin.py Twin): the frames in SIM and REAL
(hmi/core/twin_link.frame_from), switching on and off, restart with every new job session, --twin, status into the
HMI context, the shutdown hook - and a real SIM run mirrored by a fake twin thread that applies every diff like
TwinScene (no stone lost or doubled, whatever states the polling skips)."""
from __future__ import annotations

import math
import threading
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from hmi.core.run_controller import RunController, RunOptions
from hmi.core.snapshot import RunSnapshot
from hmi.core.sources import AresLive, UrSnapshot, _none_snapshot
from hmi.core.twin_link import frame_from, twin_model
from hmi.views.twin_panel import TwinPanel
from hmi_fakes import FakeRunController, make_ctx, short_sim_session, wait_until
from mauer import config
from mauer.reference import Pose2D

pytestmark = pytest.mark.usefixtures("no_lab_network")

tm = twin_model()


class FakeTwin:
    """robodk/twin.py Twin stand-in. poll_s: a thread that calls frame_fn and applies the diffs like TwinScene."""

    instances: list = []

    def __init__(self, cfg, job, frame_fn, settings, on_status, poll_s: float | None = None) -> None:
        self.cfg, self.job, self.frame_fn, self.settings, self.on_status = cfg, job, frame_fn, settings, on_status
        self.poll_s = poll_s
        self.started = self.stopped = 0
        self.scene = None
        self.frames: list = []
        self.objects: dict = {}
        self.shown = tm.StoneState({}, {}, frozenset(), None)
        self.bad: list = []
        self.snapshots: list = []
        self._stop = threading.Event()
        self._thread = None
        FakeTwin.instances.append(self)

    def start(self) -> None:
        self.started += 1
        self.on_status("starting", "fake")
        if self.poll_s is not None:
            self._thread = threading.Thread(target=self._loop, name="fake-twin", daemon=True)
            self._thread.start()
        self.on_status("running", "fake")

    def _loop(self) -> None:
        kinds = tm.wall_kinds(self.job)
        while not self._stop.wait(self.poll_s):
            f = self.frame_fn()
            if f is None:
                continue
            self.frames.append(f)
            ops = tm.plan_ops(self.shown, f.stones, kinds)
            for op in ops:
                if op.kind in ("move", "remove") and op.src not in self.objects:
                    self.bad.append(op)
            self.objects = tm.apply_ops(self.objects, ops, make=lambda op: object())
            self.shown = f.stones

    def stop(self, timeout_s: float = 15.0) -> bool:
        self.stopped += 1
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout_s)
        self.on_status("stopped", "fake")
        return True

    def stats(self) -> dict:
        return {"state": "running", "port": 20630, "rate_hz": 10.0, "tick_ms": 11.0, "lag_ms": 15.0, "ticks": 5,
                "renders": 3, "frame_errors": 0, "last_error": None}

    def request_snapshot(self, path, eye=None, target=None, show=False, done=lambda ok, p: None) -> None:
        self.snapshots.append(path)
        done(True, str(path))


@pytest.fixture(autouse=True)
def _reset():
    FakeTwin.instances = []
    yield


def panel(qapp, tmp_path, ctl=None, start_twin=False, poll_s=None):
    ctl = ctl if ctl is not None else FakeRunController(config.load()["hmi"])
    ctx = make_ctx(qapp, tmp_path, controller=ctl)
    ctx.start_twin = start_twin
    p = TwinPanel(ctx, twin_factory=lambda *a: FakeTwin(*a, poll_s=poll_s))
    return p, ctx, ctl


def test_switch_on_waits_for_a_job_then_follows_the_session(qapp, tmp_path):
    p, ctx, ctl = panel(qapp, tmp_path)
    assert not p.restart_btn.isEnabled() and not p.save_btn.isEnabled()
    p.enable.setChecked(True)
    assert FakeTwin.instances == [] and "starts when a job is loaded" in p.values["state"].text()
    ctl.set_session(short_sim_session())
    assert wait_until(lambda: ctx.twin_state[0] == "running", 2.0, qapp)
    tw = FakeTwin.instances[0]
    assert tw.started == 1 and tw.job is short_sim_session().job and tw.settings.port >= 20630
    f = tw.frame_fn()                                  # the frame source: the initial state of the job at park
    assert f.stones == tm.StoneState.initial(tw.job) and f.q_rad == tuple(tw.job.park_q_rad)
    p.refresh()
    assert p.values["port"].text() == "20630" and p.save_btn.isEnabled()
    ctl.set_session(short_sim_session(2, 0))          # a new session: the twin is rebuilt for it
    assert wait_until(lambda: len(FakeTwin.instances) == 2 and FakeTwin.instances[1].started == 1, 5.0, qapp)
    assert tw.stopped == 1 and tw.frame_fn() is None   # the old twin gets no frames of the new job
    assert wait_until(lambda: ctx.twin_state[0] == "running", 2.0, qapp)
    p.enable.setChecked(False)
    assert wait_until(lambda: FakeTwin.instances[1].stopped == 1, 5.0, qapp)
    assert ctx.twin_state[0] == "off" and not p.link.running


def test_twin_flag_starts_with_the_first_session_and_close_stops_it(qapp, tmp_path):
    p, ctx, ctl = panel(qapp, tmp_path, start_twin=True)
    assert p.enable.isChecked() and FakeTwin.instances == []
    ctl.set_session(short_sim_session())
    assert wait_until(lambda: FakeTwin.instances and FakeTwin.instances[0].started == 1, 2.0, qapp)
    ctx.run_shutdown_hooks()                           # MainWindow.closeEvent
    assert FakeTwin.instances[0].stopped == 1          # synchronous: RoboDK is closed before the HMI exits
    assert not p.link.start()                          # no restart after the shutdown


def test_save_view_goes_to_the_results_folder_without_a_run(qapp, tmp_path):
    p, ctx, ctl = panel(qapp, tmp_path)
    ctl.set_session(short_sim_session())
    p.enable.setChecked(True)
    assert wait_until(lambda: ctx.twin_state[0] == "running", 2.0, qapp)
    p.save_view()
    assert wait_until(lambda: "view saved" in p.message.text(), 2.0, qapp)
    path = FakeTwin.instances[0].snapshots[0]
    assert path.parent.name == "results" and path.name.startswith("twin_") and path.suffix == ".png"


def test_bad_twin_settings_are_reported_not_raised(qapp, tmp_path):
    import copy

    from hmi.core.session import JobSession
    s = short_sim_session()
    cfg = copy.deepcopy(s.cfg)
    cfg["hmi"]["twin"]["port"] = 20500
    p, ctx, ctl = panel(qapp, tmp_path)
    ctl.set_session(JobSession(cfg, None, s.job, None, "test", "bad_port"))
    p.enable.setChecked(True)
    assert wait_until(lambda: ctx.twin_state[0] == "lost", 2.0, qapp) and "[hmi.twin]" in ctx.twin_state[1]
    assert FakeTwin.instances == []


def test_main_window_has_the_twin_tab_and_its_shutdown_hook(qapp, tmp_path):
    from hmi_fakes import make_window
    w, _fw, ctx = make_window(qapp, tmp_path, controller=FakeRunController(config.load()["hmi"]))
    assert isinstance(w.twin, TwinPanel) and not w.twin.enable.isChecked()
    assert w.twin.link.shutdown in ctx._hooks
    w.close()


def test_sim_run_mirrored_without_losing_or_doubling_a_stone(qapp, tmp_path):
    """A real SIM run (short straight job with a station trip: 6 + 3 stones, 4 in the magazine at the start) polled
    at 100 Hz: every diff is applied like TwinScene, every move / remove finds its object, the end state matches."""
    from hmi_fakes import short_job, straight_job10
    from hmi.core.session import JobSession
    cfg, job10 = straight_job10()
    session = JobSession(cfg, None, short_job(job10, n0=6, n1=3, fill=4), None, "test", "short_trip")
    ctl = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs")
    p, ctx, _ = panel(qapp, tmp_path, ctl=ctl, poll_s=0.005)
    try:
        states = []
        ctl.state_changed.connect(lambda s, d: states.append(s))
        ctl.set_session(session)
        p.enable.setChecked(True)
        ctl.prepare(RunOptions("sim", scenario="none", sim_step_s=0.02))
        assert wait_until(lambda: states and states[-1] == "ready", 30.0, qapp), ctl.state
        ctl.start()
        assert wait_until(lambda: states and states[-1] == "done", 60.0, qapp), ctl.state
        tw = FakeTwin.instances[-1]
        final = tm.StoneState.from_snapshot(ctl.snapshot)
        assert wait_until(lambda: tw.shown == final, 5.0, qapp)
        assert tw.bad == []
        counts = {"mag": 0, "station": 0, "wall": 0, "tool": 0}
        for where, _ in tw.objects:
            counts[where] += 1
        assert counts == final.counts() and counts["wall"] == 9 and counts["tool"] == 0
        assert any(f.stones.held is not None for f in tw.frames)               # a stone in the jaws was seen
        assert any(f.ares_label == "sim truth" for f in tw.frames)
        assert ctl.snapshot.reloads >= 1
        mid = [f for f in tw.frames if f.stones.held is not None][0]
        assert mid.q_rad is not None and len(mid.q_rad) == 6
    finally:
        ctx.run_shutdown_hooks()
        assert ctl.shutdown(15.0)


# ── frames (hmi/core/twin_link.frame_from) ────────────────────────────────────
def ur_snap(source: str, q, age_s=None, t=1000.0) -> UrSnapshot:
    return _none_snapshot(source, q, t=t, age_s=age_s)


def test_frame_before_a_run_is_the_initial_state_at_park():
    s = short_sim_session()
    f = frame_from(None, None, None, s)
    assert f.q_rad == tuple(s.job.park_q_rad)
    assert f.ares == s.job.stops[0].ares and f.stones == tm.StoneState.initial(s.job)
    assert np.allclose(f.T_wall_station, s.job.station.T_wall_station) and "not started" in f.caption


def test_frame_in_sim_shows_the_true_pose_and_station():
    s = short_sim_session()
    snap = RunSnapshot.initial(s)
    true = Pose2D(10.0, -5.0, math.radians(1.0))

    T_true = np.array(s.job.station.T_wall_station, float)
    T_true[0, 3] += 7.0
    live = AresLive(snap.pose_est, "estimate", "ok", true, None)
    q = [0.1, -1.5, 1.4, -1.5, -1.6, 0.0]
    f = frame_from(snap, ur_snap("sim", q), live, s, SimpleNamespace(T_wall_station_true=T_true))
    assert f.ares == true and f.ares_label == "sim truth" and f.q_rad == tuple(q)
    assert f.T_wall_station[0, 3] == pytest.approx(s.job.station.T_wall_station[0][3] + 7.0)


def test_frame_in_real_shows_the_estimate_and_the_rtde_sample_time():
    s = short_sim_session()
    snap = RunSnapshot.initial(s)
    est = Pose2D(100.0, 20.0, 0.0)
    live = AresLive(est, "estimate+odometry", "odometry", None, None)
    held = {"from": "magazine", "slot": "m1", "kind": "full", "stone": [0, 0], "unknown": False}
    snap = replace(snap, held=held, placed=frozenset({(0, 1)}))
    f = frame_from(snap, ur_snap("rtde", [0.0] * 6, age_s=0.25, t=1000.0), live, s)
    assert f.ares == est and f.ares_label == "estimate+odometry (odometry)"
    assert f.t == pytest.approx(999.75)
    assert f.stones.held == ("magazine", "m1", "full") and f.stones.placed == frozenset({(0, 1)})
    assert np.allclose(f.T_wall_station, snap.T_wall_station) and "jaws: full stone from magazine" in f.caption
