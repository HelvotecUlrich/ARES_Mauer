"""RoboDK digital twin (robodk/twin.py) in its OWN RoboDK instance on an API port >= 20630: frames fed by hand (pick,
place, joints), a SIM run of the config's first stop mirrored through the HMI's TwinLink with a picture of the twin
view (results/twin_test_start.png, results/twin_test.png; git-ignored), and the user closing RoboDK under a running
twin. The window stays minimised except for the pictures (a minimised RoboDK does not render).

Opt-in like tests/test_robodk_l.py (each test starts and closes a separate RoboDK; the user's RoboDK on 20500/20501
is never touched - every connect is checked):
    py.exe -m pytest -m robodk tests/test_twin_robodk.py -o addopts="" -q      (or MAUER_ROBODK=1)
"""
from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytestmark = [pytest.mark.robodk, pytest.mark.usefixtures("no_lab_network")]


@pytest.fixture(autouse=True, scope="module")    # module scope: skips before anything starts RoboDK
def _opt_in(request):
    """Opt-in: these start a separate RoboDK instance."""
    import os
    if "robodk" not in (request.config.getoption("markexpr") or "") and os.environ.get("MAUER_ROBODK") != "1":
        pytest.skip("RoboDK tests are opt-in: py.exe -m pytest -m robodk tests/test_twin_robodk.py")


REPO = Path(__file__).resolve().parent.parent
if not Path(r"C:\RoboDK\bin\RoboDK.exe").exists():
    pytest.skip("RoboDK not installed", allow_module_level=True)

from hmi.core.twin_link import twin_model  # noqa: E402  (puts robodk/ on sys.path)
from hmi_fakes import short_sim_session, wait_until  # noqa: E402

tm = twin_model()
import rdk_common as rc  # noqa: E402
import twin as rdk_twin  # noqa: E402

RESULTS = REPO / "results"
EXIT_OK = (0, 3221225477)        # 0xC0000005: RoboDK 6.0's known harmless crash on exit
START_S = 180.0                  # RoboDK start + build_station.build (~6 s measured; STEP import if a cache is stale)
PORTS: list[int] = []


def guarded_connect(*, new_instance: bool, port: int, minimized: bool = True):
    """rdk_common.connect, only for a NEW instance on a twin port (never the user's RoboDK)."""
    assert new_instance and port >= 20630 and port not in (20500, 20501), port
    PORTS.append(port)
    return rc.connect(new_instance=True, port=port, minimized=minimized)


def settings(cfg) -> "tm.TwinSettings":
    return replace(tm.TwinSettings.from_config(cfg), visible=False)


def stone_pixels(path: Path) -> int:
    """Pixels in the colour of a full stone (build_station.STONE, shaded): red clearly above green and blue."""
    import cv2
    img = cv2.imread(str(path)).astype(int)
    b, g, r = img[..., 0], img[..., 1], img[..., 2]
    return int(np.count_nonzero((r > 110) & (r - g > 55) & (r - b > 70)))


def until(pred, timeout_s: float) -> bool:
    """pred() true within timeout_s (no Qt event loop needed: the twin runs in its own thread)."""
    end = time.monotonic() + timeout_s
    while not pred():
        if time.monotonic() > end:
            return False
        time.sleep(0.02)
    return True


def caught_up(tw, stones) -> bool:
    return tw.scene is not None and tw.scene.shown == stones


def test_twin_mirrors_frames_and_closes_its_instance():
    s = short_sim_session()
    job = s.job
    s0 = tm.StoneState.initial(job)
    q0 = tuple(job.park_q_rad)
    t0 = job.stones()[0]
    mid = t0.slot or sorted(s0.magazine)[0]
    q1 = tuple(np.asarray(q0) + np.radians([20.0, 5.0, -5.0, 0.0, 10.0, 0.0]))
    frame = [tm.TwinFrame(q0, job.stops[0].ares, "test", np.asarray(job.station.T_wall_station), s0, "initial")]
    states: list = []
    tw = rdk_twin.Twin(s.cfg, job, lambda: frame[0], settings(s.cfg), lambda st_, d: states.append(st_),
                       connect=guarded_connect)
    tw.start()
    try:
        assert until(lambda: caught_up(tw, s0) or tw.state == "lost", START_S), tw.stats()
        assert tw.state == "running" and tw.port >= 20630
        assert tw.scene.stone_count() == s0.counts()
        picked = tm.StoneState({k: v for k, v in s0.magazine.items() if k != mid}, s0.station, frozenset(),
                               ("magazine", mid, t0.kind))
        frame[0] = replace(frame[0], q_rad=q1, stones=picked, caption="picked")
        assert until(lambda: caught_up(tw, picked), 10.0)
        assert tw.scene.stone_count()["tool"] == 1
        joints = tw.scene.it["robot"].Joints().list()        # Robolink serialises calls with its lock
        assert np.allclose(joints, np.degrees(q1), atol=1e-3)
        placed = replace(picked, placed=frozenset({tuple(t0.key)}), held=None)
        frame[0] = replace(frame[0], stones=placed, caption="placed")
        assert until(lambda: caught_up(tw, placed), 10.0)
        n = tw.scene.stone_count()
        assert n["wall"] == 1 and n["tool"] == 0 and n["mag"] == len(s0.magazine) - 1
        assert tw.renders >= 3 and tw.tick_ms > 0.0 and tw.rate_hz > 0.0
    finally:
        assert tw.stop(60.0)
    assert states[0] == "starting" and "running" in states and states[-1] == "stopped"
    assert tw.proc is not None and tw.proc.poll() is not None and tw.exit_code in EXIT_OK
    assert all(p >= 20630 for p in PORTS)


def test_sim_run_of_the_first_stop_mirrored_with_a_picture(qapp, tmp_path):
    """The config's job (the C), stop 0 in SIM (24 stones, one station trip) through RunController + TwinLink; the
    twin catches up with the final state and the picture shows more stone pixels than before the run."""
    from hmi.core.run_controller import RunController, RunOptions
    from hmi.core.session import build_from_config
    from hmi.core.twin_link import TwinLink
    from hmi_fakes import make_ctx
    from mauer import config

    twins: list = []

    def factory(cfg, job, frame_fn, sett, on_status):
        tw = rdk_twin.Twin(cfg, job, frame_fn, replace(sett, visible=False), on_status, connect=guarded_connect)
        twins.append(tw)
        return tw
    ctl = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs")
    ctx = make_ctx(qapp, tmp_path, controller=ctl)
    link = TwinLink(ctx, twin_factory=factory)
    states: list = []
    ctl.state_changed.connect(lambda st_, d: states.append(st_))
    try:
        session = build_from_config()
        ctl.set_session(session)
        assert link.start()
        tw = twins[0]
        s0 = tm.StoneState.initial(session.job)
        assert wait_until(lambda: caught_up(tw, s0) or tw.state == "lost", START_S, qapp), tw.stats()
        assert tw.state == "running" and ctx.twin_state[0] == "running"
        RESULTS.mkdir(exist_ok=True)
        shots: dict = {}
        tw.request_snapshot(RESULTS / "twin_test_start.png", show=True, done=lambda ok, p: shots.update(start=ok))
        assert wait_until(lambda: "start" in shots, 30.0, qapp) and shots["start"]

        ctl.prepare(RunOptions("sim", scenario="none", sim_step_s=0.02, stop_after=0))
        assert wait_until(lambda: states and states[-1] == "ready", 60.0, qapp), (ctl.state, states)
        ctl.start()
        assert wait_until(lambda: states and states[-1] in ("done", "error", "aborted", "paused"), 180.0, qapp)
        assert states[-1] == "done", ctl.snapshot.error
        final = tm.StoneState.from_snapshot(ctl.snapshot)
        assert len(final.placed) == len(session.job.stops[0].stones) and ctl.snapshot.reloads >= 1
        assert wait_until(lambda: caught_up(tw, final), 10.0, qapp), (tw.stats(), tw.scene.stone_count())
        n = tw.scene.stone_count()
        assert n == final.counts() and n["tool"] == 0 and not tw.scene.skipped
        assert tw.state == "running" and tw.frame_errors == 0, tw.last_error

        tw.request_snapshot(RESULTS / "twin_test.png", show=True, done=lambda ok, p: shots.update(end=ok))
        assert wait_until(lambda: "end" in shots, 30.0, qapp) and shots["end"]
        before, after = stone_pixels(RESULTS / "twin_test_start.png"), stone_pixels(RESULTS / "twin_test.png")
        print(f"twin: {tw.stats()}; stones on screen {n}; stone pixels {before} -> {after}")
        assert after > before + 2000, (before, after)
    finally:
        link.shutdown()
        assert ctl.shutdown(15.0)
    for tw in twins:
        assert not tw.alive and tw.proc is not None and tw.proc.poll() is not None and tw.exit_code in EXIT_OK


def test_twin_survives_robodk_being_closed():
    s = short_sim_session()
    s0 = tm.StoneState.initial(s.job)
    frame = tm.TwinFrame(tuple(s.job.park_q_rad), s.job.stops[0].ares, "test", None, s0)
    states: list = []
    tw = rdk_twin.Twin(s.cfg, s.job, lambda: frame, settings(s.cfg), lambda st_, d: states.append((st_, d)),
                       connect=guarded_connect)
    tw.start()
    try:
        assert until(lambda: caught_up(tw, s0) or tw.state == "lost", START_S), tw.stats()
        tw.proc.kill()                                  # the user closes RoboDK (or it crashes)
        assert until(lambda: not tw.alive, 30.0)
        assert tw.state == "lost" and states[-1] == ("lost", "RoboDK was closed")
    finally:
        assert tw.stop(30.0)

