"""Magazine dry run (mauer/magtest.py, tools/make_magtest.py) in the Mauer HMI: the RunController runs a magtest job
with mauer.magtest.MagazineTest; REAL opens the UR only (no AresAds, no camera, no calibration - ARES is not moved and
nothing is measured), needs no --ares, has its own preflight and start checklist. Fakes only (no_lab_network)."""
from __future__ import annotations

import copy
import dataclasses
import importlib.util

import numpy as np
import pytest

from hmi.core.rigs import MOTIONS, RealFactories
from hmi.core.run_controller import RunController, RunOptions, enables
from hmi_fakes import StubLink, wait_until
from mauer import REPO, config
from mauer import job as mjob
from mauer.magtest import MagazineTest

pytestmark = pytest.mark.usefixtures("no_lab_network")


def _tool(name):
    spec = importlib.util.spec_from_file_location(f"tool_{name}", REPO / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def mt_path(tmp_path_factory):
    job = _tool("make_magtest").build(config.load(), rounds=1)
    return mjob.save(job, tmp_path_factory.mktemp("jobs") / "magtest.json")


@pytest.fixture
def mt_session(mt_path):
    from hmi.core.session import load_job_file
    return load_job_file(mt_path)


def test_the_dry_place_is_a_motion_behind_halt_and_real_needs_no_ares():
    assert "dry_place" in MOTIONS
    assert enables("loaded", "real", ares_enabled=False, magtest=True)["prepare"] == (True, "")
    assert not enables("loaded", "real", ares_enabled=False)["prepare"][0]          # a wall job still needs --ares


def test_sim_magazine_dry_run_through_the_controller(qapp, tmp_path, mt_session):
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs")
    snaps = []
    c.snapshot_changed.connect(snaps.append)
    try:
        c.set_session(mt_session)
        c.prepare(RunOptions("sim", scenario="none", sim_step_s=0.0))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp), c.state
        assert isinstance(c.sequencer, MagazineTest)
        assert c.preflight.mode == "sim" and any("payload_cog_mm" in i.text for i in c.preflight.items)
        n = len(mt_session.job.meta["magtest"]["moves"])
        c.start()
        assert wait_until(lambda: c.state == "done", 180.0, qapp), (c.state, c.sequencer.result.error)
        w = c.rig.world
        assert len(c.sequencer.placed) == n and w.robot.calls["dry_place"] == n and w.robot.calls["place_wall"] == 0
        assert set(w.mag_stones) == set(mt_session.job.meta["magtest"]["start_fill"])
        assert wait_until(lambda: snaps and snaps[-1].n_placed == n == snaps[-1].n_stones, 5.0, qapp)
    finally:
        assert c.shutdown(15.0)


def test_real_magazine_dry_run_opens_the_ur_only(qapp, tmp_path, mt_session):
    calls: list = []
    f = RealFactories(link=lambda cfg: StubLink(order=calls), ads=lambda cfg: calls.append("ads"),
                      camera=lambda cfg: calls.append("camera"), intrinsics=lambda cfg: calls.append("intrinsics"),
                      handeye=lambda cfg: calls.append("handeye"))
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs", ares_enabled=False, real_factories=f)
    asks, reports = [], []
    c.confirm_requested.connect(asks.append)
    c.preflight_done.connect(reports.append)
    try:
        c.set_session(mt_session)
        assert c.can("prepare", "real") == (True, "")                     # no --ares: ARES is not moved
        c.prepare(RunOptions("real", step=True))
        assert wait_until(lambda: c.state == "ready" and reports, 30.0, qapp), c.state
        assert not any(x in calls for x in ("ads", "camera", "intrinsics", "handeye")), calls
        assert c.rig.robot is not None and c.rig.ads is None and c.rig.camera_kind is None
        assert isinstance(c.sequencer, MagazineTest)
        rep = reports[-1]
        assert rep.mode == "real" and rep.ok, [i.text for i in rep.items if i.blocking]
        assert any(not i.blocking and "payload_cog_mm" in i.text for i in rep.items)
        assert c.can("start") == (True, "")
        text = c.start_checklist()
        start = mt_session.job.meta["magtest"]["start_fill"]
        assert "MAGAZINE DRY RUN" in text and all(s in text for s in start) and "jaws EMPTY" in text
        assert "X growing moves the TCP RIGHT, Y growing FORWARD" in text   # the pendant check (driving direction)
        assert "beyond the UR end of ARES (the vehicle REAR)" in text
        c.start()
        assert wait_until(lambda: asks, 10.0, qapp) and asks[-1].kind == "start"
        c.answer_confirm(asks[-1].id, False)                               # declined: nothing moves
        assert wait_until(lambda: c.state == "ready", 10.0, qapp)
        c.release()
        assert wait_until(lambda: c.state == "loaded", 10.0, qapp)
        assert "link.stop" in calls
    finally:
        assert c.shutdown(15.0)


def test_a_wall_job_still_opens_the_full_rig():
    """The UR-only rig is for magtest jobs: RealRig of any other job keeps link + ADS + camera + calibration."""
    from hmi.core.rigs import RealRig
    job = mjob.Job([], mjob.Magazine([], [], [], []), None, np.eye(4), np.eye(4), [0.0] * 6, 150.0)
    assert not RealRig({}, job).ur_only


def test_mauer_tab_offers_real_without_ares_and_names_the_dry_run(qapp, tmp_path, mt_session):
    """The Mauer tab of an HMI started without --ares: REAL selectable for a magtest job (not for a wall job), the
    summary names the moves, the start slots, the hover and the hold time."""
    from hmi_fakes import make_window
    w, fw, ctx = make_window(qapp, tmp_path)
    try:
        c, tab = ctx.controller, w.mauer
        c.set_session(mt_session)
        qapp.processEvents()
        m = mt_session.job.meta["magtest"]
        assert tab.real_rb.isEnabled()
        text = tab.summary.text()
        assert f"magazine dry run: {len(m['moves'])} moves" in text and m["start_fill"][0] in text
        assert "hover 50 mm" in text and "hold 2 s" in text
        tab.real_rb.setChecked(True)
        qapp.processEvents()
        assert tab.prepare_btn.isEnabled()
        tab.sim_rb.setChecked(True)
        plain = copy.deepcopy(mt_session.job)
        plain.meta.pop("kind")                                               # any other job: a wall job
        c.set_session(dataclasses.replace(mt_session, job=plain, name="plain"))
        qapp.processEvents()
        assert not tab.real_rb.isEnabled()                                   # a wall job: REAL needs --ares
    finally:
        w.close()


def test_plan_view_draws_ares_and_the_front_poses_but_no_station(mt_session):
    """The dry run has no pick-up station (no slots, no boards): no table at the origin over ARES."""
    from hmi.views.plan_view import plan_geometry
    g = plan_geometry(mt_session.job, mt_session.cfg)
    assert g.table is None and not g.notes
    assert len(g.stones) == len(mt_session.job.meta["magtest"]["moves"])
    assert g.bounds[1] > mt_session.job.meta["magtest"]["front"]["dist_mm"]


def test_sim_robot_error_mid_move_jaws_empty_and_resume_repeat_the_move(qapp, tmp_path, mt_session):
    """A robot error at the front: Resume is refused with the slot to put the stone back on; 'Jaws empty' puts it
    back there (SIM: the simulated operator), Resume parks, repeats the move and finishes."""
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs")
    msgs = []
    c.message.connect(lambda lvl, t: msgs.append(t))
    try:
        c.set_session(mt_session)
        c.prepare(RunOptions("sim", scenario="none", sim_step_s=0.0))
        assert wait_until(lambda: c.state == "ready", 30.0, qapp), c.state
        w = c._rig.world
        w.robot.fail_on = {"dry_place": 2}                                   # protective stop in move 2
        c.start()
        assert wait_until(lambda: c.state == "error", 60.0, qapp), c.state
        src = mt_session.job.meta["magtest"]["moves"][1]["from"]
        c.resume()
        assert wait_until(lambda: any("resume refused" in t and src in t and "put it back" in t for t in msgs),
                          10.0, qapp), msgs
        assert c.can("clear_held") == (True, "")
        c.clear_held()
        assert wait_until(lambda: c.sequencer.held is None, 10.0, qapp)
        assert src in w.mag_stones and w.robot.holding is None
        assert c.step                                                        # after an error the resume is stepped
        c.set_step(False)                                                    # (the operator switches it off)
        c.resume()
        assert wait_until(lambda: c.state == "done", 180.0, qapp), (c.state, c.sequencer.result.error)
        assert set(w.mag_stones) == set(mt_session.job.meta["magtest"]["start_fill"]) and w.violations == []
    finally:
        assert c.shutdown(15.0)
