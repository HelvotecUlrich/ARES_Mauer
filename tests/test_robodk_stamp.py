"""robodk/simulate.py robodk_stamp (2026-10-07): only a complete, clean RoboDK run marks the job RoboDK-verified
(meta reach_check "robodk"), which mauer.sequencer.preflight_real requires for the real robot."""
import sys
from types import SimpleNamespace

import pytest

from mauer import REPO, config
from mauer import job as mjob

sys.path.insert(0, str(REPO / "robodk"))
sys.path.insert(0, str(REPO / "tools"))
simulate = pytest.importorskip("simulate")
import make_job  # noqa: E402


def _run(job, ok=True):
    stones = [{"ok": True, "label": t.label} for t in job.stones()]
    if not ok:
        stones[3]["ok"] = False
    looks = [{"stop": s.index, "state": "arrival", "planned": [lk.boards[0] for lk in s.looks],
              "res": {lk.boards[0]: {"ok": True, "source": "job"} for lk in s.looks}} for s in job.stops]
    return SimpleNamespace(stone_log=stones, transfer_log=[{"ok": True}], route_log=[{"hits": []}], look_log=looks,
                           park_q=[0.0] * 6, notes=[])


def test_only_a_clean_complete_run_stamps_the_job(tmp_path):
    from mauer.sequencer import preflight_real
    cfg = config.load()
    job = make_job.build_nominal(cfg)
    assert any("reach check 'kinematic' only" in p for p in preflight_real(cfg, job))
    args = SimpleNamespace(from_stop=0, max_stones=0, trips=True, report=str(REPO / "results" / "l_wall_sim.md"))
    info = {"robodk": "6.0.0"}
    assert simulate.robodk_stamp(_run(job, ok=False), job, args, info, cfg, out_dir=tmp_path) is None
    partial = SimpleNamespace(**{**vars(args), "max_stones": 5})
    assert simulate.robodk_stamp(_run(job), job, partial, info, cfg, out_dir=tmp_path) is None
    path = simulate.robodk_stamp(_run(job), job, args, info, cfg, out_dir=tmp_path)
    assert path is not None and path.name == "nominal_C_robodk.json"
    stamped = mjob.load(path)
    assert stamped.meta["reach_check"] == "robodk" and stamped.meta["robodk_sim"]["stones"] == job.n_stones
    assert job.meta["reach_check"] == "kinematic"                               # the original untouched
    assert not any("reach check" in p for p in preflight_real(cfg, stamped))


def test_a_run_during_which_the_code_changed_is_not_stamped(tmp_path):
    """The stamp carries the git state of the run's start (info["git"]); a commit or edit under the stamp paths
    during the ~20 min run would make it claim code it did not verify (2026-10-08)."""
    cfg = config.load()
    job = make_job.build_nominal(cfg)
    args = SimpleNamespace(from_stop=0, max_stones=0, trips=True, report=str(REPO / "results" / "l_wall_sim.md"))
    gone = {"git_commit": "0" * 40, "git_dirty": False}                    # a start commit the code no longer is
    assert simulate.robodk_stamp(_run(job), job, args, {"robodk": "6.0.0", "git": gone}, cfg, out_dir=tmp_path) is None
    now = mjob.git_state()
    path = simulate.robodk_stamp(_run(job), job, args, {"robodk": "6.0.0", "git": now}, cfg, out_dir=tmp_path)
    assert path is not None
    assert {k: mjob.load(path).meta["robodk_sim"][k] for k in now} == now
