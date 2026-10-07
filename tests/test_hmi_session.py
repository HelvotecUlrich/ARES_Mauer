"""Job sessions (docs/HMI_DESIGN.md D-H11): a job always comes with the config variant it was built with."""
from __future__ import annotations

import json

import pytest

from hmi.core.run_controller import RunController
from hmi.core.session import SessionError, build_from_config, list_jobs, list_variants, load_job_file
from hmi_fakes import wait_until
from mauer import config
from mauer import job as mjob

pytestmark = pytest.mark.usefixtures("no_lab_network")


@pytest.fixture(scope="module")
def built():
    """The nominal jobs of the main config and of variant c_acb (tools/make_job.py build_nominal, ~1 s each)."""
    return {None: build_from_config(None), "c_acb": build_from_config("c_acb")}


def test_build_from_config_names_and_variants(built):
    main, var = built[None], built["c_acb"]
    assert (main.name, main.variant, main.variant_label, main.source, main.path) == \
        ("nominal_C", None, "main", "built", None)
    assert (var.name, var.variant, var.cfg["_variant"]) == ("nominal_C_c_acb", "c_acb", "c_acb")
    assert var.job.meta["config_variant"] == "c_acb" and "config_variant" not in main.job.meta
    assert "c_acb" in list_variants()


def test_load_job_file_loads_the_jobs_own_variant(built, tmp_path):
    for v, s in built.items():
        path = mjob.save(s.job, tmp_path / f"{s.name}.json")
        loaded = load_job_file(path)
        assert loaded.variant == v and config.variant_of(loaded.cfg) == v and loaded.source == "file"
        assert loaded.name == s.name and loaded.job.n_stones == s.job.n_stones
    assert [p.name for p in list_jobs(tmp_path)] == ["nominal_C_c_acb.json", "nominal_C.json"]   # newest first


def test_load_job_file_refusals(built, tmp_path):
    with pytest.raises(SessionError, match="not readable"):
        load_job_file(tmp_path / "missing.json")
    d = mjob.to_dict(built["c_acb"].job)
    d["meta"]["config_variant"] = "no_such_variant"
    p = tmp_path / "bad_variant.json"
    p.write_text(json.dumps(d), encoding="utf-8")
    with pytest.raises(SessionError, match="no_such_variant"):
        load_job_file(p)
    d = mjob.to_dict(built[None].job)
    d["stops"][0]["stones"][0]["T_wall_tcp"] = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 5, 1]]
    p = tmp_path / "invalid.json"
    p.write_text(json.dumps(d), encoding="utf-8")
    with pytest.raises(SessionError):
        load_job_file(p)


def test_controller_loads_in_the_run_thread(qapp, tmp_path, built):
    path = mjob.save(built["c_acb"].job, tmp_path / "nominal_C_c_acb.json")
    c = RunController(config.load()["hmi"], runs_dir=tmp_path / "runs")
    sessions, states = [], []
    c.session_loaded.connect(sessions.append)
    c.state_changed.connect(lambda s, d: states.append(s))
    try:
        c.load_file(path)
        assert c.state == "loading" and not c.can("prepare", "sim")[0]
        assert wait_until(lambda: c.state == "loaded" and sessions, 30.0, qapp)
        assert c.session.variant == "c_acb" and sessions[-1].name == "nominal_C_c_acb"
        assert c.snapshot.n_stones == built["c_acb"].job.n_stones and c.snapshot.seq_state == "idle"
        c.load_file(tmp_path / "missing.json")
        assert wait_until(lambda: c.state == "loaded" and states[-1] == "loaded" and len(states) >= 4, 30.0, qapp)
        assert c.session.variant == "c_acb"                       # a failed load keeps the previous session
    finally:
        assert c.shutdown(10.0)
