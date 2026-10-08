"""Job sessions (docs/HMI_DESIGN.md D-H11): a job always comes with the config variant it was built with."""
from __future__ import annotations

import json

import pytest

from hmi.core.preflight import config_drift, sim_preflight
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


def test_the_session_keeps_the_hash_of_the_config_it_parsed(built, tmp_path):
    """Review 2026-10-08: the run uses session.cfg, the preflight reads the file - an edit after the load (even one
    reverted to the job's config) is reported until the job is loaded again."""
    toml = tmp_path / "station.toml"
    toml.write_bytes(config.STATION_TOML.read_bytes())
    path = mjob.save(built[None].job, tmp_path / "nominal_C.json")
    s = load_job_file(path, toml)
    assert s.config_sha256 == mjob.config_sha256(toml) == mjob.config_sha256()
    assert built[None].config_sha256 == mjob.config_sha256() and built["c_acb"].config_sha256 == \
        mjob.config_sha256(None, "c_acb")
    assert config_drift(s.cfg, toml, s.config_sha256) == []
    original = toml.read_bytes()
    toml.write_bytes(b"# edited after the load\n" + original)       # (an edit inside [hmi*] would not count)
    drift = config_drift(s.cfg, toml, s.config_sha256)
    assert len(drift) == 1 and "changed on disk since the job was loaded" in drift[0]
    rep = sim_preflight(s.cfg, s.job, toml, s.config_sha256)
    assert rep.items[0].text == drift[0] and not rep.items[0].blocking    # SIM: informative
    reloaded = load_job_file(path, toml)                                    # loading again takes the new file
    assert config_drift(reloaded.cfg, toml, reloaded.config_sha256) == []
    toml.write_bytes(original)
    assert config_drift(reloaded.cfg, toml, reloaded.config_sha256) != []
    assert config_drift(s.cfg, toml, None) == []                            # test sessions carry no hash


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
