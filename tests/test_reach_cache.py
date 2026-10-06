"""mauer.reach_cache: one RoboDK reach table per configuration key (several wall distances side by side), the old
single-table file still read, the key the same as tools/make_job.py reach_key."""
import importlib.util
import json

import pytest

from conftest import l_config
from mauer import REPO, reach_cache


def _make_job():
    spec = importlib.util.spec_from_file_location("tool_make_job_rc", REPO / "tools" / "make_job.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_committed_cache_has_the_l_and_the_configured_distance():
    from mauer import config
    tables = reach_cache.read()
    dists = sorted(v["dist"] for v in tables.values())
    assert 740.0 in dists and config.load()["wall"]["dist_nominal"] in dists
    assert reach_cache.get(l_config(), 740.0) is not None
    t = reach_cache.get(config.load(), config.load()["wall"]["dist_nominal"])
    assert sorted(t) == list(range(config.load()["wall"]["courses"])) and all(len(d) == 101 for d in t.values())


def test_store_keeps_other_tables_and_reads_version_1(tmp_path):
    cfg = l_config()
    p = tmp_path / "reach.json"
    t740 = {0: {0.0: True, 20.0: False}}
    p.write_text(json.dumps({"key": reach_cache.key(cfg, 740.0), "dist": 740.0, "table": {"0": {"0.0": True, "20.0": False}}}))
    assert reach_cache.get(cfg, 740.0, p) == t740                          # version 1 file
    k = reach_cache.store(cfg, 840.0, {0: {0.0: False}}, p)
    assert k == reach_cache.key(cfg, 840.0) != reach_cache.key(cfg, 740.0)
    assert reach_cache.get(cfg, 740.0, p) == t740 and reach_cache.get(cfg, 840.0, p) == {0: {0.0: False}}
    assert json.loads(p.read_text())["version"] == 2
    assert reach_cache.get(cfg, 790.0, p) is None


def test_key_is_make_jobs_and_the_loader_refuses_other_distances(tmp_path):
    mj = _make_job()
    cfg = l_config()
    assert mj.reach_key(cfg, 840.0) == reach_cache.key(cfg, 840.0)
    p = tmp_path / "reach.json"
    reach_cache.store(cfg, 740.0, {0: {0.0: True}}, p)
    assert mj.load_reach_table(cfg, 740.0, p)[0] == {0: {0.0: True}}
    with pytest.raises(ValueError, match="another configuration"):
        mj.load_reach_table(cfg, 840.0, p)
    assert mj.load_reach_table(cfg, 840.0, p, allow_stale=True)[0] == {0: {0.0: True}}   # the only table
