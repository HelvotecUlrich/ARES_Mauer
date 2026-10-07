"""Config variants (2026-10-07): config/variants/<name>.toml laid over config/station.toml (mauer.config.load,
merge), their status tags and sha256 in the provenance (mauer.job), the variant in the job meta, the real-run
preflight and the default output names of the tools."""
import sys

import pytest

from mauer import REPO, config
from mauer import job as mjob

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_job  # noqa: E402


def test_merge_replaces_arrays_and_merges_tables():
    base = {"wall": {"shape": "C", "legs": [{"name": "A"}, {"name": "B"}]}, "ur": {"host": "x"}}
    over = {"wall": {"legs": [{"name": "Z"}]}, "deck": {"layers": 2}}
    out = config.merge(base, over)
    assert out == {"wall": {"shape": "C", "legs": [{"name": "Z"}]}, "ur": {"host": "x"}, "deck": {"layers": 2}}
    assert base["wall"]["legs"] == [{"name": "A"}, {"name": "B"}]                 # inputs unchanged


def test_variant_c_acb_overrides_the_legs_only():
    main, var = config.load(), config.load(variant="c_acb")
    assert config.variant_of(main) is None and config.suffix(main) == ""
    assert config.variant_of(var) == "c_acb" and config.suffix(var) == "_c_acb"
    assert [lg["n0"] for lg in main["wall"]["legs"]] == [5.5, 7, 5]
    assert [lg["n0"] for lg in var["wall"]["legs"]] == [5, 7, 5] and var["wall"]["legs"][2]["runs_through"] is True
    for sec in ("ur", "brick", "deck", "pickup_station", "camera"):
        assert main[sec] == var[sec], sec
    assert main["wall"]["shape"] == var["wall"]["shape"] == "C"
    with pytest.raises(FileNotFoundError, match="known: .*c_acb"):
        config.load(variant="no_such_variant")


def test_status_tags_and_sha_include_the_variant():
    st_main, st_var = mjob.config_status(), mjob.config_status(variant="c_acb")
    assert "[[wall.legs]] C.runs_through" not in st_main
    assert st_var["[[wall.legs]] C.runs_through"]["status"] == "CONFIRMED"
    assert st_var["[[wall.legs]] A.n0"]["source"].startswith("CONFIRMED: Samuel 2026-10-07 (the C's legs A and C")
    assert st_var["[ur] host"] == st_main["[ur] host"]                            # keys outside the overlay
    assert mjob.config_sha256() != mjob.config_sha256(variant="c_acb")
    cfg = config.load(variant="c_acb")
    keys = {d["key"] for d in mjob.depends_on(cfg, ["[[wall.legs]] *"])}
    assert "[[wall.legs]] C.xyz_in_wall" in keys


def test_job_records_its_variant_and_the_preflight_checks_it(tmp_path):
    from mauer.sequencer import preflight_real
    cfg = config.load(variant="c_acb")
    job = make_job.build_nominal(cfg)
    assert job.meta["config_variant"] == "c_acb"
    assert job.config_sha256 == mjob.config_sha256(variant="c_acb")
    p_ok = preflight_real(cfg, job)
    assert not any("config variant" in p or "sha256 differs" in p for p in p_ok)
    p_bad = preflight_real(config.load(), job)                                   # main config, variant job
    assert any("config variant None loaded, the job was built with 'c_acb'" in p for p in p_bad)
    assert make_job.main(["--variant", "c_acb", "--out", str(tmp_path / "j.json")]) == 0
    assert mjob.load(tmp_path / "j.json").meta["config_variant"] == "c_acb"
