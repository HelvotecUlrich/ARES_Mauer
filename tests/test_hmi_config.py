"""HMI settings in config/station.toml: the mapping to the amr_hmi config dict (docs/HMI_DESIGN.md section 5.2), the
checks, the status tags and the ARES outline of the odometry map."""
from __future__ import annotations

import copy

import pytest

from hmi.amr.ui import constants as C
from hmi.core.config import AMR_HMI_KEYS, amr_cfg, check_amr_cfg
from hmi_fakes import AMR_CFG
from mauer import config
from mauer import job as mjob
from mauer.ares import ads

pytestmark = pytest.mark.usefixtures("no_lab_network")


@pytest.fixture(scope="module")
def cfg():
    return config.load()


def test_amr_cfg_mapping_table(cfg):
    a = amr_cfg(cfg)
    aa, h = cfg["ares_ads"], cfg["hmi"]
    assert a["ads"] == {"ams_net_id": aa["ams_net_id"], "ads_port": aa["port"], "host_ip": aa["host_ip"],
                        "timeout_ms": aa["timeout_ms"]}
    assert a["plc"] == {"prefix": "", "to_plc_struct": "GVL_HMI.stToPlc", "from_plc_struct": "GVL_HMI.stFromPlc"}
    assert a["hmi"] == {k: h[k] for k in AMR_HMI_KEYS} and len(AMR_HMI_KEYS) == 9
    assert a["frame"] == h["frame"]
    m = a["move"]
    for k in ("default_distance_mm", "default_angle_deg", "test_distance_mm", "test_speed_mms"):
        assert m[k] == h["move"][k]
    assert (m["default_speed_mms"], m["default_rot_speed_degs"], m["default_accel_mms2"]) == \
        (aa["speed_mms"], aa["rot_speed_degs"], aa["accel_mms2"])
    assert (m["max_distance_mm"], m["max_angle_deg"], m["max_speed_mms"], m["max_rot_speed_degs"],
            m["max_accel_mms2"], m["min_distance_mm"], m["min_angle_deg"]) == \
        (ads.MAX_DIST_MM, ads.MAX_ANGLE_DEG, ads.MAX_SPEED_MMS, ads.MAX_ROT_SPEED_DEGS, ads.MAX_ACCEL_MMS2,
         ads.MIN_MOVE_MM, ads.MIN_MOVE_DEG)


def test_station_toml_reproduces_the_amr_config_yaml(cfg):
    """Every value the amr code reads equals the MA config.yaml it was tested with (except the ADS address, which
    hmi_fakes replaces by a TEST-NET address)."""
    a = amr_cfg(cfg)
    for section in ("plc", "hmi", "frame", "move"):
        assert a[section] == AMR_CFG[section], section
    assert (a["ads"]["ads_port"], a["ads"]["timeout_ms"]) == (AMR_CFG["ads"]["ads_port"], AMR_CFG["ads"]["timeout_ms"])


def test_variants_do_not_touch_the_hmi_settings(cfg):
    for v in sorted(p.stem for p in config.VARIANTS.glob("*.toml")):
        c = config.load(variant=v)
        assert amr_cfg(c) == amr_cfg(cfg) and c["hmi"] == cfg["hmi"], v


def test_checks_pass_on_the_station_config(cfg):
    assert check_amr_cfg(cfg) == []


@pytest.mark.parametrize("edit, frag", [
    (lambda c: c["hmi"].update(jog_accel_mms2=100.0), "jog_accel_mms2"),
    (lambda c: c["hmi"].update(heartbeat_interval_ms=250), "heartbeat_interval_ms"),
    (lambda c: c["hmi"]["frame"].update(plus_y_is_left=False), "differ"),
    (lambda c: c["hmi"]["frame"].update(verified=False), "not verified"),
    (lambda c: c["hmi"]["twin"].update(port=20599), "own RoboDK API port"),
    (lambda c: c["hmi"]["twin"].update(port=20495, port_tries=10), "user's RoboDK"),
    (lambda c: c.pop("ares_ads"), "[ares_ads]"),
])
def test_checks_find_each_problem(cfg, edit, frag):
    c = copy.deepcopy(cfg)
    edit(c)
    assert any(frag in p for p in check_amr_cfg(c)), check_amr_cfg(c)


def test_missing_table_is_named(cfg):
    c = copy.deepcopy(cfg)
    del c["hmi"]["move"]
    with pytest.raises(KeyError, match=r"\[hmi.move\]"):
        amr_cfg(c)


def test_every_hmi_key_has_a_status_tag():
    st = {k: v for k, v in mjob.config_status().items() if k.startswith("[hmi")}
    assert len(st) >= 30
    assert all(v["status"] in ("CONFIRMED", "ASSUMPTION", "PLACEHOLDER") for v in st.values()), \
        {k: v["status"] for k, v in st.items() if v["status"] not in ("CONFIRMED", "ASSUMPTION", "PLACEHOLDER")}


def test_odometry_map_outline_is_the_ares_chassis(cfg):
    """amr ui/constants ROBOT_LENGTH_M / ROBOT_WIDTH_M (kept as copied) = [ares] length / width (CAD)."""
    assert C.ROBOT_LENGTH_M * 1000.0 == pytest.approx(cfg["ares"]["length"])
    assert C.ROBOT_WIDTH_M * 1000.0 == pytest.approx(cfg["ares"]["width"])
