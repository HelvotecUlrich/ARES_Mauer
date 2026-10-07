"""config/station.toml -> the config dict of the amr_hmi code (hmi/amr), and the checks of the HMI settings.

amr_hmi read config.yaml; here every value comes from station.toml (docs/HMI_DESIGN.md section 5): [hmi] (the nine
amr "hmi" keys), [hmi.frame], [hmi.move] (panel presets), [ares_ads] (ADS target, timeout, the relative-move
defaults) and the PLC limits of mauer/ares/ads.py (GVL_Move init values). The amr widgets read the MAIN config
(variants never override [hmi] or [ares_ads]); runs use the session config with the job's variant.
"""
from __future__ import annotations

from typing import Any, Mapping

from mauer.ares import ads

# the keys of the amr config.yaml "hmi" section (MA amr_hmi commit 5935c5b)
AMR_HMI_KEYS = ("poll_interval_ms", "heartbeat_interval_ms", "reconnect_interval_s", "jog_speed_mms",
                "jog_rot_speed_degs", "default_speed_limit_mms", "jog_accel_mms2", "accel_outside_manual_mms2",
                "watchdog_timeout_s")
TWIN_MIN_PORT = 20630               # [hmi.twin] port: never the user's RoboDK (20500/20501), simulate.py, the tests
USER_RDK_PORTS = (20500, 20501)


def _table(cfg: Mapping, *path: str) -> Mapping:
    node: Any = cfg
    for i, name in enumerate(path):
        if not isinstance(node, Mapping) or not isinstance(node.get(name), Mapping):
            raise KeyError(f"config/station.toml has no [{'.'.join(path[:i + 1])}] table (needed by the HMI)")
        node = node[name]
    return node


def amr_cfg(cfg: Mapping) -> dict:
    """The amr_hmi config dict {"ads", "plc", "hmi", "frame", "move"} from station.toml; KeyError names the missing
    table. Every amr consumer keeps its shape: AdsWorker cfg['ads'/'plc'/'hmi'], FrameConfig cfg['frame'],
    MoveLimits / MoveDefaults cfg['move'], JogPanel cfg['hmi']."""
    a, h = _table(cfg, "ares_ads"), _table(cfg, "hmi")
    fr, mv = _table(cfg, "hmi", "frame"), _table(cfg, "hmi", "move")
    return {
        "ads": {"ams_net_id": str(a["ams_net_id"]), "ads_port": int(a["port"]), "host_ip": str(a.get("host_ip", "")),
                "timeout_ms": int(a.get("timeout_ms", 1000))},
        "plc": {"prefix": "", "to_plc_struct": ads.TO.rstrip("."), "from_plc_struct": ads.FROM.rstrip(".")},
        "hmi": {k: h[k] for k in AMR_HMI_KEYS},
        "frame": {"plus_y_is_left": bool(fr["plus_y_is_left"]), "plus_omega_is_ccw": bool(fr["plus_omega_is_ccw"]),
                  "verified": bool(fr["verified"])},
        "move": {"default_distance_mm": float(mv["default_distance_mm"]),
                 "default_angle_deg": float(mv["default_angle_deg"]),
                 "test_distance_mm": float(mv["test_distance_mm"]), "test_speed_mms": float(mv["test_speed_mms"]),
                 "default_speed_mms": float(a["speed_mms"]),
                 "default_rot_speed_degs": float(a.get("rot_speed_degs", ads.DEFAULT_ROT_SPEED_DEGS)),
                 "default_accel_mms2": float(a["accel_mms2"]),
                 "max_distance_mm": ads.MAX_DIST_MM, "max_angle_deg": ads.MAX_ANGLE_DEG,
                 "max_speed_mms": ads.MAX_SPEED_MMS, "max_rot_speed_degs": ads.MAX_ROT_SPEED_DEGS,
                 "max_accel_mms2": ads.MAX_ACCEL_MMS2, "min_distance_mm": ads.MIN_MOVE_MM,
                 "min_angle_deg": ads.MIN_MOVE_DEG},
    }


def check_amr_cfg(cfg: Mapping) -> list[str]:
    """Problems of the HMI settings (empty = ok): [hmi] jog_accel_mms2 < [ares_ads] accel_mms2 (the PLC caps the
    move acceleration with fAccel_mms); heartbeat_interval_ms >= 1000 * ads.HB_WINDOW_S (AresAds preflight, pattern
    A); frame flags differ; frame not verified; [hmi.twin] port < 20630 or a port range containing 20500/20501."""
    p: list[str] = []
    try:
        a = amr_cfg(cfg)
    except (KeyError, TypeError, ValueError) as e:
        return [f"HMI config: {e}"]
    h, fr = a["hmi"], a["frame"]
    acc = float(_table(cfg, "ares_ads")["accel_mms2"])
    if float(h["jog_accel_mms2"]) < acc:
        p.append(f"[hmi] jog_accel_mms2 {h['jog_accel_mms2']} < [ares_ads] accel_mms2 {acc}: the PLC caps the move "
                 "acceleration with fAccel_mms (written on MANUAL entry)")
    if float(h["heartbeat_interval_ms"]) >= 1000.0 * ads.HB_WINDOW_S:
        p.append(f"[hmi] heartbeat_interval_ms {h['heartbeat_interval_ms']} >= {1000.0 * ads.HB_WINDOW_S:.0f}: the "
                 "AresAds preflight would not see the heartbeat change (HB_WINDOW_S, pattern A)")
    if fr["plus_y_is_left"] != fr["plus_omega_is_ccw"]:
        p.append("[hmi.frame] plus_y_is_left and plus_omega_is_ccw differ: only both true or both false is "
                 "physically consistent (amr README direction verification) - stop and check the PLC kinematics")
    if not fr["verified"]:
        p.append("[hmi.frame] verified = false: jog / relative-move directions not verified on the robot")
    tw = (cfg.get("hmi", {}) or {}).get("twin")
    if isinstance(tw, Mapping):
        port, tries = int(tw.get("port", TWIN_MIN_PORT)), max(1, int(tw.get("port_tries", 1)))
        if port < TWIN_MIN_PORT:
            p.append(f"[hmi.twin] port {port} < {TWIN_MIN_PORT}: the twin needs its own RoboDK API port")
        if any(port <= q < port + tries for q in USER_RDK_PORTS):
            p.append(f"[hmi.twin] ports {port}..{port + tries - 1} include the user's RoboDK (20500/20501)")
    return p
