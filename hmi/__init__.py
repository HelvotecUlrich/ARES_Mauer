"""Mauer HMI: the ARES HMI v2 (amr_hmi) extended for the brick-laying runs of this repo - `py.exe -m hmi`.

Provenance: hmi/amr/ is a copy of the MA repo `10_robot/hmi/amr_hmi` (commit 5935c5b, 2026-09-28), copied
2026-10-07; the MA repo is not modified. Adaptations of the copy (docs/HMI_DESIGN.md section 4):
- imports are relative (package hmi.amr); config.yaml is replaced by config/station.toml [hmi], [hmi.frame],
  [hmi.move] and [ares_ads] (hmi/core/config.py amr_cfg); amr_hmi's hardware tools list_symbols.py, ads_diag.py and
  ads_portscan.py are not copied.
- logic.jog_enabled / go_enabled: optional run_lock reason (default "": amr behaviour); ui/move_panel.py
  set_run_lock, ui/odom_map.py OdomPanel.set_reset_lock, ui/control_widget.py set_run_lock / set_reset_lock and its
  banner: a Mauer REAL run locks jog / GO while it runs and the odometry reset while it is loaded.
- ui/main_window.py became hmi/main_window.py (Mauer tabs, ConfirmBar, HALT also stops the run, the window never
  creates an AdsWorker itself), main.py became hmi/main.py (command line, no YAML; apply_dark_theme unchanged).

Layout: hmi/amr (the amr copy), hmi/core (services: config, ADS worker setup, sessions, rigs, preflight, snapshots,
read-only sources, RunController), hmi/views (Mauer widgets), hmi/main_window.py, hmi/main.py.
Conventions: docs/HMI_DESIGN.md (design, threads, resource ownership), docs/ARCHITECTURE.md (frames, units).
"""
