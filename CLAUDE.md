# ARES_Mauer – instructions for agents

Side project to Samuel's master thesis (repo `C:\Users\samue\Masterarbeit`, not part of it): UR5 (CB3) on ARES
lays bricks. See README.md for the concept.

- Never invent dimensions, positions or results. Every parameter lives in `config/station.toml` with a status tag
  (CONFIRMED / ASSUMPTION / PLACEHOLDER) and a source; results state which placeholders they depend on.
- Frame `ARES base_link` = MA repo `ares_description` (x forward, y left, z up, floor, centre between steering axes).
- RoboDK runs on Windows; run scripts with `py.exe` (API from `C:\RoboDK\Python`). Rebuild the station with
  `build_station.py` instead of editing the .rdk by hand.
- MA-repo rules that apply here too: physical quantities with units, assumptions marked, no secrets in git,
  small thematic commits.
- ARES facts come from the MA repo (PLC v2.9 relative move `GVL_HMI.stToPlc.fMoveX_mm`, calibration E003 01.10.2026).
