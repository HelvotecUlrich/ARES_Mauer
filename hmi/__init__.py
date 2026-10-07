"""Mauer HMI: the ARES HMI v2 (amr_hmi) extended for the brick-laying runs of this repo.

Provenance: hmi/amr/ is a copy of the MA repo `10_robot/hmi/amr_hmi` (commit 5935c5b, 2026-09-28), copied
2026-10-07; the MA repo is not modified. Adaptations of the copy (docs/HMI_DESIGN.md section 4):
- imports are relative (package hmi.amr); amr_hmi's hardware tools list_symbols.py, ads_diag.py and ads_portscan.py
  are not copied.

Conventions: docs/HMI_DESIGN.md (design, threads, resource ownership), docs/ARCHITECTURE.md (frames, units).
"""
