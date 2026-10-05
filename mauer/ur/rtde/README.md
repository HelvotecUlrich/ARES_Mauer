# Vendored: official UR RTDE Python client

| | |
|---|---|
| Source | https://github.com/UniversalRobots/RTDE_Python_Client_Library |
| Version | 2.8.4 (`setup.py`, package name `UrRtde`) |
| Commit | `28db033e2cde98f782431452eb810fb76ebad17c` (2026-10-02, "Add property read command supported from 5.26/10.15 (#31)") |
| License | BSD 3-Clause, Copyright (c) 2022-2026 Universal Robots A/S – see `LICENSE` (copied unchanged) |
| Copied on | 2026-10-05 |

Why vendored: pure Python (struct/socket/select/logging), runs unchanged under Windows Python 3.14; `ur_rtde`
has no cp313/cp314 wheels and needs Boost to build (research 2026-10-05, `mauer/ur/link.py` docstring).

## Files

Copied from `rtde/` of that commit: `__init__.py`, `rtde.py`, `serialize.py`, `rtde_config.py`, `rtde_decode.py`.
Not copied: `csv_reader.py` (imports numpy), `csv_writer.py`, `csv_binary_writer.py` (absolute `from rtde import`),
examples, packaging.

## Modifications

One line, so the package works as `mauer.ur.rtde` instead of a top-level `rtde`:

```diff
--- rtde/rtde.py (28db033), line 33
-    from rtde import serialize
+    from . import serialize  # ARES_Mauer: relative import (vendored as mauer.ur.rtde)
```

All other files are byte-identical to the upstream commit (sha256 of the upstream `rtde.py` before the change:
`f5c740b44dad4cbf10f2393ea7b3fd19bc10e658bb5cb1914419811a2b96402d`).

Usage: `from mauer.ur.rtde.rtde import RTDE` – see `mauer/ur/link.py`.
