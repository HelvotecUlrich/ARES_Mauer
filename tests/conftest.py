import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def straight_config():
    """The station config as a STRAIGHT wall: [[wall.legs]] removed and the wall boards back on the straight layout
    of the commit before the L (W0..W7 every 800 mm from u = 100, wall frame, no leg) - for the tests of the
    straight-wall behaviour (tools/make_job.py --length N, robodk/simulate.py)."""
    import copy

    from mauer import config
    cfg = copy.deepcopy(config.load())
    cfg["wall"].pop("legs", None)
    cfg["wall"]["shape"] = "straight"
    cfg["wall"]["dist_nominal"] = 740.0                   # the straight wall's distance (reach study 2026-10-02)
    for t in cfg["targets"]:
        if t["parent"] == "wall":
            i = int(t["name"][1:])
            t.pop("leg", None)
            t["xyz"] = [60.0 + 800.0 * i, 147.0, 4.1]
            t["rpy_deg"] = [180.0, 0.0, 0.0]
    return cfg


# The L of 2026-10-05 (commit 307c1e0): legs A = 12 and B = 6 stones by a butt corner, the 8 wall boards on block centres
# (leg, x in the leg frame). The config itself is the C since 2026-10-06.
L_LEGS = [{"name": "A", "n0": 12, "xyz_in_wall": [0.0, 0.0, 0.0], "rpy_in_wall_deg": [0.0, 0.0, 0.0]},
          {"name": "B", "n0": 6, "xyz_in_wall": [2340.0, -62.69, 0.0], "rpy_in_wall_deg": [0.0, 0.0, -90.0]}]
L_BOARDS = {"W0": ("A", 60.0), "W1": ("A", 460.0), "W2": ("A", 860.0), "W3": ("A", 1460.0), "W4": ("A", 2060.0),
            "W5": ("B", 60.0), "W6": ("B", 460.0), "W7": ("B", 1060.0)}


def l_config():
    """The station config with the L wall of 2026-10-05 (L_LEGS, L_BOARDS, wall distance 740 mm) - for the tests of
    the L behaviour (job v2 routes, leg change, half stones), independent of the wall in config/station.toml."""
    import copy

    from mauer import config
    cfg = copy.deepcopy(config.load())
    cfg["wall"]["shape"] = "L"
    cfg["wall"]["dist_nominal"] = 740.0                   # the L's wall distance (the C: 840 mm)
    cfg["wall"]["legs"] = copy.deepcopy(L_LEGS)
    for t in cfg["targets"]:
        if t["parent"] == "wall":
            leg, x = L_BOARDS[t["name"]]
            t["leg"] = leg
            t["xyz"] = [x, 147.0, 4.1]
            t["rpy_deg"] = [180.0, 0.0, 0.0]
    return cfg
