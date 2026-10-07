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
    cfg["wall"]["dist_nominal"] = 840.0                   # 740 (reach study 2026-10-02) until 2026-10-06: with the
    # stones pins up the first course stands 22 mm lower and the RoboDK reach of course 0 at 740 mm has an 80 mm gap
    # beside ARES (results/reach_table.json) - the greedy planner cannot step over it
    _wall_boards(cfg, {f"W{i}": (None, 60.0 + 800.0 * i) for i in range(8)})
    return cfg


def _wall_boards(cfg: dict, layout: dict) -> None:
    """Replace the config's wall boards by all 8 existing boards W0..W7 (ids 30 + 10 i, independent of how many the
    configured wall uses - the C of 2026-10-07 has W7 as a spare) at {name: (leg or None, x)}."""
    walls = []
    for i in range(8):
        leg, x = layout[f"W{i}"]
        t = {"name": f"W{i}", "parent": "wall", "first_id": 30 + 10 * i, "xyz": [x, 147.0, 4.1],
             "rpy_deg": [180.0, 0.0, 0.0]}
        if leg is not None:
            t["leg"] = leg
        walls.append(t)
    cfg["targets"] = walls + [t for t in cfg["targets"] if t["parent"] != "wall"]      # wall boards first, as in the file


# The L of 2026-10-05 (commit 307c1e0): legs A = 12 and B = 6 stones by a butt corner, the 8 wall boards on block centres
# (leg, x in the leg frame). The config itself is the C since 2026-10-06.
L_LEGS = [{"name": "A", "n0": 12, "xyz_in_wall": [0.0, 0.0, 0.0], "rpy_in_wall_deg": [0.0, 0.0, 0.0]},
          {"name": "B", "n0": 6, "xyz_in_wall": [2340.0, -62.69, 0.0], "rpy_in_wall_deg": [0.0, 0.0, -90.0]}]
L_BOARDS = {"W0": ("A", 60.0), "W1": ("A", 460.0), "W2": ("A", 860.0), "W3": ("A", 1460.0), "W4": ("A", 2060.0),
            "W5": ("B", 60.0), "W6": ("B", 460.0), "W7": ("B", 1060.0)}


def l_config():
    """The station config with the L wall of 2026-10-05 (L_LEGS, L_BOARDS; wall distance 840 mm since 2026-10-06, was
    740 - see straight_config) - for the tests of the L behaviour (job v2 routes, leg change, half stones),
    independent of the wall in config/station.toml."""
    import copy

    from mauer import config
    cfg = copy.deepcopy(config.load())
    cfg["wall"]["shape"] = "L"
    cfg["wall"]["ares_inside"] = False                    # ARES worked outside the L's corner
    cfg["wall"]["dist_nominal"] = 840.0                   # the L's wall distance (was 740, see straight_config)
    cfg["wall"]["legs"] = copy.deepcopy(L_LEGS)
    _wall_boards(cfg, L_BOARDS)
    return cfg


# The C of 2026-10-06 (commit 5787d41): legs A = 10, B = 7, C = 5 stones, ARES inside, the 8 wall boards (leg, x in
# the leg frame). The config is the C A 5 / B 7 / C 5 since 2026-10-07 (W7 spare).
C10_LEGS = [
    {"name": "A", "n0": 10, "xyz_in_wall": [0.0, 0.0, 0.0], "rpy_in_wall_deg": [0.0, 0.0, 0.0], "side": "right",
     "dist": 580.0},
    {"name": "B", "n0": 7, "xyz_in_wall": [1940.0, 62.69, 0.0], "rpy_in_wall_deg": [0.0, 0.0, 90.0], "side": "front",
     "dist": 840.0},
    {"name": "C", "n0": 5, "xyz_in_wall": [1877.31, 1402.69, 0.0], "rpy_in_wall_deg": [0.0, 0.0, 180.0],
     "side": "left", "dist": 580.0}]
C10_BOARDS = {"W0": ("A", 260.0), "W1": ("A", 660.0), "W2": ("A", 1060.0), "W3": ("A", 1660.0), "W4": ("B", 260.0),
              "W5": ("B", 860.0), "W6": ("C", 260.0), "W7": ("C", 860.0)}


def c10_config():
    """The station config with the C of 2026-10-06 (C10_LEGS, C10_BOARDS) - for tests of code whose numbers were
    tuned on that board spread (mauer.reference frame fits), independent of the wall in config/station.toml."""
    import copy

    from mauer import config
    cfg = copy.deepcopy(config.load())
    cfg["wall"]["legs"] = copy.deepcopy(C10_LEGS)
    _wall_boards(cfg, C10_BOARDS)
    return cfg
