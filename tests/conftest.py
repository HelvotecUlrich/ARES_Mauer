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
    for t in cfg["targets"]:
        if t["parent"] == "wall":
            i = int(t["name"][1:])
            t.pop("leg", None)
            t["xyz"] = [60.0 + 800.0 * i, 147.0, 4.1]
            t["rpy_deg"] = [180.0, 0.0, 0.0]
    return cfg
