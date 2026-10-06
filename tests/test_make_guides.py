"""tools/make_guides.py (floor guides + floor station, design 2026-10-06, stones pins up): the printed locating cone
(closed mesh, fit in the STEP socket), the MDF pieces of the configured C (peg holes in diagonal sockets of every
stone, V-tabs under every board plate, no overlaps in the room or on the sheets, sheet size) and the floor station
(windows hold the plain plates, cones under every stack)."""
import math
import sys
from collections import Counter

import numpy as np
import pytest

from mauer import REPO, config, floor

if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
import make_guides as mg  # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load()


@pytest.fixture(scope="module")
def data(cfg):
    return mg.build(cfg, with_job=False)


def _closed(V, T):
    """Every directed edge once and its reverse once: closed, consistently oriented (outward normals)."""
    e = Counter((int(a), int(b)) for t in T for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])))
    return all(c == 1 for c in e.values()) and all(e.get((b, a)) == 1 for a, b in e)


def _volume(V, T):
    return float(sum(np.dot(V[a], np.cross(V[b], V[c])) for a, b, c in T) / 6.0)


def test_locating_cone_is_closed_and_fits_the_socket(data):
    p = data["params"]
    shells = mg.locator_mesh(p)
    assert len(shells) == 2 and all(_closed(*sh) for sh in shells)
    V, T = mg.locator_mesh(p, print_orientation=False)[0]
    n = mg.N_SEG
    k = n / (2 * math.pi) * math.sin(2 * math.pi / n)                       # polygon / circle area
    r0, r1 = mg.cone_r(p, 0.0), mg.cone_r(p, p.locator_h)
    assert _volume(V, T) == pytest.approx(math.pi * k * p.locator_h / 3 * (r0 * r0 + r0 * r1 + r1 * r1), rel=1e-9)
    for z in np.linspace(0.0, p.locator_h, 8):                              # socket cone (STEP) - cone = clearance
        socket = 18.0 - (18.0 - 7.508) * z / 22.5
        assert socket - mg.cone_r(p, z) == pytest.approx(p.clearance)
    assert p.locator_h < p.socket_depth and p.peg_len < p.mdf_t
    Vp = np.vstack([sh[0] for sh in shells])                                # print orientation: narrow top on the bed
    assert Vp[:, 2].min() == pytest.approx(0.0) and Vp[:, 2].max() == pytest.approx(p.locator_h + p.peg_len)


def test_stones_pins_up_stand_on_the_strip(cfg, data):
    p = data["params"]
    assert cfg["brick"]["pins_up"] is True
    assert cfg["wall"]["base_z"] == pytest.approx(p.mdf_t) == pytest.approx(cfg["pickup_station"]["holder_z"])
    assert sorted(mg.socket_xy(p, "full")) == pytest.approx([(-50.25, -25.714), (-50.25, 25.714), (50.25, -25.714),
                                                              (50.25, 25.714)])      # STEP socket centres
    loc = mg.locator_xy(p, "full")
    assert len(loc) == 2 and loc[0] == pytest.approx((-loc[1][0], -loc[1][1]))     # diagonally opposite sockets
    mesh = REPO / cfg["brick"]["mesh"]
    if mesh.exists():                                                        # robodk/make_half_stone.py output
        sys.path.insert(0, str(REPO / "robodk"))
        import make_half_stone as mh
        t = mh.read_stl(mesh)
        assert float(t[..., 2].min()) == pytest.approx(0.0, abs=1e-3)
        assert float(t[..., 2].max()) == pytest.approx(cfg["brick"]["height"] + cfg["brick"]["pin_length"], abs=1e-3)


def _leg(data, name):
    return next(lg for lg in data["legs"] if lg.name == name)


def _holes_in_leg(data):
    """{leg: sorted (u, v) of the peg holes} over all pieces (wall frame -> leg frame)."""
    p = data["params"]
    out = {lg.name: [] for lg in data["legs"]}
    for pc in data["leg_pieces"]:
        for c, dia in pc.holes:
            assert dia == pytest.approx(p.peg_hole_d)
            w = pc.to_wall(c)
            for lg in data["legs"]:
                u, v = lg.from_wall(*w)
                if -1e-6 <= u <= lg.n0 * p.pitch + 1e-6 and abs(v) < p.h:
                    out[lg.name].append((round(u, 6), round(v, 6)))
    return {k: sorted(v) for k, v in out.items()}


def test_every_first_course_stone_has_two_cones_in_diagonal_sockets(data):
    p = data["params"]
    holes = _holes_in_leg(data)
    a, b = p.pin_along / 2, p.pin_across / 2
    for lg in data["legs"]:
        want = sorted((round(k * p.pitch + p.pitch / 2 + s * a, 6), round(s * b, 6)) for k in range(lg.n0)
                      for s in (-1, 1))
        assert holes[lg.name] == pytest.approx(want, abs=1e-6), lg.name
    names = [pc.name for pc in data["leg_pieces"]]
    assert names == ["A0-A2", "A3-A5", "A6-A7", "A8-B1", "B2-B4", "B5-C1", "C2-C4"]


def _tab_apexes(data):
    """V-tab apexes of all leg pieces in the wall frame."""
    p = data["params"]
    out = []
    for pc in data["leg_pieces"]:
        pts = pc.outline
        for a, b, c in zip(pts, pts[1:] + pts[:1], pts[2:] + pts[:2]):
            if abs(math.dist(a, c) - p.vtab[0]) < 1e-6 and abs(math.dist(a, b) - math.hypot(p.vtab[0] / 2,
                                                                                          p.vtab[1])) < 1e-6:
                out.append(pc.to_wall(b))
    return out


def test_every_board_plate_finds_a_v_tab_in_one_of_its_notches(data):
    p = data["params"]
    apex = _tab_apexes(data)
    assert data["sites"], "no wall board plates in the config"
    for s in data["sites"]:
        lg = _leg(data, s.leg)
        notches = [lg.to_wall(k * p.pitch, p.h + p.vtab[1]) for k in (s.k, s.k + 1)]
        assert any(math.dist(n, a) < 1e-6 for n in notches for a in apex), s.board
    for a in apex:                                                           # tabs only on the ARES side, at joints
        assert any(abs(lg.from_wall(*a)[1] - (p.h + p.vtab[1])) < 1e-6
                   and abs((lg.from_wall(*a)[0] / p.pitch) - round(lg.from_wall(*a)[0] / p.pitch)) < 1e-9
                   for lg in data["legs"])


def _inside(q, P):
    """Point in a (non-convex) polygon, ray casting; points on the boundary count as outside."""
    if min(math.dist(q, a) for a in P) < 1e-9:
        return False
    for a, b in zip(P, P[1:] + P[:1]):
        ab = np.subtract(b, a)
        t = np.dot(np.subtract(q, a), ab) / max(np.dot(ab, ab), 1e-18)
        if 0.0 <= t <= 1.0 and math.dist(q, np.add(a, t * ab)) < 1e-9:
            return False
    inside = False
    for (x1, y1), (x2, y2) in zip(P, P[1:] + P[:1]):
        if (y1 > q[1]) != (y2 > q[1]) and q[0] < x1 + (q[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _crosses(P, Q):
    def seg(a, b, c, d):
        def o(p_, q, r):
            return (q[0] - p_[0]) * (r[1] - p_[1]) - (q[1] - p_[1]) * (r[0] - p_[0])
        return o(a, b, c) * o(a, b, d) < -1e-9 and o(c, d, a) * o(c, d, b) < -1e-9
    E = list(zip(P, P[1:] + P[:1]))
    F = list(zip(Q, Q[1:] + Q[:1]))
    return any(seg(a, b, c, d) for a, b in E for c, d in F) or any(_inside(q, P) for q in Q) \
        or any(_inside(q, Q) for q in P)


def test_pieces_do_not_overlap_in_the_room_and_dovetails_interlock(data):
    pcs = data["leg_pieces"] + data["station_pieces"]
    polys = [[pc.to_wall(q) for q in pc.outline] for pc in pcs]
    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            assert not _crosses(polys[i], polys[j]), (pcs[i].name, pcs[j].name)
    p = data["params"]
    for a, b in zip(data["leg_pieces"], data["leg_pieces"][1:]):          # consecutive pieces butt at the joint,
        A = [a.to_wall(q) for q in a.outline]                             # the tail sits in the socket with the
        B = [b.to_wall(q) for q in b.outline]                             # joint clearance
        assert min(math.dist(x, y) for x in A for y in B) < 1e-6, (a.name, b.name)
        tail = [q for q in B if _inside(q, floor.hull(A))]                 # the wide end of the tail (the neck
        assert len(tail) == 2 and all(0.0 < floor.point_dist(q, A) <= p.joint_c + 1e-6 for q in tail), \
            (a.name, b.name)                                              # lies on the joint line)


def test_pieces_fit_the_laser_and_do_not_overlap_on_the_sheets(data):
    p = data["params"]
    sheets = mg.nest(data["leg_pieces"] + data["station_pieces"], p)
    assert len(sheets) <= 4
    for items in sheets:
        polys = [[mg._place(q, rot, off) for q in pc.outline] for pc, rot, off in items]
        for P in polys:
            a = np.asarray(P)
            assert a[:, 0].min() >= p.sheet_margin - 1e-6 and a[:, 0].max() <= p.sheet[0] - p.sheet_margin + 1e-6
            assert a[:, 1].min() >= p.sheet_margin - 1e-6 and a[:, 1].max() <= p.sheet[1] - p.sheet_margin + 1e-6
        for i in range(len(polys)):
            for j in range(i + 1, len(polys)):
                assert not _crosses(polys[i], polys[j])
    for pc in data["leg_pieces"] + data["station_pieces"]:
        x0, y0, x1, y1 = pc.bbox()
        assert min(x1 - x0, y1 - y0) <= p.sheet[1] - 2 * p.sheet_margin
        for c, dia in pc.holes:                                              # every hole inside its piece
            assert _inside(c, pc.outline) and floor.point_dist(c, pc.outline) > dia / 2


def test_station_windows_hold_the_plates_and_blocks_sit_on_the_row(cfg, data):
    p = data["params"]
    ps = cfg["pickup_station"]
    wins = [w for pc in data["station_pieces"] for w in pc.windows]
    plates = mg._station_plates(cfg)
    assert len(wins) == len(plates) == 2
    for name, c, (w, h) in plates:
        win = next(x for x in wins if _inside(c, x))
        a = np.asarray(win)
        assert a[:, 0].max() - a[:, 0].min() == pytest.approx(w + 2 * p.window_c)
        assert a[:, 1].max() - a[:, 1].min() == pytest.approx(h + 2 * p.window_c)
        assert np.mean(a, axis=0) == pytest.approx(c)
    blocks = [(b[1], b[3]) for pc in data["station_pieces"] for b in pc.blocks]
    want = [((float(x), float(y)), "full") for x, y in ps["slots_xy"]] + \
           [((float(x), float(y)), "half") for x, y in ps["half_slots_xy"]]
    assert sorted(blocks) == sorted(want)
    for pc in data["station_pieces"]:
        x0, _, x1, _ = pc.bbox()
        assert x1 - x0 <= p.seg_max
    dock_y = float(ps["ares_xyz"][1])
    assert float(ps["row_y"]) - dock_y == pytest.approx(cfg["wall"]["dist_nominal"])      # row at the wall distance


def test_cli_writes_everything(tmp_path):
    assert mg.main(["--out", str(tmp_path), "--no-map"]) == 0
    for f in ("laser_sheet_1.dxf", "laser_test.dxf", "laser_sheets.svg", "laser_sheets.png", "locating_cone.stl",
              "locating_cone.scad"):
        assert (tmp_path / f).stat().st_size > 0, f
    dxf = (tmp_path / "laser_sheet_1.dxf").read_text()
    assert dxf.startswith("0\nSECTION") and "CUT" in dxf and "ENGRAVE" in dxf and dxf.rstrip().endswith("EOF")
