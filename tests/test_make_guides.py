"""tools/make_guides.py (floor guides + floor station, design 2026-10-06, stones pins up): the printed locating cone
(closed mesh, fit in the STEP socket), the MDF pieces of the configured C (peg holes in diagonal sockets of every
stone, V-tabs under every board plate, no overlaps in the room or on the sheets, sheet size) and the floor station
(windows hold the plain plates, cones under every stack)."""
import copy
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
    for lg in data["legs"]:                       # a half stone (A 5 1/2): its two cones in its own pin pair
        want = sorted(q for u0, u1, kind in mg.course0(p, lg)
                      for q in ([(round((u0 + u1) / 2 + s * a, 6), round(s * b, 6)) for s in (-1, 1)] if kind == "full"
                                else [(round((u0 + u1) / 2, 6), round(s * b, 6)) for s in (-1, 1)]))
        assert holes[lg.name] == pytest.approx(want, abs=1e-6), lg.name
    names = [pc.name for pc in data["leg_pieces"]]
    assert names == ["A0-A2", "A3-B1", "B2-B4", "B5-B6", "B7-C1", "C2-C4", "C5-C6"]   # the C of 2026-10-08: A 5 /
    # B 9 (through both corners) / C 5 with the door on B7-C1 and the strip to W7's spare block on C5-C6


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
        def o(p_, q, r):                         # signed distance [mm] of r from the line p_ -> q
            return ((q[0] - p_[0]) * (r[1] - p_[1]) - (q[1] - p_[1]) * (r[0] - p_[0])) / max(math.dist(p_, q), 1e-12)
        def apart(x, y):                         # strictly on both sides, each by more than 1 um
            return (x > 1e-6 and y < -1e-6) or (x < -1e-6 and y > 1e-6)
        return apart(o(a, b, c), o(a, b, d)) and apart(o(c, d, a), o(c, d, b))
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
    assert len(sheets) <= 5                      # 5 since the two-row station (2026-10-09); 4 before
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


def test_station_cut_never_runs_through_a_stack_of_the_other_row(cfg, data):
    """Two rows: the cut between two half stacks of row 1 (x 740) would run through a cone of the full stack of row 2
    at x 679 (cones at 679 +- 50.25) - no cut there, and no other cut keeps the pieces <= segment_max."""
    c2 = copy.deepcopy(cfg)
    ps = c2["pickup_station"]
    ps["slots_xy"] = [[679.0, float(ps["row2_y"])]]
    ps["half_slots_xy"] = [[680.0, float(ps["row_y"])], [800.0, float(ps["row_y"])]]
    with pytest.raises(ValueError, match="no cut between the stacks"):
        mg.station_pieces(c2, data["params"], mg._station_plates(c2))


def test_guides_are_cut_from_5_mm_mdf_the_existing_plates_stay_4(cfg, data):
    """Samuel 2026-10-09: the MDF of the guides (wall strips, station piece) is 5 mm thick instead of 4 - the first
    course and the station stones stand 1 mm higher, the cone pegs are 1 mm shorter than the MDF. The board plates
    (W0..W7, S0 / S1, 2026-10-05) and the deck plate on ARES (2026-10-08) are made already: 4 mm."""
    p = data["params"]
    assert cfg["guides"]["mdf_t"] == 5.0 and p.mdf_t == 5.0
    assert cfg["wall"]["base_z"] == 5.0 and cfg["pickup_station"]["holder_z"] == 5.0
    assert p.peg_len == pytest.approx(p.mdf_t - 1.0)
    assert cfg["plates"]["mdf_t"] == 4.0 and cfg["deck"]["holder_z"] == 4.0
    assert all(abs(float(t["xyz"][2]) - 4.1) < 1e-9 for t in cfg["targets"] if t.get("parent") in ("wall", "station"))


def test_cli_writes_everything(tmp_path):
    assert mg.main(["--out", str(tmp_path), "--no-map"]) == 0
    for f in ("laser_sheet_1.dxf", "laser_test.dxf", "laser_sheets.svg", "laser_sheets.png", "locating_cone.stl",
              "locating_cone.scad"):
        assert (tmp_path / f).stat().st_size > 0, f
    dxf = (tmp_path / "laser_sheet_1.dxf").read_text()
    assert dxf.startswith("0\nSECTION") and "CUT" in dxf and "ENGRAVE" in dxf and dxf.rstrip().endswith("EOF")


def test_kerf_compensation_of_the_cut_lines(data, tmp_path):
    """2026-10-07: ~0.8 mm play in the first test cut's dovetail (0.1 mm clearance per side + the uncompensated kerf)
    -> the DXF cut lines are offset by kerf/2: outlines outside, holes / windows inside the nominal geometry."""
    p = data["params"]
    assert p.kerf > 0 and p.joint_c < 0.1
    sq = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)]
    big = np.asarray(mg.offset_polygon(sq, 0.15))
    assert big.min(axis=0) == pytest.approx((-0.15, -0.15)) and big.max(axis=0) == pytest.approx((100.15, 50.15))
    small = np.asarray(mg.offset_polygon(sq, -0.15))
    assert small.min(axis=0) == pytest.approx((0.15, 0.15)) and small.max(axis=0) == pytest.approx((99.85, 49.85))
    pc = data["leg_pieces"][0]
    outline, holes, windows = mg.cut_geometry(pc, p)
    for q, n in zip(outline, pc.outline):                                   # every vertex moved outwards by >= kerf/2
        assert math.dist(q, n) >= p.kerf / 2 - 1e-9 and not _inside(q, pc.outline)
    assert [d for _, d in holes] == pytest.approx([d - p.kerf for _, d in pc.holes])
    sheet = mg.nest([pc], p)[0]
    dxf = mg.sheet_dxf(sheet, p, "t")
    radii = {round(float(e.split("\n40\n")[1].split("\n")[0]), 4) for e in dxf.ents if e.startswith("0\nCIRCLE\n8\nCUT")}
    assert radii == {round((p.peg_hole_d - p.kerf) / 2, 4)}


def test_corner_where_c_runs_through_is_one_piece_and_the_ends_line_up():
    """Variant c_acb (2026-10-07, Samuel: the ends of A and C line up): C runs through its corner with B; the L-piece
    B5-C1 covers B's last stone, the filled corner gap and C's first stones, and A's start lies on the line of C's
    end."""
    data = mg.build(config.load(variant="c_acb"), with_job=False)
    pc = next(pc for pc in data["leg_pieces"] if pc.name == "B5-C1")
    assert "C runs through" in pc.notes
    poly = [pc.to_wall(q) for q in pc.outline]
    legs = {lg.name: lg for lg in data["legs"]}
    A, B, C = legs["A"], legs["B"], legs["C"]
    P = data["params"].pitch
    for q in (B.to_wall(B.n0 * P - 1.0, 0.0), B.to_wall(B.n0 * P + 1.5, 0.0),     # B's end, the corner gap
              C.to_wall(P / 2, 0.0), C.to_wall(1.5 * P, 0.0)):                    # C's first two stones
        assert _inside(q, poly), q
    assert A.to_wall(0.0, 0.0)[0] == pytest.approx(C.to_wall(C.n0 * P, 0.0)[0], abs=1e-9)


def test_a_leg_of_five_and_a_half_stones_ends_with_a_half_stone_on_the_corner_piece():
    """The C of 2026-10-07 (variant c_a55): A 5 1/2 stones - course 0 = 5 full stones + a half stone at the corner
    end, on the corner L-piece A4-B1 with the two cones of its own pin pair; A's joints (V-tabs) stay at whole
    pitches."""
    data = mg.build(config.load(variant="c_a55"), with_job=False)
    p = data["params"]
    A = next(lg for lg in data["legs"] if lg.name == "A")
    st = mg.course0(p, A)
    assert [k for _, _, k in st] == ["full"] * 5 + ["half"] and st[-1][1] == pytest.approx(1100.0)
    pc = next(pc for pc in data["leg_pieces"] if pc.name.startswith("A4-"))
    assert [b[0] for b in pc.blocks if b[0].startswith("A")] == ["A4", "A5"] and pc.blocks[1][3] == "half"


def test_every_piece_name_is_engraved_on_the_piece(data):
    """Review 2026-10-07: from the bbox corner the name of a corner L-piece placed turned on the sheet lay outside the
    piece. The label starts inside the first stone block and runs along it; 120 mm of its baseline stay on the piece,
    for both layouts."""
    for d in (data, mg.build(config.load(variant="c_acb"), with_job=False)):
        p = d["params"]
        for pc in d["leg_pieces"] + d["station_pieces"]:
            (x, y), ang = mg.piece_label_anchor(pc, p)
            for t in (0.0, 60.0, 120.0):
                q = (x + t * math.cos(ang), y + t * math.sin(ang))
                assert _inside(q, pc.outline), (pc.name, t)


def _covered(data, w):
    return any(_inside(w, [pc.to_wall(q) for q in pc.outline]) for pc in data["leg_pieces"])


def test_the_strip_runs_through_the_door_and_beyond_the_free_end_to_the_spare_block(data):
    """The C of 2026-10-08: leg C's strip runs through the door (the 400 mm piece stays located by the corner L-piece
    at B, not measured) with cones only under the course-0 stones beside the door - half, full, half (start_half) -
    and on beyond C's free end to the spare block of board W7 (k = 6), whose plate finds its V-tab."""
    p = data["params"]
    C = _leg(data, "C")
    assert mg.course0(p, C) == [(600.0, 700.0, "half"), (700.0, 900.0, "full"), (900.0, 1000.0, "half")]
    holes = _holes_in_leg(data)["C"]
    assert len(holes) == 6 and all(u > 600.0 for u, _ in holes)
    for u in (100.0, 300.0, 500.0, 800.0, 1100.0, 1300.0):                    # door, the piece, the spare block
        assert _covered(data, C.to_wall(u, 0.0)), u
    assert not _covered(data, C.to_wall(1500.0, 0.0))
    w7 = next(s for s in data["sites"] if s.board == "W7")
    assert w7.leg == "C" and w7.k == 6 and w7.spare


def test_the_window_courses_leave_course_0_and_the_guides_of_leg_a_alone(data):
    p = data["params"]
    A = _leg(data, "A")
    assert [k for _, _, k in mg.course0(p, A)] == ["full"] * 5
