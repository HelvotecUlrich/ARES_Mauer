"""Board definitions (mauer.vision.targets) and printables: id ranges, OpenCV 4.14 board frame, PDF/PNG output."""
import re

import numpy as np
import pytest

from mauer import config
from mauer.vision import detect, printables, targets
from mauer.vision.dataset import read_image


@pytest.fixture(scope="module")
def specs():
    return targets.board_specs(config.load())


def test_board_specs_from_config(specs):
    cfg = config.load()
    assert "calib" in specs
    assert [n for n in specs if n != "calib"] == [t["name"] for t in cfg["targets"]]
    c = specs["calib"]
    assert (c.squares_x, c.squares_y, c.dictionary, c.first_id) == (11, 8, "DICT_5X5_100", 0)
    assert c.n_markers == 44 and c.n_corners == 70
    for t in cfg["targets"]:
        s = specs[t["name"]]
        assert s.first_id == t["first_id"] and s.dictionary == cfg["boards"]["ref"]["dictionary"]
        assert s.n_markers == 10 and s.last_id == t["first_id"] + 9


def test_make_board_uses_the_id_range(specs):
    for s in specs.values():
        b = targets.make_board(s)
        assert np.array_equal(b.getIds().ravel(), np.arange(s.first_id, s.first_id + s.n_markers))
        assert not b.getLegacyPattern()


def test_board_frame_convention(specs):
    """OpenCV 4.14: origin = top-left outer corner, x right, y down, z into the board; corner 0 at (sq, sq, 0);
    top-left square black, first marker in square (1, 0)."""
    s = specs["W0"]
    sq = s.square_mm
    obj = targets.corners_obj(s)
    assert obj.shape == (s.n_corners, 3)
    assert np.allclose(obj[0], [sq, sq, 0]) and np.allclose(obj[1], [2 * sq, sq, 0])          # ids row by row
    assert np.allclose(obj[s.squares_x - 1], [sq, 2 * sq, 0])
    assert np.allclose(obj[-1], [(s.squares_x - 1) * sq, (s.squares_y - 1) * sq, 0])
    black, markers = targets.layout(s)
    assert (0, 0) in black and len(black) == s.squares_x * s.squares_y - s.n_markers
    mid, x0, y0 = markers[0]
    gap = (sq - s.marker_mm) / 2
    assert mid == s.first_id and np.isclose(x0, sq + gap) and np.isclose(y0, gap)
    # the rendered board image agrees: top-left square black, square (1, 0) white around its marker
    n = 40
    img = targets.make_board(s).generateImage((s.squares_x * n, s.squares_y * n), marginSize=0, borderBits=1)
    assert img[n // 2, 2] == 0 and img[2, n + n // 2] == 255


def test_board_spec_validation_and_overlap():
    a = targets.BoardSpec("A", 5, 4, 16.0, 12.0, "DICT_APRILTAG_36h11", 30)
    b = targets.BoardSpec("B", 5, 4, 16.0, 12.0, "DICT_APRILTAG_36h11", 39)       # 39 shared with A
    with pytest.raises(ValueError, match="share marker ids"):
        targets.check_id_ranges([a, b])
    targets.check_id_ranges([a, targets.BoardSpec("C", 5, 4, 16.0, 12.0, "DICT_5X5_100", 30)])   # other dict: ok
    with pytest.raises(ValueError):
        targets.make_board(targets.BoardSpec("D", 11, 8, 15.0, 11.0, "DICT_5X5_100", 60))        # ids 60..103 > 99
    with pytest.raises(ValueError):
        targets.make_board(targets.BoardSpec("E", 5, 4, 16.0, 16.0, "DICT_5X5_100", 0))          # marker >= square
    assert targets.BoardSpec.from_dict(a.to_dict()) == a


def test_view_geometry_upright():
    """tilt = roll = 0: camera on the printed side, image x = board x, image y = board y."""
    T = targets.T_board_cam_looking_at((40.0, 32.0), 320.0)
    assert np.allclose(T[:3, :3], np.eye(3)) and np.allclose(T[:3, 3], [40.0, 32.0, -320.0])
    T = targets.T_board_cam_looking_at((40.0, 32.0), 300.0, tilt_deg=30.0, azimuth_deg=70.0, roll_deg=45.0)
    z = T[:3, 2]
    assert np.isclose(np.degrees(np.arccos(z[2])), 30.0)                       # tilt against the board normal
    assert np.allclose(T[:3, 3] + 300.0 * z, [40.0, 32.0, 0.0])                # optical axis through the centre


def test_pdf_page_and_png(specs, tmp_path):
    for name in ("calib", "W0"):
        s = specs[name]
        pdf = printables.board_pdf(s, tmp_path / f"{name}.pdf")
        data = pdf.read_bytes()
        assert data.startswith(b"%PDF") and len(data) > 1000
        page, pw, ph = printables.page_for(s)
        box = [float(v) for v in re.search(rb"/MediaBox\s*\[\s*([\d.\s]+)\]", data).group(1).split()]
        assert np.allclose(box[2:], [pw * 72 / 25.4, ph * 72 / 25.4], atol=0.01)          # page in points
        assert page.startswith("A4")                                                     # both boards fit A4

        size_mm = printables.board_png(s, tmp_path / f"{name}.png", px_per_mm=20.0)
        assert size_mm == ((s.squares_x + 2) * s.square_mm, (s.squares_y + 2) * s.square_mm)
        img = read_image(tmp_path / f"{name}.png")
        n = round(s.square_mm * 20.0)
        assert img.shape == ((s.squares_y + 2) * n, (s.squares_x + 2) * n)
        # the texture is detectable and has the stated scale and origin (one square of quiet zone)
        det = detect.detect_boards(img, specs, border_px=10)[name]
        assert det.n == s.n_corners
        px_per_mm = n / s.square_mm
        expect = (det.obj_pts[:, :2] + s.square_mm) * px_per_mm - 0.5          # pixel-centre convention
        assert np.abs(det.img_pts - expect).max() < 0.1


def test_pdf_geometry_raster_and_tool(specs, tmp_path):
    """The rectangles drawn into the PDF, rasterised, give every corner at its nominal position (no PDF renderer
    needed); tools/print_targets.py writes and checks PDF + PNG."""
    import importlib.util
    from mauer import REPO
    for name in ("calib", "S0"):
        s = specs[name]
        img = printables.rects_raster(s, px_per_mm=20.0)
        det = detect.detect_boards(img, specs, border_px=2)[name]
        assert det.n == s.n_corners and set(det.marker_ids) == set(s.ids)
        expect = (det.obj_pts[:, :2] + s.square_mm) * 20.0 - 0.5
        assert np.abs(det.img_pts - expect).max() < 0.5                  # < 0.025 mm
    spec = importlib.util.spec_from_file_location("print_targets", REPO / "tools" / "print_targets.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    assert tool.main(["--out", str(tmp_path / "t"), "--boards", "calib", "W3"]) == 0
    assert sorted(p.name for p in (tmp_path / "t").iterdir()) == ["W3.pdf", "W3.png", "calib.pdf", "calib.png"]
    assert tool.main(["--out", str(tmp_path / "t"), "--boards", "nope"]) == 2
