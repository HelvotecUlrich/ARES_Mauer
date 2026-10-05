"""Printable ChArUco boards: exact-scale vector PDF (reportlab) and a PNG texture for the RoboDK simulated camera.

Geometry comes from OpenCV's own CharucoBoard (targets.layout: black squares, marker positions and bits), so the
print matches what the detector expects. Research 2026-10-05 (make_target_pdf.py + verify_pdf.py, rasterised at
600 dpi): scale error -0.001 % (11x8 board) / +0.011 % (5x4 board), all corners detected.

PDF sheet: board at 100 % scale, crosshair at the board origin (top-left outer corner), x / y arrows of the OpenCV
board frame (x right, y down, z into the board), board name, dictionary, id range, nominal square / marker size,
"print at 100 %" instruction and a 100 mm scale bar to check the print. Print with "actual size" (no fit to page),
mount flat and rigid, measure the square size over the whole board and enter the measured value in station.toml.
All lengths in mm (reportlab works in points: 1 mm = 72 / 25.4 pt).
"""
from __future__ import annotations

import datetime as _dt
from pathlib import Path

import cv2
import numpy as np
from reportlab.lib.pagesizes import A0, A1, A2, A3, A4
from reportlab.lib.units import mm
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen import canvas

from .dataset import write_png
from .targets import BoardSpec, layout, make_board, marker_cells

PAGES_MM = [(n, p[0] / mm, p[1] / mm) for n, p in (("A4", A4), ("A3", A3), ("A2", A2), ("A1", A1), ("A0", A0))]
MARGIN_MM = 10.0          # page margin (most printers cannot print the outer ~5 mm)
LEFT_MM = 20.0            # room for the y arrow / crosshair left of the board
TOP_MM = 15.0             # room for the x arrow / crosshair above the board
TEXT_MM = 55.0            # text block + scale bar below the board
FONT, FONT_BOLD, FONT_PT, LINE_MM = "Helvetica", "Helvetica-Bold", 8.0, 3.6
SCALE_BAR_MM = 100.0


def page_for(spec: BoardSpec) -> tuple[str, float, float]:
    """Smallest ISO page (portrait before landscape) that holds board, marks and text: (name, width, height) mm."""
    bw, bh = spec.size_mm
    need_w = 2 * MARGIN_MM + LEFT_MM + max(bw, SCALE_BAR_MM) + 5.0
    need_h = 2 * MARGIN_MM + TOP_MM + bh + TEXT_MM
    for name, w, h in PAGES_MM:
        for orient, (pw, ph) in (("portrait", (w, h)), ("landscape", (h, w))):
            if pw >= need_w and ph >= need_h:
                return f"{name} {orient}", pw, ph
    raise ValueError(f"board {spec.name} ({bw} x {bh} mm) does not fit on A0")


def _wrap(text: str, width_mm: float, font: str, size: float) -> list[str]:
    words, lines, cur = text.split(), [], ""
    for w in words:
        t = f"{cur} {w}".strip()
        if stringWidth(t, font, size) / mm <= width_mm or not cur:
            cur = t
        else:
            lines.append(cur)
            cur = w
    return lines + ([cur] if cur else [])


def _arrow(c: canvas.Canvas, x0: float, y0: float, x1: float, y1: float, head: float = 2.5) -> None:
    """Arrow in page points from (x0, y0) to (x1, y1)."""
    c.line(x0, y0, x1, y1)
    d = np.array([x1 - x0, y1 - y0], float)
    d /= np.linalg.norm(d)
    n = np.array([-d[1], d[0]])
    tip = np.array([x1, y1])
    p = c.beginPath()
    p.moveTo(*tip)
    p.lineTo(*(tip - head * mm * d + 0.45 * head * mm * n))
    p.lineTo(*(tip - head * mm * d - 0.45 * head * mm * n))
    p.close()
    c.drawPath(p, stroke=0, fill=1)


def board_rects(spec: BoardSpec) -> list[tuple[float, float, float, float]]:
    """The black rectangles of the printed board as (x_left, y_top, width, height) [mm, board frame]: black
    squares and the black cells of every marker (horizontal runs merged). board_pdf draws exactly these;
    rects_raster rasterises them to verify the PDF geometry with the detector (no PDF renderer needed)."""
    sq, mk = spec.square_mm, spec.marker_mm
    black, markers = layout(spec)
    rects = [(i * sq, j * sq, sq, sq) for i, j in black]
    for mid, x0, y0 in markers:
        cells = marker_cells(spec.dictionary, mid)
        n = cells.shape[0]
        s = mk / n
        for r in range(n):
            k = 0
            while k < n:
                if not cells[r, k]:
                    k += 1
                    continue
                k1 = k
                while k1 < n and cells[r, k1]:
                    k1 += 1
                rects.append((x0 + k * s, y0 + r * s, (k1 - k) * s, s))
                k = k1
    return rects


def rects_raster(spec: BoardSpec, px_per_mm: float = 20.0) -> np.ndarray:
    """uint8 raster of board_rects with a white margin of one square: a pixel is black when its centre lies inside
    a rectangle. Board coordinate x maps to pixel coordinate (x + square) * px_per_mm - 0.5."""
    m = spec.square_mm
    bw, bh = spec.size_mm
    img = np.full((int(round((bh + 2 * m) * px_per_mm)), int(round((bw + 2 * m) * px_per_mm))), 255, np.uint8)
    eps = 1e-9
    for x, y, w, h in board_rects(spec):
        i0, i1 = (int(np.ceil((v + m) * px_per_mm - 0.5 - eps)) for v in (x, x + w))
        j0, j1 = (int(np.ceil((v + m) * px_per_mm - 0.5 - eps)) for v in (y, y + h))
        img[j0:j1, i0:i1] = 0
    return img


def board_pdf(spec: BoardSpec, path: str | Path) -> Path:
    """Write an exact-scale vector PDF of the board (page size from page_for)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    page_name, pw, ph = page_for(spec)
    bw, bh = spec.size_mm
    sq, mk = spec.square_mm, spec.marker_mm
    # board origin on the page (mm from the top-left page corner): board block centred horizontally
    ox = (pw - (LEFT_MM + max(bw, SCALE_BAR_MM))) / 2.0 + LEFT_MM
    oy = MARGIN_MM + TOP_MM + max(0.0, (ph - 2 * MARGIN_MM - TOP_MM - bh - TEXT_MM) / 2.0)

    def P(x: float, y: float) -> tuple[float, float]:
        """board frame (mm, y down) -> page points (y up)."""
        return (ox + x) * mm, (ph - (oy + y)) * mm

    c = canvas.Canvas(str(path), pagesize=(pw * mm, ph * mm))
    c.setTitle(f"ChArUco board {spec.name}")
    c.setAuthor("ARES_Mauer tools/print_targets.py")
    c.setSubject(spec.describe())
    c.setFillColorRGB(0, 0, 0)
    c.setStrokeColorRGB(0, 0, 0)

    for x0, y0, w, h in board_rects(spec):
        x, y = P(x0, y0 + h)                     # reportlab: lower-left corner, y up
        c.rect(x, y, w * mm, h * mm, stroke=0, fill=1)

    # origin crosshair (outside the board, along its top and left edges) and frame arrows
    c.setLineWidth(0.2 * mm)
    c.line(*P(-12.0, 0.0), *P(-1.5, 0.0))
    c.line(*P(0.0, -12.0), *P(0.0, -1.5))
    c.setLineWidth(0.3 * mm)
    _arrow(c, *P(0.0, -7.0), *P(30.0, -7.0))
    _arrow(c, *P(-7.0, 0.0), *P(-7.0, 30.0))
    c.setFont(FONT_BOLD, 9)
    c.drawString(*P(31.5, -6.0), "x")
    c.drawString(*P(-8.2, 34.5), "y")
    c.setFont(FONT, 6.5)
    c.drawString(*P(-18.0, -9.5), "origin")

    # text block and 100 mm scale bar below the board
    lines = [
        (FONT_BOLD, f"{spec.name} - ChArUco {spec.squares_x} x {spec.squares_y}, {spec.dictionary}, marker ids "
                    f"{spec.first_id}..{spec.last_id}"),
        (FONT, f"Nominal: square {sq:g} mm, marker {mk:g} mm, board {bw:g} x {bh:g} mm ({spec.n_corners} corners)."),
        (FONT_BOLD, "PRINT AT 100 % (actual size, no fit-to-page / scaling). Mount flat on a rigid plate."),
        (FONT, f"Check the scale bar with a ruler, measure the board width over all {spec.squares_x} squares "
               f"(nominal {bw:g} mm) and enter the measured square size in config/station.toml "
               "(0.1 % scale error = 0.32 mm depth error at 320 mm)."),
        (FONT, "Board frame (OpenCV 4.14 CharucoBoard): origin = top-left outer corner (crosshair), x right, "
               "y down, z into the board."),
        (FONT, f"{page_name}, generated {_dt.date.today().isoformat()} by tools/print_targets.py, "
               f"OpenCV {cv2.__version__}."),
    ]
    y = bh + 7.0
    x_text = MARGIN_MM - ox                       # text from the left page margin (board coordinates)
    width = pw - 2 * MARGIN_MM
    for font, text in lines:
        c.setFont(font, FONT_PT)
        for ln in _wrap(text, width, font, FONT_PT):
            y += LINE_MM
            c.drawString(*P(x_text, y), ln)
    y += 5.0
    c.setLineWidth(0.25 * mm)
    c.line(*P(0.0, y), *P(SCALE_BAR_MM, y))
    for k in range(11):
        t = 2.5 if k % 5 == 0 else 1.5
        c.line(*P(k * 10.0, y - t), *P(k * 10.0, y))
    c.setFont(FONT, 7)
    c.drawString(*P(0.0, y + 3.5), f"scale bar {SCALE_BAR_MM:g} mm (ticks 10 mm)")
    if oy + y + 3.5 > ph - MARGIN_MM / 2:          # layout guard (fixed text, cannot happen with page_for)
        raise RuntimeError(f"text block of {spec.name} runs off the {page_name} page")
    c.showPage()
    c.save()
    return path


def board_png(spec: BoardSpec, path: str | Path, px_per_mm: float = 20.0) -> tuple[float, float]:
    """PNG texture of the board with a white quiet zone of one square on every side (for RoboDK).

    The pixels per square are rounded to an integer, so the real resolution is round(square * px_per_mm) / square.
    Returns the size of the PNG content incl. the margin, (width_mm, height_mm) = ((nx + 2) * square,
    (ny + 2) * square); the board origin sits at (square, square) mm from the image's top-left corner."""
    n = int(round(spec.square_mm * px_per_mm))
    if n < 8:
        raise ValueError(f"px_per_mm {px_per_mm} too small ({n} px per square)")
    img = make_board(spec).generateImage(((spec.squares_x + 2) * n, (spec.squares_y + 2) * n), marginSize=n,
                                         borderBits=1)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, img)
    return (spec.squares_x + 2) * spec.square_mm, (spec.squares_y + 2) * spec.square_mm
