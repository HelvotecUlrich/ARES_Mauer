"""Plan view: the job in the wall frame (docs/HMI_DESIGN.md section 10.3) - stones (planned / placed / current), the
stops with a ghost ARES footprint, the routes, the floor obstacles (legs, plates, station table), and the ARES pose:
the sequencer's estimate coloured by its status, the SIM truth dashed, the live REAL pose during a move.

plan_geometry() is pure (tested); PlanView only paints it (QPainter, repaint through update()).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from mauer import floor
from mauer.job import Job
from mauer.reference import Pose2D
from mauer.vision.targets import board_specs

Pt = tuple[float, float]
GRID_MM = 500.0
MARGIN_MM = 300.0
STATUS_COLOURS = {"ok": "#44CC44", "odometry": "#FFAA00", "unknown": "#FF4444"}
COURSE_COLOURS = ("#B8743C", "#C98A50", "#A86634", "#D29A62", "#9A5A2C", "#DDA874")


@dataclass(frozen=True)
class PlanGeometry:
    stones: dict                              # key -> polygon [(x, y)] [mm, wall frame]
    course: dict                              # key -> course
    order: tuple                              # stone keys in job order
    stops: tuple                              # Pose2D per stop
    routes: tuple                             # (kind, [Pose2D]) per route ("stop" | "to_station" | "from_station")
    obstacles: tuple                          # (name, kind, polygon)
    table: tuple | None                       # station table polygon
    dock: Pose2D
    ares: floor.AresShape
    bounds: tuple                             # (x0, x1, y0, y1) [mm]
    notes: tuple = field(default=())

    def footprint(self, p: Pose2D) -> list[Pt]:
        return self.ares.footprint(p)


def plan_geometry(job: Job, cfg: Mapping) -> PlanGeometry:
    """Everything PlanView draws, in the wall frame [mm]."""
    b, hb = cfg.get("brick", {}) or {}, cfg.get("half_brick", {}) or {}
    width = float(b.get("width", 120.0))
    full = float(b.get("length", 200.0))
    stones, course = {}, {}
    for t in job.stones():
        T = np.asarray(t.T_wall_tcp, float)
        length = float(t.length_mm or (hb.get("length", full / 2.0) if t.kind == "half" else full))
        stones[t.key] = floor.rect_poly(T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0]), length, width)
        course[t.key] = int(t.course)
    routes = []
    for s in job.stops:
        for kind, r in (("stop", s.route), ("to_station", s.route_to_station), ("from_station", s.route_from_station)):
            if r:
                routes.append((kind, list(r)))
    notes = []
    obstacles: list = []
    table = None
    try:
        if job.station.slots or job.station.boards:      # a job without a station (magazine dry run): no table
            table = tuple(floor.station_table_poly(cfg, job.station.T_wall_station))
    except (KeyError, TypeError, ValueError) as e:
        notes.append(f"station table not drawn: {e}")
    if job.legs:
        try:
            specs = board_specs(cfg)
            obs = floor.job_obstacles(cfg, job.legs, job.station.T_wall_station,
                                      {n: sp.size_mm for n, sp in specs.items()})
            obstacles = [(o.name, o.kind, tuple(o.poly)) for o in obs if o.kind != "table"]
        except (KeyError, TypeError, ValueError) as e:
            notes.append(f"floor model not drawn: {e}")
    ares = floor.AresShape.from_config(cfg) if "ares" in cfg else floor.AresShape()
    dock = job.station.dock_in_wall
    pts: list[Pt] = [p for poly in stones.values() for p in poly]
    for p in [*(s.ares for s in job.stops), dock]:
        pts += ares.footprint(p)
    for _, r in routes:
        pts += [(w.x_mm, w.y_mm) for w in r]
    for _, _, poly in obstacles:
        pts += list(poly)
    if table:
        pts += list(table)
    xs, ys = [p[0] for p in pts] or [0.0], [p[1] for p in pts] or [0.0]
    bounds = (min(xs) - MARGIN_MM, max(xs) + MARGIN_MM, min(ys) - MARGIN_MM, max(ys) + MARGIN_MM)
    return PlanGeometry(stones, course, tuple(t.key for t in job.stones()), tuple(s.ares for s in job.stops),
                        tuple(routes), tuple(obstacles), table, dock, ares, bounds, tuple(notes))


class PlanView(QWidget):
    """QPainter plan in the wall frame (x right, y up), fitted to the job; set_session / set_snapshot / set_live."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._geo: PlanGeometry | None = None
        self._snap = None
        self._live = None
        self.show_routes = True
        self.show_stops = True
        self.setMinimumSize(320, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    @property
    def geometry_(self) -> PlanGeometry | None:
        return self._geo

    def set_session(self, session) -> None:
        self._geo = plan_geometry(session.job, session.cfg) if session is not None else None
        self._snap, self._live = None, None
        self.update()

    def set_snapshot(self, snap) -> None:
        self._snap = snap
        self.update()

    def set_live(self, live) -> None:
        self._live = live
        self.update()

    # ── painting ─────────────────────────────────────────────────────────────
    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#141414"))
        geo = self._geo
        if geo is None:
            p.setPen(QColor("#888888"))
            p.drawText(self.rect(), Qt.AlignCenter, "no job loaded")
            p.end()
            return
        W, H = self.width(), self.height()
        x0, x1, y0, y1 = geo.bounds
        scale = min((W - 20) / max(x1 - x0, 1.0), (H - 20) / max(y1 - y0, 1.0))
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

        def px(x: float, y: float) -> QPointF:
            return QPointF(W / 2 + (x - cx) * scale, H / 2 - (y - cy) * scale)

        def poly(points) -> QPolygonF:
            return QPolygonF([px(x, y) for x, y in points])

        # grid
        p.setPen(QPen(QColor("#242424"), 1))
        vx0, vx1 = cx - W / 2 / scale, cx + W / 2 / scale
        vy0, vy1 = cy - H / 2 / scale, cy + H / 2 / scale
        g = math.floor(vx0 / GRID_MM) * GRID_MM
        while g <= vx1:
            p.drawLine(px(g, vy0), px(g, vy1))
            g += GRID_MM
        g = math.floor(vy0 / GRID_MM) * GRID_MM
        while g <= vy1:
            p.drawLine(px(vx0, g), px(vx1, g))
            g += GRID_MM
        # obstacles and station table
        p.setPen(QPen(QColor("#5A5A5A"), 1))
        p.setBrush(QBrush(QColor(90, 90, 90, 60)))
        for _, _, pl in geo.obstacles:
            p.drawPolygon(poly(pl))
        if geo.table:
            p.setBrush(QBrush(QColor(70, 90, 120, 70)))
            p.drawPolygon(poly(geo.table))
        # routes
        if self.show_routes:
            colours = {"stop": "#3A6A9A", "to_station": "#6A5A2A", "from_station": "#5A6A2A"}
            for kind, r in geo.routes:
                p.setPen(QPen(QColor(colours.get(kind, "#555555")), 1))
                p.drawPolyline(QPolygonF([px(w.x_mm, w.y_mm) for w in r]))
        # stops: ghost ARES footprints with the index
        if self.show_stops:
            p.setBrush(Qt.NoBrush)
            for k, s in enumerate(geo.stops):
                p.setPen(QPen(QColor("#3C3C3C"), 1, Qt.DashLine))
                p.drawPolygon(poly(geo.footprint(s)))
                p.setPen(QColor("#777777"))
                c = px(s.x_mm, s.y_mm)
                p.drawText(QRectF(c.x() - 20, c.y() - 8, 40, 16), Qt.AlignCenter, str(k))
            p.setPen(QPen(QColor("#4A4A2A"), 1, Qt.DotLine))
            p.drawPolygon(poly(geo.footprint(geo.dock)))
        # stones
        snap = self._snap
        placed = snap.placed if snap is not None else frozenset()
        current = snap.stone.key if snap is not None and snap.stone is not None else None
        for key in geo.order:
            pl = geo.stones[key]
            if key in placed:
                col = QColor(COURSE_COLOURS[geo.course[key] % len(COURSE_COLOURS)])
                p.setPen(QPen(QColor("#E0C090"), 1))
                p.setBrush(QBrush(col))
            else:
                p.setPen(QPen(QColor("#7A6A5A"), 1))
                p.setBrush(Qt.NoBrush)
            if key == current:
                p.setPen(QPen(QColor("#FFDD33"), 3))
            p.drawPolygon(poly(pl))
        p.setBrush(Qt.NoBrush)
        # ARES: SIM truth dashed, estimate solid (status colour), live pose during a REAL move
        live = self._live
        if live is not None and live.true_pose is not None:
            p.setPen(QPen(QColor("#AAAAFF"), 1, Qt.DashLine))
            p.drawPolygon(poly(geo.footprint(live.true_pose)))
        est = snap.pose_est if snap is not None else None
        status = snap.pose_status if snap is not None else "ok"
        if live is not None and live.src == "estimate+odometry" and live.pose is not None:
            self._draw_ares(p, px, poly, geo, live.pose, "#66CCFF", 2)
        if est is not None:
            self._draw_ares(p, px, poly, geo, est, STATUS_COLOURS.get(status, "#FF4444"), 2)
        # caption
        p.setPen(QColor("#888888"))
        cap = f"wall frame, grid {GRID_MM / 1000:.1f} m"
        if snap is not None:
            cap += f" | placed {len(placed)}/{snap.n_stones}"
            if est is not None:
                cap += f" | ARES {status} ({snap.pose_src})"
        p.drawText(QRectF(6, H - 20, W - 12, 16), Qt.AlignLeft, cap)
        p.end()

    @staticmethod
    def _draw_ares(p: QPainter, px, poly, geo: PlanGeometry, pose: Pose2D, colour: str, width: int) -> None:
        p.setPen(QPen(QColor(colour), width))
        p.setBrush(QBrush(QColor(QColor(colour).red(), QColor(colour).green(), QColor(colour).blue(), 40)))
        p.drawPolygon(poly(geo.footprint(pose)))
        p.setBrush(Qt.NoBrush)
        hl = geo.ares.length / 2
        p.drawLine(px(pose.x_mm, pose.y_mm), px(pose.x_mm + hl * math.cos(pose.theta_rad),
                                                pose.y_mm + hl * math.sin(pose.theta_rad)))
