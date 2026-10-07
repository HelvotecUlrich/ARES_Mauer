"""
odom_map.py - Odometry map (spec 6.6, section 9): trail of (fPosX_m, fPosY_m), robot outline 1.12 x 0.60 m rotated
by fPosTheta_deg, target marker of the running move, grid 0.5 m.

Drawn PHYSICALLY oriented, seen from above, through the config flags (frame.physical_pose):
    screen right = PLC +X (forward at the odometry origin)
    screen up    = physical left:  screen_y       = (+1 if plus_y_is_left else -1) * y_plc
    heading      = physical CCW:   screen_heading = (+1 if plus_omega_is_ccw else -1) * theta_plc
The axis labels show this mapping. Trail, pose and target are STORED in the PLC frame (as read / as the PLC
measures the move) and only transformed for drawing.
Note: if exactly one flag differs from the PLC documentation (+Y left, +omega CCW), the PLC odometry frame is
mirrored against its rotation sense; single straight moves and rotations are drawn correctly, but the position
integrated after a rotation is mirrored by the PLC itself (not an HMI effect) - the direction tests decide.
"Reset pose" is locked while a relative move runs (the PLC defers a reset until the move has ended).
Repaints only when the pose / trail / target changed (called from the status update, no own timer).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from .. import frame as F
from . import constants as C
from .widgets import PulseBtn, lbl, set_text

TRAIL_MIN_STEP_M = 0.005     # append a trail point after > 5 mm
TRAIL_MAX_POINTS = 20000
JUMP_RESET_M = 0.5           # pose jump within one poll (e.g. odometry reset) -> new trail
GRID_M = 0.5

Pose = Tuple[float, float, float]


def axis_labels(frame_cfg: F.FrameConfig) -> Tuple[str, str]:
    """(horizontal, vertical) axis captions of the map, e.g. ('forward = PLC +X', 'left = PLC -Y (config, ...)')."""
    y_sign = "+" if frame_cfg.plus_y_is_left else "-"
    return ("forward = PLC +X",
            f"left = PLC {y_sign}Y (config, {frame_cfg.status_word})")


def rotation_label(frame_cfg: F.FrameConfig) -> str:
    """Caption of the rotation sense, e.g. 'seen from above, CCW = PLC +theta (config, unverified)'."""
    t_sign = "+" if frame_cfg.plus_omega_is_ccw else "-"
    return f"seen from above, CCW = PLC {t_sign}theta (config, {frame_cfg.status_word})"


class OdomMap(QWidget):
    """QPainter map of the odometry pose (data in the PLC frame, drawn physically oriented)."""

    def __init__(self, parent: Optional[QWidget] = None, frame_cfg: Optional[F.FrameConfig] = None) -> None:
        super().__init__(parent)
        self._frame = frame_cfg if frame_cfg is not None else F.FrameConfig()
        self._trail: Deque[Tuple[float, float]] = deque(maxlen=TRAIL_MAX_POINTS)
        self._pose: Pose = (0.0, 0.0, 0.0)
        self._target: Optional[Pose] = None
        self._note = ""
        self.setMinimumSize(260, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    # ── data ──────────────────────────────────────────────────────────────────
    @property
    def trail(self) -> Deque[Tuple[float, float]]:
        return self._trail

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def target(self) -> Optional[Pose]:
        return self._target

    @property
    def frame(self) -> F.FrameConfig:
        return self._frame

    def screen_pose(self, pose: Pose) -> Pose:
        """PLC pose (x, y, theta_deg) -> drawn pose (forward, left, CCW heading) with the config flags."""
        return F.physical_pose(pose[0], pose[1], pose[2], self._frame)

    def set_pose(self, x: float, y: float, theta_deg: float) -> None:
        new = (float(x), float(y), float(theta_deg))
        if new == self._pose and self._trail:
            return
        if self._trail:
            lx, ly = self._trail[-1]
            step = math.hypot(new[0] - lx, new[1] - ly)
            if step > JUMP_RESET_M:
                self._trail.clear()
                self._trail.append((new[0], new[1]))
            elif step > TRAIL_MIN_STEP_M:
                self._trail.append((new[0], new[1]))
        else:
            self._trail.append((new[0], new[1]))
        self._pose = new
        self.update()

    def set_target(self, target: Optional[Pose]) -> None:
        if target != self._target:
            self._target = target
            self.update()

    def set_note(self, note: str) -> None:
        if note != self._note:
            self._note = note
            self.update()

    def clear_trail(self) -> None:
        self._trail.clear()
        self._trail.append((self._pose[0], self._pose[1]))
        self.update()

    # ── drawing ───────────────────────────────────────────────────────────────
    def _bounds(self) -> Tuple[float, float, float, float]:
        """View bounds in drawn (physical) coordinates."""
        pts = [self.screen_pose((x, y, 0.0)) for x, y in self._trail]
        pts.append(self.screen_pose(self._pose))
        if self._target is not None:
            pts.append(self.screen_pose(self._target))
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        r = C.ROBOT_LENGTH_M * 0.75
        x0, x1 = min(xs) - r, max(xs) + r
        y0, y1 = min(ys) - r, max(ys) + r
        # minimum view 3 m x 3 m around the content
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        w = max(x1 - x0, 3.0)
        h = max(y1 - y0, 3.0)
        return cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2

    def paintEvent(self, _event: Any) -> None:  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#141414"))
        W, H = self.width(), self.height()
        margin = 28
        x0, x1, y0, y1 = self._bounds()
        scale = min((W - 2 * margin) / (x1 - x0), (H - 2 * margin) / (y1 - y0))   # px per m, fixed aspect
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

        def to_px(x: float, y: float) -> QPointF:
            """Drawn (forward, left) coordinates -> pixels (forward to the right, left up)."""
            return QPointF(W / 2 + (x - cx) * scale, H / 2 - (y - cy) * scale)

        # grid
        vx0 = cx - (W / 2) / scale
        vx1 = cx + (W / 2) / scale
        vy0 = cy - (H / 2) / scale
        vy1 = cy + (H / 2) / scale
        p.setPen(QPen(QColor("#262626"), 1))
        gx = math.floor(vx0 / GRID_M) * GRID_M
        while gx <= vx1:
            p.drawLine(to_px(gx, vy0), to_px(gx, vy1))
            gx += GRID_M
        gy = math.floor(vy0 / GRID_M) * GRID_M
        while gy <= vy1:
            p.drawLine(to_px(vx0, gy), to_px(vx1, gy))
            gy += GRID_M
        # axes through the odometry origin, captions show the config mapping
        p.setPen(QPen(QColor("#555555"), 1))
        p.drawLine(to_px(vx0, 0.0), to_px(vx1, 0.0))
        p.drawLine(to_px(0.0, vy0), to_px(0.0, vy1))
        p.setPen(QColor("#AAAAAA"))
        h_txt, v_txt = axis_labels(self._frame)
        p.drawText(QRectF(W - 250, H / 2 - (0 - cy) * scale - 18, 245, 16), Qt.AlignRight, h_txt + " \u2192")
        ox = W / 2 + (0 - cx) * scale
        p.drawText(QRectF(ox + 4, 4, W - ox - 8, 16), Qt.AlignLeft, "\u2191 " + v_txt)
        p.setPen(QColor("#777777"))
        p.drawText(QRectF(6, H - 38, W - 12, 16), Qt.AlignLeft, rotation_label(self._frame))

        # trail
        if len(self._trail) > 1:
            p.setPen(QPen(QColor("#44AAFF"), 2))
            pts = []
            for x, y in self._trail:
                sx, sy, _ = self.screen_pose((x, y, 0.0))
                pts.append(to_px(sx, sy))
            p.drawPolyline(QPolygonF(pts))

        # target
        if self._target is not None:
            tx, ty, tth = self.screen_pose(self._target)
            c = to_px(tx, ty)
            p.setPen(QPen(QColor("#FFAA00"), 2))
            s = 8
            p.drawLine(QPointF(c.x() - s, c.y() - s), QPointF(c.x() + s, c.y() + s))
            p.drawLine(QPointF(c.x() - s, c.y() + s), QPointF(c.x() + s, c.y() - s))
            t = math.radians(tth)
            p.drawLine(c, to_px(tx + 0.3 * math.cos(t), ty + 0.3 * math.sin(t)))

        # robot outline (length along robot forward), drawn with the physical CCW heading
        x, y, th = self.screen_pose(self._pose)
        t = math.radians(th)
        ct, st = math.cos(t), math.sin(t)
        hl, hw = C.ROBOT_LENGTH_M / 2, C.ROBOT_WIDTH_M / 2
        corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
        poly = QPolygonF([to_px(x + a * ct - b * st, y + a * st + b * ct) for a, b in corners])
        p.setPen(QPen(QColor("#DDDDDD"), 2))
        p.setBrush(QBrush(QColor(80, 160, 80, 70)))
        p.drawPolygon(poly)
        p.setPen(QPen(QColor("#55DD55"), 3))
        p.drawLine(to_px(x, y), to_px(x + hl * ct, y + hl * st))   # heading (front)
        p.setBrush(Qt.NoBrush)

        # scale / note
        p.setPen(QColor("#888888"))
        p.drawText(QRectF(6, H - 20, W - 12, 16), Qt.AlignLeft, f"grid {GRID_M:.1f} m")
        if self._note:
            p.setPen(QColor("#FFAA00"))
            p.drawText(QRectF(6, 24, W - 12, 40), Qt.AlignLeft | Qt.TextWordWrap, self._note)
        p.end()


class OdomPanel(QWidget):
    """Map + pose readout + 'Reset pose' (bCmdOdomReset pulse, locked during a move) + 'Clear trail'."""

    RESET_LOCK_HINT = "Reset pose is locked while a relative move runs."

    def __init__(self, worker: Any, parent: Optional[QWidget] = None,
                 frame_cfg: Optional[F.FrameConfig] = None) -> None:
        super().__init__(parent)
        self._worker = worker
        self._frame = frame_cfg if frame_cfg is not None else F.FrameConfig()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        self.map = OdomMap(frame_cfg=self._frame)
        lay.addWidget(self.map, 1)
        self._pose_lbl = QLabel("PLC odom: x - m   y - m   theta - deg")
        self._pose_lbl.setStyleSheet("color:#CCCCCC; font-family: monospace;")
        self._pose_lbl.setWordWrap(True)
        lay.addWidget(self._pose_lbl)
        row = QHBoxLayout()
        self._btn_reset = PulseBtn("Reset pose", "#1A3A5A", min_w=110)
        self._btn_reset.setToolTip("bCmdOdomReset pulse: odometry pose := 0 (also clears the trail).\n"
                                   "Locked while a relative move runs (the PLC defers a reset until the move ends).")
        self._btn_reset.clicked.connect(self._reset_pose)
        self._btn_clear = PulseBtn("Clear trail", "#333333", min_w=110)
        self._btn_clear.clicked.connect(self.map.clear_trail)
        self._reset_hint = lbl("", "#FFAA00")
        self._reset_hint.setWordWrap(True)
        row.addWidget(self._btn_reset)
        row.addWidget(self._btn_clear)
        row.addWidget(self._reset_hint, 1)
        lay.addLayout(row)
        self._available = True
        self._move_active = False

    @property
    def reset_allowed(self) -> bool:
        return self._available and not self._move_active

    def _reset_pose(self) -> None:
        if not self.reset_allowed:          # defensive: a queued click after the move started
            set_text(self._reset_hint, self.RESET_LOCK_HINT if self._move_active else "")
            return
        self._worker.pulse({}, ("bCmdOdomReset",), "odometry reset")
        self.map.set_target(None)
        self.map.clear_trail()

    def _apply_reset_enable(self) -> None:
        self._btn_reset.setEnabled(self.reset_allowed)
        set_text(self._reset_hint, self.RESET_LOCK_HINT if (self._available and self._move_active) else "")

    def set_available(self, available: bool) -> None:
        self._available = available
        self._apply_reset_enable()
        self.map.set_note("" if available else
                          "Odometry not available: PLC interface v1 (load PLC build v2).")

    def update_status(self, data: Dict[str, Any]) -> None:
        self._move_active = bool(data.get("bMoveActive", False))
        self._apply_reset_enable()
        x = float(data.get("fPosX_m", 0.0))
        y = float(data.get("fPosY_m", 0.0))
        th = float(data.get("fPosTheta_deg", 0.0))
        self.map.set_pose(x, y, th)
        dist = data.get("fOdomDist_m")
        extra = f"   path {float(dist):.3f} m" if dist is not None else ""
        glitch = data.get("nOdomGlitchCnt")
        if glitch:
            extra += f"   glitches {int(glitch)}"
        side = f" ({F.physical_y_name(y, self._frame)})" if abs(y) >= 0.0005 else ""
        rot = f" ({F.physical_rot_name(th, self._frame)})" if abs(th) >= 0.005 else ""
        set_text(self._pose_lbl, f"PLC odom: x {x:+.3f} m   y {y:+.3f} m{side}   theta {th:+.2f} deg{rot}{extra}")
