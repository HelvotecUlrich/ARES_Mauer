"""ChArUco board definitions: the calibration board and the reference boards at the wall and the pick-up station.

All boards come from config/station.toml: `[boards.calib]` is the intrinsics / hand-eye board (spec name "calib"),
every `[[targets]]` entry is a reference board with the `[boards.ref]` geometry and its own marker id range starting
at `first_id` (spec name = target name, e.g. "W0", "S1").

Board frame = OpenCV 4.14 CharucoBoard frame (checked 2026-10-05 with OpenCV 4.14.0, see tests/test_vision_targets.py):
origin at the top-left OUTER corner of the printed board, x to the right, y down, z INTO the board. The top-left
square is black, the markers sit in the white squares; chessboard corner 0 is at (square, square, 0) and the corner
ids run row by row. Marker ids of a board are first_id .. first_id + n_markers - 1, row-major over the white squares.
Lengths in mm. `setLegacyPattern(False)` always (the OpenCV >= 4.6 pattern; boards printed by tools/print_targets.py).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache
from typing import Mapping

import cv2
import numpy as np

from .. import geometry as g


@dataclass(frozen=True)
class BoardSpec:
    """Geometry of one ChArUco board (frozen, hashable -> usable as a cache key).

    square_mm / marker_mm are the PRINTED sizes: enter the measured values in station.toml (a 0.1 % scale error gives
    0.1 % depth error, 0.32 mm at 320 mm). marker_mm = black square edge of the marker (OpenCV / AprilTag definition).
    """
    name: str
    squares_x: int
    squares_y: int
    square_mm: float
    marker_mm: float
    dictionary: str          # cv2.aruco predefined dictionary name, e.g. "DICT_5X5_100", "DICT_APRILTAG_36h11"
    first_id: int = 0

    @property
    def n_markers(self) -> int:
        """Markers on the board = white squares (top-left square is black)."""
        return (self.squares_x * self.squares_y) // 2

    @property
    def ids(self) -> np.ndarray:
        """Marker ids first_id .. first_id + n_markers - 1 (int32)."""
        return np.arange(self.first_id, self.first_id + self.n_markers, dtype=np.int32)

    @property
    def last_id(self) -> int:
        return self.first_id + self.n_markers - 1

    @property
    def n_corners(self) -> int:
        """Inner chessboard (ChArUco) corners."""
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def size_mm(self) -> tuple[float, float]:
        """Board width (x) and height (y) [mm], outer edge of the chessboard."""
        return self.squares_x * self.square_mm, self.squares_y * self.square_mm

    @property
    def centre_mm(self) -> tuple[float, float]:
        """Board centre in the board frame [mm] (z = 0)."""
        w, h = self.size_mm
        return w / 2.0, h / 2.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Mapping) -> "BoardSpec":
        return cls(name=str(d["name"]), squares_x=int(d["squares_x"]), squares_y=int(d["squares_y"]),
                   square_mm=float(d["square_mm"]), marker_mm=float(d["marker_mm"]),
                   dictionary=str(d["dictionary"]), first_id=int(d.get("first_id", 0)))

    def describe(self) -> str:
        w, h = self.size_mm
        return (f"{self.name}: {self.squares_x}x{self.squares_y} squares of {self.square_mm:g} mm, markers "
                f"{self.marker_mm:g} mm {self.dictionary} ids {self.first_id}..{self.last_id}, board {w:g} x {h:g} mm")


# ── dictionaries and boards ──────────────────────────────────────────────────
@lru_cache(maxsize=None)
def dictionary(name: str) -> cv2.aruco.Dictionary:
    """cv2.aruco predefined dictionary by name ("DICT_5X5_100", "DICT_APRILTAG_36h11", ...)."""
    if not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary {name!r}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def dictionary_size(name: str) -> int:
    """Number of markers in a predefined dictionary (DICT_5X5_100: 100, DICT_APRILTAG_36h11: 587)."""
    return int(dictionary(name).bytesList.shape[0])


def validate(spec: BoardSpec) -> None:
    """Raise ValueError for a geometrically impossible board or an id range outside the dictionary."""
    if spec.squares_x < 2 or spec.squares_y < 2:
        raise ValueError(f"{spec.name}: need at least 2 x 2 squares")
    if not 0.0 < spec.marker_mm < spec.square_mm:
        raise ValueError(f"{spec.name}: marker_mm must be in (0, square_mm)")
    if spec.first_id < 0 or spec.last_id >= dictionary_size(spec.dictionary):
        raise ValueError(f"{spec.name}: ids {spec.first_id}..{spec.last_id} outside {spec.dictionary} "
                         f"(0..{dictionary_size(spec.dictionary) - 1})")


def make_board(spec: BoardSpec) -> cv2.aruco.CharucoBoard:
    """OpenCV CharucoBoard with the explicit marker ids first_id .. first_id + n_markers - 1 (new pattern)."""
    validate(spec)
    board = cv2.aruco.CharucoBoard((spec.squares_x, spec.squares_y), float(spec.square_mm), float(spec.marker_mm),
                                   dictionary(spec.dictionary), spec.ids)
    board.setLegacyPattern(False)
    return board


def corners_obj(spec: BoardSpec) -> np.ndarray:
    """(N, 3) chessboard corners in the board frame [mm], index = ChArUco corner id."""
    return np.asarray(make_board(spec).getChessboardCorners(), float).reshape(-1, 3)


def layout(spec: BoardSpec) -> tuple[list[tuple[int, int]], list[tuple[int, float, float]]]:
    """Printable geometry, taken from OpenCV's own board (not re-derived):
    black squares as (column i, row j) and markers as (id, x_left_mm, y_top_mm) in the board frame."""
    board = make_board(spec)
    sq = spec.square_mm
    white, markers = set(), []
    for corners, mid in zip(board.getObjPoints(), board.getIds().ravel()):
        c = np.asarray(corners, float).reshape(4, 3)
        centre = c[:, :2].mean(axis=0)
        white.add((int(centre[0] // sq), int(centre[1] // sq)))
        markers.append((int(mid), float(c[:, 0].min()), float(c[:, 1].min())))
    black = [(i, j) for j in range(spec.squares_y) for i in range(spec.squares_x) if (i, j) not in white]
    return black, markers


def marker_cells(dictionary_name: str, marker_id: int, border_bits: int = 1) -> np.ndarray:
    """Bit grid of a marker incl. its black border, True = black cell, shape (n, n) with n = markerSize + 2."""
    d = dictionary(dictionary_name)
    n = d.markerSize + 2 * border_bits
    return cv2.aruco.generateImageMarker(d, int(marker_id), n, borderBits=border_bits) < 128


# ── config ────────────────────────────────────────────────────────────────────
def board_specs(cfg: Mapping) -> dict[str, BoardSpec]:
    """{"calib": [boards.calib], <target name>: [boards.ref] geometry + target first_id, ...} from station.toml.

    Raises ValueError for duplicate names or overlapping id ranges of boards with the same dictionary (a marker id
    must identify exactly one board)."""
    boards = cfg["boards"]
    c = boards["calib"]
    specs = {"calib": BoardSpec("calib", int(c["squares_x"]), int(c["squares_y"]), float(c["square_mm"]),
                                float(c["marker_mm"]), str(c["dictionary"]), int(c.get("first_id", 0)))}
    r = boards["ref"]
    for t in cfg.get("targets", []):
        name = str(t["name"])
        if name in specs:
            raise ValueError(f"duplicate board name {name!r}")
        specs[name] = BoardSpec(name, int(r["squares_x"]), int(r["squares_y"]), float(r["square_mm"]),
                                float(r["marker_mm"]), str(r["dictionary"]), int(t["first_id"]))
    check_id_ranges(specs.values())
    return specs


def check_id_ranges(specs) -> None:
    """ValueError if two boards of the same dictionary share a marker id (or a board is invalid)."""
    seen: dict[str, list[BoardSpec]] = {}
    for s in specs:
        validate(s)
        for o in seen.get(s.dictionary, []):
            if s.first_id <= o.last_id and o.first_id <= s.last_id:
                raise ValueError(f"boards {o.name} ({o.first_id}..{o.last_id}) and {s.name} "
                                 f"({s.first_id}..{s.last_id}) share marker ids of {s.dictionary}")
        seen.setdefault(s.dictionary, []).append(s)


# ── viewing geometry (used by synth tests and hand-eye pose planning) ────────
def T_board_cam_looking_at(centre_mm, dist_mm: float, tilt_deg: float = 0.0, azimuth_deg: float = 0.0,
                           roll_deg: float = 0.0) -> np.ndarray:
    """OpenCV camera pose in the board frame, optical axis through the board point `centre_mm` (x, y[, z]).

    The camera sits on the printed side (board -z) at `dist_mm` from that point; `tilt_deg` = angle between the
    optical axis and the board normal (+z), `azimuth_deg` = direction of the tilt about the normal (0 = camera
    displaced towards board -x), `roll_deg` = rotation about the optical axis. tilt = roll = 0 gives an upright view:
    image x = board x, image y = board y (origin top-left in the image)."""
    c = np.zeros(3)
    cm = np.asarray(centre_mm, float).ravel()
    c[:len(cm)] = cm
    t, a = np.radians(tilt_deg), np.radians(azimuth_deg)
    z = np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)])   # viewing direction, into the board
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z                                  # board x projected normal to z
    if np.linalg.norm(x) < 1e-9:                                              # cannot happen for tilt < 90 deg
        x = np.array([0.0, 1.0, 0.0]) - z[1] * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z]) @ g.rotz(np.radians(roll_deg))[:3, :3]
    return g.make_T(R, c - dist_mm * z)
