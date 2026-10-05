"""Rigid-body helpers on 4x4 numpy matrices (float64, mm, rad).

Naming: T_a_b is the pose of frame b expressed in frame a, so p_a = T_a_b @ p_b and T_a_c = T_a_b @ T_b_c.
UR poses are lists [x, y, z, rx, ry, rz] in metres and a rotation vector (axis * angle, rad), as URScript and RTDE
use them. Never average or compare rotation vectors component-wise – convert to matrices or quaternions first.
"""
from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np


# ── constructors ──────────────────────────────────────────────────────────────
def transl(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def rotx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[1:3, 1:3] = [[c, -s], [s, c]]
    return T


def roty(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    return T


def rotz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def make_T(R: np.ndarray, t: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def split_T(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:3, :3].copy(), T[:3, 3].copy()


def inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def pose_xyz_rpy(xyz_mm: Sequence[float], rpy_deg: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """transl(xyz) @ rotz(yaw) @ roty(pitch) @ rotx(roll) – roll/pitch/yaw about the fixed x, y, z axes."""
    r, p, y = np.radians(np.asarray(rpy_deg, float))
    return transl(*xyz_mm) @ rotz(y) @ roty(p) @ rotx(r)


def xyz_rpy(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of pose_xyz_rpy: (xyz [mm], rpy [deg]); pitch in [-90, 90] deg."""
    R = T[:3, :3]
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(R[2, 0]) < 1.0 - 1e-12:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: put everything into yaw
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return T[:3, 3].copy(), np.degrees([roll, pitch, yaw])


# ── rotations ─────────────────────────────────────────────────────────────────
def R_to_quat(R: np.ndarray) -> np.ndarray:
    """Unit quaternion [w, x, y, z] with w >= 0 (Shepperd's method, robust at all angles)."""
    R = np.asarray(R, float)
    tr = np.trace(R)
    cands = [tr, R[0, 0], R[1, 1], R[2, 2]]
    i = int(np.argmax(cands))
    if i == 0:
        s = 2.0 * np.sqrt(max(1.0 + tr, 0.0))
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif i == 1:
        s = 2.0 * np.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 0.0))
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif i == 2:
        s = 2.0 * np.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 0.0))
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 0.0))
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def quat_to_R(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotvec_to_R(v: Sequence[float]) -> np.ndarray:
    """Rotation vector (axis * angle, rad) -> 3x3 matrix (Rodrigues)."""
    v = np.asarray(v, float).reshape(3)
    a = np.linalg.norm(v)
    if a < 1e-12:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + K
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


def R_to_rotvec(R: np.ndarray) -> np.ndarray:
    """3x3 matrix -> rotation vector with angle in [0, pi] (via the quaternion, robust near 0 and pi)."""
    q = R_to_quat(R)
    s = np.linalg.norm(q[1:])
    if s < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(s, q[0])
    return q[1:] / s * angle


def angle_of(R: np.ndarray) -> float:
    """Rotation angle of a 3x3 matrix [rad], 0..pi."""
    return float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


# ── UR poses ──────────────────────────────────────────────────────────────────
def ur_to_T(p: Sequence[float]) -> np.ndarray:
    """UR pose [x, y, z (m), rx, ry, rz (rad rotation vector)] -> 4x4 in mm."""
    p = np.asarray(p, float).reshape(6)
    return make_T(rotvec_to_R(p[3:]), p[:3] * 1000.0)


def T_to_ur(T: np.ndarray) -> list[float]:
    """4x4 in mm -> UR pose [m, rad rotation vector]."""
    return [*(T[:3, 3] / 1000.0).tolist(), *R_to_rotvec(T[:3, :3]).tolist()]


def ur_str(p: Sequence[float]) -> str:
    """URScript pose literal p[x, y, z, rx, ry, rz] with enough digits (0.1 µm, 1e-9 rad)."""
    return "p[" + ", ".join(f"{v:.9f}" for v in p) + "]"


# ── comparison and averaging ─────────────────────────────────────────────────
def pose_delta(T_a: np.ndarray, T_b: np.ndarray) -> tuple[float, float]:
    """(translation distance [mm], rotation angle [deg]) between two poses of the same frame."""
    D = inv(T_a) @ T_b
    return float(np.linalg.norm(D[:3, 3])), float(np.degrees(angle_of(D[:3, :3])))


def average_T(Ts: Iterable[np.ndarray]) -> np.ndarray:
    """Mean pose: arithmetic mean of the positions, rotation = principal eigenvector of sum(q q^T) (Markley)."""
    Ts = list(Ts)
    if not Ts:
        raise ValueError("average_T of an empty list")
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    M = np.zeros((4, 4))
    for T in Ts:
        q = R_to_quat(T[:3, :3])
        M += np.outer(q, q)
    w, V = np.linalg.eigh(M)
    return make_T(quat_to_R(V[:, np.argmax(w)]), t)


def spread(Ts: Iterable[np.ndarray]) -> dict:
    """Scatter of several measurements of one pose around their mean: max/rms position [mm] and angle [deg]."""
    Ts = list(Ts)
    m = average_T(Ts)
    d = np.array([pose_delta(m, T) for T in Ts])
    return {"n": len(Ts), "pos_max_mm": float(d[:, 0].max()), "pos_rms_mm": float(np.sqrt(np.mean(d[:, 0] ** 2))),
            "ang_max_deg": float(d[:, 1].max()), "ang_rms_deg": float(np.sqrt(np.mean(d[:, 1] ** 2))), "mean": m}


def fit_rigid(P_src: np.ndarray, P_dst: np.ndarray, w: np.ndarray | None = None) -> np.ndarray:
    """Least-squares rigid transform T with P_dst ≈ T @ P_src (Kabsch, no scale). P_* are (N, 3), N >= 3 points
    not all on one line; w optional (N,) weights."""
    P = np.asarray(P_src, float).reshape(-1, 3)
    Q = np.asarray(P_dst, float).reshape(-1, 3)
    if len(P) != len(Q) or len(P) < 3:
        raise ValueError("fit_rigid needs two point sets of equal length >= 3")
    w = np.ones(len(P)) if w is None else np.asarray(w, float).reshape(-1)
    w = w / w.sum()
    cp, cq = w @ P, w @ Q
    H = (P - cp).T @ ((Q - cq) * w[:, None])
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return make_T(R, cq - R @ cp)


def apply(T: np.ndarray, P: np.ndarray) -> np.ndarray:
    """Transform (N, 3) points."""
    P = np.asarray(P, float).reshape(-1, 3)
    return P @ T[:3, :3].T + T[:3, 3]


# ── RoboDK interop (lazy import, only where RoboDK is present) ───────────────
def from_robodk(M) -> np.ndarray:
    """robodk.robomath.Mat (4x4, mm) -> numpy."""
    return np.array([[M[i, j] for j in range(4)] for i in range(4)], float)


def to_robodk(T: np.ndarray):
    """numpy 4x4 -> robodk.robomath.Mat (requires C:\\RoboDK\\Python on sys.path)."""
    from robodk.robomath import Mat
    return Mat(np.asarray(T, float).tolist())
