import numpy as np
import pytest

from mauer import geometry as g


@pytest.mark.parametrize("angle", [0.0, 1e-9, 1e-4, 0.3, 2.0, np.pi - 1e-7, np.pi])
def test_rotvec_roundtrip(angle):
    rng = np.random.default_rng(int(angle * 1e6) % 1000)
    for _ in range(20):
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        R = g.rotvec_to_R(ax * angle)
        assert np.allclose(g.rotvec_to_R(g.R_to_rotvec(R)), R, atol=1e-9)
        assert abs(np.linalg.norm(g.R_to_rotvec(R)) - angle) < 1e-6 or angle > np.pi - 1e-6


def test_ur_roundtrip_and_units():
    T = g.pose_xyz_rpy([100.0, -200.0, 300.0], [10, -20, 170])
    p = g.T_to_ur(T)
    assert np.allclose(p[:3], [0.1, -0.2, 0.3])
    assert np.allclose(g.ur_to_T(p), T)


def test_xyz_rpy_inverse():
    for rpy in ([0, 0, 0], [10, -20, 30], [180, 0, 0], [-170, 45, -90]):
        T = g.pose_xyz_rpy([1, 2, 3], rpy)
        assert np.allclose(g.pose_xyz_rpy(*g.xyz_rpy(T)), T, atol=1e-9)


def test_fit_rigid_and_average():
    rng = np.random.default_rng(0)
    T = g.pose_xyz_rpy([500, 20, -30], [3, -4, 100])
    P = rng.normal(size=(20, 3)) * 100
    assert np.allclose(g.fit_rigid(P, g.apply(T, P)), T, atol=1e-9)
    Ts = [T @ g.pose_xyz_rpy(rng.normal(size=3) * 0.1, rng.normal(size=3) * 0.01) for _ in range(50)]
    d, a = g.pose_delta(g.average_T(Ts), T)
    assert d < 0.05 and a < 0.01


def test_inv_and_delta():
    T = g.pose_xyz_rpy([1, 2, 3], [30, 40, 50])
    assert np.allclose(T @ g.inv(T), np.eye(4))
    assert g.pose_delta(T, T @ g.transl(3, 4, 0)) == pytest.approx((5.0, 0.0), abs=1e-9)
