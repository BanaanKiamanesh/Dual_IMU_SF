"""Unit tests for the deterministic quaternion/math core.

These have no RNG and must hold to machine precision -- they pin the port's
math to MATLAB's definitions independent of any noise model.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf.quaternion import (  # noqa: E402
    integrate_quaternion,
    quat_multiply,
    quat_normalize,
    quat_to_euler,
    quat_to_rotm,
    vector_angle,
    wrap_to_pi,
)


def test_identity_quat_is_identity_rotation():
    R = quat_to_rotm([1, 0, 0, 0])
    assert np.allclose(R, np.eye(3))


def test_quat_multiply_identity():
    q = np.array([0.5, 0.5, 0.5, 0.5])
    assert np.allclose(quat_multiply([1, 0, 0, 0], q), q)


def test_quat_normalize_unit():
    q = quat_normalize([2, 0, 0, 0])
    assert np.allclose(q, [1, 0, 0, 0])


def test_quat_normalize_zero_returns_identity():
    assert np.allclose(quat_normalize([0, 0, 0, 0]), [1, 0, 0, 0])


def test_rotm_is_orthonormal():
    q = quat_normalize([0.3, -0.2, 0.5, 0.78])
    R = quat_to_rotm(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(R) - 1.0) < 1e-12


def test_90deg_yaw_about_z():
    # +90 deg yaw quaternion: w=cos(45), z=sin(45)
    c = np.cos(np.pi / 4)
    q = [c, 0, 0, c]
    euler = quat_to_euler(q)
    assert np.allclose(euler, [0, 0, np.pi / 2], atol=1e-12)
    # rotating body x-axis into nav frame should give +y
    R = quat_to_rotm(q)
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_euler_roundtrip_small_angles():
    # small angles avoid gimbal/wrap ambiguity; rebuild quat and compare euler
    roll, pitch, yaw = 0.1, -0.2, 0.3
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    q = np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])
    assert np.allclose(quat_to_euler(q), [roll, pitch, yaw], atol=1e-9)


def test_integrate_quaternion_constant_rate():
    # integrate pure z-rate for many small steps, compare to closed form
    dt = 1e-3
    wz = 1.0  # rad/s
    q = np.array([1.0, 0, 0, 0])
    for _ in range(1000):
        q = integrate_quaternion(q, [0, 0, wz], dt)
    yaw = quat_to_euler(q)[2]
    assert abs(yaw - 1.0) < 1e-3  # ~1 rad after 1 s


def test_vector_angle():
    assert abs(vector_angle([1, 0, 0], [0, 1, 0]) - np.pi / 2) < 1e-12
    assert abs(vector_angle([1, 0, 0], [1, 0, 0])) < 1e-12


def test_wrap_to_pi():
    # MATLAB mod(x+pi,2pi)-pi maps exactly pi -> -pi (range [-pi, pi)).
    assert np.allclose(wrap_to_pi(np.array([0.0, np.pi, 3 * np.pi, -3 * np.pi])),
                       [0.0, -np.pi, -np.pi, -np.pi])
    assert np.allclose(wrap_to_pi(np.array([0.5, -0.5, 2 * np.pi + 0.5])),
                       [0.5, -0.5, 0.5])


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL {fn.__name__}")
            traceback.print_exc()
    print(f"\n{passed}/{len(fns)} passed")
    sys.exit(0 if passed == len(fns) else 1)
