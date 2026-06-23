"""Quaternion and small-angle math, ported 1:1 from the MATLAB ``math/`` folder.

Quaternion convention: ``q = [w, x, y, z]`` (scalar first), mapping body-frame
vectors into the navigation frame via ``R(q)``. All functions accept and return
NumPy arrays.
"""

from __future__ import annotations

import numpy as np

_EPS = np.finfo(float).eps


def quat_multiply(q1, q2) -> np.ndarray:
    """Hamilton product ``q1 (x) q2`` (MATLAB ``QuatMultiply``)."""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_normalize(q) -> np.ndarray:
    """Return the unit quaternion (MATLAB ``QuatNormalize``)."""
    q = np.asarray(q, dtype=float).ravel()
    q_norm = np.linalg.norm(q)
    if q_norm < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / q_norm


def quat_to_rotm(q) -> np.ndarray:
    """Rotation matrix ``R_nb`` from quaternion (MATLAB ``QuatToRotm``)."""
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q) -> np.ndarray:
    """ZYX roll-pitch-yaw [rad] from quaternion (MATLAB ``QuatToEuler``)."""
    q = quat_normalize(q)
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sin_pitch = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def integrate_quaternion(q, omega_body, dt) -> np.ndarray:
    """First-order quaternion integration (MATLAB ``IntegrateQuaternion``)."""
    q = quat_normalize(q)
    omega_body = np.asarray(omega_body, dtype=float).ravel()
    omega_quat = np.array([0.0, omega_body[0], omega_body[1], omega_body[2]])
    q_dot = 0.5 * quat_multiply(q, omega_quat)
    q_next = q + q_dot * dt
    return quat_normalize(q_next)


def vector_angle(vector_a, vector_b) -> float:
    """Angle [rad] between two vectors (MATLAB ``VectorAngle``)."""
    a = np.asarray(vector_a, dtype=float).ravel()
    b = np.asarray(vector_b, dtype=float).ravel()
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    cos_angle = np.clip(np.dot(a, b), -1.0, 1.0)
    return float(np.arccos(cos_angle))


def wrap_to_pi(angle):
    """Wrap angle(s) to (-pi, pi] (MATLAB ``WrapToPi``)."""
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi
