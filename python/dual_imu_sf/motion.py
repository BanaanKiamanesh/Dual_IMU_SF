"""True motion and ideal sensor inputs, ported from
``simulation/GenerateTrueMotion.m`` and ``GenerateTrueSensorInputs.m``."""

from __future__ import annotations

import numpy as np

from .quaternion import quat_to_rotm


def generate_true_motion(t: float) -> np.ndarray:
    """Body angular rate [rad/s] of the prescribed reference trajectory."""
    return np.array([
        np.deg2rad(12) * np.sin(0.7 * t),
        np.deg2rad(8) * np.cos(0.5 * t),
        np.deg2rad(15) + np.deg2rad(4) * np.sin(0.3 * t),
    ])


def generate_true_sensor_inputs(q_true, config, t: float = 0.0):
    """Ideal body-frame specific force and magnetic field for attitude ``q_true``.

    Specific force is ``f^b = R^T (a^n - g^n)``. With no external acceleration
    (``config.external_accel_fn is None``) this reduces to ``R^T (-g^n)``, exactly
    the original project. A non-zero ``a^n`` models linear acceleration that
    corrupts the accelerometer's gravity reading.
    """
    R_true = quat_to_rotm(q_true)
    accel_nav = -config.gravity_nav
    fn = getattr(config, "external_accel_fn", None)
    if fn is not None:
        accel_nav = accel_nav + np.asarray(fn(t), dtype=float).ravel()
    specific_force_true = R_true.T @ accel_nav
    magnetic_field_true = R_true.T @ config.magnetic_field_nav
    return specific_force_true, magnetic_field_true
