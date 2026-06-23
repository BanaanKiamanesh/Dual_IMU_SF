"""Attitude estimators, ported from ``filters/AttitudeEKF.m`` and
``filters/MahonyAHRS.m``.

Both expose the same interface as the MATLAB classes: an ``update(accel, gyro,
mag)`` step plus ``quaternion`` and ``gyro_bias`` read-only views, so the
simulation driver can treat them interchangeably.
"""

from __future__ import annotations

import numpy as np

from .quaternion import (
    integrate_quaternion,
    quat_normalize,
    quat_to_rotm,
)


def _col(v) -> np.ndarray:
    return np.asarray(v, dtype=float).ravel()


def _mrdivide(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """MATLAB ``a / b`` (solve ``x * b = a`` for x)."""
    return np.linalg.solve(b.T, a.T).T


class AttitudeEKF:
    """Quaternion + gyro-bias EKF with accel/mag vector corrections.

    State ``x = [qw, qx, qy, qz, bgx, bgy, bgz]``.
    """

    def __init__(self, sample_time: float = 0.01):
        self.sample_time = sample_time

        self.x = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2])
        self.Q = np.diag([1e-7, 1e-7, 1e-7, 1e-7, 1e-8, 1e-8, 1e-8])

        self.accel_measurement_variance = (0.015 ** 2) * np.ones(3)
        self.mag_measurement_variance = (0.012 ** 2) * np.ones(3)

        self.gravity_reference = np.array([0.0, 0.0, 1.0])
        self.magnetic_reference = np.array([1.0, 0.0, 0.0])

        self.accel_norm_gate = (7.0, 12.5)
        self.mag_norm_gate = (0.5, 1.5)

        self.normalize_references()

    # --- read-only views matching MATLAB dependent properties ---
    @property
    def quaternion(self) -> np.ndarray:
        return self.x[:4].copy()

    @property
    def gyro_bias(self) -> np.ndarray:
        return self.x[4:7].copy()

    def normalize_references(self) -> None:
        self.gravity_reference = self.gravity_reference / np.linalg.norm(self.gravity_reference)
        self.magnetic_reference = self.magnetic_reference / np.linalg.norm(self.magnetic_reference)

    def update(self, accel_meas, gyro_meas, mag_meas) -> None:
        accel_meas = _col(accel_meas)
        gyro_meas = _col(gyro_meas)
        mag_meas = _col(mag_meas)

        self._predict(gyro_meas)

        accel_norm = np.linalg.norm(accel_meas)
        mag_norm = np.linalg.norm(mag_meas)

        use_accel = self.accel_norm_gate[0] < accel_norm < self.accel_norm_gate[1]
        use_mag = self.mag_norm_gate[0] < mag_norm < self.mag_norm_gate[1]

        if use_accel or use_mag:
            self._correct(accel_meas, mag_meas, use_accel, use_mag)

    def _predict(self, gyro_meas) -> None:
        x_pred = self._state_transition(self.x, gyro_meas)
        F = self._numerical_state_jacobian(self.x, gyro_meas)

        self.x = x_pred
        self.P = F @ self.P @ F.T + self.Q
        self._normalize_state()

    def _correct(self, accel_meas, mag_meas, use_accel, use_mag) -> None:
        z_parts = []
        r_parts = []

        if use_accel:
            accel_meas = accel_meas / np.linalg.norm(accel_meas)
            z_parts.append(accel_meas)
            r_parts.append(self.accel_measurement_variance)

        if use_mag:
            mag_meas = mag_meas / np.linalg.norm(mag_meas)
            z_parts.append(mag_meas)
            r_parts.append(self.mag_measurement_variance)

        z = np.concatenate(z_parts)
        R = np.diag(np.concatenate(r_parts))

        h = self._measurement_model(self.x, use_accel, use_mag)
        H = self._numerical_measurement_jacobian(self.x, use_accel, use_mag)

        innovation = z - h
        S = H @ self.P @ H.T + R
        K = _mrdivide(self.P @ H.T, S)

        self.x = self.x + K @ innovation

        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self._normalize_state()

    # --- model functions ---
    def _state_transition(self, x, gyro_meas) -> np.ndarray:
        q = x[:4]
        gyro_bias = x[4:7]
        corrected_gyro = gyro_meas - gyro_bias
        q_next = integrate_quaternion(q, corrected_gyro, self.sample_time)
        return np.concatenate([q_next, gyro_bias])

    def _measurement_model(self, x, use_accel, use_mag) -> np.ndarray:
        q = x[:4]
        Rnb = quat_to_rotm(q)
        parts = []
        if use_accel:
            accel_pred = Rnb.T @ self.gravity_reference
            accel_pred = accel_pred / np.linalg.norm(accel_pred)
            parts.append(accel_pred)
        if use_mag:
            mag_pred = Rnb.T @ self.magnetic_reference
            mag_pred = mag_pred / np.linalg.norm(mag_pred)
            parts.append(mag_pred)
        return np.concatenate(parts)

    def _numerical_state_jacobian(self, x, gyro_meas) -> np.ndarray:
        n = x.size
        F = np.zeros((n, n))
        dx = 1e-6
        for i in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[i] += dx
            x_minus[i] -= dx
            x_plus[:4] = quat_normalize(x_plus[:4])
            x_minus[:4] = quat_normalize(x_minus[:4])
            f_plus = self._state_transition(x_plus, gyro_meas)
            f_minus = self._state_transition(x_minus, gyro_meas)
            F[:, i] = (f_plus - f_minus) / (2 * dx)
        return F

    def _numerical_measurement_jacobian(self, x, use_accel, use_mag) -> np.ndarray:
        n = x.size
        h = self._measurement_model(x, use_accel, use_mag)
        m = h.size
        H = np.zeros((m, n))
        dx = 1e-6
        for i in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[i] += dx
            x_minus[i] -= dx
            x_plus[:4] = quat_normalize(x_plus[:4])
            x_minus[:4] = quat_normalize(x_minus[:4])
            h_plus = self._measurement_model(x_plus, use_accel, use_mag)
            h_minus = self._measurement_model(x_minus, use_accel, use_mag)
            H[:, i] = (h_plus - h_minus) / (2 * dx)
        return H

    def _normalize_state(self) -> None:
        self.x[:4] = quat_normalize(self.x[:4])
        self.P = 0.5 * (self.P + self.P.T)


class MahonyAHRS:
    """Nonlinear complementary filter with proportional-integral feedback."""

    def __init__(self, sample_time: float = 0.01):
        self.sample_time = sample_time

        self.quaternion_state = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro_bias_state = np.zeros(3)

        self.kp = 2.0
        self.ki = 0.05

        self.gravity_reference = np.array([0.0, 0.0, 1.0])
        self.magnetic_reference = np.array([1.0, 0.0, 0.0])

        self.accel_norm_gate = (7.0, 12.5)
        self.mag_norm_gate = (0.5, 1.5)

        self.normalize_references()

    @property
    def quaternion(self) -> np.ndarray:
        return self.quaternion_state.copy()

    @property
    def gyro_bias(self) -> np.ndarray:
        return self.gyro_bias_state.copy()

    def normalize_references(self) -> None:
        self.gravity_reference = self.gravity_reference / np.linalg.norm(self.gravity_reference)
        self.magnetic_reference = self.magnetic_reference / np.linalg.norm(self.magnetic_reference)

    def update(self, accel_meas, gyro_meas, mag_meas) -> None:
        accel_meas = _col(accel_meas)
        gyro_meas = _col(gyro_meas)
        mag_meas = _col(mag_meas)

        accel_norm = np.linalg.norm(accel_meas)
        mag_norm = np.linalg.norm(mag_meas)

        use_accel = self.accel_norm_gate[0] < accel_norm < self.accel_norm_gate[1]
        use_mag = self.mag_norm_gate[0] < mag_norm < self.mag_norm_gate[1]

        correction_error = np.zeros(3)
        Rnb = quat_to_rotm(self.quaternion_state)

        if use_accel:
            accel_unit = accel_meas / accel_norm
            accel_pred = Rnb.T @ self.gravity_reference
            accel_pred = accel_pred / np.linalg.norm(accel_pred)
            correction_error = correction_error + np.cross(accel_unit, accel_pred)

        if use_mag:
            mag_unit = mag_meas / mag_norm
            mag_pred = Rnb.T @ self.magnetic_reference
            mag_pred = mag_pred / np.linalg.norm(mag_pred)
            correction_error = correction_error + np.cross(mag_unit, mag_pred)

        if use_accel or use_mag:
            self.gyro_bias_state = self.gyro_bias_state - self.ki * correction_error * self.sample_time

        corrected_gyro = gyro_meas - self.gyro_bias_state + self.kp * correction_error
        self.quaternion_state = integrate_quaternion(
            self.quaternion_state, corrected_gyro, self.sample_time)
