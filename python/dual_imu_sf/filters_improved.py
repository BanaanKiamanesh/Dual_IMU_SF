"""Improved attitude estimator: an Error-State (Multiplicative) Kalman Filter.

This is the "better Kalman filter" deliverable. It addresses the structural
weaknesses of the ported :class:`dual_imu_sf.filters.AttitudeEKF` (a 7-state
quaternion-in-the-state EKF with numerical Jacobians):

1. **Consistent covariance / no over-parameterization.** The attitude error is
   carried as a minimal 3-vector ``delta_theta`` (a small body-frame rotation),
   not as four correlated quaternion components. The 6x6 error covariance never
   develops the rank deficiency of a 7x7 quaternion covariance, and the output
   quaternion is unit-norm by construction (multiplicative injection).
   This is the standard MEKF/ESKF used in spacecraft and modern VIO/AHRS.

2. **Analytic Jacobians.** The error-state transition and the vector-observation
   measurement Jacobians are closed form (no finite differences), which is both
   faster and numerically cleaner than the original's central-difference 7x7/6x7
   Jacobians (14 model evaluations per step).

3. **Adaptive measurement covariance (Sabatini).** The accelerometer only senses
   gravity direction when the body is not accelerating. Instead of a hard
   accept/reject norm gate, the accel measurement covariance is inflated smoothly
   in proportion to ``(||a|| - g)^2``, so transient linear accelerations are
   de-weighted rather than either trusted fully or dropped entirely.

References
----------
* J. Sola, "Quaternion kinematics for the error-state Kalman filter",
  arXiv:1711.02508, 2017.
* F. L. Markley, "Attitude error representations for Kalman filtering",
  J. Guidance, Control, and Dynamics, 2003.
* A. M. Sabatini, "Quaternion-based extended Kalman filter for determining
  orientation by inertial and magnetic sensing", IEEE T-BME, 2006.

Interface matches the ported filters (``update(accel, gyro, mag)`` plus
``quaternion`` and ``gyro_bias`` views) so it drops into the existing harness.
"""

from __future__ import annotations

import numpy as np

from .quaternion import quat_multiply, quat_normalize, quat_to_rotm


def _col(v) -> np.ndarray:
    return np.asarray(v, dtype=float).ravel()


def skew(v) -> np.ndarray:
    """Skew-symmetric matrix ``[v]x`` such that ``[v]x w = v x w``."""
    x, y, z = v
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def exp_quat(delta_theta) -> np.ndarray:
    """Rotation-vector -> quaternion (``Exp`` map), scalar-first."""
    delta_theta = _col(delta_theta)
    angle = np.linalg.norm(delta_theta)
    if angle < 1e-12:
        q = np.array([1.0, 0.5 * delta_theta[0], 0.5 * delta_theta[1], 0.5 * delta_theta[2]])
        return quat_normalize(q)
    axis = delta_theta / angle
    half = 0.5 * angle
    return np.concatenate([[np.cos(half)], np.sin(half) * axis])


class ErrorStateKF:
    """Error-State (Multiplicative) EKF for attitude + gyro bias.

    Nominal state: unit quaternion ``q`` (body->nav) and gyro bias ``b``.
    Error state:   ``[delta_theta(3), delta_b(3)]`` with 6x6 covariance ``P``.
    """

    def __init__(self, sample_time: float = 0.01):
        self.sample_time = sample_time

        # Nominal state
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.b = np.zeros(3)

        # 6x6 error covariance: [delta_theta; delta_b]
        self.P = np.diag([
            (0.1) ** 2, (0.1) ** 2, (0.1) ** 2,      # initial attitude error [rad]
            (0.05) ** 2, (0.05) ** 2, (0.05) ** 2,   # initial gyro-bias error [rad/s]
        ])

        # Process noise spectral densities
        self.gyro_noise_std = 0.008      # rad/s, angular random walk driving delta_theta
        self.gyro_bias_rw_std = 1.8e-3   # rad/s, gyro-bias random walk (matches IMU model)

        # Measurement noise on the unit accel/mag direction vectors
        self.accel_noise_std = 0.015
        self.mag_noise_std = 0.015

        # Adaptive accel covariance: R_accel scaled by 1 + k * (||a||/g - 1)^2
        self.adaptive_accel = True
        self.gravity_magnitude = 9.81
        self.accel_adapt_gain = 2000.0

        # References (set by the configurator; nav-frame unit vectors)
        self.gravity_reference = np.array([0.0, 0.0, 1.0])
        self.magnetic_reference = np.array([1.0, 0.0, 0.0])

        # Norm gates (wide; adaptation does the fine de-weighting)
        self.accel_norm_gate = (5.0, 15.0)
        self.mag_norm_gate = (0.3, 2.0)

        self.normalize_references()

    # --- views matching the ported filters ---
    @property
    def quaternion(self) -> np.ndarray:
        return self.q.copy()

    @property
    def gyro_bias(self) -> np.ndarray:
        return self.b.copy()

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
            self._correct(accel_meas, mag_meas, use_accel, use_mag, accel_norm)

    def _predict(self, gyro_meas) -> None:
        dt = self.sample_time
        omega = gyro_meas - self.b  # nominal corrected rate

        # Nominal quaternion propagation (first-order, matches the truth model)
        omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
        self.q = quat_normalize(self.q + 0.5 * quat_multiply(self.q, omega_quat) * dt)

        # Error-state transition  F = [[I - [w]x dt, -I dt], [0, I]]
        F = np.eye(6)
        F[0:3, 0:3] = np.eye(3) - skew(omega) * dt
        F[0:3, 3:6] = -np.eye(3) * dt

        # Discrete process noise
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = (self.gyro_noise_std ** 2) * dt * np.eye(3)
        Q[3:6, 3:6] = (self.gyro_bias_rw_std ** 2) * dt * np.eye(3)

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def _correct(self, accel_meas, mag_meas, use_accel, use_mag, accel_norm) -> None:
        Rt = quat_to_rotm(self.q).T  # nav -> body
        z_parts, h_parts, H_parts, r_parts = [], [], [], []

        if use_accel:
            z = accel_meas / accel_norm
            h0 = Rt @ self.gravity_reference
            h0 = h0 / np.linalg.norm(h0)
            H = np.zeros((3, 6))
            H[:, 0:3] = skew(h0)            # dh/d(delta_theta) = [h0]x ; dh/db = 0
            r = (self.accel_noise_std ** 2)
            if self.adaptive_accel:
                ratio = accel_norm / self.gravity_magnitude - 1.0
                r = r * (1.0 + self.accel_adapt_gain * ratio * ratio)
            z_parts.append(z); h_parts.append(h0); H_parts.append(H)
            r_parts.append(r * np.ones(3))

        if use_mag:
            mag_norm = np.linalg.norm(mag_meas)
            z = mag_meas / mag_norm
            h0 = Rt @ self.magnetic_reference
            h0 = h0 / np.linalg.norm(h0)
            H = np.zeros((3, 6))
            H[:, 0:3] = skew(h0)
            z_parts.append(z); h_parts.append(h0); H_parts.append(H)
            r_parts.append((self.mag_noise_std ** 2) * np.ones(3))

        z = np.concatenate(z_parts)
        h = np.concatenate(h_parts)
        H = np.vstack(H_parts)
        R = np.diag(np.concatenate(r_parts))

        innovation = z - h
        S = H @ self.P @ H.T + R
        K = np.linalg.solve(S.T, (self.P @ H.T).T).T  # P H^T S^-1

        delta_x = K @ innovation
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        # Inject error into the nominal state (multiplicative for attitude)
        delta_theta = delta_x[0:3]
        delta_b = delta_x[3:6]
        self.q = quat_normalize(quat_multiply(self.q, exp_quat(delta_theta)))
        self.b = self.b + delta_b
        # Error state is reset to zero implicitly (kept in P).
