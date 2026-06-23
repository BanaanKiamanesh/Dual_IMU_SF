"""Single- and dual-IMU sensor models, ported from ``sensors/IMU.m`` and
``sensors/DualIMU.m``.

The error chain (per :meth:`IMU.measure`) is applied in exactly the MATLAB order:
bias-instability random walk -> scale factor -> static bias -> white noise ->
saturation -> quantization. ``randn`` is drawn even when the corresponding
standard deviation is zero, matching MATLAB so the stochastic stream stays
aligned regardless of backend.
"""

from __future__ import annotations

import numpy as np


def _col(v) -> np.ndarray:
    return np.asarray(v, dtype=float).ravel()


class IMU:
    """Triaxial gyro/accel/mag model with configurable error sources."""

    def __init__(self, rng, sample_time: float = 0.01):
        self.rng = rng
        self.sample_time = sample_time

        self.accel_white_noise_std = 0.02 * np.ones(3)
        self.gyro_white_noise_std = 0.005 * np.ones(3)
        self.mag_white_noise_std = 0.01 * np.ones(3)

        self.accel_bias_instability_std = 1e-4 * np.ones(3)
        self.gyro_bias_instability_std = 1e-5 * np.ones(3)
        self.mag_bias_instability_std = 1e-5 * np.ones(3)

        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.mag_bias = np.zeros(3)

        self.accel_scale_factor = np.ones(3)
        self.gyro_scale_factor = np.ones(3)
        self.mag_scale_factor = np.ones(3)

        self.accel_range = 16 * 9.81
        self.gyro_range = np.deg2rad(300)
        self.mag_range = 2.0

        self.accel_bits = 16
        self.gyro_bits = 16
        self.mag_bits = 16

        self.enable_bias = True
        self.enable_white_noise = True
        self.enable_bias_instability = True
        self.enable_scale_factor = True
        self.enable_saturation = True
        self.enable_quantization = True

    def measure(self, specific_force_true, angular_rate_true, magnetic_field_true):
        accel = _col(specific_force_true)
        gyro = _col(angular_rate_true)
        mag = _col(magnetic_field_true)

        self._update_bias_instability()

        if self.enable_scale_factor:
            accel = self.accel_scale_factor * accel
            gyro = self.gyro_scale_factor * gyro
            mag = self.mag_scale_factor * mag

        if self.enable_bias:
            accel = accel + self.accel_bias
            gyro = gyro + self.gyro_bias
            mag = mag + self.mag_bias

        if self.enable_white_noise:
            accel = accel + self.accel_white_noise_std * self.rng.randn(3)
            gyro = gyro + self.gyro_white_noise_std * self.rng.randn(3)
            mag = mag + self.mag_white_noise_std * self.rng.randn(3)

        if self.enable_saturation:
            accel = self._saturate(accel, self.accel_range)
            gyro = self._saturate(gyro, self.gyro_range)
            mag = self._saturate(mag, self.mag_range)

        if self.enable_quantization:
            accel = self._quantize(accel, self.accel_range, self.accel_bits)
            gyro = self._quantize(gyro, self.gyro_range, self.gyro_bits)
            mag = self._quantize(mag, self.mag_range, self.mag_bits)

        return accel, gyro, mag

    def _update_bias_instability(self) -> None:
        if not self.enable_bias_instability:
            return
        sqrt_dt = np.sqrt(self.sample_time)
        self.accel_bias = self.accel_bias + self.accel_bias_instability_std * sqrt_dt * self.rng.randn(3)
        self.gyro_bias = self.gyro_bias + self.gyro_bias_instability_std * sqrt_dt * self.rng.randn(3)
        self.mag_bias = self.mag_bias + self.mag_bias_instability_std * sqrt_dt * self.rng.randn(3)

    @staticmethod
    def _saturate(value, sensor_range):
        return np.clip(value, -sensor_range, sensor_range)

    @staticmethod
    def _quantize(value, sensor_range, bits):
        step = 2 * sensor_range / (2 ** bits)
        return np.round(value / step) * step


class DualIMU:
    """Two co-located IMUs fused with inverse-variance weighting."""

    def __init__(self, rng, sample_time: float = 0.01):
        self.sample_time = sample_time
        # Both units share one stochastic stream, matching MATLAB's global RNG:
        # IMU1.Measure consumes its draws, then IMU2.Measure consumes the next.
        self.imu1 = IMU(rng, sample_time)
        self.imu2 = IMU(rng, sample_time)

        self.mag_bias_calibration1 = np.zeros(3)
        self.mag_bias_calibration2 = np.zeros(3)
        self.mag_scale_factor_calibration1 = np.ones(3)
        self.mag_scale_factor_calibration2 = np.ones(3)

        self.accel_weights = np.ones((3, 2))
        self.gyro_weights = np.ones((3, 2))
        self.mag_weights = np.ones((3, 2))

    def measure(self, specific_force_true, angular_rate_true, magnetic_field_true):
        accel1, gyro1, mag1_raw = self.imu1.measure(
            specific_force_true, angular_rate_true, magnetic_field_true)
        accel2, gyro2, mag2_raw = self.imu2.measure(
            specific_force_true, angular_rate_true, magnetic_field_true)

        mag1_cal = self._calibrate(mag1_raw, self.mag_bias_calibration1, self.mag_scale_factor_calibration1)
        mag2_cal = self._calibrate(mag2_raw, self.mag_bias_calibration2, self.mag_scale_factor_calibration2)

        accel_fused = self._weighted_average(accel1, accel2, self.accel_weights)
        gyro_fused = self._weighted_average(gyro1, gyro2, self.gyro_weights)
        mag_fused = self._weighted_average(mag1_cal, mag2_cal, self.mag_weights)
        mag_fused_raw = self._weighted_average(mag1_raw, mag2_raw, self.mag_weights)

        data = {
            "accel1": accel1, "accel2": accel2, "accel_fused": accel_fused,
            "gyro1": gyro1, "gyro2": gyro2, "gyro_fused": gyro_fused,
            "mag1_raw": mag1_raw, "mag2_raw": mag2_raw,
            "mag1_calibrated": mag1_cal, "mag2_calibrated": mag2_cal,
            "mag_fused_raw": mag_fused_raw, "mag_fused": mag_fused,
            "accel_disagreement": np.linalg.norm(accel1 - accel2),
            "gyro_disagreement": np.linalg.norm(gyro1 - gyro2),
            "mag_raw_disagreement": np.linalg.norm(mag1_raw - mag2_raw),
            "mag_calibrated_disagreement": np.linalg.norm(mag1_cal - mag2_cal),
            "gyro_bias1": self.imu1.gyro_bias.copy(),
            "gyro_bias2": self.imu2.gyro_bias.copy(),
            "gyro_bias_fused": self._weighted_average(
                self.imu1.gyro_bias, self.imu2.gyro_bias, self.gyro_weights),
        }
        return accel_fused, gyro_fused, mag_fused, data

    @staticmethod
    def _calibrate(mag_raw, mag_bias, mag_scale_factor):
        return (_col(mag_raw) - _col(mag_bias)) / _col(mag_scale_factor)

    @staticmethod
    def _weighted_average(value1, value2, weights):
        value1 = _col(value1)
        value2 = _col(value2)
        weights = np.asarray(weights, dtype=float)
        if weights.ndim == 1:
            weights = np.tile(weights, (3, 1))
        if weights.shape[1] != 2:
            raise ValueError("Weights must be a 3-by-2 matrix or a 1-by-2 row vector.")
        weight_sum = weights[:, 0] + weights[:, 1]
        return (weights[:, 0] * value1 + weights[:, 1] * value2) / weight_sum
