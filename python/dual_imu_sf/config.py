"""Simulation configuration, ported from ``simulation/BuildConfig.m``.

``build_config()`` returns a :class:`Config` whose defaults reproduce the
original project (Dual-IMU EKF, seed 2, 30 s at 100 Hz). The two IMU parameter
sets and the inverse-variance fusion weights match the MATLAB values exactly.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class IMUParams:
    accel_white_noise_std: np.ndarray
    gyro_white_noise_std: np.ndarray
    mag_white_noise_std: np.ndarray
    accel_bias_instability_std: np.ndarray
    gyro_bias_instability_std: np.ndarray
    mag_bias_instability_std: np.ndarray
    accel_bias: np.ndarray
    gyro_bias: np.ndarray
    mag_bias: np.ndarray
    accel_scale_factor: np.ndarray
    gyro_scale_factor: np.ndarray
    mag_scale_factor: np.ndarray
    accel_range: float = 16 * 9.81
    gyro_range: float = float(np.deg2rad(300))
    mag_range: float = 2.0
    accel_bits: int = 16
    gyro_bits: int = 16
    mag_bits: int = 16
    enable_bias: bool = True
    enable_white_noise: bool = True
    enable_bias_instability: bool = True
    enable_scale_factor: bool = True
    enable_saturation: bool = True
    enable_quantization: bool = True


@dataclass
class Config:
    sensor_mode: str = "dual"          # 'single' | 'dual'
    filter_mode: str = "ekf"           # 'ekf' | 'mahony' | 'eskf'
    random_seed: int = 2

    # Optional external (linear) acceleration in the nav frame as a function of
    # time t -> np.array(3). Default None == pure-gravity specific force (matches
    # the original project). Used to stress-test robustness to non-gravity accel.
    external_accel_fn: object = None

    sample_time: float = 0.01
    end_time: float = 30.0

    gravity_nav: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    magnetic_field_nav: np.ndarray = field(default_factory=lambda: np.array([0.45, 0.05, 0.89]))

    imu: IMUParams = None
    imu_mag_bias_calibration: np.ndarray = None
    imu_mag_scale_factor_calibration: np.ndarray = None

    dual_imu1: IMUParams = None
    dual_imu2: IMUParams = None
    dual_mag_bias_calibration1: np.ndarray = None
    dual_mag_bias_calibration2: np.ndarray = None
    dual_mag_scale_factor_calibration1: np.ndarray = None
    dual_mag_scale_factor_calibration2: np.ndarray = None
    dual_accel_weights: np.ndarray = None
    dual_gyro_weights: np.ndarray = None
    dual_mag_weights: np.ndarray = None

    # EKF
    ekf_initial_state: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    ekf_initial_covariance: np.ndarray = field(
        default_factory=lambda: np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2]))
    ekf_process_noise: np.ndarray = field(
        default_factory=lambda: np.diag([1e-7, 1e-7, 1e-7, 1e-7, 3.5e-8, 3.5e-8, 3.5e-8]))
    ekf_accel_measurement_variance: np.ndarray = field(
        default_factory=lambda: (0.011 ** 2) * np.ones(3))
    ekf_mag_measurement_variance: np.ndarray = field(
        default_factory=lambda: (0.009 ** 2) * np.ones(3))
    ekf_accel_norm_gate: tuple = (7.0, 12.5)
    ekf_mag_norm_gate: tuple = (0.5, 1.5)

    # Mahony
    mahony_initial_quaternion: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    mahony_initial_gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mahony_kp: float = 2.0
    mahony_ki: float = 0.05
    mahony_accel_norm_gate: tuple = (7.0, 12.5)
    mahony_mag_norm_gate: tuple = (0.5, 1.5)

    # Error-State KF (the improved filter)
    eskf_gyro_noise_std: float = 0.008
    eskf_gyro_bias_rw_std: float = 1.8e-3
    eskf_accel_noise_std: float = 0.015
    eskf_mag_noise_std: float = 0.015
    eskf_adaptive_accel: bool = True
    eskf_accel_adapt_gain: float = 2000.0
    eskf_init_attitude_std: float = 0.1
    eskf_init_bias_std: float = 0.05

    @property
    def time(self) -> np.ndarray:
        return np.arange(0.0, self.end_time + self.sample_time / 2, self.sample_time)

    @property
    def number_of_steps(self) -> int:
        return self.time.size


def build_config() -> Config:
    cfg = Config()

    cfg.magnetic_field_nav = cfg.magnetic_field_nav / np.linalg.norm(cfg.magnetic_field_nav)

    imu1 = IMUParams(
        accel_white_noise_std=4.7e-2 * np.ones(3),
        gyro_white_noise_std=3.33e-2 * np.ones(3),
        mag_white_noise_std=0.010 * np.ones(3),
        accel_bias_instability_std=7.36e-4 * np.ones(3),
        gyro_bias_instability_std=1.8e-3 * np.ones(3),
        mag_bias_instability_std=np.zeros(3),
        accel_bias=np.array([0.03, -0.02, 0.04]),
        gyro_bias=np.deg2rad([0.4, -0.3, 0.2]),
        mag_bias=np.array([0.02, -0.01, 0.015]),
        accel_scale_factor=np.array([1.002, 0.998, 1.001]),
        gyro_scale_factor=np.array([1.001, 0.999, 1.002]),
        mag_scale_factor=np.array([1.010, 0.990, 1.005]),
    )

    # imu2 = imu1 with selected fields overridden (MATLAB: imu2 = imu1; then edits)
    imu2 = IMUParams(
        accel_white_noise_std=5.2e-2 * np.ones(3),
        gyro_white_noise_std=2.9e-2 * np.ones(3),
        mag_white_noise_std=0.012 * np.ones(3),
        accel_bias_instability_std=7.36e-4 * np.ones(3),
        gyro_bias_instability_std=1.8e-3 * np.ones(3),
        mag_bias_instability_std=np.zeros(3),
        accel_bias=np.array([-0.025, 0.035, -0.015]),
        gyro_bias=np.deg2rad([-0.25, 0.20, -0.15]),
        mag_bias=np.array([-0.015, 0.012, -0.010]),
        accel_scale_factor=np.array([0.999, 1.003, 0.997]),
        gyro_scale_factor=np.array([0.998, 1.002, 0.999]),
        mag_scale_factor=np.array([0.995, 1.008, 0.992]),
    )

    cfg.imu = imu1
    cfg.imu_mag_bias_calibration = imu1.mag_bias.copy()
    cfg.imu_mag_scale_factor_calibration = imu1.mag_scale_factor.copy()

    cfg.dual_imu1 = imu1
    cfg.dual_imu2 = imu2
    cfg.dual_mag_bias_calibration1 = imu1.mag_bias.copy()
    cfg.dual_mag_bias_calibration2 = imu2.mag_bias.copy()
    cfg.dual_mag_scale_factor_calibration1 = imu1.mag_scale_factor.copy()
    cfg.dual_mag_scale_factor_calibration2 = imu2.mag_scale_factor.copy()

    cfg.dual_accel_weights = np.column_stack([
        1.0 / imu1.accel_white_noise_std ** 2,
        1.0 / imu2.accel_white_noise_std ** 2])
    cfg.dual_gyro_weights = np.column_stack([
        1.0 / imu1.gyro_white_noise_std ** 2,
        1.0 / imu2.gyro_white_noise_std ** 2])
    cfg.dual_mag_weights = np.column_stack([
        1.0 / imu1.mag_white_noise_std ** 2,
        1.0 / imu2.mag_white_noise_std ** 2])

    return cfg
