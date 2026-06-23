"""Top-level simulation driver, ported from ``simulation/RunAHRSSimulation.m``
together with the ``Configure*`` helper functions.

``run_ahrs_simulation(config)`` generates the reference motion, simulates the
chosen sensor configuration, runs the chosen estimator, and returns a results
dict (NumPy arrays) plus a ``metrics`` sub-dict, mirroring the MATLAB struct.
"""

from __future__ import annotations

import numpy as np

from .config import Config, IMUParams
from .filters import AttitudeEKF, MahonyAHRS
from .filters_improved import ErrorStateKF
from .imu import IMU, DualIMU
from .metrics import compute_metrics
from .motion import generate_true_motion, generate_true_sensor_inputs
from .quaternion import (
    integrate_quaternion,
    quat_to_euler,
    vector_angle,
    wrap_to_pi,
)
from .rng import Rng


def _apply_imu_params(imu: IMU, p: IMUParams) -> None:
    imu.accel_white_noise_std = p.accel_white_noise_std.copy()
    imu.gyro_white_noise_std = p.gyro_white_noise_std.copy()
    imu.mag_white_noise_std = p.mag_white_noise_std.copy()
    imu.accel_bias_instability_std = p.accel_bias_instability_std.copy()
    imu.gyro_bias_instability_std = p.gyro_bias_instability_std.copy()
    imu.mag_bias_instability_std = p.mag_bias_instability_std.copy()
    imu.accel_bias = p.accel_bias.copy()
    imu.gyro_bias = p.gyro_bias.copy()
    imu.mag_bias = p.mag_bias.copy()
    imu.accel_scale_factor = p.accel_scale_factor.copy()
    imu.gyro_scale_factor = p.gyro_scale_factor.copy()
    imu.mag_scale_factor = p.mag_scale_factor.copy()
    imu.accel_range = p.accel_range
    imu.gyro_range = p.gyro_range
    imu.mag_range = p.mag_range
    imu.accel_bits = p.accel_bits
    imu.gyro_bits = p.gyro_bits
    imu.mag_bits = p.mag_bits
    imu.enable_bias = p.enable_bias
    imu.enable_white_noise = p.enable_white_noise
    imu.enable_bias_instability = p.enable_bias_instability
    imu.enable_scale_factor = p.enable_scale_factor
    imu.enable_saturation = p.enable_saturation
    imu.enable_quantization = p.enable_quantization


def _configure_single_imu(config: Config, rng: Rng) -> IMU:
    imu = IMU(rng, config.sample_time)
    _apply_imu_params(imu, config.imu)
    return imu


def _configure_dual_imu(config: Config, rng: Rng) -> DualIMU:
    dual = DualIMU(rng, config.sample_time)
    _apply_imu_params(dual.imu1, config.dual_imu1)
    _apply_imu_params(dual.imu2, config.dual_imu2)
    dual.mag_bias_calibration1 = config.dual_mag_bias_calibration1.copy()
    dual.mag_bias_calibration2 = config.dual_mag_bias_calibration2.copy()
    dual.mag_scale_factor_calibration1 = config.dual_mag_scale_factor_calibration1.copy()
    dual.mag_scale_factor_calibration2 = config.dual_mag_scale_factor_calibration2.copy()
    dual.accel_weights = config.dual_accel_weights.copy()
    dual.gyro_weights = config.dual_gyro_weights.copy()
    dual.mag_weights = config.dual_mag_weights.copy()
    return dual


def _configure_ekf(config: Config) -> AttitudeEKF:
    ekf = AttitudeEKF(config.sample_time)
    ekf.x = config.ekf_initial_state.copy()
    ekf.P = config.ekf_initial_covariance.copy()
    ekf.Q = config.ekf_process_noise.copy()
    ekf.accel_measurement_variance = config.ekf_accel_measurement_variance.copy()
    ekf.mag_measurement_variance = config.ekf_mag_measurement_variance.copy()
    ekf.gravity_reference = np.array([0.0, 0.0, 1.0])
    ekf.magnetic_reference = config.magnetic_field_nav.copy()
    ekf.accel_norm_gate = config.ekf_accel_norm_gate
    ekf.mag_norm_gate = config.ekf_mag_norm_gate
    ekf.normalize_references()
    return ekf


def _configure_eskf(config: Config) -> ErrorStateKF:
    eskf = ErrorStateKF(config.sample_time)
    eskf.gyro_noise_std = config.eskf_gyro_noise_std
    eskf.gyro_bias_rw_std = config.eskf_gyro_bias_rw_std
    eskf.accel_noise_std = config.eskf_accel_noise_std
    eskf.mag_noise_std = config.eskf_mag_noise_std
    eskf.adaptive_accel = config.eskf_adaptive_accel
    eskf.accel_adapt_gain = config.eskf_accel_adapt_gain
    eskf.gravity_magnitude = float(np.linalg.norm(config.gravity_nav))
    eskf.gravity_reference = np.array([0.0, 0.0, 1.0])
    eskf.magnetic_reference = config.magnetic_field_nav.copy()
    eskf.P = np.diag([
        config.eskf_init_attitude_std ** 2] * 3 + [config.eskf_init_bias_std ** 2] * 3)
    eskf.normalize_references()
    return eskf


def _configure_mahony(config: Config) -> MahonyAHRS:
    mahony = MahonyAHRS(config.sample_time)
    mahony.quaternion_state = config.mahony_initial_quaternion.copy()
    mahony.gyro_bias_state = config.mahony_initial_gyro_bias.copy()
    mahony.kp = config.mahony_kp
    mahony.ki = config.mahony_ki
    mahony.gravity_reference = np.array([0.0, 0.0, 1.0])
    mahony.magnetic_reference = config.magnetic_field_nav.copy()
    mahony.accel_norm_gate = config.mahony_accel_norm_gate
    mahony.mag_norm_gate = config.mahony_mag_norm_gate
    mahony.normalize_references()
    return mahony


def run_ahrs_simulation(config: Config) -> dict:
    rng = Rng(config.random_seed)

    sensor_mode = config.sensor_mode.lower()
    filter_mode = config.filter_mode.lower()

    if sensor_mode == "single":
        imu = _configure_single_imu(config, rng)
    elif sensor_mode == "dual":
        dual_imu = _configure_dual_imu(config, rng)
    else:
        raise ValueError("config.sensor_mode must be 'single' or 'dual'.")

    if filter_mode == "ekf":
        ahrs = _configure_ekf(config)
    elif filter_mode == "mahony":
        ahrs = _configure_mahony(config)
    elif filter_mode == "eskf":
        ahrs = _configure_eskf(config)
    else:
        raise ValueError("config.filter_mode must be 'ekf', 'mahony', or 'eskf'.")

    time = config.time
    n_steps = config.number_of_steps
    dt = config.sample_time

    angular_rate_true = np.zeros((3, n_steps))
    specific_force_true = np.zeros((3, n_steps))
    mag_field_true = np.zeros((3, n_steps))

    gyro_meas = np.zeros((3, n_steps))
    accel_meas = np.zeros((3, n_steps))
    mag_meas = np.zeros((3, n_steps))
    mag_meas_calibrated = np.zeros((3, n_steps))

    accel_meas1 = np.zeros((3, n_steps))
    accel_meas2 = np.zeros((3, n_steps))
    gyro_meas1 = np.zeros((3, n_steps))
    gyro_meas2 = np.zeros((3, n_steps))
    mag_meas1_raw = np.zeros((3, n_steps))
    mag_meas2_raw = np.zeros((3, n_steps))
    mag_meas1_calibrated = np.zeros((3, n_steps))
    mag_meas2_calibrated = np.zeros((3, n_steps))

    q_true = np.zeros((n_steps, 4))
    q_est = np.zeros((n_steps, 4))

    euler_true = np.zeros((3, n_steps))
    euler_est = np.zeros((3, n_steps))
    euler_error = np.zeros((3, n_steps))

    gyro_bias_true = np.zeros((3, n_steps))
    gyro_bias_est = np.zeros((3, n_steps))
    gyro_bias_true1 = np.zeros((3, n_steps))
    gyro_bias_true2 = np.zeros((3, n_steps))

    accel_disagreement = np.zeros(n_steps)
    gyro_disagreement = np.zeros(n_steps)
    mag_raw_disagreement = np.zeros(n_steps)
    mag_calibrated_disagreement = np.zeros(n_steps)

    mag_direction_error_raw = np.zeros(n_steps)
    mag_direction_error_calibrated = np.zeros(n_steps)

    q_true[0, :] = np.array([1.0, 0.0, 0.0, 0.0])
    q_est[0, :] = ahrs.quaternion

    for k in range(n_steps):
        t = time[k]

        angular_rate_true[:, k] = generate_true_motion(t)
        sf, mf = generate_true_sensor_inputs(q_true[k, :], config, t)
        specific_force_true[:, k] = sf
        mag_field_true[:, k] = mf

        if sensor_mode == "single":
            accel_raw, gyro_raw, mag_raw = imu.measure(
                specific_force_true[:, k], angular_rate_true[:, k], mag_field_true[:, k])

            mag_calibrated = ((mag_raw - config.imu_mag_bias_calibration)
                              / config.imu_mag_scale_factor_calibration)

            accel_meas[:, k] = accel_raw
            gyro_meas[:, k] = gyro_raw
            mag_meas[:, k] = mag_raw
            mag_meas_calibrated[:, k] = mag_calibrated

            accel_meas1[:, k] = accel_raw
            accel_meas2[:, k] = accel_raw
            gyro_meas1[:, k] = gyro_raw
            gyro_meas2[:, k] = gyro_raw
            mag_meas1_raw[:, k] = mag_raw
            mag_meas2_raw[:, k] = mag_raw
            mag_meas1_calibrated[:, k] = mag_calibrated
            mag_meas2_calibrated[:, k] = mag_calibrated

            gyro_bias_true[:, k] = imu.gyro_bias
            gyro_bias_true1[:, k] = imu.gyro_bias
            gyro_bias_true2[:, k] = imu.gyro_bias
            # disagreement arrays remain zero in single mode
        else:
            accel_fused, gyro_fused, mag_fused_calibrated, d = dual_imu.measure(
                specific_force_true[:, k], angular_rate_true[:, k], mag_field_true[:, k])

            accel_meas[:, k] = accel_fused
            gyro_meas[:, k] = gyro_fused
            mag_meas[:, k] = d["mag_fused_raw"]
            mag_meas_calibrated[:, k] = mag_fused_calibrated

            accel_meas1[:, k] = d["accel1"]
            accel_meas2[:, k] = d["accel2"]
            gyro_meas1[:, k] = d["gyro1"]
            gyro_meas2[:, k] = d["gyro2"]
            mag_meas1_raw[:, k] = d["mag1_raw"]
            mag_meas2_raw[:, k] = d["mag2_raw"]
            mag_meas1_calibrated[:, k] = d["mag1_calibrated"]
            mag_meas2_calibrated[:, k] = d["mag2_calibrated"]

            gyro_bias_true[:, k] = d["gyro_bias_fused"]
            gyro_bias_true1[:, k] = d["gyro_bias1"]
            gyro_bias_true2[:, k] = d["gyro_bias2"]

            accel_disagreement[k] = d["accel_disagreement"]
            gyro_disagreement[k] = d["gyro_disagreement"]
            mag_raw_disagreement[k] = d["mag_raw_disagreement"]
            mag_calibrated_disagreement[k] = d["mag_calibrated_disagreement"]

        mag_direction_error_raw[k] = vector_angle(mag_field_true[:, k], mag_meas[:, k])
        mag_direction_error_calibrated[k] = vector_angle(
            mag_field_true[:, k], mag_meas_calibrated[:, k])

        ahrs.update(accel_meas[:, k], gyro_meas[:, k], mag_meas_calibrated[:, k])

        q_est[k, :] = ahrs.quaternion
        gyro_bias_est[:, k] = ahrs.gyro_bias

        euler_true[:, k] = quat_to_euler(q_true[k, :])
        euler_est[:, k] = quat_to_euler(q_est[k, :])
        euler_error[:, k] = wrap_to_pi(euler_est[:, k] - euler_true[:, k])

        if k < n_steps - 1:
            q_true[k + 1, :] = integrate_quaternion(
                q_true[k, :], angular_rate_true[:, k], dt)

    results = {
        "config": config,
        "sensor_mode": sensor_mode,
        "filter_mode": filter_mode,
        "time": time,
        "angular_rate_true": angular_rate_true,
        "specific_force_true": specific_force_true,
        "mag_field_true": mag_field_true,
        "gyro_meas": gyro_meas,
        "accel_meas": accel_meas,
        "mag_meas": mag_meas,
        "mag_meas_calibrated": mag_meas_calibrated,
        "accel_meas1": accel_meas1,
        "accel_meas2": accel_meas2,
        "gyro_meas1": gyro_meas1,
        "gyro_meas2": gyro_meas2,
        "mag_meas1_raw": mag_meas1_raw,
        "mag_meas2_raw": mag_meas2_raw,
        "mag_meas1_calibrated": mag_meas1_calibrated,
        "mag_meas2_calibrated": mag_meas2_calibrated,
        "q_true": q_true,
        "q_est": q_est,
        "euler_true": euler_true,
        "euler_est": euler_est,
        "euler_error": euler_error,
        "gyro_bias_true": gyro_bias_true,
        "gyro_bias_est": gyro_bias_est,
        "gyro_bias_true1": gyro_bias_true1,
        "gyro_bias_true2": gyro_bias_true2,
        "accel_disagreement": accel_disagreement,
        "gyro_disagreement": gyro_disagreement,
        "mag_raw_disagreement": mag_raw_disagreement,
        "mag_calibrated_disagreement": mag_calibrated_disagreement,
        "mag_direction_error_raw": mag_direction_error_raw,
        "mag_direction_error_calibrated": mag_direction_error_calibrated,
    }
    results["metrics"] = compute_metrics(results)
    return results
