"""Console reporting, ported from ``utilities/PrintResults.m``."""

from __future__ import annotations

import numpy as np


def print_results(results: dict) -> None:
    m = results["metrics"]
    sensor = results["sensor_mode"]
    filt = results["filter_mode"]
    is_dual = sensor == "dual"

    print(f"Sensor mode: {sensor.upper()}")
    print(f"Filter mode: {filt.upper()}")
    print()

    print(f"Final roll error  [deg]: {np.rad2deg(m['final_roll_error']):.4f}")
    print(f"Final pitch error [deg]: {np.rad2deg(m['final_pitch_error']):.4f}")
    print(f"Final yaw error   [deg]: {np.rad2deg(m['final_yaw_error']):.4f}")
    print()

    print(f"Roll RMSE  [deg]: {np.rad2deg(m['roll_rmse']):.4f}")
    print(f"Pitch RMSE [deg]: {np.rad2deg(m['pitch_rmse']):.4f}")
    print(f"Yaw RMSE   [deg]: {np.rad2deg(m['yaw_rmse']):.4f}")
    print()

    label = "fused gyro bias" if is_dual else "gyro bias"
    print(f"Final estimated {label} [deg/s]:")
    _print_vec(np.rad2deg(results["gyro_bias_est"][:, -1]))
    print()
    print(f"Final true {label} [deg/s]:")
    _print_vec(np.rad2deg(results["gyro_bias_true"][:, -1]))
    print()

    if is_dual:
        print("Final IMU 1 true gyro bias [deg/s]:")
        _print_vec(np.rad2deg(results["gyro_bias_true1"][:, -1]))
        print()
        print("Final IMU 2 true gyro bias [deg/s]:")
        _print_vec(np.rad2deg(results["gyro_bias_true2"][:, -1]))
        print()
        print("Final fused gyro bias error [deg/s]:")
    else:
        print("Final gyro bias error [deg/s]:")
    _print_vec(np.rad2deg(m["final_gyro_bias_error"]))
    bias_err_norm = np.linalg.norm(np.rad2deg(m["final_gyro_bias_error"]))
    print(f"Bias error norm [deg/s]: {bias_err_norm:.4f}")
    print()

    pre = "fused " if is_dual else ""
    print(f"Raw {pre}magnetometer direction RMSE [deg]: "
          f"{np.rad2deg(m['raw_mag_direction_rmse']):.4f}")
    print(f"Calibrated {pre}magnetometer direction RMSE [deg]: "
          f"{np.rad2deg(m['calibrated_mag_direction_rmse']):.4f}")
    print()

    if is_dual:
        print("Dual IMU disagreement RMS:")
        print(f"Accel [m/s^2]: {m['accel_disagreement_rms']:.4f}")
        print(f"Gyro  [rad/s]: {m['gyro_disagreement_rms']:.4f}")
        print(f"Mag raw [-]: {m['mag_raw_disagreement_rms']:.4f}")
        print(f"Mag cal [-]: {m['mag_calibrated_disagreement_rms']:.4f}")
        print()

    print("Maximum quaternion norm error:")
    print(f"True: {m['q_true_norm_error_max']:.3e}")
    print(f"{filt.upper()} : {m['q_est_norm_error_max']:.3e}")


def _print_vec(v) -> None:
    v = np.asarray(v).ravel()
    print(f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
