"""Result metrics, ported from ``utilities/ComputeMetrics.m``."""

from __future__ import annotations

import numpy as np


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


def compute_metrics(results: dict) -> dict:
    euler_error = results["euler_error"]
    m = {}

    m["roll_rmse"] = _rms(euler_error[0, :])
    m["pitch_rmse"] = _rms(euler_error[1, :])
    m["yaw_rmse"] = _rms(euler_error[2, :])

    m["final_roll_error"] = float(euler_error[0, -1])
    m["final_pitch_error"] = float(euler_error[1, -1])
    m["final_yaw_error"] = float(euler_error[2, -1])

    m["final_gyro_bias_error"] = (
        results["gyro_bias_est"][:, -1] - results["gyro_bias_true"][:, -1])

    m["raw_mag_direction_rmse"] = _rms(results["mag_direction_error_raw"])
    m["calibrated_mag_direction_rmse"] = _rms(results["mag_direction_error_calibrated"])

    m["accel_disagreement_rms"] = _rms(results["accel_disagreement"])
    m["gyro_disagreement_rms"] = _rms(results["gyro_disagreement"])
    m["mag_raw_disagreement_rms"] = _rms(results["mag_raw_disagreement"])
    m["mag_calibrated_disagreement_rms"] = _rms(results["mag_calibrated_disagreement"])

    m["accel_disagreement_max"] = float(np.max(results["accel_disagreement"]))
    m["gyro_disagreement_max"] = float(np.max(results["gyro_disagreement"]))
    m["mag_raw_disagreement_max"] = float(np.max(results["mag_raw_disagreement"]))
    m["mag_calibrated_disagreement_max"] = float(np.max(results["mag_calibrated_disagreement"]))

    q_true_norm = np.linalg.norm(results["q_true"], axis=1)
    q_est_norm = np.linalg.norm(results["q_est"], axis=1)
    m["q_true_norm_error_max"] = float(np.max(np.abs(q_true_norm - 1)))
    m["q_est_norm_error_max"] = float(np.max(np.abs(q_est_norm - 1)))

    return m
