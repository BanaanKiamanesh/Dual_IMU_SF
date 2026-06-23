"""Plotting, ported from ``utilities/PlotResults.m``.

Figures are written to ``out_dir`` as PNGs (uses the non-interactive Agg
backend so it works headless). Call :func:`plot_results` with a results dict
from :func:`dual_imu_sf.run_ahrs_simulation`.
"""

from __future__ import annotations

import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _unwrap(arr):
    return np.unwrap(arr, axis=1)


def plot_results(results: dict, out_dir: str = "figures", prefix: str = "") -> list:
    os.makedirs(out_dir, exist_ok=True)
    time = results["time"]
    filt = results["filter_mode"].upper()
    is_dual = results["sensor_mode"] == "dual"
    sensor_name = "Dual-IMU" if is_dual else "Single-IMU"
    meas_label = "fused" if is_dual else "measured"
    saved = []

    def _save(fig, name):
        path = os.path.join(out_dir, f"{prefix}{name}.png")
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        saved.append(path)

    euler_true = _unwrap(results["euler_true"])
    euler_est = _unwrap(results["euler_est"])

    for idx, label in enumerate(["Roll", "Pitch", "Yaw"]):
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(time, np.rad2deg(euler_true[idx]), lw=1.5, label="True")
        ax.plot(time, np.rad2deg(euler_est[idx]), "--", lw=1.2, label=filt)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"{label} [deg]")
        ax.grid(True)
        ax.legend()
        ax.set_title(f"{sensor_name} {filt} {label} Estimate")
        _save(fig, f"{label.lower()}_estimate")

    # Attitude error
    fig, ax = plt.subplots(figsize=(8, 4))
    for idx, label in enumerate(["Roll Error", "Pitch Error", "Yaw Error"]):
        ax.plot(time, np.rad2deg(results["euler_error"][idx]), lw=1.2, label=label)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Attitude Error [deg]")
    ax.grid(True)
    ax.legend()
    ax.set_title(f"{sensor_name} {filt} AHRS Attitude Error")
    _save(fig, "attitude_error")

    # Gyro measurement
    fig, ax = plt.subplots(figsize=(8, 4))
    for idx, c in enumerate("xyz"):
        ax.plot(time, results["angular_rate_true"][idx], lw=1.5, label=f"w_{c} true")
    for idx, c in enumerate("xyz"):
        ax.plot(time, results["gyro_meas"][idx], "--", lw=1.0, label=f"w_{c} {meas_label}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Rate [rad/s]")
    ax.grid(True)
    ax.legend(ncol=2, fontsize=8)
    ax.set_title(f"{sensor_name} Gyroscope Measurement")
    _save(fig, "gyro_measurement")

    # Gyro bias true vs estimated
    fig, ax = plt.subplots(figsize=(8, 4))
    for idx, c in enumerate("xyz"):
        ax.plot(time, np.rad2deg(results["gyro_bias_true"][idx]), lw=1.5, label=f"b_{c} true")
    for idx, c in enumerate("xyz"):
        ax.plot(time, np.rad2deg(results["gyro_bias_est"][idx]), "--", lw=1.0, label=f"b_{c} est")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Gyro Bias [deg/s]")
    ax.grid(True)
    ax.legend(ncol=2, fontsize=8)
    ax.set_title(f"{sensor_name} {filt} Gyroscope Bias")
    _save(fig, "gyro_bias")

    # Magnetometer direction error
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(time, np.rad2deg(results["mag_direction_error_raw"]), lw=1.2, label="Raw")
    ax.plot(time, np.rad2deg(results["mag_direction_error_calibrated"]), "--", lw=1.2,
            label="Calibrated")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Magnetic Direction Error [deg]")
    ax.grid(True)
    ax.legend()
    ax.set_title(f"{sensor_name} Magnetometer Direction Error")
    _save(fig, "mag_direction_error")

    if is_dual:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(time, results["accel_disagreement"], lw=1.2, label="Accel [m/s^2]")
        ax.plot(time, results["gyro_disagreement"], lw=1.2, label="Gyro [rad/s]")
        ax.plot(time, results["mag_calibrated_disagreement"], lw=1.2, label="Mag cal [-]")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Sensor-to-Sensor Disagreement")
        ax.grid(True)
        ax.legend()
        ax.set_title("Dual-IMU Disagreement Metrics")
        _save(fig, "dual_disagreement")

    return saved
