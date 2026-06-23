"""Benchmark the improved Error-State KF against the ported EKF and Mahony.

Runs a Monte-Carlo study (seeds 1..N) for each (sensor, filter) combination and
reports mean +/- std attitude RMSE, final gyro-bias error norm, and wall-clock
time per run. This quantifies the "better Kalman filter" claim.

Usage:
    python scripts/compare_filters.py            # 50 runs
    python scripts/compare_filters.py --runs 20
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402


def monte_carlo(sensor, filt, runs):
    cfg = build_config()
    cfg.sensor_mode = sensor
    cfg.filter_mode = filt
    roll = np.zeros(runs); pitch = np.zeros(runs); yaw = np.zeros(runs)
    bias_norm = np.zeros(runs)
    t0 = time.time()
    for i in range(runs):
        cfg.random_seed = i + 1
        m = run_ahrs_simulation(cfg)["metrics"]
        roll[i] = np.rad2deg(m["roll_rmse"])
        pitch[i] = np.rad2deg(m["pitch_rmse"])
        yaw[i] = np.rad2deg(m["yaw_rmse"])
        bias_norm[i] = np.linalg.norm(np.rad2deg(m["final_gyro_bias_error"]))
    per_run = (time.time() - t0) / runs
    return roll, pitch, yaw, bias_norm, per_run


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=50)
    args = ap.parse_args()
    runs = args.runs

    print(f"Filter comparison over {runs} Monte-Carlo runs (mean +/- std)\n")
    hdr = (f"{'sensor':<7}{'filter':<8}{'roll':>14}{'pitch':>14}{'yaw':>14}"
           f"{'bias[deg/s]':>14}{'ms/run':>9}")
    print(hdr)
    print("-" * len(hdr))
    for sensor in ["single", "dual"]:
        for filt in ["ekf", "eskf", "mahony"]:
            roll, pitch, yaw, bnorm, per_run = monte_carlo(sensor, filt, runs)
            print(f"{sensor:<7}{filt:<8}"
                  f"{np.mean(roll):>7.4f}+/-{np.std(roll):<5.4f}"
                  f"{np.mean(pitch):>7.4f}+/-{np.std(pitch):<5.4f}"
                  f"{np.mean(yaw):>7.4f}+/-{np.std(yaw):<5.4f}"
                  f"{np.mean(bnorm):>8.4f}     "
                  f"{per_run * 1000:>6.1f}")
        print()


if __name__ == "__main__":
    main()
