"""Monte-Carlo harness, mirroring ``RunMonteCarloSingleIMU.m`` /
``RunMonteCarloDualIMU.m``.

Runs ``--runs`` simulations (seeds 1..N, matching the MATLAB ``runIndex`` loop)
for a chosen sensor/filter and reports mean +/- std of attitude RMSE and final
gyro-bias error. With ``--all`` it reproduces the Report's Table 2 (all four
cases).

Usage:
    python scripts/run_monte_carlo.py --sensor dual --filter ekf
    python scripts/run_monte_carlo.py --all
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402


def monte_carlo(sensor, filt, runs):
    cfg = build_config()
    cfg.sensor_mode = sensor
    cfg.filter_mode = filt

    roll = np.zeros(runs)
    pitch = np.zeros(runs)
    yaw = np.zeros(runs)
    final_bias_err = np.zeros((3, runs))

    for i in range(runs):
        cfg.random_seed = i + 1  # seeds 1..N (MATLAB runIndex)
        r = run_ahrs_simulation(cfg)
        m = r["metrics"]
        roll[i] = np.rad2deg(m["roll_rmse"])
        pitch[i] = np.rad2deg(m["pitch_rmse"])
        yaw[i] = np.rad2deg(m["yaw_rmse"])
        final_bias_err[:, i] = np.rad2deg(m["final_gyro_bias_error"])

    return {
        "roll": roll, "pitch": pitch, "yaw": yaw,
        "final_bias_err": final_bias_err,
    }


def _fmt(x):
    return f"{np.mean(x):.4f} +/- {np.std(x):.4f}"


def _report(name, res):
    print(f"== {name} ==")
    print(f"Roll  RMSE [deg]: {_fmt(res['roll'])}")
    print(f"Pitch RMSE [deg]: {_fmt(res['pitch'])}")
    print(f"Yaw   RMSE [deg]: {_fmt(res['yaw'])}")
    print()


def main():
    parser = argparse.ArgumentParser(description="Dual-IMU AHRS Monte Carlo.")
    parser.add_argument("--sensor", choices=["single", "dual"], default="dual")
    parser.add_argument("--filter", choices=["ekf", "mahony"], default="ekf")
    parser.add_argument("--runs", type=int, default=50)
    parser.add_argument("--all", action="store_true",
                        help="reproduce Table 2: all four cases")
    args = parser.parse_args()

    if args.all:
        cases = [
            ("single", "ekf", "Single IMU EKF"),
            ("dual", "ekf", "Dual IMU EKF"),
            ("single", "mahony", "Single IMU Mahony"),
            ("dual", "mahony", "Dual IMU Mahony"),
        ]
        print(f"Monte Carlo over {args.runs} runs (mean +/- std)\n")
        for sensor, filt, name in cases:
            _report(name, monte_carlo(sensor, filt, args.runs))
        return

    name = f"{args.sensor.title()} IMU {args.filter.upper()}"
    _report(name, monte_carlo(args.sensor, args.filter, args.runs))


if __name__ == "__main__":
    main()
