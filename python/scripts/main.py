"""Single-run driver, mirroring ``Main.m``.

Usage:
    python scripts/main.py                      # default: dual IMU + EKF, seed 2
    python scripts/main.py --sensor single --filter mahony
    python scripts/main.py --all                # Table-1 style 4-case comparison
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402
from dual_imu_sf.reporting import print_results  # noqa: E402


def _run_one(sensor, filt, seed):
    cfg = build_config()
    cfg.sensor_mode = sensor
    cfg.filter_mode = filt
    cfg.random_seed = seed
    return run_ahrs_simulation(cfg)


def _table1(seed):
    cases = [
        ("single", "ekf", "Single IMU EKF"),
        ("dual", "ekf", "Dual IMU EKF"),
        ("single", "mahony", "Single IMU Mahony"),
        ("dual", "mahony", "Dual IMU Mahony"),
    ]
    print(f"Main-run attitude results (seed={seed})\n")
    header = f"{'Case':<20}{'Roll':>9}{'Pitch':>9}{'Yaw':>9}{'BiasNorm':>10}"
    print(header)
    print("-" * len(header))
    for sensor, filt, name in cases:
        r = _run_one(sensor, filt, seed)
        m = r["metrics"]
        roll = np.rad2deg(m["roll_rmse"])
        pitch = np.rad2deg(m["pitch_rmse"])
        yaw = np.rad2deg(m["yaw_rmse"])
        bias_norm = np.linalg.norm(np.rad2deg(m["final_gyro_bias_error"]))
        print(f"{name:<20}{roll:>9.4f}{pitch:>9.4f}{yaw:>9.4f}{bias_norm:>10.4f}")


def main():
    parser = argparse.ArgumentParser(description="Dual-IMU AHRS single run.")
    parser.add_argument("--sensor", choices=["single", "dual"], default="dual")
    parser.add_argument("--filter", choices=["ekf", "mahony"], default="ekf")
    parser.add_argument("--seed", type=int, default=2)
    parser.add_argument("--all", action="store_true",
                        help="print 4-case Table-1 comparison instead of one run")
    args = parser.parse_args()

    if args.all:
        _table1(args.seed)
        return

    results = _run_one(args.sensor, args.filter, args.seed)
    print_results(results)


if __name__ == "__main__":
    main()
