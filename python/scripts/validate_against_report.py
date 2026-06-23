"""Validate the Python port against the published MATLAB results (Report.pdf).

Reproduces Table 2 (Monte-Carlo attitude RMSE, 50 runs) and compares each value
against the report. Because the noise *realization* differs between MATLAB's and
NumPy's generators, single-run values differ, but the 50-run means must agree
within Monte-Carlo sampling error -- that is the meaningful sense of "same
results" for a stochastic study. PASS = |mine - report| within 3x the combined
standard error of the mean (or a small floor).
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402

# Report.pdf, Table 2: mean +/- std over 50 runs, [deg]
REPORT = {
    ("single", "ekf"): {"roll": (0.1968, 0.0125), "pitch": (0.1584, 0.0137), "yaw": (0.4333, 0.0343)},
    ("dual", "ekf"): {"roll": (0.0855, 0.0075), "pitch": (0.0708, 0.0038), "yaw": (0.1736, 0.0118)},
    ("single", "mahony"): {"roll": (0.2944, 0.0590), "pitch": (0.2952, 0.0770), "yaw": (1.1752, 0.3679)},
    ("dual", "mahony"): {"roll": (0.2045, 0.0433), "pitch": (0.1773, 0.0503), "yaw": (0.6905, 0.3342)},
}
NAMES = {
    ("single", "ekf"): "Single IMU EKF",
    ("dual", "ekf"): "Dual IMU EKF",
    ("single", "mahony"): "Single IMU Mahony",
    ("dual", "mahony"): "Dual IMU Mahony",
}


def monte_carlo(sensor, filt, runs):
    cfg = build_config()
    cfg.sensor_mode = sensor
    cfg.filter_mode = filt
    out = {"roll": np.zeros(runs), "pitch": np.zeros(runs), "yaw": np.zeros(runs)}
    for i in range(runs):
        cfg.random_seed = i + 1
        m = run_ahrs_simulation(cfg)["metrics"]
        out["roll"][i] = np.rad2deg(m["roll_rmse"])
        out["pitch"][i] = np.rad2deg(m["pitch_rmse"])
        out["yaw"][i] = np.rad2deg(m["yaw_rmse"])
    return out


def main():
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    print(f"Validating against Report.pdf Table 2 ({runs} runs/case)\n")
    header = f"{'Case / metric':<26}{'report':>16}{'python':>16}{'|diff|':>9}  verdict"
    print(header)
    print("-" * len(header))

    all_pass = True
    for key in [("single", "ekf"), ("dual", "ekf"), ("single", "mahony"), ("dual", "mahony")]:
        res = monte_carlo(*key, runs)
        for metric in ["roll", "pitch", "yaw"]:
            rep_mean, rep_std = REPORT[key][metric]
            my_mean = float(np.mean(res[metric]))
            my_std = float(np.std(res[metric]))
            # Combined standard error of the two 50-run means (independent realizations).
            sem = np.hypot(rep_std, my_std) / np.sqrt(runs)
            tol = max(3 * sem, 0.01)
            diff = abs(my_mean - rep_mean)
            ok = diff <= tol
            all_pass = all_pass and ok
            label = f"{NAMES[key]} {metric}"
            print(f"{label:<26}{rep_mean:>8.4f}+/-{rep_std:<5.4f}"
                  f"{my_mean:>8.4f}+/-{my_std:<5.4f}{diff:>9.4f}  {'PASS' if ok else 'CHECK'}")
        print()

    print("RESULT:", "ALL PASS" if all_pass else "some metrics outside tolerance (see CHECK)")


if __name__ == "__main__":
    main()
