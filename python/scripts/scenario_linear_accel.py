"""Robustness scenario: transient linear acceleration corrupts the accelerometer.

The benign trajectory of the original project has no linear acceleration, so the
accelerometer always reads pure gravity and both filters do well. Real platforms
accelerate. Here a burst of nav-frame linear acceleration is injected for a few
seconds; the accelerometer then no longer points along gravity.

* The ported EKF uses a fixed measurement covariance behind a hard norm gate:
  while ||a|| stays inside the gate it trusts the corrupted reading fully.
* The Error-State KF inflates the accel covariance ~ (||a|| - g)^2 (Sabatini),
  smoothly de-weighting the disturbed accelerometer.

We report attitude RMSE during and after the disturbance window.
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402

# Acceleration burst (nav frame), active during [t0, t1] seconds.
T0, T1 = 12.0, 17.0
AMP = np.array([4.0, -3.0, 2.0])  # m/s^2, ~0.5 g sustained shove


def burst(t):
    if T0 <= t <= T1:
        # smooth raised-cosine envelope to look like real motion
        w = 0.5 * (1 - np.cos(2 * np.pi * (t - T0) / (T1 - T0)))
        return AMP * w
    return np.zeros(3)


def run(filt, seed, disturbed):
    cfg = build_config()
    cfg.sensor_mode = "dual"
    cfg.filter_mode = filt
    cfg.random_seed = seed
    if disturbed:
        cfg.external_accel_fn = burst
    return run_ahrs_simulation(cfg)


def windowed_rmse(results, mask):
    e = results["euler_error"][:, mask]
    return np.rad2deg(np.sqrt(np.mean(e ** 2, axis=1)))  # [roll, pitch, yaw]


def main():
    runs = 20
    print(f"Linear-acceleration robustness, dual IMU, {runs} seeds")
    print(f"Burst {AMP.tolist()} m/s^2 over t in [{T0},{T1}] s\n")

    accum = {("ekf", "during"): [], ("ekf", "after"): [],
             ("eskf", "during"): [], ("eskf", "after"): []}

    for seed in range(1, runs + 1):
        for filt in ["ekf", "eskf"]:
            r = run(filt, seed, disturbed=True)
            t = r["time"]
            during = (t >= T0) & (t <= T1)
            after = t > T1
            accum[(filt, "during")].append(windowed_rmse(r, during))
            accum[(filt, "after")].append(windowed_rmse(r, after))

    hdr = f"{'filter':<8}{'window':<8}{'roll':>9}{'pitch':>9}{'yaw':>9}"
    print(hdr)
    print("-" * len(hdr))
    for filt in ["ekf", "eskf"]:
        for window in ["during", "after"]:
            arr = np.array(accum[(filt, window)])
            mean = arr.mean(axis=0)
            print(f"{filt:<8}{window:<8}{mean[0]:>9.4f}{mean[1]:>9.4f}{mean[2]:>9.4f}")
        print()

    # Headline: roll+pitch error during the burst (accel-driven axes)
    ekf_rp = np.array(accum[("ekf", "during")])[:, :2].mean()
    eskf_rp = np.array(accum[("eskf", "during")])[:, :2].mean()
    print(f"Mean roll+pitch RMSE during burst:  EKF {ekf_rp:.4f} deg   "
          f"ESKF {eskf_rp:.4f} deg   ({100*(1-eskf_rp/ekf_rp):+.1f}% vs EKF)")


if __name__ == "__main__":
    main()
