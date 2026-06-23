"""Tests for the improved Error-State Kalman Filter."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402
from dual_imu_sf.filters_improved import ErrorStateKF, exp_quat, skew  # noqa: E402


def test_skew_matches_cross_product():
    a = np.array([0.2, -0.5, 1.3])
    b = np.array([1.0, 2.0, -3.0])
    assert np.allclose(skew(a) @ b, np.cross(a, b))


def test_exp_quat_is_unit_and_small_angle():
    q = exp_quat([0.0, 0.0, 0.0])
    assert np.allclose(q, [1, 0, 0, 0])
    q = exp_quat([0.01, -0.02, 0.03])
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12


def test_eskf_runs_and_is_accurate():
    cfg = build_config()
    cfg.sensor_mode = "dual"
    cfg.filter_mode = "eskf"
    cfg.random_seed = 1
    m = run_ahrs_simulation(cfg)["metrics"]
    assert m["q_est_norm_error_max"] < 1e-9          # unit quaternion by construction
    assert np.rad2deg(m["yaw_rmse"]) < 0.5           # accurate on the benign trajectory


def test_eskf_beats_ekf_on_dual_average():
    """The headline claim: ESKF >= EKF accuracy on the standard dual case."""
    def total(filt):
        s = 0.0
        for seed in range(1, 11):
            cfg = build_config()
            cfg.sensor_mode = "dual"
            cfg.filter_mode = filt
            cfg.random_seed = seed
            m = run_ahrs_simulation(cfg)["metrics"]
            s += np.rad2deg(m["roll_rmse"] + m["pitch_rmse"] + m["yaw_rmse"])
        return s / 10
    assert total("eskf") < total("ekf")


def test_adaptive_helps_under_linear_acceleration():
    """Adaptive accel covariance reduces error during a linear-accel burst."""
    T0, T1 = 12.0, 17.0
    amp = np.array([4.0, -3.0, 2.0])

    def burst(t):
        if T0 <= t <= T1:
            w = 0.5 * (1 - np.cos(2 * np.pi * (t - T0) / (T1 - T0)))
            return amp * w
        return np.zeros(3)

    def during_rp(adaptive):
        cfg = build_config()
        cfg.sensor_mode = "dual"
        cfg.filter_mode = "eskf"
        cfg.random_seed = 1
        cfg.external_accel_fn = burst
        cfg.eskf_adaptive_accel = adaptive
        r = run_ahrs_simulation(cfg)
        t = r["time"]
        mask = (t >= T0) & (t <= T1)
        return np.rad2deg(np.sqrt(np.mean(r["euler_error"][:2, mask] ** 2)))

    assert during_rp(True) < during_rp(False)


def test_default_path_unchanged_by_accel_hook():
    """external_accel_fn=None must reproduce the original pure-gravity behavior."""
    cfg = build_config()
    cfg.sensor_mode = "dual"
    cfg.filter_mode = "ekf"
    cfg.random_seed = 1
    m = run_ahrs_simulation(cfg)["metrics"]
    # Pinned from the validated run (dual EKF, seed 1).
    assert abs(np.rad2deg(m["yaw_rmse"]) - 0.1761) < 1e-3


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL {fn.__name__}")
            traceback.print_exc()
    print(f"\n{passed}/{len(fns)} passed")
    sys.exit(0 if passed == len(fns) else 1)
