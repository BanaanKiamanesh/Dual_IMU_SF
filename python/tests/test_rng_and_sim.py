"""Tests for the MATLAB-compatible Mersenne Twister and a simulation smoke test."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf.rng import MatlabMersenneTwister, Rng  # noqa: E402
from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402


def test_mt_matches_matlab_uniform_stream():
    """Seed 5489 reproduces MATLAB's documented startup ``rand(1,5)``."""
    mt = MatlabMersenneTwister(5489)
    vals = [round(mt.rand(), 4) for _ in range(5)]
    assert vals == [0.8147, 0.9058, 0.1270, 0.9134, 0.6324], vals


def test_rng_randn_shape_and_reproducible():
    r1 = Rng(7)
    r2 = Rng(7)
    a = r1.randn(3)
    b = r2.randn(3)
    assert a.shape == (3,)
    assert np.allclose(a, b)  # same seed -> same stream


def test_simulation_runs_and_norms_stay_unit():
    cfg = build_config()
    cfg.sensor_mode = "dual"
    cfg.filter_mode = "ekf"
    cfg.random_seed = 1
    r = run_ahrs_simulation(cfg)
    m = r["metrics"]
    # quaternion norms preserved to machine precision
    assert m["q_true_norm_error_max"] < 1e-9
    assert m["q_est_norm_error_max"] < 1e-9
    # dual path is genuinely active (nonzero disagreement)
    assert m["gyro_disagreement_rms"] > 0
    # attitude errors are physically reasonable (< a few degrees RMSE)
    assert np.rad2deg(m["yaw_rmse"]) < 1.0


def test_dual_beats_single_ekf_on_average():
    """Inverse-variance fusion should reduce EKF RMSE (report's central claim)."""
    def yaw(sensor, seed):
        cfg = build_config()
        cfg.sensor_mode = sensor
        cfg.filter_mode = "ekf"
        cfg.random_seed = seed
        return np.rad2deg(run_ahrs_simulation(cfg)["metrics"]["yaw_rmse"])

    single = np.mean([yaw("single", s) for s in range(1, 6)])
    dual = np.mean([yaw("dual", s) for s in range(1, 6)])
    assert dual < single


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
