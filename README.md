# Dual IMU Sensor Fusion AHRS

This repository implements a MATLAB simulation framework for attitude estimation using single-IMU and dual-IMU sensor configurations. The simulated IMU includes gyroscope, accelerometer, and magnetometer models with configurable white noise, bias, bias instability, scale-factor error, saturation, and quantization. The project generates a known reference motion, simulates realistic sensor measurements, and estimates attitude using quaternion-based AHRS algorithms.

Two attitude estimators are included: an Extended Kalman Filter (EKF) and a Mahony AHRS filter. The EKF estimates both attitude and gyroscope bias using accelerometer and magnetometer correction, while the Mahony filter provides a lighter nonlinear complementary-filter alternative with proportional-integral feedback. Both filters use the same simulated sensor data, which allows direct comparison between estimator behavior under identical motion and noise conditions.

The project also supports dual-IMU fusion. In this mode, two independent IMU models observe the same motion with different sensor error parameters, and their measurements are combined using inverse-variance weighted fusion before being passed to the attitude estimator. The repository includes scripts for running single simulations, Monte Carlo tests, comparing single versus dual IMU performance, and exporting plots and result data for report generation.

## Python port (`python/`)

A faithful NumPy port of the full framework lives in [`python/`](python/). It
reproduces the published Monte-Carlo results (`Report.pdf`, Table 2) to within
sampling error and adds an **improved Error-State (Multiplicative) Kalman Filter**
that is more accurate on the dual-IMU case, ~3× faster (analytic Jacobians), and
markedly more robust to linear-acceleration disturbances (adaptive measurement
covariance). See [`python/README.md`](python/README.md) and
[`python/IMPROVED_FILTER.md`](python/IMPROVED_FILTER.md).

```bash
cd python
pip install -r requirements.txt
python scripts/main.py --all                 # four-case attitude RMSE (seed 2)
python scripts/run_monte_carlo.py --all       # Monte-Carlo study (Report Table 2)
python scripts/compare_filters.py             # EKF vs ESKF vs Mahony
python scripts/make_comparison_plots.py       # comparison bar charts -> python/figures/
python scripts/validate_against_report.py     # pass/fail vs published numbers
```
