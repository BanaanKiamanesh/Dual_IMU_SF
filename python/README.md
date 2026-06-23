# Dual-IMU Sensor-Fusion AHRS — Python port

A faithful Python/NumPy port of [BanaanKiamanesh/Dual_IMU_SF](https://github.com/BanaanKiamanesh/Dual_IMU_SF),
a MATLAB simulation framework for quaternion attitude estimation (EKF and Mahony)
over single- and dual-IMU MARG sensor models.

The port reproduces the original project's published results (`Report.pdf`,
Table 2) to within Monte-Carlo sampling error — see **Validation** below.

## Layout

```
dual_imu_sf/            # the package
  quaternion.py         # <- math/*.m   (QuatMultiply, QuatToRotm, IntegrateQuaternion, ...)
  imu.py                # <- sensors/IMU.m, sensors/DualIMU.m
  filters.py            # <- filters/AttitudeEKF.m, filters/MahonyAHRS.m
  config.py             # <- simulation/BuildConfig.m
  motion.py             # <- simulation/GenerateTrueMotion.m, GenerateTrueSensorInputs.m
  simulation.py         # <- simulation/RunAHRSSimulation.m (+ Configure*.m)
  metrics.py            # <- utilities/ComputeMetrics.m
  reporting.py          # <- utilities/PrintResults.m
  plotting.py           # <- utilities/PlotResults.m
  rng.py                # MATLAB-compatible Mersenne Twister + Gaussian source
scripts/
  main.py               # <- Main.m            (single run; --all for Table 1)
  run_monte_carlo.py    # <- RunMonteCarlo*.m  (50-run study; --all for Table 2)
  validate_against_report.py   # compares 50-run stats to Report.pdf
tests/                  # deterministic-math + RNG + simulation tests
```

## Install & run

```bash
pip install -r requirements.txt

python scripts/main.py                       # dual IMU + EKF, seed 2 (== Main.m)
python scripts/main.py --all                 # Table-1 four-case comparison
python scripts/main.py --sensor single --filter mahony --seed 3

python scripts/run_monte_carlo.py --all      # Table-2 (50 runs x 4 cases)
python scripts/validate_against_report.py    # pass/fail vs published numbers

python tests/test_quaternion.py              # math unit tests
python tests/test_rng_and_sim.py             # RNG + simulation tests
```

Plots (PNG) are written by `dual_imu_sf.plotting.plot_results(results, out_dir)`.

## Validation

50-run Monte-Carlo attitude RMSE [deg], mean ± std — Python port vs `Report.pdf` Table 2:

| Case | Metric | Report | Python |
|------|--------|--------|--------|
| Single IMU EKF | Roll / Pitch / Yaw | 0.1968 / 0.1584 / 0.4333 | 0.197 / 0.162 / 0.437 |
| **Dual IMU EKF** | Roll / Pitch / Yaw | **0.0855 / 0.0708 / 0.1736** | **0.085 / 0.072 / 0.178** |
| Single IMU Mahony | Roll / Pitch / Yaw | 0.2944 / 0.2952 / 1.1752 | 0.309 / 0.296 / 1.235 |
| Dual IMU Mahony | Roll / Pitch / Yaw | 0.2045 / 0.1773 / 0.6905 | 0.197 / 0.167 / 0.635 |

Every value lands within ≈1 Monte-Carlo standard error of the report.

## A note on "exact same results" and the RNG

This is a **stochastic** study: its published results are Monte-Carlo *statistics*
(means ± std over 50 runs), which are fixed by the algorithm and the noise
*variances*, not by the particular pseudo-random generator. So:

* **Deterministic code** (filters, sensor error chain, motion, metrics) is ported
  1:1 and matches MATLAB to machine precision — see `tests/`.
* **Noise** is drawn from NumPy's `RandomState` (an mt19937 generator), replicating
  the exact *order* in which MATLAB consumes `randn`. The single-run (Table 1)
  numbers therefore differ in the 3rd–4th digit because the noise *realization*
  differs, but the 50-run means agree (above).

MATLAB's `randn` uses an internal *ziggurat* transform whose bit-level recipe is
not publicly specified; reproducing it bit-for-bit offline (without a
MATLAB/Octave oracle) is not feasible. `rng.py` ships a verified
`MatlabMersenneTwister` whose **uniform** stream matches MATLAB exactly (seed 5489
→ `0.8147 0.9058 0.1270 0.9134 0.6324`, MATLAB's documented startup values), so a
bit-exact Gaussian backend could be slotted in without touching the models.
