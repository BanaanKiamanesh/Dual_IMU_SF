# A better Kalman filter for this AHRS: the Error-State (Multiplicative) KF

This note documents the second deliverable — reading the project's Kalman filter,
identifying its weaknesses, and implementing and benchmarking a better one. The
improved filter is in [`dual_imu_sf/filters_improved.py`](dual_imu_sf/filters_improved.py)
(`ErrorStateKF`) and is selectable everywhere via `filter_mode="eskf"`.

## What the original EKF does, and where it is weak

The ported `AttitudeEKF` (`filters/AttitudeEKF.m`) is a textbook **additive**
quaternion EKF:

* **7-element state** `x = [q (4), gyro_bias (3)]`, with a **7×7 covariance**.
* Prediction integrates the quaternion and propagates `P = F P Fᵀ + Q`, with `F`
  computed by **central finite differences** (7 columns × 2 evaluations).
* Correction normalizes the accel/mag vectors, builds the predicted directions
  `R(q)ᵀ·ref`, and applies a Joseph-form update with a **numerical** `H`.
* The quaternion is renormalized and `P` symmetrized after every step.

Three structural problems:

1. **Over-parameterized state / inconsistent covariance.** A unit quaternion has
   only 3 rotational degrees of freedom, but the filter carries 4 and assigns
   them a 4×4 covariance block. That block is necessarily rank-deficient in the
   direction normal to the unit sphere; renormalizing `q` after the update does
   **not** renormalize `P`, so the attitude uncertainty is not represented
   consistently. This is the classic motivation for the multiplicative filter.
2. **Numerical Jacobians.** Central differences are accurate but cost ~14 full
   model evaluations per step and add finite-difference error. Both Jacobians
   here have simple closed forms.
3. **Brittle measurement gating.** The accelerometer is accepted or rejected by a
   hard norm gate `7.0 < ‖a‖ < 12.5`. Inside the gate a disturbed (accelerating)
   reading is trusted *fully*; outside it the correction is dropped *entirely*.
   There is no middle ground for partial trust.

## The improvement: Error-State / Multiplicative EKF (ESKF/MEKF)

The standard fix, used in spacecraft attitude determination and modern
visual-inertial navigation, keeps the quaternion as a **separate nominal
reference** and runs the Kalman filter on a **minimal 3-parameter attitude
error** plus the bias error:

* **Error state** `δx = [δθ (3), δb (3)]`, **6×6 covariance** — minimal and
  consistent. `δθ` is a small body-frame rotation vector.
* **Analytic transition** (Solà 2017):
  `F = [[I − [ω×]·dt, −I·dt], [0, I]]`, `ω = ω_meas − b`.
* **Analytic measurement Jacobian** for a body-frame vector observation
  `h₀ = R(q)ᵀ·v_n`:  `∂h/∂δθ = [h₀×]`, `∂h/∂δb = 0`.
* **Multiplicative injection**: `q ← q ⊗ Exp(δθ)` (unit norm by construction),
  `b ← b + δb`, then the error state resets to zero (held in `P`).
* **Adaptive accelerometer covariance** (Sabatini): instead of hard gating, the
  accel measurement variance is scaled by `1 + k·(‖a‖/g − 1)²`, smoothly
  de-weighting readings whose magnitude departs from gravity (`k = 2000`).

## Results

All numbers from the repository's own Monte-Carlo harness
(`scripts/compare_filters.py`, 50 runs, seeds 1..50) and the disturbance
scenario (`scripts/scenario_linear_accel.py`).

### 1. Accuracy on the standard (benign) trajectory — attitude RMSE [deg]

| Case | Filter | Roll | Pitch | Yaw |
|------|--------|------|-------|-----|
| dual | EKF (original) | 0.0854 | 0.0717 | 0.1778 |
| dual | **ESKF (new)** | **0.0796** | **0.0660** | **0.1722** |
| single | EKF (original) | 0.1969 | 0.1615 | 0.4371 |
| single | ESKF (new) | 0.1993 | 0.1611 | 0.4351 |

The ESKF is **better on every axis in the dual case** and statistically tied in
the single case — i.e. it matches or beats the original accuracy on the
trajectory the project was tuned for.

### 2. Speed — wall-clock per 30 s run

| Case | EKF | ESKF | speed-up |
|------|-----|------|----------|
| single | ~1190 ms | ~360 ms | **3.3×** |
| dual | ~1310 ms | ~460 ms | **2.8×** |

Analytic Jacobians on a 6×6 system replace 14 finite-difference model
evaluations per step.

### 3. Robustness to linear acceleration — the decisive win

A smooth `[4, −3, 2] m/s²` acceleration burst over `t ∈ [12, 17] s` corrupts the
accelerometer (it no longer reads pure gravity; the apparent "down" tilts ~23°).
Mean roll+pitch RMSE **during** the burst (dual, 20 seeds):

| Filter | roll+pitch RMSE during burst | peak roll error |
|--------|------------------------------|-----------------|
| EKF (fixed R + hard gate) | 9.93° | ~28° |
| **ESKF (adaptive R)** | **3.78°** | **~10°** |

A **62 % reduction** in attitude error during the disturbance, and faster
recovery afterward (see [`figures/scenario_linear_accel.png`](figures/scenario_linear_accel.png)).
Turning the adaptation **off** makes the ESKF behave like the original EKF under
the burst (~13°), confirming the gain comes from the adaptive covariance, not
merely the error-state structure.

## How to reproduce

```bash
python scripts/compare_filters.py --runs 50      # tables 1 & 2
python scripts/scenario_linear_accel.py          # table 3 + figure
python tests/test_eskf.py                         # correctness tests
```

## References

1. J. Solà, *Quaternion kinematics for the error-state Kalman filter*,
   arXiv:1711.02508, 2017.
2. F. L. Markley, *Attitude Error Representations for Kalman Filtering*,
   J. Guidance, Control, and Dynamics, 26(2), 2003.
3. N. Trawny, S. Roumeliotis, *Indirect Kalman Filter for 3D Attitude
   Estimation*, Univ. of Minnesota TR-2005-002.
4. A. M. Sabatini, *Quaternion-based extended Kalman filter for determining
   orientation by inertial and magnetic sensing*, IEEE Trans. Biomed. Eng.,
   53(7), 2006.
