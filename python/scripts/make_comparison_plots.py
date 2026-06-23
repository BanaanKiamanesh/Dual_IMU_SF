"""Generate comparison bar charts for the README / PR.

Produces, in ``python/figures/``:
  1. mc_rmse_comparison.png      - Monte-Carlo attitude RMSE, all 6 filter/sensor
                                   combinations, grouped by roll/pitch/yaw (std bars).
  2. speed_comparison.png        - wall-clock ms per 30 s run, per combination.
  3. validation_vs_report.png    - Python port vs published MATLAB Report (Table 2)
                                   for the four original cases (proves fidelity).
  4. disturbance_robustness.png  - error during a linear-acceleration burst,
                                   original EKF vs adaptive ESKF.

Usage:
    python scripts/make_comparison_plots.py            # 50 runs (matches report)
    python scripts/make_comparison_plots.py --runs 20  # quicker
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dual_imu_sf import build_config, run_ahrs_simulation  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "figures")

# (sensor, filter) -> display label
CASES = [
    ("single", "ekf", "Single\nEKF"),
    ("dual", "ekf", "Dual\nEKF"),
    ("single", "eskf", "Single\nESKF"),
    ("dual", "eskf", "Dual\nESKF"),
    ("single", "mahony", "Single\nMahony"),
    ("dual", "mahony", "Dual\nMahony"),
]

# Report.pdf Table 2 (50-run mean +/- std, [deg]) for the four original cases.
REPORT = {
    ("single", "ekf"): (0.1968, 0.1584, 0.4333),
    ("dual", "ekf"): (0.0855, 0.0708, 0.1736),
    ("single", "mahony"): (0.2944, 0.2952, 1.1752),
    ("dual", "mahony"): (0.2045, 0.1773, 0.6905),
}

# Linear-acceleration burst for the robustness figure.
T0, T1 = 12.0, 17.0
AMP = np.array([4.0, -3.0, 2.0])


def _burst(t):
    if T0 <= t <= T1:
        w = 0.5 * (1 - np.cos(2 * np.pi * (t - T0) / (T1 - T0)))
        return AMP * w
    return np.zeros(3)


def monte_carlo(sensor, filt, runs):
    cfg = build_config()
    cfg.sensor_mode = sensor
    cfg.filter_mode = filt
    roll = np.zeros(runs); pitch = np.zeros(runs); yaw = np.zeros(runs)
    t0 = time.time()
    for i in range(runs):
        cfg.random_seed = i + 1
        m = run_ahrs_simulation(cfg)["metrics"]
        roll[i] = np.rad2deg(m["roll_rmse"])
        pitch[i] = np.rad2deg(m["pitch_rmse"])
        yaw[i] = np.rad2deg(m["yaw_rmse"])
    ms_per_run = 1000 * (time.time() - t0) / runs
    return dict(roll=roll, pitch=pitch, yaw=yaw, ms=ms_per_run)


def disturbance_during(filt, runs):
    rp = []
    for seed in range(1, runs + 1):
        cfg = build_config()
        cfg.sensor_mode = "dual"
        cfg.filter_mode = filt
        cfg.random_seed = seed
        cfg.external_accel_fn = _burst
        r = run_ahrs_simulation(cfg)
        t = r["time"]
        mask = (t >= T0) & (t <= T1)
        e = np.rad2deg(np.sqrt(np.mean(r["euler_error"][:, mask] ** 2, axis=1)))
        rp.append(e)
    return np.array(rp).mean(axis=0)  # [roll, pitch, yaw]


def _grouped_bar(ax, labels, series, colors, ylabel, title, errs=None):
    x = np.arange(len(labels))
    width = 0.26
    names = ["Roll", "Pitch", "Yaw"]
    for j in range(3):
        yerr = errs[j] if errs is not None else None
        ax.bar(x + (j - 1) * width, series[j], width, label=names[j],
               color=colors[j], yerr=yerr, capsize=3, edgecolor="black", linewidth=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=50)
    args = ap.parse_args()
    runs = args.runs
    os.makedirs(OUT, exist_ok=True)
    colors = ["#1f77b4", "#ff7f0e", "#ffd11a"]

    print(f"Running Monte Carlo ({runs} runs) for {len(CASES)} cases ...")
    data = {(s, f): monte_carlo(s, f, runs) for s, f, _ in CASES}

    labels = [lbl for _, _, lbl in CASES]

    # --- 1. Monte-Carlo RMSE comparison ---
    means = [[np.mean(data[(s, f)][m]) for s, f, _ in CASES] for m in ("roll", "pitch", "yaw")]
    stds = [[np.std(data[(s, f)][m]) for s, f, _ in CASES] for m in ("roll", "pitch", "yaw")]
    fig, ax = plt.subplots(figsize=(10, 5))
    _grouped_bar(ax, labels, means, colors, "Attitude RMSE [deg]",
                 f"Monte-Carlo Attitude RMSE ({runs} runs) - EKF vs ESKF vs Mahony", errs=stds)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "mc_rmse_comparison.png"), dpi=130)
    plt.close(fig)

    # --- 2. Speed comparison ---
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ms = [data[(s, f)]["ms"] for s, f, _ in CASES]
    bar_colors = ["#1f77b4" if f == "ekf" else "#2ca02c" if f == "eskf" else "#9467bd"
                  for s, f, _ in CASES]
    bars = ax.bar(labels, ms, color=bar_colors, edgecolor="black", linewidth=0.4)
    for b, v in zip(bars, ms):
        ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.0f}", ha="center", va="bottom", fontsize=8)
    ax.set_ylabel("Wall-clock [ms per 30 s run]")
    ax.set_title("Filter runtime (lower is better) - ESKF uses analytic Jacobians")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "speed_comparison.png"), dpi=130)
    plt.close(fig)

    # --- 3. Validation vs report (four original cases) ---
    orig = [("single", "ekf"), ("dual", "ekf"), ("single", "mahony"), ("dual", "mahony")]
    orig_labels = ["Single EKF", "Dual EKF", "Single Mahony", "Dual Mahony"]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), sharey=False)
    for ax, mi, mname in zip(axes, range(3), ["Roll", "Pitch", "Yaw"]):
        x = np.arange(len(orig))
        width = 0.38
        rep = [REPORT[k][mi] for k in orig]
        mine = [np.mean(data[k][("roll", "pitch", "yaw")[mi]]) for k in orig]
        ax.bar(x - width / 2, rep, width, label="MATLAB report", color="#7f7f7f",
               edgecolor="black", linewidth=0.4)
        ax.bar(x + width / 2, mine, width, label="Python port", color="#1f77b4",
               edgecolor="black", linewidth=0.4)
        ax.set_xticks(x); ax.set_xticklabels(orig_labels, rotation=18, fontsize=8)
        ax.set_title(f"{mname} RMSE [deg]")
        ax.grid(True, axis="y", alpha=0.3)
        if mi == 0:
            ax.legend(fontsize=8)
    fig.suptitle(f"Validation: Python port reproduces MATLAB Report Table 2 ({runs} runs)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "validation_vs_report.png"), dpi=130)
    plt.close(fig)

    # --- 4. Disturbance robustness ---
    print("Running linear-acceleration disturbance scenario ...")
    ekf_d = disturbance_during("ekf", min(runs, 20))
    eskf_d = disturbance_during("eskf", min(runs, 20))
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    x = np.arange(3)
    width = 0.36
    ax.bar(x - width / 2, ekf_d, width, label="EKF (fixed R + hard gate)",
           color="#1f77b4", edgecolor="black", linewidth=0.4)
    ax.bar(x + width / 2, eskf_d, width, label="ESKF (adaptive R)",
           color="#2ca02c", edgecolor="black", linewidth=0.4)
    ax.set_xticks(x); ax.set_xticklabels(["Roll", "Pitch", "Yaw"])
    ax.set_ylabel("RMSE during burst [deg]")
    ax.set_title("Robustness to transient linear acceleration (dual IMU)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "disturbance_robustness.png"), dpi=130)
    plt.close(fig)

    print("Saved 4 comparison figures to", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
