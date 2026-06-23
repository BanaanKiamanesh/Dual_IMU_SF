"""Dual-IMU sensor-fusion AHRS — Python port of BanaanKiamanesh/Dual_IMU_SF.

Quaternion-based attitude estimation (EKF and Mahony) over single- and dual-IMU
MARG sensor models, with a Monte-Carlo harness for statistical comparison.
"""

from .config import Config, IMUParams, build_config
from .filters import AttitudeEKF, MahonyAHRS
from .filters_improved import ErrorStateKF
from .imu import IMU, DualIMU
from .simulation import run_ahrs_simulation
from .metrics import compute_metrics

__all__ = [
    "Config",
    "IMUParams",
    "build_config",
    "AttitudeEKF",
    "MahonyAHRS",
    "ErrorStateKF",
    "IMU",
    "DualIMU",
    "run_ahrs_simulation",
    "compute_metrics",
]
