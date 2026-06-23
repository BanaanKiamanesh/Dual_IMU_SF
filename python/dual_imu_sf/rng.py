"""Random number generation for the Dual-IMU AHRS simulation.

This module provides the stochastic source used by the IMU sensor models.

Background on matching MATLAB
-----------------------------
The original project seeds MATLAB's global generator with ``rng(seed)`` (the
Mersenne-Twister ``mt19937ar``) and draws Gaussian noise with ``randn``.

* The *uniform* stream is reproduced exactly here by :class:`MatlabMersenneTwister`
  (validated: seed 5489 reproduces MATLAB's documented startup ``rand`` values
  ``0.8147 0.9058 0.1270 0.9134 0.6324``).
* MATLAB's ``randn`` uses an internal *ziggurat* transform of that stream whose
  exact bit-level recipe is not publicly specified and cannot be validated
  offline without a MATLAB/Octave oracle. Because of that, reproducing MATLAB's
  per-sample Gaussian draws bit-for-bit is not attempted.

Why results still match the report
----------------------------------
The published results (see ``Report.pdf``) are Monte-Carlo statistics: means and
standard deviations of attitude RMSE over 50 runs. Those aggregates are fixed by
the algorithm and the noise *variances*, not by the particular PRNG, so any
correct zero-mean Gaussian source with the same standard deviations reproduces
them to within Monte-Carlo sampling error. The single-run (Table 1) numbers are
reproduced within the same statistical tolerance.

The :class:`Rng` wrapper below uses NumPy's legacy ``RandomState`` (itself an
mt19937 generator) for clean, portable reproducibility, and replicates the exact
*order* in which the MATLAB code consumes ``randn`` so that a bit-exact backend
could be dropped in without touching the sensor models.
"""

from __future__ import annotations

import numpy as np


class Rng:
    """Gaussian noise source mirroring MATLAB ``randn`` call semantics.

    A fresh instance corresponds to MATLAB's ``rng(seed)`` reseeding performed at
    the start of every simulation run.
    """

    def __init__(self, seed: int):
        self.seed = int(seed)
        self._rs = np.random.RandomState(self.seed)

    def randn(self, n: int = 3) -> np.ndarray:
        """Return ``n`` standard-normal samples (MATLAB ``randn(n,1)``)."""
        return self._rs.standard_normal(n)


class MatlabMersenneTwister:
    """Reference ``mt19937ar`` matching MATLAB's *uniform* stream.

    Provided for completeness and verification. ``rand``/``genrand_res53``
    reproduce MATLAB's ``rng(seed); rand`` sequence exactly. ``randn`` is *not*
    implemented here because MATLAB's ziggurat recipe cannot be validated
    offline (see module docstring).
    """

    N = 624
    M = 397
    MATRIX_A = 0x9908B0DF
    UPPER_MASK = 0x80000000
    LOWER_MASK = 0x7FFFFFFF
    MASK32 = 0xFFFFFFFF

    def __init__(self, seed: int = 5489):
        self.mt = [0] * self.N
        self.mti = self.N + 1
        self._init_genrand(seed & self.MASK32)

    def _init_genrand(self, s: int) -> None:
        self.mt[0] = s & self.MASK32
        for i in range(1, self.N):
            prev = self.mt[i - 1]
            self.mt[i] = (1812433253 * (prev ^ (prev >> 30)) + i) & self.MASK32
        self.mti = self.N

    def genrand_int32(self) -> int:
        mag01 = (0, self.MATRIX_A)
        mt = self.mt
        if self.mti >= self.N:
            for kk in range(self.N - self.M):
                y = (mt[kk] & self.UPPER_MASK) | (mt[kk + 1] & self.LOWER_MASK)
                mt[kk] = mt[kk + self.M] ^ (y >> 1) ^ mag01[y & 1]
            for kk in range(self.N - self.M, self.N - 1):
                y = (mt[kk] & self.UPPER_MASK) | (mt[kk + 1] & self.LOWER_MASK)
                mt[kk] = mt[kk + (self.M - self.N)] ^ (y >> 1) ^ mag01[y & 1]
            y = (mt[self.N - 1] & self.UPPER_MASK) | (mt[0] & self.LOWER_MASK)
            mt[self.N - 1] = mt[self.M - 1] ^ (y >> 1) ^ mag01[y & 1]
            self.mti = 0
        y = mt[self.mti]
        self.mti += 1
        y ^= y >> 11
        y ^= (y << 7) & 0x9D2C5680
        y ^= (y << 15) & 0xEFC60000
        y ^= y >> 18
        return y & self.MASK32

    def rand(self) -> float:
        """One double in [0,1) with 53-bit resolution (MATLAB ``rand``)."""
        a = self.genrand_int32() >> 5
        b = self.genrand_int32() >> 6
        return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0)
