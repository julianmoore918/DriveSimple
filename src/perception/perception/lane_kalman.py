"""
lane_kalman.py
==============

Constant-velocity Kalman filter that smooths the quadratic lane-polynomial
coefficients (a, b, c) produced by a UFLD-v2 detector, before they are handed
to a Stanley lateral controller.

Polynomial convention (vehicle / BEV frame, x forward, y lateral):

    y(x) = a * x**2 + b * x + c
        a : quadratic curvature term   (units 1/m)
        b : linear slope  = heading of the lane at x = 0   (dimensionless)
        c : lateral offset = cross-track error at x = 0    (units m)

State vector (interleaved position + rate, 6-D, matches lane_kalman.py in repo):

    x = [ a, a_dot, b, b_dot, c, c_dot ]^T

Measurement vector (from np.polyfit(deg=2), 3-D):

    z = [ a, b, c ]^T

Motion model : nearly-constant-velocity (NCV) on each coefficient.
Sampling     : dt = 0.2 s  (5 Hz detector).

Where each matrix / constant comes from is documented at its definition.

References
----------
[1] Y. Bar-Shalom, X. R. Li, T. Kirubarajan, "Estimation with Applications to
    Tracking and Navigation: Theory, Algorithms and Software",
    Wiley-Interscience, 2001. ISBN 978-0-471-41655-5.
    -> NCV transition F and observation H (kinematic-model chapter); the
       continuous-white-noise-acceleration (CWNA) process-noise block Q;
       chi-square measurement validation gating; Joseph-form covariance update.
[2] H. Loose, U. Franke, "B-Spline-Based Road Model for 3D Lane Recognition",
    IEEE ITSC, 2010. -> lane state predicted by applying ego motion
    (velocity v and yaw rate psi_dot); origin of the optional egomotion term.
[3] The MathWorks, Inc., "Tuning Kalman Filter to Improve State Estimation",
    MATLAB Sensor Fusion and Tracking Toolbox documentation.
    -> Q/R tuning procedure, dt = 0.2 s CV example, and initialising the
       rate states of P0 with larger uncertainty than the position states.
[4] A. Becker, "Kalman Filter from the Ground Up" (kalmanfilter.net).
    -> R taken directly from the measurement/sensor covariance.
"""
from __future__ import annotations

import numpy as np


# ---------------------------------------------------------------------------
# Building blocks (all literature-defined, see refs in module docstring)
# ---------------------------------------------------------------------------
def ncv_transition_block(dt: float) -> np.ndarray:
    """2x2 nearly-constant-velocity transition block  [[1, dt], [0, 1]]  -- [1]."""
    return np.array([[1.0, dt],
                     [0.0, 1.0]])


def cwna_process_block(dt: float, q: float) -> np.ndarray:
    """
    2x2 continuous-white-noise-acceleration process-noise block for one
    coefficient (Bar-Shalom [1]):

        Q_block = q * [[ dt**3 / 3, dt**2 / 2 ],
                       [ dt**2 / 2, dt        ]]

    q is the power spectral density of the (unmodelled) coefficient
    acceleration. It is the one quantity that is NOT read from a datasheet --
    it is the tuning knob ([1], [3]). Give a, b, c different q values, because
    they differ by orders of magnitude (curvature << offset).
    """
    return q * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0],
                         [dt ** 2 / 2.0, dt]])


def _block_diag(*blocks: np.ndarray) -> np.ndarray:
    n = sum(b.shape[0] for b in blocks)
    m = sum(b.shape[1] for b in blocks)
    out = np.zeros((n, m))
    r = c = 0
    for b in blocks:
        rr, cc = b.shape
        out[r:r + rr, c:c + cc] = b
        r += rr
        c += cc
    return out


# ---------------------------------------------------------------------------
# Filter
# ---------------------------------------------------------------------------
class LaneKalmanFilter:
    """
    6-state constant-velocity Kalman filter over quadratic lane coefficients.

    Robustness features:
      * predict-only coasting when the detector drops out (z is None),
      * chi-square Mahalanobis gating to reject outlier detections ([1]),
      * per-frame adaptive R (pass the covariance from np.polyfit(cov=True)),
      * Joseph-form covariance update for numerical stability ([1]).
    """

    POS_IDX = (0, 2, 4)   # a,     b,     c
    RATE_IDX = (1, 3, 5)  # a_dot, b_dot, c_dot

    def __init__(
        self,
        dt: float = 0.2,                       # 5 Hz detector
        q=(1e-4, 1e-2, 1e-1),                  # PSD for (a, b, c) -- TUNE [1],[3]
        r=(1e-4, 1e-3, 1e-2),                  # fallback meas. var if no cov [4]
        p0_pos=(1e-2, 1e-2, 1e-1),             # initial position variance
        p0_rate_scale: float = 10.0,           # rates start more uncertain [3]
        gate_threshold: float = 7.815,         # chi-square, 3 DOF, 95% [1]
        use_joseph: bool = True,
    ):
        self.dt = float(dt)
        self.q = np.asarray(q, float)
        self.gate_threshold = float(gate_threshold)
        self.use_joseph = bool(use_joseph)

        # F : block-diagonal NCV transition (6x6)                          [1]
        self.F = _block_diag(*[ncv_transition_block(dt) for _ in range(3)])

        # H : select the position (coefficient) states only (3x6)          [1]
        self.H = np.zeros((3, 6))
        for row, idx in enumerate(self.POS_IDX):
            self.H[row, idx] = 1.0

        # Q : block-diagonal CWNA process noise (6x6)                      [1]
        self.Q = _block_diag(*[cwna_process_block(dt, qi) for qi in self.q])

        # R : fallback measurement noise; prefer the per-frame polyfit cov [4]
        self.R_default = np.diag(np.asarray(r, float))

        # State and covariance
        self.x = np.zeros(6)
        p0 = np.zeros(6)
        for i, idx in enumerate(self.POS_IDX):
            p0[idx] = p0_pos[i]
        for i, idx in enumerate(self.RATE_IDX):
            p0[idx] = p0_pos[i] * p0_rate_scale
        self.P = np.diag(p0)

        self.initialized = False
        self.n_coast = 0
        self.n_rejected = 0

    # -- initialisation -----------------------------------------------------
    def initialize(self, z, R=None) -> np.ndarray:
        """Seed the state from the first measurement (rates left at zero)."""
        z = np.asarray(z, float)
        self.x[:] = 0.0
        for i, idx in enumerate(self.POS_IDX):
            self.x[idx] = z[i]
        Rm = self.R_default if R is None else np.asarray(R, float)
        for i, idx in enumerate(self.POS_IDX):
            self.P[idx, idx] = Rm[i, i]
        self.initialized = True
        return self.coeffs

    # -- update (a posteriori) ---------------------------------------------
    def update(self, z, R=None) -> np.ndarray:
        """
        Correct with measurement z = [a, b, c]. Pass R = polyfit covariance for
        per-frame adaptive noise. z = None -> coast (keep the prediction).
        """
        if z is None:
            self.n_coast += 1
            return self.coeffs

        z = np.asarray(z, float)
        R = self.R_default if R is None else np.asarray(R, float)

        nu = z - self.H @ self.x                 # innovation
        S = self.H @ self.P @ self.H.T + R       # innovation covariance
        Sinv = np.linalg.inv(S)

        # Mahalanobis gating: reject frames that are wildly inconsistent   [1]
        d2 = float(nu @ Sinv @ nu)
        if d2 > self.gate_threshold:
            self.n_rejected += 1
            return self.coeffs                   # keep the prior, do not correct

        K = self.P @ self.H.T @ Sinv             # Kalman gain
        self.x = self.x + K @ nu

        I = np.eye(6)
        if self.use_joseph:                      # Joseph-form update       [1]
            A = I - K @ self.H
            self.P = A @ self.P @ A.T + K @ R @ K.T
        else:
            self.P = (I - K @ self.H) @ self.P
        return self.coeffs

    # -- outputs ------------------------------------------------------------
    @property
    def coeffs(self) -> np.ndarray:
        """Smoothed [a, b, c]."""
        return self.x[list(self.POS_IDX)].copy()

    @property
    def state(self) -> np.ndarray:
        """Full 6-state [a, a_dot, b, b_dot, c, c_dot]."""
        return self.x.copy()

    def sample(self, x_grid) -> np.ndarray:
        """Resample the smoothed polyline y_hat(x) = a x^2 + b x + c."""
        a, b, c = self.coeffs
        x_grid = np.asarray(x_grid, float)
        return a * x_grid ** 2 + b * x_grid + c

    def stanley_terms(self):
        """
        Controller inputs derived from the smoothed coefficients:
            cross_track e_ct = c
            heading     theta_e = arctan(b)
            curvature   kappa = 2 a / (1 + b^2)^(3/2)   (feedforward)
        """
        a, b, c = self.coeffs
        cross_track = c
        heading = np.arctan(b)
        curvature = 2.0 * a / (1.0 + b ** 2) ** 1.5
        return cross_track, heading, curvature
    
    def _rebuild_matrices(self, dt: float):
        self.dt = float(dt)
        self.F = _block_diag(*[ncv_transition_block(dt) for _ in range(3)])
        self.Q = _block_diag(*[cwna_process_block(dt, qi) for qi in self.q])

    def set_q(self, q) -> None:
        """Live-update the process-noise PSDs (a, b, c) and rebuild Q
        with the current dt. Called by the perception node's parameter
        callback so `ros2 param set /lane_detection_node kf_q_c ...`
        takes effect without restarting."""
        self.q = np.asarray(q, float)
        self.Q = _block_diag(*[cwna_process_block(self.dt, qi) for qi in self.q])

    def predict(self, dt: float | None = None, egomotion=None):
        if dt is not None and abs(dt - self.dt) > 1e-6:
            self._rebuild_matrices(dt)
        self.x = self.F @ self.x
        if egomotion is not None:
            v, yaw_rate = egomotion
            a, b = self.x[0], self.x[2]
            self.x[2] += 2.0 * a * v * self.dt - yaw_rate * self.dt
            self.x[4] += b * v * self.dt
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.coeffs

    def step(self, z, R=None, dt: float | None = None, egomotion=None):
        if not self.initialized:
            if z is None: return self.coeffs
            return self.initialize(z, R)
        self.predict(dt=dt, egomotion=egomotion)
        return self.update(z, R)



# ---------------------------------------------------------------------------
# Smoke test: synthetic lane, does the filter reduce coefficient RMSE?
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    rng = np.random.default_rng(0)
    dt = 0.2
    kf = LaneKalmanFilter(dt=dt)

    T = 200
    t = np.arange(T) * dt
    a_true = 0.010 * np.sin(0.10 * t)     # slowly varying curvature
    b_true = 0.100 * np.sin(0.05 * t)     # heading
    c_true = 0.500 * np.sin(0.02 * t)     # lateral offset
    truth = np.stack([a_true, b_true, c_true], axis=1)

    meas_sigma = np.array([2e-3, 1e-2, 5e-2])   # detector noise on a, b, c
    R_frame = np.diag(meas_sigma ** 2)

    meas = np.zeros((T, 3))
    est = np.zeros((T, 3))
    for k in range(T):
        # simulate a 5% detector dropout
        if rng.random() < 0.05:
            z = None
            meas[k] = np.nan
        else:
            z = truth[k] + rng.normal(0.0, meas_sigma)
            meas[k] = z
        est[k] = kf.step(z, R=R_frame)

    valid = ~np.isnan(meas[:, 0])
    rmse_raw = np.sqrt(np.mean((meas[valid] - truth[valid]) ** 2, axis=0))
    rmse_kf = np.sqrt(np.mean((est - truth) ** 2, axis=0))

    print("coefficient           a          b          c")
    print("RMSE raw detector  {:.3e}  {:.3e}  {:.3e}".format(*rmse_raw))
    print("RMSE Kalman        {:.3e}  {:.3e}  {:.3e}".format(*rmse_kf))
    print("frames coasted:", kf.n_coast, " frames rejected:", kf.n_rejected)