"""7-State Local ENU Interacting Multiple Model (IMM) Kalman Filter.

State vector:
    x = [e, n, u, v_e, v_n, v_u, omega]^T
Models:
    Model 1: Constant Velocity (CV, omega = 0)
    Model 2: Coordinated Turn (CT, omega != 0) with EKF / exact discretization
Model Transition Matrix:
    Pi = [[0.95, 0.05],
          [0.10, 0.90]]

Authoritative reference: docs/05_TRACKING_AND_STATE_ESTIMATION.md Section 3.
"""

from __future__ import annotations

import math
from typing import Final

import numpy as np
import numpy.typing as npt

_PI_DEFAULT: Final[npt.NDArray[np.float64]] = np.array(
    [[0.95, 0.05], [0.10, 0.90]], dtype=np.float64
)


class IMMEstimator:
    """Interacting Multiple Model (IMM) filter combining CV and CT kinematic models."""

    def __init__(
        self,
        e0: float = 0.0,
        n0: float = 0.0,
        u0: float = 0.0,
        ve0: float = 0.0,
        vn0: float = 0.0,
        vu0: float = 0.0,
        omega0: float = 0.0,
        initial_mu_cv: float = 0.9,
    ) -> None:
        self.Pi: npt.NDArray[np.float64] = _PI_DEFAULT.copy()
        mu_cv = max(0.01, min(0.99, initial_mu_cv))
        self.mu: npt.NDArray[np.float64] = np.array([mu_cv, 1.0 - mu_cv], dtype=np.float64)

        x_init = np.array([e0, n0, u0, ve0, vn0, vu0, omega0], dtype=np.float64)
        self.x1: npt.NDArray[np.float64] = x_init.copy()
        self.x2: npt.NDArray[np.float64] = x_init.copy()

        # Initial Covariances (e, n, u, ve, vn, vu, omega)
        p_diag = np.array([25.0, 25.0, 25.0, 100.0, 100.0, 25.0, 0.01], dtype=np.float64)
        self.P1: npt.NDArray[np.float64] = np.diag(p_diag)
        self.P2: npt.NDArray[np.float64] = np.diag(p_diag)

        # Spectral densities for process noise
        self.sigma_ah_cv: float = 0.3
        self.sigma_av_cv: float = 0.2

        self.sigma_ah_ct: float = 3.0
        self.sigma_av_ct: float = 1.0
        self.sigma_w_ct: float = 0.05

        self.last_update_time_s: float | None = None
        self._c: npt.NDArray[np.float64] = self.Pi.T @ self.mu

    @property
    def mode_probability_cv(self) -> float:
        """Probability of Constant Velocity (CV) model."""
        return float(self.mu[0])

    @property
    def mode_probability_turn(self) -> float:
        """Probability of Coordinated Turn (CT) model."""
        return float(self.mu[1])

    @property
    def state(self) -> npt.NDArray[np.float64]:
        """Combined mixed state estimate [e, n, u, ve, vn, vu, omega]."""
        s: npt.NDArray[np.float64] = self.mu[0] * self.x1 + self.mu[1] * self.x2
        return s

    @property
    def covariance(self) -> npt.NDArray[np.float64]:
        """Combined mixed state covariance matrix."""
        x_comb = self.state
        d1 = (self.x1 - x_comb)[:, None]
        d2 = (self.x2 - x_comb)[:, None]
        p_comb: npt.NDArray[np.float64] = self.mu[0] * (self.P1 + d1 @ d1.T) + self.mu[1] * (
            self.P2 + d2 @ d2.T
        )
        return 0.5 * (p_comb + p_comb.T)

    def predict(self, dt_s: float) -> None:
        """Execute IMM mixing and model-matched state and covariance prediction forward by dt_s."""
        if dt_s <= 0.0:
            return

        # 1. Model Mixing
        c: npt.NDArray[np.float64] = self.Pi.T @ self.mu
        c = np.maximum(c, 1e-12)
        self._c = c

        mu_mix = np.zeros((2, 2), dtype=np.float64)
        for i in range(2):
            for j in range(2):
                mu_mix[i, j] = self.Pi[i, j] * self.mu[i] / c[j]

        x01: npt.NDArray[np.float64] = mu_mix[0, 0] * self.x1 + mu_mix[1, 0] * self.x2
        x02: npt.NDArray[np.float64] = mu_mix[0, 1] * self.x1 + mu_mix[1, 1] * self.x2

        d11 = (self.x1 - x01)[:, None]
        d21 = (self.x2 - x01)[:, None]
        P01: npt.NDArray[np.float64] = mu_mix[0, 0] * (self.P1 + d11 @ d11.T) + mu_mix[1, 0] * (
            self.P2 + d21 @ d21.T
        )

        d12 = (self.x1 - x02)[:, None]
        d22 = (self.x2 - x02)[:, None]
        P02: npt.NDArray[np.float64] = mu_mix[0, 1] * (self.P1 + d12 @ d12.T) + mu_mix[1, 1] * (
            self.P2 + d22 @ d22.T
        )

        dt = dt_s
        dt3_3 = (dt**3) / 3.0
        dt2_2 = (dt**2) / 2.0

        # 2. Model 1: Constant Velocity (CV)
        F1 = np.eye(7, dtype=np.float64)
        F1[0, 3] = dt
        F1[1, 4] = dt
        F1[2, 5] = dt

        self.x1 = F1 @ x01
        self.x1[6] = 0.0  # Turn rate identically zero in CV model

        qh1 = self.sigma_ah_cv**2
        qv1 = self.sigma_av_cv**2
        Q1 = np.zeros((7, 7), dtype=np.float64)
        for idx, q in [(0, qh1), (1, qh1), (2, qv1)]:
            Q1[idx, idx] = dt3_3 * q
            Q1[idx, idx + 3] = dt2_2 * q
            Q1[idx + 3, idx] = dt2_2 * q
            Q1[idx + 3, idx + 3] = dt * q
        Q1[6, 6] = 1e-6 * dt

        P1_pred = F1 @ P01 @ F1.T + Q1
        self.P1 = 0.5 * (P1_pred + P1_pred.T)

        # 3. Model 2: Coordinated Turn (CT)
        w = float(x02[6])
        F2 = np.eye(7, dtype=np.float64)
        F2[2, 5] = dt
        x2_next = x02.copy()

        if abs(w) > 1e-4:
            sw = math.sin(w * dt)
            cw = math.cos(w * dt)
            f1 = sw / w
            f2 = (1.0 - cw) / w

            x2_next[0] = x02[0] + f1 * x02[3] - f2 * x02[4]
            x2_next[1] = x02[1] + f2 * x02[3] + f1 * x02[4]
            x2_next[2] = x02[2] + x02[5] * dt
            x2_next[3] = cw * x02[3] - sw * x02[4]
            x2_next[4] = sw * x02[3] + cw * x02[4]

            F2[0, 3] = f1
            F2[0, 4] = -f2
            F2[1, 3] = f2
            F2[1, 4] = f1
            F2[3, 3] = cw
            F2[3, 4] = -sw
            F2[4, 3] = sw
            F2[4, 4] = cw

            df1_dw = (w * dt * cw - sw) / (w * w)
            df2_dw = (w * dt * sw - (1.0 - cw)) / (w * w)
            F2[0, 6] = df1_dw * x02[3] - df2_dw * x02[4]
            F2[1, 6] = df2_dw * x02[3] + df1_dw * x02[4]
            F2[3, 6] = -dt * sw * x02[3] - dt * cw * x02[4]
            F2[4, 6] = dt * cw * x02[3] - dt * sw * x02[4]
        else:
            x2_next[0] = x02[0] + dt * x02[3]
            x2_next[1] = x02[1] + dt * x02[4]
            x2_next[2] = x02[2] + dt * x02[5]

            F2[0, 3] = dt
            F2[1, 4] = dt
            F2[0, 6] = -0.5 * dt * dt * x02[4]
            F2[1, 6] = 0.5 * dt * dt * x02[3]
            F2[3, 6] = -dt * x02[4]
            F2[4, 6] = dt * x02[3]

        self.x2 = x2_next

        qh2 = self.sigma_ah_ct**2
        qv2 = self.sigma_av_ct**2
        qw2 = self.sigma_w_ct**2
        Q2 = np.zeros((7, 7), dtype=np.float64)
        for idx, q in [(0, qh2), (1, qh2), (2, qv2)]:
            Q2[idx, idx] = dt3_3 * q
            Q2[idx, idx + 3] = dt2_2 * q
            Q2[idx + 3, idx] = dt2_2 * q
            Q2[idx + 3, idx + 3] = dt * q
        Q2[6, 6] = dt * qw2

        P2_pred = F2 @ P02 @ F2.T + Q2
        self.P2 = 0.5 * (P2_pred + P2_pred.T)

    def update_measurement(
        self,
        z: npt.NDArray[np.float64],
        H: npt.NDArray[np.float64],
        R: npt.NDArray[np.float64],
    ) -> None:
        """Perform Joseph-form measurement update and IMM model probability adjustment."""
        c = self._c
        results: list[tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], float]] = []

        eye7 = np.eye(7, dtype=np.float64)

        for x, P in [(self.x1, self.P1), (self.x2, self.P2)]:
            y = z - H @ x
            S = H @ P @ H.T + R
            S = 0.5 * (S + S.T)

            # Invert innovation covariance safely
            S_inv = np.linalg.inv(S)
            K = P @ H.T @ S_inv

            x_up = x + K @ y

            # Joseph-form covariance update: (I - K*H)*P*(I - K*H)^T + K*R*K^T
            IK = eye7 - K @ H
            P_up = IK @ P @ IK.T + K @ R @ K.T
            P_up = 0.5 * (P_up + P_up.T)

            # Gaussian Likelihood: exp(-0.5 * (m*ln(2pi) + ln|S| + y^T S^-1 y))
            m = len(z)
            sign, logdet = np.linalg.slogdet(S)
            logdet_val = logdet if sign > 0 else 0.0
            quad = float(y.T @ S_inv @ y)
            log_lik = -0.5 * (m * math.log(2.0 * math.pi) + logdet_val + quad)
            results.append((x_up, P_up, log_lik))

        self.x1, self.P1, ll1 = results[0]
        self.x2, self.P2, ll2 = results[1]

        # Model probability update
        log_c = np.log(np.maximum(c, 1e-12))
        log_prob = log_c + np.array([ll1, ll2], dtype=np.float64)
        log_prob -= np.max(log_prob)
        prob = np.exp(log_prob)
        prob_norm = prob / np.sum(prob)

        # Floor probabilities at 1e-4 to prevent total model extinction
        self.mu = np.clip(prob_norm, 1e-4, 1.0 - 1e-4)
        self.mu /= np.sum(self.mu)

    def update_position(
        self,
        e_m: float,
        n_m: float,
        u_m: float,
        sigma_h: float = 2.5,
        sigma_u: float = 7.6,
    ) -> None:
        """Update filter with 3D position measurement in local ENU coordinates."""
        z = np.array([e_m, n_m, u_m], dtype=np.float64)
        H = np.zeros((3, 7), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        R = np.diag([sigma_h**2, sigma_h**2, sigma_u**2]).astype(np.float64)
        self.update_measurement(z, H, R)

    def update_velocity(
        self,
        ve_mps: float,
        vn_mps: float,
        vu_mps: float,
        sigma_v: float = 1.0,
        sigma_vu: float = 0.5,
    ) -> None:
        """Update filter with 3D velocity measurement in local ENU coordinates."""
        z = np.array([ve_mps, vn_mps, vu_mps], dtype=np.float64)
        H = np.zeros((3, 7), dtype=np.float64)
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        H[2, 5] = 1.0
        R = np.diag([sigma_v**2, sigma_v**2, sigma_vu**2]).astype(np.float64)
        self.update_measurement(z, H, R)

    def update_altitude(self, u_m: float, sigma_u: float = 7.6) -> None:
        """Update filter with scalar vertical altitude measurement."""
        z = np.array([u_m], dtype=np.float64)
        H = np.zeros((1, 7), dtype=np.float64)
        H[0, 2] = 1.0
        R = np.array([[sigma_u**2]], dtype=np.float64)
        self.update_measurement(z, H, R)

    def predict_position(self, dt_s: float) -> tuple[float, float, float]:
        """Extrapolate East, North, Up position forward by dt_s seconds without mutating state."""
        st = self.state
        if dt_s <= 0.0:
            return float(st[0]), float(st[1]), float(st[2])

        w = float(st[6])
        if abs(w) > 1e-4:
            sw = math.sin(w * dt_s)
            cw = math.cos(w * dt_s)
            f1 = sw / w
            f2 = (1.0 - cw) / w
            e = st[0] + f1 * st[3] - f2 * st[4]
            n = st[1] + f2 * st[3] + f1 * st[4]
        else:
            e = st[0] + dt_s * st[3]
            n = st[1] + dt_s * st[4]

        u = st[2] + dt_s * st[5]
        return float(e), float(n), float(u)


__all__ = [
    "IMMEstimator",
]
