"""Estimating the compound left in the screw (topic 3) and using it as feedforward.

FillEKF is an extended Kalman filter on a reduced physics model. Its model choices:
- m = 12 compartments carry the partial fill along the channel (the plant uses 80 cells).
- State x = [f_1..f_m, f_in, theta, Zf]:
  - f_in is the fill at the feed opening (feed efficiency).
  - theta is the compound consistency K (Pa s^n) of the batch in the screw.
  - Zf is the length of the fully filled zone.
- Inputs: screw speed N, melt temperature T, gear-pump speed, feed-on flag, and the
  batch-change event from the line's production records.
- Measurements: drive torque and head pressure.
- It uses its own, deliberately imperfect, constants:
  - n_e = 0.25 and b_e = 0.010 (the plant has 0.2226 and 0.0089);
  - gear-pump fill pressure 10 % high;
  - friction torque 5 % high;
  - initial theta 20 % low.

EKF equations: Simon, *Optimal State Estimation*, Wiley 2006, Ch. 13 (sections not
re-checked).

Observability, in plain words:
- Torque on its own sees theta * (wetted length) and cannot tell a viscosity change from
  a feed change.
- The gear pump takes a known volume per revolution. So in steady state the screw speed
  needed to hold the pressure fixes the fill at the front: f_m Qd(N) = Q_gp. With that
  the torque gives theta.
- After a batch change the EKF lets theta move. Otherwise a torque ramp is read as a feed
  change, which is right for feed disturbances and wrong for batch changes. The batch
  event is what separates the two.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .physics import Pump
from .plant import BAR, RPM, Extruder


@dataclass
class EkfConfig:
    m: int = 12
    ts: float = 0.5
    n_e: float = 0.25
    b_e: float = 0.010
    T0: float = 90.0
    P_fill_err: float = 1.10
    friction_err: float = 1.05
    theta0_err: float = 0.80
    r_torque: float = 400.0  # N m (sensor noise + flight ripple + model error)
    r_p: float = 1.0  # bar
    q_f: float = 0.002
    q_fin: float = 0.01
    q_theta_rel: float = 0.002
    q_theta_batch_rel: float = 0.04  # while a new batch travels through the screw
    q_zf: float = 0.005
    use_batch_event: bool = True


class FillEKF:
    def __init__(self, plant: Extruder, cfg: EkfConfig | None = None):
        self.cfg = c = cfg or EkfConfig()
        self.pump: Pump = plant.pump
        s = plant.p.screw
        self.s = s
        self.A = s.A
        self.dz = s.Z / c.m
        self.rho = plant.p.compound.rho
        self.Fp = self.pump.Fp
        self.qd1 = self.pump.q_drag(1.0)  # m3 per rev
        # gear pump: displacement x relative speed known; fill curve from the datasheet
        self.q_gp = plant.q_gp_design
        self.P_fill = plant.p.P_fill_bar * BAR * c.P_fill_err
        self.friction = plant.p.torque_friction * c.friction_err
        th0 = plant.p.compound.K * plant.p.compound.k_scale * c.theta0_err
        # initialise from a steady-state guess: fill from the speed/gear-pump balance
        N = plant.N
        f0 = 0.75
        self.x = np.r_[np.full(c.m, f0), f0, th0, 0.15]
        self.Pm = np.diag(np.r_[np.full(c.m, 0.05**2), 0.05**2, (0.3 * th0) ** 2, 0.05**2])
        self.batch_timer = 0.0
        self.N = N

    # ------------------------------------------------------------- model pieces
    def eta(self, theta: float, N: float, T: float) -> float:
        c = self.cfg
        g = max(self.pump.gdot_channel(max(N, 1e-3)), 0.05)
        return theta * g ** (c.n_e - 1.0) * math.exp(-c.b_e * (T - c.T0))

    def q_out(self, P_bar: float, n_gp_rel: float) -> float:
        return self.q_gp * n_gp_rel * (1 - math.exp(-max(P_bar, 0.0) * BAR / self.P_fill))

    def _kv(self, N: float) -> float:
        """Viscous power per unit area and unit viscosity: (Vbz^2 + 4 Vbx^2)/H."""
        vb = self.pump.vb(N)
        return vb * vb * (self.pump.cos_phi**2 + 4 * self.pump.sin_phi**2) / self.s.H

    def wetted(self, x: np.ndarray) -> float:
        """Filled channel length: partial fill plus the fully filled zone at the tip."""
        m = self.cfg.m
        return self.dz * float(np.sum(x[:m])) + x[m + 2] * (1 - x[m - 1])

    def torque_model(self, x: np.ndarray, N: float, T: float, P_bar: float) -> float:
        """tau = eta kv W L /(2 pi N) + P H Vbz W/(4 pi N) + friction (physics.channel_power)."""
        m = self.cfg.m
        eta = self.eta(x[m + 1], N, T)
        w = 2 * math.pi * max(N, 1e-3)
        vz = self.pump.vb(N) * self.pump.cos_phi
        return (eta * self._kv(N) * self.s.W * self.wetted(x) + 0.5 * P_bar * BAR * self.s.H * vz * self.s.W) / w             + self.friction

    def p_model(self, x: np.ndarray, N: float, T: float, P_meas: float, n_gp_rel: float) -> float:
        m = self.cfg.m
        theta, Zf = x[m + 1], max(x[m + 2], 1e-3)
        eta = self.eta(theta, N, T)
        qp = max(self.pump.q_drag(N) - self.q_out(P_meas, n_gp_rel), 0.0)
        G = qp * 12 * eta / (self.s.W * self.s.H**3 * self.Fp)
        return G * Zf / BAR

    def h_jac(self, x: np.ndarray, N: float, T: float, P_meas: float, n_gp_rel: float) -> np.ndarray:
        """Analytic measurement Jacobian (rows: torque, pressure)."""
        m = self.cfg.m
        theta, Zf, fm = x[m + 1], max(x[m + 2], 1e-3), x[m - 1]
        eta = self.eta(theta, N, T)
        k = self._kv(N) * self.s.W / (2 * math.pi * max(N, 1e-3))
        H = np.zeros((2, m + 3))
        H[0, :m] = eta * k * self.dz
        H[0, m - 1] = eta * k * (self.dz - Zf)
        H[0, m + 1] = (eta / theta) * k * self.wetted(x)
        H[0, m + 2] = eta * k * (1 - fm)
        p = self.p_model(x, N, T, P_meas, n_gp_rel)
        H[1, m + 1] = p / theta
        H[1, m + 2] = p / Zf
        return H

    def f_dyn(self, x: np.ndarray, N: float, P_bar: float, n_gp_rel: float, feed_on: bool) -> np.ndarray:
        c = self.cfg
        m = c.m
        out = x.copy()
        Qd = self.pump.q_drag(N)
        cfl = min(Qd / self.A * c.ts / self.dz, 1.0)
        f = x[:m]
        fin = x[m] if feed_on else 0.0
        out[0] = f[0] + cfl * (fin - f[0])
        out[1:m] = f[1:] + cfl * (f[:-1] - f[1:])
        fm = min(max(f[-1], 0.0), 0.98)
        dZf = (fm * Qd - self.q_out(P_bar, n_gp_rel)) / ((1 - fm) * self.A)
        out[m + 2] = min(max(x[m + 2] + c.ts * dZf, 1e-3), self.s.Z)
        out[:m] = np.clip(out[:m], 0.0, 1.0)
        out[m] = min(max(out[m], 0.0), 1.0)
        return out

    def f_jac(self, x: np.ndarray, N: float, P_bar: float, n_gp_rel: float, feed_on: bool) -> np.ndarray:
        """Analytic state Jacobian of f_dyn (advection is linear; Zf row from the front balance)."""
        c = self.cfg
        m = c.m
        Qd = self.pump.q_drag(N)
        cfl = min(Qd / self.A * c.ts / self.dz, 1.0)
        F = np.eye(m + 3)
        for j in range(m):
            F[j, j] = 1.0 - cfl
            if j > 0:
                F[j, j - 1] = cfl
        F[0, m] = cfl if feed_on else 0.0
        fm = min(max(x[m - 1], 0.0), 0.98)
        F[m + 2, m - 1] = c.ts * (Qd - self.q_out(P_bar, n_gp_rel)) / ((1 - fm) ** 2 * self.A)
        return F

    @staticmethod
    def _jac(fun, x: np.ndarray, rel: float = 1e-4) -> np.ndarray:
        y0 = np.atleast_1d(fun(x))
        J = np.zeros((len(y0), len(x)))
        for i in range(len(x)):
            h = rel * max(abs(x[i]), 1e-3)
            xp = x.copy()
            xp[i] += h
            J[:, i] = (np.atleast_1d(fun(xp)) - y0) / h
        return J

    # ------------------------------------------------------------- filter step
    def step(self, meas: dict, n_gp_rel: float, feed_on: bool = True, batch_event: bool = False) -> None:
        c = self.cfg
        m = c.m
        N = meas["N_rpm"] * RPM
        T = meas["T"]
        P = meas["P_bar"]
        if batch_event and c.use_batch_event:
            # transit time of the pumping section at the current speed
            self.batch_timer = self.s.Z / max(self.pump.q_drag(N) / self.A, 1e-3)
        # predict
        F = self.f_jac(self.x, N, P, n_gp_rel, feed_on)
        self.x = self.f_dyn(self.x, N, P, n_gp_rel, feed_on)
        th = self.x[m + 1]
        q_th = c.q_theta_batch_rel if self.batch_timer > 0 else c.q_theta_rel
        self.batch_timer = max(self.batch_timer - c.ts, 0.0)
        Q = np.diag(np.r_[np.full(m, c.q_f**2), (c.q_fin if feed_on else 1e-4) ** 2, (q_th * th) ** 2,
                          c.q_zf**2])
        self.Pm = F @ self.Pm @ F.T + Q
        # update with torque and pressure
        def h(z):
            return np.array([self.torque_model(z, N, T, P), self.p_model(z, N, T, P, n_gp_rel)])

        H = self.h_jac(self.x, N, T, P, n_gp_rel)
        y = np.array([meas["torque"], P]) - h(self.x)
        Rm = np.diag([c.r_torque**2, c.r_p**2])
        Sm = H @ self.Pm @ H.T + Rm
        K = self.Pm @ H.T @ np.linalg.inv(Sm)
        self.x = self.x + K @ y
        self.x[:m + 1] = np.clip(self.x[:m + 1], 0.0, 1.0)
        self.x[m + 1] = max(self.x[m + 1], 1e3)
        self.x[m + 2] = min(max(self.x[m + 2], 1e-3), self.s.Z)
        self.Pm = (np.eye(len(self.x)) - K @ H) @ self.Pm
        self.Pm = 0.5 * (self.Pm + self.Pm.T)
        self.N = N

    # ------------------------------------------------------------- outputs
    def mass(self) -> float:
        return self.rho * self.A * self.wetted(self.x)

    def mass_sigma(self) -> float:
        """1-sigma of the mass estimate from the filter covariance."""
        m = self.cfg.m
        g = np.zeros(len(self.x))
        g[:m] = self.rho * self.A * self.dz
        g[m - 1] -= self.rho * self.A * self.x[m + 2]
        g[m + 2] = self.rho * self.A * (1 - self.x[m - 1])
        return float(math.sqrt(max(g @ self.Pm @ g, 0.0)))

    def front_fill(self) -> float:
        return float(self.x[self.cfg.m - 1])

    def speed_for_balance(self, P_sp_bar: float, n_gp_rel: float) -> float:
        """Screw speed (rpm) at which the fill reaching the front matches the gear pump."""
        q = self.q_out(P_sp_bar, n_gp_rel)
        return q / (max(self.front_fill(), 0.05) * self.qd1) / RPM


def static_mass_balance(plant_like: Extruder, N_rpm: float, P_bar: float, n_gp_rel: float) -> float:
    """Baseline without torque: steady-state fill from the gear-pump/screw balance.

    f = Q_gp / Qd(N), and M = rho A (f (Z - Zf) + Zf) with Zf ignored. Correct only in
    steady state.
    """
    pump = plant_like.pump
    q = plant_like.q_gp_design * n_gp_rel * (1 - math.exp(-P_bar / plant_like.p.P_fill_bar))
    f = min(q / max(pump.q_drag(N_rpm * RPM), 1e-9), 1.0)
    return plant_like.p.compound.rho * plant_like.p.screw.volume * f
