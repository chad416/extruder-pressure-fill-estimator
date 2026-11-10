"""Control-oriented simulator of a cold-feed rubber extruder: pumping section + head.

States
- N      screw speed (rev/s), behind a speed-controlled drive (first order + ramp limit)
- f[i]   fill fraction of the partially filled channel, carried along by the drag velocity
- Zf     length of the fully filled zone at the screw tip (down-channel, m)
- P      head pressure at the screw tip (Pa), compressible head volume
- T      compound temperature in the pumping section (degC), one lumped node

Flows
- intake        Q_in = eps * c_in * N         (feed opening, flood fed through the feed roll)
- screw output  Q_s  = Qd(N) - Qp(P/Zf, eta)  (drag minus pressure flow over the filled zone)
- outlet        die: Q_out = Q_ref (P / (P_ref kappa aT))^(1/n)      (power-law die)
                gear pump: Q_out = q_gp N_gp (1 - exp(-P/P_fill))   (positive displacement)

The fill-front balance is (1 - f_front) A dZf/dt = f_front Qd - Q_s. A partly filled
channel cannot carry pressure, so the pressure rises from 0 at the front to P at the
tip. This is the starve-fed screw picture in Tadmor & Gogos Sec. 9.x and Brzoskowski
et al. 1987 [S4]. The material in the screw is
    M = rho A (sum_partial f dz + Zf).

Why these states: the head pressure (topic 2) and the material left in the screw
(topic 3) are coupled through Zf. This is the reason the two topics are one project.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from .physics import Compound, Pump, Screw

RPM = 1.0 / 60.0  # rev/s per rpm
BAR = 1e5


@dataclass(frozen=True)
class Params:
    screw: Screw = field(default_factory=Screw)
    compound: Compound = field(default_factory=Compound)
    outlet: str = "gearpump"  # "gearpump" (primary case) or "die" (contrast case)
    # nominal operating point (design point of the die) [assumed]
    N_nom_rpm: float = 30.0
    P_nom_bar: float = 60.0  # gear-pump inlet setpoint; the die case uses 150 bar [assumed]
    T_nom: float = 120.0
    # head [assumed]
    V_head: float = 4.0e-3  # m3 between screw tip and die / gear pump
    B_eff: float = 0.5e9  # effective bulk modulus incl. entrained air, Pa
    # feed: fraction of the drag capacity taken in at eps = 1 [assumed]
    fill_nom: float = 0.75
    T_feed: float = 40.0
    # thermal: wiped-surface heat transfer to the temperature-controlled barrel/screw
    UA: float = 1100.0  # W/K [assumed, calibrated to ~120 degC at the design point, see dossier 8.2]
    T_barrel: float = 80.0
    # drive [assumed, vendor class 185 kW / 45 rpm]
    tau_drive: float = 0.3
    ramp_rpm_s: float = 5.0
    N_min_rpm: float = 2.0
    N_max_rpm: float = 45.0
    P_rated_W: float = 185e3
    torque_friction: float = 1500.0  # N m, gearbox + thrust bearing [assumed]
    drive_eff: float = 0.94
    # gear pump (outlet == "gearpump") [assumed]
    P_fill_bar: float = 15.0
    # numerics
    cells: int = 80
    substeps: int = 5  # 20 ms at TS = 0.1 s; fastest plant mode is ~0.3 s

    @property
    def torque_rated(self) -> float:
        return self.P_rated_W / (2 * math.pi * self.N_max_rpm * RPM)


class Extruder:
    """Simulator. Call `step(N_sp_rpm, dt)` once per controller sample."""

    def __init__(self, p: Params | None = None, *, kappa: float = 1.0, eps: float = 1.0,
                 seed: int | None = 0):
        self.p = p or Params()
        self.pump = Pump(self.p.screw, self.p.compound)
        s = self.p.screw
        self.dz = s.Z / self.p.cells
        self.kappa = kappa
        self.eps = eps
        self.feed_on = True
        self.T_barrel = self.p.T_barrel
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        # intake coefficient: volume per revolution at eps = 1
        self.c_in = self.p.fill_nom * self.pump.q_drag(1.0)
        # die / gear pump reference
        Nn = self.p.N_nom_rpm * RPM
        self.Q_ref = self.p.fill_nom * self.pump.q_drag(Nn)
        self.N_gp_rel = 1.0  # gear-pump speed relative to its design speed
        self.q_gp_design = self.Q_ref / (1 - math.exp(-self.p.P_nom_bar / self.p.P_fill_bar))
        self.reset_steady(self.p.N_nom_rpm)

    # ------------------------------------------------------------------ physics
    @property
    def kappa_tip(self) -> float:
        """Viscosity factor of the batch at the screw tip (batches travel with the compound)."""
        return self._kl[-1]

    def eta_screw(self, N: float, T: float) -> float:
        return self.p.compound.eta(self.pump.gdot_channel(max(N, 1e-3)), T, self.kappa_tip)

    def a_T(self, T: float) -> float:
        return math.exp(-self.p.compound.b * (T - self.p.T_nom))

    def q_out(self, P: float, T: float) -> float:
        if P <= 0:
            return 0.0
        if self.p.outlet == "gearpump":
            return self.q_gp_design * self.N_gp_rel * (1 - math.exp(-P / (self.p.P_fill_bar * BAR)))
        n = self.p.compound.n
        return self.Q_ref * (P / (self.p.P_nom_bar * BAR * self.kappa_tip * self.a_T(T))) ** (1.0 / n)

    def q_screw(self, N: float, P: float, Zf: float, T: float) -> float:
        G = max(P, 0.0) / max(Zf, 1e-4)
        return self.pump.q_drag(N) - self.pump.q_pressure(G, self.eta_screw(N, T))

    # ------------------------------------------------------------------ state
    def reset_steady(self, N_rpm: float) -> None:
        """Put the plant at the steady state for speed N_rpm (feed on, current kappa/eps)."""
        N = N_rpm * RPM
        self.N = N
        self.Nsp_ramped = N
        f0 = min(1.0, self.eps * self.c_in * N / self.pump.q_drag(N))
        self.f = np.full(self.p.cells, f0)
        self.kc = np.full(self.p.cells, self.kappa)
        self._refresh()
        Q = f0 * self.pump.q_drag(N)
        T = self.p.T_nom
        for _ in range(200):  # fixed point on T
            P = self._p_for_q(Q, T)
            Zf = self._zf_for(N, P, Q, T)
            self.P, self.Zf, self.T = P, Zf, T
            Pd = self.dissipation()
            T_new = (Pd + self.p.compound.rho * Q * self.p.compound.cp * self.p.T_feed
                     + self.p.UA * self.T_barrel) / (self.p.compound.rho * Q * self.p.compound.cp + self.p.UA)
            if abs(T_new - T) < 1e-6:
                break
            T = 0.5 * T + 0.5 * T_new
        self.P, self.Zf, self.T = P, Zf, T

    def _p_for_q(self, Q: float, T: float) -> float:
        if self.p.outlet == "gearpump":
            r = min(Q / (self.q_gp_design * self.N_gp_rel), 0.999999)
            return -self.p.P_fill_bar * BAR * math.log(1 - r)
        n = self.p.compound.n
        return self.p.P_nom_bar * BAR * self.kappa_tip * self.a_T(T) * (Q / self.Q_ref) ** n

    def _zf_for(self, N: float, P: float, Q: float, T: float) -> float:
        qp = self.pump.q_drag(N) - Q  # required back-flow
        if qp <= 0:
            return self.p.screw.Z
        G = qp * 12 * self.eta_screw(N, T) / (self.p.screw.W * self.p.screw.H**3 * self.pump.Fp)
        return min(P / G, self.p.screw.Z)

    # ------------------------------------------------------------------ outputs
    def front_index(self) -> int:
        return int((self.p.screw.Z - self.Zf) / self.dz)

    def partial_volume(self) -> float:
        """Volume of compound in the partially filled channel."""
        k = self.front_index()
        s = self.p.screw
        k = min(max(k, 0), self.p.cells)
        full = self._cs[k] * self.dz
        frac = (s.Z - self.Zf) - k * self.dz
        if k < self.p.cells:
            full += self._fl[k] * frac
        return full * s.A

    def _refresh(self) -> None:
        """Cache the cumulative fill profile (the profile only changes once per step)."""
        self._cs = np.concatenate([[0.0], np.cumsum(self.f)]).tolist()
        self._ck = np.concatenate([[0.0], np.cumsum(self.f * self.kc)]).tolist()
        self._fl = self.f.tolist()
        self._kl = self.kc.tolist()

    def partial_kvolume(self) -> float:
        """Sum of kappa * fill * volume over the partially filled channel (torque weight)."""
        k = min(max(self.front_index(), 0), self.p.cells)
        s = self.p.screw
        full = self._ck[k] * self.dz
        frac = (s.Z - self.Zf) - k * self.dz
        if k < self.p.cells:
            full += self._fl[k] * self._kl[k] * frac
        return full * s.A

    def mass(self) -> float:
        """Compound mass in the pumping section, kg (head volume excluded)."""
        s = self.p.screw
        return self.p.compound.rho * (self.partial_volume() + s.A * self.Zf)

    def mech_power(self) -> float:
        N, T = self.N, self.T
        eta = self.eta_screw(N, T)
        s = self.p.screw
        G = max(self.P, 0.0) / max(self.Zf, 1e-4)
        p_full = self.pump.channel_power_per_area(N, eta, G) * s.W * self.Zf
        eta1 = eta / self.kappa_tip  # per unit kappa
        p_part = self.pump.channel_power_per_area(N, eta1, 0.0) * self.partial_kvolume() / s.H
        fill_mean = (self.partial_volume() + s.A * self.Zf) / s.volume
        return p_full + p_part + self.pump.land_power(N, T, self.kappa_tip, fill_mean)

    def dissipation(self) -> float:
        """Viscous heat into the compound: drive power minus the pressure work P*Q."""
        Qs = self.q_screw(self.N, self.P, self.Zf, self.T)
        return max(self.mech_power() - max(self.P, 0.0) * max(Qs, 0.0), 0.0)

    def torque(self) -> float:
        return self.mech_power() / (2 * math.pi * max(self.N, 1e-3)) + self.p.torque_friction

    def electrical_power(self) -> float:
        return (self.torque() * 2 * math.pi * self.N) / self.p.drive_eff + 2000.0

    def truth(self) -> dict:
        Qs = self.q_screw(self.N, self.P, self.Zf, self.T)
        return {
            "t": self.t, "N_rpm": self.N / RPM, "P_bar": self.P / BAR, "Zf": self.Zf,
            "T": self.T, "M": self.mass(), "Q_s": Qs, "Q_out": self.q_out(self.P, self.T),
            "torque": self.torque(), "power": self.electrical_power(),
            "kappa": self.kappa, "kappa_tip": self.kappa_tip, "eps": self.eps, "fill_in": float(self.f[0]),
        }

    # ------------------------------------------------------------------ dynamics
    def step(self, N_sp_rpm: float, dt: float = 0.05) -> None:
        p = self.p
        s = p.screw
        c = p.compound
        hs = dt / p.substeps
        Nsp = min(max(N_sp_rpm, p.N_min_rpm), p.N_max_rpm) * RPM
        ramp = p.ramp_rpm_s * RPM
        for _ in range(p.substeps):
            # drive: ramp-limited setpoint, then first-order speed loop
            d = Nsp - self.Nsp_ramped
            self.Nsp_ramped += max(-ramp * hs, min(ramp * hs, d))
            self.N += (self.Nsp_ramped - self.N) * hs / p.tau_drive
            N, P, Zf, T = self.N, self.P, self.Zf, self.T
            Qd = self.pump.q_drag(N)
            Qs = self.q_screw(N, P, Zf, T)
            k = min(max(self.front_index() - 1, 0), p.cells - 1)
            f_front = self._fl[k]
            # fill front: (1 - f_front) A dZf/dt = f_front Qd - Qs
            zf_min = 2e-3
            denom = max(1.0 - f_front, 0.05) * s.A
            dZf = (f_front * Qd - Qs) / denom
            Zf_new = Zf + dZf * hs
            if Zf_new >= s.Z:  # flooded: the whole section is full
                Zf_new = s.Z
            if Zf_new < zf_min:
                # The filled zone is empty, so the screw delivers only what is available: the
                # inflow at the front plus what is left above the minimum length. Before 0.2.0
                # the length was clamped instead. That created material and held the head at
                # about 9 bar after a run-out had emptied the screw.
                Qs = f_front * Qd + (Zf - zf_min) * denom / hs
                Zf_new = zf_min
            # head pressure
            dP = p.B_eff / p.V_head * (Qs - self.q_out(P, T))
            P_new = max(P + dP * hs, 0.0)
            # temperature of the compound in the section
            M = max(self.mass(), 0.2)
            q_in = Qd * self._fl[0]
            Pd = self.dissipation()
            dT = (Pd + c.rho * q_in * c.cp * (p.T_feed - T) - p.UA * (T - self.T_barrel)) / (M * c.cp)
            self.Zf, self.P, self.T = Zf_new, P_new, T + dT * hs
            self.t += hs
        # partially filled channel: upwind advection with the conveying velocity Qd/A
        Qd = self.pump.q_drag(self.N)
        v = Qd / s.A
        cfl = min(v * dt / self.dz, 1.0)
        f_in = 0.0
        if self.feed_on:
            f_in = min(1.0, self.eps * self.c_in * self.N / max(Qd, 1e-12))
        f_old = self.f.copy()
        self.f[1:] = f_old[1:] - cfl * (f_old[1:] - f_old[:-1])
        self.f[0] = f_old[0] - cfl * (f_old[0] - f_in)
        # viscosity factor travels with the compound (fill-weighted upwind mixing)
        fk = f_old * self.kc
        fk_new = fk.copy()
        fk_new[1:] = fk[1:] - cfl * (fk[1:] - fk[:-1])
        fk_new[0] = fk[0] - cfl * (fk[0] - f_in * self.kappa)
        self.kc = np.where(self.f > 1e-6, fk_new / np.maximum(self.f, 1e-6), self.kappa)
        # cells uncovered by a retreating front take the upstream values
        k = self.front_index()
        if 0 < k < p.cells:
            self.f[k:] = self.f[k - 1]
            self.kc[k:] = self.kc[k - 1]
        self._refresh()

    # ------------------------------------------------------------------ sensors
    def measure(self, dt: float = 0.05) -> dict:
        """Noisy plant signals as the PLC would read them [assumed sensor model, dossier 8.3]."""
        r = self.rng
        tr = self.truth()
        phase = 2 * math.pi * self.N * self.t
        ripple = 0.01 * tr["P_bar"] * math.sin(phase)  # 1 % flight-passing ripple at screw frequency
        P_m = round((tr["P_bar"] + ripple + r.normal(0, 0.3)) * 10) / 10  # 0.1 bar resolution
        tq = tr["torque"] * (1 + 0.005 * math.sin(phase)) + r.normal(0, 0.005 * self.p.torque_rated)
        return {
            "t": tr["t"], "P_bar": P_m, "N_rpm": tr["N_rpm"] + r.normal(0, 0.02),
            "torque": tq, "power": tq * 2 * math.pi * self.N / self.p.drive_eff + 2000 + r.normal(0, 300),
            "T": tr["T"] + r.normal(0, 0.2),
        }


def operating_point(p: Params | None = None, *, P_bar: float = 60.0, N_rpm: float = 30.0,
                    kappa: float = 1.0, seed: int | None = 0) -> Extruder:
    """Gear-pump plant in steady state at screw speed N_rpm and inlet pressure P_bar.

    The gear-pump speed (throughput) is set so that the screw at N_rpm delivers exactly
    the gear-pump volume at P_bar.
    """
    p = p or Params()
    e = Extruder(p, kappa=kappa, seed=seed)
    N = N_rpm * RPM
    Q = e.eps * e.c_in * N
    e.N_gp_rel = Q / (e.q_gp_design * (1 - math.exp(-P_bar / p.P_fill_bar)))
    e.reset_steady(N_rpm)
    return e


def nominal(outlet: str = "gearpump", **kw) -> Extruder:
    p = replace(Params(), outlet=outlet, P_nom_bar=60.0 if outlet == "gearpump" else 150.0)
    return Extruder(p, **kw)
