"""Batch-fed twin-screw roller-die unit (TSRD) under an internal mixer: a structural model.

Why this second plant exists. The cold-feed extruder of `plant.py` is fed continuously and
discharges into a gear pump. A published thesis on pressure control in tyre mixing
(docs/sources/REFERENCES.md [T1]) describes a different machine for the same control
problem: a twin-screw unit that takes whole batches from the mixer above it and pushes the
compound through a roller die (a two-roll calender fed under pressure). The pressure in
front of the rollers is held by the screw speed with a PI controller (Siemens PID_Compact),
after a filling phase at fixed speed. The thesis reports three things a model must show:
- the pressure rises in jumps while the screw speed rises smoothly;
- the feed arrives in batches, so the screws sometimes run short of material and the
  dynamics change;
- the settling of the start-up depends strongly on the recipe.

What the model is. A lumped mass model with the right structure, not a flow simulation of
conical twin screws. Every parameter is [assumed]; none comes from a real machine.

States
- m_h     compound in the hopper (kg), with its mass-averaged viscosity factor
- f[i]    fill of the conveying volume of the screws, carried forward by the screw rotation
- V_ch    compound in the chamber between screw tips and rollers while it is still filling
- P       chamber pressure once the chamber is full (Pa)
- N       screw speed behind a speed-controlled drive

Flows
- intake     Q_in  = V_rev N eps(m_h),  eps = eps_max (1 - exp(-m_h / m_c))
             the screws take less per revolution when little is left in the hopper
- delivery   Q_del = f_tip V_rev N      positive conveying of what has arrived at the tips
- leakage    Q_lk  = l Q_ref P / (P_nom kappa)     back through the screw clearances
- roller die Q_out = Q_ref [(1 - s) u_roll + s u_roll^(1-n) P / (P_nom kappa)]
             drag flow carried by the rolls plus pressure flow through the nip. This is the
             Reynolds (lubrication) equation for two rolls turning at the same speed: the
             throughput is a drag term and a term linear in the inlet pressure over the
             viscosity. The viscosity is taken at the roll shear rate, as plant.py takes it
             at the channel shear rate, so it scales with u_roll^(n-1). The chamber
             therefore sits at
                 P = P_nom kappa (Q_net / Q_ref - (1 - s) u_roll) / (s u_roll^(1-n)),
             and nothing at all while the screws deliver less than the rolls drag away.
             When a filling chamber becomes full, the pressure steps from zero to that
             value: the "jump". The gain from speed to pressure is proportional to the
             viscosity factor kappa, which is why the recipe matters.

While the chamber is not yet full it carries no pressure, and the rolls take what the bank
in front of them can feed. That is the filling phase of a start-up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

RPM = 1.0 / 60.0
BAR = 1e5


@dataclass(frozen=True)
class TsrdParams:
    rho: float = 1150.0  # kg/m3
    n: float = 0.2226  # power-law index, same published compound as plant.py
    # screws (both together) [assumed]
    V_rev: float = 10.7e-3  # m3 conveyed per revolution at full fill
    V_screw: float = 60e-3  # conveying volume
    cells: int = 30
    eps_max: float = 0.62  # fill taken in per revolution from a well-filled hopper
    m_c: float = 25.0  # kg; below about 2 m_c the intake starves
    leak: float = 0.05  # back-leakage as a fraction of Q_ref at nominal pressure
    # chamber and roller die [assumed]
    V_ch: float = 40e-3  # m3 between screw tips and rollers
    V_bank: float = 20e-3  # m3 of bank at which the rolls are fully fed without pressure
    B_eff: float = 0.1e9  # Pa, compound with entrained air
    P_nom_bar: float = 20.0
    Q_ref: float = 1.28e-3  # m3/s at nominal (about 5.3 t/h)
    s_p: float = 0.30  # share of pressure flow in the nip throughput at nominal
    # drive [assumed]
    N_nom_rpm: float = 12.0
    N_min_rpm: float = 2.0
    N_max_rpm: float = 30.0
    tau_drive: float = 0.5
    ramp_rpm_s: float = 2.0
    P_rated_W: float = 250e3
    torque_friction: float = 3000.0  # N m
    c_visc: float = 1100.0  # N m per kg in the screws at nominal speed and kappa = 1
    # batches from the mixer [assumed]
    m_batch: float = 220.0  # kg
    substeps: int = 2  # 50 ms; the pressure step is implicit, the conveying CFL stays below 0.3

    @property
    def torque_rated(self) -> float:
        return self.P_rated_W / (2 * math.pi * self.N_max_rpm * RPM)

    @property
    def batch_period(self) -> float:
        """Mixer cycle that balances the nominal throughput, s."""
        return self.m_batch / (self.rho * self.Q_ref)


class Tsrd:
    """Simulator. Call `step(N_sp_rpm, dt)` once per controller sample."""

    def __init__(self, p: TsrdParams | None = None, *, seed: int | None = 0):
        self.p = p = p or TsrdParams()
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.u_roll = 1.0  # roller speed relative to nominal
        self.p_zero_bar = 0.0  # transducer zero offset, as in plant.py
        self.dv = p.V_screw / p.cells
        self.empty()

    # ------------------------------------------------------------------ states
    def empty(self) -> None:
        """Machine empty and stopped at minimum speed: the state before a start-up."""
        p = self.p
        self.m_h = 0.0
        self.k_h = 1.0
        self.f = np.zeros(p.cells)
        self.kc = np.ones(p.cells)
        self.V_ch = 0.0
        self.k_ch = 1.0
        self.P = 0.0
        self.N = p.N_min_rpm * RPM
        self.Nsp_ramped = self.N
        self.m_out = 0.0  # compound that has left through the rollers, kg
        self.m_in = 0.0  # compound dropped into the hopper, kg
        self._q_out = 0.0
        self._q_del = 0.0

    def steady(self, m_h: float = 120.0, kappa: float = 1.0) -> None:
        """Running at the nominal point with `m_h` kg in the hopper."""
        p = self.p
        self.empty()
        self.m_h, self.k_h = m_h, kappa
        eps = self.eps()
        self.N = self.Nsp_ramped = p.Q_ref / (p.V_rev * eps)  # intake = roller-die throughput
        self.f[:] = eps
        self.f[-1] = eps * (1 + p.leak)  # the leakage is re-delivered from the tip
        self.kc[:] = kappa
        self.V_ch, self.k_ch = p.V_ch, kappa
        self.P = p.P_nom_bar * BAR * kappa
        self.m_in = self.mass_total()

    def drop_batch(self, m: float | None = None, kappa: float = 1.0) -> None:
        m = self.p.m_batch if m is None else m
        self.k_h = (self.k_h * self.m_h + kappa * m) / (self.m_h + m)
        self.m_h += m
        self.m_in += m

    # ------------------------------------------------------------------ physics
    def eps(self) -> float:
        return self.p.eps_max * (1.0 - math.exp(-self.m_h / self.p.m_c))

    def q_leak(self, P: float) -> float:
        return self.p.leak * self.p.Q_ref * max(P, 0.0) / (self.p.P_nom_bar * BAR * self.k_ch)

    def q_roll(self, P: float) -> float:
        """Roller-die throughput with a full, pressurised chamber."""
        p = self.p
        x = max(P, 0.0) / (p.P_nom_bar * BAR * self.k_ch)
        return p.Q_ref * ((1 - p.s_p) * self.u_roll + p.s_p * self.u_roll ** (1 - p.n) * x)

    def pressure_for(self, q_net: float) -> float:
        """Static chamber pressure at which the nip takes q_net (leakage neglected)."""
        p = self.p
        ex = (q_net / p.Q_ref - (1 - p.s_p) * self.u_roll) / (p.s_p * self.u_roll ** (1 - p.n))
        return p.P_nom_bar * BAR * self.k_ch * max(ex, 0.0)

    def mass_screws(self) -> float:
        return self.p.rho * self.dv * float(np.sum(self.f))

    def mass_chamber(self) -> float:
        """Compound in the chamber, including what the pressure has compressed into it."""
        return self.p.rho * (self.V_ch + self.p.V_ch * self.P / self.p.B_eff)

    def mass_total(self) -> float:
        return self.m_h + self.mass_screws() + self.mass_chamber()

    def torque(self) -> float:
        p = self.p
        N = max(self.N, 1e-3)
        k_mean = float(np.sum(self.f * self.kc)) / max(float(np.sum(self.f)), 1e-9)
        visc = p.c_visc * k_mean * (N / (p.N_nom_rpm * RPM)) ** p.n * self.mass_screws()
        return p.torque_friction + visc + self.P * max(self._q_del, 0.0) / (2 * math.pi * N)

    def truth(self) -> dict:
        return {"t": self.t, "N_rpm": self.N / RPM, "P_bar": self.P / BAR, "m_h": self.m_h,
                "m_s": self.mass_screws(), "m_ch": self.mass_chamber(), "M": self.mass_total(),
                "eps": self.eps(), "f_tip": float(self.f[-1]), "Q_out": self._q_out,
                "torque": self.torque(), "full": float(self.V_ch >= self.p.V_ch)}

    # ------------------------------------------------------------------ dynamics
    def step(self, N_sp_rpm: float, dt: float = 0.1) -> None:
        p = self.p
        hs = dt / p.substeps
        Nsp = min(max(N_sp_rpm, p.N_min_rpm), p.N_max_rpm) * RPM
        ramp = p.ramp_rpm_s * RPM
        for _ in range(p.substeps):
            d = Nsp - self.Nsp_ramped
            self.Nsp_ramped += max(-ramp * hs, min(ramp * hs, d))
            self.N += (self.Nsp_ramped - self.N) * hs / p.tau_drive
            N = self.N
            # --- intake from the hopper, limited by what is there
            v_turn = p.V_rev * N * hs  # volume swept in this substep
            v_in = min(v_turn * self.eps(), self.m_h / p.rho)
            # --- conveying: upwind transport of fill and of fill x viscosity factor
            cfl = min(v_turn / self.dv, 1.0)
            f_old, fk_old = self.f.copy(), self.f * self.kc
            v_del = cfl * f_old[-1] * self.dv  # volume arriving at the tips
            self.f[1:] = f_old[1:] - cfl * (f_old[1:] - f_old[:-1])
            self.f[0] = f_old[0] - cfl * f_old[0] + v_in / self.dv
            fk = fk_old.copy()
            fk[1:] = fk_old[1:] - cfl * (fk_old[1:] - fk_old[:-1])
            fk[0] = fk_old[0] - cfl * fk_old[0] + self.k_h * v_in / self.dv
            k_del = fk_old[-1] / f_old[-1] if f_old[-1] > 1e-9 else self.k_ch
            self.kc = np.where(self.f > 1e-9, fk / np.maximum(self.f, 1e-9), self.k_h)
            self.m_h -= p.rho * v_in
            # --- chamber
            if self.V_ch < p.V_ch:  # filling, no pressure
                v_out = min((1 - p.s_p) * p.Q_ref * self.u_roll * min(self.V_ch / p.V_bank, 1.0) * hs,
                            self.V_ch + v_del)
                v_new = self.V_ch + v_del - v_out
                if v_del > 0:
                    self.k_ch += (k_del - self.k_ch) * min(v_del / max(self.V_ch + v_del, 1e-9), 1.0)
                if v_new >= p.V_ch:  # just filled: the surplus compresses the chamber
                    self.P = p.B_eff / p.V_ch * (v_new - p.V_ch)
                    v_new = p.V_ch
                self.V_ch, v_leak = v_new, 0.0
            else:  # full: implicit pressure step (the chamber is stiff)
                self.k_ch += (k_del - self.k_ch) * min(v_del / p.V_ch, 1.0)
                c = p.B_eff / p.V_ch
                P0, Pn = self.P, self.P
                for _ in range(30):
                    g = Pn - P0 - c * (v_del - (self.q_leak(Pn) + self.q_roll(Pn)) * hs)
                    dq = (self.q_roll(Pn + 100.0) + self.q_leak(Pn + 100.0)
                          - self.q_roll(Pn) - self.q_leak(Pn)) / 100.0
                    step = g / (1.0 + c * dq * hs)
                    Pn -= step
                    if abs(step) < 1.0:
                        break
                if Pn <= 0.0:  # the rolls take more than arrives: the chamber starts to empty
                    v_avail = self.V_ch + P0 / c + v_del  # includes the decompressed volume
                    v_out = min((1 - p.s_p) * p.Q_ref * self.u_roll * hs, v_avail)
                    v_new = v_avail - v_out
                    self.P = c * max(v_new - p.V_ch, 0.0)
                    self.V_ch, v_leak = min(v_new, p.V_ch), 0.0
                else:
                    v_leak = self.q_leak(Pn) * hs
                    v_out = v_del - v_leak - (Pn - P0) / c
                    self.P = Pn
            # leakage goes back into the last screw cell; overflow backs up along the screws
            if v_leak > 0:
                self.f[-1] += v_leak / self.dv
                for i in range(p.cells - 1, 0, -1):
                    if self.f[i] <= 1.0:
                        break
                    self.f[i - 1] += self.f[i] - 1.0
                    self.f[i] = 1.0
            self.m_out += p.rho * v_out
            self._q_out = v_out / hs
            self._q_del = v_del / hs
            self.t += hs

    # ------------------------------------------------------------------ sensors
    def measure(self) -> dict:
        """Signals as the PLC reads them [assumed sensor model]."""
        r = self.rng
        tr = self.truth()
        phase = 2 * math.pi * 2 * self.N * self.t  # two screw tips per revolution
        P_m = tr["P_bar"] * (1 + 0.01 * math.sin(phase)) + r.normal(0, 0.1) + self.p_zero_bar
        tq = tr["torque"] * (1 + 0.005 * math.sin(phase)) + r.normal(0, 0.005 * self.p.torque_rated)
        return {"t": self.t, "P_bar": round(P_m * 20) / 20, "N_rpm": tr["N_rpm"] + r.normal(0, 0.01),
                "torque": tq, "u_roll": self.u_roll}
