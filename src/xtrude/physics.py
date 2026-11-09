"""Screw-pumping physics: rheology, drag and pressure flow, power.

Every relation here is a textbook lubrication-approximation result. The references in
the docstrings point to where the relation comes from. "[assumed]" marks a modelling
choice made in this repo.

Units are SI throughout: m, s, Pa, Pa*s, W, N*m, K (temperatures in degC where named
T_*), rev/s for N inside this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import cached_property


@dataclass(frozen=True)
class Compound:
    """Power-law compound with an exponential (approximate Arrhenius) temperature shift.

    eta = kappa * K * gdot**(n-1) * exp(-b (T - T0))

    Source of n, b and T0: Ha, Cho, Kim, Kim, "Finite element analysis of rubber
    extrusion forming process", KSME 2007, Table 1, rubber compound "w6043":
    k = 0.9881e6, n = 0.2226, temperature coefficient 0.8917e-2, reference temperature
    363 K. The published K is used in SI units (Pa*s^n), with natural time 1 s
    [assumed: the units are not stated in the extracted text].

    K is scaled by `k_scale` (default 0.4) so that the nominal drive torque fits the
    drive class of the reference machine. See params.py and the dossier, section 8.2.
    `kappa` is the batch-to-batch viscosity factor (1.0 = nominal batch).
    """

    K: float = 0.9881e6
    n: float = 0.2226
    b: float = 0.8917e-2
    T0: float = 90.0  # degC (363 K)
    k_scale: float = 0.4
    rho: float = 1150.0  # kg/m3  [assumed, typical filled compound]
    cp: float = 1900.0  # J/(kg K)  [assumed]
    gdot_min: float = 0.05  # 1/s, lower cap that keeps the power law finite at rest [assumed]

    def eta(self, gdot: float, T: float, kappa: float = 1.0) -> float:
        g = max(gdot, self.gdot_min)
        return kappa * self.k_scale * self.K * g ** (self.n - 1.0) * math.exp(-self.b * (T - self.T0))


@dataclass(frozen=True)
class Screw:
    """Single-flighted metering (pumping) section.

    Geometry is assumed for a 150 mm cold-feed rubber extruder. A vendor listing of this
    size class gives 4.5-45 rpm, a 185 kW drive and 1100 kg/h maximum output; see
    docs/sources/REFERENCES.md [S5].
    """

    D: float = 0.150  # barrel diameter, m
    lead: float = 0.150  # square pitch [assumed]
    H: float = 0.018  # channel depth, m [assumed, 12 % of D]
    e: float = 0.015  # flight width, m [assumed]
    delta: float = 0.5e-3  # flight clearance, m [assumed]
    L: float = 1.2  # axial length of the pumping section, m [assumed, 8 D]

    @cached_property
    def phi(self) -> float:
        """Helix angle at the barrel."""
        return math.atan(self.lead / (math.pi * self.D))

    @cached_property
    def W(self) -> float:
        """Channel width perpendicular to the flight."""
        return self.lead * math.cos(self.phi) - self.e

    @cached_property
    def Z(self) -> float:
        """Down-channel length of the pumping section."""
        return self.L / math.sin(self.phi)

    @cached_property
    def A(self) -> float:
        """Channel cross-section W*H."""
        return self.W * self.H

    @cached_property
    def volume(self) -> float:
        return self.A * self.Z


def shape_factors(H: float, W: float, terms: int = 2000) -> tuple[float, float]:
    """Drag and pressure-flow shape factors for a rectangular channel (Newtonian).

    Fd = 16 W/(pi^3 H) * sum_odd tanh(i pi H / 2W)/i^3
    Fp = 1 - 192 H/(pi^5 W) * sum_odd tanh(i pi W / 2H)/i^5

    Tadmor & Gogos, Principles of Polymer Processing, 2nd ed., Wiley 2006, Sec. 6.3,
    flat-plate model of the single-screw pump; also Rauwendaal, Polymer Extrusion,
    Ch. 7 (melt conveying). These are standard results, quoted from memory: the page
    numbers were not re-checked. The Fd series converges like 1/i^2 for shallow channels,
    so 2000 terms are used. With 15 terms, Fd at H/W = 0.001 was 1.4 % low.
    """
    sd = sum(math.tanh(i * math.pi * H / (2 * W)) / i**3 for i in range(1, 2 * terms, 2))
    sp = sum(math.tanh(i * math.pi * W / (2 * H)) / i**5 for i in range(1, 2 * terms, 2))
    Fd = 16 * W / (math.pi**3 * H) * sd
    Fp = 1 - 192 * H / (math.pi**5 * W) * sp
    return Fd, Fp


@dataclass(frozen=True)
class Pump:
    """Precomputed pumping coefficients for one screw and one compound."""

    screw: Screw = field(default_factory=Screw)
    compound: Compound = field(default_factory=Compound)

    def __post_init__(self) -> None:
        Fd, Fp = shape_factors(self.screw.H, self.screw.W)
        object.__setattr__(self, "Fd", Fd)
        object.__setattr__(self, "Fp", Fp)

    def vb(self, N: float) -> float:
        """Barrel surface speed relative to the screw, m/s (N in rev/s)."""
        return math.pi * self.screw.D * N

    @cached_property
    def cos_phi(self) -> float:
        return math.cos(self.screw.phi)

    @cached_property
    def sin_phi(self) -> float:
        return math.sin(self.screw.phi)

    def gdot_channel(self, N: float) -> float:
        """Nominal channel shear rate Vb/H, used to evaluate eta [Rauwendaal, Ch. 7]."""
        return self.vb(N) / self.screw.H

    def q_drag(self, N: float) -> float:
        """Drag flow Qd = 1/2 Vbz W H Fd."""
        s = self.screw
        return 0.5 * self.vb(N) * self.cos_phi * s.W * s.H * self.Fd

    def q_pressure(self, G: float, eta: float) -> float:
        """Pressure back-flow Qp = W H^3 Fp G /(12 eta), G = dP/dz down-channel > 0."""
        s = self.screw
        return s.W * s.H**3 * self.Fp * G / (12.0 * eta)

    def channel_power_per_area(self, N: float, eta: float, G: float) -> float:
        """Drive power per unit channel area for a full channel.

        Barrel-wall work for the lubrication velocity field: the down-channel
        Couette-Poiseuille profile plus the closed cross-channel circulation.
        p = eta (Vbz^2 + 4 Vbx^2)/H + G H Vbz / 2
        Derived here from the wall shear stress. It matches the cos^2 + 4 sin^2 form in
        Tadmor & Gogos Sec. 6.3 (power in the metering section; page not re-checked).
        """
        vb = self.vb(N)
        vz = vb * self.cos_phi
        vx = vb * self.sin_phi
        return eta * (vz * vz + 4.0 * vx * vx) / self.screw.H + 0.5 * G * self.screw.H * vz

    def land_power(self, N: float, T: float, kappa: float, fill: float) -> float:
        """Viscous power in the flight clearance, eta evaluated at Vb/delta.

        Rauwendaal, Ch. 7 (power in the flight clearance). Scaled by the mean fill
        [assumed].
        """
        s = self.screw
        vb = self.vb(N)
        eta_f = self.compound.eta(vb / s.delta, T, kappa)
        return eta_f * vb * vb / s.delta * s.e * s.Z * fill
