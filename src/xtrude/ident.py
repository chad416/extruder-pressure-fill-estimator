"""Identification of the speed -> pressure dynamics from a technician-style test.

The test runs open loop around the operating point:
- 10 s at rest
- a speed doublet (+a, -a, +a, 5 s each)
- a pseudo-random binary sequence held for 0.1-3 s
- 10 s at rest

The test is about two minutes long. The amplitude is small, 0.3 rpm (1 % of speed). The
gear-pump fill curve makes the plant gain depend on the pressure: at ±1 rpm a linear
model reaches only 85 % fit even on noise-free data, and at ±0.1 rpm it reaches 98 %
(results/identification.md).

The model is output error (OE), y = B(q^-1)/A(q^-1) u, fitted by nonlinear least
squares on the free-run simulation error. This is consistent for white measurement
noise, whereas an equation-error ARX fit is biased under the 0.3 bar noise and the
flight-passing ripple. See Ljung, *System Identification: Theory for the User*, 2nd ed.,
Prentice Hall 1999, Sec. 4.2 (OE model structure) and Ch. 7–8 (consistency); sections
not re-checked.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

from .plant import RPM, Extruder

TS = 0.1  # controller sample time, s (one OB30 call)


@dataclass
class Record:
    t: np.ndarray
    u: np.ndarray  # speed setpoint, rpm
    y: np.ndarray  # measured pressure, bar
    torque: np.ndarray
    power: np.ndarray
    n_meas: np.ndarray
    T: np.ndarray
    u0: float
    y0: float

    def to_csv(self, path: str) -> None:
        a = np.column_stack([self.t, self.u, self.y, self.torque, self.power, self.n_meas, self.T])
        np.savetxt(path, a, delimiter=",", header="t_s,N_sp_rpm,P_bar,torque_Nm,power_W,N_rpm,T_C",
                   comments="", fmt="%.6g")

    @classmethod
    def from_csv(cls, path: str) -> "Record":
        a = np.loadtxt(path, delimiter=",", skiprows=1)
        n0 = max(int(5 / TS), 1)
        return cls(a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6],
                   float(np.median(a[:n0, 1])), float(np.mean(a[:n0, 2])))


def prbs(n: int, hold: int, seed: int) -> np.ndarray:
    """+-1 sequence, each level held for a random 1..hold samples."""
    r = np.random.default_rng(seed)
    out = np.empty(n)
    i, lvl = 0, 1.0
    while i < n:
        k = int(r.integers(1, hold + 1))
        out[i:i + k] = lvl
        lvl = -lvl
        i += k
    return out


def test_signal(n: int, seed: int) -> np.ndarray:
    seq = np.zeros(n)
    k0 = int(10 / TS)
    d = int(5 / TS)
    seq[k0:k0 + d] = 1.0
    seq[k0 + d:k0 + 3 * d] = -1.0
    seq[k0 + 3 * d:k0 + 4 * d] = 1.0
    k1 = k0 + 4 * d + int(5 / TS)
    k2 = n - int(10 / TS)
    if k2 > k1:
        seq[k1:k2] = prbs(k2 - k1, int(3 / TS), seed)
    return seq


def step_test(plant: Extruder, amp_rpm: float = 0.3, seed: int = 1, duration: float = 120.0,
              noise: bool = True) -> Record:
    n = int(round(duration / TS))
    u0 = plant.N / RPM
    seq = test_signal(n, seed)
    cols = {k: np.empty(n) for k in ("t", "u", "y", "tq", "pw", "nm", "T")}
    y0 = plant.truth()["P_bar"]
    for k in range(n):
        u = u0 + amp_rpm * seq[k]
        plant.step(u, TS)
        m = plant.measure() if noise else plant.truth()
        cols["t"][k] = plant.t
        cols["u"][k] = u
        cols["y"][k] = m["P_bar"]
        cols["tq"][k] = m["torque"]
        cols["pw"][k] = m["power"]
        cols["nm"][k] = m["N_rpm"]
        cols["T"][k] = m["T"]
    return Record(cols["t"], cols["u"], cols["y"], cols["tq"], cols["pw"], cols["nm"], cols["T"], u0, y0)


@dataclass
class Model:
    """Discrete plant model y = q^-nk B(q^-1)/A(q^-1) u, deviations in rpm -> bar."""

    a: np.ndarray  # [1, a1, ..., ana]
    b: np.ndarray  # [0]*nk + [b0, ...]
    fit_pct: float = float("nan")
    rmse: float = float("nan")

    def simulate(self, du: np.ndarray) -> np.ndarray:
        return lfilter(self.b, self.a, du)

    @property
    def dc_gain(self) -> float:
        return float(np.sum(self.b) / np.sum(self.a))

    @property
    def poles(self) -> np.ndarray:
        return np.roots(self.a)

    def freqresp(self, w: np.ndarray) -> np.ndarray:
        z = np.exp(-1j * w * TS)
        return np.polyval(self.b[::-1], z) / np.polyval(self.a[::-1], z)


def fit_oe(rec: Record, na: int = 3, nb: int = 3, nk: int = 1) -> Model:
    u = rec.u - rec.u0
    y = rec.y - rec.y0

    def sim(x):
        return lfilter(np.r_[np.zeros(nk), x[na:]], np.r_[1.0, x[:na]], u)

    best = None
    for p0 in (np.linspace(0.98, 0.6, na), np.linspace(0.95, 0.4, na)):
        x0 = np.r_[np.poly(p0)[1:], np.full(nb, 0.05)]

        def res(x):
            if np.any(np.abs(np.roots(np.r_[1.0, x[:na]])) >= 0.9995):
                return 1e3 * np.ones_like(y)
            return sim(x) - y

        s = least_squares(res, x0)
        if best is None or s.cost < best.cost:
            best = s
    x = best.x
    m = Model(np.r_[1.0, x[:na]], np.r_[np.zeros(nk), x[na:]])
    e = m.simulate(u) - y
    m.rmse = float(np.sqrt(np.mean(e**2)))
    m.fit_pct = float(100 * (1 - np.linalg.norm(e) / np.linalg.norm(y - y.mean())))
    return m


def validate(m: Model, rec: Record) -> tuple[float, float]:
    """Free-run RMSE (bar) and NRMSE fit (%) on another record."""
    u = rec.u - rec.u0
    y = rec.y - rec.y0
    e = m.simulate(u) - y
    return float(np.sqrt(np.mean(e**2))), float(100 * (1 - np.linalg.norm(e) / np.linalg.norm(y - y.mean())))


def true_model(plant_factory, amp_rpm: float = 0.1, na: int = 3, nb: int = 3) -> Model:
    """Noise-free small-signal model of a plant variant (the 'truth' for robustness checks)."""
    return fit_oe(step_test(plant_factory(), amp_rpm=amp_rpm, seed=7, duration=180.0, noise=False), na, nb)
