"""Pressure controllers: baseline PID, RST pole placement, small linear MPC.

All three are written in the q^-1 (backward shift) domain at TS = 0.1 s. They are
analysed the same way, through their linear feedback part R/S on identified or "true"
small-signal models.

References (sections quoted from the books' tables of contents, pages not re-checked):
- [AW] K. J. Astrom, B. Wittenmark, *Computer-Controlled Systems: Theory and Design*,
  3rd ed., Prentice Hall 1997.
  - Ch. 5: pole placement, polynomial approach, Diophantine equation.
  - Ch. 9 / Sec. 9.4: anti-windup for polynomial controllers through an observer
    polynomial A_o.
  - Ch. 8: the digital PID.
- [LZ] I. D. Landau, G. Zito, *Digital Control Systems*, Springer 2006, Ch. 2–3.
  - RST structure and fixed parts Hs/Hr.
  - Robustness margins: modulus margin dM = 1/Ms >= 0.5, delay margin >= Ts.
- [AH] K. J. Astrom, T. Hagglund, *Advanced PID Control*, ISA 2006.
  - Ms as the robustness measure.
  - Back-calculation anti-windup.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .ident import TS, Model

# ---------------------------------------------------------------- polynomial helpers


def pmul(*ps: np.ndarray) -> np.ndarray:
    out = np.array([1.0])
    for p in ps:
        out = np.convolve(out, p)
    return out


def padd(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    n = max(len(a), len(b))
    return np.pad(a, (0, n - len(a))) + np.pad(b, (0, n - len(b)))


def peval(p: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Evaluate p(q^-1) = p0 + p1 q^-1 + ... at q = z."""
    zi = 1.0 / z
    return np.polyval(p[::-1], zi)


def cont_poles_to_poly(w0: float, zeta: float) -> np.ndarray:
    """Second-order continuous poles -> discrete polynomial 1 + p1 q^-1 + p2 q^-2."""
    if zeta < 1:
        r = math.exp(-zeta * w0 * TS)
        return np.array([1.0, -2 * r * math.cos(w0 * TS * math.sqrt(1 - zeta**2)), r * r])
    s1 = -w0 * (zeta - math.sqrt(zeta**2 - 1))
    s2 = -w0 * (zeta + math.sqrt(zeta**2 - 1))
    return pmul(np.array([1, -math.exp(s1 * TS)]), np.array([1, -math.exp(s2 * TS)]))


# ---------------------------------------------------------------- RST representation


@dataclass
class RST:
    """Controller S(q^-1) u = T(q^-1) r - R(q^-1) y (r already reference-model filtered)."""

    R: np.ndarray
    S: np.ndarray
    T: np.ndarray
    Bm: np.ndarray = field(default_factory=lambda: np.array([1.0]))
    Am: np.ndarray = field(default_factory=lambda: np.array([1.0]))
    Ao: np.ndarray | None = None  # anti-windup observer polynomial, monic, deg = deg S
    name: str = "rst"


def solve_diophantine(Ap: np.ndarray, B: np.ndarray, P: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Minimal-degree solution of Ap S + B R = P ([AW] Sec. 5.2, Sylvester matrix).

    deg S = deg B - 1, deg R = deg Ap - 1, deg P <= deg Ap + deg B - 1.
    """
    na, nb = len(Ap) - 1, len(B) - 1
    ns, nr = nb - 1, na - 1
    n = na + nb
    if len(P) - 1 > n - 1:
        raise ValueError("deg P too high for the minimal solution")
    M = np.zeros((n, n))
    for j in range(ns + 1):
        M[j:j + na + 1, j] += Ap
    for j in range(nr + 1):
        M[j:j + nb + 1, ns + 1 + j] += B
    rhs = np.pad(P, (0, n - len(P)))
    x = np.linalg.solve(M, rhs)
    return x[:ns + 1], x[ns + 1:]


def design_rst(model: Model, w0: float, zeta: float = 0.9, aux: float = 0.5, n_aux: int | None = None,
               wm: float | None = None, zm: float = 1.0, notch_hr: np.ndarray | None = None,
               ao_pole: float = 0.6) -> RST:
    """Pole placement with an integrator in S (Hs = 1 - q^-1).

    - Dominant closed-loop poles: continuous w0, zeta.
    - Remaining poles: real auxiliary poles at `aux` [LZ, Sec. 3.3].
    - Tracking: T = P / B(1), driven through the reference model Bm/Am (w = wm), so that
      tracking and regulation are tuned separately (two degrees of freedom).
    """
    Hs = np.array([1.0, -1.0])
    Hr = np.array([1.0]) if notch_hr is None else notch_hr
    A = model.a
    B = model.b
    Ap = pmul(A, Hs)
    Bp = pmul(B, Hr)
    n_p = len(Ap) - 1 + len(Bp) - 1 - 1  # max deg P
    Pd = cont_poles_to_poly(w0, zeta)
    k_aux = n_p - 2 if n_aux is None else n_aux
    P = Pd
    for _ in range(max(k_aux, 0)):
        P = pmul(P, np.array([1.0, -aux]))
    S1, R1 = solve_diophantine(Ap, Bp, P)
    S = pmul(S1, Hs)
    R = pmul(R1, Hr)
    Bsum = float(np.sum(B))
    T = P / Bsum
    wm = wm if wm is not None else w0
    Am = cont_poles_to_poly(wm, zm)
    Bm = np.array([np.sum(Am)])  # unit static gain, no zeros
    ds = len(S) - 1
    Ao = pmul(*[np.array([1.0, -ao_pole])] * ds) if ds > 0 else np.array([1.0])
    return RST(R, S, T, Bm, Am, Ao, name=f"rst w0={w0:g}")


def pid_as_rst(Kp: float, Ti: float, Td: float, Nf: float = 10.0, b: float = 1.0) -> RST:
    """The discrete PID used in FB_PressurePID, written as R/S for analysis.

    u = Kp (b r - y) + I + D
    I(k) = I(k-1) + Kp TS/Ti (r - y)                 (backward Euler, current error)
    D(k) = ad D(k-1) - kd (y(k) - y(k-1)),  ad = Td/(Td + Nf TS), kd = Kp Td Nf/(Td + Nf TS)
    """
    Ki = Kp * TS / Ti if Ti > 0 else 0.0
    ad = Td / (Td + Nf * TS) if Td > 0 else 0.0
    kd = Kp * Td * Nf / (Td + Nf * TS) if Td > 0 else 0.0
    one_m = np.array([1.0, -1.0])
    den_d = np.array([1.0, -ad])
    S = pmul(one_m, den_d)
    R = padd(padd(Kp * pmul(one_m, den_d), Ki * den_d), kd * pmul(one_m, one_m))
    T = padd(b * Kp * pmul(one_m, den_d), Ki * den_d)
    return RST(R, S, T, name=f"pid Kp={Kp:.3g} Ti={Ti:.3g} Td={Td:.3g}")


# ---------------------------------------------------------------- robustness analysis

W = np.logspace(-3, math.log10(math.pi / TS * 0.999), 1500)


@dataclass
class Margins:
    Ms: float
    Mt: float
    gm: float  # gain margin (ratio)
    pm_deg: float
    dm_s: float  # delay margin
    wc: float  # gain crossover rad/s
    stable: bool


def loop(ctrl: RST, plant: Model, w: np.ndarray = W) -> np.ndarray:
    z = np.exp(1j * w * TS)
    return peval(plant.b, z) * peval(ctrl.R, z) / (peval(plant.a, z) * peval(ctrl.S, z))


def closed_loop_poles(ctrl: RST, plant: Model) -> np.ndarray:
    Pc = padd(pmul(plant.a, ctrl.S), pmul(plant.b, ctrl.R))
    return np.roots(Pc)


def margins(ctrl: RST, plant: Model) -> Margins:
    L = loop(ctrl, plant)
    S = 1 / (1 + L)
    Tc = L / (1 + L)
    stable = bool(np.all(np.abs(closed_loop_poles(ctrl, plant)) < 1.0))
    mag = np.abs(L)
    ph = np.unwrap(np.angle(L))
    # gain crossovers
    idx = np.where(np.diff(np.sign(mag - 1.0)) != 0)[0]
    pms, dms, wcs = [], [], []
    for i in idx:
        wc = W[i]
        pm = (math.degrees(ph[i]) + 180.0) % 360.0
        pm = pm if pm <= 180 else pm - 360
        pms.append(pm)
        wcs.append(wc)
        dms.append(math.radians(pm) / wc if pm > 0 else 0.0)
    # phase crossovers (-180 deg)
    idx2 = np.where(np.diff(np.sign(np.sin(ph) * 0 + ((np.degrees(ph) + 180) % 360) - 180)) != 0)[0]
    gms = [1 / mag[i] for i in idx2 if mag[i] > 1e-9 and abs(((math.degrees(ph[i]) + 180) % 360) - 180) > 90]
    return Margins(
        Ms=float(np.max(np.abs(S))), Mt=float(np.max(np.abs(Tc))),
        gm=float(min(gms)) if gms else float("inf"),
        pm_deg=float(min(pms)) if pms else float("nan"),
        dm_s=float(min(dms)) if dms else float("nan"),
        wc=float(max(wcs)) if wcs else float("nan"),
        stable=stable,
    )


def load_response(ctrl: RST, plant: Model, n: int = 600, d_rpm: float = 1.0) -> np.ndarray:
    """Pressure deviation after a step input disturbance of d_rpm (feed loss equivalent)."""
    # y = B/(A S + B R) * S * d
    Pc = padd(pmul(plant.a, ctrl.S), pmul(plant.b, ctrl.R))
    num = pmul(plant.b, ctrl.S)
    from scipy.signal import lfilter
    return lfilter(num, Pc, d_rpm * np.ones(n))


def setpoint_response(ctrl: RST, plant: Model, n: int = 400) -> tuple[np.ndarray, np.ndarray]:
    """Output and controller output for a 1 bar setpoint step through Bm/Am."""
    from scipy.signal import lfilter
    r = lfilter(ctrl.Bm, ctrl.Am, np.ones(n))
    Pc = padd(pmul(plant.a, ctrl.S), pmul(plant.b, ctrl.R))
    y = lfilter(pmul(plant.b, ctrl.T), Pc, r)
    u = lfilter(pmul(plant.a, ctrl.T), Pc, r)
    return y, u


# ---------------------------------------------------------------- runtime controllers (float64 reference)


class RSTRuntime:
    """S u = T r* - R y with saturation and the observer anti-windup of [AW] Sec. 9.4.

    Ao v = T r* - R y + (Ao - S) u_sat,   u_sat = sat(v)
    S and Ao are monic, so v(k) only needs past u_sat and v. Without saturation this
    reduces to S u = T r* - R y. Feedforward u_ff is added after the controller, and
    the history keeps only the controller's own share (u_sat - u_ff).
    """

    def __init__(self, c: RST, u_lo: float, u_hi: float, du_max: float):
        self.c = c
        self.u_lo, self.u_hi, self.du_max = u_lo, u_hi, du_max
        self.D = padd(c.Ao, -c.S)  # Ao - S, D[0] = 0
        n = max(len(c.S), len(c.Ao), len(c.T), len(c.R))
        self.r = np.zeros(n)
        self.y = np.zeros(n)
        self.us = np.zeros(n)  # controller share of u_sat, us[0] = latest
        self.v = np.zeros(n)
        self.rm_in = np.zeros(len(c.Bm))
        self.rm_out = np.zeros(len(c.Am))
        self.u_last = 0.0

    def init(self, r: float, y: float, u: float) -> None:
        self.r[:] = r
        self.y[:] = y
        self.us[:] = u
        self.v[:] = u
        self.rm_in[:] = r
        self.rm_out[:] = r
        self.u_last = u

    def step(self, r: float, y: float, u_ff: float = 0.0) -> float:
        c = self.c
        self.rm_in = np.roll(self.rm_in, 1)
        self.rm_in[0] = r
        rs = float(np.dot(c.Bm, self.rm_in) - np.dot(c.Am[1:], self.rm_out[:len(c.Am) - 1]))
        self.rm_out = np.roll(self.rm_out, 1)
        self.rm_out[0] = rs
        self.r = np.roll(self.r, 1)
        self.r[0] = rs
        self.y = np.roll(self.y, 1)
        self.y[0] = y
        acc = float(np.dot(c.T, self.r[:len(c.T)]) - np.dot(c.R, self.y[:len(c.R)]))
        acc += float(np.dot(self.D[1:], self.us[:len(self.D) - 1]))
        acc -= float(np.dot(c.Ao[1:], self.v[:len(c.Ao) - 1]))
        v = acc
        u = v + u_ff
        u = min(max(u, self.u_last - self.du_max), self.u_last + self.du_max)
        u = min(max(u, self.u_lo), self.u_hi)
        self.v = np.roll(self.v, 1)
        self.v[0] = v
        self.us = np.roll(self.us, 1)
        self.us[0] = u - u_ff
        self.u_last = u
        return u


class PIDRuntime:
    """Float64 mirror of FB_PressurePID (the SCL block is checked against the float32 copy)."""

    def __init__(self, Kp: float, Ti: float, Td: float, u_lo: float, u_hi: float, du_max: float,
                 Nf: float = 10.0, b: float = 1.0, Tt: float | None = None):
        self.Kp, self.Ti, self.Td, self.Nf, self.b = Kp, Ti, Td, Nf, b
        self.Tt = Tt if Tt is not None else max(math.sqrt(max(Ti * Td, 0.0)), 0.5 * Ti)
        self.u_lo, self.u_hi, self.du_max = u_lo, u_hi, du_max
        self.I = 0.0
        self.D = 0.0
        self.y1 = 0.0
        self.u1 = 0.0

    def init(self, r: float, y: float, u: float) -> None:
        self.I = u - self.Kp * (self.b * r - y)
        self.D = 0.0
        self.y1 = y
        self.u1 = u

    def step(self, r: float, y: float, u_ff: float = 0.0) -> float:
        ad = self.Td / (self.Td + self.Nf * TS) if self.Td > 0 else 0.0
        kd = self.Kp * self.Td * self.Nf / (self.Td + self.Nf * TS) if self.Td > 0 else 0.0
        self.D = ad * self.D - kd * (y - self.y1)
        self.I += self.Kp * TS / self.Ti * (r - y)
        v = self.Kp * (self.b * r - y) + self.I + self.D + u_ff
        u = min(max(v, self.u1 - self.du_max), self.u1 + self.du_max)
        u = min(max(u, self.u_lo), self.u_hi)
        self.I += TS / self.Tt * (u - v)  # back-calculation
        self.y1 = y
        self.u1 = u
        return u


class MPCRuntime:
    """Small linear MPC (DMC form) on the identified OE model.

    - Prediction: model free response plus the step-response matrix G (Np x Nc).
    - Offset-free action: a filtered output-disturbance estimate d = y - y_model, which
      is the classic DMC assumption (Cutler & Ramaker 1980; Camacho & Bordons, *Model
      Predictive Control*, 2nd ed., Springer 2007, Ch. 2; sections not re-checked).
    - Cost: sum (r - y)^2 + lam sum du^2.
    - Constraints: |du| <= du_max and u_lo <= u <= u_hi.
    - Solver: projected gradient with a fixed iteration count. The cost is deterministic
      for the PLC budget, but the cumulative u-box projection is approximate.
    """

    def __init__(self, model: Model, u_lo: float, u_hi: float, du_max: float, Np: int = 60,
                 Nc: int = 3, lam: float = 5.0, alpha: float = 0.9, iters: int = 20):
        self.m = model
        self.u_lo, self.u_hi, self.du_max = u_lo, u_hi, du_max
        self.Np, self.Nc, self.lam, self.alpha, self.iters = Np, Nc, lam, alpha, iters
        from scipy.signal import lfilter
        step = lfilter(model.b, model.a, np.ones(Np + 1))[1:]
        G = np.zeros((Np, Nc))
        for j in range(Nc):
            G[j:, j] = step[:Np - j]
        self.G = G
        self.H = G.T @ G + lam * np.eye(Nc)
        self.Hinv_GT = np.linalg.solve(self.H, G.T)
        self.step_size = 1.0 / np.linalg.eigvalsh(self.H).max()
        na, nb = len(model.a), len(model.b)
        self.yh = np.zeros(na)  # model output history (deviation)
        self.uh = np.zeros(nb)  # model input history (deviation)
        self.u0 = 0.0
        self.y0 = 0.0
        self.d = 0.0
        self.u_last = 0.0

    def init(self, r: float, y: float, u: float) -> None:
        self.u0, self.y0 = u, y
        self.yh[:] = 0.0
        self.uh[:] = 0.0
        self.d = 0.0
        self.u_last = u

    def _model_step(self, yh: np.ndarray, uh: np.ndarray, du_dev: float) -> tuple[np.ndarray, np.ndarray, float]:
        a, b = self.m.a, self.m.b
        uh = np.roll(uh, 1)
        uh[0] = du_dev
        y = float(np.dot(b, uh) - np.dot(a[1:], yh[:len(a) - 1]))
        yh = np.roll(yh, 1)
        yh[0] = y
        return yh, uh, y

    def step(self, r: float, y: float, u_ff: float = 0.0) -> float:
        # disturbance estimate from the current model output
        ym = self.yh[0]
        self.d = self.alpha * self.d + (1 - self.alpha) * ((y - self.y0) - ym)
        # free response with u held at its last value
        yh, uh = self.yh.copy(), self.uh.copy()
        ud = self.u_last - self.u0
        free = np.empty(self.Np)
        for j in range(self.Np):
            yh, uh, yj = self._model_step(yh, uh, ud)
            free[j] = yj
        e = (r - self.y0) - free - self.d
        du = self.Hinv_GT @ e  # unconstrained optimum
        # projected gradient on the box constraints
        g_lin = self.G.T @ e
        for _ in range(self.iters):
            du = np.clip(du, -self.du_max, self.du_max)
            u_path = self.u_last + np.cumsum(du)
            u_clip = np.clip(u_path, self.u_lo, self.u_hi)
            du = np.diff(np.r_[self.u_last, u_clip])
            if np.allclose(u_path, u_clip) and np.all(np.abs(du) <= self.du_max + 1e-12):
                break
            du = du - self.step_size * (self.H @ du - g_lin)
        du = np.clip(du, -self.du_max, self.du_max)
        u = min(max(self.u_last + du[0] + u_ff, self.u_lo), self.u_hi)
        u = min(max(u, self.u_last - self.du_max), self.u_last + self.du_max)
        self.yh, self.uh, _ = self._model_step(self.yh, self.uh, u - self.u0)
        self.u_last = u
        return u
