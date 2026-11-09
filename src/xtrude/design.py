"""Robust tuning of the PID baseline and the RST controller on one uncertainty set.

Both controllers are tuned the same way, which keeps the comparison fair:
- Objective: minimise the worst-case IAE of the pressure after a 1 rpm-equivalent input
  disturbance (a feed loss), across all plant variants.
- Constraints, which must hold on every variant:
  - closed-loop stable
  - Ms <= MS_MAX
  - speed ripple caused by the measured flight-passing pressure ripple <= RIPPLE_MAX
  - RMS speed noise caused by the pressure noise <= NOISE_MAX
The PID has no plant model. The RST is designed on the model identified from the noisy
technician test, then checked on the noise-free "true" variants.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import differential_evolution
from scipy.signal import lfilter

from .control import RST, design_rst, load_response, margins, padd, peval, pid_as_rst, pmul
from .ident import TS, Model, true_model
from .plant import Params, operating_point

MS_MAX = 1.6  # modulus margin >= 0.625 [LZ recommends >= 0.5; tighter here for the plant spread]
RIPPLE_MAX = 0.10  # rpm amplitude of speed ripple from 0.6 bar flight ripple
NOISE_MAX = 0.10  # rpm RMS of speed from 0.3 bar white pressure noise
P_RIPPLE = 0.6
P_NOISE = 0.3


@dataclass
class Variant:
    name: str
    model: Model
    w_ripple: float  # rad/s of the flight-passing ripple at this speed


def variants(cache: dict | None = None) -> list[Variant]:
    base = Params()
    spec = [
        ("nominal", {}, {}),
        ("kappa 0.7", {}, {"kappa": 0.7}),
        ("kappa 1.3", {}, {"kappa": 1.3}),
        ("P 40 bar", {}, {"P_bar": 40.0}),
        ("P 80 bar", {}, {"P_bar": 80.0}),
        ("N 20 rpm", {}, {"N_rpm": 20.0}),
        ("N 40 rpm", {}, {"N_rpm": 40.0}),
        ("B 0.25 GPa", {"B_eff": 0.25e9}, {}),
        ("B 1.0 GPa", {"B_eff": 1.0e9}, {}),
        ("drive tau 0.6 s", {"tau_drive": 0.6}, {}),
    ]
    out = []
    for name, pk, ok in spec:
        if cache is not None and name in cache:
            m = cache[name]
        else:
            p = replace(base, **pk)
            m = true_model(lambda p=p, ok=ok: operating_point(p, **ok))
            if cache is not None:
                cache[name] = m
        n_rpm = ok.get("N_rpm", 30.0)
        out.append(Variant(name, m, 2 * math.pi * n_rpm / 60.0))
    return out


def notch(w: float, r: float = 0.85) -> tuple[np.ndarray, np.ndarray]:
    """Second-order notch at w rad/s, unit DC gain: (num, den) in q^-1.

    It follows the screw speed: w = 2 pi N/60 (one flight). FB_RippleNotch recomputes
    the coefficients when the speed changes.
    """
    c = math.cos(w * TS)
    num = np.array([1.0, -2 * c, 1.0])
    den = np.array([1.0, -2 * r * c, r * r])
    g = np.sum(den) / np.sum(num)
    return g * num, den


def with_filter(m: Model, w: float | None) -> Model:
    if w is None:
        return m
    nb, da = notch(w)
    return Model(pmul(m.a, da), pmul(m.b, nb))


def noise_rms(c: RST, m: Model, sigma: float) -> float:
    """RMS of u for white measurement noise sigma: u = -R A / (A S + B R) n."""
    Pc = padd(pmul(m.a, c.S), pmul(m.b, c.R))
    h = lfilter(pmul(c.R, m.a), Pc, np.r_[1.0, np.zeros(999)])
    return float(sigma * np.sqrt(np.sum(h**2)))


def ripple_amp(c: RST, m: Model, w: float, amp: float) -> float:
    z = np.exp(1j * w * TS)
    Pc = padd(pmul(m.a, c.S), pmul(m.b, c.R))
    return float(amp * abs(peval(pmul(c.R, m.a), z) / peval(Pc, z)))


@dataclass
class Score:
    iae: float
    peak: float
    ms: float
    ripple: float
    noise: float
    feasible: bool


def score(c: RST, vs: list[Variant], use_notch: bool = False) -> Score:
    """Worst case over the variants. With use_notch the measurement passes a notch that
    tracks each variant's own screw speed. The controller sees plant * notch, while the
    load response is still measured on the true (unfiltered) pressure."""
    iae, peak, ms, rip, noi, ok = 0.0, 0.0, 0.0, 0.0, 0.0, True
    for v in vs:
        mf = with_filter(v.model, v.w_ripple if use_notch else None)
        mg = margins(c, mf)
        if not mg.stable:
            return Score(math.inf, math.inf, math.inf, math.inf, math.inf, False)
        if use_notch:
            nb, da = notch(v.w_ripple)
            # y_true = B S d / (A S Fa... ) -> closed loop with filter in the feedback path
            Pc = padd(pmul(v.model.a, c.S, da), pmul(v.model.b, c.R, nb))
            ld = lfilter(pmul(v.model.b, c.S, da), Pc, np.ones(600))
        else:
            ld = load_response(c, v.model)
        iae = max(iae, float(np.sum(np.abs(ld)) * TS))
        peak = max(peak, float(np.max(np.abs(ld))))
        ms = max(ms, mg.Ms)
        rip = max(rip, ripple_amp(c, mf, v.w_ripple, P_RIPPLE))
        noi = max(noi, noise_rms(c, mf, P_NOISE))
    ok = ms <= MS_MAX and rip <= RIPPLE_MAX and noi <= NOISE_MAX
    return Score(iae, peak, ms, rip, noi, ok)


def _penalised(s: Score) -> float:
    if not math.isfinite(s.iae):
        return 1e6
    pen = 0.0
    pen += 200 * max(0.0, s.ms - MS_MAX)
    pen += 2000 * max(0.0, s.ripple - RIPPLE_MAX)
    pen += 2000 * max(0.0, s.noise - NOISE_MAX)
    return s.iae + pen


def tune_pid(vs: list[Variant], derivative: bool = True, seed: int = 0, use_notch: bool = False):
    def make(x):
        Kp, Ti, Td = 10 ** x[0], 10 ** x[1], (x[2] if derivative else 0.0)
        return pid_as_rst(Kp, Ti, Td)

    bounds = [(-2.5, 0.5), (-0.7, 1.7), (0.0, 2.0)] if derivative else [(-2.5, 0.5), (-0.7, 1.7)]

    def f(x):
        return _penalised(score(make(x if derivative else np.r_[x, 0.0]), vs, use_notch))

    r = differential_evolution(f, bounds, seed=seed, maxiter=60, popsize=12, tol=1e-6, polish=True)
    x = r.x if derivative else np.r_[r.x, 0.0]
    c = make(x)
    return c, (10 ** x[0], 10 ** x[1], x[2]), score(c, vs, use_notch)


def tune_rst(design_model: Model, vs: list[Variant], seed: int = 0, use_notch: bool = False,
             w_design: float = math.pi):
    dm = with_filter(design_model, w_design if use_notch else None)

    def make(x):
        return design_rst(dm, w0=x[0], zeta=x[1], aux=x[2], wm=x[3])

    bounds = [(0.1, 2.0), (0.6, 1.0), (0.0, 0.9), (0.1, 2.0)]

    def f(x):
        try:
            return _penalised(score(make(x), vs, use_notch))
        except (np.linalg.LinAlgError, ValueError):
            return 1e6

    r = differential_evolution(f, bounds, seed=seed, maxiter=60, popsize=12, tol=1e-6, polish=True)
    c = make(r.x)
    return c, tuple(r.x), score(c, vs, use_notch)
