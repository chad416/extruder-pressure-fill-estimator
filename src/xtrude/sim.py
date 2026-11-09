"""Closed-loop production scenarios: disturbances, controller, estimator, feedforward.

The scenario generator draws the disturbances seen on a cold-feed line [assumed
magnitudes, dossier 8.4]:
- feed-efficiency steps from strip changes (+-8 %, every 60–240 s);
- compound batch changes (viscosity factor 0.8–1.25, every 200–400 s), which the line's
  production records announce;
- line-speed (gear-pump speed) changes of +-10 %;
- one barrel-temperature step of +-5 K;
- optionally, run-out at the end: the feed stops while the screw keeps turning.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .estimator import EkfConfig, FillEKF, static_mass_balance
from .ident import TS
from .plant import Extruder, nominal


@dataclass
class Event:
    t: float
    kind: str  # "eps" | "kappa" | "gp" | "Tb" | "feed_off" | "sp"
    value: float = 0.0


@dataclass
class Scenario:
    duration: float
    events: list[Event] = field(default_factory=list)
    seed: int = 0


def random_scenario(seed: int, duration: float = 1200.0, runout: bool = False,
                    kappa_range: tuple[float, float] = (0.8, 1.25)) -> Scenario:
    r = np.random.default_rng(seed)
    ev: list[Event] = []
    t, eps = 30.0, 1.0
    while t < duration:
        eps = float(np.clip(eps + r.uniform(-0.08, 0.08), 0.85, 1.05))
        ev.append(Event(t, "eps", eps))
        t += r.uniform(60, 240)
    t = r.uniform(100, 300)
    while t < duration:
        ev.append(Event(t, "kappa", float(r.uniform(*kappa_range))))
        t += r.uniform(200, 400)
    for _ in range(2):
        ev.append(Event(float(r.uniform(100, duration - 100)), "gp", float(r.uniform(0.9, 1.1))))
    ev.append(Event(float(r.uniform(200, duration - 200)), "Tb", float(80 + r.choice([-5, 5]))))
    if runout:
        ev.append(Event(duration - 240.0, "feed_off"))
    ev.sort(key=lambda e: e.t)
    return Scenario(duration, ev, seed)


@dataclass
class Log:
    cols: dict[str, list] = field(default_factory=dict)

    def add(self, **kw) -> None:
        for k, v in kw.items():
            self.cols.setdefault(k, []).append(float(v))

    def arr(self, k: str) -> np.ndarray:
        return np.asarray(self.cols[k])


def run(scn: Scenario, controller, *, P_sp: float = 60.0, plant: Extruder | None = None,
        ekf_cfg: EkfConfig | None = None, use_ekf: bool = False, feedforward: bool = False,
        log_every: int = 10, plant_seed: int | None = None) -> Log:
    """Simulate a scenario. `controller` has init(r, y, u) and step(r, y, u_ff) -> u (rpm)."""
    e = plant or nominal(seed=plant_seed if plant_seed is not None else scn.seed)
    meas = e.measure()
    controller.init(P_sp, meas["P_bar"], e.N * 60)
    ekf = FillEKF(e, ekf_cfg) if (use_ekf or feedforward) else None
    n = int(round(scn.duration / TS))
    ev = list(scn.events)
    gp_rel0 = e.N_gp_rel
    every_ekf = int(round((ekf.cfg.ts if ekf else 0.5) / TS))
    log = Log()
    u = e.N * 60
    ff_ref = None
    batch_flag = False
    m_mb = float("nan")
    for k in range(n):
        t = k * TS
        while ev and ev[0].t <= t:
            x = ev.pop(0)
            if x.kind == "eps":
                e.eps = x.value
            elif x.kind == "kappa":
                e.kappa = x.value
                batch_flag = True
            elif x.kind == "gp":
                e.N_gp_rel = gp_rel0 * x.value
            elif x.kind == "Tb":
                e.T_barrel = x.value
            elif x.kind == "feed_off":
                e.feed_on = False
            elif x.kind == "sp":
                P_sp = x.value
        meas = e.measure()
        u_ff = 0.0
        if ekf is not None and k % every_ekf == 0:
            ekf.step(meas, e.N_gp_rel, e.feed_on, batch_flag)
            batch_flag = False
        if feedforward and ekf is not None and t > 60.0 and e.feed_on:
            n_ff = ekf.speed_for_balance(P_sp, e.N_gp_rel)
            if ff_ref is None:
                ff_ref = n_ff
                controller.ff_last = 0.0
            # deviation from the balance speed at engagement; the integrator holds the DC part
            u_ff = n_ff - ff_ref
        u = controller.step(P_sp, meas["P_bar"], u_ff)
        e.step(u, TS)
        if k % log_every == 0:
            tr = e.truth()
            if ekf is not None:
                m_hat, m_sig = ekf.mass(), ekf.mass_sigma()
            else:
                m_hat, m_sig = float("nan"), float("nan")
            m_mb = static_mass_balance(e, meas["N_rpm"], meas["P_bar"], e.N_gp_rel)
            q_ref = e.q_gp_design * e.N_gp_rel
            log.add(t=t, P=tr["P_bar"], P_meas=meas["P_bar"], u=u, N=tr["N_rpm"], T=tr["T"],
                    M=tr["M"], M_hat=m_hat, M_sig=m_sig, M_mb=m_mb, torque=meas["torque"],
                    power=meas["power"], Q_out=tr["Q_out"], Q_rel=tr["Q_out"] / q_ref,
                    kappa_tip=tr["kappa_tip"], eps=e.eps, feed=float(e.feed_on),
                    gp=e.N_gp_rel / gp_rel0, P_sp=P_sp, T_meas=meas["T"], u_ff=u_ff)
    return log
