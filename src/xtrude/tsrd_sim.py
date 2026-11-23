"""Start-up and production scenarios for the batch-fed twin-screw roller-die unit (tsrd.py).

Sequence of a start-up, as the source thesis describes it [T1]: the unit is filled at a
fixed screw speed until the pressure shows that it is full, and only then is the PI
controller switched on. This module adds three things a PLC can hold in a recipe table,
all learned from earlier runs of the same recipe, none needing a model in the controller:

- the screw speed at which the recipe ran in steady state (preload at the hand-over);
- the number of screw revolutions after the first batch drop at which the unit was full
  (the unit is volumetric, so this is a count, not a time);
- the gain of the recipe, used to scale the controller gain.

It also holds the material model used for topic 3 and for feedforward: a batch-and-
throughput mass balance with a short conveying register for the screws.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .control import PIDRuntime
from .ident import TS
from .tsrd import BAR, RPM, Tsrd, TsrdParams

# Robust PI for the unit (rpm per bar, s): SIMC rule on the highest-gain variant, see
# scripts/tsrd_study.py section A. Fixed for every recipe unless gain scheduling is on.
KP_ROBUST = 0.14
TI_ROBUST = 3.5


@dataclass
class Batch:
    t: float
    m: float
    kappa: float


def batch_plan(seed: int, n: int, p: TsrdParams, kappa: float = 1.0, jitter: float = 12.0,
               kappa_sd: float = 0.04, m_sd: float = 0.02, second_after: float = 60.0) -> list[Batch]:
    """Batch drops from the mixer [assumed magnitudes].

    The first batch falls at t = 0 into the empty unit. About 90 kg of it stays in the screws
    and the chamber. `second_after` is the time to the second drop and sets the reserve the
    hopper carries from then on: 60 s gives a comfortable reserve, the full mixer cycle
    (about 150 s) gives none, and the screws then starve before every drop.
    """
    r = np.random.default_rng(seed)
    out, t = [], 0.0
    for i in range(n):
        out.append(Batch(t, p.m_batch * (1 + r.normal(0, m_sd)), kappa * (1 + r.normal(0, kappa_sd))))
        t += second_after if i == 0 else p.batch_period + r.uniform(-jitter, jitter)
    return out


def u_roll_for(p: TsrdParams, kappa: float, P_sp: float) -> float:
    """Roller speed at which the unit takes the average rate of the mixer at the setpoint.

    Solves (1 - s) u + s u^(1-n) P_sp / (P_nom kappa) = 1. A line is run this way: the
    take-off speed is matched to the mixer, and the pressure setpoint fixes the rest.
    """
    x = P_sp / (p.P_nom_bar * kappa)
    lo, hi = 0.2, 2.0
    for _ in range(60):
        u = 0.5 * (lo + hi)
        if (1 - p.s_p) * u + p.s_p * u ** (1 - p.n) * x > 1.0:
            hi = u
        else:
            lo = u
    return 0.5 * (lo + hi)


@dataclass
class RecipeMemory:
    """What the PLC keeps per recipe from earlier runs."""

    n_ss: float | None = None  # steady screw speed at the setpoint, rpm
    rev_full: float | None = None  # screw revolutions from the first drop to "full"
    gain_rel: float = 1.0  # plant gain of this recipe relative to the tuning recipe


@dataclass
class StartupConfig:
    P_sp: float = 20.0
    N_fill: float = 16.0  # rpm during filling
    P_thr: float = 3.0  # bar; reading above this hands over to the PI
    Kp: float = KP_ROBUST
    Ti: float = TI_ROBUST
    memory: RecipeMemory | None = None
    use_preload: bool = False  # start the PI from the remembered steady speed
    use_rev_count: bool = False  # ramp to the remembered speed just before the unit is full
    use_gain: bool = False  # divide Kp by the remembered relative gain
    rev_lead: float = 0.9  # revolutions before `rev_full` at which the ramp-down starts
    feedforward: bool = False  # feedforward from the material model, with a hold while starved
    starve_fill: float = 0.6  # tip fill, relative to a full hopper, below which the PI is held
    ff_max: float = 1.6  # largest feedforward speed relative to the remembered speed
    model_err: dict = field(default_factory=dict)  # parameter errors of the material model


class MaterialModel:
    """Remaining material in hopper, screws and chamber, from signals the PLC has.

    Inputs: batch-drop events with the batch weight from the mixer scale, screw speed, and
    the throughput leaving the rollers (from roller speed and sheet weight per metre, or a
    belt scale). The screws are a short register of fills moved on by the revolutions.
    Its parameters carry errors (`err`) to keep the estimate honest:
    eps_max, m_c (intake curve), V_rev, and the throughput calibration `q_cal`.
    """

    def __init__(self, p: TsrdParams, err: dict | None = None, cells: int = 10):
        e = err or {}
        self.p = p
        self.eps_max = p.eps_max * e.get("eps_max", 1.0)
        self.m_c = p.m_c * e.get("m_c", 1.0)
        self.V_rev = p.V_rev * e.get("V_rev", 1.0)
        self.q_cal = e.get("q_cal", 1.0)
        self.cells = cells
        self.dv = p.V_screw / cells
        self.f = np.zeros(cells)
        self.m_in = 0.0
        self.m_out = 0.0
        self.v_ch = 0.0
        self.revs = 0.0  # screw revolutions since the first batch drop

    def drop(self, m: float) -> None:
        self.m_in += m

    def eps(self, m_h: float) -> float:
        return self.eps_max * (1.0 - math.exp(-max(m_h, 0.0) / self.m_c))

    def m_screws(self) -> float:
        return self.p.rho * self.dv * float(np.sum(self.f))

    def m_total(self) -> float:
        return max(self.m_in - self.m_out, 0.0)

    def m_hopper(self) -> float:
        return max(self.m_total() - self.m_screws() - self.p.rho * self.v_ch, 0.0)

    def step(self, N_rpm: float, q_out_meas: float, dt: float, chamber_full: bool | None = None) -> None:
        """`chamber_full` is the pressure signal above its threshold: the chamber then holds
        its full volume, whatever the integrated balance of the chamber says."""
        p = self.p
        N = N_rpm * RPM
        if self.m_in > 0:
            self.revs += N * dt
        self.m_out += p.rho * q_out_meas * self.q_cal * dt
        v_turn = self.V_rev * N * dt
        m_h = self.m_hopper()
        v_in = min(v_turn * self.eps(m_h), m_h / p.rho)
        cfl = min(v_turn / self.dv, 1.0)
        f_old = self.f.copy()
        v_del = cfl * f_old[-1] * self.dv
        self.f[1:] = f_old[1:] - cfl * (f_old[1:] - f_old[:-1])
        self.f[0] = f_old[0] - cfl * f_old[0] + v_in / self.dv
        self.v_ch = min(max(self.v_ch + v_del - q_out_meas * self.q_cal * dt, 0.0), p.V_ch)
        if chamber_full:
            self.v_ch = p.V_ch

    def f_tip(self) -> float:
        return float(self.f[-1])

    def time_to_empty(self, q_out_meas: float) -> float:
        """Seconds until hopper and screws are empty at the present throughput."""
        return (self.m_hopper() + self.m_screws()) / max(self.p.rho * q_out_meas * self.q_cal, 1e-9)


def mass_from_torque(p: TsrdParams, torque: float, N_rpm: float, P_bar: float, q_del: float,
                     kappa_hat: float = 1.0, friction_err: float = 1.0) -> float:
    """Screw content from the drive torque alone (the literal reading of topic 3).

    Inverts the torque balance of tsrd.py with an assumed recipe viscosity factor.
    """
    N = max(N_rpm * RPM, 1e-3)
    visc = torque - p.torque_friction * friction_err - P_bar * BAR * q_del / (2 * math.pi * N)
    return max(visc, 0.0) / (p.c_visc * kappa_hat * (N / (p.N_nom_rpm * RPM)) ** p.n)


@dataclass
class Log:
    cols: dict[str, list] = field(default_factory=dict)

    def add(self, **kw) -> None:
        for k, v in kw.items():
            self.cols.setdefault(k, []).append(float(v))

    def arr(self, k: str) -> np.ndarray:
        return np.asarray(self.cols[k])


def run(batches: list[Batch], cfg: StartupConfig, *, duration: float, seed: int = 0,
        p: TsrdParams | None = None, log_every: int = 10, u_roll: float = 1.0,
        p_zero_bar: float = 0.0) -> Log:
    """Start-up from an empty unit, then production. Logged once per second by default."""
    p = p or TsrdParams()
    e = Tsrd(p, seed=seed)
    e.u_roll = u_roll
    e.p_zero_bar = p_zero_bar
    mem = cfg.memory or RecipeMemory()
    kp = cfg.Kp / mem.gain_rel if cfg.use_gain else cfg.Kp
    pid = PIDRuntime(kp, cfg.Ti, 0.0, p.N_min_rpm, p.N_max_rpm, p.ramp_rpm_s * TS)
    mm = MaterialModel(p, cfg.model_err)
    todo = list(batches)
    phase = 0  # 0 filling at fixed speed, 1 ramp to the remembered speed, 2 PI
    u = p.N_min_rpm
    t_handover = float("nan")
    ff_ref = None
    log = Log()
    for k in range(int(round(duration / TS))):
        t = k * TS
        while todo and todo[0].t <= t:
            b = todo.pop(0)
            e.drop_batch(b.m, b.kappa)
            mm.drop(b.m)
        meas = e.measure()
        mm.step(meas["N_rpm"], e._q_out, TS, chamber_full=meas["P_bar"] > cfg.P_thr)
        if phase == 0:
            u = cfg.N_fill if e.m_in > 0 else p.N_min_rpm
            if (cfg.use_rev_count and mem.rev_full is not None and mem.n_ss is not None
                    and mm.revs >= mem.rev_full - cfg.rev_lead):
                phase = 1
            elif meas["P_bar"] > cfg.P_thr:
                phase = 2
                t_handover = t
                if cfg.use_preload and mem.n_ss:
                    # the integrator starts at the remembered steady speed; the P term adds
                    # its share on top while the pressure is still below the setpoint
                    pid.init(cfg.P_sp, cfg.P_sp, mem.n_ss)
                    pid.u1 = u
                else:
                    pid.init(cfg.P_sp, meas["P_bar"], u)  # bumpless from the filling speed
        elif phase == 1:
            u = mem.n_ss  # the drive ramp limits how fast this is reached
            if meas["P_bar"] > cfg.P_thr:
                phase = 2
                t_handover = t
                pid.init(cfg.P_sp, cfg.P_sp, mem.n_ss)
                pid.u1 = u
        if phase == 2:
            u_ff, starved = 0.0, False
            if cfg.feedforward and t > t_handover + 30.0:
                # speed that keeps the delivery constant as the fill at the tips changes
                rel = mm.f_tip() / mm.eps_max
                n_ref = mem.n_ss or u
                n_ff = n_ref * min(1.0 / max(rel, 1e-3), cfg.ff_max)
                if ff_ref is None:
                    ff_ref = n_ff
                u_ff = n_ff - ff_ref
                starved = rel < cfg.starve_fill
            # While the screws are starved the pressure cannot be held by speed. The PI is fed
            # a zero error, so its integrator does not wind up against missing material.
            u = pid.step(cfg.P_sp, cfg.P_sp if starved else meas["P_bar"], u_ff)
        e.step(u, TS)
        if k % log_every == 0:
            tr = e.truth()
            log.add(t=t, P=tr["P_bar"], P_meas=meas["P_bar"], u=u, N=tr["N_rpm"], phase=phase,
                    m_h=tr["m_h"], m_s=tr["m_s"], M=tr["M"], torque=meas["torque"], Q_out=tr["Q_out"],
                    full=tr["full"], f_tip=tr["f_tip"], revs=mm.revs,
                    m_s_hat=mm.m_screws(), m_h_hat=mm.m_hopper(), M_hat=mm.m_total(),
                    m_s_tq=mass_from_torque(p, meas["torque"], meas["N_rpm"], meas["P_bar"], e._q_del),
                    tte=mm.time_to_empty(e._q_out), handover=t_handover)
    return log


def startup_metrics(log: Log, P_sp: float = 20.0, band: float = 0.10) -> dict:
    """Metrics of the thesis [T1]: settling into +-10 % of the setpoint and the control area.

    Time runs from the first reading above zero. ISE is on the normalised error, in s.
    """
    t, P = log.arr("t"), log.arr("P")
    i0 = int(np.argmax(P > 0.05)) if np.any(P > 0.05) else len(P) - 1
    err = (P[i0:] - P_sp) / P_sp
    out = np.abs(err) > band
    last = int(np.max(np.nonzero(out)[0])) + 1 if np.any(out) else 0
    dt = float(t[1] - t[0])
    return {"t_first": float(t[i0]), "Ts": float(last * dt), "ISE": float(np.sum(err[:int(300 / dt)] ** 2) * dt),
            "P_peak": float(P.max()), "settled": bool(last < len(err) - int(30 / dt))}


def arx_gain(log: Log, t0: float, t1: float) -> float:
    """Static gain (bar per rpm) from logged closed-loop data between t0 and t1.

    First-order ARX by least squares on the 1 s log, the method of the source thesis [T1]:
    y(k) = a y(k-1) + b u(k-1) + c, gain = b / (1 - a). No test signal is needed. The
    hand-over transient of a start-up moves the speed by several rpm.
    """
    t, y, u = log.arr("t"), log.arr("P_meas"), log.arr("N")
    s = np.nonzero((t >= t0) & (t <= t1))[0]
    A = np.column_stack([y[s[:-1]], u[s[:-1]], np.ones(len(s) - 1)])
    a, b, _ = np.linalg.lstsq(A, y[s[1:]], rcond=None)[0]
    return float(b / (1.0 - a))


def learn_memory(log: Log, P_thr: float = 3.0, gain_ref: float | None = None) -> RecipeMemory:
    """Fill the recipe table from one logged run (1 s samples are enough)."""
    t, P, N, revs = log.arr("t"), log.arr("P_meas"), log.arr("N"), log.arr("revs")
    i_full = int(np.argmax(P > P_thr))
    settled = (t > t[i_full] + 40) & (t < t[i_full] + 80)
    mem = RecipeMemory(n_ss=float(np.mean(N[settled])), rev_full=float(revs[i_full]))
    if gain_ref is not None:
        mem.gain_rel = arx_gain(log, t[i_full], t[i_full] + 40) / gain_ref
    return mem
