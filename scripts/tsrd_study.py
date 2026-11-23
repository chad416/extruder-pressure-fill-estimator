"""Study of the batch-fed twin-screw roller-die unit (src/xtrude/tsrd.py).

Writes results/tsrd.md, results/tsrd_startup.csv and docs/img/tsrd_*.png.

    python scripts/tsrd_study.py            # 8 seeds per case
    python scripts/tsrd_study.py --quick    # 2 seeds (CI smoke)

A. What the plant looks like to a controller: gain and lag by recipe, and how well the gain
   can be read from an ordinary logged start-up (no test signal).
B. Start-up: PI switched on at a pressure threshold, against the same PI helped by a recipe
   table (steady speed, revolutions to full, relative gain).
C. Production when the screws starve at the end of each batch: PI alone against PI with
   feedforward from the material model and a hold while starved.
D. Topic 3: how much compound is left. Mass balance, conveying register and torque.
"""

from __future__ import annotations

import argparse
import os
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from xtrude.tsrd import RPM, Tsrd, TsrdParams  # noqa: E402
from xtrude.tsrd_sim import (KP_ROBUST, TI_ROBUST, StartupConfig, arx_gain, batch_plan,  # noqa: E402
                             learn_memory, run, startup_metrics, u_roll_for)

RES = ROOT / "results"
IMG = ROOT / "docs" / "img"
KAPPAS = (0.7, 1.0, 1.4)
P_SP = 20.0
SECOND_AFTER, JITTER = 115.0, 20.0  # section C: a thin hopper reserve and a restless mixer
RUNOUT = 260.0  # s simulated after the last drop, long enough for the unit to run empty
MODEL_ERR = {"eps_max": 0.95, "m_c": 1.2, "V_rev": 1.03, "q_cal": 1.02}

C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED = "#333333", "#8a8a8a"
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": INK,
                     "ytick.color": INK, "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.6,
                     "lines.linewidth": 1.5, "legend.frameon": False})

STARTUP_CASES = {
    "PI": {},
    "PI + gain": {"use_gain": True},
    "PI + preload": {"use_preload": True},
    "PI + preload + rev count": {"use_preload": True, "use_rev_count": True},
    "PI + all three": {"use_preload": True, "use_rev_count": True, "use_gain": True},
}


def md_table(header, rows) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def pmap(fn, items):
    # at most 8 workers: 16 of them ran a laptop with 2.5 GB free out of memory
    with ProcessPoolExecutor(max_workers=min(8, os.cpu_count() or 1)) as ex:
        return list(ex.map(fn, items))


# ------------------------------------------------------------------ A
def open_loop_step(kappa: float) -> tuple[float, float]:
    e = Tsrd(seed=0)
    e.steady(m_h=1e6, kappa=kappa)
    n0 = e.N / RPM
    for _ in range(600):
        e.step(n0, 0.1)
    p0 = e.P / 1e5
    y = []
    for _ in range(600):
        e.step(n0 + 0.3, 0.1)
        y.append(e.P / 1e5)
    y = np.array(y)
    d = y[-1] - p0
    return d / 0.3, 0.1 * float(np.argmax(y - p0 > 0.632 * d))


def learn(kappa: float):
    """One earlier run of the recipe, logged at 1 s, fills the recipe table."""
    p = TsrdParams()
    log = run(batch_plan(900 + int(10 * kappa), 3, p, kappa=kappa), StartupConfig(), duration=200,
              seed=900 + int(10 * kappa), u_roll=u_roll_for(p, kappa, P_SP))
    i_full = int(np.argmax(log.arr("P_meas") > 3.0))
    t_full = float(log.arr("t")[i_full])
    return learn_memory(log), arx_gain(log, t_full, t_full + 40)


# ------------------------------------------------------------------ B
def startup_one(args) -> dict:
    kappa, case, seed, mem = args
    p = TsrdParams()
    cfg = StartupConfig(memory=mem, **STARTUP_CASES[case])
    log = run(batch_plan(seed, 3, p, kappa=kappa), cfg, duration=200, seed=seed, u_roll=u_roll_for(p, kappa, P_SP))
    m = startup_metrics(log, P_SP)
    return {"kappa": kappa, "case": case, "seed": seed, **m}


# ------------------------------------------------------------------ C and D
def production_one(args) -> dict:
    kappa, ff, err, seed, mem = args
    p = TsrdParams()
    cfg = StartupConfig(memory=mem, use_preload=True, use_rev_count=True, feedforward=ff,
                        model_err=MODEL_ERR if err else {})
    plan = batch_plan(seed, 8, p, kappa=kappa, second_after=SECOND_AFTER, jitter=JITTER)
    t_last = plan[-1].t
    # the run goes on after the last drop until the unit has run empty (the run-out of topic 3)
    log = run(plan, cfg, duration=t_last + RUNOUT, seed=seed, u_roll=u_roll_for(p, kappa, P_SP))
    t, P = log.arr("t"), log.arr("P")
    s = (t > 100.0) & (t < t_last + 100.0)  # production: the hopper still holds the last batch
    dev = np.abs(P[s] - P_SP) / P_SP
    M, Mh = log.arr("M")[s], log.arr("M_hat")[s]
    ms, msh, mst = log.arr("m_s")[s], log.arr("m_s_hat")[s], log.arr("m_s_tq")[s]
    # time-to-empty prediction, taken whenever the truth says 30 s are left; the run-out at
    # the end gives at least one such moment per run
    r = t > 100.0
    tte_true = (log.arr("m_h")[r] + log.arr("m_s")[r]) / np.maximum(p.rho * log.arr("Q_out")[r], 1e-9)
    tte_hat = log.arr("tte")[r]
    k = np.nonzero((tte_true[:-1] > 30.0) & (tte_true[1:] <= 30.0))[0]
    return {"kappa": kappa, "ff": ff, "err": err, "seed": seed,
            "P_peak": float(P[s].max()), "P_min": float(P[s].min()), "out_pct": float(100 * np.mean(dev > 0.10)),
            "N_peak": float(log.arr("N")[s].max()),
            "M_rmse": float(np.sqrt(np.mean((Mh - M) ** 2))), "M_end": float(Mh[-1] - M[-1]),
            "ms_rmse": float(np.sqrt(np.mean((msh - ms) ** 2))), "ms_tq_rmse": float(np.sqrt(np.mean((mst - ms) ** 2))),
            "tte_err": [float(x) for x in (tte_hat[k] - 30.0)]}


def main() -> None:  # noqa: C901
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    n = 2 if a.quick else 8
    p = TsrdParams()

    # ---- A
    learned = {k: learn(k) for k in KAPPAS}
    mems = {k: v[0] for k, v in learned.items()}
    g_ref = learned[1.0][1]
    tab_a = []
    for k in KAPPAS:
        g, t63 = open_loop_step(k)
        mems[k].gain_rel = learned[k][1] / g_ref
        tab_a.append([f"{k:.1f}", f"{g:.2f}", f"{t63:.1f}", f"{learned[k][1]:.2f}", f"{g / open_loop_step(1.0)[0]:.2f}",
                      f"{mems[k].gain_rel:.2f}", f"{mems[k].n_ss:.2f}", f"{mems[k].rev_full:.1f}"])

    # ---- B
    rows_b = pmap(startup_one, [(k, c, 10 + i, mems[k]) for k in KAPPAS for c in STARTUP_CASES for i in range(n)])
    tab_b = []
    for k in KAPPAS:
        for c in STARTUP_CASES:
            rs = [r for r in rows_b if r["kappa"] == k and r["case"] == c]
            tab_b.append([f"{k:.1f}", c, f"{np.mean([r['Ts'] for r in rs]):.1f}", f"{np.max([r['Ts'] for r in rs]):.0f}",
                          f"{np.mean([r['ISE'] for r in rs]):.2f}", f"{np.mean([r['P_peak'] for r in rs]):.1f}",
                          f"{sum(r['settled'] for r in rs)} of {len(rs)}"])

    # ---- C, D
    jobs = [(k, ff, err, 30 + i, mems[k]) for k in KAPPAS for ff, err in ((False, False), (True, False), (True, True))
            for i in range(n)]
    rows_c = pmap(production_one, jobs)
    tab_c, tab_d = [], []
    names = {(False, False): "PI", (True, False): "PI + feedforward and hold",
             (True, True): "PI + feedforward and hold, model with errors"}
    for k in KAPPAS:
        for key, name in names.items():
            rs = [r for r in rows_c if r["kappa"] == k and (r["ff"], r["err"]) == key]
            tab_c.append([f"{k:.1f}", name, f"{np.mean([r['P_peak'] for r in rs]):.1f}", f"{np.max([r['P_peak'] for r in rs]):.1f}",
                          f"{np.mean([r['P_min'] for r in rs]):.1f}", f"{np.mean([r['out_pct'] for r in rs]):.1f}",
                          f"{np.mean([r['N_peak'] for r in rs]):.1f}"])
    for err, name in ((False, "exact parameters"), (True, "parameters with errors")):
        rs = [r for r in rows_c if r["ff"] and r["err"] == err]
        tte = np.array([x for r in rs for x in r["tte_err"]])
        tab_d.append([name, f"{np.mean([r['M_rmse'] for r in rs]):.1f}", f"{np.mean([r['M_end'] for r in rs]):+.1f}",
                      f"{np.mean([r['ms_rmse'] for r in rs]):.1f}", f"{np.mean([r['ms_tq_rmse'] for r in rs]):.1f}",
                      f"{np.mean(tte):+.1f} ± {np.std(tte):.1f} ({len(tte)})" if len(tte) else "—"])

    # ---- figures
    k = 1.0
    la = run(batch_plan(10, 3, p, kappa=k), StartupConfig(memory=mems[k]), duration=140, seed=10)
    lb = run(batch_plan(10, 3, p, kappa=k), StartupConfig(memory=mems[k], use_preload=True), duration=140, seed=10)
    lb2 = run(batch_plan(10, 3, p, kappa=k), StartupConfig(memory=mems[k], use_preload=True, use_rev_count=True),
              duration=140, seed=10)
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.4), sharex=True)
    for lg, c, name in ((la, C1, "PI, bumpless from the filling speed"), (lb, C2, "PI + preload"),
                        (lb2, C3, "PI + preload + revolution count")):
        ax[0].plot(lg.arr("t"), lg.arr("P"), color=c, label=name)
        ax[1].plot(lg.arr("t"), lg.arr("N"), color=c)
    ax[0].axhline(P_SP, color=MUTED, lw=0.8)
    ax[0].axhspan(0.9 * P_SP, 1.1 * P_SP, color=MUTED, alpha=0.12, lw=0)
    ax[0].set_ylabel("chamber pressure, bar")
    ax[1].set_ylabel("screw speed, rpm")
    ax[1].set_xlabel("time from the first batch drop, s")
    ax[0].set_xlim(40, 140)
    ax[0].legend()
    ax[0].set_title("Start-up: filling at fixed speed, then pressure control", loc="left", fontsize=9)
    fig.tight_layout()
    fig.savefig(IMG / "tsrd_startup.png", dpi=150)
    plt.close(fig)

    dur = 5.2 * p.batch_period
    cfg0 = StartupConfig(memory=mems[k], use_preload=True, use_rev_count=True)
    cfg1 = StartupConfig(memory=mems[k], use_preload=True, use_rev_count=True, feedforward=True, model_err=MODEL_ERR)
    lc = run(batch_plan(30, 6, p, kappa=k, second_after=SECOND_AFTER, jitter=JITTER), cfg0, duration=dur, seed=30)
    ld = run(batch_plan(30, 6, p, kappa=k, second_after=SECOND_AFTER, jitter=JITTER), cfg1, duration=dur, seed=30)
    fig, ax = plt.subplots(3, 1, figsize=(7.2, 6.0), sharex=True)
    for lg, c, name in ((lc, C1, "PI"), (ld, C2, "PI + feedforward and hold (model with errors)")):
        ax[0].plot(lg.arr("t"), lg.arr("P"), color=c, label=name)
        ax[1].plot(lg.arr("t"), lg.arr("N"), color=c)
    ax[0].axhline(P_SP, color=MUTED, lw=0.8)
    ax[0].set_ylabel("chamber pressure, bar")
    ax[0].legend(loc="upper left")
    ax[1].set_ylabel("screw speed, rpm")
    ax[2].plot(ld.arr("t"), ld.arr("m_h") + ld.arr("m_s"), color=INK, label="truth")
    ax[2].plot(ld.arr("t"), ld.arr("m_h_hat") + ld.arr("m_s_hat"), color=C2, ls="--", label="material model")
    ax[2].set_ylabel("hopper + screws, kg")
    ax[2].set_xlabel("time from the first batch drop, s")
    ax[2].legend(loc="upper right")
    ax[0].set_title("Production with a thin hopper reserve: the screws run short before some drops", loc="left",
                    fontsize=9)
    fig.tight_layout()
    fig.savefig(IMG / "tsrd_production.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 2.8))
    s = ld.arr("t") > 100
    ax.plot(ld.arr("t")[s], ld.arr("m_s")[s], color=INK, label="truth")
    ax.plot(ld.arr("t")[s], ld.arr("m_s_hat")[s], color=C2, ls="--", label="conveying register (events + speed)")
    ax.plot(ld.arr("t")[s], ld.arr("m_s_tq")[s], color=C3, lw=0.9, label="from torque alone")
    ax.set_ylabel("compound in the screws, kg")
    ax.set_xlabel("time from the first batch drop, s")
    ax.legend(ncol=3, loc="lower center", fontsize=8)
    fig.tight_layout()
    fig.savefig(IMG / "tsrd_screw_content.png", dpi=150)
    plt.close(fig)

    with open(RES / "tsrd_startup.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows_b[0]))
        w.writeheader()
        w.writerows(rows_b)

    md = f"""# Batch-fed twin-screw roller-die unit: start-up, starvation and remaining material

Written by `scripts/tsrd_study.py`{' --quick' if a.quick else ''}: {n} seeds per case. Simulation only, on the
structural model `src/xtrude/tsrd.py`. Every parameter of that model is assumed. Setpoint
{P_SP:.0f} bar, robust PI Kp = {KP_ROBUST} rpm/bar, Ti = {TI_ROBUST} s, controller sample 0.1 s, logs at 1 s.

## A. The plant as a controller sees it

Open-loop step of +0.3 rpm at the running point with a full hopper. "Gain from a logged
start-up" is a first-order ARX fit on 40 s of the 1 s closed-loop log after the hand-over,
with no test signal. The recipe table (last three columns) is filled from that one run.

{md_table(["Viscosity factor", "Gain, bar/rpm", "63 % time, s", "Gain from a logged start-up, bar/rpm",
           "True gain relative to factor 1.0", "Learned relative gain", "Learned steady speed, rpm",
           "Learned revolutions to full"], tab_a)}

## B. Start-up

Filling at {StartupConfig().N_fill:.0f} rpm, hand-over to the PI when the reading passes {StartupConfig().P_thr:.0f} bar. A
second batch follows after 60 s, so the hopper holds a reserve. Metrics as in the source
thesis: Ts = time from the first pressure until the pressure stays within ±10 % of the
setpoint; ISE = integral of the squared relative error over the first 300 s after it.

{md_table(["Viscosity factor", "Controller", "Ts mean, s", "Ts worst, s", "ISE, s", "Peak pressure, bar",
           "Settled"], tab_b)}

## C. Production with a thin hopper reserve

Eight batches. The second follows {SECOND_AFTER:.0f} s after the first, then one per mixer cycle with
±{JITTER:.0f} s jitter, so the hopper holds only about {(p.batch_period - SECOND_AFTER) * p.rho * p.Q_ref:.0f} kg before a punctual drop and runs
short when the mixer is late. Statistics from t = 100 s to 100 s after the last drop. "Outside ±10 %" is a share of time. The
model errors are eps_max ×{MODEL_ERR['eps_max']}, m_c ×{MODEL_ERR['m_c']}, V_rev ×{MODEL_ERR['V_rev']} and throughput calibration ×{MODEL_ERR['q_cal']}.

{md_table(["Viscosity factor", "Controller", "Peak pressure mean, bar", "Peak pressure worst, bar",
           "Lowest pressure, bar", "Outside ±10 %, %", "Peak speed, rpm"], tab_c)}

## D. Remaining material (topic 3)

From the same production runs. "Total" is hopper + screws + chamber from the mass balance
of batch weights and roller-die throughput. "Screws" compares the conveying register of
the material model with an inversion of the torque balance that assumes the nominal
recipe viscosity. Each run continues after the last drop until the unit is empty. The
time-to-empty error is taken each time the true value passes 30 s.

{md_table(["Material model", "Total: RMSE, kg", "Total: error at the end, kg", "Screws, register: RMSE, kg",
           "Screws, torque alone: RMSE, kg", "Time to empty at 30 s: error, s (events)"], tab_d)}
"""
    (RES / "tsrd.md").write_text(md, encoding="utf-8", newline="\n")
    print(f"wrote {RES / 'tsrd.md'}")


if __name__ == "__main__":
    main()
