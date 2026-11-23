"""Transducer zero-drift study (FMEA row F3). Writes results/drift.md, results/drift.csv
and docs/img/drift_*.png.

    python scripts/drift_study.py            # 12 paired scenarios per case
    python scripts/drift_study.py --quick    # 3 scenarios (CI smoke)

Three questions:
A. What does a zero offset of the head-pressure transducer do to the true pressure, the
   throughput (gauge) and the margin to gear-pump starvation?
B. Does the fill estimator notice it? The FMEA of revision B named the EKF pressure
   innovation as the condition-monitoring flag. That claim is tested here.
C. Can the offset be read at run-out, when the screw is empty and the head has drained?

The controller is the robust PID of results/study.md (0.108, 0.472, 0.169). Offsets are
applied as a constant from t = 0: a drift over days is static within a 20-minute run. One
extra case ramps 8 bar in 15 minutes, far faster than a real drift, to give the innovation
its best chance.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from xtrude.control import PIDRuntime  # noqa: E402
from xtrude.ident import TS  # noqa: E402
from xtrude.plant import operating_point  # noqa: E402
from xtrude.sim import Event, random_scenario, run  # noqa: E402
from xtrude.zero_check import RunoutZeroCheck  # noqa: E402

RES = ROOT / "results"
IMG = ROOT / "docs" / "img"
OFFSETS = (-8.0, -4.0, -2.0, 0.0, 2.0, 4.0, 8.0)
SETPOINTS = (60.0, 40.0)
P_STARVE = 25.0  # bar; below this the gear pump starves (dossier §7.2)
P_FILL = 15.0  # bar; assumed gear-pump fill constant (TBD-4)
T0 = 180.0  # s; skip the transient in which the loop moves to the shifted pressure

C_SP = {60.0: "#2a78d6", 40.0: "#eb6834"}
INK, MUTED = "#333333", "#8a8a8a"
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": INK,
                     "ytick.color": INK, "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.6,
                     "lines.linewidth": 1.6, "legend.frameon": False})


def pid() -> PIDRuntime:
    return PIDRuntime(0.108, 0.472, 0.169, 2, 45, 0.5)


def one_run(args) -> dict:
    """One closed-loop production run with a constant transducer offset."""
    p_sp, off, i, dur = args
    scn = random_scenario(i, dur)
    scn.events.insert(0, Event(0.0, "pz", off))
    plant = operating_point(P_bar=p_sp, seed=100 + i)
    log = run(scn, pid(), P_sp=p_sp, plant=plant, use_ekf=True)
    t = log.arr("t")
    s = t > T0
    P, M, Mh = log.arr("P")[s], log.arr("M")[s], log.arr("M_hat")[s]
    return {
        "P_sp": p_sp, "offset": off, "seed": i,
        "P_true_mean": float(P.mean()), "P_true_min": float(P.min()),
        "starve_frac": float(np.mean(P < P_STARVE)),
        "Q_mean": float(log.arr("Q_out")[s].mean()),
        "M_err_mean": float(np.mean(Mh - M)), "M_err_rmse": float(np.sqrt(np.mean((Mh - M) ** 2))),
        "nu_p_mean": float(log.arr("nu_p")[s].mean()), "nu_tq_mean": float(log.arr("nu_tq")[s].mean()),
    }


def ramp_run(args) -> dict:
    """Drift ramp of 8 bar in 15 minutes (32 bar/h) from t = 300 s, at 60 bar."""
    i, dur = args
    scn = random_scenario(i, dur)
    scn.events.append(Event(300.0, "pz_rate", 32.0))
    scn.events.sort(key=lambda e: e.t)
    log = run(scn, pid(), P_sp=60.0, plant=operating_point(P_bar=60.0, seed=100 + i), use_ekf=True)
    t, nu = log.arr("t"), log.arr("nu_p")
    before, after = (t > T0) & (t < 300), t > dur - 300
    return {"seed": i, "nu_before": float(nu[before].mean()), "nu_after": float(nu[after].mean()),
            "offset_end": float(log.arr("P_zero")[-1]),
            "M_err_before": float(np.mean((log.arr("M_hat") - log.arr("M"))[before])),
            "M_err_after": float(np.mean((log.arr("M_hat") - log.arr("M"))[after]))}


def runout_run(args) -> dict:
    """Production, then run-out with the zero check running at the PLC cycle (100 ms)."""
    off, i, dur = args
    plant = operating_point(P_bar=60.0, seed=100 + i)
    plant.p_zero_bar = off
    scn = random_scenario(i, dur, runout=True)
    ctl = pid()
    meas = plant.measure()
    ctl.init(60.0, meas["P_bar"], plant.N * 60)
    chk = RunoutZeroCheck()
    ev = list(scn.events)
    gp0 = plant.N_gp_rel
    false_capture = False
    t_off = t_got = None
    for k in range(int(round(dur / TS))):
        t = k * TS
        while ev and ev[0].t <= t:
            x = ev.pop(0)
            if x.kind == "eps":
                plant.eps = x.value
            elif x.kind == "kappa":
                plant.kappa = x.value
            elif x.kind == "gp":
                plant.N_gp_rel = gp0 * x.value
            elif x.kind == "Tb":
                plant.T_barrel = x.value
            elif x.kind == "feed_off":
                plant.feed_on = False
                t_off = t
        meas = plant.measure()
        got = chk.step(plant.feed_on, meas["torque"], meas["P_bar"], TS)
        if got is not None:
            t_got = t - t_off
            if plant.P / 1e5 > 0.5:
                false_capture = True  # captured while the head still held pressure
        plant.step(ctl.step(60.0, meas["P_bar"]), TS)
    return {"offset": off, "seed": i, "zero": chk.zero_bar, "t_capture": t_got,
            "false_capture": false_capture, "P_true_end": plant.P / 1e5}


def pmap(fn, items):
    with ProcessPoolExecutor() as ex:
        return list(ex.map(fn, items))


def md_table(header, rows) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def sens(p_bar: float) -> float:
    """Analytic throughput sensitivity of the assumed fill curve, % per bar."""
    e = math.exp(-p_bar / P_FILL)
    return 100.0 * e / (P_FILL * (1.0 - e))


def main() -> None:  # noqa: C901
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    n, dur = (3, 600.0) if a.quick else (12, 1200.0)

    rows = pmap(one_run, [(sp, off, i, dur) for sp in SETPOINTS for off in OFFSETS for i in range(n)])
    ramps = pmap(ramp_run, [(i, max(dur, 1200.0)) for i in range(n)])
    ro_offsets = (-4.0, 0.0, 4.0, 8.0)
    runouts = pmap(runout_run, [(off, i, dur) for off in ro_offsets for i in range(n)])

    def sel(sp, off):
        return [r for r in rows if r["P_sp"] == sp and r["offset"] == off]

    # ---- A. consequences
    tab_a, fig_q = [], {}
    for sp in SETPOINTS:
        base = {r["seed"]: r for r in sel(sp, 0.0)}
        for off in OFFSETS:
            rs = sel(sp, off)
            dq = [100.0 * (r["Q_mean"] / base[r["seed"]]["Q_mean"] - 1.0) for r in rs]
            fig_q.setdefault(sp, []).append((off, float(np.mean(dq)), float(np.min(dq)), float(np.max(dq))))
            tab_a.append([f"{sp:.0f}", f"{off:+.0f}", f"{np.mean([r['P_true_mean'] for r in rs]):.1f}",
                          f"{np.mean(dq):+.2f}", f"{min(r['P_true_min'] for r in rs):.1f}",
                          f"{100 * np.mean([r['starve_frac'] for r in rs]):.2f}",
                          f"{sum(r['starve_frac'] > 0 for r in rs)} of {len(rs)}"])

    # ---- B. what the estimator sees
    tab_b = []
    for sp in SETPOINTS:
        base = {r["seed"]: r for r in sel(sp, 0.0)}
        null = [abs(r["nu_p_mean"]) for r in sel(sp, 0.0)]
        thr = max(null)  # the largest run-mean seen with no offset: zero false alarms on these runs
        for off in OFFSETS:
            rs = sel(sp, off)
            dm = [r["M_err_mean"] - base[r["seed"]]["M_err_mean"] for r in rs]
            flagged = sum(abs(r["nu_p_mean"]) > thr for r in rs) if off != 0.0 else 0
            tab_b.append([f"{sp:.0f}", f"{off:+.0f}", f"{np.mean(dm):+.3f}",
                          f"{np.mean([r['M_err_rmse'] for r in rs]):.3f}",
                          f"{np.mean([r['nu_p_mean'] for r in rs]):+.3f} ± {np.std([r['nu_p_mean'] for r in rs]):.3f}",
                          f"{np.mean([r['nu_tq_mean'] for r in rs]):+.3f}",
                          "—" if off == 0.0 else f"{flagged} of {len(rs)}"])
    ramp_line = (f"{np.mean([r['nu_before'] for r in ramps]):+.3f} before the ramp, "
                 f"{np.mean([r['nu_after'] for r in ramps]):+.3f} in its last 5 minutes "
                 f"(offset {np.mean([r['offset_end'] for r in ramps]):.1f} bar at the end, {len(ramps)} runs)")

    # ---- C. zero check at run-out
    tab_c = []
    for off in ro_offsets:
        rs = [r for r in runouts if r["offset"] == off]
        got = [r for r in rs if r["zero"] is not None]
        err = [r["zero"] - off for r in got]
        tab_c.append([f"{off:+.0f}", f"{len(got)} of {len(rs)}",
                      f"{np.mean(err):+.2f}" if err else "—", f"{np.max(np.abs(err)):.2f}" if err else "—",
                      f"{np.mean([r['t_capture'] for r in got]):.0f}, {np.max([r['t_capture'] for r in got]):.0f}" if got else "—",
                      f"{sum(r['false_capture'] for r in rs)}"])

    # ---- figures
    fig, ax = plt.subplots(1, 2, figsize=(9.0, 3.3))
    for sp in SETPOINTS:
        o = [x[0] for x in fig_q[sp]]
        m = [x[1] for x in fig_q[sp]]
        lo = [x[1] - x[2] for x in fig_q[sp]]
        hi = [x[3] - x[1] for x in fig_q[sp]]
        ax[0].errorbar(o, m, yerr=[lo, hi], color=C_SP[sp], marker="o", ms=4, capsize=2,
                       label=f"setpoint {sp:.0f} bar")
    ax[0].axhline(0, color=MUTED, lw=0.8)
    ax[0].set_xlabel("transducer zero offset, bar (reading minus true pressure)")
    ax[0].set_ylabel("throughput change, %")
    ax[0].set_title("A. The loop holds the reading; the output moves", loc="left", fontsize=9)
    ax[0].legend()
    for sp in SETPOINTS:
        o = np.array([r["offset"] for r in rows if r["P_sp"] == sp])
        v = np.array([r["nu_p_mean"] for r in rows if r["P_sp"] == sp])
        ax[1].scatter(o + (0.25 if sp == 60.0 else -0.25), v, s=10, color=C_SP[sp], alpha=0.8,
                      label=f"setpoint {sp:.0f} bar")
    ax[1].axhline(0, color=MUTED, lw=0.8)
    ax[1].set_ylim(-1.0, 1.0)
    ax[1].set_xlabel("transducer zero offset, bar")
    ax[1].set_ylabel("run-mean pressure innovation, in σ")
    ax[1].set_title("B. The estimator's pressure innovation does not move", loc="left", fontsize=9)
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(IMG / "drift_consequence_detection.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.6, 3.3))
    for off in ro_offsets:
        got = [r["zero"] for r in runouts if r["offset"] == off and r["zero"] is not None]
        ax.scatter([off] * len(got), got, s=12, color="#2a78d6", alpha=0.8)
    lim = [min(ro_offsets) - 1, max(ro_offsets) + 1]
    ax.plot(lim, lim, color=MUTED, lw=0.8)
    ax.set_xlabel("true zero offset, bar")
    ax.set_ylabel("offset read at run-out, bar")
    ax.set_title("C. Zero check at run-out (simulated head drains to 0)", loc="left", fontsize=9)
    fig.tight_layout()
    fig.savefig(IMG / "drift_zero_check.png", dpi=150)
    plt.close(fig)

    # ---- outputs
    with open(RES / "drift.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    md = f"""# Transducer zero drift (FMEA row F3)

Written by `scripts/drift_study.py`{' --quick' if a.quick else ''}: {n} paired production scenarios of {dur:.0f} s per
case, robust PID (0.108, 0.472, 0.169), fill EKF running. Simulation only. Offset = reading
minus true pressure, constant during a run. Statistics are taken after t = {T0:.0f} s.

## A. Consequence in closed loop

The controller holds the *reading* at the setpoint, so the true pressure sits at setpoint
minus offset. With the assumed gear-pump fill curve (TBD-4) the throughput follows at
{sens(60):.2f} % per bar at 60 bar and {sens(40):.2f} % per bar at 40 bar (analytic slope of the curve).
Starvation = true pressure below {P_STARVE:.0f} bar.

{md_table(["Setpoint, bar", "Offset, bar", "Mean true pressure, bar", "Throughput change, %",
           "Lowest true pressure, bar", "Time below 25 bar, %", "Runs that touched 25 bar"], tab_a)}

## B. What the fill estimator sees

Mass-error shift = change of the mean estimate error against the same scenario with no
offset. The innovation is normalised by its own 1-σ from the filter. "Flagged" counts runs
whose run-mean pressure innovation exceeds the largest value seen in the no-offset runs of
the same setpoint, a threshold with zero false alarms on those runs. If an offset changed
nothing at all, about one run in {n + 1} would still exceed it by chance.

{md_table(["Setpoint, bar", "Offset, bar", "Mass-error shift, kg", "Mass RMSE, kg",
           "Pressure innovation, run mean ± sd (σ)", "Torque innovation, run mean (σ)", "Flagged"], tab_b)}

Drift ramp of 32 bar/h at 60 bar (8 bar in 15 minutes): mean pressure innovation
{ramp_line}.

## C. Zero check at run-out

`xtrude.zero_check.RunoutZeroCheck` on {n} run-outs per offset at 60 bar. It arms when the
feed is off and the torque is below 4 kN m, and latches the slow-filtered reading once a
2 s and a 10 s filter agree within 0.3 bar for 20 s. A capture counts as false when the true
head pressure was still above 0.5 bar at that moment.

{md_table(["True offset, bar", "Captured", "Mean error, bar", "Largest error, bar",
           "Captured after feed stop, s (mean, latest)", "False captures"], tab_c)}

This part rests on the simulated head draining to 0 bar at run-out. Whether a real head
does is open (dossier §12, Q5). A 4–20 mA transmitter also cannot report far below zero, so
a negative offset larger than its under-range would be clipped.
"""
    (RES / "drift.md").write_text(md, encoding="utf-8")
    print(f"wrote {RES / 'drift.md'}")


if __name__ == "__main__":
    main()
