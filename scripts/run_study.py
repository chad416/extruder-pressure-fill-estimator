"""Run the full study and write results/*.md, results/*.csv and docs/img/*.png.

The stages are cached in results/cache/ (git-ignored), so an interrupted run resumes
where it stopped. Delete the cache to start from scratch.

    python scripts/run_study.py            # all stages
    python scripts/run_study.py --quick    # fewer Monte Carlo runs (CI smoke)
"""

from __future__ import annotations

import argparse
import math
import pickle
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from xtrude import scl  # noqa: E402
from xtrude.control import (MPCRuntime, PIDRuntime, RSTRuntime, design_rst, margins,  # noqa: E402
                            pid_as_rst)
from xtrude.design import MS_MAX, NOISE_MAX, RIPPLE_MAX, score, tune_pid, tune_rst, variants  # noqa: E402
from xtrude.estimator import EkfConfig  # noqa: E402
from xtrude.ident import TS, fit_oe, step_test, validate  # noqa: E402
from xtrude.learned import MLP, Ridge, features  # noqa: E402
from xtrude.plant import RPM, Params, nominal, operating_point  # noqa: E402
from xtrude.sim import Event, Scenario, random_scenario, run  # noqa: E402

RES = ROOT / "results"
IMG = ROOT / "docs" / "img"
CACHE = RES / "cache"
for d in (RES, IMG, CACHE):
    d.mkdir(parents=True, exist_ok=True)

# validated categorical palette (dataviz skill, slots 1-5, fixed order)
C = {"PID": "#2a78d6", "RST": "#eb6834", "MPC": "#1baf7a", "PID+FF": "#eda100", "RST+FF": "#e87ba4"}
INK = "#333333"
MUTED = "#8a8a8a"
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": INK,
                     "ytick.color": INK, "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.6,
                     "lines.linewidth": 1.6, "legend.frameon": False})


def cached(name: str, fn):
    p = CACHE / f"{name}.pkl"
    if p.exists():
        return pickle.loads(p.read_bytes())
    t = time.time()
    v = fn()
    p.write_bytes(pickle.dumps(v))
    print(f"  [{name}] {time.time() - t:.0f} s", flush=True)
    return v


def md_table(header: list[str], rows: list[list]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(x) for x in r) + " |")
    return "\n".join(out)


def f(x, n=2):
    return f"{x:.{n}f}"


# ============================================================ 1. operating map
def stage_model() -> str:
    rows = []
    for n in (15, 20, 25, 30, 35, 40, 45):
        e = operating_point(N_rpm=n, P_bar=60.0)
        tr = e.truth()
        q = tr["Q_s"] * e.p.compound.rho * 3600
        rows.append([n, f(q, 0), f(tr["torque"] / 1e3, 1), f(100 * tr["torque"] / e.p.torque_rated, 0),
                     f(tr["power"] / 1e3, 0), f(tr["power"] / 1e3 / q, 3), f(tr["T"], 1), f(tr["Zf"], 3),
                     f(tr["M"], 2)])
    outlet_figure()
    return md_table(["N rpm", "output kg/h", "torque kNm", "torque % rated", "power kW", "SEC kWh/kg",
                     "melt T degC", "Zf m", "M kg"], rows)


def outlet_figure() -> None:
    """Open-loop speed steps: power-law die (+3 rpm) vs gear pump (+0.3 rpm and +1 rpm)."""
    fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.0))
    cases = (("die", 33.0, C["PID"], "30 -> 33 rpm"), ("gearpump", 30.3, C["RST"], "30 -> 30.3 rpm"),
             ("gearpump", 31.0, C["MPC"], "30 -> 31 rpm"))
    for outlet, n1, col, lab in cases:
        e = nominal(outlet)
        t, p = [], []
        for i in range(int(120 / TS)):
            e.step(n1 if i * TS >= 10 else 30.0, TS)
            tr = e.truth()
            t.append(tr["t"])
            p.append(tr["P_bar"])
        k = 0 if outlet == "die" else 1
        ax[k].plot(t, p, color=col, label=f"speed {lab} at 10 s")
    ax[0].set_title("Power-law die outlet", fontsize=9, loc="left")
    ax[1].set_title("Gear-pump outlet (primary case)", fontsize=9, loc="left")
    ax[1].axhline(150, color=MUTED, ls="--", lw=1)
    ax[1].text(2, 153, "overpressure trip 150 bar (SPS-323)", fontsize=7, color=INK)
    ax[1].set_ylim(50, 175)
    for a in ax:
        a.set_xlabel("time (s)")
        a.set_ylabel("head pressure (bar)")
        a.legend(fontsize=7, loc="lower right")
    fig.tight_layout()
    fig.savefig(IMG / "outlet_step_response.png", dpi=160)
    plt.close(fig)


# ============================================================ 2. identification
def stage_ident():
    amp_rows = []
    for amp in (0.1, 0.3, 1.0):
        r0 = step_test(nominal(seed=1), amp_rpm=amp, seed=1, noise=False)
        m0 = fit_oe(r0)
        amp_rows.append([amp, f(r0.y.min(), 1) + " - " + f(r0.y.max(), 1), f(m0.fit_pct, 1)])
    r1 = step_test(nominal(seed=1), seed=1)
    r2 = step_test(nominal(seed=2), seed=2)
    r1.to_csv(str(RES / "technician_test_1.csv"))
    r2.to_csv(str(RES / "technician_test_2.csv"))
    m = fit_oe(r1)
    v_rmse, v_fit = validate(m, r2)
    fig, ax = plt.subplots(2, 1, figsize=(8.6, 4.2), sharex=True)
    ax[0].plot(r2.t, r2.y, color=MUTED, lw=0.8, label="measured (validation record)")
    ax[0].plot(r2.t, r2.y0 + m.simulate(r2.u - r2.u0), color=C["PID"], label="identified model, free run")
    ax[0].set_ylabel("pressure (bar)")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[1].plot(r2.t, r2.u, color=C["RST"])
    ax[1].set_ylabel("speed setpoint (rpm)")
    ax[1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(IMG / "identification.png", dpi=160)
    plt.close(fig)
    return {"model": m, "amp_rows": amp_rows, "fit": m.fit_pct, "v_rmse": v_rmse, "v_fit": v_fit}


# ============================================================ 3. controller design
def stage_design(mid):
    cache: dict = {}
    vs = variants(cache)
    out = {"variants": cache}
    out["pi"] = tune_pid(vs, derivative=False)
    out["pid"] = tune_pid(vs, derivative=True)
    out["rst"] = tune_rst(mid, vs)
    out["pid_notch"] = tune_pid(vs, derivative=True, use_notch=True)
    out["rst_notch"] = tune_rst(mid, vs, use_notch=True)
    # sensitivity: ripple limit relaxed
    import xtrude.design as D
    old = D.RIPPLE_MAX
    D.RIPPLE_MAX = 0.2
    out["pid_r02"] = tune_pid(vs, derivative=True)
    out["rst_r02"] = tune_rst(mid, vs)
    D.RIPPLE_MAX = old
    return out


def step_metrics(c, vs, amp=4.0, band=0.5):
    """Worst case over the variants: settling time into +-band and overshoot for an amp-bar step."""
    from xtrude.control import setpoint_response
    worst_t, worst_ov = 0.0, 0.0
    for v in vs:
        y, _ = setpoint_response(c, v.model, n=1200)
        y = amp * y
        outside = np.where(np.abs(y - amp) > band)[0]
        t_s = (outside[-1] + 1) * TS if len(outside) else 0.0
        worst_t = max(worst_t, t_s)
        worst_ov = max(worst_ov, float(y.max() - amp))
    return worst_t, worst_ov


def design_report(des, vs):
    rows = []
    for key, label in (("pi", "PI"), ("pid", "PID"), ("rst", "RST"), ("pid_notch", "PID + speed-tracking notch"),
                       ("rst_notch", "RST + speed-tracking notch"), ("pid_r02", "PID, ripple limit 0.2 rpm"),
                       ("rst_r02", "RST, ripple limit 0.2 rpm")):
        c, par, s = des[key]
        st, ov = step_metrics(c, vs)
        rows.append([label, f(s.iae, 2), f(s.peak, 2), f(s.ms, 2), f(s.ripple, 3), f(s.noise, 3), f(st, 1), f(ov, 2),
                     "yes" if s.feasible else "no", ", ".join(f"{p:.3g}" for p in par)])
    # per-variant margins for PID and RST
    mrows = []
    for v in vs:
        mp = margins(des["pid"][0], v.model)
        mr = margins(des["rst"][0], v.model)
        mrows.append([v.name, f(v.model.dc_gain, 1), f(mp.Ms, 2), f(mp.gm, 1), f(mp.pm_deg, 0), f(mp.dm_s, 2),
                      f(mr.Ms, 2), f(mr.gm, 1), f(mr.pm_deg, 0), f(mr.dm_s, 2)])
    # figure: load responses on all variants
    from xtrude.control import load_response
    fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.0), sharey=True)
    for k, key in enumerate(("pid", "rst")):
        for v in vs:
            y = load_response(des[key][0], v.model)
            ax[k].plot(np.arange(len(y)) * TS, y, color=C["PID" if key == "pid" else "RST"], lw=1.0, alpha=0.75)
        ax[k].set_title(("PID" if key == "pid" else "RST") + ", 10 plant variants", fontsize=9, loc="left")
        ax[k].set_xlabel("time after 1 rpm feed-equivalent disturbance (s)")
    ax[0].set_ylabel("pressure deviation (bar)")
    fig.tight_layout()
    fig.savefig(IMG / "load_response_variants.png", dpi=160)
    plt.close(fig)
    return rows, mrows


# ============================================================ 4. nonlinear Monte Carlo
def make_controllers(des, mid):
    pidp = des["pid"][1]
    rst = des["rst"][0]
    return {
        "PID": lambda: PIDRuntime(pidp[0], pidp[1], pidp[2], 2.0, 45.0, 0.5),
        "RST": lambda: RSTRuntime(rst, 2.0, 45.0, 0.5),
        "MPC": lambda: MPCRuntime(mid, 2.0, 45.0, 0.5, lam=5.0, alpha=0.8),
        "PID+FF": lambda: PIDRuntime(pidp[0], pidp[1], pidp[2], 2.0, 45.0, 0.5),
        "RST+FF": lambda: RSTRuntime(rst, 2.0, 45.0, 0.5),
    }


def metrics(log, t0=60.0):
    t = log.arr("t")
    s = t >= t0
    on = log.arr("feed")[s] > 0.5
    P = log.arr("P")[s][on]
    dev = P - log.arr("P_sp")[s][on]
    q = log.arr("Q_rel")[s][on]
    u = log.arr("u")[s][on]
    return {"P_std": float(dev.std()), "P_p99": float(np.percentile(np.abs(dev), 99)),
            "out3": float(np.mean(np.abs(dev) > 3.0) * 100), "gauge_std": float(100 * q.std() / q.mean()),
            "u_travel": float(np.sum(np.abs(np.diff(u))) / (len(u) / 60.0))}


WORKERS = 3  # parallel scenario runs (each process needs ~0.2 GB)


def _mc_one(args):
    des, mid, i, dur = args
    ctrls = make_controllers(des, mid)
    scn = random_scenario(100 + i, dur, runout=(i % 2 == 0))
    out, logs = {}, {}
    for k, mk in ctrls.items():
        log = run(scn, mk(), feedforward=k.endswith("FF"), use_ekf=k.endswith("FF"), plant_seed=100 + i)
        out[k] = metrics(log)
        if k == "RST+FF" or i == 0:
            logs[k] = log
    return i, out, logs


def _pmap(fn, items):
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=WORKERS) as ex:
        res = []
        for r in ex.map(fn, items):
            res.append(r)
            print(f"    done {len(res)}/{len(items)}", flush=True)
        return res


def stage_mc(des, mid, n_runs, dur):
    res = {k: [None] * n_runs for k in ("PID", "RST", "MPC", "PID+FF", "RST+FF")}
    logs = {}
    for i, out, lg in _pmap(_mc_one, [(des, mid, i, dur) for i in range(n_runs)]):
        for k, v in out.items():
            res[k][i] = v
        logs[i] = lg["RST+FF"]
        if i == 0:
            for k, v in lg.items():
                logs[f"{k}_0"] = v
    return res, logs


def _nob_one(args):
    des, i, dur = args
    scn = random_scenario(100 + i, dur, runout=(i % 2 == 0))
    return run(scn, RSTRuntime(des["rst"][0], 2.0, 45.0, 0.5), use_ekf=True,
               ekf_cfg=EkfConfig(use_batch_event=False), plant_seed=100 + i)


def stage_ekf_nobatch(n_runs, dur, des):
    return dict(enumerate(_pmap(_nob_one, [(des, i, dur) for i in range(n_runs)])))


def _train_one(args):
    des, i, dur, kappa_range, seed0, use_ekf = args
    scn = random_scenario(seed0 + i, dur, runout=(i % 2 == 0), kappa_range=kappa_range)
    return run(scn, RSTRuntime(des["rst"][0], 2.0, 45.0, 0.5), use_ekf=use_ekf, plant_seed=seed0 + i)


def stage_train_logs(n_train, dur, des, kappa_range=(0.8, 1.25), seed0=500, use_ekf=False):
    return _pmap(_train_one, [(des, i, dur, kappa_range, seed0, use_ekf) for i in range(n_train)])


def run_errors(log, key):
    t = log.arr("t")
    s = t >= 60
    M = log.arr("M")[s]
    Mh = log.arr(key)[s]
    return float(np.sqrt(np.mean((Mh - M) ** 2))), float(np.max(np.abs(Mh - M)))


def runout_error(log, key):
    """Error at the end of a run-out (feed off) and when P first drops below 25 bar."""
    feed = log.arr("feed")
    if feed.min() > 0.5:
        return None
    M, Mh, P = log.arr("M"), log.arr(key), log.arr("P")
    i0 = int(np.argmax(feed < 0.5))
    below = np.where((P < 25.0) & (np.arange(len(P)) > i0))[0]
    i1 = int(below[0]) if len(below) else len(P) - 1
    return float(Mh[i1] - M[i1]), float(Mh[-1] - M[-1]), float(M[i1]), float(M[-1])


def ci(vals):
    a = np.asarray(vals)
    return float(a.mean()), float(2 * a.std(ddof=1)) if len(a) > 1 else 0.0


# ============================================================ 5. PLC budget
def stage_budget(des) -> tuple[str, dict]:
    fbs = scl.load(sorted(ROOT.glob("plc/FB_*.scl")))
    c = des["rst"][0]
    counts = {}
    # PID
    s = fbs["FB_PressurePID"]()
    s.Enable = True
    s.Pv = np.float32(60.0)
    s.Setpoint = np.float32(60.0)
    scl.reset_ops()
    s()
    counts["FB_PressurePID"] = dict(scl.OPS)
    # RST
    r = fbs["FB_PressureRST"]()
    load_rst(r, c)
    r.Enable = True
    scl.reset_ops()
    r()
    counts["FB_PressureRST"] = dict(scl.OPS)
    # EKF, worst-case call (filter step)
    k = fbs["FB_FillEKF"]()
    k.Nrpm, k.Tmelt, k.Pbar, k.Torque, k.GpRel, k.FeedOn, k.PSp = (
        np.float32(30), np.float32(120), np.float32(60), np.float32(25700), np.float32(1.0), True, np.float32(60))
    k()
    for _ in range(3):
        k()
    scl.reset_ops()
    k()  # 5th call runs the filter step
    counts["FB_FillEKF (filter call)"] = dict(scl.OPS)
    scl.reset_ops()
    k()
    counts["FB_FillEKF (other calls)"] = dict(scl.OPS)
    sv = fbs["FB_PressureSupervisor"]()
    sv.ModeRequest, sv.PChannelOk, sv.TorqueOk, sv.EkfValid = 3, True, True, True
    scl.reset_ops()
    sv()
    counts["FB_PressureSupervisor"] = dict(scl.OPS)
    # datasheet times, CPU 1513-1 PN (6ES7513-1AL02-0AB0): bit 40 ns, word 48 ns, fixed 64 ns, float 256 ns.
    t_ns = {"real": 256, "cmp": 256, "math": 256 * 20, "mov": 48, "int": 64}
    rows = []
    total = 0.0
    for name, ops in counts.items():
        tt = sum(ops[k] * t_ns[k] for k in t_ns) / 1e3  # us
        if "other" not in name:
            total += tt
        rows.append([name, ops["real"], ops["cmp"], ops["math"], ops["int"], ops["mov"], f(tt, 0)])
    return md_table(["block", "Real ops", "Real compares", "EXP/LN/SQRT", "integer/loop", "moves",
                     "estimate us (datasheet)"], rows), {"total_us": total, "counts": counts}


def load_rst(r, c):
    def put(arr, vals):
        for i in range(len(arr)):
            arr[i] = np.float32(vals[i] if i < len(vals) else 0.0)
    put(r.R, c.R)
    put(r.S, c.S)
    put(r.T, c.T)
    put(r.Ao, c.Ao)
    put(r.Bm, c.Bm)
    put(r.Am, c.Am)
    r.nR, r.nS, r.nT, r.nAo = len(c.R) - 1, len(c.S) - 1, len(c.T) - 1, len(c.Ao) - 1


def write_params_scl(des, mid):
    """plc/tia/FC_LoadParams.scl: loads the designed coefficients into the instance DBs.

    The OB30 calls it once, when "LineIO".ParamsLoaded is FALSE (first cycle after start).
    """
    c = des["rst"][0]
    pidp = des["pid"][1]
    lines = [
        "// Generated by scripts/run_study.py. Do not edit; re-run the study instead.",
        "",
        'FUNCTION "FC_LoadParams" : Void',
        "TITLE = Load the designed RST and PID parameters",
        "{ S7_Optimized_Access := 'TRUE' }",
        "VERSION : 0.1",
        "BEGIN",
    ]
    for name, arr in (("R", c.R), ("S", c.S), ("T", c.T), ("Ao", c.Ao), ("Bm", c.Bm), ("Am", c.Am)):
        for i, v in enumerate(arr):
            lines.append(f'	"iRST".{name}[{i}] := {float(v):.9E};')
    lines += [f'	"iRST".nR := {len(c.R) - 1};', f'	"iRST".nS := {len(c.S) - 1};',
              f'	"iRST".nT := {len(c.T) - 1};', f'	"iRST".nAo := {len(c.Ao) - 1};',
              f'	"LineIO".PidKp := {pidp[0]:.6E};', f'	"LineIO".PidTi := {pidp[1]:.6E};',
              f'	"LineIO".PidTd := {pidp[2]:.6E};',
              "	\"LineIO\".ParamsLoaded := TRUE;", "END_FUNCTION", ""]
    (ROOT / "plc" / "tia").mkdir(exist_ok=True)
    (ROOT / "plc" / "tia" / "FC_LoadParams.scl").write_text("\n".join(lines), encoding="utf-8")


# ============================================================ main
def main() -> None:  # noqa: C901
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    n_mc, dur, n_train = (2, 600.0, 3) if a.quick else (24, 1200.0, 24)
    tag = "q" if a.quick else "full"
    t_all = time.time()
    print("1 model", flush=True)
    model_tab = cached("model", stage_model)
    print("2 identification", flush=True)
    idn = cached("ident", stage_ident)
    mid = idn["model"]
    print("3 controller design", flush=True)
    des = cached("design", lambda: stage_design(mid))
    vs = variants(des["variants"])
    drows, mrows = design_report(des, vs)
    write_params_scl(des, mid)
    print("4 closed-loop Monte Carlo", flush=True)
    mc, mclogs = cached(f"mc_{tag}", lambda: stage_mc(des, mid, n_mc, dur))
    nob = cached(f"ekf_nobatch_{tag}", lambda: stage_ekf_nobatch(n_mc, dur, des))
    print("5 learned estimators", flush=True)
    train = cached(f"train_{tag}", lambda: stage_train_logs(n_train, dur, des))
    ood_ekf = cached(f"ood_ekf_{tag}", lambda: stage_train_logs(max(2, n_mc // 2), dur, des,
                                                                 kappa_range=(0.6, 1.5), seed0=900, use_ekf=True))
    print("6 PLC budget", flush=True)
    budget_tab, budget = stage_budget(des)

    # ---------------- closed-loop table and figures
    names = list(mc)
    cl_rows = []
    for k in names:
        vals = mc[k]
        row = [k]
        for key, n in (("P_std", 2), ("P_p99", 2), ("out3", 2), ("gauge_std", 3), ("u_travel", 1)):
            m_, e_ = ci([v[key] for v in vals])
            row.append(f"{m_:.{n}f} +- {e_:.{n}f}")
        cl_rows.append(row)
    fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.0))
    for j, (key, lab) in enumerate((("P_std", "pressure std (bar)"), ("P_p99", "99th pct |deviation| (bar)"))):
        data = [[v[key] for v in mc[k]] for k in names]
        bp = ax[j].boxplot(data, patch_artist=True, widths=0.55, medianprops={"color": INK})
        for patch, k in zip(bp["boxes"], names):
            patch.set_facecolor(C[k])
            patch.set_edgecolor(C[k])
        ax[j].set_xticks(range(1, len(names) + 1), names, fontsize=8)
        ax[j].set_ylabel(lab)
    fig.suptitle(f"Closed loop, {n_mc} random production scenarios x {dur:.0f} s", fontsize=9, x=0.02, ha="left")
    fig.tight_layout()
    fig.savefig(IMG / "closed_loop_mc.png", dpi=160)
    plt.close(fig)
    # timeline for run 0, PID vs RST+FF
    lp, lr = mclogs["PID_0"], mclogs["RST+FF_0"]
    fig, ax = plt.subplots(3, 1, figsize=(8.6, 6.2), sharex=True)
    ax[0].plot(lp.arr("t"), lp.arr("P"), color=C["PID"], lw=1.0, label="PID")
    ax[0].plot(lr.arr("t"), lr.arr("P"), color=C["RST+FF"], lw=1.0, label="RST+FF")
    ax[0].set_ylabel("head pressure (bar)")
    ax[0].set_ylim(40, 75)
    ax[0].legend(loc="lower left", ncol=2, fontsize=8)
    ax[1].plot(lr.arr("t"), lr.arr("eps"), color=INK, lw=1.0, label="feed efficiency (truth)")
    ax[1].plot(lr.arr("t"), lr.arr("kappa_tip"), color=MUTED, lw=1.0, ls="--", label="viscosity factor at tip (truth)")
    ax[1].set_ylabel("disturbances")
    ax[1].legend(loc="lower left", ncol=2, fontsize=8)
    ax[2].plot(lr.arr("t"), lr.arr("M"), color=INK, lw=1.0, label="mass in screw (truth)")
    mh, ms = lr.arr("M_hat"), lr.arr("M_sig")
    ax[2].plot(lr.arr("t"), mh, color=C["RST+FF"], lw=1.0, label="EKF estimate")
    ax[2].fill_between(lr.arr("t"), mh - 2 * ms, mh + 2 * ms, color=C["RST+FF"], alpha=0.2, lw=0, label="EKF +-2 sigma")
    ax[2].set_ylabel("compound in screw (kg)")
    ax[2].set_xlabel("time (s)")
    ax[2].legend(loc="lower left", ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(IMG / "scenario_timeline.png", dpi=160)
    plt.close(fig)

    # ---------------- estimator comparison
    Xtr = {w: [] for w in ("tpr", "full")}
    ytr = []
    for lg in train:
        for w in Xtr:
            X, y = features(lg, w)
            Xtr[w].append(X)
        ytr.append(y)
    ytr = np.concatenate(ytr)
    models = {}
    for w in Xtr:
        X = np.vstack(Xtr[w])
        models[("ridge", w)] = Ridge(1.0).fit(X, ytr)
        models[("mlp", w)] = MLP(seed=0).fit(X, ytr)

    def est_rows(test_logs, nob_logs=None, ekf_key=True):
        rows, per = [], {}
        for i, lg in enumerate(test_logs):
            M = lg.arr("M")[60:]
            per.setdefault("static mass balance (N, gear pump)", []).append(
                float(np.sqrt(np.mean((lg.arr("M_mb")[60:] - M) ** 2))))
            if ekf_key and not np.all(np.isnan(lg.arr("M_hat"))):
                per.setdefault("EKF, torque + pressure, batch events", []).append(run_errors(lg, "M_hat")[0])
            if nob_logs is not None:
                per.setdefault("EKF, no batch events", []).append(run_errors(nob_logs[i], "M_hat")[0])
            for (kind, w), mdl in models.items():
                X, y = features(lg, w)
                pred = mdl.predict(X)
                per.setdefault(f"{kind.upper()} on {'torque/power/rpm' if w == 'tpr' else 'all signals'}", []).append(
                    float(np.sqrt(np.mean((pred - y) ** 2))))
        for k, v in per.items():
            m_, e_ = ci(v)
            rows.append([k, f(m_, 3), f(e_, 3), f(max(v), 3)])
        return rows, per

    test_logs = [mclogs[i] for i in range(n_mc)]
    est_in, per_in = est_rows(test_logs, nob)
    est_ood, per_ood = est_rows(ood_ekf)
    # run-out errors
    ro_rows = []
    ro_ekf, ro_ekf_end, ro_mb = [], [], []
    for i, lg in enumerate(test_logs):
        r_e = runout_error(lg, "M_hat")
        r_m = runout_error(lg, "M_mb")
        if r_e:
            ro_rows.append([i, f(r_e[2], 2), f(r_e[0], 2), f(r_m[0], 2), f(r_e[3], 2), f(r_e[1], 2), f(r_m[1], 2)])
            ro_ekf.append(r_e[0])
            ro_ekf_end.append(r_e[1])
            ro_mb.append(r_m[0])
    ro_summary = ""
    if len(ro_ekf) > 1:
        ra, rb, rc = np.asarray(ro_ekf), np.asarray(ro_ekf_end), np.asarray(ro_mb)
        ro_summary = (
            f"\nRun-out summary over {len(ra)} run-outs (signed error, mean +- 2 std):\n"
            f"- EKF at the starvation point: {ra.mean():+.3f} +- {2 * ra.std(ddof=1):.3f} kg "
            f"(max |error| {np.abs(ra).max():.3f} kg, SPS-312 threshold 0.5 kg, objective 0.25 kg)\n"
            f"- EKF at the end of the run-out: {rb.mean():+.3f} +- {2 * rb.std(ddof=1):.3f} kg "
            f"(max |error| {np.abs(rb).max():.3f} kg)\n"
            f"- Static mass balance at the starvation point: {rc.mean():+.2f} +- {2 * rc.std(ddof=1):.2f} kg\n")
        fig, ax = plt.subplots(figsize=(8.6, 2.6))
        ax.axhspan(-0.25, 0.25, color="#e6e6e6", lw=0)
        ax.axhline(0, color=MUTED, lw=0.8)
        ax.plot(np.arange(len(ra)), ra, "o", color=C["RST+FF"], ms=6, label="EKF error at the starvation point")
        ax.plot(np.arange(len(ra)), rb, "s", color=C["PID"], ms=5, label="EKF error at the end of the run-out")
        ax.text(len(ra) - 0.6, 0.2, "SPS-312 objective +-0.25 kg", fontsize=7, color=INK, ha="right")
        ax.set_ylim(-0.3, 0.3)
        ax.set_xlabel("run-out (one per even-numbered scenario)")
        ax.set_ylabel("estimate - truth (kg)")
        ax.legend(fontsize=7, loc="lower left", ncol=2)
        fig.tight_layout()
        fig.savefig(IMG / "runout_errors.png", dpi=160)
        plt.close(fig)
    # summary for scripts/aoa.py: RMSE in/out of range and mean |error| at the run-out point
    short = {"static mass balance (N, gear pump)": "static mass balance",
             "EKF, torque + pressure, batch events": "EKF", "MLP on torque/power/rpm": "MLP tpr",
             "MLP on all signals": "MLP full", "RIDGE on torque/power/rpm": "ridge tpr",
             "RIDGE on all signals": "ridge full"}
    ro_err = {v: [] for v in short.values()}
    for lg in test_logs:
        feed = lg.arr("feed")
        if feed.min() > 0.5:
            continue
        M, P = lg.arr("M"), lg.arr("P")
        i0 = int(np.argmax(feed < 0.5))
        below = np.where((P < 25.0) & (np.arange(len(P)) > i0))[0]
        i1 = int(below[0]) if len(below) else len(P) - 1
        ro_err["EKF"].append(abs(lg.arr("M_hat")[i1] - M[i1]))
        ro_err["static mass balance"].append(abs(lg.arr("M_mb")[i1] - M[i1]))
        for (kind, w), mdl in models.items():
            X, _ = features(lg, w)
            nm = f"{kind} {'tpr' if w == 'tpr' else 'full'}".replace("mlp", "MLP")
            ro_err[nm].append(abs(float(mdl.predict(X[i1 - 60:i1 - 59])[0]) - M[i1]))
    est_summary = {}
    for long_name, s_name in short.items():
        if long_name in per_in and long_name in per_ood:
            est_summary[s_name] = {"in": float(np.mean(per_in[long_name])), "ood": float(np.mean(per_ood[long_name])),
                                   "runout": float(np.mean(ro_err[s_name])) if ro_err[s_name] else float("nan")}
    (CACHE / "estimators.pkl").write_bytes(pickle.dumps(est_summary))
    # error-bar figure
    keys = list(per_in)[::-1]
    fig, ax = plt.subplots(figsize=(8.6, 3.8))
    ys = np.arange(len(keys))
    for per, col, lab, off in ((per_in, C["PID"], "in range (kappa 0.8-1.25)", 0.16),
                               (per_ood, C["RST"], "out of training range (kappa 0.6-1.5)", -0.16)):
        for y, k in zip(ys, keys):
            if k not in per:
                continue
            v = np.asarray(per[k])
            lo, hi = np.percentile(v, [10, 90])
            ax.plot([lo, hi], [y + off, y + off], color=col, lw=2, solid_capstyle="round")
            ax.plot(np.median(v), y + off, "o", color=col, ms=5)
        ax.plot([], [], "o-", color=col, label=lab)
    ax.set_yticks(ys, keys, fontsize=8)
    ax.set_xlabel("RMSE of compound mass in the screw per run (kg); dot = median, bar = 10th-90th percentile")
    ax.set_xlim(left=0)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(IMG / "estimator_errors.png", dpi=160)
    plt.close(fig)

    # ---------------- write results
    nom = vs[0].model
    txt = []
    txt.append("# Study results\n")
    txt.append(f"Generated by `scripts/run_study.py` ({'quick' if a.quick else 'full'} mode) in "
               f"{(time.time() - t_all) / 60:.0f} min. Everything is simulated; see the dossier for the "
               "sourced and assumed values.\n")
    txt.append("## 1. Operating map (gear-pump case, inlet pressure 60 bar)\n")
    txt.append(model_tab + "\n")
    txt.append("The vendor class has a 185 kW drive, 45 rpm maximum and 1100 kg/h maximum output at no head "
               "pressure (REFERENCES S5). The published specific energy for a 40 mm rubber extruder is "
               "0.3-1.7 kWh/kg (REFERENCES S6).\n")
    txt.append("## 2. Identification\n")
    txt.append(md_table(["test amplitude rpm", "pressure range bar", "linear OE fit, noise-free %"], idn["amp_rows"]) + "\n")
    txt.append(f"- Identification record (0.3 rpm, noisy): fit {idn['fit']:.1f} %.\n"
               f"- Validation record: free-run RMSE {idn['v_rmse']:.2f} bar, fit {idn['v_fit']:.1f} %.\n"
               f"- Identified poles: {', '.join(f'{p:.4f}' for p in mid.poles)}; DC gain {mid.dc_gain:.1f} bar/rpm.\n"
               f"- True nominal small-signal model: poles {', '.join(f'{p.real:.4f}' for p in nom.poles)}; "
               f"DC gain {nom.dc_gain:.1f} bar/rpm.\n")
    vrows = [[v.name, f(v.model.dc_gain, 1), ", ".join(f"{abs(p):.3f}" for p in v.model.poles)] for v in vs]
    txt.append("Plant variants (noise-free identification at 0.1 rpm):\n\n" +
               md_table(["variant", "DC gain bar/rpm", "|poles|"], vrows) + "\n")
    txt.append("## 3. Controller design on the uncertainty set\n")
    txt.append(f"Constraints on all 10 variants: Ms <= {MS_MAX}, speed ripple from 0.6 bar flight ripple <= "
               f"{RIPPLE_MAX} rpm, RMS speed noise from 0.3 bar sensor noise <= {NOISE_MAX} rpm. "
               "Objective: worst-case IAE (bar s) after a 1 rpm feed-equivalent input step.\n\n")
    txt.append(md_table(["controller", "worst IAE", "worst peak bar", "worst Ms", "ripple rpm", "noise rpm",
                         "4-bar step settle s (worst)", "overshoot bar (worst)", "feasible", "parameters"],
                        drows) + "\n")
    txt.append("\nMargins per variant (PID | RST):\n\n" + md_table(
        ["variant", "DC gain", "PID Ms", "PID GM", "PID PM deg", "PID DM s", "RST Ms", "RST GM", "RST PM deg",
         "RST DM s"], mrows) + "\n")
    txt.append(f"## 4. Closed loop, nonlinear plant ({n_mc} scenarios x {dur:.0f} s, mean +- 2 std)\n")
    txt.append(md_table(["controller", "pressure std bar", "99th pct |dev| bar", "% time |dev| > 3 bar",
                         "gauge (output) std %", "speed travel rpm/min"], cl_rows) + "\n")
    txt.append("## 5. Compound left in the screw\n")
    txt.append(f"In-range test set ({n_mc} runs, same scenarios as section 4, every second run ends with "
               "a 240 s run-out):\n\n")
    txt.append(md_table(["estimator", "mean RMSE kg", "2 std kg", "worst run kg"], est_in) + "\n")
    txt.append(f"\nOut-of-range viscosity (kappa 0.6-1.5; {len(ood_ekf)} runs; learned models trained on 0.8-1.25):\n\n")
    txt.append(md_table(["estimator", "mean RMSE kg", "2 std kg", "worst run kg"], est_ood) + "\n")
    cov_in = [float(np.mean(np.abs(lg.arr("M_hat")[60:] - lg.arr("M")[60:]) <= 2 * lg.arr("M_sig")[60:]))
              for lg in test_logs]
    cov_ood = [float(np.mean(np.abs(lg.arr("M_hat")[60:] - lg.arr("M")[60:]) <= 2 * lg.arr("M_sig")[60:]))
               for lg in ood_ekf]
    txt.append(f"\nEKF uncertainty calibration (SPS-313): the truth lies within +-2 sigma for "
               f"{100 * np.mean(cov_in):.1f} % of the logged seconds in range "
               f"(worst run {100 * min(cov_in):.1f} %) and {100 * np.mean(cov_ood):.1f} % out of range "
               f"(worst {100 * min(cov_ood):.1f} %). A well-calibrated Gaussian estimate gives about 95 %.\n")
    txt.append("\nRun-out (feed stopped, screw still turning). Error = estimate - truth:\n\n")
    txt.append(md_table(["run", "M at P < 25 bar", "EKF error", "mass-balance error", "M at end", "EKF error end",
                         "mass-balance error end"], ro_rows) + "\n")
    txt.append(ro_summary)
    txt.append("## 6. PLC cycle budget (CPU 1513-1 PN, OB30 = 100 ms)\n")
    txt.append(budget_tab + "\n")
    txt.append(f"\nSum of the worst-case calls: {budget['total_us']:.0f} us = "
               f"{budget['total_us'] / 1e3:.2f} ms per 100 ms cycle "
               f"({budget['total_us'] / 1e3:.1f} % load) at datasheet instruction times. "
               "EXP/LN are costed at 20 float operations each (assumption). Array indexing in optimized DBs "
               "and FB call overhead are not in the datasheet figures, so the dossier applies a 3x margin.\n")
    (RES / "study.md").write_text("\n".join(txt), encoding="utf-8")
    import csv
    with open(RES / "closed_loop_mc.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["controller", "run", "P_std", "P_p99", "out3", "gauge_std", "u_travel"])
        for k, vals in mc.items():
            for i, v in enumerate(vals):
                w.writerow([k, i, v["P_std"], v["P_p99"], v["out3"], v["gauge_std"], v["u_travel"]])
    print(f"done in {(time.time() - t_all) / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
