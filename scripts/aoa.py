"""Analysis of alternatives (Wasson ch. 32): weighted scoring with utility functions and a
+-10 % weight sensitivity check, for the pressure controller and for the fill estimator.

The scores come from results/closed_loop_mc.csv and the cached design and estimator
results. The few judgement scores (maintainability, implementation risk) are entered
here with their reason, so the arithmetic can be checked (Wasson p.694 warns about
scoring errors; P32.5: an undocumented trade study is opinion).

    python scripts/aoa.py   ->  results/aoa.md
"""

from __future__ import annotations

import csv
import itertools
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
RES = ROOT / "results"


def utility(x: float, worst: float, best: float) -> float:
    """Linear utility, 0 at `worst`, 1 at `best`, clipped (Wasson Fig. 32.9, linear profile)."""
    u = (x - worst) / (best - worst)
    return float(min(max(u, 0.0), 1.0))


def rank(alts: dict[str, dict[str, float]], weights: dict[str, float]) -> list[tuple[str, float]]:
    tot = sum(weights.values())
    out = [(a, sum(weights[k] * s[k] for k in weights) / tot * 100) for a, s in alts.items()]
    return sorted(out, key=lambda t: -t[1])


def sensitivity(alts, weights) -> list[str]:
    """Cut each weight by 10 % (renormalised) and raise it by 10 %; report winner changes."""
    base = rank(alts, weights)[0][0]
    notes = []
    for k, f in itertools.product(weights, (0.9, 1.1)):
        w = dict(weights)
        w[k] *= f
        win = rank(alts, w)[0][0]
        if win != base:
            notes.append(f"weight '{k}' x{f}: winner changes {base} -> {win}")
    return notes or [f"no single +-10 % weight change moves the winner away from {base}"]


def table(alts, weights, raw=None) -> str:
    keys = list(weights)
    head = "| alternative | " + " | ".join(f"{k} ({weights[k]})" for k in keys) + " | FOM |"
    sep = "|" + "---|" * (len(keys) + 2)
    rows = [head, sep]
    for a, fom in rank(alts, weights):
        cells = []
        for k in keys:
            r = f" ({raw[a][k]})" if raw and k in raw.get(a, {}) else ""
            cells.append(f"{alts[a][k]:.2f}{r}")
        rows.append(f"| {a} | " + " | ".join(cells) + f" | **{fom:.1f}** |")
    return "\n".join(rows)


def main() -> None:
    mc: dict[str, list[dict]] = {}
    with open(RES / "closed_loop_mc.csv") as fh:
        for r in csv.DictReader(fh):
            mc.setdefault(r["controller"], []).append({k: float(v) for k, v in r.items() if k not in ("controller",)})
    des = pickle.loads((RES / "cache" / "design.pkl").read_bytes())
    ms = {"PID": des["pid"][2].ms, "PID+FF": des["pid"][2].ms, "RST": des["rst"][2].ms, "RST+FF": des["rst"][2].ms,
          "MPC": float("nan")}
    # ---------------- controller AoA
    weights = {"regulation": 30, "robustness": 15, "degradation": 10, "PLC load": 10, "maintainability": 20,
               "implementation risk": 15}
    # judgement scores (0..1) with the reason recorded
    judge = {
        "PID": {"degradation": 0.6, "maintainability": 1.0, "implementation risk": 1.0},
        "PID+FF": {"degradation": 0.9, "maintainability": 0.8, "implementation risk": 0.7},
        "RST": {"degradation": 0.6, "maintainability": 0.6, "implementation risk": 0.8},
        "RST+FF": {"degradation": 0.9, "maintainability": 0.5, "implementation risk": 0.6},
        "MPC": {"degradation": 0.6, "maintainability": 0.3, "implementation risk": 0.4},
    }
    load_ms = {"PID": 0.02, "PID+FF": 1.7, "RST": 0.05, "RST+FF": 1.7, "MPC": 2.5}
    alts, raw = {}, {}
    for a in ("PID", "PID+FF", "RST", "RST+FF", "MPC"):
        p99 = float(np.mean([r["P_p99"] for r in mc[a]]))
        alts[a] = {
            "regulation": utility(p99, worst=4.0, best=1.0),
            "robustness": utility(ms[a], worst=2.0, best=1.3) if np.isfinite(ms[a]) else 0.3,
            "degradation": judge[a]["degradation"],
            "PLC load": utility(load_ms[a], worst=10.0, best=0.0),
            "maintainability": judge[a]["maintainability"],
            "implementation risk": judge[a]["implementation risk"],
        }
        raw[a] = {"regulation": f"p99 {p99:.2f} bar", "robustness": f"Ms {ms[a]:.2f}" if np.isfinite(ms[a]) else "not analysed",
                  "PLC load": f"{load_ms[a]} ms"}
    txt = ["# Analysis of alternatives\n",
           "Method: Wasson ch. 32 (12-step method, p.689-690; normalised 100-point weights, Fig. 32.8; "
           "linear utility functions, Fig. 32.9; +-10 % weight sensitivity, Sec. 32.7). "
           "Scores are 0..1; FOM = weighted sum on 0..100.\n",
           "## Pressure controller\n",
           "Utility ranges:\n"
           "- regulation: 99th percentile of the pressure deviation in the closed-loop Monte Carlo, 4 bar -> 0, 1 bar -> 1;\n"
           "- robustness: worst Ms over the 10 plant variants, 2.0 -> 0, 1.3 -> 1 (MPC was not analysed in the "
           "frequency domain, scored 0.3);\n"
           "- PLC load: worst-case execution per 100 ms cycle from results/study.md section 6, rounded up "
           "(MPC: 60-step prediction + 20 projected-gradient iterations, estimated), 10 ms -> 0, 0 -> 1.\n",
           "Judgement scores and their reasons:\n"
           "- degradation: alternatives with the fill estimator keep a model-based fallback and give an early "
           "feed-loss warning (0.9); the others only have the supervisor's fallbacks (0.6).\n"
           "- maintainability: plant engineers already maintain PID loops (1.0); a feedforward signal adds one "
           "explainable block (0.8); RST coefficients need the design tool to change (0.6); RST+FF both (0.5); "
           "MPC needs a model, horizons and a solver (0.3).\n"
           "- implementation risk: PID is the current solution (1.0); RST compiled and float32-checked here (0.8); "
           "FF depends on the estimator and a batch-event signal from the line (-0.2 / -0.3); MPC in SCL not built (0.4).\n",
           table(alts, weights, raw) + "\n", "Sensitivity:\n"]
    txt += [f"- {n}" for n in sensitivity(alts, weights)]
    # assumption tests (Wasson P32.10: a study is only as valid as its assumptions)
    txt.append("\nAssumption tests (not a re-scoring; they show what the ranking depends on):\n")
    alts_b = {a: dict(v) for a, v in alts.items()}
    for a in alts_b:
        p99 = float(np.mean([r["P_p99"] for r in mc[a]]))
        alts_b[a]["regulation"] = utility(p99, worst=1.6, best=0.7)
    txt.append(f"- Regulation utility stretched to the observed range (1.6 bar -> 0, 0.7 bar -> 1): "
               f"ranking {', '.join(f'{a} {f:.1f}' for a, f in rank(alts_b, weights))}.")
    w_c = dict(weights)
    w_c["actuator wear"] = 10
    alts_c = {a: dict(v) for a, v in alts.items()}
    for a in alts_c:
        trav = float(np.mean([r["u_travel"] for r in mc[a]]))
        alts_c[a]["actuator wear"] = utility(trav, worst=13.0, best=7.0)
    txt.append(f"- Adding an actuator-wear factor (10 points; speed travel 13 rpm/min -> 0, 7 -> 1; traces to MOS-1): "
               f"ranking {', '.join(f'{a} {f:.1f}' for a, f in rank(alts_c, w_c))}.")
    for wreg in (40, 50, 60):
        w_d = dict(weights)
        w_d["regulation"] = wreg
        txt.append(f"- Regulation weight {wreg} with the stretched utility: winner {rank(alts_b, w_d)[0][0]}.")
    # ---------------- estimator AoA (from the study.md numbers stored in the cache)
    txt.append("\n## Fill estimator\n")
    est = pickle.loads((RES / "cache" / "estimators.pkl").read_bytes()) if (RES / "cache" / "estimators.pkl").exists() else None
    if est:
        w2 = {"accuracy in range": 35, "accuracy out of range": 25, "run-out accuracy": 15, "sensors needed": 10,
              "explainability": 15}
        alts2, raw2 = {}, {}
        sensors = {"static mass balance": 1.0, "EKF": 0.7, "MLP tpr": 1.0, "MLP full": 0.7, "ridge tpr": 1.0,
                   "ridge full": 0.7}
        expl = {"static mass balance": 1.0, "EKF": 0.8, "MLP tpr": 0.2, "MLP full": 0.2, "ridge tpr": 0.5,
                "ridge full": 0.5}
        for a, v in est.items():
            alts2[a] = {"accuracy in range": utility(v["in"], 0.6, 0.05), "accuracy out of range": utility(v["ood"], 0.6, 0.05),
                        "run-out accuracy": utility(v["runout"], 1.5, 0.1), "sensors needed": sensors[a],
                        "explainability": expl[a]}
            raw2[a] = {"accuracy in range": f"{v['in']:.3f} kg", "accuracy out of range": f"{v['ood']:.3f} kg",
                       "run-out accuracy": f"{v['runout']:.2f} kg"}
        txt.append("Utility: RMSE 0.6 kg -> 0, 0.05 kg -> 1 (in and out of the training viscosity range); "
                   "mean |error| at the run-out point 1.5 kg -> 0, 0.1 kg -> 1. Sensors: torque/power/rpm only = 1, "
                   "plus head pressure and gear-pump speed = 0.7. Explainability: physics balance 1, EKF 0.8, ridge 0.5, MLP 0.2.\n")
        txt.append(table(alts2, w2, raw2) + "\n")
        txt.append("Sensitivity:\n")
        txt += [f"- {n}" for n in sensitivity(alts2, w2)]
    (RES / "aoa.md").write_text("\n".join(txt) + "\n", encoding="utf-8")
    print("\n".join(txt))


if __name__ == "__main__":
    main()
