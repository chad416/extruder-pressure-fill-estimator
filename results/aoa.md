# Analysis of alternatives

Method: Wasson ch. 32 (12-step method, p.689-690; normalised 100-point weights, Fig. 32.8; linear utility functions, Fig. 32.9; +-10 % weight sensitivity, Sec. 32.7). Scores are 0..1; FOM = weighted sum on 0..100.

## Pressure controller

Utility ranges:
- regulation: 99th percentile of the pressure deviation in the closed-loop Monte Carlo, 4 bar -> 0, 1 bar -> 1;
- robustness: worst Ms over the 10 plant variants, 2.0 -> 0, 1.3 -> 1 (MPC was not analysed in the frequency domain, scored 0.3);
- PLC load: worst-case execution per 100 ms cycle from results/study.md section 6, rounded up (MPC: 60-step prediction + 20 projected-gradient iterations, estimated), 10 ms -> 0, 0 -> 1.

Judgement scores and their reasons:
- degradation: alternatives with the fill estimator keep a model-based fallback and give an early feed-loss warning (0.9); the others only have the supervisor's fallbacks (0.6).
- maintainability: plant engineers already maintain PID loops (1.0); a feedforward signal adds one explainable block (0.8); RST coefficients need the design tool to change (0.6); RST+FF both (0.5); MPC needs a model, horizons and a solver (0.3).
- implementation risk: PID is the current solution (1.0); RST compiled and float32-checked here (0.8); FF depends on the estimator and a batch-event signal from the line (-0.2 / -0.3); MPC in SCL not built (0.4).

| alternative | regulation (30) | robustness (15) | degradation (10) | PLC load (10) | maintainability (20) | implementation risk (15) | FOM |
|---|---|---|---|---|---|---|---|
| PID | 0.98 (p99 1.05 bar) | 0.57 (Ms 1.60) | 0.60 | 1.00 (0.02 ms) | 1.00 | 1.00 | **89.0** |
| PID+FF | 1.00 (p99 0.78 bar) | 0.57 (Ms 1.60) | 0.90 | 0.83 (1.7 ms) | 0.80 | 0.70 | **82.4** |
| RST | 0.97 (p99 1.09 bar) | 0.76 (Ms 1.46) | 0.60 | 0.99 (0.05 ms) | 0.60 | 0.80 | **80.5** |
| RST+FF | 1.00 (p99 0.83 bar) | 0.76 (Ms 1.46) | 0.90 | 0.83 (1.7 ms) | 0.50 | 0.60 | **77.8** |
| MPC | 0.80 (p99 1.60 bar) | 0.30 (not analysed) | 0.60 | 0.75 (2.5 ms) | 0.30 | 0.40 | **54.0** |

Sensitivity:

- no single +-10 % weight change moves the winner away from PID

Assumption tests (not a re-scoring; they show what the ranking depends on):

- Regulation utility stretched to the observed range (1.6 bar -> 0, 0.7 bar -> 1): ranking PID+FF 79.8, PID 77.8, RST+FF 73.4, RST 68.3, MPC 30.0.
- Adding an actuator-wear factor (10 points; speed travel 13 rpm/min -> 0, 7 -> 1; traces to MOS-1): ranking PID 84.6, RST 78.8, PID+FF 76.2, RST+FF 73.7, MPC 56.4.
- Regulation weight 40 with the stretched utility: winner PID+FF.
- Regulation weight 50 with the stretched utility: winner PID+FF.
- Regulation weight 60 with the stretched utility: winner PID+FF.

## Fill estimator

Utility: RMSE 0.6 kg -> 0, 0.05 kg -> 1 (in and out of the training viscosity range); mean |error| at the run-out point 1.5 kg -> 0, 0.1 kg -> 1. Sensors: torque/power/rpm only = 1, plus head pressure and gear-pump speed = 0.7. Explainability: physics balance 1, EKF 0.8, ridge 0.5, MLP 0.2.

| alternative | accuracy in range (35) | accuracy out of range (25) | run-out accuracy (15) | sensors needed (10) | explainability (15) | FOM |
|---|---|---|---|---|---|---|
| EKF | 0.92 (0.093 kg) | 0.83 (0.145 kg) | 1.00 (0.02 kg) | 0.70 | 0.80 | **87.0** |
| MLP full | 0.93 (0.090 kg) | 0.83 (0.144 kg) | 0.97 (0.14 kg) | 0.70 | 0.20 | **77.7** |
| ridge full | 0.73 (0.200 kg) | 0.58 (0.281 kg) | 0.47 (0.84 kg) | 0.70 | 0.50 | **61.5** |
| MLP tpr | 0.56 (0.293 kg) | 0.41 (0.375 kg) | 0.98 (0.13 kg) | 1.00 | 0.20 | **57.5** |
| static mass balance | 0.32 (0.426 kg) | 0.24 (0.470 kg) | 0.00 (3.61 kg) | 1.00 | 1.00 | **42.0** |
| ridge tpr | 0.42 (0.370 kg) | 0.20 (0.492 kg) | 0.19 (1.24 kg) | 1.00 | 0.50 | **39.8** |

Sensitivity:

- no single +-10 % weight change moves the winner away from EKF
