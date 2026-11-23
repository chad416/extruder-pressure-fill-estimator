# Batch-fed twin-screw roller-die unit: start-up, starvation and remaining material

Written by `scripts/tsrd_study.py`: 8 seeds per case. Simulation only, on the
structural model `src/xtrude/tsrd.py`. Every parameter of that model is assumed. Setpoint
20 bar, robust PI Kp = 0.14 rpm/bar, Ti = 3.5 s, controller sample 0.1 s, logs at 1 s.

## A. The plant as a controller sees it

Open-loop step of +0.3 rpm at the running point with a full hopper. "Gain from a logged
start-up" is a first-order ARX fit on 40 s of the 1 s closed-loop log after the hand-over,
with no test signal. The recipe table (last three columns) is filled from that one run.

| Viscosity factor | Gain, bar/rpm | 63 % time, s | Gain from a logged start-up, bar/rpm | True gain relative to factor 1.0 | Learned relative gain | Learned steady speed, rpm | Learned revolutions to full |
|---|---|---|---|---|---|---|---|
| 0.7 | 4.03 | 2.1 | 3.33 | 0.70 | 0.78 | 11.67 | 15.4 |
| 1.0 | 5.76 | 2.7 | 4.26 | 1.00 | 1.00 | 11.72 | 16.2 |
| 1.4 | 8.06 | 3.5 | 5.36 | 1.40 | 1.26 | 11.63 | 17.2 |

## B. Start-up

Filling at 16 rpm, hand-over to the PI when the reading passes 3 bar. A
second batch follows after 60 s, so the hopper holds a reserve. Metrics as in the source
thesis: Ts = time from the first pressure until the pressure stays within ±10 % of the
setpoint; ISE = integral of the squared relative error over the first 300 s after it.

| Viscosity factor | Controller | Ts mean, s | Ts worst, s | ISE, s | Peak pressure, bar | Settled |
|---|---|---|---|---|---|---|
| 0.7 | PI | 13.4 | 14 | 1.40 | 25.6 | 8 of 8 |
| 0.7 | PI + gain | 9.4 | 10 | 1.15 | 24.2 | 8 of 8 |
| 0.7 | PI + preload | 7.8 | 10 | 1.05 | 23.1 | 8 of 8 |
| 0.7 | PI + preload + rev count | 3.0 | 3 | 0.86 | 20.8 | 8 of 8 |
| 0.7 | PI + all three | 2.9 | 3 | 0.85 | 20.7 | 8 of 8 |
| 1.0 | PI | 12.2 | 13 | 0.82 | 26.4 | 8 of 8 |
| 1.0 | PI + gain | 12.2 | 13 | 0.82 | 26.4 | 8 of 8 |
| 1.0 | PI + preload | 7.9 | 9 | 0.40 | 23.5 | 8 of 8 |
| 1.0 | PI + preload + rev count | 3.0 | 3 | 1.28 | 21.2 | 8 of 8 |
| 1.0 | PI + all three | 3.0 | 3 | 1.28 | 21.2 | 8 of 8 |
| 1.4 | PI | 12.2 | 13 | 1.48 | 27.3 | 8 of 8 |
| 1.4 | PI + gain | 15.1 | 16 | 2.09 | 29.2 | 8 of 8 |
| 1.4 | PI + preload | 8.2 | 9 | 0.94 | 23.7 | 8 of 8 |
| 1.4 | PI + preload + rev count | 3.8 | 4 | 1.48 | 21.3 | 8 of 8 |
| 1.4 | PI + all three | 4.0 | 4 | 1.51 | 21.4 | 8 of 8 |

## C. Production with a thin hopper reserve

Eight batches. The second follows 115 s after the first, then one per mixer cycle with
±20 s jitter, so the hopper holds only about 51 kg before a punctual drop and runs
short when the mixer is late. Statistics from t = 100 s to 100 s after the last drop. "Outside ±10 %" is a share of time. The
model errors are eps_max ×0.95, m_c ×1.2, V_rev ×1.03 and throughput calibration ×1.02.

| Viscosity factor | Controller | Peak pressure mean, bar | Peak pressure worst, bar | Lowest pressure, bar | Outside ±10 %, % | Peak speed, rpm |
|---|---|---|---|---|---|---|
| 0.7 | PI | 31.5 | 56.4 | 12.3 | 14.1 | 18.2 |
| 0.7 | PI + feedforward and hold | 23.6 | 30.2 | 12.4 | 3.3 | 16.8 |
| 0.7 | PI + feedforward and hold, model with errors | 23.1 | 24.9 | 10.6 | 8.2 | 15.8 |
| 1.0 | PI | 36.7 | 78.5 | 11.2 | 15.5 | 19.0 |
| 1.0 | PI + feedforward and hold | 25.0 | 36.3 | 10.8 | 4.3 | 17.2 |
| 1.0 | PI + feedforward and hold, model with errors | 23.8 | 28.7 | 9.8 | 9.5 | 16.1 |
| 1.4 | PI | 42.2 | 95.7 | 9.6 | 16.9 | 19.9 |
| 1.4 | PI + feedforward and hold | 28.3 | 44.9 | 10.4 | 5.9 | 17.6 |
| 1.4 | PI + feedforward and hold, model with errors | 26.1 | 38.6 | 8.6 | 11.3 | 16.5 |

## D. Remaining material (topic 3)

From the same production runs. "Total" is hopper + screws + chamber from the mass balance
of batch weights and roller-die throughput. "Screws" compares the conveying register of
the material model with an inversion of the torque balance that assumes the nominal
recipe viscosity. Each run continues after the last drop until the unit is empty. The
time-to-empty error is taken each time the true value passes 30 s.

| Material model | Total: RMSE, kg | Total: error at the end, kg | Screws, register: RMSE, kg | Screws, torque alone: RMSE, kg | Time to empty at 30 s: error, s (events) |
|---|---|---|---|---|---|
| exact parameters | 0.1 | +0.1 | 0.2 | 10.0 | +1.2 ± 0.2 (49) |
| parameters with errors | 18.5 | -31.1 | 3.9 | 10.0 | -15.3 ± 4.9 (49) |
