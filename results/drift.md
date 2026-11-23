# Transducer zero drift (FMEA row F3)

Written by `scripts/drift_study.py`: 12 paired production scenarios of 1200 s per
case, robust PID (0.108, 0.472, 0.169), fill EKF running. Simulation only. Offset = reading
minus true pressure, constant during a run. Statistics are taken after t = 180 s.

## A. Consequence in closed loop

The controller holds the *reading* at the setpoint, so the true pressure sits at setpoint
minus offset. With the assumed gear-pump fill curve (TBD-4) the throughput follows at
0.12 % per bar at 60 bar and 0.50 % per bar at 40 bar (analytic slope of the curve).
Starvation = true pressure below 25 bar.

| Setpoint, bar | Offset, bar | Mean true pressure, bar | Throughput change, % | Lowest true pressure, bar | Time below 25 bar, % | Runs that touched 25 bar |
|---|---|---|---|---|---|---|
| 60 | -8 | 67.9 | +0.77 | 57.2 | 0.00 | 0 of 12 |
| 60 | -4 | 63.9 | +0.44 | 53.5 | 0.00 | 0 of 12 |
| 60 | -2 | 61.9 | +0.23 | 51.3 | 0.00 | 0 of 12 |
| 60 | +0 | 59.9 | +0.00 | 50.0 | 0.00 | 0 of 12 |
| 60 | +2 | 57.9 | -0.27 | 48.2 | 0.00 | 0 of 12 |
| 60 | +4 | 55.9 | -0.57 | 46.8 | 0.00 | 0 of 12 |
| 60 | +8 | 51.9 | -1.32 | 43.1 | 0.00 | 0 of 12 |
| 40 | -8 | 47.9 | +3.09 | 39.7 | 0.00 | 0 of 12 |
| 40 | -4 | 43.9 | +1.75 | 36.2 | 0.00 | 0 of 12 |
| 40 | -2 | 41.9 | +0.93 | 34.5 | 0.00 | 0 of 12 |
| 40 | +0 | 39.9 | +0.00 | 33.1 | 0.00 | 0 of 12 |
| 40 | +2 | 38.0 | -1.07 | 31.5 | 0.00 | 0 of 12 |
| 40 | +4 | 36.0 | -2.29 | 29.9 | 0.00 | 0 of 12 |
| 40 | +8 | 32.0 | -5.27 | 26.7 | 0.00 | 0 of 12 |

## B. What the fill estimator sees

Mass-error shift = change of the mean estimate error against the same scenario with no
offset. The innovation is normalised by its own 1-σ from the filter. "Flagged" counts runs
whose run-mean pressure innovation exceeds the largest value seen in the no-offset runs of
the same setpoint, a threshold with zero false alarms on those runs. If an offset changed
nothing at all, about one run in 13 would still exceed it by chance.

| Setpoint, bar | Offset, bar | Mass-error shift, kg | Mass RMSE, kg | Pressure innovation, run mean ± sd (σ) | Torque innovation, run mean (σ) | Flagged |
|---|---|---|---|---|---|---|
| 60 | -8 | -0.073 | 0.155 | -0.007 ± 0.013 | -0.005 | 0 of 12 |
| 60 | -4 | -0.041 | 0.127 | -0.008 ± 0.013 | -0.004 | 0 of 12 |
| 60 | -2 | -0.022 | 0.111 | -0.010 ± 0.011 | -0.004 | 0 of 12 |
| 60 | +0 | +0.000 | 0.093 | -0.008 ± 0.012 | -0.003 | — |
| 60 | +2 | +0.023 | 0.077 | -0.010 ± 0.013 | -0.005 | 0 of 12 |
| 60 | +4 | +0.050 | 0.064 | -0.005 ± 0.009 | -0.003 | 0 of 12 |
| 60 | +8 | +0.116 | 0.078 | -0.008 ± 0.012 | -0.004 | 0 of 12 |
| 40 | -8 | -0.241 | 0.405 | -0.009 ± 0.009 | -0.004 | 0 of 12 |
| 40 | -4 | -0.138 | 0.304 | -0.009 ± 0.011 | -0.006 | 0 of 12 |
| 40 | -2 | -0.075 | 0.242 | -0.009 ± 0.011 | -0.005 | 1 of 12 |
| 40 | +0 | +0.000 | 0.171 | -0.011 ± 0.012 | -0.005 | — |
| 40 | +2 | +0.085 | 0.097 | -0.009 ± 0.013 | -0.005 | 1 of 12 |
| 40 | +4 | +0.185 | 0.068 | -0.007 ± 0.012 | -0.003 | 0 of 12 |
| 40 | +8 | +0.436 | 0.286 | -0.012 ± 0.013 | -0.006 | 1 of 12 |

Drift ramp of 32 bar/h at 60 bar (8 bar in 15 minutes): mean pressure innovation
-0.018 before the ramp, -0.004 in its last 5 minutes (offset 8.0 bar at the end, 12 runs).

## C. Zero check at run-out

`xtrude.zero_check.RunoutZeroCheck` on 12 run-outs per offset at 60 bar. It arms when the
feed is off and the torque is below 4 kN m, and latches the slow-filtered reading once a
2 s and a 10 s filter agree within 0.3 bar for 20 s. A capture counts as false when the true
head pressure was still above 0.5 bar at that moment.

| True offset, bar | Captured | Mean error, bar | Largest error, bar | Captured after feed stop, s (mean, latest) | False captures |
|---|---|---|---|---|---|
| -4 | 12 of 12 | +0.04 | 0.10 | 106, 112 | 0 |
| +0 | 12 of 12 | +0.04 | 0.09 | 106, 111 | 0 |
| +4 | 12 of 12 | +0.04 | 0.09 | 105, 109 | 0 |
| +8 | 12 of 12 | +0.04 | 0.07 | 105, 109 | 0 |

This part rests on the simulated head draining to 0 bar at run-out. Whether a real head
does is open (dossier §12, Q5). A 4–20 mA transmitter also cannot report far below zero, so
a negative offset larger than its under-range would be clipped.
