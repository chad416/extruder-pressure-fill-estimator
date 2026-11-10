# Traceability: requirement → implementation → verification → evidence

The requirements are defined in [DOSSIER.md §4](DOSSIER.md). The verification methods are
I (inspection), A (analysis), D (demonstration) and T (test), following [W] p.270–272. This
table is the requirements verification traceability matrix (RVTM) of DOSSIER §11.1, extended
with the code and test locations.

| Req. | Need | Implemented in | Verified by | Evidence | Status (10 Nov 2026) |
|---|---|---|---|---|---|
| SPS-201 Scrap ≤ baseline | N1 | whole system | T: line trial VAL-1 | — | open (plant) |
| SPS-202 Start-up waste ≤ baseline | N2 | whole system | T: VAL-1 | — | open (plant) |
| SPS-203 Auto-mode availability ≥ 99 % | MOS-1 | `FB_PressureSupervisor` | A: DOSSIER §9.3 + T | DOSSIER §9.3 (Ai ≈ 0.9999, estimate) | A met (estimate); T open |
| SPS-301 Within ±3 bar ≥ 99 % of time | N1, N3 | `FB_PressureRST`/`FB_PressurePID` + `FB_FillEKF` FF | A: `scripts/run_study.py` §4 | results/study.md §4, `docs/img/closed_loop_mc.png` | A: threshold met by all; objective not met; T open |
| SPS-302 Peak ≤ 3.5 bar after 1 rpm feed loss | N1 | `xtrude.design` tuning | A: `design.score` on 10 variants | results/study.md §3 | threshold met |
| SPS-303 Ms ≤ 1.6 on all variants | N3 | `xtrude.design` | A: `control.margins` | results/study.md §3 margins table | met (RST 1.46, PID 1.60) |
| SPS-304 4-bar step: ≤ 20 s, overshoot ≤ 1 bar | N2 | RST reference model Bm/Am | A: `run_study.step_metrics` | results/study.md §3 | RST met; PID needs a setpoint filter |
| SPS-305 Speed ripple ≤ 0.1 rpm, noise ≤ 0.1 rpm | N1, N5 | tuning constraints | A: `design.ripple_amp`, `design.noise_rms` | results/study.md §3 | met (design constraint) |
| SPS-306 Mode change ≤ 0.2 rpm step | N5 | tracking in all controller FBs | T: `tests/test_scl.py` (Enable = FALSE tracking) | pytest | T met in translator; PLCSIM open |
| SPS-311 Mass RMSE ≤ 0.3 kg | N4 | `FB_FillEKF`, `xtrude.estimator.FillEKF` | A: Monte Carlo | results/study.md §5 | met (0.093 kg mean, 0.116 kg worst) |
| SPS-312 Run-out error ≤ 0.5 kg | N2, N4 | TVD transport in `FB_FillEKF` v0.2 | A: Monte Carlo; T: `test_runout_estimate_at_starvation_point` | results/study.md §5, `docs/img/runout_errors.png` | met (+0.013 ± 0.029 kg, max 0.035 kg) |
| SPS-313 1-σ reported | N4 | `FB_FillEKF.MassSigma` | I + A: ±2σ coverage | results/study.md §5 coverage line | met (99.9 % in range, 97.7 % out of range) |
| SPS-321 Pressure fault → SpeedHold in 1 cycle | N5 | `FB_PressureSupervisor` | T: `test_supervisor_fallbacks`, `test_frozen_pressure_detected` | pytest | met in translator; PLCSIM open |
| SPS-322 Torque fault → FF off in 1 cycle | N5 | `FB_PressureSupervisor` | T: `test_supervisor_fallbacks` | pytest | met in translator |
| SPS-323 Overpressure → ramp down + latch | N5 | `FB_PressureSupervisor` | T: `test_supervisor_fallbacks` | pytest | met in translator |
| SPS-361 Execution ≤ 5 ms per 100 ms incl. margin | constraint | all FBs | A: operation count × datasheet times (`run_study.stage_budget`); T: CPU cycle statistics (VT-07) | results/study.md §6 | A met (1.46 ms, 4.39 ms with 3× margin, **estimate**); T open |
| SPS-362 SCL, TIA V18, 0 errors / 0 warnings | constraint | `plc/` | T: TIA Openness compile | results/tia_compile*.txt, docs/img/tia_*.png | met |
| SPS-363 Existing signals only | constraint | `plc/tia/LineIO_OB_DB.scl` | I | DOSSIER §6 interface table | met |
| SPS-364 Re-identification ≤ 2 min at ±0.3 rpm | constraint | `xtrude.ident.step_test` | A + D | results/study.md §2 | A met; D open (plant) |

## Code-to-requirement index

| File | Requirements |
|---|---|
| `plc/FB_PressurePID.scl` | SPS-301…306 (baseline) |
| `plc/FB_PressureRST.scl` | SPS-301…306 |
| `plc/FB_FillEKF.scl` | SPS-301 (FF), SPS-311…313, SPS-361 |
| `plc/FB_PressureSupervisor.scl` | SPS-203, SPS-306, SPS-321…323 |
| `plc/tia/*` | SPS-361…363 |
| `src/xtrude/design.py` | SPS-302, 303, 305 |
| `src/xtrude/ident.py` | SPS-364 |
