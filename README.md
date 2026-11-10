# extruder-pressure-fill-estimator

This project controls the pressure in front of the outlet pump of a **cold-feed rubber extruder** using a model, and it estimates **how much compound is left in the screw**. Both run as Siemens **SCL** blocks compiled in TIA Portal V18, and the work is laid out as a systems-engineering dossier that follows Wasson's spine.

- **Read first:** [DOSSIER.md](DOSSIER.md) ([PDF](docs/DOSSIER.pdf)). It covers the need, ConOps, modes, requirements, architecture, interfaces, budgets, M&S, FMEA, trade study and V&V plan.
- **Numbers:** [results/study.md](results/study.md) and [results/aoa.md](results/aoa.md).
- **Traceability and open items:** [TRACEABILITY.md](TRACEABILITY.md) and [OPEN_ITEMS.md](OPEN_ITEMS.md).
- **Data:** everything here is **simulated**. No data from a real line has been used. [DOSSIER §8.2](DOSSIER.md) lists which parameters are sourced and which are assumed.

![Pressure, disturbances and the screw-fill estimate in one production scenario](docs/img/scenario_timeline.png)

## Findings

The numbers come from simulation: 24 random production scenarios of 1200 s each, run as
paired comparisons.

1. **What the pressure sits in front of matters.**
   - **Open die:** the speed-to-pressure gain is close to zero once viscous heating settles.
   - **Gear pump:** the plant is lag-dominated, with a gain of 6.7–110 bar/rpm across 40–80 bar.

   ![Outlet step responses](docs/img/outlet_step_response.png)

2. **A robust RST does not regulate better than a robust PID** when both are tuned on the same
   10 plant variants under the same limits.
   - **Regulation:** 99th-percentile deviation 1.09 vs 1.05 bar.
   - **Where the RST is better:**
     - margin: Ms 1.46 vs 1.60;
     - 13 % less speed travel, better in all 24 runs;
     - zero setpoint overshoot.

   The 0.5 Hz flight ripple limits both controllers.

   ![Load responses on the plant variants](docs/img/load_response_variants.png)

3. **The fill estimator is where the gain is.** A feed change reaches the pressure about 35 s
   after the torque has already seen it. Feedforward from the estimate cuts the
   99th-percentile deviation by 22–28 % (PID+FF better in 21 of 24 runs, RST+FF in 23 of 24).

   ![Closed-loop Monte Carlo](docs/img/closed_loop_mc.png)

4. **Compound left in the screw** (about 8 kg in the pumping section):

   | Estimator | RMSE in range | RMSE out of training range | Needs labels? |
   |---|---|---|---|
   | Physics EKF | 0.093 kg | 0.145 kg | no |
   | MLP on torque, power and rpm only | 0.29 kg | 0.38 kg | yes |

   - **Batch-change events:** without them the EKF error rises to 0.31 kg.
   - **Run-out:** at the point the gear pump starts to starve, the EKF is within ±0.04 kg in all
     12 run-outs. Version 0.1.0 was up to 0.62 kg off. Second-order transport fixed it.
   - **Uncertainty:** the reported ±2σ band holds the truth 99.9 % of the time.

   ![Estimator errors](docs/img/estimator_errors.png)
   ![Run-out errors](docs/img/runout_errors.png)

5. **PLC fit.**
   - **Budget:** 1.46 ms per 100 ms cycle at datasheet instruction times, 4.4 ms with a 3× margin.
   - **Compile:** all SCL blocks compile in TIA Portal V18 with 0 errors and 0 warnings.
   - **Float32 checks:** the float32 runs match the Python references.
   - **A defect float32 exposed:** rounding the RST coefficients to float32 broke T(1) = R(1),
     which caused a pressure offset of several bar. The SCL now enforces that identity, and
     S(1) = 0, in its code structure.

**What is not done:**
- PLCSIM run (S7-PLCSIM is not installed; see OPEN_ITEMS.md);
- execution time measured on a real CPU;
- any data from a real line.

The V&V plan is in [DOSSIER §11](DOSSIER.md).

## What is in the repo

| Path | Content |
|---|---|
| `src/xtrude/physics.py` | Screw pumping: power-law viscosity with temperature shift, drag/pressure flow with shape factors, power |
| `src/xtrude/plant.py` | Extruder simulator: partially filled screw, moving fill front, batches carried with the compound, compressible head, gear-pump or die outlet, drive, sensors |
| `src/xtrude/ident.py` | Technician test (±0.3 rpm, 2 min) and output-error identification |
| `src/xtrude/control.py` | PID, RST pole placement (Diophantine, observer anti-windup), DMC-type MPC, robustness analysis |
| `src/xtrude/design.py` | Robust tuning of PID and RST on 10 plant variants under the same constraints |
| `src/xtrude/estimator.py` | Screw-fill EKF (12 compartments + feed fill + consistency + filled length) and the feedforward speed |
| `src/xtrude/learned.py` | Ridge and MLP estimators on torque/power/rpm (topic-3 literal) and on all signals |
| `src/xtrude/scl.py` | Translator that runs the SCL in Python with float32 Real and counts operations |
| `plc/FB_*.scl` | `FB_PressurePID`, `FB_PressureRST`, `FB_FillEKF`, `FB_PressureSupervisor` |
| `plc/tia/` | OB30 (100 ms), DBs, generated `FC_LoadParams` |
| `scripts/run_study.py` | Whole study → `results/`, `docs/img/` |
| `scripts/aoa.py` | Trade study → `results/aoa.md` |
| `scripts/tia_*.ps1` | TIA Openness build, OB30 conversion, screenshot |

## Run it

```bash
pip install -e .[test]
python -m pytest -q                 # plant, control, estimator, SCL equivalence
python scripts/run_study.py         # about an hour on a laptop; stages are cached
python scripts/aoa.py
```

```powershell
powershell -ExecutionPolicy Bypass -File scripts\tia_build.ps1 -OutDir C:\TIA_xtrude
powershell -ExecutionPolicy Bypass -File scripts\tia_ob30.ps1  -Project <path.ap18>
powershell -ExecutionPolicy Bypass -File scripts\tia_screenshot.ps1 -Project <path.ap18> -Block FB_PressureRST -Out docs\img\tia_FB_PressureRST.png
```

## Licence

MIT, see [LICENSE](LICENSE).
