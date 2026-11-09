# Progress notes

**Status on 10 Nov 2026:** version 0.1.0 is complete and committed to local git. Nothing is pushed.

## Done
- Model, identification, controllers, estimators, SCL blocks.
- 24 tests pass.
- Full study in results/study.md and the trade study in results/aoa.md.
- TIA V18 compile: 0 errors, 0 warnings, with OB30 at 100 ms. The project is in C:\TIA_xtrude\xtrude_20261109_225203.
- DOSSIER.md and docs/DOSSIER.pdf.

## Open (see DOSSIER §11–13)
- **PLCSIM run (V03):** needs S7-PLCSIM V18, which is not installed.
- **Timing on the CPU:** the OB30 execution time has not been measured on a real CPU.
- **EKF run-out accuracy:** SPS-312 is not met. The EKF overestimates the last 0.3 kg by up to 0.6 kg.
- **Plant questions Q1–Q4:**
  - which outlet type the line has;
  - the current PID settings;
  - which signals are available;
  - whether a batch-change event exists.
- **Future work:** a learned residual on top of the EKF, once weighed purges exist as labels.
