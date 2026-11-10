# Progress notes

**Status on 10 Nov 2026:** version 0.2.0 is complete and committed to local git. Nothing is pushed.

## Done
- Model, identification, controllers, estimators and SCL blocks. 26 tests pass.
- Full study in results/study.md and the trade study in results/aoa.md.
- **TIA V18:** 0 errors, 0 warnings, including OB30 at 100 ms. Last build: C:\TIA_xtrude\xtrude_20261110_024438.
- **0.2.0 fixes:**
  - EKF superbee transport: the run-out error fell from up to 0.62 kg to 0.04 kg or less.
  - The plant no longer creates material in an empty screw.
  - Leaner EKF covariance loops: 4.39 ms with margin, within the 5 ms allocation.
- DOSSIER.md (revision B) and docs/DOSSIER.pdf.

## Open (see DOSSIER §11–13)
- **PLCSIM run (V03):** not installed for any TIA version here. It needs a Siemens SiOS login.
- **Timing on a real CPU:** the OB30 execution time is not measured (VT-07).
- **Plant questions Q1–Q8:** listed in PLAN.md §5c and DOSSIER §12.
- **Future work:** a learned residual on top of the EKF, once weighed purges exist as labels.
