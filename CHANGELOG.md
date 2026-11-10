# Changelog

## 0.2.0 (10 Nov 2026)

- **EKF (Python and FB_FillEKF v0.2):**
  - The fill transport now uses a second-order TVD scheme (superbee) instead of first-order
    upwind.
  - Fill process noise now grows with the local fill step.
  - Together these fix the run-out overestimate of 0.1.0, which was 0.33–0.62 kg at the
    starvation point and missed SPS-312. They also give the ±2σ band honest coverage.
- **Plant:** when the filled zone empties, the screw now delivers only the material that is
  available. In 0.1.0 a clamp on the filled-zone length created material, which held the head
  at about 9 bar after a run-out.
- **Study:** adds the SPS-313 coverage statistic. Results are re-run.
- **TIA V18:** recompiled with 0 errors and 0 warnings. The new screenshot is
  `docs/img/tia_FB_FillEKF.png`.
- **Tests:** a run-out regression test and an empty-screw mass test.

## 0.1.0 (10 Nov 2026)

- Extruder model:
  - partially filled screw with a moving fill front;
  - batch viscosity travels with the compound;
  - compressible head;
  - gear-pump or power-law-die outlet;
  - drive and sensors.
- Technician identification test (±0.3 rpm, 2 min) with an output-error fit.
- Controllers:
  - PID with back-calculation anti-windup;
  - RST pole placement with observer anti-windup;
  - DMC-type MPC.
- Robust tuning of PID and RST on 10 plant variants under the same constraints.
- Screw-fill EKF with feedforward speed; ridge and MLP estimators for comparison.
- SCL blocks:
  - FB_PressurePID, FB_PressureRST (float32-safe structural form);
  - FB_FillEKF (sparse covariance update);
  - FB_PressureSupervisor (modes, plausibility, fallbacks);
  - OB30 + DBs + generated FC_LoadParams.
- TIA Portal V18 compile through Openness: 0 errors, 0 warnings, with OB30 at 100 ms.
- Study (24 Monte Carlo scenarios), trade study, systems-engineering dossier (Markdown and PDF).
