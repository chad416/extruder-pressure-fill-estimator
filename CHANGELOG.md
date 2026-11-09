# Changelog

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
