# Changelog

## 0.4.0 (23 Nov 2026)

- **Second plant case:** `xtrude.tsrd` models a batch-fed twin-screw unit with a roller die
  (hopper, conveying screws, chamber, roller-die nip), the machine a public thesis describes
  for this control problem [T1]. Structural model, every parameter assumed.
- **Start-up, starvation and remaining material:** `xtrude.tsrd_sim` and
  `scripts/tsrd_study.py` write `results/tsrd.md`, `results/tsrd_startup.csv` and three figures.
  - A recipe table of two numbers from one earlier log (steady speed, revolutions to full)
    takes the start-up peak from 25.6–27.3 to 20.8–21.3 bar and the settling from 12–13 to 3–4 s.
  - Gain scheduling from an ARX fit of the same log did not help and made the stiffest recipe worse.
  - With a late mixer the plain PI surges (worst peaks 56–96 bar). A hold while the screws are
    short, with a feedforward from a material balance, gives 30–45 bar.
  - The material balance drifts with the throughput calibration (−31 kg after eight batches
    at 2 %); the screw register does not (3.9 kg).
- **Dossier revision D:** summary item 8, §8.5, questions Q6 and Q7, decisions D14 and D15.
- **Tests:** nine new tests in `tests/test_tsrd.py`.

## 0.3.0 (23 Nov 2026)

- **Transducer zero-drift study (FMEA row F3):** `scripts/drift_study.py` writes
  `results/drift.md`, `results/drift.csv` and two figures.
  - The loop holds the reading, so a +8 bar offset costs 1.3 % of throughput at a 60 bar
    setpoint and 5.3 % at 40 bar.
  - The EKF pressure innovation does not respond to offsets of ±8 bar or to a 32 bar/h ramp.
    The dossier's revision-B claim that it would is withdrawn.
  - The offset biases the mass estimate by up to 0.44 kg (40 bar setpoint, +8 bar).
- **Run-out zero check:** `xtrude.zero_check.RunoutZeroCheck` reads the offset when the screw
  is empty and the head has drained (within 0.10 bar in 48 simulated run-outs). Python only.
- **Simulator:** the plant has a transducer zero offset (`p_zero_bar`), and scenarios accept
  `pz` (offset) and `pz_rate` (drift) events. With no offset the results are unchanged.
- **Estimator:** `FillEKF` exposes its last innovation and innovation σ. The estimate itself
  is unchanged.
- **Dossier revision C:** summary item 6, the F3 row and its score, question Q5, decision D13.
- **Tests:** five new tests in `tests/test_drift.py`.

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
- **Documentation:**
  - run-out error bars (`docs/img/runout_errors.png`);
  - `TRACEABILITY.md`;
  - `OPEN_ITEMS.md` with the publish commands;
  - the dossier decision log (§14).

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
