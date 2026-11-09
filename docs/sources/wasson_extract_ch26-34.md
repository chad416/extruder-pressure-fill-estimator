# Wasson (2016, 2nd ed.), ch. 26–28, 31–34: reading notes with page references

Page references are given as printed page / PDF page; in these chapters the printed number is the PDF number − 30. The notes are paraphrased.

## Ch 26: Architecture (p.542–574 / PDF 572–604)
- **Architecting vs engineering vs designing** are separate, iterative activities (Fig 26.1, p.546/576).
- **Minimum architecture description** (p.547/577):
  - a conceptual view
  - Level 0 and Level 1 block diagrams
  - an N² diagram
  - a text description of how the architecture serves the use cases
- **P26.1** stakeholder concerns and views (p.548/578).
- **Four views** (P26.6 and Fig 26.4, p.551–552/581–582): Requirements, Operations, Behavioral and Physical. Wasson does not use the term "logical architecture"; the nearest equivalent is the Behavioral view.
- **P26.7** validity: the architecture must be configurable in every phase, mode and state (p.550–553).
- **P26.12** a quantity that is not measured cannot be controlled (p.554/584).
- **P26.15** monitor, command and control (MC2) is a capability (p.559/589). Real-time closed-loop control must not wait on display tasks.
- **Faults:**
  - **P26.16** detect, isolate, neutralise and contain faults (p.559–560/589–590).
  - A failure means not meeting the specification (p.560).
  - **P26.17** fault tolerance does not replace removing latent defects (p.560).
  - Fault matrix, Fig 26.8 (p.561/591).
- **Redundancy** (p.565–567/595–597):
  - Types: hot, cold and k-of-n; like and unlike.
  - A single shared connection cancels the redundancy (Fig 26.11).

## Ch 27: Interfaces (p.575–598 / PDF 605–628)
- **P27.1** interfaces have transfer functions (p.575/605).
- **P27.3** an interface is owned by the owner of the highest-level architecture that shows it (p.577/607).
- **Interface documents:** IRS / ICD / IDD (p.578/608). A small project uses one system ICD (p.582).
- **IRS outline** under SPS §3.4 covers operational, behavioral and physical characteristics (p.580/610).
- **12-step interface definition methodology** (p.583/613). Step 7 is timing requirements.
- **P27.6** each interface gets a unique source→destination mnemonic (p.583).
- **Integrity:** a 2–3× safety factor, plus frame and error checking (p.595). Grounding and EMI are also on p.595.
- **P27.9** the FMEA includes effects on adjacent entities (p.596). **P27.10** fault containment at interfaces (p.596).
- **P27.11/27.12** minimise the number of interfaces (p.597/627).

## Ch 28: System integration, test and evaluation (SITE) (p.599–622 / PDF 629–652)
- **Definitions** (p.599–601/629–631):
  - a test case (TC) covers both acceptable and unacceptable inputs
  - regression testing
  - test discrepancy
- **Objective:** test with acceptable and unacceptable inputs, including out-of-range values (p.602/632).
- **P28.1** incremental verification (p.605/635).
- **Simulate / stimulate / emulate** the environment (p.607–608/637–638). This is the basis for running a plant model in the loop.
- **P28.3** one test can verify several requirements. **P28.4** designers do not formally test their own work (p.608/638).
- **Regression testing** (p.609/639).
- **Discrepancy reports:** priority classes 1–4 (Table 28.1, p.610/640).
- **Plans:** TEMP and SIVP (p.610).
- **TC derivation:** phase → mode → use case (UC) → scenario → TC (Table 28.2, p.614/644). The ATP step table is Table 28.3.
- **P28.9** calibrated test equipment (p.618).
- **P28.10** plan test points in advance (p.619).
- **Heuristic 28.1** SITE needs at least 40 % of the schedule (p.619).

## Ch 31: Performance budgets and margins (p.666–681 / PDF 696–711)
- **Definitions:** design-to MOP, safety margin, latency (p.667/697).
- **P31.1** every design needs budgets plus margins. **P31.2** MOP risk thresholds (p.667).
- **Allocation methods:**
  - **P31.3** equivalence (p.668/698).
  - **P31.4** apportionment, used for timing (p.668).
  - **P31.5** synthesis by models or simulation (p.669/699).
- **Margins:** typically 2–3× (p.669).
- **Worked allocation:** a 100-unit limit becomes a 90-unit design-to MOP plus a 10-unit margin (Fig 31.1, p.670/700). Keep the reserve at a high level instead of stacking margins at every level.
- **Cycle-time convention** (Fig 31.3–31.4, p.673/703).
- **P31.6** processing time = queue + processing + transport (p.674/704).
- **Task timing:** t_alloc = t_exec + t_margin, and the +3σ late finish must stay ≤ the budget (Fig 31.7, p.674–676).
- **Series tasks:** variances add; Monte Carlo is suggested (p.675–678).
- **§31.4 real-time and frame-based systems:** allocate a slice of each frame to each task (p.679/709).
- **P31.7** optimise only after the defects are removed (p.679).

## Ch 32: Trade study / analysis of alternatives (AoA) (p.682–702 / PDF 712–732)
- **Definitions** (p.682–683/712–713): decision factor, criteria, figure of merit (FOM), sensitivity analysis (e.g. a 10 % weight cut), utility function.
- **P32.1** analysis vs trade study (p.683).
- **Trade space:** screen out candidates first (Fig 32.4, p.687/717).
- **12-step method** (p.689–690/719–720):
  - problem
  - stakeholders
  - factors and weights
  - criteria
  - utility functions
  - 2–6 candidates
  - scoring
  - sensitivity
  - rank
  - Trade Study Report (TSR)
  - peer review
- **TSR outline:** Table 32.1 (p.691/721).
- **Normalised 100-point scoring** (Fig 32.8, p.691–694). A higher cost or risk scores lower (p.693). Check the arithmetic (p.694).
- **Utility curves:** Fig 32.9 (p.695/725).
- **Sensitivity check** (§32.7, p.696/726): perturb the weights by about 10 % and see whether the winner changes.
- **P32.5** "An undocumented trade study is nothing more than personal opinion." (p.697/727)
- **P32.10** a study is only as valid as its assumptions and data (p.699–700/729–730).

## Ch 33: Modeling and simulation (p.703–720 / PDF 733–750)
- **Definitions** (p.704–705/734–735):
  - accreditation: acceptable for a specific purpose
  - fidelity
  - Monte Carlo
  - validation: the degree to which a model represents the real world for its intended use
- Ch 33 gives no separate definition of model verification.
- **Reuse** existing models first (§33.3.3, p.706).
- **P33.1** state the fidelity explicitly (p.706/736).
- **Validation process:** empirical data, then compare, then calibrate, then configuration management (p.706–707).
- **Fidelity matrix:** the minimum order of approximation needed (p.708/738).
- **Monte Carlo performance allocation** (Ex 33.8, p.709–711).
- **Test-bed evolution:** simulated parts are replaced by real ones (Fig 33.6, p.715).
- **P33.4** a simulation is only as valid as its validation against field data (p.716/746).
- **P33.5** all models are wrong, some are useful (p.718/748).
- **M&S pitfalls:** p.718–719.

## Ch 34: Reliability, maintainability, availability (RMA) (p.721–790 / PDF 751–820)
- **P34.1** reliability is conditional: a mission duration and an environment (p.725/755).
- **Failure** means out-of-specification performance. Drift counts as a failure (p.728).
- **P34.3** define what counts as a failure, with the stakeholders (p.729/759).
- **P34.11** MTBF for repairable items, MTTF for non-repairable ones (p.735/765). Eq 34.4: MTBF = MTTF + MTTR. Eq 34.5: λ = 1/MTBF (p.736/766).
- **P34.12** RMA figures are estimates (p.740/770). Each estimate must state five criteria (p.741/771).
- **P34.29** robustness through fault detection, tolerance and containment, informed by the FMEA (p.761–762/791–792).
- **FMEA:**
  - failure effects are local, next level up, and end effect (p.762)
  - method: Fig 34.17 (p.763/793)
  - Eq 34.40 RPN = S×O (from MIL-STD-882E); Eq 34.41 RPN = S×O×D (after Dhillon) (p.765/795)
  - worksheet columns, Fig 34.20 (p.766/796): ID, item, failure mode, local effect, next-level effect, end effect, detection method, compensating provision, severity, remarks
  - the FMEA must say how each failure is detected and localised (p.767)
- **Mean downtime:** MDT = MAMT + LDT + ADT (Eq 34.42, p.774/804).
- **Availability:**
  - Ao = MTBM/(MTBM+MDT), Eq 34.51
  - Aa = MTBM/(MTBM+MAMT), Eq 34.52
  - Ai = MTBF/(MTBF+MCMT), Eq 34.53 (p.780–781/810–811)
- **Condition-based maintenance and the P-F curve** (Fig 34.25, p.787/817).
