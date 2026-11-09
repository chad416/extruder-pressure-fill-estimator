# Wasson (2016, 2nd ed.), ch. 4–7, 13, 19–22: reading notes with page references

Source: C. S. Wasson, *System Engineering Analysis, Design, and Development*, 2nd ed., Wiley, 2016.
The chapters were extracted from the PDF with PyMuPDF and read in full. Page references are given as **printed page / PDF page**; in these chapters printed = PDF − 30. All text here is paraphrase.

**Where the book is inconsistent**
- **Verification methods:** ch. 13 lists Inspection, Examination, Analysis, Demonstration and Test, plus Validation of Records (p.280–281). Ch. 19/22 list IADT plus Similarity.
- **SMART-VT:** Principle 22.3 (p.448) gives the A as "Actionable"; Fig 22.1 gives it as "Achievable". These notes cite P22.3.
- **TBR:** does not appear in these chapters. Only TBD and TBS (To Be Supplied) do.

## Ch 4: Roles, missions, applications (p.76–98 / PDF 106–128)
- Problem Space vs Solution Space, and the problem-statement definition (p.77/107).
- P4.5 System I/O fitness-for-use (p.81/111).
- Table 4.2 stakeholder roles: User, End User, Owner, Maintainer, Developer, independent test agency (ITA) (p.84–85).
- P4.10 Problem/Opportunity/Solution spaces (p.88/118). The worst case is a perfect specification for the wrong problem.
- P4.13 problem vs symptom (p.90/120).
- P4.15 problem statement: no cause, no blame, no solution (p.91/121).
- Problem-statement guidelines (p.92/122): one sentence, no root cause, the operating conditions, no solution, no blame.
- Initial operational capability (IOC), then full operational capability (FOC), by incremental builds (p.93).
- Leverage existing assets first. New development only through an analysis of alternatives (AoA) (p.96/126).

## Ch 5: User needs, mission analysis, use cases (p.99–128 / PDF 129–158)
- Definitions (p.100–101/130–131):
  - key performance parameter (KPP)
  - measure of effectiveness (MOE): outcome-level
  - measure of performance (MOP): specification-level
  - measure of suitability (MOS): supportability
  - use case (UC) and UC scenario, including misuse
- Five Whys (p.104–105). Caution 5.2: needs vs requirements (p.105/135).
- Mission Analysis Methodology, 10 steps (p.107/137).
- P5.5 mission reliability (p.109/139): at least one performance objective, one reliability objective and one constraint objective.
- P5.7 phases: Pre-Mission, Mission, Post-Mission (p.110/140).
- P5.10 MOE and P5.11 MOS (p.115/145). Fig 5.8: MOE with its contributing MOPs (p.116/146).
- UC attributes 1–13 (p.119/149).
- UC principles P5.12–5.23 (p.120–122/150–152): outcome, actors, preconditions, trigger, main success scenario, alternate scenarios.

## Ch 6: ConOps (p.129–146 / PDF 159–176)
- ConOps definition (p.129/159).
- P6.1 System Operations Model (p.131/161) and Fig 6.1 (p.132/162).
- Caution 6.1: most downstream problems trace back to having no agreed operations concept (p.135).
- Author's Note 6.1: untrained writers specify only "Conduct Mission" (p.137/167).
- Fig 6.3 coverage matrix of operations × phases (p.137–138).
- **Table 6.1 ConOps outline** (p.140/170):
  - Introduction
  - References
  - Missions
  - System Concepts
  - Architecture
  - Operations Model
- **Table 6.4 Operations Concept UCs** (p.143/173):
  - Pre-Mission: Power Up, Initialize, Configure, Calibrate
  - Mission: Normal, Degraded and Emergency Ops
  - Post-Mission: Safe, Analyze, Report

## Ch 7: Phases, modes, states (p.147–173 / PDF 177–203)
- Definitions (p.148–149/178–179):
  - Mode: a user-selectable label that enables a set of capabilities.
  - State: an observable, measurable attribute. Four kinds: system, operational, configuration, dynamic.
- P7.4 Modes (p.157/187), with mode-diagram rules 1–3.
- Table 7.1 state-transition table (current state / event / trigger / next state) (p.158/188).
- Author's Note 7.3: the current mode's exit criteria are the next mode's preconditions.
- P7.5 triggering events (p.159/189).
- Author's Note 7.4: define exit criteria, not both entry and exit criteria.
- Table 7.3 / Fig 7.8 generalised modes (p.162/192):
  - Pre-Mission: Start-Up, Configure, Calibrate
  - Mission: Normal, Degraded
  - Post-Mission: Safe, Analysis, Maintenance
- P7.9 dynamic states, watchdogs, exception handling (p.166–168/196–198).
- P7.10 allowable/prohibited actions (p.169/199). Fig 7.14 mode × UC matrix (p.171/201).
- Constraints go into specification §3.6, with caution/warning thresholds and an FMEA (p.172/202).

## Ch 13: V&V strategy (p.270–292 / PDF 300–322)
- Verification method definitions (p.270–272/300–302): Analysis, Demonstration, Inspection, Test; also Similarity and Validation of Records.
- Table 13.1 cost-to-correct by phase (p.274/304).
- P13.3 Verification: compliance with the specified requirements (p.275/305).
- P13.4 Validation: the right system for the operational need (p.275).
- Verification does not prove the need is met (p.279/309).
- P13.8 V&V strategy (p.278).
- P13.10 methods (p.280).
- P13.11 use the fewest and cheapest methods that suffice (p.283/313).
- P13.12 compliance testing: one test collects data for many requirements (p.283).
- P13.13 validation by field trials, with acceptance criteria set beforehand (p.284/314).
- P13.15 validation independence (p.289/319).
- P13.16 unapproved changes after verification invalidate it (p.289/319).

## Ch 19: Specification concepts (p.397–414 / PDF 427–444)
- Seven specification objectives (p.399/429): what, how well, when, interfaces, environment, constraints, verification.
- P19.6 solution independence: state what and how well, not how (p.401/431).
- P19.9 single interpretation (p.401).
- P19.10 coverage: no TBD/TBS, and a verification method for every requirement (p.401).
- P19.13 traceability (p.402).
- P19.14 MOP values trace to a baselined analysis (p.402/432).
- Fig 19.5 operating conditions: Normal, Abnormal, Emergency, Catastrophic (p.408/438).
- Threshold vs Objective requirements (p.410/440).
- P19.22 condition-based requirements (p.411/441): Abnormal covers external failures, degraded operation and internal failures.

## Ch 20: Specification approaches (p.415–428 / PDF 445–458)
- **Table 20.1 specification outline** (p.417–418/447–448):
  - 1 Introduction
  - 2 References
  - 3.1 Missions
  - 3.2 Operational performance (MOEs, RMA)
  - 3.3 Capabilities
  - 3.4 Interfaces
  - 3.5 Environment
  - 3.6 Design constraints
  - 4 Qualification provisions
  - 6 Notes/assumptions
- P20.1: one MOP per requirement, stated as parameter + magnitude + unit (p.418/448).
- Fig 20.1 MOE/MOS/MOP/TPM (p.418).
- Put RMA in §3.2 (p.419/449).
- Specify external interfaces only (p.420/450).
- P20.5 point-design avoidance (p.424/454).
- Filling a TBD with a quick guess is not specification development (p.426/456).

## Ch 21: Derivation, allocation, traceability (p.429–445 / PDF 459–475)
- Children state how; the parent states why (p.430).
- Decision-support analyses set MOP values (p.432–433).
- Fig 21.5 over-/under-specification (p.435/465).
- Lineage numbering, e.g. R3 → R31 (p.435).
- P21.2 allocate down to a single architecture entity (p.437/467).
- Each MOE should have two or more MOPs (p.438/468).
- P21.3 vertical and horizontal traceability (p.439).
- P21.4 technical performance measures (TPMs) (p.442–445/472–475):
  - prediction → current → actual, with ±3σ limits
  - choose the hard TPMs linked to KPPs

## Ch 22: Requirement statements (p.446–464 / PDF 476–494)
- Fig 22.1 decision process (p.447/477): UC outcome, capability, how well, conditions, verification.
- P22.3 SMART-VT (p.448/478).
- P22.4 choose the least-cost verification method (p.448).
- P22.5 requirement structure (p.449): actor + shall + verb + outcome + performance level + condition.
- Caution 22.1 shall/will/goal (p.451/481).
- P22.9 one capability, one outcome (p.451).
- P22.10 accuracy and tolerance (p.452/482). P22.11 response time (p.452).
- P22.15 unacceptable outcomes (p.453/483). P22.16 conditional actions (p.453).
- Fig 22.2 method-selection chain (p.455/485).
- Requirements verification traceability matrix (RVTM) columns (p.457/487); Table 22.3 (p.458).
- P22.20 no compound requirements (p.460/490). P22.21 no "all/etc./and/or" (p.461/491).
- Table 22.4: 25 validation criteria (p.463/493).
- P22.22 official requirement (p.462/492).
