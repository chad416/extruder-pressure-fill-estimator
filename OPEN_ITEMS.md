# Open items (state on 10 Nov 2026)

Everything that can be done on this PC without a human is done. What is left needs a person,
a licence, or the plant.

## 1. Needs you

| Item | Why it is blocked | What to do |
|---|---|---|
| **S7-PLCSIM run (SPEC V03, DOSSIER §11.2)** | S7-PLCSIM is not installed for any TIA version on this PC (V16, V18, V20). This was checked in Program Files and the Siemens registry on 10 Nov. Factory IO ships only a driver folder for PLCSIM Advanced. Installing PLCSIM needs a Siemens Industry Online Support login and a licence. | Install S7-PLCSIM V18 (same version as TIA). Then: open `C:\TIA_xtrude\xtrude_20261110_024438\xtrude_20261110_024438.ap18`, download to PLCSIM, set `"LineIO".ModeRequest := 2`, feed a pressure signal (for example a watch table), and record a Trace of `"LineIO".SpeedSp`, `"iRST".Out` and `"iEKF".Mass`. |
| **Publish to GitHub** | Publishing is outward-facing, so it waits for your approval. | Run the commands in §3. |

## 2. Needs the plant (DOSSIER §12, PLAN §5c)

- **Q1–Q8:** outlet type, current PID settings, signals, batch event, compounds, transducer, weighed purges, historian data.
- **On-target timing (VT-07):** the OB30 net execution time from the CPU's cycle statistics. Until then SPS-361 rests on an **estimate**: 1.46 ms per 100 ms cycle from operation counts × datasheet instruction times, 4.39 ms with a 3× margin.
- **Line trial:** VT-01 … VT-08 and VAL-1.

## 3. Publish commands (run in the morning after review)

**Pre-publish check.** Run on 10 Nov for this repo and for `06_tcu-zone-pid-tuning`, covering
the working tree and every commit in history. Result: clean.
- No company, site or contact names.
- No secrets, phone numbers or e-mail addresses. Commits use the GitHub noreply address.
- No build caches, logs or TIA project files tracked.
- The largest file is `docs/DOSSIER.pdf` at 1.4 MB.

```powershell
cd "C:\Users\chand\Downloads\New CLI\SW_PROJECTS\12_extruder-pressure-fill-estimator"
gh repo create chad416/extruder-pressure-fill-estimator --public --source . --remote origin --push --description "Model-based head-pressure control and screw-fill estimation for a cold-feed rubber extruder: physics model, PID/RST/MPC, EKF, Siemens SCL (TIA V18), Wasson-style systems engineering dossier. Simulation only."
gh repo edit chad416/extruder-pressure-fill-estimator --add-topic plc --add-topic siemens --add-topic scl --add-topic tia-portal --add-topic control-systems --add-topic kalman-filter --add-topic systems-engineering --add-topic extrusion
gh run list --repo chad416/extruder-pressure-fill-estimator --limit 1
```

```powershell
cd "C:\Users\chand\Downloads\New CLI\SW_PROJECTS\06_tcu-zone-pid-tuning"
gh repo create chad416/tcu-zone-pid-tuning --public --source . --remote origin --push --description "Identification and robust PI tuning for heating/cooling temperature-control zones, with a split-range SCL controller compiled in TIA Portal V18. Simulation only."
gh repo edit chad416/tcu-zone-pid-tuning --add-topic plc --add-topic siemens --add-topic scl --add-topic pid --add-topic system-identification
gh run list --repo chad416/tcu-zone-pid-tuning --limit 1
```

Both repo names were checked as free on 10 Nov.

**After pushing:**
- check that the CI run is green;
- then the outreach email may carry the link.
