# BeltTwin — Tests & Measurements

System: TwinCAT 3 PLC (10 ms task) → OPC UA (TF6100) → Node-RED (4 Hz) → MQTT → Unity twin, plus CSV logging, scenario runner and live RF detector. All measurements were taken on a single PC (Windows, TwinCAT 4026.24, Node-RED 5.0.4, Unity 2022.3.61f1).

**How to read this document.**
- **Part A** is the current system. A1–A9 describe sensor model v2 (24 Sep 2026), which the Phase 1 dataset and ML results are based on. **A10–A11 describe sensor model v3 and the v3 runner (29 Sep 2026), which Phase 2 is based on.** A5 (latency) is current for both.
- **Part B** is the history: measurements on sensor model v1 (17–22 Sep). They're kept for traceability, and each one is marked as still valid or superseded.
- "TODO" means not yet measured or tested. Don't quote those items anywhere.

---

# Part A — Current system (sensor model v2: A1–A9; sensor model v3: A10–A14; Simscape: A15)

## A1. Sensor model v2 (PLC change, 24 Sep)

These changes invalidate every earlier noise band and detection lead time. Trip times are unaffected.

| Change | Before | After |
|---|---|---|
| Temperature noise | none (perfectly flat) | Gaussian, σ = 0.1 °C, output only |
| Current noise | two fixed sinusoids | sinusoid ×0.04 + Gaussian σ = 0.04 A |
| Vibration noise | two fixed sinusoids | sinusoid ×0.04 + Gaussian σ = 0.03 |
| Random source | none | xorshift32, seeded from the clock at PLC start |
| Noise phase | REAL, wrapped at 1000 (a step every 10 s) | LREAL, wrapped at 20π (continuous) |
| Thermal filter state | REAL (float stall) | LREAL |
| Settled temperatures at speed 80 | 61.989 / 35.977 °C (stalled) | 62.0 / 36.0 °C (the model targets) |
| Current when not Running | half-wave-rectified noise | smooth decay to 0 A |
| Bearing wear | reset on Start/Reset | persists until `bMaintenanceCmd` |
| `nFaultCount` | counted manual injections only | counts every trip |
| Speed change in Running | step | ramp, 0.5 per scan |

**The float stall explained** (why the old 35.977 °C wasn't 36.0): between 32 and 64 a REAL (float32) has a spacing of 2⁻¹⁸ ≈ 3.8×10⁻⁶. An update of error × K is lost once it falls below half that spacing. With K = 0.0000833 (bearing), the filter stops 0.023 °C short, and with K = 0.000167 (motor), 0.0114 °C short. Both shortfalls matched the observed values exactly.

**Healthy steady state:** the new noise bands still need re-measuring from campaign healthy data. *(TODO)*

**Persistence and maintenance verified (24 Sep):** the campaign run straight after a wear trip ran 328 s or more of warm-up without re-faulting, so maintenance cleared the wear.

---

## A2. Trip times across speeds

Measured by the scenario runner, as `trip_ts − inject_ts` in the manifest. **These include the runner's 0.4–0.5 s detection lag** (0.5 s tick plus the OPC UA read). The true trip is the first CSV row with `eState` = 3.

| Fault | Predicted | Speed 40 | Speed 60 | Speed 70 | Speed 80 |
|---|---|---|---|---|---|
| Jam | 2.0 s | 2.50 s | 2.53 / 2.50 s | — | — |
| Slip | 26.7 s | 27.1 s | — | — | — |
| Overload | 53.3 s | — | — | 53.6 s | 53.7 s |
| Wear | 400 s | 400.1 s | — | — | — |

**Trip time does not depend on speed,** as the model predicts: severity rates are per scan and independent of speed. The earlier manual measurement for overload (53.95 s at 80) was about 0.5 s high. The runner-measured 53.6–53.7 s is the prediction plus the known lag.

*(Extend this table from the full campaign manifest.)*

---

## A3. Staleness detection: a hole found and fixed (24 Sep)

**Original design:** `cache` set `lastUpdate` every time an OPC UA read *returned*. `build json` stopped publishing if `lastUpdate` was more than 2 s old.

**The hole:** a stopped PLC still answers OPC UA reads with its last values. Found by deliberately stopping the PLC: `build json` stayed green and published minutes-old data at a steady 4 Hz, exactly the silent failure the detector was meant to catch.

**The fix:** `lastUpdate` is set only when `nPlcTime` *changes* (the PLC clock advancing).

**Verified:** PLC stopped → `build json` red "stale" within about 2 s, and no MQTT messages. PLC started → back to publishing. Unity is protected by the same change, because it receives nothing and shows NO DATA.

**It proved itself on 24 Sep, 12:57:** the Node-RED OPC UA session died (reads returned null). `build json` went stale, the scenario runner paused instead of injecting faults into frozen data, and no bad rows were recorded. The root cause of the session death is unknown, because the terminal output wasn't captured.

---

## A4. `nPlcTime` is wall-clock time, not uptime (correction)

`nPlcTime := TIME()` returns **Windows FILETIME milliseconds (since 1601-01-01 UTC) modulo 2³²**.

- **Evidence:** it continued in step with wall-clock time across a restart. Its change over 46 min (+2,774,822 ms) matched the change in `ts` (+2,774,993 ms). Converting `readTs` gave 4,264,670,083 against `nPlcTime` 4,264,670,052, a 31 ms difference, which is read latency.
- **Wrap prediction verified:** predicted for 23 Sep 19:21:41 CEST; back-calculated from later data as 19:21:40. The next wrap is around 12 Nov 2026.
- **Consequence:** `nPlcTime` is a usable PLC-side timestamp, which is what makes A5 possible.

---

## A5. End-to-end latency, PLC scan to subscriber

Measured with `latency_probe.py`, 240 samples at 4 Hz each. The PLC timestamp comes from `nPlcTime` (see A4).

**Definition:** this is the latency of a *sampled value*, from the PLC scan that produced it to the MQTT subscriber. An event in the PLC additionally waits 0–250 ms for the next 4 Hz poll. Don't quote these numbers as "fault to display".

### Before: separate poll and publish timers (25 Sep)

| Hop | min | median | p95 | max (ms) |
|---|---|---|---|---|
| PLC scan → OPC UA read complete | 36 | 57 | 86 | 180 |
| Read → Node-RED publish | 11 | 155 | 177 | 184 |
| Publish → MQTT subscriber | 2 | 4 | 5 | 16 |
| **Total: PLC scan → subscriber** | **167** | **214** | **226** | **229** |

**Finding: 72 % of the latency was the Node-RED design, not the protocols.** Poll and publish were two independent 4 Hz timers, so a fresh value waited in the cache for the next publish tick.

### After: publish on arrival (29 Sep)

**The fix:** `fan-out` opens a batch on every poll. `cache` emits a trigger once all 26 variables of the batch have arrived, and `build json` publishes immediately (only if `nPlcTime` changed). The old 4 Hz publish timer became a 1 Hz watchdog that only updates the stale status.

| Hop | Run 1 (idle) min / median / p95 / max | Run 2 (during campaign v3) min / median / p95 / max |
|---|---|---|
| PLC scan → OPC UA read complete | 13 / 28 / 43 / 70 | 19 / 29 / 43 / 81 |
| Read → Node-RED publish | 1 / 2 / 5 / 8 | 1 / 2 / 4 / 22 |
| Publish → MQTT subscriber | 1 / 2 / 3 / 14 | 1 / 2 / 3 / 3 |
| **Total: PLC scan → subscriber** | **16 / 32 / 48 / 75** | **22 / 32 / 47 / 86** |

**Result: 214 → 32 ms median, 226 → 47–48 ms p95,** reproduced under the runner's campaign load. The predicted 60–90 ms was beaten because the first hop also fell, from 57 to 28–29 ms median. **That drop is not explained.** Less contention in Node-RED without the second timer is plausible but unverified; don't state it as the cause.

**Unity (25 Sep, before the fix):** MQTT hop 8 / p95 18 ms, and data age (OPC UA read → display) 174 / p95 192 ms, both over 240 samples.

**Unity (2–3 Oct, after the fix),** from the HMI's own rolling 240-sample display, read off four screenshots: MQTT hop 8–12 / p95 15–21 ms, data age **11–14 / p95 20–27 ms**. Data age fell by about 160 ms, consistent with the 155 ms read → publish hop removed in Node-RED. (Display readings, not a logged measurement.)

**Caveats:** `nPlcTime` resolution is 10 ms, and the method assumes the TwinCAT and Windows clocks are aligned. There were no negative ages in any run, so any offset is small but not proven to be zero.

**This supersedes the earlier 197 / 221 ms figure,** which covered only OPC UA read → Unity and excluded the PLC side.

---

## A6. Unity MQTT auto-reconnect: verified (25 Sep)

With Unity playing, `Restart-Service mosquitto` produced NO DATA and a grey belt, then automatic recovery with `reconnects 1`, rate back at 4.0 Hz, and the message count continuing. Node-RED's MQTT node and the live detector also reconnected by themselves. The detector correctly discarded its buffer across the data gap and rebuilt 30 s of history ("warming up").

---

## A7. Scenario runner (data collection)

- **Two dry runs (23–24 Sep):** jam at 60, and healthy with a speed change from 60 to 40. Manifest correct, labels switching at injection, row counts consistent (10 developing rows = 2.5 s, 42 faulted rows = 10.5 s).
- **Settle check:** the first version (0.5 °C on single samples) let the bearing sit 0.2–0.4 °C below target at injection. Fixed with a 10 s rolling mean within 0.15 °C. Residual drift at injection is about 0.12 °C, at about 0.001 °C/s, which is below the sensor noise.
- **Warm-up duration:** 233–498 s. That's about three-quarters of the roughly 10 minutes per run.
- **Resume:** the logic that subtracts `ok` runs with 0 stale ticks from the plan was unit-tested against a sample manifest (100 − 4 = 96, with failed and stale runs correctly re-planned). **Not yet exercised live.**

---

## A8. Random Forest classifier: results (26–27 Sep)

**Dataset:** 116 `ok` runs from the campaign: jam 22, slip 23, overload 25, wear 22, and healthy 24 (11 steady, 13 with a mid-run speed change). That's 75,554 labelled samples, at speeds 40–80.

**Known issue: lost inject writes.** 4 runs (one per fault type) ended `no_trip_timeout`. For the jam run, current during `developing` peaked at 5.58 A, the healthy level, so the inject command never reached the PLC. That's about 3 % of OPC UA inject writes lost without an error. The runner's timeouts caught every case, and those runs are excluded. Speed writes are verified by read-back; inject and maintenance writes are not. *(Fix: a PLC injection counter so the runner can verify each inject.)*

**Method:**
- 26 causal features (later 31) from current, both temperatures, vibration, belt ratio and speed: 2 s mean, std and slope, 30 s slope, and the short-minus-long delta.
- Never used as features: `eState`, `eFaultCode`, counters, `phase` or `label`.
- The trip is relabelled from the first `eState` = 3 row.
- 5-fold cross-validation grouped by run. A detection counts after 3 consecutive identical predictions.

**Results, before and after adding speed-context features:**

| | 26 features | + speed context (31) |
|---|---|---|
| Faults detected before the PLC trip | 92 / 92 | **92 / 92** |
| Median delay after injection: jam / slip / overload / wear | 0.41 / 1.29 / 1.91 / 5.91 s | **0.41 / 1.37 / 1.91 / 5.91 s** |
| Worst-case delay | 0.56 / 1.67 / 2.40 / 7.61 s | 0.56 / 1.82 / 2.40 / 7.61 s |
| Runs with a false alarm in a healthy segment | 7 / 116 | **1 / 116** |
| of which speed-change runs | **6 / 13** | **0 / 13** |
| Jam precision / recall (sample level) | 0.84 / 0.87 | 0.99 / 0.87 |

**Finding:** the first model mistook commanded speed changes for faults. Current, vibration and temperatures all shift with speed, and the top feature (`rMotorCurrent_delta_sl`) spiked. Giving the model speed-trend features, which are legitimate operating context because the setpoint is commanded, removed those false alarms without slowing detection. **Caveat:** this fix was chosen after seeing the failures in the same cross-validation, so the "after" column is optimistic. *(TODO: confirm on about 10 fresh speed-change runs never used for training.)*

**Lead time over the PLC's own trip** (PLC trip time minus median detection delay): slip about 25 s, overload about 52 s, wear about 394 s. Jam gains only about 1.6 s, because it trips in 2 s. That's the sampling-rate limit (B6).

**Limitations. State these whenever the numbers are quoted:**
1. **Simulated data.** Vibration responds to wear linearly with no lag, and the noise is clean and Gaussian. Real bearing wear is not detectable in 6 s. The numbers mostly measure how separable the simulated faults are.
2. **`rPosition` is noise-free,** so `belt_ratio` is a perfect encoder. That makes slip and jam unrealistically easy.
3. **Validated at thermal steady state only.** Warm-up data was never in training. *(Next: add the warm-up phases, which are all healthy and already in the CSVs, and re-evaluate.)*
4. **The manual fault has no sensor signature** and is not a class.

## A9. Live detector

**Train/serve parity:** the live feature code (numpy, last sample only) against the training code (pandas, whole run) gave **0 prediction mismatches in 1,600 samples**, with a maximum relative feature difference of 3.6 × 10⁻⁷ (float rounding).

**Incident, 27 Sep, found during the demo:** a slip injected at 11:33:04 was *recognised* in data from 11:33:06.7, 2.7 s after injection, as in cross-validation. But the event was only emitted at **11:33:31, 24.6 s late**, after the PLC had tripped. **Cause:** each sample rebuilt a pandas DataFrame over the whole buffer, which took more than the 250 ms budget on the target PC, so a backlog grew continuously. The stalled network loop also missed MQTT keepalives, so the broker dropped the detector every 2.5–3.5 min, and each reconnect cleared its 30 s buffer.

**Fix:**
- Features for the newest sample only, in numpy: 0.25 ms instead of 15 ms (measured on the dev machine).
- A catch-up guard: samples that arrive more than 1 s late are buffered but not predicted.
- MQTT keepalive raised from 30 to 60 s.
- A health line every minute.

**After the fix:** handle time 76 ms average, 213 ms maximum (budget 250 ms). Lag 147 ms. Event delay 152 ms. No reconnects.

**Live demo (27 Sep, 11:58):** at speed 80, at thermal steady state, slip was injected from Unity. The RF banner went purple, then showed **"RF called SLIP 25.3 s before the PLC tripped"**, which means detection about 1.4 s after injection, in line with cross-validation.

**Cold-start miss (27 Sep, 11:18):** a jam injected 21 s after START was not detected. That's expected: the detector needs 30 s of Running data before it predicts at all, and a cold machine is outside the training distribution (see A8, limitation 3).

**MQTT last will (`detector_offline`):** *(TODO: close the detector window with its X, not Ctrl+C, and confirm Unity shows "RF: detector offline".)*

---

## A10. Sensor model v3 (PLC change, 29 Sep)

Purpose: realism before Phase 2 (residual-based detection). The healthy plant (gains, thermal time constants, fault signatures at trip) is unchanged from v2. Commit `5648c42`; v2 is tagged `sensor-v2-final`.

| Element | v3 | Reason |
|---|---|---|
| Random source | Box-Muller on xorshift32, tails to about 6.6σ | v2 summed 4 uniforms, which clips at ±3.46σ and would fake zero false alarms for any threshold above that |
| Load disturbance (hidden) | Two OU processes, σ 0.04 each, τ 20 s and 600 s, clamped ±0.25. Same current (6 A) and motor-temperature (35 °C) coefficients as overload; bearing +5 °C, vibration +1.2 per unit | Overload is extra load, so an unmeasured load is the realistic confounder. Expected current wander about ±0.34 A against +4.8 A at trip |
| Belt creep | 1 % + 0.10 × load | A healthy belt never runs at drum speed; removes the perfect belt ratio |
| Encoder | `rPosition` quantised to 0.25 units | Belt-ratio noise about 1.0 % per sample at speed 40, 0.5 % at 80 |
| Ambient | True: OU around 22 °C, σ 1.0 °C, τ 1 h. Sensor: σ 0.05 °C, 0.1 °C resolution | Thermal models follow real ambient, so the behaviour model must use `rAmbientTemp` |
| Temperatures | σ 0.08 °C, 0.1 °C resolution | EL3202-style resolution |
| Current | σ 0.05 A, 0.01 A resolution | Load disturbance dominates |
| Vibration | σ = 4 % of level + 0.02, 0.01 resolution | RMS noise scales with level |
| Spikes | Current ±0.5–1.5 A, vibration +0.5–2.0, probability 10⁻⁴ per scan while running (about 1.4 per sensor per hour of samples) | Separates single-sample thresholds from persistence-based detectors |
| Fault onset | Time to trip = nominal × log-uniform [0.5, 2]. Jam linear (1–4 s). Slip (12.5–50 s), overload (25–100 s) and wear (200–800 s) quadratic: the rate starts at 0 | Progressive faults start slowly; trip time is no longer a learnable constant. No random dead time, on purpose (labels start at injection) |
| Wear → vibration | 5.625 × wear² (3.6 at trip, as in v2) | Early wear is invisible in RMS vibration (P-F curve) |
| `nInjectCount` | New OPC UA variable, +1 on every inject edge | Runner verifies every inject (A8 known issue) |
| Precision | All severities, rates and filters LREAL | Quadratic onset starts at about 10⁻¹⁰ per scan; REAL would stall |

**Not modelled, on purpose:** noise on `rSpeed` (it is the behaviour model's exogenous input), sensor dropouts, measuring-wheel eccentricity.

**The PLC trip is an oracle.** It fires on hidden severity ≥ 0.8, so in Phase 2 it is the functional-failure event (ground truth), not a detector. The fixed-threshold baseline is computed offline on measured signals only.

**Verified (29 Sep):**
- Build: 0 errors. Boot project activated at 10:05 (`Port_851.app`, `.crc`, `_boot.tizip`).
- UaExpert: **26 variables** under `MAIN`, all Good, `nInjectCount` present; no hidden internals (`rLoad`, `rAmbientTrue`, `aGauss`) visible.
- Jam injected from UaExpert: `nInjectCount` 0 → 1 at 10:19:19.262, `eState` 3 and `eFaultCode` 1 at 10:19:21.262 (2.0 s, inside 1–4 s). `rPosition` 102.5 (a multiple of 0.25). Temperatures on 0.1 °C steps. Current decayed smoothly to 0 after the trip.
- After reset, motor temperature fell from 32.1 to 22 °C within about 20 min, consistent with the 60 s time constant.

**Healthy v3 noise bands:** *(TODO: measure from campaign v3 healthy and soak runs.)*

---

## A11. Scenario runner v3 and dry run (29 Sep)

Commit `0f3a399` (`nodered/scenario_runner.js`, `nodered/flows.json`).

**Changes:**
- Data goes to `data\v3\` (campaign) or `data\v3dry\` (dry run). The v2 manifest is never read, so resume can't mistake v2 runs for v3 runs.
- **Inject verification:** `nInjectCount` must rise by exactly 1 within 2 s; up to 3 tries; a rise of 2 is recorded as `double_inject`.
- **Maintenance verification** through `nMaintenanceCount`, up to 3 tries.
- **Settle is slope-based:** after at least 240 s, the 60 s regression slope must be below 0.01 °C/s (motor) and 0.004 °C/s (bearing), held for 10 ticks. The v2 criterion (10 s mean within 0.15 °C of `ambient + k·speed`) would time out under v3 load and ambient disturbances. Simulated against the v3 model: warm-up 4–9 min, bearing within 0.6 °C of equilibrium at settle.
- Timeouts cover twice the nominal trip time: jam 15, slip 90, overload 150, wear 1000 s.
- Plan: 150 runs = 25 per fault type, 20 healthy, 20 healthy with a speed change, 10 soak runs of 30 min, over speeds 40–80, shuffled. The runner refuses to start if `nInjectCount` isn't in the cache.
- CSV rows include `nInjectCount`, so the exact injection sample can be recovered offline.
- Manifest adds `inject_tries` and `maint_tries`.

**Tested before deployment** against a mock PLC: full dry plan; a lost jam inject (retried, `ok`, 2 tries); a lost maintenance write (retried, `ok`); an inject that never arrives (`inject_write_failed` after 3 tries); a delayed inject that lands twice (`double_inject`).

**Dry run on the real system, 4 / 4 `ok`:**

| Run | Warm-up | Inject → trip | `inject_tries` | `maint_tries` | Stale ticks |
|---|---|---|---|---|---|
| jam @ 80 | 407 s | 3.50 s | 1 | 1 | 0 |
| slip @ 60 | 245 s | 39.06 s | 1 | 1 | 0 |
| overload @ 40 | 315 s | 34.57 s | 1 | 1 | 0 |
| healthy_change 60 → 40 | 309 s | — | — | 1 | 0 |

Trip times include the runner's roughly 0.5 s read lag. Wear was not part of the dry run.

**CSV check (jam run):** phase row counts at 4 Hz match the manifest exactly: warmup 1629 rows (407 s), healthy 149 (37 s, hold 37 s), developing 14 (3.5 s), faulted 42 (10.5 s). `nInjectCount` present (1 → 2).

**Campaign v3 (29–30 Sep): 150 / 150 `ok`, 0 stale ticks,** every run type exactly on target (jam, slip, overload, wear 25 each; healthy 20; healthy_change 20; soak 10). About 25.0 h of data at 4 Hz (360k samples), backed up as `BeltTwin_v3_dataset_2026-09-30.zip` (156 files, 6.98 MB, on Google Drive). A 5-hour internet outage during the campaign had no effect: the pipeline is local only.

- **Inject writes:** 99 / 100 landed on the first try, 1 needed a retry (caught by the verification; run `ok`). **Maintenance writes:** 150 / 150 on the first try. Lost writes still happen occasionally (1 of 250 here, about 3 % in v2; too few events to claim a difference). The cause is unknown; verification makes it harmless.
- **Time to trip (inject → trip, runner-side):** jam 1.5–4.0 s (median 2.0), slip 13.0–48.1 s (24.5), overload 27.6–86.8 s (51.6), wear 201–775 s (379). All inside the v3 design ranges.
- **Warm-up:** 245–501 s (median 289 s).

**Boot-project autostart (30 Sep):** it was **off**, not just unverified as the Phase 1 handoff said. Enabled (commit `ec977c6`) and verified: after Activate Configuration, `nPlcTime` ticked in UaExpert without Login or F5. Before that, UaExpert showed `BadDeviceFailure` after a reboot.

---

## A12. Behaviour model and residuals (Phase 2, step 4, 30 Sep)

Scripts: `ml/make_split.py` (run once), `ml/split_v3.csv` (committed), `ml/behaviour_model.py`. Parameters in `ml/behaviour_v3.json`; residuals per run in `data\v3\residuals\`.

**Pre-registered split (fixed before any model was fitted):** jam, slip, overload 15 train / 10 test each; healthy 12/8; healthy_change 12/8; soak 6/4; **wear 0 / 25** (the unseen fault). Spread evenly over speeds, seed 20260930.

**Protocol:** the model is fitted only on the healthy segments of train runs (everything before the first `nInjectCount` increment), 11.3 h. Train-run residuals are **out-of-fold** (4 folds grouped by run), so detector tuning never sees a residual from a model fitted on the same run. Test runs get residuals from the model fitted on all train runs. All statistics below are train-only, out-of-fold. **Test runs have not been summarised.** (Changed from the original plan to check the model on healthy test runs: that would have spent the test set on model development.)

**Inputs:** `rSpeed`, `rAmbientTemp`, motor on (eState 1 or 2). Nothing fault-affected.

| Output | Model | Identified | v3 PLC truth |
|---|---|---|---|
| Motor temperature | first-order output-error simulation from the run's start | τ 59.7 s, gain 0.515 °C per speed unit, offset −0.95 °C, load gain 5.83 °C/A | τ 59.9 s, 0.5, 0, 35/6 = 5.83 |
| Bearing temperature | same | τ 119.8 s, 0.177, −0.14 °C, 0.831 °C/A | 120.0 s, 0.175, 0, 5/6 = 0.833 |
| Current | same structure, fast lag | τ 0.57 s, 1.84 A + 0.0901 A per speed unit | 0.5 s, 2.0 + 0.0875 |
| Vibration | static, σ = 0.051 + 0.038 × level | 0.170 + 0.0255 per speed unit; load gain 0.199 per A | 0.2 + 0.025; 1.2/6 = 0.2 |
| Belt ratio | 2 s window; load-compensated: 1 − max(0, creep0 + k × load proxy) | 0.990; creep 0.0099 + 0.0168 per A | creep 0.01 + 0.10 × load = 0.01 + 0.0167 per A, clamped at 0 |

**Model order was chosen from the data:** a second-order thermal model gave no improvement in grouped CV (motor 1.578 vs 1.578 °C, bearing 0.2104 vs 0.2106 °C), so first order was kept.

**Two residual families:**
- **Input-only** (`current`, `motor`, `bearing`, `vib`, `ratio`): sensitive to everything that departs from the healthy model, including the unmeasured load.
- **Load-compensated** (`motor_lc`, `bearing_lc`, `vib_lc`, `ratio_lc`): the current residual is used as a load proxy. It explains the healthy temperature wander almost completely (correlation 0.985 between the motor residual and the current-based load estimate). **Overload is extra load, so `_lc` residuals are blind to overload by construction**; the current residual still carries it.

**Residual σ (out-of-fold, healthy, motor on, after 60 s):**

| Channel | σ | Expected floor |
|---|---|---|
| current | 0.367 A | about 0.35 A (noise + load) |
| motor | 1.63 °C | about 1.5 °C (load through 60 s dynamics) |
| bearing | 0.215 °C | about 0.22 °C |
| vib | 0.117 | about 0.13 at speed 80 |
| ratio (2 s) | 0.0068 | about 0.006 (creep + encoder) |
| **motor_lc** | **0.087 °C** | sensor noise floor about 0.085 °C |
| **bearing_lc** | **0.085 °C** | same |
| vib_lc | 0.094 | |
| ratio_lc (2 s) | 0.0036 | |

Load compensation cuts the motor-temperature residual about 19× and the bearing residual 2.5×, to the sensor noise floor. The bearing gain matters for the unseen fault: wear heats the bearing but adds no current.

**Acceptance criteria (stated before fitting):**
1. *Bias below 0.2σ at every speed and warm-up stage:* **met** for all `_lc` channels (largest point estimate 0.14σ; per-run mean residual within ±0.13σ for motor_lc and ±0.07σ for bearing_lc, 5th–95th percentile). For the input-only temperature residuals, every CI includes 0, but point estimates reach 0.38σ in segments after 480 s (soak runs). Cause: the slow load gives each run its own offset (per-run mean residual −1.3σ to +1.2σ, 5th–95th percentile), and there are only 6 train soak runs. The bias of input-only residuals can't be pinned down below about ±0.3σ with this data.
2. *σ within 20 % of the floor:* **met** for all channels.
3. *No warm-up bias:* **met**, after two fixes found by this check (below).

**Two bugs found by the warm-up check, both fixed:**
- **Biased time constant.** Fitting the motor model without the load proxy gave τ = 66 s instead of 60 s. A synthetic check (same inputs, known τ = 60 s, simulated load) gave 56–65 s across 8 load realisations: the unmeasured load confounds identification. Fitting jointly with the current residual as an auxiliary input gives 59.7 s. The input-only residual reuses those dynamics.
- **Initial-state error.** The simulated thermal state was initialised with the mean of the first 8 samples. A motor still cooling from the previous run (about 0.5 °C/s) made that mean about 1 s late and started the simulation about 0.5 °C off, decaying with τ. A line through 8 samples fixed the bias but left about 1σ of random start error, which decayed over minutes and drove healthy CUSUM sums into the hundreds (found in step 5). **Final fix:** the initial state is estimated by least squares over the first 30 s with the load-compensated model, and **thermal residuals are not monitored during those 30 s** (state-estimation guard; faults are never injected that early).

**Sanity check on train fault runs only** (median z over the 2 s before the trip):

| | jam | overload | slip |
|---|---|---|---|
| current | +10.2 | +11.8 | −3.0 |
| vib | +29.1 | +7.7 | +16.3 |
| ratio | −18.8 | +0.3 | −60.8 |
| motor_lc | −2.6 | +1.2 | +9.3 |

The signs match the plant: slip lowers current, so `motor_lc` expects a cooler motor. Overload barely shows in the `_lc` residuals, as designed.

**Belt creep saturates (found in step 5):** a linear load compensation of the belt ratio over-corrected when the load proxy was strongly negative, because creep cannot go below 0 (the belt never outruns the drum). Healthy CUSUM sums on `ratio_lc` reached 106. A saturating creep model fixed it.

**Pipeline finding:** single-sample belt ratios have σ about 2.3–2.8 %, against about 0.6 % expected. Cause: `rPosition` and `nPlcTime` are separate OPC UA reads, so each row is not one PLC scan. Using the PLC clock over a 2 s window brings it to 0.68 %. *Fix (open): read all variables in one OPC UA Read call, or compute belt speed in the PLC.*

**Honest limitation:** I designed the simulator, so the model structure was easy to get right. The order was still selected from the data and every coefficient was identified, not copied. The agreement with the PLC constants is a check on the identification, not a result.

---

## A13. Detectors, tuned on train runs (Phase 2, step 5, 1 Oct)

Script: `ml/detectors.py`. Thresholds and settings in `ml/detectors_v3.json`; models `ml/iforest_v3.joblib`, `ml/rf_v3.joblib`.

**Rules (fixed before any result):** every detector reduces to one statistic and one threshold, tuned on train runs only to the **same budget of 0.5 false alarms per healthy motor-on hour** (11.25 h, so at most 5 events). An alarm event is an upward threshold crossing while the motor is on. Detection is the first crossing between the injection sample (first `nInjectCount` increment) and the trip; delay is measured on the PLC clock, also as a fraction of time to trip. Isolation Forest and RF scores are out-of-fold (same 4 folds as the behaviour model). Wear is not in train. **The test set has not been used.**

| Detector | Design |
|---|---|
| fixed | Static, speed-independent limits on measured signals: current, motor T, bearing T, vibration high, 2 s belt ratio low. Limits at the train-healthy 99.9th percentile × one common factor; 1 s debounce |
| cusum | Two-sided CUSUM, k = 0.5, z clipped to ±4, on `current`, `vib_lc`, `ratio_lc`, `motor_lc`, `bearing_lc`. Channels with healthy lag-1 autocorrelation above 0.2 are pre-whitened with AR(1) (current 0.978, ratio_lc about 0.3) |
| iforest | Isolation Forest on 2 s and 30 s means of 9 residual z channels, trained on healthy samples; 3-sample debounce |
| rf | Random Forest on 31 raw-signal features (Phase 1 design), classes healthy / jam / slip / overload; statistic 1 − P(healthy); 3-sample debounce |

**Train results** (thresholds tuned on the same data, so false-alarm rates are at or below budget by construction):

| Detector | FA/h | jam | overload | slip |
|---|---|---|---|---|
| fixed | 0.44 | 15/15, 1.00 s (0.57) | 12/15, 28.0 s (0.78) | 15/15, 9.8 s (0.33) |
| cusum | 0.36 | 8/15, 2.00 s (0.62) | 15/15, 16.3 s (0.41) | 15/15, 7.0 s (0.27) |
| iforest | 0.44 | 6/15, 2.63 s (0.82) | 15/15, 27.3 s (0.76) | 14/15, 15.8 s (0.55) |
| rf | 0.44 | 14/15, 0.75 s (0.38) | 15/15, 19.7 s (0.44) | 14/15, 7.8 s (0.28) |

Cells: detected / runs, median delay after injection (median fraction of time to trip). Numbers from the reference run on the project PC (scikit-learn version there: RF threshold 0.617; fixed, CUSUM and Isolation Forest identical to the development run).

**Reading (train only, not the result):**
- **CUSUM on load-compensated residuals is the best detector for the progressive faults** (overload and slip), ahead of the supervised RF, without seeing a single fault in training.
- **CUSUM misses half the jams.** A jam trips in 1–4 s, and with z clipped at 4 and k = 0.5 the sum gains at most 3.5 per sample, so reaching 27.8 takes 8 samples (2 s). The clip protects against spikes; the price is slow response to sudden faults. That's a known CUSUM trade-off (a Shewhart limit alongside would fix it, but it wasn't in the pre-stated design).
- **Isolation Forest is the weakest.** It scores only "how unusual", with no direction or persistence, so its threshold is set by healthy outliers.
- **Fixed thresholds** do well on jam and slip but miss 3 overloads: one static current limit across speeds 40–80 must sit above healthy current at speed 80.

**Changes during step 5, all from healthy train diagnostics, none from fault delays:** (1) initial-state estimation over 30 s with a monitoring guard (A12); (2) saturating creep model for `ratio_lc` (A12); (3) AR(1) pre-whitening for CUSUM channels: without it, the load-driven autocorrelation of the current residual (lag-1 0.978) pushed healthy CUSUM sums above 2000.

**Step 6 (test set, single evaluation): see A14.**

---

## A14. Test-set evaluation, single run (Phase 2, step 6, 2 Oct)

Script: `ml/evaluate.py`. Frozen settings from step 5, nothing fitted or tuned. `ml/TEST_EVALUATED.txt` records the fingerprint of the frozen settings; the script refuses a second test evaluation if they change. Per-run results in `ml/results_test_v3.csv`. Before the test run, the script was checked on train runs only: fixed and CUSUM reproduced the step-5 numbers exactly.

**Test set:** 75 runs (wear 25, jam 10, slip 10, overload 10, healthy 8, healthy_change 8, soak 4), **9.94 healthy motor-on hours**.

**False alarms** (budget on train: 0.5 per hour; 95 % Poisson CI):

| Detector | False alarms | Per hour | 95 % CI |
|---|---|---|---|
| fixed | 1 | 0.10 | [0.00, 0.56] |
| cusum | 4 | 0.40 | [0.11, 1.03] |
| iforest | 0 | 0.00 | [0.00, 0.37] |
| rf | 7 | 0.70 | [0.28, 1.45] |

**Detection** (detected / runs, median delay after injection, median fraction of time to trip in brackets; max fraction after the slash):

| Fault | fixed | cusum | iforest | rf |
|---|---|---|---|---|
| jam | 9/10, 1.00 s (0.67 / 0.80) | 2/10, 1.88 s (0.88 / 0.89) | 0/10 | **10/10, 0.62 s (0.50 / 0.60)** |
| slip | 10/10, 8.27 s (0.36 / 0.75) | **10/10, 5.76 s (0.30 / 0.39)** | 9/10, 12.3 s (0.62 / 0.98) | **10/10, 6.39 s (0.28 / 0.44)** |
| overload | 9/10, 54.1 s (0.80 / 0.96) | **10/10, 25.9 s (0.37 / 0.46)** | 10/10, 42.9 s (0.65 / 0.78) | **10/10, 26.3 s (0.38 / 0.45)** |
| **wear (unseen)** | 25/25, 227 s (0.71 / 0.81) | **25/25, 64.7 s (0.17 / 0.22)** | 20/25, 200 s (0.56 / 0.93) | 11/25, 334 s (0.97 / 1.00) |

**Findings:**
1. **On the unseen fault, the residual approach wins clearly.** CUSUM on load-compensated residuals detected all 25 wear runs at a median 17 % of time to trip (at most 22 %). The supervised Random Forest detected only 11 of 25, at a median 97 % of time to trip, essentially at the moment of failure. Fixed limits caught all 25, but at 71 %. This is the one difference in the table that is large compared with the sample sizes.
2. **On the faults it was trained on, the RF is as good as CUSUM or better:** best on jam (10/10, 0.62 s), tied with CUSUM on slip (0.28 vs 0.30 of time to trip) and overload (0.38 vs 0.37). With 10 runs per fault, the slip and overload differences are within noise.
3. **CUSUM misses most jams (2/10)**, as predicted from train (8/15). The sum needs about 8 samples (2 s) to reach the threshold and jams trip in 1–4 s. A known trade-off of the pre-stated design (z clipped at 4 against spikes); not changed after the test.
4. **False alarms stayed near budget for fixed, CUSUM and Isolation Forest.** The RF rose from 0.44 (train, out-of-fold) to 0.70 per hour; the 95 % CI [0.28, 1.45] includes the budget, so this is a weak indication of overrun, not a proven one.
5. **Train and test agree for CUSUM** (false alarms 0.36 → 0.40/h; overload 0.41 → 0.37 of time to trip; slip 0.27 → 0.30), so the tuning did not overfit.
6. **Isolation Forest is the weakest overall** (0/10 jam, 5 wear misses), consistent with train.

**Limitations to state with these numbers:**
- **The data is simulated, and the simulator was written by the same person who designed the detectors.** In the v3 model, wear heats the bearing in proportion to wear from the start (through its 120 s time constant) while vibration only grows with wear². That is what lets an ambient- and load-compensated bearing-temperature residual see wear early. Real bearing degradation has different and less clean signatures; the size of the advantage should not be expected to carry over unchanged.
- **Small samples:** 10 runs per trained fault type, 25 wear runs, 9.94 healthy test hours. False-alarm rates have wide CIs.
- **Matched budget, not matched rate:** all detectors were tuned to the same false-alarm budget on train; on test the realised rates differ (RF highest).
- **The PLC trip is the oracle failure event** (hidden severity ≥ 0.8), not a protection relay.

---

## A15. Simscape plant model: robustness test on independent physics (3–4 Oct)

**Question:** do the Phase 2 results only hold because the plant model (PLC sensor model v3) was written by the same person who designed the detectors? The conveyor was rebuilt from MathWorks library physics (MATLAB R2026b; Simulink, Simscape, Simscape Electrical, Simscape Driveline; university licence) and the same evaluation was repeated with a new pre-registered split.

Scripts in `simscape/` (MATLAB, generate and wire the models themselves): `build_conveyor_step1.m` … `step4.m` (development steps), `build_conveyor_step5.m` (parameterised model), `run_simscape_campaign.m` (campaign and CSV export). Python: the existing pipeline with `--tag sim` (`split_sim.csv`, `behaviour_sim.json`, `detectors_sim.json`, `results_test_sim.csv`, `TEST_EVALUATED_sim.txt`).

### Plant

| Part | Simscape model | Calibration against the PLC model at speed 60 |
|---|---|---|
| Drive | DC motor (Simscape Electrical, built-in thermal variant: winding resistance rises with temperature), PI speed control, 1 ms voltage driver, gear 20:1 | Current 7.25 A (PLC map 7.25 A) |
| Belt | Drum → friction clutch (drum–belt grip, can slip) → wheel and axle (r = 0.1 m) → belt mass 100 kg, roller friction | Belt ratio exactly 1 while the grip holds (no creep) |
| Load | 30 N mean + two random first-order components (τ 20 s and 600 s, 12 N each) | Load–current correlation +1.00 (before sensor noise) |
| Thermal | Motor winding (copper loss I²R); drum bearing heated by its own friction loss (bearing ≈ 5.4 W; seals/scrapers separate); ambient drift σ 1 °C, τ 1 h | Motor +30.6 K, τ ≈ 76 s; bearing +10.4 K (900 s run), τ 124 s |
| Faults | Jam: braking force on the belt up to 1500 N. Slip: grip capacity × (1 − s). Overload: +300 N load. Wear: extra bearing friction up to 2.4 N·m. Hidden severity s as in v3 (jam linear, others quadratic, random duration ×0.5–2), trip at s = 0.8 | All four trip at their nominal times |

Component physics is from the library; component parameters are assumed and calibrated to the PLC model's steady state. **Vibration is not modelled** (`rVibration` is NaN); all four detectors run without it.

**Development findings (single runs):**
- Slip has no signature until the grip falls below the demand (s ≈ 0.65): the belt ratio stays at exactly 1 for about 90 % of the time to trip, then breaks away.
- Overload heats the motor more strongly than in the PLC model (copper loss ∝ I²).
- **The PLC model broke energy conservation for wear:** it heated the bearing without extra motor power. In Simscape the motor supplies the bearing's extra friction loss (about +0.35 A at the trip, smaller than the load wander of σ ≈ 0.23 A).
- First calibration error, fixed: attributing all drum friction (91.5 W) to the bearing made wear heat the motor to 110 °C. Split into bearing (5.4 W) and seals/scrapers.
- Simulink issues fixed along the way: algebraic loops (1 ms driver, 50 ms load-direction lag, 0.5 s heat lag), Simulink-PS converter derivatives (input filtering), PI anti-windup chattering (zero-crossing detection off, adaptive algorithm).

### Campaign

150 / 150 runs `ok` in 204 min computing time (about 29.5 h simulated, about 9× faster than real time): the same plan as campaign v3. Per run: 420 s warm-up, healthy hold (faults 20–60 s, healthy 180 s, soak 1800 s), fault until 10 s after the trip. Sensors as v3 (current 0.05 A noise, 0.01 A resolution, spikes; temperatures 0.08 °C, 0.1 °C resolution). CSV format identical to `data/v3` (checked on a 5-run dry run). Dataset `BeltTwin_simscape_dataset_2026-10-04.zip`, not in git.

**Difference from the PLC data:** the healthy belt ratio is far cleaner (σ 0.0001 per sample vs about 0.025), because the Simscape belt has no creep and all signals are sampled at the same instant (no non-atomic OPC UA reads).

### Behaviour model

The unchanged v3 model structure (steady-state temperature linear in speed) **failed** the acceptance check: speed-dependent bias up to ±1.2σ (bearing and motor), because bearing loss grows with speed² and copper loss with current². As with the model order in v3, the structure was selected on the **training** runs by grouped cross-validation:

| Residual σ (out-of-fold, healthy) | Linear in speed | With speed² term |
|---|---|---|
| Bearing | 0.313 °C | **0.086 °C** (sensor noise floor) |
| Motor, load-compensated | 1.26 °C | 0.49 °C |
| Speed bias | up to ±1.2σ | below 0.2σ (one bin 0.22σ) |

Identified bearing τ 120.2 s (plant 120 s). The bearing has no load gain (−0.003 °C/A), physically correct: the load sits on the rollers. The motor stays well above the noise floor (0.49 °C): I² heating and temperature-dependent resistance are not captured by the model. The v3 model stays linear and its frozen files are unchanged (checked byte-identical after the code change).

### Detectors (train) and single test evaluation

The same four detector designs, tuned on the Simscape training runs to 0.5 false alarms per healthy hour (vibration channels removed). Test set evaluated once (reference run on the project PC; RF threshold 0.8658).

**False alarms** on 12.16 healthy test hours:

| Detector | False alarms | Per hour | 95 % CI |
|---|---|---|---|
| fixed | 2 | 0.16 | [0.02, 0.59] |
| cusum | 7 | 0.58 | [0.23, 1.19] |
| iforest | 7 | 0.58 | [0.23, 1.19] |
| rf | 2 | 0.16 | [0.02, 0.59] |

**Detection** (detected / runs, median delay, median fraction of time to trip):

| Fault | fixed | cusum | iforest | rf |
|---|---|---|---|---|
| jam | 10/10, 1.00 s (0.53) | 1/10, 3.25 s (0.81) | 0/10 | **10/10, 0.75 s (0.35)** |
| slip | 5/10, 25.8 s (0.95) | 1/10, 44.5 s (0.94) | 3/10, 34.8 s (0.98) | 6/10, 19.4 s (0.93) |
| overload | 10/10, 35.8 s (0.74) | 10/10, 33.5 s (0.65) | 8/10, 45.5 s (0.76) | **10/10, 33.1 s (0.53)** |
| **wear (unseen)** | 17/25, 335 s (0.77) | **25/25, 87.3 s (0.16)** | 14/25, 164 s (0.42) | 0/25 |

**Findings:**
1. **The main result replicates on independent physics:** CUSUM on load-compensated residuals detected all 25 unseen wear runs at a median 16 % of time to trip (PLC data: 17 %). The Random Forest detected none (PLC data: 11/25).
2. **On trained faults the Random Forest is again as good or better** (jam, overload).
3. **Slip is late for every detector** (93–98 % of time to trip): physically there is no signature before the grip breaks away.
4. **CUSUM is weaker on overload than on the PLC data** (0.65 vs 0.37): the load-compensated motor residual is less clean (0.49 °C, lag-1 autocorrelation 0.968, so it is pre-whitened).
5. **False alarms:** CUSUM and Isolation Forest at 0.58 per hour, slightly above budget (CI includes it).

**Limitations to state with these numbers:**
- The behaviour model's structure (speed² term) had to be re-selected from the Simscape training data; the method transferred, the model structure did not.
- Still simulated, and the Simscape component parameters were chosen and calibrated by the same person. The physics comes from the library, not the parameters.
- Wear heats the bearing in any plausible physics, which favours a bearing-temperature residual in both models.
- No vibration channel; the belt ratio is free of creep and pipeline artefacts.

---

# Part B — History: sensor model v1 (17–22 Sep 2026)

## B1. OPC UA exposure (still valid, apart from the count)

- **Server device:** "TwinCAT 3 PLC (TMC) - Filtered", using `Port_851.tmc`, with only pragma-tagged variables exposed. The original "TwinCAT Symbol Server" (unfiltered) setting would have exposed every internal variable.
- **Verified in UaExpert:** 23 variables, all `Good`, and no hidden variables (fault severities, wear state, internal targets) visible. **v2 exposes 25**, adding `bMaintenanceCmd` and `nMaintenanceCount`. That was re-verified on 24 Sep, and none of the new internals (`nRng`, `aGauss`, `...Clean` states) are visible.
- NodeId format: `ns=4;s=MAIN.<name>`.

## B2. Motor-current noise bug (fixed 20 Sep, still valid)

- **Original code:** noise was written into the filter state (`rMotorCurrent := rMotorCurrent + rNoise*0.08` after the filter step). That makes a random walk.
- **Measured swing:** 7.9–10.1 A at speed 80, against a documented ±0.08 A. The equilibrium deviation is about 0.064 / 0.02 ≈ 3.2 A.
- **Fix:** the filter runs on a hidden clean state (`rCurrentClean`), and noise is added to the output only.
- **After the fix:** 8.95–9.08 A at speed 80.
- Vibration never had this bug, because it was recomputed from scratch every scan. v2 applies the same output-only principle to both temperatures.

## B3. Healthy steady state at speed 80 (superseded by A1)

| Signal | v1 value |
|---|---|
| Motor current | 9.0 ± 0.08 A |
| Motor temp | 61.989 °C *(float stall, target 62.0)* |
| Bearing temp | **35.977 °C** *(float stall, target 36.0; an earlier version of this document said 35.77, either a transcription error or a reading taken before the bearing had fully settled)* |
| Vibration | 2.19 |
| Position | +20.0 per 250 ms sample |

The motor's thermal time constant is about 60 s and the bearing's about 120 s, so full warm-up from 22 °C takes 5–10 min. **The time constants are still valid.** The v1 temperatures were noiseless, so the values were identical to 17 digits between samples.

## B4. Trip times at speed 80 (still valid, now confirmed across speeds in A2)

Severity accumulates per scan while Running and trips at ≥ 0.8. **Predicted trip time = 0.8 / rate / 100** (100 scans per second).

| Fault | Bit | Code | Measured | Predicted |
|---|---|---|---|---|
| Jam | `bJamInject` | 1 | 2.01 s | 2.0 s |
| Slip | `bSlipInject` | 2 | 26.33 s | 26.7 s |
| Bearing wear | `bWearInject` | 3 | about 400 s | 400 s |
| Overload | `bOverloadInject` | 4 | 53.95 s *(about 0.5 s high; see A2)* | 53.3 s |
| Manual | `bFaultInject` | 5 | instant | instant |

These supersede the values in older notes (2.5 s, 30 s, 7 min and 60 s), which were wrong.

## B5. Fault signatures at trip vs healthy, speed 80 (the directions still hold)

| Fault | Current | Motor temp | Bearing temp | Vibration | Position |
|---|---|---|---|---|---|
| Jam | 9.0 → 17.15 (+8.1) | 62.11 (+0.12) | unchanged | 2.19 → 7.61 | **stalls** (20 → 1.6 per sample) |
| Slip | 9.0 → 7.50 (**−1.5**) | unchanged | rises | 2.19 → 4.20 | **halves** (20 → 10.45) |
| Wear | unchanged | unchanged | 35.77 → 51.90 (+16.1)* | 2.19 → 5.83 | unaffected |
| Overload | 9.0 → 13.43 (+4.4) | 61.99 → 71.44 (+9.5) | unchanged | 2.19 → 3.19 | unaffected |

\* Measured from the recorded baseline of 35.77. From the true stalled baseline of 35.977, the rise is +15.9 °C. See B3.
| Manual | none | none | none | none | none |

**Key findings (still valid):**
- Jam and slip both cause a speed/position mismatch, so **position data alone cannot separate them**. Motor current separates them cleanly and in opposite directions (+8.1 A against −1.5 A). Don't claim they're non-separable in general.
- Each fault has a distinct signature across the four sensors. No two faults move the same set of signals in the same direction.
- **Thermal lag limits fast faults.** Overload reached only +9.5 °C of its +28 °C steady-state target in 54 s, with a 60 s time constant. Wear, at 400 s, reached +16 of +22.4 °C. Motor temperature is useless for jam, a 2 s fault.
- **Vibration responds with no lag** and rises in all four real faults. It's the best general-purpose early indicator, but it can't tell the faults apart on its own.
- Manual fault has no sensor signature, so it's excluded from the classifier.

## B6. Detection lead time, v1 (SUPERSEDED: invalid under v2 noise)

This is the first sample outside the v1 healthy noise band, before the PLC trips.

| Fault | Lead before PLC trip | Samples of transient at 4 Hz |
|---|---|---|
| Jam | about 2.0 s | about 8 |
| Slip | about 23 s | about 105 |
| Overload | about 50 s | about 215 |
| Wear | about 380 s | about 1600 |

These were measured with noiseless temperatures and deterministic sinusoidal noise, so they overstate how detectable the faults are. *(TODO: re-derive from v2 campaign data.)*

**Still valid as a structural point:** jam is the limiting case, with only about 8 samples at 4 Hz between onset and trip. Detection is feasible, prediction is not. That's a sampling-rate limitation, and it should be stated as one.

## B7. Latency, v1 (SUPERSEDED by A5)

22 Sep, 240 samples, streaming 23 process variables at 4 Hz: MQTT hop (Node-RED publish → Unity) 20 ms median / 36 ms p95, and data age (OPC UA read → Unity display) **197 ms median / 221 ms p95**. This excludes the PLC side. Never quote it as "PLC to display". The inherited figure "below 600 ms" was never measured on this system and must not be used.

## B8. Sampling-rate decision (still valid)

- **4 Hz (250 ms) READ polling.**
- **10 Hz was rejected:** OPC UA read round trips took 200–290 ms, so about 90 % of rows were duplicates.
- **SUBSCRIBE mode was rejected:** updates arrived about 1 s apart, including 9 identical messages within 800 ms.

## B9. Unity twin functional tests (still valid)

START/STOP/RESET from Unity, JAM injection (boxes stall, belt turns red, fault banner), and link loss (orange NO DATA banner, grey belt, buttons disabled) all pass. Auto-reconnect was not tested under v1; it was verified in A6.

## B10. Issues found in v1, and their status

| Issue (found 20 Sep) | Status |
|---|---|
| All trip times in the earlier handover were too long | Fixed: measured values in B4 and A2 |
| `nFaultCount` counts manual injections only | **Fixed in v2:** counts every trip |
| Half-wave-rectified current noise in Faulted | **Fixed in v2:** smooth decay to 0 A when not Running |
| Motor-current noise accumulated (swing about 2.3 A) | Fixed 20 Sep (B2) |
| Bearing wear resets to 0 on Start (physically wrong) | **Fixed in v2:** persists until maintenance |
| Noise phase wrapped at 1000, not a multiple of 2π | **Fixed in v2:** LREAL phase, wrapped at 20π |
| Noise was deterministic (two fixed sinusoids) | **Fixed in v2:** random Gaussian component added |
| Temperatures stalled short of target (float32) | **Fixed in v2:** LREAL thermal states |
