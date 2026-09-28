# BeltTwin — Tests & Measurements

System: TwinCAT 3 PLC (10 ms task) → OPC UA (TF6100) → Node-RED (4 Hz) → MQTT → Unity twin, plus CSV logging, scenario runner and live RF detector. All measurements were taken on a single PC (Windows, TwinCAT 4026.24, Node-RED 5.0.4, Unity 2022.3.61f1).

**How to read this document.**
- **Part A** is the current system, sensor model v2 from 24 Sep 2026. It is what the dataset and ML results are based on.
- **Part B** is the history: measurements on sensor model v1 (17–22 Sep). They're kept for traceability, and each one is marked as still valid or superseded.
- "TODO" means not yet measured or tested. Don't quote those items anywhere.

---

# Part A — Current system (sensor model v2)

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

## A5. End-to-end latency, PLC scan to subscriber (25 Sep)

`latency_probe.py`, 240 samples at 4 Hz. The PLC timestamp comes from `nPlcTime` (see A4).

| Hop | min | median | p95 | max (ms) |
|---|---|---|---|---|
| PLC scan → OPC UA read complete | 36 | 57 | 86 | 180 |
| Read → Node-RED publish | 11 | 155 | 177 | 184 |
| Publish → MQTT subscriber | 2 | 4 | 5 | 16 |
| **Total: PLC scan → subscriber** | **167** | **214** | **226** | **229** |

**Finding: 72 % of the latency is the Node-RED design, not the protocols.** Poll and publish are two independent 4 Hz timers, so a fresh value waits in the cache for the next publish tick. The fix, publishing on arrival of a new `nPlcTime`, is deferred until after the data campaign, so row timing stays consistent within the dataset. Expected result: about 60–90 ms total. *(TODO: re-measure after the fix.)*

**Unity, same day:** MQTT hop 8 / p95 18 ms, and data age (OPC UA read → display) 174 / p95 192 ms, both over 240 samples. PLC scan → display is therefore about 230 ms median. That's an estimate from adding the hops, not a direct measurement.

**Caveats:** `nPlcTime` resolution is 10 ms, and the method assumes the TwinCAT and Windows clocks are aligned. There were no negative ages, so any offset is small but not proven to be zero.

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

**MQTT last will (`detector_offline`):** **Verified 27 Sep:** closing the detector window (a simulated crash) made the broker publish the last-will message, and Unity showed "RF: detector offline" within seconds.

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
