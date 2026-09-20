\# Fault characterisation — 20 Sep 2026



PLC task 10 ms. Telemetry at 4 Hz (250 ms) via OPC UA -> Node-RED -> MQTT.

Severity accumulates per scan while Running; trips at severity >= 0.8.

Predicted trip = 0.8 / rate / 100 scans per second.



\## Healthy steady state at speed 80

| Signal | Value |

|---|---|

| rMotorCurrent | 9.0 +/- 0.08 |

| rMotorTemp | 61.989 |

| rBearingTemp | 35.77 |

| rVibration | 2.19 |

| rPosition | +20.0 per 250 ms sample |



Motor thermal time constant \~60 s, bearing \~120 s. Warm-up from 22 C takes 5-10 min.



\## Trip times



| Fault | Inject bit | Code | Measured | Predicted | Old notes |

|---|---|---|---|---|---|

| Jam | bJamInject | 1 | 2.01 s | 2.0 s | 2.5 s (wrong) |

| Slip | bSlipInject | 2 | 26.33 s | 26.7 s | 30 s (wrong) |

| Bearing wear | bWearInject | 3 | \~400 s (6.67 min) | 400 s | 7 min |

| Overload | bOverloadInject | 4 | 53.95 s | 53.3 s | 60 s |

| Manual | bFaultInject | 5 | instant | instant | instant |



\## Signatures at trip (vs healthy)



| Fault | Current | Motor temp | Bearing temp | Vibration | Position |

|---|---|---|---|---|---|

| Jam | 9.0 -> 17.15 (+8.1) | 62.11 (+0.12) | unchanged | 2.19 -> 7.61 | STALLS (20 -> 1.6) |

| Slip | 9.0 -> 7.50 (-1.5) | unchanged | rises | 2.19 -> 4.20 | HALVES (20 -> 10.45) |

| Wear | unchanged | unchanged | 35.77 -> 51.90 (+16.1) | 2.19 -> 5.83 | unaffected |

| Overload | 9.0 -> 13.43 (+4.4) | 61.99 -> 71.44 (+9.5) | unchanged | 2.19 -> 3.19 | unaffected |

| Manual | none | none | none | none | none |



\## Key findings

\- Jam and slip both cause a speed/position mismatch, so d\_pos\_dt cannot separate

&#x20; them. Motor current separates them cleanly and in OPPOSITE directions:

&#x20; jam +8.1 A, slip -1.5 A.

\- Each fault has a distinct signature across the four sensors. No two faults

&#x20; move the same set of signals in the same direction.

\- Thermal lag limits fast faults: overload reached only +9.5 C of its +28 C

&#x20; steady-state target in 54 s (60 s time constant). Wear, at 400 s, reached

&#x20; +16.1 C of +22.4 C. Motor temp is useless for jam (2 s fault).

\- Vibration responds with no lag and rises in all four real faults. Best

&#x20; general-purpose early indicator; cannot discriminate on its own.



\## Detection lead time (first sample outside healthy noise band)

| Fault | Lead before PLC trip | Samples of transient at 4 Hz |

|---|---|---|

| Jam | \~2.0 s | \~8 |

| Slip | \~23 s | \~105 |

| Overload | \~50 s | \~215 |

| Wear | \~380 s | \~1600 |



Jam is the limiting case: 8 samples at 4 Hz. Detection is feasible,

prediction is not. Note as a sampling-rate limitation in the thesis.



\## Issues found

\- All documented trip times in the earlier handover were too long. Use measured.

\- nFaultCount counts manual injections only, not severity trips.

\- In Faulted state rMotorCurrent shows half-wave-rectified noise instead of 0,

&#x20; because the negative clamp cuts noise below zero. Nonphysical, cosmetic.

\- rMotorCurrent noise accumulated (swing \~2.3 A vs spec +/-0.08) until fixed on

&#x20; 20 Sep: the filter now runs on rCurrentClean and noise is applied only to the

&#x20; published output.

\- Bearing wear resets to 0 on Start. Physically wrong; degradation

