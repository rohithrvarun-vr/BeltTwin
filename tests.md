\# Fault characterisation — 20 Sep 2026



Sampling: 4 Hz (250 ms). PLC task 10 ms.

Healthy steady state at speed 80:

\- rMotorCurrent 9.0 ± 0.08

\- rMotorTemp 61.99

\- rBearingTemp 35.77

\- rVibration 2.19

\- rPosition +20.0 per 250 ms sample



| Fault | Inject bit | Code | Trip measured | Trip predicted | Signals |

|---|---|---|---|---|---|

| Jam | bJamInject | 1 | 2.01 s | 2.0 s | current 9.0→17.15, vibration 2.19→7.61, position delta 20.0→1.56 |

| Manual | bFaultInject | 5 | instant | instant | no sensor change; only fault that increments nFaultCount |

| Slip | bSlipInject | 2 | | 26.7 s | |

| Overload | bOverloadInject | 4 | | 53.3 s | |

| Wear | bWearInject | 3 | | 6.7 min | |



\## Notes

\- Documented trip times in the old handover were wrong. Jam measured 2.01 s, not 2.5 s.

&#x20; Predicted = 0.8 / rate / 100 scans per second.

\- Jam gives only \~8 samples of transient at 4 Hz before the trip.

\- Motor temp is useless for jam (60 s thermal time constant vs 2 s fault).

\- nFaultCount only counts manual injections, not severity trips.

\- In Faulted state, rMotorCurrent shows half-wave-rectified noise instead of clean 0,

&#x20; because the negative clamp cuts the noise below zero. Cosmetic, nonphysical.



\## Data files

data/run\_2026-09-20T13-53-15-524Z.csv — jam trip at line 95

