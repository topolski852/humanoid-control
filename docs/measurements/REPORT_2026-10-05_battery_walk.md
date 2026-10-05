# Battery walk after the right-leg rebuild, 2026-10-05: the joints deliver full torque

measC-full on the rug, **on the robot's battery** (24.4 V at rest), after the right-leg rebuild and
an encoder/flux recalibration. The ESC config is verified identical to the studio config file
(`scripts/verify_device_config.py`; `left_ankle_pitch` watchdog set 0 → 1000). Network verified
by replay. No faults; the freeze guard and mode watcher stayed quiet.

**Short:** 2 bouts, 4.4 s of walking. A longer session is needed before these become the baseline.

## Delivered torque (brief items S1 and W): no shortfall

The ESC-measured output torque (param 0x048), read passively from the slow poll by
`power_log.py`, ~3.3 Hz per joint, 400 samples each:

| joint | delivered \|τ\| p95 | **max** | cap |
|---|---|---|---|
| left / right knee_pitch | 2.5 / 3.8 | **16.0 / 12.6** | 12 |
| left / right hip_pitch | 1.8 / 1.2 | **12.5 / 11.1** | 12 |
| left / right ankle_pitch | 3.2 / 1.6 | 7.0 / 7.9 | 7 |

Bus voltage: median 23.95 V, **min 22.30 V** (sag 1.65 V). No collapse.

**The hip_pitch and knee joints deliver their full 12 N·m cap on stable power.** The training PC's
plant-ID showed the stalls require these joints to deliver only ~40–50% of cap (~5–6 N·m). That
fits the **bench supply collapsing to ~6 V on hard steps** (REPORT_2026-09-29_power_collapse.md),
not a joint, firmware or current-path limit.

## Gait on stable power, against the bench-supply walks and sim

| measC-full | bench supply, rug (09-29) | **battery, rug (10-05)** | sim (vx 0.6) |
|---|---|---|---|
| knee L/R corr | −0.35 … −0.45 | **−0.63** (bout 1 **−0.86**) | −0.87 |
| knee swing L/R | 0.61 / 0.65 rad | **0.68 / 0.76** | 0.85 |
| gait | 1.0–1.4 Hz | **1.64 Hz** | 1.58 |
| clearance median / p10 | 32 / 14 mm | **52 / 36 mm** | 53 / 44 |
| stall fraction | 17–54% | **8%** (1 stall) | 0.01% |
| torso \|heading rate\| | 5.7–10.8 °/s | 4.5 °/s | |
| tilt p95 | 8–19° | 12.6° | |

On a stiff supply the gait moves most of the way to sim. Bout 1's knee alternation (−0.86) and
foot clearance match sim.

## Flags

- **Ankle_pitch velocity spikes of 24–25 rad/s** on both ankles. The left one at −50.5°, past its
  ±45° limit (a foot slap or hardstop bounce). One spike was while standing (vx 0). No faults
  this time, but above the ~13 rad/s where encoder faults appeared on other joints.
- Sagittal commanded-torque saturation is still 18–34% of ticks, but the ESCs deliver up to the
  cap. The remaining gap to sim is no longer explained by a delivery shortfall.

## For training

- **Do not train around a reduced sagittal torque budget.** The joints deliver the cap; the
  09-29 shortfall was the bench supply.
- **All 09-29 walk metrics** (stalls, knee corr, clearance, sagittal torque gap) **are superseded**
  by battery data once a longer session is in.

Data: `measC-rug-battB-20261005_walk.json`, `power_log_*_battB.json`,
`walk_*_measC-rug-battB_{can,M2M3M6}.json`; tick log `recordings/run_1791235133913906592_6654.jsonl`.
