# measF-full-ft on hardware, 2026-10-06

measF = measC-full fine-tuned with a soft-landing reward (foot speed at touchdown), aimed at the
ankle_pitch foot-strike spikes. Network verified by replay. Right knee gearbox repaired and
everything recalibrated before this session, so tilt values are not comparable with earlier days.

## Stand (bench supply)

| measF | settle 0–120 s | **untouched 120–300 s** |
|---|---|---|
| tilt median | 9.04° | **8.91°** |
| tilt rate p95 | **0.68 °/s** | **0.04 °/s** |
| joint vel p99 | 0.73 | 0.73 |

- Untouched, measF is at the IMU floor, like measC (0.04 °/s).
- **Its stand-up transient is less steady:** 0.68 °/s while settling (measC 0.06, measE 0.21),
  with two sustained low-frequency wobbles about 20 s after engage (settle 4–5 s).
- **The push phase was cut short by a fault** (next section). No usable push-recovery numbers.

## Left-knee encoder fault caused by bench-supply brownouts during pushes

At 14:35:40, during the push phase, **left_knee_pitch raised 0x2000** (encoder fault), then the
EMCY flood (191k frames) jammed the bus and the service E-stopped. `power_log.py` shows the
**bench supply (20 V, 10 A max) collapsing to ~5.7 V on every firm push**:

| time before the fault | bus V (all ESCs) |
|---|---|
| −24.1 s | 5.74 |
| −9.7 … −5.2 s (several pushes) | 5.69 – 5.91 |
| **−1.0 s** | **7.25** |
| **0 s** | **left_knee 0x2000** |

The knee was moving under 4 rad/s, so this is not an overspeed fault. It is the **first direct
sequence linking supply brownouts to an encoder fault** (on 09-29 it was only suspected). The ESC
`undervoltage_threshold` is 0, so the ESCs run straight through a collapse to ~6 V.

**Consequences:**
- Stand tests with pushes must run on the battery.
- The bench supply is for quiet standing only.
- Consider a real ESC undervoltage threshold (operator decision).

Data: `measF-stand-20261006_stand.json`, `hold_*_measF-stand_*`, `power_log_*_measF-stand.json`;
tick log `recordings/run_1791311391316269335_7661.jsonl`.
