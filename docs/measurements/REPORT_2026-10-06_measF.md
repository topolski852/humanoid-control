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

## Stand on the battery (14:44): fall, then a left_ankle_pitch RUNAWAY — stand rejected

measF on the battery (bus min 22.64 V, no supply problem). Before the event: untouched tilt
10.0°, rate 0.05 °/s (IMU floor); settle rate 1.86 °/s (shaky stand-up again).
**The operator rejects this stand:** "it moved unexpectedly and kept driving the motor into the
hardstop, which heated up."

- **t = 399.6 s:** the robot pitched forward to 50° (rate 189 °/s).
- **From t = 401.4 s:** the **left_ankle_pitch reading ran away**, from −0.8 rad to −128 rad over
  ~7 s at a steady 18–25 rad/s.
  - The target sat at −0.785 rad (the joint limit), and the commanded torque pointed the other
    way.
  - The joint was at its hardstop while the motor kept turning: a free-spinning motor (gear
    train disengaged) or a corrupted encoder/commutation angle. This is the joint with the
    re-glued encoder magnet.
- The ESC raised no fault. After the session its reading still showed −134 rad.

### The same runaway signature appeared earlier, in the measE walk (2026-10-05)

In measE's bout 2 (t = 29.7–30.9 s) the **right knee** reading ran from 78° to **265°** in 0.2 s
at ~20 rad/s. The operator released 0.1 s later. That is the knee whose gearbox was later found
dislocated: **the failure began during the measE walk, not the rope walk.**
- measE bout 2 is corrupt, and bout 3 ran on a possibly compromised knee.
- **Part of measE's "walks weird / more drastic" verdict may have been hardware.**
- The measE rejection rests partly on confounded data.

### Guard update

`scripts/measure/freeze_guard.py` now also E-stops when any joint reads more than **0.5 rad outside
its mechanical limits**, which is physically impossible. Replayed over all 36 tick logs, it fires
on this ankle runaway (t = 401.4 s) and on measE's knee runaway (t = 30.82 s). The only other
hits are two 09-23 logs, from the old reversed ankle_roll calibration. 0.2 rad false-fired on a
real end-of-bout hip move.

## measC-full stand on the battery (16:30), the reference for measF

After the right-knee ESC swap and recalibration. Clean: no faults, freeze/runaway guards quiet,
bus voltage min 23.34 V, median 24.47 V. Network verified.

| measC-full, battery | settle | untouched | pushes |
|---|---|---|---|
| tilt median | 5.00° | 5.01° | 5.40° (max 17.0°) |
| tilt rate p95 | **0.08 °/s** | **0.06 °/s** | 2.77 °/s |
| joint vel p99 | 0.73 | 0.73 | 1.22 |

**Push recovery (firm pushes, CAN 100 Hz):**

| push | peak | ring | settle | frequency |
|---|---|---|---|---|
| 1 | 4494 ct | 7.2 s | 5.2 s | 0.56 Hz |
| 2 | 4931 ct | 6.0 s | 1.5 s | 0.22 Hz |
| 3 (small) | 51 ct | 0.5 s | 1.8 s | 0.22 Hz |

Untouched residual motion is 1.9× baseline (a small 0.24 Hz sway; "marginal damping").

**Against measF:**
- measC settles from stand-up at 0.08 °/s; measF at 0.68–1.86 °/s.
- measC survives firm pushes up to 17° of tilt; measF fell (to 50°) under a push on the battery.
- Both sit at the IMU floor untouched.
- Tilt medians are on different calibrations (measF was before the ESC swap) and are not
  comparable.

### Stance width explains the first two pushes (operator observation, confirmed by FK)

The operator: "the first two pushes went out of control and I had to hold the robot. Once its
feet stood further apart it levelled off easier." Foot positions from FK of the logged joint
angles (ankle_roll origins):

| | stance width | left foot ahead of right |
|---|---|---|
| trained default_pose | **179 mm** | 0 mm |
| untouched / before push 1 | **153–156 mm** | 0 mm |
| after push 2 (stepped out) | **238 mm** | +36 mm |

- **measC stands ~25 mm narrower on hardware than its trained stance.** Width here is pure
  joint-angle geometry, so this is the policy's standing posture on the robot, or a
  hip_roll/ankle_roll calibration offset pulling the feet in. It is not the floor.
- At that width two firm pushes needed the operator to catch it. **Push events 1 and 2 above are
  operator-held and are not valid push-recovery numbers.**
- The pushes made it step out to 238 mm, one foot ahead, and from there it handled pushes
  easily.
- **For training:** compare measC's stand width in sim against 153 mm on hardware. A
  narrower-than-trained stance is a direct robustness gap. Consider randomising the initial
  stance or rewarding a stance width near default.
