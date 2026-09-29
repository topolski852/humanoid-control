# Heading on hardware, 2026-09-29: IMU, torso yaw, and the robot-side loop

Robot-PC brief item 3. The loop is built and verified in `dry` mode; it has not yet been run
`on`.

## 1. IMU heading is usable

| check | result |
|---|---|
| **sign** (operator turned the robot 45° CCW from above) | fused yaw **+49.7°**, gyro-z positive: CCW-positive, same sense as sim's +ang_vel_z |
| **drift at rest** (5 min squatting, IDLE) | +0.001°, and the gyro reads exactly 0: the IMU clamps its gyro when still, so a resting test cannot show drift |
| **drift under a policy stand** (180 s untouched, feet planted, gyro live) | 0.1–1.0 °/min (gyro-z bias −0.002 / −0.016 °/s) |
| **fused yaw vs gyro while walking** | rate correlation 0.94, no lag |

IMU drift is at most about 1 °/min. The veer to correct is 6–11 °/s, so drift doesn't limit a
heading loop over any walk this robot can do.

## 2. New finding: the torso twists 12–31° every step

With the heading loop in `dry` mode, the logs show large yaw motion **at the gait frequency**:

| policy | yaw swing per step (peak-to-peak) | gyro-z \|p95\| | yaw oscillation = knee frequency? |
|---|---|---|---|
| measC-full (7 bouts) | 13–31° (typ. ~18°) | 83–142 °/s | yes, in 6 of 7 bouts |
| smoothA-full (4 bouts) | 12–19° | 81–104 °/s | yes, in 3 of 4 |

- **It is real rotation, not IMU noise.** Fused yaw and gyro agree.
- **The slow net veer (3–17 °/s per bout) sits under this oscillation.** It is not the same thing.
- **Operator support may damp it, or add to it.** It still appears in every bout, for both
  policies.
- **For training: check sim's base yaw-rate distribution while walking.** On hardware, |ang_vel_z|
  p95 is ~1.4–2.5 rad/s. If sim is far below that, it is a plant or contact mismatch. Candidates:
  - foot friction or contact, since a sliding stance foot lets the torso counter-rotate
  - hip_yaw compliance under load
  - how far this is outside the ang_vel_z observations the policy saw

  The ang_vel_z observation itself is also affected: the policy sees values this large on
  hardware.

## 3. The loop: `humanoid_control/heading.py`

Runs in `PolicyRunner.step()` before the observation is built. Modes are `off` (default), `dry`
and `on`, via `POST /api/heading`; the mode can't be changed while moving.

- **Walking, no operator yaw:**
  `wz = clip(0.5 · lowpass(wrap(target − yaw)), ±0.5 rad/s)`. 0.5 is sim's
  `heading_control_stiffness`.
- **Operator yaw:** passed through unchanged. The target follows the robot, so the loop never
  fights the stick.
- **Standing:** wz as commanded (0, as sim's standing envs). The target re-latches.
- **Low-pass, τ = 1.0 s**, added because of §2. Without it, the per-step twist becomes a
  ±0.3 rad/s turn command that flips direction every step: nothing like the slowly varying
  commands the policy trained on.
  - Replaying the dry-run walk: the unfiltered command peaks at 0.34 rad/s with ~1 sign flip per
    second; filtered, 0.21 rad/s and 0.26 flips per second.
  - Offline: a ±15° twist at 1.4 Hz gives ≤ 0.016 rad/s, while a steady 10 °/s veer still draws
    −0.35 rad/s after 5 s, against the veer.
- **Logged per tick:** the IMU quaternion, the operator's command, and the heading state (yaw,
  target, raw and filtered error, `wz_loop`, `wz_sent`).
- **Tests:** `scripts/test_heading.py`, 22/22.

## 4. Next

`on`-mode walks with measC-full, the same bouts as this morning. Compare net veer per bout against
the loop-off baseline (measC: −66, −21, +20, −17, −19, +34, −8° per bout; 3–17 °/s).

## Files

- `imu_yaw_drift_20260929T140505_squat.json`
- `measC-heading-dry-20260929_walk.json`
- Tick log: `recordings/run_1790705543932967453_23984.jsonl` (heading fields included)
