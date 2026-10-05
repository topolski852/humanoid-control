# Knee cap 12 → 18 N·m (brief Test 2), 2026-10-05: the left knee overspeeds and drops off the bus

measC-full on the battery and the rug, with `torque_limit` raised **12 → 18 N·m on both knees
only**. Applied live through `POST /api/knee_torque_limit` (readback 18.0 / 18.0). Operator
approved. Network verified by replay.

## What happened

- **One bout of 2.8 s.** About 7 s after engage, **left_knee_pitch went OFFLINE**: it stopped
  answering on CAN, its last firmware mode POSITION. The operator disarmed 3 s later.
- **The tick log shows a violent knee swing just before.**

| t (s) | left knee | target | velocity | commanded demand |
|---|---|---|---|---|
| 21.34 | 80.4° | 63.9° | +1.9 rad/s | −15.9 N·m |
| 21.42 | 73.9° | 20.3° | −4.9 | −34.8 |
| 21.58 | **16.1°** | 37.4° | **−20.4** | +47.3 |
| 21.66 | 16.1° (frozen) | 15.1° | −20.4 (frozen) | — |

- The knee fell 80° → 16° in ~0.16 s at **20.4 rad/s**, well above the ~13 rad/s where these ESCs
  have been seen to lose the encoder (`TRAINING_INPUT.actuator.joint_velocity_limit`). The
  reading then **froze**, and the node went silent.
- Short bout metrics, for completeness only: knee corr −0.53, swing 0.54/0.85, 1.06 Hz, heading
  +62° in 2.8 s, no stalls.

## Reading

- **The extra knee authority converts straight into overspeed.** At 12 N·m the knees mostly stay
  under 13 rad/s (the exceptions so far were runaways and impacts). At 18 the commanded swings
  (±35–47 N·m demand) are realised faster, past the encoder's tracking speed.
- **Raising the knee cap is not viable on its own.** It needs a joint-speed limit with it: a
  policy-side velocity penalty below ~13 rad/s, or an ESC `velocity_limit`. Then retest.
- The demand gap (16–29% of hip/knee ticks above 12 N·m) therefore can't simply be met with more
  cap on this hardware.

## Restore

- **Right knee:** restored to 12.0, read back.
- **Left knee:** unreachable (OFFLINE), so the restore write failed. A power cycle reboots its
  ESC, and the first calibration afterwards writes the config file's 12 to every joint. To be
  verified with `scripts/verify_device_config.py` before further motion.

Data: `measC-cap18-20261005_walk.json`, `power_log_*_measC-cap18.json`,
`walk_*_measC-cap18_*`; tick log `recordings/run_1791240061115073235_6709.jsonl`.
