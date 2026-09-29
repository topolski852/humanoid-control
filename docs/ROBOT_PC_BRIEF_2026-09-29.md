# Robot PC brief — 2026-09-29

**Start here.** This is the work queue for the robot PC. It is self-contained: a new session can
build and run everything below from this file plus the documents it links.

**Division of labour.** The training PC (humanoid-policy) builds training changes and bundles.
This PC builds the measurement tools, runs them on the hardware, and writes the results back to
`docs/measurements/` so the training side can use them. Results come back through git.

**Context.** measC-full is the best stander measured: tilt rate 0.04 °/s untouched, 10× better than
smoothA. But it walks in a shuffle and in a circle. M7 (2026-09-28) refuted the soft-joint theory.
`kp = 45` on every servo, and `Kt·gear` and the link masses are verified. So the 3–5× sim/hardware
torque gap is **dynamic**. The step response (item 2) is the measurement that can find it.

| # | item | tools exist? | gates |
|---|---|---|---|
| 1 | A/B stand: measD vs measC-full | yes | nothing — quick |
| 2 | **Step response (M7 item 4)** | **no — build it** | **the next training round** |
| 3 | Heading loop in the policy runner | **no — build it** | straight-line walking |
| 4 | Right leg inspection | physical | how hard anything can be tested |

---

## 1. A/B stand: measD vs measC-full

Both bundles are staged in `policies/` and pass `test_policy_compat`. Their contracts are
identical, so switching in the dropdown swaps only the network. **No ESC changes.**

Protocol and background are in
[`policies/walk_measD-fast_2026-09-28/NOTE.md`](../policies/walk_measD-fast_2026-09-28/NOTE.md).
In short:

* Calibrate once for both runs. The ±0.029 rad spread between calibrations is larger than the
  effect being tested.
* Run `capture_run.sh <label>-stand 480 hold` on each: 120 s settle, **180 s untouched**, then
  small matched pushes.
* No hard push. It faulted `right_knee_pitch` last time.
* Run measD first, then measC-full, back to back.
* Compare on the untouched window, plus ring duration after each push.

Sim ranks measC-full slightly **above** measD. So this A/B tests whether the sim ranking holds for a
small difference. It is not a test of the friction change.

---

## 2. Step response — M7 item 4. BUILD, THEN RUN. Highest priority for training.

### Why

M7's static test ruled out every static cause of the torque gap. What's left is dynamic:
reflected inertia (armature), damping, the firmware torque filter, and friction while moving. None
of these shows at rest. All of them show in a step response.

### What the sim model predicts

Hanging leg, `kp = 45`, `kd = 1.5`. Inertia is computed from the URDF about each joint axis, plus
the sim armature. Damping is `kd` plus the sim viscous friction.

| joint | J total kg·m² | f_n | ζ | overshoot | peak time |
|---|---|---|---|---|---|
| hip_pitch | 0.208 | **2.34 Hz** | **0.27** | **42%** | 222 ms |
| hip_roll | 0.224 | 2.25 Hz | 0.26 | 43% | 230 ms |
| knee_pitch | 0.081 | 3.75 Hz | 0.43 | 23% | 148 ms |
| ankle_pitch | 0.021 | 7.31 Hz | 0.80 | 1% | 114 ms |

These are linear second-order predictions. Pose-dependent inertia, gravity and friction will shift
them a little. That's fine. What matters is the size of any discrepancy:

* **Hip rings at a lower frequency than 2.34 Hz** → more inertia than modeled. `J ∝ 1/f_n²`, so a
  2× frequency drop means 4× the inertia.
* **Hip overshoots much less than 42%, or doesn't ring** → more damping than modeled. This would
  directly explain the gap, since a damped joint needs far more torque to move at gait speed.
* **A dead time before the joint starts moving** → transport latency plus the firmware
  `torque_filter_alpha = 0.1454` (~50 Hz), which sim doesn't model.
* **Final position stops short of the target** → the friction deadband.

The hips are the sharpest test. They should ring visibly at ~2.3 Hz, well inside what 100 Hz
sampling can resolve.

### What to build: `scripts/measure/m7_step_response.py`

Base it on `m7_sweep.py`. It already has the safety scaffolding and the lessons from M7 v1 and v2.

**Setup.** Same as M7: robot hanging, feet off the ground.
* Hold **all 12 joints in POSITION** throughout. v1's IDLE neighbours sagged.
* Read back `position_ki = 0` on every ESC before moving.
* Start from a pose with modest gravity load, at least 3° from any hardstop.
* Record the full 12-joint starting pose. The training side needs it to replay the step in sim.

**Joints.** In this order:
1. `left_hip_pitch`, `right_hip_pitch` (M6C12, where the gap is ~5×).
2. `left_ankle_pitch`, `right_ankle_pitch` (MAD5010, where the gap is ~2.5×).
3. Optional: `left_knee_pitch`. **Do not step `right_knee_pitch`** until it has been inspected. It
   has faulted three times.

**Stimulus.** The commanded position must be a **true step, not a ramp**. Ramping destroys the
measurement. This breaks the "ramp everything" rule the other M7 tools follow, so keep the steps
small and bounded in code:

| joint | linear steps | saturation step (optional, last) |
|---|---|---|
| hip_pitch | 0.05, 0.10 rad (2.25, 4.5 N·m) | 0.35 rad (15.8 N·m > 12 cap) |
| ankle_pitch | 0.03, 0.06 rad (1.35, 2.7 N·m) | **none.** The ankle is 3D-printed and has broken before. |

* For each size: step out, hold, step back. The return step gives the opposite direction for free.
* Repeat 3× for repeatability.
* Hold at least 1.5 s after each step. The predicted hip settling time is ~1.0 s.
* The hip saturation step probes M7 item 3, the real torque cap under dynamic demand.

**Record per sample, at the highest rate available.** Capture: time, commanded position, measured
position, velocity, and ESC torque/current. Use the same ESC torque source `m7_sweep.py` uses.
* Capture the time series passively (`candump`, as `capture_run.sh` does).
* **Don't open a second daemon UDP client for recording.** It steals packets from the web service.
  See the note in the measA report §8.
* Timestamp the command step itself, so dead time is measurable.
* **Sample rate.** PDO4 is 100 Hz. That is plenty for the hips (peak at ~220 ms). The ankle rises
  in ~55 ms, only ~5 samples. If the firmware allows raising `fast_frame_frequency` on the one
  stepped joint for the test, do it and restore it afterwards. Otherwise record the ankle as
  resolution-limited and say so.

**Safety.**
* Require `--i-am-present` and a confirmation that the legs hang free.
* Clamp step size in code.
* Abort if `|vel| > 5 rad/s` on any joint. The right knee faulted at 5.84.
* Abort on any EMCY frame.
* On Ctrl-C, finish the step, then return home → DAMPING → IDLE.
* Write raw data to disk after every step, not only at the end.

### What to report

`docs/measurements/m7_step_<timestamp>.json` must include:
* the raw time series per step
* the exact commanded sequence
* the 12-joint starting pose

Then fit and report, per joint and step size:
* dead time (ms)
* 10–90% rise time, peak time, overshoot %, 2% settling time
* damped frequency from the ringing
* steady-state error
* **fitted `J_eff` and `c_eff`** from a second-order model with `kp = 45` held fixed
* for the saturation step: peak ESC torque against the 12.0 cap

Compare each against the prediction table above.

Write `REPORT_2026-09-29_M7_step.md`, and add an `actuator.m7_step_response` section to
`TRAINING_INPUT.json`.

**What the training side does with it:** it replays the identical commanded steps from the
recorded starting pose in sim and compares the curves. A frequency mismatch means correcting the
armature. An overshoot mismatch means correcting the damping. Dead time means modeling the filter
or latency. Any of these becomes the plant correction for the next training round.

---

## 3. Heading loop — BUILD, THEN TEST

The design and rationale are in
[`HARDWARE_PLAN_2026-09-28.md` §2](HARDWARE_PLAN_2026-09-28.md). The short version: sim closes
the heading loop in its command generator. The robot has no heading feedback. The policy has no
heading observation. So drift is guaranteed, and it produced the circle: `hip_yaw` drifted 14–22°
over 12 s against a zero yaw command.

**Implementation, in the policy runner, before the observation is built:**

```
heading_error = wrap_to_pi(heading_target - yaw_now)          # yaw from IMU quaternion (0x59)
wz            = clip(0.5 * heading_error, -wz_max, +wz_max)   # 0.5 = sim heading_control_stiffness
```

* **Behind a flag, default OFF.** Item 1's A/B must not be affected.
* Latch `heading_target` when the policy engages. When the operator commands a nonzero yaw rate,
  advance the target (`heading_target += wz_operator · dt`) rather than fighting the operator.
* `wz_max ≤ 1.2 rad/s`, the trained `ang_vel_z` range. Anything beyond is out of distribution.
* Log `yaw_now`, `heading_error` and `wz` every tick in `StepRecorder`.

**Check the sign before any walk. This is the most likely failure mode.** A wrong sign is positive
feedback, and the robot will spin up. Run the loop in a dry-run mode that computes `wz` without
applying it. Then turn the robot by hand and confirm `wz` opposes the rotation. The sim's yaw is
about world +z; the IMU's convention may differ.

**Tests:**
1. Measure IMU yaw drift at rest over 5 minutes. The 6-DOF IMU has no magnetometer, so yaw drifts.
   This bounds how long a walk the loop can keep straight.
2. Run a supported walk with measC-full (or the A/B winner), **loop OFF, then loop ON**. Measure
   `hip_yaw` drift over the gait window and the total heading change. Baseline: 14–22° over 12 s.

---

## 4. Right leg inspection

See [`HARDWARE_PLAN_2026-09-28.md` §3](HARDWARE_PLAN_2026-09-28.md). There have been four faults,
all on the right leg: `right_knee_pitch` ×3 and `right_ankle_pitch` ×1. Check the shared
infrastructure (wiring, connectors, bus termination) before swapping parts.

Stale-hold is a useful early warning. `right_ankle_pitch` degraded to 4.17% stale-hold before it
faulted. It is the only joint in any capture to show non-zero stale-hold.

---

## Don't

These are unchanged from `HARDWARE_PLAN_2026-09-28.md` §4, plus M7:

* Don't lower `kp`, correct `Kt`/gear, or change link masses. M7 verified all three.
* Don't cut the standing fraction or tighten the velocity hinge on the training side. The data
  contradicts both.
* Don't raise the ankle torque cap. Its demand exceeds the physical ceiling, and the part has
  broken before.
* Don't model a per-joint delay stagger from a capture that contains a degraded joint. This is the
  `slot_structure()` lesson from 2026-09-28.

## Per-cycle metrics to keep reporting

For any new bundle, report:
* tilt rate p95 on the untouched stand
* excursions >8°
* joint velocity p99
* ring duration after a push
* for a walk: gait frequency, knee swing amplitude, and `hip_yaw` drift

The hardware constants (M1/M3/M6/M8/B5, M7 stiffness) don't need re-measuring.
