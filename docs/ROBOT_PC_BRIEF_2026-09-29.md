# Robot PC brief — 2026-09-29

**Start here.** This is the work queue for the robot PC. It is self-contained: a new session can
build and run everything below from this file plus the documents it links.

**Division of labour.** The training PC (humanoid-policy) builds training changes and bundles.
This PC builds the measurement tools, runs them on the hardware, and writes the results back to
`docs/measurements/` so the training side can use them. Results come back through git.

---

## UPDATE after the 2026-09-29 session — read this first. It supersedes the queue further down.

### What the session settled

* **measC-full is the base for the next round.** It's the best stander and, now that it has been
  walked for real, the best walker too. Knee swing is 0.61–0.65 rad, 1.41 Hz, and the best bouts
  reach knee correlation −0.73. The A/B tied on balance, measD stood up worse, and sim's ranking
  held.
* **The 09-28 "measC walk" was the August bundle.** Every walk number in that report, and the
  "3–5× on six joints" torque table in `HARDWARE_PLAN_2026-09-28.md`, compared measC's sim with a
  different network's hardware. Those numbers are invalid.
* **Heading loop:** built, sign-verified, low-passed for the per-step twist, and validated in dry
  mode. It has **not** been tested with the loop on, because the right knee faulted.

### The corrected sim-vs-hardware picture (computed on the training PC, 2026-09-29)

Same network (measC-full), same command (**vx 0.6**, which is what the walks used; earlier sim
evals ran at 0.3). Hardware numbers are pooled from the two clean walk captures.

| joint | hw/sim torque p95 | sim sat | hw sat |
|---|---|---|---|
| hip_pitch L / R | **2.6× / 3.1×** | 0.1% | 16–21% |
| knee_pitch L / R | **3.1× / 1.9×** | ~1% | 10–24% |
| ankle_pitch L / R | 0.9× / 1.1× | ~10% | 5–9% |
| hip_roll, hip_yaw | 1.0× | ~0% | 1–3% |

* **The gap is confined to the sagittal joints.** hip_roll and hip_yaw use the **same M6C12 actuator
  model** as hip_pitch and knee, and they match sim. So the actuator model (armature, damping, the
  torque filter) is probably not the main cause. The difference is what the sagittal joints do:
  carry body weight through stance, and swing the leg.
* **The torso twists 2.4–4× more than in sim.** Hardware |ω_z| p95 is 1.4–2.5 rad/s; sim is
  0.60–0.67. This answers the question in `REPORT_2026-09-29_heading.md` §2: it points at foot
  contact.
* **"Stuck, then breaks free" (operator) shows in the bout data.** measC's knee correlation swings
  between −0.73 and 0.00 across bouts, and its gait between 1.49 and 0.69 Hz. **It is not the
  encoder freezing:** right-knee stale-hold is 0.000% in all four clean walk captures.
  - The two leading candidates are both contact effects, which no hanging-leg test can see.
  - **Swing-foot scuffing:** hardware knee swing is 0.63 rad against 0.86 in sim, so the foot
    clears the floor less. The toe catches, the hip and knee saturate, then it breaks free.
  - **Stance-foot slip or pivot:** this fits the extra torso twist.
* **The training PC is running plant-identification evals now.** It replays measC-full on
  perturbed sim plants (lower foot friction, more sagittal damping, more sagittal inertia) to see
  which reproduces the hardware signature. The results will be appended here.

### Corrections to the session's own record

* **The right knee has faulted at least six times, not five.**
  `walk_20260929T141225_measC-heading-dry_can.json` ends in a node-8 flood: 198,379 EMCY frames,
  codes 0x2040 / 0x2000, starting ~40 s in. It's not in the commit messages or the heading report.
  The walking bouts in that capture end at ~33 s and show normal right-knee swing, so they look
  valid. That makes three node-8 faults on 09-29.

### New queue

| # | item | needs the robot? | why |
|---|---|---|---|
| **A** | **Right knee inspect / replace** | physical | **Blocks all walking.** Six faults, three in one day, one frozen silently at rest. |
| **B** | **Stuck-event analysis** on existing tick logs | **no** | Explains "stuck, then breaks free". |
| **C** | **Swing-foot clearance** from existing tick logs (FK) | **no** | Directly tests the scuffing hypothesis. |
| **D** | **Floor friction** of the foot sole | minutes, robot off | Tests the contact hypothesis; sim trains μ 0.4–1.2. |
| E | Heading loop **on** walk | after A | Tool is ready; baseline veer is 3–17 °/s per bout. |
| F | Step response (spec below, §2) | after A | Now a damping check rather than the lead. |
| G | Daemon state bugs (clear_faults / disarm) | code | Leaves joints unfed; likely caused the three-hip E-stop. |

B, C and D need no powered robot, so they can run while the knee is out.

#### B. Stuck-event analysis — tick logs already recorded

Inputs: the four clean walk logs listed in `REPORT_2026-09-29_walk_measC_vs_smoothA.md` "Files",
plus the two `measC-dry2` logs.

1. Within trigger-held, stick-forward bouts, flag **stall windows**. A stall is ≥ 0.3 s where
   **both knees move < 0.5 rad/s**, *or* hip_pitch or knee_pitch sits at its torque cap
   (reconstructed kp·err − kd·vel ≥ 0.98 × cap). Tune the thresholds if they obviously
   misfire, and say what you used.
2. For each stall, report: start time, duration, which leg, per-joint position error and
   saturation, **both foot heights from FK** (see C), IMU pitch/roll and yaw rate, and action
   magnitude.
3. For each stall, also report **how it ends**: which joint releases first, and whether a foot
   height changes just before it breaks free.
4. Totals: the number of stalls, the fraction of walking time stalled, and whether the stall rate
   differs between measC and smoothA or correlates with heading veer.

#### C. Swing-foot clearance — FK on the same tick logs

* Get foot positions from the 12 logged joint angles using `leg_gravity.LegModel.fk`. Use the
  `*_ankle_roll` link origin for each foot. Rotate them to the world frame using the logged IMU
  quaternion.
* **Proxy, defined exactly so both sides compute the same number:**
  - Let `d(t) = z_left(t) − z_right(t)`.
  - Split the walk into **steps at every sign change of `d`**.
  - Each step's **clearance = max |d|** between two consecutive sign changes. That is the swing
    foot's peak height above the stance foot.
  - Drop half-steps shorter than 0.15 s; those are jitter at the crossing.
  - Report the median and the 10th percentile of per-step clearance. The 10th percentile is the
    one that catches scuffing.
* Report the distribution per bout, and **stall vs non-stall**. If clearance drops toward zero
  during stalls, scuffing is confirmed.
* The training PC will compute **the same proxy in sim**, so the two numbers are directly comparable.
  Use exactly this definition.

#### D. Floor friction

With the robot unpowered, measure the foot sole on **the floor surface the walks happen on**.
Either tilt a board of that surface until the foot slides (μ = tan θ), or pull the foot with a
spring scale (μ = F / m g). Report static and kinetic μ, and name the surface. Sim trains on
μ 0.4–1.2. If the real value is below 0.4, the policy has never seen that floor.

#### Standing notes for all walk captures

* Keep walking at **full stick (vx ≈ 0.6)**, or record the command. `walk_metrics.py` already logs
  `command_vx_mean`. The training side matches its eval to it.
* Every walk is **supported**. The operator's hand can add yaw torque and load or unload the legs.
  Note in the report when support was heavy.

---

## Original 2026-09-29 queue (kept for the item specs)

The table below is superseded by the UPDATE above. Item 1 is **done**
(`REPORT_2026-09-29_AB_measD_vs_measC.md`). Item 3 is **built**, but not yet run with the loop on
(`REPORT_2026-09-29_heading.md`). The step-response spec in §2 still stands as written.

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
