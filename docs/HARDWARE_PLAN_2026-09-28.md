# Hardware Plan — 2026-09-28

> **The current work queue is [ROBOT_PC_BRIEF_2026-09-29.md](ROBOT_PC_BRIEF_2026-09-29.md)** — start
> there. It orders the open items and specs the two tools that don't exist yet (step response,
> heading loop). This document remains the background for §2 (heading) and §3 (right leg).

Robot-side work following the measC-full session. **Training is paused until item 1 returns.**

> **Update 2026-09-28, evening: M7's stiffness test has returned, and the hypothesis is refuted.**
> Every leg servo delivers kp = 45 as commanded; `Kt · gear` and the URDF masses check out against
> gravity. Don't soften kp in sim. The torque gap is real commanded demand; its cause is dynamic
> and not yet measured. Training can resume. See
> [REPORT_2026-09-28_M7.md](measurements/REPORT_2026-09-28_M7.md).

Reads from: [REPORT_2026-09-28_measC.md](measurements/REPORT_2026-09-28_measC.md),
[REPORT_2026-09-28_measC_walk.md](measurements/REPORT_2026-09-28_measC_walk.md),
[TRAINING_INPUT.json](measurements/TRAINING_INPUT.json).

---

## What measC settled — do not undo this

**The round-2 premise is confirmed.** measC restored an explicit observation-uncertainty margin on
top of the measured staleness and quantisation, and measA's lurching is gone:

| untouched stand | tilt rate p95 | tilt median | excursions >8° |
|---|---|---|---|
| **measC** | **0.04 °/s** | **2.71°** | **0 / 3 min** |
| smoothA (previous best) | 0.40 °/s | 5.34° | 0.2 /min |
| measA (regression) | 20.60 °/s | 2.89° | 11.7 /min |

10× better than smoothA, 500× better than measA, and it stands half as far off vertical. The
observation model is not the open question any more.

---

## 1. M7 — commanded vs delivered torque. CRITICAL PATH

**RESULT 2026-09-28: stiffness is not the cause.** Servo kp is 44.8–45.2 (median, all 10 joints)
and ESC torque equals kp·err, so the table below reflects real demand. See
[REPORT_2026-09-28_M7.md](measurements/REPORT_2026-09-28_M7.md). Items 3 and 4 below (saturation
point, time constant) are still open, and item 4 is now the lead.

M7 has been "blocking" since 2026-09-23. It is now the **critical path**, and the reason has
changed: it is no longer a knee-specific question.

Comparing measC's sim eval against its own hardware capture, **sim under-predicts torque demand on
every major leg joint**:

| joint | cap | SIM p95 | HW p95 | gap | SIM sat | HW sat |
|---|---|---|---|---|---|---|
| left_ankle_pitch | 7 | 9.3 | 27.5 | 3.0× | 13.6% | 27.2% |
| right_ankle_pitch | 7 | 8.4 | 17.7 | 2.1× | 10.4% | 22.8% |
| left_hip_pitch | 12 | 5.4 | 25.6 | **4.7×** | **0.0%** | **33.9%** |
| right_knee_pitch | 12 | 6.4 | 32.6 | **5.1×** | **0.2%** | **29.7%** |
| right_hip_pitch | 12 | 5.2 | 25.9 | **5.0×** | **0.0%** | **29.0%** |
| left_knee_pitch | 12 | 7.3 | 21.9 | 3.0× | 0.2% | 25.1% |

Hips and knees saturate a **third of all ticks on hardware and essentially never in sim**. This
fully accounts for the shuffle: the policy learns a 0.742 rad stride, the joints saturate, and
0.17–0.24 rad comes out. No reward term addresses that.

### The hypothesis to test first: effective stiffness

A **uniform** 3–5× gap across six joints spanning three motor families and two different torque
caps points at something they all share. The obvious candidate is **`kp = 45`**. If the real
joints' effective stiffness is materially below 45 N·m/rad, then sim believes a given position
error produces ~3× more restoring torque than the robot actually generates — which explains every
row above simultaneously, and explains why raising caps and adding penalties kept moving the
problem rather than removing it.

**Test:** hold a joint at a commanded position, apply a known static load to force a measured
position error, and record the torque actually holding it.

    kp_eff = tau_actual / position_error

Compare against 45. Do this for at least one joint per motor family (M6C12 hip/knee, MAD5010
ankle). If `kp_eff` comes back near 15–20 rather than 45, that is the whole sim2real story and the
fix is a plant correction, not a reward.

### What else M7 must separate

Hardware "torque" in every report so far is **reconstructed** as `kp·err − kd·vel`, never measured.
That is circular: a joint that cannot track has large error, which inflates the inferred demand.
M7 breaks the circle. Report each of these separately:

1. **Commanded → reported current.** Does the ESC deliver the current it is asked for? Use the CAN
   current feedback with the documented scale factor for that model.
2. **Reported current → actual output torque.** Validates `Kt · gear` end to end, i.e. gearbox
   efficiency and Kt accuracy. Needs a real load measurement — a lever arm with known mass is
   enough; a load cell is better.
3. **Saturation point.** Sweep demand upward until delivered torque stops rising. That is the real
   cap, as opposed to the configured one.
4. **Deadband and time constant.** Include the firmware's `torque_filter_alpha = 0.1454` (~50 Hz
   at the 2 kHz position loop) in the analysis; the sim actuator model has no equivalent filter.
5. **Direction asymmetry.** Both faulting joints are on the right leg; check both directions.

`scripts/bench_sweep.py` may already cover part of this — check before writing new tooling.

### Why this gates training

Retraining against a plant that is 3–5× too easy will keep producing gaits the robot cannot
execute, whatever else changes in the reward. The two candidate training responses — a global
torque penalty, or deliberately tightened sim effort limits — are both guesses at a correction
factor that M7 measures directly.

---

## 2. Heading loop on the robot — the circle

The robot walked in a circle: both `hip_yaw` joints drifted 14–22° monotonically over 12 s against
a **zero** yaw-rate command and never recovered. The operator watched it happen and the CAN data
reproduces it.

Report §5a calls this a missing reward term. It is worse than that — **the policy cannot regulate
heading even in principle:**

* Training runs `heading_command=True` with `rel_heading_envs=1.0`, so the sim *command generator*
  closes the heading loop (`heading_control_stiffness = 0.5`) and hands the policy an already-
  corrective yaw rate. The policy never learns heading regulation because it never needs to.
* On the robot, `PolicyRunner` supplies a fixed `(vx, vy, wz)` with no heading feedback whatsoever.
* The 45-dim observation contains **no heading information**. `projected_gravity` is yaw-invariant
  by construction, and `base_ang_vel` gives yaw *rate*, not accumulated yaw.

The loop that keeps the robot straight in simulation does not exist on the hardware. Drift is
guaranteed, and no reward weight can fix an unobservable state.

### What to build

Mirror the sim command generator. In the policy runner, before building the observation:

```
heading_error = wrap_to_pi(heading_target - heading_now)     # heading_now from IMU yaw
wz_command    = clip(0.5 * heading_error, -wz_max, +wz_max)  # 0.5 = heading_control_stiffness
```

`heading_target` is latched when the policy engages. The IMU already publishes quaternion
(frame 0x59), so yaw is available without new hardware.

Two cautions:
* **Yaw from a 6-DOF IMU drifts** — there is no magnetometer to correct it. Over a 10–20 s walk
  that is acceptable; over minutes it is not. Log the raw yaw so the drift rate is measurable.
* Keep `wz_max` inside the trained command range (`ang_vel_z` was ±1.2 rad/s) or the policy will
  see an out-of-distribution command.

This needs no retrain and no contract change. A heading *observation* is the better long-term
answer, but it changes the 45-dim contract and invalidates every existing bundle, so it should
wait until the torque gap is resolved.

---

## 3. Right leg — four faults, all on one side

| joint | faults | latest trigger |
|---|---|---|
| `right_knee_pitch` (node 8) | **3** | 5.84 rad/s, +0.437 rad excursion |
| `right_ankle_pitch` | 1 | 17.7 N·m p95 against a 7.0 cap |

The knee case is not explainable by load or velocity: in the same push, `left_knee_pitch` absorbed
a **larger** excursion (+0.587 rad) and more implied torque (26.4 vs 19.7 N·m) and did not fault,
while two ankles survived 8.8–9.8 rad/s. That is a marginal part, not a commanded-motion problem.

`right_ankle_pitch` also degraded measurably before failing — 4.17% stale-hold, 267 ms maximum gap,
mean sample age 10.2 ms against 4.9 ms elsewhere. It is the only joint in any capture with non-zero
stale-hold, which makes stale-hold a useful early-warning signal for a failing joint.

**Actions:** inspect or replace the `right_knee_pitch` encoder; inspect `right_ankle_pitch`. Because
both faults are on the same leg, check the shared infrastructure before replacing parts — wiring,
connectors, bus termination. That is cheaper to rule out than a part-by-part swap.

Every fault costs a power cycle plus recalibration, and the knee now gates how hard the robot can
be tested at all.

---

## 4. Do NOT do these

Each of these looks reasonable and is contradicted by the data.

* **Do not cut `rel_standing_envs` back from 0.30.** Report §5b names it as the prime suspect for
  the small stride. It cannot be: measC's stride **in sim** is 0.742 rad against smoothA's 0.849 —
  only 13% lower. If 30% standing environments had taught it to shuffle, sim would show a shuffle.
  The stride collapses between sim and hardware, so it is item 1, not the standing fraction. Cutting
  it would risk the 10× standing win to fix something it did not cause.
* **Do not tighten the joint-velocity hinge.** It sits at 8.0 rad/s. The knee faulted at
  **5.84 rad/s** while two ankles survived 8.8–9.8 rad/s in the same disturbance, so no velocity
  threshold separates them and no setting of this term would have prevented either fault.
* **Do not raise the ankle torque cap.** `ankle_pitch` demands 27.5 N·m p95 against a 7.0 cap and a
  19.76 N·m physical ceiling, peaking at 50.9 N·m. That is not a cap problem, and the ankle is
  3D-printed and has already broken once mechanically. Reduce demand instead.
* **Do not trust knee correlation as a sim-side predictor.** Sim said −0.708, hardware delivered
  −0.65 — the same as smoothA's −0.64. The sim gain did not transfer. Gait *frequency* did
  (1.70 Hz hardware against 1.54 predicted, versus smoothA's 1.05 against the same 1.54), so keep
  using that one.

---

## 5. Order of work

1. **M7** (§1), starting with effective stiffness. **Stiffness done 2026-09-28: refuted.** Step
   response / time constant (item 4) is next.
2. **Heading loop** (§2). Independent of M7 and the reason the robot cannot walk straight.
3. **Right leg inspection** (§3). Gates how hard anything can be tested.
4. Re-run a stand + walk capture once 1–3 are done, and add **both ankles** to the reconstructed-
   torque table — they are now a known failure point and were absent from earlier tables.

Once M7 returns, the training side has a concrete correction to apply rather than a guess. Until
then another 30 h run would be shaped by a plant we know is wrong by 3–5×.
