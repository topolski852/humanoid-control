# Policy Contract — legs-only stand-up (sim ↔ real)

This is the **single interface** the `humanoid-policy` trainer and this `humanoid-control`
runtime must agree on **exactly**. The machine-readable version is
[`configs/leg_policy_params.json`](configs/leg_policy_params.json) — import that on the
trainer side; don't hand-copy numbers. This doc explains it.

> Status: **`joint_order` / `obs` layout / `action_scale` / `default_pose` CONFIRMED**
> (2026-07-01) against the `humanoid-policy` trainer repo, whose legs env implements exactly
> this 45-dim obs, 12-dim action, `action_scale = 0.25`, canonical L→R joint order, and deep-squat
> `default_pose`. Effort/Kt/gear/limits remain **device truth** pulled from the ESCs; **gains now
> flow the other way** — the uniform kp=45 / kd=1.5 (§6) is set by the trainer contract
> and written *to* the ESCs. The trainer can regenerate a matching machine-readable contract via
> `scripts/rsl_rl/play.py` (writes `configs/leg_policy_contract.json`); diff it against
> `configs/leg_policy_params.json` to keep sim and hardware in sync.

## 1. Scope
- **Legs only, 12 joints.** Arms are parked/disabled (no CAN adapters connected).
- Task: squat → stand. Not walking.

## 2. Canonical joint order (indices 0–11)
Left leg then right leg, each `[hip_roll, hip_yaw, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]`:

```
 0 left_hip_roll     3 left_knee_pitch    6 right_hip_roll    9 right_knee_pitch
 1 left_hip_yaw      4 left_ankle_pitch   7 right_hip_yaw    10 right_ankle_pitch
 2 left_hip_pitch    5 left_ankle_roll    8 right_hip_pitch  11 right_ankle_roll
```
All 12-vectors (obs joint blocks, actions, targets, default_pose) use this order.

## 3. Observation (45)
Concatenated in this exact field order (matches Berkeley `rl_controller` and
`observation.py`, which asserts it against the contract):

| slice | field | notes |
|--|--|--|
| 0:3 | `command` | velocity command (3,) — zero for stand-up |
| 3:6 | `base_ang_vel` | rad/s, base frame (IMU; **stub = 0** for now) |
| 6:9 | `projected_gravity` | gravity unit vec in base frame (IMU; **stub = [0,0,−1]**) |
| 9:21 | `joint_pos − default_pose` | 12, canonical order (**relative to default**) |
| 21:33 | `joint_vel` | 12, canonical order |
| 33:45 | `prev_action` | 12, previous clipped (pre-scale) action |

## 4. Action (12)
```
target = clip(action, action_limit_lower, action_limit_upper) * action_scale + default_pose
target = clamp(target, position_limit_lower, position_limit_upper)   # hard safety
prev_action = clip(action)   # pre-scale, fed back into the next obs
```
`action_scale = 0.25`. The clip bounds are the trainer's exported `action_limit_lower/upper`
(**±4.0**), carried in the contract's `action` block. The runtime previously clipped at ±100,
letting the policy drive targets it never saw in training (observed |action| 10.17 on 2026-08-24);
fixed 2026-08-24 in `ActionMapper`.

## 5. Timing
`policy_dt = 0.04 s` (25 Hz policy). `control_dt = 0.004 s`. The daemon runs its own 200 Hz
PDO loop regardless; the policy streams targets at `policy_dt`.

## 6. Per-joint params
**Gains: uniform `kp = 45.0`, `kd = 1.5` on all 12 leg joints** (reverted 2026-08-24). These are the
bench-tuned values from `humanoid-tuner`, and they are the **sim↔real contract**, not a sim-only
knob: `humanoid-policy` trains on them (`1fb90de`) and the deployed walk policy (`6a6f171`) was
trained with them. `kp`→firmware `position_kp`, `kd`→`velocity_kp` (acts as Kd),
`effort`→`torque_limit` (Nm). Effort/Kt/gear remain **per-joint device truth** pulled from the ESCs.

> **kp45 vs kp20 A/B — resolved on hardware in favour of kp45 (2026-08-24).** The Berkeley-default
> retrain (uniform kp=20 / kd=2.0, `humanoid-policy d2d031f`, run `2026-08-21_21-20-23_kp20-berkeley`)
> **failed to stabilise on the robot** where kp45 did, so both the policy and the gains were reverted
> to the kp45 bundle. This matches the sim ranking (kp45 led kp20 by ~8% reward / ~4% episode length),
> which the A/B was expected to *contradict* if the hardware jitter were gain-related — it did not.
> Superseded bundles live in `humanoid-policy/deploy/walk/archive/`:
> `2026-08-18_kp45-bench-tuned/` (the bundle now restored and live),
> the kp20 Berkeley retrain, and `2026-07-18_actuator-model-asymmetric-gains/`
> (per-joint kp 10.5–68.4 / kd 0.5–9.8).

| idx | joint | kp | kd | effort | Kt | gear | default_pose |
|--|--|--|--|--|--|--|--|
| 0 | left_hip_roll | 45.0 | 1.50 | 6.0 | 0.08958 | +15 | +0.0296 |
| 1 | left_hip_yaw | 45.0 | 1.50 | 12.0 | 0.08958 | −15 | +0.0038 |
| 2 | left_hip_pitch | 45.0 | 1.50 | 9.5 | 0.08958 | +15 | +0.9817 |
| 3 | left_knee_pitch | 45.0 | 1.50 | 11.0 | 0.08958 | +15 | +2.4435 |
| 4 | left_ankle_pitch | 45.0 | 1.50 | 6.0 | 0.06588 | +15 | −0.7854 |
| 5 | left_ankle_roll | 45.0 | 1.50 | 7.0 | 0.06588 | +15 | +0.0136 |
| 6 | right_hip_roll | 45.0 | 1.50 | 6.0 | 0.08958 | −15 | +0.0296 |
| 7 | right_hip_yaw | 45.0 | 1.50 | 6.0 | 0.08958 | +15 | +0.0038 |
| 8 | right_hip_pitch | 45.0 | 1.50 | 9.5 | 0.08958 | −15 | +0.9817 |
| 9 | right_knee_pitch | 45.0 | 1.50 | 11.0 | 0.08958 | −15 | +2.4435 |
| 10 | right_ankle_pitch | 45.0 | 1.50 | 6.0 | 0.06588 | −15 | −0.7854 |
| 11 | right_ankle_roll | 45.0 | 1.50 | 7.0 | 0.06588 | −15 | +0.0136 |

Notes:
- ⚠️ The `default_pose` column above is the **original squat→stand** pose and is stale for the
  walk bundle. The machine-readable defaults are authoritative:
  `configs/leg_policy_params.json` (device frame, what the runner uses) and
  `policies/walk/leg_policy_contract.json` (URDF frame, as exported by the trainer). The walk
  policy's offset is the **stand** pose (hip_pitch −0.24, knee +0.83, ankle_pitch −0.56).
- ⚠️ **`left_knee_pitch` torque-direction inversion — no longer reproducing; cause NOT confirmed.**
  On 2026-08-21 it held on enable but ran away from any commanded target into a hardstop
  (err 1.32 rad, pinned 96.8% of the run). It was NOT a `gear_ratio` sign issue — `gear_ratio`
  enters the firmware loop squared (`motor_controller.c:389` and `:414`), so negating it cannot
  invert a loop. Since 2026-08-24 the joint tracks normally (0% pinned, err 0.21–0.48 rad,
  symmetric with the right knee) across four runs. **But the same joint threw `ERROR_ENCODER_FAULT`
  (0x2000) on 2026-08-24**, and an intermittently corrupt encoder frame would produce exactly the
  original symptom — holds on enable (zero error at seed), then diverges once commanded, in a
  direction set by the sign of the first error. Treat as latent until the encoder path is hardened.
- ⚠️ **Knee `torque_limit` raised 6.0 → 11.0 Nm (2026-08-24), right_ankle_roll 6.0 → 7.0.** The
  knees were torque-starved: a persistent *static* knee droop (mean err −0.136 / −0.198 rad,
  |mean|/rms ≈ 0.5) implied 6.1 / 8.9 Nm of steady extension demand against a 6.0 Nm cap, saturating
  38.7% / 45.1% of steps. Both knees sagged together (L/R pos corr **+0.91**) instead of stepping —
  the same policy gives **−0.84…−0.92** in an ideal loop, so the gait was never executable. Motor
  ceiling is Kt·I·gear ≈ 26.9 Nm and `left_hip_yaw` already runs 12.0 Nm on the identical actuator.
  **The trainer's `_CONTRACT_EFFORT` still says 6.0 and must be updated before the next retrain.**
- ℹ️ Historical: `right_ankle_pitch` was found with `torque_limit = 0` on the ESC (producing no
  torque at all) and that zero briefly reached flash. Corrected to 6.0 Nm and verified 2026-08-24.
- ℹ️ The `hip_pitch` sim↔hardware sign inversion flagged by the 2026-07-14 divergence report was
  addressed in the studio config by flipping both hip_pitch `gear_ratio` signs (left −15→+15,
  right +15→−15); `right_ankle_roll` also moved to −15. The gear column here follows device truth.
- **Kt (torque constant):** 0.08958 on the 8 big joints (150 KV M6C12: hip roll/yaw/pitch
  + knee), 0.06588 on the 4 ankles (200 KV 5010). Torque/gain scaling must use per-joint Kt.
- **gear_ratio is signed** — it sets output-side direction to match the URDF/policy frame.
  The runtime works in the daemon **display frame** (gear sign + `position_offset` already
  applied), which is the same frame the policy sees.
- **position_offset resets every power cycle** (re-calibrated per session). It's captured
  in the JSON for reference but is a runtime/session concern, not a sim parameter.
- **`right_ankle_roll` encoder is broken** — its `default_pose` mirrors `left_ankle_roll`;
  its live position is unreliable until the encoder is replaced.

## 7. default_pose / starting pose
The `default_pose` above is the policy starting pose: the current physical pose captured
per-joint (avg of left/right for symmetry; ankle_roll copied from left), **clamped to the
URDF `position_limits`**. It's a deep squat — the intended start for squat→stand. See
[`configs/policy_starting_pose.json`](configs/policy_starting_pose.json) for provenance.

## 8. Base state / IMU
No IMU yet. `base_ang_vel` and `projected_gravity` come from an upright **stub**
(`[0,0,−1]`, `0`). When the daemon publishes a `base` telemetry block (planned — see
`docs/DAEMON_SPEC.md §9`), swap `UprightStubBaseState` → `TelemetryBaseState`; nothing else
changes. A stubbed base can hold/track a pose but **cannot close a real balance loop** —
keep the robot supported until the IMU lands.

## 9. Trainer alignment (resolved 2026-07-01, `humanoid-policy`)
- ✅ `joint_order` and obs field order — locked; trainer's legs obs group matches exactly.
- ✅ `command` is the 3-vec velocity command, **zeroed for stand-up** (kept in the obs to preserve
  the 45-dim layout; the trainer's stand-up task uses a zero-range velocity command).
- ✅ `action_scale = 0.25`; `default_pose` **is** the action offset (Isaac `use_default_offset=True`),
  and the trainer's stand-up `init_state` = this deep-squat `default_pose`.
- ✅ Sim PD gains = the firmware `kp/kd` above. As of 2026-08-24 that is uniform **45.0 / 1.5**,
  set in the trainer's `HUMANOID_BIPED_WALK_CFG` / `HUMANOID_BIPED_SQUAT_CFG` (still per-joint
  dicts so re-asymmetrizing is a value edit). The old left/right asymmetry is retired — the ESCs
  are now flashed to the uniform bench-tuned values, so sim and hardware match.
- ⚠️ **IMU prerequisite for hardware stand-up:** the trainer policy observes real
  `base_ang_vel` / `projected_gravity`, but this runtime still feeds the upright **stub** (§8).
  A stubbed base cannot close a balance loop — wire live base telemetry (`docs/DAEMON_SPEC.md §9`)
  before running a stand-up policy unsupported. Training can proceed in parallel.
- ℹ️ Joint names differ by a `leg_` prefix only (trainer `leg_left_hip_roll_joint` vs runtime
  `left_hip_roll_joint`); the mapping is **positional** and the trainer export strips the prefix,
  so no runtime change is required.
