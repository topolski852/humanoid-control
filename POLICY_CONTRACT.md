# Policy Contract — legs-only stand-up (sim ↔ real)

This is the **single interface** the `humanoid-policy` trainer and this `humanoid-control`
runtime must agree on **exactly**. The machine-readable version is
[`configs/leg_policy_params.json`](configs/leg_policy_params.json) — import that on the
trainer side; don't hand-copy numbers. This doc explains it.

> Status: **`joint_order` / `obs` layout / `action_scale` / `default_pose` CONFIRMED**
> (2026-07-01) against the `humanoid-policy` trainer repo, whose legs env implements exactly
> this 45-dim obs, 12-dim action, `action_scale = 0.25`, canonical L→R joint order, and deep-squat
> `default_pose`. Per-joint gains/effort/Kt/gear/limits remain **device truth** pulled from the
> ESCs. The trainer can regenerate a matching machine-readable contract via
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
target = clip(action, ±100) * action_scale + default_pose
target = clamp(target, position_limit_lower, position_limit_upper)   # hard safety
prev_action = clip(action)   # pre-scale, fed back into the next obs
```
`action_scale = 0.25`.

## 5. Timing
`policy_dt = 0.04 s` (25 Hz policy). `control_dt = 0.004 s`. The daemon runs its own 200 Hz
PDO loop regardless; the policy streams targets at `policy_dt`.

## 6. Per-joint params (pulled from ESCs — device truth)
Use these **exact per-joint** values in sim (the joints are 3D-printed and individually
tuned; do **not** substitute uniform defaults). `kp`→firmware `position_kp`,
`kd`→`velocity_kp` (acts as Kd), `effort`→`torque_limit` (Nm).

| idx | joint | kp | kd | effort | Kt | gear | default_pose |
|--|--|--|--|--|--|--|--|
| 0 | left_hip_roll | 20.0 | 4.00 | 6.0 | 0.08958 | +15 | +0.0296 |
| 1 | left_hip_yaw | 10.5 | 0.50 | 12.0 | 0.08958 | −15 | +0.0038 |
| 2 | left_hip_pitch | 68.4 | 9.80 | 9.5 | 0.08958 | −15 | +0.9817 |
| 3 | left_knee_pitch | 27.0 | 2.45 | 6.0 | 0.08958 | +15 | +2.4435 |
| 4 | left_ankle_pitch | 18.0 | 2.00 | 6.0 | 0.06588 | +15 | −0.7854 |
| 5 | left_ankle_roll | 23.3 | 4.00 | 7.0 | 0.06588 | +15 | +0.0136 |
| 6 | right_hip_roll | 20.0 | 4.00 | 6.0 | 0.08958 | −15 | +0.0296 |
| 7 | right_hip_yaw | 20.0 | 1.00 | 6.0 | 0.08958 | +15 | +0.0038 |
| 8 | right_hip_pitch | 68.4 | 9.80 | 9.5 | 0.08958 | +15 | +0.9817 |
| 9 | right_knee_pitch | 30.0 | 1.22 | 6.0 | 0.08958 | −15 | +2.4435 |
| 10 | right_ankle_pitch | 20.0 | 0.50 | 6.0 | 0.06588 | −15 | −0.7854 |
| 11 | right_ankle_roll | 20.0 | 2.00 | 6.0 | 0.06588 | +15 | +0.0136 |

Notes:
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
- ✅ Sim PD gains = the per-joint firmware `kp/kd` above — implemented in the trainer's
  `HUMANOID_LITE_BIPED_SQUAT_CFG` (per-joint dicts; note the left/right asymmetry is device truth).
- ⚠️ **IMU prerequisite for hardware stand-up:** the trainer policy observes real
  `base_ang_vel` / `projected_gravity`, but this runtime still feeds the upright **stub** (§8).
  A stubbed base cannot close a balance loop — wire live base telemetry (`docs/DAEMON_SPEC.md §9`)
  before running a stand-up policy unsupported. Training can proceed in parallel.
- ℹ️ Joint names differ by a `leg_` prefix only (trainer `leg_left_hip_roll_joint` vs runtime
  `left_hip_roll_joint`); the mapping is **positional** and the trainer export strips the prefix,
  so no runtime change is required.
