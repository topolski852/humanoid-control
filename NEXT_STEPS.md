# Next steps — first supported "standing in place" test (WALK policy)

Goal: get the robot to **stand in place, supported**, driven by the **walk** policy at a zero
velocity command. This is the first end-to-end hardware test of a trained policy. Do it with the
robot **gantried/supported, E-stop armed, user present**.

## 0. What's already done (pull first)
- `git pull` **both** repos: `humanoid-control` (this) and `humanoid-policy`.
- ✅ **Policy↔device sign fix is in** (`config.py` `policy_frame_sign`, applied in `observation.py`
  + `action.py`). Right-leg `hip_roll`, `hip_yaw`, `ankle_roll` are flipped between the URDF
  (policy) frame and the device frame; left + all pitch joints are unchanged. Verified by an
  obs/action round-trip. **No further code change needed for signs.**
- ✅ **hip_pitch is NOT flipped** — sim and device limits match (`[-1.899, +0.983]`), which proves
  the sign convention agrees. The old "hip inversion" worry is resolved.
- The three exported policies live in `humanoid-policy/deploy/{standup,walk,squat}/`
  (`policy.onnx` + `leg_policy_contract.json` + `policy_latest.yaml`). See that repo's `deploy/README.md`.

## 1. Set the contract `default_pose` to the WALK policy's pose  ⚠️ REQUIRED
`configs/leg_policy_params.json` currently holds an **old** pose (hip_pitch +0.98, etc.). The runner
**ramps to `default_pose` and uses it as the policy's action offset**, so it MUST equal the pose the
walk policy was trained from (the stand pose), in the **device frame** (both legs same sign):

| joint (L and R identical) | default (rad) |
|---|---|
| hip_roll | **+0.1100** |
| hip_yaw | 0.0000 |
| hip_pitch | **−0.2400** |
| knee_pitch | **+0.8300** |
| ankle_pitch | **−0.5600** |
| ankle_roll | **−0.0700** |

Write these into each joint's `default_pose` in `configs/leg_policy_params.json`. (Provenance:
`humanoid-policy/deploy/walk/policy_latest.yaml`, with the 3 mirrored right joints negated to device
frame — hip_roll/hip_yaw/ankle_roll.)

> Note: `default_pose` differs **per policy** (standup uses the deep squat, walk/squat use this stand
> pose). For the full stand→walk→squat sequence, the runner must swap `default_pose` (and gains) when
> it switches policies. For now we're only running walk, so this single set is fine.

## 2. Gains: WALK trained with flat kp=20 / kd=2  ⚠️ CHECK
The walk policy was trained with **uniform kp=20, kd=2 on all 12 joints** (not the per-joint device
gains). The ESCs must match. Confirm the studio `humanoid_lite.json` / ESC `position_kp`/`velocity_kp`
are set to 20/2 for the legs before running walk. (The per-joint device gains in `POLICY_CONTRACT.md`
are for the *standup* policy, a later run.)

## 3. IMU / base state  ⚠️ IMPORTANT
The control code still feeds `UprightStubBaseState` (constant `projected_gravity=[0,0,-1]`,
`base_ang_vel=0`) in `runner.py`, `scripts/run_policy.py`, and `web/service.py`. The walk policy is a
**balance** policy — it needs **live** IMU `base_ang_vel` + `projected_gravity` to hold itself up.
- Now that the IMU is installed, wire `TelemetryBaseState` (see `base_state.py` + `POLICY_CONTRACT.md`
  §8 / `docs/DAEMON_SPEC.md §9`) and **validate the IMU signs by physically tilting the robot** before
  trusting it.
- With the **stub** and the robot **supported + upright**, the walk policy will hold a rough stance
  (it thinks it's perfectly balanced), which is an acceptable *plumbing* test — but it is NOT real
  balance and must not be trusted unsupported.

## 4. Deploy the walk ONNX + run
```bash
# copy the walk policy where run_policy.py can load it
cp ../humanoid-policy/deploy/walk/policy.onnx  ./legs.onnx
cp ../humanoid-policy/deploy/walk/policy.onnx.data ./   # if present

# read-only telemetry sanity first
python scripts/smoke_test.py --connect --seconds 3

# supported, E-stop armed: ramp to the (new) default pose and hold
python scripts/hold_pose.py --i-am-present

# then run the walk policy at ZERO velocity command (stand in place)
python scripts/run_policy.py --policy legs.onnx --i-am-present
#   feed command = [0, 0, 0] so it tracks zero velocity (stands, maybe steps in place)
```

## 5. What to look for
- **Legs move symmetrically** (left and right the same way) — confirms the sign fix. If a right-leg
  roll/yaw goes opposite the left, the sign map is wrong.
- Robot **holds the stand pose** without runaway; targets stay inside limits (runner clamps).
- If it drifts/tips: expected with the stub base — wire the IMU (step 3) before judging balance.

## Safety
Robot gantried/supported, user present, E-stop (ENTER/'q'/Ctrl-C or web E-STOP) armed. Never
unsupported until the IMU is validated and balance is proven.
