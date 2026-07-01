# humanoid-control — build brief

You are Claude Code working in `/home/nse/humanoid-control`, a **new** repo that
runs **on the robot's onboard PC**. Your job over this week: build the runtime that
makes this Berkeley Humanoid Lite robot **stand up (squat → stand)** using a learned
policy — **legs only, not walking**. Safety first; the robot is real hardware.

## 0. Read these first (do not skip)
- `docs/HANDOFF.md` — the source-verified reference: system architecture, firmware
  modes + control law, CAN protocol, the **22-joint model** (CAN ids, signed
  gear_ratio, phase_inverted, and the **mixed-motor map**: 150KV `MAD_M6C12` on the
  8 big leg joints, 200KV `MAD_5010` on ankles+arms), the full `JointConfig` schema,
  a **runnable `DaemonClient` position-control loop**, the actuator state machine,
  watchdog + calibration, and the **planned IMU telemetry contract**.
- `docs/DAEMON_SPEC.md` — the daemon as-built: UDP API (ports **9001** command /
  **9000** telemetry / **9002** priority E-stop), SDO map, control-loop timing,
  CLI flags, telemetry schema, IMU stub.
Everything you need to command the robot is in those two files. The daemon owns CAN;
you only speak UDP to it — never touch CAN directly.

## 1. Hard safety rules (non-negotiable)
- **Do not drive the robot beyond a supported/gantry dry-run without the user
  present and confirming.** Ask before any motion that could make it fall.
- Always clamp targets to each joint's `position_limits`; respect `torque_limit`.
- On start, **ramp slowly to a known default/crouch pose** — never step to a target.
- Wire up **E-stop** (`estop_all()`, port 9002) and a keyboard kill from day one.
- Bring-up order is **hold → small moves → policy**, never policy-first.

## 2. Reuse, don't reinvent
- **Daemon:** already here in `daemon/` (copied from humanoid-studio). Build it:
  `cd daemon && make`. Smoke-test: run it against the connected legs and confirm
  telemetry streams (see DAEMON_SPEC for CLI flags). Only one daemon may own CAN at
  a time — stop the humanoid-studio app's daemon first.
- **Python client:** vendor these three self-contained modules from
  `/home/nse/humanoid-studio/backend/humanoid/` into this repo (e.g. under
  `humanoid_control/daemon/`): `daemon_client.py`, `robot_config.py`, `actuator.py`.
  `daemon_client.py` only imports the other two (no other internal deps). They give
  you `DaemonClient`, `DaemonActuatorProxy` (`set_position`, `get_cached_state`,
  `enable(Mode.POSITION)`, `estop`, …), `RobotConfig`/`JointConfig`, `Mode`,
  `ActuatorState`. Prefer importing these over re-implementing the protocol.
- **Config (single source of truth):** the robot's live config is
  `/home/nse/humanoid-studio/configs/humanoid_lite.json` — it's updated by the
  humanoid-studio app during commissioning/tuning. Point the daemon's `--config`
  and `RobotConfig.from_json(...)` at that path; **do not fork it.**

## 3. What to build (milestones, in order)
1. **Daemon smoke test** — build + run; confirm all connected leg joints appear in
   telemetry and you can read `get_cached_state()` for each.
2. **Vendored client + config load** — load `RobotConfig`, connect `DaemonClient`,
   list the 12 leg joints in a fixed canonical order (define it once; it MUST match
   the trainer's joint order). Print live joint pos/vel at ~50 Hz.
3. **Hold-pose loop (NO policy yet)** — enable POSITION on the legs, ramp to a
   default pose, hold it at ~50–100 Hz using `set_position` per joint. This proves
   the full command path and your safety scaffolding. Test in support.
4. **Observation / action plumbing** — assemble the policy I/O without a real net:
   - Obs (legs-only, 12 joints): `[projected_gravity(3), base_ang_vel(3),
     joint_pos(12), joint_vel(12), prev_action(12)]` (add a zero command vector if
     the trainer uses one). Confirm ordering/signs against the config.
   - Action: `target = action * action_scale + default_pose`, clamp to limits, send
     per leg joint. Arms are **parked** (held at a fixed pose or left disabled).
   - Validate with an **identity/zero policy** (action=0 → holds default_pose).
5. **PolicyRunner** — load an ONNX/torch policy and run the loop at the trainer's
   `policy_dt`. Model the interface on Berkeley's
   `/home/nse/Berkeley-Humanoid-Lite/source/.../policy/rl_controller.py` (adapt —
   it assumes an IMU and its own joint order; don't copy blindly).
6. **Supported bring-up** — with the user, in a gantry, low torque: squat → stand.

## 4. Base state / IMU (pluggable)
The robot has **no IMU yet**. Design base-state as a swappable source:
- **Now:** a stub returning upright — `projected_gravity = [0, 0, -1]`,
  `base_ang_vel = [0, 0, 0]`.
- **Later:** the daemon will publish a `base` block in telemetry
  (`quaternion`, `angular_velocity`, `projected_gravity`) per the planned contract
  in the docs — switch the source to read that. Keep the interface stable so the
  policy code doesn't change when the IMU lands.
Note: a stubbed-upright base state can hold a pose but **cannot close a real balance
loop** — keep the robot supported until the IMU is integrated.

## 5. Notes for the trainer (running on a separate PC)
Whoever updates the policy trainer must match this runtime exactly:
- **Legs-only, 12 joints**, the canonical order you define here.
- **Mixed motors:** per-joint `torque_constant` (Kt) differs — 0.08958 (150KV) on
  hip roll/yaw/pitch + knee, 0.06588 (200KV) on ankles. Torque/gain scaling must use
  the right Kt per joint.
- **Signed gear ratios**, per-joint `position_limits`, `default_pose`, `action_scale`,
  and control dt must equal what this runtime uses.
- Obs must match §3.4 (including whether/where `projected_gravity` is used).
Write these down in a `configs/` or `POLICY_CONTRACT.md` so sim ↔ real stays aligned.

## 6. Keep the daemon in sync
`daemon/` is a copy of humanoid-studio's. If the daemon changes in either place,
sync the other (or propose extracting the daemon into its own shared repo). Don't let
them silently diverge.

---
Start at milestone 1. Ask the user before anything that moves the robot.