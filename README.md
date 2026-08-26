# humanoid-control

Onboard runtime control for the Berkeley Humanoid Lite robot. This repo runs **on
the robot's PC**: it loads a learned policy and drives the leg joints to stand
(squat → stand) by talking to the real-time CAN daemon over UDP.

```
policy runner (Python)  ──UDP :9001/:9000──▶  daemon (C++, owns CAN @200 Hz)  ──CAN──▶  ESCs (22 joints)
```

> **▶ Deploying a trained policy? Start with [`NEXT_STEPS.md`](NEXT_STEPS.md)** — the step-by-step for
> the first supported "stand in place" test with the walk policy (contract `default_pose`, gains, IMU,
> and the policy↔device joint-sign fix that's already landed).

## Layout
- `daemon/` — the real-time CAN daemon (C++). **Copied verbatim from
  [`humanoid-studio`](https://github.com/topolski852/humanoid-studio); keep the two
  in sync.** Build with `cd daemon && make`. Only one daemon may own the CAN bus at
  a time (don't run this and the humanoid-studio app's daemon simultaneously).
- `humanoid_control/` — the Python control package (this repo's core):
  - `daemon/` — **vendored** studio client (`daemon_client.py`, `robot_config.py`,
    `actuator.py` trimmed to `ActuatorState`). The only thing that talks to the daemon.
  - `config.py` — loads the policy contract (`configs/leg_policy_params.json`).
  - `layout.py` — **which limbs are attached to this machine** (see *Robot layout* below).
    Deliberately separate from the policy contract: the contract is the sim↔real interface
    the trainer agreed to, not a description of the hardware in the room.
  - `base_state.py` — pluggable base state (upright stub now → IMU telemetry later).
  - `observation.py` / `action.py` — obs assembly (45) and action→target mapping + clamps.
  - `policy.py` — Policy ABC + Zero/Onnx/Torch loaders.
  - `safety.py` — E-stop (port 9002), keyboard kill, ramp-to-pose.
  - `interface.py` / `runner.py` — leg read/write adapter and the `PolicyRunner` loop.
- `scripts/` — `smoke_test.py` (read-only telemetry), `hold_pose.py` (M3, motion),
  `run_policy.py` (M5, motion). Motion scripts require `--i-am-present`.
- `configs/` — `esc_pull_latest.json` (per-joint config pulled from the ESCs),
  `policy_starting_pose.json`, `leg_policy_params.json` (the trainer contract).
- `POLICY_CONTRACT.md` — the sim↔real interface the trainer must match.
- `docs/HANDOFF.md`, `docs/DAEMON_SPEC.md` — authoritative firmware/CAN/daemon reference.

## Quick start
```bash
cd daemon && make                 # build the daemon
# ensure the leg CAN adapters are powered (can_left_leg / can_right_leg come up)
./build/humanoid_daemon --config /home/nse/humanoid-studio/configs/humanoid_lite.json &
cd .. && pip install -r requirements.txt
python scripts/smoke_test.py --connect --seconds 3    # read-only telemetry (no motion)
```
Motion (user present, robot supported/gantried, E-stop = ENTER/'q'/Ctrl-C):
```bash
python scripts/hold_pose.py --i-am-present            # M3: ramp to default_pose and hold
python scripts/run_policy.py --policy legs.onnx --i-am-present   # M5: run a trained policy
```

## Wireless web control
Command the robot from any PC/phone on the same WiFi — no monitor, cables, or held-open SSH.
A long-lived FastAPI server (`humanoid_control/web/`) wraps the same lifecycle the scripts
drive (connect → arm → ramp/hold → run policy) and serves a React control page over the LAN.

```bash
pip install -r requirements.txt        # now includes fastapi + uvicorn
cd app && npm install && npm run build && cd ..   # build the UI → app/dist (Node 18+)
python -m humanoid_control.web          # binds 0.0.0.0:8000
# open http://<robot-ip>:8000  in a browser on this PC
```
The page streams live joint telemetry (`/ws/telemetry`, ~20 Hz), has an always-visible **E-STOP**,
an **Arm** toggle (the web equivalent of `--i-am-present`), and Ramp/Run/Stop controls. The
robot's 200 Hz CAN loop and 25 Hz policy run entirely on the onboard PC — the browser is a
supervisor, not in the control loop, so LAN latency doesn't affect control quality.

**Deadman (wireless safety).** Motion requires a live heartbeat over `/ws/control`; if the
browser tab closes or WiFi drops mid-motion, the server auto-fires E-STOP. A robot-local
**gamepad deadman** (Bluetooth Xbox → auto-kill on battery-death / signal-loss) is prepared in
`humanoid_control/web/gamepad.py` but **disabled by default** (enable with `HUMANOID_GAMEPAD_ENABLE=1`).

Env: `HUMANOID_WEB_HOST`/`HUMANOID_WEB_PORT`, `HUMANOID_CONFIG` (robot config),
`HUMANOID_LAYOUT` (robot layout file), `HUMANOID_WEB_PASSWORD` (optional login),
`HUMANOID_POLICY_DIR` (checkpoint list). For auto-start on boot, see
[`deploy/`](deploy/README.md).

> **Which robot config?** There is more than one copy of `humanoid_lite.json` on a typical
> machine and they drift — the studio GUI writes to `~/.config/humanoid-studio/`, the repo
> checkout keeps its own. `resolve_robot_config_path()` searches `$HUMANOID_CONFIG` → the
> studio user config → the repo copy and **logs which one won** at startup. Point the daemon's
> `--config` at the same file, or you will run gains you did not set.

## Robot layout (which limbs are attached)
The app drives whatever is actually plugged in — legs on a gantry, a single arm on the bench,
or the full robot. Tick the limbs in the web UI's **Settings** tab; the choice is written to
`~/.config/humanoid-control/robot_layout.json` and read at startup, so each machine (this bench
PC, the torso PC later) keeps its own answer. With no file the default is **both legs**, which
is exactly the behaviour that predates the setting.

The layout is the joint set for everything downstream: telemetry, the health check that gates
`connect`, calibration, and what the Robot tab draws. That is the point — with the legs
unpowered, offline leg joints are the expected state, not a fault.

```bash
# the same thing over the API
curl -X PUT localhost:8000/api/layout -H 'Content-Type: application/json' \
     -d '{"enabled":["left_arm"],"imu_expected":true}'
```

**Motion still requires both legs.** Every motion path here commands the twelve contract leg
joints, so Hold / Run / Manual are refused on an arm-only layout with a message saying so. An
arm layout is a look-and-calibrate configuration: the Robot tab draws it live from the encoders
and the Calibration tab can zero it, but nothing here drives an arm yet.

**Arm joints are drawn raw** (`policy_frame_sign = +1`). There is no trained arm policy, so
there is no frame to reconcile to, and inventing one would hide exactly what the visualizer
exists to expose. Move a joint by hand: if the drawing moves the *opposite* way the
`gear_ratio` sign is wrong; if it moves the right way but sits in the wrong *place* that is a
zero offset. The Robot tab's **vs URDF** column says which of the two each joint's range implies.

Note the device names are authoritative and differ from the URDF asset: the arm's fifth joint
is `{side}_wrist_yaw_joint` on the hardware and `elbow_roll` in the URDF. The mapping lives in
`scripts/gen_viz_kinematics.py`.

## Safety (non-negotiable)
Never drive the robot beyond a supported/gantry dry-run without a human present. Targets
are always clamped to per-joint `position_limits`; the runner ramps to the default pose on
start (never steps); E-stop (priority port 9002) and a keyboard kill are always armed. The
upright base-state stub can hold a pose but **cannot** close a real balance loop — keep the
robot supported until the IMU lands.

## Config is shared, not forked
The robot's live per-joint config (gains, offsets, gear signs, limits — updated by
the humanoid-studio app during commissioning/tuning) is the single source of truth
at `/home/nse/humanoid-studio/configs/humanoid_lite.json`. Point the daemon
(`--config`) and `RobotConfig` at it so tuning done in the app is reflected here.

## Status
- ✅ M1 daemon smoke test (all 12 legs enumerate, IDLE, live telemetry, fw v3.2.0)
- ✅ M2 vendored client + config load + canonical order + 50 Hz telemetry stream
- ✅ M4 obs/action plumbing validated offline (zero-policy identity, safety clamp)
- ✅ M5 PolicyRunner + ONNX/Torch loaders (code complete; untested with a real net)
- ⏳ M3 hold-pose and M6 supported squat→stand — **require the user present + support**
- Trainer contract exported to `POLICY_CONTRACT.md` / `configs/leg_policy_params.json`