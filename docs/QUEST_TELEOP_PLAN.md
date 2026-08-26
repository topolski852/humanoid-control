# Meta Quest 3 arm teleop — implementation plan

Status: **plan only, nothing implemented.** Target: drive the bench left arm from a Quest 3
controller in passthrough, and restructure the dashboard's Control card into
Control / Control method / Quest.

Written against `main` @ `444e199`. Reference reading: `README.md`, `POLICY_CONTRACT.md`,
wiki page *Arm Joint Frames and Calibration*, and the `xr_teleoperate` clone at
`/home/nse/xr_teleoperate`.

---

## 1. What is actually in `xr_teleoperate`

Read from your clone at `/home/nse/xr_teleoperate` (`https://github.com/Nbot07/xr_teleoperate`)
— a fork of `unitreerobotics/xr_teleoperate`. Default branch `main` is upstream v1.6
(`7dc9aa1`, 2026-05-28).

> **Note on the submodule.** The findings below about `televuer` come from initialising
> `teleop/televuer`, which is **not** checked out in your clone. To read it yourself:
> `git submodule update --init teleop/televuer`. Nothing in this plan depends on it staying
> initialised — it is reference material, not a dependency.

**Branches carrying your own work** (all descend from upstream, newest first):

| Branch | Tip | Contains |
|---|---|---|
| `teleop-safety` | 2026-08-10 `ba85233` | **superset of everything below** + the gantry-free safety framework (`teleop/safety/`), sim-validated scenario harness, self-collision check |

Your local checkout of `teleop-safety` is one commit **ahead** of the remote (`4047d69`,
*"safety: clear the hand/knee contact in the squat pose"* — unpushed). Nothing in this plan
reads that commit; noted only so the tip hashes above are not confusing.
| `jetson-integration` | 2026-08-06 | merge of `onboard-deployment` into `hand-arm-walk-nod` |
| `onboard-deployment` | 2026-08-06 | cable-free operation from the robot's own computer |
| `hand-arm-walk-nod` | 2026-08-04 | double-nod walk toggle, arms-as-joystick, finger freeze while walking |
| `inspire-controller-grasp` | 2026-07-31 | controller-mode finger grasp on G1 + inspire_dfx |
| `g1-sw1.5.3-loco-fix` | 2026-07-12 | FSM 1→4→501 walk fix |

`teleop-safety` is the branch to read. It contains all the others.

### The Quest work is real, but it is not ours to integrate

It is **not a stub** — it is a complete, working XR teleop stack. It is also **entirely
Unitree**. Every consumer of the XR data talks Unitree DDS (`rt/arm_sdk`, `LocoClient`,
`GetFsmId`), solves IK with pinocchio/casadi against a G1/H1/R1 URDF, and drives Dex/Inspire/
BrainCo hands. Nothing on the robot side transfers to a Berkeley Lite arm on a UDP daemon.

What *is* relevant is the input half, and that lives in one submodule.

### Transport: `televuer` → `vuer` → WebXR

- Submodule `teleop/televuer` → `unitreerobotics/televuer` v4.0.0, pinned at `766de45`.
- `televuer` is a thin wrapper over **[Vuer](https://github.com/vuer-ai/vuer) 0.0.60** — a
  Python asyncio HTTPS + WSS server on **port 8012** that serves a WebXR page.
- **The Quest is a browser client.** You open
  `https://<host>:8012?ws=wss://<host>:8012` in the Quest's browser and enter immersive
  mode. There is no APK, no OpenXR runtime on the robot, no Unity.
- WebXR requires a secure context, so a **self-signed cert is mandatory**
  (`~/.config/xr_teleoperate/{cert,key}.pem`).
- Vuer runs in its own `multiprocessing.Process`. Browser events land in three async
  handlers — `CAMERA_MOVE`, `CONTROLLER_MOVE`, `HAND_MOVE` — which write into
  `multiprocessing.Array`/`Value` shared memory. The consumer **polls** `get_tele_data()`;
  there is no queue and no back-pressure.

### Data it emits (controller mode)

From `televuer/src/televuer/tv_wrapper.py`, `TeleData`:

- `head_pose`, `left_wrist_pose`, `right_wrist_pose` — 4×4 SE(3), **metres**.
- Per controller: `trigger` (bool) + `triggerValue` (float, **inverted** to 10.0→0.0 so it
  matches the hand-tracking pinch convention), `squeeze` + `squeezeValue` (0→1),
  `thumbstick` (bool) + `thumbstickValue` (2-vector), `aButton`, `bButton`.
- `motion_data_ready` (bool).
- Raw basis is **OpenXR** (y up, z back, x right); `televuer` converts to a robot basis
  (z up, y left, x front) via `T_ROBOT_OPENXR`. In controller mode the pose needs no
  initial-pose fixup — the controller grip pose already matches the Unitree arm URDF
  convention.
- Rate: whatever the browser's XR render loop produces (72–120 Hz on a Quest 3). The
  `fps=60` kwarg on the handlers is a hint; nothing enforces it server-side.

### Three findings that decide the architecture

1. **The wrist pose is pre-baked into a G1 waist frame.**
   `transform_IPunitree_Brobot_world_arm_to_head_then_waist()` subtracts the head pose, then
   adds hardcoded `+0.15 m` in x and `+0.45 m` in z to get to the G1's waist joint, with
   `arm_reference_mode="head_yaw"` by default. Those are G1 body dimensions. We would be
   undoing this transform before we could use it.

2. **There is no liveness signal of any kind.** No timestamp, no sequence number, no frame
   counter. `motion_data_ready` is a **one-way latch** — set `True` on the first event and
   never cleared. Both `on_controller_move` and `on_hand_move` wrap their whole body in a
   bare `except: pass`. If the headset disconnects, the browser tab dies, or WiFi drops, the
   last pose sits in shared memory forever and every field reads healthy. A polling consumer
   cannot tell "operator holding still" from "link dead".

   Your own fork already found this and worked around it in
   `teleop/safety/gestures.py` — `HandGestureTracker` detects staleness by
   **value-change detection** (`np.allclose` against the last sample), with the comment
   *"Quest hand tracking silently repeats the last pose when hands leave the cameras' FOV."*
   That idea is worth keeping. The code is not — it is shaped around 25-point hand landmarks.

3. **Dependency conflict.** `televuer` pins `numpy<2.0.0` and `vuer==0.0.60`. This venv has
   **numpy 2.5.1**, which `onnxruntime` (the policy path) is built against. Installing
   `televuer` into `humanoid-control` downgrades numpy across the whole runtime.

### Verdict

We are in **passthrough with no image streaming**, which is the half of `televuer` that
carries all its weight (ZMQ/WebRTC stereo planes, opencv, aiortc, shared-memory frame
writers). Of what is left we use the controller pose and the trigger. Adapting it means
carrying a numpy pin, a pinned 2-year-old `vuer`, a second process, a G1 body transform to
undo, and a transport with no liveness — to obtain roughly 200 lines of WebXR JavaScript.

**We build our own bridge**, and keep `xr_teleoperate` as a reference for two things: the
OpenXR→robot basis change (verified below), and the change-detection watchdog idea.

---

## 2. The transport we will build

### Shape

```
Quest 3 browser (WebXR, passthrough)
        │  wss://<robot>:8443/ws/xr        ← TLS, self-signed
        ▼
humanoid-control FastAPI  ── QuestSource ──► ControlService
        │                                    (run gate, pose command, liveness)
        └── serves the WebXR page from app/dist/xr/
```

The page is **served by the existing server** — same origin, same auth token, one process.
`humanoid-control` runs unchanged with no Quest present: the endpoint and page exist but
nothing connects, exactly as `HUMANOID_GAMEPAD_ENABLE` gates the gamepad thread today.

### TLS without breaking the LAN UI

WebXR needs HTTPS. Do **not** move `:8000` to TLS — that breaks every existing bookmark and
the `deploy/` unit. Instead run a **second uvicorn `Server` in a thread on `:8443` with
`ssl_certfile`/`ssl_keyfile`, serving the same ASGI app**. Plain HTTP on 8000 for the LAN UI
is unchanged; the Quest uses 8443. Gate on `HUMANOID_QUEST_ENABLE=1` plus
`HUMANOID_QUEST_CERT`/`HUMANOID_QUEST_KEY` (default `~/.config/humanoid-control/`).
Generation is one `openssl req -x509` line; document it in `deploy/`.

The Quest browser will show a certificate interstitial once per cert. That is acceptable and
is what `televuer` does too.

### Wire format (client → server, one message per XR frame)

JSON over the websocket. Text frames, ~60 Hz.

```json
{
  "seq": 12345,
  "t": 88123.4,
  "session": "a3f91c",
  "head":  { "p": [0.0, 1.62, 0.0], "q": [0,0,0,1], "tracked": true },
  "left":  { "p": [-0.3, 1.1, -0.35], "q": [0,0,0,1], "tracked": true,
             "trigger": 0.83, "squeeze": 0.0, "stick": [0.0, 0.0],
             "a": false, "b": false, "stickPress": false },
  "right": { "...": "same shape, or null when absent" }
}
```

- `seq` — monotonic, from the client, starts at 0 each XR session. **The liveness primitive.**
- `t` — `performance.now()` in ms. Client clock; used only for jitter diagnostics, never for
  control decisions (no clock sync).
- `session` — random per `XRSession`; a change means the operator re-entered immersive mode
  and every anchor must be discarded.
- `p` — position in metres, `local-floor` reference space. `q` — orientation quaternion, xyzw.
- `tracked` — `false` when `XRFrame.getPose()` returns null **or** `XRPose.emulatedPosition`
  is true.
- `trigger` — `gamepad.buttons[0].value`, 0→1, **not** inverted. (`televuer`'s 10→0 inversion
  exists only to match its hand-pinch convention; we have no reason to copy it.)

Server → client is a single small status frame at ~5 Hz (`state`, `armed`, `gate`, `reason`)
so the page can render a HUD in the headset. Not on the control path.

### Frame conversion (verified against `televuer`)

WebXR `local-floor`: `+X` right, `+Y` up, `−Z` forward. Robot URDF: `+X` forward, `+Y` left,
`+Z` up.

```
x_robot = −z_webxr
y_robot = −x_webxr
z_robot = +y_webxr
```

This is exactly `televuer`'s `T_ROBOT_OPENXR`:
```
[[ 0, 0,−1],
 [−1, 0, 0],
 [ 0, 1, 0]]
```
Cross-checked row by row. Good — an independent derivation agreeing with a working system is
worth more than either alone.

---

## 3. Pose → arm: what is actually controllable

### The IK is position-only. Say so out loud.

`ArmChain.jacobian()` is **(3, n)** — a position Jacobian of the tool point. `ik_step()`
solves `dq = Jᵀ(JJᵀ + λ²I)⁻¹ dx` for a 3-vector `dx`. The controlled point is the **wrist
pivot**, and `arm_kinematics.py` documents why: the fifth joint is an inline twist that moves
the wrist ~0.1 cm/rad, so including the hand would add a near-singular column.

So the mapping is:

- **Controller position → hand position.** 3 DoF. Real.
- **Controller orientation → nothing**, except optionally its roll about its own forward axis
  → `wrist_yaw`, layered on as a direct joint rate exactly as `right_x` is today.
- The arm cannot hold hand *orientation*. It has 4 positioning joints and one twist. Any UI
  that draws a 6-DoF gizmo would be lying.

**Recommendation: ship position-only first.** Add the roll→`wrist_yaw` mapping as a second
step once position feels right, behind a tuning flag. Reason: wrist twist and hand position
are the two things that will fight each other during the first bench session, and debugging
them together is how you end up unable to tell a sign error from a zero error — the exact
trap the wiki page warns about.

### Relative, clutched, anchored at trigger press

Absolute mapping is wrong here for the ordinary reason: the operator's hand and a 29 cm arm
bolted to a bench do not share a workspace, and any absolute map makes most of the operator's
reach unreachable and most of the arm's reach unreachable.

The clutch falls out of the existing design **for free**:

- The trigger is already the run gate. On engage, `_deadman_worker` already calls
  `teleop.reset(q_now)` — "Seeding every engage is what stops the arm jumping to wherever the
  target was left."
- So: **trigger press = deadman engage = clutch anchor.** We additionally latch the
  controller's pose at that instant as `p_anchor_xr`.
- Release → arm goes IDLE (unchanged), anchor discarded. Re-press → re-anchor at wherever the
  controller now is. That is a mouse-lift ratchet, for free, with no new button.

Per tick, the desired hand position is:

```
p_hand_desired = p_hand_at_anchor + scale · R_align · (p_controller − p_anchor_xr)
```

### Alignment: fixed yaw offset, not head-relative

`televuer` defaults to `arm_reference_mode="head_yaw"` — the arm reference rotates with the
operator's head yaw. That makes sense for a humanoid whose head you are wearing. It is wrong
for a bench arm: turning your head to look at the arm would rotate the mapping under you.

**Use a fixed yaw offset**, `HUMANOID_QUEST_YAW_DEG`, default 0, applied as `R_align =
Rz(yaw)`. The operator stands in a known place facing the bench. If the mapping feels rotated,
it is one number to change, and it stays changed. Capture-headset-yaw-at-anchor is a plausible
future refinement; do not start there.

### Scale

`ArmChain.reach_bounds()` on this arm gives a shell of roughly 0.1–0.3 m about the shoulder.
A comfortable human arm sweep is ~0.6 m. **Default `scale = 0.5`**, configurable. At 1:1 the
operator runs out of arm before running out of room and spends the session against
`_leash`/reach clamps.

### Where this lands in the code

Add a **fourth frame** to `TeleopTuning.frame`: `"pose"`, and a sibling method on `ArmTeleop`:

```python
def step_pose(self, q_measured, p_desired, dt, *, creep=False):
    """Servo the hand toward an absolute desired position. Same solver, same clamps."""
```

It computes `dx_des = p_desired − chain.tool(q)`, clamps `|dx_des|` to `speed·dt`
(`speed_normal` / `speed_creep` — so the existing "how fast may the hand move" numbers still
mean what they say), and calls the **same `chain.ik_step(...)`** with the same damping,
posture null-space drift and `max_step` cap. The reach-shell clamp from `_leash` applies to
`p_desired` before the solve.

This deliberately reuses everything. No new solver, no new limit handling, no new clamping
loop — a bug in `ik_step` stays a single bug. The existing `step()` is untouched, so the
gamepad path cannot regress.

`info` gains `"frame": "pose"` plus `tracking_error_m` (distance the hand is behind the
commanded point) — that number is the honest answer to "why does it feel laggy".

### Flag: the arm session ticks at 25 Hz

`_deadman_worker` uses `dt = self.contract.policy_dt` = **0.04 s**. That is the *leg policy's*
rate, inherited by the arm session for no arm-specific reason (there is already a comment in
`arm_teleop.py` about a limit that "silently depended on the loop rate: sized for 50 Hz, it
halved when the session turned out to run at the policy's 25 Hz").

25 Hz is tolerable for sticks. For a 6-DoF pose input it is a visible 40 ms of staircase.
**Recommendation:** give the arm branch its own tick rate (`HUMANOID_ARM_HZ`, default 50) and
leave the leg path on `policy_dt`. This is a small, separable change — but do it *before* the
first Quest bench session, or you will spend that session tuning gains against a rate problem.

---

## 4. Backend changes

### 4.1 New file: `humanoid_control/web/xr.py`

`QuestSource` — the Quest's equivalent of `GamepadDeadman`, but websocket-driven rather than
thread-driven.

Responsibilities:
- Parse and validate each frame; reject malformed ones **loudly** (counter surfaced in
  telemetry), never `except: pass`.
- Track liveness (§5) and own the `seq` bookkeeping.
- Convert WebXR → robot frame, apply `R_align` and `scale`.
- Latch/discard the anchor on trigger rising/falling edge and on `session` change.
- Call `service.set_run_gate(...)`, `service.set_arm_pose_command(...)`,
  `service.mark_source_alive("quest")`.
- Maintain a status dict for telemetry: connected, seq rate, last-frame age, tracked,
  trigger value, anchor state, dropped-frame count, whether it currently owns input.

### 4.2 `humanoid_control/web/server.py`

- `@app.websocket("/ws/xr")` — auth via the same `_ws_authed` token check. On accept,
  `service.attach_quest()`; on close, `service.detach_quest()`.
- TLS listener thread on `:8443` (§2), gated on `HUMANOID_QUEST_ENABLE`.
- Serve `app/dist/xr/` (Vite emits it as a second entry point).

### 4.3 `humanoid_control/web/service.py`

**a. Pose command channel.** Alongside `_arm_command` (the stick quad):

```python
self._arm_pose_command = None   # (p_desired_robot: np.ndarray(3), seq: int) | None
def set_arm_pose_command(self, p_desired, seq): ...
```
Guarded by the existing `_command_lock`. Kept separate from `_arm_command` rather than
overloaded — the two are different kinds of thing (velocity vs position) and conflating them
is how a stale stick value ends up interpreted as a position.

**b. Per-source liveness — this fixes an existing defect.**

Today `deadman_ok()` is:
```python
self._control_clients > 0 and (now - self._last_heartbeat) < _DEADMAN_TIMEOUT_S
```
`_control_clients` and `_last_heartbeat` are **global across all sources**. The browser's
`/ws/control` heartbeat and the gamepad's `mark_heartbeat()` write the same variable. So with
the browser page open, a gamepad that stops beating still reads `deadman_ok() == True`.
(Gamepad loss happens to be covered by a *separate* path — `GamepadDeadman._run` fires
`gamepad-absent` when `_find_device()` returns None — but that is a second mechanism, not this
one working.)

Adding the Quest to the same global would make it worse: **a stalled Quest would read healthy
because the browser tab is beating.**

Replace with a registry:
```python
self._sources: dict[str, float] = {}       # source name -> last monotonic heartbeat
def mark_source_alive(self, source: str) -> None
def source_alive(self, source: str) -> bool
def deadman_ok(self) -> bool:              # the ACTIVE input source must be alive
```
`deadman_ok()` becomes "the source currently holding the input token is alive", with the
browser `/ws/control` still required for web-driven `hold`/`run_policy` sessions. Keep the
existing 1.0 s `_DEADMAN_TIMEOUT_S`. This is a contained change — three call sites — and it
makes the Quest's liveness independently checkable, which is the whole point.

**c. Input arbitration.** New state, modelled directly on the existing `_control_mode`:

```python
self._input_source = "xbox"                # "xbox" | "quest" | "web"
def available_input_sources(self) -> list[str]
def set_input_source(self, source: str) -> None
```
`set_input_source` raises `ControlError(..., 409)` while `self._state in _ACTIVE_STATES` —
byte-for-byte the rule `set_control_mode` already enforces ("Disarm before switching control
mode"). **Only the token holder's `set_run_gate` / command writes are honoured**; writes from
a non-owner are dropped and counted, and the count is surfaced in telemetry so an ignored
controller is *visible* rather than mysterious.

E-STOP is never gated by the token. `trigger_estop` stays reachable from every source,
including a Quest that does not own input.

**d. `_deadman_worker` arm branch.** One branch, in the block that currently reads
`self._arm_command`:

```python
if self._input_source == "quest":
    p_des, seq = self._arm_pose_command
    q_target, info = teleop.step_pose(q_now, p_des, dt, creep=...)
else:
    q_target, info = teleop.step(q_now, cmd, dt, creep=...)
```
Everything around it — engage/`teleop.reset`, `group.enable_position()`, `send_targets`,
`check_health`, release → `group.idle()`, the `finally: group.idle()` — is **untouched**. The
Quest inherits the whole safety envelope rather than reimplementing it, which is the same
bargain arm teleop already made with the gamepad.

**e. Recorder.** `ArmRunRecorder.record()` gains optional `xr=` — the raw controller pose,
`seq`, trigger value, `tracked`, and frame age. Without it, "the arm did something I did not
expect" is unanswerable for the Quest path, and the recorder exists precisely because the arm
has no policy to fall back on.

### 4.4 Telemetry

`telemetry_snapshot()` gains, next to the existing `"gamepad"` block:

```python
"input_source": self._input_source,
"input_sources": self.available_input_sources(),
"quest": {"enabled": ..., "connected": ..., "session": ...,
          "seq": ..., "hz": ..., "age_ms": ..., "tracked": ...,
          "trigger": ..., "anchored": ..., "owns_input": ...,
          "dropped": ..., "reason": ...},
```

### 4.5 Routes

- `POST /api/input_source` `{source}` → `set_input_source`.
- `available_input_sources` is already implied by the telemetry snapshot; no separate GET.

---

## 5. Liveness, tracking loss and timeouts

Per the decision: **stall → IDLE (recoverable), loss → E-STOP (latched)**.

### The run gate is recomputed from scratch every tick

```
run_gate = fresh_frame AND tracked AND (trigger ≥ threshold)
```

All three, evaluated **server-side**, from **the same frame**. The trigger is never a latched
boolean held across frames — that is the specific failure where a stale message leaves
"trigger held" true while the operator has already let go and walked away.

### The ladder

| Condition | Threshold | Action | Recovery |
|---|---|---|---|
| No new `seq` | **200 ms** | clear run gate → arm IDLE | automatic on next frame |
| No new `seq` | **1.0 s** (`_DEADMAN_TIMEOUT_S`) | `trigger_estop("quest-timeout")` | reconnect |
| `tracked == false` | immediate | clear run gate → IDLE | automatic when tracking returns |
| `seq` advancing but pose **bit-identical** > 500 ms while gate held | 500 ms | clear run gate → IDLE + loud warning | automatic |
| websocket closes while Quest owns input **and** state ∈ `_ACTIVE_STATES` | immediate | `trigger_estop("quest-disconnect")` | reconnect |
| `XRSession` ends / `visibilitystate != "visible"` (headset removed) | immediate | client sends explicit `end`/`blur` frame → clear run gate → IDLE | re-enter XR |
| `session` id changes | immediate | discard anchor, clear run gate | re-press trigger |

200 ms is ~12 missed frames at 60 Hz and 5 arm ticks at 25 Hz — long enough to ride out
ordinary WiFi jitter, short enough that the arm stops before the operator has finished
noticing. 1.0 s reuses the number the rest of the system already uses for controller loss.

### Why the frozen-value check earns its place

`seq` catches a dead *client*. It does not catch a client that is alive and looping but whose
pose source has frozen — which is exactly the Quest failure `xr_teleoperate`'s own
`gestures.py` documents ("silently repeats the last pose when hands leave the cameras' FOV").
Real 6-DoF tracking jitters at the micrometre level; a *bit-identical* pose across many frames
means a repeated buffer, not a steady hand. Compare with `atol=0` — exact equality — so a
genuinely motionless-but-live controller never trips it.

### Headset removal

Quest 3 proximity sensor → `XRSession.visibilityState` goes `hidden`. The page listens for
`visibilitychange` on the session and sends one explicit frame with `tracked: false` before it
stops rendering (the XR animation loop pauses when hidden, so we cannot rely on the 200 ms
stall alone to be *fast*, though it is the backstop). **This signal does not exist in
`televuer`'s `TeleData` at any layer** — it is a direct benefit of owning the page.

### Client-side belt and braces

The page also runs its own watchdog: if it cannot send for 200 ms (websocket
`bufferedAmount` climbing, or `send()` throwing), it stops sending entirely rather than
resuming with a backlog of stale frames. A queue of old poses arriving in a burst after a
stall is worse than silence.

---

## 6. Arbitration — what happens in each case

**Quest connects while the gamepad is armed.** The websocket is **accepted**. The Quest gets
telemetry and its card populates — tracking quality, seq rate, trigger — so the operator can
confirm the Quest works *before* committing to it. It does **not** get the input token: its
`set_run_gate` and pose writes are dropped, counted, and the Quest card reads
*"connected — Xbox is the active control method"*. To hand over: disarm, switch method,
re-arm.

Rejected alternatives, and why:
- *Silently transfer authority* — two live sources both believing they are driving. No.
- *Refuse the connection* — then you cannot tell whether the Quest is working until you have
  already disarmed and committed, which encourages exactly the "just try it and see" loop you
  do not want with an arm powered.

**Gamepad unplugged while the Quest is armed.** Untouched — `GamepadDeadman` still fires
`gamepad-absent` if a session is live. Consider narrowing that to fire only when the gamepad
*owns* input; as written it would E-STOP a healthy Quest session because an unrelated pad's
battery died. **Flagging this as a real bug the new token makes visible.**

**Both connected, neither armed.** Fine. The token only matters at arm time and during a
session.

**Web-driven `hold` / `run_policy`.** These are `"web"`-sourced and already require the
browser `/ws/control` deadman. `_preflight_motion()` keeps gating on that. Unchanged.

---

## 7. Capability gating — I recommend *not* adding one

The question was whether Quest control should be a new layout capability. **No**, and I think
adding one would quietly break the property it is meant to preserve.

`RobotLayout.capabilities` is a **derived** property — computed from which limbs are enabled:

```python
if self.has_both_legs: caps.append("walk")
if self.arms:          caps.append("arm_teleop")
if self.enabled:       caps.append("pose")
```

The layout file records **which limbs are physically attached to this machine**. It is
deliberately not a settings file — the module docstring is explicit that joint membership is a
catalog in code precisely so "a hand-edit can't invent a joint the daemon has never heard of".

A Quest is not a limb. There is nothing about it to derive from `enabled`, so a `quest`
capability would have to be a *stored flag* — the first one — and the layout would stop being
a description of the hardware in the room.

**The correct analogue is `HUMANOID_GAMEPAD_ENABLE`.** The gamepad is an input device gated by
an env switch; *what it may drive* is gated on capabilities. Do the same:

- `HUMANOID_QUEST_ENABLE=1` — whether the TLS listener, the page and `/ws/xr` exist at all.
- Quest control requires capability **`arm_teleop`**, which is unchanged code.

The "config changes behaviour, not code" property is fully preserved: bolt on the right arm,
tick it in Settings, and the Quest can drive it — `available_control_modes()`,
`SESSION_CAPABILITY["arm"]` and `select_arm()` all already handle a second arm with no edits.

---

## 8. Frontend

### 8.1 Control card → status / lifecycle only

`app/src/components/ControlPanel.jsx`: delete **Section 3 · Motion** and everything it owns —
the policy `<select>`, Hold, Run policy, and the `useEffect` that syncs `deadmanSelect`. Keep
sections 1 (Connection) and 2 (Arm), the calibration warning, the deadman warning, and the
error strip. Keep the `mini` tier as-is.

**Keep `Stop` on this card.** The task says motion goes; a stop button does not. Removing a
way to stop a moving robot to satisfy a card boundary is not a trade worth making. Render it
whenever `motion` is true, as the compact tier already does.

> **Trap.** Lines 48–54 of `ControlPanel.jsx` are load-bearing and non-obvious: the checkpoint
> `<select>` calls `api.deadmanSelect('policy', checkpoint)` on every entry to `CONNECTED`.
> That is what makes the **gamepad's A button** arm a policy session instead of falling back
> to a `ZeroPolicy` hold where the sticks do nothing. This effect must move with the dropdown
> into the Control method card, including the `syncedRef` guard and the
> "only sync from `CONNECTED`" rule (`select_session` throws while a session is live).

### 8.2 New card: Control method

`app/src/components/ControlMethodPanel.jsx`, catalog id `control-method`, group `Control`.

- Segmented selector: **Xbox controller | Quest | Policy**. Options come from
  `t.input_sources`; unavailable ones render disabled with the reason
  (`"gamepad deadman not enabled"`, `"HUMANOID_QUEST_ENABLE not set"`, `"no arm configured"`).
- Selecting calls `POST /api/input_source`. Disabled while `t.state` is ARMED/HOLDING/RUNNING,
  with the tooltip "Disarm to change control method" — mirroring the 409 the backend returns,
  so the UI never offers an action the service will refuse.
- **Xbox** selected → a one-line reminder of the button map, a link to the Controller card.
- **Quest** selected → arm selector (when >1 arm), scale and yaw-offset readout, and the
  connect URL `https://<host>:8443/xr` with a QR code. Typing that into a Quest browser by
  hand with two controllers is genuinely unpleasant; the QR is worth the 3 kB.
- **Policy** selected → the checkpoint dropdown (with its `deadmanSelect` sync), plus
  **Ramp to pose / Hold**, **Run policy** and **Stop**, carrying their existing disabled logic
  verbatim.
- Placeholders for `generic gamepad` / `PlayStation` are **not** rendered. A disabled option
  for something that does not exist is noise; add them when the sources exist.

### 8.3 New card: Quest

`app/src/components/QuestPanel.jsx`, catalog id `quest`, group `Diagnostics`, with
`tiers: { mini: 5, compact: 9 }` so it can sit as a glanceable strip.

Reads the `t.quest` block:
- Connection dot + session id.
- **Link health**: frame rate (Hz), age of last frame (ms), dropped/malformed count. Colour by
  the §5 ladder — green < 200 ms, amber to 1.0 s, red past it.
- **Tracking**: `tracked` per controller; amber when the controller is untracked, with
  "controller out of view" rather than a bare boolean.
- **Clutch**: `anchored` / not, and trigger value as a bar. This is the single most useful
  number during a bench session — it answers "is the arm about to move" at a glance.
- **Authority**: "driving" / "connected, Xbox is active", from `owns_input`.
- Mini tier: one line — dot, Hz, clutch state.

### 8.4 Controller card: leave it Xbox-specific

`GamepadPanel` is a raw evdev dump — every advertised button code, phantom-button marking, the
detected `xpad` vs `bluetooth-hid` axis layout, the deadband and trigger threshold. That is
*valuable precisely because it is device-specific*; it exists so "a binding that maps the wrong
physical button is visible rather than inferred".

A "generic controller card" that adapts would be a switch statement over two data shapes with
nothing in common — an evdev button/axis table and a WebXR pose+trigger stream — and would
make both worse. **Recommendation: per-device cards.** Rename the catalog entry's title from
`Controller` to `Xbox controller` (the `id` stays `gamepad` — renaming an id orphans every
saved layout, per the catalog comment). Quest gets its own card. If a PlayStation pad lands
later it will be evdev too and can share `GamepadPanel` with a different button map, which is
the only sharing that was ever real.

### 8.5 `DEFAULT_LAYOUT` and the migration trap

Add to the `control` tab: `control-method#1` under `control-panel#1`, and `quest#1` under
`gamepad#1`. Rough arrangement (left column, 4 wide):

| card | y | h |
|---|---|---|
| `control-panel#1` | 0 | 6 (shrinks — motion section gone) |
| `control-method#1` | 6 | 8 |
| `gamepad#1` | 14 | 11 |
| `quest#1` | 25 | 9 |
| `imu#1` | 34 | 4 |
| `robot-mini#1` | 38 | 11 |

`joint-table#1` stays at `x: 4`.

> **Trap.** `useLayouts.js` `read()` returns any saved layout with `tabs.length` **as-is** —
> the `version` field on `DEFAULT_LAYOUT` is currently decorative and nothing consults it.
> `migrateV1` only fires off the *legacy* localStorage key. So anyone who has touched the
> dashboard has `humanoid_dashboard_v2` saved, and would get the new motion-less Control card
> with **no Control method card anywhere** — no way to reach Run policy at all, and no
> indication why.
>
> Bump `DEFAULT_LAYOUT.version` to `3` and add a real `migrateV2` that keys off
> `parsed.version < 3`: keep every existing card, position and setting, and **insert**
> `control-method` (immediately below any `control-panel`) and `quest` (below any `gamepad`),
> shifting later cards down. If a tab has no `control-panel`, append `control-method` to the
> first tab so the controls are never unreachable.

---

## 9. Verification

### 9.1 Arm powered **off** — plumbing (do all of this first)

The daemon can run with no CAN adapters; the layout can be set to `left_arm` with the ESCs
unpowered. Everything below is exercisable at a desk.

1. **Frame conversion, pure unit test.** `tests/test_xr_frames.py` — feed known WebXR poses
   (1 m along each WebXR axis), assert the robot-frame result, and assert equality with
   `televuer`'s `T_ROBOT_OPENXR` matrix applied to the same vectors. Two independent
   derivations agreeing is the point.
2. **`step_pose` against goldens.** Extend the arm-teleop tests: a desired point 1 cm ahead of
   the current hand produces a `dq` that moves the tool ~1 cm; a desired point 10 m away
   produces a `dq` capped at `speed·dt` and does **not** blow up; a desired point inside a
   joint limit slides along the constraint rather than stopping. Compare `step_pose` and
   `step` on an equivalent command and assert both route through `ik_step` with the same
   clamps.
3. **Liveness ladder, simulated clock.** Drive `QuestSource` from a fake frame generator with
   a controllable monotonic clock. Assert each row of the §5 table: 200 ms stall clears the
   gate and *does not* E-STOP; 1.0 s E-STOPs; `tracked: false` clears the gate; identical
   poses for 500 ms with the gate held clears it; `session` change discards the anchor.
   **This is the test that matters most** — it is the one thing `xr_teleoperate` got wrong.
4. **Arbitration.** With a fake gamepad and a fake Quest both "connected": assert non-owner
   `set_run_gate` is dropped and counted; assert `set_input_source` raises 409 in
   ARMED/HOLDING/RUNNING; assert E-STOP works from the non-owner.
5. **Per-source liveness regression.** Assert that with the browser `/ws/control` heartbeating,
   a stalled Quest still reads `deadman_ok() == False`. This fails on today's code — that is
   the point.
6. **Real headset, no robot.** Layout = `left_arm`, ESCs unpowered, daemon running.
   Put the Quest on, open `https://<robot>:8443/xr`, enter immersive, hold the trigger and
   move. Watch:
   - the Quest card: Hz, age, tracked, clutch;
   - the arm run log (`ArmRunRecorder` is always on for an armed arm session) — the recorded
     `q_target` stream should be smooth and within limits, and the recorded `xr` block should
     show the anchor latching on trigger press;
   - the wireframe (`robot-mini`) will **not** move — it draws live encoders, and there are
     none. Confirm this is understood before the powered run, or it reads as a failure.
   Deliberately walk out of WiFi range, put the controller behind your back, and take the
   headset off. Confirm each produces the §5 behaviour in the log.
7. **Frontend.** `npm run build`, load with a saved v2 layout in localStorage, confirm the
   migration inserts both cards and preserves positions. Confirm the checkpoint dropdown in
   its new home still issues `deadmanSelect` on entry to `CONNECTED`.

### 9.2 Arm powered **on**

Preconditions, from the wiki page and the repo's own rules: a second person present, the
headset in passthrough, the arm bolted to the plate, zeros **re-taught after the power cycle**
(single-turn encoders — non-negotiable), and the daemon pointed at the same
`humanoid_lite.json` the web server resolved (it logs which one won).

1. **Calibrate.** T-pose teach, verify against the relaxed-hang expectation
   (`shoulder_pitch ≈ 0`, `shoulder_roll` −15°…−8°, `elbow_pitch ≈ 0`).
2. **Xbox first, unchanged.** Arm, hold a trigger, drive the joint frame. This is the
   regression check that the refactor did not touch the working path. If this feels different,
   stop and fix that before introducing the Quest.
3. **Quest, gate only.** Select Quest as the control method, arm, and press/release the trigger
   **without moving your hand**. The arm should enable POSITION and hold, then go IDLE. Nothing
   should move. If the arm twitches on engage, the anchor is not latching at the same instant
   as `teleop.reset` — fix before proceeding.
4. **Quest, creep, one axis.** Creep speed. Move the controller ~10 cm along one axis at a
   time and confirm the hand moves the right way. **A wrong sign and a wrong zero look
   different** — a wrong sign moves opposite, a wrong offset moves correctly but from the wrong
   place. Fix `HUMANOID_QUEST_YAW_DEG` / the basis conversion, not the calibration.
5. **Quest, creep, free motion.** Small volume. Watch `tracking_error_m` and `at_limit` in the
   arm log. Expect the hand to lag and to slide along constraints — both are designed
   behaviour, and the log says which is happening.
6. **Loss drills, with the arm powered and a second person on the E-STOP.** Controller behind
   your back mid-motion → IDLE. Headset off mid-motion → IDLE. WiFi off mid-motion → IDLE at
   200 ms, E-STOP at 1 s. Do these *deliberately*, early, at creep speed. They are the whole
   reason for §5.
7. **Normal speed** only after all of the above.

---

## 10. Things I think are bad ideas, and other flags

**On the UI split — I agree with it, with one change.** Control card = lifecycle, Control
method = what drives it, Quest = device status is a clean decomposition, and it scales to a
second and third input device without further surgery. The one change: **`Stop` stays on the
Control card**, and Hold/Run live with the Policy method as decided. My only reservation is
that "Policy" is not really the same *kind* of thing as "Xbox controller" and "Quest" — those
are input devices, Policy is an autonomy mode. But the card answers "what is driving the
robot", and for that question a policy genuinely is one of the answers. It reads fine.

**Do not vendor `televuer`.** Covered in §1. Worth repeating that the numpy pin is not a
packaging nuisance — it is `onnxruntime` and the policy path.

**Do not use `xr_teleoperate`'s `teleop/safety/` framework here.** It is good work, and it is
solving the *opposite* problem: a G1 without a gantry is actively stabilised, so its core
principle is "the safe stop is a controlled descent, not a torque cut". A bolted-down bench arm
is the classic case where torque cut *is* safe, which is why IDLE-on-release is right here and
must stay. Borrow the change-detection idea, nothing else.

**`gamepad-absent` will E-STOP a healthy Quest session.** `GamepadDeadman._run` E-STOPs
whenever no gamepad device is found and the state is ARMED/HOLDING/RUNNING — regardless of
which source owns input. Once the Quest can arm a session, an unrelated Xbox pad going flat in
a drawer kills a Quest run. Narrow it to fire only when the gamepad owns the token. Small fix,
but it is a new bug the moment the second input source lands.

**The single global heartbeat is a latent defect today** (§4.3b), not just an obstacle to this
work. Worth fixing regardless of whether the Quest ships.

**25 Hz arm tick** (§3). Fix before the first powered Quest session, not after.

**No orientation control, and the UI must not imply otherwise** (§3). Position only, wrist
twist optional and deferred.

**Certificate friction is real.** Self-signed means an interstitial in the Quest browser and a
per-device trust step. There is no way around it — WebXR requires a secure context and the
robot has no public DNS name. Budget 20 minutes the first time and document it in `deploy/`.

**Out of scope, as instructed:** slow return to a relaxed pose on release. The
release → IDLE path in `_deadman_worker` is left exactly as it is.
