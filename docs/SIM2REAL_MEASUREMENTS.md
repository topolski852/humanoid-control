# Sim2Real Measurement Spec

**Purpose:** define exactly what we need measured on the physical robot so that
`humanoid-policy` can model the real signal path in training instead of guessing at it.

This document is the *request*. It does not contain the tools. Whoever picks this up on the
robot should build (or fix) the capture tooling, run the captures, and commit the results.

> **Scope: legs only.** The policy runs on the 12 leg joints. This robot has two CAN adapters
> (`can_left_leg`, `can_right_leg`) and no arms attached. The studio config also defines
> `can_left_arm` / `can_right_arm` channels — those are not populated on this machine and are
> out of scope. Ignore them.
>
> **Source of truth:** this repo (`/home/nse/humanoid-control`) and the hardware physically
> attached to it. An earlier revision of this document was written against a different machine
> with a left arm and a different directory layout; every path and topology claim below has been
> re-verified against *this* robot.

---

## Why we are doing this

Menlo Research published two write-ups on closing the sim2real gap on their Asimov humanoid
([Feb 2026](https://news.asimov.inc/p/noise-is-all-you-need),
[Aug 2026](https://menlo.ai/research/zero-shot-sim2real-asimov)). Their conclusion after eight
months, in their own words:

> Only domain randomize what's actually random. [...] if we randomized a parameter that isn't
> actually random on the hardware, the policy starts to hedge against distributions that don't
> exist. The model should never compensate for bad hardware understanding.

That is the opposite of "turn the noise up." It means every noise, delay, or randomization term
in the training sim should trace back to a number measured on this robot. Right now most of ours
do not.

What our walk policy currently trains against, and where the number came from:

| Training term | Current value | Provenance |
|---|---|---|
| Actuator latency (legs) | 7.2 ms → delay (1,2) substeps | **measured** on bench (humanoid-tuner) |
| Actuator latency (ankles) | 12 ms → delay (2,3) substeps | **measured** on bench |
| Stick-slip friction | coulomb 0.4293, viscous 0.1374 | **measured** on bench |
| Robot mass | 12.61 kg | **measured** |
| `base_ang_vel` noise | U(±0.3) rad/s | guessed — gyro auto-zeroes at rest, so the floor is **unmeasurable** (B5) |
| `projected_gravity` noise | U(±0.05) | guessed |
| `joint_pos` noise | U(±0.05) rad | guessed — **see M6, likely ~500× too wide** |
| `joint_vel` noise | U(±2.0) rad/s | guessed, already flagged as suspect |
| Observation delay | **none modeled** | — |
| Gyro bias / drift | **none modeled** | **keep it that way** — unmeasurable on this IMU (B5) |
| Encoder quantization | **none modeled** | — |
| Sample dropout / stale-hold | **none modeled** | — |

The bench-measured actuator layer is good. Everything *downstream* of the motor — the CAN
transport, the IMU, the encoder path, and the timing of the policy loop itself — is unmodeled or
invented. That is the gap this spec is meant to close.

For reference, the numbers Asimov trains against on comparable hardware: gyro noise U(±0.01)
rad/s, joint_pos U(±0.01) rad, actuator command lag 0–5 ms, joint calibration offset ±0.02 rad,
PD gains ×[0.8, 1.2]. Our per-step observation noise is roughly **5× wider than theirs** on a
slower control loop. Measurement should either justify that or let us shrink it.

---

## This robot, as actually configured

Verified against `/home/nse/humanoid-studio/configs/humanoid_lite.json` and the live kernel.

**Two buses, six joints each.** This is the structure M1/M2 must model — not a four-bus stagger.

| `can_left_leg` | node | | `can_right_leg` | node |
|---|---|---|---|---|
| left_hip_roll | 1 | | right_hip_roll | 2 |
| left_hip_yaw | 3 | | right_hip_yaw | 4 |
| left_hip_pitch | 5 | | right_hip_pitch | 6 |
| left_knee_pitch | 7 | | right_knee_pitch | 8 |
| left_ankle_pitch | 11 | | right_ankle_pitch | 12 |
| left_ankle_roll | 13 | | right_ankle_roll | 14 |

Both buses: 1 Mbit, sample point 0.750, `gs_usb` driver, state `ERROR-ACTIVE`.

**All 12 leg joints share the same sensing and control configuration** — `cpr` 4096 (12-bit),
`|gear_ratio|` 15, `position_kp` 45.0, `velocity_kp` 1.5, `fast_frame_frequency` 100 Hz.
Left/right gear signs are opposite on every pair, which is correct per the calibration convention.
So for M1–M6 purposes the joints are interchangeable, with two exceptions to keep in mind: the
per-joint `torque_limit` spread (next section) and the B1 defects below.

Two legitimate motor families do exist, but they only affect the current loop, not sensing:
hips + knees (`torque_constant` 0.08958, `current_kp` 0.190956, `current_ki` 4538.42) and ankles
(0.06588, 0.166452, 5747.85). Do not mistake these for anomalies.

**Encoder quantum is analytically known: `2π / 4096 / 15` = `1.023e-4` rad = `0.0059°`,
identical on all 12 joints — and now confirmed on hardware.** A 300-sample position capture at
rest (2026-08-29) shows the dithering joints moving by exactly **1.00 encoder count,
0.000102 rad peak-to-peak**, matching the prediction to three significant figures.

That makes M6 a *verification* rather than a discovery, and it already settles the headline: our
trained `joint_pos` noise of U(±0.05) rad is **~490× the measured at-rest dither**, and even
Asimov's ±0.01 rad is ~98×. M6 proper still needs to run *under load*, where the floor will be
higher — but the guessed value is now known to be far too wide, not merely suspected.

**Firmware filters** (`humanoid-esc-firmware`, applied on the motor before we see anything):
- `velocity_filter_alpha` = 0.7154 — 2 kHz cutoff out of the 10 kHz commutation loop
  (`humanoid-esc-firmware` → `Core/Src/encoder.c:95`)
- `torque_filter_alpha` = 0.1454 — roughly 50 Hz
  (`humanoid-esc-firmware` → `Core/Src/position_controller.c:103`)

### Torque limits are already a sim↔real term — keep them synchronised

Per-joint torque caps are **not** a hardware-only concern. They exist on both sides and must agree:

- **Hardware:** `torque_limit` per joint in the studio config, written to the ESC and enforced in
  firmware. This is the real clamp.
- **Sim:** `_CONTRACT_EFFORT` in
  `humanoid-policy/source/humanoid_policy_assets/humanoid_policy_assets/robots/humanoid.py`, which
  feeds `effort_limit` on the IsaacLab actuator configs and clips actuator output during training.

If sim clips lower than hardware, the policy never learns to use torque the robot has. If sim
clips higher, the policy learns to rely on torque the robot cannot deliver. Either way it is a
sim2real gap of exactly the kind this document exists to close, and it is *free* to get right —
unlike the noise terms below, no measurement is needed, only bookkeeping.

**Synchronised 2026-08-29.** Hardware and `_CONTRACT_EFFORT` now agree on all 12 joints:

| joint | N·m | | joint | N·m |
|---|---|---|---|---|
| left_hip_roll | 6.0 | | right_hip_roll | 6.0 |
| left_hip_yaw | **12.0** | | right_hip_yaw | **6.0** |
| left_hip_pitch | 9.5 | | right_hip_pitch | 9.5 |
| left_knee_pitch | 11.0 | | right_knee_pitch | 11.0 |
| left_ankle_pitch | 6.0 | | right_ankle_pitch | 6.0 |
| left_ankle_roll | 7.0 | | right_ankle_roll | 7.0 |

Three of these were diverged until now — both knees (sim 6.0 vs hardware 11.0) and
`right_ankle_roll` (6.0 vs 7.0). The hardware values were raised deliberately on 2026-08-24 after
`run_1787604267152086081` showed persistent knee droop saturating the 6.0 N·m cap on 38.7% / 45.1%
of policy steps; the training side was flagged as must-fix-before-retrain at the time and had not
been updated. `_CONTRACT_EFFORT` has now been raised to match.

Two things to keep straight:

- **The left/right hip_yaw asymmetry (12.0 / 6.0) is correct.** It is confirmed on both devices by
  SDO readback and was already present in `_CONTRACT_EFFORT`. Do not symmetrise it.
- **`humanoid-policy/deploy/walk/leg_policy_contract.json` stays at the old values** (knees 6.0,
  right_ankle_roll 6.0). That file is a *record of what the currently deployed policy was trained
  with*, not a config to be updated. The next retrain picks up `_CONTRACT_EFFORT` and will export a
  fresh contract.

**Therefore the deployed walk policy was trained against 6.0 N·m knees while the robot runs 11.0.**
That mismatch is live right now and will persist until the next retrain. It is worth keeping in
mind when interpreting any capture taken before then — particularly M7, which is measuring torque
tracking against a sim actuator model whose clip point does not match the hardware's.

---

## Known blockers to fix first

These are verified against this machine. Fix them before capturing anything.

### B1 — Frozen firmware filters on two left-leg joints — **RESOLVED & VERIFIED ON DEVICE 2026-08-29**

Two joints held filter alphas that no other joint had. A filter alpha of `0.0` is not
"unfiltered" — it is **frozen**. Every one of these filters is an EMA of the form:

```c
x += alpha * (new_value - x);
```

With `alpha = 0` the update term is identically zero and `x` never leaves its initialised value.

| joint | field | was | now | effect of the old value |
|---|---|---|---|---|
| `left_hip_yaw` | `velocity_filter_alpha` | **0.0** | 0.7154 | `encoder->velocity` frozen at `0.0f` — **reported a permanent zero velocity regardless of actual motion** |
| `left_hip_yaw` | `torque_filter_alpha` | 0.1 | 0.1454 | ~1.45× slower torque response than every peer |
| `left_hip_yaw` | `bus_voltage_filter_alpha` | 0.01 | 0.2696 | ~27× too slow; bus voltage lags the real rail badly |
| `left_hip_yaw` | `velocity_limit` | 0.0 | 20.0 | latent — only clamps in `MODE_VELOCITY`, and the policy runs `MODE_POSITION`, so this was inert. A landmine if anything ever uses velocity mode |
| `left_hip_roll` | `bus_voltage_filter_alpha` | **0.0** | 0.2696 | `bus_voltage_measured` frozen at `NOMINAL_BUS_VOLTAGE` |

The `left_hip_yaw` velocity freeze is the serious one: the policy was being fed a **constant zero
for one of its 12 velocity inputs**.

The `left_hip_roll` bus-voltage freeze matters more than it looks. `bus_voltage_measured` is the
PWM normalisation divisor (`powerstage.c:60-62`,
`v_a = .5f * ((v_a / bus_voltage_measured) + 1.f)`), so with it frozen at nominal the FOC
voltage→duty conversion ignores real rail sag — an effective gain error that grows exactly when
the rail sags hardest, i.e. under walking load. It also makes the `while (bus_voltage_measured < 9)`
startup gate at `motor_controller.c:508` pass unconditionally.

**Backup before the edit:** `configs/humanoid_lite.json.bak-20260829-pre-hipyaw-filter`.

**Device readback — DONE 2026-08-29, and it is the worst case.** `READ_CONFIG` SDO against the
live ESCs confirms **the frozen values are on the motors**, not just in the JSON:

| joint | field | device truth | verdict |
|---|---|---|---|
| `left_hip_yaw` | `velocity_filter_alpha` | **0.0** | frozen — confirmed |
| `left_hip_yaw` | `torque_filter_alpha` | 0.100000001 | confirmed |
| `left_hip_yaw` | `bus_voltage_filter_alpha` | 0.009999999 | confirmed |
| `left_hip_yaw` | `velocity_limit` | 0.0 | confirmed |
| `left_hip_roll` | `bus_voltage_filter_alpha` | **0.0** | frozen — confirmed |
| `right_hip_yaw` | (all) | 0.7154 / 0.2696 / 20.0 | healthy control case |

**Consequence: `left_hip_yaw` has been reporting a constant zero velocity in every run to date.**
Every prior observation involving that joint's velocity is invalid, and any earlier tuning or
diagnosis that leaned on it should be revisited. `left_hip_roll` has been running its FOC PWM
normalisation against a frozen nominal bus voltage for the same period.

**Resolution — and the operational gotcha that nearly hid it.** Editing the JSON does **nothing**
on its own. The daemon reads the robot config **once at startup** and writes parameters to the ESCs
as a *delta* against that in-memory copy. Power-cycling the motors does not help — the stale copy
is in the daemon, not the ESCs. A reconnect against a stale daemon compares old-against-old, sees
no change, and writes nothing. The full sequence is:

```bash
sudo systemctl restart humanoid-daemon   # reload config from disk (no reload cmd exists)
python scripts/smoke_test.py --connect   # apply_all_configs: wake to IDLE, write params, no motion
# then re-read the devices and confirm
```

**Verified 2026-08-29 after running exactly that:** all 12 leg joints now read back matching the
on-disk config, zero mismatches. `left_hip_yaw` holds `velocity_filter_alpha = 0.7154` and
`left_hip_roll` holds `bus_voltage_filter_alpha = 0.2696`.

**Functional confirmation, not just a config readback.** A working EMA shows LSB dither at rest; a
frozen one reads exactly `0.0` forever. Sampling velocity at 50 Hz with the robot stationary,
`left_hip_yaw` now produces 159 distinct values over 199 samples — it is genuinely alive, not just
configured correctly.

Three joints (`left_ankle_pitch`, `left_ankle_roll`, `right_ankle_roll`) *do* still read exactly
zero velocity, but this is **correct behaviour, not a second defect**: their encoders are
perfectly still — 1 distinct position across 300 samples, 0 counts peak-to-peak — so there is no
motion for the filter to report. Their device configs read the correct 0.7154. Note the limitation
for future checks: at rest this test can only prove a filter *alive*, never prove one *frozen*,
because a still encoder and a dead filter look identical.

(One incidental: single parameters occasionally come back `None` in a `READ_CONFIG` sweep — a
per-parameter read miss on an otherwise clean node, not a config problem. They read fine on retry.)

**The hip_yaw torque asymmetry is real — do not symmetrise it.** `left_hip_yaw` runs a 12.0 N·m
`torque_limit` where `right_hip_yaw` runs 6.0. This looks like the same kind of anomaly as the
filters above, and it is not. It is deliberate, it is confirmed on the device by SDO readback, and
**the training sim already encodes the same asymmetry** (`_CONTRACT_EFFORT` in
`humanoid-policy/source/humanoid_policy_assets/.../robots/humanoid.py`). Making the two sides match
would *break* sim↔real agreement rather than restore it. See "Torque limits" below.

**One open item on `left_hip_yaw`: its current limit gives almost no torque headroom.**

| joint | `torque_limit` | `current_limit` | motor ceiling = Kt·I·gear | headroom |
|---|---|---|---|---|
| `left_hip_yaw` | 12.0 N·m | **10.0 A** | 0.08958 × 10 × 15 = **13.4 N·m** | **1.12×** |
| every other hip/knee | 6.0–11.0 N·m | 20.0 A | 0.08958 × 20 × 15 = 26.9 N·m | 2.4–4.5× |

`left_hip_yaw` is the only leg joint whose commanded torque cap sits within 12% of what its current
limit can physically deliver. Every other joint has at least 2.4× margin. If `torque_constant` is
even slightly optimistic, or the rail sags, this joint becomes torque-starved at exactly the point
the policy expects its full 12 N·m — the same failure mode already diagnosed on the knees. Raising
`current_limit` to 20.0 to match its peers would restore normal margin without changing the torque
cap the policy trains against. Left unchanged because it loosens a protection limit; worth a
deliberate decision.

### B2 — Python config search path — **FIXED 2026-08-29**

`ROBOT_CONFIG_CANDIDATES` in [config.py:42-47](../humanoid_control/config.py#L42-L47) previously
searched only:

```
~/.config/humanoid-studio/humanoid_lite.json
~/humanoid/humanoid-studio/configs/humanoid_lite.json     ← other machine's nested layout
<repo>/configs/humanoid_lite.json
```

None exist here. The config actually lives at
`/home/nse/humanoid-studio/configs/humanoid_lite.json` — repos are flat siblings under
`/home/nse/`: `humanoid-control`, `humanoid-studio`, `humanoid-policy`, `humanoid-esc-firmware`.

Every Python entry point — `smoke_test.py`, `hold_pose.py`, `run_policy.py`, the web service —
died at startup with `FileNotFoundError`. The daemon was unaffected because it is launched with an
explicit `--config`.

Fixed by adding the flat-sibling path to the candidate list, ahead of the nested one so both
machine layouts resolve. `scripts/smoke_test.py` now runs with no `HUMANOID_CONFIG` override.
`$HUMANOID_CONFIG` still overrides everything if you need to point at a different copy.

*(`scripts/can_monitor.py` and `README.md` are **correct** as written — they already point at the
real path. An earlier draft of this document claimed otherwise; it was describing the other
machine. Do not "fix" them.)*

### B3 — The policy scripts hardcode a fake IMU

This is the blocker that silently ruins IMU captures.

- [run_policy.py:53](../scripts/run_policy.py#L53) — `base_source=UprightStubBaseState()`
- [hold_pose.py:46](../scripts/hold_pose.py#L46) — `base_source=UprightStubBaseState()`

This is not a fallback that might trigger; it is the hardcoded value. `UprightStubBaseState`
returns a **constant** upright gravity vector and **zero** angular velocity. Any capture taken
through these scripts produces perfectly clean, perfectly fake IMU data.

Only the web service path uses real base state
([service.py:1022](../humanoid_control/web/service.py#L1022),
[service.py:1522](../humanoid_control/web/service.py#L1522), both `TelemetryBaseState`). Nothing
outside `scripts/imu_monitor.py` ever constructs `SerialImuBaseState`.

Before any M4/M5 capture: wire a real source into the capture path, and record which
`BaseStateSource` was live in every capture's metadata. A capture that does not name its base
source should be discarded.

### B5 — Gyro auto-zeroes at rest — **NOT a fault; M5 is unmeasurable and must be dropped**

**Resolved 2026-08-29.** The gyro is healthy. Rotating the robot produces live angular-velocity
readings in the web UI's IMU panel; it reports exactly zero *only while stationary*. This is the
sensor's internal resting auto-zero, not a dead sensor. `base_ang_vel` is real during motion,
which is the regime the policy runs in — **the policy runs are not blocked by this.**

The investigation is kept below because the evidence is genuinely confusing and someone will
re-derive it otherwise, and because it changes what M5 can deliver.

The IMU streams **seven** frame types, all at 99.78 Hz, zero checksum failures — the sensor is
alive and healthy. Accelerometer and magnetometer dither normally. But every gyro frame carries
an all-zero payload:

```
55 52 00 00 00 00 00 00 d3 09 83     <- 0x52 gyro: axes are literally 0x0000
55 51 56 fd 09 00 96 07 d3 09 7b     <- 0x51 accel: live, dithering
```

Across 1995 gyro frames: mean 0.0, std 0.0, min 0.0, max 0.0 on all three axes. A live MEMS
gyro cannot produce that — its LSB is 0.061 °/s and it dithers at rest.

A trap to avoid when re-checking this: a naive duplicate-payload test reports that **93% of gyro
payloads differ**, which looks like live data. They differ only in the trailing temperature field
(2515 → 2525). The three axis words are identical zeros throughout. Compare `frame[2:8]`, not the
whole payload.

**Configuration is ruled out.** Register readback from the sensor:

| register | value | meaning |
|---|---|---|
| `0x01` CALSW | 0x0000 | normal — not stuck in a calibration mode |
| `0x02` RSW | 0x025F | output content — **gyro output IS enabled** (matches the 7 types seen) |
| `0x03` RRATE | 9 | 100 Hz |
| `0x04` BAUD | 9 | 921600 |

So the sensor is correctly configured, emits gyro frames on schedule, and fills them with zeros.
Corroborating: euler and quaternion frames are 99.3% / 96.5% byte-identical consecutively, i.e.
the orientation fusion is barely updating — what you would expect with no gyro input.

**Why this is worse than it looks:** `WitMotionReader::snapshot()` sets `valid` from
`have_quat_` alone. Quaternion present + gyro absent = a sample the daemon reports as **VALID**.
Nothing anywhere flags it. The policy consumes `base_ang_vel = [0, 0, 0]` and believes it.

**Resolution: motion settles it.** Rotating the robot makes the angular-velocity rows in the web
UI's IMU panel respond immediately. Rest auto-zero, confirmed. (`scripts/imu_gyro_live.py` runs
the same test from the CLI with the daemon stopped, if you need numbers rather than a UI.)

Note for anyone reading the IMU panel: its `● live` indicator is driven by `projected_gravity`
alone and never inspects `angular_velocity`, so the panel reads "live" in both cases. Tilt /
Pitch / Roll come from the accelerometer-derived gravity vector and stay accurate even if the
gyro contributes nothing — an accurate tilt readout is *not* evidence the gyro works. Watch the
three Angular-velocity rows specifically.

#### What this costs us: M5 is unmeasurable — drop the term, do not guess it

The sensor clamps its gyro output to exactly `0x0000` while stationary. M5 asked for the
per-axis bias, its drift over ten minutes, and an Allan-deviation curve. **None of that can be
measured on this hardware**, because the sensor suppresses precisely the signal M5 wanted to
characterise. A 10-minute rest capture yields exactly zero, which is the clamp, not the bias.

By this document's own rule — *only randomize what's actually random* — the correct action is to
**leave `NoiseModelWithAdditiveBiasCfg` off `base_ang_vel` entirely** rather than populate it
with a plausible-looking number. An unmeasurable parameter modelled from a guess is the exact
failure Menlo warns about, and here the measurement gap is a property of the sensor, not of our
effort. Report M5 as "not measurable on this IMU" and move on.

`scripts/measure/m4m5_imu.py` now reports `all_axes_zero_fraction` and a verdict line on every
gyro capture, so a clamped capture can never be mistaken for a low-noise one:

* `~1.0` → capture sat entirely inside the deadband; bias/noise/drift figures are meaningless.
* `~0.0` → the robot was moving throughout; the deadband never bit and the rates are real.

#### What is still worth measuring: the deadband, during an actual walk

The auto-zero is a **deterministic, non-random** hardware behaviour, so if it matters it should
be modelled as a fixed deadband, not as noise. The question that decides whether it matters at
all is simply: *during walking, how often is the reported gyro exactly zero?*

Run a normal walk capture and read `all_axes_zero_fraction`:

* near 0% → the deadband never bites at walking rates. Ignore it; no sim term needed.
* materially above 0% → the policy is being fed zeros during genuine slow rotation, and a
  deadband belongs in the observation model.

This needs no special rig — it falls out of the three policy runs already planned.

### B4 — Python IMU baud defaults are wrong (the daemon's is fine)

- [imu_monitor.py:34](../scripts/imu_monitor.py#L34) — `--baud` defaults to 9600
- [imu.py:96](../humanoid_control/imu.py#L96) — `WitMotionReader.__init__` defaults to 9600

The **daemon is correct** — it defaults to 921600 and is currently running as
`--imu-device /dev/humanoid_imu`, with the port confirmed at 921600 baud. The 9600 defaults only
affect Python-side measurement harnesses, which is exactly what this spec asks people to build.
Pass the baud explicitly in any harness.

---

## IMU sample rate: how fast should we run it?

Short answer: **200 Hz is free on bandwidth and USB. Run it — but validate that the frames are
genuinely new before trusting the extra rate.**

**Bandwidth is a non-issue — now measured, not estimated.** The sensor actually streams **seven**
frame types, not the four assumed here originally: 0x50 time, 0x51 accel, 0x52 gyro, 0x53 euler,
0x54 mag, 0x56 pressure, 0x59 quaternion. Measured byte rate at 100 Hz is **7671 B/s**
(≈61 kbps with 8N1 framing), matching 7 × 11 × 100 exactly.

| Output rate | Line rate | % of 921600 baud |
|---|---|---|
| 100 Hz (measured) | 61 kbps | **6.7%** |
| 200 Hz (max, projected) | 123 kbps | 13.3% |

Measured frame timing at 100 Hz: inter-frame interval mean 10.02 ms, p95 10.18, p99 10.32,
max 12.06; **zero checksum failures** and **zero staleness-gate trips** over the capture. The
transport is clean — see B5 for the payload problem, which is a different thing entirely.

**USB contention is a non-issue too.** Verified topology on this machine:

```
Bus 03 (480M root hub)
├── Port 3: ch341  (IMU, /dev/humanoid_imu)          12M  ← own transaction translator
└── Port 4: hub (480M)
    ├── Port 1: gs_usb  (can_left_leg)               12M
    └── Port 2: gs_usb  (can_right_leg)              12M
```

The IMU and the CAN adapters are all full-speed (12 Mbit) devices, but the IMU sits directly on
the root hub while both CAN adapters sit behind a separate downstream hub. They therefore do not
share a transaction translator and do not contend for its scheduling. Combined traffic is a
rounding error against 480 Mbit. There is no bandwidth argument for staying at 100 Hz.

**The real caveat, and it matters for this spec specifically.** The policy runs at 25 Hz and
`SerialImuBaseState.get()` only ever returns the *latest* sample, so anything above ~100 Hz buys
the *controller* nothing. The benefit is entirely to *measurement*: you cannot characterise a
10 ms inter-frame distribution well when your sampling interval is also 10 ms.

But WitMotion sensors will happily accept a 200 Hz output-rate register setting even when their
internal fusion runs slower, and then emit each fused value twice. Duplicated frames are
**indistinguishable from a stale-hold**, which is precisely what M3 and M4 are trying to measure.
Running at 200 Hz without checking would manufacture the exact artifact we are hunting for.

**So:** set 200 Hz (`scripts/imu_setup.py --rate 200`), then before capturing, confirm that
consecutive 0x59 quaternion payloads actually *differ*. If they repeat pairwise, the fusion is
100 Hz — drop back to 100 Hz and note it, because at that point the higher rate is strictly
harmful to M3/M4. If they are genuinely distinct, keep 200 Hz for all captures.

Either way, record the configured rate *and* the validated effective rate in capture metadata.

---

## Before building anything: check what already exists

Answered for this machine, as of 2026-08-29:

1. **`_arm_recording/` does not exist.** There are no prior captures on this machine — not arm,
   not leg. `StepRecorder` writes `run_<nanos>_<pid>.jsonl` only when `HUMANOID_RECORD_DIR` is
   set, and it has never been set here. Every measurement below needs a fresh capture.
2. **Kernel CAN counters — baseline captured, both buses clean:**

   | | re-started | bus-errors | arbit-lost | error-warn | error-pass | bus-off | RX pkts | TX pkts |
   |---|---|---|---|---|---|---|---|---|
   | `can_left_leg` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
   | `can_right_leg` | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

   These are post-bring-up baselines with **zero traffic through them** — they are a clean
   starting point, not evidence of a healthy walk. Re-read
   `ip -details -statistics link show <iface>` after each capture and report the delta.
3. Motors are boot-silent and currently read `OFFLINE` until woken. The config-file hazard in B1
   is resolved, so `--connect` now pushes correct values — but do the B1 device readback on nodes
   1 and 3 before trusting any velocity data from them.

**What `StepRecorder.record()` already logs**, which an earlier draft got wrong — it *does*
capture the IMU-derived observation fields ([recorder.py:56-66](../humanoid_control/recorder.py#L56-L66)):
`t`, `base_valid`, `projected_gravity`, `base_ang_vel`, `joint_pos`, `joint_vel`, `obs`,
`action`, `targets`, `command`.

**What it genuinely lacks**, and these gaps block the measurements noted:
- per-joint sample age (blocks M2)
- CAN frame arrival timestamps (blocks M1)
- raw IMU frame arrival timestamps and per-frame-type counts (blocks M4, M5)
- no record of which `BaseStateSource` was live (see B3)

Extending it is expected work.

---

## Capture protocol — what to actually run

Every tool in `scripts/measure/` is READ-ONLY: none of them drive the robot. You start the
policy; they observe. `capture_run.sh` runs the CAN capture and the tick sampler
**concurrently**, which is required — M2 is computed by correlating the two, so a capture where
they do not overlap in time yields nothing.

### Why "standing still" is the high-value condition

Standing under an active policy is not the same as a powered-down robot sitting on a bench. The
joints are **loaded** (carrying the robot's weight) and **actively correcting**, and the body is
swaying slightly. That makes a stand run the best available source for:

* **M6** — encoder noise *under load*, which is what the spec actually asks for. A capture with
  the motors idle measures an unloaded joint and understates the floor.
* **M3** — dropouts are rare events. You cannot prove a 0.1% dropout rate from 60 seconds. Ten
  minutes at 25 Hz across 12 joints is 180,000 joint-ticks, enough to see rates down to
  ~1-in-60,000.
* **M8** — thermal drift needs continuous operation, bucketed by elapsed time.
* **B5 deadband** — does the gyro report real rates while the robot balances, or does the
  resting auto-zero swallow them? This is the regime where it matters most, because rate
  feedback is exactly what a balance controller uses.

A walk attempt then adds the loaded/dynamic regime: busier bus, faster joint motion, larger
angular rates.

### Per policy

Start the policy, let it settle, then in a second terminal:

```bash
# Phase A — standing, policy active, no walk command
scripts/measure/capture_run.sh <label>-stand 600 hold

# Phase B — walking, for as long as is safe
scripts/measure/capture_run.sh <label>-walk 120 walk
```

`<label>` is one of `baseline`, `smoothB`, `smoothA`.

### Suggested order

Run **Smooth A last**. The commit that replaced it in the deploy folder states it *"faulted the
robot's encoders across the whole leg"*. Capturing baseline and B first means an A-run incident
cannot cost you the other two datasets.

1. `baseline` — reference. Note it is the only bundle trained at the old 6.0 N·m knee plant.
2. `smoothB` — current deploy bundle; smoothest in sim.
3. `smoothA` — best falls/min in sim, known-risky on hardware. E-stop ready.

### Durations, and why

| Phase | Duration | Binding constraint |
|---|---|---|
| Stand | **10 min** | M3 rare-event sensitivity and M8 thermal buckets. M1/M2 distributions are fully settled within 60 s; the extra time buys tail coverage, not a sharper mean. |
| Walk | **2 min**, or as long as safe | Enough for M1/M2 under bus load and for the gyro-deadband answer. Extend if the robot tolerates it. |

M8's buckets are minute 0–2, 8–10 and 18–20. A 10-minute stand fills the first two — enough to
see drift. If you want the third, run one stand at **20 min**; doing that for a single policy is
sufficient, since thermal behaviour is a property of the hardware, not the network.

### Once only, not per policy

* **M4 (true IMU frame timing)** needs the serial port, so the daemon must be stopped and no
  policy can be running. Transport timing does not depend on which policy is loaded, so do it
  once:

  ```bash
  sudo systemctl stop humanoid-daemon
  python scripts/measure/m4m5_imu.py --via serial --seconds 300 --activity rest
  sudo systemctl start humanoid-daemon
  ```

* **M5 (gyro bias/drift)** — skip it. Unmeasurable on this sensor; see B5.
* **M7 (torque tracking)** — deferred. Needs a controlled static load and a commanded-torque
  sweep, which a policy run cannot provide. Separate bench session.

### Expected disk use

A 10-minute stand writes roughly 13 MB of raw CAN timestamps plus ~5 MB of tick log. Six
captures (3 policies x 2 phases) land near 80 MB in `docs/measurements/`. Keep the raw sidecars
until the analysis is settled — they are what lets any number here be recomputed.

---

## The measurements

Priority order. M1–M4 unblock the most training-side work.

### M1 — CAN frame arrival timing, per joint

**Question:** what is the distribution of inter-frame intervals for each motor's PDO4
fast-frame, per bus?

**Why:** PDO4 is configured for 100 Hz on all 12 joints. If the real distribution is tight
around 10 ms, observation delay is a fixed offset and needs no randomization. If it is broad or
bimodal, it needs a modeled range. This single number decides whether we add a delay term at all.

**How:** passive `candump` with hardware or kernel timestamps on **both** leg buses
simultaneously, during a real walk run. Read-only, safe alongside the daemon.
`scripts/can_monitor.py` already decodes the node→joint map and frame types (`0x9` = PDO4) and
its config path is correct — it is the right starting point, but it aggregates to a per-second
rate. We need the raw per-frame timestamps, not the rate.

**Report per joint:** mean, median, p95, p99, max inter-frame interval; stddev; count of
intervals > 2× nominal; total frames; capture duration. Plus per-bus kernel counter deltas.

---

### M2 — Sample age at policy tick

**Question:** at the instant the policy builds its observation vector, how old is each joint's
most recent position/velocity sample?

**Why:** this becomes the per-joint observation delay in training. It is the direct analogue of
Asimov's technique — they group joints by position in the CAN poll schedule and serve
earlier-polled joints staler data (their figures: 6–9 ms stale for the first group, 3–5 ms
middle, 0–2 ms last).

**Our structure is two buses of six**, so we should expect *two* parallel stagger patterns, not
one — and the two buses may or may not be phase-aligned with each other. Whether left and right
legs see systematically different sample ages is itself worth reporting, because an asymmetric
delay between legs is a very different thing for a walk policy than a uniform one. Measure it,
don't copy Asimov's grouping.

Worth knowing: Asimov's published config *documents* this slot table in detail but does not
actually set the lag values on the observation terms. The idea is sound; their shipped code does
not implement it. We should implement what we measure.

**How:** timestamp each joint's last-received PDO4 in the daemon or the client, and log
`tick_time - last_rx_time` per joint on every policy tick. Needs a new field in
`StepRecorder.record()`.

**Report per joint:** mean, median, p95, max age in ms; whether joints cluster into groups by
age (report the grouping — that is the structure we will model); and whether the two buses differ.

---

### M3 — Dropout and stale-hold rate

**Question:** how often does a joint's value repeat across consecutive policy ticks because no
new sample arrived, and how long do those repeat runs last?

**Why:** this maps to a "sensor refreshes slower than control" term, not to added noise. Adding
noise to model a dropout is wrong — it produces a jittery signal where the real one is *frozen*.
The policy runs at 25 Hz against a 100 Hz feed, so under nominal conditions this should be near
zero. Any significant nonzero result is important.

**Watch out for B1 here.** The JSON is fixed, but if the *device* still holds
`velocity_filter_alpha = 0.0`, `left_hip_yaw` will show a 100% stale-hold rate on velocity — a
firmware config bug masquerading as a transport dropout. Do the B1 device readback first, or this
measurement will produce a confidently wrong training term.

**How:** detectable from per-tick JSONL if joint values are logged at full precision — count
consecutive identical `joint_pos` entries per joint. Cross-check against M1's gap counts and the
daemon's `bus_health` (`tx_dropped`, `rx_frames` per interface).

**Report per joint:** fraction of ticks serving a repeated sample; histogram of repeat-run
lengths; and separately, any hard dropouts (node SILENT > 1.5 s, the daemon's OFFLINE threshold).

---

### M4 — IMU frame timing and health

**Question:** what is the real inter-frame interval distribution from the IM10A, and how often is
a sample stale at the moment the policy reads it?

**Why:** `SerialImuBaseState` gates on `stale_after_s = 0.1` and marks the sample invalid past
that. We need to know how close to that gate we normally run, and what the policy is fed when it
trips. Note that on a trip it returns a *constant upright* gravity vector and *zero* angular
velocity — the same values as the stub — so a staleness trip is indistinguishable from B3 in the
logged output unless `base_valid` is checked. Log `base_valid`.

**How:** log arrival timestamps in `WitMotionReader._run()`, and log sample age at each policy
tick. `frames_total` already exists as a counter. Run at the daemon's real baud (921600) and at
the validated effective output rate (see the IMU rate section above). **Resolve B3 first** — this
measurement is meaningless against `UprightStubBaseState`.

**Report:** inter-frame interval mean/p95/max; dropped or checksum-failed frame rate; count of
staleness-gate trips per minute; and per-frame-type rates (0x51 accel, 0x52 gyro, 0x53 euler,
0x59 quaternion) since they may not all arrive at the same rate. Also report the duplicate-frame
fraction per type — that is the 200 Hz validation from the section above.

---

### M5 — Gyro bias and drift at rest

**Question:** with the robot powered, warm, and completely still, what is the per-axis gyro mean
offset, and how does it evolve over ~10 minutes?

**Why:** a constant bias and a slow-drifting bias are different failure modes needing different
sim terms. IsaacLab ships `NoiseModelWithAdditiveBiasCfg`, which resamples a bias at each episode
reset — the right shape for this, and currently unused by us. Zero-mean per-step noise, which is
all we model today, cannot represent a bias at all.

This also decides whether any *integrated* quantity is usable. Menlo removed integrated velocity
from their observations specifically because it drifts.

**How:** robot stationary on a stable surface, powered and thermally settled. Log raw gyro and
accel for 10+ minutes. Repeat at least twice. **Resolve B3 first** — the stub reports exactly
zero angular velocity, which would read as a perfect, bias-free gyro.

**Report:** per-axis mean and stddev; drift rate (deg/s per minute) over the window; ideally an
Allan deviation curve, but a simple drift-vs-time plot is enough to make the call. Also report
the accel-derived gravity vector's deviation from vertical — the IMU mounting is
identity/x-fwd,y-left,z-up per bring-up, and this is the check that it still holds.

---

### M6 — Encoder resolution and noise floor

**Question:** with a joint commanded to hold a fixed position under load, what is the
quantization step of the reported position and the residual noise around it?

**Why:** directly tests whether our `joint_pos` U(±0.05) rad — about 2.9° — is real. **We already
know the analytical answer for the quantum: 1.02e-4 rad (0.0059°)**, from `cpr` 4096 and gear 15,
uniform across all 12 leg joints. So this measurement is a verification with a specific
prediction, and the interesting output is the *residual noise above* that floor, not the floor
itself.

If the measured noise lands anywhere near the quantum, we are training the policy to hedge
against roughly 500× more position uncertainty than exists — exactly the failure Menlo warns
about, and probably the single highest-value correction in this document.

**How:** `scripts/hold_pose.py` or `go_to_pose.py` to hold, log reported position at full rate
for 60 s per joint under a representative load. All 12 leg joints are the same motor family and
configuration, so a representative sample (one hip, one knee, one ankle, both sides) is
sufficient — but include `left_hip_yaw` explicitly, as the B1 control case.

**Report per joint:** smallest nonzero delta between consecutive distinct readings (the measured
quantization step, compare against 1.02e-4 rad); stddev of the residual after removing any slow
trend; peak-to-peak.

Same treatment for `joint_vel`, which matters more: velocity is differentiated from position and
therefore much noisier. The firmware applies `velocity_filter_alpha = 0.7154` before we ever see
it — report the noise floor of the **filtered** signal, since that is what the policy consumes.
(And confirm the B1 device readback is clean, or `left_hip_yaw` will report a velocity noise floor
of exactly zero — which would look like a beautifully quiet sensor rather than a dead one.)

---

### M7 — Torque and current tracking under load

**Question:** commanded torque vs reported current/torque, under a known static load.

**Why:** validates the bench-fit stick-slip actuator model against the *deployed* firmware, which
is not the same thing as the bench. The firmware applies `torque_filter_alpha = 0.1454` (roughly
50 Hz at the 2 kHz position loop); the sim's actuator model has no equivalent filter. If that
filter materially shapes the torque response, it belongs in the sim.

**How:** hold a joint against a known load, sweep commanded torque, log commanded vs reported.
`scripts/bench_sweep.py` may already cover part of this — check before writing new tooling.

**Report:** commanded-vs-reported curve per joint; deadband; apparent time constant of the torque
response; and any asymmetry between directions. Since all 12 leg joints share a configuration, a
representative sample is fine — but note whether hips, knees and ankles differ mechanically
enough to warrant separate fits.

---

### M8 — Thermal drift

**Question:** do M1, M2, M6, and M7 give the same answers at minute 1 and at minute 20 of
continuous operation?

**Why:** this decides whether a parameter goes into domain randomization or gets fixed in
hardware. Menlo found CPU thermal throttling was "quietly degrading actuator responses over long
runs" and their fix was a fan, not a wider randomization range. If our numbers are stable, we
model tight distributions. If they drift, we either widen the sim ranges or fix the thermals —
and we should know which.

**How:** run M1/M2 during a long continuous walk or hold session, bucketed by elapsed time. Log
CPU temperature and any throttling flags alongside. Repeat M6/M7 cold and warm.

**Report:** each statistic bucketed into minute 0–2, 8–10, and 18–20 windows, plus CPU temp and
throttle state over the same window.

---

## Output format

Write results as JSON so the training side can consume them directly, plus a short markdown
summary for humans. Suggested location: `docs/measurements/` in this repo.

```jsonc
{
  "capture_id": "walk_20260901T1200",
  "date": "2026-09-01",
  "duration_s": 300,
  "conditions": {
    "activity": "walk | hold | rest | bench",
    "base_state_source": "SerialImuBaseState | TelemetryBaseState | UprightStubBaseState",
    "b1_device_readback": {
      "left_hip_yaw_velocity_filter_alpha": 0.0,
      "left_hip_yaw_torque_filter_alpha": 0.0,
      "left_hip_roll_bus_voltage_filter_alpha": 0.0
    },
    "policy_hz": 25,
    "daemon_hz": 200,
    "pdo4_hz": 100,
    "imu_baud": 921600,
    "imu_rate_hz_configured": 200,
    "imu_rate_hz_effective": 0,
    "cpu_temp_start_c": 0,
    "cpu_temp_end_c": 0,
    "notes": "anything unusual — a fault, a reboot, a loose fixture"
  },
  "per_joint": {
    "left_hip_pitch": {
      "bus": "can_left_leg",
      "node_id": 5,
      "frame_interval_ms":  {"mean": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0, "std": 0},
      "sample_age_ms":      {"mean": 0, "p50": 0, "p95": 0, "max": 0},
      "stale_hold_fraction": 0.0,
      "stale_run_lengths":  {"1": 0, "2": 0, "3+": 0},
      "encoder_quantum_rad": 0.0,
      "encoder_quantum_rad_predicted": 1.02e-4,
      "pos_noise_std_rad": 0.0,
      "vel_noise_std_rad_s": 0.0
    }
  },
  "imu": {
    "frame_interval_ms": {"mean": 0, "p95": 0, "max": 0},
    "per_type_rate_hz": {"0x51": 0, "0x52": 0, "0x53": 0, "0x59": 0},
    "duplicate_frame_fraction": {"0x51": 0.0, "0x52": 0.0, "0x53": 0.0, "0x59": 0.0},
    "dropped_frame_rate": 0.0,
    "checksum_fail_rate": 0.0,
    "staleness_trips_per_min": 0.0,
    "gyro_bias_dps": [0, 0, 0],
    "gyro_drift_dps_per_min": [0, 0, 0],
    "gyro_noise_std_dps": [0, 0, 0],
    "gravity_vector_deviation_deg": 0.0
  },
  "bus": {
    "can_left_leg":  {"bus_off": 0, "error_passive": 0, "arb_lost": 0,
                      "bus_errors": 0, "rx_frames": 0, "tx_dropped": 0},
    "can_right_leg": {"bus_off": 0, "error_passive": 0, "arb_lost": 0,
                      "bus_errors": 0, "rx_frames": 0, "tx_dropped": 0}
  }
}
```

Every number should be traceable to a capture. If something could not be measured, say so
explicitly rather than filling in a plausible value — an honest gap is more useful to us than a
guess, because a guess will silently become a training parameter.

## What happens next

These numbers come back to `humanoid-policy` and map onto specific terms:

| Measurement | Training term it feeds |
|---|---|
| M1, M2 | Per-joint observation delay (custom `ModifierBase`; IsaacLab has no per-term delay built in). Two-bus structure → possibly asymmetric left/right delay |
| M3 | Stale-hold / refresh-period term — a hold, not added noise |
| M4 | IMU observation delay and staleness handling |
| M5 | `NoiseModelWithAdditiveBiasCfg` on `base_ang_vel`; decides if integrated quantities are usable |
| M6 | Replaces the guessed `joint_pos` / `joint_vel` noise scales; adds quantization at 1.02e-4 rad |
| M7 | Validates or corrects the stick-slip actuator model; possible action/torque filter |
| M8 | Decides randomization *width* — or tells us to fix hardware instead |
