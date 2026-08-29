# Sim2Real Measurement Spec

**Purpose:** define exactly what we need measured on the physical robot so that
`humanoid-policy` can model the real signal path in training instead of guessing at it.

This document is the *request*. It does not contain the tools. Whoever picks this up on the
robot should build (or fix) the capture tooling, run the captures, and commit the results.

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
| `base_ang_vel` noise | U(±0.3) rad/s | guessed |
| `projected_gravity` noise | U(±0.05) | guessed |
| `joint_pos` noise | U(±0.05) rad | guessed |
| `joint_vel` noise | U(±2.0) rad/s | guessed, already flagged as suspect |
| Observation delay | **none modeled** | — |
| Gyro bias / drift | **none modeled** | — |
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

## Before building anything: check what already exists

We may already have usable captures. Check these first and report what is there:

1. `_arm_recording/runs/` — per-tick JSONL from `StepRecorder`. As of writing, every file is
   `arm_<limb>_*.jsonl` (arm teleop). A **leg policy** run would be named
   `run_<nanos>_<pid>.jsonl`. If any of those exist, they are the highest-value existing data.
2. Anything written while `HUMANOID_RECORD_DIR` was set during a `scripts/run_policy.py` run.
3. Kernel CAN counters survive since interface bring-up — `ip -details -statistics link show`
   per bus gives cumulative bus-off / error-passive / arb-lost / bus-error counts for free,
   right now, with no capture run needed. Record them before doing anything else.
4. Any `candump` logs, daemon logs, or telemetry dumps lying around from previous walk attempts.

Note what `StepRecorder.record()` does **not** currently log, because these gaps block several
measurements below: no per-joint sample age, no IMU fields at all, no `base_ang_vel` or
`projected_gravity`, no CAN frame timestamps. Extending it is expected work.

---

## Known blockers to fix first

- **`scripts/can_monitor.py` has a stale config path.** Line 26 points at
  `/home/nse/humanoid-studio/configs/humanoid_lite.json`; the repos moved and the file now lives
  at `/home/nse/humanoid/humanoid-studio/configs/humanoid_lite.json`. The script will crash on
  start until this is fixed. Same stale path appears in `README.md` (lines 43, 127).
- **`scripts/imu_monitor.py` defaults to `--baud 9600`**, but the daemon runs the IM10A at
  921600. Any timing measured through the monitor at the wrong baud is meaningless.
- **`humanoid_control/imu.py` `WitMotionReader.__init__` also defaults to baud 9600.** Pass the
  real baud explicitly in any measurement harness.
- **Confirm which `BaseStateSource` is actually live during a walk run.** `runner.py` falls back
  to `UprightStubBaseState`, which returns a *constant* upright gravity vector and *zero* angular
  velocity. If a capture runs against the stub, every IMU measurement below is garbage. Verify
  it is `ImuBaseState` or `TelemetryBaseState` and record which one, in every capture's metadata.

---

## The measurements

Priority order. M1–M4 are the ones that unblock the most training-side work.

### M1 — CAN frame arrival timing, per joint

**Question:** what is the distribution of inter-frame intervals for each motor's PDO4
fast-frame, per bus?

**Why:** PDO4 is configured for 100 Hz (`fast_frame_frequency: 100` in the studio config). If the
real distribution is tight around 10 ms, observation delay is a fixed offset and needs no
randomization. If it is broad or bimodal, it needs a modeled range. This single number decides
whether we add a delay term at all.

**How:** passive `candump` with hardware or kernel timestamps on all four leg buses,
simultaneously, during a real walk run. Read-only, safe alongside the daemon.
`scripts/can_monitor.py` already decodes the node→joint map and frame types (`0x9` = PDO4) — it
is the right starting point, but it currently aggregates to a per-second rate. We need the raw
per-frame timestamps, not the rate.

**Report per joint:** mean, median, p95, p99, max inter-frame interval; stddev; count of
intervals > 2× nominal; total frames; capture duration.

---

### M2 — Sample age at policy tick

**Question:** at the instant the policy builds its observation vector, how old is each joint's
most recent position/velocity sample?

**Why:** this is the number that becomes the per-joint observation delay in training. It is the
direct analogue of the technique Asimov describes — they group joints by position in the CAN
poll schedule and serve earlier-polled joints staler data (their figures: 6–9 ms stale for the
first group, 3–5 ms for the middle, 0–2 ms for the last). We have four buses and 12 leg joints,
so our stagger pattern will be our own, not theirs. Measure it, don't copy it.

Worth knowing: Asimov's published config *documents* this slot table in detail but does not
actually set the lag values on the observation terms. The idea is sound; their shipped code
does not implement it. We should implement what we measure.

**How:** timestamp each joint's last-received PDO4 in the daemon or the client, and log
`tick_time - last_rx_time` per joint on every policy tick. Needs a new field in
`StepRecorder.record()`.

**Report per joint:** mean, median, p95, max age in ms; and whether joints cluster into groups by
age (if they do, report the grouping — that is the structure we will model).

---

### M3 — Dropout and stale-hold rate

**Question:** how often does a joint's value repeat across consecutive policy ticks because no
new sample arrived, and how long do those repeat runs last?

**Why:** this maps to a "sensor refreshes slower than control" term, not to added noise. Adding
noise to model a dropout is wrong — it produces a jittery signal where the real one is *frozen*.
The policy runs at 25 Hz against a 100 Hz feed, so under nominal conditions this should be near
zero. Any significant nonzero result is important.

**How:** detectable from existing per-tick JSONL if joint values are logged at full precision —
count consecutive identical `joint_pos` entries per joint. Cross-check against M1's gap counts
and against the daemon's `bus_health` (`tx_dropped`, `rx_frames` per interface).

**Report per joint:** fraction of ticks serving a repeated sample; histogram of repeat-run
lengths; and separately, any hard dropouts (node SILENT > 1.5 s, the daemon's OFFLINE threshold).

---

### M4 — IMU frame timing and health

**Question:** what is the real inter-frame interval distribution from the IM10A, and how often is
a sample stale at the moment the policy reads it?

**Why:** `ImuBaseState` gates on `stale_after_s = 0.1` and marks the sample invalid past that.
We need to know how close to that gate we normally run, and what the policy is actually fed when
it trips.

**How:** log arrival timestamps in `WitMotionReader._run()`, and log sample age at each policy
tick. `WitMotionReader.frames_total()` already exists as a counter. Run at the daemon's real baud
(921600) and the flashed output rate (100 Hz per `scripts/imu_setup.py`).

**Report:** inter-frame interval mean/p95/max; dropped or checksum-failed frame rate; count of
staleness-gate trips per minute; and per-frame-type rates (0x51 accel, 0x52 gyro, 0x53 euler,
0x59 quaternion) since they may not all arrive at the same rate.

---

### M5 — Gyro bias and drift at rest

**Question:** with the robot powered, warm, and completely still, what is the per-axis gyro mean
offset, and how does it evolve over ~10 minutes?

**Why:** a constant bias and a slow-drifting bias are different failure modes and need different
sim terms. IsaacLab already ships `NoiseModelWithAdditiveBiasCfg`, which resamples a bias at each
episode reset — exactly the right shape for this, and currently unused by us. Zero-mean per-step
noise, which is all we model today, cannot represent a bias at all.

This also decides whether any *integrated* quantity is usable. Menlo removed integrated velocity
from their observations specifically because it drifts.

**How:** robot stationary on a stable surface, powered and thermally settled. Log raw gyro and
accel for 10+ minutes. Repeat at least twice.

**Report:** per-axis mean and stddev; drift rate (deg/s per minute) over the window; ideally an
Allan deviation curve, but a simple drift-vs-time plot is enough to make the call. Also report
the accel-derived gravity vector's deviation from vertical — that feeds the mounting-rotation
question tracked separately.

---

### M6 — Encoder resolution and noise floor

**Question:** with a joint commanded to hold a fixed position under load, what is the
quantization step of the reported position and the residual noise around it?

**Why:** directly tests whether our `joint_pos` U(±0.05) rad — about 2.9° — is real. Asimov uses
±0.01 rad. If our encoders are cleaner than we assume, we are training the policy to hedge
against 3° of position uncertainty that does not exist, which is exactly the failure Menlo warns
about.

**How:** `scripts/hold_pose.py` or `go_to_pose.py` to hold, log reported position at full rate
for 60 s per joint under a representative load. Do this for at least one joint of each motor
family (M6C12 leg, MAD5010 ankle).

**Report per joint:** smallest nonzero delta between consecutive distinct readings (the
quantization step); stddev of the residual after removing any slow trend; peak-to-peak.

Same treatment for `joint_vel`, which matters more: velocity is usually differentiated from
position and therefore much noisier. Note that the firmware applies
`velocity_filter_alpha = 0.7154` before we ever see it — report the noise floor of the
**filtered** signal, since that is what the policy consumes.

---

### M7 — Torque and current tracking under load

**Question:** commanded torque vs reported current/torque, under a known static load.

**Why:** validates the bench-fit stick-slip actuator model against the *deployed* firmware,
which is not the same thing as the bench. The firmware applies
`torque_filter_alpha = 0.1454` (roughly 50 Hz at the 2 kHz position loop); the sim's actuator
model has no equivalent filter. If that filter materially shapes the torque response, it belongs
in the sim.

**How:** hold a joint against a known load, sweep commanded torque, log commanded vs reported.
`scripts/bench_sweep.py` may already cover part of this — check before writing new tooling.

**Report:** commanded-vs-reported curve per motor family; deadband; apparent time constant of the
torque response; and any asymmetry between directions.

---

### M8 — Thermal drift

**Question:** do M1, M2, M6, and M7 give the same answers at minute 1 and at minute 20 of
continuous operation?

**Why:** this is the measurement that decides whether a parameter goes into domain randomization
or gets fixed in hardware. Menlo found CPU thermal throttling was "quietly degrading actuator
responses over long runs" and their fix was a fan, not a wider randomization range. If our
numbers are stable, we model tight distributions. If they drift, we either widen the sim ranges
or fix the thermals — and we should know which.

**How:** run M1/M2 during a long continuous walk or hold session, bucketed by elapsed time. Log
CPU temperature and any throttling flags alongside. Repeat M6/M7 cold and warm.

**Report:** each of the above statistics bucketed into minute 0–2, 8–10, and 18–20 windows, plus
CPU temp and throttle state over the same window.

---

## Output format

Please write results as JSON so the training side can consume them directly, plus a short
markdown summary for humans. Suggested location: `docs/measurements/` in this repo.

```jsonc
{
  "capture_id": "walk_20260901T1200",
  "date": "2026-09-01",
  "duration_s": 300,
  "conditions": {
    "activity": "walk | hold | rest | bench",
    "base_state_source": "ImuBaseState | TelemetryBaseState | UprightStubBaseState",
    "policy_hz": 25,
    "daemon_hz": 200,
    "pdo4_hz": 100,
    "imu_baud": 921600,
    "imu_rate_hz": 100,
    "cpu_temp_start_c": 0,
    "cpu_temp_end_c": 0,
    "notes": "anything unusual — a fault, a reboot, a loose fixture"
  },
  "per_joint": {
    "left_hip_pitch": {
      "frame_interval_ms":  {"mean": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0, "std": 0},
      "sample_age_ms":      {"mean": 0, "p50": 0, "p95": 0, "max": 0},
      "stale_hold_fraction": 0.0,
      "stale_run_lengths":  {"1": 0, "2": 0, "3+": 0},
      "encoder_quantum_rad": 0.0,
      "pos_noise_std_rad": 0.0,
      "vel_noise_std_rad_s": 0.0
    }
  },
  "imu": {
    "frame_interval_ms": {"mean": 0, "p95": 0, "max": 0},
    "dropped_frame_rate": 0.0,
    "staleness_trips_per_min": 0.0,
    "gyro_bias_dps": [0, 0, 0],
    "gyro_drift_dps_per_min": [0, 0, 0],
    "gyro_noise_std_dps": [0, 0, 0]
  },
  "bus": {
    "can_left_leg": {"bus_off": 0, "error_passive": 0, "arb_lost": 0,
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
| M1, M2 | Per-joint observation delay (custom `ModifierBase`; IsaacLab has no per-term delay built in) |
| M3 | Stale-hold / refresh-period term — a hold, not added noise |
| M4 | IMU observation delay and staleness handling |
| M5 | `NoiseModelWithAdditiveBiasCfg` on `base_ang_vel`; decides if integrated quantities are usable |
| M6 | Replaces the guessed `joint_pos` / `joint_vel` noise scales; adds quantization |
| M7 | Validates or corrects the stick-slip actuator model; possible action/torque filter |
| M8 | Decides randomization *width* — or tells us to fix hardware instead |
