# Hardware Plan — 2026-09-23

Companion to the training changes landed in `humanoid-policy` after the 2026-09-23 measurement
round. This is the robot-side half: what to change on the ESCs, what to measure next, and in
what order.

Reads from: [REPORT_2026-09-23_smoothA.md](measurements/REPORT_2026-09-23_smoothA.md),
[REPORT_2026-09-23_smoothB.md](measurements/REPORT_2026-09-23_smoothB.md),
[SIM2REAL_MEASUREMENTS.md](SIM2REAL_MEASUREMENTS.md).

---

## Headline: the sim plant is too easy at the knees, by ~300×

Before flashing anything, this is the result that should shape expectations for the next round.

With the sim-side torque metric now implemented (`scripts/rsl_rl/eval_plant_compare.py`), the
**Smooth A bundle was replayed in simulation** and its torque demand compared against what the
same bundle demanded on hardware:

| joint | sim saturation | hardware saturation | sim \|tau\| p95 | hardware \|tau\| p95 |
|---|---|---|---|---|
| `left_knee_pitch` | **0.16%** | **52.1%** | 6.6 N·m | **28.0 N·m** |
| `right_knee_pitch` | **0.17%** | **33.3%** | 7.1 N·m | **23.1 N·m** |
| `left_hip_pitch` | 0.03% | 13.0% | 5.3 N·m | 17.1 N·m |
| `right_hip_pitch` | 0.04% | 37.3% | 5.1 N·m | 24.2 N·m |
| `left_ankle_pitch` | **27.6%** | not measured | 15.3 N·m | not measured |
| `right_ankle_pitch` | **21.6%** | not measured | 14.0 N·m | not measured |

Also: sim gait 1.54 Hz vs hardware 1.05 Hz; knee L/R correlation −0.77 in sim vs −0.64 on
hardware. The sim is healthier than the robot on every axis that was measured.

**What this means.** The Smooth A report's recommendation — add a peak-torque penalty — has now
been implemented, but it cannot fix the knee problem, because **in simulation the knees are not
saturating in the first place.** A reward term prices behaviour the policy actually exhibits; a
penalty on knee overshoot has almost nothing to act on when sim knee p95 is 6.6 N·m against a
12 N·m cap. In sim that penalty will act on the *ankles* instead.

So the dominant sim2real gap is not the reward shaping. It is that the simulated knee tracks its
target while the real knee cannot — the real knee sits 0.208 rad behind, and that error is what
produces the 28 N·m reconstructed demand. Something in the real knee's load path is absent from
the model.

**This makes M7 the critical measurement**, not merely the highest-value unrun one. It is the
only item below that can explain a 4× torque discrepancy at a single joint.

Caveat on the comparison: hardware torque is *reconstructed* as `kp·err − kd·vel`, not measured.
If the real knee saturates, it tracks poorly, which inflates the reconstructed demand — partly
circular. M7 breaks that circularity by measuring commanded vs reported torque directly.

---

## 1. ESC config changes

`humanoid-policy` now derives torque caps **per motor type** rather than per joint. The 12 leg
joints are 8 × MAD M6C12 150KV (hip roll/yaw/pitch, knee) and 4 × MAD 5010 200KV (ankle
pitch/roll) — confirmed three ways: `torque_constant` in the studio config, the
`_LEG_GROUP`/`_ANKLE_GROUP` split, and the motor mass audit in
`humanoid-policy/source/humanoid_policy_assets/humanoid_policy_assets/configs/actuators/PROVENANCE.md`.

Eight physically identical M6C12 motors were carrying four different caps (6.0, 9.5, 11.0, 12.0)
with `hip_yaw` asymmetric L/R. Flash these instead:

| joint (both sides) | motor | `torque_limit` | was | `current_limit` | was |
|---|---|---|---|---|---|
| hip_roll | M6C12 | **12.0** | 6.0 | **20.0** | 20.0 |
| hip_yaw | M6C12 | **12.0** | L 12.0 / R 6.0 | **20.0** | L **10.0** / R 20.0 |
| hip_pitch | M6C12 | **12.0** | 9.5 | **20.0** | 20.0 |
| knee_pitch | M6C12 | **12.0** | 11.0 | **20.0** | 20.0 |
| ankle_pitch | MAD5010 | **7.0** | 6.0 | **20.0** | 20.0 |
| ankle_roll | MAD5010 | **7.0** | 7.0 | **20.0** | **6.0** |

Value rationale: the highest cap already in service on that motor type. Nothing loses authority,
nothing exceeds what an identical motor already runs. Both land well inside the physical ceiling
(`Kt · gear · current_limit`): M6C12 12.0 of 26.87 N·m (45%), MAD5010 7.0 of 19.76 N·m (35%).
The headroom is deliberate — the policy has to learn it cannot have 28 N·m, and the August
6.0 → 11.0 knee raise already showed that removing the cap removes an accidental low-pass on the
policy's 4–5 Hz command content.

### Two limits are currently unreachable

The bolded `current_limit` values are bugs, from Smooth B §2a:

- Both `ankle_roll` run **6 A** where everything else runs 20 A. Ceiling is
  `0.06588 × 15 × 6 = 5.93 N·m`, so the configured 7.0 N·m **can never be produced** — current
  binds first. Training has been asking for 7.0 N·m from a joint that tops out at 5.93.
- `left_hip_yaw` at **10 A** gives 13.44 N·m, only 12% over its 12.0 cap — no room for Kt error
  or rail sag.

### How to apply

`scripts/write_policy_gains.py` **will not do this.** It writes `position_kp` / `velocity_kp`
only and explicitly reads back and preserves `torque_limit` unchanged. These changes must go into
the studio config (`humanoid-studio/configs/humanoid_lite.json`) and be applied via
`connect` / `apply_all_configs`.

Remember the ordering trap documented in that script: a `connect` rewrites gains from the studio
config, so the sequence is **connect → write_policy_gains → arm → test**, without reconnecting in
between.

### Verify after flashing

Read back all 12 and confirm `torque_limit` and `current_limit` match the table, then confirm the
exported `leg_policy_contract.json` from the next training run agrees. The contract is generated
from the training-side `_CONTRACT_EFFORT`, so the two cannot drift silently once both are set.

---

## 2. Resolve the `torque_constant` question — blocks report comparison

Smooth B §2b records `torque_constant` changing **0.08958 → 0.06588** at 10:56 on 2026-09-23 on
`left_knee_pitch`, `right_knee_pitch`, `left_hip_roll` and `right_hip_pitch`.

That assignment implies **6 M6C12 / 6 MAD5010**, which contradicts the 8/4 split confirmed three
independent ways above. Those four joints are physically M6C12, so 0.06588 is the wrong constant
for them.

Two consequences:

1. **It is almost certainly a config corruption, not a motor swap.** Restore 0.08958 on those
   four joints. A stale copy of the studio config on the training PC still carries the correct
   8/4 `Kt` values and can be used as a reference for that field only — it is *out of date* on
   `torque_limit`, so do not copy it wholesale.
2. **Until it is resolved, the two reports are not strictly comparable.** Smooth A was captured
   at 10:24/10:46 and Smooth B at 11:00, i.e. either side of the change. If it was a swap they
   are different hardware; if it was corruption they are the same. Settle this before drawing any
   A-vs-B conclusion.

A wrong `torque_constant` also means the ESC's current→torque conversion is wrong, so any torque
telemetry from those four joints in the Smooth B window is suspect.

---

## 3. Measurements, in priority order

### M7 — commanded vs reported torque under static load — **CRITICAL**

Promoted from "highest-value unrun" to blocking, on the strength of the sim comparison above.
This is the only measurement that can explain a 4× knee torque discrepancy.

Hold a joint against a known static load, sweep commanded torque, log commanded vs reported.
`scripts/bench_sweep.py` may already cover part of this — check before writing new tooling.

Report per motor family: commanded-vs-reported curve, deadband, apparent time constant, and any
direction asymmetry. Include the firmware's `torque_filter_alpha = 0.1454` (~50 Hz at the 2 kHz
position loop) in the analysis — the sim actuator model has no equivalent filter, and 4 Hz is
where Smooth B rings.

Do the knees first, and at a bent-knee load representative of the walk posture rather than at
zero load.

### Ankle torque saturation on hardware — new gap

The reports characterised knees and hip_pitch but not ankles. Sim now says `ankle_pitch`
saturates 22–28% of ticks at 15.3 / 14.0 N·m p95 against a 7.0 cap — by far the worst joint in
simulation. Whether the real ankles do the same is unknown. Add both ankle joints to the
reconstructed-torque table in the next capture; it is free, the data is already logged.

### `joint_vel` noise floor — no robot time needed

The one training value in the new observation model that is derived rather than measured
(currently U(±0.05) rad/s, ~8× above the derived floor, ~40× tighter than the old U(±2.0)).

PDO4 carries velocity in payload bytes 4–7, and the `.frames.jsonl` sidecars from the existing
captures are already on the robot PC. Apply the same short-window median method
`scripts/measure/analyse.py` uses for position — **and the same warning**: do not linearly
detrend a long capture, use 0.2 s windows and take the median across them.

### Gyro noise during motion — the only route around B5

B5 established that the IMU auto-zeroes its gyro at rest, so the noise floor cannot be measured
standing still. But the **high-frequency residual of the gyro during the existing walk capture**
is not suppressed by that clamp: subtract a smoothed version of the signal from itself and
characterise what remains.

This is the only path to replacing the guessed `base_ang_vel` U(±0.3) rad/s, which after this
round is one of only two remaining un-measured observation values (the other being
`projected_gravity` U(±0.05)). For scale: the WitMotion's LSB is ~0.001 rad/s and the stand
capture's max |ω| was 1.920 rad/s, so ±0.3 is very likely far too wide — but the project's rule
is to measure rather than infer, so it stays untouched until this is run.

### Smooth B limit-cycle rate

Smooth B rang at 4.11 Hz for 15.8 s without decaying after one push, but only **two** disturbance
events were captured and only one went unstable. The effect is unambiguous; its probability is
not.

Repeat the 600 s stand with **8–10 deliberate pushes ~30 s apart**, each left to run free rather
than damped by hand. The metric is ring duration per event — settle time is unreliable below
~50 ct peak because the 10%-of-peak target lands in the noise floor.

### Calibration offset — unmeasured, low priority

Training randomises joint zero-offset by ±0.05 rad (`add_all_joint_default_pos`). That value was
never measured; Asimov uses ±0.02. Re-zero a joint several times and record the spread of the
resulting offsets. Cheap, and it would retire the last unmeasured event-level randomisation.

---

## 4. What NOT to do

Established by measurement across two independent 600 s captures. Recorded here so it is not
relitigated:

- **No stale-hold / dropout modelling.** 0.00% stale-hold on every joint across 1.4M frames,
  despite 0.10% transport-layer frame loss. The loss is real and completely invisible at the
  observation layer — the policy runs at 25 Hz against a 100 Hz feed, so a single lost frame
  still leaves fresh data.
- **No gyro bias term.** Unmeasurable on this IMU; do not populate it with a plausible guess.
- **No thermal widening, and no cooling fix.** 54–70 °C, zero throttle events, temperature *fell*
  9 °C over both runs, sample age identical between minute 0–2 and 8–10. Menlo's thermal finding
  does not reproduce here.
- **No per-joint observation-delay stagger.** Cross-joint age spread is 0.17–0.84 ms against a
  10 ms frame period. There is no poll-slot structure on this 2-bus robot; Asimov's per-CAN-slot
  model does not transfer.
- **Do not chase the frame-loss asymmetry.** Four joints lose ~0.025% and the other eight
  0.12–0.18%. Unexplained, uniformly distributed in time, and invisible downstream.

---

## 5. The loop, going forward

Most of the measurement work does not repeat. M1/M3/M6/M8/B5 are hardware constants and already
reproduced across two independent captures (frame loss 0.1058% vs 0.0995%, sample age 4.78 vs
4.84 ms, noise floor 0.409 vs 0.423 ct). Re-running them each cycle spends a robot session
re-learning a constant.

**Four metrics are policy-dependent and worth re-measuring every cycle:**

1. knee torque saturation fraction (target: 52% → <10%)
2. knee L/R correlation during gait (target: −0.64 → −0.85)
3. ring duration after a push
4. gait frequency (hardware 1.05 Hz vs sim 1.54 Hz)

And the cheap half of the loop now exists in sim. Before spending a robot session, run:

```bash
EVAL_GAIT=1 python scripts/rsl_rl/eval_plant_compare.py --plant modeled \
    --num_envs 128 --steps 600 --load_run <run> --headless --out eval.json
```

which reports `torque_sat_frac`, `torque_p95_nm`, `torque_peak_nm` per joint alongside
`gait_hz_median` and `knee_corr_median` — directly comparable to the hardware table. Where those
two disagree, as they now do at the knee, the gap is in the **plant**, and no amount of reward
tuning will close it.
