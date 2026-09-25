# measA-full — hardware result and next-training plan — 2026-09-25

**Policy:** `walk_measA-full_2026-09-25` (humanoid-policy `8022b17`)
**Captures:** `walk_20260925T124840_measA-run1_*` (41.4 s, ended in an ESC fault)
**Verdict: a regression.** Operator ranking is **smoothA > smoothB > measA**. measA is the first
bundle trained on measured data and it is the worst of the three on hardware.

---

## 1. What happened

Two attempts, neither sustainable. The second ended in a hard fault at 41 s.

### The fault chain, from the CAN capture

| t | event |
|---|---|
| 29–33 s | `right_knee_pitch` moving normally, 1.0–1.7 rad/s |
| 34–35 s | velocity climbs to **5.77 rad/s** |
| 35–36 s | spikes to **13.02 rad/s** |
| **35.71 s** | knee raises `ERROR_ENCODER_FAULT` (0x2000, confirmed against the firmware enum) and begins flooding EMCY |
| 35.71–41.25 s | **51,484 EMCY frames in 5.5 s ≈ 9,300/s** from that one node |
| ~41.3 s | `can_right_leg` drops off the bus; recovery required a power cycle |

The other five right-leg joints emitted 3–5 EMCY each; the left leg emitted **none**. A single
node flooded and jammed its own bus.

**The encoder is not the root cause.** It faults at the end of a monotonic velocity ramp, not at
a random moment, and the encoders read fine afterwards. The ESC cannot track the encoder at that
speed — a known firmware limitation, superseded by the planned ESC upgrade. **The actionable
item is on the training side: do not command motion that fast.**

### Balance quality, measured

| run | tilt med | tilt p95 | tilt max | **tilt rate p95** | excursions >8° |
|---|---|---|---|---|---|
| smoothA stand (860 s) | 5.34° | 7.81° | 10.48° | **0.4 °/s** | 3 → 0.2/min |
| smoothB stand (710 s) | 5.26° | 5.29° | 6.23° | **0.1 °/s** | 0 |
| measA run1 (26 s) | 2.89° | 5.34° | 15.64° | **20.6 °/s** | 5 → 11.7/min |
| measA run2 (10 s) | 3.59° | 9.14° | 14.19° | **24.6 °/s** | 3 → 17.6/min |

**Tilt rate is the discriminator, not tilt magnitude.** measA moves the body **50–60× faster**
than either predecessor. That matches the operator description exactly — it does not drift and
correct, it lurches, and has to be caught by hand. Excursion rate is ~56–84× smoothA's.

Note smoothB has the *tightest* tilt of all three. Its known problem is the 4 Hz limit cycle
after a push, not standing tilt — so tilt alone does not rank policies, and smoothB's low number
here should not be read as "smoothB balances best".

### Joint velocity — the constraint the ESC imposes

| run | all-joint p95 | p99 | max |
|---|---|---|---|
| smoothA stand | 0.06 | 0.73 | **1.08 rad/s** |
| smoothB stand | 0.06 | 0.73 | 8.05 rad/s |
| measA run1 | **0.74** | **5.09** | **15.49 rad/s** |

smoothA never exceeded 1.1 rad/s while standing. measA's p99 alone is 5.09 — **7× the
predecessors** — and it peaks at 15.5. The ESC faulted at 13.02.

*(`right_knee_pitch` reads p95 = p99 = max = 13.02 because the value froze at the fault; treat
it as the last valid sample, not a distribution.)*

---

## 2. Why it likely regressed — leading hypothesis

**The observation noise was tightened ~1200×, and that removed robustness the policy needed.**

The 2026-09-23 reports found the measured `joint_pos` noise floor is 4.18e-5 rad against a
trained U(±0.05), and recommended shrinking it. measA did exactly that. But the measured floor
is the **sensor** noise floor, not the total uncertainty the policy faces on hardware. The wide
old noise was accidentally covering effects that are real, non-random, and still unmodelled:

* **Calibration offset drift** — `position_offset` resets every power cycle and is recaptured by
  hand. The ±0.05 rad randomisation covering this was never measured (flagged as an open item in
  the hardware plan) and is the same order as the noise that was removed.
* **The unexplained knee load path** — the 4× torque discrepancy between sim and hardware is
  unresolved and blocks on M7. Whatever it is, it is absent from the model.
* **Backlash and link flex** in 3D-printed joints.
* **IMU mounting error**, ground irregularity, a support hand on the robot.

Menlo's rule is *only randomise what's actually random*. That is right, but it applies to
parameters measured as **not random**. Here the measurement established the *sensor* floor while
leaving plant uncertainty unmeasured, and the noise term was carrying both.

**Supporting evidence:** sim already warned. `knee_corr_median` fell to −0.497 from −0.666 on
this bundle while every other sim metric improved, and the commit flagged it as "worth watching
on hardware". It moved the wrong way in sim and is far worse on hardware.

### Alternatives not yet excluded

1. **The raised torque caps.** All 8 M6C12 joints went to 12.0 N·m; the knee rose 11.0 → 12.0.
   More authority with less clean knee coordination is a plausible route to a 13 rad/s excursion.
   Not separable from the noise change without another run, since measA was trained against the
   new caps and cannot be fairly run on the old ones.
2. **The staleness / quantisation model itself** could be mis-specified rather than merely too
   tight. Less likely — both were measured directly — but not excluded.

---

## 3. Recommendations for the next training round

### 3a. Re-introduce observation noise, but as an explicit uncertainty budget

Do **not** revert to U(±0.05). Split the term:

| source | value | basis |
|---|---|---|
| sensor floor | 4.2e-5 rad | measured |
| calibration offset | ±0.02 rad, resampled per episode | **needs measuring** — re-zero a joint 10× and take the spread. Asimov uses ±0.02 |
| unmodelled plant | the remainder | a deliberate robustness margin, documented as such |

The honest framing is that `joint_pos` noise was doing two jobs and only one of them has been
measured. Keep them separate so the next regression is attributable.

### 3b. Add a joint-velocity penalty — hard hardware constraint

The ESC cannot track the encoder above roughly 13 rad/s at the joint, and that is a **hardware
limit until the new ESC lands**, not a tuning preference.

* penalise `|joint_vel|` above **2 rad/s** (smoothA's standing max was 1.08)
* treat **8 rad/s** as a hard ceiling in reward shaping
* the fault occurred at 13.02; leave real margin below it

This is the single change most likely to keep the next bundle alive long enough to measure.

### 3c. Make tilt rate a first-class training metric

Tilt rate separated these three policies where tilt magnitude did not, and it matches the
operator's subjective ranking. Targets from hardware:

| metric | smoothA (best) | measA | target |
|---|---|---|---|
| tilt rate p95 | 0.4 °/s | 20.6 °/s | **< 1 °/s** |
| excursions >8° | 0.2/min | 11.7/min | **< 0.5/min** |
| joint vel p99 | 0.73 rad/s | 5.09 rad/s | **< 1.5 rad/s** |

### 3d. Treat smoothA as the baseline to beat

smoothA is the operator's best policy and the only one that stands unattended for 10 minutes. A
new bundle should be compared against smoothA's numbers above before it is considered an
improvement, not against sim metrics alone. Sim ranked measA best on falls/min and it is the
worst on hardware.

### 3e. M7 remains blocking

Unchanged from the hardware plan. The 4× knee torque discrepancy is a plant modelling error and
no reward shaping addresses it.

---

## 4. Standing constraint the training side should know

**No policy to date has walked without the operator physically supporting the robot.** Every
walk figure in every report — including smoothA's 22 s of gait at −0.64 knee correlation — was
produced with a hand on the robot.

This matters for interpretation:

* Reported gait metrics describe a **supported** walk. An unsupported one is a harder problem
  and has not been demonstrated.
* Sim `fall_rate/min` is not comparable to hardware, because hardware falls are being prevented
  by hand. measA's sim fall rate of 0.020/min is the best of any bundle and it cannot stand alone.
* **Standing unattended is the nearer milestone than walking.** smoothA clears it; smoothB clears
  it until pushed; measA does not. Optimising for unattended standing stability is more likely to
  produce a deployable policy than optimising gait quality.

---

## 5. Suggested order of work

1. Measure the calibration-offset spread (§3a) — cheap, no policy, retires an unmeasured value.
2. Retrain with the velocity penalty (§3b) and the split noise budget (§3a).
3. Compare against smoothA's hardware numbers (§3c) before deploying.
4. Run M7 in parallel — it is independent of the training loop and blocks the knee question.

A walk attempt on measA is possible but low value: it lurches at 20 °/s while standing and
faulted at 41 s. The informative capture would be a **fresh stand** of the next bundle, compared
against the §3c table.
