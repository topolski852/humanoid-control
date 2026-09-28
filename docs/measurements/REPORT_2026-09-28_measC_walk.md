# measC-full — walk attempt — 2026-09-28

**Capture:** `walk_20260928T185838_measC-walk1_*` — 126 s, ended in an ESC fault
**Companion:** [REPORT_2026-09-28_measC.md](REPORT_2026-09-28_measC.md) (the stand result)

**Headline: it walked, and the bottleneck moved from the knees to the ankles.**

---

## 1. Gait

10 s of genuine stepping, t=48–60 s.

| metric | measC (hardware) | smoothA (hardware) | measC (sim) |
|---|---|---|---|
| knee L/R correlation | **−0.65** | −0.64 | −0.708 |
| **gait frequency** | **1.70 Hz** | 1.05 Hz | 1.54 Hz |
| seconds of gait | 10 s | 22 s in bursts | continuous |
| in-phase (SAG) | 51 s at +0.91 | 46 s at +0.88 | — |

**Gait frequency is the real improvement.** smoothA walked at 1.05 Hz against a 1.54 Hz sim
prediction — a 32% shortfall. measC walks at **1.70 Hz against 1.54 predicted**, i.e. it now
*overshoots* slightly rather than dragging. That is the closest hardware/sim gait agreement
measured.

Knee correlation is unchanged from smoothA (−0.65 vs −0.64) despite sim predicting −0.708, so the
sim gain on that metric did not transfer. Still clearly alternating and nowhere near the +0.91
in-phase signature.

51 s of in-phase knees at +0.91 sits alongside the 10 s of gait — the same torque-starvation
signature smoothA showed (46 s at +0.88). It has not gone away.

---

## 2. The ankles are now the binding constraint

Reconstructed torque (`kp·err − kd·vel`, kp 45 / kd 1.5) over the 38 s the policy ran:

| joint | cap | mean err | τ p95 | τ max | **sat %** |
|---|---|---|---|---|---|
| `left_ankle_pitch` | **7.0** | +0.112 | **27.52** | **50.93** | **27.2%** |
| `right_ankle_pitch` | **7.0** | +0.071 | 17.72 | 25.69 | 22.8% |
| `left_hip_pitch` | 12.0 | −0.042 | 25.55 | 31.76 | 33.9% |
| `right_knee_pitch` | 12.0 | −0.141 | 32.56 | 38.50 | 29.7% |
| `right_hip_pitch` | 12.0 | +0.063 | 25.88 | 34.63 | 29.0% |
| `left_knee_pitch` | 12.0 | −0.043 | 21.88 | 41.82 | 25.1% |

### The knees improved

| | smoothA | measC |
|---|---|---|
| `left_knee` saturation | 52.1% | **25.1%** |
| `right_knee` saturation | 33.3% | 29.7% |
| `left_knee` mean error | −0.208 rad | **−0.043 rad** |

Left-knee saturation halved and the static droop is largely gone (−0.208 → −0.043 rad). The
torque-saturation reward did something.

### But `left_ankle_pitch` now demands 27.5 N·m against a 7.0 cap

That is **3.9× the cap** and — more seriously — **above the MAD5010 physical ceiling of
19.76 N·m** (`Kt · I · gear` = 0.06588 × 20 × 15). Peak demand touches 50.93 N·m, 2.6× the
ceiling.

**Sim predicted this precisely.** The hardware plan recorded sim `ankle_pitch` saturating 22–28%
of ticks at 15.3 / 14.0 N·m p95 against the 7.0 cap, and called it "by far the worst joint in
simulation" with hardware ankle torque unmeasured. Hardware now measures 22.8–27.2% saturation.
The prediction transferred; the measurement had simply never been taken.

So the torque problem was not solved, it **moved**. The reward pushed demand off the knees and
onto the ankles, which have a cap 5 N·m lower and a ceiling 7 N·m lower.

---

## 3. A different joint faulted — `right_ankle_pitch`

`right_ankle_pitch` raised `ERROR_ENCODER_FAULT` at t=74.09 s and emitted **~481,000 EMCY
frames** (197,514 recorded, 284,387 past the cap) over ~21 s. Payload ordering was the familiar
`0x2000` → `0x2040` → `0x0040`.

This is the **first ankle fault**; the previous three were all `right_knee_pitch`. Both faulting
joints are on the right leg.

Unlike the knee faults, this one has an obvious mechanical cause: that joint was demanding
17.7 N·m p95 and 25.7 N·m peak from an actuator whose physical ceiling is 19.76 N·m, with a 7.0
cap. It is being asked for more than it can deliver, continuously, during gait.

Before the fault it also degraded measurably rather than failing cleanly: 4.17% stale-hold,
267 ms maximum gap, mean sample age 10.2 ms against 4.9 ms for every other joint. **It is the
only joint in any capture to show non-zero stale-hold.**

---

## 4. An analysis false positive, fixed

The first run of `analyse.py` reported **"joints cluster into distinct poll slots — model the
grouping"** off a 5.40 ms cross-joint age spread. That was wrong, and it is the kind of error
that would have put a fake per-joint stagger into training.

The spread came entirely from `right_ankle_pitch`, whose age was inflated to 10.2 ms by its own
dropouts. Excluding degraded joints, the spread is **0.25 ms** — consistent with every previous
capture, and the verdict returns to "no slot structure; do NOT add a per-joint stagger".

`slot_structure()` now excludes any joint with stale-hold above 0.5% and names it in the result.
A dropout is a fault to fix, not structure to model.

---

## 5. What this means for the next round

1. **Raise the ankle caps, or reduce ankle demand.** `ankle_pitch` demands 27.5 N·m p95 against a
   7.0 cap and a 19.76 N·m ceiling. The options are not equivalent:
   * raising the cap toward the ceiling (say 12 N·m, 61% of ceiling, matching the M6C12 ratio)
     gives the joint authority it currently lacks — but the ankle is 3D-printed and has already
     broken once mechanically.
   * reducing demand in training is safer for the part but needs a reward term, and the last
     attempt at that (the knee torque penalty) moved load rather than removing it.

   **Recommendation: reduce demand, not raise the cap.** The mechanical part is the weak link and
   50.93 N·m peak demand at an ankle is not a cap problem.

2. **The torque reward needs to be global, not per-joint.** Penalising knee saturation moved the
   load to the ankles. A term over *all* joints normalised by each joint's own cap would price
   the redistribution instead of rewarding it.

3. **Knee correlation did not transfer** (sim −0.708, hardware −0.65). Worth noting before
   trusting that metric as a sim-side predictor.

4. **Gait frequency now tracks sim** (1.70 vs 1.54 predicted, against smoothA's 1.05 vs 1.54).
   Real progress, and worth keeping whatever produced it.

5. **Right leg, both faulting joints.** `right_knee_pitch` ×3 and now `right_ankle_pitch`.
   Inspect both encoders. If the right leg has a systematic issue — wiring, connector, bus
   termination — that is cheaper to find than to keep replacing parts.

---

## 6. Unchanged

* **M7** still blocking. The knee sim/hardware torque gap remains, and the ankle result makes a
  direct torque measurement more valuable, not less.
* **No unsupported walking.** This attempt was supported like all previous ones.
* Transport is clean on the eleven healthy joints: 0.00% stale-hold, 0.25 ms age spread.
