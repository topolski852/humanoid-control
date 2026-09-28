# measC-full — hardware result — 2026-09-28

**Policy:** `walk_measC-full_2026-09-28` (humanoid-policy `88c31a7`)
**Capture:** `hold_20260928T183554_measC-stand_*` — 540 s stand, ended in an ESC fault
**Verdict: the best standing policy measured to date, by a wide margin.** The round-2 premise
held: restoring the observation uncertainty margin fixed measA's lurching.

Also, and separately: **the right knee is a marginal part, not a policy problem.** See §3.

---

## 1. Standing — measC is 10× better than the previous best

Phases were run deliberately: settle, then a long untouched hold, then small pushes, then one
hard push that faulted the knee.

| phase | tilt med | tilt p95 | **tilt rate p95** | excursions >8° |
|---|---|---|---|---|
| 0–120 s settle | 2.70° | 2.72° | 0.06 °/s | 2 |
| **120–300 s UNTOUCHED** | **2.71°** | **2.73°** | **0.04 °/s** | **0** |
| 300–400 s small pushes | 2.77° | 3.36° | 7.95 °/s | 3 |

### Against every previous bundle, untouched

| policy | tilt med | tilt rate p95 | excursions >8° | joint vel p99 |
|---|---|---|---|---|
| **measC** | **2.71°** | **0.04 °/s** | **0 / 3 min** | **0.67 rad/s** |
| smoothA (previous best) | 5.34° | 0.40 °/s | 0.2 /min | 0.73 rad/s |
| smoothB | 5.26° | 0.10 °/s | 0 | 0.73 rad/s |
| measA (regression) | 2.89° | 20.60 °/s | 11.7 /min | 5.09 rad/s |

**Tilt rate is 10× better than smoothA and 500× better than measA.** It also stands
*straighter* — 2.71° median against 5.3° for both smoothA and smoothB, i.e. roughly half the
lean. Over 180 s untouched the tilt p95 and max are identical to two decimal places (2.73°),
which is a policy holding a position rather than drifting and correcting.

Joint velocity while untouched peaked at **0.73 rad/s**, against measA's 5.09 p99. Nothing near
any limit.

**The round-2 hypothesis is confirmed.** measA collapsed observation noise to the measured sensor
floor and lurched at 20.6 °/s; measC restored an explicit uncertainty margin on top of the
measured staleness and quantisation, and the lurching is gone. That was the whole premise and it
transferred from sim.

---

## 2. Push recovery — normal, no limit cycle

17 disturbance events. Ring duration, excluding the hard push that faulted:

* median ~1.5 s, range 0.5–6.2 s
* dominant frequency 0.22–0.67 Hz throughout

| policy | ring duration | frequency |
|---|---|---|
| smoothA | 0.2–4.8 s | 0.22–0.56 Hz |
| **measC** | **0.5–6.2 s** | **0.22–0.67 Hz** |
| smoothB | up to **15.8 s** | **4.11 Hz** — limit cycle |

measC behaves like smoothA: slow sway that bleeds off. **Smooth B's 4 Hz limit cycle does not
reproduce here.**

Two reading notes. The `<<< SUSTAINED` flags on events 11–16 are a threshold artifact, not a
finding: those pushes arrive ~1.5 s apart, so the next one begins before the previous settles and
settle time inflates. Ring duration is the robust metric — all of them are 0.8–1.8 s. And the
tool's "UNTOUCHED half" line is meaningless for this capture: it splits the run in halves, and
here the second half contains the pushes and the fault. The real untouched figure is §1's
120–300 s window.

Event #17 (t=404.5 s, peak 3668 ct, ring 20.8 s) **is the fault**, not a limit cycle.

---

## 3. The knee fault — it is the part, not the policy

`right_knee_pitch` (node 8) faulted for the **third time**, again `ERROR_ENCODER_FAULT`. It
emitted **over 1.2 million EMCY frames** (200,000 recorded, 1,045,470 dropped past the cap) in
~20 s, then the bus stopped carrying host traffic.

Error payloads, in order of appearance: `0x2000` ENCODER_FAULT (10,506) → `0x2040`
ENCODER_FAULT + WATCHDOG_TIMEOUT (179,296) → `0x0040` watchdog alone (89). That ordering matches
`can_recovery.py` exactly: the encoder faults first, the EMCY flood then takes the whole 1 Mbit,
no watchdog feed wins arbitration, and the rest of the bus faults on watchdog.

### Velocity is not the trigger

| | velocity at/near fault |
|---|---|
| measA fault (2026-09-25) | 13.02 rad/s |
| **measC fault (2026-09-28)** | **5.84 rad/s** |
| `left_ankle_pitch`, same push | 9.78 rad/s — **no fault** |
| `right_ankle_pitch`, same push | 8.78 rad/s — **no fault** |

The right knee faulted at 5.84 rad/s while two ankles survived 8.8–9.8 rad/s in the same
disturbance. **A velocity threshold does not explain this**, and the joint-velocity hinge —
whether at 2.0 or the corrected 8.0 rad/s — would not have prevented it.

### Load is not the trigger either

Position excursion from the hold point during the hard push, with implied PD torque (kp 45):

| joint | excursion | implied τ | cap | peak vel | outcome |
|---|---|---|---|---|---|
| `left_knee_pitch` | **+0.587 rad** | **26.4 N·m** | 12.0 | 4.30 rad/s | **no fault** |
| `right_knee_pitch` | +0.437 rad | 19.7 N·m | 12.0 | 5.84 rad/s | **FAULTED** |

**The left knee absorbed a larger excursion and more implied torque than the right, and did not
fault.** Both saturated their 12.0 N·m cap; only the right one failed.

### Conclusion

Three faults, all on node 8, and this time at *lower* load and *lower* excursion than the
contralateral joint survived. That is a marginal encoder or a marginal mechanical coupling on
that specific joint, not a property of the policy or of the commanded motion.

**Recommendation: inspect or replace the right knee encoder before the next hardware session.**
It is now the limiting factor on how hard the robot can be tested, and every fault costs a power
cycle plus a recalibration. Until then, treat it as the known weak point rather than reading its
faults as policy failures.

Corollary for training: the velocity hinge at 8.0 rad/s is still defensible as a general safety
term — smoothA peaks at 14.98 rad/s in sim, past the fault threshold — but it should **not** be
expected to prevent this fault, and it should not be tightened further on the strength of it.

---

## 4. Still not measured

* **Ankle torque on hardware.** Sim flags `ankle_pitch` as the worst joint for saturation
  (22–28% of ticks). This capture shows both ankles saturating their 7.0 N·m cap during the hard
  push (implied 12.4 / 13.0 N·m) and reaching 8.8–9.8 rad/s. A reprinted ankle broke before this
  session; the ankles are the next most likely part to fail and their real torque is unknown.
* **Walking.** No walk attempt this round. measC's stand is good enough to justify one.
* **M7** — unchanged, still the only route to the 4× knee torque discrepancy.
* **Encoder drift within a run** — still unmeasured, and now more interesting given node 8.

---

## 5. What to do next

1. **Inspect the right knee encoder** (§3). It gates everything else.
2. **Attempt a walk with measC.** Its stand is the best measured; the gait metrics
   (knee correlation target −0.85, gait frequency against sim's 1.54 Hz) are the open question.
   Expect to support the robot — no policy has walked unsupported.
3. **Add both ankles to the reconstructed-torque table** on the next walk capture. Free, and the
   ankles are now a known failure point.
4. Do **not** tighten the velocity hinge on the strength of this fault (§3).
