# Sim2Real Measurement Results — Smooth B — 2026-09-23

**Robot:** humanoid_lite, legs only (12 joints, 2 CAN buses), host `nse-MINI-S`
**Policy under test:** `walk_smoothB-full_2026-08-29`
**Companion report:** [REPORT_2026-09-23_smoothA.md](REPORT_2026-09-23_smoothA.md)
**Capture:** 600 s stand, `hold_20260923T110050_smoothB-stand_*`
**No walk data** — this round is stand only. Walk figures live in the Smooth A report.

Both bundles predate the measurement work, so neither was trained on any of this. Reports are
kept separate because the two captures are **not taken under identical hardware conditions** —
see §5.

---

## 1. The distinguishing finding: Smooth B can enter a sustained limit cycle

The operator's report was that Smooth A settles on its own after a push while Smooth B "keeps
oscillating unless you hold it still". That is measurable, and it reproduces.

Two pushes were applied in the first half of the run, then the robot was left untouched.

| event | peak | ring duration | dominant freq | outcome |
|---|---|---|---|---|
| B #1 (t=6.8 s) | 253 ct | 8.8 s | 0.22 Hz | decayed |
| **B #2 (t=36.0 s)** | **323 ct** | **15.8 s** | **4.11 Hz** | **sustained** |

Motion after B #2, in encoder counts per 0.5 s:

```
323  248  277  236  233  266  228  258  255  211  300  311  248  266  252  255
```

Flat for eight seconds at ~250 counts, 48 zero-crossings. **The amplitude does not decay** —
this is a limit cycle, not a decaying disturbance response.

Smooth A, for the same measurement across 15 pushes:

```
319   65   28  212  262  111   28   10   11   10    4    6    3    3    3    4
```

| | Smooth A (15 events) | Smooth B (2 events) |
|---|---|---|
| ring duration | 0.2 – 4.8 s | 8.8 s and **15.8 s** |
| dominant frequency | 0.22 – 0.56 Hz | 0.22 Hz and **4.11 Hz** |
| decays unaided | every event | 1 of 2 |

The frequency difference is the substantive one. Smooth A's disturbance response is a slow
0.2–0.6 Hz body sway that bleeds off. Smooth B's bad event is a fast **4 Hz** oscillation that
sustains — roughly ten times the frequency, and it does not lose energy.

### Why this matters for training

**A limit cycle is not observation noise and must not be modelled as such.** It is a closed-loop
instability between the policy and the plant: the policy's response to its own induced motion
reinforces rather than damps it. Widening randomisation ranges will not address it and may hide
it. The levers are the ones that set stability margin — gains, action smoothness, modelled
actuator lag, and the gait itself.

Note the direction of the surprise. Smooth B is the *smoother* policy in sim (`jvel_rms` 1.696
vs A's 1.840; `actrate_rms` 0.316 vs 0.357) and it is the one that goes unstable on hardware.
Sim smoothness did not predict hardware stability. Smooth A's better sim falls/min (0.141 vs
0.188) is the metric that tracked reality here.

### Caveat on scope

Only **two** disturbance events, and only one went unstable. The effect is unambiguous where it
occurs — 15.8 s of non-decaying 4 Hz ringing is not marginal — but the *rate* at which a push
triggers it is not established. A follow-up run with 8–10 deliberate pushes would pin that down,
and is the single most valuable next capture for this policy.

### Undisturbed behaviour is fine

With nobody touching it, Smooth B is quiet: median motion 1.5 ct (1.24× baseline), p95 2.75,
p99 3.65, max 8.5. The instability is **disturbance-triggered, not spontaneous**. Smooth A's
untouched figure is 1.8 ct (1.44×), so B is marginally quieter at rest.

---

## 2. Torque limits — flagged for revisit

Three separate problems, all of which desync sim from hardware.

### 2a. Two joints are configured above their physical ceiling

`torque_limit` cannot exceed what the current limit allows: `ceiling = Kt · I_limit · gear`.

| joint | `torque_limit` | `current_limit` | ceiling | status |
|---|---|---|---|---|
| `left_ankle_roll` | 7.0 N·m | 6.0 A | **5.93 N·m** | **unreachable** |
| `right_ankle_roll` | 7.0 N·m | 6.0 A | **5.93 N·m** | **unreachable** |
| `left_hip_yaw` | 12.0 N·m | 10.0 A | 13.44 N·m | 12% headroom |
| every other leg joint | 6.0 – 11.0 | 20.0 A | 19.76 – 26.87 | 2–4× headroom |

Both `ankle_roll` joints run `current_limit` 6.0 A where everything else is 20.0, so current
binds long before torque does. **`_CONTRACT_EFFORT` trains those ankles at 7.0 N·m, which the
robot cannot deliver.** Either raise `current_limit` to make 7.0 reachable, or lower both the
hardware limit and `_CONTRACT_EFFORT` to ~5.9.

`left_hip_yaw`'s 12% margin is not an error but leaves no room for Kt error or rail sag.

### 2b. The knee ceiling is lower than the Smooth A report states

`torque_constant` was changed at **10:56 on 2026-09-23** from 0.08958 → 0.06588 on
`left_knee_pitch`, `right_knee_pitch`, `left_hip_roll` and `right_hip_pitch`.

| | Smooth A report says | current config |
|---|---|---|
| knee motor ceiling | 26.9 N·m | **19.76 N·m** |
| measured knee p95 demand | 28.0 N·m | 28.0 N·m |
| over ceiling by | 1.04× | **1.42×** |

**The conclusion strengthens**: the policy is not marginally over the physical ceiling, it is
42% over. The Smooth A report's number needs correcting.

**OPEN QUESTION, blocks interpretation:** was this a motor swap during the rebuild, or a
correction of a previously wrong constant? If motors were swapped, the Smooth A capture
(10:24 / 10:46) was taken on different hardware from this Smooth B capture (11:00) and the two
are **not directly comparable**. If it was a config correction, both captures are on the same
hardware and comparison is valid.

### 2c. Raising the knee cap did not fix starvation

Carried over from the Smooth A walk data (no walk data exists for Smooth B). Knees saturate
52.1% / 33.3% of ticks at the 11.0 N·m cap; static demand is now under the cap but p95 dynamic
demand is 28 N·m. Full analysis in the Smooth A report §2.

---

## 3. Signal path — matches Smooth A, as expected

These are hardware properties and should not vary by policy. They do not, which is itself a
useful check that the measurement is sound.

| | Smooth A stand | **Smooth B stand** |
|---|---|---|
| frames | 720,313 | **720,423** |
| frame loss | 0.1058% | **0.0995%** |
| sample age mean | 4.78 ms | **4.84 ms** |
| age spread across joints | 0.84 ms | **0.17 ms** |
| **stale-hold** | **0.00%** | **0.00%** |
| encoder quantum | 1.00 ct | **1.00 ct** |
| noise floor (median) | 0.409 ct / 4.18e-5 rad | **0.423 ct / 4.32e-5 rad** |
| trained `joint_pos` noise vs floor | ~1200× | **~1157×** |
| CPU temp | 54–65 °C, fell 9 °C | **56–70 °C, fell 9 °C** |
| throttle events | 0 | **0** |

Every conclusion from the Smooth A report reproduces independently:

* **`joint_pos` noise is ~1157× too wide** — shrink U(±0.05) rad to ~1e-4.
* **No per-joint delay stagger** — spread 0.17 ms, 2% of the frame period. One uniform
  0–10 ms delay term.
* **No stale-hold term** — 0.00% across 720k frames despite 0.0995% transport loss.
* **No thermal widening** — zero throttle events, temperature fell over the run.
* **Encoder quantisation 1.023e-4 rad**, uniform, if quantisation is modelled at all.

---

## 4. What to change in training

Identical to the Smooth A report for items 1–7, plus one specific to this policy:

| # | Change | Evidence |
|---|---|---|
| 8 | **Investigate the 4 Hz closed-loop instability** before deploying Smooth B further | 15.8 s of non-decaying 4 Hz ringing after a single push |

On item 8: 4 Hz sits well inside the band the actuator model governs — the bench-measured
actuator lag is 7.2 ms for legs and 12 ms for ankles, and the firmware's
`torque_filter_alpha = 0.1454` is roughly a 50 Hz pole. If sim's actuator model damps faster
than the real one at 4 Hz, a policy can learn a response that is stable in sim and marginal on
hardware. That is a specific, testable hypothesis and it is exactly the class of gap M7
(commanded vs reported torque, including the firmware filter) was written to close. **M7 has
not been run and is now the highest-value unrun measurement.**

---

## 5. Limitations

1. **Two disturbance events only**, one unstable. The effect is clear; its probability is not.
2. **No walk data for Smooth B.** Torque-saturation figures are carried from Smooth A.
3. **Possible hardware change between captures** — see §2b. Unresolved.
4. **Torque is reconstructed**, not measured (`kp·err − kd·vel`); M7 would validate it.
5. **Limit-cycle frequency is from the CAN position stream**, not an IMU. Body-frame
   oscillation may differ from joint-space oscillation.
6. Settling times for small events (<50 ct peak) are unreliable — the 10%-of-peak target lands
   near the noise floor. **Ring duration is the robust discriminator**, not settle time.

---

## 6. Reproducing

```bash
scripts/measure/capture_run.sh smoothB-stand 600 hold
python scripts/measure/settle_analysis.py \
    --capture docs/measurements/<...>_smoothB-stand_can.json --label smoothB \
    --compare docs/measurements/<...>_smoothA-stand_can.json --compare-label smoothA
```

For the follow-up that would settle §1's open question: same stand capture, but apply **8–10
deliberate pushes** spaced ~30 s apart, and let each one run free rather than damping it by
hand. The metric to read is ring duration per event.
