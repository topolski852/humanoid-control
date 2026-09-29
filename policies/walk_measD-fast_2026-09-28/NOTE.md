# walk_measD-fast — 2026-09-28

**measC-full fine-tuned with one change: the M7 ankle friction. For an A/B against measC-full.**

* **Run:** `2026-09-28_21-52-39_measD-fast`, fast profile, resumed from measC-full `model_5999`
  for 5000 iterations
* **Checkpoint:** `model_10998.pt`
* **Source:** `humanoid-policy` `0727494`, `policy.onnx.data` md5 `41fb55a99cd59f0d90e15d2f25855244`
* **Gains / caps:** kp 45.0 / kd 1.5, 12.0 / 7.0 — **contract identical to measC-full**. No ESC
  reflash; switching between the two in the dropdown swaps the network and nothing else.

## What changed from measC-full

**Only ankle breakaway friction, 0.249 → 0.40 N·m** (humanoid-policy `908d3e0`). M7 measured
~0.40 on both ankles, holding against gravity; the bench value with ±30% randomisation topped out
at 0.324, so training never met the real number. Coulomb (kinetic) friction is unchanged at the
bench 0.222 — M7 is static and says nothing about friction while moving.

This is ~0.15 N·m on a 7 N·m cap. M7 showed the 3–5× sim/hardware torque gap is **dynamic**, so
this is **not expected to fix the shuffle or the circle**.

## Sim says measD is slightly WORSE — this is a prediction to test

Both evaluated on the same plant (**including** the corrected ankle friction), 128 envs × 600
steps:

| | measC-full | measD |
|---|---|---|
| stand tilt p95 °/s | **16.2** | 18.2 |
| stand falls/min | **0.000** | 0.039 |
| walk tilt p95 °/s | **50.3** | 60.7 |
| walk knee corr | **−0.706** | −0.554 |
| walk >10 rad/s events | **15** | 25 |
| ankle sat L / R | **13.7 / 10.3 %** | 15.0 / 11.5 % |
| knee swing rad | 0.749 | **0.790** |

measC-full handles the corrected friction without losing anything, so the friction change isn't
what separates them. The likely cause is the fine-tune itself: 5000 iterations at the fast
profile's batch (~16× fewer samples per update than measC-full trained on) added gradient noise
to an already-converged policy.

**So the A/B tests the sim ranking, not the friction.** Sim predicts measC-full ≥ measD. If
hardware agrees, the eval is tracking small differences as well as large ones. If measD comes out
*better* on hardware, the friction correction mattered more than sim can show — which would be
worth knowing.

## A/B protocol

Keep everything except the network identical, so the comparison is clean:

1. **One calibration** for both runs — the offset spread is ±0.029 rad between calibrations,
   which is larger than the difference being tested. Don't recalibrate between them.
2. **Same stand protocol for each**, as in the measC capture: settle 120 s, **untouched 180 s**,
   then a set of small matched pushes. `capture_run.sh <label>-stand 480 hold`.
3. **Skip the hard push.** It faulted `right_knee_pitch` last time (third fault on that node) and
   would end the session. If the right knee hasn't been inspected yet, small pushes only.
4. **Order:** measD first, then measC-full, back to back.

**Compare on the untouched window**, since that's where measC's result was cleanest:

| metric | measC-full, 2026-09-28 |
|---|---|
| tilt rate p95 | 0.04 °/s |
| tilt median | 2.71° |
| excursions >8° | 0 / 3 min |
| joint vel p99 | 0.67 rad/s |

Plus ring duration per push. A short supported walk on each is optional; measC's walk was a
shuffle in a circle, and the heading loop (HARDWARE_PLAN_2026-09-28 §2) is what fixes the circle,
not either of these bundles.

A caution on reading the result: measC's untouched tilt rate is already 0.04 °/s, near the floor
of what the IMU resolves. Two good standers may be indistinguishable there, in which case push
recovery (ring duration) is the more sensitive discriminator.
