# walk_measA-full — 2026-09-25

**First bundle trained against measured hardware data** (the 2026-09-23 round).

* **Run:** `2026-09-23_18-04-28_measA-full`, resumed after a power loss as
  `2026-09-24_09-32-44_measA-full-resumed`, completed to 5999
* **Checkpoint:** `model_5999.pt`, chosen by eval replay rather than by default
  (`--plateau` was off, so there is no `model_best.pt`)
* **Source:** `humanoid-policy` commit `8022b17`, `policy.onnx.data` md5
  `935096d5fbdf67f79a169cdb065f2352`
* **Gains:** kp 45.0 / kd 1.5 — **unchanged** from smoothA/smoothB
* **Torque caps:** motor-typed — 12.0 Nm on the 8 M6C12 joints, 7.0 Nm on the 4 MAD5010 ankles

## What is new in it

Trained with the measured observation model rather than guessed values: transport staleness
tau ~ U(0, 10 ms), encoder quantisation 1.023e-4 rad, and observation noise at the measured
floor instead of the ~1200x-too-wide U(+-0.05) rad. Plus a torque-saturation reward and the
motor-typed caps.

## Required ESC state

This bundle will not behave as trained unless the robot carries the matching caps. Applied
2026-09-25 per `docs/HARDWARE_PLAN_2026-09-23.md` §1:

| joint (both sides) | motor | torque_limit | current_limit |
|---|---|---|---|
| hip_roll, hip_yaw, hip_pitch, knee_pitch | M6C12 | 12.0 Nm | 20.0 A |
| ankle_pitch, ankle_roll | MAD5010 | 7.0 Nm | 20.0 A |

Also restored `torque_constant` to the 8/4 motor split (0.08958 / 0.06588) — five joints were
carrying the ankle constant, which made the ESC's current→torque conversion wrong.

## What to watch on hardware

Sim says falls dropped 2x versus the alternative checkpoints and ankle saturation is ~4-5%.
One flagged regression: **`knee_corr_median` fell to -0.497** from -0.666, against -0.64
measured on hardware for smoothA. Knee correlation is one of the two metrics the Smooth A
report names for judging a round, and it moved the wrong way while everything else improved.

Knee torque saturation stays ~0.1% in sim against 52.1% / 33.3% measured on hardware —
unchanged and expected. That gap is in the **plant**, not the policy, and needs M7.
