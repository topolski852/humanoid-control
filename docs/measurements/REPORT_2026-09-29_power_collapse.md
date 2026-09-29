# The bench supply collapses under walking load, 2026-09-29. Read before using any walk data.

**Every walk measured on 2026-09-29 ran on a bench supply that cannot deliver walking peaks.**
Logged directly on the 16:02 walk (measC-full on the rug): **when the robot takes a hard step,
the whole robot's supply collapses from ~19.5 V to ~5.8 V** for up to ~2.4 s.

## Measurement

- **Supply:** 20.15 V set, **10 A limit** (operator). The robot draws ~1.5 A idle and reads
  ~19.8 V on its own meters.
- **Method:** `scripts/measure/power_log.py`, which reads each ESC's bus voltage and I_q passively
  from the daemon's slow-poll SDO replies (~3.3 Hz per node, candump only).
- **Result:** all 12 ESCs, on **both** CAN buses, drop to 5.75–6.10 V at the same instant.

| t after engage | total commanded \|τ\| (Σ over 12 joints, kp·err − kd·vel) | bus V |
|---|---|---|
| standing | 8–15 N·m | 19.5 |
| ordinary stepping | 25–60 N·m | 18.6–19.5 |
| **first hard step, tilt 10.5°** | **96 N·m** | **5.75, held ~2.4 s** |
| **later hard step, tilt 11°** | **97 N·m** | **5.86** |

This is the supply dropping into constant-current mode: the whole robot's rail collapses. At ~6 V
the ESCs cannot produce commanded torque, the legs fold, and the knees end up pinned at their caps.
**That is the stall / "stance-knee buckling" signature in REPORT_2026-09-29_contact_B_C.md.**

## Consequences

1. **All 2026-09-29 walk metrics are confounded by supply collapse:** stall fraction, knee
   buckling, knee alternation, and the hip/knee saturation in the "sagittal torque gap". That
   includes the morning walks, which had no voltage log but ran on the same supply.
   - **Do not retrain on the sagittal gap until it is re-measured on adequate power.**
   - Stand metrics (tilt rate, posture) draw little current and are much less affected.
   - Hanging tests (M7, step response) are unaffected.
2. **The afternoon decline** (rug stall fraction ~25% → 54% on the same supply) may be the supply
   folding back earlier as it warms. Unconfirmed.
3. **Right-knee node-8 faults may be brownout-induced, not a failing part.**
   - ESC `undervoltage_threshold` is **0 (disabled)**, so the ESCs run straight through a
     collapse to 6 V, and an encoder browning out would fault this way.
   - Not proven: the 16:02 knee fault (0x2000) came ~14 s after the last dip the ~3 Hz log
     caught, so a shorter dip may have been missed.
   - Today's walk sessions logged 4 node-8 faults.
4. **The torso twist** (hardware 2.4–4× sim) may also be affected, since yaw control is lost
   during a collapse. The rug-vs-concrete twist difference still stands.

## Needed next

- **Re-run the walks on a supply that can deliver the peaks:** a charged battery with an adequate
  C rating, or a bench supply rated well above 10 A (≥ 30 A suggested), with `power_log.py`
  running.
- **Consider a real ESC undervoltage threshold** (e.g. ~16 V), so a collapse faults cleanly
  instead of browning out. This is a config change for the operator to decide.
- **Hold the right-knee teardown** until a walk on adequate power shows whether the faults persist.

Data: `power_log_*_rugA.json`, `measC-rug-A-20260929_walk.json`, capture
`walk_*_measC-rug-A_can.json`, tick log `recordings/run_1790712130453961590_17179.jsonl`.
