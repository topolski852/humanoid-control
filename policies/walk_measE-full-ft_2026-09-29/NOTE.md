# walk_measE-full-ft_2026-09-29

**measC-full fine-tuned on the wider floor friction. Candidate for the next hardware test.**

* **Run:** `2026-09-29_15-06-36_measE-full-ft`, full-profile batch, resumed from measC-full `model_5999.pt` for 1500 iterations
* **Checkpoint:** `model_7498.pt`
* **Source:** `humanoid-policy` `1931a15`, `policy.onnx.data` md5 `1491c91f44303775e3efbcf5ab55811f`
* **Contract identical to measC-full** (12.0 / 7.0 caps, kp 45 / kd 1.5, stand pose). No ESC
  reflash; switching in the dropdown swaps only the network.

## What changed from measC-full

1. **Floor friction randomisation 0.4–1.2 → 0.25–1.2** (`86003fc`). Plant identification found
   foot μ 0.3 is the only sim change that reproduces the hardware torso twist (yaw p95 2.18 rad/s
   vs 1.4–2.5 measured, 0.60 on the old plant). See `docs/ROBOT_PC_BRIEF_2026-09-29.md`.
2. **Ankle breakaway friction 0.40** from M7 (`908d3e0`).

## Sim comparison vs measC-full (vx 0.6 = the hardware walking command)

```
       metric              measC-full     measE
stand  tilt p95 deg/s          16.200    15.741
stand  falls/min                0.000     0.000
stand  episode s               24.000    24.000
walk   falls/min                0.000     0.059
walk   knee corr               -0.876    -0.884
walk   knee swing rad           0.852     0.931
walk   gait Hz                  1.583     1.583
walk   fwd m/s                  0.580     0.582
walk   yaw p95 rad/s            0.674     0.655
walk   clearance p10 m          0.043     0.047
mu03   falls/min                0.176     0.176
mu03   yaw p95 rad/s            2.179     1.745
mu03   knee corr               -0.854    -0.873
mu03   fwd m/s                  0.531     0.549
walk   hip_pitch p95              6.1       6.3
walk   knee_pitch p95             8.1       7.9
walk   ankle_pitch p95            8.2       8.7
stand  >10 rad/s evts               6         8
walk   >10 rad/s evts              22        48

  gate PASS  stand fall/min <= 0.25
  gate PASS  stand episode >= 22 s
  gate PASS  stand >10 rad/s frac <= 1e-4
GATE=PASS
```

The `mu03` rows are the target condition. Whether they transfer depends on item D, the direct
floor friction measurement. Not expected to fix the sagittal torque gap, which the plant-ID
suggests is confounded by the supporting hand.

## Test

Only after the right knee is repaired. Stand first (`capture_run.sh <label>-stand 480 hold`,
compare stand-up transient and matched pushes against measC-full on one calibration), then a
short walk. Score walks with `walk_metrics.py`, including torso yaw rate against the 1.4–2.5
rad/s baseline. **An unsupported or lightly spotted bout is worth more than a long held one.**
