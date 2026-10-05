# Rope-supported walk, 2026-10-05: the supporting hand was most of the sagittal "torque gap"

measC-full on the battery and the rug, knee cap 12, heading loop off, **supported by a slack
rope instead of the operator's hands**. Network verified; no faults (capture clean, freeze guard
and mode watcher quiet). Bus voltage median 24.04 V, min 23.29 V. 3 bouts, 11.0 s of walking.

| measC-full, battery | hand-supported (pooled 18.2 s) | **rope (11.0 s)** | sim (vx 0.6) |
|---|---|---|---|
| **commanded sat hip_pitch L/R** | 29 / 26% | **4.0 / 3.3%** | ~0.1% |
| commanded sat knee L/R | 20 / 16% | 20.7 / 5.1% | ~1% |
| **stall fraction** | 6% | **0%** | 0.01% |
| **knee L/R corr** | −0.39 | **−0.60** (bout 3 −0.75) | −0.87 |
| **max joint speed** | 24–29 rad/s | **12.3 rad/s** (none > 13) | |
| torso \|yaw rate\| p95 | 2.12 rad/s | 1.92 rad/s | 0.67 (μ 0.3: 2.18) |
| **net heading per bout** | 8.3 °/s, mixed sign | **+11.0, +16.5, +22.8 °/s (all left)** | 0 |
| knee swing L/R | 0.63 / 0.64 rad | 0.46 / 0.60 | 0.85 |
| clearance median / p10 | 41.5 / 22.7 mm | 22.5 / 18.2 | 53 / 44 |
| gait | 1.64 Hz | 1.06 Hz | 1.58 |
| delivered \|τ\| p95, knee L/R | 3.9 / 3.3 N·m | **5.0 / 0.7** | |

## Reading

1. **The operator's hand caused most of the sagittal demand gap.**
   - With a slack rope, hip_pitch commanded saturation falls from ~27% to ~4%, and stalls go to 0.
   - Knee alternation improves (−0.39 → −0.60, best bout −0.75), and no joint exceeds 13 rad/s.
   - The training PC's plant-ID predicted the hand confound (a body-frame rearward force moved
     cadence and knee alternation toward hardware). This measures it.
2. **Unmasked, the robot has a steady LEFT yaw bias:** +11 to +23 °/s in every bout, all the same
   sign. The hand had been holding the heading and mixing the veer's sign. This is the heading
   loop's target.
3. **A left/right asymmetry appears.** The left knee carries most of the load: delivered p95
   5.0 vs 0.7 N·m, commanded saturation 21% vs 5%. Together with the left turn, it suggests a
   lateral imbalance. Candidates:
   - the right ankle_pitch sitting 6° off the left in stance since the rebuild (stand report);
   - the asymmetric calibration;
   - the policy favouring the left leg.
4. **Still short of sim:** smaller swing (0.46/0.60 vs 0.85), lower clearance (22 vs 53 mm),
   slower cadence (1.06 vs 1.58 Hz).

## For training

- **Retire the "sagittal torque gap" as a plant correction.** The 09-29 version was the bench
  supply. The hand-supported battery version (16–29%) mostly disappears without the hand (3–5% on
  hips). The remaining knee saturation is one-sided (left 21%, right 5%).
- **New targets:**
  - the steady left yaw bias, while the heading loop is tested on the robot;
  - the left/right knee load asymmetry;
  - the short, low, slow stride relative to sim.
- **From now on, prefer rope-supported data** over hand-supported data for any sim comparison.

Data: `measC-rope-20261005_walk.json`, `power_log_*_measC-rope.json`,
`walk_*_measC-rope_{can,M2M3M6}.json`; tick log `recordings/run_1791240579593261730_4551.jsonl`.
