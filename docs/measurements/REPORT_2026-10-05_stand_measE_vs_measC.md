# Stand A/B: measE-full-ft vs measC-full, 2026-10-05 (bench supply, same calibration)

measE (`walk_measE-full-ft_2026-09-29`: measC-full fine-tuned on floor friction 0.25–1.2 plus
the M7 ankle friction) against measC-full, back to back on one calibration, after the right-leg
rebuild. Both networks verified exactly by replay.
- **Supply:** bench 19.3–19.6 V. No collapse; standing draws little current.
- **Faults:** none; the freeze guard and mode watcher stayed quiet.
- **Protocol:** 0–120 s settle, 120–300 s untouched, then small pushes.

| | **measE-full-ft** | **measC-full** |
|---|---|---|
| tilt median, untouched | **8.74°** | **4.78°** |
| IMU pitch (untouched) | +8.72° | +4.67° |
| tilt rate p95, untouched | 0.06 °/s | 0.04 °/s |
| tilt rate p95, settle (stand-up) | 0.21 °/s | 0.06 °/s |
| excursions > 8° | 1 per phase (the lean sits at ~8.7°, so these are threshold crossings, not excursions) | 0 |
| joint vel p99 | 0.73 | 0.72 |
| push response | 1 push (peak 47 ct): ring 0.2 s, settle 1.8 s | 2 pushes (peak 94–119 ct): ring 0.8–1.0 s, settle 1.5 s |

## Reading

1. **Both stand equally still untouched.** 0.04–0.06 °/s is at the IMU floor.
2. **measE leans ~4° further forward than measC on the same robot and calibration.** That is
   measE's own posture. Its stand-up transient is also less steady (0.21 vs 0.06 °/s).
3. **The rebuild and recalibration added ~4° of lean to every policy.**
   - measC read +0.67° IMU pitch on 09-29 and +4.67° today, with the leg joints in nearly the
     same pose (within ±3.4°).
   - The operator confirms the visible lean matches the IMU, so the IMU did not move.
   - Today measC stands with **ankles 6° apart** (left −26.6°, right −32.7°), against 1.5° on
     09-29. **Suspect the right ankle_pitch calibration after the rebuild.**
4. **Push recovery is not comparable.** Pushes were small and unmatched: measE's one push was
   about half the size of measC's. A repeatable push is still needed.

## For training

- measE's extra ~4° forward lean is a posture change from the fine-tune. Worth checking in sim
  (stand pitch for measE vs measC).
- Compare tilt only within one calibration. Today's absolute values include ~4° from the
  rebuild.

Data: `measE-stand-20261005_stand.json`, `measC-stand-20261005_stand.json`,
`hold_*_measE-stand_*`, `hold_*_measC-stand2*_*`, `power_log_*_measE-stand.json`,
`power_log_*_measC-stand2.json`. Tick logs: `recordings/run_1791237850347138600_6920.jsonl`
(measE), `run_1791238424942108149_6920.jsonl` (measC).
