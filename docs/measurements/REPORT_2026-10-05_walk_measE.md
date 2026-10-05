# measE-full-ft walk on the battery, 2026-10-05: bigger steps, but more twist (against sim)

measE (`walk_measE-full-ft_2026-09-29`: measC-full fine-tuned on floor friction 0.25–1.2 plus M7
ankle friction), on the battery and on the rug, same protocol as the measC battery walks.
- **Network:** verified by replay. A first attempt ran the dropdown's default `walk` after a
  reboot; the watcher caught it within seconds, and it was stopped and discarded.
- **Faults:** none. The freeze guard and mode watcher stayed quiet. Bus voltage min 23.38 V,
  median 24.50 V.
- **Walking:** 3 bouts, 8.9 s.

| battery, rug | measC-full (pooled 18.2 s) | **measE-full-ft (8.9 s)** | sim (vx 0.6) measE / measC |
|---|---|---|---|
| knee L/R corr | −0.39 | −0.39 | −0.884 / −0.876 |
| knee swing L/R | 0.63 / 0.64 rad | **0.77 / 0.77** | 0.931 / 0.852 |
| clearance median / p10 | 41.5 / 22.7 mm | **52.4 / 36.5** | p10 47 / 43 |
| gait | 1.64 Hz | **1.27 Hz** | 1.58 / 1.58 |
| stall fraction | 6% | 7% | |
| **torso \|yaw rate\| p95** | 2.12 rad/s | **2.60 rad/s** | μ 0.3: **1.75 / 2.18** |
| net heading rate per bout | 8.3 °/s | 14.1 °/s | |
| commanded sat knee L/R | 20 / 16% | 30 / 23% | |
| commanded sat hip_pitch L/R | 29 / 26% | 21 / 18% | |
| delivered \|τ\| max, knee L/R | 12.0–16.0 / 12.4–12.6 | 11.9 / **20.3** (impact) | |

## Reading

1. **measE moves the way sim says on stride:** bigger knee swing and higher foot clearance, closer
   to sim. But **cadence drops** (1.27 vs 1.64 Hz; sim says unchanged), and **knee saturation
   rises**.
2. **The torso twist went the wrong way.**
   - Sim predicted the friction fine-tune would cut yaw p95 from 2.18 to 1.75 rad/s on a
     μ 0.3 floor.
   - On the robot it rose from 2.12 to 2.60, and the net veer grew too.
   - **So low kinetic friction alone does not explain the hardware twist,** or the rug's contact
     (compliance, pivoting in the pile) is not captured by a μ change. The kinetic-μ measurement
     (item D) is still pending.
3. **Knee alternation is unchanged** (−0.39 for both), still far from sim's −0.88.
4. **Overall, measE is not a clear upgrade over measC on hardware.** Only 8.9 s of walking: the
   twist result should be confirmed with another session before acting on it.

## Hardware flags

- **right_knee delivered 20.3 N·m** at its peak, against a 12 N·m torque_limit. That is a
  transient, almost certainly impact back-drive at foot strike, not the controller exceeding its
  cap.
- **left_ankle_pitch reached 29.2 rad/s** (10 ticks over 13 rad/s; the right knee had 6, the
  right ankle 3). This is the highest yet. No encoder faults this session.

Data: `measE-walk-20261005_walk.json`, `power_log_*_measE-walk2.json`,
`walk_*_measE-walk2_{can,M2M3M6}.json`; tick log `recordings/run_1791239581891331095_6709.jsonl`.

## Operator verdict (2026-10-05)

"measE is walking weird and moving more drastic than measC." **measE testing is stopped. measC-full
remains the best bundle** and the base for the next round. The numbers agree: measE has more
twist, more veer, more knee saturation and slower cadence for its bigger steps.
