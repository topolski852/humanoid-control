# Walk: measC-full vs smoothA-full, 2026-09-29

**measC-full's first real walk.** The 09-28 walk attributed to it actually ran the August
`policies/walk` bundle (see the correction in REPORT_2026-09-28_measC_walk.md).

**Verdict: measC-full is the better walker of the two.** It alternates its knees more clearly,
steps closer to sim's cadence, and saturates about 30% less. smoothA takes slightly bigger steps
but works harder for them. **Neither walks in a circle**: both veer 6–11°/s, in a direction that
changes from bout to bout, with hip_yaw steady to within a few degrees.

## Protocol

- Supported walks, stick fully forward (vx 0.6), no yaw command. The room forces several short
  bouts per policy.
- **Only ticks with the trigger held and the stick forward are scored.** The tick log is split
  into bouts at every trigger release and every stick release, and bouts under 1 s are dropped.
  Metrics are per bout, pooled by duration.
- **The network is verified from the data for every run**, by replaying the logged observations
  through every staged ONNX: an exact match each time.
- No faults: no EMCY frames in any of the four CAN captures.
- **Order:** measC run 1, smoothA run 1, measC run 2 (3 bouts), smoothA run 2 (3 bouts). All on
  one calibration.
- **Tool:** `scripts/measure/walk_metrics.py`.

## Results (both runs pooled, 4 bouts each)

| | **measC-full** | **smoothA-full** | sim reference |
|---|---|---|---|
| walking time scored | 11.5 s | 18.6 s | |
| **knee L/R correlation** | **−0.41** | −0.04 | −0.84 … −0.92 |
| **gait frequency** | **1.41 Hz** | 1.11 Hz | ~1.5 Hz |
| knee swing L / R | 0.61 / 0.65 rad | 0.72 / 0.60 rad | 0.75 (measC), 0.85 (smoothA) |
| \|heading rate\| per bout | 10.8 °/s | 5.7 °/s | 0 (commanded) |
| heading change per bout | −66, −21, +20, −17° | +14, −37, −48, +6° | |
| hip_yaw drift per bout (largest side) | 2.0° | 4.2° | |
| tilt p95 | 12.1° | 12.1° | |
| max joint velocity | 7.8 rad/s | 8.1 rad/s | |

Torque saturation, reconstructed kp·err − kd·vel against each bundle's own contract caps. M7
confirmed this is real commanded torque.

| joint L / R | measC-full | smoothA-full |
|---|---|---|
| hip_pitch | 18.5 / 21.6% | 30.0 / 29.5% |
| knee_pitch | 24.7 / 10.8% | 33.4 / 33.6% |
| ankle_pitch | 6.3 / 8.4% | 14.9 / 7.3% |
| hip_roll | 0.7 / 1.7% | 6.9 / 6.5% |
| right_hip_yaw | 3.1% | 13.8% |

## What this says

1. **measC-full steps, it doesn't shuffle.** Knee swing is 0.61–0.65 rad, about 85% of its
   0.742 rad sim stride. The "shuffle" (0.17–0.24 rad) belonged to the August bundle. Its best
   bout reached a knee correlation of −0.73 at 1.49 Hz, close to sim.
2. **measC-full alternates better and matches sim cadence.** Knee correlation is −0.41 against
   smoothA's −0.04, and gait is 1.41 against 1.11 Hz (sim ~1.5).
   - smoothA's bigger steps come with knees that barely alternate, and with 30–34% saturation on
     hips and knees.
   - That fits the operator's impression that smoothA "was taking steps": it throws its legs
     harder. But measC's gait is closer to what it was trained to do.
3. **Both are still well short of sim.** Knee correlation is −0.41 at best against −0.84 to −0.92,
   and saturation is 11–25% on hips and knees against almost none in sim. The torque gap is real
   on the same network (M7 showed the reconstruction is honest). The step-response measurement is
   what can locate it.
4. **Heading: a veer, not a circle.**
   - Both policies turn the body 6–11°/s while walking, but **the direction flips between bouts**
     (measC: −66, −21, +20, −17°).
   - hip_yaw stays within ~2–4° per bout, so this is not the steady hip_yaw walk-off seen in the
     August bundle.
   - Some of it may be the operator's supporting hand; the walks are supported. Even so, the
     robot has no heading feedback, and nothing corrects a veer once it starts.
   - **This is the number the heading loop has to bring down: ~6–11°/s per bout.**
5. **Correction to an earlier commit message (`598be18`).** It reported 19–25 s of "in-phase knees
   (sag)" for both policies. Those windows came from the CAN capture after the trigger was
   released, while the robot was being handled. They are not policy behaviour and should be
   ignored.

## For training

- **Prefer measC-full as the base** for the next walking round. It has the best gait structure on
  hardware and the best stand (tilt rate 0.04 °/s).
- **The gap to close is amplitude under the torque caps, not cadence.** Cadence already matches
  (1.41 Hz). Knee alternation and saturation don't.
- **Heading:** a veer of up to ~11°/s that changes direction. The robot-side heading loop is being
  built next (robot-PC brief item 3). A training-side heading or yaw-rate tracking term would
  address the same problem from the policy side.

## Files

- **measC-full:** `walk_20260929T134148_measC-walk_{can,M2M3M6}.json` and
  `walk_20260929T134654_measC-walk2_{can,M2M3M6}.json`; scores in `measC-walk-20260929_walk.json`
  and `measC-walk2-20260929_walk.json`.
- **smoothA-full:** `walk_20260929T134351_smoothA-walk_{can,M2M3M6}.json` and
  `walk_20260929T135010_smoothA-walk2_{can,M2M3M6}.json`; scores in `smoothA-walk-20260929_walk.json`
  and `smoothA-walk2-20260929_walk.json`.
- **Tick logs** (robot PC, not in git): `recordings/run_1790703704747124133_2328.jsonl`,
  `run_1790704012610710849_2328.jsonl` (measC-full); `run_1790703826926256452_2328.jsonl`,
  `run_1790704208451381629_2328.jsonl` (smoothA-full).
