# A/B stand: measD-fast vs measC-full, 2026-09-29

**Verdict: on hardware, the two bundles can't be told apart as standers.** On balance while
untouched and on push recovery they are equal within what these measurements resolve. Where they
do differ, sim's ranking (measC-full ≥ measD) holds on the one balance signal that separates them:
**measD wobbles more during the stand-up ramp**.

The finding that is new is not about balance: **measD chose a different stance.** It stands with
its knees about 11° straighter and its ankles about 7° less bent. That leans it 2.3° further
forward and costs about 15% less holding torque.

## Protocol

This followed `policies/walk_measD-fast_2026-09-28/NOTE.md`.
- **Same calibration, back to back:** measD engaged at 13:12:13, measC-full at 13:21:07.
- **Phases, each 480 s:** 0–120 s settle, 120–300 s untouched, 300–480 s small pushes.
- **No hard push.** `right_knee_pitch` is still uninspected.
- **The network is verified from the data, not the UI.** The logged observations were replayed
  through every staged ONNX. Each run matches its bundle exactly (action error 0) and misses
  every other one by 0.12–0.28 on average. The service does not log which checkpoint it loaded,
  and the dropdown's default "walk" entry is a third, different network, so this check is needed.
- **Both captures were clean.** No faults, 0% stale-hold on all 12 joints, CAN age spread
  0.49 ms and 0.72 ms.

Tools: `scripts/measure/stand_metrics.py` (tilt, from the runner's tick log) and
`settle_analysis.py` (push ring, from the CAN capture).

## Results

| | **measD** | **measC-full** |
|---|---|---|
| **untouched tilt rate p95** | 0.06 °/s | 0.04 °/s |
| untouched tilt median | 3.05° | 1.02° |
| untouched excursions >8° | 0 | 0 |
| untouched joint vel p99 | 0.73 rad/s | 0.73 rad/s |
| **stand-up excursions >8°** (first 3.2 s) | **5**, peaks 8.1–10.6° | 1, at t = 0 |
| push ring duration (7 pushes each) | 0.2–0.5 s | 0.2–0.8 s |
| push settle time | 1.0–1.8 s | 1.0–1.2 s |
| push peak motion | 54–78 ct | 58–131 ct |
| torque saturation during pushes | 0% all joints | 0% all joints |

- **Untouched balance: a tie.** 0.06 against 0.04 °/s is at the IMU floor: both hold still to
  within what it resolves, as `NOTE.md` warned. Both show no excursions and the same joint
  velocity.
- **Push recovery: a tie, possibly slightly in measC's favour.** Pushes weren't matched in size.
  measC-full took pushes up to 1.7× larger in peak motion, yet it settled at least as fast
  (1.0–1.2 s against 1.0–1.8 s). Neither run showed ongoing oscillation; both return to baseline
  at ~0.22 Hz.
- **Stand-up: measD is worse.** It crossed 8° five times in the first 3.2 s of the 5 s ramp to
  default pose; measC-full crossed once. This is the one balance difference outside the noise,
  and it agrees with sim (stand falls/min 0.039 against 0.000).

## The stance difference

Mean joint position over the untouched window (degrees, device frame):

| joint | measD | measC-full | D − C |
|---|---|---|---|
| left_knee_pitch | 36.95 | 46.91 | **−9.96** |
| right_knee_pitch | 35.89 | 48.15 | **−12.26** |
| left_ankle_pitch | −20.29 | −27.56 | **+7.27** |
| right_ankle_pitch | −21.39 | −29.08 | **+7.70** |
| left_hip_pitch | −14.70 | −16.13 | +1.43 |
| right_hip_pitch | −15.75 | −17.53 | +1.78 |
| others | | | within ±1.7 |

Base tilt from the IMU gravity vector: measD **+2.93° pitch**, measC-full **+0.67°**. Roll is
the same for both (+0.85° / +0.77°). So the 2° tilt gap is forward pitch, and it comes from
the straighter knees.

Holding torque over the untouched window, reconstructed as kp·(target − pos) − kd·vel. That is
real commanded torque; M7 showed ESC torque = kp·err.

| | measD | measC-full |
|---|---|---|
| sum of median \|τ\| over 12 joints | **8.48 N·m** | 9.95 N·m |
| left_ankle_pitch | **0.37** | 1.74 |
| right_ankle_pitch | 1.39 | 1.23 |
| knees L / R | 1.80 / 0.99 | 2.21 / 1.83 |
| hip_roll L / R | 0.48 / 0.65 | 0.05 / 0.14 |

The straighter knee makes standing cheaper. measD shifts load from the knees and left ankle onto
the hip rolls. Whether the ankle friction change (0.249 → 0.40 N·m) caused this, or the
fine-tune's noise did, can't be separated from one A/B.

## For training

- **Sim's ranking holds** for a small difference, on the one signal that resolves it (stand-up
  transient). That supports continuing to use the sim eval to rank bundles.
- **The friction change did not measurably help or hurt standing.** It did coincide with a
  straighter, cheaper stance.
- **The untouched stand no longer separates good bundles.** Both are at the IMU floor. For future
  A/Bs the discriminators are the **stand-up transient** and **matched push recovery**. The
  pushes here weren't matched in size, which limits that comparison. A repeatable push (a
  pendulum or a fixed drop) would make it a real number.
- **Calibration moves tilt more than policy does.** The same measC-full bundle read 2.71° on
  2026-09-28 and 1.02° today. Compare tilt medians only within one calibration.
- **Recommended bundle for the next hardware tests: measC-full.** That's for the step response
  and the heading-loop walk. measD has no balance advantage and a worse stand-up.

## Files

- `hold_20260929T131213_measD-stand_{can,M2M3M6,settle}.json`,
  `measD-stand-20260929_stand.json`
- `hold_20260929T132108_measC-stand_{can,M2M3M6,settle}.json`,
  `measC-stand-20260929_stand.json`
- Tick logs (not in git, on the robot PC): `recordings/run_1790701933166047999_2328.jsonl`
  (measD), `recordings/run_1790702467483884217_2328.jsonl` (measC-full)
