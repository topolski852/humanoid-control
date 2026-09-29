# Stuck events and swing-foot clearance, 2026-09-29 (brief items B and C)

Offline, from the six measC-full / smoothA-full walk tick logs recorded today. No robot needed.
Tool: `scripts/measure/gait_contact.py`. Data: `contact-20260929_gait_contact.json`.
Scope: only ticks with the trigger held and the stick forward (vx 0.6), 16 bouts, 64 s.

**Verdict:** "stuck, then breaks free" is the **stance knee buckling at its torque cap** with both
feet on the ground. It is **not** swing-foot scuffing: the toe does not catch and snag.

## C. Swing-foot clearance (the brief's exact proxy)

- Foot heights come from FK of the 12 logged joint angles (`*_ankle_roll` origins), projected on
  world-up (−projected_gravity).
- `d = z_L − z_R`. Steps are split at sign changes of d, and clearance is the per-step max|d|.
- Half-steps shorter than 0.15 s are dropped, as are the two bout-edge segments.
- Frames were verified: the logged observation equals sign × joint_pos − default exactly. At
  stand, the feet are level to within 1.5 mm.

| | steps | median | **p10** | in-stall median / p10 | not-stall median / p10 |
|---|---|---|---|---|---|
| **measC-full** | 80 | **32.2 mm** | **13.7 mm** | 29.2 / 11.4 | 32.2 / 20.3 |
| smoothA-full | 31 | 38.7 mm | 16.1 mm | 22.1 / 4.4 | 39.8 / 25.2 |

**Against sim** (training PC, measC-full at vx 0.6: median 53 mm, p10 44 mm):

- Hardware clearance is **~60% of sim at the median** (32 vs 53 mm) and **~⅓ at p10** (14 vs 44 mm).
- The feet lift clearly less than in sim, consistent with the smaller knee swing (0.63 vs 0.86 rad).
- It is **not near the 0–1 cm** that would confirm scuffing. Even the lowest decile clears 14 mm,
  and 20 mm outside stalls.

- **If sim clearance is far above 32 mm**, clearance is a real gap: sim's knee swing is 0.86 rad
  against 0.63 on hardware.
- **The in-stall numbers do not mean scuffing.** During stalls both feet are down (see B), so
  |d| is small because neither foot is in swing, not because a swinging foot is dragging.

## B. Stalls

Definition, tuned from the brief's:
- A stall is **the same hip_pitch or knee_pitch joint at ≥ 0.98 × cap for ≥ 0.3 s**, or both
  knees under 0.5 rad/s for ≥ 0.3 s.
- The first version OR'd the four joints tick by tick. That chained the normal
  left-then-right stance saturation into multi-second "stalls" (26–47% of all walking, up to
  2.3 s), so saturation runs are now found per joint and then combined.

| | walking | stalls | **fraction of walking stalled** | median / max duration |
|---|---|---|---|---|
| **measC-full** | 45.5 s, 12 bouts | 11 | **17%** | 0.64 / 2.24 s (both policies pooled) |
| smoothA-full | 18.6 s, 4 bouts | 8 | **36%** | |

What a stall looks like, consistently:

1. **A knee pinned at its cap, bent further than commanded.** The error (target − position) is
   **−0.31 to −0.53 rad**, a demand of 14–24 N·m against a 12 N·m cap. The knee is being told to
   straighten under load and can't.
2. **Both feet down.** The mean d during stalls is within ±9 mm: double support, not a swing.
3. **Torso pitched forward 3–7°**, with yaw rate 19–39 °/s.
4. **No knees-still events at all.** The robot never stops stepping entirely. It sags, and the
   leg saturates.
5. **How it ends:** usually the same knee comes off the cap first. measC: right knee 8,
   left hip_pitch 4, right hip_pitch 2, left knee 1. Foot height changes by less than 15 mm in the
   last 0.2 s, so no foot is snagging and breaking free.
6. **Stall fraction is unrelated to heading veer** (per-bout r = −0.22 measC, −0.28 smoothA).
7. **The left/right split is even for measC** (5 / 6). This is not the right-knee hardware
   fault. Those runs were excluded or ended before it, and the fault presents as a frozen reading,
   not saturation.

## What it means for training

- The stuck events are **sagittal stance-torque starvation**: sinking into a crouch the knees
  can't push out of at 12 N·m. That fits the corrected torque table in the brief (hip_pitch and
  knee_pitch 2–3× sim, everything else ~1×).
- Candidates the training PC can separate in sim: the crouch depth the policy chooses, a heavier
  stance load than modelled, or stance-phase damping or inertia on the sagittal joints.
- **Scuffing is not supported as the stall mechanism.** Clearance is still worth comparing to sim
  as a gait-quality number.

## Files

- **measC-full:** `recordings/run_1790703704747124133_2328.jsonl`,
  `run_1790704012610710849_2328.jsonl`, `run_1790705543932967453_23984.jsonl`,
  `run_1790707044544619019_6844.jsonl`
- **smoothA-full:** `run_1790703826926256452_2328.jsonl`, `run_1790704208451381629_2328.jsonl`
- All six are on the robot PC (not in git). Per-stall and per-step records are in the JSON.

## D. Floor friction (preliminary, one trial)

**Walk surface:** rug over concrete (the parents' basement), with a **PLA foot sole**.
**Tilt test:** the foot slid at **30°** (853 g on the foot), so **μ_static ≈ tan 30° = 0.58**.
The tilt test is independent of the load.

- 0.58 is **inside** the training range (μ 0.4–1.2) and about twice the μ 0.3 that reproduced the
  torso twist in the plant-ID eval. **Low static friction alone does not explain the twist.**
- **Remaining candidates:**
  - **Kinetic friction.** It can be well below static once the foot slides, which is the case
    the μ 0.3 eval represents. Pending: the lowest angle at which the foot keeps sliding.
  - **Rug compliance.** The foot sinks into and pivots on the pile. Sim's rigid ground has no
    equivalent; this is a contact model, not a μ.
- Two more repeats and a kinetic angle are pending. The next test location will have hard
  flooring; measure it there too, and keep each comparison on one surface.

## Test 1 on bare concrete (15:50): confounded by foot slip, so not a rug-vs-hard-floor answer

measC-full, knee cap 12, one session, 3 bouts, 13.6 s. Clean: no faults, network verified.

| | rug (4 sessions) | bare concrete |
|---|---|---|
| stall fraction | 17% | 40% |
| knee L/R corr | −0.35 … −0.45 | −0.02 |
| hip_pitch / knee sat | 15–23% / 12–25% | 30–36% / 22–33% |
| tilt p95 | 8–12° | 17° |
| torso heading rate | 5.7–10.8 °/s | 3.3 °/s |
| clearance median / p10 | 32 / 14 mm | 35 / 8 mm |

**The operator saw the PLA feet slip out from under the robot on the concrete.** The robot tried to
step, and the stance foot slid. The higher stall fraction and worse knee alternation are therefore
mostly **slip**: a stance foot sliding out folds the leg and puts the knee at its cap, which the
stall detector can't distinguish from buckling.

- Bare PLA on concrete is almost certainly below the μ 0.4 that training starts at. No μ number,
  because the concrete floor can't be tilt-tested.
- This is the regime measE's 0.25–1.2 friction range targets.
- **Next:** Test 1b and Test 2 (knee cap 18) are moved to **the rug**, with the same calibration
  and back to back, where feet don't slide out and the 17% baseline exists.
- **Suggestion for both sides:** rubber or TPU sole pads would remove PLA slip from every future
  test, on any floor.

## Test 1b on the rug (15:53): the robot walked much worse than in the morning; the decline is gradual

measC-full, knee cap 12, 3 bouts, **23.2 s**, same rug as the morning. Clean: no faults, network
verified.

| measC-full on rug | morning (4 sessions) | **15:53** |
|---|---|---|
| stall fraction | 17% | **54%** |
| knee L/R corr | −0.35 … −0.45 | **+0.03** |
| tilt p95 | 8–12° | **19.4°** (max 25°) |
| hip_pitch sat L / R | 15–23% | **30 / 43%** |
| torso heading rate | 5.7–10.8 °/s | 2.4 °/s |

- **The standing pose is unchanged.** Every joint is within ~2–3° of the morning stand, standing
  tilt is 0–1°, and holding errors are the same. So this is not calibration or a mechanical shift.
  The robot stands the same and walks worse.
- **CORRECTION: the robot runs from a bench power supply, not a battery.** It has for every test
  (operator, 2026-09-29), so the battery-sag hypothesis in the first version of this section is
  withdrawn. The 19.5–19.8 V at rest is the supply's output.
- **The decline is gradual across the afternoon, on the same supply.** measC rug stall fraction per
  session: 13:41–13:47 ~12–41% (pooled ~25%) → 14:12 49/0/0% → 14:37 50/29/36/0/51% →
  **15:53 54%**, while right-knee faults grew over the same hours.
- **Candidates:**
  - motor or driver **heat**. The ESCs expose no temperature, only an over-temp error flag, which
    never fired.
  - **mechanical** loosening after repeated E-stops, slips and falls.
  - the supply's **current limit**. If walking peaks exceed it, the supply drops into
    current-limit mode and the bus sags *only during steps*, which a reading at rest can't show.
- **Next:** walk with the bus voltage and motor current logged passively (ESC slow-poll replies,
  ~3 Hz per node). Then a cool-down of 20–30 min and the same walk again. A drop in stalls after
  cooling points at heat.
- **Test 2 (knee cap 18) stays on hold** until the decline is explained: a cap comparison on a
  drifting robot is not interpretable.
- The morning-vs-afternoon comparisons in this report may be partly battery state. The morning
  rug numbers (17%) are the best-supplied data.
