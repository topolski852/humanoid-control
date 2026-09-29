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
