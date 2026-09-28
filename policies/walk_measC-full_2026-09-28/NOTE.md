# walk_measC-full — 2026-09-28

**Round 2 after the measA regression. First bundle to beat smoothA in sim.**

* **Run:** `2026-09-25_20-04-02_measC-full`, full profile, 1d 04:00:48, no crashes
* **Checkpoint:** `model_5999.pt`, chosen by eval over 5800/5600
* **Source:** `humanoid-policy` `88c31a7`, `policy.onnx.data` md5 `0b274791cd16d1daf063e914273eea59`
* **Gains:** kp 45.0 / kd 1.5 — unchanged
* **Torque caps:** 12.0 / 7.0 — unchanged. **No ESC reflash needed**; the contract matches the
  live config on all 12 joints (verified 2026-09-28).

## What changed from measA

1. **Restored the observation uncertainty margin** — sensor floor *plus* an explicit plant
   margin, on top of the measured staleness and quantisation. This is the direct response to the
   measA regression, whose leading hypothesis was that collapsing noise to the measured sensor
   floor removed cover for unmeasured plant uncertainty.
2. **Joint-velocity hinge at 8 rad/s**, weight −0.5.
3. **Standing command fraction 0.02 → 0.30.**

## Sim comparison (5999 vs the bundle it must beat)

| | measC 5999 | smoothA |
|---|---|---|
| STAND tilt p95 °/s | **16.19** | 23.58 |
| STAND jvel p99 | **0.86** | 1.53 |
| STAND falls/min | **0.00** | 0.23 |
| STAND >10 rad/s events | **6** | 35 |
| WALK tilt p95 °/s | **50.15** | 59.86 |
| WALK knee corr | **−0.708** | −0.655 |
| WALK falls/min | **0.00** | 0.37 |

Best knee correlation of any bundle to date, and the first to beat smoothA on it. Zero falls in
both regimes. Excursion *count* — the statistic that separated measA (150/304 events, faulted on
hardware) from smoothA (35/25, runs fine) — is better than any predecessor.

## A correction worth carrying

The 2.0 rad/s velocity threshold in `REPORT_2026-09-25_measA.md` §3b **was wrong**. It cited
smoothA's *standing* max of 1.08 rad/s as the basis for a limit that binds during *walking*.
smoothA's own walk capture gives a median per-joint max of 6.90 rad/s with peaks of 7.1–8.8, so
a hinge at 2.0 priced its normal gait — and measB-fast came out a shuffle (knee swing 0.391 rad
against smoothA's 0.849, knee corr −0.158). 8.0 rad/s sits above smoothA's walking range and
5 rad/s below the 13.02 fault.

Also from that eval: **smoothA peaks at 14.98 rad/s in sim**, past the fault threshold. It is not
inherently safe either — it has just not been unlucky yet. That is the argument for keeping the
term at all rather than dropping it.

## Untested premise

The whole point of this round — that restoring the uncertainty margin stops measA's 20 °/s
lurching — has only been shown in sim. The hardware metrics to check, from
`REPORT_2026-09-25_measA.md` §3c:

| metric | smoothA (bar) | measA (failed) | target |
|---|---|---|---|
| tilt rate p95 | 0.4 °/s | 20.6 °/s | < 1 °/s |
| excursions >8° | 0.2/min | 11.7/min | < 0.5/min |
| joint vel p99 | 0.73 rad/s | 5.09 rad/s | < 1.5 rad/s |

Note the sim tilt-rate numbers (16–24 °/s) are **not** comparable to the hardware ones
(0.1–24 °/s) — different plant, different disturbances. Compare each within its own domain.

`rel_standing_envs` 0.02 → 0.30 is not separable from the hinge change in this bundle; if the
gait is good on hardware, both were fine.
