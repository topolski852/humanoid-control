# walk_measF-full-ft_2026-10-06

**measC-full fine-tuned with a soft-landing reward. Candidate for the next hardware test.**

* **Run:** `2026-10-05_19-28-16_measF-full-ft`, full-profile batch, resumed from measC-full `model_5999.pt` for 1500 iterations
* **Checkpoint:** `model_7498.pt`
* **Source:** `humanoid-policy` `305d4ad`, `policy.onnx.data` md5 `42154643cd18ae679dca0109b8036a59`
* **Contract identical to measC-full** (12.0 / 7.0 caps, kp 45 / kd 1.5, stand pose). No ESC
  reflash; switching in the dropdown swaps only the network.

## What changed from measC-full

**One change: the soft-landing reward** (`4242373`). It penalizes foot speed at touchdown. The
target is the ankle_pitch impact spikes (24–29 rad/s at foot strike, 13.2 N·m back-drive on a
7 N·m ankle). In sim, measC-full plants its foot still moving ~1.9 m/s horizontally, and that
speed is the best predictor of the ankle peak that follows.

The plant is measC-full's exactly. measE's floor-friction widening and ankle breakaway were
reverted (`55b867c`).

## Sim comparison vs measC-full (vx 0.6; "carpet" = rigid floor at the tilt-tested μ 0.58)

```
       metric              measC-full     measF
stand  tilt p95 deg/s          16.190    14.852
stand  falls/min                0.000     0.000
stand  episode s               24.000    24.000
walk   falls/min                0.000     0.059
walk   fwd m/s                  0.580     0.578
walk   knee corr               -0.878    -0.882
walk   knee swing rad           0.859     0.907
walk   gait Hz                  1.583     1.500
walk   yaw p95 rad/s            0.598     0.652
walk   clearance med m          0.053     0.054
walk   clearance p10 m          0.043     0.043
walk   land vxy p50             1.904     1.649
walk   land vxy p95             2.543     2.362
walk   land vz p95              0.550     0.535
walk   impact F p95 N         416.757   421.597
walk   ankle peak p95           7.381     6.983
walk   ankle max r/s           13.774    13.483
carpet falls/min                0.059     0.078
carpet fwd m/s                  0.584     0.582
carpet yaw p95 rad/s            0.655     0.662
carpet land vxy p95             2.518     2.345
carpet land vz p95              0.542     0.540
carpet ankle peak p95           7.616     7.120
carpet ankle max r/s           13.045    13.268
carpet strikes >13 r/s          0.000     0.000
walk   hip_pitch p95              6.1       6.3
walk   knee_pitch p95             8.1       7.7
walk   ankle_pitch p95            8.2       9.2
stand  >10 rad/s evts               6         5
walk   >10 rad/s evts              29        14

  gate PASS  stand fall/min <= 0.25
  gate PASS  stand episode >= 22 s
  gate PASS  stand >10 rad/s frac <= 1e-4
  gate PASS  walk fwd speed >= 0.9 x measC
  gate PASS  walk knee swing >= 0.8 x measC
  gate PASS  walk knee corr <= -0.6
  gate PASS  walk fall/min <= 0.25
GATE=PASS
```

## Test (on the carpet)

1. Stand first: `capture_run.sh <label>-stand 480 hold`.
2. Then walk on the carpet, battery-powered, with the same calibration as a measC-full bout run
   back to back.
3. **Primary metric:** the ankle_pitch velocity peaks at foot strike. Report the count over
   13 rad/s and the max, against measC-full's 24–25 and measE's 29.2.
4. **Also check:** stance stalls, knee L/R correlation, clearance, torso yaw rate, and the operator
   verdict. A soft landing bought by shuffling is a regression.
