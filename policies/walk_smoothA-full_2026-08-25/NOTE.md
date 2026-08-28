# walk_smoothA-full — 2026-08-25

The dropdown shows a directory name and nothing else, so this records what the name means.

* **Run:** `humanoid-policy` `logs/rsl_rl/biped/2026-08-25_02-13-41_smoothA-full`
* **Checkpoint:** `model_5999.pt` (final of 6000)
* **Preset:** `HUMANOID_SMOOTH_PRESET=a` — `action_rate_l2 -0.05`, `action_l2 -0.002`,
  `dof_vel_l2 -1e-4`. Smooth B is the same code with roughly double those weights; that is the
  only difference between the two.
* **Gains:** uniform `kp=45 / kd=1.5`, matching `configs/leg_policy_params.json`.
* **Verified on copy:** `policy.onnx.data` md5 `89c1c1a0e0b78419eb3451edec0480c2`, identical to
  both `humanoid-policy/deploy/walk/` and that run's own `exported/`.

## Why it is here

Sim picked A over B, then partly retracted the reasoning — `docs/walk-smoothness-sweep.md` §10
records that the screener had false-called every policy as collapsed, and that measured
properly B is *smoother* than A at similar speed. A still scores best on stability
(falls/min 0.141 vs 0.234 baseline). The standing conclusion is that A-FULL is a real
improvement but "whether it fixes the hardware jitter is unverified and not verifiable from
sim" — hence running both on the robot.

## What switching policy does and does not change

It swaps the **network only**. Gains, stand pose, action scale and timing all keep coming from
`configs/leg_policy_params.json`. That is safe here because this bundle agrees with the runtime
on every one of them — checked automatically now, see `scripts/test_policy_compat.py`. A bundle
that disagrees is greyed out in the dropdown and refused by the API.
