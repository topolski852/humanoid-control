# walk_smoothB-full — extracted 2026-08-29

* **Run:** `humanoid-policy` `logs/rsl_rl/biped/2026-08-28_00-09-42_smoothB-full-resumed`
* **Checkpoint:** `model_5999.pt`
* **Preset:** `HUMANOID_SMOOTH_PRESET=B` — `-0.1 / -0.005 / -3e-4` (roughly 2x preset A)
* **Gains:** uniform `kp=45 / kd=1.5`; knee effort 11.0 Nm (matches flashed hardware)
* **Source:** extracted from `humanoid-policy` commit `7d5b55f` (`deploy/walk/`), which
  replaced A-full there. `policy.onnx.data` md5 `5b85452c487d2bb68660aec5b688d9d6`.

Sim comparison at cmd_vx 0.5, 128 envs (eval_plant_compare.py, modeled plant):

| | fwd | jvel_rms | actrate_rms | falls/min | gait Hz | knee corr |
|---|---|---|---|---|---|---|
| kp45 baseline | 0.500 | 2.145 | 0.480 | 0.234 | 1.62 | -0.83 |
| A-full | 0.485 | 1.840 | 0.357 | **0.141** | 1.56 | -0.84 |
| B-full | 0.480 | **1.696** | **0.316** | 0.188 | 1.50 | -0.765 |

B is the smoothest in sim; A still has the best falls/min.

## Why 7d5b55f replaced A in the deploy folder

That commit's message states A-full **"faulted the robot's encoders across the whole leg"**
on hardware. Treat an A-full run as known-risky and be ready on the E-stop. This is the
reason B exists as a deployed bundle.
