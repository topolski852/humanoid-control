# humanoid-control

Onboard runtime control for the Berkeley Humanoid Lite robot. This repo runs **on
the robot's PC**: it loads a learned policy and drives the leg joints to stand
(squat → stand) by talking to the real-time CAN daemon over UDP.

```
policy runner (Python)  ──UDP :9001/:9000──▶  daemon (C++, owns CAN @200 Hz)  ──CAN──▶  ESCs (22 joints)
```

## Layout
- `daemon/` — the real-time CAN daemon (C++). **Copied verbatim from
  [`humanoid-studio`](https://github.com/topolski852/humanoid-studio); keep the two
  in sync.** Build with `cd daemon && make`. Only one daemon may own the CAN bus at
  a time (don't run this and the humanoid-studio app's daemon simultaneously).
- `docs/HANDOFF.md`, `docs/DAEMON_SPEC.md` — the authoritative, source-verified
  reference for the firmware, CAN protocol, daemon UDP API, joint model, and the
  planned IMU telemetry contract. **Read these first.**
- `PROMPT.md` — the build plan / kickoff brief for this repo.
- Control package — to be written (see `PROMPT.md`).

## Config is shared, not forked
The robot's live per-joint config (gains, offsets, gear signs, limits — updated by
the humanoid-studio app during commissioning/tuning) is the single source of truth
at `/home/nse/humanoid-studio/configs/humanoid_lite.json`. Point the daemon
(`--config`) and `RobotConfig` at it so tuning done in the app is reflected here.

## Status
Scaffold: daemon + docs in place. Control code not yet written — start from `PROMPT.md`.