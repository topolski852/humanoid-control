#!/usr/bin/env python3
"""
Bench "bicycle-kick" sine sweep — a show-off demo.  *** THIS MOVES THE ROBOT. ***

For a robot mounted on the bench (NOT in the squat pose). Holds hip_yaw + hip_roll at 0°,
and sweeps hip_pitch / knee_pitch / ankle_pitch as sine waves on both legs (legs antiphase
by default → a pedaling look). ankle_roll is ignored (parked IDLE — broken encoder).

Default sweep (deg): hip_pitch center −50 amp 25 · knee_pitch center 30 amp 25 ·
ankle_pitch center 0 amp 25. Held: hip_yaw 0, hip_roll 0.

Safety: POSITION mode actively holds the non-swept joints so gravity can't back-drive them.
Every target is clamped to URDF position_limits. On start we RECONCILE each joint's firmware
limits to its live offset (this session's recal left them stale) so the clamp is correct.
Ramps from the current mounted pose to the sweep start (never steps). E-stop: ENTER/'q'/Ctrl-C.

    python scripts/bench_sweep.py --i-am-present [--freq 0.3] [--seconds 20] [--leg-phase-deg 180]
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import math
import sys
import time

import numpy as np

from humanoid_control import LegPolicyContract, LIVE_ROBOT_CONFIG_PATH, EstopController, ramp_to_pose
from humanoid_control.daemon import DaemonClient, RobotConfig

DEG = math.pi / 180.0

# joint-type -> (center_deg, amplitude_deg). amp 0 = held. ankle_roll excluded entirely.
SWEEP_SPEC = {
    "hip_roll":    (0.0,   0.0),
    "hip_yaw":     (0.0,   0.0),
    "hip_pitch":   (-50.0, 25.0),
    "knee_pitch":  (30.0,  25.0),
    "ankle_pitch": (0.0,   25.0),
}
EXCLUDE = {"ankle_roll"}
SIDES = ("left", "right")


def _read_live_offset(client: DaemonClient, name: str, attempts: int = 6) -> float | None:
    """READ_CONFIG is flaky (one param may drop); retry until position_offset is non-null."""
    for _ in range(attempts):
        cfg = client.read_device_config(name)
        off = cfg.get("config", cfg).get("position_offset")
        if off is not None:
            return off
        time.sleep(0.2)
    return None


def reconcile_limits(client: DaemonClient, robot_cfg: RobotConfig, joints: list[str]) -> bool:
    """Re-write each joint's firmware position limits = URDF limits, offset-adjusted by the
    LIVE device offset. Returns False if a live offset couldn't be read."""
    ok = True
    for name in joints:
        jc = robot_cfg.joints[name]
        lo, hi = jc.position_limits.lower_bound, jc.position_limits.upper_bound
        off = _read_live_offset(client, name)
        if off is None:
            print(f"  [reconcile] {name}: could not read live offset — SKIPPING", file=sys.stderr)
            ok = False
            continue
        json_off = jc.position_offset
        warn = "" if abs(json_off - off) < 1e-4 else f"  ⚠️ config offset {json_off:+.4f} != device {off:+.4f}"
        client.apply_config(name, {
            "position_offset": off,           # keep device's live zero
            "position_limit_min": lo,         # URDF limits (daemon adds offset internally)
            "position_limit_max": hi,
        })
        print(f"  [reconcile] {name}: limits=[{lo:+.3f},{hi:+.3f}] @ offset {off:+.4f}{warn}", file=sys.stderr)
    return ok


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(LIVE_ROBOT_CONFIG_PATH))
    ap.add_argument("--i-am-present", action="store_true",
                    help="REQUIRED: human present, robot secured to the bench")
    ap.add_argument("--freq", type=float, default=0.3, help="sine frequency (Hz)")
    ap.add_argument("--seconds", type=float, default=20.0, help="sweep duration (s)")
    ap.add_argument("--leg-phase-deg", type=float, default=180.0,
                    help="phase offset of right leg vs left (180 = pedaling)")
    ap.add_argument("--ramp", type=float, default=4.0, help="ramp-in duration (s)")
    ap.add_argument("--rate", type=float, default=100.0, help="command rate (Hz)")
    args = ap.parse_args()

    if not args.i_am_present:
        print("REFUSING: bench_sweep moves the robot. Re-run with --i-am-present, robot secured.",
              file=sys.stderr)
        return 2

    contract = LegPolicyContract.load()
    robot_cfg = RobotConfig.from_json(args.config)

    # Build the controlled-joint list + per-joint sine params (canonical-ish order, both legs).
    controlled: list[str] = []
    center = {}
    amp = {}
    leg_phase = {"left": 0.0, "right": args.leg_phase_deg * DEG}
    for side in SIDES:
        for jt, (c_deg, a_deg) in SWEEP_SPEC.items():
            if jt in EXCLUDE:
                continue
            name = f"{side}_{jt}_joint"
            controlled.append(name)
            center[name] = c_deg * DEG
            amp[name] = a_deg * DEG

    # Per-joint URDF limit arrays (for clamping), aligned to `controlled`.
    lo = np.array([robot_cfg.joints[n].position_limits.lower_bound for n in controlled], dtype=np.float32)
    hi = np.array([robot_cfg.joints[n].position_limits.upper_bound for n in controlled], dtype=np.float32)
    c_arr = np.array([center[n] for n in controlled], dtype=np.float32)
    a_arr = np.array([amp[n] for n in controlled], dtype=np.float32)
    ph_arr = np.array([leg_phase[n.split("_")[0]] for n in controlled], dtype=np.float32)

    # Sanity: warn if any sweep extreme exceeds a URDF limit (it will be clamped).
    for i, n in enumerate(controlled):
        top, bot = c_arr[i] + a_arr[i], c_arr[i] - a_arr[i]
        if top > hi[i] + 1e-4 or bot < lo[i] - 1e-4:
            print(f"  ⚠️ {n}: sweep [{bot:+.3f},{top:+.3f}] exceeds limits [{lo[i]:+.3f},{hi[i]:+.3f}] "
                  f"— will be clamped.", file=sys.stderr)

    def targets_at(t: float) -> np.ndarray:
        raw = c_arr + a_arr * np.sin(2 * math.pi * args.freq * t + ph_arr)
        return np.clip(raw, lo, hi)

    client = DaemonClient(robot_cfg)
    estop = EstopController(client)

    def send(vec: np.ndarray) -> None:
        for name, v in zip(controlled, vec):
            client.set_position(name, float(v))

    def read_current() -> np.ndarray:
        out = np.zeros(len(controlled), dtype=np.float32)
        for i, name in enumerate(controlled):
            st = client.get_cached_joint_state(name)
            out[i] = st["position"] if st else np.nan
        return out

    await client.start()
    try:
        client.apply_all_configs()                 # wake to IDLE (no motion)
        await asyncio.sleep(0.4)
        print(f"[bench] reconciling firmware limits for {len(controlled)} joints "
              f"(excluding {sorted(EXCLUDE)})...", file=sys.stderr)
        reconcile_limits(client, robot_cfg, controlled)

        # Health: controlled joints must be online + fault-free.
        for name in controlled:
            st = client.get_cached_joint_state(name)
            if st is None or (st.get("state") or st.get("joint_state")) == "OFFLINE":
                print(f"REFUSING: {name} offline.", file=sys.stderr); return 1
            if st.get("error"):
                print(f"REFUSING: {name} error=0x{int(st['error']):04x}.", file=sys.stderr); return 1

        start_pose = read_current()
        if np.any(np.isnan(start_pose)):
            print("REFUSING: could not read all joint positions.", file=sys.stderr); return 1

        # Enable POSITION only on controlled joints; seed hold at current pose (no jerk).
        for name in controlled:
            client.set_mode(name, "POSITION")
        send(start_pose)
        time.sleep(0.05)

        # Ramp current -> sweep start (t=0), then run the sine.
        print(f"[bench] ramping to sweep start over {args.ramp:.0f}s...", file=sys.stderr)
        if not ramp_to_pose(start=start_pose, goal=targets_at(0.0), send=send,
                            duration_s=args.ramp, rate_hz=min(args.rate, 100.0),
                            should_abort=lambda: estop.fired):
            print("[bench] aborted during ramp-in.", file=sys.stderr)
            return 1

        print(f"[bench] SWEEPING @ {args.freq} Hz for {args.seconds:.0f}s. "
              f"E-stop: ENTER/'q'/Ctrl-C.", file=sys.stderr)
        dt = 1.0 / args.rate
        t0 = time.monotonic()
        next_tick = t0
        while not estop.fired:
            t = time.monotonic() - t0
            if t >= args.seconds:
                break
            send(targets_at(t))
            next_tick += dt
            sleep = next_tick - time.monotonic()
            await asyncio.sleep(sleep if sleep > 0 else 0)

        # Graceful finish: if not estopped, ramp back to the mounted start pose, then IDLE.
        if not estop.fired:
            print("[bench] ramping back to start pose...", file=sys.stderr)
            ramp_to_pose(start=targets_at(time.monotonic() - t0), goal=start_pose, send=send,
                         duration_s=args.ramp, rate_hz=min(args.rate, 100.0),
                         should_abort=lambda: estop.fired)
            for name in controlled:
                client.set_mode(name, "IDLE")
            print("[bench] done; joints IDLE.", file=sys.stderr)
        return 0
    finally:
        await client.stop()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
