#!/usr/bin/env python3
"""
Go to a named pose (from configs/poses.json) and hold it.  *** MOVES THE ROBOT. ***

Set each leg joint to a per-joint target defined in configs/poses.json, ramp there from
the current pose, and hold until E-stop (or --seconds). Joints omitted from the pose (and
anything in always_skip, e.g. the broken ankle_roll) are left DISABLED/uncommanded.

Assumes offsets are calibrated for this session — targets are in the calibrated frame. We
reconcile firmware limits to the live device offsets first and warn on any mismatch.
E-stop: ENTER/'q'/Ctrl-C. Requires --i-am-present.

    python scripts/go_to_pose.py --list
    python scripts/go_to_pose.py --pose zero --i-am-present
    python scripts/go_to_pose.py --pose standing --i-am-present [--seconds 0] [--ramp 4]
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import sys
import time

import numpy as np

from humanoid_control import LIVE_ROBOT_CONFIG_PATH, EstopController, ramp_to_pose, reconcile_firmware_limits
from humanoid_control.daemon import DaemonClient, RobotConfig
from humanoid_control.poses import DEG, LEG_JOINTS, load_poses, pose_names, resolve_pose


def _print_poses(data) -> None:
    print("Available poses (configs/poses.json):", file=sys.stderr)
    for name in pose_names(data):
        targets, skipped = resolve_pose(data, name)
        parts = ", ".join(f"{jn.replace('_joint','')}={v/DEG:+.0f}°" for jn, v in targets.items())
        print(f"  {name:12s} {parts}", file=sys.stderr)
        print(f"  {'':12s} (skipped: {', '.join(j.replace('_joint','') for j in skipped)})", file=sys.stderr)


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pose", help="pose name from configs/poses.json")
    ap.add_argument("--list", action="store_true", help="list poses and exit")
    ap.add_argument("--poses-file", default=None, help="override poses json path")
    ap.add_argument("--config", default=str(LIVE_ROBOT_CONFIG_PATH))
    ap.add_argument("--i-am-present", action="store_true")
    ap.add_argument("--seconds", type=float, default=0.0, help="hold duration; 0 = until E-stop")
    ap.add_argument("--ramp", type=float, default=4.0)
    ap.add_argument("--rate", type=float, default=100.0)
    args = ap.parse_args()

    data = load_poses(args.poses_file) if args.poses_file else load_poses()
    if args.list or not args.pose:
        _print_poses(data)
        return 0 if args.list else 2

    targets_rad, skipped = resolve_pose(data, args.pose)
    if not args.i_am_present:
        print(f"REFUSING: go_to_pose moves the robot. Re-run with --i-am-present.", file=sys.stderr)
        _print_poses(data)
        return 2

    robot_cfg = RobotConfig.from_json(args.config)
    # Controlled joints in canonical order.
    controlled = [j for j in LEG_JOINTS if j in targets_rad]
    lo = np.array([robot_cfg.joints[n].position_limits.lower_bound for n in controlled], dtype=np.float32)
    hi = np.array([robot_cfg.joints[n].position_limits.upper_bound for n in controlled], dtype=np.float32)
    raw = np.array([targets_rad[n] for n in controlled], dtype=np.float32)
    goal = np.clip(raw, lo, hi)

    print(f"[pose:{args.pose}] {len(controlled)} joints; skipping "
          f"{[j.replace('_joint','') for j in skipped]}", file=sys.stderr)
    for i, n in enumerate(controlled):
        clamp = "" if abs(goal[i] - raw[i]) < 1e-6 else "  (CLAMPED to limit)"
        print(f"    {n:26s} -> {goal[i]:+.4f} rad ({goal[i]/DEG:+.1f}°){clamp}", file=sys.stderr)

    client = DaemonClient(robot_cfg)
    estop = EstopController(client)

    def send(vec) -> None:
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
        client.apply_all_configs()               # wake to IDLE (no motion)
        await asyncio.sleep(0.4)
        print(f"[pose] reconciling firmware limits for {len(controlled)} joints...", file=sys.stderr)
        reconcile_firmware_limits(client, robot_cfg, controlled)

        for name in controlled:
            st = client.get_cached_joint_state(name)
            if st is None or (st.get("state") or st.get("joint_state")) == "OFFLINE":
                print(f"REFUSING: {name} offline.", file=sys.stderr); return 1
            if st.get("error"):
                print(f"REFUSING: {name} error=0x{int(st['error']):04x}.", file=sys.stderr); return 1

        start_pose = read_current()
        if np.any(np.isnan(start_pose)):
            print("REFUSING: could not read all joint positions.", file=sys.stderr); return 1

        for name in controlled:
            client.set_mode(name, "POSITION")
        send(start_pose)                          # seed hold at current (no jerk)
        time.sleep(0.05)

        print(f"[pose] ramping to '{args.pose}' over {args.ramp:.0f}s...", file=sys.stderr)
        if not ramp_to_pose(start=start_pose, goal=goal, send=send,
                            duration_s=args.ramp, rate_hz=min(args.rate, 100.0),
                            should_abort=lambda: estop.fired):
            print("[pose] aborted during ramp-in.", file=sys.stderr); return 1

        dur = "until E-stop" if args.seconds <= 0 else f"for {args.seconds:.0f}s"
        print(f"[pose] holding '{args.pose}' {dur}. E-stop: ENTER/'q'/Ctrl-C.", file=sys.stderr)
        t0 = time.monotonic()
        while not estop.fired:
            if args.seconds > 0 and (time.monotonic() - t0) >= args.seconds:
                break
            send(goal)
            await asyncio.sleep(0.1)

        if not estop.fired:
            for name in controlled:
                client.set_mode(name, "IDLE")
            print("[pose] done; controlled joints IDLE.", file=sys.stderr)
        return 0
    finally:
        await client.stop()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
