#!/usr/bin/env python3
"""
Milestone 3 — hold-pose loop.  *** THIS MOVES THE ROBOT. ***

Enables POSITION on the 12 leg joints, ramps slowly from the current pose to the
contract default_pose, and holds it (ZeroPolicy → action=0 → target=default_pose). This
exercises the full command path + safety scaffolding with no learned net.

SAFETY: only run with the user present and the robot supported/gantried. Requires the
explicit ``--i-am-present`` flag. E-stop: press ENTER/'q' or Ctrl-C.

    python scripts/hold_pose.py --i-am-present [--ramp 5] [--seconds 20]
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import sys

from humanoid_control import (
    LegPolicyContract, LIVE_ROBOT_CONFIG_PATH, resolve_robot_config_path, PolicyRunner, ZeroPolicy,
    UprightStubBaseState, EstopController,
)
from humanoid_control.daemon import DaemonClient, RobotConfig


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(resolve_robot_config_path() or LIVE_ROBOT_CONFIG_PATH))
    ap.add_argument("--i-am-present", action="store_true",
                    help="REQUIRED confirmation that a human is present and the robot is supported")
    ap.add_argument("--ramp", type=float, default=5.0, help="ramp duration to default_pose (s)")
    ap.add_argument("--seconds", type=float, default=20.0, help="hold duration (s)")
    args = ap.parse_args()

    if not args.i_am_present:
        print("REFUSING: hold_pose moves the robot. Re-run with --i-am-present, robot supported.",
              file=sys.stderr)
        return 2

    contract = LegPolicyContract.load()
    print(contract.summary(), file=sys.stderr)
    client = DaemonClient(RobotConfig.from_json(args.config))
    estop = EstopController(client)  # SIGINT + keyboard kill armed
    runner = PolicyRunner(
        client, contract, ZeroPolicy(contract.num_joints),
        base_source=UprightStubBaseState(), estop=estop, ramp_seconds=args.ramp,
    )
    await runner.connect()                     # no motion
    if not runner.prepare():                   # MOTION: enable + ramp to default_pose
        print("aborted during ramp.", file=sys.stderr)
        await client.stop()
        return 1
    await runner.run(max_seconds=args.seconds)  # MOTION: hold default_pose
    await client.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
