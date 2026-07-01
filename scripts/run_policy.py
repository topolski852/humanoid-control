#!/usr/bin/env python3
"""
Milestone 5 — run a trained policy.  *** THIS MOVES THE ROBOT. ***

Loads an ONNX/Torch checkpoint and runs the policy loop at the contract ``policy_dt``.
Same safety path as hold_pose (connect → verify → enable → ramp → run; E-stop always on).

SAFETY: user present, robot supported/gantried, low torque. Requires ``--i-am-present``.
Until the IMU lands the base state is an upright stub — this can hold/track but cannot
close a real balance loop; keep the robot supported.

    python scripts/run_policy.py --policy checkpoints/legs.onnx --i-am-present [--seconds 30]
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import sys

import numpy as np

from humanoid_control import (
    LegPolicyContract, LIVE_ROBOT_CONFIG_PATH, PolicyRunner, load_policy,
    UprightStubBaseState, EstopController,
)
from humanoid_control.daemon import DaemonClient, RobotConfig


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", required=True, help="path to .onnx or .pt checkpoint")
    ap.add_argument("--config", default=str(LIVE_ROBOT_CONFIG_PATH))
    ap.add_argument("--command", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="velocity command vector (3,)")
    ap.add_argument("--i-am-present", action="store_true")
    ap.add_argument("--ramp", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=None, help="max run time (s); default until E-stop")
    args = ap.parse_args()

    if not args.i_am_present:
        print("REFUSING: run_policy moves the robot. Re-run with --i-am-present, robot supported.",
              file=sys.stderr)
        return 2

    contract = LegPolicyContract.load()
    print(contract.summary(), file=sys.stderr)
    policy = load_policy(args.policy, num_actions=contract.num_joints)
    print(f"[run_policy] loaded {type(policy).__name__} from {args.policy}", file=sys.stderr)

    client = DaemonClient(RobotConfig.from_json(args.config))
    estop = EstopController(client)
    runner = PolicyRunner(
        client, contract, policy,
        base_source=UprightStubBaseState(),
        command=np.array(args.command, dtype=np.float32),
        estop=estop, ramp_seconds=args.ramp,
    )
    await runner.connect()
    if not runner.prepare():
        print("aborted during ramp.", file=sys.stderr)
        await client.stop()
        return 1
    await runner.run(max_seconds=args.seconds)
    await client.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
