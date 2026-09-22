#!/usr/bin/env python3
"""
Milestone 5 — run a trained policy.  *** THIS MOVES THE ROBOT. ***

Loads an ONNX/Torch checkpoint and runs the policy loop at the contract ``policy_dt``.
Same safety path as hold_pose (connect → verify → enable → ramp → run; E-stop always on).

SAFETY: user present, robot supported/gantried, low torque. Requires ``--i-am-present``.
The balance loop is unproven on this robot — keep it supported.

BASE STATE COMES FROM THE DAEMON'S IMU, the same source the web path uses. The policy
observes ``base_ang_vel`` and ``projected_gravity``; feeding it the upright stub fabricates
six of its 45 observations and tells it the robot is perfectly level and perfectly still,
forever. This script passed that stub for its whole life. ``--stub-base`` restores it
deliberately (bench work with no IMU attached); otherwise a missing ``base`` block is
reported by the runner rather than silently substituted.

The daemon only reads the IMU when it was started with ``--imu-device`` — the robot config
carries no ``imu`` block (see scripts/start_stack.sh). If the daemon says `imu: disabled`,
telemetry carries `base: null` and the runner falls back to the stub with a warning.

    python scripts/run_policy.py --policy checkpoints/legs.onnx --i-am-present [--seconds 30]
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import sys

import numpy as np

from humanoid_control import (
    LegPolicyContract, LIVE_ROBOT_CONFIG_PATH, resolve_robot_config_path, PolicyRunner, load_policy,
    TelemetryBaseState, UprightStubBaseState, EstopController,
)
from humanoid_control.daemon import DaemonClient, RobotConfig


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", required=True, help="path to .onnx or .pt checkpoint")
    ap.add_argument("--config", default=str(resolve_robot_config_path() or LIVE_ROBOT_CONFIG_PATH))
    ap.add_argument("--command", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    help="velocity command vector (3,)")
    ap.add_argument("--i-am-present", action="store_true")
    ap.add_argument("--ramp", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=None, help="max run time (s); default until E-stop")
    ap.add_argument("--stub-base", action="store_true",
                    help="feed the upright stub instead of the daemon's IMU (no IMU attached)")
    ap.add_argument("--require-imu", action="store_true",
                    help="refuse to run if the daemon reports no fresh IMU data")
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
    # Same base source as humanoid_control.web.service: the daemon's `base` telemetry block.
    base_source = (UprightStubBaseState() if args.stub_base
                   else TelemetryBaseState(lambda: {"base": client.latest_base()}))
    if args.stub_base:
        print("[run_policy] --stub-base: base state is FABRICATED (upright, still). "
              "The policy cannot balance on this — keep the robot supported.", file=sys.stderr)
    runner = PolicyRunner(
        client, contract, policy,
        base_source=base_source,
        command=np.array(args.command, dtype=np.float32),
        estop=estop, ramp_seconds=args.ramp,
        require_valid_base=args.require_imu and not args.stub_base,
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
