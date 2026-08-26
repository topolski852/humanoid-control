#!/usr/bin/env python3
"""
Milestone 1/2 smoke test — READ ONLY, no motion.

Connects to the running daemon, loads the contract + live robot config, and prints the 12
leg joints' live position/velocity in canonical order at ~50 Hz. Optionally wakes the
joints to IDLE first (``--connect``: NMT IDLE + config delta-write, zero torque).

    python scripts/smoke_test.py [--connect] [--hz 50] [--seconds 5]
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import sys

from humanoid_control import LegPolicyContract, LIVE_ROBOT_CONFIG_PATH, resolve_robot_config_path
from humanoid_control.daemon import DaemonClient, RobotConfig


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(resolve_robot_config_path() or LIVE_ROBOT_CONFIG_PATH))
    ap.add_argument("--connect", action="store_true",
                    help="wake joints to IDLE (apply_all_configs) — zero torque, no motion")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    contract = LegPolicyContract.load()
    cfg = RobotConfig.from_json(args.config)
    client = DaemonClient(cfg)
    await client.start()
    try:
        print(f"PING: {client.ping()}")
        if args.connect:
            print("APPLY_ALL_CONFIGS (wake to IDLE, no motion)...")
            client.apply_all_configs()
            await asyncio.sleep(0.4)

        legs = list(contract.joint_order)
        period = 1.0 / args.hz
        n = int(args.seconds * args.hz)
        print(f"Streaming {len(legs)} leg joints @ {args.hz:.0f} Hz for {args.seconds:.0f}s "
              f"(canonical order):\n")
        for k in range(n):
            row = []
            offline = 0
            for name in legs:
                st = client.get_cached_joint_state(name)
                if st is None:
                    row.append(f"{name.split('_joint')[0]:>16}=OFFLINE")
                    offline += 1
                else:
                    row.append(f"{name.split('_joint')[0]:>16}={st['position']:+.3f}")
            if k % max(1, int(args.hz // 5)) == 0:  # ~5 lines/sec
                tag = f" [{offline} offline — run with --connect]" if offline else ""
                sys.stdout.write("  " + "  ".join(row[:6]) + tag + "\n")
            await asyncio.sleep(period)
        print("\nsmoke test done.")
        return 0
    finally:
        await client.stop()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
