#!/usr/bin/env python3
"""
Write a trained policy's per-joint PD gains (kp/kd) to the ESCs.  *** WRITES TO HARDWARE. ***

Sets each joint's position_kp and velocity_kp (Kd) to the values the policy was trained with,
using the surgical WRITE_GAINS SDO — it does NOT touch position_offset, limits, or gear, so
your session calibration is preserved. position_ki and torque_limit are read and written back
UNCHANGED (only the PD gains change).

⚠️  These gains are volatile: a `connect` / apply_all_configs (including the web app's Connect
button) rewrites gains from the studio config. Run the sequence: connect → THIS script → arm →
test, without reconnecting in between. Requires --yes to actually write.

    python scripts/write_policy_gains.py --policy walk --yes
    python scripts/write_policy_gains.py --policy walk --dry-run     # show what would change

Exit code: 0 = every joint written + verified, 1 = one or more failed.
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import json
import os
import sys
from pathlib import Path

from humanoid_control import LIVE_ROBOT_CONFIG_PATH, resolve_robot_config_path
from humanoid_control.daemon import DaemonClient, DaemonError, RobotConfig

# Trained-policy bundles: this repo's self-hosted policies/ dir (NOT the sibling humanoid-policy
# checkout, which can sit on any branch). Override with the env var.
_DEFAULT_DEPLOY = Path(os.environ.get(
    "HUMANOID_POLICY_DEPLOY", str(Path(__file__).resolve().parent.parent / "policies")))


def _resolve_contract(policy: str) -> Path:
    p = Path(policy)
    if p.is_file():
        return p
    if p.is_dir():
        return p / "leg_policy_contract.json"
    cand = _DEFAULT_DEPLOY / policy / "leg_policy_contract.json"
    if cand.is_file():
        return cand
    raise SystemExit(f"Could not find a policy contract for {policy!r} (looked at {p}, {cand}).")


def _read_config(client: DaemonClient, name: str, attempts: int = 6) -> dict:
    """Read a joint's live config, retrying the occasional dropped SDO (null param)."""
    last = None
    for _ in range(attempts):
        try:
            cfg = client.read_device_config(name)
        except DaemonError as exc:
            last = exc
            continue
        if cfg.get("position_kp") is not None and cfg.get("velocity_kp") is not None:
            return cfg
    raise DaemonError(f"incomplete config after {attempts} reads ({last or 'null gains'})")


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True, help="bundle name (walk|standup|squat), dir, or contract json")
    ap.add_argument("--config", default=str(resolve_robot_config_path() or LIVE_ROBOT_CONFIG_PATH), help="studio config (joint set)")
    ap.add_argument("--yes", action="store_true", help="REQUIRED to actually write to the ESCs")
    ap.add_argument("--dry-run", action="store_true", help="show the before→after plan, write nothing")
    ap.add_argument("--wake", action="store_true", help="set mode IDLE first to wake boot-silent motors")
    args = ap.parse_args()

    if not args.yes and not args.dry_run:
        print("REFUSING: this writes gains to the ESCs. Re-run with --yes (or --dry-run to preview).",
              file=sys.stderr)
        return 2

    contract_path = _resolve_contract(args.policy)
    data = json.loads(contract_path.read_text())
    order = list(data["canonical_joint_order"])
    by = {j["joint_name"]: j for j in data["joints"]}
    print(f"policy contract : {contract_path}", file=sys.stderr)
    print(f"mode            : {'DRY-RUN (no writes)' if args.dry_run else 'WRITE'}", file=sys.stderr)

    cfg = RobotConfig.from_json(args.config) if Path(args.config).exists() else None
    client = DaemonClient(cfg)
    await client.start()
    await asyncio.sleep(0.5)
    if args.wake:
        try:
            client.set_all_mode("IDLE")
        except DaemonError as exc:
            print(f"⚠️  wake failed: {exc}", file=sys.stderr)

    rows = []
    failed = 0
    warn_no_torque = []
    try:
        for name in order:
            want_kp = float(by[name]["kp"])
            want_kd = float(by[name]["kd"])
            try:
                cur = _read_config(client, name)
            except DaemonError as exc:
                failed += 1
                rows.append((name, want_kp, want_kd, None, None, None, None, f"READ FAIL ({exc})"))
                continue
            old_kp, old_kd = float(cur["position_kp"]), float(cur["velocity_kp"])
            keep_ki = float(cur.get("position_ki") or 0.0)      # preserved unchanged
            keep_tl = float(cur.get("torque_limit") or 0.0)     # preserved unchanged
            if keep_tl == 0.0:
                warn_no_torque.append(name)

            if args.dry_run:
                rows.append((name, want_kp, want_kd, old_kp, old_kd, old_kp, old_kd, "would write"))
                continue

            try:
                client.write_gains(name, kp=want_kp, ki=keep_ki, velocity_kp=want_kd, torque_limit=keep_tl)
            except DaemonError as exc:
                failed += 1
                rows.append((name, want_kp, want_kd, old_kp, old_kd, None, None, f"WRITE FAIL ({exc})"))
                continue
            # verify by reading back
            try:
                back = _read_config(client, name)
                new_kp, new_kd = float(back["position_kp"]), float(back["velocity_kp"])
            except DaemonError as exc:
                rows.append((name, want_kp, want_kd, old_kp, old_kd, None, None, f"WROTE, VERIFY FAIL ({exc})"))
                continue
            ok = abs(new_kp - want_kp) < 0.05 and abs(new_kd - want_kd) < 0.05
            if not ok:
                failed += 1
            rows.append((name, want_kp, want_kd, old_kp, old_kd, new_kp, new_kd, "✓ verified" if ok else "✗ MISMATCH"))
    finally:
        await client.stop()

    def g(kp, kd):
        return f"{kp:6.2f}/{kd:5.2f}" if kp is not None else "   —  /  — "

    print(f"\nWrite policy '{args.policy}' gains  (kp/kd only; ki + torque_limit preserved)")
    print(f"{'joint':22s} | {'target':>12s} | {'was':>12s} | {'now':>12s} | result")
    print("-" * 76)
    for name, wkp, wkd, okp, okd, nkp, nkd, res in rows:
        print(f"{name.replace('_joint',''):22s} | {g(wkp,wkd):>12s} | {g(okp,okd):>12s} | {g(nkp,nkd):>12s} | {res}")
    print("-" * 76)
    wrote = len(order) - failed
    print(f"{wrote}/{len(order)} {'planned' if args.dry_run else 'written+verified'}, {failed} failed.")
    if warn_no_torque:
        print(f"⚠️  torque_limit=0 (won't produce torque — check these motors): "
              f"{', '.join(n.replace('_joint','') for n in warn_no_torque)}", file=sys.stderr)
    if not args.dry_run:
        print("⚠️  Volatile: a connect/apply_all_configs will overwrite these. Do NOT reconnect "
              "before the test.", file=sys.stderr)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
