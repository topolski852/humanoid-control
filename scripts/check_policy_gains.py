#!/usr/bin/env python3
"""
Check that the robot's LIVE per-joint gains match what a trained policy expects.

READ-ONLY — never writes to the ESCs. It reads each joint's live position_kp / velocity_kp
off the device (READ_CONFIG via the daemon) and diffs them against a policy bundle's expected
gains, so you can confirm "the gains on the robot are what the taught policy was trained with"
at any point — including mid-tuning, without clobbering the values you're tuning.

A policy bundle is a folder in this repo's ``policies/<name>/`` containing
``leg_policy_contract.json`` (canonical joint order + per-joint kp/kd). The robot's daemon
must be running; boot-silent motors may need waking first (``--wake`` sets mode IDLE, which
does NOT change gains).

    python scripts/check_policy_gains.py --policy walk
    python scripts/check_policy_gains.py --policy /path/to/leg_policy_contract.json --wake

Exit code: 0 = every joint within tolerance, 1 = one or more mismatches / unreachable joints.
"""
import _bootstrap  # noqa: F401
import argparse
import asyncio
import json
import os
import sys
from pathlib import Path

from humanoid_control import LegPolicyContract, LIVE_ROBOT_CONFIG_PATH, resolve_robot_config_path
from humanoid_control.daemon import DaemonClient, DaemonError, RobotConfig

# Default location of the trained-policy bundles: this repo's self-hosted policies/ dir (NOT the
# sibling humanoid-policy checkout, which can sit on any branch). Override with the env var.
_DEFAULT_DEPLOY = Path(os.environ.get(
    "HUMANOID_POLICY_DEPLOY", str(Path(__file__).resolve().parent.parent / "policies")))


def _resolve_contract(policy: str) -> Path:
    """Accept a bundle name (walk), a bundle dir, or a direct contract json path."""
    p = Path(policy)
    if p.is_file():
        return p
    if p.is_dir():
        return p / "leg_policy_contract.json"
    cand = _DEFAULT_DEPLOY / policy / "leg_policy_contract.json"
    if cand.is_file():
        return cand
    raise SystemExit(f"Could not find a policy contract for {policy!r} "
                     f"(looked at {p}, {cand}). Use --policy <name|dir|contract.json>.")


def _read_live_gains(client: DaemonClient, name: str, attempts: int = 6) -> tuple[float, float]:
    """Read live position_kp / velocity_kp, retrying the occasional dropped SDO
    (READ_CONFIG sometimes returns a null param — same reason reconcile.py retries)."""
    last = None
    for _ in range(attempts):
        try:
            dev = client.read_device_config(name)
        except DaemonError as exc:
            last = exc
            continue
        kp, kd = dev.get("position_kp"), dev.get("velocity_kp")
        if kp is not None and kd is not None:
            return float(kp), float(kd)
    raise DaemonError(f"incomplete config after {attempts} reads ({last or 'null gains'})")


def _expected_gains(contract_path: Path) -> tuple[list[str], dict[str, tuple[float, float]]]:
    data = json.loads(contract_path.read_text())
    order = list(data["canonical_joint_order"])
    by = {j["joint_name"]: j for j in data["joints"]}
    gains = {n: (float(by[n]["kp"]), float(by[n]["kd"])) for n in order}
    return order, gains


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True,
                    help="policy bundle name (walk|standup|squat), bundle dir, or contract json path")
    ap.add_argument("--config", default=str(resolve_robot_config_path() or LIVE_ROBOT_CONFIG_PATH),
                    help="studio robot config (for the reference 'config' column + joint set)")
    ap.add_argument("--kp-tol", type=float, default=0.5, help="allowed |Δkp| before flagging (default 0.5)")
    ap.add_argument("--kd-tol", type=float, default=0.1, help="allowed |Δkd| before flagging (default 0.1)")
    ap.add_argument("--wake", action="store_true",
                    help="set all joints to IDLE first to wake boot-silent motors (does NOT change gains)")
    args = ap.parse_args()

    contract_path = _resolve_contract(args.policy)
    order, expected = _expected_gains(contract_path)
    print(f"policy contract : {contract_path}", file=sys.stderr)

    # Studio config gains for a reference column (what apply_all_configs WOULD write).
    cfg = None
    config_gains: dict[str, tuple[float, float]] = {}
    if Path(args.config).exists():
        cfg = RobotConfig.from_json(args.config)
        for name in order:
            jc = cfg.joints.get(name)
            if jc is not None:
                config_gains[name] = (float(jc.position_kp), float(jc.velocity_kp))

    client = DaemonClient(cfg)
    await client.start()
    await asyncio.sleep(0.5)   # let a telemetry frame land before judging liveness
    if not client.is_running():
        print("\n⚠️  No telemetry from the daemon yet — if reads below fail, check it's running.",
              file=sys.stderr)
    if args.wake:
        try:
            client.set_all_mode("IDLE")   # wake only; gains untouched
        except DaemonError as exc:
            print(f"⚠️  wake (set_all_mode IDLE) failed: {exc}", file=sys.stderr)

    rows = []
    mismatches = 0
    unreachable = 0
    try:
        for name in order:
            exp_kp, exp_kd = expected[name]
            cfg_kp, cfg_kd = config_gains.get(name, (None, None))
            try:
                live_kp, live_kd = _read_live_gains(client, name)
            except DaemonError as exc:
                unreachable += 1
                rows.append((name, exp_kp, exp_kd, cfg_kp, cfg_kd, None, None, f"UNREACHABLE ({exc})"))
                continue
            bad = abs(live_kp - exp_kp) > args.kp_tol or abs(live_kd - exp_kd) > args.kd_tol
            if bad:
                mismatches += 1
            if live_kp == 0.0:
                verdict = "MISMATCH kp=0 (disabled?)"
            else:
                verdict = "MISMATCH" if bad else "ok"
            rows.append((name, exp_kp, exp_kd, cfg_kp, cfg_kd, live_kp, live_kd, verdict))
    finally:
        await client.stop()

    # ── report ───────────────────────────────────────────────────────────────
    def g(kp, kd):
        return f"{kp:6.2f}/{kd:5.2f}" if kp is not None else "   —  /  — "

    print(f"\nGains check vs policy '{args.policy}'  (tol: kp±{args.kp_tol}, kd±{args.kd_tol})")
    print(f"{'joint':24s} | {'policy kp/kd':>12s} | {'LIVE kp/kd':>12s} | {'config kp/kd':>12s} | verdict")
    print("-" * 78)
    for name, ekp, ekd, ckp, ckd, lkp, lkd, verdict in rows:
        mark = "✓" if verdict == "ok" else ("✗" if verdict.startswith("MISMATCH") else "?")
        short = name.replace("_joint", "")
        print(f"{short:24s} | {g(ekp,ekd):>12s} | {g(lkp,lkd):>12s} | {g(ckp,ckd):>12s} | {mark} {verdict}")

    ok = len(order) - mismatches - unreachable
    print("-" * 78)
    print(f"{ok}/{len(order)} match, {mismatches} mismatch, {unreachable} unreachable.")
    if mismatches:
        print("→ Live robot gains differ from what this policy was trained with. Either re-tune the\n"
              "  robot to the policy's gains, or retrain the policy with the robot's per-joint gains.",
              file=sys.stderr)
    if unreachable and unreachable == len(order):
        print("→ All joints unreachable. Start the daemon (and try --wake for boot-silent motors).",
              file=sys.stderr)
    return 0 if (mismatches == 0 and unreachable == 0) else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
