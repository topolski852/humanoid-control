"""
Reconcile firmware position limits to the live device offset.

The daemon stores each joint's position limits **offset-adjusted** (limit + position_offset;
see daemon actuator.cpp). So after an offset recalibration or a power cycle, the firmware's
stored limits are in a stale frame and it clamps commands to the wrong range. This re-applies
each joint's URDF limits using its **live** device offset, so the firmware clamps correctly
(e.g. a held joint commanded to 0 actually goes to 0 instead of a clamped, drifted value).

Reads are via READ_CONFIG, which occasionally drops one SDO param — so ``read_live_offset``
retries until ``position_offset`` is non-null.
"""
from __future__ import annotations

import sys
import time

from .daemon import DaemonClient, RobotConfig


def read_live_offset(client: DaemonClient, name: str, attempts: int = 6) -> float | None:
    for _ in range(attempts):
        resp = client.read_device_config(name)
        off = resp.get("config", resp).get("position_offset")
        if off is not None:
            return off
        time.sleep(0.2)
    return None


def reconcile_firmware_limits(
    client: DaemonClient,
    robot_cfg: RobotConfig,
    joints: list[str],
    *,
    log: bool = True,
) -> bool:
    """Rewrite each joint's firmware limits = URDF limits @ live device offset.

    Returns False if any joint's live offset couldn't be read (that joint is skipped).
    Logs a warning where the device offset disagrees with the config (stale calibration).
    """
    ok = True
    for name in joints:
        jc = robot_cfg.joints[name]
        lo, hi = jc.position_limits.lower_bound, jc.position_limits.upper_bound
        off = read_live_offset(client, name)
        if off is None:
            if log:
                print(f"  [reconcile] {name}: could not read live offset — SKIPPING", file=sys.stderr)
            ok = False
            continue
        warn = "" if abs(jc.position_offset - off) < 1e-4 else \
            f"  ⚠️ config offset {jc.position_offset:+.4f} != device {off:+.4f}"
        client.apply_config(name, {
            "position_offset": off,        # keep the device's live zero
            "position_limit_min": lo,      # URDF limits; daemon adds offset internally
            "position_limit_max": hi,
        })
        if log:
            print(f"  [reconcile] {name}: limits=[{lo:+.3f},{hi:+.3f}] @ offset {off:+.4f}{warn}",
                  file=sys.stderr)
    return ok
