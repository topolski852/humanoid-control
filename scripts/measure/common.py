"""Shared helpers for the sim2real measurement captures (docs/SIM2REAL_MEASUREMENTS.md).

Everything here is READ-ONLY with respect to the robot: no config writes, no mode changes,
no motion. Safe to run alongside a live daemon and during a walk run.

Conventions that the whole measurement suite depends on:

* **Wall-clock, not monotonic.** Every timestamp written by these tools is `time.time()`
  (CLOCK_REALTIME, seconds). `candump -t a` timestamps are also CLOCK_REALTIME, which is what
  makes M2 (sample age) computable by correlating a CAN capture against a policy tick log
  WITHOUT patching the C++ daemon. Do not "improve" this to `time.monotonic()` — it would
  silently break the correlation, because the two clocks have unrelated epochs.
* **Legs only.** 12 joints on two buses. Arm channels in the studio config are ignored.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT))

from humanoid_control import LegPolicyContract, resolve_robot_config_path  # noqa: E402

# CANopen-style arbitration id split, matching scripts/can_monitor.py.
NODE_MASK = 0x7F
FUNC_SHIFT = 7
FUNC_MASK = 0xF
FUNC_PDO4 = 0x9   # passive position+velocity broadcast at fast_frame_frequency
FUNC_PDO2 = 0x5
FUNC_HEARTBEAT = 0xE

# Encoder geometry — identical on all 12 leg joints, verified 2026-08-29.
ENCODER_CPR = 4096
GEAR_ABS = 15.0
JOINT_QUANTUM_RAD = (2.0 * np.pi / ENCODER_CPR) / GEAR_ABS   # 1.023e-4 rad


# --------------------------------------------------------------------------- config


def robot_config_path() -> Path:
    p = resolve_robot_config_path()
    if p is None:
        raise SystemExit(
            "No robot config found. Set $HUMANOID_CONFIG or fix ROBOT_CONFIG_CANDIDATES."
        )
    return Path(p)


def leg_node_map() -> dict[tuple[str, int], str]:
    """{(can_channel, can_id): joint_name} for leg joints only."""
    cfg = json.loads(robot_config_path().read_text())
    out: dict[tuple[str, int], str] = {}
    for j in cfg["joints"].values():
        ch = j["can_channel"]
        if "leg" not in ch:
            continue
        out[(ch, int(j["can_id"]))] = j["joint_name"]
    return out


def canonical_joint_order() -> list[str]:
    """The 12 leg joints in the order the policy's observation vector uses."""
    return list(LegPolicyContract.load().joint_order)


def leg_channels() -> list[str]:
    """Leg CAN channels that are actually UP right now."""
    r = subprocess.run(["ip", "-o", "link", "show", "type", "can"],
                       capture_output=True, text=True)
    chans = []
    for line in r.stdout.splitlines():
        m = re.search(r"\d+:\s+(can_\S+?):.*\bUP\b", line)
        if m and "leg" in m.group(1):
            chans.append(m.group(1))
    return sorted(chans)


# --------------------------------------------------------------------------- stats


def stats(samples, scale: float = 1.0) -> dict:
    """Distribution summary. `scale` converts units (e.g. 1e3 for s -> ms)."""
    a = np.asarray(list(samples), dtype=float)
    if a.size == 0:
        return {"n": 0}
    a = a * scale
    return {
        "n": int(a.size),
        "mean": float(a.mean()),
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "p99": float(np.percentile(a, 99)),
        "max": float(a.max()),
        "min": float(a.min()),
        "std": float(a.std(ddof=0)),
    }


# --------------------------------------------------------------------------- health


def bus_counters(chan: str) -> dict:
    """Kernel CAN counters + controller state for one interface."""
    r = subprocess.run(["ip", "-s", "-d", "link", "show", chan],
                       capture_output=True, text=True)
    out: dict = {"state": None, "restarted": None, "bus_errors": None, "arb_lost": None,
                 "error_warn": None, "error_passive": None, "bus_off": None,
                 "rx_packets": None, "tx_packets": None, "rx_dropped": None,
                 "tx_dropped": None}
    m = re.search(r"can state (\S+)", r.stdout)
    if m:
        out["state"] = m.group(1)
    lines = r.stdout.splitlines()
    for i, ln in enumerate(lines):
        if "re-started" in ln and "bus-errors" in ln and i + 1 < len(lines):
            nums = re.findall(r"\d+", lines[i + 1])
            keys = ["restarted", "bus_errors", "arb_lost", "error_warn",
                    "error_passive", "bus_off"]
            if len(nums) >= 6:
                out.update(dict(zip(keys, (int(x) for x in nums[:6]))))
        if ln.strip().startswith("RX:") and i + 1 < len(lines):
            nums = re.findall(r"\d+", lines[i + 1])
            if len(nums) >= 4:
                out["rx_packets"], out["rx_dropped"] = int(nums[1]), int(nums[3])
        if ln.strip().startswith("TX:") and i + 1 < len(lines):
            nums = re.findall(r"\d+", lines[i + 1])
            if len(nums) >= 4:
                out["tx_packets"], out["tx_dropped"] = int(nums[1]), int(nums[3])
    return out


def all_bus_counters() -> dict:
    return {c: bus_counters(c) for c in leg_channels()}


def counter_delta(before: dict, after: dict) -> dict:
    """after - before for the numeric fields; keeps `state` from `after`."""
    out: dict = {}
    for chan, a in after.items():
        b = before.get(chan, {})
        d = {"state": a.get("state")}
        for k, v in a.items():
            if k == "state" or v is None:
                continue
            bv = b.get(k)
            d[k] = (v - bv) if isinstance(bv, int) else v
        out[chan] = d
    return out


def cpu_temp_c() -> float | None:
    """Package temperature in °C, or None if no usable sensor."""
    best = None
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            kind = (zone / "type").read_text().strip()
            milli = int((zone / "temp").read_text().strip())
        except Exception:
            continue
        if "pkg" in kind or "x86" in kind or "cpu" in kind.lower():
            return milli / 1000.0
        best = best if best is not None else milli / 1000.0
    return best


def throttle_flags() -> dict:
    """Best-effort thermal-throttle indicators for M8."""
    out: dict = {}
    try:
        core = Path("/sys/devices/system/cpu/cpu0/thermal_throttle")
        for f in ("core_throttle_count", "package_throttle_count"):
            p = core / f
            if p.exists():
                out[f] = int(p.read_text().strip())
    except Exception:
        pass
    return out


def daemon_info() -> dict:
    """PID / uptime / argv of the running daemon — records WHICH config it loaded.

    A daemon started before the config was last edited is running a stale copy; see the
    B1 write-up in docs/SIM2REAL_MEASUREMENTS.md.
    """
    try:
        pid = subprocess.run(["pgrep", "-f", "humanoid_daemon"],
                             capture_output=True, text=True).stdout.split()
        if not pid:
            return {"running": False}
        pid = int(pid[0])
        cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode().strip()
        started = os.path.getmtime(f"/proc/{pid}")
        cfg = robot_config_path()
        return {
            "running": True,
            "pid": pid,
            "cmdline": cmd,
            "started_epoch": started,
            "uptime_s": time.time() - started,
            "config_path": str(cfg),
            "config_mtime_epoch": cfg.stat().st_mtime,
            "config_newer_than_daemon": cfg.stat().st_mtime > started,
        }
    except Exception as exc:
        return {"running": None, "error": str(exc)}


# --------------------------------------------------------------------------- output


def capture_meta(activity: str, **extra) -> dict:
    """Metadata block every capture must carry."""
    meta = {
        "capture_id": f"{activity}_{time.strftime('%Y%m%dT%H%M%S')}",
        "activity": activity,
        "date": time.strftime("%Y-%m-%d"),
        "started_epoch": time.time(),
        "host": os.uname().nodename,
        "daemon": daemon_info(),
        "cpu_temp_start_c": cpu_temp_c(),
        "throttle_start": throttle_flags(),
        "encoder_quantum_rad": JOINT_QUANTUM_RAD,
    }
    meta.update(extra)
    return meta


def finish_meta(meta: dict) -> dict:
    meta["ended_epoch"] = time.time()
    meta["duration_s"] = meta["ended_epoch"] - meta["started_epoch"]
    meta["cpu_temp_end_c"] = cpu_temp_c()
    meta["throttle_end"] = throttle_flags()
    return meta


def default_outdir() -> Path:
    d = REPO_ROOT / "docs" / "measurements"
    d.mkdir(parents=True, exist_ok=True)
    return d


def write_json(path: Path, obj: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, default=float) + "\n")
    return path
