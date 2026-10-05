#!/usr/bin/env python3
"""READ-ONLY: compare every leg ESC's live config against the robot config file.

    .venv/bin/python scripts/verify_device_config.py            # all 12 leg joints
    .venv/bin/python scripts/verify_device_config.py --json out.json

Writes NOTHING to any ESC. Each joint is read with READ_CONFIG (an SDO read per parameter,
~2-7 s per joint). READ_CONFIG occasionally drops a single parameter, so up to --attempts reads
are merged, keeping the first non-null value of each.

Telemetry is received on a spare port (--tel-port, default 9050), never 9000, so this does not
steal the web service's telemetry. The daemon only pushes telemetry to 9000; the command port
answers whoever asks.

Expected differences, labelled rather than flagged:
  * position_offset: the web leg calibration writes a fresh offset every session.
  * position_limit_min / _max: the daemon stores limits offset-adjusted (limit + position_offset).
Anything else that differs beyond float32 rounding is a real mismatch between file and device.

Context (2026-10-05): the first config write after a daemon start or motor power cycle (in
practice, the first leg calibration) sends EVERY parameter from the daemon's copy of this file,
including electrical_offset, encoder_position_offset and watchdog_timeout. So the file is what
the ESCs run after that click; this tool checks that they agree.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import _bootstrap  # noqa: F401,E402
from humanoid_control import resolve_robot_config_path  # noqa: E402
from humanoid_control.daemon import DaemonClient, RobotConfig  # noqa: E402

LEGS = [f"{s}_{j}_joint" for s in ("left", "right")
        for j in ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")]
EXPECTED = {"position_offset", "position_limit_min", "position_limit_max"}
# Integer ESC params come back from READ_CONFIG as the raw u32 bits reinterpreted as float32
# (100 reads as 1.4e-43, 1000 as 1.4e-42). Reinterpret them before comparing.
INT_PARAMS = {"fast_frame_frequency", "watchdog_timeout", "pole_pairs", "cpr"}


def decode(key: str, v):
    if key in INT_PARAMS and isinstance(v, float):
        import struct
        return struct.unpack("<I", struct.pack("<f", v))[0]
    return v


def file_value(jc: dict, key: str):
    if key == "position_limit_min":
        return (jc.get("position_limits") or {}).get("min")
    if key == "position_limit_max":
        return (jc.get("position_limits") or {}).get("max")
    return jc.get(key)


def same(a, b) -> bool:
    if a is None or b is None:
        return False
    try:
        a, b = float(a), float(b)
    except (TypeError, ValueError):
        return a == b
    return math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-6)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joints", nargs="*", default=LEGS)
    ap.add_argument("--attempts", type=int, default=4)
    ap.add_argument("--tel-port", type=int, default=9050)
    ap.add_argument("--json")
    args = ap.parse_args()

    cfg_path = resolve_robot_config_path()
    raw = json.loads(Path(cfg_path).read_text())["joints"]
    client = DaemonClient(RobotConfig.from_json(str(cfg_path)), tel_port=args.tel_port)
    import asyncio
    asyncio.run(client.start())
    print(f"config file: {cfg_path}\nreading {len(args.joints)} joints (read-only) ...\n", flush=True)

    report, n_bad = {}, 0
    try:
        for j in args.joints:
            dev: dict = {}
            for _ in range(args.attempts):
                try:
                    c = client.read_device_config(j)
                except Exception as exc:
                    print(f"  {j}: read failed ({exc})", flush=True)
                    continue
                for k, v in c.items():
                    if dev.get(k) is None and v is not None:
                        dev[k] = v
                if all(v is not None for v in dev.values()) and dev:
                    break
            rows = []
            for k in sorted(dev):
                dev[k] = decode(k, dev[k])
                fv = file_value(raw[j], k)
                if fv is None and k not in raw[j]:
                    continue                       # device-only param, nothing to compare
                ok = same(dev[k], fv)
                kind = "ok" if ok else ("expected" if k in EXPECTED else
                                        "UNREAD" if dev[k] is None else "MISMATCH")
                rows.append({"param": k, "device": dev[k], "file": fv, "status": kind})
            bad = [r for r in rows if r["status"] in ("MISMATCH", "UNREAD")]
            n_bad += len(bad)
            report[j] = rows
            print(f"{j:26s} {sum(r['status'] == 'ok' for r in rows):2d} ok  "
                  f"{sum(r['status'] == 'expected' for r in rows)} expected  "
                  f"{len(bad)} mismatched/unread", flush=True)
            for r in rows:
                if r["status"] in ("MISMATCH", "UNREAD"):
                    print(f"    {r['status']:8s} {r['param']:26s} device {r['device']!r:>22}  "
                          f"file {r['file']!r}", flush=True)
    finally:
        asyncio.run(client.stop())

    print(f"\n{'ALL MATCH' if n_bad == 0 else f'{n_bad} MISMATCHED/UNREAD PARAMETER(S)'} "
          f"(position_offset and limits excluded: session calibration)")
    if args.json:
        Path(args.json).write_text(json.dumps({"config_file": str(cfg_path), "joints": report},
                                              indent=2, default=str))
        print(f"wrote {args.json}", file=sys.stderr)
    return 0 if n_bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
