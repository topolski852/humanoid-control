#!/usr/bin/env python3
"""M4/M5 — IMU frame timing, health, and gyro bias/drift at rest.

Two capture paths, because the daemon holds /dev/humanoid_imu open exclusively:

  --via daemon  (default, SAFE alongside a running daemon)
        Samples the daemon's telemetry `base` block. Gives M5 (gyro bias + drift) and
        staleness-gate trips. Cannot give true per-frame timing — telemetry is its own
        100 Hz clock, so it cannot resolve the IMU's own inter-frame distribution.

  --via serial  (requires the daemon STOPPED — it will refuse otherwise)
        Reads /dev/humanoid_imu directly and timestamps every WitMotion frame. This is the
        only way to get real M4 numbers: inter-frame intervals, per-frame-type rates,
        checksum failures, and the duplicate-frame fraction that tells you whether a
        configured 200 Hz output rate is real or just each fused sample sent twice.

        sudo systemctl stop humanoid-daemon
        python scripts/measure/m4m5_imu.py --via serial --seconds 600 --activity rest
        sudo systemctl start humanoid-daemon

No pyserial dependency: baud via `stty`, raw bytes via os.read (same approach as
scripts/imu_setup.py, which is why it works when `import serial` does not).

For M5 the robot must be powered, thermally settled, and COMPLETELY STILL for 10+ minutes.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import select
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import _bootstrap  # noqa: F401,E402
import common as C  # noqa: E402
from humanoid_control.daemon import DaemonClient, RobotConfig  # noqa: E402

DEVICE = "/dev/humanoid_imu"
BAUD = 921600
HEADER = 0x55
FRAME_LEN = 11
TYPES = {0x51: "accel", 0x52: "gyro", 0x53: "euler", 0x59: "quaternion"}
GYRO_SCALE = 2000.0 / 32768.0     # -> deg/s
STALE_AFTER_S = 0.1               # SerialImuBaseState gate


# ------------------------------------------------------------------ serial path


def daemon_running() -> bool:
    return bool(subprocess.run(["pgrep", "-f", "humanoid_daemon"],
                               capture_output=True, text=True).stdout.strip())


def telemetry_port_taken() -> str | None:
    """Who, if anyone, already holds the daemon's telemetry port.

    The daemon BROADCASTS telemetry to one UDP destination (127.0.0.1:9000). UDP delivers each
    datagram to a single bound socket, so a second binder does not get a copy — it steals
    packets from the first. On 2026-08-29 this tool's `--via daemon` path did exactly that to
    the web service running a live policy: `base_valid` flickered false, the UI showed the
    daemon and IMU as dead, and the run had to be aborted mid-capture.

    So: never bind this port while anything else holds it.
    """
    r = subprocess.run(["ss", "-unap"], capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if ":9000" in line and "users:" in line:
            return line.strip()
    return None


def capture_serial(seconds: float, device: str, baud: int) -> dict:
    subprocess.run(["stty", "-F", device, str(baud), "raw", "-echo"], check=True)
    fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
    buf = bytearray()
    arrivals: dict[int, list[float]] = defaultdict(list)
    payloads: dict[int, list[bytes]] = defaultdict(list)
    gyro: list[tuple[float, float, float, float]] = []
    checksum_fail = 0
    total_bytes = 0
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            now = time.time()
            total_bytes += len(chunk)
            buf.extend(chunk)
            while len(buf) >= FRAME_LEN:
                if buf[0] != HEADER:
                    del buf[0]
                    continue
                frame = bytes(buf[:FRAME_LEN])
                if (sum(frame[:10]) & 0xFF) != frame[10]:
                    checksum_fail += 1
                    del buf[0]
                    continue
                t = frame[1]
                arrivals[t].append(now)
                payloads[t].append(frame[2:10])
                if t == 0x52:
                    d = np.frombuffer(frame[2:8], dtype="<i2").astype(float) * GYRO_SCALE
                    gyro.append((now, float(d[0]), float(d[1]), float(d[2])))
                del buf[:FRAME_LEN]
    finally:
        os.close(fd)
    return {"arrivals": dict(arrivals), "payloads": dict(payloads), "gyro": gyro,
            "checksum_fail": checksum_fail, "total_bytes": total_bytes}


def analyse_serial(cap: dict, duration: float) -> dict:
    out: dict = {"per_type": {}, "checksum_fail": cap["checksum_fail"],
                 "total_bytes": cap["total_bytes"],
                 "byte_rate_Bps": cap["total_bytes"] / duration if duration else None}
    all_ts: list[float] = []
    for t, ts in sorted(cap["arrivals"].items()):
        name = TYPES.get(t, hex(t))
        all_ts.extend(ts)
        gaps = np.diff(sorted(ts))
        pl = cap["payloads"][t]
        dupes = sum(1 for a, b in zip(pl, pl[1:]) if a == b)
        out["per_type"][name] = {
            "frames": len(ts),
            "rate_hz": (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 else None,
            "interval_ms": C.stats(gaps, 1e3) if gaps.size else {"n": 0},
            "duplicate_frame_fraction": dupes / max(len(pl) - 1, 1),
            "duplicate_frames": dupes,
        }
    if all_ts:
        gaps = np.diff(sorted(all_ts))
        out["all_frames_interval_ms"] = C.stats(gaps, 1e3)
        out["staleness_trips"] = int((gaps > STALE_AFTER_S).sum())
        out["staleness_trips_per_min"] = out["staleness_trips"] / (duration / 60.0)
    return out


# ------------------------------------------------------------------ daemon path


async def capture_daemon(seconds: float, hz: float) -> dict:
    cfg = RobotConfig.from_json(str(C.robot_config_path()))
    client = DaemonClient(cfg)
    await client.start()
    samples: list[tuple[float, list, list]] = []
    nulls = 0
    period = 1.0 / hz
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            b = client.latest_base()
            if b is None:
                nulls += 1
            else:
                samples.append((time.time(),
                                list(b.get("angular_velocity") or []),
                                list(b.get("projected_gravity") or [])))
            await asyncio.sleep(period)
    finally:
        await client.stop()
    return {"samples": samples, "nulls": nulls}


# ------------------------------------------------------------------ M5 analysis


def analyse_gyro(t: np.ndarray, w: np.ndarray, units: str) -> dict:
    """Per-axis bias, noise and drift — plus the auto-zero diagnostics that decide whether
    the bias numbers mean anything at all.

    This sensor clamps its gyro output to EXACTLY zero while stationary (confirmed 2026-08-29:
    1995 consecutive all-zero frames at rest, then live rates as soon as it is rotated). So a
    "bias" computed over a rest capture is the sensor's clamp, not its bias, and must not be
    fed to NoiseModelWithAdditiveBiasCfg. `zero_fraction` is the number that tells you which
    regime a capture was in:

        zero_fraction ~1.0  -> capture was entirely inside the auto-zero deadband; bias/noise
                               figures here are meaningless.
        zero_fraction ~0.0  -> the joint was moving throughout; the deadband never bit, so the
                               policy sees real angular rate in this regime.
    """
    if t.size < 10:
        return {"note": "insufficient samples", "n": int(t.size)}
    rel = t - t[0]
    minutes = rel / 60.0
    axes = {}
    for i, ax in enumerate("xyz"):
        v = w[:, i]
        slope = float(np.polyfit(minutes, v, 1)[0]) if minutes[-1] > 0 else 0.0
        nz = v[v != 0.0]
        axes[ax] = {
            "mean": float(v.mean()),
            "std": float(v.std(ddof=0)),
            "min": float(v.min()),
            "max": float(v.max()),
            "drift_per_min": slope,
            "zero_fraction": float((v == 0.0).mean()),
            "smallest_nonzero_abs": float(np.abs(nz).min()) if nz.size else None,
        }

    all_zero_rows = float(np.all(w == 0.0, axis=1).mean())
    if all_zero_rows > 0.99:
        verdict = ("AUTO-ZERO CLAMPED: >99% of samples are exactly zero on all axes. The sensor "
                   "was stationary and suppressed its own output. Bias/noise/drift below are "
                   "the clamp, NOT sensor characteristics — do not use them for M5.")
    elif all_zero_rows > 0.05:
        verdict = (f"PARTIALLY CLAMPED: {100*all_zero_rows:.1f}% of samples are exactly zero. "
                   f"The capture straddles the auto-zero deadband; treat statistics with care.")
    else:
        verdict = (f"LIVE: only {100*all_zero_rows:.2f}% of samples are exactly zero — the "
                   f"deadband did not bite in this regime, so these are real rates.")

    return {
        "n": int(t.size),
        "span_s": float(rel[-1]),
        "units": units,
        "all_axes_zero_fraction": all_zero_rows,
        "verdict": verdict,
        "bias": [axes[a]["mean"] for a in "xyz"],
        "noise_std": [axes[a]["std"] for a in "xyz"],
        "drift_per_min": [axes[a]["drift_per_min"] for a in "xyz"],
        "per_axis": axes,
    }


def gravity_deviation_deg(g: np.ndarray) -> dict | None:
    """How far the mean projected-gravity vector sits off vertical [0,0,-1]."""
    if g.size == 0:
        return None
    m = g.mean(axis=0)
    n = np.linalg.norm(m)
    if n == 0:
        return None
    cos = float(np.clip(np.dot(m / n, [0.0, 0.0, -1.0]), -1.0, 1.0))
    return {"mean_vector": m.tolist(), "norm": float(n),
            "deviation_deg": float(np.degrees(np.arccos(cos)))}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--via", choices=["daemon", "serial"], default="daemon")
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--hz", type=float, default=100.0, help="daemon-path sample rate")
    ap.add_argument("--device", default=DEVICE)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--activity", default="rest")
    ap.add_argument("--label", default="")
    ap.add_argument("--outdir", default=None)
    args = ap.parse_args()

    meta = C.capture_meta(args.activity, measurement="M4/M5", via=args.via,
                          policy_label=args.label, imu_device=args.device,
                          imu_baud=args.baud)

    if args.via == "serial":
        if daemon_running():
            print("REFUSING: the daemon holds the IMU port open exclusively.\n"
                  "  sudo systemctl stop humanoid-daemon\n"
                  "  python scripts/measure/m4m5_imu.py --via serial ...\n"
                  "  sudo systemctl start humanoid-daemon", file=sys.stderr)
            return 2
        print(f"[M4/M5] serial capture {args.seconds:.0f}s @ {args.baud} ...", file=sys.stderr)
        cap = capture_serial(args.seconds, args.device, args.baud)
        meta = C.finish_meta(meta)
        res = analyse_serial(cap, meta["duration_s"])
        g = np.array([[x[1], x[2], x[3]] for x in cap["gyro"]], dtype=float)
        t = np.array([x[0] for x in cap["gyro"]], dtype=float)
        res["gyro"] = analyse_gyro(t, g, "deg/s") if t.size else {"note": "no gyro frames"}
        result = {"_meta": meta, "imu": res}
    else:
        holder = telemetry_port_taken()
        if holder:
            print("REFUSING: something already holds the daemon telemetry port 9000:\n"
                  f"  {holder}\n\n"
                  "Binding it a second time STEALS packets from that process — if it is the web\n"
                  "service, the running policy loses base state and faults. Stop the web service\n"
                  "first, or read the IMU through the web API instead. Use --via serial (daemon\n"
                  "stopped) for real M4 frame timing.", file=sys.stderr)
            return 2
        print(f"[M4/M5] daemon-telemetry capture {args.seconds:.0f}s @ {args.hz:g} Hz ...",
              file=sys.stderr)
        cap = asyncio.run(capture_daemon(args.seconds, args.hz))
        meta = C.finish_meta(meta)
        s = cap["samples"]
        t = np.array([x[0] for x in s], dtype=float)
        w = np.array([x[1] for x in s], dtype=float) if s else np.zeros((0, 3))
        gv = np.array([x[2] for x in s], dtype=float) if s else np.zeros((0, 3))
        res = {
            "samples": len(s),
            "null_base_reads": cap["nulls"],
            "note": ("telemetry-clocked, NOT true IMU frame timing — use --via serial for M4"),
            "gyro": analyse_gyro(t, w, "as-reported by daemon (rad/s)") if w.size else
                    {"note": "no base samples"},
            "gravity": gravity_deviation_deg(gv),
        }
        if t.size > 1:
            res["telemetry_interval_ms"] = C.stats(np.diff(t), 1e3)
        result = {"_meta": meta, "imu": res}

    outdir = Path(args.outdir) if args.outdir else C.default_outdir()
    stem = meta["capture_id"] + (f"_{args.label}" if args.label else "") + f"_m4m5_{args.via}"
    out = C.write_json(outdir / f"{stem}.json", result)
    print(json.dumps(result["imu"], indent=2, default=float)[:2200])
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
