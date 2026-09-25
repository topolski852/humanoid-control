#!/usr/bin/env python3
"""M1/M2/M3/M6 — passive CAN capture. The ONLY live capture tool; everything else is offline.

Captures PDO4 fast-frames on both leg buses with kernel timestamps AND decodes their payload
(position + velocity, little-endian f32 at bytes 0-3 / 4-7 — see actuator.cpp:50-53). One
passive stream therefore yields:

  * **M1** inter-frame interval distribution, per joint
  * **M2** sample age at the policy tick (offline, in analyse.py)
  * **M3** dropouts, from real frame gaps
  * **M6** encoder quantisation and noise, at the full 100 Hz rather than the policy's 25 Hz

### Why this does not use a DaemonClient

It must not, and neither must anything else that runs during a policy.

The daemon *broadcasts* telemetry to a single UDP destination, 127.0.0.1:9000
(`robot.cpp:22`). A second process that binds that port does not get a copy — UDP delivers each
datagram to ONE bound socket, so a second client **steals roughly half the packets from the web
service**, which is what actually runs the policy. On 2026-08-29 an earlier version of this
suite did exactly that: `TelemetryBaseState` started returning None on the stolen packets, the
policy's `base_valid` went false intermittently, the web UI showed the daemon and IMU as dead,
and the run had to be aborted. The tool reported itself as read-only, and it was — with respect
to the robot. It was not read-only with respect to the telemetry stream.

`candump` reads from a packet socket on the CAN interface. It takes a *copy* of frames the
daemon also receives; it cannot starve anything. That is why the whole measurement path was
moved onto it.

Nothing in scripts/measure/ may import DaemonClient. If you need IMU data, go through the web
service's own API so there is exactly one telemetry consumer.

    python scripts/measure/m1_can_timing.py --seconds 600 --label smoothA-stand --activity hold
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

# candump -t a:  (1788029164.047771)  can_left_leg  48D   [8]  BC 6F F0 3C 00 00 00 00
LINE = re.compile(
    r"\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s*([0-9A-Fa-f ]*)"
)

NOMINAL_HZ = 100.0
NOMINAL_MS = 1000.0 / NOMINAL_HZ
THERMAL_EVERY_S = 5.0
MAX_EVENTS = 200000     # cap on recorded EMCY/heartbeat frames; overflow is counted, not stored


_STOP = {"flag": False}


def _install_stop_handler() -> None:
    """Ctrl-C / SIGTERM ends the capture EARLY BUT CLEANLY, keeping what was collected.

    A walk attempt is unpredictable and often has to be cut short. Without this the capture
    writes only after its deadline, so stopping it loses the entire run — which is exactly
    what happened to an aborted 10-minute stand on 2026-08-29. Now a stop signal just breaks
    the read loop and everything collected so far is analysed and written.
    """
    import signal

    def _h(signum, _frame):
        if _STOP["flag"]:          # second signal: give up immediately
            raise KeyboardInterrupt
        _STOP["flag"] = True
        print(f"\n[capture] signal {signum} — finishing early and writing what we have ...",
              file=sys.stderr)

    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(s, _h)
        except (ValueError, OSError):
            pass


def capture(chans: list[str], seconds: float) -> dict:
    """Passive candump. Returns per-joint parallel arrays of (ts, pos, vel) plus thermals."""
    _install_stop_handler()
    nodes = C.leg_node_map()
    proc = subprocess.Popen(
        ["candump", "-t", "a"] + chans,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
    )
    # Non-PDO4 frames are the FAULT record. An EMCY (func 0x1) burst is how this robot fails:
    # a node floods EMCY, the bus jams, SDOs stop being acked, and the whole leg goes 0x40.
    # Counting those frames is not enough -- the timestamp, node and payload are the diagnosis,
    # so they are recorded in full up to a cap (an EMCY flood can be tens of thousands of
    # frames and must not exhaust memory mid-capture).
    events: list[dict] = []
    events_dropped = 0
    ts_by: dict[str, list[float]] = defaultdict(list)
    pos_by: dict[str, list[float]] = defaultdict(list)
    vel_by: dict[str, list[float]] = defaultdict(list)
    other_funcs: dict[str, int] = defaultdict(int)
    thermal: list[dict] = []
    next_thermal = 0.0
    short_frames = 0
    deadline = time.time() + seconds
    try:
        for line in proc.stdout:
            now = time.time()
            if now >= deadline or _STOP["flag"]:
                break
            if now >= next_thermal:
                next_thermal = now + THERMAL_EVERY_S
                thermal.append({"t": now, "cpu_temp_c": C.cpu_temp_c(),
                                **C.throttle_flags()})
            m = LINE.search(line)
            if not m:
                continue
            chan, arb = m.group(2), int(m.group(3), 16)
            name = nodes.get((chan, arb & C.NODE_MASK))
            if name is None:
                continue
            func = (arb >> C.FUNC_SHIFT) & C.FUNC_MASK
            if func != C.FUNC_PDO4:
                other_funcs[f"{name}:0x{func:X}"] += 1
                if func in (C.FUNC_EMCY, C.FUNC_HEARTBEAT):
                    if len(events) < MAX_EVENTS:
                        events.append({"t": float(m.group(1)), "joint": name, "chan": chan,
                                       "func": f"0x{func:X}",
                                       "name": {0x1: "EMCY", 0xE: "HB"}.get(func, "?"),
                                       "data": m.group(5).strip()})
                    else:
                        events_dropped += 1
                continue
            raw = bytes.fromhex(m.group(5).replace(" ", ""))
            if len(raw) < 8:
                short_frames += 1
                continue
            pos, vel = struct.unpack("<ff", raw[:8])
            ts_by[name].append(float(m.group(1)))
            pos_by[name].append(pos)
            vel_by[name].append(vel)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    return {"ts": dict(ts_by), "pos": dict(pos_by), "vel": dict(vel_by),
            "other_funcs": dict(other_funcs), "thermal": thermal,
            "short_frames": short_frames, "stopped_early": _STOP["flag"],
            "events": events, "events_dropped": events_dropped}


def analyse_timing(ts_by: dict[str, list[float]]) -> dict:
    out = {}
    for joint in C.canonical_joint_order():
        ts = ts_by.get(joint, [])
        if len(ts) < 2:
            out[joint] = {"frames": len(ts), "note": "insufficient frames"}
            continue
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        span = ts[-1] - ts[0]
        out[joint] = {
            "frames": len(ts),
            "span_s": span,
            "rate_hz": (len(ts) - 1) / span if span > 0 else None,
            "interval_ms": C.stats(gaps, 1e3),
            "gaps_over_2x_nominal": int(sum(1 for g in gaps if g * 1e3 > 2 * NOMINAL_MS)),
            "gaps_over_10x_nominal": int(sum(1 for g in gaps if g * 1e3 > 10 * NOMINAL_MS)),
            "first_ts": ts[0],
            "last_ts": ts[-1],
        }
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--activity", default="walk", help="walk | hold | rest | bench")
    ap.add_argument("--outdir", default=None)
    args = ap.parse_args()

    chans = C.leg_channels()
    if not chans:
        print("No leg CAN interfaces are UP.", file=sys.stderr)
        return 1

    meta = C.capture_meta(args.activity, measurement="M1/M2/M3/M6",
                          policy_label=args.label, channels=chans,
                          nominal_pdo4_hz=NOMINAL_HZ,
                          capture_method="passive candump (no daemon client)")
    before = C.all_bus_counters()
    print(f"[capture] {args.seconds:.0f}s on {', '.join(chans)}"
          f"{' label=' + args.label if args.label else ''} ...", file=sys.stderr)
    cap = capture(chans, args.seconds)
    after = C.all_bus_counters()
    meta = C.finish_meta(meta)
    meta["stopped_early"] = cap.get("stopped_early", False)
    if cap["thermal"]:
        temps = [x["cpu_temp_c"] for x in cap["thermal"] if x.get("cpu_temp_c")]
        if temps:
            meta["cpu_temp_min_c"], meta["cpu_temp_max_c"] = min(temps), max(temps)
            meta["cpu_temp_rise_c"] = temps[-1] - temps[0]

    result = {
        "_meta": meta,
        "per_joint": analyse_timing(cap["ts"]),
        "thermal_series": cap["thermal"],
        "bus_counters_before": before,
        "bus_counters_after": after,
        "bus_counters_delta": C.counter_delta(before, after),
        "non_pdo4_frames": cap["other_funcs"],
        "short_frames": cap["short_frames"],
        "fault_events": cap["events"],
        "fault_events_dropped": cap["events_dropped"],
    }

    outdir = Path(args.outdir) if args.outdir else C.default_outdir()
    stem = meta["capture_id"] + (f"_{args.label}" if args.label else "") + "_can"
    out = C.write_json(outdir / f"{stem}.json", result)

    frames_path = outdir / f"{stem}.frames.jsonl"
    with frames_path.open("w") as f:
        for joint in sorted(cap["ts"]):
            f.write(json.dumps({"joint": joint, "ts": cap["ts"][joint],
                                "pos": cap["pos"][joint],
                                "vel": cap["vel"][joint]}) + "\n")

    total = sum(len(v) for v in cap["ts"].values())
    print(f"\n[capture] {total} PDO4 frames over {meta['duration_s']:.1f}s", file=sys.stderr)
    print(f"{'joint':26s} {'frames':>7s} {'Hz':>7s} {'mean':>7s} {'p95':>7s} "
          f"{'p99':>7s} {'max':>8s} {'>2x':>5s}")
    for joint in C.canonical_joint_order():
        r = result["per_joint"][joint]
        if "interval_ms" not in r:
            print(f"{joint:26s} {r['frames']:>7d}   (insufficient frames)")
            continue
        s = r["interval_ms"]
        print(f"{joint:26s} {r['frames']:>7d} {r['rate_hz']:>7.1f} {s['mean']:>7.2f} "
              f"{s['p95']:>7.2f} {s['p99']:>7.2f} {s['max']:>8.2f} "
              f"{r['gaps_over_2x_nominal']:>5d}")
    print(f"\nwrote {out}\n      {frames_path}", file=sys.stderr)
    print(f"\nnow run:  python scripts/measure/analyse.py --capture {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
