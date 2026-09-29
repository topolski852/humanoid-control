#!/usr/bin/env python3
"""Log ESC bus voltage and q-axis current per joint, passively, during a session.

    python scripts/measure/power_log.py --seconds 90 --label rug-walk

Written 2026-09-29: the walk degraded across the afternoon on a constant bench supply (stall
fraction ~25% -> 54%) while the stand stayed identical. If walking peaks exceed the supply's
current limit, the supply falls into constant-current mode and the bus sags only during steps,
which a reading at rest cannot show. This shows it.

PASSIVE: runs candump and decodes the daemon's own slow-poll SDO replies (func 0xB, byte0 0x43):
param 0x100 = bus voltage, 0x048 = torque measured, and I_q from its param id below. It sends
nothing and opens no daemon socket. About 3 samples per second per node.
"""
from __future__ import annotations

import argparse
import re
import struct
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

LINE = re.compile(r"\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s*([0-9A-Fa-f ]*)")
P_BUS_V, P_TORQUE = 0x100, 0x048


def iq_param_id() -> int | None:
    hdr = Path(__file__).resolve().parents[2] / "daemon/src/motor/recoil_protocol.hpp"
    m = re.search(r"PARAM_CURRENT_CONTROLLER_I_Q_MEASURED\s*=\s*(0x[0-9A-Fa-f]+)", hdr.read_text())
    return int(m.group(1), 16) if m else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--label", default="session")
    args = ap.parse_args()
    p_iq = iq_param_id()
    nodes = C.leg_node_map()
    chans = C.leg_channels()
    proc = subprocess.Popen(["candump", "-t", "a"] + chans, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, bufsize=1)
    series = defaultdict(lambda: {"t": [], "v": [], "iq": [], "t_iq": []})
    t_end = time.time() + args.seconds
    print(f"logging bus V / I_q for {args.seconds:.0f}s (passive) ...", flush=True)
    try:
        for line in proc.stdout:
            if time.time() >= t_end:
                break
            m = LINE.search(line)
            if not m:
                continue
            arb = int(m.group(3), 16)
            if (arb >> C.FUNC_SHIFT) & C.FUNC_MASK != 0xB:
                continue
            raw = bytes.fromhex(m.group(5).replace(" ", ""))
            if len(raw) != 8 or raw[0] != 0x43:
                continue
            name = nodes.get((m.group(2), arb & C.NODE_MASK))
            if not name:
                continue
            pid = struct.unpack("<H", raw[1:3])[0]
            val = struct.unpack("<f", raw[4:8])[0]
            t = float(m.group(1))
            if pid == P_BUS_V:
                series[name]["t"].append(t); series[name]["v"].append(val)
            elif p_iq is not None and pid == p_iq:
                series[name]["t_iq"].append(t); series[name]["iq"].append(val)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    allv = np.concatenate([np.asarray(s["v"]) for s in series.values() if s["v"]]) if series else np.array([])
    summary = {}
    print(f"\n{'joint':26s} {'V min':>6s} {'V p5':>6s} {'V med':>6s} {'|Iq| p95':>8s} {'n':>4s}")
    for j in sorted(series):
        s = series[j]
        v = np.asarray(s["v"]); iq = np.abs(np.asarray(s["iq"]))
        summary[j] = {"v_min": float(v.min()) if len(v) else None,
                      "v_p5": float(np.percentile(v, 5)) if len(v) else None,
                      "v_median": float(np.median(v)) if len(v) else None,
                      "iq_abs_p95": float(np.percentile(iq, 95)) if len(iq) else None, "n": len(v)}
        f = lambda x: f"{x:6.2f}" if x is not None else "   -  "  # noqa: E731
        print(f"{j:26s} {f(summary[j]['v_min'])} {f(summary[j]['v_p5'])} {f(summary[j]['v_median'])} "
              f"{f(summary[j]['iq_abs_p95']):>8s} {summary[j]['n']:>4d}")
    if len(allv):
        print(f"\nALL: bus V min {allv.min():.2f}  p5 {np.percentile(allv, 5):.2f}  median "
              f"{np.median(allv):.2f}  -> sag {np.median(allv) - allv.min():.2f} V")
    out = C.default_outdir() / f"power_log_{time.strftime('%Y%m%dT%H%M%S')}_{args.label}.json"
    C.write_json(out, {"_meta": C.finish_meta(C.capture_meta("power", measurement="bus_voltage_current",
                                                              label=args.label, iq_param=p_iq)),
                       "summary": summary,
                       "series": {j: {k: list(map(float, v)) for k, v in s.items()} for j, s in series.items()}})
    print(f"wrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
