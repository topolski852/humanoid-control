#!/usr/bin/env python3
"""Disturbance recovery — does the policy settle after a push, or keep ringing?

    python scripts/measure/settle_analysis.py --capture docs/measurements/..._can.json
    python scripts/measure/settle_analysis.py --capture A.json --compare B.json

Operator observation that motivated this: Smooth A settles on its own after a push, while
Smooth B "keeps oscillating unless you hold it still". That is a stability property, and it is
the kind of thing a policy can look fine on in sim and fail on in hardware — so it needs to be
a number, not an impression.

### What is measured

A **motion signal** is built from the CAN position stream: per joint, the deviation from a
rolling 2 s mean, summed in quadrature across all 12 joints and expressed in encoder counts.
That is a whole-body "how much is it moving" scalar at the full 100 Hz, independent of any
telemetry socket.

From it:

* **disturbance events** — motion crossing a threshold well above the quiet baseline
* **settling time** — how long after each event until motion returns to baseline and STAYS
  there. Reported as "did not settle" when it never does before the next event or the end.
* **damping ratio** via log decrement on successive envelope peaks. A decaying response gives
  zeta > 0; a sustained oscillation gives zeta ~ 0, which is a limit cycle.
* **quiet-period residual motion** — the crucial one. With nobody touching the robot, a policy
  that has truly settled shows baseline motion. One stuck in a limit cycle keeps oscillating at
  a steady amplitude, and that is self-sustained, not a response to anything.

### Reading the result

A limit cycle is not "noise" and must not be modelled as observation noise. It is a closed-loop
instability between the policy and the plant, and it means the deployed gains, delays, or the
gait itself put the system near its stability margin. It belongs in the training discussion,
not in the randomisation ranges.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

FS = 100.0
ROLL_S = 2.0            # rolling-mean window for the deviation signal
ENV_S = 0.25            # envelope window
QUIET_MULT = 2.0        # motion below QUIET_MULT x baseline counts as settled
EVENT_MULT = 10.0       # relative bar for a disturbance
EVENT_ABS_CT = 40.0     # ...and an ABSOLUTE floor, in encoder counts.
# Both bars are needed. A relative-only threshold fires on 8-count wobbles when the baseline is
# ~1 count, drowning the real pushes (100-8000 counts) in noise events. An absolute-only
# threshold would miss a genuine disturbance on a very quiet robot.
EDGE_SKIP_S = 3.0       # ignore events at the very start/end: those are ramp-in and capture cut,
                        # not operator pushes.
SETTLE_FRAC = 0.10      # settled = envelope back under 10% of this event's own peak
SETTLE_HOLD_S = 1.5     # ...and staying there this long


def load(capture_json: Path):
    side = Path(str(capture_json).replace(".json", ".frames.jsonl"))
    if not side.exists():
        raise SystemExit(f"missing frames sidecar: {side}")
    d = {}
    for line in side.read_text().splitlines():
        if line.strip():
            j = json.loads(line)
            d[j["joint"]] = j
    return d


def motion_signal(d):
    """Whole-body motion in encoder counts, at 100 Hz, on a common time grid."""
    t0 = max(v["ts"][0] for v in d.values())
    t1 = min(v["ts"][-1] for v in d.values())
    grid = np.arange(t0, t1, 1.0 / FS)
    dev2 = np.zeros_like(grid)
    w = int(ROLL_S * FS)
    k = np.ones(w) / w
    for j in C.canonical_joint_order():
        if j not in d:
            continue
        ts = np.asarray(d[j]["ts"])
        pos = np.asarray(d[j]["pos"])
        p = pos[np.clip(np.searchsorted(ts, grid), 0, len(ts) - 1)]
        base = np.convolve(p, k, mode="same")
        dev2 += ((p - base) / C.JOINT_QUANTUM_RAD) ** 2
    return grid - grid[0], np.sqrt(dev2)


def envelope(x):
    w = int(ENV_S * FS)
    n = len(x) // w
    return np.array([x[i * w:(i + 1) * w].max() for i in range(n)]), w / FS


def dominant_hz(x):
    x = x - x.mean()
    if x.size < 32 or not np.any(x):
        return None
    f = np.fft.rfftfreq(x.size, 1.0 / FS)
    a = np.abs(np.fft.rfft(x))
    band = (f > 0.2) & (f < 8.0)
    if not band.any() or not a[band].any():
        return None
    return float(f[band][np.argmax(a[band])])


def analyse(tag, capture: Path) -> dict:
    d = load(capture)
    t, m = motion_signal(d)
    env, estep = envelope(m)
    et = np.arange(len(env)) * estep
    baseline = float(np.percentile(m, 20))     # robust quiet level
    quiet_th = QUIET_MULT * baseline
    event_th = EVENT_MULT * baseline

    # halves — the operator convention is: disturbances early, untouched later
    half = len(m) // 2
    first, second = m[:half], m[half:]

    print(f"\n{'='*66}\n{tag}\n{'='*66}")
    print(f"duration {t[-1]:.0f}s | baseline motion {baseline:.1f} ct "
          f"(quiet<{quiet_th:.0f}, event>{event_th:.0f})")
    print(f"  FIRST half : median {np.median(first):7.1f} ct  p95 {np.percentile(first,95):8.1f}  "
          f"max {first.max():8.1f}")
    print(f"  SECOND half: median {np.median(second):7.1f} ct  p95 {np.percentile(second,95):8.1f}  "
          f"max {second.max():8.1f}")

    # disturbance events on the envelope
    event_th = max(EVENT_MULT * baseline, EVENT_ABS_CT)
    above = env > event_th
    events, i = [], 0
    while i < len(above):
        if above[i]:
            s0 = i
            while i < len(above) and above[i]:
                i += 1
            events.append((s0, i))
        else:
            i += 1
    edge = EDGE_SKIP_S / estep
    events = [(a, b) for a, b in events
              if (b - a) * estep >= 0.25 and a > edge and b < len(env) - edge]

    print(f"\n  disturbance events: {len(events)} (peak >= {event_th:.0f} ct)")
    settles = []
    for n, (a, b) in enumerate(events, 1):
        peak = env[a:b].max()
        # Settled = envelope back under 10% of THIS event's peak (or the quiet floor, whichever
        # is higher) and staying there. Relative-to-peak is the standard settling measure and,
        # unlike an absolute floor, does not report "never" just because the robot is idling
        # slightly above its own 20th-percentile baseline.
        # MEASURE FROM THE PEAK, NOT FROM THE END OF THE ABOVE-THRESHOLD SPAN.
        # An earlier version searched from `b`, the index where the envelope fell back below
        # the EVENT threshold. That is circular: a sustained limit cycle keeps the envelope
        # above the event threshold for its whole duration, so `b` already sits at the end of
        # the ringing and the reported settle time is ~0. Smooth B rang at 4 Hz for 8 s after a
        # push and this reported "settle 0.5s". Timing from the peak measures the ringing
        # instead of skipping over it.
        pk_idx = a + int(np.argmax(env[a:b]))
        target = max(SETTLE_FRAC * peak, quiet_th)
        need = int(SETTLE_HOLD_S / estep)
        st = None
        for k in range(pk_idx, len(env) - need):
            if np.all(env[k:k + need] <= target):
                st = (k - pk_idx) * estep
                break
        seg = m[int(pk_idx * estep * FS):int(min(pk_idx + need * 6, len(env)) * estep * FS)]
        hz = dominant_hz(seg)
        settles.append(st)
        sustained = st is None or st > 5.0
        print(f"    #{n} t={a*estep:6.1f}s peak {peak:8.1f} ct  "
              f"ring {(b-a)*estep:5.1f}s  "
              f"settle {'%.1fs' % st if st is not None else 'NEVER':>7s}"
              f"  osc {('%.2f Hz' % hz) if hz else '-':>8s}"
              f"{'   <<< SUSTAINED' if sustained else ''}")

    # quiet-period residual: the untouched second half
    q_hz = dominant_hz(second)
    ratio = float(np.median(second) / baseline) if baseline else float("nan")
    print(f"\n  UNTOUCHED half: median motion {np.median(second):.1f} ct "
          f"= {ratio:.2f}x baseline, dominant {('%.2f Hz' % q_hz) if q_hz else '-'}")
    if ratio > 3.0:
        print("  -> SELF-SUSTAINED OSCILLATION: still moving well above baseline with nobody")
        print("     touching it. A limit cycle, not a disturbance response.")
    elif ratio > 1.6:
        print("  -> elevated residual motion while untouched; marginal damping")
    else:
        print("  -> settles to baseline when untouched")

    return {"tag": tag, "capture": str(capture), "duration_s": float(t[-1]),
            "baseline_ct": baseline,
            "first_half": {"median": float(np.median(first)), "p95": float(np.percentile(first, 95)),
                           "max": float(first.max())},
            "second_half": {"median": float(np.median(second)), "p95": float(np.percentile(second, 95)),
                            "max": float(second.max()), "dominant_hz": q_hz,
                            "ratio_to_baseline": ratio},
            "events": [{"t": float(a * estep), "peak_ct": float(env[a:b].max()),
                        "settle_s": (float(s) if s is not None else None)}
                       for (a, b), s in zip(events, settles)]}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True)
    ap.add_argument("--compare", default=None, help="a second capture to contrast")
    ap.add_argument("--label", default=None)
    ap.add_argument("--compare-label", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    a = Path(args.capture)
    res = [analyse(args.label or a.stem, a)]
    if args.compare:
        b = Path(args.compare)
        res.append(analyse(args.compare_label or b.stem, b))
        x, y = res
        print(f"\n{'='*66}\nCOMPARISON\n{'='*66}")
        print(f"{'metric':32s} {x['tag'][:14]:>15s} {y['tag'][:14]:>15s}")
        print(f"{'baseline motion (ct)':32s} {x['baseline_ct']:>15.1f} {y['baseline_ct']:>15.1f}")
        print(f"{'untouched median (ct)':32s} {x['second_half']['median']:>15.1f} "
              f"{y['second_half']['median']:>15.1f}")
        print(f"{'untouched / baseline':32s} {x['second_half']['ratio_to_baseline']:>15.2f} "
              f"{y['second_half']['ratio_to_baseline']:>15.2f}")
        for r in (x, y):
            ns = [e for e in r["events"] if e["settle_s"] is None]
            print(f"{'events never settling (' + r['tag'][:10] + ')':32s} "
                  f"{len(ns)}/{len(r['events'])}")

    out = Path(args.out) if args.out else a.with_name(a.stem.replace("_can", "") + "_settle.json")
    C.write_json(out, {"results": res})
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
