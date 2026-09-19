#!/usr/bin/env python3
"""Record the joint trajectory while an arm is moved BY HAND between two poses.

WHY: a static pose tells you where the joints ended up; it cannot tell you which joint
actually carried a motion. On 2026-09-19 the arms showed shoulder_pitch swinging ~90 deg
between T-pose and relaxed, when that transition should be almost pure shoulder_roll — and no
single zero offset could make both poses geometrically correct. Watching the move decides
whether pitch genuinely rotates (so the reference pose is wrong) or whether a motion that is
physically roll is being reported as pitch (so an axis/frame is wrong).

READ-ONLY. Polls the web server's /api/status; never commands, never opens a CAN socket.
The arm must be IDLE (hand-movable) — connect in the web UI first.

Usage:
    python scripts/record_transition.py --out run.json          # Ctrl-C to stop
"""
from __future__ import annotations

import argparse, json, math, os, sys, time, urllib.request

DEG = 180.0 / math.pi
KS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "wrist_yaw")


def fetch(url, timeout=2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        body = json.loads(r.read().decode())
    if not body.get("success"):
        raise RuntimeError(body.get("error") or "status call failed")
    return body["data"]


def jmap(data):
    js = data.get("joints") or {}
    return js if isinstance(js, dict) else {j.get("name"): j for j in js}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8000")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--move-deg", type=float, default=2.0,
                    help="per-joint movement (deg) before a sample counts as 'moving'")
    args = ap.parse_args()

    status = f"{args.url.rstrip('/')}/api/status"
    d = fetch(status)
    if d.get("state") != "CONNECTED":
        print(f"state is {d.get('state')} — connect first.", file=sys.stderr)
        return 2

    names = [n for n in jmap(d) if n.endswith("_joint")]
    samples = []
    t0 = time.time()
    period = 1.0 / max(args.hz, 1.0)
    print(f"recording {len(names)} joints at {args.hz:.0f} Hz — move the arms, Ctrl-C to stop")
    try:
        while True:
            tick = time.time()
            try:
                d = fetch(status)
            except Exception:
                time.sleep(period); continue
            jm = jmap(d)
            samples.append({
                "t": round(tick - t0, 4),
                "q": {n: round((jm.get(n, {}).get("position") or 0.0) * DEG, 4) for n in names},
            })
            if len(samples) % 60 == 0:
                tmp = args.out + ".tmp"
                with open(tmp, "w") as f:
                    json.dump({"names": names, "samples": samples}, f)
                os.replace(tmp, args.out)
                first, last = samples[0]["q"], samples[-1]["q"]
                moved = {n: last[n] - first[n] for n in names}
                top = sorted(moved.items(), key=lambda kv: -abs(kv[1]))[:3]
                print(f"  {samples[-1]['t']:6.1f}s  {len(samples):5d} samples   biggest so far: "
                      + ", ".join(f"{n.replace('_joint','')} {v:+.1f}" for n, v in top))
            time.sleep(max(0.0, period - (time.time() - tick)))
    except KeyboardInterrupt:
        pass

    with open(args.out, "w") as f:
        json.dump({"names": names, "samples": samples}, f)
    print(f"\n{len(samples)} samples over {samples[-1]['t']:.1f}s -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
