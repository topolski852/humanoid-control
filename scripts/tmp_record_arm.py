#!/usr/bin/env python3
"""TEMPORARY — record the left arm's joint angles while it is moved BY HAND through a
sequence of known poses, so the device<->URDF mapping can be solved from real data.

Delete this once the arm frame is settled.

WHY: the arm is drawn from raw device angles with no frame correction (there is no trained arm
policy, so there is nothing to correct TO). If the drawing and the physical arm disagree, the
disagreement is the finding — but reading it off a moving picture is hopeless. Holding the arm
at four poses whose URDF angles we can compute turns it into arithmetic: for each joint, fit
`device = a * urdf + b` across the poses. `a` is the gear-ratio sign, `b` is the calibration
zero offset.

READ-ONLY. This never commands the robot. It polls the web server's /api/status, so it does not
touch the CAN bus and cannot conflict with the daemon's telemetry port.

The arm must be IDLE (zero torque, hand-movable). Connect in the web UI first; connect leaves
every joint IDLE, which is what you want.

Usage:
    python scripts/tmp_record_arm.py                    # guided capture
    python scripts/tmp_record_arm.py --url http://otherhost:8000
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path

DEG = 180.0 / math.pi

ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
]
SHORT = ["sh_pitch", "sh_roll", "sh_yaw", "elbow", "wrist"]

# The sequence. Alternating relaxed poses are deliberate: they are the repeatability check.
# If the three RELAXED captures do not agree to a degree or two, the encoders are drifting or
# slipping and every other number here is suspect.
STEPS = [
    ("RELAX-1", "Let the arm hang RELAXED at the side, straight down. Hold still."),
    ("LEFT", "Raise the arm STRAIGHT OUT TO THE LEFT, horizontal, elbow straight. Hold still."),
    ("RELAX-2", "Let it hang RELAXED again. Hold still."),
    ("FORWARD", "Raise the arm STRAIGHT FORWARD, horizontal, elbow straight. Hold still."),
    ("RELAX-3", "Let it hang RELAXED again. Hold still."),
    ("CROSSED", "Bring the arm ACROSS THE FRONT of the robot, as if folding arms. Hold still."),
]


def fetch(url: str, timeout: float = 2.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        body = json.loads(r.read().decode())
    if not body.get("success"):
        raise RuntimeError(body.get("error") or "status call failed")
    return body["data"]


def arm_sample(data: dict) -> tuple[list[float], list[dict]]:
    """(positions in rad, raw joint dicts) for the arm joints, in ARM_JOINTS order."""
    by_name = {j["name"]: j for j in data.get("joints", [])}
    missing = [n for n in ARM_JOINTS if n not in by_name]
    if missing:
        raise RuntimeError(
            "the server is not reporting these arm joints: "
            + ", ".join(missing)
            + "\n  -> set the layout to include the left arm (Settings tab) and reconnect."
        )
    rows = [by_name[n] for n in ARM_JOINTS]
    pos = []
    for j in rows:
        p = j.get("position")
        pos.append(float(p) if isinstance(p, (int, float)) else math.nan)
    return pos, rows


def preflight(url: str) -> None:
    data = fetch(url)
    _, rows = arm_sample(data)
    offline = [SHORT[i] for i, j in enumerate(rows) if not j.get("online")]
    if offline:
        sys.exit(f"ERROR: arm joints offline: {', '.join(offline)}. Power the arm and connect.")
    powered = [SHORT[i] for i, j in enumerate(rows)
               if str(j.get("state", "")).upper() not in ("IDLE", "", "NONE")]
    if powered:
        print(f"  !! these joints are NOT IDLE: {', '.join(powered)}")
        print("     They may resist being moved, or hold position. Stop any session first.")
    faults = [f"{SHORT[i]}=0x{int(j.get('error') or 0):04x}"
              for i, j in enumerate(rows) if j.get("error")]
    if faults:
        print(f"  !! firmware errors present: {', '.join(faults)}")
    print(f"  state={data.get('state')}  layout={(data.get('layout') or {}).get('describe')}")


def fmt(pos: list[float]) -> str:
    return " ".join(f"{SHORT[i]}={pos[i]*DEG:+6.1f}" for i in range(len(pos)))


def status_line(text: str) -> None:
    """Overwrite the current line. Padded, or the tail of a longer previous line survives."""
    sys.stdout.write("\r" + text.ljust(110)[:110])
    sys.stdout.flush()


def clear_line() -> None:
    sys.stdout.write("\r" + " " * 110 + "\r")
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://localhost:8000", help="humanoid-control web server")
    ap.add_argument("--out-dir", default=None, help="where to write the recording")
    ap.add_argument("--rate", type=float, default=25.0, help="poll rate Hz (data updates ~10 Hz)")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds of stillness that counts as 'holding a pose'")
    ap.add_argument("--still-deg", type=float, default=0.8,
                    help="max peak-to-peak movement (deg) across the settle window to count as still")
    ap.add_argument("--move-deg", type=float, default=8.0,
                    help="movement (deg) required after a capture before the next one can arm")
    args = ap.parse_args()

    out_dir = Path(args.out_dir) if args.out_dir else Path(__file__).resolve().parent.parent / "_arm_recording"
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    raw_path = out_dir / f"arm_raw_{stamp}.csv"
    cap_path = out_dir / f"arm_poses_{stamp}.json"

    status_url = args.url.rstrip("/") + "/api/status"
    print("=" * 78)
    print("ARM MOTION RECORDING — read-only, nothing is ever commanded")
    print("=" * 78)
    try:
        preflight(status_url)
    except urllib.error.URLError as exc:
        sys.exit(f"ERROR: cannot reach {status_url}: {exc}\n  -> is the web server running?")
    except RuntimeError as exc:
        sys.exit(f"ERROR: {exc}")

    print(f"  raw log  -> {raw_path}")
    print(f"  captures -> {cap_path}")
    print()
    print("Move the arm by hand. Each step captures automatically once you hold still for")
    print(f"{args.settle:.1f}s. Move at least {args.move_deg:.0f} deg between poses. Ctrl-C to abort.")
    print()

    dt = 1.0 / args.rate
    win = max(3, int(args.settle * args.rate))
    still_thresh = args.still_deg / DEG
    move_thresh = args.move_deg / DEG

    captures: list[dict] = []
    t0 = time.monotonic()

    raw_f = raw_path.open("w", newline="")
    raw = csv.writer(raw_f)
    raw.writerow(["t_s", "step", "phase"] + [f"{s}_rad" for s in SHORT] + [f"{s}_deg" for s in SHORT])

    try:
        for step_name, instruction in STEPS:
            print("-" * 78)
            print(f"[{len(captures)+1}/{len(STEPS)}]  {step_name}")
            print(f"        {instruction}")
            history: deque = deque(maxlen=win)
            # Every step must begin with real movement, so a lingering hold cannot be
            # captured twice. The first step is exempt: the arm is already where it is.
            phase = "settle" if not captures else "move"
            anchor = None
            last_print = 0.0
            while True:
                loop_start = time.monotonic()
                try:
                    data = fetch(status_url)
                    pos, _ = arm_sample(data)
                except (urllib.error.URLError, RuntimeError) as exc:
                    print(f"  (telemetry hiccup: {exc})")
                    time.sleep(0.5)
                    continue

                t = time.monotonic() - t0
                raw.writerow([f"{t:.3f}", step_name, phase]
                             + [f"{v:.6f}" for v in pos] + [f"{v*DEG:.2f}" for v in pos])

                if any(math.isnan(v) for v in pos):
                    time.sleep(dt)
                    continue

                history.append(pos)
                if anchor is None:
                    anchor = pos

                if phase == "move":
                    moved = max(abs(pos[i] - anchor[i]) for i in range(len(pos)))
                    if moved >= move_thresh:
                        phase = "settle"
                        history.clear()
                        clear_line()
                        print(f"        moving ({moved*DEG:.0f} deg) — now hold the pose still")
                    elif t - last_print > 0.4:
                        last_print = t
                        status_line(f"        move the arm...  {fmt(pos)}")
                    time.sleep(max(0.0, dt - (time.monotonic() - loop_start)))
                    continue

                # settle: capture once the whole window is quiet
                if len(history) == history.maxlen:
                    spread = [max(h[i] for h in history) - min(h[i] for h in history)
                              for i in range(len(pos))]
                    if max(spread) <= still_thresh:
                        mean = [sum(h[i] for h in history) / len(history) for i in range(len(pos))]
                        captures.append({
                            "step": step_name,
                            "t_s": round(t, 3),
                            "joints": ARM_JOINTS,
                            "rad": [round(v, 6) for v in mean],
                            "deg": [round(v * DEG, 2) for v in mean],
                            "spread_deg": [round(s * DEG, 2) for s in spread],
                        })
                        clear_line()
                        print(f"        CAPTURED  {fmt(mean)}")
                        print(f"                  (steady to {max(spread)*DEG:.2f} deg)")
                        break
                    if t - last_print > 0.4:
                        last_print = t
                        status_line(f"        hold still ({max(spread)*DEG:4.1f} deg)  {fmt(pos)}")
                time.sleep(max(0.0, dt - (time.monotonic() - loop_start)))
    except KeyboardInterrupt:
        print("\n\nAborted by user — partial data kept.")
    finally:
        raw_f.close()
        cap_path.write_text(json.dumps({
            "_meta": {
                "recorded_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "source": "device frame, raw encoder positions as the daemon reports them",
                "note": "Hand-moved, zero torque. No commands were sent.",
                "raw_csv": str(raw_path),
            },
            "captures": captures,
        }, indent=2) + "\n")

    if captures:
        print()
        print("=" * 78)
        print("CAPTURED POSES (device frame, degrees)")
        print("=" * 78)
        print(f"{'step':10s}" + "".join(f"{s[:11]:>12s}" for s in SHORT))
        for c in captures:
            print(f"{c['step']:10s}" + "".join(f"{v:>12.1f}" for v in c["deg"]))
        rel = [c for c in captures if c["step"].startswith("RELAX")]
        if len(rel) > 1:
            print()
            print("Repeatability of the RELAXED pose (max spread across repeats, deg):")
            spread = [max(r["deg"][i] for r in rel) - min(r["deg"][i] for r in rel)
                      for i in range(len(SHORT))]
            print(f"{'':10s}" + "".join(f"{v:>12.1f}" for v in spread))
            worst = max(spread)
            print(f"  -> worst {worst:.1f} deg. Above ~3 deg means an encoder is drifting or the")
            print("     'relaxed' pose was not actually repeated, and the fit below is unreliable.")
    print()
    print(f"Saved: {cap_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
