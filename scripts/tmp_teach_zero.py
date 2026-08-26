#!/usr/bin/env python3
"""TEMPORARY — teach the arm's zero from a KNOWN HELD POSE, since the arm has no hardstops.

Delete this once the arm frame is settled.

WHY NOT THE CALIBRATION TAB: humanoid_control/calibration.py computes
``offset = lower_pos - min_rad`` from two mechanical hardstop captures. This arm has no
hardstops, so that flow has nothing to capture. Instead, hold the arm in a pose whose URDF
angles are computable and solve the offset directly:

    displayed = raw - position_offset          (firmware works in RAW; see actuator.cpp,
                                                which writes position_limit +- position_offset)

    to move the displayed angle by  delta = urdf_expected - displayed_now
    the offset must move the OTHER way:   new_offset = old_offset - delta

SAFETY: writes ``position_offset`` only — never gear, gains, or limits. Nothing is commanded
to move. Dry-run unless you pass --apply, and every write is read back and verified.

Usage:
    python scripts/tmp_teach_zero.py                 # measure + show, write nothing
    python scripts/tmp_teach_zero.py --apply         # write the offsets
    python scripts/tmp_teach_zero.py --pose relaxed  # use the hanging pose instead
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
import urllib.request
from pathlib import Path

DEG = 180.0 / math.pi
DAEMON = ("127.0.0.1", 9001)

# Poses whose URDF angles are computable. Only joints listed here are taught; anything absent
# is left alone, because guessing a zero is worse than leaving a known-bad one visible.
#
# `side` is the reference pose: the arm horizontal out to the side is easy to hit accurately
# against any level edge, and it pins three joints at once. The two axial joints
# (shoulder_yaw, wrist_yaw) are deliberately NOT taught — a straight arm's pose says nothing
# about rotation ABOUT the arm, so there is no expected value to solve against.
POSES = {
    # THE reference pose. A T-pose is the canonical zero in most rigs, it is the easiest pose
    # to hold accurately (a level edge gives you horizontal), and it defines all five joints in
    # one hold.
    #
    # Three of the five are MEASURED — pitch, roll and elbow are fixed by geometry, and roll's
    # +74.8 is where the wrist's height equals the shoulder's (NOT +90: the URDF's roll zero
    # sits ~23 deg out from vertical, and the roll axis is offset ~5 cm from the pitch axis, so
    # joint angle and visual elevation are not 1:1).
    #
    # Two are DECLARED — shoulder_yaw and wrist_yaw are inline twists, and a straight arm gives
    # no geometric constraint on rotation ABOUT the arm. Zero here simply means "in the T-pose
    # the arm is not twisted", which is what a reference pose is for. Note the URDF's own zero
    # for shoulder_yaw sits ~15 deg from this; that costs ~15 deg of drawn forearm-plane
    # accuracy once the elbow bends, and nothing else.
    "tpose": {
        "help": "T-POSE: arm STRAIGHT OUT TO THE SIDE, horizontal, elbow straight,\n"
                "        forearm untwisted, claw in its neutral orientation.\n"
                "        Use a level edge (table, door frame) to judge horizontal.",
        "expect_deg": {
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 74.8,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
        },
        "declared": ("left_shoulder_yaw_joint", "left_wrist_yaw_joint"),
    },
    "side": {
        "help": "Hold the arm STRAIGHT OUT TO THE SIDE, horizontal, elbow straight.\n"
                "        Like 'tpose' but leaves the two twist joints alone.",
        "expect_deg": {
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 74.8,
            "left_elbow_pitch_joint": 0.0,
        },
    },
    "forward": {
        "help": "Hold the arm STRAIGHT FORWARD, horizontal, elbow straight.",
        "expect_deg": {
            "left_shoulder_pitch_joint": -90.0,
            "left_elbow_pitch_joint": 0.0,
        },
    },
    "relaxed": {
        "help": "Let the arm hang RELAXED, straight down at the side, elbow straight.\n"
                "        Less precise than 'side' — a hanging arm rests a few degrees out.",
        "expect_deg": {
            "left_shoulder_pitch_joint": 0.0,
            "left_elbow_pitch_joint": 0.0,
        },
    },
    # The ONLY pose that pins an inline twist. A straight arm says nothing about rotation about
    # the arm, so shoulder_yaw needs the elbow bent: the forearm then acts as the pointer.
    # In the URDF, hanging with the elbow at 90 deg, yaw = +15 puts the forearm dead forward
    # (yaw = -75 points it outward, +105 across the body) — so the human-natural "elbow bends
    # forward" build corresponds to +15, not 0.
    "yaw": {
        "help": "Let the arm hang at your side, BEND THE ELBOW 90 deg, and point the\n"
                "        forearm STRAIGHT FORWARD (the natural human rest position).\n"
                "        This is the only pose that can measure shoulder_yaw.",
        "expect_deg": {
            "left_shoulder_yaw_joint": 0.0,
        },
        # Not taught — reported as an independent cross-check of the 'side' calibration, which
        # knew nothing about this pose.
        "check_deg": {
            "left_shoulder_pitch_joint": (-12.0, 12.0),
            "left_shoulder_roll_joint": (-20.0, 0.0),
            "left_elbow_pitch_joint": (75.0, 105.0),
        },
    },
    # Same easy-to-hold pose as 'side', but with the elbow bent so the forearm becomes a
    # pointer for the twist. yaw = +15 is forearm-dead-forward here just as it is hanging.
    #
    # NOTE ON yaw = 0: the URDF's own geometry puts forearm-dead-forward at about +12 to +15,
    # not 0. Zero is the OPERATOR'S chosen convention ("forearm forward is neutral"), which
    # costs ~15 deg of drawn forearm-plane accuracy and nothing else. To switch to the
    # URDF-exact value, change these two entries to 15.0 and re-teach.
    #
    # wrist_yaw is DECLARED, not measured: the URDF models the hand as one fixed link with no
    # gripper, so no geometry says which claw orientation is zero. Teaching 0 here simply
    # defines "however the claw sits in this pose" as zero. That is a legitimate convention —
    # it is just worth knowing it is a choice rather than a measurement.
    "side_bent": {
        "help": "Hold the arm STRAIGHT OUT TO THE SIDE (as before), then BEND THE ELBOW\n"
                "        90 deg with the forearm pointing STRAIGHT FORWARD.\n"
                "        Keep the claw however you consider its neutral orientation.",
        "expect_deg": {
            "left_shoulder_yaw_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
        },
        "check_deg": {
            "left_shoulder_pitch_joint": (-12.0, 12.0),
            "left_shoulder_roll_joint": (60.0, 90.0),
            "left_elbow_pitch_joint": (75.0, 105.0),
        },
    },
}


def daemon_cmd(msg: dict, timeout: float = 30.0) -> dict:
    """Raw UDP to the daemon's command port. Does NOT bind the telemetry port, so this can run
    alongside the web server."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(msg).encode(), DAEMON)
        return json.loads(s.recvfrom(65535)[0].decode())
    finally:
        s.close()


def live_positions(url: str) -> dict[str, float]:
    with urllib.request.urlopen(url.rstrip("/") + "/api/status", timeout=5) as r:
        body = json.loads(r.read().decode())
    if not body.get("success"):
        raise SystemExit(f"status failed: {body.get('error')}")
    out = {}
    for j in body["data"]["joints"]:
        p = j.get("position")
        if isinstance(p, (int, float)):
            out[j["name"]] = float(p)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="tpose", choices=sorted(POSES))
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--apply", action="store_true", help="actually write the offsets")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds to average the held pose over")
    args = ap.parse_args()

    pose = POSES[args.pose]
    print("=" * 76)
    print(f"TEACH ZERO — pose '{args.pose}'   ({'APPLY' if args.apply else 'DRY RUN'})")
    print("=" * 76)
    print(f"        {pose['help']}")
    print()
    print(f"Averaging for {args.settle:.1f}s — hold it steady...")

    samples: list[dict[str, float]] = []
    t_end = time.monotonic() + args.settle
    while time.monotonic() < t_end:
        samples.append(live_positions(args.url))
        time.sleep(0.05)
    if not samples:
        raise SystemExit("no telemetry")
    names = sorted(set().union(*[set(s) for s in samples]))
    held = {n: sum(s[n] for s in samples if n in s) / max(1, sum(1 for s in samples if n in s))
            for n in names}
    spread = {n: (max(s[n] for s in samples if n in s) - min(s[n] for s in samples if n in s))
              for n in names}
    worst = max(spread.values()) * DEG
    print(f"  steady to {worst:.2f} deg over {len(samples)} samples"
          + ("   <-- MOVING, hold stiller" if worst > 2.0 else ""))
    print()

    # Cross-check joints this pose does NOT teach. Because the pose is different from the one
    # they were calibrated at, agreement here is real evidence rather than arithmetic.
    checks = pose.get("check_deg") or {}
    if checks:
        print("Cross-check of joints already taught (this pose was not used to set them):")
        for joint, (lo, hi) in checks.items():
            if joint not in held:
                continue
            v = held[joint] * DEG
            ok = lo <= v <= hi
            print(f"  {joint:28s} {v:+7.1f} deg   expected {lo:+.0f}..{hi:+.0f}"
                  f"   [{'OK' if ok else 'OUT OF RANGE'}]")
        print()

    plan = []
    for joint, want_deg in pose["expect_deg"].items():
        if joint not in held:
            print(f"  {joint}: not reported — skipped")
            continue
        cfg = daemon_cmd({"type": "READ_CONFIG", "joint_name": joint}).get("config", {})
        old = cfg.get("position_offset")
        if old is None:
            print(f"  {joint}: could not read position_offset — skipped")
            continue
        now_deg = held[joint] * DEG
        delta = (want_deg - now_deg) / DEG          # radians the display must move
        plan.append({
            "joint": joint, "old_offset": float(old), "new_offset": float(old) - delta,
            "now_deg": now_deg, "want_deg": want_deg, "delta_deg": delta * DEG,
        })

    if not plan:
        raise SystemExit("nothing to teach")

    declared = set(pose.get("declared") or ())
    print(f"{'joint':28s}{'reads':>9s}{'should be':>11s}{'shift':>9s}{'offset: old -> new':>26s}  source")
    for p in plan:
        tag = "declared" if p["joint"] in declared else "measured"
        print(f"{p['joint']:28s}{p['now_deg']:+9.1f}{p['want_deg']:+11.1f}{p['delta_deg']:+9.1f}"
              f"{p['old_offset']:+13.4f} ->{p['new_offset']:+9.4f}  {tag}")
    if declared:
        print()
        print("  'declared' = an inline twist with no geometric constraint in this pose; zero")
        print("  here DEFINES untwisted rather than measuring it. See the notes in POSES.")
    print()

    if not args.apply:
        print("DRY RUN — nothing written. Re-run with --apply to commit.")
        return 0

    print("Writing position_offset (nothing else) ...")
    ok = True
    for p in plan:
        try:
            r = daemon_cmd({"type": "APPLY_CONFIG", "joint_name": p["joint"],
                            "config": {"position_offset": p["new_offset"]}}, timeout=25.0)
            if r.get("type") != "ACK":
                print(f"  {p['joint']}: REFUSED {r}")
                ok = False
                continue
        except Exception as exc:
            print(f"  {p['joint']}: write failed: {exc}")
            ok = False
            continue
        print(f"  {p['joint']}: ok")
    time.sleep(0.6)

    # Verify by reading the joint back. If a sign convention were wrong the angle would move
    # AWAY from target, and we would rather say so than leave a confidently wrong zero.
    print()
    print("Verifying...")
    after = live_positions(args.url)
    for p in plan:
        got = after.get(p["joint"])
        if got is None:
            print(f"  {p['joint']}: no readback")
            ok = False
            continue
        err = got * DEG - p["want_deg"]
        verdict = "OK" if abs(err) < 3.0 else "OFF"
        if abs(err) >= 3.0:
            ok = False
        print(f"  {p['joint']:28s} now {got*DEG:+7.1f} deg, wanted {p['want_deg']:+7.1f}"
              f"  err {err:+6.1f}  [{verdict}]")
    print()
    if ok:
        print("All taught joints landed on target. Refresh the browser and check the drawing.")
        print("NOTE: position_offset is lost on power-down — re-run this after every power cycle,")
        print("      or copy the values into the robot config to make them the startup default.")
    else:
        print("SOMETHING DID NOT LAND. The offsets written are recorded above; re-run the dry")
        print("run to see the current state before changing anything else.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
