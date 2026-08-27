#!/usr/bin/env python3
"""Offline checks for arm calibration profiles and the human→robot mapping.

No robot, no headset. Run::

    .venv/bin/python scripts/test_arm_profile.py

The mapping is what decides whether the robot matches the operator, and its two jobs are
easy to state and easy to get wrong:

  1. Remove the tracker's SYSTEMATIC offset. Measured on real hardware, a genuinely straight
     arm reads as 21.6 deg of elbow flexion — drive from raw angles and the robot sits
     permanently bent.
  2. Fit the operator's range onto the robot's much tighter one, without moving the zero.
"""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.arm_kinematics import ArmChain          # noqa: E402
from humanoid_control.arm_profile import (                     # noqa: E402
    JOINTS, ArmProfile, default_profile_path, delete, load, load_all, save)
from humanoid_control.layout import LIMB_JOINTS                # noqa: E402

PASS, FAIL = [], []
D = np.radians


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


CHAIN = ArmChain(list(LIMB_JOINTS["left_arm"]))


def capture(relaxed, others):
    """Build a CalibrationRun-style capture dict."""
    out = {"relaxed": {"angles": list(relaxed), "upper_len": 0.26, "fore_len": 0.26}}
    for k, v in others.items():
        out[k] = {"angles": list(v), "upper_len": 0.26, "fore_len": 0.26}
    return out


def main() -> int:
    print("\n── the zero removes the tracker's systematic offset ─────────────")
    # The real measurement: a straight arm reads 21.6 deg of elbow bend.
    cap = capture([D(-4), D(4), D(-6), D(21.6), D(10)],
                  {"tpose": [D(0), D(83), D(-32), D(21.6), D(10)],
                   "forward": [D(80), D(5), D(-10), D(24), D(15)],
                   "elbow90": [D(-2), D(3), D(-5), D(105), D(12)],
                   "reach_up": [D(70), D(40), D(-20), D(30), D(20)]})
    p = ArmProfile.from_capture(cap)

    check("zero comes from the RELAXED pose, not the mean",
          abs(np.degrees(p.zero_rad[3]) - 21.6) < 0.01,
          f"elbow zero {np.degrees(p.zero_rad[3]):.1f}°")

    # The whole point: relaxed input must map to the robot's rest pose, NOT 21.6 deg of bend.
    out = p.to_robot(np.array(cap["relaxed"]["angles"]), CHAIN)
    check("a relaxed arm maps to the robot's zero (offset removed)",
          np.allclose(np.degrees(out), 0, atol=0.5), f"{np.degrees(out).round(2)}")
    check("elbow specifically is not left bent",
          abs(np.degrees(out[3])) < 0.5, f"{np.degrees(out[3]):.2f}°")

    print("\n── range is mapped onto the robot's limits ─────────────────────")
    lo, hi = np.degrees(CHAIN.limits_lower), np.degrees(CHAIN.limits_upper)
    for key in ("tpose", "forward", "elbow90", "reach_up"):
        r = np.degrees(p.to_robot(np.array(cap[key]["angles"]), CHAIN))
        inside = bool(np.all(r >= lo - 1e-6) and np.all(r <= hi + 1e-6))
        check(f"{key:<9} maps inside every joint limit", inside, str(r.round(1)))

    print("\n── each DOF moves its OWN joint (this is what 'mirroring' means) ─")
    base = np.array(cap["relaxed"]["angles"])
    for i, jn in enumerate(JOINTS):
        moved = base.copy()
        moved[i] += D(25)
        r0 = p.to_robot(base, CHAIN)
        r1 = p.to_robot(moved, CHAIN)
        d = np.degrees(np.abs(r1 - r0))
        others = np.delete(d, i)
        check(f"moving {jn:<15} moves only {jn}",
              d[i] > 1.0 and float(others.max()) < 1e-6,
              f"target {d[i]:.1f}°, worst other {others.max():.3f}°")

    print("\n── guards ──────────────────────────────────────────────────────")
    # A sweep where the operator barely moved must not become an enormous gain.
    tiny = capture([0, 0, 0, 0, 0], {"a": [D(1), D(1), D(1), D(1), D(1)]})
    pt = ArmProfile.from_capture(tiny)
    r = pt.to_robot(np.array([D(10), 0, 0, 0, 0]), CHAIN)
    check("a tiny measured range does not produce a huge gain",
          abs(np.degrees(r[0])) <= 10 * 3.0 + 1e-6, f"{np.degrees(r[0]):.1f}° for 10° input")

    far = p.to_robot(np.array([D(400), D(400), D(400), D(400), D(400)]), CHAIN)
    check("absurd input clamps to the limits rather than wrapping",
          bool(np.all(far >= CHAIN.limits_lower - 1e-9)
               and np.all(far <= CHAIN.limits_upper + 1e-9)))

    print("\n── persistence ─────────────────────────────────────────────────")
    with tempfile.TemporaryDirectory() as td:
        os.environ["HUMANOID_ARM_PROFILES"] = str(Path(td) / "p.json")
        check("no profile before saving", load() is None)
        save(p)
        back = load()
        check("round-trips through disk", back is not None)
        check("zero survives the round trip",
              np.allclose(back.zero_rad, p.zero_rad, atol=1e-9))
        check("mapping is identical after reload",
              np.allclose(back.to_robot(base, CHAIN), p.to_robot(base, CHAIN)))
        p2 = ArmProfile.from_capture(cap, name="someone-else")
        save(p2)
        check("profiles are keyed by name (multi-operator ready)",
              set(load_all()) == {"default", "someone-else"}, str(sorted(load_all())))
        check("delete removes only the named profile",
              delete("someone-else") and set(load_all()) == {"default"})
        check("deleting a missing profile is False, not an error", delete("nope") is False)
        os.environ.pop("HUMANOID_ARM_PROFILES")

    print("\n── malformed input ─────────────────────────────────────────────")
    try:
        ArmProfile.from_capture({})
        check("an empty capture is rejected", False, "no error raised")
    except ValueError:
        check("an empty capture is rejected", True)
    short = ArmProfile.from_dict("x", {"zero_rad": [0.1]})
    check("a truncated stored profile is padded, not fatal",
          len(short.zero_rad) == len(JOINTS))

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
