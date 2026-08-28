#!/usr/bin/env python3
"""Does the robot arm move the way the operator moved? Direction, not magnitude.

    .venv/bin/python scripts/test_arm_direction.py

WHY THIS IS A SEPARATE SUITE. `test_arm_mirror.py` already proves the acceptance property
that mirroring exists for — move one joint, one joint moves — and it passed comfortably
while `shoulder_pitch` was driving the robot BACKWARD when the operator reached FORWARD.
It had to: it feeds joint targets straight in and checks that the right joint tracks them.
A sign error preserves "one joint in, one joint out" perfectly. So does the noise floor,
the leash, the rate limit, and every other property that suite checks.

The missing question was never "how much" — it was "which way", and answering it requires
leaving joint space entirely:

    operator body pose  ->  human_angles  ->  to_robot  ->  FORWARD KINEMATICS  ->  hand xyz

and then asserting the hand went the same direction the operator's hand did. FK is what
makes the test independent of the convention under test; comparing angles to angles would
just restate whichever sign convention the code already believes in.

Frames, since three meet here and mixing them up is how the bug got in:
  * WebXR      +x right, +y up, -z forward   (what the headset reports)
  * robot      +x forward, +y left, +z up    (webxr_to_robot converts)
  * the robot's shoulder_pitch is NEGATIVE forward, which is the specific fact that
    disagreed with the anatomical decomposition and shipped as an inverted joint.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.arm_kinematics import ArmChain          # noqa: E402
from humanoid_control.arm_profile import ArmProfile, JOINTS    # noqa: E402
from humanoid_control.arm_retarget import human_angles         # noqa: E402
from humanoid_control.layout import LIMB_JOINTS                # noqa: E402
from humanoid_control.web.xr import webxr_to_robot             # noqa: E402

PASS, FAIL = [], []
CHAIN = ArmChain(list(LIMB_JOINTS["left_arm"]))


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


def wide_profile() -> ArmProfile:
    """A deliberately generous identity-ish profile.

    A real capture would clamp these poses to its own measured range and hide the sign under
    a saturation. The point here is the DIRECTION of travel, so the mapping is given enough
    room that the answer cannot be an artefact of the operator's calibrated limits.
    """
    n = len(JOINTS)
    return ArmProfile(name="test", zero_rad=[0.0] * n,
                      lo_rad=[-math.pi / 2] * n, hi_rad=[math.pi / 2] * n,
                      upper_len_m=0.26, fore_len_m=0.22)


def body(elbow: list[float], wrist: list[float]) -> dict:
    """One body frame, WebXR coordinates, with the left arm placed by offsets from the
    shoulder. Everything else is a plausible standing torso."""
    sh = np.array([-0.20, 1.42, 0.00])
    e, w = np.array(elbow, float), np.array(wrist, float)
    joints = {
        "hips": [0.0, 0.95, 0.0],
        "chest": [0.0, 1.35, 0.0],
        "left-shoulder": [-0.17, 1.45, 0.0],
        "right-shoulder": [0.17, 1.45, 0.0],
        "left-arm-upper": sh,
        "left-arm-lower": sh + e,
        "left-hand-wrist-twist": sh + e + w * 0.9,
        "left-hand-wrist": sh + e + w,
    }
    return {n: {"p": webxr_to_robot(np.asarray(p, float)).tolist(), "e": False}
            for n, p in joints.items()}


def retarget(elbow: list[float], wrist: list[float]):
    """Operator pose -> (robot joint vector, robot hand position)."""
    arm = human_angles(body(elbow, wrist), side="left")
    assert arm is not None, "the fixture body must decompose"
    q = wide_profile().to_robot(arm.as_array(), CHAIN)
    return q, np.asarray(CHAIN.tool(q)).reshape(-1)[:3]


def main() -> int:
    prof = wide_profile()

    print("\n── the regression: forward is forward ──────────────────────────")
    # WebXR: -z is forward, -y is down. A straight arm swung about the shoulder by `deg`.
    #
    # MODERATE angles on purpose. A full horizontal reach pins shoulder_pitch against its
    # -90 deg limit and drags roll along with it, and a saturated joint answers "which way"
    # with a clamp rather than with the mapping — the hand then lands near the origin and the
    # FK assertions below become meaningless. 40 deg leaves every joint in its linear region.
    def swing(deg: float) -> tuple[list[float], list[float]]:
        a = math.radians(deg)
        u = np.array([0.0, -math.cos(a), -math.sin(a)])   # down at 0, forward as deg grows
        return list(u * 0.26), list(u * 0.22)

    down_e, down_w = swing(0.0)
    fwd_e, fwd_w = swing(40.0)
    back_e, back_w = swing(-40.0)
    down = down_e

    q_down, _ = retarget(down_e, down_w)
    q_fwd, _ = retarget(fwd_e, fwd_w)
    q_back, _ = retarget(back_e, back_w)
    i = CHAIN.joint_names.index("left_shoulder_pitch_joint")

    check("reaching FORWARD gives negative shoulder_pitch",
          q_fwd[i] < q_down[i] and q_fwd[i] < 0,
          f"down {math.degrees(q_down[i]):+.1f}° -> forward {math.degrees(q_fwd[i]):+.1f}°")
    check("reaching BACKWARD gives positive shoulder_pitch",
          q_back[i] > q_down[i],
          f"down {math.degrees(q_down[i]):+.1f}° -> back {math.degrees(q_back[i]):+.1f}°")
    check("forward and backward land on OPPOSITE sides of hanging",
          (q_fwd[i] - q_down[i]) * (q_back[i] - q_down[i]) < 0)

    print("\n── the same question asked of the HAND, via FK ─────────────────")
    # Independent of any joint-sign convention: put the arm forward and the robot's hand
    # must end up further forward (+x) than it was hanging. This is the assertion that would
    # have caught the shipped bug, and the only one here that cannot be fooled by agreeing
    # with whatever sign the code already uses.
    x_down = np.asarray(CHAIN.tool(q_down)).reshape(-1)[0]
    x_fwd = np.asarray(CHAIN.tool(q_fwd)).reshape(-1)[0]
    x_back = np.asarray(CHAIN.tool(q_back)).reshape(-1)[0]
    check("operator forward -> robot hand moves FORWARD (+x)",
          x_fwd > x_down, f"x {x_down:+.3f} -> {x_fwd:+.3f}")
    check("operator backward -> robot hand moves BACKWARD (-x)",
          x_back < x_fwd, f"x {x_back:+.3f} vs forward {x_fwd:+.3f}")

    print("\n── roll: abduction takes the arm OUTWARD ───────────────────────")
    # Left arm: away from the body is +y in the robot frame.
    q_side, p_side = retarget([-0.26, 0.0, 0.0], [-0.22, 0.0, 0.0])
    _, p_down = retarget(down_e, down_w)
    j = CHAIN.joint_names.index("left_shoulder_roll_joint")
    check("arm out to the side increases shoulder_roll",
          q_side[j] > q_down[j],
          f"{math.degrees(q_down[j]):+.1f}° -> {math.degrees(q_side[j]):+.1f}°")
    check("...and the robot hand actually moves outward (+y)",
          p_side[1] > p_down[1], f"y {p_down[1]:+.3f} -> {p_side[1]:+.3f}")

    print("\n── elbow: bending flexes, it does not extend ───────────────────")
    # Upper arm down, forearm forward = a right angle at the elbow.
    q_bent, _ = retarget(down_e, [0.0, 0.0, -0.22])
    k = CHAIN.joint_names.index("left_elbow_pitch_joint")
    check("a bent elbow increases elbow_pitch",
          q_bent[k] > q_down[k] + math.radians(30),
          f"straight {math.degrees(q_down[k]):+.1f}° -> bent {math.degrees(q_bent[k]):+.1f}°")
    check("a straight arm keeps the elbow near its zero",
          abs(math.degrees(q_down[k])) < 25, f"{math.degrees(q_down[k]):+.1f}°")

    print("\n── every result stays inside the robot's limits ────────────────")
    for label, q in (("down", q_down), ("forward", q_fwd), ("back", q_back),
                     ("side", q_side), ("elbow bent", q_bent)):
        check(f"{label} respects joint limits",
              bool(np.all(q >= CHAIN.limits_lower - 1e-9)
                   and np.all(q <= CHAIN.limits_upper + 1e-9)))

    print("\n── a profile cannot silently invert a joint ────────────────────")
    # from_capture derives lo/hi from the poses themselves, so a capture whose maximum came
    # before its minimum must still produce lo < hi rather than a negative span that would
    # flip the mapping.
    cap = {
        "relaxed": {"angles": [0.0] * len(JOINTS), "upper_len": 0.26, "fore_len": 0.22},
        "tpose": {"angles": [-1.0] + [0.0] * (len(JOINTS) - 1), "upper_len": 0.26,
                  "fore_len": 0.22},
        "forward": {"angles": [0.5] + [0.0] * (len(JOINTS) - 1), "upper_len": 0.26,
                    "fore_len": 0.22},
    }
    built = ArmProfile.from_capture(cap)
    check("lo is below hi for every joint",
          all(built.lo_rad[n] <= built.hi_rad[n] for n in range(len(JOINTS))),
          f"pitch {math.degrees(built.lo_rad[0]):+.1f}..{math.degrees(built.hi_rad[0]):+.1f}")
    check("the zero comes from the relaxed pose",
          abs(built.zero_rad[0]) < 1e-9)

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
