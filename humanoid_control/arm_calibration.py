"""
Teach an arm's zero from a known held pose.

The arms have no hardstops, so ``calibration.py``'s two-capture method does not apply to them:
there is nothing to drive against. Instead the operator holds the arm in a pose whose URDF
angles are computable and the offsets are solved directly.

This has to be done after EVERY power cycle, and flashing does not help. The AS5600 is
single-turn absolute; behind 15:1 gearing it wraps every ~24 deg of output travel, so on
power-up the true joint angle is ambiguous by multiples of that no matter what
``position_offset`` is stored. A stored offset that still matches proves nothing about where
the joint actually is — which is why a freshly powered arm reads nonsense while sitting
physically relaxed.

The maths::

    displayed  = raw - position_offset      # firmware works in RAW; actuator.cpp writes
                                            # position_limit +- position_offset
    delta      = urdf_expected - displayed_now
    new_offset = old_offset - delta         # the offset moves OPPOSITE to the display

See the wiki page "Arm Joint Frames and Calibration" for the frame conventions.
"""
from __future__ import annotations

import math

DEG = math.pi / 180.0

# THE reference pose: arm straight out to the side, horizontal, elbow straight, forearm
# untwisted, claw neutral. A T-pose is easy to hold accurately (any level edge gives you
# horizontal) and it defines all five joints in one hold.
#
# MEASURED — fixed by geometry:
#   shoulder_pitch  0     no pitch when the arm is straight out to the side
#   shoulder_roll  +74.8  where the wrist's height equals the shoulder's. NOT +90: the URDF's
#                         roll zero sits ~23 deg out from vertical, and the roll axis is offset
#                         ~5 cm from the pitch axis, so joint angle and visual elevation are
#                         not 1:1.
#   elbow_pitch     0     straight
#
# DECLARED — the two inline twists. A straight arm gives no geometric constraint on rotation
# ABOUT the arm, so zero here DEFINES "untwisted" rather than measuring it. Note the URDF's own
# zero for shoulder_yaw sits ~15 deg from this; the cost is ~15 deg of drawn forearm-plane
# accuracy once the elbow bends, and nothing else.
T_POSE_LEFT_DEG = {
    "shoulder_pitch": 0.0,
    "shoulder_roll": 74.8,
    "shoulder_yaw": 0.0,
    "elbow_pitch": 0.0,
    "wrist_yaw": 0.0,
}

# Joints whose value is a convention rather than a measurement, surfaced so the UI can say so.
DECLARED = ("shoulder_yaw", "wrist_yaw")

# The URDF's arms are exact mirrors — every right-arm limit is the negation of the left's — so
# the right arm's T-pose is the negated left one.
MIRRORED = True


def t_pose_targets(limb: str) -> dict[str, float]:
    """{joint_name: expected angle in radians} for a limb held in the T-pose."""
    if not limb.endswith("_arm"):
        raise ValueError(f"{limb} is not an arm")
    side = "right" if limb.startswith("right") else "left"
    sign = -1.0 if (side == "right" and MIRRORED) else 1.0
    return {f"{side}_{t}_joint": sign * v * DEG for t, v in T_POSE_LEFT_DEG.items()}


def solve_offset(displayed_now: float, expected: float, old_offset: float) -> float:
    """New ``position_offset`` that makes a joint reading ``displayed_now`` read ``expected``."""
    return old_offset - (expected - displayed_now)


def is_declared(joint_name: str) -> bool:
    return any(joint_name.endswith(f"{t}_joint") for t in DECLARED)
