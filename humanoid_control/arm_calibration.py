"""
Teach an arm's zero from a known held pose.

The arms have no hardstops, so ``calibration.py``'s two-capture method does not apply to them:
there is nothing to drive against. Instead the operator holds the arm in a pose whose URDF
angles are computable and the offsets are solved directly. See ``T_POSE_LEFT_DEG`` below for
the exact hold — it is NOT the textbook untwisted T-pose, and the difference matters.

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

# THE reference pose: arm straight out to the side, horizontal, elbow straight, claw neutral,
# and the upper arm rolled so THE ELBOW BENDS HORIZONTALLY (forearm swings forward/back, not
# up/down). A T-pose is easy to hold accurately (any level edge gives you horizontal) and it
# defines all five joints in one hold.
#
# WHY THE ELBOW BENDS HORIZONTALLY. Held that way the arm carries its own weight against the
# joint geometry instead of sagging, so the operator can hold it square for the ~1.5 s sample.
# An untwisted hold is harder to keep level, and a T-pose that is not actually level poisons
# shoulder_roll — the one joint here whose value is measured rather than declared.
#
# MEASURED — fixed by geometry:
#   shoulder_roll  +74.8  where the wrist's height equals the shoulder's. NOT +90: the URDF's
#                         roll zero sits ~23 deg out from vertical, and the roll axis is offset
#                         ~5 cm from the pitch axis, so joint angle and visual elevation are
#                         not 1:1. Verified against the vendored URDF: roll +74.8 alone puts
#                         the left wrist 33.2 cm out along +Y at exactly shoulder height.
#   elbow_pitch     0     straight
#
# DECLARED — the three values the hold DEFINES rather than measures: the two inline twists
# (shoulder_yaw, wrist_yaw) and shoulder_pitch, which at this roll has become an inline twist
# too. A straight arm gives no geometric constraint on rotation ABOUT the arm, so these values
# fix the held twist instead of measuring it. That is precisely why they must match the pose
# people actually hold: shoulder_yaw does not detect your forearm rotation, it adopts it.
# Declaring 0 while holding the arm rolled 90 deg bakes a 90 deg error into the zero, invisible
# in the T-pose itself (the joint reads its target either way) and only surfacing once the
# elbow leaves zero — e.g. dropping to a desk pose.
#
# pitch +90 and yaw -90 CANCEL for the hand position: checked against the vendored URDF, the
# declared T-pose below puts the left wrist within 1 mm of where a pure roll +74.8 does. They
# are not describing where the hand is, they are describing how the arm is TWISTED to get
# there — the quarter turn on each that makes the elbow bend horizontally and the arm carry
# its own weight. Both therefore sit outside the URDF's own soft limits (pitch [-90,+45], yaw
# [+-45]); that is expected, because the firmware clamps against the WIDER hard tier in
# humanoid_lite.json, and after calibration a relaxed arm reads ~0 on both, well inside.
T_POSE_LEFT_DEG = {
    # 90, not 0. The reference hold rotates shoulder_pitch a quarter turn from relaxed so its
    # axis goes HORIZONTAL — that is what stops the arm falling and lets the operator hold the
    # pose unsupported. Declaring 0 describes a T-pose where pitch never moved, which bakes a
    # 90 deg error into the zero: the arm then reads ~-90 hanging (its own limit) instead of
    # ~0, and the render/teleop move pitch backwards, so raising the real arm lowers the
    # drawn one. Measured both arms: relaxed lands within 2 deg of zero with this value.
    "shoulder_pitch": 90.0,
    "shoulder_roll": 74.8,
    # -90, for the same reason as shoulder_pitch above: the reference hold rotates this joint
    # a quarter turn from relaxed too, so the elbow bends HORIZONTALLY and the arm carries its
    # own weight. Both quarter-turns are what make the T-pose self-supporting. Verified: with
    # -90 here a relaxed arm reads +0.8 (left) / -2.6 (right); with -45 it read ~+46, which is
    # why an earlier comment here argued for -45 — that reasoning is superseded.
    "shoulder_yaw": -90.0,
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
