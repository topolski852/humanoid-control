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
#   shoulder_pitch  0     no pitch when the arm is straight out to the side
#   shoulder_roll  +74.8  where the wrist's height equals the shoulder's. NOT +90: the URDF's
#                         roll zero sits ~23 deg out from vertical, and the roll axis is offset
#                         ~5 cm from the pitch axis, so joint angle and visual elevation are
#                         not 1:1.
#   elbow_pitch     0     straight
#
# DECLARED — the two inline twists. A straight arm gives no geometric constraint on rotation
# ABOUT the arm, so the value here DEFINES the held twist rather than measuring it. That is
# precisely why this constant must match the pose people actually hold: shoulder_yaw does not
# detect your forearm rotation, it adopts it. Declaring 0 while holding the arm rolled 90 deg
# bakes a 90 deg error into the zero, invisible in the T-pose itself (the joint reads its
# target either way) and only surfacing once the elbow leaves zero — e.g. dropping to a
# desk pose.
#
# -45 is the value, not 0 and not +90. Derived from measurement against the URDF limits
# (shoulder_yaw is +-45): holding this T-pose and then dropping to a desk pose moves the joint
# ~90 deg, which is its ENTIRE range, so the two poses sit at opposite ends. Travel is positive
# going T-pose -> desk, which puts the T-pose hold at the negative end. The right arm gets +45
# via MIRRORED. An earlier 0 here (and a briefly-tried +90) both put the joint outside its own
# limits once the elbow left zero.
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
    # -90 here a relaxed arm reads +0.8 (left) / -2.6 (right); with -45 it read ~+46.
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
