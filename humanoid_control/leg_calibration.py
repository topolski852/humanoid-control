"""
Teach both legs' zeros from one held stance.

WHY THIS EXISTS ALONGSIDE ``calibration.py``. That module's two-capture flow is per joint and
needs each joint driven by hand to BOTH of its hardstops. Twelve joints times two stops is a
long, error-prone session, and it has to be redone after every power cycle — including the
ones forced by a single ESC crashing and taking the whole robot's power with it. The stance
below fixes every leg joint at once from a pose the operator can physically recreate in
seconds, so recovering from a crash costs one button press instead of a full session.

THE STANCE — "folded, feet together". The robot is folded so four joints per leg rest against
a mechanical hardstop, and the feet are pushed together so the remaining two sit square::

    hip_pitch     LOWER hardstop     thigh folded fully back
    knee_pitch    LOWER hardstop     knee straight
    ankle_pitch   UPPER hardstop     foot folded fully onto its pitch stop
    ankle_roll    LOWER hardstop     foot rolled fully onto its roll stop
    hip_roll      0  (declared)      legs parallel, feet touching
    hip_yaw       0  (declared)      feet pointing straight ahead

WHICH END EACH STOP IS CANNOT BE READ OFF A LIVE ANGLE. A joint resting on its lower stop
whose offset is wrong by the full range reads exactly what it would read sitting correctly
calibrated on its UPPER stop — the two are indistinguishable from any single measurement. The
two ankle axes sit on OPPOSITE ends in this stance, which is not guessable and was not
guessable from the readings either: ankle_roll was initially set to `upper` for precisely that
reason and had to be corrected against the physical robot. The only test that separates them is motion: move the
joint off the stop by hand and check that the number moves the way the joint does. Anything
that changes an offset can shift a reading; nothing here can reverse one, so a reading that
moves the WRONG way is a gear_ratio sign problem and not something this module can fix.

MEASURED VS DECLARED. A hardstop angle is a measurement: ``calibration.py`` already treats the
configured position limits as the physical stops (``offset = lower_pos - min_rad``), so the
limit is by definition what a joint reads while resting against that stop. The two remaining
joints have no stop in this stance, so their value is DECLARED by the stance rather than
measured — "feet touching and pointing forward" defines square, it does not prove it. Every
joint reports which kind it got, so a declared zero is never mistaken for a measured one.

BOTH LEGS TAKE THE SAME TARGETS, UNNEGATED. The display frame is not mirrored: left and right
carry identical position limits (see the policy contract), because the physical mirroring is
absorbed by the opposite gear-ratio signs set at commissioning and by ``policy_frame_sign`` on
the way into the policy. A symmetric stance therefore reads the SAME display-frame value on
both legs — which is what makes "mirror the good leg onto the dead one" a copy and not a
negation. Negating here would silently double the error on every roll and yaw joint.

MIRRORING. ``hip_roll`` and ``hip_yaw`` are the two joints the stance only declares. When the
opposite leg's matching joint is already calibrated and online, its live reading is a better
target than the bare declaration: it carries whatever real asymmetry the stance has, instead
of assuming perfect squareness. That is also the only way to give a joint a target when its
own ESC is dead and it must be re-zeroed from its twin later.

ACCURACY. This method is absolute — it writes the offset that makes the joint read the target
right now, so it resolves the single-turn encoder's multi-turn ambiguity outright (the AS5600
wraps every ~24 deg of output behind the 15:1 gearing). The trade is that the operator's stance
error lands directly in the zero. Hardstops are worth a degree or so; the declared joints are
worth however square the feet are. Use ``calibration.py``'s two-capture flow when a joint needs
to be better than that.

The offset maths is the same as ``arm_calibration``::

    displayed  = raw - position_offset
    new_offset = old_offset - (expected - displayed_now)
"""
from __future__ import annotations

import math

DEG = math.pi / 180.0

# Joint types that rest on a hardstop in the stance -> which limit that stop is.
# Keys are the side-stripped joint type, so one table serves both legs.
HARDSTOP: dict[str, str] = {
    "hip_pitch":   "lower",
    "knee_pitch":  "lower",
    # Note the two ankle axes are on OPPOSITE ends. Verified against the robot; do not
    # "tidy" them into agreement.
    "ankle_pitch": "upper",
    "ankle_roll":  "lower",
}

# Joint types the stance DECLARES rather than measures, and the angle it declares (radians).
DECLARED: dict[str, float] = {
    "hip_roll": 0.0,
    "hip_yaw":  0.0,
}

# The joints this stance can zero at all. Anything else on a leg is not covered.
STANCE_TYPES: tuple[str, ...] = tuple(HARDSTOP) + tuple(DECLARED)


def joint_type(joint_name: str) -> str:
    """'right_ankle_roll_joint' -> 'ankle_roll'."""
    t = joint_name
    for side in ("left_", "right_"):
        if t.startswith(side):
            t = t[len(side):]
            break
    return t[:-len("_joint")] if t.endswith("_joint") else t


def opposite_joint(joint_name: str) -> str | None:
    """The same joint on the other leg, or None if the name is not sided."""
    if joint_name.startswith("left_"):
        return "right_" + joint_name[len("left_"):]
    if joint_name.startswith("right_"):
        return "left_" + joint_name[len("right_"):]
    return None


def covers(joint_name: str) -> bool:
    """True when the stance defines an angle for this joint."""
    return joint_type(joint_name) in STANCE_TYPES


def is_declared(joint_name: str) -> bool:
    """True when the stance DECLARES this joint's angle instead of measuring it."""
    return joint_type(joint_name) in DECLARED


def stance_target(joint_name: str, limits: tuple[float, float]) -> tuple[float, str]:
    """``(expected display-frame angle in radians, source)`` for one joint in the stance.

    ``limits`` is the joint's ``(lower, upper)`` in the same display frame — for the legs that
    is the POLICY CONTRACT's pair, which is what every clamp in the runtime already uses.
    ``source`` is ``'measured'`` (resting on a hardstop) or ``'declared'`` (fixed by the stance).
    """
    t = joint_type(joint_name)
    if t in HARDSTOP:
        lower, upper = limits
        if upper <= lower:
            raise ValueError(f"{joint_name}: limits are not ordered ({lower}, {upper})")
        return (lower if HARDSTOP[t] == "lower" else upper), "measured"
    if t in DECLARED:
        return DECLARED[t], "declared"
    raise ValueError(f"{joint_name}: the folded stance does not define a {t} angle")


def solve_offset(displayed_now: float, expected: float, old_offset: float) -> float:
    """New ``position_offset`` that makes a joint reading ``displayed_now`` read ``expected``."""
    return old_offset - (expected - displayed_now)
