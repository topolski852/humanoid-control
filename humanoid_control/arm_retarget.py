"""
Human arm pose (WebXR body tracking) → robot arm joint angles.

Pure functions, no I/O, so this is testable offline against recorded captures exactly the
way ``arm_kinematics`` is. Nothing here talks to a robot or a socket.

WHAT THE HEADSET ACTUALLY GIVES US, and why it matters here. The WebXR Body Tracking module
reports 83 joints including ``left-shoulder``, ``left-arm-upper`` and ``left-arm-lower``, but
on a Quest those are **inferred**, not observed — there are no cameras watching your torso.
Meta solves the upper body from the headset and controller poses plus body proportions. The
wrist is therefore excellent (it IS the controller), the elbow is a solved estimate, and the
shoulder is the least-constrained joint in that solve.

Measured on this hardware with the arm held still: forearm length varied 58 mm and upper-arm
length 136 mm over 20 s, against segments of 25 cm and 33 cm. Since limbs do not change
length, that is all measurement error, and it works out at roughly ±12° of phantom shoulder
angle. That number is why :func:`joint_noise_deg` exists and why the caller is expected to
filter, or to use these angles only as a posture hint rather than a direct command.

FRAME. All positions arrive in the robot frame (+X forward, +Y left, +Z up) — the caller
converts with ``xr.webxr_to_robot`` before getting here. Angles come back in the robot's own
joint convention, matching the URDF, so they are directly comparable to telemetry.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Joints this module needs. A capture missing any of them cannot be retargeted.
#
# NOTE `arm-upper`, NOT `shoulder`. The spec exposes both, and they are different points:
# `left-shoulder` sits at the clavicle/scapula end, `left-arm-upper` at the glenohumeral
# joint where the humerus actually starts. Measured on this operator, shoulder→elbow is
# 40.8 cm while arm-upper→elbow is 26.2 cm — the first is an upper arm plus half a
# collarbone, and using it skewed the segment DIRECTION as well as its length.
REQUIRED = ("chest", "hips", "left-arm-upper", "left-arm-lower",
            "left-hand-wrist-twist", "left-hand-wrist",
            "left-shoulder", "right-shoulder")


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])


def _pos(joints: dict, name: str) -> np.ndarray | None:
    j = joints.get(name)
    if not isinstance(j, dict) or j.get("p") is None:
        return None
    p = np.asarray(j["p"], dtype=float)
    return p if p.shape == (3,) and np.all(np.isfinite(p)) else None


@dataclass(frozen=True)
class HumanArm:
    """One sample of the operator's arm, in robot-frame radians.

    Angles use the ROBOT's conventions so they can be compared with telemetry directly:
      shoulder_pitch  forward swing, - = forward (the ROBOT's sense, not anatomy — see below)
      shoulder_roll   abduction, + = away from the body
      shoulder_yaw    humeral rotation about the upper-arm axis
      elbow           flexion, 0 = straight
      wrist           twist about the forearm axis
    """

    shoulder_pitch: float
    shoulder_roll: float
    shoulder_yaw: float
    elbow: float
    wrist: float
    upper_len: float
    fore_len: float

    def as_array(self) -> np.ndarray:
        return np.array([self.shoulder_pitch, self.shoulder_roll, self.shoulder_yaw,
                         self.elbow, self.wrist])


def torso_frame(joints: dict) -> np.ndarray | None:
    """Orthonormal basis of the operator's torso, as columns [forward, left, up].

    ARM ANGLES ARE MEANINGLESS IN THE WORLD FRAME. Turn on the spot and every world-frame
    shoulder angle changes while your actual posture has not moved at all. Measured on this
    operator standing normally, the shoulder line sat 38.5 deg off the robot frame — which
    read out as ~88 deg of both pitch AND roll for an arm that was simply out to the side.

    Built from positions rather than the chest's reported ORIENTATION: on an inferred upper
    body the joint orientations are the least trustworthy part of the solve, while the
    shoulder line and the hips→chest axis are the same geometry the solver was fitting.
    """
    ls, rs = _pos(joints, "left-shoulder"), _pos(joints, "right-shoulder")
    ch, hp = _pos(joints, "chest"), _pos(joints, "hips")
    if ls is None or rs is None or ch is None or hp is None:
        return None
    left = ls - rs
    up0 = ch - hp
    if float(np.linalg.norm(left)) < 0.05 or float(np.linalg.norm(up0)) < 0.05:
        return None
    y = _unit(left)                       # operator's left
    fwd = np.cross(y, _unit(up0))         # right-handed: X = Y x Z
    if float(np.linalg.norm(fwd)) < 1e-6:
        return None
    x = _unit(fwd)
    z = np.cross(x, y)                    # re-derived so the basis is exactly orthonormal
    return np.column_stack([x, y, z])


def human_angles(joints: dict, *, side: str = "left") -> HumanArm | None:
    """Decompose tracked body joints into arm angles. None if the sample is unusable.

    Deliberately geometric rather than quaternion-based: the shoulder's reported ORIENTATION
    is the least trustworthy part of an inferred upper body, while the segment DIRECTIONS
    follow from joint positions and are the same information the solver used. Working from
    directions keeps us one step closer to what was actually estimated.
    """
    pre = f"{side}-"
    sh = _pos(joints, f"{pre}arm-upper")     # glenohumeral joint, NOT `shoulder` — see REQUIRED
    el = _pos(joints, f"{pre}arm-lower")
    wr = _pos(joints, f"{pre}hand-wrist")
    R = torso_frame(joints)
    if sh is None or el is None or wr is None or R is None:
        return None

    # Everything below is in the TORSO frame, so the angles describe the operator's posture
    # rather than which way they happen to be facing.
    Rt = R.T
    upper = Rt @ (el - sh)               # shoulder → elbow
    fore = Rt @ (wr - el)                # elbow → wrist
    upper_len = float(np.linalg.norm(upper))
    fore_len = float(np.linalg.norm(fore))
    if upper_len < 0.05 or fore_len < 0.05:
        return None                      # degenerate: joints collapsed onto each other

    u = _unit(upper)
    f = _unit(fore)

    # Elbow flexion: the angle between the segments. 0 = straight, +pi = fully folded.
    # This is the single most robust quantity available, because it depends only on the
    # RELATIVE direction of two segments and so is immune to whole-arm placement error.
    elbow = float(np.arccos(np.clip(np.dot(u, f), -1.0, 1.0)))

    # Shoulder angles from the upper-arm direction, decomposed in the ROBOT'S OWN joint
    # order — pitch about the lateral axis first, then roll about the forward axis, which is
    # how the physical chain is built (shoulder_pitch is proximal to shoulder_roll). Hanging
    # straight down is the zero: u = (0, 0, -1).
    #
    # Taking both angles as arctan2(·, -u_z) — the obvious first guess — is degenerate: for a
    # horizontal arm -u_z ≈ 0, so BOTH blow up toward 90 deg at once. Measured on a real
    # T-pose it reported pitch 55 deg and roll 83 deg for an arm that was simply out to the
    # side. Matching the kinematic order puts the singularity where the robot cannot reach
    # anyway (straight up) instead of in the middle of the working range.
    #
    #   u_x = sin(pitch)                 -> pitch = asin(u_x)
    #   u_y = cos(pitch)·sin(roll)       -> roll  = atan2(u_y, -u_z)
    #   u_z = -cos(pitch)·cos(roll)
    # NEGATED, and this is the whole subtlety of the line. The decomposition below is
    # anatomical: u_x is the forward component of the upper arm, so asin(u_x) is POSITIVE
    # when the operator's arm swings forward. The ROBOT's shoulder_pitch runs the other way
    # — forward kinematics puts the hand at x=+0.242 for pitch -60 and x=-0.197 for pitch
    # +45, so negative pitch is forward. This function's contract is to emit the ROBOT's
    # convention (it is documented as decomposing in robot kinematic order and its output
    # feeds joint targets directly), so the sign belongs here rather than as a fudge factor
    # further downstream.
    #
    # Measured on hardware: without this the operator moved their arm forward and the robot
    # arm swung back. Nothing else was mirrored wrongly — roll abducts the same way on both,
    # and elbow flexion agrees — which is exactly why a single inverted joint is easy to ship.
    pitch = -float(np.arcsin(np.clip(u[0], -1.0, 1.0)))
    roll = float(np.arctan2(u[1], -u[2]))

    # Humeral rotation: where the forearm sits around the upper-arm axis. Projecting the
    # forearm into the plane perpendicular to the upper arm removes the elbow's contribution,
    # leaving only the twist. Undefined when the arm is straight (the projection vanishes),
    # which is correct — you cannot see humeral rotation in a straight arm.
    perp = f - np.dot(f, u) * u
    if float(np.linalg.norm(perp)) < 1e-3:
        yaw = 0.0
    else:
        perp = _unit(perp)
        # Reference: "forward" projected into the same plane.
        fwd = np.array([1.0, 0.0, 0.0])
        ref = _unit(fwd - np.dot(fwd, u) * u)
        cross = np.cross(ref, perp)
        yaw = float(np.arctan2(float(np.dot(cross, u)), float(np.dot(ref, perp))))

    # Wrist twist about the forearm axis, from the dedicated twist joint when present.
    wrist = 0.0
    tw = _pos(joints, f"{pre}hand-wrist-twist")
    if tw is not None:
        tv = Rt @ (wr - tw)
        if float(np.linalg.norm(tv)) > 1e-3:
            t = _unit(tv)
            perp_t = t - np.dot(t, f) * f
            if float(np.linalg.norm(perp_t)) > 1e-3:
                perp_t = _unit(perp_t)
                ref_t = _unit(u - np.dot(u, f) * f)
                if float(np.linalg.norm(ref_t)) > 1e-3:
                    cross_t = np.cross(ref_t, perp_t)
                    wrist = float(np.arctan2(float(np.dot(cross_t, f)),
                                             float(np.dot(ref_t, perp_t))))

    return HumanArm(pitch, roll, yaw, elbow, wrist, upper_len, fore_len)


def joint_noise_deg(samples: list[HumanArm]) -> dict[str, float]:
    """Per-angle spread (max-min) in degrees over a set of samples.

    Run this on a capture taken while HOLDING STILL: anything non-zero is measurement error
    that would be commanded straight into a motor. This is a far more direct quality metric
    than segment-length drift, because it is the quantity the robot actually receives.
    """
    if not samples:
        return {}
    a = np.array([s.as_array() for s in samples])
    names = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist")
    return {n: float(np.degrees(a[:, i].max() - a[:, i].min())) for i, n in enumerate(names)}
