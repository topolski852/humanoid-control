"""
HeadingHold — close the heading loop on the robot, the way sim closes it in its command generator.

Why: sim trains with ``heading_command``: the command generator turns a heading error into
``ang_vel_z = clip(heading_control_stiffness * wrap(target - yaw), ±max)``. The policy never
sees heading, only the yaw-rate command. On the robot nothing closes that loop, so any veer is
never corrected. Measured 2026-09-29 (REPORT_2026-09-29_walk_measC_vs_smoothA.md): measC-full
and smoothA-full veer 6-11 deg/s while walking, in a direction that changes from bout to bout.
IMU yaw drift at rest is ~0.1-1 deg/min, negligible at walking timescales.

Behaviour, per tick (``update``):

* **Walking** (|vx| or |vy| above deadband) with **no operator yaw**: hold the heading latched
  at the start of the bout; ``wz = clip(K * wrap(target - yaw), ±wz_max)``.
* **Operator yaw** (|wz_op| above deadband): pass the operator's wz through unchanged and let the
  target follow the robot, so the loop never fights the stick. Re-latching, rather than
  integrating wz_op into the target, avoids the ~2*wz_op/K steady-state lag a P-loop would
  build while chasing a moving target.
* **Standing**: wz passes through as commanded (0 when standing, matching sim's standing envs,
  which zero the whole command), and the target re-latches, so a bout starts from wherever the
  robot is facing.

Modes: ``off`` (default: command untouched, nothing computed), ``dry`` (compute and log, send
the operator's command unchanged: use this for the sign check), ``on`` (apply).

SIGN: yaw is atan2 over the IMU quaternion [w, x, y, z]. With the IMU mounted x-fwd / y-left /
z-up it increases counter-clockwise from above, the same sense as sim's +ang_vel_z, so the
correction is +K * (target - yaw). **Confirm on hardware in ``dry`` mode before ``on``**: turn
the robot left by hand during a walk bout; ``wz_loop`` must go NEGATIVE (turn back right). A
wrong sign is positive feedback, and the robot will spin up.
"""
from __future__ import annotations

import math
import os

import numpy as np

MODES = ("off", "dry", "on")


def yaw_from_quat(q) -> float:
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HeadingHold:
    def __init__(self, mode: str | None = None, k: float | None = None,
                 wz_max: float | None = None, deadband: float = 0.05):
        self.mode = mode or os.environ.get("HUMANOID_HEADING", "off")
        if self.mode not in MODES:
            raise ValueError(f"heading mode must be one of {MODES}, got {self.mode!r}")
        # 0.5 = sim heading_control_stiffness. wz_max stays inside the trained ang_vel_z range
        # (1.2); the measured veer is 0.1-0.2 rad/s, so 0.5 leaves ample authority.
        self.k = float(k if k is not None else os.environ.get("HUMANOID_HEADING_K", 0.5))
        self.wz_max = float(wz_max if wz_max is not None else
                            os.environ.get("HUMANOID_HEADING_WZ_MAX", 0.5))
        self.deadband = deadband
        self.target: float | None = None
        self.last: dict = {}

    def reset(self) -> None:
        """New engage: forget the target; the next tick latches the current heading."""
        self.target = None

    def update(self, command: np.ndarray, quaternion) -> np.ndarray:
        """Return the command to feed the policy this tick. Never mutates ``command``."""
        cmd = np.asarray(command, dtype=np.float32)
        if self.mode == "off" or quaternion is None:
            self.last = {"mode": self.mode, "valid": quaternion is not None}
            return cmd
        yaw = yaw_from_quat(quaternion)
        walking = abs(float(cmd[0])) > self.deadband or abs(float(cmd[1])) > self.deadband
        op_yaw = abs(float(cmd[2])) > self.deadband
        if self.target is None or not walking or op_yaw:
            self.target = yaw
        err = wrap_pi(self.target - yaw)
        wz_loop = float(np.clip(self.k * err, -self.wz_max, self.wz_max))
        active = walking and not op_yaw
        out = cmd.copy()
        if active and self.mode == "on":
            out[2] = wz_loop
        self.last = {"mode": self.mode, "valid": True, "yaw": yaw, "target": self.target,
                     "heading_error": err, "wz_loop": wz_loop, "active": active,
                     "wz_sent": float(out[2])}
        return out
