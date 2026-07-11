"""
Pluggable base-state source (orientation + angular velocity).

The policy needs ``projected_gravity`` (gravity unit vector in the base frame; ≈[0,0,-1]
upright) and ``base_ang_vel``. We express those behind a swappable interface so the policy
code doesn't change with the base-state source:

- ``TelemetryBaseState`` (default with the IMU): reads the daemon telemetry ``base`` block
  (an external WitMotion USB IMU, read by the daemon — see docs/DAEMON_SPEC.md §9). Yields
  ``valid=False`` whenever the daemon reports no fresh IMU data (``base: null``).
- ``UprightStubBaseState``: always upright, zero angular velocity — the no-IMU fallback.

⚠️ A stubbed-upright base state can *hold* a pose but cannot close a real balance loop, and
even with the IMU the balance loop is unproven — keep the robot supported until validated.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class BaseState:
    projected_gravity: np.ndarray   # (3,) gravity unit vector in base frame
    base_ang_vel: np.ndarray        # (3,) rad/s, base frame
    valid: bool = True              # False if no fresh IMU data (stub is always True)


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector ``v`` by the inverse of quaternion ``q`` = [w, x, y, z].

    Matches Berkeley ``rl_controller.quat_rotate_inverse`` so
    ``projected_gravity = quat_rotate_inverse(base_quat, [0,0,-1])``.
    """
    q = np.asarray(q, dtype=np.float32)
    v = np.asarray(v, dtype=np.float32)
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * (np.dot(q_vec, v)) * 2.0
    return a - b + c


class BaseStateSource(ABC):
    @abstractmethod
    def get(self) -> BaseState:
        ...


class UprightStubBaseState(BaseStateSource):
    """Fixed upright base state — the only correct choice until an IMU exists."""

    _GRAVITY = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    _ZERO = np.zeros(3, dtype=np.float32)

    def get(self) -> BaseState:
        return BaseState(
            projected_gravity=self._GRAVITY.copy(),
            base_ang_vel=self._ZERO.copy(),
            valid=True,
        )


class TelemetryBaseState(BaseStateSource):
    """Reads the planned daemon telemetry ``base`` block.

    ``telemetry_getter`` returns the latest telemetry frame dict (or None); the daemon emits
    the ``base`` block from the external IMU (docs/DAEMON_SPEC.md §9). It accepts either a
    precomputed ``projected_gravity`` (what the daemon ships — same convention as
    ``quat_rotate_inverse`` here) or a bare ``quaternion``, converting the latter if needed.
    """

    _GRAVITY_WORLD = np.array([0.0, 0.0, -1.0], dtype=np.float32)

    def __init__(self, telemetry_getter: Callable[[], dict | None]):
        self._get_tel = telemetry_getter

    def get(self) -> BaseState:
        tel = self._get_tel()
        base = (tel or {}).get("base") if isinstance(tel, dict) else None
        if not base:
            # No IMU data — signal invalid so the caller can refuse to run a balance loop.
            return BaseState(
                projected_gravity=self._GRAVITY_WORLD.copy(),
                base_ang_vel=np.zeros(3, dtype=np.float32),
                valid=False,
            )
        ang_vel = np.array(base.get("angular_velocity", [0, 0, 0]), dtype=np.float32)
        if base.get("projected_gravity") is not None:
            pg = np.array(base["projected_gravity"], dtype=np.float32)
        else:
            pg = quat_rotate_inverse(base["quaternion"], self._GRAVITY_WORLD)
        return BaseState(projected_gravity=pg, base_ang_vel=ang_vel, valid=True)
