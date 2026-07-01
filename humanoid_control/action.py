"""
Action → joint-target mapping.

    target = clip(action) * action_scale + default_pose        (Berkeley convention)
    target = clamp(target, position_limit_lower, position_limit_upper)   (SAFETY)

The clamp to per-joint position limits is a hard safety net: even a wild policy output
can never command a joint past its configured range. ``prev_action`` stores the *clipped,
pre-scale* action (what the trainer feeds back into the observation).
"""
from __future__ import annotations

import numpy as np

from .config import LegPolicyContract


class ActionMapper:
    def __init__(self, contract: LegPolicyContract, action_clip: float = 100.0):
        self.contract = contract
        self._scale = contract.action_scale
        self._default = contract.default_pose.astype(np.float32)
        self._lo = contract.pos_limit_lower.astype(np.float32)
        self._hi = contract.pos_limit_upper.astype(np.float32)
        self._n = contract.num_joints
        # Berkeley uses ±10000 (effectively none); we keep a sane finite clip so a NaN/huge
        # policy output can't overflow before the position-limit clamp catches it.
        self._action_clip = float(action_clip)
        self.prev_action = np.zeros(self._n, dtype=np.float32)

    def reset(self) -> None:
        self.prev_action = np.zeros(self._n, dtype=np.float32)

    def map(self, action: np.ndarray) -> np.ndarray:
        """Return clamped (12,) position targets and update ``prev_action``."""
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        assert action.shape == (self._n,), f"action dim {action.shape} != {self._n}"
        # Guard against NaN/inf before anything downstream sees it.
        action = np.nan_to_num(action, nan=0.0, posinf=self._action_clip, neginf=-self._action_clip)
        clipped = np.clip(action, -self._action_clip, self._action_clip)
        self.prev_action = clipped
        target = clipped * self._scale + self._default
        return np.clip(target, self._lo, self._hi)
