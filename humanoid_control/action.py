"""
Action → joint-target mapping.

    target = clip(action) * action_scale + default_pose        (Berkeley convention)
    target = clamp(target, position_limit_lower, position_limit_upper)   (SAFETY)

The clip bounds come from the trainer contract (``action_limit_lower/upper``) so the runtime
clips exactly where the trainer did; a wider runtime clip would let the policy command targets
it never saw in training. ``prev_action`` stores the *clipped, pre-scale* action (what the
trainer feeds back into the observation), so the bound also keeps that obs term in range.

The clamp to per-joint position limits is a separate hard safety net: even a wild policy output
can never command a joint past its configured range.
"""
from __future__ import annotations

import numpy as np

from .config import LegPolicyContract


class ActionMapper:
    def __init__(self, contract: LegPolicyContract,
                 action_clip: float | None = None,
                 action_clip_upper: float | None = None):
        self.contract = contract
        self._scale = contract.action_scale
        self._default = contract.default_pose.astype(np.float32)
        # policy(URDF) -> device frame per-joint sign flip (mirrored right-leg roll/yaw joints).
        self._sign = contract.policy_frame_sign.astype(np.float32)
        self._lo = contract.pos_limit_lower.astype(np.float32)
        self._hi = contract.pos_limit_upper.astype(np.float32)
        self._n = contract.num_joints
        # Pre-scale action clip. Defaults to the TRAINER's exported action_limit_lower/upper
        # (contract `action` block) — the trainer clips to this range before scaling, so a wider
        # runtime clip lets the policy drive targets it never saw in training. Explicit args
        # override (asymmetric supported); passing only ``action_clip`` gives a symmetric ±clip.
        if action_clip is None and action_clip_upper is None:
            lo, hi = float(contract.action_limit_lower), float(contract.action_limit_upper)
        elif action_clip_upper is None:
            lo, hi = -abs(float(action_clip)), abs(float(action_clip))
        else:
            lo, hi = float(action_clip), float(action_clip_upper)
        if not lo < hi:
            raise ValueError(f"action clip lower {lo} must be < upper {hi}")
        self._clip_lo, self._clip_hi = lo, hi
        self.prev_action = np.zeros(self._n, dtype=np.float32)

    def reset(self) -> None:
        self.prev_action = np.zeros(self._n, dtype=np.float32)

    def map(self, action: np.ndarray) -> np.ndarray:
        """Return clamped (12,) position targets and update ``prev_action``."""
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        assert action.shape == (self._n,), f"action dim {action.shape} != {self._n}"
        # Guard against NaN/inf before anything downstream sees it.
        action = np.nan_to_num(action, nan=0.0, posinf=self._clip_hi, neginf=self._clip_lo)
        clipped = np.clip(action, self._clip_lo, self._clip_hi)
        self.prev_action = clipped  # stored in POLICY frame (fed back into the obs unchanged)
        # The policy outputs a delta in the POLICY(URDF) frame; flip it to the DEVICE frame before
        # adding the device-frame default_pose, so mirrored right-leg joints drive the correct way.
        target = self._sign * (clipped * self._scale) + self._default
        return np.clip(target, self._lo, self._hi)
