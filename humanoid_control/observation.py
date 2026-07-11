"""
Observation assembly — builds the policy input vector in the exact contract order.

Layout (must match ``LegPolicyContract.obs_layout`` and the trainer):

    [ command(3), base_ang_vel(3), projected_gravity(3),
      joint_pos - default_pose (12), joint_vel(12), prev_action(12) ]   → 45

``joint_pos`` is fed **relative to default_pose** (matches Berkeley rl_controller).
The builder asserts its hardcoded field order equals the contract's declared layout, so
a trainer-side layout change fails loudly instead of silently corrupting the obs.
"""
from __future__ import annotations

import numpy as np

from .base_state import BaseState
from .config import LegPolicyContract

# The field order this module produces. Keep in lockstep with the contract file.
_EXPECTED_LAYOUT = (
    "command(3)",
    "base_ang_vel(3)",
    "projected_gravity(3)",
    "joint_pos_minus_default(12)",
    "joint_vel(12)",
    "prev_action(12)",
)


class ObservationBuilder:
    def __init__(self, contract: LegPolicyContract):
        self.contract = contract
        if tuple(contract.obs_layout) != _EXPECTED_LAYOUT:
            raise ValueError(
                "Observation layout mismatch between runtime and contract.\n"
                f"  runtime: {_EXPECTED_LAYOUT}\n"
                f"  contract: {tuple(contract.obs_layout)}\n"
                "Update observation.py and the trainer together, then regenerate the contract."
            )
        self._default_pose = contract.default_pose.astype(np.float32)
        self._n = contract.num_joints
        # device -> policy(URDF) frame per-joint sign flip (mirrored right-leg roll/yaw joints).
        self._sign = contract.policy_frame_sign.astype(np.float32)

    def build(
        self,
        joint_pos: np.ndarray,      # (12,) rad, canonical order
        joint_vel: np.ndarray,      # (12,) rad/s, canonical order
        base_state: BaseState,
        command: np.ndarray,        # (3,) velocity command
        prev_action: np.ndarray,    # (12,) previous (pre-scale, clipped) action
    ) -> np.ndarray:
        joint_pos = np.asarray(joint_pos, dtype=np.float32)
        joint_vel = np.asarray(joint_vel, dtype=np.float32)
        command = np.asarray(command, dtype=np.float32)
        prev_action = np.asarray(prev_action, dtype=np.float32)
        assert joint_pos.shape == (self._n,), joint_pos.shape
        assert joint_vel.shape == (self._n,), joint_vel.shape
        assert command.shape == (3,), command.shape
        assert prev_action.shape == (self._n,), prev_action.shape

        # joint_pos/joint_vel arrive in the DEVICE frame; convert the per-joint quantities to the
        # POLICY(URDF) frame the network was trained in by multiplying by the sign map. The subtraction
        # of default_pose (also device frame) is done first, then flipped: s*(pos-default) == P_pos -
        # P_default. prev_action is already the network's own output (policy frame) -> no flip.
        obs = np.concatenate([
            command,                                          # 3
            base_state.base_ang_vel,                          # 3
            base_state.projected_gravity,                     # 3
            self._sign * (joint_pos - self._default_pose),    # 12
            self._sign * joint_vel,                           # 12
            prev_action,                                      # 12
        ]).astype(np.float32)
        assert obs.shape == (self.contract.num_observations,), (
            f"obs dim {obs.shape} != contract {self.contract.num_observations}"
        )
        return obs
