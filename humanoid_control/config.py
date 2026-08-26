"""
Leg policy contract — the single source of truth for the sim↔real interface.

Loads ``configs/leg_policy_params.json`` (generated from the ESC pull; see repo
``POLICY_CONTRACT.md``) into a typed, immutable object the runtime and the trainer must
agree on **exactly**:

- ``joint_order``   — canonical 12-leg order (indices 0..11)
- ``default_pose``  — per-joint starting pose (rad); also the action offset
- gains / effort / Kt / signed gear / position limits — per joint, pulled from the ESCs
- ``action_scale``, ``policy_dt``, ``control_dt`` — control constants
- observation layout — the 45-dim obs field order

If the trainer changes any of these, regenerate the contract and re-load here; never
hand-edit one side.
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_log = logging.getLogger(__name__)

# Repo root = two levels up from this file (humanoid_control/config.py -> repo/)
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONTRACT_PATH = REPO_ROOT / "configs" / "leg_policy_params.json"

# --- live robot config ------------------------------------------------------
# The per-joint hardware config (gains, signed gear, limits) that the daemon is driving. There
# is more than one copy of this file on a typical machine and they DRIFT: the studio GUI writes
# to its user-config dir, while the repo checkout keeps a version-controlled copy. Editing one
# and launching the daemon against the other is a silent, hard-to-see failure — the robot runs
# gains you didn't set.
#
# So resolve in a fixed order and LOG which copy won, rather than naming a single path that may
# not exist. Point the daemon's --config at whatever this resolves to.
ROBOT_CONFIG_CANDIDATES = (
    Path("~/.config/humanoid-studio/humanoid_lite.json").expanduser(),   # studio GUI writes here
    Path("~/humanoid/humanoid-studio/configs/humanoid_lite.json").expanduser(),
    REPO_ROOT / "configs" / "humanoid_lite.json",
)


def resolve_robot_config_path() -> Path | None:
    """First existing robot config: ``$HUMANOID_CONFIG`` then ``ROBOT_CONFIG_CANDIDATES``.

    Returns None when none exist (the runtime then runs telemetry-only and refuses to connect).
    An explicit ``$HUMANOID_CONFIG`` that does not exist is reported rather than skipped — a
    typo'd override must not silently fall through to a different robot's gains.
    """
    env = os.environ.get("HUMANOID_CONFIG")
    if env:
        p = Path(env).expanduser()
        if p.exists():
            _log.info("robot config: %s (from $HUMANOID_CONFIG)", p)
            return p
        _log.error("$HUMANOID_CONFIG=%s does not exist — falling back to the search path.", p)
    for p in ROBOT_CONFIG_CANDIDATES:
        if p.exists():
            _log.info("robot config: %s", p)
            return p
    _log.warning("no robot config found; searched: %s",
                 ", ".join(str(p) for p in ROBOT_CONFIG_CANDIDATES))
    return None


# Back-compat alias for callers that want a single path (may not exist).
LIVE_ROBOT_CONFIG_PATH = ROBOT_CONFIG_CANDIDATES[0]

# --- policy vs device frame -------------------------------------------------
# The policy is trained in the URDF frame, which is left<->right MIRROR-symmetric (a symmetric
# stance uses OPPOSITE signs on left/right). The robot's device/ESC frame is UN-mirrored (both legs
# use the SAME sign for a symmetric stance). So exactly these right-leg roll/yaw joints have opposite
# sign between the two frames and must be sign-flipped at the policy<->device boundary. Left joints
# and all pitch joints already match. See humanoid-policy README "Joint SIGN / frame convention" and
# POLICY_CONTRACT.md.
POLICY_FRAME_MIRRORED_JOINTS = (
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_ankle_roll_joint",
)


@dataclass(frozen=True)
class LegPolicyContract:
    """Immutable, ordered view of the legs-only policy contract."""

    joint_order: tuple[str, ...]          # length 12, canonical order
    default_pose: np.ndarray              # (12,) rad — starting pose / action offset
    kp: np.ndarray                        # (12,) firmware position_kp
    kd: np.ndarray                        # (12,) firmware velocity_kp (acts as Kd)
    effort_limit: np.ndarray              # (12,) Nm (firmware torque_limit)
    torque_constant: np.ndarray           # (12,) Nm/A
    gear_ratio: np.ndarray                # (12,) signed
    position_offset: np.ndarray           # (12,) rad (session calibration; informational)
    pos_limit_lower: np.ndarray           # (12,) rad
    pos_limit_upper: np.ndarray           # (12,) rad

    action_scale: float
    policy_dt: float
    control_dt: float
    num_observations: int
    obs_layout: tuple[str, ...]

    meta: dict

    # Pre-scale action clip, from the trainer's exported ``action_limit_lower/upper``.
    # The trainer CLIPS actions to this range before scaling, so anything wider lets the
    # policy command targets it never saw in training. Defaults are deliberately wide so an
    # older contract without the field behaves exactly as before (no silent tightening).
    action_limit_lower: float = -100.0
    action_limit_upper: float = 100.0

    # --- derived ---------------------------------------------------------
    @property
    def num_joints(self) -> int:
        return len(self.joint_order)

    @property
    def policy_frame_sign(self) -> np.ndarray:
        """(12,) of ±1: multiply a *device-frame* per-joint quantity by this to get the *policy*
        (URDF) frame, and vice-versa (the map is its own inverse). ``-1`` on the URDF-mirrored
        right-leg joints (see ``POLICY_FRAME_MIRRORED_JOINTS``), ``+1`` elsewhere.
        """
        return np.array(
            [-1.0 if n in POLICY_FRAME_MIRRORED_JOINTS else 1.0 for n in self.joint_order],
            dtype=np.float32,
        )

    def index_of(self, joint_name: str) -> int:
        return self.joint_order.index(joint_name)

    def clamp_targets(self, targets: np.ndarray) -> np.ndarray:
        """Clamp a (12,) target vector to per-joint position limits."""
        return np.clip(targets, self.pos_limit_lower, self.pos_limit_upper)

    # --- loading ---------------------------------------------------------
    @classmethod
    def load(cls, path: str | Path = DEFAULT_CONTRACT_PATH) -> "LegPolicyContract":
        data = json.loads(Path(path).read_text())
        order = list(data["canonical_joint_order"])
        by_name = {j["joint_name"]: j for j in data["joints"]}
        # Preserve canonical order regardless of list order in the file.
        rows = [by_name[n] for n in order]

        def col(key: str) -> np.ndarray:
            return np.array([r[key] for r in rows], dtype=np.float32)

        ctrl = data["control"]
        obs = data["observation"]
        act = data.get("action", {})
        return cls(
            joint_order=tuple(order),
            default_pose=col("default_pose"),
            kp=col("kp"),
            kd=col("kd"),
            effort_limit=col("effort_limit"),
            torque_constant=col("torque_constant"),
            gear_ratio=col("gear_ratio"),
            position_offset=col("position_offset"),
            pos_limit_lower=col("position_limit_lower"),
            pos_limit_upper=col("position_limit_upper"),
            action_scale=float(ctrl["action_scale"]),
            policy_dt=float(ctrl["policy_dt"]),
            control_dt=float(ctrl["control_dt"]),
            num_observations=int(obs["num_observations"]),
            obs_layout=tuple(obs["layout"]),
            meta=data.get("_meta", {}),
            action_limit_lower=float(act.get("action_limit_lower", -100.0)),
            action_limit_upper=float(act.get("action_limit_upper", 100.0)),
        )

    def summary(self) -> str:
        lines = [
            f"LegPolicyContract: {self.num_joints} joints, "
            f"policy_dt={self.policy_dt}s ({1/self.policy_dt:.0f} Hz), action_scale={self.action_scale}",
            f"  obs({self.num_observations}): {' + '.join(self.obs_layout)}",
            f"  action clip: [{self.action_limit_lower:+.1f}, {self.action_limit_upper:+.1f}] (pre-scale)",
        ]
        for i, n in enumerate(self.joint_order):
            lines.append(
                f"  [{i:2d}] {n:24s} default={self.default_pose[i]:+.4f} "
                f"kp={self.kp[i]:6.2f} kd={self.kd[i]:5.2f} eff={self.effort_limit[i]:4.1f} "
                f"lim=[{self.pos_limit_lower[i]:+.3f},{self.pos_limit_upper[i]:+.3f}]"
            )
        if self.meta.get("STATUS"):
            lines.append(f"  STATUS: {self.meta['STATUS']}")
        return "\n".join(lines)
