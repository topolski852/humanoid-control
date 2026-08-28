"""
Policy loaders.

``Policy.forward(obs: (num_obs,)) -> action: (12,)``.

- ``ZeroPolicy``    — action=0 → target holds default_pose. Use to validate the full
  obs/action/command path with no learned net (Milestone 4).
- ``OnnxPolicy`` / ``TorchPolicy`` — load a trained checkpoint (Milestone 5). Adapted
  from Berkeley ``rl_controller.py`` but with **no** joint-order/obs assumptions baked in:
  obs is assembled by ``ObservationBuilder`` and the action is returned raw for
  ``ActionMapper``. onnxruntime / torch are imported lazily so the package works without
  them installed.
"""
from __future__ import annotations

import json
import math

from abc import ABC, abstractmethod

import numpy as np


class Policy(ABC):
    num_actions: int = 12

    @abstractmethod
    def forward(self, obs: np.ndarray) -> np.ndarray:
        ...


class ZeroPolicy(Policy):
    """Always returns zeros → ActionMapper yields exactly default_pose (identity check)."""

    def __init__(self, num_actions: int = 12):
        self.num_actions = num_actions

    def forward(self, obs: np.ndarray) -> np.ndarray:
        return np.zeros(self.num_actions, dtype=np.float32)


class ConstantPolicy(Policy):
    """Returns a fixed action vector (handy for probing a single-joint response)."""

    def __init__(self, action: np.ndarray):
        self._action = np.asarray(action, dtype=np.float32).reshape(-1)
        self.num_actions = self._action.shape[0]

    def forward(self, obs: np.ndarray) -> np.ndarray:
        return self._action.copy()


class OnnxPolicy(Policy):
    def __init__(self, checkpoint_path: str, num_actions: int = 12):
        import onnxruntime as ort  # lazy
        self.num_actions = num_actions
        self._sess = ort.InferenceSession(checkpoint_path)
        self._input = self._sess.get_inputs()[0].name

    def forward(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        out = self._sess.run(None, {self._input: x})[0]
        return np.asarray(out, dtype=np.float32).reshape(-1)[: self.num_actions]


class TorchPolicy(Policy):
    def __init__(self, checkpoint_path: str, device: str = "cpu", num_actions: int = 12):
        import torch  # lazy
        self._torch = torch
        self.num_actions = num_actions
        self._device = device
        self._model = torch.load(checkpoint_path, map_location=device)
        self._model.eval()

    def forward(self, obs: np.ndarray) -> np.ndarray:
        torch = self._torch
        x = torch.from_numpy(np.asarray(obs, dtype=np.float32)).unsqueeze(0).to(self._device)
        with torch.no_grad():
            out = self._model(x)
        return out.detach().cpu().squeeze(0).numpy().astype(np.float32)[: self.num_actions]


def load_policy(checkpoint_path: str, **kwargs) -> Policy:
    """Pick a loader by file extension."""
    p = checkpoint_path.lower()
    if p.endswith(".onnx"):
        return OnnxPolicy(checkpoint_path, **kwargs)
    if p.endswith((".pt", ".pth", ".jit")):
        return TorchPolicy(checkpoint_path, **kwargs)
    raise ValueError(f"Unrecognized policy format: {checkpoint_path} (want .onnx/.pt)")


# ── bundle compatibility ─────────────────────────────────────────────────────
#
# A trained policy is only valid against the gains, stand pose and timing it was trained with.
# The runtime does NOT switch contracts when you switch policy: the network comes from the
# selected bundle, everything else comes from configs/leg_policy_params.json. So picking a
# bundle trained at different gains silently runs it against a robot it has never seen —
# humanoid-policy's deploy README puts it plainly: "switching policy without switching
# defaults+gains makes the robot snap to the wrong reference."
#
# Nothing checked this. A mismatched bundle loaded fine and failed — if at all — as an
# onnxruntime shape error on the FIRST step, which is after the ramp has already put the robot
# in the stand pose. This is the preflight that turns that into a refusal.

# Gains are compared exactly: they are flashed to the ESCs and are either right or not.
_GAIN_TOL = 1e-6
# Poses in radians. 0.5 deg of slop absorbs float32 round-tripping through JSON without
# admitting a genuinely different stand pose.
_POSE_TOL = math.radians(0.5)


def bundle_issues(contract_path: str | None, contract) -> list[str]:
    """Why a policy bundle must not be run against ``contract``. Empty list = compatible.

    THE FRAME TRAP. The trainer exports in URDF frame, where the right leg is mirrored; the
    runtime works in device frame. `right_hip_roll`, `right_hip_yaw` and `right_ankle_roll`
    therefore have OPPOSITE SIGNS in the two, by design — `contract.policy_frame_sign` is the
    map. Comparing default poses without applying it flags every correct bundle (a stand pose
    of -0.11 against +0.11) while saying nothing about a genuinely wrong one. Measured: it
    would reject both the live policy and smooth A and accept nothing.

    A bundle with no contract file is NOT rejected. Loose weight files and older exports are
    legitimately contract-less, and refusing to run anything unverifiable would break the
    fallback path in /api/policies. Unknown is reported as unknown, not as safe.
    """
    if not contract_path:
        return []
    try:
        with open(contract_path) as fh:
            b = json.load(fh)
    except Exception as exc:                                     # noqa: BLE001
        return [f"contract unreadable ({exc})"]

    issues: list[str] = []

    order = tuple(b.get("canonical_joint_order") or ())
    if order and order != tuple(contract.joint_order):
        return ["joint order differs from the runtime contract"]   # nothing else is comparable

    ctl = b.get("control") or {}
    for key, mine in (("action_scale", contract.action_scale),
                      ("policy_dt", contract.policy_dt),
                      ("control_dt", contract.control_dt)):
        theirs = ctl.get(key)
        if theirs is not None and abs(float(theirs) - float(mine)) > 1e-9:
            issues.append(f"{key} {theirs} != runtime {mine}")

    obs = (b.get("observation") or {}).get("num_observations")
    if obs is not None and int(obs) != int(contract.num_observations):
        issues.append(f"{obs} observations != runtime {contract.num_observations}")

    rows = b.get("joints") or []
    by_name = ({j["joint_name"]: j for j in rows} if isinstance(rows, list) else rows)
    sign = contract.policy_frame_sign
    bad_gain, bad_pose = [], []
    for i, name in enumerate(contract.joint_order):
        j = by_name.get(name)
        if not j:
            continue
        short = name.replace("_joint", "")
        for key, mine in (("kp", contract.kp[i]), ("kd", contract.kd[i])):
            theirs = j.get(key)
            if theirs is not None and abs(float(theirs) - float(mine)) > _GAIN_TOL:
                bad_gain.append(f"{short} {key} {theirs:g}!={float(mine):g}")
        theirs = j.get("default_pose")
        if theirs is not None:
            # Bundle is URDF frame; bring it into device frame before comparing.
            if abs(float(theirs) * float(sign[i]) - float(contract.default_pose[i])) > _POSE_TOL:
                bad_pose.append(short)

    if bad_gain:
        issues.append(f"trained at different gains ({len(bad_gain)}): " + ", ".join(bad_gain[:3])
                      + (" …" if len(bad_gain) > 3 else ""))
    if bad_pose:
        issues.append("different stand pose: " + ", ".join(bad_pose[:4])
                      + (" …" if len(bad_pose) > 4 else ""))
    return issues
