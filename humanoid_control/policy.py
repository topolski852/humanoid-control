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
