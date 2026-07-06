"""
LegInterface — thin adapter over DaemonClient for the 12 leg joints in canonical order.

Everything above this layer works in fixed-order (12,) numpy vectors; this class is the
only place that maps to/from per-joint daemon calls. Reads use the telemetry cache
(``get_cached_joint_state`` — no round-trip); writes use ``set_position`` (display-frame
rad, the same frame the policy works in).
"""
from __future__ import annotations

import numpy as np

from .config import LegPolicyContract
from .daemon import DaemonClient


class JointOfflineError(RuntimeError):
    pass


class JointFaultError(RuntimeError):
    pass


class LegInterface:
    def __init__(self, client: DaemonClient, contract: LegPolicyContract):
        self.client = client
        self.contract = contract
        self.joints = list(contract.joint_order)
        self._n = len(self.joints)

    # --- reads -----------------------------------------------------------
    def read_states(self, *, require_online: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """Return (joint_pos(12), joint_vel(12)) in canonical order from the telemetry cache."""
        pos = np.zeros(self._n, dtype=np.float32)
        vel = np.zeros(self._n, dtype=np.float32)
        for i, name in enumerate(self.joints):
            st = self.client.get_cached_joint_state(name)
            if st is None:
                if require_online:
                    raise JointOfflineError(f"{name}: no cached telemetry (offline?)")
                pos[i] = np.nan
                vel[i] = np.nan
                continue
            pos[i] = st.get("position", np.nan)
            vel[i] = st.get("velocity", np.nan)
        return pos, vel

    def joint_status(self) -> list[dict]:
        """Per-joint {name, state, error, mode} for health checks/logging."""
        out = []
        for name in self.joints:
            st = self.client.get_cached_joint_state(name) or {}
            out.append({
                "name": name,
                "state": st.get("state") or st.get("joint_state"),
                "error": st.get("error", 0),
                "mode": st.get("mode"),
            })
        return out

    def check_health(self) -> None:
        """Raise if any leg joint is offline or reporting a firmware error."""
        for s in self.joint_status():
            if s["state"] in (None, "OFFLINE"):
                raise JointOfflineError(f"{s['name']} is {s['state']}")
            if s["error"]:
                raise JointFaultError(f"{s['name']} error=0x{int(s['error']):04x}")

    # --- writes ----------------------------------------------------------
    def send_targets(self, targets: np.ndarray) -> None:
        """Send (12,) display-frame position targets, one SET_POSITION per joint."""
        targets = np.asarray(targets, dtype=np.float32).reshape(-1)
        assert targets.shape == (self._n,), targets.shape
        for name, t in zip(self.joints, targets):
            self.client.set_position(name, float(t))

    def enable_position(self) -> None:
        for name in self.joints:
            self.client.set_mode(name, "POSITION")

    def idle(self) -> None:
        for name in self.joints:
            self.client.set_mode(name, "IDLE")

    def disable(self) -> None:
        """Set the leg joints to DISABLED (PWM off, silent) — used on disconnect."""
        for name in self.joints:
            self.client.set_mode(name, "DISABLED")
