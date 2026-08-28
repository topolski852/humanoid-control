"""
JointGroupInterface — thin adapter over DaemonClient for an ordered group of joints.

Everything above this layer works in fixed-order (N,) numpy vectors; this class is the
only place that maps to/from per-joint daemon calls. Reads use the telemetry cache
(``get_cached_joint_state`` — no round-trip); writes use ``set_position`` (display-frame
rad, the same frame the policy works in).

``LegInterface`` is the 12-leg specialization the policy path uses: it takes its joint order
straight from the policy contract, so the sim↔real interface stays the single source of truth
for anything the trained policy touches. Other groups (an arm, a whole configured robot) pass
their joint list explicitly — see ``humanoid_control.layout``.
"""
from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from .config import LegPolicyContract
from .daemon import DaemonClient


class JointOfflineError(RuntimeError):
    pass


class JointFaultError(RuntimeError):
    pass


class JointGroupInterface:
    def __init__(self, client: DaemonClient, joints: Iterable[str]):
        self.client = client
        self.joints = list(joints)
        self._n = len(self.joints)

    # --- reads -----------------------------------------------------------
    def read_states(self, *, require_online: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """Return (joint_pos(N), joint_vel(N)) in group order from the telemetry cache."""
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
        """Raise if any joint in the group is offline or reporting a firmware error."""
        for s in self.joint_status():
            if s["state"] in (None, "OFFLINE"):
                raise JointOfflineError(f"{s['name']} is {s['state']}")
            if s["error"]:
                raise JointFaultError(f"{s['name']} error=0x{int(s['error']):04x}")

    # --- writes ----------------------------------------------------------
    def send_targets(self, targets: np.ndarray) -> None:
        """Send (N,) display-frame position targets, one SET_POSITION per joint."""
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

    def damp(self) -> None:
        """Set the group to DAMPING: powered viscous resistance — the motor fights motion
        (hard to back-drive) but holds no position target. The default 'armed but
        deadman-released' rest state, because IDLE is zero torque and a raised limb falls.

        REQUIRES A DAEMON THAT FEEDS IT. The firmware watchdog runs in DAMPING; an unfed
        joint faults ERROR_WATCHDOG_TIMEOUT (0x0040) in about a second and the session
        E-STOPs. Actuator::tick() sends DAMPING joints a PDO2 every 10th tick for exactly
        this reason. Against an older daemon build this method looks like it works and then
        takes the session down a second later.
        """
        for name in self.joints:
            self.client.set_mode(name, "DAMPING")

    def disable(self) -> None:
        """Set the group to DISABLED (PWM off, silent) — used on disconnect."""
        for name in self.joints:
            self.client.set_mode(name, "DISABLED")


class LegInterface(JointGroupInterface):
    """The 12 leg joints, ordered by the policy contract.

    Kept as its own type because the policy path is contract-bound: the runner, the observation
    layout and the action mapping all assume exactly these joints in exactly this order.
    """

    def __init__(self, client: DaemonClient, contract: LegPolicyContract):
        super().__init__(client, contract.joint_order)
        self.contract = contract
