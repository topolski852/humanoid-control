"""
PolicyRunner — the legs-only policy control loop.

Bring-up order (enforced by the phases below): connect (wake+config) → verify online →
enable POSITION → **ramp** to default_pose → run policy at ``policy_dt``. Never steps to a
target; always clamps to limits; E-stop checked every tick.

⚠️ MOTION: ``prepare()`` (ramp) and ``run()`` move the robot. Only call them with the
user present and the robot supported/gantried. Scripts gate this behind an explicit flag.
"""
from __future__ import annotations

import sys
import time

import numpy as np

from .action import ActionMapper
from .base_state import BaseStateSource, UprightStubBaseState
from .config import LegPolicyContract
from .daemon import DaemonClient
from .interface import LegInterface
from .observation import ObservationBuilder
from .policy import Policy
from .safety import EstopController, ramp_to_pose


class PolicyRunner:
    def __init__(
        self,
        client: DaemonClient,
        contract: LegPolicyContract,
        policy: Policy,
        *,
        base_source: BaseStateSource | None = None,
        command: np.ndarray | None = None,
        estop: EstopController | None = None,
        ramp_seconds: float = 4.0,
        require_valid_base: bool = False,
    ):
        self.client = client
        self.contract = contract
        self.policy = policy
        self.legs = LegInterface(client, contract)
        self.obs_builder = ObservationBuilder(contract)
        self.action_mapper = ActionMapper(contract)
        self.base_source = base_source or UprightStubBaseState()
        self.command = np.asarray(command if command is not None else np.zeros(3), dtype=np.float32)
        self.estop = estop or EstopController(client)
        self.ramp_seconds = ramp_seconds
        self.require_valid_base = require_valid_base
        self._warned_invalid_base = False

    # --- lifecycle -------------------------------------------------------
    async def connect(self) -> None:
        """Open sockets, wake + configure joints (no motion). Then verify legs are online."""
        await self.client.start()
        await self.client.connect()          # apply_all_configs: NMT IDLE + delta-write config
        time.sleep(0.3)
        self.legs.check_health()             # raises if any leg offline/faulted
        print("[runner] connected; all 12 leg joints online and healthy.", file=sys.stderr)

    def prepare(self, should_abort=None) -> bool:
        """MOTION: enable POSITION and ramp from current pose to default_pose.

        Returns False if aborted. Seeds prev_action=0 so the first policy obs is consistent
        with holding default_pose. ``should_abort`` (optional) is OR'd with the E-stop so a
        caller can bail the ramp on another condition (e.g. a released deadman trigger); it
        must return True to abort. Enabling POSITION from DAMPING is jerk-free because the
        daemon seeds the firmware position target at the current pose on the mode change.
        """
        pos, _ = self.legs.read_states()
        goal = self.contract.clamp_targets(self.contract.default_pose)
        self.legs.enable_position()
        # Seed the hold at the *current* pose so enabling doesn't jerk, then ramp.
        self.legs.send_targets(pos)
        time.sleep(0.05)
        ctrl_hz = 1.0 / self.contract.control_dt
        abort = (lambda: self.estop.fired or bool(should_abort and should_abort()))
        ok = ramp_to_pose(
            start=pos, goal=goal, send=self.legs.send_targets,
            duration_s=self.ramp_seconds, rate_hz=min(ctrl_hz, 100.0),
            should_abort=abort,
        )
        self.action_mapper.reset()
        if ok:
            print("[runner] ramped to default_pose; holding.", file=sys.stderr)
        return ok

    def step(self) -> None:
        """One policy tick: read → obs → policy → action → clamped targets → send."""
        base = self.base_source.get()
        if not base.valid and not self._warned_invalid_base:
            msg = "[runner] base state INVALID (no IMU data)."
            if self.require_valid_base:
                raise RuntimeError(msg + " require_valid_base=True → refusing to run.")
            print(msg + " Running upright-stub; keep the robot supported.", file=sys.stderr)
            self._warned_invalid_base = True

        joint_pos, joint_vel = self.legs.read_states()
        obs = self.obs_builder.build(
            joint_pos=joint_pos, joint_vel=joint_vel, base_state=base,
            command=self.command, prev_action=self.action_mapper.prev_action,
        )
        action = self.policy.forward(obs)
        targets = self.action_mapper.map(action)   # clipped, scaled, clamped to limits
        self.legs.send_targets(targets)

    async def run(self, max_seconds: float | None = None) -> None:
        """MOTION: run the policy loop at policy_dt until E-stop / duration / fault."""
        import asyncio
        dt = self.contract.policy_dt
        t0 = time.monotonic()
        next_tick = t0
        print(f"[runner] policy loop @ {1/dt:.0f} Hz. E-stop: ENTER/'q' or Ctrl-C.", file=sys.stderr)
        try:
            while not self.estop.fired:
                if max_seconds is not None and (time.monotonic() - t0) >= max_seconds:
                    print("[runner] max_seconds reached.", file=sys.stderr)
                    break
                self.legs.check_health()       # trips on fault → we IDLE in finally
                self.step()
                next_tick += dt
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    await asyncio.sleep(sleep)
                else:
                    next_tick = time.monotonic()   # overran; rebase
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Clean stop: legs → IDLE (zero torque). E-stop path is separate (estop_all)."""
        try:
            self.legs.idle()
            print("[runner] legs set to IDLE.", file=sys.stderr)
        except Exception as exc:
            print(f"[runner] idle() error: {exc}; firing estop.", file=sys.stderr)
            self.estop.trigger("shutdown-fallback")
