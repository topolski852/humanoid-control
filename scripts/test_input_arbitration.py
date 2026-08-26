#!/usr/bin/env python3
"""Offline checks for input-source arbitration and per-source deadman liveness.

No robot, no daemon, no network — builds a ControlService against a stub client and drives
its state machine directly. Run it before and after touching anything in the deadman path::

    .venv/bin/python scripts/test_input_arbitration.py

The repo has no pytest; these are plain asserts in the style of ``scripts/smoke_test.py`` so
the check has no dependency that a bench machine might not have.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.config import LegPolicyContract          # noqa: E402
from humanoid_control.layout import RobotLayout                 # noqa: E402
from humanoid_control.web import service as svc_mod             # noqa: E402
from humanoid_control.web.service import ControlError, ControlService, SessionState  # noqa: E402

PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail and not cond else ''}")


class StubClient:
    """The minimum DaemonClient surface ControlService touches while idle."""

    def is_running(self) -> bool:
        return True

    def get_cached_joint_state(self, name):
        return {"position": 0.0, "state": "IDLE"}

    def latest_base(self):
        return {}


def build(gamepad_enabled: bool) -> ControlService:
    import os
    if gamepad_enabled:
        os.environ["HUMANOID_GAMEPAD_ENABLE"] = "1"
    else:
        os.environ.pop("HUMANOID_GAMEPAD_ENABLE", None)
    os.environ.pop("HUMANOID_QUEST_ENABLE", None)
    return ControlService(
        StubClient(), LegPolicyContract.load(), config_present=True,
        layout=RobotLayout(enabled=("left_arm",), imu_expected=False),
    )


def main() -> int:
    print("\n── per-source liveness ──────────────────────────────────────────")
    s = build(gamepad_enabled=True)

    check("default input source is xbox when the gamepad is enabled",
          s.input_source == "xbox", s.input_source)

    # THE REGRESSION THIS EXISTS FOR: a live browser must not vouch for a dead gamepad.
    s.mark_source_alive("web")
    check("browser alive does NOT make a silent gamepad look alive",
          s.deadman_ok() is False,
          "deadman_ok() true with only 'web' beating while xbox holds the token")

    s.mark_source_alive("xbox")
    check("gamepad alive satisfies the deadman while it holds the token",
          s.deadman_ok() is True)

    s.drop_source("xbox")
    check("dropping the gamepad source is immediately not-alive",
          s.deadman_ok() is False)

    s.mark_source_alive("xbox")
    s._sources["xbox"] = time.monotonic() - (svc_mod._DEADMAN_TIMEOUT_S + 0.1)
    check("a stale gamepad heartbeat expires after the deadman timeout",
          s.source_alive("xbox") is False)

    print("\n── the session keeps the deadman that armed it ──────────────────")
    s = build(gamepad_enabled=True)
    s.mark_source_alive("xbox")
    check("idle: deadman of record is the token holder",
          s.deadman_source() == "xbox", s.deadman_source())
    s._session_deadman = "quest"
    check("live session: deadman of record is whoever armed it",
          s.deadman_source() == "quest", s.deadman_source())
    s._session_deadman = None

    print("\n── arbitration: only the token holder commands ──────────────────")
    s = build(gamepad_enabled=True)

    s.set_run_gate(True, source="xbox")
    check("token holder may set the run gate", s._run_gate.is_set() is True)
    s.set_run_gate(False, source="xbox")

    s.set_run_gate(True, source="quest")
    check("non-owner run-gate write is DROPPED", s._run_gate.is_set() is False)
    check("non-owner write is counted, not silent",
          s._ignored_writes.get("quest") == 1, str(s._ignored_writes))

    s.set_arm_command(1.0, 0.0, 0.0, 0.0, source="quest")
    check("non-owner arm command is dropped",
          float(s._arm_command[0]) == 0.0, str(s._arm_command))
    s.set_arm_command(1.0, 0.0, 0.0, 0.0, source="xbox")
    check("token holder's arm command lands",
          float(s._arm_command[0]) == 1.0, str(s._arm_command))

    s.set_walk_command(0.5, 0.0, 0.0, source="quest")
    check("non-owner walk command is dropped", float(s._command[0]) == 0.0)

    print("\n── switching the control method ─────────────────────────────────")
    s = build(gamepad_enabled=True)
    try:
        s.set_input_source("quest")
        check("unavailable source is refused", False, "no error raised")
    except ControlError as exc:
        check("unavailable source is refused (HUMANOID_QUEST_ENABLE unset)",
              exc.status == 409, str(exc))

    check("available sources reflect what is actually enabled",
          s.available_input_sources() == ["xbox", "web"], str(s.available_input_sources()))

    s.set_input_source("web")
    check("switching source while idle is allowed", s.input_source == "web")

    s._state = SessionState.ARMED
    try:
        s.set_input_source("xbox")
        check("switching source mid-session is refused", False, "no error raised")
    except ControlError as exc:
        check("switching source mid-session is refused",
              exc.status == 409 and "Disarm" in str(exc), str(exc))
    s._state = SessionState.CONNECTED

    print("\n── E-STOP is never gated by the token ───────────────────────────")
    s = build(gamepad_enabled=True)
    s.set_input_source("web")            # xbox no longer holds the token
    s.trigger_estop("test-nonowner")
    check("a non-owner source can still E-STOP", s.estop.fired is True)
    check("E-STOP moves the service to ESTOPPED", s._state == SessionState.ESTOPPED)

    print("\n── arm tick rate is independent of the leg policy ───────────────")
    check("arm loop defaults to 50 Hz, not the policy's 25 Hz",
          svc_mod._ARM_HZ == 50.0, str(svc_mod._ARM_HZ))
    contract = LegPolicyContract.load()
    check("leg policy dt is unchanged at 25 Hz",
          abs(contract.policy_dt - 0.04) < 1e-9, str(contract.policy_dt))

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
