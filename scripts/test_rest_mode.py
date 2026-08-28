#!/usr/bin/env python3
"""What the arm does when it is armed but not driving.

    .venv/bin/python scripts/test_rest_mode.py

The arm rests far more often than it drives: every released trigger, every lost tracker,
every aborted ramp, every torn-down session. It used to rest in IDLE — zero torque — which
for a 5-DOF arm is not resting, it is falling.

The reason given in the code was that DAMPING faults the firmware watchdog in ~1 s because
the daemon does not feed it. That has the relationship backwards, and the daemon's own
sources say so in three places:

    docs/HANDOFF.md         "the firmware drops to MODE_DAMPING and sets
                             ERROR_WATCHDOG_TIMEOUT ... The watchdog does not fire in
                             IDLE/DISABLED/DAMPING"
    robot.cpp:772           "The firmware's watchdog only fires in motion modes
                             (not IDLE/DAMPING/DISABLED)"
    actuator.cpp:343        "The firmware watchdog does not fire in IDLE mode"

DAMPING is where the watchdog PUTS a motor when its commands stop. It is the firmware's own
fail-safe, it needs no feed, and it holds indefinitely.

IDLE is still wanted sometimes — a damped joint is unpleasant to back-drive, so
hand-positioning, checking free play and re-zeroing all want the motors limp. So it stays
available as a deliberate choice, which is exactly what these tests pin down: DAMPING unless
somebody asks otherwise, and no path that quietly reverts.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.layout import RobotLayout                 # noqa: E402
from humanoid_control.config import LegPolicyContract           # noqa: E402
from humanoid_control.web.service import ControlError, ControlService  # noqa: E402

PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


class StubClient:
    def is_running(self): return True
    def get_cached_joint_state(self, name): return {"position": 0.0, "state": "IDLE"}
    def latest_base(self): return {}
    def estop_all(self): pass


class SpyGroup:
    """Records which rest state it was asked for, in order."""

    def __init__(self): self.calls = []
    def idle(self): self.calls.append("idle")
    def damp(self): self.calls.append("damp")


def build():
    return ControlService(
        StubClient(), LegPolicyContract.load(), config_present=True,
        layout=RobotLayout(enabled=("left_arm",), imu_expected=False),
    )


def main() -> int:
    print("\n── the default is the safe one ─────────────────────────────────")
    svc = build()
    check("a fresh service rests in DAMPING", svc.rest_mode == "damping", svc.rest_mode)

    g = SpyGroup()
    svc._rest(g)
    check("_rest damps by default", g.calls == ["damp"], str(g.calls))

    print("\n── IDLE is available, but only on request ──────────────────────")
    svc.set_rest_mode("idle")
    g = SpyGroup(); svc._rest(g)
    check("choosing idle makes _rest go limp", g.calls == ["idle"], str(g.calls))
    check("rest_mode reports the choice", svc.rest_mode == "idle")

    svc.set_rest_mode("damping")
    g = SpyGroup(); svc._rest(g)
    check("switching back damps again", g.calls == ["damp"], str(g.calls))

    print("\n── bad input is refused, not coerced ───────────────────────────")
    for bad in ("DAMPING", "Idle", "brake", "", "none", "hold"):
        try:
            svc.set_rest_mode(bad)
            check(f"{bad!r} is refused", False, "accepted")
        except ControlError as exc:
            check(f"{bad!r} is refused", exc.status == 400, str(exc))
    check("a refused change leaves the mode untouched", svc.rest_mode == "damping",
          svc.rest_mode)

    print("\n── it survives being set to what it already is ─────────────────")
    svc.set_rest_mode("damping")
    g = SpyGroup(); svc._rest(g)
    check("setting the current mode is a no-op, not a reset", g.calls == ["damp"])

    print("\n── the telemetry the UI card reads ─────────────────────────────")
    snap = svc.telemetry_snapshot()
    ctl = snap.get("control") or {}
    check("control block carries 'rest'", ctl.get("rest") == "damping", str(ctl.get("rest")))
    svc.set_rest_mode("idle")
    check("...and it tracks changes",
          (svc.telemetry_snapshot().get("control") or {}).get("rest") == "idle")

    print("\n── a restart returns to safe ───────────────────────────────────")
    # Deliberately not persisted. Inheriting IDLE across a restart would mean an operator who
    # went limp once to reposition the arm gets a falling arm on the next boot, with nothing
    # on screen to explain why.
    svc.set_rest_mode("idle")
    check("a new service does not inherit idle", build().rest_mode == "damping")

    print("\n── the interface really has both, spelled as expected ──────────")
    from humanoid_control.interface import JointGroupInterface
    for meth in ("idle", "damp", "enable_position", "send_targets"):
        check(f"JointGroupInterface.{meth} exists", hasattr(JointGroupInterface, meth))

    print("\n── every rest path in the worker goes through _rest ────────────")
    # The four sites were four separate group.idle() calls, each carrying the same wrong
    # watchdog comment. If a new one appears that calls group.idle() directly, it will not
    # honour the operator's choice — and this is the check that notices.
    src = (Path(__file__).resolve().parent.parent
           / "humanoid_control" / "web" / "service.py").read_text()
    worker = src.split("def _deadman_worker", 1)[1]
    check("no bare group.idle() left in the deadman worker",
          "group.idle()" not in worker,
          worker.count("group.idle()") and "still present" or "")
    check("the worker rests via _rest", worker.count("self._rest(group)") >= 4,
          f"{worker.count('self._rest(group)')} call sites")

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
