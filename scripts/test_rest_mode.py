#!/usr/bin/env python3
"""What the arm does when it is armed but not driving.

    .venv/bin/python scripts/test_rest_mode.py

The arm rests far more often than it drives: every released trigger, every lost tracker,
every aborted ramp, every torn-down session. It used to rest in IDLE — zero torque — which
for a 5-DOF arm is not resting, it is falling.

DAMPING is the right rest state and is NOT yet the default, which needs explaining.

docs/HANDOFF.md and two C++ comments all say the firmware watchdog does not fire in
IDLE/DISABLED/DAMPING. On this hardware that is out of date. Measured: with rest set to
DAMPING, every armed session E-STOPped within seconds on ERROR_WATCHDOG_TIMEOUT (0x0040)
across all five joints. Reading the code rather than the comments says why —
`Actuator::tick()` calls `send_pdo2` only in ENABLED, so a DAMPING joint receives no frames
at all and nothing resets the watchdog counter.

So the default stays IDLE until the daemon feeds DAMPING, and these tests pin the parts that
are already right: one code path for every rest transition, both modes reachable, bad input
refused, and the choice never silently persisted across a restart.
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
    print("\n── the default is the one the firmware can actually hold ───────")
    # IDLE, and not because it is better — a limp arm falls. Because DAMPING trips
    # ERROR_WATCHDOG_TIMEOUT on this firmware until the daemon feeds it. See the docstring.
    svc = build()
    check("a fresh service rests in IDLE", svc.rest_mode == "idle", svc.rest_mode)

    g = SpyGroup()
    svc._rest(g)
    check("_rest idles by default", g.calls == ["idle"], str(g.calls))

    print("\n── DAMPING is reachable, for when the daemon feeds it ──────────")
    svc.set_rest_mode("damping")
    g = SpyGroup(); svc._rest(g)
    check("choosing damping makes _rest damp", g.calls == ["damp"], str(g.calls))
    check("rest_mode reports the choice", svc.rest_mode == "damping")

    svc.set_rest_mode("idle")
    g = SpyGroup(); svc._rest(g)
    check("switching back idles again", g.calls == ["idle"], str(g.calls))

    print("\n── bad input is refused, not coerced ───────────────────────────")
    for bad in ("DAMPING", "Idle", "brake", "", "none", "hold"):
        try:
            svc.set_rest_mode(bad)
            check(f"{bad!r} is refused", False, "accepted")
        except ControlError as exc:
            check(f"{bad!r} is refused", exc.status == 400, str(exc))
    check("a refused change leaves the mode untouched", svc.rest_mode == "idle",
          svc.rest_mode)

    print("\n── it survives being set to what it already is ─────────────────")
    svc.set_rest_mode("idle")
    g = SpyGroup(); svc._rest(g)
    check("setting the current mode is a no-op, not a reset", g.calls == ["idle"])

    print("\n── the telemetry the UI card reads ─────────────────────────────")
    snap = svc.telemetry_snapshot()
    ctl = snap.get("control") or {}
    check("control block carries 'rest'", ctl.get("rest") == "idle", str(ctl.get("rest")))
    svc.set_rest_mode("damping")
    check("...and it tracks changes",
          (svc.telemetry_snapshot().get("control") or {}).get("rest") == "damping")

    print("\n── a restart returns to the default ────────────────────────────")
    # Deliberately not persisted, in both directions. Right now that means a restart cannot
    # leave you in the mode that E-STOPs; once the daemon feeds DAMPING and the default flips,
    # it will mean a restart cannot leave you with an arm that falls.
    svc.set_rest_mode("damping")
    check("a new service does not inherit the last choice", build().rest_mode == "idle")

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
