#!/usr/bin/env python3
"""What the arm does when it is armed but not driving.

    .venv/bin/python scripts/test_rest_mode.py

The arm rests far more often than it drives: every released trigger, every lost tracker,
every aborted ramp, every torn-down session. It used to rest in IDLE — zero torque — which
for a 5-DOF arm is not resting, it is falling.

DAMPING is now the default, and it took a daemon change to get there.

`Actuator::tick()` used to call `send_pdo2` only in ENABLED, so a DAMPING joint received no
frames at all and the firmware watchdog expired after 1000 ms — every armed session E-STOPped
within seconds on ERROR_WATCHDOG_TIMEOUT (0x0040), all five joints, twice measured. Polling
was not the answer either: DAMPING was already in the slow-poll gate getting an SDO read every
100 ms and faulted regardless, which is how we know only a PDO resets the counter. The daemon
now feeds DAMPING joints; see scripts/verify_damping_feed.py for the hardware proof.

These tests cover the Python side, which cannot see any of that: one code path for every rest
transition, both modes reachable, bad input refused, and the choice never silently persisted
across a restart.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.layout import RobotLayout                 # noqa: E402
from humanoid_control.config import LegPolicyContract           # noqa: E402
from humanoid_control.web.service import (ControlError, ControlService,  # noqa: E402
                                          SessionState)

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
    print("\n── the default is the one that does not drop the arm ───────────")
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

    print("\n── a restart returns to the default ────────────────────────────")
    # Deliberately not persisted. An operator who went limp once to reposition the arm would
    # otherwise get a falling arm on the next boot, with nothing on screen to explain why.
    svc.set_rest_mode("idle")
    check("a new service does not inherit the last choice", build().rest_mode == "damping")

    print("\n── the interface really has both, spelled as expected ──────────")
    from humanoid_control.interface import JointGroupInterface
    for meth in ("idle", "damp", "enable_position", "send_targets"):
        check(f"JointGroupInterface.{meth} exists", hasattr(JointGroupInterface, meth))

    print("\n── set_joint_mode: the immediate override ──────────────────────")
    # The Rest state card has two halves that are easy to conflate. set_rest_mode is policy
    # for the NEXT release and deliberately leaves a resting joint alone; set_joint_mode
    # re-commands the joint now. Without the second one, an operator whose arm is resting in
    # DAMPING has no way to make it limp — changing the default appears to do nothing,
    # because nothing has triggered a rest transition since.
    import humanoid_control.web.service as svc_mod
    svc = build()
    calls = []

    class RecordingGroup(SpyGroup):
        def __init__(self, client, joints):
            super().__init__()
            self.joints = list(joints)
            calls.append(self)

    real_group = svc_mod.JointGroupInterface
    svc_mod.JointGroupInterface = RecordingGroup
    try:
        svc._state = SessionState.CONNECTED
        out = svc.set_joint_mode("idle")
        check("idle is commanded immediately",
              calls and calls[-1].calls == ["idle"], str(calls[-1].calls if calls else None))
        check("it reports what it touched",
              out.get("mode") == "idle" and out.get("joints") == 5, str(out))

        calls.clear()
        svc.set_joint_mode("damping")
        check("damping is commanded immediately", calls[-1].calls == ["damp"])

        print("\n── ...but never while the arm is driving ───────────────────────")
        for st in (SessionState.HOLDING, SessionState.RUNNING):
            svc._state = st
            calls.clear()
            try:
                svc.set_joint_mode("idle")
                check(f"refused while {st.name}", False, "accepted")
            except ControlError as exc:
                check(f"refused while {st.name}", exc.status == 409 and not calls, str(exc))

        svc._state = SessionState.ARMED
        calls.clear()
        svc.set_joint_mode("idle")
        check("ARMED-but-resting IS allowed (the whole point)", calls[-1].calls == ["idle"])

        print("\n── it validates like the policy setter does ────────────────────")
        svc._state = SessionState.CONNECTED
        for bad in ("DAMPING", "Idle", "limp", ""):
            calls.clear()
            try:
                svc.set_joint_mode(bad)
                check(f"{bad!r} is refused", False, "accepted")
            except ControlError as exc:
                check(f"{bad!r} is refused", exc.status == 400 and not calls)

        print("\n── the override does not change the policy ─────────────────────")
        svc.set_rest_mode("damping")
        svc.set_joint_mode("idle")
        check("going limp now leaves rest_mode alone", svc.rest_mode == "damping",
              svc.rest_mode)
        g = SpyGroup(); svc._rest(g)
        check("...so the next release still damps", g.calls == ["damp"], str(g.calls))
    finally:
        svc_mod.JointGroupInterface = real_group

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
