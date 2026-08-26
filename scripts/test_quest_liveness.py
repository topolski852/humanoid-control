#!/usr/bin/env python3
"""Offline checks for the Quest link's liveness ladder, clutch and E-STOP.

This is the most important test in the Quest work. The transport it replaces (televuer)
emitted no timestamps and no sequence numbers, and latched `motion_data_ready` true forever,
so a dead headset was indistinguishable from a still one. Every row of the ladder in
docs/QUEST_TELEOP_PLAN.md is asserted here, against a fake clock so it runs instantly::

    .venv/bin/python scripts/test_quest_liveness.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np                                              # noqa: E402

from humanoid_control.config import LegPolicyContract           # noqa: E402
from humanoid_control.layout import RobotLayout                 # noqa: E402
from humanoid_control.web import xr as xr_mod                   # noqa: E402
from humanoid_control.web.service import ControlService, SessionState  # noqa: E402

PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


class Clock:
    """Fake monotonic clock so a 1-second timeout costs no wall time."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class StubClient:
    def is_running(self):
        return True

    def get_cached_joint_state(self, name):
        return {"position": 0.0, "state": "IDLE"}

    def latest_base(self):
        return {}

    def estop_all(self):
        pass


def build(live_session: bool = False):
    """A service with the Quest holding the token, plus a QuestSource on a fake clock."""
    import os
    os.environ["HUMANOID_QUEST_ENABLE"] = "1"
    os.environ.pop("HUMANOID_GAMEPAD_ENABLE", None)
    os.environ.pop("HUMANOID_QUEST_HAND", None)
    os.environ["HUMANOID_QUEST_SCALE"] = "1.0"      # 1:1 keeps the arithmetic checkable
    os.environ["HUMANOID_QUEST_YAW_DEG"] = "0"

    svc = ControlService(
        StubClient(), LegPolicyContract.load(), config_present=True,
        layout=RobotLayout(enabled=("left_arm",), imu_expected=False),
    )
    svc._input_source = "quest"
    q = xr_mod.QuestSource(svc)
    svc.quest = q
    if live_session:
        svc._state = SessionState.ARMED
        svc._session_deadman = "quest"
    q.attach()
    return svc, q


def frame(seq, *, p=(0.0, 0.0, 0.0), trigger=0.0, tracked=True, b=False,
          session="s1", side="left"):
    f = {"seq": seq, "t": float(seq), "session": session,
         "head": {"p": [0, 1.6, 0], "q": [0, 0, 0, 1], "tracked": True}}
    f[side] = {"p": list(p), "q": [0, 0, 0, 1], "tracked": tracked,
               "trigger": trigger, "squeeze": 0.0, "stick": [0, 0],
               "a": False, "b": b, "stickPress": False}
    f["right" if side == "left" else "left"] = None
    return f


def main() -> int:
    clock = Clock()
    xr_mod.time.monotonic = clock          # patch the module's clock

    print("\n── frame conversion (verified against televuer) ─────────────────")
    # WebXR: +X right, +Y up, -Z forward.  Robot: +X forward, +Y left, +Z up.
    T_ROBOT_OPENXR = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    for name, v in [("forward (-Z)", [0, 0, -1]), ("up (+Y)", [0, 1, 0]),
                    ("right (+X)", [1, 0, 0]), ("arbitrary", [0.3, -1.2, 0.75])]:
        ours = xr_mod.webxr_to_robot(v)
        theirs = T_ROBOT_OPENXR @ np.asarray(v, dtype=float)
        check(f"webxr_to_robot matches televuer's basis change: {name}",
              np.allclose(ours, theirs), f"{ours} vs {theirs}")
    check("forward maps to +X", np.allclose(xr_mod.webxr_to_robot([0, 0, -1]), [1, 0, 0]))
    check("up maps to +Z", np.allclose(xr_mod.webxr_to_robot([0, 1, 0]), [0, 0, 1]))
    check("right maps to -Y (robot +Y is LEFT)",
          np.allclose(xr_mod.webxr_to_robot([1, 0, 0]), [0, -1, 0]))

    print("\n── the happy path: trigger drives, release stops ────────────────")
    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.0))
    check("trigger released → no run gate", svc._run_gate.is_set() is False)
    check("a frame marks the source alive", svc.source_alive("quest") is True)

    q.on_frame(frame(2, p=(0, 0, 0), trigger=0.9))
    check("trigger held → run gate set", svc._run_gate.is_set() is True)
    check("clutch anchors on the trigger's rising edge", q.status()["anchored"] is True)
    check("anchoring frame commands ~zero displacement",
          float(np.linalg.norm(svc._arm_pose_command[0])) < 1e-6)

    # Move 10 cm along WebXR -Z (forward) → robot +X.
    q.on_frame(frame(3, p=(0, 0, -0.1), trigger=0.9))
    delta = svc._arm_pose_command[0]
    check("displacement is measured from the anchor, in robot frame",
          np.allclose(delta, [0.1, 0.0, 0.0], atol=1e-6), str(delta))

    q.on_frame(frame(4, p=(0, 0, -0.1), trigger=0.0))
    check("trigger release drops the run gate", svc._run_gate.is_set() is False)
    check("trigger release discards the anchor", q.status()["anchored"] is False)

    q.on_frame(frame(5, p=(0, 0, -0.1), trigger=0.9))
    q.on_frame(frame(6, p=(0, 0, -0.1), trigger=0.9))
    check("re-press re-anchors at the NEW position (ratchet, no jump)",
          float(np.linalg.norm(svc._arm_pose_command[0])) < 1e-6,
          str(svc._arm_pose_command[0]))

    print("\n── scale ────────────────────────────────────────────────────────")
    svc, q = build()
    q.scale = 0.5
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    q.on_frame(frame(2, p=(0, 0, -0.2), trigger=0.9))
    check("scale halves the commanded displacement",
          np.allclose(svc._arm_pose_command[0], [0.1, 0, 0], atol=1e-6),
          str(svc._arm_pose_command[0]))

    print("\n── LADDER: 200 ms stall → IDLE, no E-STOP ───────────────────────")
    svc, q = build(live_session=True)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    check("gate is up before the stall", svc._run_gate.is_set() is True)
    clock.advance(0.1)
    q.tick()
    check("100 ms of silence does NOT drop the gate", svc._run_gate.is_set() is True)
    clock.advance(0.15)          # 250 ms total
    q.tick()
    check("250 ms of silence drops the run gate", svc._run_gate.is_set() is False)
    check("a stall does NOT E-STOP", svc.estop.fired is False)
    q.on_frame(frame(2, p=(0, 0, 0), trigger=0.9))
    check("the gate recovers by itself on the next frame",
          svc._run_gate.is_set() is True)

    print("\n── LADDER: 1 s silence → E-STOP ─────────────────────────────────")
    svc, q = build(live_session=True)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    clock.advance(0.5)
    q.tick()
    check("500 ms does not yet E-STOP", svc.estop.fired is False)
    clock.advance(0.6)           # 1.1 s total
    q.tick()
    check("1.1 s of silence E-STOPs a live session", svc.estop.fired is True)
    check("E-STOP latches the service", svc._state == SessionState.ESTOPPED)

    print("\n── LADDER: silence with NO live session must not E-STOP ─────────")
    svc, q = build(live_session=False)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    clock.advance(2.0)
    q.tick()
    check("silence while idle drops the gate but does NOT E-STOP",
          svc.estop.fired is False and svc._run_gate.is_set() is False)

    print("\n── LADDER: tracking loss → IDLE ─────────────────────────────────")
    svc, q = build(live_session=True)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    q.on_frame(frame(2, p=(0, 0, 0), trigger=0.9, tracked=False))
    check("untracked controller drops the run gate", svc._run_gate.is_set() is False)
    check("tracking loss does NOT E-STOP", svc.estop.fired is False)
    check("tracking loss discards the anchor", q.status()["anchored"] is False)
    q.on_frame(frame(3, p=(0, 0, 0), trigger=0.9))
    check("tracking recovery re-anchors rather than resuming the old frame",
          q.status()["anchored"] is True
          and float(np.linalg.norm(svc._arm_pose_command[0])) < 1e-6)

    print("\n── LADDER: frozen pose while the trigger is held ────────────────")
    svc, q = build(live_session=True)
    for i in range(1, 4):
        q.on_frame(frame(i, p=(0, 0, -0.05), trigger=0.9))
        clock.advance(0.05)
    check("gate is up while the pose is fresh", svc._run_gate.is_set() is True)
    for i in range(4, 20):                     # seq advances, pose IDENTICAL
        q.on_frame(frame(i, p=(0, 0, -0.05), trigger=0.9))
        clock.advance(0.05)
    check("a bit-identical pose for >500 ms drops the gate",
          svc._run_gate.is_set() is False, q.status()["reason"])
    check("a frozen pose does NOT E-STOP", svc.estop.fired is False)
    check("the source still reads alive (seq IS advancing)",
          svc.source_alive("quest") is True)
    # A frozen sender must STAY released, not flap the gate every FROZEN_S.
    for i in range(20, 40):
        q.on_frame(frame(i, p=(0, 0, -0.05), trigger=0.9))
        clock.advance(0.05)
        if svc._run_gate.is_set():
            break
    check("a frozen sender stays released (the gate does not flap)",
          svc._run_gate.is_set() is False, "gate re-engaged on a still-frozen stream")
    q.on_frame(frame(40, p=(0, 0, -0.20), trigger=0.9))    # pose finally moves
    check("the gate returns once the pose genuinely changes",
          svc._run_gate.is_set() is True, q.status()["reason"])
    # A genuinely moving controller must never trip it.
    svc, q = build(live_session=True)
    for i in range(1, 40):
        q.on_frame(frame(i, p=(0, 0, -0.05 - i * 1e-6), trigger=0.9))   # micrometre jitter
        clock.advance(0.05)
    check("micrometre jitter is NOT treated as frozen", svc._run_gate.is_set() is True)

    # Regression (found end-to-end, not by the checks above): a stall normally ENDS with the
    # operator having held still through the gap, so the first frame back carries the same
    # pose as the last frame before. If the frozen window is not reset on release, that reads
    # as a frozen sender and the link is released again the instant it recovers — the gate
    # would never come back.
    svc, q = build(live_session=True)
    q.on_frame(frame(1, p=(0, 0, -0.05), trigger=0.9))
    clock.advance(0.9)                     # stall past STALL_S, still under LOSS_S
    q.tick()
    check("stall released the gate (setup)", svc._run_gate.is_set() is False)
    q.on_frame(frame(2, p=(0, 0, -0.05), trigger=0.9))     # SAME pose, link is back
    check("recovering on an UNCHANGED pose is not mistaken for a frozen sender",
          svc._run_gate.is_set() is True, q.status()["reason"])
    check("recovery after a stall re-anchors", q.status()["anchored"] is True)

    print("\n── LADDER: new XR session discards the anchor ───────────────────")
    svc, q = build(live_session=True)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9, session="s1"))
    q.on_frame(frame(2, p=(0, 0, -0.3), trigger=0.9, session="s2"))
    check("a session-id change re-anchors instead of jumping",
          float(np.linalg.norm(svc._arm_pose_command[0])) < 1e-6,
          str(svc._arm_pose_command[0]))

    print("\n── LADDER: disconnect during a live session → E-STOP ────────────")
    svc, q = build(live_session=True)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    q.detach()
    check("closing the link during a live session E-STOPs", svc.estop.fired is True)
    check("detach marks the source not-alive", svc.source_alive("quest") is False)

    svc, q = build(live_session=False)
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    q.detach()
    check("closing the link while idle does NOT E-STOP", svc.estop.fired is False)

    print("\n── E-STOP button (B/Y), unconditional ───────────────────────────")
    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.0, b=True))
    check("B/Y E-STOPs even with the trigger released", svc.estop.fired is True)

    svc, q = build()
    svc._input_source = "xbox"          # Quest does NOT hold the token
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9, b=True))
    check("B/Y E-STOPs even when the Quest holds no input token",
          svc.estop.fired is True)
    check("a non-owner's pose command is still dropped",
          svc._arm_pose_command is None)

    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.0, b=True, tracked=False))
    check("B/Y E-STOPs even when the controller is untracked", svc.estop.fired is True)

    print("\n── bad input is rejected, loudly ────────────────────────────────")
    svc, q = build()
    q.on_frame(frame(5, p=(0, 0, 0), trigger=0.9))
    q.on_frame(frame(3, p=(0, 0, -1.0), trigger=0.9))     # replay / reorder
    check("an out-of-order seq is rejected, not acted on",
          float(np.linalg.norm(svc._arm_pose_command[0])) < 1e-6)
    check("a rejected frame is counted", q.status()["dropped"] >= 1)

    before = q.status()["dropped"]
    q.on_frame({"garbage": True})
    q.on_frame({"seq": "not-an-int"})
    check("malformed frames are counted, not raised",
          q.status()["dropped"] >= before + 2)

    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9))
    q.on_frame(frame(2, p=(float("nan"), 0, 0), trigger=0.9))
    check("a non-finite pose drops the gate instead of commanding NaN",
          svc._run_gate.is_set() is False)

    print("\n── which controller drives ──────────────────────────────────────")
    svc, q = build()
    check("defaults to the hand matching the configured arm (left)",
          q.status()["hand"] == "left", q.status()["hand"])

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
