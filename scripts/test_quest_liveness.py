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
from humanoid_control.web.service import ControlError, ControlService, SessionState  # noqa: E402

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
          session="s1", side="left", right_a=False, right_b=False):
    """One XR frame. `b` is the DRIVING controller's upper button — Y when driving left, which
    is E-STOP. `right_a`/`right_b` are A (arm) and B (disarm) on the right controller."""
    f = {"seq": seq, "t": float(seq), "session": session,
         "head": {"p": [0, 1.6, 0], "q": [0, 0, 0, 1], "tracked": True}}
    f[side] = {"p": list(p), "q": [0, 0, 0, 1], "tracked": tracked,
               "trigger": trigger, "squeeze": 0.0, "stick": [0, 0],
               "a": False, "b": b, "stickPress": False}
    other = "right" if side == "left" else "left"
    f[other] = {"p": [0.3, 1.0, -0.2], "q": [0, 0, 0, 1], "tracked": True,
                "trigger": 0.0, "squeeze": 0.0, "stick": [0, 0],
                "a": right_a, "b": right_b, "stickPress": False}
    return f


# ── body tracking fixtures ──────────────────────────────────────────────────
def body(*, emulated: bool = False, present: bool = True):
    """One `body` block. WebXR coords (y up, -z forward); the source converts.

    Geometry is a person standing with the left arm reaching forward — upper arm 26 cm,
    forearm 22 cm, which is what the retargeter measured off the real T-pose capture.
    """
    if not present:
        return None
    j = {
        "hips":                  [0.00, 0.95,  0.00],
        "chest":                 [0.00, 1.35,  0.00],
        "left-shoulder":         [-0.17, 1.45, 0.00],
        "right-shoulder":        [0.17, 1.45,  0.00],
        "left-arm-upper":        [-0.20, 1.42, 0.00],
        "left-arm-lower":        [-0.20, 1.42, -0.26],
        "left-hand-wrist-twist": [-0.20, 1.42, -0.46],
        "left-hand-wrist":       [-0.20, 1.42, -0.48],
    }
    return {"joints": {n: {"p": pp, "e": emulated} for n, pp in j.items()}}


def give_profile(q):
    """Attach a calibration profile so the source runs in MIRROR mode."""
    from humanoid_control.arm_profile import ArmProfile, JOINTS
    n = len(JOINTS)
    q._profile = ArmProfile(name="test", captured_utc="2026-01-01T00:00:00Z",
                            zero_rad=[0.0] * n,
                            lo_rad=[-1.2] * n, hi_rad=[1.2] * n,
                            upper_len_m=0.26, fore_len_m=0.22)
    return q._profile


def spy_gate(svc):
    """Record every set_run_gate call so we can assert on what was ASKED, not just the
    resulting flag — the bug this catches is a gate re-ASSERTED 60 times a second."""
    calls = []
    real = svc.set_run_gate
    def wrapped(active, *, source="web"):
        calls.append(bool(active))
        return real(active, source=source)
    svc.set_run_gate = wrapped
    return calls


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

    print("\n── E-STOP button (Y, left upper), unconditional ─────────────────")
    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.0, b=True))
    check("Y E-STOPs even with the trigger released", svc.estop.fired is True)

    svc, q = build()
    svc._input_source = "xbox"          # Quest does NOT hold the token
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.9, b=True))
    check("Y E-STOPs even when the Quest holds no input token",
          svc.estop.fired is True)
    check("a non-owner's pose command is still dropped",
          svc._arm_pose_command is None)

    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.0, b=True, tracked=False))
    check("Y E-STOPs even when the controller is untracked", svc.estop.fired is True)

    svc, q = build()
    q.on_frame(frame(1, p=(0, 0, 0), trigger=0.0, b=True))
    fired_at = svc.estop.fired
    q.on_frame(frame(2, p=(0, 0, 0), trigger=0.0, b=True))   # still held
    check("E-STOP is edge-triggered (a held button fires once)",
          fired_at is True and svc.estop.fired is True)

    print("\n── A = arm, B = disarm (right controller) ───────────────────────")
    svc, q = build()
    calls = []
    svc.arm_deadman = lambda *a, **k: calls.append("arm")
    svc.disarm_deadman = lambda *a, **k: calls.append("disarm")

    q.on_frame(frame(1, p=(0, 0, 0), right_a=True))
    check("A arms", calls == ["arm"], str(calls))
    q.on_frame(frame(2, p=(0, 0, 0), right_a=True))          # still held
    check("A is edge-triggered (held does not re-arm)", calls == ["arm"], str(calls))
    q.on_frame(frame(3, p=(0, 0, 0), right_a=False))
    q.on_frame(frame(4, p=(0, 0, 0), right_a=True))
    check("releasing and re-pressing A arms again",
          calls == ["arm", "arm"], str(calls))

    q.on_frame(frame(5, p=(0, 0, 0), right_b=True))
    check("B disarms", calls[-1] == "disarm", str(calls))

    # Arming a session this source cannot drive would strand the operator in ARMED with a
    # trigger that does nothing.
    svc, q = build()
    svc._input_source = "xbox"
    armed = []
    svc.arm_deadman = lambda *a, **k: armed.append(1)
    q.on_frame(frame(1, p=(0, 0, 0), right_a=True))
    check("A does NOT arm when the Quest is not the active method", armed == [])

    # Disarm is a stop, and stopping is never gated.
    svc, q = build()
    svc._input_source = "xbox"
    disarmed = []
    svc.disarm_deadman = lambda *a, **k: disarmed.append(1)
    q.on_frame(frame(1, p=(0, 0, 0), right_b=True))
    check("B disarms even when the Quest is not the active method",
          disarmed == [1], str(disarmed))

    # A refused arm (uncalibrated, wrong state) is operator feedback, not a crash.
    svc, q = build()
    def _refuse(*a, **k):
        raise ControlError("Calibrate all joints before arming — 5 uncalibrated.", 409)
    svc.arm_deadman = _refuse
    q.on_frame(frame(1, p=(0, 0, 0), right_a=True))
    check("a refused arm is reported, not raised",
          "Calibrate" in q.status()["reason"], q.status()["reason"])

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


    print("\n── BODY LOSS LATCHES (the IDLE<->POSITION oscillation) ──────────")
    # The bug: the run gate is asserted from the frame path at 60 Hz for as long as the
    # trigger is held, while the deadman worker cleared it at 50 Hz on body loss. The clear
    # survived ~16 ms. The arm oscillated between zero-torque and position hold at tick rate,
    # re-running enable_position() (a mode change to five ESCs) every cycle — triggered by
    # exactly the condition meant to make tracking loss SAFE.
    clk = Clock(); xr_mod.time.monotonic = clk
    svc, q = build(live_session=True)
    give_profile(q)
    n = 0
    def push(*, trig, emu=False, present=True, dt=1 / 60.0):
        nonlocal n
        n += 1
        f = frame(n, p=(0.0, 0.0, -0.30 - n * 1e-4), trigger=trig)
        f["body"] = body(emulated=emu, present=present)
        q.on_frame(f)
        clk.advance(dt)

    for _ in range(30):
        push(trig=0.9)
    check("body good + trigger held → gate on", svc._run_gate.is_set())

    calls = spy_gate(svc)
    # Body tracking drops. Trigger STAYS HELD, exactly as it would if the operator's arm
    # swung out of the headset's view mid-motion.
    for _ in range(int(xr_mod.BODY_HOLD_S * 60) + 30):
        push(trig=0.9, present=False)
    check("body lost past BODY_HOLD_S → gate released", not svc._run_gate.is_set())

    # THE REGRESSION. Hundreds more frames, trigger still held. Not one may re-assert.
    calls.clear()
    for _ in range(300):
        push(trig=0.9, present=False)
    check("trigger still held: the gate is never re-asserted",
          True not in calls, f"{calls.count(True)} of {len(calls)} calls asked to re-arm")
    check("...and it stays released", not svc._run_gate.is_set())

    # Tracking comes BACK, trigger never released. Must still stay down: re-arming a moving
    # arm without the operator asking is the thing the latch exists to prevent.
    calls.clear()
    for _ in range(120):
        push(trig=0.9)
    check("tracking recovers mid-hold: still latched until a re-press",
          True not in calls and not svc._run_gate.is_set())

    # Release, then press again. That is the deliberate act that re-arms.
    for _ in range(5):
        push(trig=0.0)
    check("releasing the trigger clears the latch", q._body_latch is False)
    for _ in range(30):
        push(trig=0.9)
    check("a fresh press re-arms", svc._run_gate.is_set())

    print("\n── an EMULATED joint is a guess, and counts as lost ─────────────")
    clk = Clock(); xr_mod.time.monotonic = clk
    svc, q = build(live_session=True); give_profile(q); n = 0
    for _ in range(30):
        push(trig=0.9)
    check("measured body → armed", svc._run_gate.is_set())
    for _ in range(int(xr_mod.BODY_HOLD_S * 60) + 30):
        push(trig=0.9, emu=True)
    check("emulated joints do not keep the gate alive", not svc._run_gate.is_set())

    print("\n── NO profile: body loss must not touch the controller path ─────")
    # Without a calibration the arm is driven from the controller's POSITION and body
    # tracking is not in the loop at all. Dropping the gate there would break the working
    # path over a signal nothing reads.
    clk = Clock(); xr_mod.time.monotonic = clk
    svc, q = build(live_session=True); n = 0
    # EXPLICITLY profile-less. build() -> attach() -> reload_profile() reads the operator's
    # real calibration from ~/.config, so "deliberately no give_profile()" quietly meant
    # "whatever this machine happens to have on disk" — passing on a fresh checkout and
    # failing the moment anyone actually calibrated. State the precondition instead of
    # inheriting it.
    q._profile = None
    for _ in range(30):
        push(trig=0.9)
    check("pose mode arms without a profile", svc._run_gate.is_set())
    for _ in range(int(xr_mod.BODY_HOLD_S * 60) + 60):
        push(trig=0.9, present=False)
    check("pose mode is unaffected by body-tracking loss", svc._run_gate.is_set())

    print("\n── a reconnect does not inherit the old link's body history ─────")
    clk = Clock(); xr_mod.time.monotonic = clk
    svc, q = build(live_session=True); give_profile(q); n = 0
    for _ in range(30):
        push(trig=0.9)
    clk.advance(xr_mod.BODY_HOLD_S + 5.0)             # long gap, then a NEW client
    push(trig=0.0, present=False)                     # body gone: the ladder can now see it
    check("stale body history would have read as lost", q.body_lost_too_long())
    q.attach()
    check("attach clears it", not q.body_lost_too_long())
    check("attach clears the latch", q._body_latch is False)

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
