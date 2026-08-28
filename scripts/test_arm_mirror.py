#!/usr/bin/env python3
"""Offline checks for whole-arm mirroring (ArmTeleop.step_mirror).

No robot, no headset. Run::

    .venv/bin/python scripts/test_arm_mirror.py

The acceptance property for this whole feature is simple to state: MOVE ONE JOINT, ONE JOINT
MOVES. That is what the controller-position path could never do — it chases a point, so the
IK redistributes motion and bending only your elbow moved four joints.

The second thing checked here is the one that has bitten twice: the command must be allowed
to LEAD the encoder. A target set directly from the retargeter would pin the position error
at a fraction of a degree, ask for ~0.19 Nm against a ~2.8 Nm gravity load, and the shoulder
would sit still exactly as it did before the leash fix.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.arm_kinematics import ArmChain          # noqa: E402
from humanoid_control.arm_teleop import ArmTeleop, TeleopTuning  # noqa: E402
from humanoid_control.layout import LIMB_JOINTS               # noqa: E402

PASS, FAIL = [], []
DT = 1.0 / 50.0
KP, GRAVITY_NM = 45.0, 2.8          # position_kp from the ESCs; load from the wiki
NAMES = [n.replace("_joint", "").replace("left_", "") for n in LIMB_JOINTS["left_arm"]]


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


def fresh():
    ch = ArmChain(list(LIMB_JOINTS["left_arm"]))
    tel = ArmTeleop(ch, tuning=TeleopTuning(frame="mirror"))
    q = np.radians([0.0, -10.0, 0.0, 20.0, 0.0])      # a plausible relaxed pose
    tel.reset(q)
    return ch, tel, q


def settle(tel, q, targets, ticks=400, **kw):
    info = {}
    for _ in range(ticks):
        q, info = tel.step_mirror(q, targets, DT, **kw)
    return q, info


def main() -> int:
    print("\n── the acceptance property: one joint in, one joint out ────────")
    ch, tel, q0 = fresh()
    base = q0.copy()
    for i, n in enumerate(NAMES):
        ch2, tel2, q = fresh()
        tgt = base.copy()
        tgt[i] += np.radians(20)
        tgt = ch2.clamp(tgt)
        q, _ = settle(tel2, q, tgt)
        moved = np.degrees(np.abs(q - base))
        others = np.delete(moved, i)
        check(f"target {n:<16} moves only {n}",
              moved[i] > 5.0 and float(others.max()) < 0.5,
              f"{moved[i]:.1f}° target, worst other {others.max():.3f}°")

    print("\n── it reaches the commanded posture ────────────────────────────")
    ch, tel, q = fresh()
    tgt = ch.clamp(np.radians([20.0, 30.0, -15.0, 60.0, 25.0]))
    q, info = settle(tel, q, tgt)
    err = np.degrees(np.abs(q - tgt))
    check("converges to the retargeted joint vector",
          float(err.max()) < 1.0, f"worst {err.max():.2f}°")
    check("reports the per-joint error for diagnosis",
          isinstance(info.get("joint_err_deg"), list) and len(info["joint_err_deg"]) == ch.n)
    check("frame is reported as mirror", info.get("frame") == "mirror")

    print("\n── the command LEADS the encoder (the stall that bit twice) ────")
    # Stuck-joint model: a joint moves only if commanded torque beats its gravity load. A
    # simulated chain follows perfectly, which is exactly why this failure hid for so long.
    def drive_with_stiction(ticks=400):
        chain = ArmChain(list(LIMB_JOINTS["left_arm"]))
        tel = ArmTeleop(chain, tuning=TeleopTuning(frame="mirror"))
        q0 = np.radians([0.0, -10.0, 0.0, 20.0, 0.0])
        q = q0.copy(); tel.reset(q)
        tgt = chain.clamp(np.radians([10.0, 45.0, 0.0, 40.0, 0.0]))
        need = np.array([GRAVITY_NM, GRAVITY_NM, GRAVITY_NM * .6, GRAVITY_NM * .25, .05])
        for _ in range(ticks):
            qt, _ = tel.step_mirror(q, tgt, DT)
            lead = qt - q
            q = np.where(np.abs(lead) * KP > need, q + lead * 0.35, q)
        return np.degrees(q - q0)

    moved = drive_with_stiction()
    check("the shoulder lifts against gravity (roll moves)",
          abs(moved[1]) > 10.0, f"roll moved {moved[1]:+.1f}°")
    check("the leash commands more torque than the gravity load",
          np.radians(TeleopTuning().pose_leash_deg) * KP > GRAVITY_NM,
          f"{np.radians(TeleopTuning().pose_leash_deg)*KP:.2f} Nm vs {GRAVITY_NM} Nm")

    print("\n── rate limiting ───────────────────────────────────────────────")
    ch, tel, q = fresh()
    far = ch.clamp(np.radians([40.0, 70.0, 40.0, 89.0, 40.0]))
    q1, _ = tel.step_mirror(q, far, DT)
    step = np.degrees(np.abs(q1 - q)).max()
    cap = np.degrees(TeleopTuning().mirror_rate_normal * DT)
    check("a distant target moves at most one rate-limited step",
          step <= cap + 1e-6, f"{step:.3f}° vs cap {cap:.3f}°")

    ch, tel, q = fresh()
    q1, _ = tel.step_mirror(q, far, DT, creep=True)
    scr = np.degrees(np.abs(q1 - q)).max()
    check("creep is slower than normal", scr < step, f"creep {scr:.3f}° vs {step:.3f}°")

    print("\n── HOLD on tracking loss ───────────────────────────────────────")
    ch, tel, q = fresh()
    q, _ = settle(tel, q, ch.clamp(np.radians([15.0, 25.0, 0.0, 45.0, 0.0])), ticks=200)
    held = q.copy()
    for _ in range(200):                       # tracking gone: hold=True, no targets
        q, info = tel.step_mirror(q, None, DT, hold=True)
    check("hold freezes the arm where it was",
          float(np.degrees(np.abs(q - held)).max()) < 0.5,
          f"drifted {np.degrees(np.abs(q-held)).max():.3f}°")
    check("hold is reported in info", info.get("hold") is True)
    check("hold reports not-commanding", info.get("commanding") is False)

    print("\n── bad input holds rather than flinging ────────────────────────")
    for label, bad in (("NaN", [np.nan, 0, 0, 0, 0]), ("inf", [np.inf, 0, 0, 0, 0])):
        ch, tel, q = fresh()
        before = q.copy()
        q1, info = tel.step_mirror(q, np.array(bad), DT)
        check(f"a {label} target holds instead of moving",
              bool(np.all(np.isfinite(q1)))
              and float(np.degrees(np.abs(q1 - before)).max()) < 1e-6)

    print("\n── limits are enforced ─────────────────────────────────────────")
    ch, tel, q = fresh()
    q, _ = settle(tel, q, np.radians([200.0, 200.0, 200.0, 200.0, 200.0]), ticks=600)
    check("absurd targets clamp to the joint limits",
          bool(np.all(q >= ch.limits_lower - 1e-9) and np.all(q <= ch.limits_upper + 1e-9)),
          f"{np.degrees(q).round(1)}")

    print("\n── the other frames still work ─────────────────────────────────")
    ch, tel, q = fresh()
    tel.tuning = TeleopTuning(frame="pose")
    _, info = tel.step_pose(q, np.array([0.0, 0.02, 0.0]), DT)
    check("step_pose is unaffected", info.get("frame") == "pose")
    tel.tuning = TeleopTuning()
    _, info = tel.step(q, [0.0, 0.0, 0.0, 0.0], DT)
    check("step (joint frame) is unaffected", info.get("frame") == "joint")

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
