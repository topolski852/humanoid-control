#!/usr/bin/env python3
"""Offline checks for ArmTeleop.step_pose — the 6-DOF-tracker (Quest) frame.

No robot, no daemon. Solves against the same vendored URDF kinematics the wireframe draws
from, so a failure here is a real kinematics/controller failure, not a fixture artefact::

    .venv/bin/python scripts/test_arm_pose_teleop.py
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


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


def fresh():
    chain = ArmChain(list(LIMB_JOINTS["left_arm"]))
    tel = ArmTeleop(chain)
    q = (chain.limits_lower + chain.limits_upper) / 2.0     # mid-range: away from every stop
    tel.reset(q)
    return chain, tel, q


def settle(tel, chain, q, delta, ticks, creep=False):
    """Run the servo to convergence and return (q, last_info)."""
    info = {}
    for _ in range(ticks):
        q, info = tel.step_pose(q, delta, DT, creep=creep)
    return q, info


def main() -> int:
    print("\n── anchoring ────────────────────────────────────────────────────")
    chain, tel, q = fresh()
    hand0 = chain.tool(q)
    check("reset() anchors the pose frame at the current hand",
          np.allclose(tel.pose_anchor, hand0))

    q1, info = tel.step_pose(q, [0.0, 0.0, 0.0], DT)
    check("zero displacement holds position (no drift)",
          float(np.linalg.norm(chain.tool(q1) - hand0)) < 1e-4,
          f"moved {np.linalg.norm(chain.tool(q1) - hand0)*1000:.2f} mm")
    check("zero displacement reports not-commanding", info["commanding"] is False)

    print("\n── it actually tracks ───────────────────────────────────────────")
    chain, tel, q = fresh()
    hand0 = chain.tool(q)
    target_delta = np.array([0.0, 0.0, 0.02])          # 2 cm up: well inside the shell
    q, info = settle(tel, chain, q, target_delta, ticks=400)
    reached = chain.tool(q) - hand0
    check("hand converges to a 2 cm commanded displacement",
          float(np.linalg.norm(reached - target_delta)) < 2e-3,
          f"reached {reached.round(4)} vs asked {target_delta.round(4)}")
    check("converged tracking error is ~0",
          info["tracking_error_m"] < 2e-3, str(info["tracking_error_m"]))

    print("\n── speed cap: a far target must not lunge ───────────────────────")
    chain, tel, q = fresh()
    hand0 = chain.tool(q)
    q1, info = tel.step_pose(q, [10.0, 0.0, 0.0], DT)      # 10 m away — absurd on purpose
    moved = float(np.linalg.norm(chain.tool(q1) - hand0))
    cap = TeleopTuning().speed_normal * DT
    check("a 10 m target moves the hand at most one speed-limited step",
          moved <= cap + 1e-6, f"moved {moved*1000:.2f} mm, cap {cap*1000:.2f} mm")
    check("a 10 m target produces finite joint targets", bool(np.all(np.isfinite(q1))))
    check("an out-of-shell target is reported as clipped", info["clipped"] is True)
    check("tracking error is reported large, not hidden",
          info["tracking_error_m"] > 0.05, str(info["tracking_error_m"]))

    print("\n── creep is slower than normal ──────────────────────────────────")
    chain, tel, q = fresh()
    h0 = chain.tool(q)
    qn, _ = tel.step_pose(q, [0.0, 0.0, 10.0], DT, creep=False)
    d_normal = float(np.linalg.norm(chain.tool(qn) - h0))
    chain, tel, q = fresh()
    h0 = chain.tool(q)
    qc, _ = tel.step_pose(q, [0.0, 0.0, 10.0], DT, creep=True)
    d_creep = float(np.linalg.norm(chain.tool(qc) - h0))
    check("creep moves strictly less than normal per tick",
          d_creep < d_normal, f"creep {d_creep*1000:.2f} mm vs normal {d_normal*1000:.2f} mm")

    print("\n── limits and safety ────────────────────────────────────────────")
    chain, tel, q = fresh()
    q, info = settle(tel, chain, q, [0.0, 0.0, 10.0], ticks=600)   # drive hard into the stop
    lo, hi = chain.limits_lower, chain.limits_upper
    check("joint targets never leave their limits",
          bool(np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9)),
          f"q={q.round(3)}")
    check("hand stays inside the reachable shell",
          chain.reach_bounds()[0] - 1e-3
          <= float(np.linalg.norm(chain.tool(q) - chain.shoulder()))
          <= chain.reach_bounds()[1] + 1e-3)

    chain, tel, q = fresh()
    hand0 = chain.tool(q)
    q1, _ = tel.step_pose(q, [float("nan"), 0.0, 0.0], DT)
    check("a NaN sample HOLDS instead of flinging the arm",
          bool(np.all(np.isfinite(q1)))
          and float(np.linalg.norm(chain.tool(q1) - hand0)) < 1e-4)

    chain, tel, q = fresh()
    hand0 = chain.tool(q)
    q1, _ = tel.step_pose(q, [float("inf"), 0.0, 0.0], DT)
    check("an inf sample HOLDS instead of flinging the arm",
          bool(np.all(np.isfinite(q1)))
          and float(np.linalg.norm(chain.tool(q1) - hand0)) < 1e-4)

    print("\n── re-anchoring (the clutch ratchet) ────────────────────────────")
    chain, tel, q = fresh()
    q, _ = settle(tel, chain, q, [0.0, 0.0, 0.02], ticks=400)
    hand_after = chain.tool(q)
    tel.reset(q)                                    # release + re-press
    check("re-anchor moves the anchor to the hand's NEW position",
          np.allclose(tel.pose_anchor, hand_after))
    q2, _ = tel.step_pose(q, [0.0, 0.0, 0.0], DT)
    check("after re-anchor, zero displacement holds (no snap back)",
          float(np.linalg.norm(chain.tool(q2) - hand_after)) < 1e-4)

    print("\n── the stick frames are untouched ───────────────────────────────")
    chain, tel, q = fresh()
    hand0 = chain.tool(q)
    q1, info = tel.step(q, [0.0, 0.0, 0.0, 0.0], DT)
    check("step() with centred sticks still holds", info["frame"] == "joint")
    tel.tuning = TeleopTuning(frame="cartesian")
    q2, info2 = tel.step(q, [0.0, 1.0, 0.0, 0.0], DT)
    check("step() cartesian frame still moves the hand",
          float(np.linalg.norm(chain.tool(q2) - hand0)) > 1e-5, info2["frame"])

    print("\n── the command must lead the encoder enough to LIFT the arm ─────")
    # THE REGRESSION THAT COST THREE WRONG DIAGNOSES. Every other check here uses a chain
    # that follows its target perfectly, so a command that leads by a hair still "works".
    # The real arm does not: a joint moves only if the commanded lead produces more torque
    # than its gravity load. Integrating the target from the ENCODER pinned that lead at
    # ~0.25 deg = 0.19 Nm against a ~2.8 Nm load, so the shoulder never moved and only the
    # elbow (carrying just the forearm) did. Simulation could not show it; the flight
    # recorder could, and did.
    KP, GRAVITY_NM = 45.0, 2.8      # position_kp from the ESCs; load from the wiki

    def drive_with_stiction(tuning, ticks=400):
        chain = ArmChain(list(LIMB_JOINTS["left_arm"]))
        tel = ArmTeleop(chain, tuning=tuning)
        q0 = np.radians([0.1, -10.3, 0.0, 39.0, 0.0])   # the real logged hanging pose
        q = q0.copy(); tel.reset(q)
        need = np.array([GRAVITY_NM, GRAVITY_NM, GRAVITY_NM * 0.6, GRAVITY_NM * 0.25, 0.05])
        for _ in range(ticks):
            qt, _ = tel.step_pose(q, np.array([0.0, 0.15, 0.10]), DT)
            lead = qt - q
            q = np.where(np.abs(lead) * KP > need, q + lead * 0.35, q)
        return np.degrees(q - q0)

    stalled = drive_with_stiction(TeleopTuning(frame="pose", pose_leash_deg=0.25))
    check("a one-tick leash cannot lift the arm (reproduces the original bug)",
          abs(stalled[1]) < 1.0, f"shoulder_roll moved {stalled[1]:+.2f}°")

    moved = drive_with_stiction(TeleopTuning(frame="pose"))
    check("the default leash DOES lift the shoulder against gravity",
          abs(moved[1]) > 10.0, f"shoulder_roll moved only {moved[1]:+.2f}°")
    check("lateral motion is served, not starved by the elbow",
          abs(moved[1]) > abs(moved[3]), f"roll {moved[1]:+.1f}° vs elbow {moved[3]:+.1f}°")

    tun = TeleopTuning(frame="pose")
    check("default leash commands more torque than the arm's gravity load",
          np.radians(tun.pose_leash_deg) * KP > GRAVITY_NM,
          f"{np.radians(tun.pose_leash_deg)*KP:.2f} Nm vs {GRAVITY_NM} Nm needed")

    print("\n── the leash still bounds a blocked joint ──────────────────────")
    chain = ArmChain(list(LIMB_JOINTS["left_arm"]))
    tel = ArmTeleop(chain, tuning=TeleopTuning(frame="pose"))
    qb = np.radians([0.1, -10.3, 0.0, 39.0, 0.0]); tel.reset(qb)
    for _ in range(600):                       # joint completely blocked: q never changes
        qt, info = tel.step_pose(qb, np.array([0.0, 0.5, 0.0]), DT)
    lead = np.degrees(np.abs(qt - qb)).max()
    check("a fully blocked joint is held at the leash, not wound up",
          lead <= TeleopTuning().pose_leash_deg + 1e-6, f"lead reached {lead:.2f}°")
    check("info reports the per-joint lead for diagnosis",
          isinstance(info.get("lead_deg"), list) and len(info["lead_deg"]) == chain.n)

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
