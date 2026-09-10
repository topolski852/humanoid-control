#!/usr/bin/env python3
"""Zeroing the legs from one folded stance.

    .venv/bin/python scripts/test_leg_calibration.py

The per-joint flow in ``calibration.py`` needs each joint driven by hand to both of its
hardstops. That is a long session, and a single ESC crashing power-cycles the whole robot and
demands the whole session again. ``teach_leg_stance`` trades accuracy for one button press:
the folded, feet-together stance already parks four joints per leg against a mechanical stop.

The stub ESC below models the thing that actually matters — ``displayed = raw - offset`` — so
these tests exercise the real offset solve AND the read-back verification, not just the
bookkeeping. Two properties carry the most risk and are checked hardest:

  * BOTH LEGS TAKE THE SAME TARGET, UNNEGATED. The display frame is not mirrored; negating
    the right leg would silently double the error on every roll and yaw joint.
  * AN OFFLINE JOINT IS NAMED, NOT GUESSED. A dead ESC takes no writes, and the whole point of
    this flow is to recover the other eleven joints from exactly that situation.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.config import LegPolicyContract              # noqa: E402
from humanoid_control.layout import RobotLayout                     # noqa: E402
from humanoid_control import leg_calibration as lc                  # noqa: E402
from humanoid_control.web.service import (ControlError, ControlService,  # noqa: E402
                                          SessionState)

PASS, FAIL = [], []
DEG = 3.141592653589793 / 180.0


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


class FakeEsc:
    """displayed = raw - position_offset, which is the whole of the firmware that matters here."""

    def __init__(self, raw: float, offset: float = 0.0, online: bool = True):
        self.raw, self.offset, self.online = raw, offset, online

    @property
    def displayed(self) -> float:
        return self.raw - self.offset


class StubClient:
    def __init__(self, motors: dict[str, FakeEsc], refuse: set[str] = frozenset(),
                 deaf: set[str] = frozenset()):
        self.motors = motors
        self.refuse = set(refuse)   # apply_config raises
        self.deaf = set(deaf)       # apply_config ACKs but the write never lands
        self.writes: list[tuple[str, float]] = []

    def is_running(self): return True
    def latest_base(self): return {}
    def estop_all(self): pass

    def get_cached_joint_state(self, name):
        m = self.motors.get(name)
        if m is None:
            return None
        if not m.online:
            return {"position": m.displayed, "state": "OFFLINE"}
        return {"position": m.displayed, "state": "IDLE"}

    def read_device_config(self, name):
        return {"position_offset": self.motors[name].offset}

    def apply_config(self, name, config=None, timeout=20.0):
        if name in self.refuse:
            raise RuntimeError("apply_config failed for " + name)
        self.writes.append((name, config["position_offset"]))
        if name not in self.deaf:
            self.motors[name].offset = config["position_offset"]


CONTRACT = LegPolicyContract.load()
LEGS = RobotLayout(enabled=("left_leg", "right_leg"), imu_expected=False)


def limits_of(joint: str) -> tuple[float, float]:
    i = CONTRACT.index_of(joint)
    return float(CONTRACT.pos_limit_lower[i]), float(CONTRACT.pos_limit_upper[i])


def stance_raw(joint: str, err_deg: float = 0.0) -> float:
    """A raw encoder reading for a joint sitting in the stance, off by ``err_deg``."""
    target, _ = lc.stance_target(joint, limits_of(joint))
    return target + err_deg * DEG


def build(motors: dict[str, FakeEsc], **kw) -> tuple[ControlService, StubClient]:
    client = StubClient(motors, **kw)
    svc = ControlService(client, CONTRACT, config_present=True, layout=LEGS)
    svc._state = SessionState.CONNECTED
    svc._TEACH_SAMPLE_S = 0.05        # the real 1.5 s only buys jitter rejection
    return svc, client


def all_in_stance(offset: float = 0.7) -> dict[str, FakeEsc]:
    """Every leg joint physically in the stance, with a stale offset that must be corrected."""
    return {n: FakeEsc(raw=stance_raw(n), offset=offset) for n in LEGS.joint_order}


def main() -> int:
    print("\n── the stance defines every leg joint, and says which kind ─────")
    for jt, which in lc.HARDSTOP.items():
        for side in ("left", "right"):
            j = f"{side}_{jt}_joint"
            lo, hi = limits_of(j)
            got, src = lc.stance_target(j, (lo, hi))
            check(f"{j.replace('_joint','')} -> {which} hardstop",
                  got == (lo if which == "lower" else hi) and src == "measured",
                  f"{got:.4f}")
    for jt in lc.DECLARED:
        for side in ("left", "right"):
            j = f"{side}_{jt}_joint"
            got, src = lc.stance_target(j, limits_of(j))
            check(f"{j.replace('_joint','')} is declared 0", got == 0.0 and src == "declared")

    print("\n── both legs take the SAME target — the display frame is not mirrored ──")
    # Negating the right leg would be invisible on the joints whose target is 0 and wrong by
    # twice the angle everywhere else. The contract's limits are the evidence: identical
    # left-to-right for five of the six types, and hip_pitch differs only by a 0.05 deg shift
    # of an otherwise equal-width range (a URDF rounding artifact, not a mirror).
    for jt in lc.STANCE_TYPES:
        l = lc.stance_target(f"left_{jt}_joint", limits_of(f"left_{jt}_joint"))[0]
        r = lc.stance_target(f"right_{jt}_joint", limits_of(f"right_{jt}_joint"))[0]
        check(f"{jt}: left and right agree to within 0.1 deg",
              abs(l - r) < 0.1 * DEG, f"{l / DEG:.4f} vs {r / DEG:.4f} deg")
        if abs(l) > 1e-9:
            check(f"{jt}: the right target is not the negation",
                  abs(l - (-r)) > 1.0 * DEG, f"{l / DEG:.2f} vs {r / DEG:.2f} deg")

    print("\n── one press zeroes a leg that is actually in the stance ───────")
    svc, client = build(all_in_stance())
    res = svc.teach_leg_stance("both")
    check("every joint reported", len(res["joints"]) == 12, str(len(res["joints"])))
    check("all twelve zeroed", res["ok"] and res["zeroed"] == 12, str(res["zeroed"]))
    landed = []
    for n in LEGS.joint_order:
        want, _ = lc.stance_target(n, limits_of(n))
        landed.append(abs(client.motors[n].displayed - want) < 1e-9)
    check("each joint now reads its stance target", all(landed))
    check("the app marks them calibrated",
          all(svc._calibrated[n] for n in LEGS.joint_order))

    print("\n── the offsets it wrote are the ones that make it read right ───")
    svc, client = build(all_in_stance(offset=0.0))
    svc.teach_leg_stance("both")
    for n in ("left_knee_pitch_joint", "right_ankle_roll_joint"):
        want, _ = lc.stance_target(n, limits_of(n))
        m = client.motors[n]
        check(f"{n.replace('_joint','')}: raw - offset == target",
              abs((m.raw - m.offset) - want) < 1e-9, f"offset={m.offset:.5f}")

    print("\n── a dead ESC is named, and the other eleven still get done ────")
    motors = all_in_stance()
    motors["right_hip_yaw_joint"].online = False
    svc, client = build(motors)
    res = svc.teach_leg_stance("both")
    dead = [r for r in res["joints"] if r["joint"] == "right_hip_yaw_joint"][0]
    check("the offline joint is reported, not silently dropped",
          len(res["joints"]) == 12 and not dead["ok"])
    check("...and says why", "offline" in dead["reason"], dead.get("reason", ""))
    check("...and is never written to",
          not any(n == "right_hip_yaw_joint" for n, _ in client.writes))
    check("...and is not marked calibrated", not svc._calibrated["right_hip_yaw_joint"])
    check("the other eleven are zeroed", res["zeroed"] == 11, str(res["zeroed"]))
    check("the run reports itself incomplete", res["ok"] is False)

    print("\n── a declared joint mirrors its calibrated twin ────────────────")
    # hip_roll / hip_yaw have no stop in this stance, so a twin that is already calibrated is
    # better evidence than assuming the feet are perfectly square.
    motors = all_in_stance()
    motors["left_hip_roll_joint"] = FakeEsc(raw=4.0 * DEG, offset=0.0)   # twin reads 4 deg
    svc, client = build(motors)
    svc._calibrated["left_hip_roll_joint"] = True
    res = svc.teach_leg_stance("right_leg")
    row = [r for r in res["joints"] if r["joint"] == "right_hip_roll_joint"][0]
    check("it is sourced as mirrored", row["source"] == "mirrored", str(row["source"]))
    check("...naming the twin it copied", row["mirrored_from"] == "left_hip_roll_joint")
    check("...and copies the angle rather than negating it",
          abs(row["target_deg"] - 4.0) < 1e-6, f"{row['target_deg']:.3f}")
    check("the right leg now reads its twin's angle",
          abs(client.motors["right_hip_roll_joint"].displayed - 4.0 * DEG) < 1e-9)

    print("\n── an uncalibrated twin is not evidence ────────────────────────")
    motors = all_in_stance()
    motors["left_hip_roll_joint"] = FakeEsc(raw=4.0 * DEG, offset=0.0)
    svc, client = build(motors)          # nothing pre-calibrated
    res = svc.teach_leg_stance("right_leg")
    row = [r for r in res["joints"] if r["joint"] == "right_hip_roll_joint"][0]
    check("it falls back to the declared zero", row["source"] == "declared", str(row["source"]))
    check("...landing on 0", abs(row["target_deg"]) < 1e-9)

    print("\n── a joint inside the run is never a mirror source ─────────────")
    # Otherwise a both-legs run has each twin mirroring the other: two readings swapped, both
    # of them about to be overwritten by the declared zero anyway. Checked with BOTH legs
    # already marked calibrated, which is the state that actually triggers it.
    motors = all_in_stance()
    motors["left_hip_roll_joint"] = FakeEsc(raw=4.0 * DEG, offset=0.0)
    motors["right_hip_roll_joint"] = FakeEsc(raw=-7.0 * DEG, offset=0.0)
    svc, client = build(motors)
    for n in LEGS.joint_order:
        svc._calibrated[n] = True
    res = svc.teach_leg_stance("both")
    srcs = {r["joint"]: r["source"] for r in res["joints"]}
    check("left_hip_roll stays 'declared' in a both-legs run",
          srcs["left_hip_roll_joint"] == "declared", str(srcs["left_hip_roll_joint"]))
    check("right_hip_roll stays 'declared' in a both-legs run",
          srcs["right_hip_roll_joint"] == "declared", str(srcs["right_hip_roll_joint"]))
    check("...so neither inherits the other's angle",
          abs(client.motors["left_hip_roll_joint"].displayed) < 1e-9
          and abs(client.motors["right_hip_roll_joint"].displayed) < 1e-9)

    print("\n── a write that does not land is caught by the read-back ───────")
    svc, client = build(all_in_stance(), deaf={"left_knee_pitch_joint"})
    res = svc.teach_leg_stance("both")
    row = [r for r in res["joints"] if r["joint"] == "left_knee_pitch_joint"][0]
    check("the deaf joint is not reported OK", not row["ok"])
    check("...with the reason", row.get("reason") == "did not land on target",
          str(row.get("reason")))
    check("...and is not marked calibrated", not svc._calibrated["left_knee_pitch_joint"])

    print("\n── a refused write is reported, not swallowed ──────────────────")
    svc, client = build(all_in_stance(), refuse={"right_hip_roll_joint"})
    res = svc.teach_leg_stance("both")
    row = [r for r in res["joints"] if r["joint"] == "right_hip_roll_joint"][0]
    check("the refused joint fails with the daemon's message",
          not row["ok"] and "apply_config failed" in row["reason"], str(row.get("reason")))

    print("\n── it refuses to run when it must not ──────────────────────────")
    svc, _ = build(all_in_stance())
    svc._state = SessionState.DISCONNECTED
    try:
        svc.teach_leg_stance("both"); check("disconnected is refused", False, "ran anyway")
    except ControlError as exc:
        check("disconnected is refused", exc.status == 409, str(exc))

    svc, _ = build(all_in_stance())
    svc.is_motion_active = lambda: True
    try:
        svc.teach_leg_stance("both"); check("an active session is refused", False, "ran anyway")
    except ControlError as exc:
        check("an active session is refused", exc.status == 409, str(exc))

    svc, _ = build(all_in_stance())
    try:
        svc.teach_leg_stance("left_arm"); check("an unconfigured limb is refused", False, "ran")
    except ControlError as exc:
        check("an unconfigured limb is refused", exc.status == 400, str(exc))

    motors = all_in_stance()
    for m in motors.values():
        m.online = False
    svc, _ = build(motors)
    try:
        svc.teach_leg_stance("both"); check("an all-offline robot is refused", False, "ran")
    except ControlError as exc:
        check("an all-offline robot is refused", exc.status == 409, str(exc))

    print("\n── a sloppy stance lands sloppy, and says how far ──────────────")
    # The method is absolute: it writes whatever makes the joint read the target NOW. Stance
    # error therefore lands in the zero one-for-one, which is the whole trade against the
    # two-capture flow. The reported shift is what exposes it.
    motors = {n: FakeEsc(raw=stance_raw(n, err_deg=0.0), offset=0.25) for n in LEGS.joint_order}
    svc, client = build(motors)
    res = svc.teach_leg_stance("both")
    shifts = [abs(r["shift_deg"]) for r in res["joints"] if r["ok"]]
    check("a true stance needs a shift equal to the stale offset",
          all(abs(s - 0.25 / DEG) < 1e-6 for s in shifts), f"{max(shifts):.3f} deg")

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    for f in FAIL:
        print(f"  FAILED: {f}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
