#!/usr/bin/env python3
"""Offline checks for DUAL-ARM teleop: both arms driven at once, one trigger each.

    .venv/bin/python scripts/test_dual_arm.py

WHAT THIS IS FOR. `test_quest_liveness.py` proves the safety ladder for ONE arm and it keeps
passing unchanged while the second arm does nothing at all — it only ever builds a left-arm
layout, so "the gate" and "the arm" are the same object there and a per-arm split cannot be
observed. The questions that only appear with two arms are:

  * does one controller's trigger drive ONE arm, leaving the other resting?
  * do the GLOBAL controls stay global — E-STOP and disarm must take both arms down?
  * does a one-arm robot still work, with no second-arm special case?

The last one is the regression that matters most: the left arm is the measured baseline, and
nothing here may change how it behaves on a bench with one arm attached.

Triggers are ACTIVATION, not a deadman. Releasing one rests THAT arm in DAMPING, which holds
it up; it is not a stop. The stop is E-STOP, and it is global.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.config import LegPolicyContract           # noqa: E402
from humanoid_control.layout import RobotLayout                 # noqa: E402
from humanoid_control.web import xr as xr_mod                   # noqa: E402
from humanoid_control.web.service import ControlService, SessionState  # noqa: E402

PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


class Clock:
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


def build(arms=("left_arm", "right_arm"), live_session=True):
    import os
    os.environ["HUMANOID_QUEST_ENABLE"] = "1"
    os.environ.pop("HUMANOID_GAMEPAD_ENABLE", None)
    os.environ.pop("HUMANOID_QUEST_HAND", None)
    os.environ["HUMANOID_QUEST_SCALE"] = "1.0"
    os.environ["HUMANOID_QUEST_YAW_DEG"] = "0"
    svc = ControlService(StubClient(), LegPolicyContract.load(), config_present=True,
                         layout=RobotLayout(enabled=arms, imu_expected=False))
    svc._input_source = "quest"
    q = xr_mod.QuestSource(svc)
    svc.quest = q
    if live_session:
        svc._state = SessionState.ARMED
        svc._session_deadman = "quest"
    q.attach()
    return svc, q


def frame(seq, *, left_trig=0.0, right_trig=0.0, left_p=None, right_p=None,
          left_b=False, right_a=False, right_b=False, session="s1"):
    """One XR frame carrying BOTH controllers independently — the whole point here."""
    def ctrl(p, trig, a=False, b=False):
        return {"p": list(p), "q": [0, 0, 0, 1], "tracked": True, "trigger": trig,
                "squeeze": 0.0, "stick": [0, 0], "a": a, "b": b, "stickPress": False}
    return {"seq": seq, "t": float(seq), "session": session,
            "head": {"p": [0, 1.6, 0], "q": [0, 0, 0, 1], "tracked": True},
            "left": ctrl(left_p or (-0.2, 1.0, -0.3), left_trig, b=left_b),
            "right": ctrl(right_p or (0.2, 1.0, -0.3), right_trig, a=right_a, b=right_b)}


def give_profiles(q):
    from humanoid_control.arm_profile import ArmProfile, JOINTS
    n = len(JOINTS)
    for h, st in q._hands.items():
        st.profile = ArmProfile(name="test", side=h, captured_utc="2026-01-01T00:00:00Z",
                                zero_rad=[0.0] * n, lo_rad=[-1.2] * n, hi_rad=[1.2] * n,
                                upper_len_m=0.26, fore_len_m=0.22)


def gates(svc):
    return {k: v.is_set() for k, v in svc._run_gates.items()}


def main() -> int:
    print("\n── layout: two arms is a distinct capability ───────────────────")
    two = RobotLayout(enabled=("left_arm", "right_arm"), imu_expected=False)
    one = RobotLayout(enabled=("left_arm",), imu_expected=False)
    check("two arms advertise dual_arm_teleop", two.can("dual_arm_teleop"))
    check("one arm does NOT", not one.can("dual_arm_teleop"))
    check("one arm still advertises arm_teleop", one.can("arm_teleop"),
          "the single-arm bench must not lose its session")
    check("why_not explains the two-arm requirement",
          "two arms" in one.why_not("dual_arm_teleop"))

    print("\n── a gate per arm, created from the layout ─────────────────────")
    svc, q = build()
    check("two arms -> two gates", sorted(svc._run_gates) == ["left_arm", "right_arm"],
          str(sorted(svc._run_gates)))
    svc1, _ = build(arms=("left_arm",))
    check("one arm -> one gate", sorted(svc1._run_gates) == ["left_arm"],
          str(sorted(svc1._run_gates)))

    print("\n── INDEPENDENCE: each trigger drives its own arm ───────────────")
    clk = Clock(); xr_mod.time.monotonic = clk
    svc, q = build()
    give_profiles(q)
    n = 0

    def push(**kw):
        nonlocal n
        n += 1
        # Nudge both poses every frame so the frozen-pose detector never trips.
        kw.setdefault("left_p", (-0.2, 1.0, -0.30 - n * 1e-4))
        kw.setdefault("right_p", (0.2, 1.0, -0.30 - n * 1e-4))
        q.on_frame(frame(n, **kw))
        clk.advance(1 / 60.0)

    for _ in range(20):
        push(left_trig=0.9, right_trig=0.0)
    g = gates(svc)
    check("left trigger only -> left arm active", g["left_arm"] is True, str(g))
    check("left trigger only -> right arm RESTS", g["right_arm"] is False, str(g))

    for _ in range(20):
        push(left_trig=0.0, right_trig=0.9)
    g = gates(svc)
    check("right trigger only -> right arm active", g["right_arm"] is True, str(g))
    check("right trigger only -> left arm RESTS", g["left_arm"] is False, str(g))

    for _ in range(20):
        push(left_trig=0.9, right_trig=0.9)
    g = gates(svc)
    check("both triggers -> both arms active", all(g.values()), str(g))
    check("any_run_gate() is true while either drives", svc.any_run_gate())

    print("\n── one controller failing does not touch the other arm ─────────")
    # The right controller stops being tracked mid-drive. Its arm must release; the left,
    # still held and still tracked, must keep driving.
    for _ in range(10):
        n += 1
        f = frame(n, left_trig=0.9, right_trig=0.9,
                  left_p=(-0.2, 1.0, -0.30 - n * 1e-4))
        f["right"]["tracked"] = False
        q.on_frame(f)
        clk.advance(1 / 60.0)
    g = gates(svc)
    check("untracked right controller releases the RIGHT arm", g["right_arm"] is False, str(g))
    check("...and the left arm keeps driving", g["left_arm"] is True, str(g))
    check("the left arm's reason is not the right's failure",
          q._hands["left"].reason == "" and "tracked" in q._hands["right"].reason,
          f"left={q._hands['left'].reason!r} right={q._hands['right'].reason!r}")

    print("\n── each arm clutches on its OWN anchor ─────────────────────────")
    svc, q = build(); give_profiles(q); n = 0
    for _ in range(5):
        push(left_trig=0.9, right_trig=0.0)
    for _ in range(5):
        push(left_trig=0.9, right_trig=0.9)
    la, ra = q._hands["left"].anchor, q._hands["right"].anchor
    check("both hands anchored", la is not None and ra is not None)
    check("the anchors are different points", la is not None and ra is not None
          and abs(float(la[1]) - float(ra[1])) > 0.1,
          f"left y={float(la[1]):+.3f} right y={float(ra[1]):+.3f}")
    lc = svc.arm_pose_command("left_arm")
    rc = svc.arm_pose_command("right_arm")
    check("each arm has its own pose command slot",
          lc is not None and rc is not None)

    print("\n── GLOBAL controls stay global ─────────────────────────────────")
    svc, q = build(); give_profiles(q); n = 0
    for _ in range(20):
        push(left_trig=0.9, right_trig=0.9)
    check("both arms driving before E-STOP", all(gates(svc).values()))
    push(left_trig=0.9, right_trig=0.9, left_b=True)     # Y on the left controller
    check("E-STOP fires", svc.estop.fired)
    check("E-STOP takes BOTH arms down", not any(gates(svc).values()), str(gates(svc)))
    # THE REGRESSION. The operator's startle reflex is to CLENCH, so both triggers are still
    # held. The frame path asserts the gate once per frame at 60 Hz, so a gate that is merely
    # cleared comes back 16 ms later. Not one of these frames may re-arm either arm.
    for _ in range(200):
        push(left_trig=0.9, right_trig=0.9)
    check("triggers still squeezed: E-STOP stays down", not any(gates(svc).values()),
          str(gates(svc)))
    # Releasing is what clears the latch — and E-STOP itself still blocks re-arming, so this
    # checks the latch, not the E-STOP.
    for _ in range(5):
        push(left_trig=0.0, right_trig=0.0)
    check("releasing the triggers clears the stop latch",
          not any(h.stop_latch for h in q._hands.values()))

    svc, q = build(); give_profiles(q); n = 0
    for _ in range(20):
        push(left_trig=0.9, right_trig=0.9)
    check("both arms driving before disarm", all(gates(svc).values()))
    push(left_trig=0.9, right_trig=0.9, right_b=True)    # B on the right controller
    check("B disarms the machine", svc._armed is False)
    check("disarm takes BOTH arms down", not any(gates(svc).values()), str(gates(svc)))
    for _ in range(200):
        push(left_trig=0.9, right_trig=0.9)
    check("triggers still squeezed: disarm stays down", not any(gates(svc).values()),
          str(gates(svc)))

    print("\n── a one-arm robot has no second-arm behaviour to go wrong ─────")
    svc, q = build(arms=("left_arm",)); give_profiles(q); n = 0
    for _ in range(20):
        push(left_trig=0.9, right_trig=0.9)      # BOTH triggers held
    check("only the configured arm gets a gate", sorted(gates(svc)) == ["left_arm"],
          str(gates(svc)))
    check("the configured arm drives", gates(svc)["left_arm"] is True)
    check("the absent arm is named, not silently ignored",
          "no such arm" in q._hands["right"].reason, q._hands["right"].reason)
    check("the absent arm never anchors", q._hands["right"].anchor is None)

    print("\n── status reports both arms ────────────────────────────────────")
    svc, q = build(); give_profiles(q); n = 0
    for _ in range(20):
        push(left_trig=0.9, right_trig=0.0)
    s = q.status()
    check("status carries a per-arm block", set(s.get("arms", {})) == {"left", "right"},
          str(sorted(s.get("arms", {}))))
    check("per-arm active flags differ", s["arms"]["left"]["active"] is True
          and s["arms"]["right"]["active"] is False)
    check("each arm names its limb",
          s["arms"]["left"]["limb"] == "left_arm"
          and s["arms"]["right"]["limb"] == "right_arm")
    check("the flat keys still describe the driving hand",
          s["tracked"] is True and "hand" in s,
          "existing single-arm readers must not break")
    st = svc.telemetry_snapshot()["control"]["arm_state"]
    check("telemetry reports per-arm state", set(st) == {"left_arm", "right_arm"},
          str(sorted(st)))
    check("telemetry agrees with the gates",
          st["left_arm"]["active"] is True and st["right_arm"]["active"] is False)

    print("\n── the snapshot must SERIALISE, with a live info dict in it ────")
    # THE REGRESSION (2026-09-19 17:44). The per-tick `info` carries numpy arrays. Surfacing
    # it raw made /api/status raise inside the JSON encoder and return 500 — so the whole web
    # UI went blank while teleop kept working perfectly, which is the worst way for this to
    # fail: the operator loses every readout and the arm stays live.
    import json as _json
    import numpy as _np

    class _FakeRig:
        limb = "left_arm"
        engaged = True
        info = {                      # the real shape, arrays and all
            "target": _np.zeros(5), "hand": _np.zeros(3), "frame": "mirror",
            "hold": False, "joint_err_deg": [1.0] * 5,
            "worst_joint_err_deg": _np.float64(3.5),
            "lead_deg": [0.1] * 5, "at_limit": False, "clipped": False,
            "commanding": True,
            "spherical": {"reach_m": 0.4, "elevation_deg": 10.0, "azimuth_deg": 5.0},
        }

    svc._rigs = [_FakeRig()]
    snap = svc.telemetry_snapshot()
    try:
        _json.dumps(snap)
        ok, why = True, ""
    except Exception as exc:                                  # noqa: BLE001
        ok, why = False, str(exc)
    check("the whole telemetry snapshot is JSON-serialisable", ok, why)
    li = snap["control"]["arm_state"]["left_arm"]["info"]
    check("no numpy arrays survive into telemetry",
          li is not None and not any(isinstance(v, _np.ndarray) for v in li.values()),
          str(li))
    check("the per-tick diagnostics are NOT broadcast at 20 Hz",
          li is not None and not ({"target", "lead_deg", "joint_err_deg"} & set(li)),
          f"leaked {sorted({'target','lead_deg','joint_err_deg'} & set(li or {}))}")
    check("the operator-facing fields ARE kept",
          li is not None and li.get("worst_joint_err_deg") == 3.5
          and li.get("commanding") is True, str(li))

    # And the engage refusal, which is the one the operator actually needs to read.
    class _BlockedRig(_FakeRig):
        info = {"engage_blocked": True, "limit_deg": 40.0,
                "worst_joint": "shoulder_pitch", "worst_deg": -73.2}

    svc._rigs = [_BlockedRig()]
    bi = svc.telemetry_snapshot()["control"]["arm_state"]["left_arm"]["info"]
    check("an engage refusal reaches the UI intact",
          bi == {"engage_blocked": True, "limit_deg": 40.0,
                 "worst_joint": "shoulder_pitch", "worst_deg": -73.2}, str(bi))

    print("\n── building the rigs must not stall the event loop ─────────────")
    # THE REGRESSION (2026-09-19 17:45). Warming ArmChain.reach_bounds() inside the rig-build
    # loop put ~1.0s of GIL-heavy numerical work per arm on the ARMING path. The Quest
    # heartbeat watchdog trips at LOSS_S = 1.0s, so arming two arms fired a spurious
    # quest-timeout E-STOP before the second rig existed — and the right arm, whose rig was
    # built last, never ran at all. The symptom reads as "the right arm won't enable"; the
    # cause is that arming took longer than the deadman allows.
    # perf_counter, NOT monotonic: the sections above do `xr_mod.time.monotonic = clk`,
    # which mutates the shared `time` module, so every monotonic() in this process is a
    # frozen fake clock. Timing anything with it silently reads 0.00s and the check passes
    # for the wrong reason — which is exactly what this block is here to catch.
    import time as _time
    _now = _time.perf_counter
    from humanoid_control.arm_kinematics import ArmChain as _AC
    from humanoid_control.layout import LIMB_JOINTS as _LJ

    svc, q = build()
    # The warm-up runs in a background thread at construction; give it room to finish.
    for _ in range(60):
        if all(getattr(svc, "_chain_cache", {}).get(lb) is not None
               for lb in ("left_arm", "right_arm")):
            break
        _time.sleep(0.1)
    check("both arm chains are warmed at startup",
          set(getattr(svc, "_chain_cache", {})) >= {"left_arm", "right_arm"},
          str(sorted(getattr(svc, "_chain_cache", {}))))

    t0 = _now()
    for lb in ("left_arm", "right_arm"):
        svc.arm_chain(lb).reach_bounds()
    warmed = _now() - t0
    check("reach_bounds is already cached, so arming pays nothing",
          warmed < 0.05, f"{warmed * 1000:.1f} ms for both arms")
    check("...and that is well inside the 1.0s deadman window",
          warmed < xr_mod.LOSS_S / 4, f"{warmed:.4f}s vs LOSS_S {xr_mod.LOSS_S}s")

    # Cold, for contrast: this is what used to run while arming.
    cold = _AC(list(_LJ["left_arm"]))
    t0 = _now(); cold.reach_bounds(); cold_s = _now() - t0
    check("a COLD chain would indeed have blown the window",
          cold_s > 0.2, f"cold {cold_s:.2f}s — why this must not happen while arming")

    worker_src = (Path(__file__).resolve().parent.parent / "humanoid_control" / "web"
                  / "service.py").read_text().split("def _deadman_worker", 1)[1]
    calls = [ln.strip() for ln in worker_src.splitlines()
             if ".reach_bounds(" in ln and not ln.strip().startswith("#")]
    check("the rig build no longer CALLS reach_bounds", not calls,
          "; ".join(calls) or "")

    print("\n── the CLIENT must sample every joint the server requires ──────")
    # THE REGRESSION (2026-09-19 17:55). BODY_JOINTS in app/xr/main.js listed the left arm's
    # joints only — a leftover from one-arm-per-session, with `right-shoulder` present purely
    # to define the torso frame. Nothing on the server could detect it: human_angles(side=
    # "right") simply returned None every frame, so the right arm sat in `hold` with its
    # target frozen, and its run log recorded ZERO human_deg rows while looking otherwise
    # healthy. Worse, `right-shoulder` still fed the torso frame, so moving the operator's
    # RIGHT arm perturbed the LEFT arm's decomposition — two symptoms, one missing list.
    #
    # This is the seam a Python test suite cannot otherwise see: the requirement lives in
    # Python and the sampling lives in JavaScript.
    import re as _re
    js = (Path(__file__).resolve().parent.parent / "app" / "xr" / "main.js").read_text()
    m = _re.search(r"const BODY_JOINTS = \[(.*?)\];", js, _re.S)
    check("BODY_JOINTS is findable in the client", m is not None)
    sampled = set(_re.findall(r"'([a-z-]+)'", m.group(1))) if m else set()
    for side in ("left", "right"):
        need = set(xr_mod.QuestSource.body_required(side))
        check(f"the client samples every joint the {side} arm needs",
              need <= sampled, f"missing {sorted(need - sampled)}")

    # And the built bundle that is actually SERVED, not just the source.
    dist = sorted((Path(__file__).resolve().parent.parent / "app" / "dist"
                   / "assets").glob("xr-*.js"))
    if dist:
        built = dist[-1].read_text()
        missing = [j for j in xr_mod.QuestSource.body_required("right") if j not in built]
        check("the BUILT bundle carries the right arm's joints too", not missing,
              f"missing {missing} — run `npm run build` in app/")
    else:
        check("a built xr bundle exists to check", False, "no app/dist/assets/xr-*.js")

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
