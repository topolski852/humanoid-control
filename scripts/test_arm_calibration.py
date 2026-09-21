#!/usr/bin/env python3
"""Offline checks for the guided operator-arm calibration LIFECYCLE.

    .venv/bin/python scripts/test_arm_calibration.py

WHAT THIS IS FOR. The capture maths is covered by test_arm_profile.py. What was never
covered — and what shipped broken — is how a run STARTS and how it ENDS:

  * `self._calib` was cleared in exactly one place, reachable only from a desktop HTTP
    DELETE. A run that reached the last pose set `done = True` and then sat there forever.
    Because `hud_frame()` short-circuits on `if self._calib is not None` before any normal
    HUD logic, the headset was pinned to the completion card through trigger pulls, E-STOP,
    disarm, and even leaving and re-entering the session — while the arm drove normally
    behind a card reading "you can take the headset off".
  * A run whose body tracking went away simply stalled mid-pose, indefinitely, with no way
    out from inside the headset.
  * Nothing on the Quest could start a run at all; it took a curl from the desktop while the
    operator stood in the headset wearing it.

So these assert on `hud_frame()` — what the operator actually sees — rather than on the
private `_calib`, because "the run object was cleared" is not the property that matters.

THE CLOCK. This file fakes `time.monotonic` in BOTH xr and xr_calib. They are the same
module object, so patching one patches the other; it is done explicitly to make that obvious
rather than surprising. Any real timing must use `perf_counter`.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.config import LegPolicyContract           # noqa: E402
from humanoid_control.layout import RobotLayout                 # noqa: E402
from humanoid_control.web import xr as xr_mod                   # noqa: E402
from humanoid_control.web import xr_calib as calib_mod          # noqa: E402
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


def build(clk):
    import os
    os.environ["HUMANOID_QUEST_ENABLE"] = "1"
    os.environ.pop("HUMANOID_GAMEPAD_ENABLE", None)
    os.environ.pop("HUMANOID_QUEST_HAND", None)
    # Both modules read time.monotonic from the same module object.
    xr_mod.time.monotonic = clk
    calib_mod.time.monotonic = clk
    svc = ControlService(StubClient(), LegPolicyContract.load(), config_present=True,
                         layout=RobotLayout(enabled=("left_arm", "right_arm"),
                                            imu_expected=False))
    svc._input_source = "quest"
    q = xr_mod.QuestSource(svc)
    svc.quest = q
    q.attach()
    q._connected = True
    q._last_rx = clk()          # tick() returns early without this
    return svc, q


def give_profiles(q):
    """A loaded profile, so the normal HUD is the MIRRORING card rather than NOT CALIBRATED —
    which makes 'the HUD handed back' unambiguous."""
    from humanoid_control.arm_profile import ArmProfile, JOINTS
    n = len(JOINTS)
    for h, st in q._hands.items():
        st.profile = ArmProfile(name="test", side=h, captured_utc="2026-01-01T00:00:00Z",
                                zero_rad=[0.0] * n, lo_rad=[-1.2] * n, hi_rad=[1.2] * n,
                                upper_len_m=0.26, fore_len_m=0.22)


def arm(vals=(0.0, 0.0, 0.0, 0.0, 0.0)):
    from humanoid_control.arm_retarget import HumanArm
    return HumanArm(*vals, 0.26, 0.22)


def run_to_completion(c, clk, sides=("left", "right"), tmpdir=None):
    """Walk every pose. Returns when the run ends."""
    import humanoid_control.arm_profile as ap
    if tmpdir is not None:
        ap.default_profile_path = lambda: tmpdir / "arm_profiles.json"
    for k in range(len(c.seq)):
        if c.is_ended:
            break
        vals = {s: [0.1 * (k + 1) * (1 if s == "left" else -1), 0.2 * (k + 1),
                    0.05, 0.3 * (k + 1), 0.02] for s in sides}
        for _ in range(8):                       # samples inside the hold window
            clk.advance(0.01)
            c._settle_until = clk() - 0.05
            c.update({s: arm(vals[s]) for s in sides}, {})
        c._settle_until = clk() - calib_mod.HOLD_S - 0.05
        c.update({s: arm(vals[s]) for s in sides}, {})
    return c


def step(name: str) -> None:
    print(f"\n── {name} " + "─" * max(0, 62 - len(name)))


def main() -> int:
    import tempfile
    tmp = Path(tempfile.mkdtemp())

    step("a finished run hands the HUD back BY ITSELF")
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration()
    run_to_completion(c, clk, tmpdir=tmp)
    check("the run reached the end", c.done, f"idx {c.idx}/{len(c.seq)}")
    check("it is in the terminal state", c.is_ended and c.end_reason == "done",
          f"reason={c.end_reason!r}")
    f = q.hud_frame()
    check("the HUD shows the completion card", f and "COMPLETE" in f.get("step", ""),
          str(f and f.get("step")))
    check("...naming both sides saved", f and "LEFT" in f.get("instruction", "")
          and "RIGHT" in f.get("instruction", ""), str(f and f.get("instruction")))
    # Not yet: the operator has to be able to read it.
    clk.advance(xr_mod.CALIB_DWELL_S - 1.0); q.tick()
    f = q.hud_frame()
    check("before the dwell expires it is still showing",
          f and "COMPLETE" in f.get("step", ""), str(f and f.get("step")))
    clk.advance(2.0); q.tick()
    f = q.hud_frame()
    check("after the dwell the run is retired", q._calib is None)
    check("THE REGRESSION: hud_frame returns the NORMAL hud again",
          f is not None and "CALIBRATION" not in f.get("step", ""),
          str(f and f.get("step")))

    step("X dismisses the finished card immediately")
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration(); run_to_completion(c, clk, tmpdir=tmp)
    check("card is up", "COMPLETE" in (q.hud_frame() or {}).get("step", ""))
    q._on_calib_x()
    check("X clears it without waiting out the dwell", q._calib is None)
    check("and the normal HUD is back",
          "CALIBRATION" not in (q.hud_frame() or {}).get("step", ""))

    step("X cancels a run in progress")
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration()
    clk.advance(4.0); c._settle_until = clk() - 0.05
    c.update({"left": arm(), "right": arm()}, {})
    check("a run is live", not c.is_ended and q._calib is c)
    q._on_calib_x()
    check("X ends it as cancelled", c.is_ended and c.end_reason == "cancelled",
          f"reason={c.end_reason!r}")
    f = q.hud_frame()
    check("the card SAYS it was cancelled", f and "CANCELLED" in f.get("step", ""),
          str(f and f.get("step")))
    check("nothing was saved", not c.saved_sides, str(c.saved_sides))
    clk.advance(xr_mod.CALIB_DWELL_S + 1.0); q.tick()
    check("it retires like any other ended run", q._calib is None)

    step("a run that loses body tracking aborts instead of stalling")
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration()
    clk.advance(4.0); c._settle_until = clk() - 0.05
    c.update({"left": arm(), "right": arm()}, {})     # tracking was working...
    clk.advance(xr_mod.CALIB_STALL_S - 1.0); q.tick()
    check("a short dropout does NOT abandon the run", not c.is_ended,
          "an ordinary occlusion must be survivable")
    clk.advance(2.0); q.tick()
    check("a long one does", c.is_ended and c.end_reason == "tracking",
          f"reason={c.end_reason!r}")
    f = q.hud_frame()
    check("the card names the cause", f and "BODY TRACKING" in f.get("instruction", ""),
          str(f and f.get("instruction")))

    step("a run started with no tracking at all still ends")
    # The stall clock has to fall back to creation time, or a calibration begun on a headset
    # without body tracking sits on POSE 1 forever — never having had a sample to age from.
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration()
    check("no sample has ever arrived", c.last_sample_at == 0.0)
    clk.advance(xr_mod.CALIB_STALL_S + 1.0); q.tick()
    check("it still aborts", c.is_ended and c.end_reason == "tracking",
          f"reason={c.end_reason!r}")

    step("a run in progress is not silently replaced")
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration()
    clk.advance(4.0); c._settle_until = clk() - 0.05
    c.update({"left": arm()}, {})
    c.idx = 2                                   # pretend two poses are banked
    try:
        q.start_calibration()
        refused = False
    except RuntimeError as exc:
        refused = "already running" in str(exc)
    check("starting again is REFUSED", refused, "a double-press must not discard poses")
    check("the original run survives", q._calib is c and c.idx == 2)
    c.end("cancelled")
    try:
        q.start_calibration(); replaced = True
    except RuntimeError:
        replaced = False
    check("but an ENDED run can be replaced", replaced and q._calib is not c)

    step("a reconnect does not inherit a finished run")
    clk = Clock(); svc, q = build(clk); give_profiles(q)
    c = q.start_calibration(); run_to_completion(c, clk, tmpdir=tmp)
    q.detach(); q.attach()
    check("attach clears the finished run", q._calib is None)
    # ...but an unfinished one survives a blip.
    c2 = q.start_calibration()
    clk.advance(4.0); c2._settle_until = clk() - 0.05
    c2.update({"left": arm()}, {})
    q.detach(); q.attach()
    check("an UNFINISHED run survives a reconnect", q._calib is c2,
          "a link blip must not discard captured poses")

    step("a partial save does not read as a clean success")
    clk = Clock(); svc, q = build(clk)
    c = calib_mod.CalibrationRun()
    c.saved_sides = ("left",)
    c.failed_note = "right: disk full"
    c.done = True
    c.end("done")
    f = c.hud(None)
    check("tone is not 'ok' when one side failed", f.get("tone") != "ok", str(f.get("tone")))
    check("the failure is on the card", "disk full" in f.get("note", ""), str(f.get("note")))
    # And the real bug: saved_to is written per side, so it is truthy even when one failed.
    c2 = calib_mod.CalibrationRun()
    c2.saved_to = "/some/path.json"      # what _persist leaves after ANY successful side
    c2.saved_sides = ()                  # ...but nothing actually saved
    c2.done = True
    c2.end("done")
    check("an all-failed save reports NOT SAVED despite a stale saved_to",
          c2.hud(None).get("instruction") == "NOT SAVED",
          str(c2.hud(None).get("instruction")))

    step("every terminal card tells the operator how to leave")
    for reason in ("done", "cancelled", "tracking"):
        r = calib_mod.CalibrationRun()
        r.end(reason)
        note = r.hud(None).get("note", "")
        check(f"'{reason}' card mentions X", "X" in note, note)

    step("the CLIENT can start a run, and the built bundle can too")
    # The seam a Python suite cannot otherwise see: the button lives in HTML, the request in
    # JavaScript, the endpoint in Python. Checked in the source AND in what is actually
    # served, because app/dist is a build artefact and is gitignored.
    root = Path(__file__).resolve().parent.parent
    html = (root / "app" / "xr" / "index.html").read_text()
    js = (root / "app" / "xr" / "main.js").read_text()
    check("the page has a calibrate button", 'id="calibrate"' in html)
    check("the client POSTs the calibrate endpoint", "/api/quest/calibrate" in js)
    check("...and wires the button to it", "calibrate: true" in js)
    dist = sorted((root / "app" / "dist" / "assets").glob("xr-*.js"))
    if dist:
        built = dist[-1].read_text()
        check("the BUILT bundle carries it too", "/api/quest/calibrate" in built,
              "run `npm run build` in app/")
    else:
        check("a built xr bundle exists to check", False, "no app/dist/assets/xr-*.js")
    # The endpoint the client names must be the one the server serves.
    routes = (root / "humanoid_control" / "web" / "routes.py").read_text()
    check("the server serves that exact path",
          '@router.post("/api/quest/calibrate"' in routes)

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
