#!/usr/bin/env python3
"""The robot-side heading loop (humanoid_control/heading.py).

    .venv/bin/python scripts/test_heading.py

Walks measured 2026-09-29 veer 6-11 deg/s with nothing to correct them: sim closes the heading
loop in its command generator, and the robot never did. These tests pin down the loop's logic
offline. The one thing they cannot prove is the IMU's sign on the real robot; that needs the
hand-turn check in ``dry`` mode (see heading.py) before ``on`` is ever used.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.heading import HeadingHold, wrap_pi, yaw_from_quat  # noqa: E402

PASS, FAIL = [], []
WALK = np.array([0.6, 0.0, 0.0], dtype=np.float32)
STAND = np.zeros(3, dtype=np.float32)


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


def q(deg):
    h = math.radians(deg) / 2
    return [math.cos(h), 0.0, 0.0, math.sin(h)]      # [w, x, y, z] about +z


def close(a, b, tol=1e-4):
    return abs(float(a) - float(b)) <= tol


def main() -> int:
    check("yaw is CCW-positive about +z", close(yaw_from_quat(q(30)), math.radians(30), 1e-6)
          and close(yaw_from_quat(q(-45)), math.radians(-45), 1e-6))

    h = HeadingHold(mode="off")
    check("off: command untouched", np.array_equal(h.update(WALK, q(20)), WALK))

    h = HeadingHold(mode="on", k=0.5, wz_max=0.5, tau=0)
    h.update(WALK, q(0))
    out = h.update(WALK, q(10))
    check("veer LEFT -> wz NEGATIVE (turn back right)", close(out[2], -0.5 * math.radians(10)),
          f"wz {out[2]:+.4f}")
    check("vx untouched", close(out[0], 0.6))
    check("input command not mutated", WALK[2] == 0.0)

    h = HeadingHold(mode="dry", tau=0)
    h.update(WALK, q(0))
    out = h.update(WALK, q(10))
    check("dry: computes but sends operator wz", out[2] == 0.0 and h.last["wz_loop"] < 0
          and h.last["active"])

    h = HeadingHold(mode="on", k=0.5, wz_max=0.3, tau=0)
    h.update(WALK, q(0))
    check("clipped to wz_max", close(h.update(WALK, q(-120))[2], 0.3))

    h = HeadingHold(mode="on", tau=0)
    h.update(WALK, q(0))
    turn = np.array([0.6, 0.0, 0.4], dtype=np.float32)
    check("operator yaw passes through (not fought)", close(h.update(turn, q(25))[2], 0.4))
    check("after the stick releases, holds the NEW heading", close(h.update(WALK, q(25))[2], 0.0, 1e-6))

    h = HeadingHold(mode="on", tau=0)
    h.update(WALK, q(0))
    check("standing sends wz 0 (sim zeroes standing envs)", h.update(STAND, q(40))[2] == 0.0)
    check("standing re-latches: next bout starts from here", close(h.update(WALK, q(40))[2], 0.0, 1e-6))

    h = HeadingHold(mode="on", k=0.5, wz_max=1.0, tau=0)
    h.update(WALK, q(179))
    out = h.update(WALK, q(-179))
    check("wraps across ±180 (2 deg, not 358)", close(out[2], -0.5 * math.radians(2), 1e-3),
          f"wz {out[2]:+.4f}")
    check("wrap_pi", close(wrap_pi(math.radians(358)), math.radians(-2), 1e-9))

    h = HeadingHold(mode="on", tau=0)
    h.update(WALK, q(0))
    h.reset()
    check("reset (new engage) re-latches", close(h.update(WALK, q(50))[2], 0.0, 1e-6))

    h = HeadingHold(mode="on", tau=0)
    check("no IMU quaternion: command untouched", np.array_equal(h.update(WALK, None), WALK))

    # Low-pass: a per-step twist must not become a per-step turn command.
    h = HeadingHold(mode="on", k=0.5, wz_max=0.5, tau=1.0)
    h.update(WALK, q(0), now=0.0)
    wz = []
    for i in range(1, 251):                       # 10 s at 25 Hz, +-15 deg twist at 1.4 Hz
        t = i * 0.04
        wz.append(float(h.update(WALK, q(15 * math.sin(2 * math.pi * 1.4 * t)), now=t)[2]))
    amp = max(abs(w) for w in wz[50:])
    check("filtered: a 1.4 Hz ±15° twist gives a small wz", amp < 0.05, f"|wz| max {amp:.3f} "
          f"(unfiltered would be {0.5 * math.radians(15):.3f})")
    h = HeadingHold(mode="on", k=0.5, wz_max=0.5, tau=1.0)
    h.update(WALK, q(0), now=0.0)
    for i in range(1, 126):                       # steady 10°/s veer for 5 s
        t = i * 0.04
        out = h.update(WALK, q(10 * t), now=t)
    check("filtered: a steady veer is still corrected, against its direction",
          out[2] < -0.2, f"wz {out[2]:+.3f} after 5 s of +10°/s")

    try:
        HeadingHold(mode="maybe")
        check("bad mode rejected", False)
    except ValueError:
        check("bad mode rejected", True)

    # Service wiring (source-level: constructing the service needs a daemon).
    svc = (Path(__file__).resolve().parent.parent / "humanoid_control/web/service.py").read_text()
    check("service passes the shared heading loop to every leg runner",
          svc.count("heading=self._heading,") >= 2, f"{svc.count('heading=self._heading,')} sites")
    check("mode change refused while moving", "if self._state in _MOTION_STATES:" in
          svc.split("def set_heading_mode")[1].split("def ")[0])
    runner = (Path(__file__).resolve().parent.parent / "humanoid_control/runner.py").read_text()
    check("runner feeds the heading-adjusted command to the observation",
          "command=command, prev_action" in runner)
    check("runner resets heading on every engage (prepare)", "self.heading.reset()" in runner)

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
