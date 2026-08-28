#!/usr/bin/env python3
"""Can the operator select a policy this robot must not run?

    .venv/bin/python scripts/test_policy_compat.py

Switching policy in the dropdown switches the NETWORK and nothing else. Gains, stand pose,
action scale and timing all keep coming from `configs/leg_policy_params.json`, so a bundle
trained at other gains runs against a robot it has never seen. humanoid-policy's deploy README
states the consequence directly: "switching policy without switching defaults+gains makes the
robot snap to the wrong reference."

Before this gate existed nothing checked it. A mismatched bundle loaded fine and failed — if
at all — as an onnxruntime shape error on the FIRST step, which is after the ramp has already
put the robot into the stand pose.

THE TEST THAT MATTERS IS THE FRAME ONE. The trainer exports in URDF frame, where the right leg
is mirrored; the runtime works in device frame. `right_hip_roll`, `right_hip_yaw` and
`right_ankle_roll` therefore hold OPPOSITE SIGNS in the two by design. A gain check that
compares default poses naively flags every correct bundle (-0.11 against +0.11) and clears
none — measured on the real bundles, it rejected both the live policy and smooth A. So the
sign-flip case is pinned here explicitly, in both directions.
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.config import (POLICY_FRAME_MIRRORED_JOINTS,  # noqa: E402
                                     LegPolicyContract)
from humanoid_control.policy import bundle_issues                    # noqa: E402

PASS, FAIL = [], []
CONTRACT = LegPolicyContract.load()


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")


def base_bundle() -> dict:
    """A contract that exactly matches the runtime, written in TRAINER (URDF) frame.

    Built from the runtime contract rather than copied from a file so the fixture cannot drift
    away from what the robot is actually running.
    """
    sign = CONTRACT.policy_frame_sign
    return {
        "canonical_joint_order": list(CONTRACT.joint_order),
        "control": {"policy_dt": CONTRACT.policy_dt,
                    "control_dt": CONTRACT.control_dt,
                    "action_scale": CONTRACT.action_scale},
        "observation": {"num_observations": CONTRACT.num_observations},
        "joints": [
            {"joint_name": n,
             "kp": float(CONTRACT.kp[i]),
             "kd": float(CONTRACT.kd[i]),
             # device -> URDF, which is what an export contains
             "default_pose": float(CONTRACT.default_pose[i]) * float(sign[i])}
            for i, n in enumerate(CONTRACT.joint_order)
        ],
    }


def issues_for(bundle: dict | None) -> list[str]:
    if bundle is None:
        return bundle_issues(None, CONTRACT)
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
        json.dump(bundle, fh)
        path = fh.name
    try:
        return bundle_issues(path, CONTRACT)
    finally:
        Path(path).unlink(missing_ok=True)


def main() -> int:
    print("\n── a bundle that matches is accepted ───────────────────────────")
    check("a contract built from the runtime passes", issues_for(base_bundle()) == [],
          str(issues_for(base_bundle()))[:90])

    print("\n── the frame flip is NOT a mismatch ────────────────────────────")
    # This is the case that broke a naive implementation. The mirrored joints legitimately
    # disagree in sign; anything else legitimately does not.
    mirrored = [n for n in CONTRACT.joint_order if n in POLICY_FRAME_MIRRORED_JOINTS]
    check("the runtime really does mirror right-leg joints", len(mirrored) == 3, str(mirrored))

    flipped = base_bundle()
    for j in flipped["joints"]:
        if j["joint_name"] in POLICY_FRAME_MIRRORED_JOINTS and abs(j["default_pose"]) > 1e-9:
            j["default_pose"] = -j["default_pose"]      # now WRONG in the trainer's own frame
    check("un-flipping a mirrored joint IS caught",
          any("stand pose" in i for i in issues_for(flipped)),
          str(issues_for(flipped))[:90])

    print("\n── gains are what actually gate it ─────────────────────────────")
    b = base_bundle(); b["joints"][0]["kp"] = float(CONTRACT.kp[0]) + 5.0
    check("a different kp is refused", any("gains" in i for i in issues_for(b)))
    b = base_bundle(); b["joints"][3]["kd"] = float(CONTRACT.kd[3]) * 2
    check("a different kd is refused", any("gains" in i for i in issues_for(b)))
    b = base_bundle()
    for j in b["joints"]:
        j["kp"], j["kd"] = 20.0, 4.0
    iss = issues_for(b)
    check("wholesale different gains are refused", any("gains" in i for i in iss))
    check("...and the message names joints, not just a count",
          any("kp" in i and "!=" in i for i in iss), str(iss)[:100])

    print("\n── the spec is checked too, not just gains ─────────────────────")
    b = base_bundle(); b["control"]["action_scale"] = 0.5
    check("a different action_scale is refused", any("action_scale" in i for i in issues_for(b)))
    b = base_bundle(); b["control"]["policy_dt"] = 0.02
    check("a different policy_dt is refused", any("policy_dt" in i for i in issues_for(b)))
    b = base_bundle(); b["observation"]["num_observations"] = 48
    check("a different observation count is refused",
          any("observation" in i for i in issues_for(b)))
    b = base_bundle(); b["canonical_joint_order"] = list(reversed(b["canonical_joint_order"]))
    iss = issues_for(b)
    check("a different joint order is refused", any("joint order" in i for i in iss))
    check("...and stops there, since nothing else lines up", len(iss) == 1, str(iss))

    print("\n── unverifiable is not the same as unsafe ──────────────────────")
    # Loose weight files and older exports legitimately have no contract. Refusing everything
    # unverifiable would break the fallback path in /api/policies.
    check("no contract file is allowed through", issues_for(None) == [])
    check("an unreadable contract is reported, not ignored",
          bundle_issues("/nonexistent/leg_policy_contract.json", CONTRACT) != [])

    print("\n── tolerance is tight on gains, loose on float32 pose ──────────")
    b = base_bundle(); b["joints"][0]["kp"] = float(CONTRACT.kp[0]) + 1e-9
    check("a rounding-level gain difference passes", issues_for(b) == [])
    b = base_bundle()
    for j in b["joints"]:
        j["default_pose"] = float(f"{j['default_pose']:.7g}")   # float32 round-trip
    check("a float32 round-tripped pose passes", issues_for(b) == [], str(issues_for(b))[:80])
    b = base_bundle(); b["joints"][2]["default_pose"] += math.radians(5)
    check("5 deg of stand-pose drift is caught",
          any("stand pose" in i for i in issues_for(b)))

    print("\n── the real bundles in this repo ───────────────────────────────")
    root = Path(__file__).resolve().parent.parent / "policies"
    seen = 0
    for sub in sorted(p for p in root.iterdir() if p.is_dir()):
        if not (sub / "policy.onnx").is_file():
            continue
        seen += 1
        cp = sub / "leg_policy_contract.json"
        iss = bundle_issues(str(cp) if cp.is_file() else None, CONTRACT)
        print(f"      {'blocked' if iss else 'ok     '}  {sub.name}")
    check("every bundle was evaluated", seen >= 1, f"{seen} bundles")

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
