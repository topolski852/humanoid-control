#!/usr/bin/env python3
"""Vendor the biped URDF kinematics into a JSON the web wireframe can render.

The URDF lives in the sibling ``humanoid-policy`` repo (it is the trainer's asset, not
ours) so the browser can never reach it. This script extracts the *shape* of the robot —
joint origins, rotation matrices and the collision primitives — into
``app/src/data/viz_kinematics.json``, which Vite inlines into the bundle at build time.

What is deliberately NOT vendored: ``default_pose``, the device-frame position limits and
``policy_frame_sign``. Those are frame-critical and the whole point of the visualizer is
to catch frame/offset mistakes, so the app fetches them live from ``GET /api/contract``
and refuses to draw if they disagree with ``joint_sign`` recorded here.

Frame conventions (URDF, and therefore this file):
  - world/base axes: +X forward, +Y left, +Z up; metres and radians
  - ``rpy`` is URDF fixed-axis roll-pitch-yaw, i.e. ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``
  - a child frame is ``T(xyz) @ R(rpy) @ R_axis(q)``; every leg joint has axis [0,0,1]

Usage:
    python scripts/gen_viz_kinematics.py            # write app/src/data/viz_kinematics.json
    python scripts/gen_viz_kinematics.py --check    # exit 1 if the checked-in file is stale
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import _bootstrap  # noqa: F401  (puts the repo root on sys.path)
from humanoid_control.config import REPO_ROOT, LegPolicyContract

URDF_PATH = Path(
    "/home/nse/humanoid-policy/source/humanoid_policy_assets/data/robots/humanoid/urdf"
    "/humanoid_biped.urdf"
)
OUT_PATH = REPO_ROOT / "app" / "src" / "data" / "viz_kinematics.json"

# onshape-to-robot writes π/2 and π truncated to 6 digits and leaves float dust like
# 2.2e-15 in place of an exact zero. Snapping both keeps the FK exact and the JSON
# readable; the correction is ~3e-6 rad, i.e. micrometres over the whole leg chain.
_SNAP_TOL = 1e-4
_SNAP_TARGETS = [n * np.pi / 2 for n in (-2, -1, 0, 1, 2)]


def snap(v: float) -> float:
    """Round float dust to 0 and near-multiples of π/2 to the exact value."""
    for t in _SNAP_TARGETS:
        if abs(v - t) < _SNAP_TOL:
            return float(t)
    return float(v)


def snap3(text: str) -> list[float]:
    return [snap(float(x)) for x in text.split()]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis RPY -> 3x3 rotation. R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def clean(x: float, ndigits: int = 9) -> float:
    """Round for JSON and normalise -0.0 to 0.0 so diffs stay stable."""
    r = round(float(x), ndigits)
    return 0.0 if r == 0 else r


def urdf_name(contract_name: str) -> str:
    """Contract name -> URDF joint name (the URDF prefixes every leg joint with `leg_`)."""
    return f"leg_{contract_name}"


# --- collision primitives ---------------------------------------------------
# The URDF's <visual> geometry is STL meshes we cannot ship to a browser, but its
# <collision> geometry is plain boxes and cylinders that already describe a square torso
# and thick legs. Using them keeps the drawing consistent with the physics model.
# Three links carry no collision shape at all (the two hip actuators and ankle_pitch);
# those are hand-picked below and tagged so it is obvious which numbers are invented.
_HANDPICKED = {
    # Short canted links between the base and the thigh. Drawn along their own joint axis,
    # which is visually honest about the 45-degree double-diagonal hip design.
    "hip_roll": {"type": "capsule", "radius": 0.045, "length": 0.058, "role": "hip"},
    "hip_yaw": {"type": "capsule", "radius": 0.045, "length": 0.058, "role": "hip"},
    "ankle_pitch": {"type": "box", "size": [0.06, 0.06, 0.05], "role": "ankle"},
}


def collision_shapes(link: ET.Element) -> list[dict]:
    out = []
    for col in link.findall("collision"):
        geom = col.find("geometry")
        origin = col.find("origin")
        xyz = snap3(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0]
        rpy = snap3(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]
        for shape in list(geom):
            if shape.tag == "box":
                out.append({
                    "type": "box",
                    "size": [clean(v) for v in snap3(shape.get("size"))],
                    "xyz": [clean(v) for v in xyz],
                    "rpy": [clean(v) for v in rpy],
                    "source": "urdf_collision",
                })
            elif shape.tag == "cylinder":
                out.append({
                    "type": "cylinder",
                    "radius": clean(float(shape.get("radius"))),
                    "length": clean(float(shape.get("length"))),
                    "xyz": [clean(v) for v in xyz],
                    "rpy": [clean(v) for v in rpy],
                    "source": "urdf_collision",
                })
    return out


def shape_role(link_name: str) -> str:
    if link_name == "base":
        return "torso"
    for key, role in (("hip_pitch", "thigh"), ("knee_pitch", "shin"),
                      ("ankle_roll", "foot"), ("hip_roll", "hip"),
                      ("hip_yaw", "hip"), ("ankle_pitch", "ankle")):
        if link_name.endswith(key):
            return role
    return "link"


def build(contract: LegPolicyContract) -> dict:
    root = ET.parse(URDF_PATH).getroot()
    joints_by_urdf = {j.get("name"): j for j in root.findall("joint")}
    links_by_name = {lk.get("name"): lk for lk in root.findall("link")}

    joints = []
    for index, name in enumerate(contract.joint_order):
        el = joints_by_urdf[urdf_name(name)]
        origin = el.find("origin")
        xyz = snap3(origin.get("xyz"))
        rpy = snap3(origin.get("rpy"))
        axis = snap3(el.find("axis").get("xyz"))
        limit = el.find("limit")
        joints.append({
            "name": name,
            "index": index,
            "urdf_name": el.get("name"),
            "parent": el.find("parent").get("link"),
            "child": el.find("child").get("link"),
            "xyz": [clean(v) for v in xyz],
            "rpy": [clean(v) for v in rpy],
            "axis": [clean(v) for v in axis],
            # Precomputed so the frontend never re-derives the Rz*Ry*Rx ordering.
            "R": [clean(v) for v in rpy_to_matrix(*rpy).flatten()],
            "limit": {
                "lower": clean(snap(float(limit.get("lower")))),
                "upper": clean(snap(float(limit.get("upper")))),
            },
        })

    # Links referenced by the leg chain, plus the root.
    wanted = ["base"] + [j["child"] for j in joints]
    links = []
    for link_name in wanted:
        shapes = collision_shapes(links_by_name[link_name])
        role = shape_role(link_name)
        if not shapes:
            spec = None
            for key, hp in _HANDPICKED.items():
                if link_name.endswith(key):
                    spec = hp
                    break
            if spec is not None:
                shapes = [{**spec, "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
                           "source": "handpicked"}]
        for s in shapes:
            s.setdefault("role", role)
        links.append({"name": link_name, "shapes": shapes})

    # The IMU is a fixed frame on the base. The wireframe orients the whole body from its
    # gravity reading, so it is worth marking where the sensor physically sits — a mounting
    # error is one of the things this tool should be able to make visible.
    imu = None
    imu_chain = [joints_by_urdf.get("imu"), joints_by_urdf.get("imu_frame")]
    if all(j is not None for j in imu_chain):
        pos = np.zeros(3)
        rot = np.eye(3)
        for el in imu_chain:
            origin = el.find("origin")
            xyz = snap3(origin.get("xyz", "0 0 0"))
            rpy = snap3(origin.get("rpy", "0 0 0"))
            pos = pos + rot @ np.array(xyz)
            rot = rot @ rpy_to_matrix(*rpy)
        imu = {
            "parent": imu_chain[0].find("parent").get("link"),
            "xyz": [clean(v) for v in pos],
            "R": [clean(v) for v in rot.flatten()],
        }

    sign = contract.policy_frame_sign
    default_urdf = (contract.default_pose * sign).astype(float)

    return {
        "_meta": {
            "schema_version": 1,
            "generated_by": "scripts/gen_viz_kinematics.py",
            "source_urdf": str(URDF_PATH),
            "source_repo_sha": source_sha(),
            "frame": (
                "URDF frame. Feed q_urdf[i] = joint_sign[i] * telemetry_position[i]; "
                "telemetry from the daemon is DEVICE frame."
            ),
            "conventions": (
                "URDF fixed-axis rpy (R = Rz*Ry*Rx); child = T(xyz)*R(rpy)*R_axis(q); "
                "+X forward, +Y left, +Z up; metres and radians."
            ),
            "note": (
                "Derived artifact - do not hand-edit. Regenerate with "
                "`python scripts/gen_viz_kinematics.py` if the URDF changes."
            ),
        },
        "joint_order": list(contract.joint_order),
        # Mirrors humanoid_control.config.LegPolicyContract.policy_frame_sign. The app
        # cross-checks this against GET /api/contract and refuses to draw on a mismatch.
        "joint_sign": [int(s) for s in sign],
        "root_link": "base",
        "imu": imu,
        "joints": joints,
        "links": links,
        "poses": {
            "zero": [0.0] * contract.num_joints,
            "default": [clean(v) for v in default_urdf],
        },
        "view": {
            "bounds": {"x": [-0.25, 0.25], "y": [-0.25, 0.25], "z": [0.0, 0.90]},
        },
    }


def source_sha() -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(URDF_PATH.parent), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if the checked-in JSON differs from a fresh generation")
    args = ap.parse_args()

    if not URDF_PATH.exists():
        print(f"error: URDF not found at {URDF_PATH}\n"
              f"       The humanoid-policy repo must be checked out beside this one.",
              file=sys.stderr)
        return 2

    contract = LegPolicyContract.load()
    doc = build(contract)
    text = json.dumps(doc, indent=2) + "\n"

    if args.check:
        if not OUT_PATH.exists():
            print(f"error: {OUT_PATH} is missing; run without --check.", file=sys.stderr)
            return 1
        current = OUT_PATH.read_text()
        # The SHA changes whenever the sibling repo moves; compare everything else.
        if _strip_sha(current) != _strip_sha(text):
            print(f"error: {OUT_PATH.relative_to(REPO_ROOT)} is stale.\n"
                  f"       Regenerate: python scripts/gen_viz_kinematics.py",
                  file=sys.stderr)
            return 1
        print(f"ok: {OUT_PATH.relative_to(REPO_ROOT)} matches the URDF.")
        return 0

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(text)
    n_hand = sum(1 for lk in doc["links"] for s in lk["shapes"]
                 if s.get("source") == "handpicked")
    print(f"wrote {OUT_PATH.relative_to(REPO_ROOT)}: "
          f"{len(doc['joints'])} joints, {len(doc['links'])} links "
          f"({n_hand} hand-picked shapes), sign={doc['joint_sign']}")
    return 0


def _strip_sha(text: str) -> str:
    doc = json.loads(text)
    doc["_meta"].pop("source_repo_sha", None)
    return json.dumps(doc, indent=2, sort_keys=True)


if __name__ == "__main__":
    raise SystemExit(main())
