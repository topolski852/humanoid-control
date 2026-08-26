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
from humanoid_control.layout import LIMB_JOINTS, LIMB_ORDER

# The FULL-BODY urdf (not humanoid_biped.urdf, which is legs only) — the arms are half the
# reason this file exists now. Searched rather than hardcoded: the sibling checkout has moved
# once already, and a stale absolute path fails as "URDF not found" long after the move.
_URDF_CANDIDATES = (
    Path("~/humanoid/humanoid-policy/source/humanoid_policy_assets/data/robots/humanoid/urdf"
         "/humanoid.urdf").expanduser(),
    Path("~/humanoid-policy/source/humanoid_policy_assets/data/robots/humanoid/urdf"
         "/humanoid.urdf").expanduser(),
)


def _find_urdf() -> Path:
    for p in _URDF_CANDIDATES:
        if p.exists():
            return p
    return _URDF_CANDIDATES[0]


URDF_PATH = _find_urdf()
OUT_PATH = REPO_ROOT / "app" / "src" / "data" / "viz_kinematics.json"

# Measured dimensions that override the URDF's COSMETIC geometry (see the file's own _meta for
# why). Joint origins are never overridden — those are the sim/real contract the policy was
# trained against, and this file has no business touching them.
DIMS_PATH = REPO_ROOT / "configs" / "robot_dimensions.json"


def load_dimensions() -> dict:
    if not DIMS_PATH.exists():
        return {}
    return json.loads(DIMS_PATH.read_text())

# --- device joint name -> URDF joint name -----------------------------------
# DEVICE names are authoritative everywhere in this runtime: they are what the ESCs, the robot
# config and the daemon use. The URDF is the trainer's asset and disagrees in two ways:
#   - it prefixes every joint with its limb (`leg_` / `arm_`)
#   - it calls the arm's fifth joint `elbow_roll`; the hardware calls it `wrist_yaw`. Same
#     physical joint (5th in the chain, distal to elbow_pitch, spins the hand), different name.
#     Berkeley's own docs name it the wrist, so the URDF is the odd one out.
_URDF_JOINT_RENAME = {
    "left_wrist_yaw_joint": "arm_left_elbow_roll_joint",
    "right_wrist_yaw_joint": "arm_right_elbow_roll_joint",
}


def urdf_joint_name(device_name: str, limb: str) -> str:
    """Device joint name -> URDF joint name."""
    if device_name in _URDF_JOINT_RENAME:
        return _URDF_JOINT_RENAME[device_name]
    prefix = "leg" if limb.endswith("_leg") else "arm"
    return f"{prefix}_{device_name}"

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


def torso_half_extent_y(link: ET.Element | None) -> float | None:
    """Half-width of the torso in world +Y, from the root link's collision box.

    Needed so an arm's mounting strut can start at the side of the chest instead of at the
    centreline. The box carries its own rpy (the humanoid's is yawed 90 deg, which swaps its
    x and y extents), so project the rotated half-extents rather than reading `size[1]`.
    """
    if link is None:
        return None
    best = None
    for col in link.findall("collision"):
        geom = col.find("geometry")
        box = geom.find("box") if geom is not None else None
        if box is None:
            continue
        half = np.array(snap3(box.get("size"))) / 2.0
        origin = col.find("origin")
        rpy = snap3(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]
        y0 = float(snap3(origin.get("xyz", "0 0 0"))[1]) if origin is not None else 0.0
        R = np.abs(rpy_to_matrix(*rpy))
        extent = float(R[1] @ half) + abs(y0)      # support of the rotated box along +Y
        best = extent if best is None else max(best, extent)
    return best


def j_axis_of(joints: list[dict], name: str) -> list[float]:
    """The local rotation axis of a joint already assembled into `joints`."""
    for j in joints:
        if j["name"] == name:
            return j["axis"]
    return [0.0, 0.0, 1.0]


def chain_frames(limb_joints: list[dict]) -> tuple[dict, dict]:
    """World joint origins and world rotation axes for one limb at the ZERO pose.

    Zero pose is the right reference here: it is the configuration the model is authored in, so
    "is this joint's axis inline with the limb" is a property of the geometry rather than of
    whatever pose the robot happens to be holding.
    """
    M = np.eye(4)
    origins, axes = {}, {}
    for j in limb_joints:
        R = np.array(j["R"], dtype=float).reshape(3, 3)
        step = np.eye(4)
        step[:3, :3] = R
        step[:3, 3] = j["xyz"]
        M = M @ step
        origins[j["name"]] = M[:3, 3].copy()
        axes[j["name"]] = (M[:3, :3] @ np.array(j["axis"], dtype=float)).copy()
        # zero pose: the joint rotation is identity, so nothing more to compose
    return origins, axes


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
    # Arm links: only shoulder_yaw and elbow_roll carry a collision cylinder in the URDF, so
    # the shoulder actuators and the hand need widths. Narrower than the legs because they are.
    "shoulder_pitch": {"type": "capsule", "radius": 0.032, "length": 0.05, "role": "shoulder"},
    "shoulder_roll": {"type": "capsule", "radius": 0.030, "length": 0.10, "role": "upper_arm"},
    "elbow_pitch": {"type": "capsule", "radius": 0.028, "length": 0.10, "role": "forearm"},
    "hand_link": {"type": "capsule", "radius": 0.026, "length": 0.06, "role": "hand"},
}


def apply_measured_torso(shapes: list[dict], torso: dict) -> list[dict]:
    """Resize the torso box to the MEASURED extents, keeping the URDF box's centre.

    The URDF's torso is 10 cm narrower than the machine that got built. Drawn that way, the
    shoulder — whose origin is 13.3 cm off the centreline — floats 5.8 cm clear of the body on a
    long strut, and reads as being mounted in the middle of the chest. At the measured 25 cm
    width the same joint sits 0.8 cm outboard: bolted to the corner, which is where it is.
    """
    if not torso:
        return shapes
    size = [torso.get("width_y"), torso.get("depth_x"), torso.get("height_z")]
    if any(v is None for v in size):
        return shapes
    for s in shapes:
        if s.get("type") == "box" and s.get("role") == "torso":
            s["urdf_size"] = s["size"]
            s["size"] = [clean(v) for v in size]
            s["source"] = "measured"
    return shapes


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
                      ("hip_yaw", "hip"), ("ankle_pitch", "ankle"),
                      ("shoulder_pitch", "shoulder"), ("shoulder_roll", "upper_arm"),
                      ("shoulder_yaw", "upper_arm"), ("elbow_pitch", "forearm"),
                      ("elbow_roll", "wrist"), ("hand_link", "hand")):
        if link_name.endswith(key):
            return role
    return "link"


def build(contract: LegPolicyContract) -> dict:
    root = ET.parse(URDF_PATH).getroot()
    joints_by_urdf = {j.get("name"): j for j in root.findall("joint")}
    links_by_name = {lk.get("name"): lk for lk in root.findall("link")}
    root_link_name = "base"
    dims = load_dimensions()

    # Every limb the robot could have, not just the ones attached right now: the model is a
    # SUPERSET and the app selects from it using the live layout. That keeps the bundled asset
    # independent of whatever happens to be plugged into one machine.
    all_joints = [(limb, j) for limb in LIMB_ORDER for j in LIMB_JOINTS[limb]]

    joints = []
    for index, (limb, name) in enumerate(all_joints):
        el = joints_by_urdf[urdf_joint_name(name, limb)]
        origin = el.find("origin")
        xyz = snap3(origin.get("xyz"))
        rpy = snap3(origin.get("rpy"))
        axis = snap3(el.find("axis").get("xyz"))
        limit = el.find("limit")
        joints.append({
            "name": name,
            "index": index,
            "limb": limb,
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

    # --- where each limb meets the torso ---------------------------------
    # A joint hanging off the root gets a strut so the limb is attached to the body rather than
    # floating beside it. WHERE that strut starts differs by limb and the difference matters:
    #   legs — the body centreline. The two hip struts together read as a pelvis, which is what
    #          the real structure is.
    #   arms — the SIDE WALL of the torso. A shoulder bolts to the side of the chest, so running
    #          its strut to the centreline draws the arm as if it grew out of the sternum, and
    #          the bright skeleton line then makes the shoulder look centred on the torso.
    # Measured width wins: the URDF box is 10 cm narrow, and mounting the shoulder to a torso
    # wall that is not where the wall actually is was the whole bug.
    measured_w = (dims.get("torso") or {}).get("width_y")
    torso_half_y = (measured_w / 2.0) if measured_w else \
        torso_half_extent_y(links_by_name.get(root_link_name))
    for j in joints:
        x, y, z = j["xyz"]
        if j["parent"] != root_link_name:
            continue
        if j["limb"].endswith("_arm") and torso_half_y is not None:
            # Clamp toward the centreline: never start the strut outboard of the joint itself.
            mount_y = np.sign(y) * min(abs(y), torso_half_y)
            j["mount"] = {"xyz": [clean(x), clean(float(mount_y)), clean(z)],
                          "source": "torso_surface"}
        else:
            j["mount"] = {"xyz": [clean(x), 0.0, clean(z)], "source": "centreline"}

    # --- inline twist joints ---------------------------------------------
    # A joint whose axis points straight down the limb rotates the limb about its own centreline,
    # so a centreline drawing shows NOTHING when it moves — however correct the kinematics are.
    # Detected geometrically (axis parallel to the direction from this joint to the end of its
    # chain, in the zero pose) rather than by name, so it stays true if the URDF changes.
    for limb in LIMB_ORDER:
        limb_joints = [j for j in joints if j["limb"] == limb]
        if len(limb_joints) < 2:
            continue
        origins, axes = chain_frames(limb_joints)
        tip_pos = origins[limb_joints[-1]["name"]]
        for j in limb_joints[:-1]:
            v = tip_pos - origins[j["name"]]
            n = float(np.linalg.norm(v))
            if n < 1e-9:
                continue
            if abs(float(np.dot(axes[j["name"]], v / n))) > 0.9:
                j["twist"] = True

    # Terminal stubs. A joint with nothing distal to it draws nothing when it rotates, which
    # would make the wrist — the joint whose sign is hardest to eyeball — invisible. The hand
    # hangs off a FIXED joint whose own origin is coincident with the wrist, so the useful point
    # is the hand's CENTRE OF MASS: real URDF geometry, ~5 cm out, and it swings with the wrist.
    for side, fixed_name in (("left", "arm_left_hand_l"), ("right", "arm_hand_r")):
        el = joints_by_urdf.get(fixed_name)
        if el is None:
            continue
        hand_link = links_by_name.get(el.find("child").get("link"))
        inertial = hand_link.find("inertial") if hand_link is not None else None
        if inertial is None:
            continue
        origin = el.find("origin")
        t = np.array(snap3(origin.get("xyz", "0 0 0")))
        r = rpy_to_matrix(*snap3(origin.get("rpy", "0 0 0")))
        com = np.array([float(v) for v in inertial.find("origin").get("xyz").split()])
        tip = t + r @ com                       # in the wrist's child-link frame
        # Jaw half-span from the wrist link's own collision cylinder, so the claw is drawn at
        # the width of the real hardware rather than an arbitrary size.
        wrist_link = links_by_name.get(f"arm_{side}_elbow_roll")
        span = 0.03
        for col in (wrist_link.findall("collision") if wrist_link is not None else []):
            cyl = col.find("geometry/cylinder")
            if cyl is not None:
                span = float(cyl.get("radius"))
        # The claw is a real off-the-shelf part (a servo "BigClaw" gripper), so its reach is a
        # measured fact, not a drawing choice. The URDF knows nothing about it — it models the
        # hand as one fixed link — so `configs/robot_dimensions.json` is the only source.
        hand_dims = dims.get("hand") or {}
        hand_len = hand_dims.get("length_closed")
        axis = np.array(j_axis_of(joints, f"{side}_wrist_yaw_joint"), dtype=float)
        reach = float(np.dot(tip, axis / (np.linalg.norm(axis) or 1.0)))
        # Jaw length is whatever carries the fingertip from the palm out to the measured length.
        jaw_length = max(0.01, hand_len - reach) if hand_len else 0.05
        for j in joints:
            if j["name"] == f"{side}_wrist_yaw_joint":
                j["tip"] = {"xyz": [clean(v) for v in tip], "source": "urdf_hand_com"}
                j["hand"] = {
                    "palm": [clean(v) for v in tip],
                    "axis": [clean(v) for v in j["axis"]],
                    "jaw_length": clean(jaw_length),
                    "jaw_span": clean(span),
                    "length_closed": clean(hand_len) if hand_len else None,
                    "part": hand_dims.get("part"),
                    "source": "measured" if hand_len else "indicator",
                    # The claw's REACH is measured; its jaw SHAPE is still drawn — the open span
                    # has not been measured, so the splay is nominal. See robot_dimensions.json.
                    "note": "reach measured (wrist pivot to closed fingertip); jaw span from the "
                            "wrist collision radius; open splay nominal until measured",
                }

    # Links referenced by every chain, plus the root.
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
        if link_name == root_link_name:
            shapes = apply_measured_torso(shapes, dims.get("torso", {}))
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

    # Frame sign, per joint of the superset.
    #   legs — from the policy contract (the URDF is mirror-symmetric, the device frame is not).
    #   arms — +1. There is no trained arm policy, so there is no frame to reconcile TO, and
    #          inventing one would be the very mistake this visualizer exists to catch. +1 means
    #          the arm is drawn at its RAW device angle, so any disagreement with the physical
    #          arm is a real finding about the device frame rather than something a fudge factor
    #          has already absorbed.
    contract_sign = dict(zip(contract.joint_order, contract.policy_frame_sign))
    contract_default = dict(zip(contract.joint_order, contract.default_pose))
    sign = [int(contract_sign.get(name, 1)) for _, name in all_joints]
    # default_pose likewise only exists for the contract joints; arms get 0 (drawn at zero) and
    # are flagged so the app never presents that as a meaningful "default".
    default_urdf = [clean(float(contract_default[name]) * contract_sign[name])
                    if name in contract_default else 0.0
                    for _, name in all_joints]
    has_default = [name in contract_default for _, name in all_joints]

    return {
        "_meta": {
            "schema_version": 2,
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
            "naming": (
                "DEVICE joint names are authoritative. `urdf_name` records the URDF's name for "
                "the same joint; note the arm's 5th joint is wrist_yaw on the hardware and "
                "elbow_roll in the URDF."
            ),
            "note": (
                "Derived artifact - do not hand-edit. Regenerate with "
                "`python scripts/gen_viz_kinematics.py` if the URDF changes."
            ),
        },
        # The full superset of joints the robot could have, in humanoid_control.layout order.
        # The app renders the subset the live layout enables.
        "joint_order": [name for _, name in all_joints],
        "limbs": {limb: list(LIMB_JOINTS[limb]) for limb in LIMB_ORDER},
        # For legs this mirrors humanoid_control.config.LegPolicyContract.policy_frame_sign, and
        # the app cross-checks it against GET /api/contract and refuses to draw on a mismatch.
        # Arms are +1 (raw device angle) — see the comment where this is built.
        "joint_sign": [int(s) for s in sign],
        "root_link": "base",
        "imu": imu,
        "joints": joints,
        "links": links,
        "poses": {
            "zero": [0.0] * len(all_joints),
            "default": list(default_urdf),
            # False where `default` is a filler zero rather than a trained default pose.
            "default_known": has_default,
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
