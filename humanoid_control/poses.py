"""
Named leg poses loaded from ``configs/poses.json`` (values in degrees).

A pose maps joints → target angles. Keys may be a joint **type** ("hip_pitch" → both legs)
or a full **joint_name** ("left_hip_pitch_joint" or shorthand "left_hip_pitch" → that joint
only, overriding the type). Any leg joint not named in the pose is **skipped** (left
uncommanded). Joints/types in ``_meta.always_skip`` are skipped in every pose (e.g. the
broken ``ankle_roll``).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .config import REPO_ROOT

DEG = math.pi / 180.0
DEFAULT_POSES_PATH = REPO_ROOT / "configs" / "poses.json"

_SIDES = ("left", "right")
_TYPES = ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")
LEG_JOINTS = tuple(f"{s}_{t}_joint" for s in _SIDES for t in _TYPES)


def _split(joint_name: str) -> tuple[str, str]:
    """'left_hip_pitch_joint' -> ('left', 'hip_pitch')."""
    side, rest = joint_name.split("_", 1)
    jtype = rest[: -len("_joint")] if rest.endswith("_joint") else rest
    return side, jtype


def load_poses(path: str | Path = DEFAULT_POSES_PATH) -> dict:
    return json.loads(Path(path).read_text())


def save_pose(name: str, joints_deg: dict[str, float], path: str | Path = DEFAULT_POSES_PATH) -> dict:
    """Create/replace a pose (values in degrees). Keys may be joint types or full names.
    Preserves _meta and other poses. Returns the updated file dict."""
    name = str(name).strip()
    if not name or name.startswith("_"):
        raise ValueError("pose name must be non-empty and not start with '_'")
    p = Path(path)
    data = json.loads(p.read_text()) if p.exists() else {"poses": {}}
    data.setdefault("poses", {})
    clean = {k: float(v) for k, v in joints_deg.items() if not str(k).startswith("_")}
    data["poses"][name] = clean
    _atomic_write(p, data)
    return data


def delete_pose(name: str, path: str | Path = DEFAULT_POSES_PATH) -> dict:
    p = Path(path)
    data = json.loads(p.read_text())
    if name not in data.get("poses", {}):
        raise KeyError(f"pose {name!r} not found")
    del data["poses"][name]
    _atomic_write(p, data)
    return data


def _atomic_write(path: Path, data: dict) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)


def pose_names(data: dict) -> list[str]:
    return list(data.get("poses", {}).keys())


def resolve_pose(data: dict, name: str) -> tuple[dict[str, float], list[str]]:
    """Return (targets_rad {joint_name: rad}, skipped [joint_name]) for pose ``name``.

    Lookup per joint: full joint_name → shorthand 'side_type' → type. Not found → skipped.
    Values in the file are degrees; returned targets are radians.
    """
    if name not in data.get("poses", {}):
        raise KeyError(f"pose {name!r} not in {sorted(pose_names(data))}")
    pose = {k: v for k, v in data["poses"][name].items() if not k.startswith("_")}
    always_skip = set(data.get("_meta", {}).get("always_skip", []))

    targets: dict[str, float] = {}
    skipped: list[str] = []
    for jn in LEG_JOINTS:
        side, jtype = _split(jn)
        if jn in always_skip or jtype in always_skip or f"{side}_{jtype}" in always_skip:
            skipped.append(jn)
            continue
        for key in (jn, f"{side}_{jtype}", jtype):   # most-specific first
            if key in pose:
                targets[jn] = float(pose[key]) * DEG
                break
        else:
            skipped.append(jn)
    return targets, skipped
