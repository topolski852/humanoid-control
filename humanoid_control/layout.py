"""
RobotLayout — which limbs are physically attached to THIS machine.

Deliberately SEPARATE from :class:`~humanoid_control.config.LegPolicyContract`. The contract is
the sim↔real interface for the trained leg policy and must stay exactly what the trainer agreed
to; it is not a description of the hardware in the room. This module answers the different
question "what is plugged in right now", so the runtime can serve one arm on a bench, a pair of
legs on a gantry, or the full robot, without touching the policy contract.

The layout lives in a machine-local file (``~/.config/humanoid-control/robot_layout.json``) so
the bench PC and the torso PC each configure themselves and a ``git pull`` cannot clobber either.
When the file is absent the default is **both legs, no arms** — byte-identical to the behaviour
before this module existed, so nothing regresses for work in flight.

Joint membership per limb is a catalog HERE, in code (device joint names, matching the studio
robot config). The file only records which limbs are present, so a hand-edit can't invent a
joint the daemon has never heard of.

Device joint names are authoritative throughout. Note the arm's fifth joint is
``{side}_wrist_yaw_joint``: the URDF asset calls the same physical joint ``elbow_roll``. See
``scripts/gen_viz_kinematics.py`` for that mapping.
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, replace
from pathlib import Path

_log = logging.getLogger(__name__)

SCHEMA_VERSION = 1

# Canonical limb order. Legs first, and in this order, so that with both legs enabled the first
# twelve entries of ``joint_order`` are exactly ``LegPolicyContract.joint_order`` — the policy
# path and every index-aligned consumer keep working untouched.
LIMB_ORDER = ("left_leg", "right_leg", "left_arm", "right_arm")

_LEG_TYPES = ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")
_ARM_TYPES = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "wrist_yaw")

LIMB_JOINTS: dict[str, tuple[str, ...]] = {
    f"{side}_{kind}": tuple(f"{side}_{t}_joint" for t in types)
    for side in ("left", "right")
    for kind, types in (("leg", _LEG_TYPES), ("arm", _ARM_TYPES))
}

LIMB_BUS: dict[str, str] = {limb: f"can_{limb}" for limb in LIMB_ORDER}

LIMB_LABEL: dict[str, str] = {
    "left_leg": "Left leg", "right_leg": "Right leg",
    "left_arm": "Left arm", "right_arm": "Right arm",
}

DEFAULT_ENABLED = ("left_leg", "right_leg")

_ENV_PATH = "HUMANOID_LAYOUT"


def default_layout_path() -> Path:
    """Machine-local layout file. ``$HUMANOID_LAYOUT`` overrides (handy for tests)."""
    env = os.environ.get(_ENV_PATH)
    if env:
        return Path(env).expanduser()
    base = os.environ.get("XDG_CONFIG_HOME") or "~/.config"
    return Path(base).expanduser() / "humanoid-control" / "robot_layout.json"


@dataclass(frozen=True)
class RobotLayout:
    """Immutable view of what hardware this machine expects to drive."""

    enabled: tuple[str, ...] = DEFAULT_ENABLED     # subset of LIMB_ORDER, in LIMB_ORDER order
    imu_expected: bool = True
    robot_name: str = "humanoid_lite"
    source: Path | None = None                    # file it came from; None = built-in default

    # --- derived ---------------------------------------------------------
    @property
    def joint_order(self) -> tuple[str, ...]:
        """Every enabled joint, limbs in ``LIMB_ORDER``, joints in catalog order."""
        return tuple(j for limb in self.enabled for j in LIMB_JOINTS[limb])

    @property
    def buses(self) -> tuple[str, ...]:
        return tuple(LIMB_BUS[limb] for limb in self.enabled)

    @property
    def has_both_legs(self) -> bool:
        """True when the 12-joint leg policy can run at all."""
        return "left_leg" in self.enabled and "right_leg" in self.enabled

    def is_enabled(self, limb: str) -> bool:
        return limb in self.enabled

    def limb_of(self, joint_name: str) -> str | None:
        for limb, joints in LIMB_JOINTS.items():
            if joint_name in joints:
                return limb
        return None

    def with_enabled(self, enabled) -> "RobotLayout":
        return replace(self, enabled=_normalize(enabled))

    def describe(self) -> str:
        """Short human label, e.g. 'legs' / 'left arm' / 'legs + both arms'."""
        if not self.enabled:
            return "nothing configured"
        legs = [l for l in self.enabled if l.endswith("_leg")]
        arms = [l for l in self.enabled if l.endswith("_arm")]
        parts = []
        if len(legs) == 2:
            parts.append("legs")
        elif legs:
            parts.append(LIMB_LABEL[legs[0]].lower())
        if len(arms) == 2:
            parts.append("both arms")
        elif arms:
            parts.append(LIMB_LABEL[arms[0]].lower())
        return " + ".join(parts)

    # --- validation ------------------------------------------------------
    def missing_joints(self, robot_config) -> dict[str, list[str]]:
        """Enabled joints the robot config has never heard of, grouped by limb.

        A layout naming joints the daemon cannot address is a configuration error, not a runtime
        one — surface it at load/save time rather than as a mystery OFFLINE joint later.
        """
        if robot_config is None:
            return {}
        known = set(robot_config.joints)
        out: dict[str, list[str]] = {}
        for limb in self.enabled:
            missing = [j for j in LIMB_JOINTS[limb] if j not in known]
            if missing:
                out[limb] = missing
        return out

    # --- serialization ---------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "_meta": {
                "schema_version": SCHEMA_VERSION,
                "note": "What hardware is attached to THIS machine. Written by the web "
                        "Settings tab; safe to hand-edit. Joint membership per limb lives in "
                        "humanoid_control/layout.py, not here.",
            },
            "robot_name": self.robot_name,
            "limbs": {limb: {"enabled": limb in self.enabled} for limb in LIMB_ORDER},
            "imu": {"expected": self.imu_expected},
        }

    @classmethod
    def from_dict(cls, data: dict, *, source: Path | None = None) -> "RobotLayout":
        limbs = data.get("limbs") or {}
        enabled = _normalize(
            limb for limb in LIMB_ORDER if bool((limbs.get(limb) or {}).get("enabled"))
        )
        return cls(
            enabled=enabled,
            imu_expected=bool((data.get("imu") or {}).get("expected", True)),
            robot_name=str(data.get("robot_name") or "humanoid_lite"),
            source=source,
        )

    def save(self, path: str | Path | None = None) -> Path:
        """Atomically write the layout. Creates the parent directory if needed."""
        p = Path(path) if path is not None else (self.source or default_layout_path())
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp = p.with_name(p.name + ".tmp")
        tmp.write_text(json.dumps(self.to_dict(), indent=2) + "\n")
        tmp.replace(p)
        _log.info("robot layout saved to %s (%s)", p, self.describe())
        return p


def _normalize(enabled) -> tuple[str, ...]:
    """Dedupe, drop unknown limbs, and force canonical LIMB_ORDER ordering."""
    given = set(enabled or ())
    unknown = given - set(LIMB_ORDER)
    if unknown:
        raise ValueError(f"unknown limb(s): {', '.join(sorted(unknown))}")
    return tuple(limb for limb in LIMB_ORDER if limb in given)


def load(path: str | Path | None = None) -> RobotLayout:
    """Load the layout, falling back to the legs-only default when the file is absent.

    A corrupt file falls back too (with a warning) rather than taking the server down: the
    Settings tab is how you'd fix it, and it can't be reached if startup fails.
    """
    p = Path(path) if path is not None else default_layout_path()
    if not p.exists():
        _log.info("no robot layout at %s — defaulting to legs only.", p)
        return RobotLayout()
    try:
        return RobotLayout.from_dict(json.loads(p.read_text()), source=p)
    except Exception as exc:
        _log.warning("robot layout %s is unreadable (%s) — defaulting to legs only.", p, exc)
        return RobotLayout()
