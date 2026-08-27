"""
Per-operator arm calibration profiles.

WHY THIS EXISTS. The Quest infers your upper body from the headset and controllers — nothing
watches your torso — so the retargeted angles carry systematic offsets, not just noise.
Measured on this operator holding a genuine T-pose with a straight arm, the retargeter
reported **21.6 degrees of elbow flexion**, because the inferred elbow sits ~5 cm off the true
shoulder-wrist line. Drive the robot from raw angles and it sits permanently bent.

That offset barely moves between samples (sd ~1-2 deg), which is exactly what a capture can
remove: record what the tracker claims for a known pose, subtract it forever after.

The profile also solves RANGE. The robot's joints are far tighter than a human's
(shoulder_yaw +/-45 deg against your ~90+, elbow 0-90, roll -15..+75), so without knowing
YOUR comfortable range the mapping either saturates the moment you move or wastes most of the
robot's travel.

STORAGE follows the two patterns already in this repo rather than inventing a third:
:func:`layout.default_layout_path` for the machine-local config location, and ``poses.py``'s
atomic keyed-entry write. Profiles are keyed by name from the start — only "default" is used
today, but that makes multiple operators a UI change later rather than a file migration.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SCHEMA = 2   # 2: ranges stored UNWRAPPED relative to the zero
DEFAULT_NAME = "default"

# Robot joint order this profile maps onto, proximal to distal. Matches ArmChain.
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist")

# Gain guards. A human range measured over a few degrees (operator barely moved during the
# sweep) would otherwise produce an enormous gain and a robot that lurches across its whole
# travel for a twitch.
MIN_HUMAN_SPAN_RAD = np.radians(12.0)
MAX_GAIN = 3.0


def default_profile_path() -> Path:
    """Machine-local profile store. ``$HUMANOID_ARM_PROFILES`` overrides (handy for tests)."""
    env = os.environ.get("HUMANOID_ARM_PROFILES")
    if env:
        return Path(env).expanduser()
    base = os.environ.get("XDG_CONFIG_HOME") or "~/.config"
    return Path(base).expanduser() / "humanoid-control" / "arm_profiles.json"


def _atomic_write(path: Path, data: dict) -> None:
    """Write via a temp file + rename, so a crash mid-write cannot leave a truncated profile
    that would silently mis-map every joint on the next session."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)


@dataclass
class ArmProfile:
    """One operator's arm calibration."""

    name: str = DEFAULT_NAME
    schema: int = SCHEMA
    captured_utc: str = ""
    # Angles (rad) the tracker reports when the operator's arm is RELAXED at their side.
    # This is the zero: subtracting it is what removes the systematic offset.
    zero_rad: list[float] = field(default_factory=lambda: [0.0] * len(JOINTS))
    # Observed min/max (rad) per DOF across the whole guided sweep — the operator's usable
    # range, which is what gets mapped onto the robot's much tighter one.
    lo_rad: list[float] = field(default_factory=lambda: [0.0] * len(JOINTS))
    hi_rad: list[float] = field(default_factory=lambda: [0.0] * len(JOINTS))
    upper_len_m: float = 0.0
    fore_len_m: float = 0.0

    # ── persistence ─────────────────────────────────────────────────────────
    def to_dict(self) -> dict:
        return {"schema": self.schema, "captured_utc": self.captured_utc,
                "zero_rad": list(self.zero_rad), "lo_rad": list(self.lo_rad),
                "hi_rad": list(self.hi_rad), "joints": list(JOINTS),
                "upper_len_m": self.upper_len_m, "fore_len_m": self.fore_len_m}

    @classmethod
    def from_dict(cls, name: str, d: dict) -> "ArmProfile":
        n = len(JOINTS)
        def _vec(key):
            v = list(d.get(key) or [])
            return (v + [0.0] * n)[:n]
        return cls(name=name, schema=int(d.get("schema", SCHEMA)),
                   captured_utc=str(d.get("captured_utc", "")),
                   zero_rad=_vec("zero_rad"), lo_rad=_vec("lo_rad"), hi_rad=_vec("hi_rad"),
                   upper_len_m=float(d.get("upper_len_m") or 0.0),
                   fore_len_m=float(d.get("fore_len_m") or 0.0))

    @classmethod
    def from_capture(cls, captured: dict, *, name: str = DEFAULT_NAME,
                     captured_utc: str = "") -> "ArmProfile":
        """Build from CalibrationRun.captured — {pose_key: {angles: [...], ...}}.

        The ZERO comes from the `relaxed` pose, deliberately not the T-pose: at 90 degrees of
        abduction pitch and yaw stop being separable for any Euler decomposition, so a T-pose
        is a poor place to measure angles from. It is still the right place to measure segment
        LENGTHS, which is why it stays in the sequence.
        """
        n = len(JOINTS)
        poses = {k: np.asarray(v["angles"], dtype=float)
                 for k, v in captured.items() if v.get("angles")}
        if not poses:
            raise ValueError("no captured poses")
        zero = poses.get("relaxed")
        if zero is None:
            zero = np.mean(np.stack(list(poses.values())), axis=0)
        allp = np.stack(list(poses.values()))
        # UNWRAP every pose relative to the zero before taking min/max. A wrist sitting near
        # +/-180 otherwise yields lo=-180, hi=+180 — a "full circle range" that makes the
        # gain calculation meaningless and the joint barely move.
        allp = zero + np.arctan2(np.sin(allp - zero), np.cos(allp - zero))
        lens = [v for v in captured.values() if v.get("upper_len")]
        return cls(
            name=name, captured_utc=captured_utc,
            zero_rad=[float(v) for v in zero[:n]],
            lo_rad=[float(v) for v in allp.min(axis=0)[:n]],
            hi_rad=[float(v) for v in allp.max(axis=0)[:n]],
            upper_len_m=float(np.mean([v["upper_len"] for v in lens])) if lens else 0.0,
            fore_len_m=float(np.mean([v["fore_len"] for v in lens])) if lens else 0.0,
        )

    # ── the mapping ─────────────────────────────────────────────────────────
    def to_robot(self, human_rad, chain) -> np.ndarray:
        """Operator's arm angles → robot joint targets (rad), clamped to the joint limits.

        Two-sided gain anchored at the zero, rather than a single linear fit across the whole
        range. Anchoring matters: it guarantees that a relaxed arm maps to the robot's rest
        pose exactly, so "let your arm hang and the robot hangs" is true by construction and
        does not drift as the operator's range estimate changes. A single fit would put the
        rest pose wherever the arithmetic happened to land.

        Each direction gets its own gain because human and robot ranges are asymmetric — your
        shoulder abducts far more than it adducts, and the robot's roll limits (-15..+75) are
        lopsided the same way. One gain would waste travel on one side and saturate the other.
        """
        h = np.asarray(human_rad, dtype=float).reshape(len(JOINTS))
        z = np.asarray(self.zero_rad, dtype=float)
        # Take the SHORTEST way round from the zero. Without this a wrist at -179 deg reads
        # as 358 degrees of excursion from a zero at +179, and slams the joint to its limit.
        h = z + np.arctan2(np.sin(h - z), np.cos(h - z))
        lo = np.asarray(self.lo_rad, dtype=float)
        hi = np.asarray(self.hi_rad, dtype=float)

        r_lo = np.asarray(chain.limits_lower, dtype=float)
        r_hi = np.asarray(chain.limits_upper, dtype=float)
        # The robot's zero IS its URDF zero. Mapping the operator's relaxed pose onto it means
        # "relax and the robot relaxes" — the most predictable anchor available, and the one
        # the operator can reproduce without thinking.
        r_zero = np.clip(np.zeros(len(JOINTS)), r_lo, r_hi)

        out = np.empty(len(JOINTS))
        for i in range(len(JOINTS)):
            d = h[i] - z[i]
            if d >= 0:
                span = max(hi[i] - z[i], MIN_HUMAN_SPAN_RAD)
                gain = min((r_hi[i] - r_zero[i]) / span, MAX_GAIN)
            else:
                span = max(z[i] - lo[i], MIN_HUMAN_SPAN_RAD)
                gain = min((r_zero[i] - r_lo[i]) / span, MAX_GAIN)
            out[i] = r_zero[i] + d * max(gain, 0.0)
        return chain.clamp(out)


# ── store ───────────────────────────────────────────────────────────────────
def load_all(path: Path | None = None) -> dict:
    p = path or default_profile_path()
    if not p.is_file():
        return {}
    try:
        return json.loads(p.read_text()).get("profiles", {}) or {}
    except Exception:                                    # noqa: BLE001
        return {}


def load(name: str = DEFAULT_NAME, path: Path | None = None) -> ArmProfile | None:
    d = load_all(path).get(name)
    return ArmProfile.from_dict(name, d) if d else None


def save(profile: ArmProfile, path: Path | None = None) -> Path:
    p = path or default_profile_path()
    all_p = load_all(p)
    all_p[profile.name] = profile.to_dict()
    _atomic_write(p, {"schema": SCHEMA, "profiles": all_p})
    return p


def delete(name: str, path: Path | None = None) -> bool:
    p = path or default_profile_path()
    all_p = load_all(p)
    if name not in all_p:
        return False
    del all_p[name]
    _atomic_write(p, {"schema": SCHEMA, "profiles": all_p})
    return True
