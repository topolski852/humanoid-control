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

SCHEMA = 3   # 2: ranges UNWRAPPED relative to the zero.  3: profiles keyed by (name, side)
DEFAULT_NAME = "default"

# Robot joint order this profile maps onto, proximal to distal. Matches ArmChain.
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist")

# Gain guards. A human range measured over a few degrees (operator barely moved during the
# sweep) would otherwise produce an enormous gain and a robot that lurches across its whole
# travel for a twitch.
MIN_HUMAN_SPAN_RAD = np.radians(12.0)
MAX_GAIN = 3.0

# Operator -> robot direction, PER SIDE, index-aligned to JOINTS.
#
# The human and robot joint conventions agree on most axes but not all, and where they differ
# the operator moves one way and the robot is COMMANDED the other. This is measured on the
# robot, not derived: with all four other joints confirmed following correctly, shoulder_pitch
# drove backwards under Quest mirror — raise the arm, the robot lowers it.
#
# This belongs HERE and not in a device/URDF frame sign or a gear_ratio, both of which were
# tried and rejected: they also change what telemetry MEANS, so they move the render and the
# calibration zeros with them. A human-convention mismatch is a property of the retargeting
# only, so it is corrected only in the retargeting.
#
# WHY IT IS PER SIDE. The URDF's arms are exact mirrors — every right-arm limit is the
# negation of the left's (left pitch [-90,45] vs right [-45,90]; left elbow [0,90] vs right
# [-90,0]) — but the HUMAN angle only mirrors on some axes:
#
#   roll, yaw, wrist   derived from u[1] (sideways), which flips sign between arms, so the
#                      human value already mirrors and NO sign is needed.
#   pitch              derived from u[0] (forward), which does NOT flip — forward is forward
#                      for both arms — so the same gesture must be negated for one side.
#   elbow              flexion magnitude, always positive, never mirrors — same treatment.
#
# Measured with a single global sign (2026-09-19): an operator pitch of -30 deg moved the left
# hand 8.3 cm BACKWARD and the right hand 15.9 cm FORWARD, with joint magnitudes 2x apart. The
# right elbow did not respond AT ALL, because its travel above r_zero is exactly 0. Making the
# sign per-side fixed both symptoms in one step and left roll/yaw untouched.
#
# Each entry is DERIVED, not chosen:  sign = urdf_mirror * human_mirror
#
#   urdf_mirror   every right joint's limits are the exact mirror of its left counterpart
#                 (left pitch [-90,+45] vs right [-45,+90], and so on for all five), so this
#                 is -1 for every joint.
#   human_mirror  measured by decomposing a gesture and its mirror image: roll, yaw and wrist
#                 come out OPPOSITE (already mirrored, -1); pitch and elbow come out the SAME
#                 (+1). Yaw and wrist were measured with poses that actually excite them — a
#                 forearm swept about the limb axis — because the arm-forward poses sit on the
#                 Euler degeneracy and answer with noise.
#
# LEFT shoulder_pitch CARRIES A -1, AND IT IS NOT NEGOTIABLE FROM THE URDF ALONE.
#
# Measured on the robot, twice, on builds with ARM_FRAME_SIGN EMPTY (config.py last written
# 15:17, servers started 16:17 and 16:20 — so no device<->URDF negation was in play either
# time): the operator raising their arm drives the left shoulder_pitch the wrong way. The
# flip was briefly removed on the theory that ARM_FRAME_SIGN had caused it; the timestamps
# rule that out, and the operator reported the inversion straight back.
#
# This means the robot's PHYSICAL shoulder_pitch axis runs opposite to the URDF's, so URDF
# space and hardware disagree about which way is forward for that one joint. Until the URDF
# is corrected the discrepancy has to live somewhere, and it lives here, in the one place
# that only affects the teleop command path. scripts/test_arm_direction.py documents the
# conflict rather than asserting the URDF's version of it.
_SIGN_LEFT = (-1.0, 1.0, 1.0, 1.0, 1.0)     # shoulder_pitch: hardware disagrees with the URDF
_SIGN_RIGHT = (1.0, 1.0, 1.0, -1.0, 1.0)    # the same mirror structure, applied to the left row
HUMAN_TO_ROBOT_SIGN = {"left": _SIGN_LEFT, "right": _SIGN_RIGHT}

SIDES = ("left", "right")


def _side_of(side: str) -> str:
    """Normalise 'left'/'right'/'left_arm'/'right_arm' to a bare side."""
    s = str(side).lower()
    return "right" if s.startswith("right") else "left"


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
    # Which of the operator's arms this was captured from. A profile is per (operator, side):
    # the two arms have genuinely different usable ranges, and mapping one arm through the
    # other's spans produces a lopsided robot for a symmetric gesture — measured at 5.7x gain
    # difference on shoulder_roll when both arms shared one profile.
    side: str = "left"
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
    # True when this side did not come from its own capture: a schema-2 profile predates the
    # per-side split, so it was promoted to both arms. The ranges are then one arm's, applied
    # to both. Surfaced rather than hidden so the operator is told to recalibrate instead of
    # being shown a half-measured profile as if it were current.
    migrated_sideless: bool = False

    # ── persistence ─────────────────────────────────────────────────────────
    def to_dict(self) -> dict:
        d = {"schema": self.schema, "side": self.side,
             "captured_utc": self.captured_utc,
             "zero_rad": list(self.zero_rad), "lo_rad": list(self.lo_rad),
             "hi_rad": list(self.hi_rad), "joints": list(JOINTS),
             "upper_len_m": self.upper_len_m, "fore_len_m": self.fore_len_m}
        if self.migrated_sideless:
            d["migrated_sideless"] = True
        return d

    @classmethod
    def from_dict(cls, name: str, d: dict, *, side: str | None = None) -> "ArmProfile":
        n = len(JOINTS)
        def _vec(key):
            v = list(d.get(key) or [])
            return (v + [0.0] * n)[:n]
        return cls(name=name,
                   side=_side_of(side if side is not None else d.get("side", "left")),
                   schema=int(d.get("schema", SCHEMA)),
                   captured_utc=str(d.get("captured_utc", "")),
                   zero_rad=_vec("zero_rad"), lo_rad=_vec("lo_rad"), hi_rad=_vec("hi_rad"),
                   upper_len_m=float(d.get("upper_len_m") or 0.0),
                   fore_len_m=float(d.get("fore_len_m") or 0.0),
                   migrated_sideless=bool(d.get("migrated_sideless")))

    @classmethod
    def from_capture(cls, captured: dict, *, name: str = DEFAULT_NAME,
                     side: str = "left", captured_utc: str = "") -> "ArmProfile":
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
            name=name, side=_side_of(side), captured_utc=captured_utc,
            zero_rad=[float(v) for v in zero[:n]],
            lo_rad=[float(v) for v in allp.min(axis=0)[:n]],
            hi_rad=[float(v) for v in allp.max(axis=0)[:n]],
            upper_len_m=float(np.mean([v["upper_len"] for v in lens])) if lens else 0.0,
            fore_len_m=float(np.mean([v["fore_len"] for v in lens])) if lens else 0.0,
        )

    # ── the mapping ─────────────────────────────────────────────────────────
    def to_robot(self, human_rad, chain, side: str | None = None) -> np.ndarray:
        """Operator's arm angles → robot joint targets (rad), clamped to the joint limits.

        ``side`` selects the operator->robot sign map (see HUMAN_TO_ROBOT_SIGN); it defaults to
        this profile's own side. Passing it explicitly is only for callers that hold a profile
        and a chain from different sides, which is a bug — but an explicit argument makes that
        bug visible rather than silently mapping one arm through the other's conventions.

        Two-sided gain anchored at the zero, rather than a single linear fit across the whole
        range. Anchoring matters: it guarantees that a relaxed arm maps to the robot's rest
        pose exactly, so "let your arm hang and the robot hangs" is true by construction and
        does not drift as the operator's range estimate changes. A single fit would put the
        rest pose wherever the arithmetic happened to land.

        Each direction gets its own gain because human and robot ranges are asymmetric — your
        shoulder abducts far more than it adducts, and the robot's roll limits (-15..+75) are
        lopsided the same way. One gain would waste travel on one side and saturate the other.
        """
        sign = HUMAN_TO_ROBOT_SIGN[_side_of(side if side is not None else self.side)]
        h = np.asarray(human_rad, dtype=float).reshape(len(JOINTS))
        z = np.asarray(self.zero_rad, dtype=float)
        # Take the SHORTEST way round from the zero. Without this a wrist at -179 deg reads
        # as 358 degrees of excursion from a zero at +179, and slams the joint to its limit.
        h = z + np.arctan2(np.sin(h - z), np.cos(h - z))
        lo = np.asarray(self.lo_rad, dtype=float)
        hi = np.asarray(self.hi_rad, dtype=float)

        # SOFT limits, deliberately NOT the firmware's. Two tiers exist:
        #   HARD — humanoid_lite.json `position_limits`, pushed to the ESC by reconcile.py.
        #          The mechanical backstop; what Studio and calibration may reach.
        #   SOFT — the URDF values vendored in app/src/data/viz_kinematics.json, which is what
        #          ArmChain loads and what this mapping uses.
        # These two sources legitimately DISAGREE (hard is wider) and must not be "synced".
        # It matters here more than at the clamp below: r_hi/r_lo also set the GAIN, so the
        # operator's full arm travel maps onto the SOFT range. Feeding the hard range in would
        # silently make headset teleop reach further for the same human motion — the opposite
        # of why the tiers exist, since the operator has least situational awareness in a headset.
        r_lo = np.asarray(chain.limits_lower, dtype=float)
        r_hi = np.asarray(chain.limits_upper, dtype=float)
        # The robot's zero IS its URDF zero. Mapping the operator's relaxed pose onto it means
        # "relax and the robot relaxes" — the most predictable anchor available, and the one
        # the operator can reproduce without thinking.
        r_zero = np.clip(np.zeros(len(JOINTS)), r_lo, r_hi)

        out = np.empty(len(JOINTS))
        for i in range(len(JOINTS)):
            # The two spans here are DIFFERENT asymmetries and must be selected separately:
            #   span — the OPERATOR's range on the side they actually moved (raw deviation).
            #   robot travel — the ROBOT's range on the side it is being sent (signed).
            # Where the sign is -1 those sides are opposite, so choosing both from the same
            # branch pairs the operator's motion with the span from their other side and the
            # gain comes out wrong — subtly, and only on joints whose ranges are lopsided,
            # which per the docstring above is most of them.
            d_raw = h[i] - z[i]
            span = max(hi[i] - z[i] if d_raw >= 0 else z[i] - lo[i], MIN_HUMAN_SPAN_RAD)
            d = d_raw * sign[i]
            travel = (r_hi[i] - r_zero[i]) if d >= 0 else (r_zero[i] - r_lo[i])
            gain = min(travel / span, MAX_GAIN)
            out[i] = r_zero[i] + d * max(gain, 0.0)
        return chain.clamp(out)


# ── store ───────────────────────────────────────────────────────────────────
#
# On disk:  {"schema": 3, "profiles": {"<name>": {"left": {...}, "right": {...}}}}
#
# Schema 2 stored ONE side-less entry per name. `_migrate` promotes such an entry to BOTH
# sides so an existing operator keeps working exactly as before the split — the alternative,
# guessing which arm it came from, would silently give one arm someone else's ranges. It is
# flagged `migrated_sideless` so the UI can say "recalibrate for per-side accuracy" rather
# than presenting a half-true profile as current.
def _migrate(entry: dict) -> dict:
    """One name's entry -> {side: dict}. Accepts schema-2 (side-less) and schema-3 shapes."""
    if not isinstance(entry, dict):
        return {}
    if any(s in entry for s in SIDES):                  # already per-side
        return {s: entry[s] for s in SIDES if isinstance(entry.get(s), dict)}
    if "zero_rad" in entry:                             # schema 2: one capture, side unknown
        return {s: {**entry, "side": s, "migrated_sideless": True} for s in SIDES}
    return {}


def load_all(path: Path | None = None) -> dict:
    """{name: {side: raw_dict}} — migrated, so callers never see the schema-2 shape."""
    p = path or default_profile_path()
    if not p.is_file():
        return {}
    try:
        raw = json.loads(p.read_text()).get("profiles", {}) or {}
    except Exception:                                    # noqa: BLE001
        return {}
    return {name: m for name, entry in raw.items() if (m := _migrate(entry))}


def load(name: str = DEFAULT_NAME, side: str = "left",
         path: Path | None = None) -> ArmProfile | None:
    s = _side_of(side)
    d = (load_all(path).get(name) or {}).get(s)
    return ArmProfile.from_dict(name, d, side=s) if d else None


def save(profile: ArmProfile, path: Path | None = None) -> Path:
    p = path or default_profile_path()
    all_p = load_all(p)
    entry = dict(all_p.get(profile.name) or {})
    entry[_side_of(profile.side)] = profile.to_dict()   # leaves the other side untouched
    all_p[profile.name] = entry
    _atomic_write(p, {"schema": SCHEMA, "profiles": all_p})
    return p


def delete(name: str, side: str | None = None, path: Path | None = None) -> bool:
    """Delete one side, or the whole operator when ``side`` is None."""
    p = path or default_profile_path()
    all_p = load_all(p)
    if name not in all_p:
        return False
    if side is None:
        del all_p[name]
    else:
        s = _side_of(side)
        if s not in all_p[name]:
            return False
        del all_p[name][s]
        if not all_p[name]:
            del all_p[name]
    _atomic_write(p, {"schema": SCHEMA, "profiles": all_p})
    return True
