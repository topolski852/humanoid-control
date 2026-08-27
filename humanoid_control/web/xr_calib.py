"""
Guided arm calibration, driven from the headset HUD.

WHY CALIBRATION IS NOT OPTIONAL. The Quest infers your upper body from the headset and
controllers — no camera watches your torso — so the numbers arrive with systematic offsets,
not just noise. Measured on this operator holding a genuine T-pose with a straight arm, the
retargeter reported **21.6 degrees of elbow flexion**, because the inferred elbow sits about
5 cm off the true shoulder-wrist line. That error barely moves between samples (sd ~1-2 deg),
which is exactly what a per-pose capture can remove: hold a known pose, record what the
tracker claims, subtract it forever after.

The same capture also solves the second problem — range. The robot's joints are far tighter
than a human's (shoulder_yaw +/-45 deg against your ~90+, elbow 0-90, roll -15..+75), so
without knowing YOUR comfortable range the mapping either saturates immediately or wastes
most of the robot's travel.

POSE ORDER MATTERS. `relaxed` comes first and is the zero reference, not the T-pose. At 90
degrees of abduction a T-pose is near-degenerate for any Euler decomposition — pitch and yaw
stop being separable — so it is a poor place to measure angles from. It is still the right
place to measure SEGMENT LENGTHS and the torso frame, which is why it is kept, just not as
the zero.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field

import numpy as np

_log = logging.getLogger(__name__)


def wrap(a):
    """Wrap angles to (-pi, pi]. Without this, a wrist sitting near +/-180 deg reports a
    360-degree "spread" the moment it crosses the boundary — which read as the operator
    thrashing about and rejected a perfectly steady hold."""
    return np.arctan2(np.sin(a), np.cos(a))


def ang_spread(a: np.ndarray) -> np.ndarray:
    """Per-column angular spread, immune to the wrap. Measured about the circular mean, so a
    joint hovering either side of the boundary reports the few degrees it actually moved."""
    c = np.arctan2(np.sin(a).mean(axis=0), np.cos(a).mean(axis=0))
    return np.abs(wrap(a - c)).max(axis=0) * 2.0


def ang_mean(a: np.ndarray) -> np.ndarray:
    """Circular mean — a plain mean of -179 and +179 gives 0, which is exactly wrong."""
    return np.arctan2(np.sin(a).mean(axis=0), np.cos(a).mean(axis=0))

# Seconds the operator must hold each pose. Long enough to average out the ~1-2 deg of
# per-sample noise, short enough that holding an arm out does not become an endurance test.
HOLD_S = 4.0
# Steadiness gate: if the arm moves more than this during the hold, the capture is rejected
# and repeated. Without it a capture taken mid-adjustment becomes a permanent offset.
STEADY_DEG = 12.0


@dataclass
class Pose:
    key: str
    instruction: str
    note: str


# Deliberately few, and each one earns its place.
POSES: tuple[Pose, ...] = (
    Pose("relaxed", "ARM RELAXED AT YOUR SIDE",
         "let it hang naturally, elbow straight — this is the zero"),
    Pose("tpose", "ARM STRAIGHT OUT TO THE SIDE",
         "shoulder height, elbow straight — measures your arm's length"),
    Pose("forward", "ARM STRAIGHT OUT IN FRONT",
         "shoulder height, elbow straight"),
    Pose("elbow90", "UPPER ARM DOWN, FOREARM FORWARD",
         "elbow bent to a right angle"),
    # NO reach-up pose. The decomposition deliberately puts its singularity at "arm
    # straight up" because the robot cannot reach there — so asking the OPERATOR to go there
    # samples exactly the degenerate region. Measured: it returned roll 152 deg with yaw
    # -136 deg for a simple raised arm. The four poses above already span more range than
    # the robot has (pitch -90..45, roll -15..75), so nothing is lost.
)


@dataclass
class CalibrationRun:
    """State machine for the guided capture. Fed one sample per XR frame."""

    seq: tuple[Pose, ...] = field(default_factory=lambda: POSES)
    idx: int = 0
    t_start: float = field(default_factory=time.monotonic)
    samples: list = field(default_factory=list)
    captured: dict = field(default_factory=dict)
    done: bool = False
    failed_note: str = ""
    saved_to: str = ""
    _settle_until: float = 0.0

    def __post_init__(self) -> None:
        # Passing seq=None positionally defeats the default_factory, and every method here
        # calls len(self.seq). Coerce rather than trust the caller.
        if not self.seq:
            self.seq = POSES
        # A moment to read the first instruction before sampling starts.
        self._settle_until = time.monotonic() + 3.0

    @property
    def current(self) -> Pose | None:
        return self.seq[self.idx] if self.idx < len(self.seq) else None

    # ── per-frame ───────────────────────────────────────────────────────────
    def update(self, arm, joints: dict) -> None:
        """One body-tracking sample. Never raises — a calibration bug must not kill the link."""
        if self.done or arm is None:
            return
        now = time.monotonic()
        if now < self._settle_until:
            return
        self.samples.append(arm.as_array())

        if now - self._settle_until < HOLD_S:
            return

        # Hold complete. Reject it if the operator was still moving: a capture taken
        # mid-adjustment becomes a permanent offset in every future session.
        a = np.array(self.samples)
        spread = float(np.degrees(ang_spread(a)).max())
        pose = self.current
        if pose is None:
            return
        if spread > STEADY_DEG:
            self.failed_note = f"too much movement ({spread:.0f} deg) — hold still, retrying"
            _log.info("quest calib: %s rejected, spread %.1f deg", pose.key, spread)
            self._restart_pose()
            return

        mean = ang_mean(a)
        self.captured[pose.key] = {
            "angles": [float(v) for v in mean],
            "spread_deg": round(spread, 2),
            "samples": len(self.samples),
            "upper_len": float(arm.upper_len),
            "fore_len": float(arm.fore_len),
        }
        _log.info("quest calib: captured %s (%d samples, spread %.1f deg)",
                  pose.key, len(self.samples), spread)
        self.failed_note = ""
        self.idx += 1
        if self.idx >= len(self.seq):
            self.done = True
            self._persist()
        else:
            self._restart_pose()

    def _persist(self) -> None:
        """Save the profile the moment the last pose lands.

        Saved here rather than left for the caller to fetch: the operator is wearing a
        headset and has no way to press a Save button, and a calibration that is only in
        memory is one crash away from being redone.
        """
        try:
            from ..arm_profile import ArmProfile, save
            prof = ArmProfile.from_capture(self.captured, captured_utc=self.profile()["captured_utc"])
            path = save(prof)
            self.saved_to = str(path)
            _log.info("quest calib: profile saved to %s", path)
        except Exception as exc:                             # noqa: BLE001
            self.saved_to = ""
            self.failed_note = f"could not save: {exc}"
            _log.error("quest calib: save FAILED (%s)", exc)

    def _restart_pose(self) -> None:
        self.samples = []
        self._settle_until = time.monotonic() + 3.0

    # ── HUD ─────────────────────────────────────────────────────────────────
    def hud(self, src) -> dict:
        if self.done:
            ok = bool(self.saved_to)
            return {"type": "hud", "tone": "ok" if ok else "err",
                    "step": "CALIBRATION COMPLETE",
                    "instruction": "DONE" if ok else "NOT SAVED",
                    "progress": 100,
                    "note": ("profile saved — you can take the headset off" if ok
                             else self.failed_note or "the profile could not be written")}
        pose = self.current
        now = time.monotonic()
        n = len(self.seq)
        if now < self._settle_until:
            remain = self._settle_until - now
            return {"type": "hud", "tone": "warn",
                    "step": f"POSE {self.idx + 1} OF {n}",
                    "instruction": pose.instruction,
                    "count": f"{remain:.0f}",
                    "progress": 0,
                    "note": self.failed_note or f"get into position — {pose.note}"}
        held = now - self._settle_until
        remain = max(0.0, HOLD_S - held)
        return {"type": "hud", "tone": "ok",
                "step": f"POSE {self.idx + 1} OF {n}  ·  HOLD STILL",
                "instruction": pose.instruction,
                "count": f"{remain:.1f}",
                "progress": round(100.0 * held / HOLD_S, 0),
                "note": pose.note}

    # ── result ──────────────────────────────────────────────────────────────
    def profile(self) -> dict:
        """The captured profile, ready to persist. Only meaningful once `done`."""
        rel = self.captured.get("relaxed", {}).get("angles")
        lens = [c for c in self.captured.values() if c.get("upper_len")]
        return {
            "schema": 1,
            "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            # Zero reference from the RELAXED pose, not the T-pose — see the module docstring.
            "zero_rad": rel,
            "upper_len_m": (round(float(np.mean([c["upper_len"] for c in lens])), 4)
                            if lens else None),
            "fore_len_m": (round(float(np.mean([c["fore_len"] for c in lens])), 4)
                           if lens else None),
            "poses": self.captured,
        }

    def summary(self) -> str:
        out = []
        for p in self.seq:
            c = self.captured.get(p.key)
            if not c:
                out.append(f"  {p.key:<10} (not captured)")
                continue
            deg = [f"{math.degrees(v):+6.1f}" for v in c["angles"]]
            out.append(f"  {p.key:<10} {' '.join(deg)}   spread {c['spread_deg']:.1f} deg")
        return "\n".join(out)
