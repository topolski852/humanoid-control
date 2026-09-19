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

from ..arm_profile import SIDES

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
    # Per side, because the four pose instructions ("relax", "T-pose", ...) already apply to
    # both arms at once: the operator performs ONE gesture and both arms are measured from it.
    # Separate ranges per side still matter — the arms are not symmetric in practice, and one
    # profile shared across both is what floored the right elbow's gain to zero.
    samples: dict = field(default_factory=lambda: {s: [] for s in SIDES})
    captured: dict = field(default_factory=lambda: {s: {} for s in SIDES})
    done: bool = False
    failed_note: str = ""
    saved_to: str = ""
    saved_sides: tuple = ()
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
    def update(self, arms, joints: dict) -> None:
        """One body-tracking sample. Never raises — a calibration bug must not kill the link.

        ``arms`` is ``{side: HumanArm | None}``; a bare HumanArm is accepted as the left arm
        so single-arm callers keep working. A side that is not tracked simply contributes no
        samples — the operator can calibrate with one arm connected and the other absent.
        """
        if self.done:
            return
        if not isinstance(arms, dict):
            arms = {"left": arms}
        live = {s: a for s, a in arms.items() if s in SIDES and a is not None}
        if not live:
            return
        now = time.monotonic()
        if now < self._settle_until:
            return
        for s, a in live.items():
            self.samples[s].append(a.as_array())

        if now - self._settle_until < HOLD_S:
            return

        pose = self.current
        if pose is None:
            return

        # Hold complete. Reject it if the operator was still moving: a capture taken
        # mid-adjustment becomes a permanent offset in every future session. Both arms are
        # doing ONE gesture, so an unsteady arm retries the pose for both rather than
        # banking a good left against a smeared right.
        spreads = {}
        for s in live:
            if not self.samples[s]:
                continue
            spreads[s] = float(np.degrees(ang_spread(np.array(self.samples[s]))).max())
        if not spreads:
            return
        worst = max(spreads, key=lambda s: spreads[s])
        if spreads[worst] > STEADY_DEG:
            self.failed_note = (f"too much movement in the {worst} arm "
                                f"({spreads[worst]:.0f} deg) — hold still, retrying")
            _log.info("quest calib: %s rejected, %s spread %.1f deg",
                      pose.key, worst, spreads[worst])
            self._restart_pose()
            return

        for s, spread in spreads.items():
            mean = ang_mean(np.array(self.samples[s]))
            self.captured[s][pose.key] = {
                "angles": [float(v) for v in mean],
                "spread_deg": round(spread, 2),
                "samples": len(self.samples[s]),
                "upper_len": float(live[s].upper_len),
                "fore_len": float(live[s].fore_len),
            }
            _log.info("quest calib: captured %s/%s (%d samples, spread %.1f deg)",
                      s, pose.key, len(self.samples[s]), spread)
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
        from ..arm_profile import ArmProfile, save
        stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        saved, failed = [], []
        for side in SIDES:
            # A side with no `relaxed` pose has no zero reference, so there is nothing to
            # save; that is the normal single-arm case, not an error.
            if not self.captured.get(side, {}).get("relaxed"):
                continue
            try:
                prof = ArmProfile.from_capture(self.captured[side], side=side,
                                               captured_utc=stamp)
                path = save(prof)
                saved.append(side)
                self.saved_to = str(path)
                _log.info("quest calib: %s profile saved to %s", side, path)
            except Exception as exc:                         # noqa: BLE001
                failed.append(f"{side}: {exc}")
                _log.error("quest calib: %s save FAILED (%s)", side, exc)
        if not saved:
            self.saved_to = ""
            self.failed_note = ("could not save: " + "; ".join(failed) if failed
                                else "no arm was tracked — nothing to save")
        elif failed:
            # Partial success is still usable, but must not read as a clean run.
            self.failed_note = "saved " + ", ".join(saved) + "; " + "; ".join(failed)
        self.saved_sides = tuple(saved)

    def _restart_pose(self) -> None:
        self.samples = {s: [] for s in SIDES}
        self._settle_until = time.monotonic() + 3.0

    # ── HUD ─────────────────────────────────────────────────────────────────
    def hud(self, src) -> dict:
        if self.done:
            ok = bool(self.saved_to)
            sides = " + ".join(self.saved_sides) if self.saved_sides else ""
            return {"type": "hud", "tone": "ok" if ok else "err",
                    "step": "CALIBRATION COMPLETE",
                    "instruction": "DONE" if ok else "NOT SAVED",
                    "progress": 100,
                    "note": (f"{sides} profile saved — you can take the headset off" if ok
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
    @property
    def captured_poses(self) -> list:
        """Pose keys captured for at least one arm, in sequence order."""
        return [p.key for p in self.seq
                if any(p.key in self.captured.get(s, {}) for s in SIDES)]

    def profile(self, side: str = "left") -> dict:
        """One side's captured profile, ready to persist. Only meaningful once `done`."""
        cap = self.captured.get(side, {})
        rel = cap.get("relaxed", {}).get("angles")
        lens = [c for c in cap.values() if c.get("upper_len")]
        return {
            "schema": 1,
            "side": side,
            "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            # Zero reference from the RELAXED pose, not the T-pose — see the module docstring.
            "zero_rad": rel,
            "upper_len_m": (round(float(np.mean([c["upper_len"] for c in lens])), 4)
                            if lens else None),
            "fore_len_m": (round(float(np.mean([c["fore_len"] for c in lens])), 4)
                           if lens else None),
            "poses": cap,
        }

    def profiles(self) -> dict:
        """{side: profile} for every side that actually captured something."""
        return {s: self.profile(s) for s in SIDES if self.captured.get(s)}

    def summary(self) -> str:
        out = []
        for side in SIDES:
            cap = self.captured.get(side, {})
            if not cap:
                continue
            out.append(f"  [{side}]")
            for p in self.seq:
                c = cap.get(p.key)
                if not c:
                    out.append(f"    {p.key:<10} (not captured)")
                    continue
                deg = [f"{math.degrees(v):+6.1f}" for v in c["angles"]]
                out.append(f"    {p.key:<10} {' '.join(deg)}   "
                           f"spread {c['spread_deg']:.1f} deg")
        return "\n".join(out) or "  (nothing captured)"
