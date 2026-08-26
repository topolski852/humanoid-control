"""
Meta Quest 3 controller → arm teleop, over a WebXR websocket.

The Quest is a BROWSER CLIENT: it opens the page this server hosts at `/xr` (over TLS, since
WebXR requires a secure context), enters immersive passthrough, and streams one JSON frame per
XR render frame to `/ws/xr`. Nothing is installed on the headset and nothing XR-specific runs
on the robot beyond this module.

Why not `xr_teleoperate`/`televuer` (the obvious reuse)? It is a Vuer/WebXR stack aimed at
Unitree robots, and three things ruled it out: it pins `numpy<2.0.0` (this runtime is on numpy
2.x for onnxruntime), it bakes wrist poses into a G1 waist frame with hardcoded body offsets,
and — decisively for a safety path — **it emits no timestamps and no sequence numbers**. Its
`motion_data_ready` is a one-way latch that is never cleared and its event handlers swallow
every exception, so a disconnected headset leaves the last pose in shared memory reading
perfectly healthy. A polling consumer cannot tell "operator holding still" from "link dead".
Owning the page instead means the liveness signal is designed in rather than inferred.

TWO INDEPENDENT SAFETY SIGNALS, mirroring the gamepad exactly:

  * Heartbeat = "this source is alive". Fed from `seq` advancing. Loss E-STOPs a live session
    via the service's presence watchdog. This is the deadman of record.
  * Run gate  = "the trigger is held". Release → the arm goes IDLE (recoverable, NOT an
    E-STOP), which is the existing behaviour and is deliberately unchanged.

The run gate is recomputed FROM SCRATCH every frame as

    gate = fresh_frame AND tracked AND (trigger >= threshold)

with all three read from the SAME frame. The trigger is never a latched boolean carried
across frames — that is the specific failure where a stale message leaves "trigger held" true
after the operator has let go and walked away.

E-STOP is the B/Y button (upper face) on EITHER controller, honoured unconditionally —
regardless of gate state, tracking, or whether this source currently holds the input token.
The Quest's Menu and System buttons are reserved by the runtime and never reach WebXR, so
they are not candidates. Note that the trigger is NOT a panic stop: the human startle reflex
is to clench, which holds a hold-to-run deadman ON. That is why a discrete E-STOP button and
a second person both matter.
"""
from __future__ import annotations

import logging
import math
import os
import time

import numpy as np

_log = logging.getLogger(__name__)

SOURCE = "quest"

# ── liveness ladder ─────────────────────────────────────────────────────────────
# Soft stall: drop the run gate (arm → IDLE), exactly as releasing the trigger does. Recovers
# by itself on the next frame. 200 ms is ~12 missed frames at 60 Hz and 10 arm ticks at 50 Hz
# — long enough to ride out ordinary WiFi jitter, short enough that the arm stops before the
# operator has finished noticing.
STALL_S = 0.2
# Hard loss: treated as controller loss and E-STOPped, the same treatment the gamepad gives an
# unplugged pad. Matches the service's _DEADMAN_TIMEOUT_S.
LOSS_S = 1.0
# `seq` advancing but the pose bit-identical: the client is alive and looping, but its pose
# source has frozen. Real 6-DOF tracking jitters at the micrometre level, so an EXACTLY equal
# pose means a repeated buffer, not a steady hand. This is the failure mode xr_teleoperate's
# own safety layer documents ("silently repeats the last pose when hands leave the FOV").
FROZEN_S = 0.5


def _f(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def webxr_to_robot(p) -> np.ndarray:
    """WebXR `local-floor` (+X right, +Y up, −Z forward) → robot URDF (+X fwd, +Y left, +Z up).

    Independently derived, then cross-checked against televuer's `T_ROBOT_OPENXR`
    ``[[0,0,-1],[-1,0,0],[0,1,0]]`` — two derivations agreeing is worth more than either
    alone. Metres in, metres out.
    """
    p = np.asarray(p, dtype=float).reshape(3)
    return np.array([-p[2], -p[0], p[1]])


class QuestSource:
    """Server-side half of the Quest link: liveness, clutch, and the pose command."""

    def __init__(self, service):
        self.service = service
        self.scale = _f("HUMANOID_QUEST_SCALE", 0.5)
        # Fixed yaw offset, NOT head-relative. televuer defaults to head-yaw-relative, which
        # suits a humanoid whose head you are wearing; for an arm bolted to a bench it would
        # rotate the mapping every time the operator turned to look at the arm.
        self.yaw = math.radians(_f("HUMANOID_QUEST_YAW_DEG", 0.0))
        self.trig_thresh = _f("HUMANOID_QUEST_TRIGGER", 0.5)
        # Which controller drives. Defaults to the hand matching the arm being driven.
        self.hand = os.environ.get("HUMANOID_QUEST_HAND", "").lower() or None

        self._connected = False
        self._session: str | None = None
        self._last_seq: int | None = None
        self._last_rx = 0.0             # monotonic time of the last ACCEPTED frame
        self._last_pose: np.ndarray | None = None
        self._last_pose_change = 0.0
        self._tracked = False
        self._trigger = 0.0
        self._anchor: np.ndarray | None = None
        self._estop_latch = False       # B/Y edge detect
        self._gate = False
        self._reason = ""
        self._dropped = 0
        self._frames = 0
        self._hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    # ── lifecycle ───────────────────────────────────────────────────────────
    def attach(self) -> None:
        self._connected = True
        self._reset_link("connected")
        _log.info("quest: client attached (scale=%.2f, yaw=%.0f deg, trigger>=%.2f)",
                  self.scale, math.degrees(self.yaw), self.trig_thresh)

    def detach(self) -> None:
        """Socket closed. If this source was the deadman for a live session, that is
        controller loss — E-STOP, the same as unplugging the gamepad."""
        self._connected = False
        self._release("disconnected")
        self.service.drop_source(SOURCE)
        if (self.service.state.name in ("ARMED", "HOLDING", "RUNNING")
                and self.service.deadman_source() == SOURCE):
            _log.error("quest: link closed during a live session — E-STOP.")
            self.service.trigger_estop("quest-disconnect")
        _log.info("quest: client detached.")

    def _reset_link(self, why: str) -> None:
        self._session = None
        self._last_seq = None
        self._last_pose = None
        self._anchor = None
        self._tracked = False
        self._trigger = 0.0
        self._reason = why

    def _release(self, why: str, *, reset_pose: bool = True) -> None:
        """Drop the run gate and the clutch anchor. Recoverable; never an E-STOP by itself.

        ``reset_pose`` forgets the pose history, restarting the frozen-detector's window at
        the next engage. That is right for every release caused by a GAP in the stream — a
        stall normally ends with the operator having held still through it, so the first
        frame back legitimately carries the previous pose and must not be read as a frozen
        sender. It is wrong for a release caused BY the freeze: clearing there would let the
        next identical frame restart the window, re-engage, freeze again 500 ms later, and
        flap the gate (and the arm) indefinitely. A freeze therefore keeps its history and
        stays released until a genuinely different pose arrives.
        """
        if self._gate or self._anchor is not None:
            _log.info("quest: run gate released (%s)", why)
        self._gate = False
        self._anchor = None
        self._reason = why
        if reset_pose:
            self._last_pose = None
            self._last_pose_change = 0.0
        self.service.set_run_gate(False, source=SOURCE)

    # ── per-frame ───────────────────────────────────────────────────────────
    def on_frame(self, msg: dict) -> None:
        """One JSON frame from the headset. Never raises: a bad frame is counted and dropped,
        and the liveness ladder then treats the gap as a stall like any other."""
        try:
            self._on_frame(msg)
        except Exception as exc:               # noqa: BLE001
            # Deliberately NOT a bare `except: pass` — the counter is surfaced in telemetry so
            # a client sending garbage is visible rather than presenting as a dead link.
            self._dropped += 1
            self._reason = f"bad frame: {exc}"
            if self._dropped in (1, 10, 100) or self._dropped % 500 == 0:
                _log.warning("quest: dropped %d malformed frame(s); last: %s",
                             self._dropped, exc)

    def _on_frame(self, msg: dict) -> None:
        now = time.monotonic()
        seq = int(msg["seq"])
        session = str(msg.get("session") or "")

        # A new XR session means the operator re-entered immersive mode: every anchor and the
        # sequence baseline are meaningless now.
        if session != self._session:
            if self._session is not None:
                _log.info("quest: new XR session %s (was %s) — anchors discarded",
                          session, self._session)
                self._release("new XR session")
            self._session = session
            self._last_seq = None

        # Reject replays / reordering outright rather than acting on an old pose.
        if self._last_seq is not None and seq <= self._last_seq:
            self._dropped += 1
            return
        self._last_seq = seq
        self._last_rx = now
        self._frames += 1
        self._tick_rate(now)

        # This source is alive. Note the heartbeat is fed by `seq` ADVANCING, not by the
        # socket being open — an open socket carrying nothing is exactly the failure a
        # timestamp-free transport cannot see.
        self.service.mark_source_alive(SOURCE)

        side = self._drive_hand()
        ctrl = msg.get(side) or {}
        other = msg.get("right" if side == "left" else "left") or {}

        # E-STOP first and unconditionally, from EITHER controller, before any gating. Rising
        # edge only, so holding the button does not spam the log.
        estop_now = bool(ctrl.get("b")) or bool(other.get("b"))
        if estop_now and not self._estop_latch:
            _log.error("quest: B/Y pressed — E-STOP.")
            self.service.trigger_estop("quest-estop")
        self._estop_latch = estop_now

        self._tracked = bool(ctrl.get("tracked"))
        self._trigger = float(ctrl.get("trigger") or 0.0)

        pos = ctrl.get("p")
        if not self._tracked or pos is None:
            self._release("controller not tracked")
            return

        p_robot = self._align(webxr_to_robot(pos))
        if not np.all(np.isfinite(p_robot)):
            self._release("non-finite pose")
            return

        # Frozen-value detection. EXACT equality on purpose: a live tracker always jitters, so
        # only a repeated buffer compares equal. A genuinely motionless-but-live controller
        # never trips this.
        if self._last_pose is None or not np.array_equal(p_robot, self._last_pose):
            self._last_pose = p_robot.copy()
            self._last_pose_change = now
        frozen = (now - self._last_pose_change) > FROZEN_S

        held = self._trigger >= self.trig_thresh
        if frozen and held:
            if self._gate:      # log the transition, not every frame of a stuck stream
                _log.warning("quest: pose frozen >%.1fs while the trigger is held — releasing.",
                             FROZEN_S)
            # Keeps the pose history: stays released until the pose genuinely changes.
            self._release("pose frozen", reset_pose=False)
            return

        if not held:
            if self._gate or self._anchor is not None:
                self._release("trigger released")
            return

        # Trigger held: clutch anchor on the rising edge, then command displacement from it.
        # The anchor is latched here and the arm-side anchor is latched by ArmTeleop.reset()
        # when the deadman worker engages, so the two meet on a displacement vector and
        # neither needs the other's coordinate system.
        if self._anchor is None:
            self._anchor = p_robot.copy()
            _log.info("quest: clutch engaged — anchored at %s", p_robot.round(3))

        delta = (p_robot - self._anchor) * self.scale
        self.service.set_arm_pose_command(delta, seq, source=SOURCE)
        self._gate = True
        self._reason = ""
        self.service.set_run_gate(True, source=SOURCE)

    # ── periodic ────────────────────────────────────────────────────────────
    def tick(self) -> None:
        """Called by the server watchdog. The receive path only runs when frames ARRIVE, so
        silence has to be noticed from the outside."""
        if not self._connected or self._last_rx == 0.0:
            return
        age = time.monotonic() - self._last_rx
        if age > LOSS_S:
            if (self.service.state.name in ("ARMED", "HOLDING", "RUNNING")
                    and self.service.deadman_source() == SOURCE):
                _log.error("quest: no frame for %.2fs during a live session — E-STOP.", age)
                self.service.trigger_estop("quest-timeout")
            self._release(f"no frame for {age:.1f}s")
        elif age > STALL_S and self._gate:
            _log.warning("quest: stalled %.0f ms — dropping the run gate.", age * 1000)
            self._release("link stalled")

    def _tick_rate(self, now: float) -> None:
        if self._hz_t0 == 0.0:
            self._hz_t0, self._hz_n0 = now, self._frames
            return
        span = now - self._hz_t0
        if span >= 1.0:
            self._hz = (self._frames - self._hz_n0) / span
            self._hz_t0, self._hz_n0 = now, self._frames

    # ── helpers ─────────────────────────────────────────────────────────────
    def _drive_hand(self) -> str:
        """Which controller drives. Follows the selected arm unless overridden, so a left-arm
        bench comes up on the left controller with nothing to configure."""
        if self.hand in ("left", "right"):
            return self.hand
        limb = (self.service.selected_limb() or "left_arm")
        return "right" if limb.startswith("right") else "left"

    def _align(self, v: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    def status(self) -> dict:
        age = (time.monotonic() - self._last_rx) if self._last_rx else None
        return {
            "enabled": True,
            "connected": self._connected,
            "session": self._session,
            "seq": self._last_seq,
            "hz": round(self._hz, 1),
            "age_ms": round(age * 1000.0, 1) if age is not None else None,
            "tracked": self._tracked,
            "trigger": round(self._trigger, 3),
            "anchored": self._anchor is not None,
            "gate": self._gate,
            "hand": self._drive_hand(),
            "scale": self.scale,
            "yaw_deg": round(math.degrees(self.yaw), 1),
            "dropped": self._dropped,
            "owns_input": self.service.input_source == SOURCE,
            "reason": self._reason,
        }


def disabled_status() -> dict:
    """What telemetry reports when the Quest bridge is not enabled at all."""
    return {"enabled": False, "connected": False, "owns_input": False,
            "reason": "HUMANOID_QUEST_ENABLE not set"}
