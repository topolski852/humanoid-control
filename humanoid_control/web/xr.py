"""
Meta Quest 3 controller → arm teleop, over a WebXR websocket.

The Quest is a BROWSER CLIENT: it opens the page this server hosts at `/xr`, enters immersive
passthrough, and streams one JSON frame per XR render frame to `/ws/xr`. Nothing is installed
on the headset and nothing XR-specific runs on the robot beyond this module.

WebXR only runs in a SECURE CONTEXT, and a self-signed certificate does not buy one: Chromium
keeps flagging an origin whose certificate you clicked through and withholds WebXR from it.
The route that works is `adb reverse tcp:8000 tcp:8000` and `http://localhost:8000/xr/`, which
Chromium trusts with no certificate at all. The TLS listener on :8443 serves the same page and
is kept for a future trusted certificate, but it is not the path to use.

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

BUTTONS. The Quest replaces the gamepad, so it carries the gamepad's lifecycle too. Note the
face buttons are SPLIT ACROSS CONTROLLERS on a Quest, unlike an Xbox pad — A/B are on the
right, X/Y on the left:

    trigger  hold to drive (deadman + clutch); release → IDLE and re-anchor
    Y (left, upper)    E-STOP — unconditional
    A (right, lower)   arm the deadman session
    B (right, upper)   disarm

E-STOP is honoured regardless of gate state, tracking, or whether this source holds the input
token; the Quest's Menu and System buttons are reserved by the runtime and never reach WebXR,
so they were not candidates. Note the trigger is NOT a panic stop: the human startle reflex is
to clench, which holds a hold-to-run deadman ON. That is why a discrete E-STOP button and a
second person both matter.

Because A/B live on the right controller and Y on the left, driving the LEFT arm puts E-STOP
on the hand that is already holding the trigger and lifecycle on the otherwise-idle hand. The
cost is that BOTH controllers must be tracked to have every function available.
"""
from __future__ import annotations

import asyncio
import logging
import math
import os
import time

import numpy as np

from ..arm_retarget import human_angles

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
# Body tracking is ALL-OR-NOTHING: the spec lets the UA emulate obscured joints or null the
# lot. An emulated joint is the headset guessing where your elbow is, and a guess must not
# drive a motor. On a bolted-down arm, freezing where it is beats dropping to IDLE mid-motion
# — so tracking loss HOLDS, and only becomes IDLE if it persists.
BODY_HOLD_S = 3.0


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
        self._conn_id = 0           # newest client wins; see attach()
        self._session: str | None = None
        self._last_seq: int | None = None
        self._last_rx = 0.0             # monotonic time of the last ACCEPTED frame
        self._last_pose: np.ndarray | None = None
        self._last_pose_change = 0.0
        self._tracked = False
        self._trigger = 0.0
        self._anchor: np.ndarray | None = None
        self._btn: dict[str, bool] = {}    # rising-edge state per bound button
        # Body tracking — observation only at this stage (see _note_body).
        self._body = None
        self._body_avail = False
        self._body_usable = False
        self._body_frames = 0
        self._body_usable_frames = 0
        self._seg_upper = 0.0
        self._seg_fore = 0.0
        self._body_robot: dict = {}
        self._human = None
        self._calib = None          # active CalibrationRun, if any
        self._body_latch = False    # body tracking was lost while armed; needs a re-press
        self._profile = None        # operator calibration, loaded at attach
        self._mirror_targets = None # retargeted robot joint targets, or None
        self._body_ok = False       # body tracking usable AND a profile loaded
        self._body_ok_at = 0.0      # monotonic time body tracking was last usable

        self._overlay = False       # dom-overlay granted by the headset?
        self._ctrls: dict = {}      # per-hand tracked/trigger, for diagnosis
        self._gate = False
        self._reason = ""
        self._dropped = 0
        self._frames = 0
        self._hz = 0.0
        self._hz_t0 = 0.0
        self._hz_n0 = 0

    # ── lifecycle ───────────────────────────────────────────────────────────
    def attach(self) -> int:
        """A client connected. Returns its CONNECTION ID.

        ONE CLIENT AT A TIME. Opening the page with an `am start` VIEW intent gives the Quest
        browser a NEW TAB rather than reusing the old one, and an abandoned tab keeps its
        websocket, its render loop and its frame stream alive in the background. Several tabs
        then feed this single object at once: their `seq` counters interleave (so frames get
        rejected as out-of-order), and `tracked`/`trigger` flicker as each overwrites the
        other. Newest wins — frames from a superseded connection are ignored.
        """
        self._conn_id += 1
        if self._connected:
            _log.warning("quest: a second client connected — superseding the previous one. "
                         "(An old browser tab left running? It will now be ignored.)")
        self._connected = True
        self._reset_link("connected")
        self.reload_profile()
        _log.info("quest: client attached as #%d (scale=%.2f, yaw=%.0f deg, trigger>=%.2f)",
                  self._conn_id, self.scale, math.degrees(self.yaw), self.trig_thresh)
        return self._conn_id

    def reload_profile(self) -> None:
        """Pick up the operator profile from disk. Called on attach and after calibration."""
        try:
            from ..arm_profile import load
            self._profile = load()
            if self._profile is None:
                _log.warning("quest: NO arm profile — angles carry the tracker's systematic "
                             "offsets; run the calibration before mirroring.")
            else:
                _log.info("quest: arm profile %r loaded (captured %s)",
                          self._profile.name, self._profile.captured_utc or "?")
        except Exception as exc:                             # noqa: BLE001
            self._profile = None
            _log.error("quest: could not load arm profile (%s)", exc)

    def detach(self, conn_id: int | None = None) -> None:
        """Socket closed. If this source was the deadman for a live session, that is
        controller loss — E-STOP, the same as unplugging the gamepad.

        A SUPERSEDED connection closing must not tear down the live one: an abandoned tab
        being garbage-collected would otherwise E-STOP a session the current tab is driving.
        """
        if conn_id is not None and conn_id != self._conn_id:
            _log.info("quest: superseded client %d disconnected (ignored)", conn_id)
            return
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
        self._body = None
        self._body_avail = False
        self._body_usable = False
        # A new link starts with no body-tracking history. Leaving _body_ok_at set from the
        # previous connection would make body_lost_too_long() true the instant a fresh client
        # attaches, on a signal from a session that is already gone.
        self._body_ok_at = 0.0
        self._body_latch = False
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
    def on_frame(self, msg: dict, conn_id: int | None = None) -> None:
        """One JSON frame from the headset. Never raises: a bad frame is counted and dropped,
        and the liveness ladder then treats the gap as a stall like any other."""
        # Ignore anything from a superseded tab. Its seq counter is independent of the live
        # one, so accepting it corrupts the liveness signal the deadman depends on.
        if conn_id is not None and conn_id != self._conn_id:
            return
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
        left = msg.get("left") or {}
        right = msg.get("right") or {}

        # ── lifecycle buttons ────────────────────────────────────────────────
        # The Quest replaces the gamepad, so it needs the gamepad's lifecycle buttons. Note
        # the face buttons are SPLIT ACROSS CONTROLLERS on a Quest, unlike an Xbox pad:
        # A/B are on the right controller, X/Y on the left. So E-STOP (Y) sits on the hand
        # that drives, and arm/disarm (A/B) on the hand that is otherwise idle.
        #
        # E-STOP first and unconditionally — before any gating, and deliberately NOT dependent
        # on tracking, the run gate, or whether this source holds the input token. Rising edge
        # only, so holding the button does not spam the log.
        self._edge("estop", bool(left.get("b")), self._on_estop)
        self._edge("arm", bool(right.get("a")), self._on_arm)
        self._edge("disarm", bool(right.get("b")), self._on_disarm)

        # Body tracking — OBSERVE ONLY at this stage. Recorded and surfaced so we can
        # measure what the headset actually delivers before any of it drives a joint;
        # nothing below reads it for control.
        self._note_body(msg.get("body"))

        # Record BOTH controllers, not just the driving one. "controller not tracked" is
        # ambiguous otherwise: it cannot distinguish "the hand you are using is untracked"
        # from "you are holding the other one".
        self._ctrls = {}
        for h in ("left", "right"):
            c = msg.get(h) or {}
            self._ctrls[h] = {"tracked": bool(c.get("tracked")),
                              "trigger": round(float(c.get("trigger") or 0.0), 2),
                              "present": bool(c),
                              "mode": c.get("mode"),
                              "gamepad": c.get("hasGamepad"),
                              "buttons": c.get("nButtons"),
                              "hand": c.get("isHand")}
        self._overlay = bool(msg.get("overlay"))
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
            # Releasing the trigger is what clears the body-loss latch below. Re-arming has
            # to cost the operator a deliberate press.
            self._body_latch = False
            return

        # BODY-LOSS LATCH (mirror mode only).
        #
        # The deadman worker also notices `body_lost_too_long()` and clears the run gate. That
        # alone is NOT enough, and the reason is a race worth spelling out: the gate is asserted
        # HERE, once per frame at 60 Hz, for as long as the trigger is held. The worker clears it
        # at 50 Hz. So the worker's clear survives ~16 ms before this path sets it again, and the
        # arm oscillates IDLE -> POSITION -> IDLE at up to 50 Hz — each engage re-running
        # enable_position() (a mode-change write to five ESCs) and re-seeding the teleop. A
        # bolted-down arm slamming between zero-torque and position hold, hundreds of CAN
        # mode-changes a second, triggered by precisely the condition the check exists to make
        # safe: the operator's arm leaving the headset's view.
        #
        # So the release has to LATCH at the source that owns the trigger. Held down, it stays
        # released; the operator has to let go and press again.
        #
        # Mirror mode only. Without a profile the arm is driven from the controller's position
        # and body tracking is not in the loop at all, so dropping the gate on body loss there
        # would break the working path for a signal nothing is reading.
        if self._profile is not None:
            if self.body_lost_too_long():
                if not self._body_latch:
                    _log.warning("quest: body tracking lost >%.1fs while the trigger is held "
                                 "— releasing. Release the trigger and press again to re-arm.",
                                 BODY_HOLD_S)
                self._body_latch = True
            if self._body_latch:
                self._release("body tracking lost")
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

    # ── body tracking (observation only for now) ────────────────────────────
    # Joints the retargeter will need. Tracked here so "is body tracking good enough on
    # this device" is answerable from data before any of it is wired to a motor.
    BODY_REQUIRED = ("chest", "left-shoulder", "left-arm-upper", "left-arm-lower",
                     "left-hand-wrist-twist", "left-hand-wrist")

    def _note_body(self, body) -> None:
        """Record body-tracking availability and quality. Never raises, never controls."""
        if not body:
            self._body_avail = False
            self._body = None
            # Clear the DERIVED values too. Leaving the last segment lengths and `usable`
            # standing made a dead feed read as live data — status showed plausible 26 cm /
            # 20 cm arm segments and usable=True while frame.body had been null throughout.
            # A stale number that looks real is worse than no number.
            self._body_usable = False
            self._seg_upper = 0.0
            self._seg_fore = 0.0
            self._body_robot = {}
            self._human = None
            # _body_ok and _mirror_targets are DERIVED TOO, and they are the two that drive a
            # motor. Clearing everything else but leaving these was worse than the stale
            # segment lengths this branch was originally written to fix: `_body_ok` stayed
            # True, so body_lost_too_long() short-circuited to False FOREVER and
            # mirror_command() went on handing out the last good targets with hold=False. The
            # entire tracking-loss ladder was dead in the one case it exists for — the body
            # block vanishing outright. Route through the same function the live path uses so
            # there is one place that decides what "usable" means.
            self._retarget(None)
            return
        self._body_avail = True
        self._body = body
        joints = body.get("joints") or {}
        # "usable" is stricter than "present": a joint the UA had to EMULATE is a guess,
        # and the whole point of body tracking here is to stop guessing where the elbow is.
        self._body_usable = all(
            isinstance(joints.get(n), dict) and not joints[n].get("e")
            for n in self.BODY_REQUIRED)
        self._body_frames += 1
        if self._body_usable:
            self._body_usable_frames += 1
        # Convert every joint into the ROBOT frame once, here, so nothing downstream has to
        # remember which convention it is holding.
        robot_joints = {}
        for name, j in joints.items():
            if isinstance(j, dict) and j.get("p") is not None:
                try:
                    robot_joints[name] = {"p": webxr_to_robot(j["p"]).tolist(),
                                          "e": bool(j.get("e"))}
                except Exception:                    # noqa: BLE001
                    pass
        self._body_robot = robot_joints

        arm = human_angles(robot_joints, side=self._drive_hand())
        self._human = arm
        self._retarget(arm)
        if self._calib is not None:
            was_done = self._calib.done
            self._calib.update(arm, robot_joints)
            if self._calib.done and not was_done:
                self.reload_profile()
        if arm is not None:
            self._seg_upper = arm.upper_len
            self._seg_fore = arm.fore_len

    # ── retargeting ─────────────────────────────────────────────────────────
    def _retarget(self, arm) -> None:
        """Operator's arm angles → robot joint targets, via their calibration profile.

        Sets `_mirror_targets` (or None) and `_body_ok`. Deliberately does NOT decide whether
        to command anything — that is the worker's call — so this stays a pure translation
        step that can be reasoned about on its own.
        """
        usable = bool(self._body_usable and arm is not None and self._profile is not None)
        if usable:
            self._body_ok_at = time.monotonic()
        self._body_ok = usable
        if not usable:
            self._mirror_targets = None
            if self._profile is None and self._body_usable:
                self._reason = "no calibration profile — run the arm calibration"
            return
        try:
            chain = self.service.arm_chain()
            self._mirror_targets = self._profile.to_robot(arm.as_array(), chain)
        except Exception as exc:                             # noqa: BLE001
            self._mirror_targets = None
            self._body_ok = False
            self._reason = f"retarget failed: {exc}"

    def mirror_command(self):
        """(targets, hold) for the arm worker, or (None, True) when it must freeze.

        HOLD, not IDLE. Body tracking drops all-or-nothing, and a brief occlusion while the
        operator reaches across themselves is normal rather than a fault — freezing rides it
        out. It only becomes IDLE if tracking stays gone past BODY_HOLD_S, because holding a
        powered arm indefinitely on lost tracking is its own hazard.
        """
        if self._mirror_targets is not None and self._body_ok:
            return self._mirror_targets, False
        gone = time.monotonic() - self._body_ok_at if self._body_ok_at else 1e9
        return None, gone <= BODY_HOLD_S

    def body_lost_too_long(self) -> bool:
        """True once tracking has been gone long enough that holding is no longer right."""
        if self._body_ok:
            return False
        return bool(self._body_ok_at) and (time.monotonic() - self._body_ok_at) > BODY_HOLD_S

    # ── in-headset HUD ──────────────────────────────────────────────────────
    def hud_frame(self) -> dict | None:
        """What to show the operator inside the headset. ~8 Hz, pushed by server.py.

        This is the ONLY feedback channel while the headset is on: passthrough cameras
        cannot resolve monitor text, so anything printed to the desktop is invisible
        exactly when it matters. Keep it short, high-contrast and unambiguous.
        """
        if not self._connected:
            return None
        if self._calib is not None:
            return self._calib.hud(self)

        a = self._human
        if a is None:
            return {"type": "hud", "tone": "warn",
                    "instruction": "NO BODY TRACKING",
                    "note": "enable 'WebXR Experiments' in chrome://flags, restart the browser"}
        if self._profile is None:
            # Say this loudly. Without a profile the arm still drives, but through the
            # controller-position path — so the operator would be wondering why their elbow
            # is not being followed, with no way to tell from inside the headset.
            return {"type": "hud", "tone": "warn",
                    "step": "NOT CALIBRATED",
                    "instruction": "NO PROFILE",
                    "note": "run the arm calibration to mirror your whole arm",
                    "live": (f"pitch {math.degrees(a.shoulder_pitch):+6.1f}   "
                             f"roll {math.degrees(a.shoulder_roll):+6.1f}\n"
                             f"yaw   {math.degrees(a.shoulder_yaw):+6.1f}   "
                             f"elbow {math.degrees(a.elbow):5.1f}")}
        held = self._trigger >= self.trig_thresh
        if not self._body_ok and self._gate:
            # Tracking dropped while driving: the arm is frozen where it was.
            return {"type": "hud", "tone": "err",
                    "step": "TRACKING LOST",
                    "instruction": "ARM HELD",
                    "note": "step back into view — the arm is frozen, not driving"}
        return {
            "type": "hud",
            "step": ("MIRRORING" if self._gate else "READY"),
            "instruction": "MIRRORING" if self._gate else "hold the trigger",
            "tone": "ok" if self._gate else "",
            "live": (f"pitch {math.degrees(a.shoulder_pitch):+6.1f}   "
                     f"roll {math.degrees(a.shoulder_roll):+6.1f}\n"
                     f"yaw   {math.degrees(a.shoulder_yaw):+6.1f}   "
                     f"elbow {math.degrees(a.elbow):5.1f}\n"
                     f"wrist {math.degrees(a.wrist):+6.1f}"),
            "progress": round(100.0 * min(self._trigger, 1.0), 0) if held else 0,
            "note": self._reason or "",
        }

    def start_calibration(self, seq=None):
        """Begin the guided calibration. Returns the sequencer."""
        from .xr_calib import CalibrationRun
        self._calib = CalibrationRun(seq) if seq else CalibrationRun()
        _log.info("quest: calibration started")
        return self._calib

    def cancel_calibration(self) -> None:
        self._calib = None

    # ── buttons ─────────────────────────────────────────────────────────────
    def _edge(self, name: str, pressed: bool, action) -> None:
        """Fire `action` on the rising edge of a button. Edge-triggered so a held button acts
        once, and so a repeated frame (a stuck sender) cannot re-fire it."""
        if pressed and not self._btn.get(name):
            try:
                action()
            except Exception as exc:                 # noqa: BLE001
                # A refused action (uncalibrated, wrong state) is normal operator feedback,
                # not a fault — surface it in the UI rather than killing the frame handler.
                self._reason = str(exc)
                _log.info("quest: %s refused (%s)", name, exc)
        self._btn[name] = pressed

    def _on_estop(self) -> None:
        _log.error("quest: Y pressed — E-STOP.")
        self.service.trigger_estop("quest-estop")

    def _dispatch(self, fn, label: str) -> None:
        """Run a BLOCKING service call OFF the event loop.

        on_frame runs in the asyncio event loop (server.py awaits receive_json and calls it
        directly). arm_deadman takes a lock and may load an ONNX policy from disk;
        disarm_deadman calls stop(wait=True), which JOINS the session thread with a 5-second
        timeout. Calling either inline stalls the whole loop for that long — no telemetry, no
        /ws/control heartbeats, no XR frames, and crucially not the deadman watchdog, which
        is itself an asyncio task. When the loop resumed the watchdog would see a stale
        heartbeat and fire a SPURIOUS E-STOP.

        routes.py already wraps both in `await _blocking(...)` for exactly this reason; the
        button path simply never did.
        """
        def run():
            try:
                fn()
            except Exception as exc:                 # noqa: BLE001
                # A refused action (uncalibrated, wrong state) is operator feedback, not a
                # fault. Surface it on the HUD.
                self._reason = str(exc)
                _log.info("quest: %s refused (%s)", label, exc)
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            run()                                    # no loop (tests) — inline is fine
            return
        loop.run_in_executor(None, run)

    def _on_arm(self) -> None:
        """A → arm the deadman session. Only when the Quest actually holds the input token:
        arming a session this source cannot then drive would strand the operator in ARMED
        with a trigger that does nothing."""
        if not self._owns():
            self._reason = "not the active control method"
            return
        _log.info("quest: A pressed — arming.")
        self._dispatch(self.service.arm_deadman, "arm")

    def _on_disarm(self) -> None:
        """B → disarm. Allowed regardless of the token: stopping is never gated."""
        _log.info("quest: B pressed — disarming.")
        self._dispatch(self.service.disarm_deadman, "disarm")

    def _owns(self) -> bool:
        return self.service.input_source == SOURCE

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
            # Body tracking, observation only for now. `usable` means every joint the
            # retargeter needs was present AND measured rather than emulated.
            "body": {
                "available": self._body_avail,
                "usable": self._body_usable,
                "present": (self._body or {}).get("present"),
                "emulated": (self._body or {}).get("emulated"),
                "usable_pct": (round(100.0 * self._body_usable_frames / self._body_frames, 1)
                               if self._body_frames else None),
                "upper_arm_m": round(self._seg_upper, 4) or None,
                "forearm_m": round(self._seg_fore, 4) or None,
                # Retargeted arm angles, degrees — the same ones the recorder stores, so a
                # run log can be compared joint-for-joint against what the robot did.
                #
                # The raw 13-joint positions used to be here too. They were a one-off
                # diagnostic for the decomposition and should never have survived: this
                # whole snapshot is pushed to EVERY browser at 20 Hz, so a debug field costs
                # bandwidth and JSON encoding forever. Read them from the flight recorder,
                # which is where per-tick detail belongs.
                "angles_deg": (None if self._human is None else {
                    "shoulder_pitch": round(math.degrees(self._human.shoulder_pitch), 2),
                    "shoulder_roll": round(math.degrees(self._human.shoulder_roll), 2),
                    "shoulder_yaw": round(math.degrees(self._human.shoulder_yaw), 2),
                    "elbow": round(math.degrees(self._human.elbow), 2),
                    "wrist": round(math.degrees(self._human.wrist), 2),
                }),
            },
            "overlay": self._overlay,
            "conn_id": self._conn_id,
            # Which operator profile is loaded. Without one the retargeted angles carry the
            # tracker's systematic offsets (a straight arm reads ~21 deg of elbow bend), so
            # the UI must be able to say "not calibrated" rather than quietly mis-mapping.
            "profile": (None if self._profile is None else {
                "name": self._profile.name,
                "captured_utc": self._profile.captured_utc,
                "upper_len_m": round(self._profile.upper_len_m, 3),
                "fore_len_m": round(self._profile.fore_len_m, 3),
            }),
            "controllers": self._ctrls,
            "owns_input": self.service.input_source == SOURCE,
            "reason": self._reason,
        }


def disabled_status() -> dict:
    """What telemetry reports when the Quest bridge is not enabled at all."""
    return {"enabled": False, "connected": False, "owns_input": False,
            "reason": "HUMANOID_QUEST_ENABLE not set"}
