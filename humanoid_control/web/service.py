"""
ControlService — the single choke point between the web layer and the robot.

Owns the shared ``DaemonClient`` + ``EstopController`` and runs the same lifecycle the CLI
scripts drive (``scripts/hold_pose.py`` / ``scripts/run_policy.py``): connect (wake+config) →
arm ("I am present") → ramp to default_pose → hold / run policy → shutdown. Everything that
moves the robot is gated behind ``armed`` (the web equivalent of ``--i-am-present``) and a
live deadman heartbeat; E-STOP is always reachable and never blocked.

Threading model:
  - The uvicorn event loop handles routes, the telemetry WS, the deadman WS, and the deadman
    watchdog. None of those block on robot I/O.
  - A **motion session runs in a dedicated worker thread** (``_session_worker``) using the
    synchronous ``PolicyRunner`` primitives (``prepare``/``step``/``shutdown``) — the ramp and
    the per-tick UDP sends block, so keeping them off the event loop keeps telemetry + deadman
    responsive.
  - Cross-thread stop is safe: ``EstopController.fired`` is a ``threading.Event`` and the
    watchdog / E-STOP route call the thread-safe ``estop.trigger()`` / set ``_stop_evt``.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from enum import Enum

import numpy as np

from ..calibration import compute_offset
from ..config import LegPolicyContract, REPO_ROOT
from ..interface import JointGroupInterface, LegInterface
from ..layout import RobotLayout
from ..policy import ZeroPolicy, load_policy
from ..runner import PolicyRunner
from ..safety import EstopController, ramp_to_pose
from ..base_state import TelemetryBaseState
from ..daemon import DaemonClient

_log = logging.getLogger(__name__)

# Deadman: motion requires a control heartbeat at least this fresh.
_DEADMAN_TIMEOUT_S = 1.0


def _arm_hz() -> float:
    """Arm teleop loop rate. Deliberately independent of the leg policy's 25 Hz."""
    try:
        hz = float(os.environ.get("HUMANOID_ARM_HZ", 50.0))
    except (TypeError, ValueError):
        return 50.0
    return hz if 5.0 <= hz <= 200.0 else 50.0


_ARM_HZ = _arm_hz()

# A commanded position older than this is no longer treated as a live target in telemetry.
# Generous relative to the 50 Hz policy loop: a finished ramp legitimately stops resending
# while the robot still holds that pose, and that hold IS the current command.
_TARGET_STALE_S = 5.0


class SessionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"   # sockets up, joints not configured/awake
    CONNECTED = "CONNECTED"         # joints configured + online, idle
    ARMED = "ARMED"                 # deadman session live, legs DAMPING, waiting for trigger
    HOLDING = "HOLDING"             # ramping/holding default_pose (ZeroPolicy)
    RUNNING = "RUNNING"             # running a learned policy
    ESTOPPED = "ESTOPPED"           # E-STOP latched; reconnect to clear
    ERROR = "ERROR"                 # a session failed (fault/offline)


# Engaged/moving states (legs in POSITION, sending targets).
_MOTION_STATES = {SessionState.HOLDING, SessionState.RUNNING}
# A deadman session is live (worker thread running) — includes the damped-but-ready ARMED
# rest state. Used to gate connect/calibrate and to scope the controller-loss watchdog.
_ACTIVE_STATES = {SessionState.ARMED, SessionState.HOLDING, SessionState.RUNNING}


class ControlError(Exception):
    """Raised for a rejected command. ``status`` is the HTTP code the route returns."""

    def __init__(self, message: str, status: int = 409):
        super().__init__(message)
        self.status = status


class ControlService:
    def __init__(self, client: DaemonClient, contract: LegPolicyContract, *, config_present: bool,
                 layout: RobotLayout | None = None, robot_config=None):
        self.client = client
        self.contract = contract
        self.config_present = config_present
        self.robot_config = robot_config

        # Two joint views, deliberately distinct:
        #   self.legs  — the 12 contract joints. The policy path is contract-bound and must not
        #                be widened by what happens to be plugged in.
        #   self.group — every joint the LAYOUT says is attached. Connect, health, fault-clearing,
        #                calibration and telemetry all work on this, so a bench arm with no legs
        #                powered is a first-class configuration rather than a broken robot.
        self.legs = LegInterface(client, contract)
        self._layout = layout or RobotLayout()

        self._state = SessionState.DISCONNECTED
        self._armed = False
        self._last_error: str | None = None

        self._lock = threading.Lock()             # guards state transitions / session start
        self._session_thread: threading.Thread | None = None
        self._stop_evt = threading.Event()        # cooperative graceful-stop signal

        # E-STOP controller (no SIGINT/keyboard — the server has no TTY and uvicorn owns SIGINT).
        # Rebuilt on every connect so a prior latched E-STOP is cleared.
        self.estop = self._new_estop()

        # Deadman: how many /ws/control clients are attached, and PER-SOURCE liveness.
        #
        # The heartbeat is per input source ("web" = the browser control page, "xbox" = the
        # gamepad thread, "quest" = the XR bridge) rather than one global timestamp. A single
        # shared timestamp meant any live source vouched for every other one: with the browser
        # page open, a gamepad that stopped beating still read as a healthy deadman. That is
        # harmless today only because gamepad loss happens to be caught by a SEPARATE path
        # (GamepadDeadman's gamepad-absent check) — a second mechanism, not this one working.
        # A network source has no such backstop, so it must be checkable on its own.
        self._control_clients = 0
        self._sources: dict[str, float] = {}     # source -> last monotonic heartbeat
        # Which source is the deadman of record for the LIVE session. Set when a session
        # starts, cleared when it ends; falls back to the active input source when idle.
        self._session_deadman: str | None = None

        # Gamepad deadman session: the run-gate (set while a trigger is held), the live walk
        # command (vx, vy, wz) written by the gamepad sticks, and the selected session to run
        # when the trigger engages. The run-gate is distinct from the heartbeat: heartbeat =
        # "controller alive" (loss → E-STOP); run-gate = "trigger held" (release → DAMP).
        self._run_gate = threading.Event()
        self._command_lock = threading.Lock()
        self._command = np.zeros(3, dtype=np.float32)
        self._arm_command = np.zeros(4, dtype=np.float32)   # raw sticks: lx, ly, ry, rx
        # 6-DOF-tracker command: (displacement-since-clutch metres in robot frame, seq).
        self._arm_pose_command: tuple[np.ndarray, int] | None = None
        self._selected = {"kind": "hold", "checkpoint": None, "limb": None}
        # Which set of things the sticks drive, and how fast. Seeded from the layout so a
        # bench arm comes up in arm mode without anyone pressing Select.
        self._control_mode = "arm" if self._layout.arms else "leg"
        self._speed_mode = "normal"

        # Gamepad presence for the UI (updated by GamepadDeadman). "enabled" reflects whether the
        # gamepad deadman thread is running at all (HUMANOID_GAMEPAD_ENABLE).
        self._gamepad = {
            "enabled": bool(os.environ.get("HUMANOID_GAMEPAD_ENABLE")),
            "connected": False,
            "name": None,
        }
        self._gamepad_input: dict = {}

        # Which input source may drive the robot. Exactly one holds the token; writes from any
        # other are dropped and COUNTED (a silently ignored controller is a support call, an
        # ignored-and-reported one is a glance at the UI). Seeded from what is actually enabled
        # so behaviour is unchanged on a machine that only has the gamepad.
        self._input_source = "xbox" if self._gamepad["enabled"] else "web"
        self._ignored_writes: dict[str, int] = {}
        # Quest bridge, attached by server.py when HUMANOID_QUEST_ENABLE is set. None means the
        # runtime has no Quest support compiled in at all — which is the normal case and must
        # stay a first-class configuration, not a degraded one.
        self.quest = None
        # Set the moment any configured joint is seen OFFLINE. A joint dropping is the only
        # thing that can invalidate a calibration mid-session (it means the ESC lost power or
        # reset, and single-turn encoders cannot recover their multi-turn zero). Cleared when
        # calibration is (re)established.
        self._joints_dropped_since_cal = True
        self._last_autowake = 0.0   # rate-limits ESC-reset auto-recovery

        # Joint set, per-joint limits and the calibration bookkeeping all follow the layout.
        # Calibration is reset to uncalibrated on every connect (a connect follows every
        # power-up, and the encoder zero is lost on power-down).
        self._apply_layout(self._layout)

    # ── layout ───────────────────────────────────────────────────────────────
    def _apply_layout(self, layout: RobotLayout) -> None:
        """(Re)build the joint set, limits and calibration state from a layout.

        Caller holds ``self._lock`` (or is ``__init__``). Calibration is intentionally dropped
        for joints that leave the set and starts False for joints that join — an encoder zero
        is only meaningful for a joint we have actually been watching.
        """
        self._layout = layout
        self._joints = list(layout.joint_order)
        self.group = JointGroupInterface(self.client, self._joints)
        self._limits = self._build_limits(self._joints)
        prev_cal = getattr(self, "_calibrated", {})
        self._calibrated = {n: bool(prev_cal.get(n, False)) for n in self._joints}
        self._cal_captures = {n: {"lower": None, "upper": None} for n in self._joints}

    def _build_limits(self, joints: list[str]) -> dict[str, tuple[float, float]]:
        """Per-joint (lower, upper) position limits in device-frame radians.

        Leg joints take their limits from the POLICY CONTRACT, not the robot config: the
        contract is what the policy was trained against and what every clamp in the runtime
        already uses, so a drifting hardware config must not quietly widen them. Joints with no
        contract entry (the arms) fall back to the live robot config.
        """
        out: dict[str, tuple[float, float]] = {}
        contract_joints = set(self.contract.joint_order)
        for name in joints:
            if name in contract_joints:
                i = self.contract.index_of(name)
                out[name] = (float(self.contract.pos_limit_lower[i]),
                             float(self.contract.pos_limit_upper[i]))
                continue
            jc = (self.robot_config.joints.get(name) if self.robot_config else None)
            if jc is not None:
                out[name] = (float(jc.position_limits.lower_bound),
                             float(jc.position_limits.upper_bound))
            else:
                # No contract row and no hardware row: don't invent a range. Report it as
                # unbounded so nothing is silently clamped to a made-up number.
                _log.warning("no position limits known for %s", name)
                out[name] = (float("-inf"), float("inf"))
        return out

    @property
    def layout(self) -> RobotLayout:
        return self._layout

    @property
    def joints(self) -> list[str]:
        """The configured joints, in layout order. Telemetry, calibration and the contract
        endpoint are all index-aligned to this."""
        return list(self._joints)

    @property
    def joint_limits(self) -> dict[str, tuple[float, float]]:
        return dict(self._limits)

    def set_layout(self, layout: RobotLayout) -> None:
        """Swap the attached-hardware layout. Refused while anything is live — the joint set
        underpins the health checks and the E-STOP scope, so it must not move under a session."""
        with self._lock:
            if self._state in _ACTIVE_STATES:
                raise ControlError("A session is active — disarm before changing the layout.", 409)
            if not layout.enabled:
                raise ControlError("Enable at least one limb.", 400)
            missing = layout.missing_joints(self.robot_config)
            if missing:
                detail = "; ".join(f"{limb}: {', '.join(js)}" for limb, js in missing.items())
                raise ControlError(
                    f"The robot config has no entry for these joints — {detail}", 400)
            # Only a change to the JOINT SET invalidates a connection — those are the joints
            # being watched and health-checked. Re-saving the same limbs (or flipping the IMU
            # flag) must not drop a live connection out from under the operator.
            joints_changed = list(layout.joint_order) != self._joints
            self._apply_layout(layout)
            if joints_changed and self._state == SessionState.CONNECTED:
                self._state = SessionState.DISCONNECTED
                self._armed = False
        _log.info("layout set: %s (%d joints)", layout.describe(), len(self._joints))

    # ── E-STOP controller lifecycle ──────────────────────────────────────────
    def _new_estop(self) -> EstopController:
        return EstopController(self.client, install_sigint=False, keyboard=False)

    # ── read-only views ──────────────────────────────────────────────────────
    @property
    def state(self) -> SessionState:
        return self._state

    @property
    def armed(self) -> bool:
        return self._armed

    def is_motion_active(self) -> bool:
        return self._state in _MOTION_STATES

    def _last_target(self, name: str) -> tuple[float | None, float | None]:
        """Last commanded position for ``name``, or (None, None) if there isn't a live one.

        A target older than ``_TARGET_STALE_S``, or one recorded before the robot went
        idle, is reported as absent so the UI drops the overlay instead of drawing a
        command that is no longer in force.
        """
        if not self.is_motion_active():
            return None, None
        entry = self.client.get_last_target(name)
        if entry is None:
            return None, None
        value, age = entry
        if age > _TARGET_STALE_S:
            return None, None
        return value, age

    def telemetry_snapshot(self) -> dict:
        """Non-blocking snapshot from the telemetry cache (no UDP round-trip)."""
        joints = []
        for i, name in enumerate(self._joints):
            lo, hi = self._limits[name]
            limit = {"min": lo, "max": hi}
            limb = self._layout.limb_of(name)
            target, target_age = self._last_target(name)
            st = self.client.get_cached_joint_state(name)
            if st is None:
                joints.append({"index": i, "name": name, "limb": limb, "online": False,
                               "calibrated": self._calibrated.get(name, False), "limit": limit,
                               "target": target, "target_age_s": target_age})
                continue
            state = st.get("state") or st.get("joint_state")
            joints.append({
                "index": i,
                "name": name,
                "limb": limb,
                "online": state not in (None, "OFFLINE"),
                "state": state,
                "mode": st.get("mode"),
                "position": st.get("position"),
                "velocity": st.get("velocity"),
                "torque": st.get("torque"),
                "error": int(st.get("error", 0) or 0),
                "calibrated": self._calibrated.get(name, False),
                "cal_captured": dict(self._cal_captures.get(name, {})),
                "limit": limit,
                # Last position we COMMANDED (display frame, same as `position`), for the
                # visualizer's target overlay. None when nothing is actively commanding.
                "target": target,
                "target_age_s": target_age,
            })
        try:
            buses = self.client.get_interface_stats()
        except Exception:
            buses = []
        return {
            "daemon_alive": self.client.is_running(),
            "config_present": self.config_present,
            "layout": {
                "enabled": list(self._layout.enabled),
                "imu_expected": self._layout.imu_expected,
                "describe": self._layout.describe(),
                "has_both_legs": self._layout.has_both_legs,
            },
            "state": self._state.value,
            "armed": self._armed,
            "selected": self._selected.get("kind"),   # gamepad deadman kind: hold|policy|manual
            "estop": self.estop.fired,
            "deadman_ok": self.deadman_ok(),
            "deadman_source": self.deadman_source(),
            "input_source": self._input_source,
            "input_sources": self.available_input_sources(),
            "ignored_writes": dict(self._ignored_writes),
            "control_clients": self._control_clients,
            "last_error": self._last_error,
            "all_calibrated": self.all_calibrated(),
            "quest": self._xr_status(),
            "gamepad": {**self._gamepad, "run_gate": self._run_gate.is_set(),
                        "input": self._gamepad_input},
            "control": {
                "mode": self._control_mode,
                "modes": self.available_control_modes(),
                "speed": self._speed_mode,
                "limb": self._selected.get("limb"),
                "arms": list(self._layout.arms),
                "capabilities": list(self._layout.capabilities),
                "sessions": self.available_sessions(),
            },
            "joints": joints,
            "buses": buses,
            # IMU base block the policy actually consumes (quaternion / angular_velocity /
            # projected_gravity), or None when the daemon reports no fresh IMU (base: null).
            "base": self.client.latest_base(),
        }

    # ── deadman ──────────────────────────────────────────────────────────────
    def control_client_connected(self) -> None:
        """A browser /ws/control client attached. The browser is the 'web' deadman source."""
        with self._lock:
            self._control_clients += 1
        self.mark_source_alive("web")

    def control_client_disconnected(self) -> None:
        with self._lock:
            self._control_clients = max(0, self._control_clients - 1)
        if self._control_clients == 0:
            self.drop_source("web")
            # Losing the deadman while a session it is responsible for is live — even damped-
            # and-armed — is an immediate E-STOP: we can no longer trust a release vs a
            # dropout. Scoped to sessions the BROWSER is the deadman for; closing a spectator
            # tab must not kill a gamepad- or Quest-driven run.
            if self._state in _ACTIVE_STATES and self.deadman_source() == "web":
                self.trigger_estop("deadman-disconnect")

    def set_gamepad_connected(self, name: str) -> None:
        with self._lock:
            self._gamepad = {"enabled": True, "connected": True, "name": name}

    def set_gamepad_input(self, state: dict) -> None:
        """Raw controller state for the UI's live input view. Diagnostic only — nothing in the
        control path reads this."""
        self._gamepad_input = state

    def set_gamepad_disconnected(self) -> None:
        with self._lock:
            self._gamepad = {**self._gamepad, "connected": False, "name": None}

    def mark_heartbeat(self) -> None:
        """Browser heartbeat (kept as the name server.py already calls)."""
        self.mark_source_alive("web")

    def _human_angles(self):
        """The operator's own arm angles this tick, degrees, or None.

        Recorded alongside the robot's joints so a run log answers "did the robot match my
        arm" directly, instead of us inferring it from hand positions.
        """
        try:
            q = self.quest
            a = getattr(q, "_human", None) if q is not None else None
            if a is None:
                return None
            return [round(float(np.degrees(v)), 2) for v in a.as_array()]
        except Exception:                                # noqa: BLE001
            return None

    def _xr_status(self) -> dict:
        """Quest link status for telemetry and the flight recorder. Never raises — a status
        read must not be able to take down the session that is reading it."""
        try:
            if self.quest is None:
                from .xr import disabled_status
                return disabled_status()
            return self.quest.status()
        except Exception as exc:                     # noqa: BLE001
            return {"enabled": True, "connected": False, "reason": f"status error: {exc}"}

    # ── per-source liveness ──────────────────────────────────────────────────
    def mark_source_alive(self, source: str) -> None:
        """One input source says it is alive. Called every loop by whatever is driving."""
        self._sources[source] = time.monotonic()

    def drop_source(self, source: str) -> None:
        """Source is gone (socket closed, device unplugged). Immediately not-alive."""
        self._sources.pop(source, None)

    def source_alive(self, source: str) -> bool:
        t = self._sources.get(source)
        return t is not None and (time.monotonic() - t) < _DEADMAN_TIMEOUT_S

    def deadman_source(self) -> str:
        """Who the deadman of record is right now: the live session's source, else the token
        holder. A session must keep being judged by the source that armed it, even if the
        token is somehow changed underneath it."""
        return self._session_deadman or self._input_source

    def deadman_ok(self) -> bool:
        """Is the deadman of record for the current (or next) session alive?"""
        return self.source_alive(self.deadman_source())

    def watch_joint_dropouts(self) -> None:
        """Watchdog hook: notice any configured joint reading OFFLINE. Cheap — reads only the
        telemetry cache, no UDP round-trip."""
        for name in self._joints:
            st = self.client.get_cached_joint_state(name)
            state = (st or {}).get("state") or (st or {}).get("joint_state")
            if st is None or state in (None, "OFFLINE"):
                self.note_joint_dropout()
                return

    def check_offline_recovery(self) -> None:
        """Auto-recover an ESC brownout/reset: firmware v3.2.0 boots DISABLED-silent, so a reset
        drops ALL joints OFFLINE and they need re-waking. When we're CONNECTED-idle (NOT armed/
        moving) and every joint has gone OFFLINE while the daemon is alive, re-wake them (NMT IDLE)
        and mark uncalibrated (the reset reverted the offsets to flash). Never touches an active
        motion session — a reset there is a fault the deadman must E-STOP, not silently paper over."""
        if self._state != SessionState.CONNECTED or not self.client.is_running():
            return
        all_offline = all(
            (st := self.client.get_cached_joint_state(n)) is None
            or (st.get("state") or st.get("joint_state")) == "OFFLINE"
            for n in self._joints
        )
        if not all_offline:
            return
        now = time.monotonic()
        if now - self._last_autowake < 3.0:
            return
        self._last_autowake = now
        _log.warning("all joints OFFLINE while CONNECTED (ESC reset?) — auto-waking (NMT IDLE).")
        try:
            self.client.wake_all()
        except Exception as exc:
            _log.warning("auto-wake failed: %s", exc)
        with self._lock:
            self._calibrated = {n: False for n in self._joints}   # reset ⇒ offsets stale

    def check_deadman_watchdog(self) -> None:
        """Called periodically by the event-loop watchdog: trip E-STOP if a motion
        session has lost its heartbeat (WiFi stall where the socket hasn't closed yet)."""
        if self._state in _ACTIVE_STATES and not self.deadman_ok():
            self.trigger_estop("deadman-timeout")

    # ── connection lifecycle (BLOCKING — call via executor) ──────────────────
    def connect(self) -> None:
        """Connect = wake motors DISABLED→IDLE and READ their live config — it NEVER writes.
        The ESCs are assumed already configured (gains/limits/offsets live on the devices),
        so connect only brings them online and verifies them; it cannot clobber tuned or
        policy gains. Allowed from DISCONNECTED, ESTOPPED, or ERROR (clears a latched E-STOP);
        refused only during active motion."""
        with self._lock:
            if self._state in _ACTIVE_STATES:
                raise ControlError("A session is active — stop/disarm it first.", 409)
            if not self.config_present:
                raise ControlError(
                    "No robot config loaded (HUMANOID_CONFIG missing) — cannot connect.", 503)
        # Wake only (DISABLED→IDLE via NMT); NO SDO config writes, so live ESC gains survive.
        self.client.wake_all()
        time.sleep(0.5)
        # Clear any latched firmware faults (e.g. a prior watchdog / deadman E-STOP) so a fault
        # doesn't block reconnect — Connect is the operator's recovery path. CLEAR_ERROR only
        # zeroes the error reg + IDLEs; it never touches tuned gains (consistent with read-only connect).
        for n in self._joints:
            st = self.client.get_cached_joint_state(n)
            if st and int(st.get("error", 0) or 0):
                try:
                    self.client.clear_error(n)
                except Exception as exc:
                    _log.warning("clear_error %s on connect failed: %s", n, exc)
        time.sleep(0.3)
        # Health-check the CONFIGURED joints, not all 12 legs: with an arm on the bench and the
        # legs unpowered, offline leg joints are the expected state, not a fault.
        self.group.check_health()   # raises if any configured joint is offline/faulted
        # Read-only sanity net: since connect no longer configures the ESCs, confirm each is
        # actually configured (a blank/unconfigured motor reads kp or torque_limit == 0).
        unconfigured = self._verify_configured()
        with self._lock:
            self.estop = self._new_estop()   # clear any prior latched E-STOP
            self._armed = False
            self._last_error = (
                "unconfigured joints (kp/torque=0): "
                + ", ".join(n.replace("_joint", "") for n in unconfigured)
            ) if unconfigured else None
            self._state = SessionState.CONNECTED
            # Calibration does NOT survive a power cycle, and flashing the offset does not
            # change that. The AS5600 is SINGLE-TURN absolute: behind 15:1 gearing the encoder
            # wraps every 1/15 of an output revolution, so on power-up the true joint angle is
            # ambiguous by ~24 deg multiples no matter what offset is stored. A stored offset
            # that still matches proves nothing about where the joint actually is.
            #
            # So: assume stale unless we have watched the joints stay online continuously since
            # they were last marked calibrated (see _joints_dropped_since_cal), which is the
            # only evidence that rules out a power cycle.
            self._calibrated = ({n: True for n in self._joints}
                                if self._calibration_still_valid() else
                                {n: False for n in self._joints})
            self._cal_captures = {n: {"lower": None, "upper": None} for n in self._joints}
        if unconfigured:
            _log.warning("connected (read-only): joints look unconfigured: %s", unconfigured)
        _log.info("connected (read-only: woke + read config, no writes); "
                  "calibration reset (uncalibrated).")

    def _verify_configured(self) -> list[str]:
        """Read each joint's live config (no writes) and return those that look unconfigured
        (kp==0 or torque_limit==0, or unreadable). Best-effort, never raises — used only as a
        read-only warning on connect.

        READ_CONFIG occasionally drops ONE SDO param per read, so we MERGE across retries: keep
        the first non-None position_kp AND the first non-None torque_limit, and only flag a joint
        once we actually have both (or exhausted retries). Retrying until just position_kp is
        present is a false-positive trap — a read with valid kp but a dropped torque_limit would
        read as torque==0 and wrongly flag a correctly-configured joint."""
        suspect: list[str] = []
        for name in self._joints:
            kp = tl = None
            for _ in range(6):
                try:
                    c = self.client.read_device_config(name)
                except Exception:
                    continue
                if kp is None and c.get("position_kp") is not None:
                    kp = c["position_kp"]
                if tl is None and c.get("torque_limit") is not None:
                    tl = c["torque_limit"]
                if kp is not None and tl is not None:
                    break
            if kp is None or tl is None:          # genuinely couldn't read after retries
                suspect.append(name)
            elif float(kp) == 0.0 or float(tl) == 0.0:
                suspect.append(name)
        return suspect

    def disconnect(self) -> None:
        """Disconnect = motors → DISABLED (PWM off, silent)."""
        self.stop(wait=True)
        try:
            self.group.disable()
        except Exception as exc:
            _log.warning("disable on disconnect failed: %s", exc)
        self.client.clear_last_targets()   # nothing is commanding the robot any more
        with self._lock:
            self._armed = False
            self._state = SessionState.DISCONNECTED

    # ── arming ("I am present / robot is supported") ─────────────────────────
    def _require_calibrated(self) -> None:
        """Raise unless every joint is calibrated. Enforced only for motion that commands a
        CALIBRATED-frame target — the learned policy and the default_pose hold. Manual
        capture-and-hold does NOT require it: it holds the live measured pose, which is valid
        in whatever frame the encoders currently report."""
        uncal = [n for n in self._joints if not self._calibrated[n]]
        if uncal:
            raise ControlError(
                f"Calibrate all joints first — {len(uncal)} uncalibrated "
                f"(e.g. {uncal[0].replace('_joint','')}).", 409)

    def arm(self) -> None:
        # NOTE: arm() does NOT require calibration. It only affirms "operator present / robot
        # supported" so manual capture-and-hold can run uncalibrated (e.g. a standing demo).
        # Calibration is still enforced at the point of motion that needs it (start_hold /
        # start_policy), so an uncalibrated arm cannot ramp to default_pose or run the policy.
        with self._lock:
            if self._state != SessionState.CONNECTED:
                raise ControlError(
                    f"Cannot arm from {self._state.value}; connect first.", 409)
            if self.estop.fired:
                raise ControlError("E-STOP is latched — reconnect to clear.", 409)
            self._armed = True
        _log.info("ARMED (operator present / robot supported).")

    # ── position_offset calibration ──────────────────────────────────────────
    def all_calibrated(self) -> bool:
        return all(self._calibrated.values())

    def _require_connected_idle(self, joint: str) -> None:
        if joint not in self._calibrated:
            raise ControlError(f"Unknown joint {joint!r}.", 404)
        if self.is_motion_active():
            raise ControlError("A motion session is active — stop it before calibrating.", 409)
        if self._state not in (SessionState.CONNECTED,):
            raise ControlError(f"Connect first (state={self._state.value}).", 409)
        st = self.client.get_cached_joint_state(joint)
        if st is None or (st.get("state") or st.get("joint_state")) in (None, "OFFLINE"):
            raise ControlError(f"{joint.replace('_joint','')} is offline.", 409)

    def cal_start(self, joint: str) -> dict:
        """Begin calibrating one joint: IDLE it (hand-movable, zero torque) and zero its
        position_offset so subsequent captures read RAW encoder position. No commanded motion."""
        self._require_connected_idle(joint)
        self.client.set_mode(joint, "IDLE")
        self.client.apply_config(joint, {"position_offset": 0.0})
        time.sleep(0.25)   # let the offset write + telemetry settle
        with self._lock:
            self._cal_captures[joint] = {"lower": None, "upper": None}
            self._calibrated[joint] = False
        _log.info("cal start %s (offset→0, IDLE).", joint)
        return {"joint": joint, "captured": self._cal_captures[joint]}

    def cal_capture(self, joint: str, which: str) -> dict:
        """Capture the RAW position at the current hardstop (which='lower'|'upper')."""
        if which not in ("lower", "upper"):
            raise ControlError("which must be 'lower' or 'upper'.", 400)
        self._require_connected_idle(joint)
        pos = self.client.get_state(joint).get("position")
        if pos is None:
            raise ControlError(f"No position reading for {joint.replace('_joint','')}.", 409)
        with self._lock:
            self._cal_captures[joint][which] = float(pos)
        _log.info("cal capture %s %s = %.5f rad", joint, which, pos)
        return {"joint": joint, "which": which, "position": float(pos),
                "captured": dict(self._cal_captures[joint])}

    def cal_apply(self, joint: str) -> dict:
        """Compute + write position_offset from the two captures; mark the joint calibrated."""
        self._require_connected_idle(joint)
        cap = self._cal_captures.get(joint, {})
        lower, upper = cap.get("lower"), cap.get("upper")
        if lower is None or upper is None:
            raise ControlError("Capture both lower and upper hardstops first.", 409)
        min_rad, max_rad = self._limits[joint]
        res = compute_offset(lower, upper, min_rad, max_rad)
        if res["flipped"]:
            raise ControlError(
                "Upper hardstop read below lower — captures swapped or gear sign wrong. "
                "Re-capture (lower stop first); not applying.", 409)
        self.client.apply_config(joint, {"position_offset": res["position_offset"]})
        time.sleep(0.1)
        with self._lock:
            self._calibrated[joint] = True
            self._cal_captures[joint] = {"lower": None, "upper": None}
            if all(self._calibrated.get(n, False) for n in self._joints):
                self._joints_dropped_since_cal = False
        _log.info("cal apply %s: offset=%.5f range_ok=%s (err=%.4f rad)",
                  joint, res["position_offset"], res["range_ok"], res["range_error_rad"])
        return {"joint": joint, "calibrated": True, **res}

    def cal_reset(self, joint: str) -> dict:
        """Discard captures for a joint (does not touch the ESC offset)."""
        with self._lock:
            if joint in self._cal_captures:
                self._cal_captures[joint] = {"lower": None, "upper": None}
        return {"joint": joint, "captured": self._cal_captures.get(joint, {})}

    # Small tolerance (rad) so a joint resting exactly at a hardstop isn't flagged by noise.
    _CAL_LIMIT_TOL = 0.05

    def cal_check_limits(self) -> list[dict]:
        """Return the joints whose live position is OUTSIDE their configured limits (or offline).
        Empty list ⇒ every joint's ESC offset looks valid."""
        bad: list[dict] = []
        for name in self._joints:
            st = self.client.get_cached_joint_state(name)
            state = (st or {}).get("state") or (st or {}).get("joint_state")
            pos = (st or {}).get("position")
            lo, hi = self._limits[name]
            if st is None or state in (None, "OFFLINE") or pos is None:
                bad.append({"joint": name, "reason": "offline", "position": None, "min": lo, "max": hi})
            elif pos < lo - self._CAL_LIMIT_TOL or pos > hi + self._CAL_LIMIT_TOL:
                bad.append({"joint": name, "reason": "out_of_limits", "position": pos, "min": lo, "max": hi})
        return bad

    # ── arm zeroing from a held pose ─────────────────────────────────────────
    _TEACH_SAMPLE_S = 1.5          # averaged, so a slightly unsteady hold still lands well
    _TEACH_STEADY_DEG = 2.0        # peak-to-peak above this and the hold is reported as shaky

    def teach_arm_zero(self, limb: str) -> dict:
        """Zero one arm from the T-pose the operator is holding.

        The arms have no hardstops, so the per-joint capture flow cannot be used on them (see
        ``humanoid_control.arm_calibration``). This samples the held pose, solves each joint's
        ``position_offset`` so it reads its known T-pose angle, writes it, and verifies by
        reading back — a write that does not land is reported rather than assumed.

        Writes ``position_offset`` only. Nothing is commanded to move.
        """
        from ..arm_calibration import is_declared, t_pose_targets

        if limb not in self._layout.arms:
            raise ControlError(
                f"{limb} is not configured — attached: {', '.join(self._layout.arms) or 'none'}",
                400)
        if self._state != SessionState.CONNECTED:
            raise ControlError(f"Connect first (state={self._state.value}).", 409)
        if self.is_motion_active():
            raise ControlError("A motion session is active — stop it before calibrating.", 409)

        targets = t_pose_targets(limb)
        joints = [n for n in self._layout.joints_of(limb) if n in targets]
        offline = [n for n in joints if not self._joint_online(n)]
        if offline:
            raise ControlError(
                "offline: " + ", ".join(n.replace("_joint", "") for n in offline), 409)

        # Average the hold. A single sample would bake in whatever jitter happened to be on that
        # frame; the spread is reported so a shaky hold is visible rather than silently accepted.
        samples: dict[str, list[float]] = {n: [] for n in joints}
        deadline = time.monotonic() + self._TEACH_SAMPLE_S
        while time.monotonic() < deadline:
            for n in joints:
                st = self.client.get_cached_joint_state(n)
                p = (st or {}).get("position")
                if isinstance(p, (int, float)):
                    samples[n].append(float(p))
            time.sleep(0.02)
        if any(not v for v in samples.values()):
            raise ControlError("No telemetry while sampling — is the daemon running?", 503)

        held = {n: sum(v) / len(v) for n, v in samples.items()}
        spread = {n: (max(v) - min(v)) for n, v in samples.items()}
        worst = max(spread.values()) * 180.0 / np.pi

        results = []
        for n in joints:
            old = self._read_offset(n)
            if old is None:
                results.append({"joint": n, "ok": False, "reason": "could not read offset"})
                continue
            want = targets[n]
            new = old - (want - held[n])
            try:
                self.client.apply_config(n, {"position_offset": float(new)}, timeout=10.0)
            except Exception as exc:
                results.append({"joint": n, "ok": False, "reason": str(exc)})
                continue
            results.append({
                "joint": n,
                "ok": True,
                "declared": is_declared(n),
                "was_deg": held[n] * 180.0 / np.pi,
                "target_deg": want * 180.0 / np.pi,
                "shift_deg": (want - held[n]) * 180.0 / np.pi,
                "offset": float(new),
            })
        time.sleep(0.4)

        # Verify from telemetry rather than trusting the ACK: a write that silently fails would
        # otherwise leave a joint marked calibrated with the wrong zero.
        for r in results:
            if not r.get("ok"):
                continue
            st = self.client.get_cached_joint_state(r["joint"]) or {}
            now = st.get("position")
            r["now_deg"] = (now * 180.0 / np.pi) if isinstance(now, (int, float)) else None
            r["error_deg"] = (None if r["now_deg"] is None
                              else round(r["now_deg"] - r["target_deg"], 2))
            if r["error_deg"] is None or abs(r["error_deg"]) > 3.0:
                r["ok"] = False
                r["reason"] = "did not land on target"

        ok = all(r.get("ok") for r in results)
        with self._lock:
            for r in results:
                if r.get("ok"):
                    self._calibrated[r["joint"]] = True
            if all(self._calibrated.get(n, False) for n in self._joints):
                self._joints_dropped_since_cal = False
        _log.info("teach %s from T-pose: %s (hold steady to %.2f deg)",
                  limb, "OK" if ok else "INCOMPLETE", worst)
        return {
            "limb": limb, "ok": ok,
            "steady_deg": round(worst, 2),
            "shaky": worst > self._TEACH_STEADY_DEG,
            "joints": results,
        }

    def _joint_online(self, name: str) -> bool:
        st = self.client.get_cached_joint_state(name)
        state = (st or {}).get("state") or (st or {}).get("joint_state")
        return st is not None and state not in (None, "OFFLINE")

    def _read_offset(self, name: str) -> float | None:
        """READ_CONFIG drops a random param per call; retry until position_offset lands."""
        for _ in range(6):
            try:
                c = self.client.read_device_config(name)
            except Exception:
                continue
            if c.get("position_offset") is not None:
                return float(c["position_offset"])
        return None

    def cal_mark_complete(self) -> dict:
        """Operator override: mark ALL joints calibrated without re-running per-joint calibration.
        Only allowed when every joint's live position is within its configured limits — a sanity
        check that the ESC offsets are still valid (e.g. the app/session restarted but the robot
        stayed powered). Refuses (marks nothing) if any joint is out of limits or offline."""
        if self._state != SessionState.CONNECTED:
            raise ControlError(f"Connect first (state={self._state.value}).", 409)
        bad = self.cal_check_limits()
        if bad:
            return {"marked": False, "out_of_limits": bad}
        with self._lock:
            self._calibrated = {n: True for n in self._joints}
            self._joints_dropped_since_cal = False
        _log.info("calibration marked complete by operator override (all joints within limits).")
        return {"marked": True, "out_of_limits": []}

    def clear_faults(self) -> dict:
        """Clear firmware errors on every joint (CLEAR_ERROR → error reg 0 + IDLE) and release a
        latched E-STOP, so the operator recovers WITHOUT a full reconnect. Preserves calibration
        (no power cycle ⇒ offsets are still valid). This is the app's fault-recovery path."""
        if not self.client.is_running():
            raise ControlError("Daemon not running.", 503)
        if self.is_motion_active():
            raise ControlError("Stop the active session before clearing faults.", 409)
        cleared = 0
        for n in self._joints:
            try:
                self.client.clear_error(n)
                cleared += 1
            except Exception as exc:
                _log.warning("clear_error %s failed: %s", n, exc)
        time.sleep(0.3)
        with self._lock:
            self.estop = self._new_estop()     # release the latched E-STOP
            self._armed = False
            self._last_error = None
            self._state = SessionState.CONNECTED
        _log.info("faults cleared on %d/%d joints; E-STOP released; state → CONNECTED.",
                  cleared, len(self._joints))
        return {"cleared": cleared}

    def disarm(self) -> None:
        with self._lock:
            self._armed = False

    # ── motion sessions ──────────────────────────────────────────────────────
    def start_hold(self, *, ramp: float = 5.0, seconds: float | None = None) -> None:
        # Leg gate BEFORE the calibration gate: on an arm-only layout there is no amount of
        # calibrating that would make this work, so "calibrate first" would be a dead end.
        self._require_legs()
        # ZeroPolicy ramps to the CALIBRATED-frame default_pose — requires calibration.
        self._require_calibrated()
        self._start_session("hold", ZeroPolicy(self.contract.num_joints),
                            command=None, ramp=ramp, seconds=seconds)

    def start_policy(self, *, checkpoint: str, command=None,
                     ramp: float = 5.0, seconds: float | None = None) -> None:
        self._require_legs()
        # The learned policy commands CALIBRATED-frame targets — requires calibration.
        self._require_calibrated()
        policy = load_policy(checkpoint, num_actions=self.contract.num_joints)
        cmd = np.array(command if command is not None else [0.0, 0.0, 0.0], dtype=np.float32)
        self._start_session("policy", policy, command=cmd, ramp=ramp, seconds=seconds,
                            checkpoint=checkpoint)

    def _require(self, capability: str) -> None:
        """Gate a session on a layout CAPABILITY rather than on limb names.

        The point is that adding a limb to the config changes what the robot will accept
        without any code changing. 'walk' needs both legs because the policy is contract-bound
        — it commands exactly the 12 leg joints with a 45-dim observation built from them, and
        there is no partial version. 'arm_teleop' needs at least one arm. 'pose' needs anything
        at all.
        """
        if not self._layout.can(capability):
            raise ControlError(self._layout.why_not(capability), 409)

    def _require_legs(self) -> None:
        """Back-compat alias for the walk gate."""
        self._require("walk")

    # ── what this machine can currently be asked to do ───────────────────────
    @property
    def capabilities(self) -> tuple[str, ...]:
        return self._layout.capabilities

    def arm_targets(self) -> tuple[str, ...]:
        """Arms available to teleop, in layout order. One entry per configured arm."""
        return self._layout.arms

    def _preflight_motion(self) -> None:
        """Common gate for any motion session (caller holds self._lock). Calibration is NOT
        checked here — it is enforced only by the motions that command a calibrated-frame target
        (start_hold / start_policy); manual capture-and-hold intentionally runs uncalibrated.

        Note this does NOT require legs — pose motion drives whatever the layout says is
        attached. The policy paths add _require_legs() on top."""
        if self._state != SessionState.CONNECTED:
            raise ControlError(
                f"Cannot start motion from {self._state.value}; connect + arm first.", 409)
        if not self._armed:
            raise ControlError("Not armed — set 'I am present' before any motion.", 409)
        if self.estop.fired:
            raise ControlError("E-STOP is latched — reconnect to clear.", 409)
        if not self.client.is_running():
            raise ControlError("Daemon not running — no telemetry.", 503)
        # Web-driven motion (hold / run_policy) is supervised from the PAGE, so it is the
        # browser that must be live — checked by name rather than via deadman_ok(), which
        # answers about the active input source and would otherwise let a gamepad vouch for
        # a closed browser tab (or refuse a browser-only run because a pad is switched off).
        if not (self._control_clients > 0 and self.source_alive("web")):
            raise ControlError(
                "No live deadman connection — open the control page and keep it focused.", 409)
        if self._session_thread and self._session_thread.is_alive():
            raise ControlError("A motion session is already running.", 409)

    def _start_session(self, kind, policy, *, command, ramp, seconds, checkpoint=None) -> None:
        with self._lock:
            self._preflight_motion()
            self._stop_evt.clear()
            new_state = SessionState.HOLDING if kind == "hold" else SessionState.RUNNING
            self._state = new_state
            # Real base state from the daemon's IMU `base` block. When the daemon
            # reports no fresh IMU data, TelemetryBaseState yields valid=False and the
            # runner falls back to the upright stub with a warning (require_valid_base
            # left False so an IMU hiccup can't hard-crash a live motion session — the
            # human + deadman remain the safety of record). Flip to True once the
            # balance loop is trusted unsupported.
            # Web-driven session: the browser is the deadman of record for its whole life.
            self._session_deadman = "web"
            runner = PolicyRunner(
                self.client, self.contract, policy,
                base_source=TelemetryBaseState(lambda: {"base": self.client.latest_base()}),
                command=command, estop=self.estop, ramp_seconds=ramp,
            )
            t = threading.Thread(
                target=self._session_worker, args=(runner, seconds, kind, checkpoint),
                name=f"motion-{kind}", daemon=True,
            )
            self._session_thread = t
            t.start()
        _log.info("motion session started: %s (ramp=%.1fs, seconds=%s)", kind, ramp, seconds)

    def _session_worker(self, runner: PolicyRunner, max_seconds, kind, checkpoint) -> None:
        """Runs the ramp + policy loop synchronously off the event loop.

        Mirrors ``PolicyRunner.run`` but with a cooperative ``_stop_evt`` for graceful stop in
        addition to the ``estop.fired`` hard stop, so /api/stop and /api/estop are distinct.
        """
        moved = False
        try:
            if not runner.prepare():   # MOTION: enable + ramp (checks estop to abort)
                _log.info("session %s aborted during ramp.", kind)
                return
            moved = True
            dt = self.contract.policy_dt
            t0 = time.monotonic()
            next_tick = t0
            while not self.estop.fired and not self._stop_evt.is_set():
                if max_seconds is not None and (time.monotonic() - t0) >= max_seconds:
                    _log.info("session %s reached max_seconds.", kind)
                    break
                self.legs.check_health()   # raises on fault → finally IDLEs
                runner.step()
                next_tick += dt
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.monotonic()
        except Exception as exc:
            _log.error("session %s error: %s", kind, exc)
            with self._lock:
                self._last_error = f"{kind}: {exc}"
            self.trigger_estop(f"{kind}-fault")
        finally:
            try:
                runner.shutdown()   # legs → IDLE
            except Exception as exc:
                _log.warning("shutdown after %s failed: %s", kind, exc)
            self._on_session_end()
            _log.info("session %s ended (moved=%s).", kind, moved)

    def _on_session_end(self) -> None:
        with self._lock:
            self._armed = False    # require an explicit re-arm before the next motion
            self._run_gate.clear()
            self._session_deadman = None   # back to judging by the active input source
            if self.estop.fired:
                self._state = SessionState.ESTOPPED
            elif self._state in _ACTIVE_STATES:
                self._state = SessionState.CONNECTED

    # ── gamepad deadman session (hold-to-run) ────────────────────────────────
    #
    # The operational flow: connect → calibrate → arm_deadman() (legs DAMPING, ARMED) →
    # hold a trigger to engage (ramp to default_pose, then run the selected session with the
    # live walk command) → release to DAMP → repeat. The gamepad is the deadman: losing the
    # controller (not merely releasing the trigger) E-STOPs via the presence watchdog.

    # Session kind -> the layout capability it needs. Adding a limb to the config enables the
    # matching kinds with no code change; that is the whole point of gating on capabilities.
    SESSION_CAPABILITY = {
        "hold": "walk",        # ZeroPolicy -> the contract default_pose
        "policy": "walk",      # a learned leg checkpoint
        "manual": "pose",      # capture-and-hold the live pose
        "arm": "arm_teleop",   # direct arm control from the sticks
    }

    def available_sessions(self) -> list[str]:
        """Session kinds this layout can actually run, for the UI to offer."""
        return [k for k, cap in self.SESSION_CAPABILITY.items() if self._layout.can(cap)]

    def select_session(self, kind: str, checkpoint: str | None = None,
                       limb: str | None = None) -> None:
        """Pick what a trigger-engage runs. Only settable while not in a live session.

        'hold'   ZeroPolicy -> default_pose          (needs both legs)
        'policy' a learned leg checkpoint            (needs both legs)
        'manual' capture-and-hold the live pose      (needs any limb)
        'arm'    direct arm control from the sticks  (needs an arm)

        ``limb`` picks which arm an 'arm' session drives; defaults to the first configured arm,
        which is the only one on a single-arm machine.
        """
        if kind not in self.SESSION_CAPABILITY:
            raise ControlError(
                f"kind must be one of {', '.join(sorted(self.SESSION_CAPABILITY))}.", 400)
        self._require(self.SESSION_CAPABILITY[kind])
        if kind == "policy" and not checkpoint:
            raise ControlError("policy session needs a checkpoint.", 400)
        if kind == "arm":
            arms = self._layout.arms
            limb = limb or arms[0]
            if limb not in arms:
                raise ControlError(
                    f"{limb} is not configured — available: {', '.join(arms) or 'none'}", 400)
        with self._lock:
            if self._state in _ACTIVE_STATES:
                raise ControlError("Disarm before changing the selected session.", 409)
            self._selected = {"kind": kind, "checkpoint": checkpoint, "limb": limb}

    def set_run_gate(self, active: bool, *, source: str = "web") -> None:
        """Deadman trigger state from the active input source: True = held (engage/run), False
        = released (damp). Distinct from the heartbeat — a release damps; a controller loss
        E-STOPs. Ignored (and counted) from a source that does not hold the input token."""
        if not self._owns_input(source):
            return
        if active:
            self._run_gate.set()
        else:
            self._run_gate.clear()

    # ── input source arbitration ─────────────────────────────────────────────
    #
    # Exactly ONE source drives the robot at a time. This is a token, not a preference: two
    # live sources both believing they are driving is the failure this exists to prevent.
    # E-STOP is deliberately NOT gated by it — any source may always stop the robot.

    INPUT_SOURCES = ("xbox", "quest", "web")

    @property
    def input_source(self) -> str:
        return self._input_source

    def _owns_input(self, source: str) -> bool:
        """True if `source` may command. Otherwise counts the ignored write, so a controller
        that is being deliberately ignored shows up in the UI instead of just feeling dead."""
        if source == self._input_source:
            return True
        self._ignored_writes[source] = self._ignored_writes.get(source, 0) + 1
        return False

    def available_input_sources(self) -> list[str]:
        """Sources this machine can actually be driven by, for the UI to offer."""
        out = ["web"]
        if self._gamepad["enabled"]:
            out.insert(0, "xbox")
        if os.environ.get("HUMANOID_QUEST_ENABLE"):
            out.insert(0, "quest")
        return out

    def set_input_source(self, source: str) -> None:
        """Pick what drives the robot. Refused mid-session for the same reason
        set_control_mode is: handing authority over while the robot is moving is exactly the
        transition nobody can supervise."""
        avail = self.available_input_sources()
        if source not in avail:
            raise ControlError(
                f"{source} input unavailable (available: {', '.join(avail)}).", 409)
        if self._state in _ACTIVE_STATES:
            raise ControlError("Disarm before switching control method.", 409)
        self._input_source = source
        self._ignored_writes.clear()
        _log.info("input source: %s", source)

    # ── control mode / speed / limb selection (gamepad-facing) ───────────────
    @property
    def control_mode(self) -> str:
        """'arm' or 'leg' — which set of things the sticks drive."""
        return self._control_mode

    @property
    def speed_mode(self) -> str:
        return self._speed_mode

    def set_speed_mode(self, mode: str) -> None:
        if mode not in ("normal", "creep"):
            raise ControlError("speed mode must be 'normal' or 'creep'.", 400)
        self._speed_mode = mode
        _log.info("speed mode: %s", mode)

    def available_control_modes(self) -> list[str]:
        """Modes this layout supports. A machine with no legs never offers leg control."""
        modes = []
        if self._layout.can("arm_teleop"):
            modes.append("arm")
        if self._layout.can("walk"):
            modes.append("leg")
        return modes

    def set_control_mode(self, mode: str) -> None:
        modes = self.available_control_modes()
        if mode not in modes:
            raise ControlError(
                f"{mode} control unavailable — layout is '{self._layout.describe()}' "
                f"(available: {', '.join(modes) or 'none'}).", 409)
        if self._state in _ACTIVE_STATES:
            raise ControlError("Disarm before switching control mode.", 409)
        self._control_mode = mode
        _log.info("control mode: %s", mode)

    def toggle_control_mode(self) -> None:
        """Select's job. A no-op when the layout supports only one mode, which is the common
        case on a single-limb bench setup."""
        modes = self.available_control_modes()
        if len(modes) < 2:
            _log.info("control mode toggle ignored — only %s available",
                      modes[0] if modes else "nothing")
            return
        self.set_control_mode(modes[(modes.index(self._control_mode) + 1) % len(modes)]
                              if self._control_mode in modes else modes[0])

    def select_arm(self, limb: str) -> None:
        """Bumper's job: pick which arm the sticks drive. Ignored when that arm is not
        configured, so LB on a right-arm-only machine does nothing rather than erroring."""
        if limb not in self._layout.arms:
            _log.info("select_arm(%s) ignored — not configured", limb)
            return
        if self._state in _ACTIVE_STATES and self._selected.get("limb") != limb:
            raise ControlError("Disarm before switching arms.", 409)
        self._selected["limb"] = limb
        _log.info("arm selected: %s", limb)

    def set_arm_command(self, left_x: float, left_y: float,
                        right_y: float, right_x: float = 0.0, *, source: str = "web") -> None:
        """Raw stick quad in [-1,1], "up"/"right" positive. ArmTeleop decides what the axes mean
        for the active frame, and owns the deadband and rate scaling, so each is applied once."""
        if not self._owns_input(source):
            return
        with self._command_lock:
            self._arm_command = np.array([left_x, left_y, right_y, right_x], dtype=np.float32)

    def set_arm_pose_command(self, delta_m, seq: int = 0, *, source: str = "web") -> None:
        """Hand DISPLACEMENT since the clutch anchor, metres, robot frame — from a 6-DOF
        tracker. Kept separate from `_arm_command` rather than overloading it: a stick quad is
        a velocity and this is a position offset, and a stale value of one interpreted as the
        other is precisely the confusion worth designing out."""
        if not self._owns_input(source):
            return
        with self._command_lock:
            self._arm_pose_command = (
                np.asarray(delta_m, dtype=np.float32).reshape(3).copy(), int(seq))

    def arm_chain(self):
        """Kinematic chain for the selected arm. Cached — building it parses the vendored
        URDF model, and the retargeter asks for it on every XR frame."""
        limb = self.selected_limb()
        if limb is None:
            raise ControlError("no arm configured", 409)
        if getattr(self, "_chain_cache", (None, None))[0] != limb:
            from ..arm_kinematics import ArmChain
            self._chain_cache = (limb, ArmChain(list(self._layout.joints_of(limb))))
        return self._chain_cache[1]

    def selected_limb(self) -> str | None:
        """Which arm a teleop session drives (first configured arm when unset)."""
        return self._selected.get("limb") or (self._layout.arms[0] if self._layout.arms else None)

    def set_walk_command(self, vx: float, vy: float, wz: float, *, source: str = "web") -> None:
        """Live locomotion command (forward, lateral, yaw) from the gamepad sticks; consumed
        by the running policy each tick. Ignored (harmless) for a 'hold' session."""
        if not self._owns_input(source):
            return
        with self._command_lock:
            self._command = np.array([vx, vy, wz], dtype=np.float32)

    def arm_deadman(self, kind: str | None = None, checkpoint: str | None = None,
                    *, ramp: float = 1.5) -> None:
        """Enter the ARMED deadman session: legs → DAMPING, spawn the trigger-driven worker.
        Requires CONNECTED + a live controller (deadman); all joints calibrated UNLESS the
        selected session is 'manual' (capture-and-hold the live pose — no calibration needed)."""
        with self._lock:
            if self._state != SessionState.CONNECTED:
                raise ControlError(f"Cannot arm from {self._state.value}; connect first.", 409)
            if self.estop.fired:
                raise ControlError("E-STOP is latched — reconnect to clear.", 409)

            # Resolve the session kind first — the calibration gate depends on it.
            # Default the session kind from the CONTROL MODE. Select switches arm/leg, so A
            # must arm whatever that mode implies — otherwise an arm-only machine defaults to
            # the leg 'hold' session and refuses to arm for lacking legs, which is confusing
            # and looks like a dead button.
            sel_kind = kind or self._default_session_kind()
            sel_ckpt = checkpoint if kind else self._selected["checkpoint"]
            sel_limb = self._selected.get("limb")
            if kind:
                self._selected = {"kind": sel_kind, "checkpoint": sel_ckpt, "limb": sel_limb}
            # Gate on what the SELECTED kind needs, so a gamepad on an arm-only machine arms an
            # arm session instead of being refused for lacking legs.
            self._require(self.SESSION_CAPABILITY.get(sel_kind, "pose"))
            if sel_kind == "arm" and not sel_limb:
                sel_limb = self._layout.arms[0]
                self._selected["limb"] = sel_limb

            if sel_kind != "manual":
                uncal = [n for n in self._joints if not self._calibrated[n]]
                if uncal:
                    raise ControlError(
                        f"Calibrate all joints before arming — {len(uncal)} uncalibrated.", 409)
            if not self.client.is_running():
                raise ControlError("Daemon not running — no telemetry.", 503)
            # The deadman of record is whatever holds the input token — checked by name so a
            # live browser tab cannot vouch for a controller that is switched off.
            if not self.source_alive(self._input_source):
                raise ControlError(
                    f"No live {self._input_source} controller — connect it first.", 409)
            if self._session_thread and self._session_thread.is_alive():
                raise ControlError("A session is already active.", 409)

            if sel_kind == "policy":
                if not sel_ckpt:
                    raise ControlError("policy session needs a checkpoint.", 400)
                policy = load_policy(sel_ckpt, num_actions=self.contract.num_joints)
            else:
                policy = ZeroPolicy(self.contract.num_joints)   # unused for 'manual'

            runner = PolicyRunner(
                self.client, self.contract, policy,
                base_source=TelemetryBaseState(lambda: {"base": self.client.latest_base()}),
                command=self._command.copy(), estop=self.estop, ramp_seconds=ramp,
            )
            self._armed = True
            self._stop_evt.clear()
            self._run_gate.clear()
            # This session is judged by the source that armed it for its whole life, even if
            # the token were somehow changed underneath it.
            self._session_deadman = self._input_source
            self._state = SessionState.ARMED
            t = threading.Thread(target=self._deadman_worker, args=(runner, sel_kind),
                                 name=f"deadman-{sel_kind}", daemon=True)
            self._session_thread = t
            t.start()
        _log.info("ARMED deadman session (kind=%s, ramp=%.1fs) — legs DAMPING, hold a trigger to run.",
                  sel_kind, ramp)

    def _calibration_still_valid(self) -> bool:
        """True only if every configured joint has stayed online since calibration was set.

        This is what lets a plain reconnect keep its calibration while a power cycle never
        does. It is deliberately one-directional: any observed dropout invalidates, and only an
        explicit (re)calibration clears it. Single-turn encoders mean a joint that lost power
        cannot be trusted again without re-teaching, however briefly it was gone.
        """
        return not self._joints_dropped_since_cal and bool(self._calibrated) \
            and all(self._calibrated.get(n, False) for n in self._joints)

    def note_joint_dropout(self) -> None:
        """Called from the watchdog when a configured joint reads OFFLINE."""
        if not self._joints_dropped_since_cal:
            _log.warning("a joint went offline — calibration is no longer trustworthy "
                         "(single-turn encoders lose their zero on power loss).")
        self._joints_dropped_since_cal = True

    def _default_session_kind(self) -> str:
        """What A arms, given the current control mode and what was last explicitly selected."""
        if self._control_mode == "arm" and self._layout.can("arm_teleop"):
            return "arm"
        chosen = self._selected.get("kind") or "hold"
        # A leg kind on a machine with no legs would refuse; fall back to something runnable.
        if not self._layout.can(self.SESSION_CAPABILITY.get(chosen, "pose")):
            runnable = self.available_sessions()
            return runnable[0] if runnable else chosen
        return chosen

    def disarm_deadman(self) -> None:
        """Leave the deadman session: stop the worker (legs → IDLE), back to CONNECTED."""
        self.stop(wait=True)

    def _deadman_worker(self, runner: PolicyRunner, kind: str) -> None:
        """Persistent trigger-driven loop. Released trigger → IDLE (rest); held trigger → engage
        then hold. 'hold'/'policy' engage by ramping to default_pose then stepping the policy with
        the live command. 'manual' instead enables POSITION and holds the LIVE pose exactly — the
        daemon seeds the firmware target from the current measured position and streams it, so no
        target is sent (no ramp, no clamp, no calibration). Survives release/re-press; exits only
        on stop or E-STOP."""
        engaged = False
        manual = (kind == "manual")
        arm_mode = (kind == "arm")
        # Arm teleop gets its OWN rate. policy_dt (25 Hz) is the leg policy's tick, inherited
        # here for no arm-specific reason; at 40 ms a 6-DOF pose input is visibly staircased.
        # Safe to raise: TeleopTuning.max_joint_rate is rad/SECOND multiplied by the real dt,
        # and the leash / reach clamps are rate-correct, so the tuning constants still hold.
        dt = (1.0 / _ARM_HZ) if arm_mode else self.contract.policy_dt
        if arm_mode:
            _log.info("arm teleop tick: %.0f Hz (dt=%.4fs)", _ARM_HZ, dt)
        engaged_state = SessionState.RUNNING if kind == "policy" else SessionState.HOLDING

        # Arm teleop drives only the selected arm's joints, so it gets its own interface and
        # its own rest/engage handling. Everything else about the session — the trigger gate,
        # the heartbeat, E-STOP, the finally-IDLE — is shared, which is the point: arm teleop
        # inherits the safety envelope rather than reimplementing it.
        limb = self._selected.get("limb")
        group = self.group
        teleop = None
        recorder = None
        if arm_mode:
            from ..arm_kinematics import ArmChain
            from ..arm_teleop import ArmTeleop, TeleopTuning
            from ..recorder import ArmRunRecorder
            arm_joints = list(self._layout.joints_of(limb))
            group = JointGroupInterface(self.client, arm_joints)
            # A 6-DOF tracker supplies an absolute hand displacement, not stick deflections,
            # so the teleop runs its 'pose' frame. Fixed at session start from the source that
            # armed it — the input token cannot change mid-session anyway.
            # MIRROR when the Quest is driving AND the operator has a calibration profile;
            # otherwise fall back to the controller-position path. Chosen once at session
            # start: the input token cannot change mid-session anyway, and switching mapping
            # under a moving arm is exactly the transition nobody can supervise.
            quest_src = (self._session_deadman == "quest")
            mirror_mode = bool(quest_src and self.quest is not None
                               and getattr(self.quest, "_profile", None) is not None)
            pose_mode = quest_src and not mirror_mode
            if mirror_mode:
                tuning = TeleopTuning(frame="mirror")
            elif pose_mode:
                tuning = TeleopTuning(frame="pose")
            else:
                tuning = TeleopTuning()
            if quest_src and not mirror_mode:
                _log.warning("arm teleop: Quest is driving but there is NO calibration "
                             "profile — falling back to controller-position mode. Run the "
                             "arm calibration to mirror your whole arm.")
            teleop = ArmTeleop(ArmChain(arm_joints), tuning=tuning)
            _log.info("arm teleop: driving %s (%d joints) in '%s' frame",
                      limb, len(arm_joints), tuning.frame)
            # Flight recorder for the whole armed session, engaged or not. Always on: the arm
            # has no policy to fall back on, and "it did not move how I expected" is only
            # answerable from the numbers afterwards.
            try:
                rec_dir = os.environ.get("HUMANOID_RECORD_DIR") or str(REPO_ROOT / "_arm_recording" / "runs")
                recorder = ArmRunRecorder(rec_dir, limb, arm_joints, teleop.tuning)
                _log.info("arm run log: %s", recorder.path)
            except Exception as exc:
                _log.warning("arm run log unavailable (%s) — continuing without it.", exc)

        try:
            group.idle()   # ARMED rest = IDLE (zero-torque). DAMPING faults the firmware
            # watchdog in ~1s (not daemon-fed) — see notes; IDLE is dormant + safe.
            if manual:
                # Disable the firmware position clamp for the whole armed session so a trigger-
                # engage holds any hand-set pose (even outside soft limits). Restored in finally.
                self._widen_position_limits()
            next_tick = time.monotonic()
            while not self.estop.fired and not self._stop_evt.is_set():
                if self._run_gate.is_set():
                    if not engaged:
                        with self._lock:
                            self._state = engaged_state
                        if arm_mode:
                            # ARM ENGAGE: enable POSITION (the daemon seeds the firmware target
                            # from the live measured position, so this is jerk-free) and seed the
                            # teleop target at the hand's ACTUAL position. Seeding every engage
                            # is what stops the arm jumping to wherever the target was left.
                            group.enable_position()
                            q_now, _ = group.read_states()
                            teleop.reset(q_now)
                        elif manual:
                            # MANUAL ENGAGE: enable POSITION and hold where the robot is. The
                            # daemon seeds the firmware target from the live measured position on
                            # the IDLE→POSITION change and streams it every tick, so we send NO
                            # target — holds the pose as-read, in range or not. Jerk-free.
                            self.legs.enable_position()
                        else:
                            # Engage: seed@current (jerk-free from DAMPING) + ramp to default_pose,
                            # bailing the instant the trigger is released or E-STOP fires.
                            ok = runner.prepare(
                                should_abort=lambda: self._stop_evt.is_set() or not self._run_gate.is_set())
                            if not ok:
                                if not (self.estop.fired or self._stop_evt.is_set()):
                                    group.idle()              # released mid-ramp → rest (IDLE, not DAMPING: watchdog)
                                    with self._lock:
                                        self._state = SessionState.ARMED
                                continue
                        engaged = True
                        next_tick = time.monotonic()
                    group.check_health()                  # raises on fault → finally IDLEs
                    if arm_mode:
                        with self._command_lock:
                            cmd = self._arm_command.copy()
                            pose_cmd = self._arm_pose_command
                        q_now, v_now = group.read_states()
                        if mirror_mode:
                            # Whole-arm mirroring. `hold` freezes the command where it is
                            # when body tracking drops — an emulated joint is the headset
                            # guessing where the operator's elbow is, and a guess must not
                            # drive a motor. Freezing beats dropping to IDLE mid-motion on a
                            # bolted-down arm; if it persists the worker releases below.
                            tgts, hold = self.quest.mirror_command()
                            q_target, info = teleop.step_mirror(
                                q_now, tgts, dt, creep=(self._speed_mode == "creep"),
                                hold=hold)
                            # Defence in depth. The Quest source latches this release itself
                            # (see QuestSource._on_frame): it has to, because the gate is
                            # re-asserted there every frame while the trigger is held, so a
                            # clear from this worker alone would be overwritten ~16 ms later
                            # and the arm would oscillate IDLE<->POSITION at tick rate. This
                            # clear is the motor-side backstop for that, not the mechanism.
                            if self.quest.body_lost_too_long():
                                _log.warning("arm teleop: body tracking lost — releasing to IDLE.")
                                self._run_gate.clear()
                        elif pose_mode:
                            # No fresh sample yet (or the source dropped it) means HOLD, not
                            # "reuse the last displacement" — a stale offset would keep
                            # driving the arm after the operator's link went quiet.
                            delta = pose_cmd[0] if pose_cmd is not None else np.zeros(3)
                            q_target, info = teleop.step_pose(
                                q_now, delta, dt, creep=(self._speed_mode == "creep"))
                        else:
                            q_target, info = teleop.step(
                                q_now, cmd, dt, creep=(self._speed_mode == "creep"))
                        group.send_targets(q_target)
                        self._arm_info = info
                        if recorder is not None:
                            recorder.record(engaged=True, run_gate=True, sticks=cmd,
                                            joint_pos=q_now, joint_vel=v_now,
                                            joint_target=q_target, info=info,
                                            speed_mode=self._speed_mode,
                                            xr=self._xr_status(),
                                            human=self._human_angles())
                    elif not manual:
                        with self._command_lock:
                            runner.command = self._command.copy()
                        runner.step()
                    # manual: daemon streams the seeded live pose; nothing to send.
                    next_tick += dt
                    sleep = next_tick - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_tick = time.monotonic()
                else:
                    if engaged:
                        group.idle()                      # trigger released → rest (IDLE, not DAMPING: watchdog)
                        engaged = False
                        with self._lock:
                            self._state = SessionState.ARMED
                    else:
                        if recorder is not None and arm_mode:
                            try:
                                with self._command_lock:
                                    cmd = self._arm_command.copy()
                                q_now, v_now = group.read_states(require_online=False)
                                recorder.record(engaged=False, run_gate=False, sticks=cmd,
                                                joint_pos=q_now, joint_vel=v_now,
                                                speed_mode=self._speed_mode,
                                                xr=self._xr_status(),
                                                human=self._human_angles())
                            except Exception:
                                pass          # a log must never break the session
                        time.sleep(0.02)                  # idle damped, waiting for trigger
        except Exception as exc:
            _log.error("deadman session error: %s", exc)
            with self._lock:
                self._last_error = f"deadman: {exc}"
            self.trigger_estop("deadman-fault")
        finally:
            try:
                group.idle()   # leaving the session → free (IDLE), disarmed
            except Exception as exc:
                _log.warning("idle after deadman session failed: %s", exc)
            if manual:
                self._restore_position_limits()   # re-arm the firmware clamp
            if recorder is not None:
                recorder.close()
                _log.info("arm run log written: %s", recorder.path)
            self._on_session_end()
            _log.info("deadman session ended.")

    # ── ESC soft position limits (widen for manual hold) ─────────────────────
    # The firmware clamps every position target to each joint's configured position_limits
    # (below our Python layer — see daemon actuator.cpp). Manual hold widens them so a
    # hand-set pose OUTSIDE the normal range is held as-is instead of being yanked to the
    # limit; they are restored when the manual session ends. ±this is far beyond any leg
    # joint's mechanical range, so the clamp never bites while we hold the live pose.
    _MANUAL_WIDE_LIMIT_RAD = 12.0

    def _write_position_limits(self, bounds: dict[str, tuple[float, float]]) -> None:
        """Best-effort per-joint ESC soft position-limit write (display-frame rad)."""
        for name, (lo, hi) in bounds.items():
            try:
                self.client.apply_config(name, {"position_limit_min": float(lo),
                                                "position_limit_max": float(hi)}, timeout=5.0)
            except Exception as exc:
                _log.warning("position_limits write %s failed: %s", name, exc)

    def _widen_position_limits(self, joints=None) -> None:
        # Scoped to the joints being commanded: dropping the firmware clamp on a limb we are not
        # driving would remove a safety net for no benefit.
        w = self._MANUAL_WIDE_LIMIT_RAD
        self._write_position_limits({n: (-w, w) for n in (joints or self._joints)})
        _log.warning("MANUAL hold: ESC soft position limits widened to ±%.1f rad "
                     "(firmware clamp disabled — holds any hand-set pose).", w)

    def _restore_position_limits(self) -> None:
        self._write_position_limits({n: self._limits[n] for n in self._joints})
        _log.info("MANUAL hold: ESC soft position limits restored to configured range.")

    # ── manual control (capture-and-hold / go-to-pose) ───────────────────────
    def current_pose_rad(self) -> dict[str, float]:
        """Live position (rad) of every configured joint, in layout order. Raises if any is
        offline — a partial pose would silently mean something different from what it says."""
        out: dict[str, float] = {}
        for name in self._joints:
            st = self.client.get_cached_joint_state(name)
            state = (st or {}).get("state") or (st or {}).get("joint_state")
            if st is None or state in (None, "OFFLINE") or st.get("position") is None:
                raise ControlError(f"{name.replace('_joint','')} offline — cannot read pose.", 409)
            out[name] = float(st["position"])
        return out

    def start_manual_hold(self, targets_rad: dict[str, float], *,
                          ramp: float = 4.0, seconds: float | None = None,
                          clamp: bool = True) -> None:
        """Ramp the named joints to target positions (rad) and hold. Non-named joints stay IDLE.
        Motion — same gates as policy. Used by capture-and-hold (all 12 = current pose) and
        go-to-pose (a saved pose, possibly a subset).

        ``clamp=True`` (default, go-to-pose) clips each target to the joint's configured position
        limits — a saved target must not command past range. ``clamp=False`` (capture-and-hold)
        holds the RAW reading, even if a (possibly uncalibrated) encoder value is outside its
        limits: the robot stays exactly where it is instead of being forced into range. Out-of-
        limit joints are logged as a warning, not corrected."""
        # Commandable = whatever the layout says is attached. Anything else is reported rather
        # than dropped silently — a "hold" that quietly leaves part of the robot free is a
        # safety surprise.
        commandable = set(self._joints)
        dropped = sorted(n for n in (targets_rad or {}) if n not in commandable)
        targets_rad = {n: float(v) for n, v in (targets_rad or {}).items() if n in commandable}
        if not targets_rad:
            raise ControlError("No valid target joints to hold.", 400)
        if dropped:
            _log.warning("manual hold: %d joint(s) not in the current layout, left uncommanded: "
                         "%s", len(dropped),
                         ", ".join(n.replace("_joint", "") for n in dropped))
        with self._lock:
            self._preflight_motion()
            self._stop_evt.clear()
            self._state = SessionState.HOLDING
            t = threading.Thread(target=self._manual_worker, args=(targets_rad, ramp, seconds, clamp),
                                 name="motion-manual", daemon=True)
            self._session_thread = t
            t.start()
        _log.info("manual hold started: %d joints (ramp=%.1fs, clamp=%s).", len(targets_rad), ramp, clamp)

    def _manual_worker(self, targets: dict[str, float], ramp: float, seconds, clamp: bool = True) -> None:
        joints = [n for n in self._joints if n in targets]   # layout order
        lo = np.array([self._limits[n][0] for n in joints], dtype=np.float32)
        hi = np.array([self._limits[n][1] for n in joints], dtype=np.float32)
        raw = np.array([targets[n] for n in joints], dtype=np.float32)
        # capture-and-hold (clamp=False) holds the raw reading; go-to-pose clamps to limits.
        goal = np.clip(raw, lo, hi) if clamp else raw
        if not clamp:
            out = [f"{joints[k].replace('_joint','')}={raw[k]:+.3f}"
                   for k in range(len(joints)) if raw[k] < lo[k] or raw[k] > hi[k]]
            if out:
                _log.warning("manual capture-hold: %d joint(s) OUTSIDE limits, holding raw: %s",
                             len(out), ", ".join(out))

        def send(vec) -> None:
            for n, v in zip(joints, vec):
                self.client.set_position(n, float(v))

        def read_current():
            out = np.zeros(len(joints), dtype=np.float32)
            for k, n in enumerate(joints):
                st = self.client.get_cached_joint_state(n)
                out[k] = st["position"] if st and st.get("position") is not None else np.nan
            return out

        def abort() -> bool:
            return self.estop.fired or self._stop_evt.is_set()

        moved = False
        try:
            self.group.check_health()
            start = read_current()
            if np.any(np.isnan(start)):
                raise RuntimeError("could not read all joint positions")
            if not clamp:
                # Capture-hold: disable the firmware clamp BEFORE POSITION so an out-of-range
                # pose isn't yanked to the limit. Restored in finally.
                self._widen_position_limits()
                time.sleep(0.05)             # let the limit writes land first
            for n in joints:
                self.client.set_mode(n, "POSITION")
            send(start)                      # seed hold at current (no jerk)
            time.sleep(0.05)
            moved = True
            ctrl_hz = 1.0 / self.contract.control_dt
            if not ramp_to_pose(start=start, goal=goal, send=send, duration_s=ramp,
                                rate_hz=min(ctrl_hz, 100.0), should_abort=abort):
                return
            t0 = time.monotonic()
            while not abort():
                if seconds is not None and (time.monotonic() - t0) >= seconds:
                    break
                self.group.check_health()
                send(goal)
                time.sleep(0.1)
        except Exception as exc:
            _log.error("manual session error: %s", exc)
            with self._lock:
                self._last_error = f"manual: {exc}"
            self.trigger_estop("manual-fault")
        finally:
            try:
                for n in joints:
                    self.client.set_mode(n, "IDLE")
            except Exception as exc:
                _log.warning("manual idle failed: %s", exc)
            if not clamp:
                self._restore_position_limits()   # re-arm the firmware clamp
            self._on_session_end()
            _log.info("manual session ended (moved=%s).", moved)

    def stop(self, *, wait: bool = True) -> None:
        """Graceful stop: signal the session to exit its loop (legs → IDLE), not an E-STOP."""
        self._stop_evt.set()
        t = self._session_thread
        if wait and t and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=5.0)

    def trigger_estop(self, reason: str = "web") -> None:
        """E-STOP: fire the priority (port 9002) stop and latch. Always safe to call."""
        self._stop_evt.set()
        self.estop.trigger(reason)
        with self._lock:
            self._armed = False
            self._state = SessionState.ESTOPPED

    def shutdown(self) -> None:
        """Process shutdown: stop any session and idle the legs (best effort)."""
        try:
            self.stop(wait=True)
        except Exception:
            pass
