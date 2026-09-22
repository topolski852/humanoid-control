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
# Generous relative to every loop that writes one (the leg policy at 25 Hz, arm teleop at
# _ARM_HZ): a finished ramp legitimately stops resending while the robot still holds that
# pose, and that hold IS the current command.
_TARGET_STALE_S = 5.0


class SessionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"   # sockets up, joints not configured/awake
    CONNECTED = "CONNECTED"         # joints configured + online, idle
    ARMED = "ARMED"                 # deadman session live, limbs at rest, waiting for trigger
    HOLDING = "HOLDING"             # ramping/holding default_pose (ZeroPolicy)
    RUNNING = "RUNNING"             # running a learned policy
    ESTOPPED = "ESTOPPED"           # E-STOP latched; reconnect to clear
    ERROR = "ERROR"                 # a session failed (fault/offline)


# Engaged/moving states (legs in POSITION, sending targets).
_MOTION_STATES = {SessionState.HOLDING, SessionState.RUNNING}
# A deadman session is live (worker thread running) — includes the damped-but-ready ARMED
# rest state. Used to gate connect/calibrate and to scope the controller-loss watchdog.
_ACTIVE_STATES = {SessionState.ARMED, SessionState.HOLDING, SessionState.RUNNING}


class _ArmRig:
    """One arm's half of a teleop session: its joints, its interface, its teleop, its gate.

    Both arms run inside ONE session. What is shared stays shared — E-STOP, arming, the
    heartbeat, the fault path, the finally-rest — and only what is genuinely per arm lives
    here. That split is the whole design: a fault on either arm still ends the session for
    both, because it disarms the machine, while a released trigger only rests its own arm.

    Built for every configured arm, so a one-arm bench produces exactly one rig and takes the
    same code path as a two-arm robot rather than a special case.
    """

    __slots__ = ("limb", "hand", "joints", "group", "teleop", "recorder",
                 "gate", "engaged", "engage_warn_at", "info")

    def __init__(self, limb, hand, joints, group, teleop, recorder, gate):
        self.limb = limb
        self.hand = hand
        self.joints = joints
        self.group = group
        self.teleop = teleop
        self.recorder = recorder
        self.gate = gate
        self.engaged = False
        self.engage_warn_at = 0.0
        self.info = None


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
        # Per-arm gates. Both arms run in one session, each activated by its own trigger, so
        # "the gate" is no longer a single boolean: the left arm can be driving while the
        # right rests. `_run_gate` stays as the whole-robot gate the Xbox path sets.
        self._run_gates: dict[str, threading.Event] = {
            limb: threading.Event() for limb in self._layout.arms}
        self._gate_lock = threading.Lock()
        # Live arm rigs while an arm session runs; read by telemetry, empty otherwise.
        self._rigs: list = []
        # Set once every configured arm's reach_bounds() has been computed — see
        # _warm_arm_chains for why chain EXISTENCE is not the same signal.
        self.chains_warm = threading.Event()
        self._warm_arm_chains()
        self._command_lock = threading.Lock()
        self._command = np.zeros(3, dtype=np.float32)
        self._arm_command = np.zeros(4, dtype=np.float32)   # raw sticks: lx, ly, ry, rx
        # 6-DOF-tracker command per arm: {limb: (displacement-since-clutch m, seq)}.
        self._arm_pose_commands: dict[str, tuple[np.ndarray, int]] = {}
        self._selected = {"kind": "hold", "checkpoint": None, "limb": None}
        # Which set of things the sticks drive, and how fast. Seeded from the layout so a
        # bench arm comes up in arm mode without anyone pressing Select.
        self._control_mode = "arm" if self._layout.arms else "leg"
        self._speed_mode = "normal"
        # REST STATE when a session is armed but not driving: trigger released, tracking
        # lost, ramp aborted, session torn down.
        #
        # DAMPING. A limp 5-DOF arm does not rest, it falls, and it rests far more often than
        # it drives.
        #
        # This required a daemon change to be possible at all: Actuator::tick() used to send
        # PDO2 only while ENABLED, so a DAMPING joint was fed by nothing and the firmware
        # watchdog expired after 1000 ms. Setting this to "damping" before that fix E-STOPped
        # every armed session within seconds on ERROR_WATCHDOG_TIMEOUT (0x0040). The daemon
        # now feeds DAMPING joints; verified on the arm with
        # scripts/verify_damping_feed.py — 60 s, five joints, no errors, no state drift.
        #
        # Deliberately NOT persisted: a restart returns to the safe choice rather than
        # silently inheriting whatever the last operator picked.
        self._rest_mode = "damping"

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
        # HOW an arm teleop session maps the operator onto the arm. Was derived implicitly at
        # session start from "is the Quest driving, and is there a calibration profile"; it is
        # now an explicit choice so the card can show it and the operator can force 'pose' even
        # when a profile exists. None means "follow availability" — the first valid method for
        # whatever source holds the token, which reproduces the old behaviour exactly.
        self._arm_method: str | None = None
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
            # Devices only — "web" is never offered as a control method (see input_devices).
            "input_devices": self.input_devices(),
            "ignored_writes": dict(self._ignored_writes),
            "control_clients": self._control_clients,
            "last_error": self._last_error,
            "all_calibrated": self.all_calibrated(),
            "quest": self._xr_status(),
            "gamepad": {**self._gamepad, "run_gate": self.any_run_gate(),
                        "input": self._gamepad_input},
            "control": {
                "mode": self._control_mode,
                "modes": self.available_control_modes(),
                "speed": self._speed_mode,
                "rest": self._rest_mode,
                "limb": self._selected.get("limb"),
                "arms": list(self._layout.arms),
                # Per-arm live state. `_arm_info` used to be written every tick and read by
                # nobody; with two arms running there is a real question to answer — which
                # arm is active, and why the other one refused to engage — so it is surfaced.
                "arm_state": self._arm_states(),
                "capabilities": list(self._layout.capabilities),
                "sessions": self.available_sessions(),
                "arm_method": self.arm_method,
                "arm_methods": self.available_arm_methods(),
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

    def _human_angles(self, hand: str | None = None):
        """The operator's own arm angles this tick, degrees, or None.

        Recorded alongside the robot's joints so a run log answers "did the robot match my
        arm" directly, instead of us inferring it from hand positions. ``hand`` picks which
        of the operator's arms — each arm's recording must carry the arm that drove it, or
        the two logs would both describe the same side.
        """
        try:
            q = self.quest
            if q is None:
                return None
            a = q._h(hand).human if hand else getattr(q, "_human", None)
            if a is None:
                return None
            return [round(float(np.degrees(v)), 2) for v in a.as_array()]
        except Exception:                                # noqa: BLE001
            return None

    # The per-tick `info` dict is NOT safe to ship as-is: `target` and `hand` are numpy
    # arrays, which the JSON encoder cannot serialise — it raises and the ENTIRE telemetry
    # snapshot 500s, taking the whole web UI down while teleop carries on working. That is
    # exactly what happened when this was first surfaced (2026-09-19 17:44): the arm drove
    # fine and the operator saw a blank page.
    #
    # So this whitelists, rather than sanitising whatever turns up. It is also the reason to
    # whitelist: the snapshot goes to every browser at 20 Hz, and `target`/`lead_deg`/
    # `joint_err_deg` are per-tick diagnostics that belong in the flight recorder (which
    # already stores them) and not in a 20 Hz broadcast.
    _ARM_INFO_KEYS = ("engage_blocked", "limit_deg", "worst_joint", "worst_deg",
                      "worst_joint_err_deg", "at_limit", "clipped", "hold", "commanding",
                      "frame")

    @staticmethod
    def _arm_info_public(info) -> dict | None:
        if not isinstance(info, dict):
            return None
        out = {}
        for k in ControlService._ARM_INFO_KEYS:
            if k not in info:
                continue
            v = info[k]
            if isinstance(v, (np.generic,)):        # numpy scalar -> python
                v = v.item()
            elif isinstance(v, np.ndarray):         # never ship an array from here
                continue
            out[k] = v
        return out or None

    def _arm_states(self) -> dict:
        """{limb: {active, engaged, info}} for every configured arm.

        Read from the live rigs when a session is running, and falls back to the gate alone
        otherwise, so the shape is the same whether or not an arm session exists.
        """
        out = {}
        for limb in self._layout.arms:
            gate = self._run_gates.get(limb)
            out[limb] = {"active": bool(gate is not None and gate.is_set()),
                         "engaged": False, "info": None}
        for rig in (self._rigs or ()):
            if rig.limb in out:
                out[rig.limb]["engaged"] = rig.engaged
                out[rig.limb]["info"] = self._arm_info_public(rig.info)
        return out

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

        held, worst = self._sample_hold(joints)

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

    # ── leg zeroing from the folded stance ───────────────────────────────────
    def teach_leg_stance(self, limb: str = "both", mirror: bool = True) -> dict:
        """Zero the legs from the folded, feet-together stance the operator is holding.

        The per-joint flow in ``calibration.py`` needs every joint driven by hand to BOTH of its
        hardstops, which is a long session to repeat after each power cycle. This takes one
        stance that already parks four joints per leg against a stop, solves every joint's
        ``position_offset`` in one pass, and verifies by reading back — a write that does not
        land is reported rather than assumed. See ``humanoid_control.leg_calibration`` for the
        stance and why both legs take the same targets unnegated.

        Offline joints are skipped and named: a dead ESC cannot be written, so its zero has to
        wait until it is back on the bus. Writes ``position_offset`` only; nothing is commanded
        to move.
        """
        from ..leg_calibration import (covers, is_declared as stance_declares,
                                       opposite_joint, solve_offset, stance_target)

        legs = self._layout.legs
        if not legs:
            raise ControlError("No leg is configured.", 400)
        if limb in ("both", "all"):
            want = list(legs)
        elif limb in legs:
            want = [limb]
        else:
            raise ControlError(
                f"{limb} is not configured — attached: {', '.join(legs) or 'none'}", 400)

        if self._state != SessionState.CONNECTED:
            raise ControlError(f"Connect first (state={self._state.value}).", 409)
        if self.is_motion_active():
            raise ControlError("A motion session is active — stop it before calibrating.", 409)

        covered = [n for l in want for n in self._layout.joints_of(l) if covers(n)]
        online = [n for n in covered if self._joint_online(n)]
        # A dead ESC takes no writes. Name it rather than failing the whole stance: losing one
        # joint is exactly the situation this flow exists to recover the other eleven from.
        results = [{"joint": n, "ok": False, "source": None,
                    "reason": "offline — ESC not on the bus"}
                   for n in covered if n not in set(online)]
        if not online:
            raise ControlError("Every leg joint is offline — nothing to calibrate.", 409)

        # Snapshot calibration BEFORE this run: a joint zeroed moments ago in this same pass is
        # not independent evidence, so it must not become a mirror source mid-run.
        with self._lock:
            pre_calibrated = dict(self._calibrated)

        # hip_roll and hip_yaw are the two the stance only DECLARES. If their twin is already
        # calibrated, online, and NOT itself part of this run, its live reading is the better
        # target — it carries whatever real asymmetry the stance has instead of assuming perfect
        # squareness. Excluding the run's own joints is what makes this well-defined: in a
        # both-legs run each twin would otherwise mirror the other, swapping two readings that
        # this very call is about to overwrite with the declared zero anyway.
        in_run = set(covered)
        mirror_src: dict[str, str] = {}
        if mirror:
            for n in online:
                if not stance_declares(n):
                    continue
                o = opposite_joint(n)
                if (o and o not in in_run and o in self._limits
                        and pre_calibrated.get(o) and self._joint_online(o)):
                    mirror_src[n] = o

        held, worst = self._sample_hold(sorted(set(online) | set(mirror_src.values())))

        targets: dict[str, tuple[float, str]] = {}
        for n in online:
            src = mirror_src.get(n)
            targets[n] = ((held[src], "mirrored") if src is not None
                          else stance_target(n, self._limits[n]))

        for n in online:
            want_rad, source = targets[n]
            old = self._read_offset(n)
            if old is None:
                results.append({"joint": n, "ok": False, "source": source,
                                "reason": "could not read offset"})
                continue
            new = solve_offset(held[n], want_rad, old)
            try:
                self.client.apply_config(n, {"position_offset": float(new)}, timeout=10.0)
            except Exception as exc:
                results.append({"joint": n, "ok": False, "source": source, "reason": str(exc)})
                continue
            results.append({
                "joint": n,
                "ok": True,
                "source": source,
                "mirrored_from": mirror_src.get(n),
                "was_deg": held[n] * 180.0 / np.pi,
                "target_deg": want_rad * 180.0 / np.pi,
                "shift_deg": (want_rad - held[n]) * 180.0 / np.pi,
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

        results.sort(key=lambda r: self._joints.index(r["joint"]))
        ok = all(r.get("ok") for r in results)
        with self._lock:
            for r in results:
                if r.get("ok"):
                    self._calibrated[r["joint"]] = True
            if all(self._calibrated.get(n, False) for n in self._joints):
                self._joints_dropped_since_cal = False
        done = sum(1 for r in results if r.get("ok"))
        _log.info("teach %s from folded stance: %d/%d joints (steady to %.2f deg)",
                  "+".join(want), done, len(results), worst)
        return {
            "limbs": want, "ok": ok,
            "zeroed": done, "total": len(results),
            "steady_deg": round(worst, 2),
            "shaky": worst > self._TEACH_STEADY_DEG,
            "joints": results,
        }

    def _sample_hold(self, joints: list[str]) -> tuple[dict[str, float], float]:
        """Average each joint's position over the sampling window.

        A single sample would bake in whatever jitter happened to land on that frame. The
        worst peak-to-peak spread (degrees) is returned alongside so a shaky hold is reported
        rather than silently accepted.
        """
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
        worst = max((max(v) - min(v)) for v in samples.values()) * 180.0 / np.pi
        return held, worst

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
        cleared, failed = 0, []
        for n in self._joints:
            try:
                self.client.clear_error(n)
                cleared += 1
            except Exception as exc:
                failed.append(n)
                _log.warning("clear_error %s failed: %s", n, exc)
        # VERIFY BY READBACK. A successful command is not a cleared fault: the daemon writes
        # the error register over SDO and the motor may never ACK. Wait past one slow-poll
        # cycle (10 Hz) so the cached state reflects the hardware, then check. Counting
        # commands that did not raise is what reported "cleared on 10/10 joints" while every
        # joint still read 0x0040 — see Actuator::clear_fault.
        time.sleep(0.5)
        still = []
        for n in self._joints:
            st = self.client.get_cached_joint_state(n)
            if st and int(st.get("error", 0) or 0):
                still.append(f"{n.replace('_joint','')}=0x{int(st['error']):04x}")
        if still or failed:
            _log.warning("clear_faults INCOMPLETE — still faulted: %s%s",
                         ", ".join(still) or "none",
                         f"; command failed on {', '.join(failed)}" if failed else "")
        with self._lock:
            self.estop = self._new_estop()     # release the latched E-STOP
            self._armed = False
            self._last_error = None
            self._state = SessionState.CONNECTED
        _log.info("faults cleared on %d/%d joints (%d still faulted after readback); "
                  "E-STOP released; state → CONNECTED.",
                  cleared, len(self._joints), len(still))
        return {"cleared": cleared, "still_faulted": still, "command_failed": failed}

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
        self._require_compatible_policy(checkpoint)
        policy = load_policy(checkpoint, num_actions=self.contract.num_joints)
        cmd = np.array(command if command is not None else [0.0, 0.0, 0.0], dtype=np.float32)
        self._start_session("policy", policy, command=cmd, ramp=ramp, seconds=seconds,
                            checkpoint=checkpoint)

    def _require_compatible_policy(self, checkpoint: str | None) -> None:
        """Refuse a bundle that was not trained against the gains this robot is running.

        The dropdown greys these out, but the UI is not the safety boundary — the API is.
        Switching policy switches the NETWORK only; gains, stand pose and timing all keep
        coming from configs/leg_policy_params.json. So a bundle trained at different gains
        runs against a robot it has never seen, and before this it failed (if at all) as an
        onnxruntime shape error on the FIRST step — after the ramp had already put the robot
        into the stand pose.

        A bundle with no contract alongside it is allowed: loose weight files and older
        exports are legitimately contract-less, and refusing everything unverifiable would
        break the fallback path. Unverified is reported as unverified, not treated as safe.
        """
        if not checkpoint:
            return
        from pathlib import Path as _Path
        from ..policy import bundle_issues
        cpath = _Path(checkpoint).parent / "leg_policy_contract.json"
        issues = bundle_issues(str(cpath) if cpath.is_file() else None, self.contract)
        if issues:
            raise ControlError(
                f"{_Path(checkpoint).parent.name!r} does not match this robot's contract: "
                + "; ".join(issues)
                + ". Re-export it or pick a compatible policy.", 409)

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
            self._clear_all_gates()
            self._session_deadman = None   # back to judging by the active input source
            if self.estop.fired:
                self._state = SessionState.ESTOPPED
            elif self._state in _ACTIVE_STATES:
                self._state = SessionState.CONNECTED

    # ── gamepad deadman session (hold-to-run) ────────────────────────────────
    #
    # The operational flow: connect → calibrate → arm_deadman() (limbs at rest, ARMED) →
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
        if kind == "policy":
            # Caught here rather than at engage: selection is a calm moment at the console,
            # engage is a hand on a trigger.
            self._require_compatible_policy(checkpoint)
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

    def set_run_gate(self, active: bool, *, source: str = "web",
                     limb: str | None = None) -> None:
        """Activation-trigger state from the active input source: True = held (engage/run),
        False = released (damp). Distinct from the heartbeat — a release damps; a controller
        loss E-STOPs. Ignored (and counted) from a source that does not hold the input token.

        ``limb`` scopes the gate to ONE arm, which is what the per-arm triggers use. None
        keeps the whole-robot semantics the Xbox path relies on: it already ORs both of its
        triggers into a single gate, so it sets every arm's gate together.
        """
        if not self._owns_input(source):
            return
        gates = ([self._run_gate_for(limb)] if limb is not None
                 else list(self._run_gates.values()) + [self._run_gate])
        for g in gates:
            g.set() if active else g.clear()

    def _warm_arm_chains(self) -> None:
        """Build and warm each arm's kinematic chain in the BACKGROUND, at startup.

        `ArmChain.reach_bounds()` grids the joint limits numerically: ~0.95s per chain, and
        GIL-heavy enough to starve the asyncio loop while it runs. Anywhere on the arming or
        ticking path that is a stall the safety watchdogs read as a dead link — it fired a
        spurious quest-timeout E-STOP during arming before this moved here. Paid once, up
        front, where nothing is waiting on it and no session exists to interrupt.

        ``chains_warm`` is SET WHEN THE WORK IS DONE, not when the chains exist. Constructing
        a chain takes ~0.5 ms and populates ``_chain_cache``; the ~950 ms per arm happens
        afterwards inside reach_bounds(). So "is `_chain_cache` populated" answers a
        different question from "has the warm-up finished", and anything that waits on the
        former is still waiting on a cold chain. Callers that need the real answer wait on
        this event.

        Best effort: a failure here costs the first tick its second back, nothing more — the
        event is set either way, so a broken chain cannot leave a waiter hanging.
        """
        limbs = list(self._layout.arms)
        if not limbs:
            self.chains_warm.set()      # nothing to warm; already as warm as it gets
            return

        def warm():
            try:
                for lb in limbs:
                    try:
                        self.arm_chain(lb).reach_bounds()
                    except Exception as exc:             # noqa: BLE001
                        _log.debug("arm chain warm-up failed for %s (%s)", lb, exc)
            finally:
                self.chains_warm.set()

        threading.Thread(target=warm, name="arm-chain-warm", daemon=True).start()

    def _run_gate_for(self, limb: str) -> threading.Event:
        """One arm's gate, created on demand so a limb that is configured later still gets
        one rather than silently sharing another arm's."""
        with self._gate_lock:
            g = self._run_gates.get(limb)
            if g is None:
                g = self._run_gates[limb] = threading.Event()
            return g

    def _clear_all_gates(self) -> None:
        """Drop every arm's gate. Used at session start and end, where "the session is
        over" must not leave one arm still believing its trigger is held."""
        self._run_gate.clear()
        for g in self._run_gates.values():
            g.clear()

    def any_run_gate(self) -> bool:
        """True while ANY arm is active. This is what the session-level ladders mean by "the
        gate": the session is running for as long as the operator is driving either arm."""
        return self._run_gate.is_set() or any(g.is_set() for g in self._run_gates.values())

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

    def input_devices(self) -> list[str]:
        """Input sources that are actual DEVICES someone drives the robot with.

        Deliberately excludes "web". The browser is not a control method — it is the console you
        calibrate and supervise from, and it holds the token by default only so that a machine
        with no controller attached still has exactly one owner. Listing it alongside Xbox and
        Quest invited the reading that a policy is a third way of driving, which it is not: the
        policy always runs on the legs, and Xbox or Quest supplies its velocity command. With
        neither connected the policy simply stands still.
        """
        return [s for s in self.available_input_sources() if s != "web"]

    # ── arm control method ───────────────────────────────────────────────────
    #
    # WHAT the arm does with the operator's motion, as distinct from WHICH DEVICE that motion
    # comes from (that is the input source above). Only mappings that actually exist are
    # offered; each is valid for exactly one source, so the list changes when the token moves.

    ARM_METHODS = {
        "quest_mirror": {
            "label": "Quest mirror",
            "source": "quest",
            "blurb": "Your whole arm drives the robot's, joint for joint. Needs a calibration profile.",
        },
        "quest_pose": {
            "label": "Quest pose",
            "source": "quest",
            "blurb": "The controller's position drives the hand; the arm solves for it.",
        },
        "xbox_cartesian": {
            "label": "Xbox cartesian",
            "source": "xbox",
            "blurb": "Sticks drive the hand along the robot's X/Y/Z axes.",
        },
    }

    def _quest_has_profile(self) -> bool:
        return self.quest is not None and getattr(self.quest, "_profile", None) is not None

    def available_arm_methods(self) -> list[dict]:
        """Arm mappings valid right now. Unavailable ones are returned WITH a reason rather than
        omitted, so "why can I not pick mirror" is answerable from the card instead of being an
        invisible property of the calibration state."""
        if not self._layout.can("arm_teleop"):
            return []
        src = self._input_source
        out = []
        for mid, meta in self.ARM_METHODS.items():
            reason = None
            if meta["source"] != src:
                reason = f"needs the {meta['source']} input source"
            elif mid == "quest_mirror" and not self._quest_has_profile():
                reason = "no arm calibration profile — run the arm calibration"
            out.append({"id": mid, "label": meta["label"], "blurb": meta["blurb"],
                        "available": reason is None, "reason": reason})
        return out

    @property
    def arm_method(self) -> str | None:
        """The mapping a session would use right now. An explicit choice wins while it stays
        valid; otherwise the first available one, which is what the old implicit derivation
        picked (mirror when the Quest has a profile, else pose, else cartesian)."""
        usable = [m["id"] for m in self.available_arm_methods() if m["available"]]
        if not usable:
            return None
        if self._arm_method in usable:
            return self._arm_method
        return usable[0]

    def set_arm_method(self, method: str) -> None:
        """Refused mid-session for the same reason the input source and control mode are:
        changing what the operator's motion MEANS while the arm is moving is exactly the
        transition nobody can supervise."""
        # UNKNOWN and UNAVAILABLE are different failures and must not collapse into one.
        # available_arm_methods() is empty on a layout with no arms, so checking membership in
        # IT first reported a perfectly valid id as "unknown" and sent a 400 where the honest
        # answer is "this robot has no arms" — a misleading message for a correct request.
        if method not in self.ARM_METHODS:
            raise ControlError(
                f"unknown arm method {method!r} "
                f"(known: {', '.join(self.ARM_METHODS)}).", 400)
        methods = {m["id"]: m for m in self.available_arm_methods()}
        if not methods:
            raise ControlError(
                f"no arm control available — layout is '{self._layout.describe()}'.", 409)
        if not methods[method]["available"]:
            raise ControlError(f"{methods[method]['label']} unavailable — "
                               f"{methods[method]['reason']}.", 409)
        if self._state in _ACTIVE_STATES:
            raise ControlError("Disarm before switching arm control method.", 409)
        self._arm_method = method
        _log.info("arm method: %s", method)

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

    @property
    def rest_mode(self) -> str:
        return self._rest_mode

    def set_rest_mode(self, mode: str) -> None:
        """What the arm does when armed but not driving.

        DAMPING is regenerative braking: the motor resists motion but holds no target. It is
        sustainable only because the daemon feeds the firmware watchdog while a joint is in
        it (Actuator::tick, DAMPING branch). It is NOT self-sustaining — an unfed DAMPING
        joint faults ERROR_WATCHDOG_TIMEOUT in about a second.

        IDLE is zero torque. A 5-DOF arm in IDLE does not rest, it FALLS, at whatever speed
        gravity and its own inertia allow. That is fine for a leg sitting on a stand and
        actively bad for an arm held out in front of a robot.

        This is settable rather than fixed because IDLE is genuinely wanted sometimes — you
        cannot back-drive a damped joint comfortably, so hand-positioning the arm, checking
        free play, or re-zeroing all want the motors limp. It is a deliberate choice made at
        the console, not the default that a released trigger silently drops you into.

        Takes effect at the next rest transition; it does not disturb a joint that is already
        resting, and never touches a driving one.
        """
        if mode not in ("damping", "idle"):
            raise ControlError("rest mode must be 'damping' or 'idle'.", 400)
        if mode != self._rest_mode:
            _log.info("rest mode: %s -> %s", self._rest_mode, mode)
        self._rest_mode = mode

    def set_joint_mode(self, mode: str, *, limb: str | None = None) -> dict:
        """Command joints into IDLE or DAMPING **right now**.

        Distinct from set_rest_mode, and the distinction is the whole point of having both.
        set_rest_mode decides what the NEXT release does and deliberately leaves a resting
        joint alone; this re-commands the joint you are looking at. Without it an operator who
        wants to reposition the arm by hand has no way to get there — the arm rests in
        DAMPING, damping is hard to back-drive, and changing the default does nothing until
        something else triggers a rest transition.

        Refused while a session is ENGAGED (HOLDING/RUNNING). Dropping a driving joint to
        IDLE mid-motion is a fall, and to DAMPING is a fight with the position controller.
        ARMED-but-resting is allowed, because that is exactly when this is wanted.

        Not sticky: the next rest transition re-applies rest_mode. That is deliberate —
        going limp is for a specific task in front of you, not a mode you leave the robot in
        and forget.
        """
        if mode not in ("damping", "idle"):
            raise ControlError("mode must be 'damping' or 'idle'.", 400)
        if not self.client.is_running():
            raise ControlError("The daemon is not running.", 503)
        if self.is_motion_active():
            raise ControlError(
                "Release the trigger before changing motor mode — the arm is driving.", 409)

        limb = limb or self._selected.get("limb")
        joints = list(self._layout.joints_of(limb)) if limb else list(self._joints)
        if not joints:
            raise ControlError("No joints to command.", 409)

        group = JointGroupInterface(self.client, joints)
        if mode == "idle":
            group.idle()
        else:
            group.damp()
        _log.info("joint mode: %s -> %s (%d joints)", limb or "all", mode.upper(), len(joints))
        return {"mode": mode, "limb": limb, "joints": len(joints)}

    def _rest(self, group) -> None:
        """Put a group into the configured rest state.

        One place, so a released trigger, a lost tracker, an aborted ramp and a torn-down
        session cannot drift apart. They were four separate `group.idle()` calls, and every
        one of them carried the same wrong comment about the watchdog.
        """
        if self._rest_mode == "idle":
            group.idle()
        else:
            group.damp()

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

    def set_arm_pose_command(self, delta_m, seq: int = 0, *, source: str = "web",
                             limb: str | None = None) -> None:
        """Hand DISPLACEMENT since the clutch anchor, metres, robot frame — from a 6-DOF
        tracker. Kept separate from `_arm_command` rather than overloading it: a stick quad is
        a velocity and this is a position offset, and a stale value of one interpreted as the
        other is precisely the confusion worth designing out.

        ``limb`` says which arm the displacement belongs to; each controller clutches
        independently, so they cannot share one slot."""
        if not self._owns_input(source):
            return
        target = limb or self.selected_limb()
        if target is None:
            return
        with self._command_lock:
            self._arm_pose_commands[target] = (
                np.asarray(delta_m, dtype=np.float32).reshape(3).copy(), int(seq))

    def arm_pose_command(self, limb: str | None = None):
        with self._command_lock:
            return self._arm_pose_commands.get(limb or self.selected_limb())

    def arm_limbs(self) -> tuple:
        """Every arm configured on this robot, in layout order."""
        return tuple(self._layout.arms)

    def arm_chain(self, limb: str | None = None):
        """Kinematic chain for one arm. Cached PER LIMB — building it parses the vendored
        URDF model, and with both arms retargeting on every XR frame a single-entry cache
        would thrash between them and rebuild the chain twice per frame."""
        limb = limb or self.selected_limb()
        if limb is None:
            raise ControlError("no arm configured", 409)
        cache = getattr(self, "_chain_cache", None)
        if cache is None:
            cache = self._chain_cache = {}
        if limb not in cache:
            from ..arm_kinematics import ArmChain
            cache[limb] = ArmChain(list(self._layout.joints_of(limb)))
        return cache[limb]

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
        """Enter the ARMED deadman session: limbs → the configured rest state, and spawn the
        trigger-driven worker.

        Not "legs → DAMPING" as this said for a long time: it rests whichever joints the
        layout enables (or just the driven arm for kind='arm'), and whether that is DAMPING
        or IDLE is the operator's choice on the Rest state card.
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
            self._clear_all_gates()
            # This session is judged by the source that armed it for its whole life, even if
            # the token were somehow changed underneath it.
            self._session_deadman = self._input_source
            self._state = SessionState.ARMED
            t = threading.Thread(target=self._deadman_worker, args=(runner, sel_kind),
                                 name=f"deadman-{sel_kind}", daemon=True)
            self._session_thread = t
            t.start()
        if sel_kind == "arm":
            # Arm sessions: say what the trigger IS. Calling it a deadman told operators that
            # letting go was the emergency stop, when releasing it rests the arm in DAMPING —
            # which holds the arm up and is the normal way to park it. E-STOP is the stop.
            _log.info("ARMED arm session (ramp=%.1fs) — resting in %s. Hold a controller's "
                      "trigger to ACTIVATE that arm; release it and that arm rests (it holds "
                      "itself, it does not drop). E-STOP and disarm apply to both arms.",
                      ramp, self._rest_mode.upper())
        else:
            _log.info("ARMED deadman session (kind=%s, ramp=%.1fs) — resting in %s, "
                      "hold a trigger to run.", sel_kind, ramp, self._rest_mode.upper())

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
        """Leave the deadman session: stop the worker (limbs → the configured rest state,
        DAMPING by default — not IDLE, as this used to claim), back to CONNECTED."""
        self.stop(wait=True)

    def _deadman_worker(self, runner: PolicyRunner, kind: str) -> None:
        """Persistent trigger-driven loop. Released trigger → the configured rest state
        (DAMPING by default); held trigger → engage
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
        rigs: list[_ArmRig] = []
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
            # The mapping is now an EXPLICIT choice (Control method card), resolved here once
            # at session start. `arm_method` falls back to the first available method when
            # nothing was chosen, which reproduces the old implicit derivation exactly:
            # mirror when the Quest has a profile, else pose, else cartesian.
            method = self.arm_method
            if method == "quest_mirror":
                tuning = TeleopTuning(frame="mirror")
            elif method == "quest_pose":
                tuning = TeleopTuning(frame="pose")
            else:
                tuning = TeleopTuning()
            if self._session_deadman == "quest" and method != "quest_mirror":
                _log.warning("arm teleop: Quest is driving in '%s' mode, not mirror — run the "
                             "arm calibration to mirror your whole arm.", method or "cartesian")
            # SHARED, PRE-WARMED chains (see _warm_arm_chains). Building a chain is free;
            # its reach_bounds() is not — ~1.0s of numerical work per chain — and doing that
            # here put >1s of GIL-heavy work on the arming path. The Quest heartbeat watchdog
            # trips at 1.0s, so arming fired a SPURIOUS E-STOP before the second arm's rig
            # was even built (observed 2026-09-19 17:45: "no frame for 1.19s"). ArmChain only
            # assigns in __init__, so one instance per limb is safe to share across threads.
            teleop = ArmTeleop(self.arm_chain(limb), tuning=tuning)
            # Which branch the per-tick loop takes. Derived from tuning.frame rather than from
            # `method`, so there is ONE source for what the session is doing — the same value
            # the teleop itself switches on. Both names were referenced in the loop below
            # without ever being assigned, so any armed arm session raised NameError on its
            # first tick and surfaced as "deadman: name 'mirror_mode' is not defined".
            mirror_mode = (tuning.frame == "mirror")
            pose_mode = (tuning.frame == "pose")
            _log.info("arm teleop: driving %s (%d joints) in '%s' frame",
                      limb, len(arm_joints), tuning.frame)
            # Flight recorder for the whole armed session, engaged or not. Always on: the arm
            # has no policy to fall back on, and "it did not move how I expected" is only
            # answerable from the numbers afterwards.
            rec_dir = (os.environ.get("HUMANOID_RECORD_DIR")
                       or str(REPO_ROOT / "_arm_recording" / "runs"))

            # One rig per CONFIGURED arm. Driving both is the normal case; a bench with one
            # arm simply produces one rig, so there is no single-arm branch to keep in step.
            # The selected limb leads so its rig is index 0 and the existing single-arm names
            # (`group`, `teleop`, `recorder`) keep pointing at the arm they always did.
            limbs = ([limb] + [a for a in self._layout.arms if a != limb]) if limb \
                else list(self._layout.arms)
            for i, lb in enumerate(limbs):
                lb_joints = arm_joints if i == 0 else list(self._layout.joints_of(lb))
                lb_group = group if i == 0 else JointGroupInterface(self.client, lb_joints)
                lb_teleop = teleop if i == 0 else ArmTeleop(self.arm_chain(lb), tuning=tuning)
                lb_rec = None
                try:
                    lb_rec = ArmRunRecorder(rec_dir, lb, lb_joints, lb_teleop.tuning)
                    _log.info("arm run log (%s): %s", lb, lb_rec.path)
                except Exception as exc:
                    _log.warning("arm run log unavailable for %s (%s) — continuing without it.",
                                 lb, exc)
                if i == 0:
                    recorder = lb_rec
                rigs.append(_ArmRig(lb, "right" if lb.startswith("right") else "left",
                                    lb_joints, lb_group, lb_teleop, lb_rec,
                                    self._run_gate_for(lb)))
            self._rigs = rigs
            _log.info("arm teleop: %d arm(s) active — %s. Each trigger ACTIVATES its own arm; "
                      "releasing it rests that arm in DAMPING. E-STOP and disarm are global.",
                      len(rigs), ", ".join(r.limb for r in rigs))

        try:
            # ARMED rest. DAMPING by default: the arm holds itself instead of dropping.
            #
            # Two wrong comments stood here before this one, in both directions, and the
            # honest version is: the firmware watchdog DOES run in DAMPING, and the daemon
            # feeds it (Actuator::tick). Neither half is optional. The firmware docs claimed
            # otherwise and that cost this repo the same bug twice; docs/HANDOFF.md §2
            # ("Watchdog → DAMPING") now records the correction, and the hardware is the
            # authority over both.
            self._rest(group)
            if manual:
                # Disable the firmware position clamp for the whole armed session so a trigger-
                # engage holds any hand-set pose (even outside soft limits). Restored in finally.
                self._widen_position_limits()
            next_tick = time.monotonic()
            while not self.estop.fired and not self._stop_evt.is_set():
                # ── arm teleop: one pass per arm, each on its own activation gate ──
                #
                # Split out from the single-gate path below because "the trigger" is no
                # longer one boolean: the left arm can be driving while the right rests, and
                # each has its own engage gate, clutch and recorder. The SESSION-level
                # concerns stay out here where they were — E-STOP and the stop event end the
                # loop for both, and a fault inside any rig propagates to the shared handler
                # and disarms the machine.
                if arm_mode:
                    any_engaged = False
                    for rig in rigs:
                        any_engaged |= self._rig_tick(rig, dt, mirror_mode, pose_mode)
                    # RUNNING/HOLDING while ANY arm is driving; back to ARMED once every arm
                    # has rested. Reported per arm in telemetry.
                    with self._lock:
                        if any_engaged and self._state != engaged_state:
                            self._state = engaged_state
                        elif not any_engaged and self._state == engaged_state:
                            self._state = SessionState.ARMED
                    if any_engaged:
                        next_tick += dt
                        sleep = next_tick - time.monotonic()
                        if sleep > 0:
                            time.sleep(sleep)
                        else:
                            next_tick = time.monotonic()
                    else:
                        time.sleep(0.02)          # idle damped, waiting for a trigger
                        next_tick = time.monotonic()
                    continue

                if self._run_gate.is_set():
                    if not engaged:
                        with self._lock:
                            self._state = engaged_state
                        if manual:
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
                                    self._rest(group)         # released mid-ramp → rest
                                    with self._lock:
                                        self._state = SessionState.ARMED
                                continue
                        engaged = True
                        next_tick = time.monotonic()
                    group.check_health()                  # raises on fault → finally IDLEs
                    if not manual:
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
                        self._rest(group)                 # trigger released → rest
                        engaged = False
                        with self._lock:
                            self._state = SessionState.ARMED
                    else:
                        time.sleep(0.02)                  # idle damped, waiting for trigger
        except Exception as exc:
            _log.error("deadman session error: %s", exc)
            with self._lock:
                self._last_error = f"deadman: {exc}"
            self.trigger_estop("deadman-fault")
        finally:
            # Leaving the session. Still the configured rest state: an arm that drops the
            # moment you disarm is the same hazard as one that drops on a released trigger.
            # Use the rest-mode control to go limp on purpose.
            #
            # EVERY rig is rested, each in its own try: if the first arm's rest raises (a
            # dead bus, a faulted ESC) the second must still be told to hold itself, and the
            # whole point of this block is that it runs when something has already gone
            # wrong. The same goes for closing the recorders.
            for tgt in ([r.group for r in rigs] if rigs else [group]):
                try:
                    self._rest(tgt)
                except Exception as exc:
                    _log.warning("rest after deadman session failed: %s", exc)
            if manual:
                self._restore_position_limits()   # re-arm the firmware clamp
            for rec in ([r.recorder for r in rigs] if rigs else [recorder]):
                if rec is None:
                    continue
                try:
                    rec.close()
                    _log.info("arm run log written: %s", rec.path)
                except Exception as exc:          # noqa: BLE001
                    _log.warning("closing an arm run log failed: %s", exc)
            self._rigs = []
            self._on_session_end()
            _log.info("deadman session ended.")

    def _rig_tick(self, rig: "_ArmRig", dt: float, mirror_mode: bool,
                  pose_mode: bool) -> bool:
        """One arm, one tick. Returns True if that arm is engaged and driving.

        This is the old single-arm body, scoped to a rig. Everything it touches belongs to
        one arm — its gate, its interface, its teleop, its recorder — so two of these run
        side by side without sharing anything that a trigger can change. It deliberately does
        NOT catch exceptions: a fault must reach the session handler and take the whole
        machine down, which is the agreed behaviour for a fault on either arm.
        """
        if not rig.gate.is_set():
            if rig.engaged:
                self._rest(rig.group)             # trigger released → rest THIS arm
                rig.engaged = False
            elif rig.recorder is not None:
                try:
                    with self._command_lock:
                        cmd = self._arm_command.copy()
                    q_now, v_now = rig.group.read_states(require_online=False)
                    rig.recorder.record(engaged=False, run_gate=False, sticks=cmd,
                                        joint_pos=q_now, joint_vel=v_now,
                                        speed_mode=self._speed_mode,
                                        xr=self._xr_status(),
                                        human=self._human_angles(rig.hand))
                except Exception:
                    pass          # a log must never break the session
            return False

        if not rig.engaged:
            # ENGAGE GATE (mirror only), BEFORE any state change or ESC write. Mirror
            # commands an ABSOLUTE pose, so engaging with the arm parked far from where the
            # operator's body says it belongs slews it across that whole gap at full rate —
            # measured 105 deg on shoulder_pitch. Make the operator close the gap instead.
            # Refusing here leaves the arm exactly as it was: still resting, nothing
            # commanded, no mode write. Per arm, so a refused right arm does not interrupt a
            # left arm that is already driving. The pose and cartesian frames are NOT gated —
            # they seed from the current position and command relative motion, so they have
            # no gap to jump.
            if mirror_mode:
                tgts_chk, _ = self.quest.mirror_command(rig.hand)
                if tgts_chk is not None:
                    q_chk = rig.teleop.chain.device_to_urdf(rig.group.read_states()[0])
                    err = np.degrees(np.asarray(tgts_chk, dtype=float) - q_chk)
                    lim = rig.teleop.tuning.engage_max_err_deg
                    if np.abs(err).max() > lim:
                        w = int(np.argmax(np.abs(err)))
                        rig.info = {
                            "engage_blocked": True, "limit_deg": lim,
                            "worst_joint": rig.joints[w].replace("_joint", ""),
                            "worst_deg": round(float(err[w]), 1),
                        }
                        if time.monotonic() - rig.engage_warn_at > 2.0:
                            rig.engage_warn_at = time.monotonic()
                            _log.warning(
                                "%s engage REFUSED — match the robot's pose first "
                                "(limit %.0f deg): %s", rig.limb, lim,
                                ", ".join(f"{n.replace('_joint','')} {e:+.0f}"
                                          for n, e in zip(rig.joints, err)
                                          if abs(e) > lim))
                        return False
            # ARM ENGAGE: enable POSITION (the daemon seeds the firmware target from the live
            # measured position, so this is jerk-free) and seed the teleop target at the
            # hand's ACTUAL position. Seeding every engage is what stops the arm jumping to
            # wherever the target was left.
            rig.group.enable_position()
            q_now, _ = rig.group.read_states()
            # read_states is DEVICE frame; the chain works in URDF. Convert at the boundary
            # (see ArmChain.device_to_urdf) — without this the teleop seeds its target from a
            # mirrored pose on any joint whose frame sign is -1, and the arm jumps on engage.
            rig.teleop.reset(rig.teleop.chain.device_to_urdf(q_now))
            rig.engaged = True

        rig.group.check_health()                  # raises on fault → session handler IDLEs
        with self._command_lock:
            cmd = self._arm_command.copy()
            pose_cmd = self._arm_pose_commands.get(rig.limb)
        q_now, v_now = rig.group.read_states()
        # DEVICE -> URDF for everything the chain touches. q_target comes back in URDF and is
        # converted straight back before it reaches a motor.
        q_now_urdf = rig.teleop.chain.device_to_urdf(q_now)
        creep = (self._speed_mode == "creep")
        if mirror_mode:
            # Whole-arm mirroring. `hold` freezes the command where it is when body tracking
            # drops — an emulated joint is the headset guessing where the operator's elbow
            # is, and a guess must not drive a motor. Freezing beats dropping to IDLE
            # mid-motion on a bolted-down arm; if it persists this releases below.
            tgts, hold = self.quest.mirror_command(rig.hand)
            q_target, info = rig.teleop.step_mirror(q_now_urdf, tgts, dt, creep=creep,
                                                    hold=hold)
            # Defence in depth. The Quest source latches this release itself (see
            # QuestSource._drive_arm): it has to, because the gate is re-asserted there every
            # frame while the trigger is held, so a clear from this worker alone would be
            # overwritten ~16 ms later and the arm would oscillate IDLE<->POSITION at tick
            # rate. This clear is the motor-side backstop for that, not the mechanism.
            if self.quest.body_lost_too_long(rig.hand):
                _log.warning("arm teleop: %s body tracking lost — releasing to rest.",
                             rig.hand)
                rig.gate.clear()
        elif pose_mode:
            # No fresh sample yet (or the source dropped it) means HOLD, not "reuse the last
            # displacement" — a stale offset would keep driving the arm after the operator's
            # link went quiet.
            delta = pose_cmd[0] if pose_cmd is not None else np.zeros(3)
            q_target, info = rig.teleop.step_pose(q_now_urdf, delta, dt, creep=creep)
        else:
            q_target, info = rig.teleop.step(q_now_urdf, cmd, dt, creep=creep)
        # URDF -> DEVICE on the way out. This is the last step before a motor moves, so it
        # must mirror the conversion on the way in exactly.
        q_target_dev = rig.teleop.chain.urdf_to_device(q_target)
        rig.group.send_targets(q_target_dev)
        rig.info = info
        if rig.recorder is not None:
            rig.recorder.record(engaged=True, run_gate=True, sticks=cmd,
                                joint_pos=q_now, joint_vel=v_now,
                                joint_target=q_target_dev, info=info,
                                speed_mode=self._speed_mode,
                                xr=self._xr_status(),
                                human=self._human_angles(rig.hand))
        return True

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
        # CAN be scoped to the joints being commanded — dropping the firmware clamp on a limb
        # we are not driving would remove a safety net for no benefit. Both current callers
        # (manual capture-hold, and the manual deadman session) command every configured joint,
        # so both take the default and widen the whole layout. _restore_position_limits always
        # restores the whole layout, so a narrower widen must not outlive its session.
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
        """Graceful stop: signal the session to exit its loop (the worker rests the limbs on
        its way out), not an E-STOP."""
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
