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
from ..config import LegPolicyContract
from ..interface import LegInterface
from ..policy import ZeroPolicy, load_policy
from ..runner import PolicyRunner
from ..safety import EstopController, ramp_to_pose
from ..base_state import TelemetryBaseState
from ..daemon import DaemonClient

_log = logging.getLogger(__name__)

# Deadman: motion requires a control heartbeat at least this fresh.
_DEADMAN_TIMEOUT_S = 1.0


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
    def __init__(self, client: DaemonClient, contract: LegPolicyContract, *, config_present: bool):
        self.client = client
        self.contract = contract
        self.config_present = config_present
        self.legs = LegInterface(client, contract)

        self._state = SessionState.DISCONNECTED
        self._armed = False
        self._last_error: str | None = None

        self._lock = threading.Lock()             # guards state transitions / session start
        self._session_thread: threading.Thread | None = None
        self._stop_evt = threading.Event()        # cooperative graceful-stop signal

        # E-STOP controller (no SIGINT/keyboard — the server has no TTY and uvicorn owns SIGINT).
        # Rebuilt on every connect so a prior latched E-STOP is cleared.
        self.estop = self._new_estop()

        # Deadman: how many /ws/control clients are attached and the last heartbeat time.
        self._control_clients = 0
        self._last_heartbeat = 0.0

        # Gamepad deadman session: the run-gate (set while a trigger is held), the live walk
        # command (vx, vy, wz) written by the gamepad sticks, and the selected session to run
        # when the trigger engages. The run-gate is distinct from the heartbeat: heartbeat =
        # "controller alive" (loss → E-STOP); run-gate = "trigger held" (release → DAMP).
        self._run_gate = threading.Event()
        self._command_lock = threading.Lock()
        self._command = np.zeros(3, dtype=np.float32)
        self._selected = {"kind": "hold", "checkpoint": None}

        # Gamepad presence for the UI (updated by GamepadDeadman). "enabled" reflects whether the
        # gamepad deadman thread is running at all (HUMANOID_GAMEPAD_ENABLE).
        self._gamepad = {
            "enabled": bool(os.environ.get("HUMANOID_GAMEPAD_ENABLE")),
            "connected": False,
            "name": None,
        }
        self._last_autowake = 0.0   # rate-limits ESC-reset auto-recovery

        # Per-joint position_offset calibration. Reset to uncalibrated on every connect
        # (a connect follows every power-up, and the encoder zero is lost on power-down).
        self._joints = list(contract.joint_order)
        self._calibrated: dict[str, bool] = {n: False for n in self._joints}
        self._cal_captures: dict[str, dict] = {n: {"lower": None, "upper": None} for n in self._joints}

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

    def telemetry_snapshot(self) -> dict:
        """Non-blocking snapshot from the telemetry cache (no UDP round-trip)."""
        joints = []
        for i, name in enumerate(self.contract.joint_order):
            limit = {"min": float(self.contract.pos_limit_lower[i]),
                     "max": float(self.contract.pos_limit_upper[i])}
            st = self.client.get_cached_joint_state(name)
            if st is None:
                joints.append({"index": i, "name": name, "online": False,
                               "calibrated": self._calibrated.get(name, False), "limit": limit})
                continue
            state = st.get("state") or st.get("joint_state")
            joints.append({
                "index": i,
                "name": name,
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
            })
        try:
            buses = self.client.get_interface_stats()
        except Exception:
            buses = []
        return {
            "daemon_alive": self.client.is_running(),
            "config_present": self.config_present,
            "state": self._state.value,
            "armed": self._armed,
            "estop": self.estop.fired,
            "deadman_ok": self.deadman_ok(),
            "control_clients": self._control_clients,
            "last_error": self._last_error,
            "all_calibrated": self.all_calibrated(),
            "gamepad": {**self._gamepad, "run_gate": self._run_gate.is_set()},
            "joints": joints,
            "buses": buses,
            # IMU base block the policy actually consumes (quaternion / angular_velocity /
            # projected_gravity), or None when the daemon reports no fresh IMU (base: null).
            "base": self.client.latest_base(),
        }

    # ── deadman ──────────────────────────────────────────────────────────────
    def control_client_connected(self) -> None:
        with self._lock:
            self._control_clients += 1
        self.mark_heartbeat()

    def control_client_disconnected(self) -> None:
        with self._lock:
            self._control_clients = max(0, self._control_clients - 1)
        # Losing the controller (deadman) while a deadman session is live — even damped-and-
        # armed — is an immediate E-STOP: we can no longer trust a release vs a dropout.
        if self._control_clients == 0 and self._state in _ACTIVE_STATES:
            self.trigger_estop("deadman-disconnect")

    def set_gamepad_connected(self, name: str) -> None:
        with self._lock:
            self._gamepad = {"enabled": True, "connected": True, "name": name}

    def set_gamepad_disconnected(self) -> None:
        with self._lock:
            self._gamepad = {**self._gamepad, "connected": False, "name": None}

    def mark_heartbeat(self) -> None:
        self._last_heartbeat = time.monotonic()

    def deadman_ok(self) -> bool:
        return (
            self._control_clients > 0
            and (time.monotonic() - self._last_heartbeat) < _DEADMAN_TIMEOUT_S
        )

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
        self.legs.check_health()   # raises if any leg offline/faulted (errors now cleared)
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
            # Fresh power-up ⇒ every joint's zero is stale ⇒ mark all uncalibrated.
            self._calibrated = {n: False for n in self._joints}
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
            self.legs.disable()
        except Exception as exc:
            _log.warning("disable on disconnect failed: %s", exc)
        with self._lock:
            self._armed = False
            self._state = SessionState.DISCONNECTED

    # ── arming ("I am present / robot is supported") ─────────────────────────
    def arm(self) -> None:
        with self._lock:
            if self._state != SessionState.CONNECTED:
                raise ControlError(
                    f"Cannot arm from {self._state.value}; connect first.", 409)
            if self.estop.fired:
                raise ControlError("E-STOP is latched — reconnect to clear.", 409)
            uncal = [n for n in self._joints if not self._calibrated[n]]
            if uncal:
                raise ControlError(
                    f"Calibrate all joints before arming — {len(uncal)} uncalibrated "
                    f"(e.g. {uncal[0].replace('_joint','')}).", 409)
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
        i = self.contract.index_of(joint)
        min_rad = float(self.contract.pos_limit_lower[i])
        max_rad = float(self.contract.pos_limit_upper[i])
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
        for i, name in enumerate(self.contract.joint_order):
            st = self.client.get_cached_joint_state(name)
            state = (st or {}).get("state") or (st or {}).get("joint_state")
            pos = (st or {}).get("position")
            lo = float(self.contract.pos_limit_lower[i])
            hi = float(self.contract.pos_limit_upper[i])
            if st is None or state in (None, "OFFLINE") or pos is None:
                bad.append({"joint": name, "reason": "offline", "position": None, "min": lo, "max": hi})
            elif pos < lo - self._CAL_LIMIT_TOL or pos > hi + self._CAL_LIMIT_TOL:
                bad.append({"joint": name, "reason": "out_of_limits", "position": pos, "min": lo, "max": hi})
        return bad

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
        self._start_session("hold", ZeroPolicy(self.contract.num_joints),
                            command=None, ramp=ramp, seconds=seconds)

    def start_policy(self, *, checkpoint: str, command=None,
                     ramp: float = 5.0, seconds: float | None = None) -> None:
        policy = load_policy(checkpoint, num_actions=self.contract.num_joints)
        cmd = np.array(command if command is not None else [0.0, 0.0, 0.0], dtype=np.float32)
        self._start_session("policy", policy, command=cmd, ramp=ramp, seconds=seconds,
                            checkpoint=checkpoint)

    def _preflight_motion(self) -> None:
        """Common gate for any motion session (caller holds self._lock). By the time a joint
        can be armed it is calibrated, so calibration is enforced at arm(), not re-checked here."""
        if self._state != SessionState.CONNECTED:
            raise ControlError(
                f"Cannot start motion from {self._state.value}; connect + arm first.", 409)
        if not self._armed:
            raise ControlError("Not armed — set 'I am present' before any motion.", 409)
        if self.estop.fired:
            raise ControlError("E-STOP is latched — reconnect to clear.", 409)
        if not self.client.is_running():
            raise ControlError("Daemon not running — no telemetry.", 503)
        if not self.deadman_ok():
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

    def select_session(self, kind: str, checkpoint: str | None = None) -> None:
        """Pick what a trigger-engage runs: 'hold' (ZeroPolicy → default_pose) or 'policy'
        (a learned checkpoint). Only settable while not in a live session."""
        if kind not in ("hold", "policy"):
            raise ControlError("kind must be 'hold' or 'policy'.", 400)
        if kind == "policy" and not checkpoint:
            raise ControlError("policy session needs a checkpoint.", 400)
        with self._lock:
            if self._state in _ACTIVE_STATES:
                raise ControlError("Disarm before changing the selected session.", 409)
            self._selected = {"kind": kind, "checkpoint": checkpoint}

    def set_run_gate(self, active: bool) -> None:
        """Deadman trigger state from the gamepad: True = held (engage/run), False = released
        (damp). Distinct from the heartbeat — a release damps; a controller loss E-STOPs."""
        if active:
            self._run_gate.set()
        else:
            self._run_gate.clear()

    def set_walk_command(self, vx: float, vy: float, wz: float) -> None:
        """Live locomotion command (forward, lateral, yaw) from the gamepad sticks; consumed
        by the running policy each tick. Ignored (harmless) for a 'hold' session."""
        with self._command_lock:
            self._command = np.array([vx, vy, wz], dtype=np.float32)

    def arm_deadman(self, kind: str | None = None, checkpoint: str | None = None,
                    *, ramp: float = 1.5) -> None:
        """Enter the ARMED deadman session: legs → DAMPING, spawn the trigger-driven worker.
        Requires CONNECTED + all joints calibrated + a live controller (deadman)."""
        with self._lock:
            if self._state != SessionState.CONNECTED:
                raise ControlError(f"Cannot arm from {self._state.value}; connect first.", 409)
            if self.estop.fired:
                raise ControlError("E-STOP is latched — reconnect to clear.", 409)
            uncal = [n for n in self._joints if not self._calibrated[n]]
            if uncal:
                raise ControlError(
                    f"Calibrate all joints before arming — {len(uncal)} uncalibrated.", 409)
            if not self.client.is_running():
                raise ControlError("Daemon not running — no telemetry.", 503)
            if not self.deadman_ok():
                raise ControlError("No live controller — connect the gamepad deadman first.", 409)
            if self._session_thread and self._session_thread.is_alive():
                raise ControlError("A session is already active.", 409)

            sel_kind = kind or self._selected["kind"]
            sel_ckpt = checkpoint if kind else self._selected["checkpoint"]
            if kind:
                self._selected = {"kind": sel_kind, "checkpoint": sel_ckpt}
            if sel_kind == "policy":
                if not sel_ckpt:
                    raise ControlError("policy session needs a checkpoint.", 400)
                policy = load_policy(sel_ckpt, num_actions=self.contract.num_joints)
            else:
                policy = ZeroPolicy(self.contract.num_joints)

            runner = PolicyRunner(
                self.client, self.contract, policy,
                base_source=TelemetryBaseState(lambda: {"base": self.client.latest_base()}),
                command=self._command.copy(), estop=self.estop, ramp_seconds=ramp,
            )
            self._armed = True
            self._stop_evt.clear()
            self._run_gate.clear()
            self._state = SessionState.ARMED
            t = threading.Thread(target=self._deadman_worker, args=(runner, sel_kind),
                                 name=f"deadman-{sel_kind}", daemon=True)
            self._session_thread = t
            t.start()
        _log.info("ARMED deadman session (kind=%s, ramp=%.1fs) — legs DAMPING, hold a trigger to run.",
                  sel_kind, ramp)

    def disarm_deadman(self) -> None:
        """Leave the deadman session: stop the worker (legs → IDLE), back to CONNECTED."""
        self.stop(wait=True)

    def _deadman_worker(self, runner: PolicyRunner, kind: str) -> None:
        """Persistent trigger-driven loop. Released trigger → DAMPING (rest); held trigger →
        engage (ramp to default_pose, abortable on release) then step the policy with the live
        command. Survives release/re-press; exits only on stop or E-STOP."""
        engaged = False
        dt = self.contract.policy_dt
        engaged_state = SessionState.RUNNING if kind == "policy" else SessionState.HOLDING
        try:
            self.legs.idle()   # ARMED rest = IDLE (zero-torque). DAMPING faults the firmware
            # watchdog in ~1s (not daemon-fed) — see notes; IDLE is dormant + safe.
            next_tick = time.monotonic()
            while not self.estop.fired and not self._stop_evt.is_set():
                if self._run_gate.is_set():
                    if not engaged:
                        with self._lock:
                            self._state = engaged_state
                        # Engage: seed@current (jerk-free from DAMPING) + ramp to default_pose,
                        # bailing the instant the trigger is released or E-STOP fires.
                        ok = runner.prepare(
                            should_abort=lambda: self._stop_evt.is_set() or not self._run_gate.is_set())
                        if not ok:
                            if not (self.estop.fired or self._stop_evt.is_set()):
                                self.legs.idle()          # released mid-ramp → rest (IDLE, not DAMPING: watchdog)
                                with self._lock:
                                    self._state = SessionState.ARMED
                            continue
                        engaged = True
                        next_tick = time.monotonic()
                    self.legs.check_health()              # raises on fault → finally IDLEs
                    with self._command_lock:
                        runner.command = self._command.copy()
                    runner.step()
                    next_tick += dt
                    sleep = next_tick - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_tick = time.monotonic()
                else:
                    if engaged:
                        self.legs.idle()                  # trigger released → rest (IDLE, not DAMPING: watchdog)
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
            try:
                self.legs.idle()   # leaving the session → free (IDLE), disarmed
            except Exception as exc:
                _log.warning("idle after deadman session failed: %s", exc)
            self._on_session_end()
            _log.info("deadman session ended.")

    # ── manual control (capture-and-hold / go-to-pose) ───────────────────────
    def current_pose_rad(self) -> dict[str, float]:
        """Live position (rad) of every joint, canonical order. Raises if any is offline."""
        out: dict[str, float] = {}
        for name in self.contract.joint_order:
            st = self.client.get_cached_joint_state(name)
            state = (st or {}).get("state") or (st or {}).get("joint_state")
            if st is None or state in (None, "OFFLINE") or st.get("position") is None:
                raise ControlError(f"{name.replace('_joint','')} offline — cannot read pose.", 409)
            out[name] = float(st["position"])
        return out

    def start_manual_hold(self, targets_rad: dict[str, float], *,
                          ramp: float = 4.0, seconds: float | None = None) -> None:
        """Ramp the named joints to target positions (rad) and hold. Non-named joints stay IDLE.
        Motion — same gates as policy. Used by capture-and-hold (all 12 = current pose) and
        go-to-pose (a saved pose, possibly a subset)."""
        targets_rad = {n: float(v) for n, v in (targets_rad or {}).items()
                       if n in self.contract.joint_order}
        if not targets_rad:
            raise ControlError("No valid target joints to hold.", 400)
        with self._lock:
            self._preflight_motion()
            self._stop_evt.clear()
            self._state = SessionState.HOLDING
            t = threading.Thread(target=self._manual_worker, args=(targets_rad, ramp, seconds),
                                 name="motion-manual", daemon=True)
            self._session_thread = t
            t.start()
        _log.info("manual hold started: %d joints (ramp=%.1fs).", len(targets_rad), ramp)

    def _manual_worker(self, targets: dict[str, float], ramp: float, seconds) -> None:
        joints = [n for n in self.contract.joint_order if n in targets]   # canonical order
        idx = [self.contract.index_of(n) for n in joints]
        lo = self.contract.pos_limit_lower[idx]
        hi = self.contract.pos_limit_upper[idx]
        goal = np.clip(np.array([targets[n] for n in joints], dtype=np.float32), lo, hi)

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
            self.legs.check_health()
            start = read_current()
            if np.any(np.isnan(start)):
                raise RuntimeError("could not read all joint positions")
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
                self.legs.check_health()
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
