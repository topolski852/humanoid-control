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
import threading
import time
from enum import Enum

import numpy as np

from ..config import LegPolicyContract
from ..interface import LegInterface
from ..policy import ZeroPolicy, load_policy
from ..runner import PolicyRunner
from ..safety import EstopController
from ..base_state import UprightStubBaseState
from ..daemon import DaemonClient

_log = logging.getLogger(__name__)

# Deadman: motion requires a control heartbeat at least this fresh.
_DEADMAN_TIMEOUT_S = 1.0


class SessionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"   # sockets up, joints not configured/awake
    CONNECTED = "CONNECTED"         # joints configured + online, idle
    HOLDING = "HOLDING"             # ramping/holding default_pose (ZeroPolicy)
    RUNNING = "RUNNING"             # running a learned policy
    ESTOPPED = "ESTOPPED"           # E-STOP latched; reconnect to clear
    ERROR = "ERROR"                 # a session failed (fault/offline)


_MOTION_STATES = {SessionState.HOLDING, SessionState.RUNNING}


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
            st = self.client.get_cached_joint_state(name)
            if st is None:
                joints.append({"index": i, "name": name, "online": False})
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
            "joints": joints,
            "buses": buses,
        }

    # ── deadman ──────────────────────────────────────────────────────────────
    def control_client_connected(self) -> None:
        with self._lock:
            self._control_clients += 1
        self.mark_heartbeat()

    def control_client_disconnected(self) -> None:
        with self._lock:
            self._control_clients = max(0, self._control_clients - 1)
        # Losing the deadman while moving is an immediate E-STOP.
        if self._control_clients == 0 and self.is_motion_active():
            self.trigger_estop("deadman-disconnect")

    def mark_heartbeat(self) -> None:
        self._last_heartbeat = time.monotonic()

    def deadman_ok(self) -> bool:
        return (
            self._control_clients > 0
            and (time.monotonic() - self._last_heartbeat) < _DEADMAN_TIMEOUT_S
        )

    def check_deadman_watchdog(self) -> None:
        """Called periodically by the event-loop watchdog: trip E-STOP if a motion
        session has lost its heartbeat (WiFi stall where the socket hasn't closed yet)."""
        if self.is_motion_active() and not self.deadman_ok():
            self.trigger_estop("deadman-timeout")

    # ── connection lifecycle (BLOCKING — call via executor) ──────────────────
    def connect(self) -> None:
        with self._lock:
            if self.is_motion_active():
                raise ControlError("A motion session is active — stop it first.", 409)
            if not self.config_present:
                raise ControlError(
                    "No robot config loaded (HUMANOID_CONFIG missing) — cannot connect.", 503)
        # apply_all_configs: wake joints DISABLED→IDLE + delta-write config (seconds).
        self.client.apply_all_configs()
        time.sleep(0.3)
        self.legs.check_health()   # raises if any leg offline/faulted
        with self._lock:
            self.estop = self._new_estop()   # clear any prior latched E-STOP
            self._armed = False
            self._last_error = None
            self._state = SessionState.CONNECTED
        _log.info("connected: all %d leg joints online.", self.contract.num_joints)

    def disconnect(self) -> None:
        self.stop(wait=True)
        try:
            self.legs.idle()
        except Exception as exc:
            _log.warning("idle on disconnect failed: %s", exc)
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
            self._armed = True
        _log.info("ARMED (operator present / robot supported).")

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

    def _start_session(self, kind, policy, *, command, ramp, seconds, checkpoint=None) -> None:
        with self._lock:
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

            self._stop_evt.clear()
            new_state = SessionState.HOLDING if kind == "hold" else SessionState.RUNNING
            self._state = new_state
            runner = PolicyRunner(
                self.client, self.contract, policy,
                base_source=UprightStubBaseState(),
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
            if self.estop.fired:
                self._state = SessionState.ESTOPPED
            elif self._state in _MOTION_STATES:
                self._state = SessionState.CONNECTED

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
