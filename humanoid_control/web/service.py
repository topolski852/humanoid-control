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

from ..calibration import compute_offset
from ..config import LegPolicyContract
from ..interface import LegInterface
from ..policy import ZeroPolicy, load_policy
from ..runner import PolicyRunner
from ..safety import EstopController, ramp_to_pose
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
        """Connect = wake motors DISABLED→IDLE + apply config. Allowed from DISCONNECTED,
        ESTOPPED, or ERROR (clears a latched E-STOP); refused only during active motion."""
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
            # Fresh power-up ⇒ every joint's zero is stale ⇒ mark all uncalibrated.
            self._calibrated = {n: False for n in self._joints}
            self._cal_captures = {n: {"lower": None, "upper": None} for n in self._joints}
        _log.info("connected: all %d leg joints online; calibration reset (uncalibrated).",
                  self.contract.num_joints)

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
