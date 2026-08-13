"""
Robot-local gamepad — the primary hold-to-run deadman for the leg controller.

Flow (see ControlService gamepad section): connect + calibrate in the web UI, then drive the
robot entirely from the controller:

  A  (BTN_SOUTH)      arm the deadman session (legs → DAMPING, ARMED). Requires calibrated,
                      UNLESS the selected session is 'manual' (capture-and-hold, no calibration).
  Y  (BTN_NORTH)      disarm (legs → IDLE, back to CONNECTED).
  B  (BTN_EAST)       hard E-STOP (latched; reconnect in the web UI to clear).
  LT or RT            deadman trigger — HOLD (either one) to engage: ramp to default_pose and run
                      the selected session ('manual' instead holds the live pose as-read).
                      RELEASE → DAMPING (rest). Re-press → re-engage.
  Left stick          walk command: up = forward (vx), left = left (vy).   [0.15 deadband]
  Right stick X       walk command: yaw (wz).                              [0.15 deadband]

Two independent safety signals:
  • Heartbeat  = "controller is alive/present". Refreshed every loop while connected; if the
    controller unplugs or its receiver drops, the heartbeat stops and the presence watchdog
    E-STOPs any live (ARMED/HOLDING/RUNNING) session. This is the deadman of record.
  • Run-gate   = "a trigger is held". Release → DAMPING (recoverable); it is NOT an E-STOP.

Enabled only when HUMANOID_GAMEPAD_ENABLE is set (checked in server.py). Needs `evdev` and
read access to the controller's /dev/input/event* (see deploy/99-humanoid-input.rules).

Env knobs:
  HUMANOID_GAMEPAD_DEVICE      substring match on device name (default: auto-detect a gamepad)
  HUMANOID_GAMEPAD_TRIG_THRESH analog-trigger activation fraction 0..1 (default 0.5)
  HUMANOID_GAMEPAD_DEADBAND    stick deadband fraction (default 0.15 — this controller drifts)
  HUMANOID_GAMEPAD_VX_MAX / _VY_MAX / _WZ_MAX   command scales (default 0.6 / 0.4 / 0.6)
"""
from __future__ import annotations

import logging
import os
import select
import threading

from .service import ControlService, ControlError

_log = logging.getLogger(__name__)

# Button map (Xbox/8BitDo Xinput layout). Override here if your controller differs.
_BTN_ESTOP = "BTN_EAST"     # B  → hard E-STOP
_BTN_DISARM = "BTN_NORTH"   # Y  → disarm
_BTN_ARM = "BTN_SOUTH"      # A  → arm deadman session
# Triggers as digital fallbacks (some modes report LT/RT as buttons, not axes).
_BTN_LT = "BTN_TL2"
_BTN_RT = "BTN_TR2"

_LOOP_S = 0.02              # command/heartbeat refresh cadence (50 Hz)
_RECONNECT_S = 2.0         # retry cadence when no controller is found

# Sign conventions mapping stick → base frame (x-forward, y-left, z-up). Flip a sign here if
# a field test shows an axis reversed. evdev sticks: up and left read NEGATIVE by convention.
_VX_SIGN = -1.0   # stick up (negative)   → +forward
_VY_SIGN = -1.0   # stick left (negative) → +left
_WZ_SIGN = -1.0   # right-stick left      → +yaw (CCW)


def _f(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class GamepadDeadman:
    def __init__(self, service: ControlService):
        self.service = service
        self.device_hint = os.environ.get("HUMANOID_GAMEPAD_DEVICE", "")
        self.trig_thresh = _f("HUMANOID_GAMEPAD_TRIG_THRESH", 0.5)
        self.deadband = _f("HUMANOID_GAMEPAD_DEADBAND", 0.15)
        self.vx_max = _f("HUMANOID_GAMEPAD_VX_MAX", 0.6)
        self.vy_max = _f("HUMANOID_GAMEPAD_VY_MAX", 0.4)
        self.wz_max = _f("HUMANOID_GAMEPAD_WZ_MAX", 0.6)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gamepad-deadman", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    # ── deadband + scaling ───────────────────────────────────────────────────
    def _deadband(self, v: float) -> float:
        """Apply the deadband and rescale so the live range stays [-1, 1] with no step at the
        edge: |v|<db → 0, else sign(v)*(|v|-db)/(1-db)."""
        if abs(v) <= self.deadband:
            return 0.0
        return (1.0 if v > 0 else -1.0) * (abs(v) - self.deadband) / (1.0 - self.deadband)

    # ── device discovery ─────────────────────────────────────────────────────
    def _find_device(self):
        import evdev
        from evdev import ecodes
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            name = (dev.name or "")
            if self.device_hint:
                if self.device_hint.lower() in name.lower():
                    return dev
                continue
            # No hint: accept the first device that looks like a gamepad (has a South button
            # and a stick axis) — skips keyboards/mice/the IMU.
            caps = dev.capabilities()
            keys = caps.get(ecodes.EV_KEY, [])
            abss = [a for a, _ in caps.get(ecodes.EV_ABS, [])]
            if getattr(ecodes, _BTN_ARM) in keys and ecodes.ABS_X in abss:
                return dev
        return None

    def _run(self) -> None:
        try:
            import evdev  # noqa: F401
            from evdev import ecodes
        except Exception as exc:
            _log.error("gamepad: evdev not available (%s) — deadman disabled.", exc)
            return

        while not self._stop.is_set():
            dev = self._find_device()
            if dev is None:
                if self.service.state.name in ("ARMED", "HOLDING", "RUNNING"):
                    self.service.trigger_estop("gamepad-absent")
                self._stop.wait(_RECONNECT_S)
                continue
            _log.info("gamepad: connected to %s", dev.name)
            self.service.control_client_connected()   # presence → deadman heartbeat source
            self.service.set_gamepad_connected(dev.name)   # surface to the UI
            try:
                self._device_loop(dev, ecodes)
            except OSError as exc:
                _log.warning("gamepad: device error (%s) — treating as disconnect.", exc)
            finally:
                # Controller lost (unplug / receiver drop) → deadman lost → E-STOP if live.
                self.service.control_client_disconnected()
                self.service.set_gamepad_disconnected()
                try:
                    dev.close()
                except Exception:
                    pass

    # ── main device loop ─────────────────────────────────────────────────────
    def _device_loop(self, dev, ecodes) -> None:
        caps = dev.capabilities()
        abs_codes = {a: info for a, info in caps.get(ecodes.EV_ABS, [])}

        def norm(code: int, signed: bool) -> float:
            info = abs_codes.get(code)
            if info is None:
                return 0.0
            val = dev.absinfo(code).value
            lo, hi = info.min, info.max
            if hi == lo:
                return 0.0
            if signed:
                mid = (hi + lo) / 2.0
                return max(-1.0, min(1.0, (val - mid) / ((hi - lo) / 2.0)))
            return max(0.0, min(1.0, (val - lo) / (hi - lo)))

        has_lt_axis = ecodes.ABS_Z in abs_codes
        has_rt_axis = ecodes.ABS_RZ in abs_codes
        estop_code = getattr(ecodes, _BTN_ESTOP)
        arm_code = getattr(ecodes, _BTN_ARM)
        disarm_code = getattr(ecodes, _BTN_DISARM)
        lt_btn = getattr(ecodes, _BTN_LT)
        rt_btn = getattr(ecodes, _BTN_RT)

        while not self._stop.is_set():
            r, _, _ = select.select([dev.fd], [], [], _LOOP_S)
            if r:
                try:
                    for event in dev.read():
                        if event.type == ecodes.EV_KEY and event.value in (0, 1):
                            self._on_button(event.code, event.value == 1,
                                            estop_code, arm_code, disarm_code)
                except BlockingIOError:
                    pass

            # Trigger (deadman run-gate): either analog trigger past threshold, or its button.
            active_keys = set(dev.active_keys())
            lt = norm(ecodes.ABS_Z, False) if has_lt_axis else (1.0 if lt_btn in active_keys else 0.0)
            rt = norm(ecodes.ABS_RZ, False) if has_rt_axis else (1.0 if rt_btn in active_keys else 0.0)
            gate = (lt >= self.trig_thresh) or (rt >= self.trig_thresh)
            self.service.set_run_gate(gate)

            # Walk command from the sticks (deadbanded); only meaningful while engaged.
            vx = _VX_SIGN * self._deadband(norm(ecodes.ABS_Y, True)) * self.vx_max
            vy = _VY_SIGN * self._deadband(norm(ecodes.ABS_X, True)) * self.vy_max
            wz = _WZ_SIGN * self._deadband(norm(ecodes.ABS_RX, True)) * self.wz_max
            self.service.set_walk_command(vx, vy, wz)

            # Heartbeat: controller is alive this loop.
            self.service.mark_heartbeat()

    def _on_button(self, code, pressed, estop_code, arm_code, disarm_code) -> None:
        if not pressed:
            return
        if code == estop_code:
            self.service.trigger_estop("gamepad-button")
        elif code == arm_code:
            try:
                self.service.arm_deadman()
            except ControlError as exc:
                _log.info("gamepad: arm rejected (%s)", exc)
        elif code == disarm_code:
            try:
                self.service.disarm_deadman()
            except Exception as exc:
                _log.info("gamepad: disarm failed (%s)", exc)
