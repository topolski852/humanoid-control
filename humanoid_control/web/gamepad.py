"""
Robot-local gamepad deadman — PREPARED BUT DISABLED BY DEFAULT.

Intended as the *primary*, WiFi-independent safety once a controller is on hand: a Bluetooth
Xbox controller paired to the robot PC acts as a deadman. It registers as a control client
(the same deadman the browser uses), so **if the controller battery dies or the Bluetooth
signal drops, its heartbeat stops and any active motion session is E-STOPped** — exactly the
behaviour requested. A dedicated button is a hard, immediate E-STOP.

This is intentionally **not enabled**. It only runs when ``HUMANOID_GAMEPAD_ENABLE`` is set,
and it imports ``evdev`` lazily so the server has no dependency on it otherwise. Install with
``pip install evdev`` and pair the controller (``bluetoothctl``) before enabling.

Env knobs:
  HUMANOID_GAMEPAD_ENABLE   any truthy value turns it on (checked in server.py)
  HUMANOID_GAMEPAD_MODE     "monitor" (default) — deadman = controller present + alive
                            "holdtorun" — heartbeat only while the ENABLE button is held
  HUMANOID_GAMEPAD_DEVICE   substring to match the input device name (default "Xbox")

Default button map (Xbox layout; override the constants below if yours differs):
  B  (BTN_EAST)   → hard E-STOP
  Y  (BTN_NORTH)  → graceful stop
  A  (BTN_SOUTH)  → start hold (ramp to default_pose)
  RB (BTN_TR)     → ENABLE / hold-to-run button (holdtorun mode)
"""
from __future__ import annotations

import logging
import os
import select
import threading
import time

from .service import ControlService

_log = logging.getLogger(__name__)

# Button codes (filled from evdev.ecodes on start to avoid an import at module load).
_BTN_ESTOP = "BTN_EAST"    # B
_BTN_STOP = "BTN_NORTH"    # Y
_BTN_HOLD = "BTN_SOUTH"    # A
_BTN_ENABLE = "BTN_TR"     # RB (hold-to-run)

_HEARTBEAT_S = 0.1         # how often we refresh the deadman heartbeat while alive
_RECONNECT_S = 2.0        # retry cadence when no controller is found


class GamepadDeadman:
    def __init__(self, service: ControlService):
        self.service = service
        self.mode = os.environ.get("HUMANOID_GAMEPAD_MODE", "monitor")
        self.device_hint = os.environ.get("HUMANOID_GAMEPAD_DEVICE", "Xbox")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gamepad-deadman", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    # ── internals ────────────────────────────────────────────────────────────
    def _find_device(self):
        import evdev  # lazy
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            if self.device_hint.lower() in (dev.name or "").lower():
                return dev
        return None

    def _run(self) -> None:
        try:
            import evdev  # noqa: F401  (validate the dependency is present)
            from evdev import ecodes
        except Exception as exc:
            _log.error("gamepad: evdev not available (%s) — deadman disabled.", exc)
            return

        estop_code = getattr(ecodes, _BTN_ESTOP)
        stop_code = getattr(ecodes, _BTN_STOP)
        hold_code = getattr(ecodes, _BTN_HOLD)
        enable_code = getattr(ecodes, _BTN_ENABLE)

        while not self._stop.is_set():
            dev = self._find_device()
            if dev is None:
                # No controller. If we're mid-motion and this is the deadman, that is unsafe.
                if self.service.is_motion_active():
                    self.service.trigger_estop("gamepad-absent")
                self._stop.wait(_RECONNECT_S)
                continue
            _log.info("gamepad: connected to %s (mode=%s)", dev.name, self.mode)
            self.service.control_client_connected()
            try:
                self._device_loop(dev, ecodes, estop_code, stop_code, hold_code, enable_code)
            except OSError as exc:
                _log.warning("gamepad: device error (%s) — treating as disconnect.", exc)
            finally:
                # Disconnect (battery dead / BT dropped) → deadman lost → E-STOP if moving.
                self.service.control_client_disconnected()
                try:
                    dev.close()
                except Exception:
                    pass

    def _device_loop(self, dev, ecodes, estop_code, stop_code, hold_code, enable_code) -> None:
        enable_held = False
        while not self._stop.is_set():
            r, _, _ = select.select([dev.fd], [], [], _HEARTBEAT_S)
            if r:
                for event in dev.read():
                    if event.type != ecodes.EV_KEY or event.value not in (0, 1):
                        continue
                    pressed = event.value == 1
                    if event.code == estop_code and pressed:
                        self.service.trigger_estop("gamepad-button")
                    elif event.code == stop_code and pressed:
                        self.service.stop(wait=False)
                    elif event.code == hold_code and pressed:
                        try:
                            self.service.start_hold()
                        except Exception as exc:
                            _log.info("gamepad: hold rejected (%s)", exc)
                    elif event.code == enable_code:
                        enable_held = pressed
            # Deadman heartbeat: presence-based (monitor) or gated by the enable button.
            if self.mode == "holdtorun":
                if enable_held:
                    self.service.mark_heartbeat()
            else:
                self.service.mark_heartbeat()
