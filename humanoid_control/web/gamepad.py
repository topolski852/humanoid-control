"""
Robot-local gamepad — the primary hold-to-run deadman for the leg controller.

Flow (see ControlService gamepad section): connect + calibrate in the web UI, then drive the
robot entirely from the controller:

  A  (BTN_SOUTH)      arm the deadman session. Requires calibrated, UNLESS the selected
                      session is 'manual' (capture-and-hold, no calibration).
  B  (BTN_EAST)       disarm (back to CONNECTED).
  START              hard E-STOP (latched; reconnect in the web UI to clear).
  SELECT             switch between ARM control and LEG control. Only offers modes the
                      configured layout actually supports.
  Y / X              creep / normal speed.
  LB / RB            (arm mode) select which arm the sticks drive.
  LT or RT            deadman trigger — HOLD (either one) to engage. RELEASE → rest.

  LEG mode   Left stick  = walk vx/vy      Right stick X = yaw (wz)
  ARM mode   Left stick X = shoulder_pitch    Left stick Y = shoulder_roll
             Right stick X = shoulder_yaw      Right stick Y = elbow_pitch
             One stick, one joint — no IK, no coupling. Cartesian and spherical frames also
             exist (TeleopTuning.frame) but trade this predictability for convenience.

E-STOP IS ON START, NOT B. B is disarm — an orderly stop through the session. E-STOP bypasses
everything via the priority port. Do not conflate them.

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
_BTN_ESTOP = "BTN_START"    # Start → hard E-STOP (B is disarm; E-STOP needs its own button)
_BTN_DISARM = "BTN_EAST"    # B  → disarm
_BTN_ARM = "BTN_SOUTH"      # A  → arm the deadman session
# NOTE: use the LETTER names, not the compass names. In Linux input BTN_NORTH is an alias for
# BTN_X (0x133) and BTN_WEST for BTN_Y (0x134) — which is swapped relative to the letters
# printed on an Xbox pad, where X is west and Y is north. Binding by compass silently maps the
# wrong physical button.
_BTN_CREEP = "BTN_Y"        # Y  → creep speed
_BTN_NORMAL = "BTN_X"       # X  → normal speed
_BTN_MODE = "BTN_SELECT"    # Select → toggle arm/leg control
_BTN_LEFT_LIMB = "BTN_TL"   # LB → drive the left arm
_BTN_RIGHT_LIMB = "BTN_TR"  # RB → drive the right arm
# Triggers as digital fallbacks (some modes report LT/RT as buttons, not axes).
_BTN_LT = "BTN_TL2"
_BTN_RT = "BTN_TR2"

_LOOP_S = 0.02              # command/heartbeat refresh cadence (50 Hz)
_RECONNECT_S = 2.0         # retry cadence when no controller is found

# This module's identity in the service's input-source arbitration. When another source holds
# the token (e.g. the Quest), the pad keeps reporting its raw state to the UI but commands
# nothing and E-STOPs nothing — it is a diagnostic, not a controller.
_SOURCE = "xbox"

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
            if getattr(ecodes, "BTN_SOUTH", None) in keys and ecodes.ABS_X in abss:
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
                # E-STOP on absence ONLY while the gamepad is the deadman of record. It used
                # to fire for any live session, which meant a pad switched off in a drawer
                # would kill a session driven by something else entirely — and with the
                # headset on, the pad being off is the normal case, not a fault.
                if (self.service.state.name in ("ARMED", "HOLDING", "RUNNING")
                        and self.service.deadman_source() == _SOURCE):
                    self.service.trigger_estop("gamepad-absent")
                self.service.drop_source(_SOURCE)
                self._stop.wait(_RECONNECT_S)
                continue
            _log.info("gamepad: connected to %s", dev.name)
            self.service.mark_source_alive(_SOURCE)        # presence → this source's heartbeat
            self.service.set_gamepad_connected(dev.name)   # surface to the UI
            try:
                self._device_loop(dev, ecodes)
            except OSError as exc:
                _log.warning("gamepad: device error (%s) — treating as disconnect.", exc)
            except Exception:
                # A BUG in the loop must not silently remove the deadman. Degrade to
                # "controller lost" — which E-STOPs any live session via the presence
                # watchdog — and let the outer loop retry, but log the traceback so the
                # defect is visible rather than presenting as a controller that went quiet.
                _log.exception("gamepad: loop failed — treating as disconnect and retrying.")
            finally:
                # Controller lost (unplug / receiver drop). The source goes not-alive at once;
                # if it was the deadman of record for a live session, the presence watchdog
                # E-STOPs on the next poll.
                self.service.drop_source(_SOURCE)
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

        # AXIS LAYOUT IS NOT UNIVERSAL. Two are common and they CONFLICT:
        #
        #   xpad (USB):      left X/Y   right RX/RY   triggers Z/RZ
        #   Bluetooth HID:   left X/Y   right Z/RZ    triggers BRAKE/GAS
        #
        # Assuming xpad on a Bluetooth pad is actively dangerous, not merely broken: the
        # deadman trigger ends up reading the RIGHT STICK, which rests at ~0.5 — exactly the
        # trigger threshold — so the run-gate flickers on a stick nobody is touching. Detect
        # by what the device advertises instead.
        bt_layout = ecodes.ABS_GAS in abs_codes and ecodes.ABS_BRAKE in abs_codes
        if bt_layout:
            AX_RIGHT_X, AX_RIGHT_Y = ecodes.ABS_Z, ecodes.ABS_RZ
            AX_LT, AX_RT = ecodes.ABS_BRAKE, ecodes.ABS_GAS
        else:
            AX_RIGHT_X, AX_RIGHT_Y = ecodes.ABS_RX, ecodes.ABS_RY
            AX_LT, AX_RT = ecodes.ABS_Z, ecodes.ABS_RZ
        _log.info("gamepad: %s axis layout (right stick 0x%02x/0x%02x, triggers 0x%02x/0x%02x)",
                  "bluetooth-hid" if bt_layout else "xpad", AX_RIGHT_X, AX_RIGHT_Y, AX_LT, AX_RT)

        has_lt_axis = AX_LT in abs_codes
        has_rt_axis = AX_RT in abs_codes
        # Some pads report BTN_START/BTN_SELECT under legacy aliases; fall back rather than
        # crashing the thread on a controller that names them differently.
        def code(name, *fallbacks):
            for n in (name, *fallbacks):
                if hasattr(ecodes, n):
                    return getattr(ecodes, n)
            return None
        codes = {
            "estop":  code(_BTN_ESTOP, "BTN_START"),
            "arm":    code(_BTN_ARM),
            "disarm": code(_BTN_DISARM),
            "creep":  code(_BTN_CREEP),
            "normal": code(_BTN_NORMAL),
            "mode":   code(_BTN_MODE, "BTN_SELECT", "BTN_BACK"),
            "left":   code(_BTN_LEFT_LIMB),
            "right":  code(_BTN_RIGHT_LIMB),
        }
        # An unresolved binding silently never matches. That is fine for an intentionally
        # unbound button, but for E-STOP it would mean the controller has no emergency stop
        # and nothing would say so. Refuse to run the loop quietly in that state.
        if codes["estop"] is None:
            _log.error("gamepad: E-STOP button %s not present on %s — the controller has NO "
                       "emergency stop. Fix the binding before relying on it.",
                       _BTN_ESTOP, dev.name)
            self.service.set_gamepad_connected(f"{dev.name} (NO E-STOP)")
        missing = [k for k, v in codes.items() if v is None and k != "arm"]
        if missing:
            _log.warning("gamepad: unresolved bindings on %s: %s", dev.name, ", ".join(missing))

        # Buttons the kernel's generic gamepad profile advertises but that do not physically
        # exist on an Xbox pad.
        PHANTOM = {getattr(ecodes, n) for n in ("BTN_C", "BTN_Z") if hasattr(ecodes, n)}
        # Not reported to the UI at all. C/Z do not exist on the hardware; SHARE (KEY_RECORD)
        # and MODE (the Xbox guide button) DO exist but are unused, so they are hidden to keep
        # the panel to what is actually in play. Remove from here to surface and bind one.
        HIDDEN = PHANTOM | {0xA7} | ({ecodes.BTN_MODE} if hasattr(ecodes, "BTN_MODE") else set())
        # Codes outside the BTN_ range that are nonetheless real buttons.
        EXTRA_NAMES = {0xA7: "SHARE"}      # KEY_RECORD — the Share button on 2020+ Xbox pads

        # Everything the device advertises minus what we hide, for the live input view.
        all_buttons = [c for c in sorted(caps.get(ecodes.EV_KEY, [])) if c not in HIDDEN]
        action_of = {v: k for k, v in codes.items() if v is not None}

        def btn_label(c):
            if c in EXTRA_NAMES:
                return EXTRA_NAMES[c]
            nm = ecodes.BTN.get(c)
            if isinstance(nm, (list, tuple)):
                # Prefer the printed letter over the compass alias — BTN_NORTH/BTN_X are the
                # same code, and the letter is what is on the controller.
                for pref in ("BTN_A", "BTN_B", "BTN_X", "BTN_Y"):
                    if pref in nm:
                        return pref.replace("BTN_", "")
                nm = nm[0]
            return (nm or f"0x{c:03x}").replace("BTN_", "")

        axis_report = [("left_x", ecodes.ABS_X, True), ("left_y", ecodes.ABS_Y, True),
                       ("right_x", AX_RIGHT_X, True), ("right_y", AX_RIGHT_Y, True),
                       ("lt", AX_LT, False), ("rt", AX_RT, False)]
        axis_report = [a for a in axis_report if a[1] in abs_codes]
        lt_btn = getattr(ecodes, _BTN_LT)
        rt_btn = getattr(ecodes, _BTN_RT)

        while not self._stop.is_set():
            # Does this pad currently drive the robot? Re-read every loop so switching the
            # control method in the UI takes effect immediately, with no thread restart.
            active = self.service.input_source == _SOURCE

            r, _, _ = select.select([dev.fd], [], [], _LOOP_S)
            if r:
                try:
                    for event in dev.read():
                        if event.type == ecodes.EV_KEY and event.value in (0, 1):
                            self._on_button(event.code, event.value == 1, codes, active)
                except BlockingIOError:
                    pass

            # Trigger (deadman run-gate): either analog trigger past threshold, or its button.
            active_keys = set(dev.active_keys())
            lt = norm(AX_LT, False) if has_lt_axis else (1.0 if lt_btn in active_keys else 0.0)
            rt = norm(AX_RT, False) if has_rt_axis else (1.0 if rt_btn in active_keys else 0.0)
            gate = (lt >= self.trig_thresh) or (rt >= self.trig_thresh)

            if active:
                self.service.set_run_gate(gate, source=_SOURCE)

                # Sticks mean different things per mode. Raw deflection is sent for ARM mode —
                # the teleop layer owns the deadband and the metres/second scaling, so the
                # deadband is applied exactly once and in the place that integrates it.
                if self.service.control_mode == "arm":
                    # Send the RAW sticks, sign-corrected so "up" is positive. What each axis
                    # means belongs to ArmTeleop — the input layer should not know or care
                    # whether the active frame is spherical or Cartesian.
                    lx = -norm(ecodes.ABS_X, True)              # stick right -> +
                    ly = -norm(ecodes.ABS_Y, True)              # stick up    -> +
                    ry = -norm(AX_RIGHT_Y, True)                # stick up    -> +
                    rx = -norm(AX_RIGHT_X, True)                # stick right -> +
                    self.service.set_arm_command(lx, ly, ry, rx, source=_SOURCE)
                else:
                    vx = _VX_SIGN * self._deadband(norm(ecodes.ABS_Y, True)) * self.vx_max
                    vy = _VY_SIGN * self._deadband(norm(ecodes.ABS_X, True)) * self.vy_max
                    wz = _WZ_SIGN * self._deadband(norm(AX_RIGHT_X, True)) * self.wz_max
                    self.service.set_walk_command(vx, vy, wz, source=_SOURCE)

            # Raw input snapshot for the UI. Reports EVERY button the device advertises, not
            # just the bound ones — the point is to see what the hardware actually sends, so a
            # binding that maps the wrong physical button is visible rather than inferred.
            self.service.set_gamepad_input({
                "buttons": [
                    {"code": c, "name": btn_label(c),
                     "pressed": c in active_keys,
                     "action": action_of.get(c),
                     "phantom": c in PHANTOM}
                    for c in all_buttons
                ],
                "axes": [
                    {"name": nm, "value": round(float(norm(cd, ctr)), 4), "centered": ctr}
                    for nm, cd, ctr in axis_report
                ],
                "layout": "bluetooth-hid" if bt_layout else "xpad",
                "deadband": self.deadband,
                "trigger_threshold": self.trig_thresh,
                "active": active,
            })

            # Heartbeat: this source is alive this loop. Sent even when the pad holds no
            # authority, so the UI can distinguish "connected but standing down" from "gone".
            self.service.mark_source_alive(_SOURCE)

    def _on_button(self, code, pressed, codes, active: bool = True) -> None:
        if not pressed:
            return
        # E-STOP first and unconditionally — it must never be gated behind state checks.
        # Deliberately NOT gated on `active` either: any connected source may always stop the
        # robot, even one that is not currently allowed to drive it.
        if code == codes["estop"]:
            self.service.trigger_estop("gamepad-button")
            return
        # Every other button commands the robot, so it belongs to whoever holds the token.
        if not active:
            return
        try:
            if code == codes["arm"]:
                self.service.arm_deadman()
            elif code == codes["disarm"]:
                self.service.disarm_deadman()
            elif code == codes["creep"]:
                self.service.set_speed_mode("creep")
            elif code == codes["normal"]:
                self.service.set_speed_mode("normal")
            elif code == codes["mode"]:
                self.service.toggle_control_mode()
            elif code == codes["left"]:
                self.service.select_arm("left_arm")
            elif code == codes["right"]:
                self.service.select_arm("right_arm")
        except ControlError as exc:
            _log.info("gamepad: rejected (%s)", exc)
        except Exception as exc:
            _log.warning("gamepad: button handler failed (%s)", exc)
