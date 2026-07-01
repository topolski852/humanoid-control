"""
ActuatorState snapshot + Recoil enums — vendored subset of humanoid-studio's
``backend/humanoid/actuator.py``.

The studio module also defines a raw-CAN ``Actuator`` class that talks to the ESC
directly via ``can_bus.CANBus`` (python-can). That path is intentionally **omitted**
here: in this repo the C++ daemon owns every CAN socket and we speak only UDP to it,
so the control code must never import python-can. ``DaemonClient`` only needs
``ActuatorState`` from this module, so that (plus the two enums its defaults reference)
is all we vendor.

``Mode`` and ``ErrorCode`` are copied verbatim from ``can_bus.py`` so the values stay
in lockstep with the firmware. If you need the full raw-CAN ``Actuator`` (offline
flashing/commissioning only), use it from humanoid-studio.
"""
from __future__ import annotations

from enum import IntEnum

from pydantic import BaseModel, computed_field


class Mode(IntEnum):
    """Firmware MotorMode (recoil_protocol.hpp)."""
    DISABLED            = 0x00
    IDLE                = 0x01
    DAMPING             = 0x02
    CALIBRATION         = 0x05
    CURRENT             = 0x10
    TORQUE              = 0x11
    VELOCITY            = 0x12
    POSITION            = 0x13
    VABC_OVERRIDE       = 0x20
    VALPHABETA_OVERRIDE = 0x21
    VQD_OVERRIDE        = 0x22
    DEBUG               = 0x80


class ErrorCode(IntEnum):
    """Firmware error bitmask (recoil_protocol.hpp)."""
    NO_ERROR             = 0b0000_0000_0000_0000
    GENERAL              = 0b0000_0000_0000_0001
    ESTOP                = 0b0000_0000_0000_0010
    INITIALIZATION_ERROR = 0b0000_0000_0000_0100
    CALIBRATION_ERROR    = 0b0000_0000_0000_1000
    POWERSTAGE_ERROR     = 0b0000_0000_0001_0000
    INVALID_MODE         = 0b0000_0000_0010_0000
    WATCHDOG_TIMEOUT     = 0b0000_0000_0100_0000
    OVER_VOLTAGE         = 0b0000_0000_1000_0000
    OVER_CURRENT         = 0b0000_0001_0000_0000
    OVER_TEMPERATURE     = 0b0000_0010_0000_0000
    CAN_RX_FAULT         = 0b0000_0100_0000_0000
    CAN_TX_FAULT         = 0b0000_1000_0000_0000
    I2C_FAULT            = 0b0001_0000_0000_0000
    ENCODER_FAULT        = 0b0010_0000_0000_0000


class ActuatorState(BaseModel):
    """Snapshot of a single actuator's real-time state."""
    position: float = 0.0          # rad (output-side, after gear ratio)
    velocity: float = 0.0          # rad/s
    torque: float = 0.0            # Nm (estimated from Iq * Kt)
    current: float = 0.0           # A  (Iq — quadrature current)
    mode: int = Mode.DISABLED
    mode_name: str = "DISABLED"
    error: int = 0
    bus_voltage: float | None = None  # V; None = SDO read failed / no data yet
    firmware_version: str | None = None  # "v3.2.0" format; None = not yet read
    timestamp: float = 0.0         # Unix time of last update

    @property
    def has_error(self) -> bool:
        return self.error != 0

    @computed_field
    @property
    def error_names(self) -> list[str]:
        return [m.name for m in ErrorCode if m != ErrorCode.NO_ERROR and (self.error & m)]
