"""
Vendored humanoid-studio daemon client — the ONLY way this repo talks to the robot.

Provenance: copied from ``humanoid-studio/backend/humanoid/`` (see repo root
``README.md`` / ``docs/DAEMON_SPEC.md``). Keep in sync with studio if the daemon
protocol changes.

- ``daemon_client.py`` / ``robot_config.py``: verbatim (only internal imports made
  relative).
- ``actuator.py``: trimmed to ``ActuatorState`` + ``Mode``/``ErrorCode`` (the raw-CAN
  ``Actuator`` class is dropped — the C++ daemon owns CAN; we never open a CAN socket).

Usage: async ``start()/stop()/connect()``; synchronous ``set_position``,
``set_mode``, ``get_cached_joint_state``, ``estop_all`` in the hot loop.
"""
from .daemon_client import (
    DaemonClient,
    DaemonActuatorProxy,
    Mode,
    DaemonError,
    DaemonNotRunningError,
    DaemonCommandError,
    DaemonNotSupportedError,
)
from .robot_config import RobotConfig, JointConfig, PositionLimits
from .actuator import ActuatorState, ErrorCode

__all__ = [
    "DaemonClient",
    "DaemonActuatorProxy",
    "Mode",
    "ErrorCode",
    "DaemonError",
    "DaemonNotRunningError",
    "DaemonCommandError",
    "DaemonNotSupportedError",
    "RobotConfig",
    "JointConfig",
    "PositionLimits",
    "ActuatorState",
]
