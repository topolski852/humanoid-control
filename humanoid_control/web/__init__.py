"""
humanoid_control.web — wireless web control for the legs-only runtime.

A long-lived FastAPI process that owns a single ``DaemonClient`` + ``EstopController`` and
exposes the same lifecycle the CLI scripts drive (connect → arm → ramp/hold → run policy),
plus live telemetry and an always-available E-STOP, over the LAN. Run with::

    python -m humanoid_control.web        # binds 0.0.0.0:8000 by default

See the repo README (Wireless web control) and ``deploy/`` for systemd units.
"""
from .service import ControlService, ControlError, SessionState

__all__ = ["ControlService", "ControlError", "SessionState"]
