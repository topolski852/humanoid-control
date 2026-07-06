"""Entry point: ``python -m humanoid_control.web``.

Binds 0.0.0.0:8000 by default so the control page is reachable from any PC on the WiFi.
Override with HUMANOID_WEB_HOST / HUMANOID_WEB_PORT. Set HUMANOID_CONFIG to the robot's live
config (defaults to LIVE_ROBOT_CONFIG_PATH), HUMANOID_WEB_PASSWORD to require a login,
HUMANOID_POLICY_DIR for the checkpoint list, HUMANOID_GAMEPAD_ENABLE=1 for the (optional)
robot-local gamepad deadman.
"""
from __future__ import annotations

import os

import uvicorn


def main() -> None:
    host = os.environ.get("HUMANOID_WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("HUMANOID_WEB_PORT", "8000"))
    uvicorn.run("humanoid_control.web.server:app", host=host, port=port,
                reload=False, log_level="info")


if __name__ == "__main__":
    main()
