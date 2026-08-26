"""
FastAPI app for wireless web control of the legs-only runtime.

  REST       : humanoid_control/web/routes.py     ({success, data, error} envelope)
  WS /ws/telemetry : server push of the robot snapshot at ~20 Hz (read-only)
  WS /ws/control   : the deadman — browser heartbeats; loss during motion → E-STOP

Owns ONE DaemonClient + EstopController for the whole process (created in ``lifespan``,
stored on ``app.state``). Bind 0.0.0.0 (default) to reach it from any PC on the WiFi.
Run with ``python -m humanoid_control.web``.
"""
from __future__ import annotations

import asyncio
import logging
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Depends, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from ..config import LegPolicyContract, resolve_robot_config_path
from ..daemon import DaemonClient, RobotConfig
from .. import layout as layout_mod
from .auth import (
    auth_required, issue_token, login_locked,
    record_login_failure, record_login_success, require_auth, token_valid,
)
from .routes import router as control_router
from .service import ControlService

logging.basicConfig(level=logging.INFO)
_log = logging.getLogger(__name__)

_TELEMETRY_HZ = 20
_WATCHDOG_HZ = 5   # deadman watchdog poll rate

# Built web UI (app/dist). Served at "/" when present so the app is reachable from any browser.
_WEB_DIR = Path(__file__).resolve().parent.parent.parent / "app" / "dist"


@asynccontextmanager
async def lifespan(app: FastAPI):
    contract = LegPolicyContract.load()
    config_path = resolve_robot_config_path()
    cfg = None
    if config_path is not None:
        try:
            cfg = RobotConfig.from_json(config_path)
        except Exception as exc:
            _log.warning("failed to load robot config %s: %s", config_path, exc)
    else:
        _log.warning("no robot config found — running without hardware config "
                     "(telemetry only; connect will be refused).")

    # What hardware is attached to THIS machine (machine-local file; legs-only default).
    robot_layout = layout_mod.load()
    missing = robot_layout.missing_joints(cfg)
    if missing:
        detail = "; ".join(f"{limb}: {', '.join(js)}" for limb, js in missing.items())
        _log.warning("layout enables joints the robot config does not define — %s", detail)

    client = DaemonClient(cfg)
    await client.start()   # open UDP sockets + telemetry receive thread (no daemon needed)
    service = ControlService(client, contract, config_present=cfg is not None,
                             layout=robot_layout, robot_config=cfg)

    app.state.client = client
    app.state.service = service

    # Deadman watchdog: trip E-STOP if a motion session loses its heartbeat.
    watchdog = asyncio.create_task(_watchdog_loop(service))

    # Robot-local gamepad hold-to-run deadman (USB Xbox/8BitDo). OFF unless HUMANOID_GAMEPAD_ENABLE
    # is set. When on, the controller drives arm → trigger-to-run → damp; see web/gamepad.py.
    gamepad = None
    if os.environ.get("HUMANOID_GAMEPAD_ENABLE"):
        try:
            from .gamepad import GamepadDeadman
            gamepad = GamepadDeadman(service)
            gamepad.start()
            _log.info("gamepad deadman enabled.")
        except Exception as exc:
            _log.error("gamepad deadman failed to start: %s", exc)

    _log.info("humanoid-control web up. contract: %d joints. config: %s. layout: %s (%d joints).",
              contract.num_joints, "loaded" if cfg else "MISSING",
              robot_layout.describe(), len(robot_layout.joint_order))
    try:
        yield
    finally:
        watchdog.cancel()
        if gamepad is not None:
            gamepad.stop()
        service.shutdown()
        await client.stop()


async def _watchdog_loop(service: ControlService) -> None:
    interval = 1.0 / _WATCHDOG_HZ
    try:
        while True:
            service.check_deadman_watchdog()
            service.watch_joint_dropouts()   # a dropout invalidates calibration
            service.check_offline_recovery()   # auto re-wake ESCs after a brownout/reset (idle only)
            await asyncio.sleep(interval)
    except asyncio.CancelledError:
        pass


app = FastAPI(title="humanoid-control web", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)


# ── auth endpoints (public) ──────────────────────────────────────────────────

class LoginBody(BaseModel):
    password: str


@app.get("/auth/status", response_model=None)
def auth_status():
    return {"success": True, "data": {"auth_required": auth_required()}, "error": None}


@app.post("/auth/login", response_model=None)
def auth_login(body: LoginBody, request: Request):
    if not auth_required():
        return {"success": True, "data": {"token": None, "auth_required": False}, "error": None}
    ip = request.client.host if request.client else "unknown"
    if login_locked(ip):
        return JSONResponse(
            {"success": False, "data": None, "error": "too many attempts — try again later"},
            status_code=429)
    token = issue_token(body.password)
    if token is None:
        record_login_failure(ip)
        time.sleep(0.5)
        return JSONResponse(
            {"success": False, "data": None, "error": "incorrect password"}, status_code=401)
    record_login_success(ip)
    return {"success": True, "data": {"token": token, "auth_required": True}, "error": None}


# Mutating control routes require a valid token (no-op unless a password is set).
app.include_router(control_router, dependencies=[Depends(require_auth)])


# ── websockets ───────────────────────────────────────────────────────────────

def _ws_authed(ws: WebSocket) -> bool:
    return not auth_required() or token_valid(ws.query_params.get("token"))


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket) -> None:
    if not _ws_authed(ws):
        await ws.close(code=1008)
        return
    await ws.accept()
    service: ControlService = ws.app.state.service
    interval = 1.0 / _TELEMETRY_HZ
    try:
        while True:
            try:
                await ws.send_json(service.telemetry_snapshot())
            except Exception:
                break
            await asyncio.sleep(interval)
    except WebSocketDisconnect:
        pass


@app.websocket("/ws/control")
async def ws_control(ws: WebSocket) -> None:
    """The deadman channel. The browser must send a heartbeat (any message) at least every
    ~250 ms while it wants motion to be allowed. Losing this socket, or letting it go silent
    during a motion session, trips E-STOP."""
    if not _ws_authed(ws):
        await ws.close(code=1008)
        return
    await ws.accept()
    service: ControlService = ws.app.state.service
    service.control_client_connected()
    try:
        while True:
            await ws.receive_text()   # any inbound frame is a heartbeat
            service.mark_heartbeat()
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        service.control_client_disconnected()


# ── static web UI (mounted last so API/WS win) ───────────────────────────────

if _WEB_DIR.is_dir():
    app.mount("/", StaticFiles(directory=str(_WEB_DIR), html=True), name="web")
    _log.info("serving web UI from %s", _WEB_DIR)
else:
    _log.info("no web UI build at %s — run `npm install && npm run build` in app/ to enable it",
              _WEB_DIR)
