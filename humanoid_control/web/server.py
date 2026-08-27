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
import threading
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
_QUEST_WATCHDOG_HZ = 20   # must be well inside xr.STALL_S (200 ms) to catch a stall promptly

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

    # Quest 3 WebXR bridge. OFF unless HUMANOID_QUEST_ENABLE is set, and the runtime is fully
    # functional without it — the headset is an input device, not part of the robot.
    quest_task, tls = None, None
    if os.environ.get("HUMANOID_QUEST_ENABLE"):
        try:
            from .xr import QuestSource
            service.quest = QuestSource(service)
            quest_task = asyncio.create_task(_quest_loop(service))
            # Started HERE, not in __main__, for two reasons: app.state.service is populated
            # by the time it can serve a request, and it shares this app object so the
            # lifespan does not run twice (which would build a second DaemonClient and a
            # second gamepad thread fighting the first).
            tls = _start_tls_listener(os.environ.get("HUMANOID_WEB_HOST", "0.0.0.0"))
            _log.info("quest XR bridge enabled.")
        except Exception as exc:
            _log.error("quest bridge failed to start: %s", exc)

    _log.info("humanoid-control web up. contract: %d joints. config: %s. layout: %s (%d joints).",
              contract.num_joints, "loaded" if cfg else "MISSING",
              robot_layout.describe(), len(robot_layout.joint_order))
    try:
        yield
    finally:
        watchdog.cancel()
        if quest_task is not None:
            quest_task.cancel()
        if tls is not None:
            tls.should_exit = True
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


def _quest_cert_paths() -> tuple[Path, Path]:
    base = Path(os.environ.get("XDG_CONFIG_HOME") or "~/.config").expanduser() / "humanoid-control"
    cert = Path(os.environ.get("HUMANOID_QUEST_CERT") or base / "cert.pem").expanduser()
    key = Path(os.environ.get("HUMANOID_QUEST_KEY") or base / "key.pem").expanduser()
    return cert, key


def _start_tls_listener(host: str):
    """A SECOND uvicorn Server on this same app object, over TLS, for the Quest.

    WebXR only runs in a secure context, so the headset needs HTTPS. Moving the whole app to
    TLS would break every existing bookmark and the deploy/ unit, so :8000 stays plain HTTP
    and the headset uses https://<robot>:8443/xr instead.

    ``lifespan="off"`` is essential: the primary server already ran it, and running it again
    would construct a second DaemonClient, a second ControlService and a second gamepad
    thread — two runtimes fighting over one robot.

    Generate the cert once::

        openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \\
            -keyout ~/.config/humanoid-control/key.pem \\
            -out    ~/.config/humanoid-control/cert.pem -subj "/CN=humanoid"
    """
    import uvicorn

    cert, key = _quest_cert_paths()
    if not (cert.is_file() and key.is_file()):
        _log.error("HUMANOID_QUEST_ENABLE is set but no TLS cert/key at %s / %s — the Quest "
                   "page will NOT be reachable. Generate one (see _start_tls_listener) or "
                   "unset HUMANOID_QUEST_ENABLE.", cert, key)
        return None
    port = int(os.environ.get("HUMANOID_QUEST_TLS_PORT", "8443"))
    server = uvicorn.Server(uvicorn.Config(
        app, host=host, port=port, log_level="warning", lifespan="off",
        ssl_certfile=str(cert), ssl_keyfile=str(key),
    ))
    threading.Thread(target=server.run, name="tls-listener", daemon=True).start()
    _log.info("TLS listener on https://%s:%d — open /xr on the Quest.", host, port)
    return server


async def _quest_hud_loop(ws: WebSocket, service: ControlService) -> None:
    """Push the in-headset HUD at ~8 Hz.

    Separate from the receive loop on purpose: the receive loop feeds the deadman, and a
    HUD send that blocks (congested link, headset throttling the tab) must never delay it.
    A failed send ends this task quietly — losing the display is bad, losing the safety
    path would be worse.
    """
    sent = 0
    errs = 0
    try:
        while True:
            try:
                frame = service.quest.hud_frame()
                if frame:
                    await ws.send_json(frame)
                    sent += 1
                    if sent == 1:
                        _log.info("quest: HUD push started")
            except Exception as exc:
                # Log and KEEP GOING. Returning here meant one transient error blanked the
                # operator's only feedback channel for the rest of the session — which is
                # exactly what a seq=None bug in the calibration did.
                errs += 1
                if errs in (1, 5) or errs % 100 == 0:
                    _log.warning("quest: HUD push error #%d (%s: %s)",
                                 errs, type(exc).__name__, exc)
            await asyncio.sleep(0.125)
    except asyncio.CancelledError:
        pass


async def _quest_loop(service: ControlService) -> None:
    """Notice Quest SILENCE. The websocket handler only runs when frames arrive, so a link
    that goes quiet is invisible from the receive path — which is the whole failure this
    bridge exists to detect. Polled well inside the 200 ms stall threshold so the arm stops
    within roughly one threshold, not two."""
    interval = 1.0 / _QUEST_WATCHDOG_HZ
    try:
        while True:
            try:
                if service.quest is not None:
                    service.quest.tick()
            except Exception:
                _log.exception("quest watchdog tick failed")   # never kill the watchdog
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


@app.websocket("/ws/xr")
async def ws_xr(ws: WebSocket) -> None:
    """The Quest link. The headset sends one JSON frame per XR render frame (~60 Hz); each
    carries a monotonic `seq` that IS the liveness signal — an open socket carrying nothing
    is exactly the failure a timestamp-free transport cannot see.

    Closing this socket while the Quest is the deadman of record for a live session is
    controller loss and E-STOPs, the same as unplugging the gamepad."""
    if not _ws_authed(ws):
        await ws.close(code=1008)
        return
    service: ControlService = ws.app.state.service
    if service.quest is None:
        await ws.close(code=1011)      # bridge not enabled — don't accept and look healthy
        return
    await ws.accept()
    service.quest.attach()
    # The HUD is the operator's ONLY feedback channel once the headset is on — passthrough
    # cameras cannot resolve monitor text. Pushed on its own task so a slow or wedged HUD
    # send can never stall the frame-receive loop that the deadman depends on.
    hud_task = asyncio.create_task(_quest_hud_loop(ws, service))
    try:
        while True:
            msg = await ws.receive_json()
            service.quest.on_frame(msg)
    except WebSocketDisconnect:
        pass
    except Exception:
        # A malformed payload that breaks receive_json must not look like a clean close:
        # fall through to detach(), which E-STOPs if a live session depended on this link.
        _log.warning("quest: websocket error — treating as disconnect.", exc_info=True)
    finally:
        hud_task.cancel()
        service.quest.detach()


# ── static web UI (mounted last so API/WS win) ───────────────────────────────

if _WEB_DIR.is_dir():
    app.mount("/", StaticFiles(directory=str(_WEB_DIR), html=True), name="web")
    _log.info("serving web UI from %s", _WEB_DIR)
else:
    _log.info("no web UI build at %s — run `npm install && npm run build` in app/ to enable it",
              _WEB_DIR)
