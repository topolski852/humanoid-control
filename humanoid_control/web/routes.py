"""
REST routes for the web control layer. All responses use the {success, data, error}
envelope (matches the frontend ``app/src/api.js``).

Read-only routes are open; every mutating route is gated by ``require_auth`` (a no-op unless
``HUMANOID_WEB_PASSWORD`` is set) at include time in ``server.py``. Blocking robot calls
(connect/disconnect/stop/load-policy) run in a threadpool so the event loop stays responsive.
"""
from __future__ import annotations

import asyncio
import os
from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from ..config import REPO_ROOT
from .service import ControlError, ControlService

router = APIRouter(tags=["control"])

_POLICY_EXTS = (".onnx", ".pt", ".pth", ".jit")


def _ok(data: object = None) -> dict:
    return {"success": True, "data": data, "error": None}


def _err(msg: str, status: int = 400) -> JSONResponse:
    return JSONResponse({"success": False, "data": None, "error": msg}, status_code=status)


def _service(request: Request) -> ControlService:
    return request.app.state.service


async def _blocking(fn, *args):
    """Run a blocking ControlService method in the default threadpool."""
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: fn(*args))


def _policy_dir() -> Path:
    return Path(os.environ.get("HUMANOID_POLICY_DIR", str(REPO_ROOT / "checkpoints")))


# ── request bodies ───────────────────────────────────────────────────────────

class HoldBody(BaseModel):
    ramp: float = 5.0
    seconds: float | None = None


class RunBody(BaseModel):
    checkpoint: str
    command: list[float] | None = None
    ramp: float = 5.0
    seconds: float | None = None


# ── read-only ────────────────────────────────────────────────────────────────

@router.get("/api/status", response_model=None)
def status(request: Request):
    return _ok(_service(request).telemetry_snapshot())


@router.get("/api/policies", response_model=None)
def policies(request: Request):
    d = _policy_dir()
    found = []
    if d.is_dir():
        for p in sorted(d.iterdir()):
            if p.is_file() and p.suffix.lower() in _POLICY_EXTS:
                found.append({"name": p.name, "path": str(p)})
    return _ok({"dir": str(d), "policies": found})


# ── connection lifecycle ─────────────────────────────────────────────────────

@router.post("/api/connect", response_model=None)
async def connect(request: Request):
    try:
        await _blocking(_service(request).connect)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    except Exception as exc:
        return _err(f"connect failed: {exc}", 502)
    return _ok(_service(request).telemetry_snapshot())


@router.post("/api/disconnect", response_model=None)
async def disconnect(request: Request):
    await _blocking(_service(request).disconnect)
    return _ok(_service(request).telemetry_snapshot())


# ── arming ───────────────────────────────────────────────────────────────────

@router.post("/api/arm", response_model=None)
def arm(request: Request):
    try:
        _service(request).arm()
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(_service(request).telemetry_snapshot())


@router.post("/api/disarm", response_model=None)
def disarm(request: Request):
    _service(request).disarm()
    return _ok(_service(request).telemetry_snapshot())


# ── motion ───────────────────────────────────────────────────────────────────

@router.post("/api/hold", response_model=None)
def hold(request: Request, body: HoldBody):
    try:
        _service(request).start_hold(ramp=body.ramp, seconds=body.seconds)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(_service(request).telemetry_snapshot())


@router.post("/api/run_policy", response_model=None)
async def run_policy(request: Request, body: RunBody):
    svc = _service(request)
    try:
        # load_policy can block (onnx/torch import + load) — off the event loop.
        await _blocking(
            lambda: svc.start_policy(
                checkpoint=body.checkpoint, command=body.command,
                ramp=body.ramp, seconds=body.seconds,
            )
        )
    except ControlError as exc:
        return _err(str(exc), exc.status)
    except Exception as exc:
        return _err(f"run_policy failed: {exc}", 400)
    return _ok(svc.telemetry_snapshot())


@router.post("/api/stop", response_model=None)
async def stop(request: Request):
    await _blocking(lambda: _service(request).stop(wait=True))
    return _ok(_service(request).telemetry_snapshot())


@router.post("/api/estop", response_model=None)
def estop(request: Request):
    # Must be instant and never blocked — fires the priority port (9002).
    _service(request).trigger_estop("web")
    return _ok(_service(request).telemetry_snapshot())
