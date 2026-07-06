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
from ..poses import DEG, delete_pose, load_poses, pose_names, resolve_pose, save_pose
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


class CaptureBody(BaseModel):
    which: str   # "lower" | "upper"


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


# ── position_offset calibration ───────────────────────────────────────────────
# Flow per joint: start (offset→0, IDLE) → move to lower stop → capture lower →
# move to upper stop → capture upper → apply (compute + write offset). No commanded
# motion — the user hand-moves the IDLE joint.

def _cal_result(request: Request, data: dict):
    return _ok({**_service(request).telemetry_snapshot(), "cal": data})


@router.post("/api/calibrate/{joint}/start", response_model=None)
async def cal_start(request: Request, joint: str):
    try:
        data = await _blocking(_service(request).cal_start, joint)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _cal_result(request, data)


@router.post("/api/calibrate/{joint}/capture", response_model=None)
async def cal_capture(request: Request, joint: str, body: CaptureBody):
    try:
        data = await _blocking(_service(request).cal_capture, joint, body.which)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _cal_result(request, data)


@router.post("/api/calibrate/{joint}/apply", response_model=None)
async def cal_apply(request: Request, joint: str):
    try:
        data = await _blocking(_service(request).cal_apply, joint)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _cal_result(request, data)


@router.post("/api/calibrate/{joint}/reset", response_model=None)
def cal_reset(request: Request, joint: str):
    data = _service(request).cal_reset(joint)
    return _cal_result(request, data)


@router.post("/api/calibrate/complete", response_model=None)
async def cal_complete(request: Request):
    """Operator override: mark all joints calibrated if every one is within limits."""
    try:
        data = await _blocking(_service(request).cal_mark_complete)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _cal_result(request, data)


# ── manual control: saved poses + capture-and-hold ────────────────────────────

class PoseBody(BaseModel):
    joints: dict[str, float]                 # key (joint type or name) → degrees


class ManualHoldBody(BaseModel):
    ramp: float = 4.0
    seconds: float | None = None


class GotoBody(BaseModel):
    pose: str | None = None                  # a saved pose name …
    target: dict[str, float] | None = None   # … or an explicit {joint_name: degrees}
    ramp: float = 4.0
    seconds: float | None = None


def _poses_payload() -> dict:
    data = load_poses()
    out = []
    for name in pose_names(data):
        targets, skipped = resolve_pose(data, name)                 # rad
        out.append({
            "name": name,
            "joints": {k: v for k, v in data["poses"][name].items() if not k.startswith("_")},
            "resolved_deg": {n: v / DEG for n, v in targets.items()},
            "skipped": [s.replace("_joint", "") for s in skipped],
        })
    return {"poses": out, "always_skip": data.get("_meta", {}).get("always_skip", [])}


@router.get("/api/poses", response_model=None)
def get_poses(request: Request):
    return _ok(_poses_payload())


@router.put("/api/poses/{name}", response_model=None)
def put_pose(request: Request, name: str, body: PoseBody):
    try:
        save_pose(name, body.joints)
    except ValueError as exc:
        return _err(str(exc), 400)
    return _ok(_poses_payload())


@router.delete("/api/poses/{name}", response_model=None)
def del_pose(request: Request, name: str):
    try:
        delete_pose(name)
    except KeyError as exc:
        return _err(str(exc), 404)
    return _ok(_poses_payload())


@router.get("/api/pose/current", response_model=None)
def current_pose(request: Request):
    try:
        rad = _service(request).current_pose_rad()
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok({"joints_deg": {n: v / DEG for n, v in rad.items()}})


@router.post("/api/manual/capture_hold", response_model=None)
async def manual_capture_hold(request: Request, body: ManualHoldBody):
    svc = _service(request)
    try:
        targets = await _blocking(svc.current_pose_rad)             # all 12, rad
        await _blocking(lambda: svc.start_manual_hold(targets, ramp=body.ramp, seconds=body.seconds))
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(svc.telemetry_snapshot())


@router.post("/api/manual/goto", response_model=None)
async def manual_goto(request: Request, body: GotoBody):
    svc = _service(request)
    if body.pose:
        try:
            targets, _ = resolve_pose(load_poses(), body.pose)      # rad
        except KeyError as exc:
            return _err(str(exc), 404)
    elif body.target:
        targets = {n: float(v) * DEG for n, v in body.target.items()}
    else:
        return _err("provide 'pose' or 'target'", 400)
    try:
        await _blocking(lambda: svc.start_manual_hold(targets, ramp=body.ramp, seconds=body.seconds))
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(svc.telemetry_snapshot())
