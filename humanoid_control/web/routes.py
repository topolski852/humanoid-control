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
from ..layout import (LIMB_BUS, LIMB_JOINTS, LIMB_LABEL, LIMB_ORDER, RobotLayout,
                      default_layout_path)
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


# Trained-policy bundles are SELF-HOSTED in this repo's policies/ dir: each bundle is a folder
# with policy.onnx (+ leg_policy_contract.json), copied in from a humanoid-policy export once
# that policy is finalized. Deliberately NOT read from the sibling humanoid-policy working tree —
# that checkout can sit on any branch, so pulling from it makes the robot's behaviour depend on
# someone else's git state. Override with the env var.
def _policy_dir() -> Path:
    return Path(os.environ.get("HUMANOID_POLICY_DIR", str(REPO_ROOT / "policies")))


# The policy that runs by default (pre-selected in the UI). Others are listed too.
_DEFAULT_POLICY = "walk"


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


class SelectBody(BaseModel):
    kind: str                          # "hold" | "policy" | "manual"
    checkpoint: str | None = None


class InputSourceBody(BaseModel):
    source: str                        # "xbox" | "quest" | "web"


class DeadmanArmBody(BaseModel):
    kind: str | None = None            # None → use the previously selected session
    checkpoint: str | None = None


# ── read-only ────────────────────────────────────────────────────────────────

@router.get("/api/status", response_model=None)
def status(request: Request):
    return _ok(_service(request).telemetry_snapshot())


class LayoutBody(BaseModel):
    enabled: list[str]                 # limb names: left_leg | right_leg | left_arm | right_arm
    imu_expected: bool = True


def _layout_payload(svc: ControlService) -> dict:
    """The layout plus everything the Settings tab needs to render it: the limb catalog, which
    joints each limb owns, its CAN bus, and whether the loaded robot config can address them."""
    lay = svc.layout
    known = set(svc.robot_config.joints) if svc.robot_config else set()
    return {
        "enabled": list(lay.enabled),
        "imu_expected": lay.imu_expected,
        "describe": lay.describe(),
        "has_both_legs": lay.has_both_legs,
        "path": str(lay.source or default_layout_path()),
        "limbs": [
            {
                "id": limb,
                "label": LIMB_LABEL[limb],
                "bus": LIMB_BUS[limb],
                "joints": list(LIMB_JOINTS[limb]),
                "enabled": limb in lay.enabled,
                # Joints this limb needs that the robot config has never heard of. A limb with
                # any of these cannot be enabled — surfaced so the UI can say why.
                "unknown_joints": [j for j in LIMB_JOINTS[limb] if known and j not in known],
            }
            for limb in LIMB_ORDER
        ],
    }


@router.get("/api/layout", response_model=None)
def get_layout(request: Request):
    return _ok(_layout_payload(_service(request)))


@router.put("/api/layout", response_model=None)
def put_layout(request: Request, body: LayoutBody):
    """Set which limbs are attached and persist it to the machine-local layout file.

    Refused while a session is live. Saving is best-effort-visible: if the file cannot be
    written the layout still applies to this process, but the response says so rather than
    pretending the setting will survive a restart.
    """
    svc = _service(request)
    try:
        new = RobotLayout(
            enabled=RobotLayout().with_enabled(body.enabled).enabled,
            imu_expected=body.imu_expected,
            robot_name=svc.layout.robot_name,
            source=svc.layout.source,
        )
    except ValueError as exc:
        return _err(str(exc), 400)
    try:
        svc.set_layout(new)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    saved, save_error = True, None
    try:
        svc.layout.save()
    except Exception as exc:
        saved, save_error = False, str(exc)
        _log_save_failure(exc)
    return _ok({**_layout_payload(svc), "saved": saved, "save_error": save_error})


def _log_save_failure(exc: Exception) -> None:
    import logging
    logging.getLogger(__name__).error("failed to persist robot layout: %s", exc)


@router.get("/api/contract", response_model=None)
def contract(request: Request):
    """Frame-critical constants the robot visualizer needs, straight from the live contract.

    The wireframe's *geometry* is vendored into the JS bundle
    (``app/src/data/viz_kinematics.json``), but these four values are exactly the ones whose
    drift would make the visualizer confidently draw the wrong robot — which is the bug class
    it exists to catch. So they are served live and the app cross-checks ``policy_frame_sign``
    against the copy recorded in the bundled model before it will draw anything.

    ``default_pose`` and ``limits`` are DEVICE frame, matching telemetry ``joints[].position``.

    Every array here is index-aligned to the CONFIGURED joints (``ControlService.joints``), which
    is the same order telemetry ``joints[]`` uses — so the visualizer can index one against the
    other without a name lookup.

    Joints outside the leg policy contract (the arms) have no trained frame and no trained
    default pose. They are served with ``policy_frame_sign = +1`` and ``default_pose = null``,
    which means the visualizer draws their RAW device angle. That is deliberate: any disagreement
    between the drawing and the physical arm is then a real finding about the device frame rather
    than something a fudge factor has already hidden.
    """
    svc = _service(request)
    c = svc.contract
    contract_joints = set(c.joint_order)
    limits = svc.joint_limits

    order, sign, default, lower, upper, in_contract = [], [], [], [], [], []
    for name in svc.joints:
        order.append(name)
        lo, hi = limits[name]
        lower.append(lo)
        upper.append(hi)
        if name in contract_joints:
            i = c.index_of(name)
            sign.append(float(c.policy_frame_sign[i]))
            default.append(float(c.default_pose[i]))
            in_contract.append(True)
        else:
            sign.append(1.0)
            default.append(None)
            in_contract.append(False)

    return _ok({
        "joint_order": order,
        "policy_frame_sign": sign,
        "default_pose": default,
        "limits": {"lower": lower, "upper": upper},
        # True where the joint is part of the trained leg policy contract; False for joints
        # that are merely attached hardware (the arms).
        "in_contract": in_contract,
    })


@router.get("/api/policies", response_model=None)
def policies(request: Request):
    """Detect available trained policies. Prefers the deploy-bundle layout
    (<dir>/<name>/policy.onnx + leg_policy_contract.json); also lists loose weight
    files as a fallback. Marks the default policy (walk) so the UI pre-selects it."""
    d = _policy_dir()
    found = []
    if d.is_dir():
        for sub in sorted(p for p in d.iterdir() if p.is_dir()):
            onnx = sub / "policy.onnx"
            if onnx.is_file():
                contract = sub / "leg_policy_contract.json"
                found.append({"name": sub.name, "path": str(onnx),
                              "contract": str(contract) if contract.is_file() else None})
        for p in sorted(d.iterdir()):   # fallback: loose weight files in the dir
            if p.is_file() and p.suffix.lower() in _POLICY_EXTS:
                found.append({"name": p.name, "path": str(p), "contract": None})
    names = {f["name"] for f in found}
    default = _DEFAULT_POLICY if _DEFAULT_POLICY in names else (found[0]["name"] if found else None)
    return _ok({"dir": str(d), "default": default, "policies": found})


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


# ── gamepad deadman session (hold-to-run) ─────────────────────────────────────
# The gamepad drives these live (A=arm, triggers=run-gate, sticks=command); these routes let
# the web UI pick the session (e.g. the walk checkpoint) and arm/disarm without the controller.

@router.post("/api/deadman/select", response_model=None)
def deadman_select(request: Request, body: SelectBody):
    try:
        _service(request).select_session(body.kind, body.checkpoint)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(_service(request).telemetry_snapshot())


@router.post("/api/input_source", response_model=None)
def set_input_source(request: Request, body: InputSourceBody):
    """Pick what drives the robot (the Control method card). Refused mid-session."""
    try:
        _service(request).set_input_source(body.source)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(_service(request).telemetry_snapshot())


@router.post("/api/deadman/arm", response_model=None)
async def deadman_arm(request: Request, body: DeadmanArmBody):
    svc = _service(request)
    try:
        await _blocking(lambda: svc.arm_deadman(kind=body.kind, checkpoint=body.checkpoint))
    except ControlError as exc:
        return _err(str(exc), exc.status)
    except Exception as exc:
        return _err(f"arm failed: {exc}", 502)
    return _ok(svc.telemetry_snapshot())


@router.post("/api/deadman/disarm", response_model=None)
async def deadman_disarm(request: Request):
    await _blocking(_service(request).disarm_deadman)
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


@router.post("/api/clear_faults", response_model=None)
async def clear_faults(request: Request):
    """Clear firmware errors on all joints + release a latched E-STOP (recover without reconnect)."""
    try:
        await _blocking(_service(request).clear_faults)
    except ControlError as exc:
        return _err(str(exc), exc.status)
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


@router.post("/api/calibrate/arm/{limb}", response_model=None)
async def cal_arm(request: Request, limb: str):
    """Zero one arm from the T-pose the operator is holding.

    The arms have no hardstops, so the per-joint capture flow does not apply to them. Blocking
    (it samples the hold for ~1.5 s and does SDO round-trips), so it runs off the event loop.
    """
    try:
        data = await _blocking(_service(request).teach_arm_zero, limb)
    except ControlError as exc:
        return _err(str(exc), exc.status)
    except Exception as exc:
        return _err(f"arm calibration failed: {exc}", 500)
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


def _poses_payload(svc: ControlService) -> dict:
    """Poses resolved against the CONFIGURED joints, so an arm-only machine does not report a
    pile of skipped leg joints it was never going to drive."""
    data = load_poses()
    joints = tuple(svc.joints)
    out = []
    for name in pose_names(data):
        targets, skipped = resolve_pose(data, name, joints)         # rad
        out.append({
            "name": name,
            "joints": {k: v for k, v in data["poses"][name].items() if not k.startswith("_")},
            "resolved_deg": {n: v / DEG for n, v in targets.items()},
            "skipped": [s.replace("_joint", "") for s in skipped],
        })
    return {"poses": out, "always_skip": data.get("_meta", {}).get("always_skip", [])}


@router.get("/api/poses", response_model=None)
def get_poses(request: Request):
    return _ok(_poses_payload(_service(request)))


@router.put("/api/poses/{name}", response_model=None)
def put_pose(request: Request, name: str, body: PoseBody):
    try:
        save_pose(name, body.joints)
    except ValueError as exc:
        return _err(str(exc), 400)
    return _ok(_poses_payload(_service(request)))


@router.delete("/api/poses/{name}", response_model=None)
def del_pose(request: Request, name: str):
    try:
        delete_pose(name)
    except KeyError as exc:
        return _err(str(exc), 404)
    return _ok(_poses_payload(_service(request)))


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
        # capture-and-hold holds the RAW live pose (clamp=False): never force a joint into its
        # limits — hold exactly where it is, even if an uncalibrated reading is out of range.
        await _blocking(lambda: svc.start_manual_hold(targets, ramp=body.ramp,
                                                      seconds=body.seconds, clamp=False))
    except ControlError as exc:
        return _err(str(exc), exc.status)
    return _ok(svc.telemetry_snapshot())


@router.post("/api/manual/goto", response_model=None)
async def manual_goto(request: Request, body: GotoBody):
    svc = _service(request)
    if body.pose:
        try:
            targets, _ = resolve_pose(load_poses(), body.pose, tuple(svc.joints))   # rad
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
