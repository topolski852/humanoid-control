"""
Opt-in per-tick recorder for ``PolicyRunner.step()`` — captures the exact policy I/O each
control tick for offline analysis / replay (see the sim2real replay tests).

Enabled ONLY when the env var ``HUMANOID_RECORD_DIR`` is set; otherwise PolicyRunner never
constructs one, so there is zero overhead in the default path. Disk writes happen on a
background thread fed by a bounded queue, so the 25 Hz control loop never blocks on I/O
(and drops a frame rather than stalling if the queue ever backs up).

One JSONL file per runner instance: the first line is a ``_meta`` header (joint order +
obs layout), each subsequent line is one tick. Fields are floats/lists (JSON-safe).
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time

import numpy as np

_OBS_LAYOUT = ("command(3),base_ang_vel(3),projected_gravity(3),"
               "joint_pos_minus_default(12),joint_vel(12),prev_action(12)")


def _l(x):
    """numpy/array -> plain list of floats; passthrough for other JSON-safe values."""
    if isinstance(x, np.ndarray):
        return x.astype(float).tolist()
    if isinstance(x, (list, tuple)):
        return [float(v) for v in x]
    return x


class StepRecorder:
    def __init__(self, out_dir: str, joint_order):
        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, f"run_{time.time_ns()}_{os.getpid()}.jsonl")
        self._f = open(self.path, "w", buffering=1)  # line-buffered: each frame hits disk
        self._t0 = time.monotonic()
        self._q: "queue.Queue[dict]" = queue.Queue(maxsize=200000)
        self._stop = threading.Event()
        self._dropped = 0
        self._thread = threading.Thread(target=self._writer, name="step-recorder", daemon=True)
        self._thread.start()
        self._f.write(json.dumps({"_meta": {
            "joint_order": list(joint_order),
            "obs_layout": _OBS_LAYOUT,
            "policy_hz": 25,
        }}) + "\n")

    def record(self, *, base, joint_pos, joint_vel, obs, action, targets, command) -> None:
        """Called inside the control loop — non-blocking; drops the frame if the queue is full."""
        rec = {
            "t": time.monotonic() - self._t0,
            "base_valid": bool(getattr(base, "valid", True)),
            "projected_gravity": _l(base.projected_gravity),
            "base_ang_vel": _l(base.base_ang_vel),
            "joint_pos": _l(joint_pos),
            "joint_vel": _l(joint_vel),
            "obs": _l(obs),
            "action": _l(action),
            "targets": _l(targets),
            "command": _l(command),
        }
        try:
            self._q.put_nowait(rec)
        except queue.Full:
            self._dropped += 1  # never block the control loop

    def _writer(self) -> None:
        while not self._stop.is_set() or not self._q.empty():
            try:
                rec = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._f.write(json.dumps(rec) + "\n")
            except Exception:
                pass

    def close(self) -> None:
        self._stop.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if self._dropped:
                self._f.write(json.dumps({"_dropped_frames": self._dropped}) + "\n")
            self._f.flush()
            self._f.close()
        except Exception:
            pass


class ArmRunRecorder:
    """Per-tick flight recorder for an arm teleop session.

    Separate from :class:`StepRecorder` because the two capture different things: that one
    records policy I/O (45-dim observation, action, targets), this one records the teleop
    chain — what the sticks said, what the target became, what the IK asked for, and what the
    joints actually did. Sharing a schema would make both harder to read.

    Unlike StepRecorder this is ALWAYS ON for arm sessions rather than opt-in. The arm is new,
    it has no policy to fall back on, and "it did not move the way I expected" is a question
    you can only answer from the numbers after the fact.

    Same I/O discipline: a background writer fed by a bounded queue, so the 50 Hz loop never
    blocks on disk and drops a frame rather than stalling.
    """

    def __init__(self, out_dir: str, limb: str, joint_order, tuning=None):
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%S")
        self.path = os.path.join(out_dir, f"arm_{limb}_{stamp}_{os.getpid()}.jsonl")
        self._f = open(self.path, "w", buffering=1)
        self._t0 = time.monotonic()
        self._q: "queue.Queue[dict]" = queue.Queue(maxsize=200000)
        self._stop = threading.Event()
        self._dropped = 0
        self._ticks = 0
        self._engaged_ticks = 0
        self._limits_hit: dict[str, int] = {}
        self._thread = threading.Thread(target=self._writer, name="arm-recorder", daemon=True)
        self._thread.start()
        meta = {
            "kind": "arm_teleop",
            "limb": limb,
            "joint_order": list(joint_order),
            "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "sticks": "raw deflection [-1,1]: left_x, left_y, right_y, right_x",
            "frame_note": "spherical: left_x=azimuth left_y=elevation right_y=reach; "
                          "right_x=wrist (direct joint rate, not through the IK)",
        }
        if tuning is not None:
            meta["tuning"] = {k: getattr(tuning, k) for k in dir(tuning)
                              if not k.startswith("_") and isinstance(
                                  getattr(tuning, k), (int, float, str, bool))}
        self._f.write(json.dumps({"_meta": meta}) + "\n")

    def record(self, *, engaged: bool, run_gate: bool, sticks, joint_pos, joint_vel,
               joint_target=None, info=None, speed_mode="normal", xr=None) -> None:
        """Non-blocking. Records every tick, engaged or not — the pre-engage frames show what
        the sticks were doing before motion started, which is where a surprise often begins.

        ``xr`` is the Quest link status when a 6-DOF tracker is driving. Recorded on EVERY
        tick, because the questions that need it after the fact ("did it lag, or did the link
        stall?", "was the clutch anchored when it lurched?") are only answerable if the link
        state and the joint state are on the same timeline."""
        self._ticks += 1
        rec = {
            "t": round(time.monotonic() - self._t0, 4),
            "engaged": bool(engaged),
            "run_gate": bool(run_gate),
            "speed": speed_mode,
            "sticks": _l(sticks),
            "joint_pos": _l(joint_pos),
            "joint_vel": _l(joint_vel),
        }
        if joint_target is not None:
            rec["joint_target"] = _l(joint_target)
        if xr and xr.get("connected"):
            # Only the fields that answer "was the link healthy at this instant" — the whole
            # status dict every tick would bloat the log with constants.
            rec["xr"] = {k: xr.get(k) for k in
                         ("seq", "hz", "age_ms", "tracked", "trigger", "anchored", "gate",
                          "dropped", "reason")
                         if xr.get(k) is not None}
        if info:
            self._engaged_ticks += 1
            rec["hand"] = _l(info.get("hand"))
            rec["target"] = _l(info.get("target"))
            if info.get("desired") is not None:
                rec["desired"] = _l(info.get("desired"))
            if info.get("tracking_error_m") is not None:
                rec["tracking_error_m"] = info.get("tracking_error_m")
            rec["error_m"] = info.get("error_m")
            rec["clipped"] = info.get("clipped")
            rec["commanding"] = info.get("commanding")
            rec["frame"] = info.get("frame")
            if info.get("spherical"):
                rec["spherical"] = info["spherical"]
            for n in (info.get("at_limit") or []):
                self._limits_hit[n] = self._limits_hit.get(n, 0) + 1
            if info.get("at_limit"):
                rec["at_limit"] = list(info["at_limit"])
        try:
            self._q.put_nowait(rec)
        except queue.Full:
            self._dropped += 1

    def _writer(self) -> None:
        while not self._stop.is_set() or not self._q.empty():
            try:
                rec = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._f.write(json.dumps(rec) + "\n")
            except Exception:
                pass

    def close(self) -> None:
        self._stop.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self._f.write(json.dumps({"_summary": {
                "duration_s": round(time.monotonic() - self._t0, 2),
                "ticks": self._ticks,
                "engaged_ticks": self._engaged_ticks,
                "dropped_frames": self._dropped,
                # Ticks spent pinned against each joint's limit — usually the first place to
                # look when the arm "would not go where I pointed it".
                "ticks_at_limit": dict(sorted(self._limits_hit.items(),
                                              key=lambda kv: -kv[1])),
            }}) + "\n")
            self._f.flush()
            self._f.close()
        except Exception:
            pass
