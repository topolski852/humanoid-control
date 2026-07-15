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
