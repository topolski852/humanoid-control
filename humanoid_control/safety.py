"""
Safety scaffolding: E-stop, keyboard kill, and slow ramp-to-pose.

Design rules (from the build brief, non-negotiable):
- Wire E-stop (priority port 9002) and a keyboard kill from day one.
- On start, ramp slowly to the default/crouch pose — never step to a target.
- Always clamp targets to each joint's position limits (done by ActionMapper / contract).
"""
from __future__ import annotations

import signal
import sys
import threading
import time
from typing import Callable

import numpy as np


class EstopController:
    """Single choke-point for stopping the robot.

    ``trigger()`` fires the daemon priority E-stop (port 9002 → all joints DAMPING at the
    next 200 Hz tick) and latches ``fired``. Optionally installs a SIGINT handler and a
    background keyboard listener so a human can kill motion instantly.
    """

    def __init__(self, client, *, install_sigint: bool = True, keyboard: bool = True):
        self._client = client
        self._fired = threading.Event()
        self._extra_callbacks: list[Callable[[], None]] = []
        if install_sigint:
            signal.signal(signal.SIGINT, self._on_sigint)
        self._kb_thread: threading.Thread | None = None
        if keyboard:
            self._start_keyboard_listener()

    @property
    def fired(self) -> bool:
        return self._fired.is_set()

    def add_callback(self, cb: Callable[[], None]) -> None:
        self._extra_callbacks.append(cb)

    def trigger(self, reason: str = "manual") -> None:
        if self._fired.is_set():
            return
        self._fired.set()
        print(f"\n*** E-STOP ({reason}) → all joints DAMPING ***", file=sys.stderr, flush=True)
        try:
            self._client.estop_all()
        except Exception as exc:  # never let estop path raise
            print(f"    estop_all() error: {exc}", file=sys.stderr, flush=True)
        for cb in self._extra_callbacks:
            try:
                cb()
            except Exception:
                pass

    # --- internals -------------------------------------------------------
    def _on_sigint(self, *_):
        self.trigger("SIGINT")

    def _start_keyboard_listener(self) -> None:
        if not sys.stdin or not sys.stdin.isatty():
            return  # no interactive terminal; rely on SIGINT

        def _loop():
            try:
                for line in sys.stdin:
                    if self._fired.is_set():
                        return
                    if line.strip().lower() in ("", "q", "stop", "kill", "e"):
                        self.trigger("keyboard")
                        return
            except Exception:
                pass

        self._kb_thread = threading.Thread(target=_loop, daemon=True)
        self._kb_thread.start()
        print("[safety] keyboard kill armed — press ENTER or 'q' to E-stop.", file=sys.stderr)


def ramp_to_pose(
    *,
    start: np.ndarray,
    goal: np.ndarray,
    send: Callable[[np.ndarray], None],
    duration_s: float,
    rate_hz: float,
    should_abort: Callable[[], bool] = lambda: False,
) -> bool:
    """Linearly interpolate from ``start`` to ``goal`` and stream targets via ``send``.

    Returns True if it completed, False if ``should_abort`` fired mid-ramp. Callers must
    have already clamped ``goal`` to position limits; intermediate points stay within the
    convex hull of start/goal so they remain in-range too.
    """
    start = np.asarray(start, dtype=np.float32)
    goal = np.asarray(goal, dtype=np.float32)
    n_steps = max(1, int(round(duration_s * rate_hz)))
    dt = 1.0 / rate_hz
    for i in range(1, n_steps + 1):
        if should_abort():
            return False
        alpha = i / n_steps
        send((1.0 - alpha) * start + alpha * goal)
        time.sleep(dt)
    return True
