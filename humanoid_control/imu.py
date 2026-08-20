"""
WitMotion serial IMU reader + base-state source.

The robot's IMU is a Hiwonder IM10A (WitMotion-family sensor) on a USB-serial port
(``/dev/ttyUSB0``, CH340). It streams the standard WitMotion protocol: 11-byte frames
``[0x55, type, d0..d7 (4x int16 LE), checksum]``, checksum = sum(first 10 bytes) & 0xFF.

Frame types we care about:
    0x51 accel        raw/32768 * 16   → g
    0x52 gyro         raw/32768 * 2000 → deg/s
    0x53 euler        raw/32768 * 180  → deg   (roll, pitch, yaw)   [diagnostics only]
    0x59 quaternion   raw/32768        → (w, x, y, z)

This module turns that stream into the two quantities the policy consumes
(:class:`~humanoid_control.base_state.BaseState`): ``projected_gravity`` and
``base_ang_vel``. It is host-agnostic — it runs the same on the trainer PC (bring-up /
verification) and on the robot PC (deployment). If/when the IMU moves into the daemon
(DAEMON_SPEC §9), the parser here is the reusable core.

⚠️ Conventions to pin at integration time (same rigor as the sim↔real joint contract):
quaternion order/handedness and the IMU→base ``mounting_rotation``. Use scripts/imu_monitor.py
to validate signs by physically tilting the robot before trusting the balance loop.
"""
from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass

import numpy as np

from .base_state import BaseState, BaseStateSource, quat_rotate_inverse

# WitMotion frame constants.
_HEADER = 0x55
_FRAME_LEN = 11
ACCEL, GYRO, EULER, QUAT = 0x51, 0x52, 0x53, 0x59

_ACCEL_SCALE = 16.0 / 32768.0       # → g
_GYRO_SCALE = 2000.0 / 32768.0      # → deg/s
_ANGLE_SCALE = 180.0 / 32768.0      # → deg
_QUAT_SCALE = 1.0 / 32768.0         # → unit quaternion component
_DEG2RAD = np.pi / 180.0
_GRAVITY_WORLD = np.array([0.0, 0.0, -1.0], dtype=np.float32)


@dataclass
class ImuSample:
    """Latest decoded values, each in physical units (or None if not yet seen)."""
    accel_g: np.ndarray | None = None        # (3,) g, IMU frame
    gyro_dps: np.ndarray | None = None        # (3,) deg/s, IMU frame
    euler_deg: np.ndarray | None = None       # (3,) roll/pitch/yaw deg  (diagnostics)
    quat_wxyz: np.ndarray | None = None       # (4,) unit quaternion (w,x,y,z)
    stamp: float = 0.0                        # monotonic time of last update


def parse_frames(buf: bytearray, sample: ImuSample) -> int:
    """Consume complete WitMotion frames from ``buf`` into ``sample``.

    Mutates ``buf`` in place (drops parsed/garbage bytes) and updates ``sample`` fields.
    Returns the number of valid frames parsed. Caller supplies the monotonic ``now`` via
    ``sample.stamp`` after this returns (kept out of here so the parser stays pure/testable).
    """
    n = 0
    while len(buf) >= _FRAME_LEN:
        if buf[0] != _HEADER:
            del buf[0]
            continue
        frame = buf[:_FRAME_LEN]
        if (sum(frame[:10]) & 0xFF) != frame[10]:
            del buf[0]           # bad checksum → resync one byte at a time
            continue
        t = frame[1]
        d = struct.unpack("<hhhh", frame[2:10])
        if t == ACCEL:
            sample.accel_g = np.array(d[:3], dtype=np.float32) * _ACCEL_SCALE
        elif t == GYRO:
            sample.gyro_dps = np.array(d[:3], dtype=np.float32) * _GYRO_SCALE
        elif t == EULER:
            sample.euler_deg = np.array(d[:3], dtype=np.float32) * _ANGLE_SCALE
        elif t == QUAT:
            sample.quat_wxyz = np.array(d[:4], dtype=np.float32) * _QUAT_SCALE
        del buf[:_FRAME_LEN]
        n += 1
    return n


class WitMotionReader:
    """Background thread that keeps the latest :class:`ImuSample` fresh.

    Uses pyserial if available; the serial port must already be opened at the right baud.
    Thread-safe: :meth:`latest` returns a snapshot copy under a lock.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 9600, timeout: float = 0.05):
        self._port_name = port
        self._baud = baud
        self._timeout = timeout
        self._ser = None
        self._buf = bytearray()
        self._sample = ImuSample()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frames_total = 0

    def start(self) -> "WitMotionReader":
        import serial  # pyserial; imported lazily so the module loads without it
        self._ser = serial.Serial(self._port_name, self._baud, timeout=self._timeout)
        self._ser.reset_input_buffer()
        self._thread = threading.Thread(target=self._run, name="witmotion-imu", daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                time.sleep(0.01)
                continue
            if not chunk:
                continue
            local = ImuSample()
            self._buf += chunk
            n = parse_frames(self._buf, local)
            if n:
                now = time.monotonic()
                with self._lock:
                    self._frames_total += n
                    # Merge: only overwrite fields this batch actually refreshed.
                    if local.accel_g is not None: self._sample.accel_g = local.accel_g
                    if local.gyro_dps is not None: self._sample.gyro_dps = local.gyro_dps
                    if local.euler_deg is not None: self._sample.euler_deg = local.euler_deg
                    if local.quat_wxyz is not None: self._sample.quat_wxyz = local.quat_wxyz
                    self._sample.stamp = now

    def latest(self) -> ImuSample:
        with self._lock:
            s = self._sample
            return ImuSample(
                accel_g=None if s.accel_g is None else s.accel_g.copy(),
                gyro_dps=None if s.gyro_dps is None else s.gyro_dps.copy(),
                euler_deg=None if s.euler_deg is None else s.euler_deg.copy(),
                quat_wxyz=None if s.quat_wxyz is None else s.quat_wxyz.copy(),
                stamp=s.stamp,
            )

    @property
    def frames_total(self) -> int:
        with self._lock:
            return self._frames_total

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass


class SerialImuBaseState(BaseStateSource):
    """:class:`BaseStateSource` backed by a live WitMotion sensor.

    ``projected_gravity`` comes from the fused quaternion (``quat_rotate_inverse(q, world_gravity)``,
    matching the trainer/base_state convention), ``base_ang_vel`` from the gyro (deg/s→rad/s).
    Both are rotated IMU→base by ``mounting_rotation`` (3×3, default identity — CALIBRATE this).

    Marks ``valid=False`` when the latest sample is older than ``stale_after_s`` or missing,
    so :class:`~humanoid_control.runner.PolicyRunner` (with ``require_valid_base=True``) refuses
    to run a balance loop on stale orientation.
    """

    def __init__(
        self,
        reader: WitMotionReader,
        *,
        mounting_rotation: np.ndarray | None = None,
        stale_after_s: float = 0.1,
    ):
        self._reader = reader
        self._R = (np.eye(3, dtype=np.float32) if mounting_rotation is None
                   else np.asarray(mounting_rotation, dtype=np.float32))
        self._stale_after_s = stale_after_s

    def get(self) -> BaseState:
        s = self._reader.latest()
        age = time.monotonic() - s.stamp
        fresh = s.quat_wxyz is not None and s.gyro_dps is not None and age <= self._stale_after_s
        if not fresh:
            return BaseState(
                projected_gravity=_GRAVITY_WORLD.copy(),
                base_ang_vel=np.zeros(3, dtype=np.float32),
                valid=False,
            )
        pg_imu = quat_rotate_inverse(s.quat_wxyz, _GRAVITY_WORLD)
        pg = (self._R @ pg_imu).astype(np.float32)
        ang_vel = (self._R @ (s.gyro_dps * _DEG2RAD)).astype(np.float32)
        return BaseState(projected_gravity=pg, base_ang_vel=ang_vel, valid=True)
