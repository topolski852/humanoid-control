#!/usr/bin/env python3
"""Live robot MAP — feet-planted encoder ↔ IMU movement check (sim2real verification).

Plant the robot's feet, leave the motors limp (DISABLED — so the legs back-drive),
then move the torso around by hand. With the feet fixed, the 12 leg encoders and the
torso IMU are two *independent* measurements of the same rigid motion: the joints form
a closed chain that DETERMINES the torso orientation, and the IMU MEASURES it. If they
disagree, that gap is your sim2real disconnect — most often an IMU frame/mounting issue
or a joint sign/offset, both of which silently corrupt the policy's base state in sim.

This tool is passive and read-only: it listens to the daemon's UDP telemetry push and
NEVER sends a command. It shows, side by side and time-synced:

  - IMU torso motion    — roll/pitch/yaw (human-intuitive) AND the exact quantities the
                          policy consumes: projected_gravity + base angular_velocity.
  - Encoder motion      — every leg joint's angle and velocity.
  - Δ-from-reference    — press [z] to zero at any pose, then move: watch the torso Δ
                          (IMU) next to which joints moved and by how much (encoders).

Live sanity checks (need no kinematic model):
  - |gravity| ≈ 1.0     — projected_gravity must be a unit vector; drift = IMU scale/cal.
  - gyro ↔ orientation  — |ω| (base_ang_vel) vs the rotation rate implied by the changing
                          quaternion. The policy eats BOTH base_ang_vel and
                          projected_gravity; if they live in different frames these two
                          rates diverge — a real, policy-breaking IMU bug you can see live.

Recording ([r] or --record) writes one JSONL per session in the shape the offline MuJoCo
feet-pinned compare (scripts/map_vs_sim.py, next) consumes: quaternion [w,x,y,z],
projected_gravity, angular_velocity, and joint_pos/joint_vel in canonical order, plus any
labeled markers you drop with [m] to segment motions.

Usage:
    python3 scripts/robot_map.py                 # live map, no recording (binds UDP 9000)
    python3 scripts/robot_map.py --record        # also record -> recordings/map_*.jsonl
    python3 scripts/robot_map.py --record run1.jsonl
    sudo python3 scripts/robot_map.py --sniff --record   # passive tap, coexists w/ web app

Keys (when run in a terminal):
    z   zero / capture the current pose as the Δ reference
    m   drop a labeled marker into the recording (segment a motion)
    r   toggle recording on/off
    q   quit

TWO telemetry sources — the daemon unicasts telemetry to 127.0.0.1:9000:
  default : BIND port 9000. Fails loud (no SO_REUSEADDR) if the web app or another map
            already holds it, rather than silently stealing its stream. Use standalone.
  --sniff : PASSIVE raw-loopback tap (AF_PACKET) — reads a *copy* of every telemetry
            packet without binding the port, so it runs happily ALONGSIDE the web app
            (e.g. while you recalibrate through the UI). Needs root; re-run under sudo.
            Same philosophy as can_monitor.py tapping the CAN bus via candump.
"""
import _bootstrap  # noqa: F401
import argparse
import json
import math
import os
import select
import socket
import struct
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

from humanoid_control import LegPolicyContract

TEL_PORT = 9000
STALE_S = 0.5          # telemetry older than this ⇒ show as stale
IMU_STALE_S = 0.5      # base block older than this ⇒ IMU shown as STALE
GYRO_MATCH_TOL = 5.0   # deg/s: |ω| vs quat-rate agreement band for the ✓ / MISMATCH flag


# --------------------------------------------------------------------------- #
# Quaternion helpers ([w, x, y, z], the daemon's convention)                  #
# --------------------------------------------------------------------------- #
def quat_to_rpy(q):
    """[w,x,y,z] -> (roll, pitch, yaw) in radians, aerospace ZYX."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quat_mul(a, b):
    """Hamilton product a ⊗ b, both [w,x,y,z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_geodesic_deg(a, b):
    """Smallest rotation angle (deg) between two unit quaternions."""
    d = abs(float(np.dot(a, b)))
    d = max(-1.0, min(1.0, d))
    return math.degrees(2.0 * math.acos(d))


# --------------------------------------------------------------------------- #
# Non-blocking single-key terminal input (graceful when not a TTY)            #
# --------------------------------------------------------------------------- #
class KeyReader:
    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self._fd = sys.stdin.fileno() if self.enabled else None
        self._old = None

    def __enter__(self):
        if self.enabled:
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if self.enabled and self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def get(self):
        """Return a pending keypress char, or None."""
        if not self.enabled:
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None


# --------------------------------------------------------------------------- #
# Recorder — one JSONL per session, matched to the offline sim-compare schema  #
# --------------------------------------------------------------------------- #
class MapRecorder:
    def __init__(self, path, joint_order):
        self.path = path
        self._f = open(path, "w", buffering=1)
        self._t0 = time.monotonic()
        self._f.write(json.dumps({"_meta": {
            "tool": "robot_map",
            "joint_order": list(joint_order),
            "units": {"quaternion": "wxyz", "angular_velocity": "rad/s (base frame)",
                      "projected_gravity": "unit vector (base frame)",
                      "joint_pos": "rad (canonical order)", "joint_vel": "rad/s"},
            "note": "feet-planted hand-move capture; feed to scripts/map_vs_sim.py",
        }}) + "\n")

    def frame(self, base, joint_pos, joint_vel):
        self._f.write(json.dumps({
            "t": round(time.monotonic() - self._t0, 4),
            "quaternion": base["quaternion"],
            "angular_velocity": base["angular_velocity"],
            "projected_gravity": base["projected_gravity"],
            "joint_pos": joint_pos, "joint_vel": joint_vel,
        }) + "\n")

    def marker(self, label):
        self._f.write(json.dumps({
            "t": round(time.monotonic() - self._t0, 4), "marker": label}) + "\n")

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Telemetry sources — both expose recv() (non-blocking: one TELEMETRY dict, or   #
# None when nothing is buffered right now) and close().                          #
# --------------------------------------------------------------------------- #
class UdpSource:
    """Bind UDP 9000 WITHOUT SO_REUSEADDR so we fail loud on contention."""

    def __init__(self):
        self._s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._s.bind(("", TEL_PORT))
        except OSError as exc:
            self._s.close()
            print(f"\nCannot bind telemetry port {TEL_PORT}: {exc}\n"
                  "Something else already holds it — most likely the web app or another\n"
                  "robot_map. Either stop it (`systemctl --user stop humanoid-web`), or\n"
                  "run this passively with:  sudo python3 scripts/robot_map.py --sniff",
                  file=sys.stderr)
            sys.exit(1)
        self._s.setblocking(False)

    def recv(self):
        while True:
            try:
                data, _ = self._s.recvfrom(65535)
            except (BlockingIOError, socket.timeout):
                return None
            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                continue
            if msg.get("type") == "TELEMETRY":
                return msg

    def close(self):
        self._s.close()


class SniffSource:
    """Passive raw-loopback tap of UDP 9000 — a *copy* of every telemetry packet,
    without binding the port, so it coexists with the web app. Needs root (AF_PACKET)."""

    def __init__(self):
        try:
            self._s = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM,
                                    socket.htons(0x0800))  # ETH_P_IP
            self._s.bind(("lo", 0))
        except PermissionError:
            print("\n--sniff needs root for a raw packet tap. Re-run:\n"
                  "  sudo python3 scripts/robot_map.py --sniff [--record]", file=sys.stderr)
            sys.exit(1)
        except OSError as exc:
            print(f"\n--sniff could not open raw loopback socket: {exc}", file=sys.stderr)
            sys.exit(1)
        self._s.setblocking(False)

    def recv(self):
        while True:
            try:
                pkt, _ = self._s.recvfrom(65535)
            except (BlockingIOError, socket.timeout):
                return None
            if len(pkt) < 28 or pkt[9] != 17:      # IPv4 min + UDP (proto 17)
                continue
            ihl = (pkt[0] & 0x0F) * 4
            sport, dport, ulen, _ = struct.unpack("!HHHH", pkt[ihl:ihl + 8])
            if TEL_PORT not in (sport, dport):
                continue
            try:
                msg = json.loads(pkt[ihl + 8:ihl + ulen])
            except (json.JSONDecodeError, ValueError):
                continue
            if isinstance(msg, dict) and msg.get("type") == "TELEMETRY":
                return msg

    def close(self):
        self._s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--record", nargs="?", const="__auto__", default=None,
                    metavar="FILE", help="record to FILE (or auto recordings/map_*.jsonl)")
    ap.add_argument("--sniff", action="store_true",
                    help="passive raw-loopback tap (needs sudo); coexists with the web app")
    ap.add_argument("--interval", type=float, default=0.1, help="redraw period (s)")
    args = ap.parse_args()

    contract = LegPolicyContract.load()
    order = list(contract.joint_order)
    default_pose = contract.default_pose.astype(float)

    source = SniffSource() if args.sniff else UdpSource()

    rec = None
    if args.record is not None:
        rec_dir = Path(__file__).resolve().parent.parent / "recordings"
        rec_dir.mkdir(exist_ok=True)
        path = (rec_dir / f"map_{time.strftime('%Y%m%dT%H%M%S')}.jsonl"
                if args.record == "__auto__" else Path(args.record))
        rec = MapRecorder(str(path), order)

    # latest telemetry snapshot
    latest = None          # parsed TELEMETRY dict
    last_rx = -1e9          # monotonic of last frame
    rx_times = []           # for Hz

    # Δ-reference + gyro/quat cross-check state
    ref_quat = None
    ref_pos = None          # dict name->rad at zero
    prev_quat = None
    prev_t = None
    quat_rate = 0.0         # deg/s implied by changing quaternion
    marker_n = 0
    recording = rec is not None

    next_draw = time.monotonic()

    with KeyReader() as keys:
        print("\033[2J", end="")  # clear once
        try:
            while True:
                # ---- drain all buffered telemetry so we always show the freshest ----
                while True:
                    msg = source.recv()
                    if msg is None:
                        break
                    now = time.monotonic()
                    latest = msg
                    last_rx = now
                    rx_times.append(now)
                    rx_times[:] = [t for t in rx_times if now - t <= 1.0]

                    base = msg.get("base")
                    joints = msg.get("joints", {})
                    jp = [float(joints.get(n, {}).get("position") or 0.0) for n in order]
                    jv = [float(joints.get(n, {}).get("velocity") or 0.0) for n in order]

                    if base and base.get("quaternion"):
                        q = np.array(base["quaternion"], dtype=float)
                        if prev_quat is not None and prev_t is not None:
                            dt = now - prev_t
                            if dt > 1e-4:
                                quat_rate = quat_geodesic_deg(prev_quat, q) / dt
                        prev_quat, prev_t = q, now
                        if recording and rec is not None:
                            rec.frame(base, jp, jv)

                # ---- keys ----
                k = keys.get()
                if k in ("q", "\x03"):
                    break
                elif k == "z" and latest and latest.get("base"):
                    ref_quat = np.array(latest["base"]["quaternion"], dtype=float)
                    joints = latest.get("joints", {})
                    ref_pos = {n: float(joints.get(n, {}).get("position") or 0.0) for n in order}
                elif k == "m" and rec is not None:
                    marker_n += 1
                    rec.marker(f"marker_{marker_n}")
                elif k == "r" and rec is not None:
                    recording = not recording

                # ---- draw ----
                now = time.monotonic()
                if now < next_draw:
                    time.sleep(min(0.02, next_draw - now))
                    continue
                next_draw = now + args.interval
                draw(order, default_pose, latest, last_rx, rx_times, ref_quat, ref_pos,
                     quat_rate, rec, recording)
        finally:
            source.close()
            if rec is not None:
                rec.close()
                print(f"\nrecording saved -> {rec.path}")


def draw(order, default_pose, latest, last_rx, rx_times, ref_quat, ref_pos,
         quat_rate, rec, recording):
    now = time.monotonic()
    age = now - last_rx
    hz = len(rx_times)
    out = ["\033[H\033[2J"]  # home + clear

    tel_ok = latest is not None and age < STALE_S
    recstr = ""
    if rec is not None:
        recstr = "  REC ●" if recording else "  rec ○"
    out.append(f"ROBOT MAP  {time.strftime('%H:%M:%S')}   "
               f"telemetry: {'ok' if tel_ok else 'STALE'} {hz:>3d} Hz{recstr}")
    out.append("-" * 72)

    base = latest.get("base") if latest else None
    if not base or not base.get("quaternion"):
        out.append("IMU: STALE / absent  (daemon base block is null — check the IMU)")
    else:
        q = np.array(base["quaternion"], dtype=float)
        roll, pitch, yaw = (math.degrees(a) for a in quat_to_rpy(q))
        pg = base.get("projected_gravity", [0, 0, 0])
        gmag = float(np.linalg.norm(pg))
        av = base.get("angular_velocity", [0, 0, 0])
        av_deg = [math.degrees(v) for v in av]
        gyro_mag = math.degrees(float(np.linalg.norm(av)))

        if ref_quat is not None:
            q_rel = quat_mul(quat_conj(ref_quat), q)
            dr, dp, dy = (math.degrees(a) for a in quat_to_rpy(q_rel))
            geo = quat_geodesic_deg(ref_quat, q)
            out.append(f"IMU Δ from ref   roll {dr:+7.2f}°  pitch {dp:+7.2f}°  "
                       f"yaw {dy:+7.2f}°   |Δ| {geo:5.2f}°")
        else:
            out.append(f"IMU absolute     roll {roll:+7.2f}°  pitch {pitch:+7.2f}°  "
                       f"yaw {yaw:+7.2f}°   [press z to zero]")

        gflag = "OK " if abs(gmag - 1.0) < 0.05 else "!! "
        out.append(f"proj_gravity  [{pg[0]:+.3f} {pg[1]:+.3f} {pg[2]:+.3f}]  "
                   f"|g|={gmag:.3f} {gflag}(policy input)")
        match = "match ✓" if abs(gyro_mag - quat_rate) < GYRO_MATCH_TOL else "MISMATCH !!"
        out.append(f"base_ang_vel  [{av_deg[0]:+6.1f} {av_deg[1]:+6.1f} {av_deg[2]:+6.1f}] °/s  "
                   f"|ω|={gyro_mag:5.1f}  quat-rate={quat_rate:5.1f} °/s  {match}")

    out.append("-" * 72)
    out.append(f"{'joint':<26}{'angle°':>9}{'Δref°':>9}{'vel°/s':>9}")
    joints = latest.get("joints", {}) if latest else {}
    for i, n in enumerate(order):
        j = joints.get(n, {})
        online = j.get("state") != "OFFLINE" and j.get("position") is not None
        if not online:
            out.append(f"{n:<26}{'OFFLINE':>9}{'':>9}{'':>9}")
            continue
        pos = float(j.get("position") or 0.0)
        vel = float(j.get("velocity") or 0.0)
        dref = ("" if ref_pos is None
                else f"{math.degrees(pos - ref_pos.get(n, pos)):+9.2f}")
        out.append(f"{n:<26}{math.degrees(pos):>9.2f}{dref:>9}{math.degrees(vel):>9.2f}")

    out.append("-" * 72)
    out.append("[z] zero ref   [m] marker   [r] rec toggle   [q] quit")
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped.")
