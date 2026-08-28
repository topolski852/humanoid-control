#!/usr/bin/env python3
"""Does a DAMPING joint survive without faulting? The one test the whole change rests on.

    .venv/bin/python scripts/verify_damping_feed.py [seconds]

Puts the configured arm's joints into DAMPING, holds them there, and watches two things per
second: the firmware `error` register and the daemon's `joint_state`.

WHY BOTH, AND NOT JUST `error`:

  * `error` catches the fault this change exists to fix. An unfed DAMPING joint sets
    ERROR_WATCHDOG_TIMEOUT (0x0040) after watchdog_timeout ms — 1000 by default. Measured
    twice on this arm before the daemon fed DAMPING: 2026-07-14, and again 2026-08-27, five
    joints, every time.

  * `joint_state` catches a subtler way to pass by accident. The daemon flips any joint that
    goes quiet for 1500 ms to OFFLINE (actuator.cpp staleness check, which does NOT exempt
    DAMPING), and an OFFLINE joint is fed by nothing and polled by nothing. It would sit there
    reporting no error simply because nobody is talking to it any more. A run that ends with
    joints OFFLINE has not demonstrated anything.

NOTHING HERE MOVES THE ARM. DAMPING is a braking mode with no position target — it resists
motion, it does not command any. The joints are returned to IDLE on the way out, including on
Ctrl-C and on any exception, because leaving a bench arm powered into a brake it did not ask
for is worse than the fall this change is trying to prevent.
"""
from __future__ import annotations

import json
import signal
import socket
import sys
import time
import urllib.request
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_control.layout import LIMB_JOINTS                   # noqa: E402

HOLD_S = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
WATCHDOG_MS = 1000          # config_loader.cpp: watchdog_timeout, default
CMD_ADDR = ("127.0.0.1", 9001)
STATUS_URL = "http://127.0.0.1:8000/api/status"

# Commands go straight to the daemon's command port; state is read back from the web API.
#
# Deliberately NOT a second DaemonClient: that binds telemetry port 9000, which the running
# web server already owns. A second binder would either fail outright or quietly steal the
# telemetry stream the dashboard and the deadman watchdog depend on — a diagnostic that
# breaks the thing it is measuring.
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_sock.settimeout(5.0)


def set_mode(joint: str, mode: str) -> None:
    req = {"type": "SET_MODE", "joint_name": joint, "mode": mode, "id": uuid.uuid4().hex}
    _sock.sendto(json.dumps(req).encode(), CMD_ADDR)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            data, _ = _sock.recvfrom(65535)
        except socket.timeout:
            break
        try:
            rsp = json.loads(data.decode())
        except Exception:                                        # noqa: BLE001
            continue
        if rsp.get("id") != req["id"]:
            continue                                             # someone else's reply
        if rsp.get("type") == "ERROR":
            raise RuntimeError(f"{joint} -> {mode}: {rsp.get('message') or rsp}")
        return
    raise RuntimeError(f"{joint} -> {mode}: no ACK from the daemon")


def main() -> int:
    joints = list(LIMB_JOINTS["left_arm"])

    def snapshot():
        with urllib.request.urlopen(STATUS_URL, timeout=3) as fh:
            data = json.load(fh)["data"]
        by_name = {j.get("name"): j for j in data.get("joints") or []}
        return [(n, (by_name.get(n) or {}).get("state"),
                 int((by_name.get(n) or {}).get("error") or 0)) for n in joints]

    print(f"\n  joints: {len(joints)}   hold: {HOLD_S:.0f}s   "
          f"firmware watchdog: {WATCHDOG_MS} ms\n")
    before = snapshot()
    for n, s, e in before:
        print(f"    {n:<28} {str(s):<9} error=0x{e:04x}")
    if any(e for _, _, e in before):
        print("\n  REFUSING: a joint already has a latched error. Clear faults first "
              "(POST /api/clear_faults) so this run measures DAMPING and not history.")
        return 2
    if any(s != "IDLE" for _, s, _ in before):
        print("\n  REFUSING: every joint must start IDLE so the only variable is DAMPING.")
        return 2

    restored = False

    def restore(*_):
        nonlocal restored
        if restored:
            return
        restored = True
        for n in joints:
            try:
                set_mode(n, "IDLE")
            except Exception as exc:                             # noqa: BLE001
                print(f"    !! could not restore {n} to IDLE: {exc}")
        print("\n  joints returned to IDLE.")

    signal.signal(signal.SIGINT, lambda *a: (restore(), sys.exit(130)))

    worst_state, first_error_at = {}, None
    try:
        print("\n  -> DAMPING")
        for n in joints:
            set_mode(n, "DAMPING")

        t0 = time.monotonic()
        while (elapsed := time.monotonic() - t0) < HOLD_S:
            time.sleep(1.0)
            snap = snapshot()
            errs = {n: e for n, _, e in snap if e}
            states = {s for _, s, _ in snap}
            for n, s, _ in snap:
                worst_state[n] = s
            if errs and first_error_at is None:
                first_error_at = elapsed
            flag = ""
            if errs:
                flag = "  <-- FAULT " + " ".join(f"{n.split('_')[1]}=0x{e:04x}"
                                                 for n, e in errs.items())
            elif states != {"DAMPING"}:
                flag = f"  <-- state drifted: {sorted(states)}"
            print(f"    t+{elapsed:5.1f}s  states={sorted(states)}  errors={len(errs)}{flag}")
            if errs:
                break
    finally:
        restore()

    after = snapshot()
    faulted = [(n, e) for n, _, e in after if e]
    drifted = [n for n, s in worst_state.items() if s != "DAMPING"]

    print()
    if faulted:
        print(f"  FAIL — faulted after {first_error_at:.1f}s:")
        for n, e in faulted:
            print(f"    {n} error=0x{e:04x}"
                  + ("  (ERROR_WATCHDOG_TIMEOUT)" if e & 0x0040 else ""))
        print("\n  The PDO2 feed is not resetting the firmware watchdog. Do not flip the\n"
              "  default to damping.")
        return 1
    if drifted:
        print("  INCONCLUSIVE — these joints left DAMPING during the run: "
              + ", ".join(drifted))
        print("  A joint the daemon dropped to OFFLINE is fed by nothing, so a clean error\n"
              "  register proves nothing about the feed.")
        return 1

    print(f"  PASS — {len(joints)} joints held DAMPING for {HOLD_S:.0f}s "
          f"({HOLD_S * 1000 / WATCHDOG_MS:.0f}x the watchdog timeout), no errors, "
          "no state drift.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
