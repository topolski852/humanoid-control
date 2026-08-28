#!/usr/bin/env python3
"""Sample the machine's state to disk, fsync'd, so a hard freeze leaves evidence.

WHY THIS EXISTS. Every freeze so far has looked identical: the machine stops instantly,
the journal ends mid-line, and there is no panic, no OOM, no MCE and nothing in pstore.
That combination means the kernel died somewhere it could not write anything down — so
the ONLY evidence available is whatever reached the platter BEFORE it went. journald
buffers, which is exactly why its last line is mid-sentence and uninformative.

So every sample here is written and fsync'd individually. The cost is one small synchronous
write every few seconds; the benefit is that the last line in this file is a true statement
about the machine a moment before it stopped, not whatever happened to be flushed.

What is sampled is chosen to discriminate between the surviving hypotheses:

  * xhci_hcd interrupts   the Quest re-enumerating on USB was the last thing in the
                          kernel log before the 4-hour boot died. A runaway or stalled
                          interrupt count is what a wedging host controller looks like.
  * mt7925e interrupts    the MediaTek combo chip, whose Bluetooth firmware was already
                          timing out. Prime suspect, and this run is the test: the frame
                          stream is on USB now, so WiFi should be nearly idle. If it
                          freezes anyway with WiFi quiet, WiFi is not the cause.
  * per-interface bytes   proves where the traffic actually went. The whole point of the
                          USB tether is that wlp7s0 stays quiet under load.
  * CAN error counters    gs_usb shares the USB tree with the headset.
  * MemAvailable          rules out slow exhaustion (no OOM was ever logged, but a freeze
                          before the OOM killer runs would look like this).

Usage:  setsid nohup python3 scripts/freeze_monitor.py <logfile> [interval_s] &
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

INTERVAL = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/nse/humanoid-logs/freeze_monitor.log"

WIFI = "wlp7s0"
CAN = "can_left_arm"


def read(path: str, default: str = "") -> str:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return default


def irq_counts() -> dict[str, int]:
    """Total interrupts per driver of interest, summed across CPUs."""
    out: dict[str, int] = {}
    try:
        with open("/proc/interrupts") as fh:
            for line in fh:
                low = line.lower()
                for key in ("xhci_hcd", "mt7925", "nvme", "igc"):
                    if key in low:
                        nums = [int(t) for t in line.split() if t.isdigit()]
                        # The IRQ number is the first field and is followed by per-CPU
                        # counts; drop it so a renumbered IRQ does not read as traffic.
                        out[key] = out.get(key, 0) + sum(nums[1:])
    except OSError:
        pass
    return out


def net_bytes(iface: str) -> tuple[int, int]:
    base = f"/sys/class/net/{iface}/statistics"
    try:
        return int(read(f"{base}/rx_bytes", "0")), int(read(f"{base}/tx_bytes", "0"))
    except ValueError:
        return 0, 0


def mem_available_mb() -> int:
    for line in read("/proc/meminfo").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) // 1024
    return -1


# /sys/class/thermal has only cooling_devices on this board — no thermal_zone* at all, so
# the obvious path silently reports 0 C and "not thermal" becomes an assumption rather than
# a measurement. hwmon has the real sensors, including one INSIDE the WiFi chip.
_HWMON: dict[str, str] = {}


def _find_hwmon() -> None:
    """Map sensor name -> temp input file, once."""
    want = {"k10temp": "cpu", "mt7925_phy0": "wifi", "amdgpu": "gpu"}
    try:
        for h in sorted(os.listdir("/sys/class/hwmon")):
            base = f"/sys/class/hwmon/{h}"
            name = read(f"{base}/name")
            if name in want and os.path.exists(f"{base}/temp1_input"):
                _HWMON[want[name]] = f"{base}/temp1_input"
    except OSError:
        pass


def temps() -> tuple[float, float]:
    """(CPU, WiFi-chip) in C. The WiFi one matters: mt7925e is the prime suspect, and a
    chip cooking itself before it wedges would be a very different story from one that
    dies cold."""
    def rd(key: str) -> float:
        path = _HWMON.get(key)
        if not path:
            return 0.0
        raw = read(path, "0")
        return int(raw) / 1000.0 if raw.lstrip("-").isdigit() else 0.0
    return rd("cpu"), rd("wifi")


def can_errors() -> str:
    """`re-started bus-errors arbit-lost error-warn error-pass bus-off` as one compact
    field. The counters are on the line AFTER that header — matching the header itself
    just prints the column names, which is what the first version did.

    The CAN adapter hangs off `1-2.1`, the same xHCI controller as the headset on `1-1`.
    If a wedging host controller is what kills this machine, both of these go quiet in
    the same sample, and that is a much sharper signal than either alone."""
    try:
        txt = subprocess.run(["ip", "-s", "-d", "link", "show", CAN],
                             capture_output=True, text=True, timeout=3).stdout
        lines = txt.splitlines()
        for i, line in enumerate(lines):
            if "bus-errors" in line and i + 1 < len(lines):
                v = lines[i + 1].split()
                if len(v) >= 6 and all(t.isdigit() for t in v[:6]):
                    return ("ok" if not any(int(t) for t in v[:6])
                            else f"rst={v[0]} berr={v[1]} arb={v[2]} off={v[5]}")
    except Exception:                                        # noqa: BLE001
        pass
    return "?"


def main() -> int:
    _find_hwmon()
    fh = open(PATH, "a", buffering=1)
    prev_irq: dict[str, int] = {}
    prev_net: dict[str, tuple[int, int]] = {}

    def emit(line: str) -> None:
        fh.write(line + "\n")
        fh.flush()
        os.fsync(fh.fileno())          # the whole point — see the module docstring

    emit("")
    emit(f"===== monitor started {time.strftime('%Y-%m-%d %H:%M:%S')} "
         f"(boot uptime {float(read('/proc/uptime', '0 0').split()[0]):.0f}s, "
         f"interval {INTERVAL}s) =====")
    emit("       time   uptime   load  memMB  cpuC wifiC | xhci/s mt7925/s | "
         "wifi rx/tx KB/s | can rx/tx KB/s | can_err")

    while True:
        try:
            now = time.strftime("%H:%M:%S")
            up = float(read("/proc/uptime", "0 0").split()[0])
            load = read("/proc/loadavg", "0 0 0").split()[0]
            irq = irq_counts()
            wifi = net_bytes(WIFI)
            can = net_bytes(CAN)

            def rate(cur: int, key: str, store: dict) -> float:
                old = store.get(key)
                store[key] = cur
                return 0.0 if old is None else max(0.0, (cur - old) / INTERVAL)

            xhci = rate(irq.get("xhci_hcd", 0), "xhci_hcd", prev_irq)
            mtk = rate(irq.get("mt7925", 0), "mt7925", prev_irq)

            def nrate(cur: tuple[int, int], key: str) -> tuple[float, float]:
                old = prev_net.get(key)
                prev_net[key] = cur
                if old is None:
                    return 0.0, 0.0
                return (max(0, cur[0] - old[0]) / INTERVAL / 1024.0,
                        max(0, cur[1] - old[1]) / INTERVAL / 1024.0)

            wrx, wtx = nrate(wifi, "wifi")
            crx, ctx = nrate(can, "can")

            tc, tw = temps()
            emit(f"  {now}  {up:7.0f}  {load:>5}  {mem_available_mb():5d}  "
                 f"{tc:4.1f} {tw:5.1f} | {xhci:6.0f} {mtk:8.0f} | "
                 f"{wrx:7.1f}/{wtx:6.1f} | {crx:6.1f}/{ctx:6.1f} | {can_errors()}")
        except Exception as exc:                             # noqa: BLE001
            try:
                emit(f"  monitor error: {exc}")
            except Exception:                                # noqa: BLE001
                return 1
        time.sleep(INTERVAL)


if __name__ == "__main__":
    raise SystemExit(main())
