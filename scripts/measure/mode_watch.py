#!/usr/bin/env python3
"""LOG-ONLY: record every change in per-joint (daemon state, firmware mode, error) at ~5 Hz.

    python scripts/measure/mode_watch.py

Written 2026-09-29 to catch a fault seen once: three hip joints reported ERROR_WATCHDOG_TIMEOUT
(0x40) while armed in DAMPING, before any engage. A joint whose firmware is in DAMPING (mode 2)
while the daemon believes it IDLE is never fed and times out after 1 s; clear_faults was shown to
create exactly that mismatch. This logs the first moment any joint disagrees or errors, so the
cause is visible next time. It never commands anything.
"""
import json, time, urllib.request
FW = {1: "IDLE", 2: "DAMPING", 0x13: "POSITION"}
prev = {}
print("mode watch (log-only) started", flush=True)
while True:
    try:
        d = json.loads(urllib.request.urlopen("http://127.0.0.1:8000/api/status", timeout=1).read())["data"]
    except Exception:
        time.sleep(0.5); continue
    ts = time.strftime("%H:%M:%S") + f".{int(time.time() * 10) % 10}"
    svc = d.get("state")
    if prev.get("_svc") != svc:
        print(ts, "SERVICE", svc, flush=True); prev["_svc"] = svc
    for j in d.get("joints", [])[:12]:
        key = (j["state"], j.get("mode"), j.get("error"))
        if prev.get(j["name"]) != key:
            fw = FW.get(j.get("mode"), j.get("mode"))
            mismatch = (j["state"] == "IDLE" and j.get("mode") == 2) or (j["state"] == "DAMPING" and j.get("mode") == 1)
            flag = " <-- MISMATCH (unfed)" if mismatch else ""
            flag += f" <-- ERROR 0x{j['error']:x}" if j.get("error") else ""
            print(ts, f"{j['name']:26s} daemon={j['state']:8s} fw={fw}{flag}", flush=True)
            prev[j["name"]] = key
    time.sleep(0.2)
