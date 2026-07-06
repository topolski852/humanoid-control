# Prompt for Claude Code — bring up wireless web control on the robot PC

You are Claude Code running **on the Berkeley Humanoid Lite robot's onboard PC** (the machine
that owns the CAN bus), in the `humanoid-control` repo. A wireless web control layer was built
and verified (no-hardware) on another machine; your job is to get it **running fully on this
machine**, reachable from a laptop on the same WiFi, and safe.

Read `README.md` (§ *Wireless web control*), `deploy/README.md`, and `POLICY_CONTRACT.md`
first. The design rationale and safety model live there — don't re-derive them.

## What this feature is
A long-lived FastAPI server (`humanoid_control/web/`) wraps the same lifecycle the CLI scripts
drive (connect → arm → ramp/hold → run policy) and serves a React control page (`app/`) over the
LAN. The robot's 200 Hz CAN loop (C++ daemon) and 25 Hz policy loop run entirely on THIS PC — the
browser is a supervisor, not in the control loop. Safety additions over the CLI: a web **Arm**
toggle (= `--i-am-present`), an always-on **E-STOP**, and a **deadman** (browser heartbeat over
`/ws/control`; if it drops mid-motion the server auto-fires `estop_all()`).

## Ground truth on this machine
- Robot live config (source of truth, do NOT fork): `/home/nse/humanoid-studio/configs/humanoid_lite.json`.
- Only ONE daemon may own CAN at a time — stop the humanoid-studio app's daemon before starting this one.
- There is a **physical disconnect kill switch** on the robot (primary E-STOP for now). A USB /
  Bluetooth-Xbox gamepad deadman is coded but **disabled** (`humanoid_control/web/gamepad.py`).
- No IMU yet: the base state is an upright stub — it can hold/track a pose but **cannot close a
  balance loop**. Keep the robot supported/gantried for ALL motion.

## Tasks (do in order; stop and ask the user before anything that moves the robot)

### 0. Get the code
Ensure this repo has the web-control code. If it's on a feature branch, `git fetch` and check it
out (or `git pull` if it's been merged to `main`). Confirm `humanoid_control/web/`, `app/`, and
`deploy/` exist.

### 1. Build + install (no hardware needed)
```bash
cd /home/nse/humanoid-control
cd daemon && make && cd ..                         # if daemon/build/humanoid_daemon is missing/stale
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
cd app && npm install && npm run build && cd ..    # builds app/dist (Node 18+)
```

### 2. No-hardware smoke test (safe)
Start the server pointing at a missing config so nothing connects, and confirm the surface works:
```bash
HUMANOID_WEB_HOST=127.0.0.1 HUMANOID_WEB_PORT=8011 HUMANOID_CONFIG=/nonexistent \
  .venv/bin/python -m humanoid_control.web &
curl -s localhost:8011/api/status | head -c 400          # 12 joints, daemon_alive false
curl -s -X POST localhost:8011/api/estop                 # success:true, state ESTOPPED
curl -s -o /dev/null -w "%{http_code}\n" localhost:8011/  # 200 (UI served)
kill %1
```
Expected: connect → 503 (no config), arm/hold → 409, E-STOP → success even with the daemon down.

### 3. Daemon up, read-only telemetry (no motion)
Make sure the humanoid-studio daemon is stopped, the leg CAN adapters are powered, then:
```bash
./daemon/build/humanoid_daemon --config /home/nse/humanoid-studio/configs/humanoid_lite.json &
HUMANOID_WEB_HOST=0.0.0.0 HUMANOID_CONFIG=/home/nse/humanoid-studio/configs/humanoid_lite.json \
  .venv/bin/python -m humanoid_control.web &
hostname -I    # note the IP → tell the user to open http://<ip>:8000
```
From the laptop browser: the joint table should stream live pos/vel at ~20 Hz; **daemon** and
**server** dots green. Click **Connect** — joints should go online. Do NOT arm/move yet.
(This mirrors `scripts/smoke_test.py`.)

### 4. Deadman drill — REQUIRES the user present + robot supported/gantried, low torque
**Ask the user to confirm they are present and the robot is supported before this step.**
With the robot supported: on the page, **Connect → Arm → Ramp to pose / Hold**. Then **close the
browser tab (or drop WiFi)**. Confirm the server logs an E-STOP (`deadman-*`) and the daemon IDLEs
the joints. This proves the wireless safety net before you trust it. Re-open the page; state reads
`ESTOPPED` until you Connect again.

### 5. Motion via the UI — user present, supported/gantried
Connect → Arm → **Ramp to pose** (confirm it *ramps*, never steps, and holds `default_pose`).
If a trained checkpoint exists (put `.onnx`/`.pt` under `checkpoints/` or set `HUMANOID_POLICY_DIR`),
pick it and **Run policy**. Verify the **E-STOP** button IDLEs instantly. (Mirrors `hold_pose.py`
/ `run_policy.py`.)

### 6. Auto-start on boot (once 2–5 pass)
Follow `deploy/README.md`: install `deploy/humanoid-daemon.service` + `deploy/humanoid-web.service`,
`systemctl enable --now` both, then reboot and confirm `http://<ip>:8000` is reachable with no SSH.
Set a static DHCP lease / mDNS name so the URL is stable. If exposing beyond a trusted LAN, set
`HUMANOID_WEB_PASSWORD` in the web unit.

## Later (optional): gamepad deadman
When a Bluetooth Xbox controller is on hand: `pip install evdev`, pair it (`bluetoothctl`), then set
`HUMANOID_GAMEPAD_ENABLE=1` (+ optionally `HUMANOID_GAMEPAD_MODE=holdtorun`) in the web service.
It registers as a deadman so a dead battery / dropped signal auto-E-STOPs during motion, plus a
hard-kill button. See `humanoid_control/web/gamepad.py` for the button map. Leave it OFF until tested.

## Non-negotiable safety
- Never move the robot beyond a supported/gantry dry-run without the user present and confirming.
- Targets are always clamped to per-joint `position_limits`; the runner ramps (never steps).
- E-STOP (priority port 9002) and the deadman are always armed during motion; the physical kill
  switch is the primary. Keep the robot supported until the IMU lands.

## Env vars (reference)
`HUMANOID_WEB_HOST` (0.0.0.0), `HUMANOID_WEB_PORT` (8000), `HUMANOID_CONFIG` (robot config path),
`HUMANOID_WEB_PASSWORD` (optional login), `HUMANOID_POLICY_DIR` (checkpoint list),
`HUMANOID_GAMEPAD_ENABLE` / `HUMANOID_GAMEPAD_MODE` (optional gamepad deadman).
