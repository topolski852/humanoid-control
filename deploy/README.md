# Deploy — auto-start on boot (fully wireless)

Bring the CAN daemon and the web control UI up on boot so the robot needs no monitor, keyboard,
or held-open SSH. Power on → wait ~15 s → open `http://<robot-ip>:8000` from any PC on the WiFi.

## One-time setup on the robot PC

```bash
cd /home/nse/humanoid-control

# 1. Build the daemon (if not already built)
cd daemon && make && cd ..

# 2. Python env for the web server
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# 3. Build the web UI (Node 18+); produces app/dist that the server serves
cd app && npm install && npm run build && cd ..

# 4. Install the IMU udev rule (stable /dev/humanoid_imu on plug-in / boot)
sudo cp deploy/99-humanoid-imu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/humanoid_imu        # → symlink to ttyUSBx

# 5. Install the units
sudo cp deploy/humanoid-daemon.service deploy/humanoid-web.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now humanoid-daemon.service humanoid-web.service
```

## IMU (external WitMotion 10-axis AHRS)
The `humanoid-daemon.service` `ExecStart` passes `--imu-device /dev/humanoid_imu`, so on boot
the daemon reads the IMU (921600 baud) and adds a `base` block (quaternion, angular velocity,
projected gravity) to its telemetry. The web control layer consumes it automatically. With no
IMU present the daemon emits `base: null` and the policy falls back to the upright stub.
- **Verify streaming (no daemon needed):** `cd daemon && make imu-test && ./build/imu_test`
- **Mounting orientation:** gravity/ang-vel must be in the robot base frame. Hold the robot
  upright and run `python scripts/imu_calibrate.py`, then paste the printed `mounting_rotation`
  into an `"imu"` block in the daemon config (see the config note below). Default is identity.

Check status / logs:
```bash
systemctl status humanoid-web.service
journalctl -u humanoid-daemon.service -f
```

## Find the robot's IP (to open the page)
```bash
hostname -I        # e.g. 192.168.1.42  →  http://192.168.1.42:8000
```
A static DHCP lease (or `.local` mDNS name) saves you re-checking.

### IMU config block (for mounting_rotation / overrides)
`--imu-device` is enough to enable the IMU, but `mounting_rotation` can only come from the
config. Add an `"imu"` block at the top level of the daemon config JSON (all keys optional):
```json
"imu": {
  "device": "/dev/humanoid_imu",
  "baud": 921600,
  "staleness_ms": 100,
  "mounting_rotation": [1.0, 0.0, 0.0, 0.0]
}
```
The CLI `--imu-device` / `--imu-baud` override `device`/`baud` and force `enabled`. If you put
the `imu` block in the studio-shared `humanoid_lite.json`, be aware the studio app may rewrite
that file — keeping IMU enablement on the CLI (as the unit does) avoids depending on it.

## Gamepad hold-to-run deadman (USB Xbox / 8BitDo)
The controller is the primary safety once armed. Enable it in `humanoid-web.service`
(`HUMANOID_GAMEPAD_ENABLE=1`, already set) and grant input access:
```bash
sudo cp deploy/99-humanoid-input.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=input
sudo usermod -aG input nse          # service must read /dev/input/event*
```
**Operating flow:** connect + calibrate in the web UI → **A** arms (limbs → the rest state, DAMPING by default) →
**hold LT or RT** to engage (ramp to default_pose, then run the selected session with the live
walk command) → **release** to damp → repeat. **Left stick** = forward/left, **right stick X** =
yaw (0.15 deadband). **B** = hard E-STOP (reconnect in the UI to clear). **Y** = disarm (→ IDLE).
- **Two safety signals:** releasing a trigger → the configured **rest state** (recoverable);
  losing the controller entirely (unplug / receiver drop) → **E-STOP** any live session.
  E-STOP also lands in DAMPING, so a raised limb resists rather than dropping.
- **Rest state is DAMPING by default**, selectable per-session on the *Rest state* card
  (damping / idle). It needs a daemon that FEEDS the firmware watchdog while a joint is
  damping — `Actuator::tick()` used to send frames only to `ENABLED` joints, and an unfed
  damping joint faults `ERROR_WATCHDOG_TIMEOUT` (0x0040) within about a second, E-STOPping the
  session. **Rebuild `daemon/` if you pull an older one**; a stale daemon makes the default
  rest state unusable rather than merely different.
- Keep the robot **supported** — the balance loop is unproven; the deadman is a safety, not a net.

## Notes
- **One daemon owns CAN.** Stop the humanoid-studio app's daemon before enabling this one.
- **CAN adapters must be up** before `humanoid-daemon.service` (see the comment in that unit).
- **Password.** On anything but a trusted LAN, set `HUMANOID_WEB_PASSWORD` in the web unit
  (it's plain HTTP — for internet exposure, front it with TLS, e.g. `tailscale serve`).
- **Rebuild the UI** after frontend changes: `cd app && npm run build` (no service restart needed;
  it's static). Restart `humanoid-web.service` after backend/Python changes.
- **Gamepad deadman** (optional, prepared but off): `pip install evdev`, pair a Bluetooth Xbox
  controller, then set `HUMANOID_GAMEPAD_ENABLE=1` in the web unit. See `humanoid_control/web/gamepad.py`.
