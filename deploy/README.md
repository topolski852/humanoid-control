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

# 4. Install the units
sudo cp deploy/humanoid-daemon.service deploy/humanoid-web.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now humanoid-daemon.service humanoid-web.service
```

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

## Notes
- **One daemon owns CAN.** Stop the humanoid-studio app's daemon before enabling this one.
- **CAN adapters must be up** before `humanoid-daemon.service` (see the comment in that unit).
- **Password.** On anything but a trusted LAN, set `HUMANOID_WEB_PASSWORD` in the web unit
  (it's plain HTTP — for internet exposure, front it with TLS, e.g. `tailscale serve`).
- **Rebuild the UI** after frontend changes: `cd app && npm run build` (no service restart needed;
  it's static). Restart `humanoid-web.service` after backend/Python changes.
- **Gamepad deadman** (optional, prepared but off): `pip install evdev`, pair a Bluetooth Xbox
  controller, then set `HUMANOID_GAMEPAD_ENABLE=1` in the web unit. See `humanoid_control/web/gamepad.py`.
