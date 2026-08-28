# Deploy — auto-start on boot (fully wireless)

Bring the CAN daemon and the web control UI up on boot so the robot needs no monitor, keyboard,
or held-open SSH. Power on → wait ~15 s → open `http://<robot-ip>:8000` from any PC on the WiFi.

## Upgrading an existing robot to the Quest build — read this first

Two things changed underneath every machine, including legs-only ones that will never see a
headset.

**1. Rebuild the daemon. This is not optional.** The rest state is now DAMPING instead of
IDLE, and the firmware watchdog *runs* in DAMPING — the daemon has to feed a damping joint or
the firmware faults `ERROR_WATCHDOG_TIMEOUT` (0x0040) after ~1 s and the session E-STOPs. That
feed is a C++ change (`Actuator::tick`). Pull the Python without rebuilding and the robot arms
fine, engages fine, and **E-STOPs one second after the operator releases the trigger** — while
possibly standing.

```bash
cd daemon && make && cd ..
sudo systemctl restart humanoid-daemon.service
```

Verify before trusting it: `.venv/bin/python scripts/verify_damping_feed.py 60` holds the
joints in DAMPING for a minute and fails loudly if any of them faults.

**2. One input source owns the robot at a time.** Walk and arm commands now carry a source and
are dropped unless that source holds the token; drops are counted and surfaced as
`ignored_writes`. The token goes to the **gamepad** at startup whenever
`HUMANOID_GAMEPAD_ENABLE=1`, which this unit sets.

The practical consequence on a legs robot: *starting* a policy from the browser still works
(the preflight checks browser liveness, not the token), but the **browser's walk commands are
ignored** while the pad holds the token — the pad's sticks drive instead. Switch the source on
the *Control method* card to drive from the browser. Nothing is broken; it just needs one
click that did not exist before.

Also new for legs: E-STOP now parks joints in DAMPING rather than IDLE, so a stopped limb
resists instead of collapsing.

## Two machine profiles

Same repo, same units, different environment. Both are set in `humanoid-web.service`.

| | legs robot | arm / Quest workstation |
|---|---|---|
| `HUMANOID_GAMEPAD_ENABLE` | `1` — the pad drives the legs | `0`, unless a pad is wanted |
| `HUMANOID_QUEST_ENABLE` | unset | **`1`** — nothing else starts the bridge |
| `--imu-device` in the daemon unit | **yes** — the walk policy needs base state | not needed |
| enabled limbs | both legs | the arm(s) |

Enabled limbs live in `~/.config/humanoid-control/robot_layout.json` (Settings tab writes it)
and are **machine-local** — they do not travel with git, so each robot needs its own. So does
`~/.config/humanoid-control/arm_profiles.json`, the per-operator Quest arm calibration.

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

## Quest 3 arm teleop (the workstation profile)
Set `HUMANOID_QUEST_ENABLE=1` in `humanoid-web.service` — nothing else starts the bridge, and
without it the headset loads `/xr/` and the websocket is simply never served.

**Getting the page into the headset.** WebXR needs a *secure context*, and a self-signed
certificate you click through does **not** qualify — Chromium keeps withholding WebXR from such
an origin. The dependable route is a USB cable:
```bash
adb reverse tcp:8000 tcp:8000        # then open http://localhost:8000/xr/ on the headset
```
`localhost` is trusted with no certificate at all. Wireless `adb connect <headset-ip>:5555`
gives the same tunnel if a cable is impractical. TLS on 8443 exists as a fallback
(`HUMANOID_QUEST_CERT` / `HUMANOID_QUEST_KEY`).

**Body tracking** needs *WebXR Experiments* enabled in `chrome://flags` on the headset. Without
it the controller-position path still works, but whole-arm mirroring does not.

**Per-operator calibration.** Mirroring maps the operator's measured joint range onto the
robot's, so each person needs a profile: four guided poses, prompted on the in-headset HUD,
stored in `~/.config/humanoid-control/arm_profiles.json`. Without one the arm falls back to
controller-position mode and the log says so.

**Controls.** Trigger = deadman (hold to drive, release to rest) · A arms · B disarms ·
Y = E-STOP. Releasing the trigger rests the arm; losing the link entirely E-STOPs it.

## Notes
- **One daemon owns CAN.** Stop the humanoid-studio app's daemon before enabling this one.
- **`deploy/freeze-monitor.service` is for the training PC only.** It samples machine state to
  disk to diagnose that machine's hard freezes, and its `ExecStart` hardcodes that checkout's
  path. Do not install it on a robot without fixing the path first.
- **`scripts/start_stack.sh` is a bench convenience**, not the robot entry point — on a robot
  the systemd units are what run the stack. It does derive its paths from its own location, so
  it works from any checkout.
- **CAN adapters must be up** before `humanoid-daemon.service` (see the comment in that unit).
- **Password.** On anything but a trusted LAN, set `HUMANOID_WEB_PASSWORD` in the web unit
  (it's plain HTTP — for internet exposure, front it with TLS, e.g. `tailscale serve`).
- **Rebuild the UI** after frontend changes: `cd app && npm run build` (no service restart needed;
  it's static). Restart `humanoid-web.service` after backend/Python changes.
- **Gamepad deadman** (optional, prepared but off): `pip install evdev`, pair a Bluetooth Xbox
  controller, then set `HUMANOID_GAMEPAD_ENABLE=1` in the web unit. See `humanoid_control/web/gamepad.py`.
