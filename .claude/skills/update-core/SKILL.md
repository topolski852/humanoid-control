---
name: update-core
description: Upgrade a robot PC's humanoid-control checkout past the humanoid-core split. Pulls, initialises the core/ submodule, rebuilds the daemon, repoints the installed systemd unit and restarts the stack. Use when asked to "run the update", "update to core", or when core/ is missing or empty after a pull.
---

# Upgrade this robot PC to the humanoid-core layout

The CAN daemon, its Python client and the arm modules moved out of this repo into the
`humanoid-core` submodule at `core/`. A plain `git pull` is not enough on a robot PC. The old
daemon binary survives in the ignored `daemon/build/`, and the installed systemd unit keeps
launching it. Nothing fails; every later daemon fix just silently doesn't apply. Your job is
to finish the move and prove the new binary is the one running.

## Ground rules

- **Restarting the daemon drops motor commands.** Before you stop any service, ask the user to
  confirm that no session is running and the robot is safe to go limp: legs on the gantry or
  sitting, arms resting. Wait for an explicit yes.
- **Never `cp deploy/humanoid-daemon.service` over the installed unit.** On the arms PC,
  `deploy/convert-to-arms-teleop.sh` rewrote it (arms config, no `--imu-device`). Copying the
  repo's unit would silently restore the full-body config. Edit the path in place, as below.
- Do not run motion scripts, `smoke_test.py --connect`, or `gen_viz_kinematics.py`. Do not
  commit or push.
- `sudo` may need a password, and you cannot type one. Check with `sudo -n true`. If it fails,
  give the user each sudo command to run themselves (they can prefix it with `!`), then carry on.

## Steps

Stop and report at the first step that doesn't check out. Don't improvise around it.

1. **Check the starting state.** The repo should be at `/home/nse/humanoid-control`, because
   the units hardcode that path. If it isn't, say so before going on. Run `git status`:
   tracked changes mean stop and show them. Record what is installed and whether this is
   the arms PC (its unit starts with a `# ARMS TELEOP` comment):
   ```bash
   systemctl cat humanoid-daemon | grep -E '^(# ARMS|WorkingDirectory|ExecStart)'
   systemctl is-active humanoid-daemon humanoid-web
   ```
   No units installed (a bench machine that uses `scripts/start_stack.sh`): do steps 2–4,
   then restart through `start_stack.sh` instead of steps 5–8.

2. **Pull and fetch core.**
   ```bash
   git pull
   git submodule update --init
   ```
   If `.gitmodules` has no `core` entry afterwards, the split isn't on this branch yet. Stop.
   Check that `core/daemon/Makefile` exists and that `humanoid_core` (a symlink at the repo
   root) resolves.

3. **Build the new daemon.** `(cd core/daemon && make)`. Confirm that
   `core/daemon/build/humanoid_daemon` exists.

4. **Check offline. No robot is involved and these are safe while the stack runs.**
   ```bash
   for t in core/tests/test_*.py; do .venv/bin/python "$t" | tail -1; done
   .venv/bin/python -c "import humanoid_control.web.server" && echo import-ok
   ```
   Every core test should end in `0 failed`.

5. **Get the user's go-ahead (see Ground rules), then stop the stack.**
   `sudo systemctl stop humanoid-web humanoid-daemon`

6. **Repoint the installed daemon unit in place.**
   ```bash
   sudo cp -a /etc/systemd/system/humanoid-daemon.service /etc/systemd/system/humanoid-daemon.service.pre-core
   sudo sed -i 's|/home/nse/humanoid-control/daemon|/home/nse/humanoid-control/core/daemon|g' /etc/systemd/system/humanoid-daemon.service
   diff /etc/systemd/system/humanoid-daemon.service.pre-core /etc/systemd/system/humanoid-daemon.service
   ```
   Only `WorkingDirectory` and the binary path in `ExecStart` may change. `--config`, any
   `--imu-device` and the `# ARMS TELEOP` comment must be exactly as they were in step 1.
   The web and IMU-init units need no edit.

7. **Remove the old daemon folder.** First confirm git no longer tracks anything in it:
   `git ls-files daemon` must print nothing. Then run `rm -rf daemon/`.

8. **Restart and prove it.** Start the daemon first, then the web server:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl start humanoid-daemon && sleep 3 && sudo systemctl start humanoid-web
   systemctl is-active humanoid-daemon humanoid-web
   systemctl show -p ExecStart humanoid-daemon      # must name core/daemon/build/
   pgrep -af humanoid_daemon                        # the running process, same path
   journalctl -u humanoid-daemon -n 30 --no-pager
   ```
   Then confirm that the daemon answers. This only checks the network reply; it doesn't wake
   the motors:
   ```bash
   python3 -c 'import json,socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.settimeout(5);s.sendto(json.dumps({"type":"PING"}).encode(),("127.0.0.1",9001));print(s.recvfrom(65535)[0][:80])'
   ```
   If either service is not active, show the journal. The fallback is to restore
   `humanoid-daemon.service.pre-core`, then `daemon-reload` and start. That only works before
   step 7, so if `daemon/` is already gone, rebuild from `git log` or ask the user.

## Report

Finish with a short summary:
- the machine (hostname, and arms or full-body)
- the old and new `ExecStart`
- the core submodule commit (`git -C core log --oneline -1`)
- test results
- whether both services are active and the daemon answered PING
- anything you skipped and why
