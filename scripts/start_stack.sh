#!/bin/bash
# Bring the robot stack up IN ORDER, waiting for each layer to be genuinely ready
# before starting the next.
#
# WHY THE ORDER MATTERS. Each layer assumes the one below it already answers:
#
#   1. CAN interface   the daemon opens a socket on it at startup and never retries.
#                      Start the daemon first and it runs blind — the joints read
#                      OFFLINE forever with no error, because the bus it wanted did
#                      not exist when it looked.
#   2. daemon          the web server's DaemonClient binds its telemetry port and
#                      starts a receive loop the moment it constructs. Start the web
#                      server first and it spends its life talking to nobody; the
#                      daemon appearing later does not re-establish anything.
#   3. web server      owns the ControlService the Quest bridge attaches to.
#   4. Quest           adb reverse tunnel + the headset page.
#
# Each step below WAITS for a positive readiness signal rather than sleeping and
# hoping. "Started" is not the same as "ready", and every ordering bug this script
# exists to prevent came from conflating the two.
#
# Usage:  scripts/start_stack.sh [--with-quest] [--quest-ip 192.168.0.149]
set -uo pipefail

REPO=/home/nse/humanoid/humanoid-control
CONFIG=${HUMANOID_CONFIG:-/home/nse/.config/humanoid-studio/humanoid_lite.json}
LOGDIR=${HUMANOID_LOGDIR:-/home/nse/humanoid-logs}
QUEST_IP=192.168.0.149
WITH_QUEST=0
CAN_IF=can_left_arm

while [ $# -gt 0 ]; do
  case "$1" in
    --with-quest) WITH_QUEST=1 ;;
    --quest-ip)   QUEST_IP="$2"; shift ;;
    *) echo "unknown arg: $1"; exit 2 ;;
  esac
  shift
done

mkdir -p "$LOGDIR"
ok()   { echo "  ✓ $*"; }
fail() { echo "  ✗ $*"; exit 1; }

# Never use `pgrep -f` to test for these: this script's own command line contains
# the pattern, so it matches itself. Ask the kernel who holds the port instead.
daemon_up() { ss -ulnp 2>/dev/null | grep -q "127.0.0.1:9001"; }
web_up()    { ss -ltn  2>/dev/null | grep -q ":8000"; }

# ── 1. CAN ───────────────────────────────────────────────────────────────────
echo "[1/4] CAN interface"
if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
  fail "$CAN_IF does not exist — is the adapter plugged in? (udev rule: /etc/udev/rules.d/99-humanoid-can.rules)"
fi
if ! ip -br link show "$CAN_IF" | grep -q UP; then
  fail "$CAN_IF exists but is DOWN — replug the adapter so udev reconfigures it"
fi
ok "$CAN_IF up at $(ip -details link show "$CAN_IF" | grep -oE 'bitrate [0-9]+' | head -1)"

# ── 2. daemon ────────────────────────────────────────────────────────────────
echo "[2/4] CAN daemon"
if daemon_up; then
  ok "already running"
else
  ( cd "$REPO" && setsid nohup stdbuf -o0 -e0 ./daemon/build/humanoid_daemon \
      --config "$CONFIG" --rt-prio 0 > "$LOGDIR/daemon.log" 2>&1 < /dev/null & )
  for i in $(seq 1 20); do daemon_up && break; sleep 0.5; done
  daemon_up || fail "daemon did not bind :9001 — see $LOGDIR/daemon.log"
  ok "started"
fi

# READY = it answers, not merely that the port is bound.
python3 - <<'PY' || fail "daemon bound its port but does not answer PING"
import json, socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(5.0)
try:
    s.sendto(json.dumps({"type": "PING"}).encode(), ("127.0.0.1", 9001))
    sys.exit(0 if b"PONG" in s.recvfrom(65535)[0] else 1)
except Exception:
    sys.exit(1)
PY
ok "answers PING"

# ── 3. web server ────────────────────────────────────────────────────────────
echo "[3/4] web server"
if web_up; then
  ok "already running"
else
  ( cd "$REPO" && PYTHONUNBUFFERED=1 HUMANOID_WEB_HOST=0.0.0.0 HUMANOID_WEB_PORT=8000 \
      HUMANOID_GAMEPAD_ENABLE=${HUMANOID_GAMEPAD_ENABLE:-1} \
      HUMANOID_QUEST_ENABLE=${HUMANOID_QUEST_ENABLE:-1} \
      setsid nohup .venv/bin/python -m humanoid_control.web \
      > "$LOGDIR/web.log" 2>&1 < /dev/null & )
  for i in $(seq 1 40); do web_up && break; sleep 0.5; done
  web_up || fail "web server did not listen on :8000 — see $LOGDIR/web.log"
  ok "started"
fi
for i in $(seq 1 20); do
  curl -sf --max-time 3 http://127.0.0.1:8000/api/status >/dev/null 2>&1 && break
  sleep 0.5
done
curl -sf --max-time 3 http://127.0.0.1:8000/api/status >/dev/null 2>&1 \
  || fail "web server is listening but /api/status does not answer"
ok "serving /api/status"

# ── 4. Quest (optional) ──────────────────────────────────────────────────────
if [ "$WITH_QUEST" = 1 ]; then
  echo "[4/4] Quest"
  command -v adb >/dev/null || fail "adb not installed"
  adb connect "$QUEST_IP:5555" >/dev/null 2>&1
  sleep 2
  if adb devices 2>/dev/null | grep -q "$QUEST_IP:5555[[:space:]]*device"; then
    adb -s "$QUEST_IP:5555" reverse tcp:8000 tcp:8000 >/dev/null 2>&1 \
      && ok "reverse tunnel up — open http://localhost:8000/xr/ on the headset" \
      || fail "could not create the reverse tunnel"
  else
    echo "  ! headset not reachable (asleep? put it on, then re-run with --with-quest)"
  fi
else
  echo "[4/4] Quest — skipped (pass --with-quest)"
fi

echo
echo "Stack up. Logs in $LOGDIR/"
echo "  UI:  http://$(hostname -I | awk '{print $1}'):8000"
