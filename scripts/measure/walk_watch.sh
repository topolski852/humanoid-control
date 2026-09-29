#!/usr/bin/env bash
# Arm a capture for the next policy engage, and verify the network within seconds.
#
#   scripts/measure/walk_watch.sh <label> <expected_bundle> [seconds] [activity]
#   scripts/measure/walk_watch.sh measC-walk walk_measC-full_2026-09-28 90 walk
#
# Waits for the web service to reach RUNNING (trigger held / policy started), starts a detached
# passive capture (capture_run.sh: candump, no daemon socket), then replays the live tick log
# through every staged ONNX and prints which network is actually running. The service does not
# log the checkpoint it loaded, and the dropdown resets to "walk" on reload, so this check is
# what makes an A/B trustworthy. Exits right after the check; the capture keeps running and
# logs to /tmp/walk_watch_<label>.log.
set -uo pipefail
LABEL="${1:?label}"; EXPECT="${2:?expected bundle name}"; SECS="${3:-90}"; ACT="${4:-walk}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
LOG="/tmp/walk_watch_${LABEL}.log"
echo "armed watcher for $LABEL (expect $EXPECT) — waiting for engage ..."
while true; do
  st=$(curl -s localhost:8000/api/status | python3 -c "import json,sys;print(json.load(sys.stdin)['data']['state'])" 2>/dev/null)
  [[ "$st" == "RUNNING" ]] && break
  sleep 0.3
done
echo "ENGAGED at $(date +%T)"
setsid nohup scripts/measure/capture_run.sh "$LABEL" "$SECS" "$ACT" > "$LOG" 2>&1 < /dev/null &
sleep 4
.venv/bin/python - "$EXPECT" <<'PY'
import sys, glob, os
sys.path.insert(0, "scripts/measure")
import stand_metrics as S
f = max(glob.glob("recordings/run_*.jsonl"), key=os.path.getmtime)
r = S.identify_policy(f, n=60)
ok = r["policy"] == sys.argv[1] and r["exact"]
print(f"recording {os.path.basename(f)}\nNETWORK: {r['policy']} exact={r['exact']}  -> "
      f"{'OK' if ok else 'WRONG POLICY'}")
PY
echo "capture log: $LOG"
