#!/usr/bin/env bash
# Start power_log.py the moment a policy engages (state RUNNING), detached, for N seconds.
#   scripts/measure/power_on_engage.sh <label> [seconds]
LABEL="${1:?label}"; SECS="${2:-90}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$REPO"
until [[ "$(curl -s localhost:8000/api/status | python3 -c "import json,sys;print(json.load(sys.stdin)['data']['state'])" 2>/dev/null)" == "RUNNING" ]]; do sleep 0.3; done
setsid nohup .venv/bin/python scripts/measure/power_log.py --seconds "$SECS" --label "$LABEL" > "/tmp/power_log_${LABEL}.log" 2>&1 < /dev/null &
echo "power log started $(date +%T) -> /tmp/power_log_${LABEL}.log"
