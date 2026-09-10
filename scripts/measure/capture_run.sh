#!/usr/bin/env bash
# Capture one labelled measurement run, then analyse it.
#
# The capture is a PASSIVE candump — it takes a copy of frames off the CAN interface and does
# NOT open a daemon telemetry socket. That matters: the daemon broadcasts telemetry to a single
# UDP destination (127.0.0.1:9000), so any second client steals packets from the web service
# that runs the policy. An earlier version of this suite did exactly that and forced a run to be
# aborted. Nothing here may use DaemonClient.
#
# Start the policy first, then run this alongside it.
#
#   scripts/measure/capture_run.sh <label> [seconds] [activity]
#   scripts/measure/capture_run.sh smoothA-stand 600 hold
#   scripts/measure/capture_run.sh smoothA-walk  120 walk
set -euo pipefail

LABEL="${1:?usage: capture_run.sh <label> [seconds] [activity]}"
SECS="${2:-60}"
ACTIVITY="${3:-walk}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
OUTDIR="docs/measurements"
mkdir -p "$OUTDIR"

echo "=== capture '$LABEL'  ${SECS}s  activity=$ACTIVITY ==="
python scripts/measure/m1_can_timing.py --seconds "$SECS" --label "$LABEL" \
       --activity "$ACTIVITY" --outdir "$OUTDIR"

CAP="$(ls -t "$OUTDIR"/*_"$LABEL"_can.json 2>/dev/null | head -1)"
if [[ -z "$CAP" ]]; then
  echo "!! no capture produced for '$LABEL'" >&2
  exit 1
fi

echo
echo "=== M2 / M3 / M6 ==="
python scripts/measure/analyse.py --capture "$CAP"
