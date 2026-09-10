#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# convert-to-arms-teleop.sh
#
# Run ONCE on a PC restored from a clone of the full-humanoid control PC, to
# turn it into the arms-only Quest teleop machine.
#
# The clone arrives with the source PC's identity (hostname, machine-id, SSH
# host keys) and its full-body configuration (leg CAN adapters by serial, an
# IMU that this machine does not have, a gamepad that would steal the input
# token from the headset). This script fixes all of that in one pass.
#
# USAGE
#   1. Boot the restored PC with the arm CAN adapters UNPLUGGED.
#   2. Plug in ONE arm adapter, run:  sudo ./convert-to-arms-teleop.sh --discover
#      Note the serial. Repeat for the other adapter.
#   3. sudo ./convert-to-arms-teleop.sh \
#          --hostname nse-ARMS \
#          --left-arm-serial  <SERIAL> \
#          --right-arm-serial <SERIAL>
#   4. Reboot. Verify with --verify.
#
# Idempotent: safe to re-run. Every file it edits is backed up to *.pre-arms.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

REPO=/home/nse/humanoid-control
STUDIO=/home/nse/humanoid-studio
CFG_SRC="$STUDIO/configs/humanoid_lite.json"
CFG_ARMS="$STUDIO/configs/humanoid_arms.json"
CAN_RULES=/etc/udev/rules.d/99-humanoid-can.rules
DAEMON_UNIT=/etc/systemd/system/humanoid-daemon.service
WEB_UNIT=/etc/systemd/system/humanoid-web.service
UNITS=(humanoid-daemon humanoid-web humanoid-imu-init)

NEW_HOSTNAME=""; LEFT_SERIAL=""; RIGHT_SERIAL=""
DO_DISCOVER=0; DO_VERIFY=0

die() { echo "ERROR: $*" >&2; exit 1; }
say() { echo -e "\033[1;36m==>\033[0m $*"; }
ok()  { echo -e "  \033[1;32m✓\033[0m $*"; }
warn(){ echo -e "  \033[1;33m!\033[0m $*"; }

backup() { [ -f "$1" ] && [ ! -f "$1.pre-arms" ] && cp -a "$1" "$1.pre-arms" && ok "backed up $1 -> $1.pre-arms" || true; }

while [ $# -gt 0 ]; do
  case "$1" in
    --hostname)          NEW_HOSTNAME="$2"; shift 2 ;;
    --left-arm-serial)   LEFT_SERIAL="$2";  shift 2 ;;
    --right-arm-serial)  RIGHT_SERIAL="$2"; shift 2 ;;
    --discover)          DO_DISCOVER=1; shift ;;
    --verify)            DO_VERIFY=1;   shift ;;
    -h|--help)           sed -n '2,30p' "$0"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

# ── --discover ───────────────────────────────────────────────────────────────
if [ "$DO_DISCOVER" = 1 ]; then
  say "CAN adapters currently visible (gs_usb net devices)"
  found=0
  for iface in /sys/class/net/*; do
    n=$(basename "$iface")
    [ "$n" = "lo" ] && continue
    [ -d "$iface/device" ] || continue
    drv=$(basename "$(readlink -f "$iface/device/driver" 2>/dev/null)" 2>/dev/null || echo "")
    ser=$(udevadm info "/sys/class/net/$n" --query=property 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')
    if [ -n "$ser" ]; then
      echo "    interface=$n  driver=$drv  ID_SERIAL_SHORT=$ser"
      found=1
    fi
  done
  [ "$found" = 0 ] && warn "no CAN adapter with a serial found — is one plugged in?"
  exit 0
fi

# ── --verify ─────────────────────────────────────────────────────────────────
if [ "$DO_VERIFY" = 1 ]; then
  say "Arms teleop machine — verification"
  echo "  hostname      : $(hostname)"
  echo "  machine-id    : $(cat /etc/machine-id 2>/dev/null)"
  echo "  ssh host key  : $(ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub 2>/dev/null | awk '{print $2}')"
  echo "  CAN links     :"; ip -br link show type can 2>/dev/null | sed 's/^/    /' || echo "    (none)"
  echo "  units         :"
  for u in "${UNITS[@]}"; do
    printf "    %-22s %-10s %s\n" "$u" "$(systemctl is-enabled $u 2>&1)" "$(systemctl is-active $u 2>&1)"
  done
  echo "  daemon config : $(grep -oE '\-\-config [^ ]+' "$DAEMON_UNIT" | head -1)"
  echo "  imu flag      : $(grep -q -- '--imu-device' "$DAEMON_UNIT" && echo 'STILL PRESENT (bad)' || echo 'removed (good)')"
  echo "  quest bridge  : $(grep -qE '^Environment=HUMANOID_QUEST_ENABLE=1' "$WEB_UNIT" && echo 'enabled (good)' || echo 'NOT enabled (bad)')"
  echo "  gamepad       : $(grep -qE '^Environment=HUMANOID_GAMEPAD_ENABLE=1' "$WEB_UNIT" && echo 'ENABLED — will steal the input token (bad)' || echo 'off (good)')"
  exit 0
fi

[ "$(id -u)" = 0 ] || die "must run as root (sudo $0 ...)"
[ -n "$NEW_HOSTNAME" ]  || die "--hostname is required"
[ -n "$LEFT_SERIAL" ]   || die "--left-arm-serial is required (get it with --discover)"
[ -n "$RIGHT_SERIAL" ]  || die "--right-arm-serial is required (get it with --discover)"
[ "$LEFT_SERIAL" != "$RIGHT_SERIAL" ] || die "left and right serials are identical"
[ -f "$CFG_SRC" ] || die "missing $CFG_SRC"

# ── 1. Stop and disable the inherited control stack ──────────────────────────
say "Stopping the inherited full-humanoid control stack"
for u in "${UNITS[@]}"; do
  systemctl stop "$u" 2>/dev/null || true
  systemctl disable "$u" 2>/dev/null || true
  ok "stopped + disabled $u"
done
warn "units are left DISABLED — re-enable deliberately once you have verified the arms"

# ── 2. Machine identity ──────────────────────────────────────────────────────
say "Resetting machine identity (clone shares the source PC's)"
OLD_HOSTNAME=$(hostname)
hostnamectl set-hostname "$NEW_HOSTNAME"
if [ -f /etc/hosts ]; then
  backup /etc/hosts
  sed -i "s/\b${OLD_HOSTNAME}\b/${NEW_HOSTNAME}/g" /etc/hosts
fi
ok "hostname $OLD_HOSTNAME -> $NEW_HOSTNAME"

rm -f /etc/machine-id /var/lib/dbus/machine-id
systemd-machine-id-setup >/dev/null 2>&1
mkdir -p /var/lib/dbus && ln -sf /etc/machine-id /var/lib/dbus/machine-id
ok "new machine-id: $(cat /etc/machine-id)   (was shared with the humanoid PC — DHCP DUID collision)"

rm -f /etc/ssh/ssh_host_*
ssh-keygen -A >/dev/null 2>&1
ok "regenerated SSH host keys"

# ── 3. CAN udev rules: legs out, arms in ─────────────────────────────────────
say "Rewriting $CAN_RULES for the arm adapters"
backup "$CAN_RULES"
canrule() { # $1=name $2=serial
  cat <<EOR
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="$2", NAME="$1", RUN+="/bin/sh -c '/usr/bin/ip link set $1 type can bitrate 1000000 restart-ms 100 2>/dev/null || /usr/bin/ip link set $1 type can bitrate 1000000; /usr/bin/ip link set $1 txqueuelen 1000; /usr/bin/ip link set $1 up'"
EOR
}
{
  echo "# Humanoid ARMS TELEOP machine — persistent CAN interface naming"
  echo "# Generated by deploy/convert-to-arms-teleop.sh on $(date -Iseconds)"
  echo "# This machine drives the ARMS ONLY. The leg adapters live on the humanoid PC."
  echo "#"
  echo "#   can_left_arm  -> serial $LEFT_SERIAL"
  echo "#   can_right_arm -> serial $RIGHT_SERIAL"
  echo "#"
  echo "# Re-discover serials with: sudo $REPO/deploy/convert-to-arms-teleop.sh --discover"
  echo
  echo "# ── Left arm ────────────────────────────────────────────────────────────────"
  canrule can_left_arm "$LEFT_SERIAL"
  echo
  echo "# ── Right arm ───────────────────────────────────────────────────────────────"
  canrule can_right_arm "$RIGHT_SERIAL"
} > "$CAN_RULES"
udevadm control --reload-rules
ok "wrote arm rules and reloaded udev"

# ── 4. Arms-only daemon config ───────────────────────────────────────────────
say "Generating arms-only config -> $CFG_ARMS"
python3 - "$CFG_SRC" "$CFG_ARMS" "$LEFT_SERIAL" "$RIGHT_SERIAL" <<'EOPY'
import json, sys
src, dst, ls, rs = sys.argv[1:5]
d = json.load(open(src))
keep = {n: j for n, j in d["joints"].items()
        if str(j.get("can_channel", "")) in ("can_left_arm", "can_right_arm")}
if not keep:
    sys.exit("no arm joints found in source config")
d["joints"] = keep
d["can_assignments"] = {ls: "left_arm", rs: "right_arm"}
d["robot_name"] = "humanoid_arms_teleop"
json.dump(d, open(dst, "w"), indent=2)
print(f"  kept {len(keep)} arm joints, dropped the leg joints")
for n, j in keep.items():
    print(f"    {n:30s} {j['can_channel']:14s} id={j['can_id']}")
EOPY
chown nse:nse "$CFG_ARMS"
ok "wrote $CFG_ARMS"

# ── 5. Daemon unit: arms config, no IMU ──────────────────────────────────────
say "Rewriting $DAEMON_UNIT (arms config, IMU removed)"
backup "$DAEMON_UNIT"
sed -i -E "s|--config [^ ]+|--config $CFG_ARMS|" "$DAEMON_UNIT"
sed -i -E "s| --imu-device [^ ]+||" "$DAEMON_UNIT"
grep -q '^# ARMS TELEOP' "$DAEMON_UNIT" || \
  sed -i "1i # ARMS TELEOP machine: no IMU on this PC, so the --imu-device flag is removed and\n# the daemon reports base:null (upright stub), which is what arm teleop expects." "$DAEMON_UNIT"
ok "config -> $CFG_ARMS, --imu-device removed"

# ── 6. Web unit: Quest on, gamepad off ───────────────────────────────────────
say "Rewriting $WEB_UNIT (Quest bridge on, gamepad off)"
backup "$WEB_UNIT"
# Gamepad must not be enabled: it takes the input token at startup and the
# headset's writes would be silently dropped as ignored_writes.
sed -i -E 's|^Environment=HUMANOID_GAMEPAD_ENABLE=1|Environment=HUMANOID_GAMEPAD_ENABLE=0|' "$WEB_UNIT"
# Quest bridge: nothing else starts it.
if grep -qE '^Environment=HUMANOID_QUEST_ENABLE=' "$WEB_UNIT"; then
  sed -i -E 's|^Environment=HUMANOID_QUEST_ENABLE=.*|Environment=HUMANOID_QUEST_ENABLE=1|' "$WEB_UNIT"
else
  sed -i -E 's|^(# Environment=HUMANOID_QUEST_ENABLE=1)|Environment=HUMANOID_QUEST_ENABLE=1|' "$WEB_UNIT"
fi
grep -qE '^Environment=HUMANOID_QUEST_ENABLE=1' "$WEB_UNIT" || \
  die "could not enable HUMANOID_QUEST_ENABLE — edit $WEB_UNIT by hand"
sed -i -E "s|^Environment=HUMANOID_CONFIG=.*|Environment=HUMANOID_CONFIG=$CFG_ARMS|" "$WEB_UNIT"
ok "HUMANOID_QUEST_ENABLE=1, HUMANOID_GAMEPAD_ENABLE=0, config -> $CFG_ARMS"

systemctl daemon-reload
ok "systemd reloaded"

# ── 7. Report ────────────────────────────────────────────────────────────────
say "Done. Remaining manual steps:"
cat <<EOF

  1. REBOOT, then plug in both arm CAN adapters and confirm the interfaces:
         ip -br link show type can
     Expect: can_left_arm and can_right_arm, both UP.

  2. Set a web password before exposing this machine — it currently serves
     0.0.0.0:8000 with HUMANOID_WEB_PASSWORD commented out, and there are now
     TWO robots on your network:
         sudo systemctl edit humanoid-web
         [Service]
         Environment=HUMANOID_WEB_PASSWORD=<something long>

  3. The arm joint calibration in $CFG_ARMS was copied from the humanoid PC.
     Electrical offsets, gear signs and position offsets are PER-ROBOT — verify
     or re-run calibration on this machine's arms before commanding motion.

  4. Re-enable the stack when you are satisfied:
         sudo systemctl enable --now humanoid-daemon humanoid-web

  5. Verify:  sudo $0 --verify

EOF
