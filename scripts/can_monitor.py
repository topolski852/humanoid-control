#!/usr/bin/env python3
"""Live CAN health monitor for the humanoid legs.

Passively watches every leg CAN bus (candump, read-only — safe to run alongside
the daemon and during a walk/stand run) and tracks, per motor node:
  - liveness (last frame age) and per-second frame rate,
  - which frame types it is emitting (PDO4 fast-frame, heartbeat, EMCY, ...),
and, per bus, the kernel CAN error counters + controller state.

It prints a refreshing table AND appends a timestamped line to an event log the
instant anything changes: a node goes SILENT or comes back, a bus error counter
(bus-off / error-passive / error-warn / arb-lost / bus-errors) increments, or a
bus changes CAN state. That log is what tells you WHAT happened at the moment a
motor drops offline during a run.

Usage:
    python3 scripts/can_monitor.py                 # watch all UP leg buses
    python3 scripts/can_monitor.py --log run1.log  # also tee events to run1.log
    python3 scripts/can_monitor.py --chan can_right_leg

Node->joint map is read live from the studio config so it never drifts.
"""
import argparse, json, os, re, subprocess, sys, time
from collections import defaultdict

CONFIG = "/home/nse/humanoid-studio/configs/humanoid_lite.json"
SILENT_S = 1.5   # matches the daemon's 1500 ms OFFLINE threshold

FUNC = {0x0: "NMT<", 0x1: "EMCY", 0x3: "TPDO1", 0x4: "ping<", 0x5: "PDO2",
        0x6: "cmd<", 0x7: "TPDO3", 0x9: "PDO4", 0xA: "RPDO4<", 0xB: "SDOr",
        0xC: "SDOw<", 0xD: "FLASH<", 0xE: "HB"}   # '<' = host->node (command TO motor)


def load_nodes(want_chan):
    cfg = json.load(open(CONFIG))
    out = {}  # (chan, node_id) -> joint_name

    def walk(o):
        if isinstance(o, dict):
            if "can_id" in o and "can_channel" in o:
                yield o
            for v in o.values():
                yield from walk(v)
        elif isinstance(o, list):
            for v in o:
                yield from walk(v)
    for j in walk(cfg):
        ch = j["can_channel"]
        if "leg" not in ch:
            continue
        if want_chan and ch != want_chan:
            continue
        out[(ch, j["can_id"])] = j.get("joint_name", "?")
    return out


def up_channels():
    r = subprocess.run(["ip", "-o", "link", "show", "type", "can"],
                       capture_output=True, text=True)
    chans = []
    for line in r.stdout.splitlines():
        m = re.search(r"\d+:\s+(can_\S+?):.*\bUP\b", line)
        if m:
            chans.append(m.group(1))
    return chans


def bus_counters(chan):
    r = subprocess.run(["ip", "-s", "-d", "link", "show", chan],
                       capture_output=True, text=True)
    state = "?"
    ms = re.search(r"can state (\S+)", r.stdout)
    if ms:
        state = ms.group(1)
    cnt = {}
    lines = r.stdout.splitlines()
    for i, ln in enumerate(lines):
        if "re-started" in ln and "bus-errors" in ln:
            nums = re.findall(r"\d+", lines[i + 1])
            keys = ["restarted", "bus_err", "arb_lost", "err_warn", "err_pass", "bus_off"]
            if len(nums) >= 6:
                cnt = dict(zip(keys, map(int, nums[:6])))
            break
    return state, cnt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chan", default=None, help="watch only this channel")
    ap.add_argument("--log", default=None, help="append event lines to this file")
    ap.add_argument("--interval", type=float, default=1.0, help="table refresh seconds")
    ap.add_argument("--vdrop", type=float, default=2.5,
                    help="log a VOLT-DIP event when a joint's bus_voltage falls this many volts "
                         "below its own running baseline (catches a leg sagging under load)")
    args = ap.parse_args()

    nodes = load_nodes(args.chan)
    if not nodes:
        print("No leg joints found in config for the requested channel.", file=sys.stderr)
        sys.exit(1)
    chans = up_channels()
    if args.chan:
        chans = [c for c in chans if c == args.chan]
    if not chans:
        print("No UP can_* interfaces (legs powered off?).", file=sys.stderr)
        sys.exit(1)

    logf = open(args.log, "a") if args.log else None

    # ordered joint list (matches table order) for the voltage time-series CSV
    ordered = sorted(nodes, key=lambda k: (k[0], k[1]))
    ordered_names = [nodes[k] for k in ordered]

    # Voltage time-series: one row/interval of each joint's MIN bus_voltage that interval.
    # Auto-created next to the event log so a clean run still yields a full voltage profile.
    vcsvf = None
    if args.log:
        vcsvf = open(args.log + ".voltage.csv", "a")
        vcsvf.write("time," + ",".join(ordered_names) + "\n")
        vcsvf.flush()

    def event(msg):
        line = f"{time.strftime('%H:%M:%S')}  {msg}"
        print("  !! " + line)
        if logf:
            logf.write(time.strftime("%Y-%m-%dT%H:%M:%S ") + msg + "\n")
            logf.flush()

    # candump all wanted channels in one stream (-ta = absolute ts).
    cmd = ["candump", "-ta"] + chans
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)

    # bus_voltage is decoded PASSIVELY from the SDO read-responses already on the CAN wire
    # (func TSDO=0xB, arb 0x580|node; byte0=0x43, bytes1-2=param_id LE, bytes4-7=value f32 LE).
    # The daemon slow-polls PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED (0x100) ~3 Hz/joint. This
    # needs NO socket and NO port — so it never contends with the control stack's UDP 9000.
    SDO_RESP_CMD  = 0x43
    PARAM_VBUS    = 0x100
    volts = {}       # joint_name -> latest bus_voltage
    vmax = {}        # joint_name -> running baseline (max seen) voltage
    vmin_win = {}    # joint_name -> min voltage since last table redraw
    dip_active = {}  # joint_name -> bool, edge-triggers one VOLT-DIP per excursion

    last_seen = {}          # (chan,node) -> monotonic
    funcs_seen = defaultdict(set)  # (chan,node) -> set(func names)
    rate_win = defaultdict(list)   # (chan,node) -> [monotonic ...] last 1s
    silent = {k: True for k in nodes}   # start assuming silent
    prev_cnt = {c: bus_counters(c)[1] for c in chans}
    prev_state = {c: bus_counters(c)[0] for c in chans}

    print(f"Watching {', '.join(chans)}  |  {len(nodes)} leg nodes  "
          f"|  SILENT threshold {SILENT_S}s"
          + (f"  |  event log: {args.log}" if args.log else "") + "\n")

    next_tick = time.monotonic()
    # groups: 1=ts 2=chan 3=arb 4=data-hex (space-separated bytes, may be empty)
    line_re = re.compile(r"\(([\d.]+)\)\s+(can_\S+)\s+([0-9A-Fa-f]+)\s+\[\d+\]\s*([0-9A-Fa-f ]*)")
    import select, struct
    while True:
        # drain candump lines (passive; no sockets)
        while True:
            r, _, _ = select.select([proc.stdout], [], [], 0)
            if not r:
                break
            ln = proc.stdout.readline()
            if not ln:
                break
            m = line_re.search(ln)
            if not m:
                continue
            chan = m.group(2)
            arb = int(m.group(3), 16)
            node = arb & 0x7F
            func = (arb >> 7) & 0xF
            key = (chan, node)
            if key not in nodes:
                continue
            now = time.monotonic()
            last_seen[key] = now
            funcs_seen[key].add(FUNC.get(func, hex(func)))
            rate_win[key].append(now)

            # passive bus_voltage decode from SDO read-responses
            if func == 0xB and m.group(4):
                try:
                    data = bytes.fromhex(m.group(4).replace(" ", ""))
                except ValueError:
                    data = b""
                if (len(data) == 8 and data[0] == SDO_RESP_CMD
                        and (data[1] | (data[2] << 8)) == PARAM_VBUS):
                    bv = struct.unpack("<f", data[4:8])[0]
                    if 0.0 < bv < 100.0:
                        jn = nodes[key]
                        volts[jn] = bv
                        vmax[jn] = max(vmax.get(jn, bv), bv)
                        vmin_win[jn] = min(vmin_win.get(jn, bv), bv)
                        thresh = vmax[jn] - args.vdrop
                        if bv < thresh and not dip_active.get(jn):
                            dip_active[jn] = True
                            event(f"VOLT-DIP {jn:<24} {bv:5.2f} V  "
                                  f"(baseline {vmax[jn]:.1f}, -{vmax[jn]-bv:.1f} V)")
                        elif bv > thresh + 0.4 and dip_active.get(jn):
                            dip_active[jn] = False

        now = time.monotonic()
        if now < next_tick:
            time.sleep(min(0.05, next_tick - now))
            continue
        next_tick = now + args.interval

        # liveness transitions + bus counters
        for key in nodes:
            age = now - last_seen.get(key, -1e9)
            is_silent = age > SILENT_S
            if is_silent and not silent[key]:
                event(f"SILENT  {nodes[key]:<24} node {key[1]:>2} on {key[0]} "
                      f"(last frame {age:.1f}s ago)")
            elif not is_silent and silent[key]:
                event(f"ALIVE   {nodes[key]:<24} node {key[1]:>2} on {key[0]}")
            silent[key] = is_silent

        for c in chans:
            state, cnt = bus_counters(c)
            if state != prev_state[c]:
                event(f"BUS-STATE {c}: {prev_state[c]} -> {state}")
                prev_state[c] = state
            for k, v in cnt.items():
                pv = prev_cnt.get(c, {}).get(k, 0)
                if v > pv:
                    event(f"BUS-ERR   {c}: {k} {pv} -> {v}  (+{v-pv})")
            prev_cnt[c] = cnt

        # ---- draw table ----
        sys.stdout.write("\033[H\033[2J")  # home + clear (portable, no $TERM needed)
        print(f"CAN MONITOR  {time.strftime('%H:%M:%S')}   "
              f"buses: " + "  ".join(
                  f"{c}={prev_state[c]}(off:{prev_cnt[c].get('bus_off',0)},"
                  f"pass:{prev_cnt[c].get('err_pass',0)})" for c in chans))
        print("-" * 78)
        vhdr = f"{'busV':>7}{'minV':>7}"
        print(f"{'joint':<26}{'nd':>3} {'bus':>14} {'state':>8} {'Hz':>5}{vhdr}  frames")
        for key in sorted(nodes, key=lambda k: (k[0], k[1])):
            chan, node = key
            age = now - last_seen.get(key, -1e9)
            win = [t for t in rate_win[key] if now - t <= 1.0]
            rate_win[key] = win
            hz = len(win)
            st = "SILENT" if age > SILENT_S else "ok"
            frames = ",".join(sorted(funcs_seen[key])) if funcs_seen[key] else "-"
            flag = "  <== OFFLINE" if age > SILENT_S else ""
            bv = volts.get(nodes[key]); mn = vmin_win.get(nodes[key])
            vcell = (f"{bv:>7.2f}{mn:>7.2f}" if bv is not None
                     else f"{'-':>7}{'-':>7}")
            print(f"{nodes[key]:<26}{node:>3} {chan:>14} {st:>8} {hz:>5}{vcell}  {frames}{flag}")
        # write the interval's per-joint MIN voltage to the CSV, then reset the window
        if vcsvf is not None:
            row = [time.strftime("%H:%M:%S")]
            for nm in ordered_names:
                v = vmin_win.get(nm)
                row.append(f"{v:.2f}" if v is not None else "")
            vcsvf.write(",".join(row) + "\n")
            vcsvf.flush()
        # reset per-interval min to the latest reading so next interval tracks a fresh dip
        for jn in list(volts):
            vmin_win[jn] = volts[jn]
        print("-" * 78)
        print("Ctrl-C to stop." + (f"  Events -> {args.log}" if args.log else
                                    "  (use --log FILE to record events)"))
        sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped.")
