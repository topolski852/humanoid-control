# Freeze baseline — recorded before the BIOS 1804 → 2402 flash

Hardware: Ryzen 9 9950X · ASUS ROG STRIX X870-A GAMING WIFI · BIOS **1804** (2025-11-12,
AGESA ComboAM5 PI 1.2.7.0) · 2×32 GB G.Skill F5-6000J3636F32G running at **JEDEC 4800**
(EXPO off) · cmdline `quiet splash amd_iommu=off iommu=soft`

## How this was measured, and the mistake it corrects

Boot DURATION is not evidence of a crash. Boots end for ordinary reasons, and using "short
boot" as a proxy for "crashed" produced two wrong conclusions in a row — first a kernel
regression story built on counting clean shutdowns as deaths, then a second kernel story
built on the same proxy in the other direction.

What actually distinguishes them is whether systemd got to write a shutdown sequence:
`reboot.target`, `systemd-shutdown`, `Journal stopped`. A crash has none of these. Its last
line is whatever routine thing happened to be logged, which is why it is usually something
as unremarkable as `sysstat-collect.service: Deactivated successfully`.

## The record

| boot | kernel | started | duration | ended |
|---|---|---|---|---|
| -27 | 6.17.0-35 | 06-28 12:48 | 3d 22h | **CRASH** |
| -25 | 6.17.0-35 | 07-03 11:21 | 7d 13h | clean |
| -24 | 6.17.0-35 | 07-14 12:35 | **26d 22h** | clean |
| -23 | 7.0.0-28 | 08-10 11:16 | 2d 08h | **CRASH** |
| -22 | 7.0.0-28 | 08-12 19:49 | 7d 21h | clean |
| -21 | 7.0.0-29 | 08-20 16:57 | 5d 23h | **CRASH** |
| -18 … 0 | mostly 7.0.0-30 | 08-26 16:51 → | 1.5–248 min | **15 crashes** |

## What this rules out

**The kernel.** Crashes occur on 6.17.0-35, 7.0.0-28, 7.0.0-29 and 7.0.0-30 — four kernels
across two upstream series, the earliest on 2026-06-28. No kernel is clean.

**RAM overclock.** EXPO is off and both DIMMs report 4800 MT/s configured, against a 6000
rating. The instability predates and outlives that change.

**The Quest stack, and load in general.** The 2026-08-27 21:25 crash was captured by
`freeze_monitor.py` two seconds before death: load 0.66 and FALLING, 55.7 GB free, WiFi at
0.8 KB/s, no WebXR session ever opened, no websocket. The machine died at rest.

## What is left

A platform-level stop. The NMI watchdog is armed (`NMI watchdog: Enabled`) and has never
once fired; there is no MCE, no panic, and `pstore` is empty every time. A kernel deadlocked
in software would have produced a stack trace from the watchdog. Nothing is taking NMIs,
which puts the failure below the OS: firmware, power delivery, or the chipset link.

Two behaviours to explain, and they may not share a cause:

1. A **rare** instability present since at least June — one crash every 2–6 days, against
   uptimes as long as 27 days.
2. A **step change on 2026-08-26**, after which nothing survives half an hour. The last clean
   boot ended 16:19 that day; `adb` and the Android platform tools were installed at 15:43,
   and the Quest was first attached over USB around the same window. Correlation only — the
   USB tree has since been disconnected entirely during crashes.

## Pass criterion after the flash

Not "it stayed up a while". This machine produced a 4h08m boot on the current BIOS in the
middle of the bad run, and 26 days before it. The bar is:

* **no crash inside the first hour**, across several boots, and
* **a multi-day uptime** reproduced at least once.

`freeze-monitor.service` runs from boot and is independent of the robot stack, so the next
crash is captured whatever it is doing.
