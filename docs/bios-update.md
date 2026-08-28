# BIOS 1804 → 2402 on the ROG STRIX X870-A GAMING WIFI

Current: **1804** (2025-11-12, AGESA ComboAM5 PI 1.2.7.0)
Target:  **2402** (2026-07-15, AGESA ComboAM5 PI 1.3.0.1b Patch A)

Nine months and a minor-AGESA jump apart. 2402's notes cite "enhanced memory performance,
system stability" — which is the class of problem we have.

## Use USB BIOS Flashback, not EZ Flash

This matters more here than it normally would. **This machine freezes every ~15 minutes**,
and the failure is below the OS — the NMI watchdog never fires, so firmware is not immune to
it. EZ Flash runs on the CPU with the board live; a freeze partway through writing the BIOS
chip is how a motherboard gets bricked.

USB BIOS Flashback runs with **the system powered off**, driven by a separate microcontroller
that does not use the CPU, the RAM, or the installed OS. None of the things that have been
failing are involved. It is the safe route on an unstable machine, and on this board it is
also the only one that needs no working boot.

## Before you start

Verify on the download page that **2402 is not labelled "Beta Version"**. ASUS ships betas in
the same list — 2401 (2026-06-26) is one. Sources disagree about 2402's status, so check the
label yourself. If it does say Beta, use **2306** (2026-06-17, AGESA 1.3.0.1b) instead; it
carries the same AGESA base without the TSME patch.

Download page: <https://www.asus.com/us/supportonly/rog%20strix%20x870-a%20gaming%20wifi/helpdesk_bios/>

## Procedure

1. **USB stick**: FAT32, ideally 32 GB or smaller. Delete everything on it — the BIOS file
   must be the only thing in the root directory.
2. **Download** the BIOS zip from the page above.
3. **Extract it.** It contains the `.CAP` file and `BIOSRenamer.exe`.
4. **Run `BIOSRenamer`.** It renames the file to `A5570.CAP`, which is the name the Flashback
   controller looks for on this board. Flashback will silently do nothing if the name is
   wrong — this is the single most common reason it "doesn't work".
5. **Copy the renamed `A5570.CAP` to the root of the stick.** Not in a folder.
6. **Shut the PC down** — fully off, but leave the PSU switched on and the power cable in.
   Flashback needs standby power.
7. **Insert the stick** into the rear-panel port marked for BIOS Flashback (check the manual
   for the exact one on this board — it is a specific single port, not any USB port).
8. **Press and hold the BIOS FlashBack button for ~3 seconds** until the LED blinks three
   times, then release.
9. **Wait.** The LED flashes throughout and **turns off when it is finished** — typically
   3–8 minutes. Do not cut power, do not press anything, do not pull the stick. A flashing
   LED that stops flashing and stays lit means it failed; a light that goes out means done.
10. Power on.

Manual (Flashback section is around p.51):
<https://dlcdnets.asus.com/pub/ASUS/mb/SocketAM5/ROG_STRIX_X870-A_GAMING_WIFI/E25346_ROG_STRIX_X870-A_GAMING_WIFI_EM_V2_WEB.pdf>

## Afterwards — settings are all back to defaults

Restore this state before drawing any conclusions:

| setting | value | why |
|---|---|---|
| **EXPO / DOCP** | **leave OFF** | Defaults are off, which is what we want. The DIMMs are rated 6000 but have been running JEDEC 4800 through this whole investigation. Do not reintroduce that variable while we are still measuring. |
| **Secure Boot** | **disabled** | It is disabled now, and the NVIDIA driver is in use. Turning it on breaks unsigned kernel modules. |
| **Boot order** | Ubuntu first | Currently `Boot0000* Ubuntu … \EFI\ubuntu\shimx64.efi`. A reset can push a removable device ahead of it. |
| **Above 4G Decoding / Resizable BAR** | enabled | Two ReBAR-capable devices present, with an NVIDIA GPU and an AMD iGPU both active. |
| **Power Supply Idle Control** | consider **Typical Current Idle** | Only if freezes continue. This is the standard AMD mitigation for hangs at low load, and every crash we have captured happened at low and falling load. Change it on its own, after the flash has had its own observation window. |

The OS side is untouched by a BIOS flash: the udev rules, the kernel cmdline
(`amd_iommu=off iommu=soft`), and `freeze-monitor.service` all survive.

Once it boots, `amd_iommu=off iommu=soft` is worth revisiting — it is somebody's older
workaround and a newer AGESA may not need it. Change it separately, not in the same window.

## How we will know it worked

See `docs/freeze-baseline.md`. The bar is deliberately not "it stayed up a while": this
machine produced a 4h08m boot in the middle of the bad run and a 26-day boot before it.

* **No crash inside the first hour**, across several boots, and
* **a multi-day uptime** reproduced at least once.

`freeze-monitor.service` starts at boot on its own, so whatever happens next is recorded
whether or not the robot stack is running. To read the verdict after the next event:

```sh
tail -30 /home/nse/humanoid-logs/freeze_monitor.log
```

The last line is the machine roughly two seconds before it stopped.
