# Contribute a GPON bundle

You run an ONU on an OLT we have no access to. One read-only script on the ONU collects what the ONU knows about
itself and about that OLT; we turn it into an offline test case, so our open firmware adapts to your OLT without
anyone touching it again.

## What you run, and what it never does

`gpon_bundle.sh` is a POSIX shell script that needs only the board's own shell and busybox. It works on the
vendor (stock) firmware and on our OpenWrt image.

- It writes nothing but its own output directory (default `/tmp`, which is RAM), then one `.tar` in it.
- It never writes flash, U-Boot variables, the vendor configuration (`mib set`, `flash set`, `nvram`, `uci`),
  never loads a module, never changes the network, and never reboots.
- On stock firmware with the vendor OMCI log switched off, it switches the log on with
  `omcicli set logfile 7 0`. That changes the running program only: it is gone at the next reboot, and the bundle
  records that it happened. The log only holds what happens after it is switched on.
- Reading `/proc/gpon` on our image zeroes the hardware counters it prints (they are clear-on-read). No
  configuration changes.
- Our test suite checks every line of the script against these rules before each release.

## What it collects

- Identity: kernel version, command line, CPU, memory, flash partition table, network counters, the interface
  MAC addresses, the release file.
- The device tree blob (`/sys/firmware/fdt`) and the kernel log.
- On our image: `/proc/gpon`, `/proc/gpon_log`, `/proc/gpon_timing`, `/proc/gpon_health`, the OMCI exchange
  (`/proc/oltcap`), the PLOAM exchange (`/proc/ploamcap`, Realtek Luna only) and the register dumps.
- On stock firmware: the vendor OMCI log, the vendor PON state (`diag ... get onu-state`), the OMCI device mode,
  and the md5 of each OMCI program and library.
- The script itself, so we know which version ran.

## Steps

1. Copy `gpon_bundle.sh` to the board's `/tmp` (scp, or paste it into `cat > /tmp/gpon_bundle.sh` in a telnet or
   serial session).
2. Run `sh /tmp/gpon_bundle.sh /tmp`. It prints the path of the `.tar`. Copy that file to your PC.
3. Stock firmware only, if you can: from your PC, `ssh <board> 'sh /tmp/gpon_bundle.sh --binaries' > binaries.tar`.
   This streams the vendor OMCI programs to your PC without storing them on the board.
4. Fill `bundle.json` (next to this guide):
   - `board`: `<SoC>/<brand>/<model>`, brand and model as printed on the case; write `?` for the SoC if
     you do not know it (the boot log in the bundle usually names it, and we fill it in);
   - `declared_by`: your GitHub handle (leave `"?"` if you contact us by email only: we assign an id);
   - `declaration`: what you can see on the case, one key per item; `null` means it does not exist, `"?"` means
     you do not know;
   - `provenance.consent`: `publish-with-credit` (results published, your handle credited),
     `publish-anonymous` (results published, no name) or `private` (results kept private);
   - `provenance.isp`, `olt_vendor`, `olt_model`, `olt_version`: what you know, else `"?"`.
5. Add it to the bundle: `tar rf gpon-bundle-<...>.tar bundle.json`.

## Contact

Open an issue on https://github.com/alphaonex86/openwrt/issues with the "GPON bundle" template and attach the
files, or send them by email to contact@herman-brule.com.

## Privacy

The bundle holds your MAC addresses, the ONU serial number and possibly service settings the OLT sent to your
ONU. We keep bundles private. Your email address, the console login and the label MAC are stored apart from
everything else and never published. Published results follow your consent: with credit, anonymous, or not at
all.

## What we do with it

`gpon_bundle.py --ingest` checks every file against the md5 list the board wrote, files the bundle in our private
archive, turns the recorded OLT exchange into a simulated OLT, replays it against our firmware for every ONU model
we support, and adds it to the checks every firmware change must pass. If a change breaks your OLT, it is refused
before it reaches you.

## On our image: what a field build needs

The exchange is only kept when the image was built with `CONFIG_GPON_OLT_DIAG` (the OMCI and PLOAM rings),
`CONFIG_GPON_PLOAM_DIAG`, `CONFIG_GPON_OMCI_DIAG` and `CONFIG_GPON_GEM_DIAG`; our images ship them on. Each ring
keeps 256 records: a provisioning longer than that is reported as overflowed, never as a complete capture.
