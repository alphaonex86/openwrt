---
name: GPON bundle from an ONU we do not hold
about: Share what your ONU tells about itself and its OLT, so our firmware can adapt to that OLT offline
title: "GPON bundle: <brand> <model> on <OLT vendor> <OLT model>"
labels: gpon-bundle
---

Thank you. Everything below is read-only on your board; .github/gpon-bundle/README.md explains each step.

1. On the board: `sh gpon_bundle.sh /tmp` (from .github/gpon-bundle/), then copy the printed `.tar` here.
2. On a stock board, optionally: `sh gpon_bundle.sh --binaries > binaries.tar` from a PC session, and attach it.
3. Fill .github/gpon-bundle/bundle.json and attach it (or append it: `tar rf <bundle>.tar bundle.json`):
   - `board`: brand and model exactly as printed on the case
   - `declared_by`: leave `"?"`, we assign an id (put your GitHub handle only if you want it on record)
   - `provenance.consent`: `private` unless you want the results published (`publish-with-credit` or `publish-anonymous`)
   - `provenance.isp`, `provenance.olt_vendor`, `provenance.olt_model`, `provenance.olt_version`: what you know
   - `declaration`: what you can see on the case; `null` when it does not exist, `"?"` when you do not know

Do not paste passwords, serial numbers or MAC addresses into this issue: they stay inside the bundle, which we
keep private. Prefer email? Send the same files to contact@herman-brule.com instead of opening an issue on
https://github.com/alphaonex86/openwrt/issues.

The skeleton, generated from our declaration tool:

```json
{
 "board": "<SOC>/<BRAND>/<MODEL>",
 "declared_by": "?",
 "stock_dump": [],
 "declaration": {
  "LAN_PANEL_PORTS": "?",
  "WIFI_BANDS": "?",
  "WIFI_MAX_STD": "?",
  "LED_PANEL_LABELS": "?",
  "FIBER_SOCKET_LABEL": "?",
  "LABEL_MAC": "?",
  "STOCK_SSID_2G": "?",
  "STOCK_SSID_5G": "?",
  "STOCK_CON_LOGIN": "?",
  "PART_SOC": "?",
  "PART_PON_OPTIC": "?",
  "PART_WIFI": "?",
  "PART_ETH": "?",
  "PART_FLASH": "?",
  "PART_LEDS": "?",
  "PART_VOICE": "?"
 },
 "provenance": {
  "consent": "private",
  "isp": "?",
  "olt_vendor": "?",
  "olt_model": "?",
  "olt_version": "?"
 }
}
```
