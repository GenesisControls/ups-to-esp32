# Third-party notices

Copyright (c) 2026 Aron Reid.

This project's own code is GPL-3.0-or-later (see `LICENSE`). The following
third-party code is included, each under its own licence, which travels with it.

| Component | Where | Licence | Copyright |
|---|---|---|---|
| MicroLink (a Tailscale client for ESP-IDF) | `firmware/components/microlink/` | MIT, see `LICENSE` there | Cameron Malone, 2025-2026 |
| x25519 (inside MicroLink) | `firmware/components/microlink/x25519-license.txt` | MIT | Cryptography Research, Inc., 2015-2016 |
| wireguard-lwip | `firmware/components/wireguard_lwip/` | BSD 3-clause, see `LICENSE` there | Daniel Hope, 2021 |
| bacnet-stack (BACnet MS/TP subset) | `firmware/components/bacnet/src/`, see `README-GENESIS.md` there | per file: GPL-2.0-or-later WITH GCC-exception-2.0 (39), MIT (18), CC-PDDC (2), BSD-3-Clause (1) | Steve Karg and contributors |
| esp-modbus | pulled in by ESP-IDF's component manager (`espressif/esp-modbus`) | Apache-2.0 | Espressif Systems |
| mDNS | pulled in by ESP-IDF's component manager (`espressif/mdns`) | Apache-2.0 | Espressif Systems |
| ESP-IDF | build dependency, not included | Apache-2.0 | Espressif Systems |
| Reel-rotation table | `tools/test/jlc_cpl_rotations.csv` | GPL-3.0-or-later | JLCKicadTools contributors |

MicroLink and wireguard-lwip are compiled into the Tailscale builds only. Rev A
images do not contain them. Changes made to vendored files are listed in each
directory's `VENDORED.md`.

bacnet-stack is vendored as a subset: only the sources its own ESP32 port
compiles for MS/TP, so most of that project is absent here. Nothing under
`firmware/components/bacnet/src/` has been modified — the build configures it
entirely with compiler defines so that updating to a new release stays a copy
rather than a merge. The per-file SPDX identifiers are the authority on its
licensing; its upstream repository carries no root licence file.

KiCad's standard symbol, footprint and 3D libraries used by the published
hardware design keep their own licences.

Tailscale is a trademark of Tailscale Inc. ESP32 and ESP-IDF are trademarks of
Espressif Systems. This project is not affiliated with or endorsed by either.
