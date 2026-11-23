<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Vendored bacnet-stack (MS/TP subset)

Third-party code. Nothing in `src/` is ours; `CMakeLists.txt` and this file are.

## Provenance

| | |
|---|---|
| Project | [bacnet-stack](https://github.com/bacnet-stack/bacnet-stack) (Steve Karg) |
| Tag | `bacnet-stack-1.7.0` |
| Archive | `codeload.github.com/bacnet-stack/bacnet-stack/tar.gz/refs/tags/bacnet-stack-1.7.0` |
| Size | 10.39 MB |
| SHA-256 | `9ebd5db0f1850be575849011e50c8a9750d2827a1e40cba89836c006d12e2629` |
| Fetched | 2026-11-22 |

## Licensing — compatible, and checked rather than assumed

The vendored **sources** carry these SPDX identifiers:

| Licence | Files |
|---|---|
| `GPL-2.0-or-later WITH GCC-exception-2.0` | 39 |
| `MIT` | 18 |
| `CC-PDDC` | 2 |
| `BSD-3-Clause` | 1 |

All four combine with this firmware's GPL-3.0-or-later. **The decisive word is
"or-later"** on the GPL files: `GPL-2.0-only` would have been incompatible with
GPLv3 and would have ended the BACnet track outright.

Two cautions for whoever revisits this:

- The upstream README describes the project as "GPL with exception", which is
  true of the project but not of most of the tree — and the proportions differ
  by subset. Across all of `src/` it is MIT-dominant (400 MIT / 93 GPL); across
  *what we compile* it is GPL-dominant, as above. Judge by the subset.
- GitHub's licence detector reports `null` for this repository and there is no
  root `LICENSE` file, so neither a badge nor a repo-level field can answer
  this. The per-file SPDX lines are the only authority. `license/` in the
  tarball holds the full texts.

This resolves "confirm `bacnet-stack` licence terms" from the project brief's
*still to confirm before release* list.

## What was taken, and why exactly this

`src/bacnet/**`:

- **every header** (270 files, 1.07 MB) — the stack cross-includes widely and
  cherry-picking headers would break on the next update for no real saving
- **60 source files** (1.75 MB) — the list is not a guess. It is
  bacnet-stack's own `ports/esp32/extra_script.py`, `m5stamplc-mstp`
  environment: its common group plus its MS/TP transport group. That is the
  upstream author's own answer to "what does MS/TP on an ESP32 need."

Two deliberate changes to that list:

- **`basic/object/bo.c` (Binary Output) is left out.** A BO models a physical
  output this board does not have, and in this stack it is commandable: a
  priority array, a relinquish default and a second route to the same value.
- **`basic/object/bv.c` (Binary Value) is added**, on 2026-11-23, for BV1 —
  the one writable point on the device, which starts a battery self-test.
  Adding BV1 cost **3584 bytes** of flash, measured end to end (0x15fd20 ->
  0x160720): this file, the object, its strings and `out_bacnet`'s write
  wrapper together. The file alone is 2700 lines, so most of it never reaches
  the image — the linker drops what the defines leave out, which here means
  intrinsic reporting and the commandable present-value paths, since
  `BACNET_OBJECT_BINARY_VALUE_COMMANDABLE` is not defined.

`bv.c` is also why `bactext.c` is still absent: its only calls into
`bactext_*()` sit inside `#if defined(INTRINSIC_REPORTING) &&
(BINARY_VALUE_INTRINSIC_REPORTING)`, neither of which is defined here. Define
either one and the link breaks until `bactext.c` is vendored too.

Nothing from `ports/esp32/` was copied. See below.

## Two traps

**`Device_Init` has three implementations upstream** — in
`basic/server/bacnet_device.c`, `basic/object/device.c` and
`basic/object/client/device-client.c`. Only the first is vendored here, because
only the first is in the port's list. `device.h` *is* present (all headers are),
so adding `device.c` back later looks harmless and would give duplicate symbols
at link time. The vendored `.c` count is 60 and `CMakeLists.txt` lists 60; if
those ever disagree, that is the bug.

**Configuration is by `-D`, not by editing `config.h`.** The defines in
`CMakeLists.txt` mirror the upstream port's `platformio.ini`. They are `PUBLIC`
because `out_bacnet` includes these headers too: a `BACDL_MSTP` or `MAX_APDU`
mismatch between the stack and its caller changes struct layouts, and that
fails at runtime rather than at build time.

## Why we did not vendor the ESP32 port layer

`ports/esp32/` contains `rs485.c`, `dlenv.c`, `bacnet_app.c` and
`mstimer_init.c`, which do roughly what `out_bacnet` needs. We read them as
reference and wrote our own instead, for two reasons:

1. **None of those four files carries an SPDX line.** Every file we vendored
   does. An unmarked file in a repository whose README says licences are marked
   per file is a question, not an answer, and it is not a question worth
   shipping to twenty customer sites under a copyleft licence.
2. They hardcode M5StamPLC pins and a fixed baud rate. Ours take the pin map
   from `board.h` and the baud and MS/TP address from the web UI, which is a
   rewrite of most of the file anyway.

## Updating

Re-run the vendoring against a new tag, keep the source list in step with that
release's `ports/esp32/extra_script.py`, and re-check the SPDX tally — a new
file under a new licence is exactly the thing that would slip through silently.
Do not edit anything under `src/`: local changes here become merge conflicts on
every update, which is why the warning suppressions live in `CMakeLists.txt`
rather than in patched sources.
