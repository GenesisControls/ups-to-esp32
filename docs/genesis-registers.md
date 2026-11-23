<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Point list — Genesis UPS→BMS adapter

What this adapter serves on the bus, as built. Firmware `ups-genesis-rs485`.

**Addresses are 0-based.** Niagara's Modbus driver is 0-based too, so the
numbers below go in as they are. If your tool is 1-based (some are, and they
usually say so), add one.

> **Where the numbers come from.** The address map is owned by the *other*
> track, `ups-modbus-monitoring`, whose `README.md` is the authority —
> discretes 0–10 and registers 0–23, each tied to the NUT variable behind it.
> This file describes what the **ESP32 adapter** does with that map. Where the
> two could disagree, that README wins and this file is the bug.
>
> One difference is expected and is not a disagreement: that repo marks
> discretes 6–10 and registers 8–18 as *planned*, because its Python bridge
> has not implemented them. **This adapter implements all of them.** An
> address is either served here or it answers exception 02 — see *Reserved*.

---

## Modbus

| | |
|---|---|
| Discrete inputs | **FC2**, mirrored as coils (**FC1**) |
| Input registers | **FC4**, mirrored as holding registers (**FC3**) |
| Register width | uint16 |
| Not available | **65535** |
| Read-only | every address below except coil 100 |

Either function code of a pair works, so a point sheet written against FC3
reads the same values as one written against FC4. That mirroring is required
by the contract, not incidental.

### Discrete inputs (FC2) / coils (FC1)

| Addr | Point | True when | Needs the UPS to report |
|---|---|---|---|
| 0 | Online | mains present, load protected | — |
| 1 | **On battery** | running from the battery | — |
| 2 | Low battery | the UPS says its battery is low | — |
| 3 | Replace battery | the UPS says the pack needs replacing | — |
| 4 | Overload | output overloaded | — |
| 5 | **UPS comm OK** | a good read within the last 10 s | — |
| 6 | AVR boost active | the UPS is boosting low mains | `BOOST` in its status |
| 7 | AVR trim active | the UPS is trimming high mains | `TRIM` in its status |
| 8 | Charging | the battery is charging | `CHRG` in its status |
| 9 | Battery age alarm | the pack is older than its service life | register 8, so a date |
| 10 | Output not protecting | `OFF` or `BYPASS` — **the load is not on the UPS** | that flag |

**Alarm on 1 (On battery) *and* on 5 (Comm OK) being false.** On comm loss
every discrete reads 0, so "On battery = false" alone is indistinguishable
from "nobody is answering". This is the single most important line in this
document.

Discrete 10 is worth a point of its own even though it is rarely true: a UPS
in bypass reports itself `Online`, and without 10 a site cannot tell a
protected load from an unprotected one.

### Input registers (FC4) / holding registers (FC3)

| Addr | Point | Unit | Reads 65535 when |
|---|---|---|---|
| 0 | Battery charge | % | not reported, or comm lost |
| 1 | Runtime remaining | s | not reported, or comm lost |
| 2 | Load | % | not reported, or comm lost |
| 3 | Input voltage | V × 10 | not reported, or comm lost |
| 4 | Output voltage | V × 10 | not reported, or comm lost |
| 5 | Battery voltage | V × 10 | not reported, or comm lost |
| 6 | Heartbeat | +1 per poll | comm lost — *configurable, see below* |
| 7 | Status bitfield | bits 0–5 = discretes 0–5 | **never — reads 0 on comm loss** |
| | *— 8–15 describe the UPS —* | | |
| 8 | Days since battery change | d | **in practice always, today** — see below |
| 9 | Self-test result | enum below | the UPS cannot self-test, or comm lost |
| 10 | Input transfer low | V × 10 | the UPS does not report the window |
| 11 | Input transfer high | V × 10 | the UPS does not report the window |
| | *— 16–23 describe the adapter —* | | |
| 16 | Firmware version | major×100 + minor | never |
| 17 | Restart counter | count | never |
| 18 | Last restart reason | `esp_reset_reason_t` | never |

Scale `V × 10` in Niagara: divide by 10. 2304 is 230.4 V.

**Register 8 will read 65535 on this board today.** Days-since-change needs a
real clock, and this adapter has none — no network means no SNTP, and nothing
yet drives the RTC the board carries. It answers "not reported" rather than
counting from an epoch that starts at boot, which would have produced a
plausible and wrong number. Discrete 9 depends on it and is therefore also
false. Don't wire either expecting data until that changes.

**Registers 10 and 11 are static for a unit** and are what make discretes 6
and 7 readable: boost and trim say the mains left the transfer window, 10 and
11 say where the window is. Most consumer UPSs do not report them.

#### Register 9 — self-test result

| Value | Meaning |
|---|---|
| 0 | no test since power-up / unknown |
| 1 | passed |
| 2 | done, warning |
| 3 | failed |
| 4 | in progress |
| 5 | aborted |
| 65535 | this UPS cannot self-test, or comm lost |

Matches the other track's encoding exactly. 6 (*scheduled*) exists in that
encoding and is not produced here.

### Reserved addresses — 12–15 and 19–23

These are **not** 65535. They are not registered in the datastore at all, so
reading one returns **Modbus exception 02 (illegal data address)**.

That distinction is deliberate and useful: 65535 means *this adapter serves
that point and cannot read it right now*, exception 02 means *that point does
not exist on this device*. A commissioning engineer who sees 02 has a typo or
a point sheet from the wrong version; one who sees 65535 has a UPS question.

Registers 12–15 and 19–23 are reserved for future UPS and adapter facts
respectively. Don't claim one without going through the other track.

### Comm-loss behaviour, exactly

When the UPS link has been quiet for **10 seconds**:

- every discrete reads **0** — including Online, which is why 5 matters
- every measurement reads **65535**
- **register 7 reads 0, not 65535.** It mirrors the discretes, and a site
  reading only the bitfield must not see a phantom "on battery"
- **register 6 (heartbeat)** reads 65535 by default, which makes it stop dead
  with everything else. It can be set to keep counting instead, which lets
  Niagara tell *adapter alive, UPS link dead* from *adapter dead* — two
  faults that want different engineers

Real measurements are clamped to **65534**, so 65535 is never a real reading.

### Writes

Every address above is read-only and the stack refuses a write with an
exception — not silently ignored, not accepted-then-reverted.

**Coil 100 is the one exception**: writing it starts one quick battery
self-test. **It is enabled by default**, and refused unless the UPS's own
descriptor says it can self-test, at most one per 30 minutes. Everything else
stays read-only.

The rate limit is **per board, not per protocol**: Modbus coil 100 and BACnet
BV1 reach the same battery, and a board serving MS/TP on the trunk and Modbus
TCP over Wi-Fi still gets one test per half hour in total. Untick *allow a
battery self-test to be started from the bus* on the board's own page to refuse
both.

---

## BACnet MS/TP

Device object name `UPS-<last 4 of MAC>`. Device instance derived from the
chip MAC unless one is configured.

**Object instance = the Modbus address + 1.** One map for both protocols;
the only arithmetic between them is that BACnet instances conventionally
start at 1 and Modbus addresses at 0.

That rule covers the point set. The one command on the device is the
exception: it is **BV1**, not BV101, because it is a command rather than a
point — see the end of this section.

| Object | Point | | Object | Point |
|---|---|---|---|---|
| BI1 | Online | | AI1 | Battery charge (%) |
| BI2 | **On battery** | | AI2 | Runtime remaining (s) |
| BI3 | Low battery | | AI3 | Load (%) |
| BI4 | Replace battery | | AI4 | Input voltage (V) |
| BI5 | Overload | | AI5 | Output voltage (V) |
| BI6 | **UPS comm OK** | | AI6 | Battery voltage (V) |
| BI7 | AVR boost active | | AI7 | Heartbeat |
| BI8 | AVR trim active | | AI9 | Days since battery change (d) |
| BI9 | Charging | | AI10 | Self-test result |
| BI10 | Battery age alarm | | AI11 | Input transfer low (V) |
| BI11 | Output not protecting | | AI12 | Input transfer high (V) |
| | | | AI17 | Firmware version |
| | | | AI18 | Restart counter |
| | | | AI19 | Last restart reason |

**There is no AI8.** Modbus register 7 is the status bitfield, and a float
whose value is `0b101101` is a Modbus workaround, not data — every bit of it
is already BI1–BI6.

**Volts are volts.** AI4 reads 230.4, not 2304: an Analog Input is a float
with a units property, so the ×10 integer trick Modbus needs is not used
here. Units are set on each object (percent, seconds, volts, days).

**Unavailable points use Reliability, not 65535.** BACnet can say "I don't
know", so a point whose value is unavailable reports
`COMMUNICATION_FAILURE` and leaves its value alone, instead of publishing a
number a trend log would happily plot. **BI6 (Comm OK) is deliberately
exempt** — the object that reports whether the link is up has to stay
readable when it is down.

### BV1 — the one writable object

| Object | Point | Writable |
|---|---|---|
| BV1 | Start battery self-test | **yes**, when enabled on the board |

Writing BV1 **Active** starts one quick battery self-test — the BACnet
counterpart of Modbus coil 100, with the same guards and the *same* half-hour
limit shared between them. Its present-value returns to Inactive immediately:
it is a momentary trigger, not a state, so it never reads as "a test is
running". Register 9 / AI10 is what reports the result.

A BMS sees `Start test` and `Idle` rather than `Active` and `Inactive`, and the
object's description says what a write does.

It is a Binary **Value**, not a Binary Output: a BO models a physical output
this board does not have. It is also not commandable — no priority array, no
relinquish default — so a plain WriteProperty to present-value is all it takes.

**With self-tests disabled on the board, BV1 is still there and still
discoverable, but a write to it earns `write-access-denied`** — a refusal the
BMS can see and log. Modbus cannot manage that: coil 100 is always writable,
and the refusal is a line in the board's own console that nobody on the trunk
ever reads.

Present-value is the **only** writable property on BV1. Object name,
description and out-of-service are refused as well, which is stricter than the
BACnet stack's own default for a Binary Value.

Every other object is read-only, and a WriteProperty to one earns
`write-access-denied` rather than `service-not-supported`, so a BMS sees a
read-only device rather than a broken one.

COV is **not** implemented. Poll the objects.

---

## Not served by this firmware

- **BACnet/IP** — out of scope for this board (decided 2026-11-22). Not an
  option, not a setting.
- **Modbus TCP** *is* available and serves this same map, with its own unit
  id and port, once the board has joined a network. Most will not.
