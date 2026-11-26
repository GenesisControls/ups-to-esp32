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
| 12 | Beeper | enum below | this UPS has no controllable alarm, or comm lost |
| 13 | **UPS state** | enum below | comm lost |
| | *— 16–23 describe the adapter —* | | |
| 16 | Firmware version | major×100 + minor | never |
| 17 | Restart counter | count | never |
| 18 | Last restart reason | enum below | never |

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
| 1 | passed |
| 2 | done, warning |
| 3 | failed |
| 4 | in progress |
| 5 | aborted |
| 6 | scheduled — the other track's value; this adapter never produces it |
| **7** | **unknown / no test since power-up** |
| 0 | not used |
| 65535 | this UPS cannot self-test, or comm lost |

**Unknown moved from 0 to 7 on 2026-11-23.** Every enumerated point here
starts at 1, so the number read over BACnet is the number read over Modbus. A
BACnet Multi-State Input's present value is 1..N by ANSI/ASHRAE 135, so an
encoding with a meaningful 0 would have forced an off-by-one on this one
register — exactly what gets an engineer to map a 3 onto a 3 and be wrong.

1–6 are untouched. They belong to the other track and 6 stays *scheduled*
rather than being reused.

> **This changes the shared map, and that track has been told.** Register 9
> is published in `ups-modbus-monitoring/README.md`, where it is still marked
> *planned* and its encoding *provisional*. The move was applied there on
> 2026-11-24 — see *Reserved addresses* below for the state of that
> relay. Don't send it a second time.

#### Register 18 — last restart reason

1-based for the same reason, and this encoding is **ours**: the other track's
README calls register 18 "track-specific in its encoding, not its meaning"
and publishes 65535 there.

| Value | Meaning | | Value | Meaning |
|---|---|---|---|---|
| 1 | Unknown | | 9 | Deep sleep |
| 2 | Power-on | | 10 | Brownout |
| 3 | External pin | | 11 | SDIO |
| 4 | Software restart | | 12 | USB peripheral |
| 5 | Panic | | 13 | JTAG |
| 6 | Interrupt watchdog | | 14 | eFuse error |
| 7 | **Task watchdog** | | 15 | Power glitch |
| 8 | Other watchdog | | 16 | CPU lockup |

It is `esp_reset_reason_t` plus one. That enum is append-only upstream, so a
value ESP-IDF adds later reads as **1, Unknown** rather than as an unnamed
state.

Worth knowing which ones matter: **5 (Panic)** and **7 (Task watchdog)** mean
the firmware crashed, **10 (Brownout)** means the supply sagged — the one to
watch on a board powered from the UPS's own USB port — and **2 (Power-on)**
after an unattended period means the board lost power.

#### Register 12 — beeper

| Value | Meaning |
|---|---|
| 1 | Disabled — it will not sound |
| 2 | Enabled — it is allowed to sound |
| 3 | Muted — the alarm sounding now is silenced; future ones are not |
| 65535 | this UPS has no controllable alarm, or comm lost |

The AudibleAlarmControl values unchanged, so again nothing is renumbered.

**This does not say whether it is beeping right now.** *Enabled* is permission,
not noise. Nothing in the UPS's status word reports an alarm sounding — it
carries `OL OB LB CHRG DISCHRG RB OVER` and no alarm bit — so there is nothing
to publish. What makes it sound is already on the bus: discretes 1–4.

> **Claimed from the reserved range on 2026-11-23**, and relayed to
> `ups-modbus-monitoring` on 2026-11-24 — see *Reserved addresses*
> below. 14–15 remain reserved.

#### Register 13 — UPS state

**The one point to watch, if you only wire one.**

| Value | Meaning | True when |
|---|---|---|
| 1 | Normal | online, battery idle |
| 2 | **Self-test running** | online **and** discharging |
| 3 | Calibration running | the UPS reports `CAL` — the deep test |
| 4 | **On battery** | the mains actually failed |
| 5 | Not protecting | `OFF` or bypass: the load is not on the UPS |
| 6 | Unknown | readable, but none of the above fits |
| 65535 | comm lost *(Reliability on BACnet)* |  |

Everything in it is already on the bus as separate discretes. That is the
point: **the combination that matters is not obvious.** A self-test is
`Online` *and* discharging — the UPS stays Online because the mains never
failed, and it is right to, or every site would raise a false mains-failure
alarm each time a test ran. The cost is that from the bus a running test has
looked identical to an idle UPS. State 2 is what tells them apart.

**State 3 depends on the unit.** `CAL` means *performing a runtime
calibration* and nothing else, so it is the only place in the whole stack
where a deep test is distinguishable from a quick one — the self-test result
register carries one *in progress* with no type. A UPS that never sets `CAL`
simply never reaches state 3, and its deep test reads as state 2.

**Worst wins.** More than one condition is true more often than not, and the
order is: on battery → not protecting → calibration → self-test → normal. On
battery beats any test deliberately: if the mains fails while a calibration
is running, the site is in a real outage and this point says so rather than
go on reporting a test.

**Nothing unexplained lands on Normal.** Anything readable that does not fit
the rules reads 6, not 1 — a summary point that reports calm when it cannot
tell is the one failure it must not have.

> **Claimed from the reserved range on 2026-11-23**, alongside register 12,
> and relayed to `ups-modbus-monitoring` on 2026-11-24 — see *Reserved
> addresses* below. 14–15 remain reserved.

### Reserved addresses — 14–15 and 19–23

These are **not** 65535. They are not registered in the datastore at all, so
reading one returns **Modbus exception 02 (illegal data address)**.

That distinction is deliberate and useful: 65535 means *this adapter serves
that point and cannot read it right now*, exception 02 means *that point does
not exist on this device*. A commissioning engineer who sees 02 has a typo or
a point sheet from the wrong version; one who sees 65535 has a UPS question.

Registers 14–15 and 19–23 are reserved for future UPS and adapter facts
respectively. Don't claim one without going through the other track.

**12 and 13 were claimed on 2026-11-23 and relayed on 2026-11-24.** Their
encodings, and register 9's move of *unknown* from 0 to 7, were applied to
`ups-modbus-monitoring/README.md` — the authority for the shared map — and
left **uncommitted** there by decision: that repo has no git history yet, so
the change is an untracked edit plus a saved diff rather than a commit. The
numbers are settled either way; don't relay them again.
`docs/register-map-change-2026-11-23.md` is what went over.

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

**Three coils are the exception**, and nothing else on the device is writable:

| Coil | Starts | Permission | Rate limit |
|---|---|---|---|
| 100 | quick self-test — seconds on battery | **on by default** | one per 30 min |
| 101 | **deep test** — runtime calibration | **off by default**, its own tick-box | one per day, and the 30 min above |
| 102 | stop a running test | either box | **none** |

All three are refused unless the UPS's own descriptor says it can self-test.

**Coil 101 is not coil 100 with a longer runtime.** A deep test runs the
battery down to its low-battery level — on a lightly loaded UPS that is well
over an hour — and the site is unprotected if the mains fails while it runs.
It has a separate permission for that reason, and is off until somebody ticks
it.

**Coil 102 is never rate limited.** Stopping ends a condition rather than
creating one, and a site that needs its battery back out of a calibration must
not be told to wait twenty minutes.

The rate limits are **per board, not per protocol**. Coil 100 and BACnet BV1
reach the same battery, so a board serving MS/TP on the trunk and Modbus TCP
over Wi-Fi still gets one quick test per half hour in total.

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
| BI9 | Charging | | **MSI10** | Self-test result |
| BI10 | Battery age alarm | | AI11 | Input transfer low (V) |
| BI11 | Output not protecting | | AI12 | Input transfer high (V) |
| | | | **MSI13** | Beeper |
| | | | **MSI14** | **UPS state** |
| | | | AI17 | Firmware version |
| | | | AI18 | Restart counter |
| | | | **MSI19** | Last restart reason |

**There is no AI8.** Modbus register 7 is the status bitfield, and a float
whose value is `0b101101` is a Modbus workaround, not data — every bit of it
is already BI1–BI6.

**MSI10, MSI13, MSI14 and MSI19 are Multi-State Inputs, not Analog Inputs**, because they
are enumerations rather than measurements. Each state carries its text, so a
BMS shows `Task watchdog` where an AI could only show `7.0`. Multi-State
Input is a different object type from Analog Input, so the instance rule is
unchanged — still the Modbus address + 1 — and MSI10 collides with nothing.

**The state numbers are the register's own**, with no offset. Both encodings
were made 1-based so that a 4 on Modbus is state 4 here. The state text is
listed under registers 9 and 18 above.

**The binaries carry text too.** Every BI reports what it means rather than
Active/Inactive — BI2 reads **On battery / On mains**, BI11 reads
**Not protecting / Protecting**, BI6 reads **OK / Lost**.

**Volts are volts.** AI4 reads 230.4, not 2304: an Analog Input is a float
with a units property, so the ×10 integer trick Modbus needs is not used
here. Units are set on each object (percent, seconds, volts, days).

**Unavailable points use Reliability, not 65535.** BACnet can say "I don't
know", so a point whose value is unavailable reports
`COMMUNICATION_FAILURE` and leaves its value alone, instead of publishing a
number a trend log would happily plot. **BI6 (Comm OK) is deliberately
exempt** — the object that reports whether the link is up has to stay
readable when it is down.

### BV1–BV3 — the only writable objects

| Object | Writes | Modbus twin | Permission |
|---|---|---|---|
| BV1 | Start battery self-test | coil 100 | on by default |
| BV2 | Start deep battery test | coil 101 | **off by default** |
| BV3 | Stop battery test | coil 102 | either of the above |

Writing one **Active** performs the action, with the same guards and the same
rate limits as its Modbus twin — they are the same board and the same battery.
Present-value returns to Inactive immediately: these are momentary triggers,
not states, so none of them ever reads as "a test is running". **MSI10** is
what reports the result.

A BMS sees `Start test`, `Start deep test` or `Stop` rather than `Active`, and
each object's description says what a write does and what it costs.

It is a Binary **Value**, not a Binary Output: a BO models a physical output
this board does not have. It is also not commandable — no priority array, no
relinquish default — so a plain WriteProperty to present-value is all it takes.

**With self-tests disabled on the board, BV1 is still there and still
discoverable, but a write to it earns `write-access-denied`** — a refusal the
BMS can see and log. Modbus cannot manage that: coil 100 is always writable,
and the refusal is a line in the board's own console that nobody on the trunk
ever reads.

Present-value is the **only** writable property on these. Object name,
description and out-of-service are refused as well, which is stricter than the
BACnet stack's own default for a Binary Value.

**Neither protocol can tell you which test is running.** The UPS reports a
single *in progress* with no test type — the Test usage is written with one
table and read back with another that has no type field. What you can see is
`OL` and `DISCHRG` together, which is the UPS discharging while the mains is
fine, and that only ever means a test.

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
