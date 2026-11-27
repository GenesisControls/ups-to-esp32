<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Commissioning the Genesis UPS→BMS adapter

For the person standing at the board. Point list: `genesis-registers.md`.

What it does: reads one USB UPS and serves it to a BMS as **Modbus RTU** or
**BACnet MS/TP** on RS485.

---

## 1. Power it up

Power from the 7–36 V terminal, or 5 V on the USB-C.

**The USB-C is for power and for flashing, not for the UPS.** The UPS plugs
into the USB-A side via the header or a powered OTG adapter — those two share
GPIO19/20 with the USB-C, so only one can have them.

## 2. Get on its web page

For **ten minutes after power-up** the board runs its own Wi-Fi access point:

| | |
|---|---|
| Network | **`ups-esp32-XXXX`** — the last 4 hex of its MAC |
| Password | none, it is open |
| Address | **http://192.168.4.1/** |
| Login | **`admin`** / **`Admin1234`** |

The page asks you to change that password and does not force it. It is in
published source, so it is not a secret — it gates writes to this page, and
nothing else. Change it if the network it sits on is not already trusted.

**After ten idle minutes the radio switches off.** The clock restarts
whenever a device is associated, so it will not close under you mid-job.

**To get it back: press BOOT, or power-cycle.** Deliberately physical — it
has to work when the network is off. A power cycle always gives a fresh ten
minutes, so a board is never unreachable to someone standing next to it.

For bench work, Wi-Fi → *Setup access point* → **hold it on until I turn this
off**. Applies immediately, survives a power cycle. Turn it off before the
board leaves the bench: an open, unauthenticated AP broadcasting for years in
a customer's building is what the ten-minute window exists to prevent.

## 3. Check the UPS is being read

**Overview** should show the model and live values. If it says the UPS link is
down:

- is the UPS actually powered, and its USB plugged in?
- **does the UPS have 5 V on its USB?** This is the usual cause. The board's
  USB-C is wired as a *device* — VBUS comes *in* through a diode — so a plain
  OTG adapter gives the UPS no power. Use a powered OTG adapter with 5 V on
  the power leg, or the 2.0 mm header with 5 V and GND.
- **RS485 / BMS → Register overview** shows every point with its raw value.
  With no UPS, discretes read 0, measurements 65535 and register 7 `0b000000`
  — that is the comm-loss contract working, not a fault.

## 4. Choose the protocol

**RS485 / BMS**, then restart the board. Changes need a restart: an output
holding a UART cannot be swapped under a live trunk, and a BMS watching a
device vanish mid-poll logs a fault.

### Modbus RTU

| Setting | Default | Notes |
|---|---|---|
| MAC address | **4** | 1–247. The slave address your BMS polls. |
| Baud | auto | **See the warning below.** |
| Data format | 8N1 | 8E1, 8O1, 8N2 also available. |

Match the BMS exactly on address, baud and format.

> **Autobaud is not built yet.** "auto" currently falls back to **9600** and
> says so in the log. If your trunk is not 9600, **set the rate explicitly**.
> A wrong rate looks exactly like a dead device.

8N2 exists because the Modbus RTU spec asks for two stop bits when there is
no parity. Most masters are 8N1. A device framing 8N1 against a strict 8N2
master produces frame errors indistinguishable from a wrong baud rate — if
everything looks right and nothing works, try 8N2.

### BACnet MS/TP

| Setting | Default | Notes |
|---|---|---|
| MAC address | **4** | **0–127** — MS/TP cannot address above 127. |
| Device ID | 0 = derive | Unique across the whole BACnet internetwork. |
| Baud | auto | Same warning as above. |
| Max info frames | 1 | Frames per token. 1 suits a device that answers. |
| Max master | 127 | Lower it to just above the highest MAC on the segment and token rotation speeds up measurably. |

Data format is not offered: ANSI/ASHRAE 135 fixes MS/TP at 8N1.

The point set is BI1&ndash;BI11 and AI1&ndash;AI19, read-only, plus **BV1
&ldquo;Start battery self-test&rdquo;**, which is the only writable object on
the device. See `genesis-registers.md`.

**Device ID 0 derives it from the chip MAC**, which is the default and the
right answer for twenty adapters. A fixed default would mean twenty devices
claiming the same instance — most likely 12345, which is in every vendor's
example code and therefore the one most likely already taken. Set a number
only if the site has its own numbering plan. The page shows what is in use.

### Note on the address

MS/TP allows 0–127, Modbus 1–247. Switching to MS/TP with an address above
127 is **refused with a reason**, not silently clamped — a clamp would leave
the BMS polling an address nothing answers on.

## 5. Wire the trunk

| Signal | Pin |
|---|---|
| A / B | the isolated RS485 terminal |
| TX / RX / DE | GPIO 17 / 18 / 21 — on-board, nothing to wire |

The transceiver is isolated and the driver-enable is the UART's own hardware
RTS, so there is no software keying to get wrong.

**The 120 Ω jumper** terminates the segment. Fit it only if this board is at a
physical *end* of the trunk. Two terminators per segment, no more — a
mid-trunk board with the jumper fitted degrades everyone.

## 6. Confirm it is on the bus

**RS485 / BMS** shows, per running output:

- **Requests** — total and in the last minute. *This is the one that matters.*
  Rising means the BMS is polling you.
- **Last request** — seconds ago
- **Per function code** — FC4/FC3/FC2/FC1 separately. Which half of a point
  sheet is being read is the fastest way to spot a half-configured master:
  input registers but never discretes means FC2 was never mapped.
- **Bus collisions** — almost always two devices sharing an address
- **Writes refused** — a master trying to write a read-only point

Counters are since the last restart and are not stored.

### If the trunk is silent

1. **Requests at 0 and other devices on the trunk working** → baud or data
   format. Set the rate explicitly rather than trusting auto.
2. **A and B swapped.** Very common. Swap them.
3. **Address.** Confirm what the BMS is polling.
4. **Termination.** Both ends, nothing in between.
5. **Collisions counting up** → duplicate address on the segment.

---

## Updating the firmware

No internet needed, and this is the normal route:

1. Join the board's access point
2. **Board → Install firmware from a file**
3. Choose the `.bin`, press **Upload and install**
4. **Restart board**
5. **Press Approve** on the *New firmware on trial* card

The board checks the image's own identity and refuses one built for a
different board **before writing anything**.

**Step 5 matters.** A freshly installed image boots *on trial*: if it is not
confirmed it reverts to the previous version. Approving it cancels that. If
you skip it, it confirms itself after **five minutes of uptime** anyway — but
a restart or power cut inside those five minutes drops you back to the old
firmware, which is a confusing half-hour if you don't know to expect it.

The automatic update check is **off** and points at the Genesis repository.
Installing has always needed a person.

---

## Resetting

**Factory reset is a button gesture, not a page button.** There is
deliberately no web route for it — clearing the network is exactly what a
board needs when nobody can reach its page, so it has to work without one.

> **Press BOOT five times within four seconds.**

It erases the `upsa` settings namespace and the stored HTTPS keys — Wi-Fi,
login, protocol, address, baud, everything — then restarts into setup with
`admin` / `Admin1234`. The HTTPS keys go too, so a previous owner's browsers
stop trusting the board.

Mind the overlap: **one** BOOT press reopens the setup access point and buys
another ten minutes. Five *within four seconds* wipes it. If you are pressing
BOOT to get back onto a board whose radio has closed, press it **once** and
wait — the log counts the presses from the second one onwards, so a console
tells you how close you are.

**Restart** — the button on the RS485 / BMS card, next to the settings that
need one.

### If a setting will not save

> **“The value was accepted but could not be stored (settings memory).”**

That is the board's settings memory refusing the write, not a bad value — the
page has already checked the range. It means the memory is full or damaged, and
a factory reset is the fix. A setting that appears to save and is back to its
old value after a power cycle is the same fault caught later.

A message naming a *range* instead (“must be 1–247”, “must be 1–65535”) is
about the value you typed, and the range it gives is the answer.

---

## What this firmware does not do

Worth knowing before you look for it:

- **Autobaud** — not built. "auto" means 9600.
- **BACnet/IP** — out of scope for this board.
- **BACnet COV** — not implemented. Poll the objects.
- **Days since battery change** (register 8, AI9) — reads *not available*.
  It needs a real clock and this board has none yet. Discrete 9, the battery
  age alarm, depends on it and is therefore always false.
- **Writes of any kind**, apart from three triggers on the RS485 / BMS card:
  the quick self-test (coil 100 / BV1, **on** by default, one per half hour),
  the **deep test** (coil 101 / BV2, **off** by default and with its own
  tick-box, one per day), and stop (coil 102 / BV3, no limit). All three are
  refused unless the UPS says it can self-test.

  Think before ticking the deep one: it runs the battery down to its
  low-battery level, which on a lightly loaded UPS is well over an hour with
  the site unprotected if the mains fails meanwhile.
- **Check-in / telemetry** — compiled out. This board reports to nobody.
