<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Register map: three changes to relay — 2026-11-23

**For the `ups-modbus-monitoring` track.** The ESP32 adapter has changed one
published encoding and claimed two reserved addresses. The map is shared, and
`ups-modbus-monitoring/README.md` is its authority, so these belong there
rather than only here.

Nothing a site reads today moves. **Discretes 0–10 and registers 0–8, 10, 11
and 16–18 are untouched**, as is register 7's six-bit width and every
comm-loss convention.

> **Applied 2026-11-24.** All three are in that README, along with a fourth
> correction made while landing them: its note on register 18 said the adapter
> publishes `esp_reset_reason_t`, where the firmware publishes that **plus
> one** (1–16, 1 = unknown), so 18 follows the 1-based rule rather than
> being an exception to it. Committed there on 2026-11-26 as `3aa6f25`, on a
> branch cut from `5cc96fb` — the first commit that repo has ever had.
> **Not pushed, and it has no remote.**

---

## Why a BACnet constraint is changing a Modbus number

Change 1 will look like it has nothing to do with the bridge, so it is worth
saying plainly: the two tracks serve **one point sheet** to the same Niagara
stations. A number that differs between them is a commissioning error waiting
to happen, and the adapter now serves BACnet as well as Modbus.

A BACnet **Multi-State Input** has a present-value of `1..N` by
ANSI/ASHRAE 135. There is no state zero. So any enumeration with a meaningful
`0` forces the two protocols to differ by one on that register, and an
engineer holding one point sheet maps a 3 onto a 3 and is wrong.

Hence the rule the adapter now follows throughout: **every enumerated register
starts at 1.** The bridge does not have to implement BACnet for the numbers to
need to agree.

---

## 1. Register 9 — self-test result: *unknown* moves from 0 to 7

**Status there:** `planned`, and the encoding is marked *provisional until a
real CyberPower has been dumped*. Nothing is wired to it.

| Value | Before | After |
|---|---|---|
| 0 | no test initiated / unknown | **not used** |
| 1 | passed | passed |
| 2 | done, warning | done, warning |
| 3 | failed / error | failed / error |
| 4 | in progress | in progress |
| 5 | aborted | aborted |
| 6 | scheduled | scheduled |
| **7** | — | **no test initiated / unknown** |
| 65535 | not reported | not reported |

**1–6 are deliberately untouched.** `6` stays *scheduled* rather than being
reused for *unknown*, which was the first proposal and was rejected for
exactly that reason — it would have changed the meaning of a published value.
Only a new row is added, and row 0 is vacated.

The adapter never emits 6: no state in its UPS layer maps to *scheduled*.

---

## 2. Register 12 — beeper *(claimed from the reserved 12–15 range)*

| Value | Meaning |
|---|---|
| 1 | Disabled — it will not sound |
| 2 | Enabled — it is allowed to sound |
| 3 | Muted — the alarm sounding now is silenced; future ones are not |
| 65535 | this UPS has no controllable alarm, or comm lost |

**Source.** HID `AudibleAlarmControl` (Power page 0x5A), which reports these
three values directly — hence no renumbering. On the bridge that is NUT's
`ups.beeper.status`, whose strings map `disabled`/`enabled`/`muted` onto 1/2/3.

**It does not mean the alarm is sounding.** *Enabled* is permission, not noise.
No UPS in either track's experience reports a sounding alarm: the status word
is `OL OB LB CHRG DISCHRG RB OVER` with no alarm bit. The conditions that make
it sound are already discretes 1–4.

---

## 3. Register 13 — UPS state *(claimed from the reserved range)*

One point a site can watch on its own.

| Value | Meaning | True when |
|---|---|---|
| 1 | Normal | online, battery idle |
| 2 | **Self-test running** | online **and** discharging |
| 3 | Calibration running | `CAL` — the deep test |
| 4 | **On battery** | the mains actually failed |
| 5 | Not protecting | `OFF` or bypass: the load is not on the UPS |
| 6 | Unknown | readable, but none of the above fits |
| 65535 | comm lost | |

**The bridge can compute this identically**, from NUT status tokens it already
parses. No new UPS data is needed on either side:

```
comm lost        -> 65535
OB               -> 4   On battery
OFF | BYPASS*    -> 5   Not protecting
CAL              -> 3   Calibration running
OL and DISCHRG   -> 2   Self-test running
OL               -> 1   Normal
otherwise        -> 6   Unknown
```

**Evaluated worst first**, in that order, because more than one is true more
often than not. *On battery* beats any test deliberately: if the mains fails
while a calibration is running, the site is in a real outage and the point
must say so rather than go on reporting a test.

**Anything unexplained reads 6, never 1.** A summary point that reports calm
when it cannot tell is the one failure it must not have.

### Why this point exists

A self-test is `OL` **and** `DISCHRG`. The UPS stays *online* because the mains
never failed, and it is right to — a test that set `OB` would raise a false
mains-failure alarm at every site every time one ran, including the tests many
units schedule themselves.

The cost is that from the bus, a running test has been **indistinguishable
from an idle UPS**: Online true, On battery false, and no published flag for
the discharge. Register 13 state 2 is what tells them apart.

`DISCHRG` still has no point of its own on the adapter; register 13 consumes
it. If the bridge wants the raw flag as a discrete, that is a separate
conversation — the useful thing it adds is spotting *test in progress but the
battery never discharged*, a transfer that did not engage.

### On state 3

`CAL` is the only place in either stack where a **deep test is distinguishable
from a quick one**. NUT's `ups.test.result` carries a single *in progress* with
no type field, and the HID Test usage is written with one table and read back
with another that has no type either.

A unit that never sets `CAL` simply never reaches state 3, and its deep test
reads as state 2. Whether a CyberPower CPS ST sets it is **not yet verified on
hardware**.

---

## After these

Still reserved: **registers 14–15** (UPS facts) and **19–23** (adapter facts).

Discretes remain **0–10**. The adapter considered adding `DISCHRG`, `CAL` and
`NOBATTERY` as discretes 11–13 and did not: register 13 covers the first two,
and the discrete range stops at 10 in the published map with no reserved band
to draw from, so extending it needs its own decision.

`NOBATTERY` remains the most interesting unpublished flag in either track. A
UPS with no battery reports Online and looks perfectly healthy on every point
that exists today.
