---
status: pending
priority: high
---

# A live config push is silently wiped by the next reconnect

## Description

Sprint 132 delivered live per-wheel calibration over the wire — and it works,
but **only within a single serial connection.** Any reconnect silently discards
it, with no error and no indication anything was lost.

Found on hardware during 132-019's bench acceptance. The first push attempt
appeared to do nothing: push, then re-measure via a *separate* script
invocation, and the robot behaved exactly as before.

## Cause

Two independent facts compose into a silent data loss:

1. **`SerialConnection.connect()` pulses DTR on every direct-USB open**, which
   resets the MCU. Every fresh script invocation therefore reboots the robot.
2. **`DRIVE` is not in `Configurator::persistIfEligible()`'s flash-persistence
   table**, so a live `DRIVE` push exists only in RAM.

Together: a live push is erased by the very next connection, and nothing
reports it. This is precisely the "config that acks OK and does nothing" class
of failure sprint 132 set out to eliminate — reintroduced one level up, at the
connection boundary rather than inside the firmware.

Confirmed by measuring in the SAME connection as the push, where the gap did
close (trapezoid 11.0 → 1.6 points, square 12.4 → 0.6 points) and
`get_config(DRIVE)` read back the exact pushed values.

## Measured facts (2026-08-04, `tovez`, direct USB)

Both halves of the diagnosis were tested rather than assumed.

**Connecting resets the MCU.** Robot clock across three consecutive
`SerialConnection` opens:

```
connect #1  PONG:t=817843 ms
connect #2  PONG:t=3085 ms      <- clock went BACKWARDS by 814.8 s
connect #3  PONG:t=3085 ms      <- deterministic; same boot-point read
```

**The reset is not needed.** `serial_conn.py`'s sprint-036 comment claims the
DTR pulse is required because the `DEVICE:` banner is boot-only. That is stale.
Opening with `dtr = False` and sending `HELLO`:

```
DEVICE:NEZHA2:robot:tovez:2314287040     <- classification works
PONG:t=40416                             <- and the clock did NOT reset
```

`HELLO` elicits the banner. The reset buys nothing and costs the config.

## Agreed fix (stakeholder, 2026-08-04)

### 1. Stop asserting DTR on connect

Open with `dtr = False`. `_banner_classify()` already sends `HELLO` and reads
the response, which is what actually produces the `DEVICE:` line today, so
classification is unaffected. Update the stale comment.

Beyond the config bug this fixes something larger: **today you cannot observe a
robot without rebooting it.** Every host connection disturbs the thing it is
measuring.

Two things to check rather than assume:

- **The relay path.** `_relay_handshake()` may genuinely need the reset to put
  the dongle in a known state. Only direct USB was tested. If the relay needs
  it, keep DTR asserted for `RADIOBRIDGE` only — a per-role decision, not a
  global one.
- **Tooling that quietly assumes a fresh boot on connect.** The bench scripts
  are believed to get their cold boot from reflashing rather than connecting,
  but that assumption tends to live in a script's timing, not its comments.

### 2. Per-group config provenance, reported on the reply

Each config group reports whether its current values came from the baked
`boot_config.cpp` or from a live wire push. Not one global flag: config is
seven groups, and `DRIVE` may be live while `PLANNER` is baked — a single flag
would have to lie about one of them.

Three constraints on the implementation:

- **Stamp it in exactly one place.** `applyGroup()` and `applyField()` are the
  only two paths that mutate `config_`. Stamping there makes it structurally
  impossible to forget; stamping at call sites is a convention that rots.
- **It must NOT be a field of the config struct.** A `source` field inside a
  group would land in the robot JSON too, breaking the read-back-equals-file
  property that is sprint 132's headline test — the file can never carry a
  runtime-assigned value. Provenance is a property of the *answer*, so it
  belongs on the `ConfigSnapshot` reply message.
- **Reset behaviour falls out for free.** `loadBaked()` stamps every group
  `BAKED`; a reset re-runs `loadBaked()`.

Scope note: with the DTR fix and no persistence, `LIVE` survives only while the
robot stays powered. After any power cycle everything reads `BAKED` by
definition. That is the correct answer to "is the robot running what I pushed"
during a tuning session; it is not, and cannot be, a claim about surviving a
power cycle.

### 3. Verified push

`get_config()` already exists, so make the read-back automatic rather than
optional: a `verify=True` path on the field/group setters that reads back and
raises on mismatch, used by default in bench tooling. 132-019 caught this
defect only because it happened to read back; that should not be luck.

### Explicitly NOT doing: persisting `DRIVE` to flash

There is room (the blob is 51 B against a 128 B ceiling after sprint 132), but
persistence reintroduces exactly the ambiguity sprint 132 removed — a robot
booting tuned values nobody in the room knows about. `.claude/rules/configuration-discipline.md`
says production boot comes from the file. Good tuned values get promoted into
the robot JSON, which is the sanctioned path and which sprint 132 showed works.

Note for anyone who revisits this: the persisted blob's `static_assert` on the
128 B ceiling lives inside `#ifndef HOST_BUILD`, so an overflow is invisible to
host tests and only breaks at ARM build time.

## Verification

- Push a `DRIVE` value, disconnect, reconnect, `get_config(DRIVE)` — the value
  is still there, and its reported source reads `LIVE`.
- Power-cycle the robot, reconnect — the value is gone and its source reads
  `BAKED`. The loss is visible, not silent.
- `DEVICE:` classification still works on direct USB with DTR deasserted, and
  the relay path still handshakes (whatever DTR policy it ends up with).
- A `verify=True` push against a deliberately-rejected value raises rather than
  returning success.
- The 132-019 workaround (measure in the same connection as the push) must
  remain valid.

## Related

- Sprint 132 ticket 019's completion notes carry the full 8-run bench record.
- [[the-configuration-object]] — the design this completes.
- `.claude/rules/configuration-discipline.md` — the rule permits ad-hoc dev
  pushes *because* read-back makes them safe. This gap undercuts that: a
  read-back in a later session shows the baked value, not what you pushed.
