---
id: '006'
title: 'Honest config provenance: DTR policy, per-group source on the reply, verified
  push'
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: A-live-config-push-is-wiped-by-the-next-reconnect.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Honest config provenance: DTR policy, per-group source on the reply, verified push

## Description

> **Plan from the issue's "Agreed fix (stakeholder, 2026-08-04)" section, not
> from any earlier options list in that file.** The issue was updated after a
> design discussion and the agreed shape is settled: three parts and one explicit
> non-goal. Source: `clasi/issues/A-live-config-push-is-wiped-by-the-next-reconnect.md`.

Sprint 132 delivered live per-wheel calibration over the wire, and it works —
but **only within a single serial connection.** Any reconnect silently discards
it, with no error and no indication anything was lost. Two independent facts
compose into silent data loss:

1. `SerialConnection.connect()` **pulses DTR on every direct-USB open**, which
   resets the MCU. Every fresh script invocation reboots the robot.
2. `DRIVE` is not in `Configurator::persistIfEligible()`'s flash-persistence
   table, so a live `DRIVE` push exists only in RAM.

This is precisely the "config that acks OK and does nothing" class of failure
sprint 132 set out to eliminate — reintroduced one level up, at the connection
boundary rather than inside the firmware.

Both halves were **measured**, not assumed. Robot clock across three consecutive
opens:

```
connect #1  PONG:t=817843 ms
connect #2  PONG:t=3085 ms      <- clock went BACKWARDS by 814.8 s
connect #3  PONG:t=3085 ms      <- deterministic; same boot-point read
```

And the reset buys nothing. `serial_conn.py`'s sprint-036 comment claims the DTR
pulse is required because the `DEVICE:` banner is boot-only. **That is stale.**
Opening with `dtr = False` and sending `HELLO`:

```
DEVICE:NEZHA2:robot:tovez:2314287040     <- classification works
PONG:t=40416                             <- and the clock did NOT reset
```

## Sequencing — why this ticket is last

There is a real argument for running this first: the DTR fix would make ticket
004's tuning session easier, since a push would survive a reconnect. Sprint
architecture Decision 5 **rejects** that. This change alters how every bench
script reaches the robot, and landing it immediately before the sprint's critical
measurement trades a convenience for a class of confounds.
`velocity_profile_gate.py` already asserts config **after** connect, which is the
working alternative, so 004 is not blocked. Do not re-order.

## Part 1 — stop asserting DTR on connect

`src/host/robot_radio/io/serial_conn.py`. Open with `dtr = False`.
`_banner_classify()` already sends `HELLO` and reads the response, which is what
actually produces the `DEVICE:` line today, so classification is unaffected.
**Update the stale comment** (around the `# Do NOT force dtr=False here` block)
rather than leaving a comment that contradicts the code.

Beyond the config bug this fixes something larger: today **you cannot observe a
robot without rebooting it** — every host connection disturbs the thing it is
measuring.

Two things to **check rather than assume**:

- **The relay path.** `_relay_handshake()` may genuinely need the reset to put
  the dongle in a known state. **Only direct USB was tested.** This is sprint
  Open Question 6. Answer it by measurement. If the relay needs it, keep DTR
  asserted for `RADIOBRIDGE` only — **a per-role policy is an acceptable and
  anticipated answer**, not a failure. Do not make it global in either direction
  without evidence.
- **Tooling that quietly assumes a fresh boot on connect.** The bench scripts are
  *believed* to get their cold boot from reflashing rather than connecting, but
  that assumption tends to live in a script's timing, not its comments. Check
  the ones that matter.

## Part 2 — per-group config provenance, reported on the reply

Each config group reports whether its current values came from the baked
`boot_config.cpp` or from a live wire push. **Not one global flag**: config is
seven groups, and `DRIVE` may be live while `PLANNER` is baked — a single flag
would have to lie about one of them.

Three constraints, all easy to miss and each one a rework if missed:

- **Stamp it in exactly one place.** `Configurator::applyGroup()` and
  `applyField()` (`src/firm/app/configurator.cpp`) are the **only two paths that
  mutate `config_`**. Stamping there makes it structurally impossible to forget.
  **Never stamp at a call site** — that is a convention, and conventions rot.
- **It must NOT be a field of the config struct.** A `source` field inside a
  group would land in the robot JSON too, breaking the **read-back-equals-file**
  property that is sprint 132's headline test — a file can never carry a
  runtime-assigned value. Provenance is a property of the *answer*, so it belongs
  on the `ConfigSnapshot` **reply message**
  (`src/protos/robot_config.proto`, `message ConfigSnapshot`).
- **Reset behaviour falls out for free.** `loadBaked()` stamps every group
  `BAKED`; a reset re-runs `loadBaked()`. Do not build separate reset handling.

Scope note to carry into the code comment: with the DTR fix and no persistence,
`LIVE` survives only while the robot stays powered. After any power cycle
everything reads `BAKED` **by definition**. That is the correct answer to "is the
robot running what I pushed" during a tuning session; it is not, and cannot be, a
claim about surviving a power cycle.

## Part 3 — verified push

`get_config()` already exists (132-011), so make the read-back **automatic rather
than optional**: a `verify=True` path on the field/group setters
(`src/host/robot_radio/robot/protocol.py`) that reads back and raises on
mismatch, used by default in bench tooling.

132-019 caught this whole defect only because it happened to read back. That
should not be luck. It also aligns with
`.claude/rules/configuration-discipline.md`, which permits ad-hoc development
pushes **because** read-back makes them safe — a guarantee this gap was
undercutting.

## ⛔ Explicitly NOT doing: persisting `DRIVE` to flash

There is room (the blob is 51 B against a 128 B ceiling after sprint 132), and it
would appear to solve the problem. **Do not.** Persistence reintroduces exactly
the ambiguity sprint 132 removed — a robot booting tuned values nobody in the
room knows about. `.claude/rules/configuration-discipline.md` says production
boot comes from the file. Good tuned values get promoted into the robot JSON,
which is the sanctioned path and which sprint 132 showed works (and which ticket
004 exercises).

Note for anyone who revisits this: the persisted blob's `static_assert` on the
128 B ceiling lives inside `#ifndef HOST_BUILD`, so an overflow is **invisible to
host tests** and only breaks at ARM build time. If you touch persistence at all,
build for ARM to learn the truth.

## Acceptance Criteria

- [ ] **DEFERRED to 004 — `tovez` was unplugged for the whole of this ticket.**
      Push a `DRIVE` value, disconnect, reconnect, `get_config(DRIVE)` — the
      value is still there, and its reported source reads `LIVE`. Only `getez`
      (relay) and `vevov` (NOT ours) were on the hub; no substitute board was
      used. `get_config_snapshot(DRIVE).source_name` is the call to make.
- [ ] **DEFERRED to 004 — same reason.** Power-cycle the robot, reconnect — the
      value is gone and its source reads `BAKED`. **The loss is visible, not
      silent.**
- [x] `DEVICE:` classification still works with DTR deasserted — verified
      against the **real `getez` relay** (banner returned in 0.007 s against a
      2.5 s budget) and pinned by
      `src/tests/unit/test_serial_conn_dtr_policy.py`, whose fakes emit a
      banner ONLY in reply to `HELLO`, never on open. The **direct-USB
      clock-does-not-reset half is NOT re-verified under the new code** — the
      measurement in this ticket's own Description was taken before the change
      and `tovez` was unavailable to repeat it. Ticket 004 closes that leg.
- [x] The relay path still handshakes under the DTR policy it ended up with,
      and the question **"does the relay need the reset?" is answered by
      measurement, not assumption: it does NOT.** Three consecutive
      handshakes on `getez` with no power cycle, `dtr=False`, all reached
      `# entering data plane`, identically to `dtr=True`. `!GO` genuinely
      gates the control plane (a `?` sent after `!GO` returns silence, not the
      channel line) — yet a no-DTR reopen finds the relay back in its control
      plane, because it leaves the data plane when the host closes the port.
      **No per-role exception was needed**; the policy is global.
- [x] The stale `serial_conn.py` comment claiming the DTR pulse is required is
      corrected — replaced with a "DTR policy" section carrying both
      measurements, and guarded by a test that fails if the old text returns.
- [x] Provenance is stamped at the `config_` **mutation sites only**, never at
      a call site. NOTE: there are **three** such sites, not the two the
      ticket anticipated — `reapplyPersistedTuning()` also writes `config_`.
      It stamps `PERSISTED` (see below). `loadBaked()` stamps every group
      `BAKED`, and re-running it is the whole of reset behaviour.
- [x] Provenance is **not** a field of the config struct, does not appear in
      any `data/robots/*.json`, and read-back-equals-file still holds —
      structurally guaranteed (`gen_messages.py`'s
      `_CONFIG_ENVELOPE_MESSAGE_NAMES` excludes `ConfigSnapshot` from both
      group-emission modes, so regenerating left the pydantic model and
      `robot_config.schema.json` byte-identical) and guarded by
      `src/tests/unit/test_config_provenance_not_in_json.py`.
- [x] Provenance is per-group, not a single global flag — the harness asserts
      a pushed group reads `LIVE` while every untouched sibling still reads
      `BAKED`.
- [x] A `verify=True` push against a deliberately-rejected value **raises**
      (`protocol.ConfigNotVerified`) rather than returning success, and is on
      by default in the shared bench push path
      (`calibration.push._push_via_proto`). `verify` defaults to False in
      `NezhaProtocol` itself so library/sim/REPL callers keep their return-code
      contract.
- [x] The 132-019 workaround — measure in the same connection as the push —
      **remains valid**: nothing about the same-connection path changed.
- [x] `DRIVE` is **not** persisted to flash. `Config::PersistedTuning` and
      `persistIfEligible()` are untouched by this ticket.

### Scope added beyond the ticket

- [x] A **fourth** `ConfigSource` value, `PERSISTED`, for groups restored at
      boot from the flash tuning snapshot. The ticket specified BAKED vs LIVE;
      a flash-restored group came from neither, and calling it `BAKED` would
      have been a fresh instance of the exact dishonesty this ticket exists to
      remove (a robot running tuned values its read-back denies). Affects
      WHEEL_CONTROL / MOTORS.travel_calib / OTOS only — the three persisted
      groups.
- [x] Two further **discovery probes** that rebooted every device they walked
      past — `serial_conn.probe_devices()` (also an MCP tool) and
      `testgui.transport._relay_probe_banner()` — moved to `dtr=False`. Merely
      asking "what is plugged in?" used to reboot the robot.
- [x] Incidental **pre-existing bug fixed**: `probe_devices()` unpacked
      `for kind, payload in demux.feed(chunk)`, but `ByteStreamDemuxer.feed()`
      returns a list of lines. Every probe of a device that answered died on
      "too many values to unpack", swallowed by the enclosing `except`. Fixed
      because it made the DTR claim unverifiable.

## Testing

- **Existing tests to run**: full collection —
  `uv run python -m pytest` (~11 min). This is the last ticket in the sprint, so
  this run is the sprint's exit gate. Compare against the baseline **by identity,
  not by count**: 5 failed / 1856 passed / 3 skipped / 12 xfailed / 2 xpassed,
  with the five named in the sprint's Test Strategy. Note ticket 004 may
  legitimately have moved `test_tour_closure_gate.py` and ticket 005 may have
  explained it — reconcile against what those tickets recorded, and name every
  entry that moved.
- **New tests to write**:
  - Host unit: provenance round-trips on the `ConfigSnapshot` reply; a group
    pushed live reads `LIVE` while an untouched sibling still reads `BAKED`
    (the specific case a single global flag would get wrong).
  - Host unit: `loadBaked()` stamps every group `BAKED`.
  - A test asserting no provenance field appears in any robot JSON and that
    read-back-equals-file still holds — this is the guard against the
    config-struct mistake.
  - Host unit: `verify=True` raises on a mismatched read-back, and returns
    normally on a match.
  - A `dtr=False` connect test at whatever level the host test harness supports
    without hardware.
- **Verification command**: `uv run python -m pytest`
- **Bench verification** (on `tovez`, by UID
  `9906360200052820a8fdb5e413abb276000000006e052820` — confirm with
  `uv run mbdeploy list`, never a remembered port): the push/disconnect/reconnect
  and power-cycle criteria above, plus a relay-path handshake check to answer the
  DTR question.
- **Never use `git stash`** — two long-lived stashes hold other people's WIP.
