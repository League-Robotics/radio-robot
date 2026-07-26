---
id: '010'
title: 'Relay connect-handshake fix: stop kFaultCommsMalformed from tripping before
  any application command'
status: done
use-cases:
- SUC-008
depends-on: []
github-issue: ''
issue: relay-handshake-trips-comms-malformed.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relay connect-handshake fix: stop kFaultCommsMalformed from tripping before any application command

## Description

Independent, largely self-contained fix — needed clean before ticket
013's bench gate can pass. `kFaultCommsMalformed` trips on a fresh,
clean `!GO` relay connect with ZERO application commands sent, only over
the relay, never over direct USB (confirmed reproducible per the linked
issue's isolated test). Root-cause and fix it.

Per sprint 124 architecture Decision 5, the framing grammar itself
(tickets 003/005) needs **no** relay firmware change — the relay's `!GO`
data-plane mode is a transparent RAW250 byte pass-through, content-
agnostic. This ticket's root cause is therefore something else: most
likely leaked `!ECHO OFF`/`!MODE RAW250`/`!GO` control-plane bytes
momentarily reaching the robot's parser before the relay fully commits
to transparent pass-through, or a partial/fragment line at the exact
moment of the RAW250 mode transition. Investigate via a targeted
pyOCD/gdb session or byte-level capture on the relay-robot leg (per
`.claude/rules/debugging.md`) if needed; the issue's own "Direction"
section is the starting point.

## Acceptance Criteria

- [ ] Reproduces the fix against the exact isolated-test sequence the
      issue itself records: fresh clean-boot firmware, relay-only
      connect, zero application commands sent, ~1 s settle, inspect
      `fault_bits` — confirm bit 3 (`kFaultCommsMalformed`) stays clear.
      **Unchecked — no hardware in this environment** (`pyocd list`
      reports no probe, no `/dev/cu.usbmodem*`). The off-hardware
      EQUIVALENT is done: `scenarioRelayHandshakeChatterNeverCountsAsMalformed()`
      (`src/tests/sim/unit/app_comms_harness.cpp`) drives the exact same
      sequence through the real `App::Comms::pump()`/`dispatchLine()`
      code `RobotLoop` calls every cycle (`state_.health.commsMalformedCount
      = comms_.malformedCount()`, `robot_loop.cpp:826`, which is the sole
      producer of `fault_bits` bit 3) and confirms `malformedCount()`
      stays 0. Verified BEFORE the fix that this same test fails
      (`malformedCount()` == 5, one per leaked line) — see completion
      notes. The stakeholder's bench run on master must confirm the same
      result live (`fault_bits` bit 3 clear after a fresh relay `!GO`
      connect, zero application commands, ~1 s settle).
- [ ] `kFaultCommsMalformed` stays clear through a fresh relay connect
      with zero application commands, across multiple trials (matching
      the "reproduced across multiple fresh-boot trials" repro rigor
      the issue itself used).
      **Unchecked — no hardware.** Off-hardware equivalent: the same
      scenario runs 3 independent trials (a fresh `App::Comms` instance
      per trial, mirroring a fresh `mbdeploy deploy` + reconnect), all
      3 confirmed clean. Live multi-trial confirmation deferred to the
      stakeholder's bench run.
- [x] Root cause (leaked control-plane bytes vs. a host-side
      connect-timing race) is stated explicitly in this ticket's closing
      notes, whichever it turns out to be. See "Root Cause" below.
- [x] If the fix requires a relay dongle firmware change (contradicting
      Decision 5's expectation that it wouldn't), that is flagged
      explicitly here and in the sprint's Migration Concerns — not
      silently absorbed. **No relay dongle firmware change is required or
      proposed.** The fix lands entirely in this repo's own firmware
      (`src/firm/app/comms.{h,cpp}`) — Decision 5's expectation holds.

## Root Cause

**Determination: most likely leaked relay control-plane bytes reaching
the robot's own `radioLink_`, not a pure host-side connect-timing race —
based on code-level evidence, not a hardware capture (none was possible
in this environment).** This is a best-supported inference, not a
hardware-confirmed fact; see "What would settle it conclusively" below
for the observation that would upgrade it from inference to proof.

**Evidence against a pure host-side timing race:**
`SerialConnection.connect()` (`src/host/robot_radio/io/serial_conn.py`)
runs the identical "classify banner, then immediately poll `PING` with no
settle delay" pattern (`_banner_classify()` → `_poll_ready()`) on BOTH the
relay path and the direct-USB path — the only difference on the relay
path is the extra `_relay_handshake()` step in between (`?`/`!ECHO OFF`/
`!MODE RAW250`/`!GO`). If early-polling-after-connect were itself
sufficient to corrupt the wire, direct USB (same immediate-poll code,
zero settle delay) should occasionally show it too. The issue's own
repro record says it never does — direct-USB soak stayed `kFaultCommsMalformed`-clear
for the entire 240 s window, every time. The relay-only, always-reproduces
(issue: "reproduced across multiple fresh-boot trials"), fires-exactly-once
shape (never re-trips during a 240 s soak, matching `kFaultI2CSafetyNet`'s
own documented boot-time one-shot pattern) is far more consistent with a
deterministic side effect of the relay's OWN handshake sequence than with
a probabilistic host-side race window — a genuine race would be expected
to show at least occasional variance across trials, not 100% reproduction.

**Why "leaked bytes" is mechanically plausible on this specific wire:**
The robot's own radio receive path (`src/firm/com/radio.cpp`,
`Radio::onData()`) only ever publishes a complete "line" to
`Comms::dispatchLine()` after it sees a well-formed custom fragment
sequence (`FLAG_START`...`FLAG_END`, this project's own proprietary
radio-datagram framing, `FRAME_HEADER`/`MTU`) — so nothing can reach the
robot's parser as raw noise; whatever arrives must be a message the relay
dongle deliberately radioed onward using this same framing. Given the
relay dongle's job is bridging USB↔radio, and this project's own
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` documents
the dongle gating "forwards robot traffic" only after `!GO` (worded from
the relay→host direction — robot replies not leaking into the dongle's
own control-plane USB output before `!GO`), the docs make **no
corresponding claim about the host→robot direction being gated before
`!GO`**. If the dongle's own firmware forwards host→USB bytes onto radio
unconditionally (or its data-plane gate engages asynchronously relative
to when it emits its own `# entering data plane` confirmation back to the
host), then `_banner_classify()`'s `HELLO` probes and/or
`_relay_handshake()`'s `?`/`!ECHO OFF`/`!MODE RAW250`/`!GO` commands would
reach the robot verbatim over radio, each miss the closed v5 verb registry
(`messages/commands.h`'s `kVerbTable[]` has no `!`/`?`/`#`-prefixed entry),
and each increment `malformedCount_` once — consistent with a
multi-increment, single-connect-window event.

**What this repository cannot settle:** the relay dongle ("gozop") is a
separate, external firmware with no source in this tree (confirmed:
`find ... -iname "*relay*"` under `src/` turns up only host-side test/bench
scripts, never dongle source). Whether the dongle literally forwards
host-plane bytes onto radio before `!GO` commits, or the gate itself
engages with a small delay relative to its own USB status reply, cannot be
distinguished from inside this repo, and no hardware was available in this
environment to capture the actual bytes crossing the relay↔robot radio leg
at the moment of the trip.

**What would settle it conclusively:** a targeted pyOCD/gdb session
(`.claude/rules/debugging.md`) breaking in `Comms::dispatchLine()` at the
`isRelayControlPlaneLine()`/`findVerb()` call (see Fix, below) during a
live relay connect, to capture the EXACT bytes the robot's `radioLink_`
delivered at the moment of each pre-fix `malformedCount_` increment — or
equivalently, a logic-analyzer/second-probe capture on the relay↔robot
radio leg across a fresh `!GO` connect (the issue's own "Direction"
section, both options unavailable in this hardware-less environment).
Either would show definitively whether the leaked content is literally
the host's own `?`/`!...` command text (supporting "leaked control-plane
bytes") or something else entirely.

## Fix

`App::Comms::dispatchLine()` (`src/firm/app/comms.cpp`) now recognizes
and silently drops (before the registry lookup, without incrementing
`malformedCount_`) any inbound line whose first byte is `#`, `!`, or `?` —
the radio-relay dongle's own control-plane sigils (a status/comment
reply, a dongle command, and the dongle's config query, respectively;
`isRelayControlPlaneLine()`). No registered v5 verb name starts with any
of these three bytes (`messages/commands.h`'s `kVerbTable[]`: `HELLO`,
`PING`, `ID`, `VER`, `DEVICE`, `PONG`, `MOVE`, `CONFIG`, `STOP`, `TLM`,
`OK`, `ERR`), so this is a narrow, symmetry-restoring carve-out — it
cannot mask a genuine malformed command, only tolerate exactly the shape
of noise the relay dongle's own protocol is known to emit. It mirrors a
tolerance the HOST side already had for `#` lines
(`serial_conn.py`'s `_handle_wire_line()`: `if line.startswith(b"#"): return`)
but the robot's own firmware never had, for any of the three sigils —
confirmed by reading the PRE-124-005 `comms.cpp` (`git show 90e424f7^`):
"ANY text line that isn't one of the two recognized commands counts as
malformed" was already true before ticket 005's framing-grammar cutover,
so v5's new grammar did NOT already fix this on its own (the sprint
description's "it is entirely possible v5's grammar already fixes this"
possibility is confirmed FALSE by this check).

This fix is entirely host-repo-side (robot firmware only); it requires no
relay dongle firmware change, consistent with Decision 5.

## Testing

- **Existing tests to run**: existing relay-connect tests/bench scripts
  (`src/tests/bench/relay_telemetry_rate.py`, `dev_exercise.py`) — deferred
  to the stakeholder's bench run (no hardware here).
- **New tests written** (`src/tests/sim/unit/app_comms_harness.cpp`,
  run via `src/tests/sim/unit/test_app_comms.py`):
  - `scenarioRelayHandshakeChatterNeverCountsAsMalformed()` — the
    isolated-repro sequence above, 3 fresh-boot trials, each queuing the
    dongle's exact control-plane vocabulary (`?...`, `!ECHO OFF`,
    `!MODE RAW250`, `!GO`, `# entering data plane`) onto `radioFake`
    (mirroring exactly where a real relay-leaked message lands —
    `radioLink_`, never the robot's own silent/unconnected USB
    `serialLink_`) with zero application commands, draining every line
    (the "settle window"), and confirming `malformedCount()` stays 0.
    Verified to FAIL against the pre-fix code (`malformedCount() == 5`).
  - `scenarioRelayCarveOutIsNarrowAndDoesNotAffectSubsequentRealCommand()` —
    confirms the carve-out does NOT broaden to non-sigil unrecognized
    lines (still counted malformed) and leaves no residual demux state
    that could corrupt a real command (`PING`) arriving immediately after
    relay chatter.
- **Verification command**: `uv run python -m pytest` (full suite, ran
  in foreground); `just build-clean` (builds both the ARM firmware and
  the host-sim library). Bench run per
  `.claude/rules/hardware-bench-testing.md` (relay path specifically) is
  the stakeholder's own follow-up on master — no hardware available here.
