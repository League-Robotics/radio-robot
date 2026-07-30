---
id: '003'
title: 'TLM command surface: TLM:ON/AUTO/OFF/NOW, STATUS tlm=, HELP, NezhaProtocol
  (Part 4)'
status: done
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: telemetry-emit-policy-rebuild-spec.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TLM command surface: TLM:ON/AUTO/OFF/NOW, STATUS tlm=, HELP, NezhaProtocol (Part 4)

## Description

Add the host-facing control surface for the `TlmMode` ticket 002 built.
Protocol v5 grammar is `<COMMAND>[':' <data>]` — mode controls are
colon-spelled only; `TLM ON` with a space is NOT valid and gets no
special-case parse.

Full inbound `TLM` surface (issue Part 4, normative table):

| Line | Effect | Reply |
|---|---|---|
| `TLM` | one frame now (existing, kept) | one binary telemetry frame |
| `TLM:NOW` | alias for bare `TLM` | one binary telemetry frame |
| `TLM:ON` | `mode_ = kOn` | the `STATUS` line |
| `TLM:AUTO` | `mode_ = kAuto` | the `STATUS` line |
| `TLM:OFF` | `mode_ = kOff` | the `STATUS` line |
| `TLM:<anything else>` | no mode change | the `HELP` line |

Decisions to implement exactly as specified:
- Arguments (`NOW`/`ON`/`AUTO`/`OFF`) are matched case-insensitively.
- Mode changes reply with the existing `STATUS` line formatter, not a new
  reply shape — no bespoke ack is invented.
- `STATUS` gains a `tlm=off|auto|on` field, appended to the existing
  `STATUS:k=v` formatter in `comms.cpp` (`sendStatus()`). This is a
  wire-visible key string — spelled exactly `tlm=` per the coding-
  standards wire-key exclusion; do not rename it.
- `HELP` is updated to list `TLM[:NOW|ON|AUTO|OFF]` (see `sendHelp()`).
- Plumbing follows the EXISTING bare-`TLM` pattern exactly: `Comms`
  parses the `TLM:` argument in the same place it currently recognizes
  bare `TLM` (`decodeInbound`'s `entry->verb == msg::Verb::TLM` branch,
  `comms.cpp` around line 247) and stages a pending mode-change/request;
  `RobotLoop::cycle()` consumes it at the same point it already consumes
  `takeTelemetryRequest()` today, calling `tlm_.setMode(...)` and/or
  forcing the emit. `Telemetry` itself never parses wire text — all
  parsing stays in `Comms`.
- No persistence: nothing is written to config; power-cycle always
  returns to `kAuto`.

Host mirror: `wire_commands.py` already carries the `TLM` verb (no verb
table change needed — confirm this by reading the current table before
assuming). `NezhaProtocol` (`src/host/robot_radio/robot/protocol.py`)
gains three trivial helpers — `tlmOn()`, `tlmOff()`, `tlmNow()` — thin
wrappers sending the lines above, following the project's Python naming
convention (lowerCamelCase, no units in identifiers).

## Acceptance Criteria

- [x] `Comms` recognizes `TLM:NOW`, `TLM:ON`, `TLM:AUTO`, `TLM:OFF`
      case-insensitively, colon-spelled only (`TLM ON` with a space is
      NOT special-cased and falls through to whatever the unrecognized-
      argument path already does).
- [x] `TLM:ON`/`TLM:AUTO`/`TLM:OFF` change `Telemetry`'s mode via the
      `RobotLoop::cycle()` → `Telemetry::setMode()` path (mirroring the
      existing `takeTelemetryRequest()` consume point) and reply with the
      `STATUS` line.
- [x] `TLM:NOW` behaves identically to bare `TLM` (one frame, no mode
      change).
- [x] `TLM:<garbage>` (anything not matching NOW/ON/AUTO/OFF) changes no
      state and replies with the `HELP` line.
- [x] `STATUS`'s formatter (`sendStatus()`) gains `tlm=off|auto|on`,
      reflecting the CURRENT mode at reply time.
- [x] `HELP`'s formatter (`sendHelp()`) lists `TLM[:NOW|ON|AUTO|OFF]`.
- [x] No mode is ever written to config/persisted storage — verified by
      a sim test that sets a non-default mode, simulates reboot
      (fresh `Telemetry`/`Comms` construction), and observes `kAuto`.
- [x] `NezhaProtocol` gains `tlmOn()`, `tlmOff()`, `tlmNow()`, each a
      thin wrapper sending the corresponding wire line — no new retry/
      backoff behavior beyond what the existing `TLM` path already has.
- [x] `wire_commands.py`'s verb table is confirmed unchanged (or updated
      if investigation shows it actually needs an entry — state which in
      the completion notes).

## Testing

- **Existing tests to run**: `Comms` unit tests covering `STATUS`/`HELP`
  formatting and the existing bare-`TLM` request path — confirm the
  `tlm=` field addition and the new `HELP` line do not break any
  existing string-match assertion (search for exact `STATUS:`/`HELP:`
  string literals in tests and update them to include the new field).
  Host-side: existing `NezhaProtocol`/`wire_commands.py` tests.
- **New tests to write**: sim tests for each row of the Part 4 table
  (each `TLM:` variant → correct mode + correct reply), plus a host-side
  test exercising `tlmOn()`/`tlmOff()`/`tlmNow()` against a
  `SimConfigConn`-backed connection (mirroring how `square_tour.py` and
  similar scripts already drive `NezhaProtocol` against sim).
- **Verification command**: `uv run pytest` (host) plus the sim
  HOST_BUILD test target for `app_comms`/`app_telemetry` harnesses.

## Completion Notes

- `wire_commands.py` confirmed unchanged: `TLM` was already present
  (`VerbEntry("TLM", True)`) before this ticket; no verb-table edit was
  needed, matching `commands.h`'s `kVerbTable[]` on the firmware side.
- `Comms::TlmAction` (comms.h) is the new enum RobotLoop consumes
  (`kNone/kFrame/kSetOff/kSetAuto/kSetOn/kUnrecognized`), replacing
  `takeTelemetryRequest()`'s plain bool with `takeTlmAction()`. All
  parsing (including the case-insensitive `NOW/ON/AUTO/OFF` match and
  the single-trailing-`\r` allowance for a human-typed line) stays in
  `comms.cpp`'s anonymous-namespace `classifyTlmArg()`; `Telemetry` never
  sees wire text.
- `Comms::sendTlmReply(TlmAction)` sends the STATUS/HELP reply on the
  SAME transport the triggering line arrived on (remembered via a
  `Transport*` member set at parse time); RobotLoop calls it AFTER
  `setStatus()` so a mode-change reply's `tlm=` field always reports the
  mode just applied, not the previous cycle's.
- `sendHelp()`'s binary-verb skip now special-cases `TLM` (still
  `binary: true` in the registry, since a real outbound telemetry frame
  IS binary) so its own inbound cleartext control surface is listed as
  `TLM[:NOW|ON|AUTO|OFF]`.
- Verified with `just build-clean` (firmware .hex + host sim lib, both
  succeeded) and `uv run python -m pytest src/tests/sim/ -q` → 418
  passed, 1 skipped, 1 xfailed, 0 failed (matches the pre-existing
  baseline). New coverage: 11 scenarios in `app_comms_harness.cpp`
  (compiled+run standalone, all pass) covering every row of the Part 4
  table plus reply-transport/reply-shape checks, and 4 end-to-end
  scenarios in `app_robot_loop_harness.cpp` proving the wire→
  `Telemetry::setMode()` path — the latter could not be executed because
  `app_robot_loop_harness.cpp`/`test_app_robot_loop.py` was ALREADY
  broken at this ticket's start (pre-existing, unrelated `App::Drive`/
  `RobotLoop` constructor API drift — confirmed via `git stash`: the
  same 28 compile errors reproduce at commit f12d28d1, before any of
  this ticket's edits; `test_app_robot_loop_harness_compiles_and_passes`
  is already `xfail(strict=False)` and stays that way). The added
  scenarios have no compile errors of their own (checked in isolation
  with `-fsyntax-only -ferror-limit=0`) and will run once that
  pre-existing, out-of-scope breakage is fixed.
