---
id: '005'
title: 'Framing grammar cutover: uniform command/reply lines, PONG/ID/VER, deletion
  of the old text/binary heuristics'
status: in-progress
use-cases:
- SUC-001
depends-on:
- '001'
- '003'
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Framing grammar cutover: uniform command/reply lines, PONG/ID/VER, deletion of the old text/binary heuristics

## Description

The centerpiece framing change. Implement the uniform grammar
`<COMMAND>[':' <data>]'\n'` in both directions, both transports:
`App::Comms` (command prefix parse/emit, dispatch by registry lookup
from ticket 001, using ticket 003's delimiter/CRC-scope codec),
`Com::SerialPort`/`Com::Radio` (unconditional `\n` split, no
text/binary heuristic at the transport layer, `send()`/`sendText()`
terminator convergence), and the host mirrors (`wire_codec.py`,
`serial_conn.py`).

New reply-plane handlers: `PONG:t=<ms>` (replacing `OK pong t=<ms>`),
`ID:<fields>` (configured-robot identity — drivetrain type + calibration
profile name/version, per sprint 124 architecture Decision 4 — exact
field names finalized in this ticket), `VER:<version>` (reads the
existing generated build-version constant, `src/firm/types/
version_generated.h` — no new version-tracking infrastructure). Confirm
the existing `DEVICE:` banner (already restored in 123-006, not
re-planned here) parses unmodified under the new grammar and that both
its emission sites end up `\n`-terminated once `sendReliable()` drops
`\r\n`.

Delete, don't deprecate: `kTextCommands[]`/`isRecognizedTextCommand()`
(`serial_port.cpp`), `_looks_like_text()`/`_TEXT_SAFE_BYTES`
(`wire_codec.py`), the dead host text-branch routing (`TLM`/`EVT`/
`OK|ERR|CFG|ID` in `serial_conn.py`'s `_handle_text_line()`), and
`App::FrameKind` as a transport-level concept (the transport delivers a
line; `Comms` decides text-vs-binary from the parsed command via the
registry).

## Acceptance Criteria

- [ ] The 123-006 hardware repro (`move_wheels` embedding a literal
      `0x0A`) executes 10/10 over direct USB (radio-relay confirmation
      is ticket 013's job). **BLOCKED — see "Bench Verification" section
      below: no physical hardware (no pyOCD probe, no `/dev/cu.usbmodem*`)
      is reachable from this session's environment.** The software-level
      guarantee this AC is meant to confirm on real hardware IS covered:
      `app_comms_harness.cpp`'s `scenarioDataContainingColonAndZeroRound
      TripsCorrectly` round-trips a binary payload engineered to contain
      an embedded `0x0A` byte, decoded correctly under the new
      `\n`-unconditional-terminator contract — the exact byte shape the
      123-006 hardware repro exercised. This is NOT a substitute for the
      real-hardware run; it only shows the fix that makes the repro
      possible is in place and independently tested.
- [x] `kTextCommands[]`/`isRecognizedTextCommand()` and
      `_looks_like_text()`/`_TEXT_SAFE_BYTES` are deleted, not just
      unused (grep-enforceable). Verified:
      `grep -rn "kTextCommands\|isRecognizedTextCommand" src/firm/` and
      `grep -rn "_looks_like_text\|_TEXT_SAFE_BYTES" src/host/ src/tests/`
      match only doc-comment prose describing the deletion, no live symbol.
- [x] The dead host text branches (`TLM`/`EVT`/`OK|ERR|CFG|ID` routing,
      `_CORR_ID_RE`) are gone, and no test depends on them. Verified via
      the same grep sweep — `_CORR_ID_RE` matches only doc-comment prose.
- [ ] `HELLO`→`DEVICE:`, `PING`→`PONG:`, `ID`→`ID:`, `VER`→`VER:` all
      answer on both transports (serial confirmed here; relay confirmed
      in ticket 013), and every emitted line parses under the grammar.
      **PARTIALLY BLOCKED — same hardware gap as AC1.** All four verbs
      are proven correct at the sim/host layer:
      `scenarioHelloRepliesWithBannerViaSendReliable`,
      `scenarioPingRepliesOkPongViaSendReliable`,
      `scenarioIdRepliesWithConfiguredIdentity`,
      `scenarioVerRepliesWithBuildVersion` (`app_comms_harness.cpp`) each
      decode the exact reply line and assert it round-trips under
      `_split_wire_line()`/the C++ grammar parser. The "serial confirmed
      here" real-hardware half of this AC is outstanding.
- [x] `DEVICE:NEZHA2:robot:<name>:<serial>` parses unmodified as command
      `DEVICE` + cleartext data on the first `:` — no format change,
      byte-freeze holds. Verified:
      `scenarioHelloRepliesWithBannerViaSendReliable` asserts the exact
      banner string round-trips verbatim through `sendReliable()`; the
      `formatBanner()` call site itself is untouched by this ticket.
- [x] Both banner emission sites (`main.cpp` boot, `robot_loop.cpp`
      `boot()`) are `\n`-terminated, not `\r\n`. Verified by code
      inspection: both go through `SerialPort::sendReliable()`/
      `Radio::send()` (main.cpp) or `Comms::sendBanner()` →
      `Transport::sendReliable()` (robot_loop.cpp), and every one of
      those now appends a single trailing `\n`, never `"\r\n"`
      (`serial_port.cpp`, `radio.cpp`).
- [x] Host reads the wire via plain `readline()` in at least one test
      path — the demonstration that the demuxer is no longer load-bearing.
      `src/tests/unit/test_wire_grammar.py`'s
      `test_plain_readline_recovers_every_line_including_embedded_0x00`
      and `test_plain_readline_via_a_serialconnection_shaped_fake`.
- [x] Grammar edge cases pass: data containing `:`; data containing
      `0x00`; a no-data verb with a stray trailing `:`; an unknown
      command (counted malformed, not crashed); a truncated line.
      `test_wire_grammar.py` (host) + `app_comms_harness.cpp`'s
      `scenarioStrayTrailingColonOnNoDataVerbHandledGracefully`,
      `scenarioTruncatedBinaryLineCountsMalformedNotCrash`,
      `scenarioDataContainingColonAndZeroRoundTripsCorrectly` (C++).

## Testing

- **Existing tests to run**: `test_serial_conn_binary_plane.py`,
  `test_protocol_binary_client.py`, `app_comms_harness.cpp`.
- **New tests to write**: the grammar edge-case tests above; a
  host/firmware round-trip test interleaving text-plane and binary-plane
  lines on the same connection.
- **Verification command**: `uv run pytest` plus the C++ sim-tests build;
  a serial bench smoke test per `.claude/rules/hardware-bench-testing.md`.

## Implementation Summary

Uniform grammar `<COMMAND>[':' <data>]'\n'` landed symmetrically on both
sides:

- **Firmware (`src/firm/app/comms.{h,cpp}`):** `App::kCobsDelimiter =
  0x0A` (COBS now keyed on `\n`, per 124-003's parameterized primitive).
  `Comms::dispatchLine()` parses the `<COMMAND>` prefix off an
  already-`\n`-delimited line and looks it up in the generated registry
  (`messages/commands.h`), dispatching by the registry's own `binary`
  flag to `decodeBinaryFrame()` or the new `dispatchCleartext()`
  (HELLO/PING/ID/VER). `Comms::sendReply()` derives the outbound verb
  from `reply.body_kind` internally — no caller-supplied command string.
  `Transport::readLine()` collapsed from `FrameKind` to `bool`.
  `Telemetry::emitSecondary()` reuses the `TLM` verb/CRC-scope (no
  registry entry exists for the secondary frame; documented, narrow,
  does not touch tickets 007-009's field-packing scope).
- **Transports (`com/serial_port.*`, `com/radio.*`):** unconditional
  `\n`-split, no heuristic; `Radio::sendText()` deleted, `send()` alone
  now appends `\n` for every outbound line on both transports.
- **Host (`wire_codec.py`, `serial_conn.py`, `sim_loop.py`,
  `sim_config.py`, `binary_bridge.py`, `clock_sync.py`):**
  `ByteStreamDemuxer.feed()` collapsed from `(kind, payload)` tuples to
  plain `list[bytes]`, split on `0x0A` alone. `_split_wire_line()`/
  `_handle_wire_line()` mirror `dispatchLine()`/`dispatchCleartext()`.
  `clock_sync.py`'s `_parse_pong_t()` updated for `PONG:t=<ms>`.

**Deletions (grep-confirmed gone, not merely unused):**
`kTextCommands[]`/`isRecognizedTextCommand()` (`serial_port.cpp`),
`App::FrameKind`/`Radio::FrameKind` (both transport-level and the app
seam), `Radio::sendText()`, `_looks_like_text()`/`_TEXT_SAFE_BYTES`
(`wire_codec.py`), `_CORR_ID_RE` and the dead `TLM`/`EVT`/
`OK|ERR|CFG|ID` text-branch routing (`serial_conn.py`).

**`ID:`/`VER:` decisions this ticket finalized:** `ID:` reports
CONFIGURED identity — `ID:<drivetrain>:<calibration-profile-name>:
<firmware-version>` (e.g. `ID:differential:tovez_nocal:v0.20260724.2` for
the currently-active `tovez_nocal.json`, or `ID:mecanum:togov:...` for
`togov.json`), built in `main.cpp` from two new generated constants,
`Config::kDrivetrainType`/`Config::kRobotProfileName`
(`scripts/gen_boot_config.py` — `kDrivetrainType` from the robot JSON's
own `identity.drivetrain_type` enum, defaulting to `"differential"` per
the schema's own documented default; `kRobotProfileName` from the boot
config JSON's own filename stem, or `"unconfigured"` for the baked-
default sentinel). `VER:` reads the existing generated
`FIRMWARE_VERSION_STR` (`version_generated.h`) directly — zero new
version-tracking infrastructure, per architecture Decision 4.

**Self-caught defect, fixed before commit:** an earlier draft of the
`ID:` wiring derived drivetrain type from
`drivetrainConfig.half_track > 0.0f` (a wire-level `msg::
DrivetrainConfig` field), reasoning it would be nonzero only for the
mecanum family. That field is never actually baked by
`defaultDrivetrainConfig()` (`boot_config.cpp`) for ANY robot profile —
it stays at its wire default-member-initializer `0.0f` always — so the
check always evaluated `false` regardless of the real drivetrain,
silently reporting "differential" even for a mecanum profile. Caught by
inspecting the generator/`boot_config.cpp` diff before finalizing this
ticket (not caught by any existing test — no test exercises `main.cpp`'s
composition root directly). Fixed by adding a proper generated
`Config::kDrivetrainType` constant baked from the robot JSON's own
`identity.drivetrain_type` field instead — verified against both
`tovez_nocal.json` (→ `"differential"`) and `togov.json` (→
`"mecanum"`). Net effect on shipped behavior: none — the only robot
JSON `just build` currently targets (`tovez_nocal.json`, differential)
happened to produce the correct string either way, so this was a latent
defect, not an observed regression, but a real one worth recording since
it would have silently mis-reported `ID:` the moment a mecanum profile
was built.

**Latent bug found and fixed along the way (not separately ticketed,
directly required by this ticket's own delimiter change):** every
`std::string`/`const char*` call site that had been passing an armored
binary line through a `strlen()`-based API became unsafe the moment
COBS re-keyed off `0x0A` (output can now legitimately contain embedded
`0x00`, which `strlen()` truncates at). Fixed across ~10 C++ test
harnesses, `SimHarness::injectCommand()`, and the `sim_ctypes.cpp` /
`sim_loop.py` ctypes boundary (`sim_inject_command` gained an explicit
`int len` parameter). A second bug in the same area:
`sim_ctypes.cpp`'s `sim_drain_tlm()` had its doc comment updated to
describe the `\n` delimiter but the actual code still wrote a literal
`0x00` — found via a failing sim-test regression, fixed.

**Red-then-green test migrations (old heuristic asserted, not merely
touched):**
`test_reader_loop_binary_branch_coexists_with_every_existing_branch`
(asserted the deleted `TLM`/`EVT`/`OK #5`/`ERR badarg #6` text routing)
→ renamed/rewritten to assert cleartext+comment+binary coexistence
against the CURRENT dispatch. `test_host_wire_codec.py`'s entire
`ByteStreamDemuxer` section rewritten for the `list[bytes]` API;
`test_demuxer_recognizes_relay_comment_lines_with_no_0x00_ever` and
`test_demuxer_recognizes_device_banner_and_pong_reply_dynamic_content`
deleted outright (they tested `_looks_like_text()`'s own behavior,
which no longer exists — no replacement needed at that layer, the
equivalent coverage is registry-based dispatch, now in
`test_wire_grammar.py`). All `"OK pong t=..."` literals updated to
`"PONG:t=..."` project-wide.

**Grammar edge-case coverage deliberately NOT added as rows to the
shared `wire_golden_vectors.txt` fixture (ticket 004):** that fixture's
schema is codec-COMPOSITION-only (a positive encode/decode round-trip,
cross-language) and already had colon/`0x00`-in-payload coverage before
this ticket. The three remaining cases this ticket names (stray
trailing colon, unknown command, truncated line) are PARSING/dispatch-
layer behaviors with no "expected_wire_hex" to assert — they live in
`test_wire_grammar.py` (mirroring `app_comms_harness.cpp`'s own scenario
set) instead. See that file's own docstring for the full rationale.

**Verification:** `uv run python -m pytest` — 1464 passed, 2 skipped, 9
xfailed, 2 xpassed (no regressions; skips/xfails pre-existing).
`just build` (ARM target) and `just build-sim` (host-sim library) both
succeed clean. `src/firm/app/DESIGN.md`, `src/firm/messages/DESIGN.md`,
and `src/firm/com/DESIGN.md` updated to describe the new grammar (the
last was not explicitly named in this ticket's own file list, but
documented in detail the exact `FrameKind`/`kTextCommands`/`sendText()`
mechanisms this ticket deletes, so leaving it stale would have been
actively misleading).

## Bench Verification — BLOCKED, no hardware reachable this session

Per `.claude/rules/hardware-bench-testing.md` and this ticket's own AC1/
AC4, the grammar cutover should be exercised on the real robot
("tovez") over direct USB before this ticket can be considered fully
done. This session's environment has **no physical hardware reachable
at all**: `pyocd list` reports "No available debug probes are
connected", there is no `/dev/cu.usbmodem*` (or any USB-serial) device
node, and `system_profiler SPUSBDataType` shows no DAPLink/micro:bit
device present. This is an environment limitation, not a code or
process question — there is nothing to flash or talk to from here.

**What is and isn't covered as a result:** every acceptance criterion
that can be verified in software (deletions, grammar parsing, cleartext
reply content, banner byte-freeze, `\n`-termination, plain-`readline()`
safety, grammar edge cases) is verified and checked off above, backed
by the full pytest suite and a successful ARM firmware build. The two
criteria that specifically require a live USB link to the robot — the
123-006 hardware repro (AC1) and the "serial confirmed" half of
HELLO/PING/ID/VER (AC4) — remain open. The identical byte-level fix
those two criteria exist to confirm (an embedded `0x0A` inside a binary
frame body no longer corrupts demux) IS independently proven at the
sim/host layer (`scenarioDataContainingColonAndZeroRoundTripsCorrectly`),
which is the strongest evidence obtainable without the physical link.

**Recommendation:** this ticket's status is left `in-progress`, not
`done`, and `move_ticket_to_done` has deliberately not been called
(matching the standing instruction not to call it directly regardless).
A follow-up session with the robot physically connected needs to run
`mbdeploy probe` / `mbdeploy deploy --build`, then repeat the
`move_wheels`-with-embedded-`0x0A` repro 10x and confirm
`HELLO`/`PING`/`ID`/`VER` all answer correctly over serial, before this
ticket's remaining two acceptance criteria can be checked off and the
ticket moved to done.
