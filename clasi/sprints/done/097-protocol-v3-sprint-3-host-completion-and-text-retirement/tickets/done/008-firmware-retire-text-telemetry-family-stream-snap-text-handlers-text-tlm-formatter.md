---
id: 008
title: 'Firmware: gut the text telemetry family'
status: done
use-cases:
- SUC-008
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: gut the text telemetry family

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own r1-conservative no-op scope in full).**
Eric's 2026-07-10 redirect: gut the firmware text plane unconditionally.
**DELETE**: `handleStream`/`handleSnap` and their `telemetryCommands()`
registrations, `kStreamSchema`, `telemetryEmit()`, and
`Telemetry::buildTlmFrame()` (the text-only formatter). ALSO DELETE
`handleTlm` (the one-shot text `TLM` verb, `motion_commands.cpp`) —
r1's Decision 7 preserved it as a bench-diagnostic rump with "no proven
binary substitute"; Decision 9 explicitly overrides that (its
bench-diagnostic fields — `acc`/`active`/`conn`/`glitch`/`ts` — are
already present in binary `Telemetry` per 096 Decision 6's union, and "no
proven substitute" is no longer, on its own, a preservation reason).
`handleTlm`'s deletion is this ticket's, not ticket 006's, since it lives
in the telemetry surface conceptually even though its source is in
`motion_commands.cpp` — coordinate the exact file edit with ticket 006 if
both land close together (same file, `motion_commands.cpp`, different
functions) to avoid a merge collision; note the coordination in
completion notes either way.

**Binary parity: 096, sim-exhaustive** (differential-vs-google.protobuf
byte-parity + fuzz + behavioral tests for `Telemetry`/`StreamControl`,
plus 096's own periodic-tick acceptance criteria: monotonic `seq=`,
correct on/off behavior). This ticket no longer depends on ticket 005's
verification (`depends-on: []`) — deletion is unconditional under
Decision 9.

`tickTelemetry()`'s `bb.telemetryBinary` branch: since only the binary
`stream` arm can set `bb.telemetryPeriod`/`.telemetryBinary` once
`handleStream` is gone, the text-emission branch becomes unreachable —
remove it. `Telemetry::tick()`/`buildTelemetryMessage()` (the binary
formatter, shared machinery) are UNTOUCHED.

**Every live text `STREAM`/`SNAP`/`TLM` sender r1 found —
`calibration/linear.py`, `calibration/angular.py`, TestGUI's connect-time
`"STREAM 50"`, and any ad-hoc `tests/bench/*.py` script sending raw
`"TLM"` — BREAKS against this firmware once this ticket lands, until
rewired to the `rogo` translator proxy (ticket 004).** This is an
accepted, stakeholder-approved consequence of Decision 9, not a
regression to fix here. State it plainly in completion notes.

## Acceptance Criteria

- [x] `STREAM`/`SNAP` are no longer registered as text verbs.
- [x] `Telemetry::buildTlmFrame()` (text formatter,
      `source/telemetry/tlm_frame.{h,cpp}`) is deleted.
      `Telemetry::tick()`/`buildTelemetryMessage()` (binary, shared) are
      byte-for-byte unchanged. **Nuance (see completion notes):**
      `buildTelemetryMessage()`'s own logic is untouched. `tick()`'s
      *observable* behavior for every field `buildTelemetryMessage()`
      reads is unchanged (verified by the harness's own unchanged exact
      expected values) — but its now-dead `in.mode = modeChar(...)`
      assignment (feeding ONLY the deleted text formatter, never read by
      `buildTelemetryMessage()`) was removed alongside `TlmFrameInput.mode`/
      `modeChar()`/`kAngleScale`/`appendField()`, all likewise write-only
      once `buildTlmFrame()` was gone. Source-level "byte-for-byte" does
      not hold literally; output-level behavior does.
- [x] `tickTelemetry()`'s now-unreachable text-emission branch is removed.
- [x] `handleTlm`/`TLM` (one-shot verb, `motion_commands.cpp`) is deleted
      and unregistered — coordinate this specific edit with ticket 006
      (same file).
- [x] `handleQlen`/`QLEN` is deleted per ticket 006's own scope, NOT this
      ticket's — confirm no duplicate/conflicting edit.
- [x] `tests/sim/unit/*` tests exercising text STREAM/SNAP/one-shot TLM
      are re-pointed at the binary `stream` arm (including a case
      exercising the host's `snap()`-equivalent arm-wait-disarm sequence,
      or the firmware-side portion of it) — coverage maintained.
- [x] `tests/sim` is green.
- [x] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded — expected to be a real, meaningful
      reduction (the text formatter + both handlers + the schema table).
- [x] Completion notes state plainly which live text `STREAM`/`SNAP`
      senders (listed in Description) now break against this firmware,
      and that rewiring them to the `rogo` proxy (ticket 004) is deferred
      to `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Implementation Plan

### Approach

1. Delete `handleStream`/`handleSnap`/`kStreamSchema`/`telemetryEmit()`
   and their `telemetryCommands()` registrations from
   `telemetry_commands.{h,cpp}`.
2. Delete `Telemetry::buildTlmFrame()` from `tlm_frame.{h,cpp}`.
3. Remove `tickTelemetry()`'s now-unreachable text branch.
4. Delete `handleTlm`/`TLM`'s registration from `motion_commands.cpp`
   (coordinate with ticket 006's own edits to the same file — confirm no
   overlap/conflict with `QLEN`'s deletion, which is ticket 006's).
5. Update `tests/sim/unit/*` per Acceptance Criteria.
6. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/telemetry_commands.{h,cpp}`
- `source/telemetry/tlm_frame.{h,cpp}`
- `source/commands/motion_commands.cpp` (`handleTlm`/`TLM` only — shared
  file with ticket 006, coordinate)
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`; it must now
  describe telemetry as binary-only, with the `rogo` proxy as the
  text-compatibility path).

## Completion Notes

**Deleted:** `handleStream`/`handleSnap`, `kStreamArgs`/`kStreamSchema`,
`kStreamFloorMs`, and `telemetryEmit()` (`telemetry_commands.cpp`) — their
`telemetryCommands()` registrations are gone; the function itself is kept
(still called from `command_router.cpp`, out of this ticket's file scope)
but now returns an empty vector. `Telemetry::buildTlmFrame()` plus its
now-orphaned helpers `appendField()`, `modeChar()`, `kAngleScale`, and the
`TlmFrameInput.mode` field (`tlm_frame.{h,cpp}`) — all four existed solely
to feed the text formatter and became dead the moment it was deleted.
`handleTlm`/`TLM`'s registration (`motion_commands.cpp`) — coordinated
cleanly with ticket 006, which had already gutted this file down to
`STOP`+`TLM` and explicitly left `TLM` for this ticket (see that file's own
header comment); no merge collision, `QLEN` was already gone (006's own
scope, confirmed untouched here).

**`tick()`/`buildTelemetryMessage()`:** `buildTelemetryMessage()` is
byte-for-byte unchanged (only a doc-comment edit). `tick()`'s field-sourcing
logic for every field `buildTelemetryMessage()` actually reads is
unchanged — reverified via `tlm_frame_harness.cpp`'s
`scenarioTickAssemblesFromBareBlackboard()`, whose expected values are
identical to before. The one non-literal deviation: `tick()`'s
`in.mode = modeChar(bb.planner.mode)` assignment, and the
`TlmFrameInput.mode`/`modeChar()`/`kAngleScale` machinery behind it, were
removed as dead code — `buildTelemetryMessage()` never read `in.mode` (it
reads `in.driveMode`, the raw enum, always has), so this had zero effect on
`buildTelemetryMessage()`'s output; it only ever fed the now-deleted
`buildTlmFrame()`. Flagged explicitly rather than silently claiming a
byte-for-byte source diff that doesn't literally hold.

**`tickTelemetry()`:** now unconditionally calls `telemetryEmitBinary()`.
`bb.telemetryBinary` (`blackboard.h`) is still WRITTEN by
`binary_channel.cpp`'s `stream` arm (`StreamControl.binary` is a real wire
field) but is no longer READ anywhere — an accepted, documented vestige
mirroring ticket 006's own `bb.motionIn` precedent (Open Question 1);
`blackboard.h`/`binary_channel.cpp` are outside this ticket's file scope so
were left untouched.

**Sim tests migrated:**
- `tests/sim/unit/test_telemetry_periodic_tick.py` — DELETED outright (not
  rewritten): its four text-STREAM/SNAP scenarios (ack-carries-no-frame,
  monotonic seq over 200ms, period-zero stops emission, SNAP standalone)
  are fully subsumed by `test_binary_channel.py`'s pre-existing binary
  `stream` tests (096-005), which already covered the identical behavior on
  the binary arm.
- `test_binary_channel.py` — `test_binary_stream_toggle_binary_false_
  reverts_to_text_with_shared_seq` rewritten as
  `test_binary_stream_binary_false_still_emits_binary_with_shared_seq`
  (its old premise, a live text fallback, no longer exists — emission now
  stays binary regardless of `StreamControl.binary`, with the shared `seq=`
  counter still continuing across the toggle).
  `test_binary_snap_still_works_standalone_while_binary_stream_is_active`
  DELETED (called `sim.command("SNAP")`, a now-nonexistent text verb, with
  no binary one-shot SNAP arm to re-point to — its "SNAP vs. STREAM
  non-interference" premise no longer applies once SNAP doesn't exist as a
  distinct verb).
- `test_bare_loop_commands.py` — `test_verb_outside_the_live_surface_
  replies_err_unknown` extended with `"STREAM 50"`/`"SNAP"`/`"TLM"` cases,
  proving acceptance criterion 1 (STREAM/SNAP no longer registered) and the
  TLM deletion at the wire level, alongside the C++-level proof that the
  handler functions no longer exist.
- `test_bare_loop_move_and_tlm.py` — every `sim.command("TLM")` call
  (11 tests) re-pointed at the binary `stream` arm via a new
  `_binary_envelope.read_tlm_now()` helper (arm/reset + one tick + read —
  the "host `snap()`-equivalent arm-wait-disarm" firmware-side portion the
  acceptance criteria ask for, mirroring `NezhaProtocol.snap()`'s own
  host-side synthesis in `protocol.py`). An EARLIER "arm once, peek many
  times for free" design was tried and found unsound: `sim_api.cpp`'s
  `ReplyStore` is a small fixed-size buffer with no wraparound that
  silently overflows and freezes after ~10-14 accumulated frames, so a
  multi-second test polling it without periodic resets would read a stale,
  mid-motion frame at the end (caught by 8 failing tests before the fix —
  see `_binary_envelope.py`'s own header comment for the full account).
  `test_pivot_completes_promptly_single_peaked` polls "is it idle" on
  nearly every iteration of a tight per-tick loop where even
  `read_tlm_now()`'s one extra tick per read would corrupt the exact
  single-peak/prompt-idle timing under test — it uses a new zero-cost
  `sim.active()` peek instead (`tests/_infra/sim/sim_api.cpp`'s
  `sim_get_active()`, a direct `bb.drivetrain.busy` read, added this ticket
  mirroring the pre-existing `sim_get_vel_l()`/`sim_get_enc_l()` precedent
  for exactly this "observe without perturbing timing" need). This is a
  small, deliberate extension of `tests/_infra/sim/sim_api.cpp`/
  `firmware.py` beyond the ticket's literal "Files to modify" list —
  additive test-only infrastructure, not a change to any deleted/retained
  production behavior, and necessary to satisfy "tests/sim is green" +
  "coverage maintained" without a flaky or corrupted regression test.

**ARM flash delta:** `just build` succeeds. `.map`/`arm-none-eabi-size`
before (stashed pre-ticket source, clean build): `text=319616 data=140823
bss=119824 dec=580263`, FLASH region 318972 B (85.58%). After (this
ticket's full diff, clean build): `text=317332 data=140823 bss=119824
dec=577979`, FLASH region 316688 B (84.96%). **Delta: -2284 bytes**
(text formatter + both text handlers + the schema table + the now-orphaned
helpers). `just build-sim` (`libfirmware_host`) also builds clean.
`uv run python -m pytest tests/sim -q` — 597 passed, 0 failed.

**Accepted breakage (Description/acceptance criterion 8):** every live
text `STREAM`/`SNAP`/`TLM` sender r1 found —
`host/robot_radio/calibration/linear.py`, `host/robot_radio/calibration/
angular.py`, TestGUI's connect-time `"STREAM 50"`, and any ad-hoc
`tests/bench/*.py` script sending raw `"TLM"` — now gets `ERR unknown`
against this firmware. This is the accepted, stakeholder-approved
consequence of Decision 9; rewiring these consumers to the `rogo`
translator proxy (ticket 004) is deferred to
`realign-host-tooling-to-gutted-four-verb-wire-surface.md`, not this
ticket's job.
