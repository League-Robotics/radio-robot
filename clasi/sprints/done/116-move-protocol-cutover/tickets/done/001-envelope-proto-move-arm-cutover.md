---
id: '001'
title: envelope.proto MOVE arm cutover
status: done
use-cases:
- SUC-050
depends-on: []
github-issue: ''
issue:
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# envelope.proto MOVE arm cutover

## Description

Foundation ticket for the whole sprint — every other ticket depends on the
regenerated `msg::Move`/`msg::MoveTwist`/`msg::MoveWheels` types existing.
Cuts `envelope.proto` over per the protocol set-point issue: add `message
Move` (`MoveTwist | MoveWheels` velocity oneof, `time | distance | angle`
stop oneof, required `timeout`, `replace`, `id`) at fresh arm number **21**
on `CommandEnvelope.cmd`; delete `Twist` (arm 19), folding it into
`CommandEnvelope`'s `reserved` list; delete `ConfigDelta.watchdog` (field
4), folding it into `ConfigDelta`'s `reserved` list. `ConfigTarget.
CONFIG_WATCHDOG` stays declared-unused (same treatment `CONFIG_PLANNER`
already got after 115's `PlannerConfigPatch` deletion — an enum value
costs nothing to leave, needs no `reserved`). No hand-written codec
changes — `python build.py` regenerates `msg::*` and the `wire.h` codec
from the edited protos.

This ticket only implements the wire-schema slice, not the full protocol
contract (queue, stop conditions, host builders, doc) — tickets 002-010
complete the rest. `completes_issue` is left at its default (`true`):
both linked issues are fully resolved within this sprint (nothing
deferred beyond ticket 010), so normal archival behavior — once every
referencing ticket across the sprint is done — is what's wanted here, not
suppression.

## Acceptance Criteria

- [x] `Move`/`MoveTwist`/`MoveWheels` messages added to `envelope.proto`,
      matching the protocol-set-point issue's shape exactly (field names,
      numbers, oneof groupings).
- [x] `Twist` message body deleted; `CommandEnvelope`'s `reserved` list
      gains `19`.
- [x] `ConfigDelta.watchdog` (field 4) deleted; `ConfigDelta`'s `reserved`
      list gains `4`. `ConfigTarget.CONFIG_WATCHDOG` enum value left
      declared, unused — not removed.
- [x] `python build.py` regenerates `msg::Move`/`MoveTwist`/`MoveWheels`
      and the `Comms`/`wire.h` codec cleanly (no hand-edits to generated
      output — fixes go in the generator/protos only, per the project's
      generated-code convention). (`scripts/gen_messages.py`/`scripts/
      gen_pb2.py` regen is clean; `build.py`'s own tree-wide ARM compile
      fails at `src/firm/app/robot_loop.cpp` exactly as expected — it still
      calls `handleTwist()`/decodes `Twist`, and is ticket 006's cutover to
      fix, not this ticket's.)
- [x] The regenerated `kCommandEnvelopeMaxEncodedSize` (`comms.h`) is
      re-measured and recorded; `kArmoredBufSize` (256 B) is confirmed to
      still have headroom over the new worst-case encoded size. (Measured:
      `kCommandEnvelopeMaxEncodedSize` stays 50 B — `config`=44B/`stop`=2B/
      `move`=38B, `config` remains the dominant arm despite `move` being a
      structurally bigger message than the `twist` it replaced;
      `kReplyEnvelopeMaxEncodedSize` stays 153B, `kMaxEnvelopeBytes`=153B,
      armored line ≈207B, well under the 256B `kArmoredBufSize`.)
- [x] `wire_test_codec.h` gains `armorMoveCommand()` helper(s) covering
      both velocity variants × all three stop kinds; `armorTwistCommand()`
      is deleted. (Two overloads, disambiguated by `MoveStopKind`'s
      type-incompatible parameter position; `stopKind`/`stopValue` cover
      all three stop kinds through each.)
- [x] A repo-wide grep for the deleted wire arms (`Twist` as a wire type,
      `watchdog`/`sTimeout` as `ConfigDelta` fields) outside `src/archive/`
      turns up only call sites this sprint's later tickets are already
      scheduled to fix (ticket 007 for host `protocol.py`, ticket 006 for
      firmware) — not a surprise this ticket needs to also fix. (Confirmed:
      `src/firm/app/robot_loop.cpp` + `src/tests/sim/unit/
      app_comms_harness.cpp` + `src/sim/sim_harness.h`/`sim_ctypes.cpp` —
      ticket 006; `protocol.py`/`nezha.py`/`cli.py`/`repl.py` + their tests
      — ticket 007. The codec-layer test harnesses this ticket's own
      Testing section requires green — `wire_codec_harness.cpp`,
      `wire_differential_harness.cpp`, `_wire_diff_driver.py`,
      `test_wire_differential.py`, `test_wire_fuzz.py` — were updated in
      this ticket, since they are this ticket's own codec-level acceptance
      surface, not later-ticket territory.)
- [x] `src/firm/messages/DESIGN.md` updated in place: §3's envelope-size
      note (`Move` at arm 21, `Twist`/`ConfigDelta.watchdog` reserved, and
      the `CommandEnvelope`/`ReplyEnvelope` size figures) reflects the
      regenerated `kCommandEnvelopeMaxEncodedSize`/`kReplyEnvelopeMaxEncodedSize`
      measured above. This doc does not ride the sprint's design overlay —
      the sprint's one co-located-`DESIGN.md` overlay slot is already used
      by `app/DESIGN.md` (the overlay mechanism cannot host two files both
      named `DESIGN.md` in one flat overlay directory) — so it is edited
      directly on the canonical doc here; see sprint.md's Design overlay
      note.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_wire_codec.py`,
  `test_wire_differential.py`, `test_wire_fuzz.py`, `test_wire_runtime.py`
  (must stay green through the regen — these exercise the codec
  generically, not `Twist`/`Move` specifically).
- **New tests to write**: proto-shape round-trip coverage for `Move` —
  encode/decode each velocity variant × each stop kind combination and
  confirm the decoded `msg::Move` matches the encoded input; confirm a
  decode attempt against the old `Twist` arm number (19) or `ConfigDelta`
  field 4 is rejected/ignored as a reserved number, not silently accepted.
- **Verification command**: `python build.py && uv run python -m pytest
  src/tests/sim/unit/test_wire_codec.py src/tests/sim/unit/test_wire_differential.py
  src/tests/sim/unit/test_wire_fuzz.py`
