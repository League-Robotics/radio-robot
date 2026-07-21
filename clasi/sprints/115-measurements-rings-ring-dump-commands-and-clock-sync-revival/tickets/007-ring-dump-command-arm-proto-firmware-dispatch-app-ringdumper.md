---
id: '007'
title: 'Ring-dump command arm: proto + firmware dispatch (App::RingDumper)'
status: open
use-cases:
- SUC-115-002
depends-on:
- '004'
- '005'
- '006'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Ring-dump command arm: proto + firmware dispatch (App::RingDumper)

## Description

Depends on tickets 004, 005, 006 (needs the `Measurements` container and
real producers so the dump path has real data to exercise against, not
just empty rings). Adds the debug ring-dump command surface: a new
`CommandEnvelope` oneof arm selects one of the five rings; firmware
replies with a burst of frames (one record per packet) terminated by a
count/done marker, over the existing frame-drain path.

## Implementation Plan

- **Approach — wire schema** (`src/protos/envelope.proto`): add a
  `RingSelector` enum (`RING_EXTERNAL`, `RING_OTOS`, `RING_ENCODER_POSE`,
  `RING_ENCODER_LEFT`, `RING_ENCODER_RIGHT` — SCREAMING_SNAKE per this
  file's existing enum convention), a `RingDump` command message wrapping
  a `RingSelector`, and a per-record reply message (a `oneof` over a pose-
  shaped and an encoder-shaped record, plus `bool done` / `uint32 count`
  fields for the terminator). Add `RingDump` to `CommandEnvelope`'s
  `oneof cmd` at the next FRESH field number after `move = 20` (do not
  reuse any number in `reserved 2, 3, 4, 5, 7 to 12, 14 to 18`). Add the
  new record/dump reply type to `ReplyEnvelope`'s `oneof body` at the
  next free number after `reserved 5 to 11` (i.e. 12 or higher, whichever
  is actually free at implementation time — verify against the file's
  current state before picking a number). Regenerate C++/Python codegen
  per the project's normal `gen_messages.py`/protoc flow.
- **Approach — firmware** (new `App::RingDumper`, `src/firm/app/ring_dumper.{h,cpp}`):
  given a `const Devices::Measurements&` and a `RingSelector`, walk the
  selected ring's currently-published samples OLDEST to NEWEST (note:
  `MeasurementRing::sample(age)` returns newest-first — the dumper must
  walk ages `kDepth-1` down to `0`, skipping any `!valid` slot, i.e. a
  ring with fewer than `kDepth` published samples so far), emit one reply
  frame per valid record, then emit the terminator frame with the total
  count sent. Read-only over `Measurements` — this module never
  publishes.
- **Approach — dispatch**: `App::RobotLoop`/`App::Comms` decode the new
  `ring_dump` `CommandEnvelope` arm and call `RingDumper`; this is one
  new `switch`/`if` branch alongside the existing `config`/`stop`/
  `twist`/`move` handling — no existing branch changes.
- **Files to create**: `src/firm/app/ring_dumper.h`, `.cpp`.
- **Files to modify**: `src/protos/envelope.proto`, `src/firm/app/robot_loop.cpp`
  (or wherever the command dispatch switch lives — confirm exact
  location), `src/firm/main.cpp`/sim composition root (construct
  `RingDumper` with a reference to the `Measurements` instance from
  ticket 004).
- **Testing plan**: sim test dumping each of the five rings — `external`
  (always empty this sprint: expect zero records + clean terminator),
  `otos`/`encoderPose`/`encoderLeft`/`encoderRight` (populated by tickets
  005/006: expect N records matching however many valid samples are
  published, oldest-first, terminator count matching frames sent).
- **Documentation updates**: none beyond proto/inline doc comments
  matching `envelope.proto`'s existing header-comment style (e.g. the
  "Every pre-102 arm this prune removes... reserved, not reused"
  discipline — new arms follow the same "fresh number, documented"
  pattern).

## Acceptance Criteria

- [ ] New `CommandEnvelope`/`ReplyEnvelope` field numbers are fresh
      (never previously used or reserved) — verified against the file's
      current `reserved` ranges before assignment.
- [ ] Dumping an empty ring (`external`, this sprint) yields zero record
      frames plus a clean done/count(=0) terminator — no hang, no error.
- [ ] Dumping a populated ring yields exactly one frame per currently-
      published record, OLDEST first, followed by the terminator; the
      terminator's count matches the number of record frames sent.
- [ ] `App::RingDumper` never publishes into any ring (read-only,
      verified by code review / a test asserting ring contents are
      unchanged after a dump).
- [ ] `RobotLoop`'s existing `config`/`stop`/`twist`/`move` dispatch
      branches are unchanged.
- [ ] Sim test dumps all five rings and confirms frame counts/shapes per
      the Testing Plan above.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` sim suite
  (confirm no existing `CommandEnvelope` dispatch test regresses); `just
  build-clean`.
- **New tests to write**: see Implementation Plan's Testing Plan bullet.
- **Verification command**: `uv run pytest`
