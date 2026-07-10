---
sprint: "095"
status: done
revises: architecture-update.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Architecture Update r1 -- Sprint 095: Per-arm wire-budget bound + incremental, non-breaking arm declaration

This is a **focused revision**, triggered by an INTERNAL-surface exception
thrown from ticket 005 (`gen_messages.py`: FieldDesc tables + `wire.{h,cpp}`
generation). It does not restate `architecture-update.md` — read that
document first for the full Sprint 095 design (Steps 1-7, Decisions 1-5).
This document adds one Decision that **revises** part of Decision-adjacent
material in the original's Step 5 ("What Changed") and Risk 3 (the
186-byte cap), and is now the active planning artifact for the schema's
`CommandEnvelope`/`ReplyEnvelope` oneof-arm set. `architecture-update.md`
itself is preserved unmodified as the calibration record of what was
originally planned.

## The exception (as thrown by ticket 005's programmer)

Ticket 005's own acceptance criterion — `kMaxEncodedSize` for
`CommandEnvelope` `<= 186` bytes, enforced by a generated `static_assert`
that fails the build — is structurally unsatisfiable as long as
`CommandEnvelope`'s declared-only `motion` arm (field 5, `PlannerCommand`,
per ticket 001/`architecture-update.md`) stays in the schema.
`PlannerCommand`'s own worst-case encoded size is **327 bytes**:
`repeated StopCondition stops = 9 [(max_count) = 4]` alone costs 160B
(repeated-message fields are never packed — each of the 4 occurrences is
separately tagged), the `goal` oneof's worst arm costs 23B, and the two
`string` fields (`corr_id`, `verb`) cost 66B each at the generator's fixed
`char[64]` width. That's before `CommandEnvelope`'s own oneof wrapper
overhead or any other arm is counted.

Unlike `DeviceId` (Open Question 3's already-anticipated fix — a brand-new
`envelope.proto` type, ticket-001-owned, no other consumer, a narrow
"shrink 3 strings" latitude the original document already sanctioned),
`PlannerCommand` is a **pre-existing `protos/planner.proto` message with a
live non-wire consumer**: `source/commands/motion_commands.cpp`'s
text-plane R/TURN/G handlers construct `msg::PlannerCommand` today
(`copyCorrId()` sizes into `corr_id[64]`; `stops_[4]`'s capacity is relied
on) — shrinking it to fit the wire budget would touch text-plane code well
outside `gen_messages.py`/`protos/envelope.proto`, and is out of a
codegen ticket's authority to decide unilaterally. The programmer correctly
declined to either (a) narrow `kMaxEncodedSize`'s computation to exclude
declared-only arms (defeats the static_assert's whole build-time-catch
purpose) or (b) redesign `motion`'s wire shape unilaterally (architecture-
level, the same level Decision 2 was made at) — and threw instead. Full
detail is preserved in ticket 005's own `exception:` frontmatter block.

## Decision 6 — revises "declare all arms now": per-arm 186B budget, arms declared incrementally as each is bounded and implemented

**Context**: the original document's Step 5 ("What Changed") and Risk 3
followed the issue's own stated design — declare every `CommandEnvelope`/
`ReplyEnvelope` oneof arm this sprint (implemented and declared-only alike)
"so 096/097/098 slot in without a schema break." That framing implicitly
assumed every declared-only arm's payload type already fits the 186-byte
envelope budget without being checked per-arm at declaration time — an
assumption ticket 005's hand-computed sizing (done BEFORE writing any
codegen, per its own due-diligence extension of Open Question 3) disproved
for the `motion` arm specifically.

**Alternatives considered**:
1. **Keep `motion` declared, exclude it from the `kMaxEncodedSize`
   computation.** *Rejected*: defeats the exact purpose Risk 3 and ticket
   005's acceptance criteria state for the generated `static_assert` — "a
   compile-time check... if a schema change pushes an envelope over budget,
   the build fails loudly here, not at runtime." Silently carving out an
   exemption is the one outcome that guarantees a FUTURE oversized-arm
   regression is caught nowhere, not even at build time.
2. **Shrink `PlannerCommand` (narrow `stops`'s `max_count` and/or
   `corr_id`/`verb`'s width) to fit.** *Rejected*: out of this sprint's and
   this ticket's scope — `PlannerCommand` has a live text-plane consumer
   (`motion_commands.cpp`'s R/TURN/G handlers) that would need
   re-verification against any shrink; changing a pre-existing,
   text-plane-serving message's shape to satisfy a not-yet-implemented
   binary arm is a disproportionate, out-of-scope edit for a codegen
   ticket to make unilaterally.
3. **Give `motion` a new, deliberately-bounded wire payload type now**
   (the same move Decision 2 made for `segment`/`replace`: a purpose-built
   `MotionSegment` rather than exposing `Motion::Segment` directly).
   *Rejected for THIS sprint*: `motion`'s consumer (`Subsystems::Planner`)
   is parked with no live path (093/094) and R/TURN/G aren't even
   registered in the text table today — designing a bounded wire shape for
   a still-parked subsystem is speculative generality the architecture
   document's own Quality Checks explicitly watch for, and the sprint that
   un-parks the Planner is better positioned to know what that subsystem
   actually needs on the wire.
4. **Remove the `motion` arm now; declare arms incrementally, each
   sprint that implements one, once its payload type is defined AND
   verified to fit.** *Chosen.*

**Why the chosen alternative**: a new `protobuf` oneof arm is a new field
number — adding one later is **non-breaking** (ticket 004's unknown-field-
skip means older firmware/host simply skip a field number they don't
recognize yet). The original "declare everything up front" caution
against a future "schema break" was solving a problem the wire format
doesn't actually have: there is no cost to reserving a oneof field number
later, and no benefit to reserving one now for a payload that doesn't fit
the budget the schema itself enforces. Declaring `motion` now, unbounded,
purchases nothing and creates the exact build-breaking conflict ticket 005
hit. The revised principle:

> Every `CommandEnvelope`/`ReplyEnvelope` oneof arm's payload must
> independently fit the 186-byte envelope budget, enforced by the
> generated `kMaxEncodedSize` static_assert at build time. A oneof arm is
> declared only once its wire-bounded payload type both exists and fits.
> Arms are added incrementally, one sprint at a time, as each is
> implemented — this is non-breaking in protobuf, so the original
> up-front-declaration caution is unnecessary.

**Consequences**:
- `protos/envelope.proto`'s `CommandEnvelope.cmd` oneof loses field 5
  (`PlannerCommand motion`) this sprint. Field number 5 is simply unused
  until a future sprint reintroduces `motion` with a bounded payload type
  (matching Decision 2's `MotionSegment` precedent) — no renumbering of
  any other arm is needed or permitted (field numbers, once declared on a
  shipped schema, are never reused for a different meaning; 5 stays
  reserved/skipped, not reassigned).
- `DeviceId`'s three `string` fields shrink `char[64]` -> `char[48]`
  (`model`/`name`/`fw_version`) per the already-sanctioned Open Question 3
  latitude — `ReplyEnvelope{DeviceId}` worst-case becomes 171B (was 210B
  unshrunk), comfortably under 186B. This is a mechanical follow-through
  of the ORIGINAL document's own OQ3, not a new decision.
- Every remaining declared-only arm this sprint retains
  (`config`/`ConfigDelta`, `pose`/`SetPose`, `otos`/`OdometerCommand`,
  `get`/`ConfigGet`, `stream`/`StreamControl`) must independently pass the
  same `kMaxEncodedSize<=186` check ticket 005 already generates — the
  programmer's own due-diligence pass reported only `PlannerCommand` over
  budget, but the generated `static_assert` is the actual proof, not the
  hand-computation; ticket 005's revised acceptance criteria (below) make
  this an explicit, checked step rather than an assumption.
- **Forward note for sprints 096/098**: `Telemetry` and `ConfigSnapshot`
  (currently empty placeholder arms on the `ReplyEnvelope` side, owned by
  096) and `SetPose`/`OdometerCommand` (098) inherit this SAME per-arm
  budget rule when they are populated with real fields — curated
  telemetry / chunked config, per the original issue's own design, is how
  096 is expected to stay inside 186B for a necessarily larger payload
  class; this is not new scope for 096, just an explicit inherited
  constraint worth restating here so it isn't rediscovered as a second
  exception.
- No change to any of the original document's Decisions 1-5, Modules
  M1-M8, diagrams, or Risk 1/2/4/5 — this revision is scoped to the single
  schema-fit issue above.

## Concrete changes (ticket 005's new Step 0 — see revised ticket)

1. Remove `PlannerCommand motion = 5;` from `CommandEnvelope.cmd` in
   `protos/envelope.proto`. Field number 5 stays reserved/skipped.
2. Shrink `DeviceId.model`/`DeviceId.name`/`DeviceId.fw_version` from
   `char[64]` to `char[48]` (via the proto generator's string-width
   convention — whatever mechanism ticket 001 used to get `char[64]` in
   the first place; if the generator has no per-field string-width option
   yet, that's ticket 005's to add, minimally, for this one case).
3. Verify every remaining declared arm
   (`ConfigDelta`/`SetPose`/`OdometerCommand`/`ConfigGet`/
   `StreamControl`) has worst-case `kMaxEncodedSize <= 186` — the
   generated `static_assert` is the authority; report each arm's computed
   size in ticket 005's completion notes regardless of outcome.

## Status

This revision is the team-lead's resolution call, recorded here per the
Exception Protocol (the conflict was correctly escalated as
architecture-level rather than resolved unilaterally by the ticket 005
programmer). No further architecture self-review round is required for
this narrow a change — Decision 6 does not touch M1-M8's boundaries,
diagrams, or dependency graph, and the original document's Quality Checks
still hold. Ticket 005 is revised separately (`tickets/005-...md`) to
carry out the concrete changes above as its first implementation step.
