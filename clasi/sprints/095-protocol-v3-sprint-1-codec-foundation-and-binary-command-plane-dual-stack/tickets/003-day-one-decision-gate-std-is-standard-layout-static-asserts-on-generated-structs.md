---
id: '003'
title: 'Day-one decision gate: std::is_standard_layout static_asserts on generated
  structs'
status: open
use-cases: [SUC-002]
depends-on: ['001']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Day-one decision gate: std::is_standard_layout static_asserts on generated structs

## Description

Decide, cheaply and BEFORE the expensive field-table/`wire.{h,cpp}`
codegen work (ticket 005) is written, whether the generator's planned
`offsetof`-based `FieldDesc` tables are viable — per the issue's own
"day-one decision gate" framing. This ticket does NOT emit any field
table; it only emits the one check that decides the fork in the road.

1. Extend `scripts/gen_messages.py` to ALSO emit, per generated struct
   that a future field-descriptor table would need `offsetof` into
   (every message reachable from `CommandEnvelope`/`ReplyEnvelope`,
   transitively — so at minimum every message in `envelope.proto`,
   `motion.proto`, `drivetrain.proto`'s `DrivetrainCommand`/`SetPose`/
   `WheelTargets`, and `common.proto`'s shared types), one
   `static_assert(std::is_standard_layout<msg::Xxx>::value, "msg::Xxx
   must be standard-layout for offsetof-based field tables");` — either
   inline in each generated header (requires `#include <type_traits>`) or
   collected into one new generated aggregator header (e.g.
   `source/messages/layout_checks.h`) that `#include`s every relevant
   header and asserts on each. Pick whichever is cleaner to generate and
   document the choice; either satisfies the acceptance criteria below.
2. Compile it as part of the NORMAL build (`just build` ARM + `just
   build-sim`) — no new build step, no new CI job.
3. Record the outcome — every struct passed, or a specific named subset
   failed — in this ticket's own completion notes.
4. **If any struct fails**: do not attempt to fix it in this ticket.
   Instead, name the failing struct(s), the likely reason (e.g. a
   non-standard-layout member introduced by a future proto change), and
   explicitly flag that ticket 005 must use the documented fallback
   (generated per-message UNROLLED decode/encode functions behind the
   SAME `msg::wire::decode`/`encode` API) for those structs specifically,
   rather than the generic `offsetof`-table walker, for THIS sprint's
   implemented arms. Do not silently proceed as if the gate passed.

Also record, in this ticket's completion notes (informational, not a
blocking acceptance criterion — see `architecture-update.md` Open
Question 5 / Risk 2): every struct these asserts check is standard-layout
but NOT trivial (every generated field has a default member initializer,
e.g. `float x = 0.0f;`). `offsetof` on a standard-layout-but-non-trivial
type is CONDITIONALLY-SUPPORTED, not unconditionally guaranteed, under
strict C++11/C++14 wording (this project targets CODAL C++11) —
unconditionally guaranteed only from C++17 onward. GCC/Clang
(arm-none-eabi-g++, this project's actual toolchain) define the behavior
in practice, matching universal embedded-C++ practice (nanopb and
protobuf-c generated bindings rely on the identical guarantee) — the
`static_assert` this ticket adds checks the theoretically load-bearing
property (standard-layout), not the C++11-vs-C++17 technicality itself,
which cannot be `static_assert`ed. Note it in the generated table's own
header comment (ticket 005's job to write that comment; this ticket's job
to surface the nuance so it isn't lost).

## Acceptance Criteria

- [ ] Every struct reachable from `CommandEnvelope`/`ReplyEnvelope`
      (transitively, per the message list above) has a generated
      `static_assert(std::is_standard_layout<msg::Xxx>::value, ...)`.
- [ ] `just build` (ARM) and `just build-sim` both compile the new
      asserts successfully — OR fail with a clear, specific compiler
      error naming the failing struct(s) (not a generic linker/other
      failure that obscures which assert tripped).
- [ ] The pass/fail outcome is written into this ticket's own completion
      notes: either "all N structs pass" (list them) or "structs X, Y
      fail; ticket 005 must use the unrolled-codegen fallback for these"
      (name them specifically).
- [ ] The `offsetof`-on-non-trivial-standard-layout C++11 nuance
      (paragraph above) is recorded in this ticket's completion notes so
      ticket 005 can carry it into the generated table's own header
      comment.
- [ ] The full existing sim suite stays green (this ticket adds compile-
      time-only asserts, zero runtime behavior change).

## Testing

- **Existing tests to run**: `just build` (ARM, confirms the asserts
  compile on the real target toolchain), `just build-sim`, `uv run python
  -m pytest tests/sim -q`.
- **New tests to write**: none beyond the compile-time asserts themselves
  — there is no runtime behavior to unit test in this ticket. If it's
  cheap, a `tests/unit/` regression guard (mirroring
  `test_gen_messages_no_getters.py`'s existing pattern of scanning
  `generate_headers()`'s in-memory output) that confirms the assert lines
  are actually present in the generated output is a reasonable bonus, not
  required.
- **Verification command**: `just build && just build-sim && uv run
  python -m pytest tests/sim -q`.
