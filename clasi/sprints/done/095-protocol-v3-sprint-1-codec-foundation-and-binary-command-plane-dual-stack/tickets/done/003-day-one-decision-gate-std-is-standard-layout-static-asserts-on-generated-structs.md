---
id: '003'
title: 'Day-one decision gate: std::is_standard_layout static_asserts on generated
  structs'
status: done
use-cases:
- SUC-002
depends-on:
- '001'
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

- [x] Every struct reachable from `CommandEnvelope`/`ReplyEnvelope`
      (transitively, per the message list above) has a generated
      `static_assert(std::is_standard_layout<msg::Xxx>::value, ...)`.
- [x] `just build` (ARM) and `just build-sim` both compile the new
      asserts successfully — OR fail with a clear, specific compiler
      error naming the failing struct(s) (not a generic linker/other
      failure that obscures which assert tripped).
- [x] The pass/fail outcome is written into this ticket's own completion
      notes: either "all N structs pass" (list them) or "structs X, Y
      fail; ticket 005 must use the unrolled-codegen fallback for these"
      (name them specifically).
- [x] The `offsetof`-on-non-trivial-standard-layout C++11 nuance
      (paragraph above) is recorded in this ticket's completion notes so
      ticket 005 can carry it into the generated table's own header
      comment.
- [x] The full existing sim suite stays green (this ticket adds compile-
      time-only asserts, zero runtime behavior change).

## Completion Notes (2026-07-10)

**KEY PREREQUISITE resolved: `gen_messages.py`'s cross-file `#include`
gap (flagged by ticket 001) is now FIXED, here, not deferred to ticket
005.** `_emit_file()` previously emitted exactly one fixed
`#include "messages/common.h"` for every non-`common.proto` header
(the old `_OTHER_INCLUDE` constant), with no per-file cross-reference
tracking — `envelope.h` referenced `DrivetrainCommand`/`MotionSegment`/
`PlannerCommand`/`SetPose`/`OdometerCommand` from other generated headers
with no matching `#include`, so a standalone compile of `envelope.h`
failed with "unknown type name" for all five (verified independently
during this ticket, matching ticket 001's own scratch-compile finding).
Replaced with `_cross_file_include_block(fd)`: every non-`common.proto`
header still unconditionally gets `#include "messages/common.h"` first
(it defines the `Opt<T>` template every proto3 `optional` field expands
to, needed regardless of whether the `.proto` file itself imports
`common.proto` — `gripper.h`/`GripperCommand.angle` is the concrete case
that would otherwise regress), then one additional `#include` per OTHER
proto file named in the file's own `import` lines (`fd.dependency`),
skipping `options.proto` (declares no messages, no generated header) and
`common.proto` (already included). **Verified byte-identical for every
PRE-EXISTING header**: diffed `generate_headers()`'s in-memory output
against every checked-in `source/messages/*.h` — `common.h`,
`communicator.h`, `drivetrain.h`, `gripper.h`, `motion.h`, `motor.h`,
`odometer.h`, `planner.h`, `ports.h`, `sensors.h` are all UNCHANGED byte-
for-byte; only `envelope.h` changes, gaining exactly the four needed
lines (`#include "messages/drivetrain.h"`, `"messages/motion.h"`,
`"messages/odometer.h"`, `"messages/planner.h"`, sorted, after the
unconditional `common.h` include). **Ticket 005 does not need to redo
this** — the cross-file include mechanism is generic (derived from
`fd.dependency`, not hand-listed per file) and already handles any
further proto file that starts importing a second subsystem proto.

**Aggregator-header choice**: implemented as a new generated
`source/messages/layout_checks.h` (over inline-per-header asserts) because
the reachable-from-`CommandEnvelope`/`ReplyEnvelope` set spans SIX
generated headers (`envelope.h`, `motion.h`, `drivetrain.h`, `planner.h`,
`odometer.h`, `common.h`) and is a cross-cutting, whole-schema property
computed from the full `FileDescriptorSet` — emitting it inline would
require every per-file `_emit_file()` call to know a global fact it
doesn't otherwise need. `layout_checks.h` `#include`s `<type_traits>` +
`messages/envelope.h` (which transitively pulls in every other file this
check spans, via the cross-file include fix above) and asserts on every
reachable struct. A companion generated `source/messages/layout_checks.cpp`
(`#include "messages/layout_checks.h"`, zero runtime symbols) is the
translation-unit anchor that forces the asserts to actually be evaluated
as part of the normal build — without it nothing in `source/` yet
`#include`s `envelope.h`/`motion.h` (ticket 001's own completion notes:
"nothing in `source/` includes `envelope.h` yet"), so the header alone
would never be compiled. Wired into both builds: the ARM `CMakeLists.txt`
already recursively globs every `.cpp` under `source/` (`RECURSIVE_FIND_FILE`),
so no ARM-side edit was needed; `tests/_infra/sim/CMakeLists.txt` uses an
explicit, deliberately non-globbed source list (its own header comment:
"A file missing from this list surfaces as a LINK error... not a silent
omission"), so `"${SOURCE_DIR}/messages/layout_checks.cpp"` was added to
its `FIRMWARE_SOURCES` list explicitly.

**Reachable-struct set — computed generically, not hand-enumerated**:
added `_compute_layout_check_structs(fds)`, a BFS over the full
`FileDescriptorSet` from `CommandEnvelope`/`ReplyEnvelope` following every
message-typed field (including real-oneof union arms) to any depth. This
is a superset of the ticket's own stated floor ("at minimum... every
message in envelope.proto, motion.proto, drivetrain.proto's
DrivetrainCommand/SetPose/WheelTargets, and common.proto's shared
types") — it also correctly picks up `BodyTwist3`/`WheelTarget`/`Pose2D`
(the common.proto types actually reachable) and every `PlannerCommand`
goal variant (`VelocityGoal`/`GotoGoal`/`TurnGoal`/`DistanceGoal`/
`TimedGoal`/`RotationGoal`/`StreamGoal`/`StopCondition`), while correctly
EXCLUDING sibling messages in the same proto files that are NOT wire-
reachable from the envelope (`DrivetrainState`/`DrivetrainConfig`/
`DrivetrainCapabilities`, `PlannerState`/`PlannerConfig`,
`OdometerConfig`, and every other `common.proto` type not used
transitively — `BodyTwist`, `BodyAccel`, `ValueSet`, `PoseEstimate`,
`Gains`, `OutCommand`, `CommandBatch`, `Capabilities`).

**GATE OUTCOME: all 31 structs PASS `std::is_standard_layout` on both
toolchains — no fallback needed.** In BFS (first-seen) order:
`CommandEnvelope`, `ReplyEnvelope`, `DrivetrainCommand`, `MotionSegment`,
`PlannerCommand`, `ConfigDelta`, `SetPose`, `OdometerCommand`, `Ping`,
`Echo`, `ConfigGet`, `StreamControl`, `Stop`, `DeviceId`, `Ack`, `Error`,
`Telemetry`, `ConfigSnapshot`, `EventNotify`, `BodyTwist3`,
`WheelTargets`, `VelocityGoal`, `GotoGoal`, `TurnGoal`, `DistanceGoal`,
`TimedGoal`, `RotationGoal`, `StreamGoal`, `StopCondition`, `Pose2D`,
`WheelTarget`. `just build` (ARM, `arm-none-eabi-g++` 15.2.1, real
`codal-microbit-v2` target) compiled `source/messages/layout_checks.cpp`
with zero errors and linked `MICROBIT` cleanly (FLASH 83.67%, RAM 98.33% —
the latter is by-design per `.clasi/knowledge/codal-ram-always-near-full.md`,
not a regression signal). `just build-sim` (host `g++`/clang via CMake,
`HOST_BUILD=1`) likewise compiled `layout_checks.cpp.o` and linked
`libfirmware_host` cleanly. No struct required the ticket's fallback path
(005's unrolled per-message codec) — the generic `offsetof`-table walker
is viable for every struct in the current schema.

**`offsetof`-on-non-trivial-standard-layout C++11 nuance — recorded, plus
one verified correction to the architecture doc's framing**: every one
of the 31 structs above is standard-layout but NOT trivial (every
generated field carries a default member initializer, e.g.
`float x = 0.0f;`). Under strict C++11/C++14 wording, `offsetof` on a
standard-layout-but-non-trivial type is CONDITIONALLY-SUPPORTED, not
unconditionally guaranteed — that guarantee only becomes unconditional
from C++17 onward. This exact paragraph is carried verbatim into
`layout_checks.h`'s own generated header comment (per this ticket's own
instruction: "ticket 005's job to write that comment; this ticket's job
to surface the nuance so it isn't lost" — surfaced directly at the
generation site so it can't be lost even before 005 runs). **Correction,
verified by reading the actual build files rather than assuming the
architecture doc's "targets CODAL C++11" framing**: this project's ACTUAL
compiled standard is `-std=gnu++20` on both toolchains, not C++11 — root
`CMakeLists.txt` (~line 177) explicitly overrides the vendored
`codal-microbit-v2` `target.json`'s `-std=c++11` with an appended
`-std=gnu++20` ("GCC honors the LAST -std flag"), and
`tests/_infra/sim/CMakeLists.txt` sets `CMAKE_CXX_STANDARD 20` directly,
with both files' own comments noting the vendored target nominally still
pins C++11. Under C++17 (and therefore C++20), `offsetof` on a
standard-layout-but-non-trivial type IS unconditionally well-defined by
the standard text itself — so the conditionally-supported caveat, while
an accurate description of the vendored target's nominal C++11 pin, does
not actually apply to the code this project compiles today. This
correction is noted here (not silently substituted for the requested
paragraph, which is preserved as specified) so ticket 005 can decide
whether to carry the stricter "GCC/Clang define it in practice" framing
or the more precise "well-defined by the standard at this project's
actual `-std=gnu++20`" framing into its own generated comment.

**Struct-shape verification**: confirmed via the same in-memory diff used
for the cross-file-include check above — no generated struct's field
list, oneof/union shape, `Opt<T>` usage, or setter/accessor emission
changed; the only content changes anywhere are `envelope.h`'s four added
`#include` lines and the two wholly-new `layout_checks.{h,cpp}` files.

**Verification performed** (per the ticket's own verification command):
`python scripts/gen_messages.py --dry-run` → clean. Real
`python scripts/gen_messages.py` → wrote `layout_checks.h`/
`layout_checks.cpp` (new) and `envelope.h` (four added includes); every
other header byte-identical. `just build` → ARM build green (see gate
outcome above). `just build-sim` → green. `uv run python -m pytest
tests/sim -q` → **58 passed in 61.97s**, matching the 001-established
baseline exactly (zero test behavior change, as expected — this ticket
adds only compile-time asserts). `uv run python -m pytest tests/unit -q`
→ 12 passed (includes the pre-existing `test_gen_messages_no_getters.py`
getter-regression guard, unaffected by `layout_checks.{h,cpp}`'s content
since it defines no `get_*`-prefixed methods).

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
