---
id: '006'
title: Differential/fuzz/range test harness vs google.protobuf
status: done
use-cases:
- SUC-005
depends-on:
- '002'
- '004'
- '005'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Differential/fuzz/range test harness vs google.protobuf

## Description

Prove the self-written firmware codec (tickets 004+005) agrees with the
host's `google.protobuf`-backed reference (ticket 002's `pb2/` bindings)
byte-for-byte, in both directions, and rejects malformed/boundary input
cleanly — this is the correctness backbone for the whole protocol-v3
program (the issue's #1 ranked risk) and the gate `BinaryChannel`
(ticket 007) is built on top of, not an afterthought.

1. **Host-compiled C++ harness** (`tests/sim/unit/wire_differential_harness.cpp`
   or reuse/extend ticket 004/005's harness if that's cleaner — team's
   call), following `runtime_blackboard_harness.cpp`'s established
   pattern exactly: links `wire_runtime.cpp` + `wire.cpp`, exposes simple
   encode/decode entry points (e.g. read a hex/base64-encoded buffer from
   stdin or argv, decode it, print the decoded fields in a
   machine-parseable format; or the reverse — read field values, encode,
   print bytes) that a Python driver can drive via `subprocess`.
2. **Differential round-trip pytest suite**
   (`tests/sim/unit/test_wire_differential.py`): for every implemented
   oneof arm (drive/segment/replace/stop/ping/echo/id), generate matched
   test inputs and feed them through BOTH codecs in BOTH directions:
   - host-encode (`pb2`) -> firmware-decode (harness) -> assert decoded
     fields match the original input.
   - firmware-encode (harness) -> host-decode (`pb2.ParseFromString`) ->
     assert decoded fields match the original input.
3. **Fuzz corpus** (>= 200 generated cases): random byte strings,
   truncated valid messages (chop a valid encoding at every byte
   boundary), oversized messages (valid encoding plus trailing garbage),
   and unknown-field-salted messages (a valid encoding with an extra,
   unrecognized field spliced in). Feed each to the firmware decoder;
   assert it NEVER crashes and NEVER reads out of bounds (run this
   specific sub-suite under ASan/UBSan on the host build) and always
   returns a clean `Result{ok=false, ...}` or, for the unknown-field
   case, `ok=true` with the unknown field correctly skipped.
4. **Boundary/range corpus**: for every `(min)`/`(max)`/`(abs_max)`/
   `(req)`-validated field in this sprint's implemented arms (per ticket
   001's transcribed bounds), construct inputs at `min-1, min, max,
   max+1, abs_max, -abs_max` (as applicable to that field) and assert the
   expected accept/reject verdict, including the correct `{fieldNumber,
   ErrCode}` on rejection.
5. **Regression guard**: run the full pre-existing sim suite
   (`tests/sim -q`, 58-test baseline per `sprint.md`) alongside this new
   suite in the same CI/verification pass, confirming it stays green.

## Acceptance Criteria

- [x] Differential round-trip passes in BOTH directions for every
      implemented oneof arm: `drive` (each `DrivetrainCommand` oneof
      variant this sprint's `BinaryChannel` will use — at minimum
      `wheels`/`neutral`), `segment`, `replace` (both `MotionSegment`
      shapes), `stop`, `ping`, `echo`, `id`.
- [x] The fuzz corpus (>= 200 cases: random, truncated-at-every-byte,
      oversized, unknown-field-salted) produces ZERO crashes and ZERO
      out-of-bounds reads, verified under ASan/UBSan on the host build.
- [x] Every validated field's boundary case (`min-1, min, max, max+1,
      abs_max, -abs_max` as applicable) produces the expected accept/
      reject verdict with the correct `{fieldNumber, ErrCode}` on
      rejection.
- [x] The full pre-existing sim suite (58 tests, `sprint.md`'s stated
      baseline) stays green alongside this new differential suite in the
      same run.
- [x] The differential suite is documented (a short header comment in the
      test file) as the correctness gate `BinaryChannel` (ticket 007)
      depends on — a future change to `wire_runtime.{h,cpp}` or the
      generated `wire.{h,cpp}` that breaks this suite must be treated as
      a blocking regression, not a suite to be casually skipped/xfailed.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim -q` (full
  58-test baseline).
- **New tests to write**: this ticket's entire scope IS the new test
  suite — `tests/sim/unit/wire_differential_harness.cpp` +
  `tests/sim/unit/test_wire_differential.py` (differential + boundary
  cases), plus a fuzz-specific test/script (may be a separate file if a
  fuzz corpus generator warrants its own module, e.g.
  `test_wire_fuzz.py`), including an ASan/UBSan-built invocation path for
  the fuzz sub-suite specifically (document the exact compiler flags used
  so it's reproducible, e.g. `-fsanitize=address,undefined`).
- **Verification command**: `uv run python -m pytest
  tests/sim/unit/test_wire_differential.py tests/sim/unit/test_wire_fuzz.py
  -q` plus the full `uv run python -m pytest tests/sim -q`.

## Completion Notes (2026-07-10)

**Files**: `tests/sim/unit/wire_differential_harness.cpp` (new,
host-compiled, argv-driven CLI: `decode <base64>` /
`encode_ok|encode_err|encode_id <args...>` — one shot per invocation,
subprocess-driven by Python rather than a fixed C++ scenario list, per the
ticket's own suggestion), `tests/sim/unit/_wire_diff_driver.py` (new, shared
non-test helper module: compile/run/parse plumbing, pb2 envelope builders,
float32 canonicalization, raw-varint splicing for the fuzz suite — imported
by both test files), `tests/sim/unit/test_wire_differential.py` (new,
differential round-trip both directions for every implemented arm + the
boundary/range corpus + a field-number-correspondence check),
`tests/sim/unit/test_wire_fuzz.py` (new, >=200-case fuzz corpus run under
ASan/UBSan). No `CMakeLists.txt` change needed — this harness is compiled
standalone by the Python test driver (same pattern as
`wire_codec_harness.cpp`/`wire_runtime_harness.cpp`, tickets 004/005), never
linked into the sim/ARM builds.

**Two real codec bugs found and fixed** (both in the GENERATOR,
`scripts/gen_messages.py`, since `wire.cpp` is 100% generated output —
regenerated via `python scripts/gen_messages.py`, isolated diff confirmed
against `git diff -- source/messages/wire.cpp` after each fix, zero other
generated header touched):

1. **`decode()`'s reset (`out = CommandEnvelope{}`) did not zero the whole
   object.** Per the C++ aggregate-init rules for a union data member, `=
   {}` value-initializes only the union's FIRST named alternative at every
   level (`cmd.drive`, and recursively `control.twist` inside it); any
   union alternative that is NOT first and is LARGER than the first
   alternative (e.g. `cmd.drive.control.wheels`, a `WheelTargets`, is far
   larger than `control`'s first alternative `BodyTwist3`) had its extra
   bytes left INDETERMINATE. `decodeInto()`'s repeated-message clamp read
   that indeterminate byte as the field's STARTING element count before
   ever writing to it. Reproduced directly: decoding a valid 2-element
   `drive.wheels` envelope came back with `w_count` values of 4 (some runs)
   or 34 (with `WheelTarget w_[4]`, i.e. an out-of-bounds array index) in a
   stack-polluted repro — a genuine uninitialized-memory read, not a
   cosmetic stale-value bug. **Fix**: `std::memset(static_cast<void*>(&out),
   0, sizeof(out))` (cast to `void*` to state intent past GCC's
   `-Wclass-memaccess`, which fires because `CommandEnvelope` is
   standard-layout + trivially copyable but not trivial — ticket 003's own
   day-one gate already established both properties, so a full-object
   memset is well-defined and safe, the same "zero it all, unconditionally"
   idiom nanopb/protobuf-c use for the identical shape).
2. **`encode()`'s string-length scan read one byte too far.**
   `decodeInto()`'s own `kString` case documents "`fd->cap` is the array's
   FULL capacity, including room for the null terminator this decoder
   ALWAYS writes" and enforces content `<= cap-1`; `encodeInto()`'s
   `kString` case scanned for the terminating `'\0'` across the FULL `[0,
   cap)` range instead of `[0, cap-1)`. For a MAX-LENGTH string (content ==
   `cap-1` bytes, e.g. a 47-char `DeviceId.model` at `str_len=48`) there is
   no guaranteed `'\0'` anywhere in `[0, cap)`, so the scan fell through to
   the reserved last byte and used ITS (potentially uninitialized, per bug
   1's same root cause — a hand-constructed `ReplyEnvelope` is never
   round-tripped through `decode()`, since `decode()` is
   `CommandEnvelope`-only per Decision 4) value, encoding a 48-byte string
   instead of 47 and producing bytes `pb2.ParseFromString()` rejected
   outright (`DecodeError`). Reproduced directly via the differential
   suite's `test_direction_b_device_id[max-length]` case. **Fix**: bound
   the scan to `static_cast<size_t>(fd.cap) - 1` (also fixes a
   `-Wsign-compare` GCC warning the unbounded version had).

Both fixes verified: (a) isolated standalone repro (`/tmp/repro2.cpp`,
scratch-only, not committed) directly confirmed bug 1's before/after
behavior with intentional stack pollution; (b) the differential suite's own
`test_direction_a_drive_wheels`/`test_direction_b_device_id` cases now pass
cleanly against the fixed generator output; (c) `just build` (ARM,
`arm-none-eabi-g++`) and `just build-sim` both clean, ZERO warnings
(confirmed both `-Wclass-memaccess` and `-Wsign-compare` are gone post-fix);
FLASH 83.67% / RAM 98.33% — unchanged from ticket 005's own reported
numbers (this code is still dead-code-stripped from the ARM image; nothing
in `source/` calls `msg::wire::decode`/`encode` yet — ticket 007 gives it a
live caller). No other generated header changed (`git diff --stat --
source/messages/` shows only `wire.cpp`).

**Differential coverage (Direction A: pb2-encode -> harness-decode)**:
`drive.twist` (+ `seed`/`standby` Opt fields), `drive.wheels` (4 elements,
mixed `speed`/`position` presence), `drive.neutral` (both `BRAKE`/`COAST`),
`segment` and `replace` (both `MotionSegment` "shapes" — the geometry-only
MOVE shape and the time/v/omega MOVER-teleop shape), `stop`, `ping`, `echo`
(empty/single-NUL/ASCII/max-64-byte payloads), `id` (empty request).
**Direction B (harness-encode -> pb2-decode)**: `ok`/`Ack` (3 q/rem
combinations incl. max `uint32`), `err`/`Error` (all 8 `ErrCode` values),
`id`/`DeviceId` (empty, typical, and max-length-47-char strings — the case
that caught bug 2). A dedicated `test_field_numbers_match_pb2_descriptors`
cross-checks `CommandEnvelope.cmd`/`ReplyEnvelope.body`/`MotionSegment`
field numbers and `ErrCode` enum values against the live `pb2` descriptors.
**Boundary/range corpus**: all 11 `(min)`/`(max)`/`(abs_max)`-validated
`MotionSegment` fields (the only validated fields in this sprint's
implemented arms — `ConfigGet.target`'s `(req)` belongs to the declared-only
`get` arm, out of scope, already covered by ticket 005's own harness), 4
cases each (`min-1`/`min`/`max`/`max+1` or `-abs_max-1`/`-abs_max`/
`abs_max`/`abs_max+1`), run through BOTH `segment` and `replace` arms — 96
boundary assertions total, each rejection checked for the exact
`{fieldNumber, ErrCode.ERR_RANGE}`.

**Fuzz corpus**: 252 generated cases (random: 60; truncated-at-every-byte
across 4 representative valid messages: ~144; oversized (+1/+5/+20/+64B
trailing garbage) across 4 messages: 16; unknown-field-salted
(prepend/append/both, field 99) across 4 messages: 12) — all run under
`-fsanitize=address,undefined -fno-omit-frame-pointer -g` (matching tickets
004/005's own ASan/UBSan invocation). **252/252 pass, ZERO crashes, ZERO
ASan/UBSan findings.** Salted cases additionally assert `decode()` returns
`OK` with `corr_id`/`cmd_kind` intact (unknown field correctly skipped, not
merely "didn't crash").

**Test summaries**:
- `uv run python -m pytest tests/sim/unit/test_wire_differential.py
  tests/sim/unit/test_wire_fuzz.py -q` → **379 passed** (126 differential/
  boundary + 253 fuzz, including the corpus-size assertion).
- `uv run python -m pytest tests/sim -q` → **441 passed** (the
  005-established 62-test baseline + this ticket's 379 new tests, zero
  regressions).
- `just build` (ARM) → green, zero warnings, FLASH 83.67%/RAM 98.33%
  (unchanged). `just build-sim` → green.

No unresolved differential disagreement remains — both real disagreements
found were fixed at the generator (not xfailed), regenerated, and
reverified clean.
