---
id: '006'
title: Differential/fuzz/range test harness vs google.protobuf
status: open
use-cases: [SUC-005]
depends-on: ['002', '004', '005']
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

- [ ] Differential round-trip passes in BOTH directions for every
      implemented oneof arm: `drive` (each `DrivetrainCommand` oneof
      variant this sprint's `BinaryChannel` will use — at minimum
      `wheels`/`neutral`), `segment`, `replace` (both `MotionSegment`
      shapes), `stop`, `ping`, `echo`, `id`.
- [ ] The fuzz corpus (>= 200 cases: random, truncated-at-every-byte,
      oversized, unknown-field-salted) produces ZERO crashes and ZERO
      out-of-bounds reads, verified under ASan/UBSan on the host build.
- [ ] Every validated field's boundary case (`min-1, min, max, max+1,
      abs_max, -abs_max` as applicable) produces the expected accept/
      reject verdict with the correct `{fieldNumber, ErrCode}` on
      rejection.
- [ ] The full pre-existing sim suite (58 tests, `sprint.md`'s stated
      baseline) stays green alongside this new differential suite in the
      same run.
- [ ] The differential suite is documented (a short header comment in the
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
