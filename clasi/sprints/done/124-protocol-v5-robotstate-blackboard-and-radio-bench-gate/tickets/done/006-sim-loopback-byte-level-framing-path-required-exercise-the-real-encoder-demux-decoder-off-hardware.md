---
id: '006'
title: 'Sim/loopback byte-level framing path (REQUIRED): exercise the real encoder/demux/decoder
  off-hardware'
status: done
use-cases:
- SUC-003
depends-on:
- '005'
github-issue: ''
issue:
- telemetry-physical-layer-corruption-and-move-ack-observability.md
- protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim/loopback byte-level framing path (REQUIRED): exercise the real encoder/demux/decoder off-hardware

## Description

**REQUIRED, non-negotiable** — the second of the stakeholder's two
structural fixes. Sim did not catch either of 123's hardware-only wire
bugs (the 0x0A corruption, the move-enqueue-ack gap) because
`sim_ctypes` bypasses the real encoder/demux/decoder entirely and passes
envelopes directly. Build a test path that serializes a command/reply
through the REAL wire codec (the same one ticket 005 just cut over) and
feeds the resulting bytes back through the REAL demux/decoder, entirely
off-hardware.

This is additive to, not a replacement for, `sim_ctypes`'s existing
envelope-passing tests — those stay as they are; this ticket adds the
byte-level path they don't cover.

## Acceptance Criteria

- [x] The sim/loopback path is distinct from and additional to
      `sim_ctypes`'s existing envelope tests — it does not replace them.
- [x] It actually invokes the byte-level codec (encode → COBS → decode),
      not a stub or shortcut around it — verify by reading the
      implementation, not just its test names.
- [x] A deliberately-reintroduced `0x0A`-in-binary-frame bug (make
      manually during review: skip the delimiter XOR) fails this test.
- [x] Runs in CI without hardware present.

## Testing

- **Existing tests to run**: existing `sim_ctypes`/`sim_loop.py` tests
  (must remain green and unaffected).
- **New tests to write**: the loopback harness itself (C++ and/or
  Python) — this ticket's entire deliverable — plus the deliberate-bug
  sanity check above (run once during review, not left in the suite).
- **Verification command**: `uv run pytest` plus the C++ sim-tests build,
  with no hardware connected.

## Completion Notes

**New file**: `src/tests/sim/system/test_sim_wire_loopback.py`
(`test_move_wheels_with_embedded_0x0a_byte_round_trips_through_real_codec`,
`test_stop_command_round_trips_through_real_codec_without_configuration`).

**Where `sim_ctypes`'s bypass actually is.** It is NOT
`SimLoop.move()`/`inject_command()` — those already route a real
`pb2.CommandEnvelope.SerializeToString()` payload through the real
production `robot_radio.io.wire_codec.encode_frame()` (COBS
delimiter-0x0A + CRC-16/CCITT-FALSE), push the resulting bytes onto
`TestSupport::FakeTransport` via `sim_inject_command()`, and read replies
back via `robot_radio.io.wire_codec.decode_frame()` +
`pb2.ReplyEnvelope.FromString()` (`SimLoop._decode_reply_frame()`) — a
real byte-level round trip on both sides already, just never exercised by
a dedicated round-trip-proving test before this ticket. The actual
bypass is the two SCALAR-argument exports, `sim_inject_twist()`/
`sim_inject_stop()` (`src/sim/sim_ctypes.cpp`): their Python callers
(`SimLoop.twist()`/`.stop()`) pass bare `float`/`duration`/`corr_id`
values across the ctypes boundary — no `bytes` object, no
`wire_codec.py` call, anywhere in that path. (Internally, on the C++
side, `SimHarness::injectMove()` does build a COBS+CRC frame via
`TestSupport::armorMoveCommand()` before pushing it onto the fake
transport — so the FIRMWARE side of `twist()`/`stop()` is not a raw
struct-copy bypass — but the HOST side never touches the real Python
wire codec at all for those two calls, which is the half this ticket's
issue was written against.)

**How the new path differs.** This ticket adds a test that (a) builds
the `CommandEnvelope`/`encode_frame()` call explicitly in the test file
itself (not hidden behind `SimLoop.move()`), so the byte-level codec
call is directly visible to a reviewer without tracing through
`sim_loop.py`; (b) computes (not hand-picks) a wheel-velocity value whose
IEEE-754 float32 encoding embeds a literal `0x0A` byte — the exact
123-006 hazard shape (a `move_wheels` envelope whose own serialized
bytes contain the frame delimiter) — and asserts the raw pre-COBS
payload genuinely contains `0x0A` while the COBS-encoded wire frame does
not; (c) pushes the frame through the REAL compiled firmware
(`TestSim::SimHarness` → real `App::Comms::pump()`/`decodeBinaryFrame()`
→ real generated `msg::wire::decode()` → real dispatch) and reads the
firmware's live PID setpoint back via `sim_cmd_vel_left()`/
`sim_cmd_vel_right()`, confirming the exact embedded-0x0A float decoded
bit-correctly, not merely "some ack arrived"; (d) confirms real encoder
motion via a second real-firmware-encode → real-host-decode round trip
(`SimLoop.drain_pending_tlm()` → `_decode_reply_frame()` →
`decode_frame()` → `pb2.ReplyEnvelope.FromString()`); (e) adds a second,
STOP-command scenario proving the loopback is general (not MOVE-specific)
and works with no `configure_from_robot()` call at all.

**Deliberate-break proof (run twice independently — by the implementing
agent and, separately, by the team-lead — same result both times).**
Edited `WireRuntime::cobsDecode()` in `src/firm/messages/wire_runtime.cpp`
to stop XOR-ing against `delimiter` (effectively reverting the firmware's
DECODE side to 0x00-keying while the host still emits 0x0A-keyed
frames — the exact 123-006 bug class: a real, correctly-produced host
frame that the firmware silently fails to demux). Rebuilt via
`just build-sim`, reran `test_sim_wire_loopback.py`:

```
FAILED test_move_wheels_with_embedded_0x0a_byte_round_trips_through_real_codec
AssertionError: no ack for corr_id=40001 observed within 25 cycles -- the real
firmware demux/decode never dispatched our real-host-encoded MOVE

FAILED test_stop_command_round_trips_through_real_codec_without_configuration
AssertionError: no ack for corr_id=40002 observed within 25 cycles
```

Both scenarios failed outright (no ack ever observed within the
25-cycle poll budget) — the broken firmware decoder silently drops
every correctly-produced host frame rather than corrupting a value, so
the assertion that fails is "an ack arrived at all," not a value
mismatch. Reverted the edit (`git diff` on the file came back empty),
rebuilt, reran: 2 passed. The team-lead's independent repeat of this
same check used a `sed`/`.bak`-file revert and hit a real gotcha worth
recording for future tickets: restoring the file via `mv` from a `.bak`
copy gave it an OLDER mtime than the already-compiled `.o`, so
`just build-sim` reported a clean "Built target firmware_host" without
actually recompiling — the tests still failed after what was, on disk,
a correct revert. `touch`ing the source before rebuilding forced the
recompile and the suite went green. This is the
`stale-incremental-build-on-volumes` gotcha (`.clasi/knowledge/` /
project memory) surfacing in a new place: a revert that silently
doesn't trigger a rebuild is indistinguishable from a broken revert
until you notice the build log said "Built" instead of showing any
compile line.

**Verification run (this session):**
- `just build` — clean (firmware `.hex` + host sim lib both built,
  `v0.20260724.2`).
- `just build-sim` — clean, `Built target firmware_host`, both before and
  after the deliberate-break round trip.
- `uv run python -m pytest src/tests/sim/system/test_sim_wire_loopback.py -v`
  — 2 passed, both scenarios (re-confirmed after the revert).
- Full suite `uv run python -m pytest` — **1476 passed, 2 skipped,
  9 xfailed, 2 xpassed in 482.59s** — run independently by both the
  implementing agent and the team-lead, identical pass/skip/xfail/xpass
  counts both times. No sign of the order-dependent
  `test_config_gate_harness_compiles_and_passes` flake ticket 004 hit.

**Hardware**: not connected in this environment (`pyocd list` /
`/dev/cu.usbmodem*` both empty) — every acceptance criterion above is
satisfiable, and was satisfied, entirely off-hardware; no criterion was
skipped or deferred.
