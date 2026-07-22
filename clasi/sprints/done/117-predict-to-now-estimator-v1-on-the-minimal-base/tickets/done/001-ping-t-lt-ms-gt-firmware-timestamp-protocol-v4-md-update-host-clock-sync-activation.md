---
id: '001'
title: PING reply timestamp (t=ms) + protocol-v4.md update + host clock-sync activation
status: done
use-cases:
- SUC-056
depends-on: []
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PING reply timestamp (t=ms) + protocol-v4.md update + host clock-sync activation

## Description

`Comms::pumpTransport()`'s `PING` handler (`comms.cpp:64-67`) replies a bare
`"OK pong"`. The set-point issue and this project's own `clock_sync.py`
docstring both describe (and `_parse_pong_t()` already tolerantly parses)
`"OK pong t=<ms>"` — a robot-clock timestamp that activates the host's
existing, complete, unit-tested `ClockSync` (min-RTT offset + skew fit).
`docs/protocol-v4.md` §2.4 currently documents this exact gap as an
"AS-BUILT divergence." This ticket closes it: append the firmware's own
clock time (ms) to the `PING` reply, update the protocol doc to match what
ships, and prove host-side that `ClockSync` actually converges against the
live (or simulated) firmware's new reply — the necessary precondition for
any future external-pose (camera) fusion work, even though that fusion
itself is out of this sprint's scope.

This is the sprint's small, independent opener — no dependency on
`App::StateEstimator` (tickets 002+), which does not consume `ClockSync` or
external timestamps this sprint at all (it only reads on-robot,
already-robot-clock-stamped encoder/OTOS readings). Also updates
`src/host/robot_radio/DESIGN.md` directly (this sprint's overlay only has
one `DESIGN.md` slot, already claimed by `src/firm/app/DESIGN.md` — see
`sprint.md`'s own Design Overlay section).

## Acceptance Criteria

- [x] `Comms::pumpTransport()`'s `PING` handler replies `OK pong t=<ms>`,
      where `<ms>` is the firmware's own current clock time (integer
      formatting only — `newlib-nano` has no `printf` float support, but
      this is not a float field, so this is a non-issue, not a workaround).
      Verified on both the serial and radio-relay transports.
- [x] `docs/protocol-v4.md` §2.4's "AS-BUILT divergence from the set-point
      issue" callout is removed/updated: the documented `PING` reply now
      matches the shipped reply exactly.
- [x] A host-side activation test (sim-first; bench if a live robot is
      reachable) drives a real `ClockSync` instance's `ping_burst()` against
      the firmware's new reply and asserts `best_offset()` is non-`None`
      after one burst — i.e. `ClockSync` is proven to actually activate
      against this firmware, not just parse a hand-written fixture string.
- [x] A sim/unit test (extending the existing `app_comms_harness.cpp`/
      `test_app_comms.py` pair) asserts the text-plane `PING` reply contains
      `t=` followed by the `now` value passed into `Comms::pump()`/
      `pumpTransport()` for that call.
- [x] `src/host/robot_radio/DESIGN.md` updated in place with a short note
      that `PING` now carries `t=<ms>` and that `ClockSync` is therefore
      live-activatable (this doc does not ride the sprint's design
      overlay — same co-located-`DESIGN.md`-slot collision noted in
      `sprint.md`'s Design Overlay section — so it is edited directly on
      the canonical doc here).

## Implementation Plan

**Approach.** `Comms` currently has no notion of "now" — `pumpTransport()`
only sees the transport and the outbound `Cmd`. `RobotLoop::cycle()`
already computes `cycleStart`/`markTime()` at the top of the cycle and
calls `comms_.pump(cmd)` from inside the `kSettle` `runAndWait` block; the
cleanest, smallest-surface change is threading that same already-computed
time through: `Comms::pump(Cmd& out, uint32_t now)` →
`pumpTransport(Transport&, Cmd&, uint32_t now)`, formatting the reply with
`std::snprintf(buf, sizeof(buf), "OK pong t=%lu", static_cast<unsigned
long>(now))` (or equivalent fixed-width formatting) instead of the current
literal `"OK pong"` string constant. This keeps `Comms` itself
`Devices::Clock`-free (no new collaborator, no bus/timing risk) — it is
handed a value, not given a way to ask for one, matching this codebase's
existing "single-loop bus/timing ownership" invariant (only `RobotLoop`
decides "now").

**Files to modify:**
- `src/firm/app/comms.h` / `comms.cpp` — `pump()`/`pumpTransport()` gain a
  `uint32_t now` parameter; the `PING` branch formats `"OK pong t=%lu"`
  instead of the literal `"OK pong"`.
- `src/firm/app/robot_loop.cpp` — the one call site (`comms_.pump(cmd)`
  inside the `kSettle` block) passes `cycleStart` (already computed at the
  top of `cycle()`) as the new argument.
- `docs/protocol-v4.md` — §2.4's `PING` row and the "AS-BUILT divergence"
  paragraph updated to match the shipped `t=<ms>` reply; remove the
  divergence callout entirely (no longer a divergence).
- `src/host/robot_radio/robot/protocol.py` — no NEW method is strictly
  required (`NezhaProtocol.send(cmd, read_timeout)` already round-trips a
  raw text command); add a small, explicit helper if the activation test's
  own plumbing needs one to adapt `send()`'s dict-shaped return into the
  raw-reply-line string `ClockSync.ping_burst(send_fn)` expects — implementer's
  call, document whichever shape is chosen.
- `src/host/robot_radio/DESIGN.md` — direct edit per the Acceptance
  Criteria above.
- `src/tests/sim/unit/app_comms_harness.cpp` / `test_app_comms.py` — extend
  with the `now`-parameter PING-reply-shape test.
- A new sim or bench activation test exercising `ClockSync` end-to-end
  (exact location — `src/tests/sim/system/` vs. `src/tests/bench/` — is the
  implementer's call, guided by whether `SimLoop` exposes a text-plane PING
  passthrough; prefer sim if it does, per this project's sim-first
  convention).

**Documentation updates:** `docs/protocol-v4.md` §2.4, `src/host/robot_radio/DESIGN.md`.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_app_comms.py`, full
  `uv run python -m pytest` suite (confirm no regression from the
  `pump()`/`pumpTransport()` signature change — check every existing call
  site, including `src/sim/sim_harness.h` and any other host-build harness
  calling `Comms::pump()` directly).
- **New tests to write**: PING-reply-carries-`t=now` sim/unit test;
  host-side `ClockSync` activation test (sim or bench).
- **Verification command**: `uv run python -m pytest src/tests/sim/unit/test_app_comms.py`;
  full suite `uv run python -m pytest`.
