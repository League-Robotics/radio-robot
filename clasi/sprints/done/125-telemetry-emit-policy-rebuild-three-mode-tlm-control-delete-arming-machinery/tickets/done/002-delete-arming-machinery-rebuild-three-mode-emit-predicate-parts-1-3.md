---
id: '002'
title: Delete arming machinery; rebuild three-mode emit predicate (Parts 1, 3)
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: telemetry-emit-policy-rebuild-spec.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete arming machinery; rebuild three-mode emit predicate (Parts 1, 3)

## Description

The core of the rebuild. Delete the entire report-on-change arming
lifecycle from `App::Telemetry` and `App::RobotLoop`, and replace
`hasSomethingToSay()`'s four-arm predicate with the three-mode
(`TlmMode{kOff, kAuto, kOn}`) predicate specified normatively in the
issue's Part 3. This ticket starts by removing the UNCOMMITTED
`tickBootSettle()`/`kBootStableCycles`/`bootSettling_` work already
sitting in the working tree (`src/firm/app/telemetry.{h,cpp}` are
currently dirty) — that work is superseded, not preserved, by this
ticket.

**Delete exactly** (issue Part 1, `telemetry.h`/`telemetry.cpp`):
- `kBootStableCycles` and the whole "wait for the flags word to hold
  still" mechanism.
- `markBootComplete()` (declaration + definition).
- `tickBootSettle()` (declaration + definition) and its call site in
  `RobotLoop::cycle()` (currently immediately after `tlm_.update(state_)`).
- Members `changeReportingArmed_`, `bootSettling_`, `stableCycles_`,
  `lastSeenFlags_`.
- Member `lastEmittedFlags_`, its `0xFFFFFFFFu` seed, the now-false "first
  frame always sends" comment, and its write at the tail of
  `emitPrimary()`.
- `hasSomethingToSay()` arm 4 (report-on-change:
  `changeReportingArmed_ && flags_ != lastEmittedFlags_`).
- The `changeReportingArmed_` gate on arm 2 (coast-down/wheel-velocity)
  — arm 2 itself is replaced by the activity hold-off below, not deleted
  outright.
- `kFlagEventBootReady` (bit 11) — the constant, its
  `setLiveFlag(kFlagEventBootReady, true)` call at `RobotLoop::boot()`'s
  tail, and its bullet in the flags layout comment and in
  `setLiveFlag()`'s doc comment. Bit 11 becomes RESERVED (same treatment
  as bit 5).

**Delete exactly** (issue Part 1, `robot_loop.cpp`):
- The stale "Arm report-on-change now" comment block above the
  `markBootComplete()` call site (the call itself is already covered
  above). Replacement boot() tail: `sendBanner()`, `sendReady()`,
  nothing else.
- The factually wrong DTR claim in boot()'s "NOT forced" comment
  ("opening a serial port asserts DTR and resets the board") — the board
  does NOT reset on port open or DTR pulse
  (`.clasi/knowledge/serial-monitor-never-shows-the-banner`). Keep the
  un-forced emit and its real rationale (no per-probe frame flood on
  power-on); delete only the invented DTR justification.

**Delete the boot-reenactment boilerplate** from
`src/tests/sim/unit/app_telemetry_harness.cpp`
(`markBootComplete()` + `2 * kBootStableCycles` warm-up ticks) — a
freshly constructed `Telemetry` now behaves identically to one on a
robot that has been up for an hour, so no warm-up is needed or
permitted (see ticket 006's criterion 12, "no test may re-add a
Telemetry lifecycle call" — that check is written there, but this
ticket must not leave anything for it to catch).

**Explicitly KEEP unchanged** (do not "clean up"): the single-assembly
`update(const Types::RobotState&)` projection, `setLiveFlag()`'s two-bit
escape hatch (minus the boot-ready bullet), the bounded ack ring/
`ackSends_[]`/`kAckRepeats`, `primaryDue()`/`kPrimaryPeriod`/
`everEmittedPrimary_`, the bare-`TLM` request path
(`Comms::takeTelemetryRequest()` → `force` — ticket 003 extends this, do
not touch it here), `READY`/`STATUS`/`HELP` verbs and the banner→`READY`
ordering, and the unforced `emit()` call inside the boot loop.

**Build** (issue Part 3, normative — implement exactly this):

```cpp
// telemetry.h
enum class TlmMode : uint8_t { kOff, kAuto, kOn };
TlmMode mode_ = TlmMode::kAuto;  // reset by construction each boot; one writer
```

```cpp
// telemetry.cpp -- the whole policy. due = cadence gate, unchanged.
const bool activity = (flags_ & kFlagActive) ||
                      (everMoved_ && (now - lastActivity_) < kCoastHoldoff);
bool unsolicited = false;
switch (mode_) {
  case TlmMode::kOff:  unsolicited = false;    break;
  case TlmMode::kAuto: unsolicited = activity; break;
  case TlmMode::kOn:   unsolicited = true;     break;
}
if (primaryDue(now) && (force || unsolicited || pendingAckDeliveries())) emitPrimary(now);
```

New state (issue Part 3, "New state replacing old arms 1+2"):
- `uint32_t lastActivity_` `// [ms]` — refreshed to `now` whenever
  `kFlagActive` is set, and whenever (`everMoved_` AND either wheel's
  staged velocity is nonzero AND the window is currently open). Coasting
  wheels keep an already-open window alive; wheels alone never OPEN a
  closed window.
- `bool everMoved_` — set true the first time `kFlagActive` is seen this
  power cycle, never cleared. Depends on ticket 001's `Motor::velocity()`
  fix for its own correctness (belt-and-suspenders, not a substitute).
- `constexpr uint32_t kCoastHoldoff = 2000;  // [ms]` — the ONE tunable,
  replacing `kBootStableCycles` and the whole arming machine.
- `pendingAckDeliveries()` — rename the existing arm-3 loop
  (`ackSends_[i] < kAckRepeats` over the ring) into this named private
  helper; behavior unchanged, honored in EVERY mode, never gated.

A public `setMode(TlmMode)` (or equivalent) is needed for ticket 003 to
call from `RobotLoop::cycle()` — add it here since it is part of the
class's new public surface, even though its caller lands in ticket 003.

## Acceptance Criteria

- [x] Every symbol in the "Delete exactly" lists above is gone from
      `src/`, with no `#if 0`/comment-out — verified by grep returning
      no hits for `kBootStableCycles`, `markBootComplete`,
      `tickBootSettle`, `bootSettling_`, `stableCycles_`,
      `lastSeenFlags_`, `lastEmittedFlags_`, `changeReportingArmed_`,
      `kFlagEventBootReady`.
  - [x] Bit 11 is RESERVED (same comment treatment as bit 5) in
      `telemetry.h`'s flags layout comment; no code sets it.
- [x] `TlmMode` enum, `mode_` member (defaults `kAuto`), `everMoved_`,
      `lastActivity_`, `kCoastHoldoff = 2000`, and
      `pendingAckDeliveries()` all exist exactly as specified above.
- [x] The emit predicate in `emit()` matches the issue's normative
      snippet exactly (three-way switch on mode, `force` /
      `unsolicited` / `pendingAckDeliveries()` as the only three
      reasons — no fourth).
- [x] `App::Telemetry` has NO arming state: a freshly constructed
      instance behaves identically to one that has run for an hour
      (verify via a sim test that constructs fresh and immediately
      checks silent-at-rest behavior, no warm-up ticks needed).
- [x] `app_telemetry_harness.cpp`'s boot-reenactment boilerplate
      (`markBootComplete()` + `2 * kBootStableCycles` ticks) is deleted;
      any existing test that relied on it is rewritten to construct
      fresh and proceed directly.
- [x] `RobotLoop::boot()`'s tail is exactly `sendBanner()`,
      `sendReady()`, nothing else between them.
- [x] The DTR-reset claim is gone from `robot_loop.cpp`'s "NOT forced"
      comment; the real rationale (no per-probe frame flood) is kept.
- [x] Existing bench-gate PASS lines (`radio_bench_gate.py`,
      `twist_drive.py`) are not run by this ticket but must not be
      structurally broken by it — the bare-`TLM` path and ack ring are
      untouched.

## Testing

- **Existing tests to run**: the full `app_telemetry_harness.cpp` and
  `app_robot_loop_harness.cpp` suites (whatever HOST_BUILD sim target
  runs them) — expect several existing scenarios written against the old
  four-arm predicate to need updating to the new mode-based one; update
  them in place rather than deleting coverage.
- **New tests to write**: full coverage is ticket 006's job (Part 8's 12
  sim criteria), but this ticket should add at minimum a smoke test per
  new piece of state (`mode_` defaults `kAuto`; `everMoved_` starts
  false; `lastActivity_` refreshes correctly on `kFlagActive`) so ticket
  006 is extending real coverage, not writing the first tests against
  brand-new code.
- **Verification command**: `uv run pytest` plus the sim HOST_BUILD test
  target that builds/runs `app_telemetry_harness`/`app_robot_loop_harness`
  (confirm exact `just`/CMake target name from the existing build setup).
