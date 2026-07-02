---
id: '003'
title: Add VW-class velocity-refresh staleness cap independent of the keepalive
status: open
use-cases: [SUC-003]
depends-on: ['002']
github-issue: ''
issue: stop-delivery-and-keepalive-watchdog-architecture.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add VW-class velocity-refresh staleness cap independent of the keepalive

## Description

CR-05b, firmware side (part of CR-04/CR-05, high). Ticket 002 narrows the
watchdog reset to `+`/motion verbs, but `+` itself still resets the
watchdog by design — that is its whole purpose. If the host's
`SerialConnection` keepalive daemon thread is still alive and still emitting
`+` (a background OS thread, independent of whatever code is actually
supposed to be refreshing the `VW` target — e.g. a frozen Qt GUI event
loop), the firmware cannot tell the difference: `+` keeps arriving, the
watchdog keeps resetting, and the robot keeps driving at the last `VW`
target. This is distinct from (and not fixed by) ticket 002 — it is the
"keepalive thread outlives a frozen VW-issuing layer" gap called out in the
issue.

Fix: `Planner` (the only code that legitimately updates an open-ended
target) stamps a timestamp on every genuine velocity-target refresh;
`Superstructure::evaluateSafety` additionally trips the watchdog if that
timestamp goes stale, using the same `sTimeoutMs` threshold, regardless of
whether `+` is still arriving. See `architecture-update.md` Step 4-5 item 3
and Design Rationale Decision 3 for the full design and why staleness is
tracked at the `Planner` layer rather than by enumerating wire verbs.

Depends on ticket 002 (extends the same `evaluateSafety` watchdog block that
ticket 002 modifies) to avoid rebase friction.

## Acceptance Criteria

- [ ] `source/superstructure/Planner.h`/`Planner.cpp` (or `PlannerBegin.cpp`)
      gain a private `uint32_t _lastVelocityRefreshMs = 0;` and a public
      getter `uint32_t lastVelocityRefreshMs() const`.
- [ ] `Planner::beginVelocity()` stamps `_lastVelocityRefreshMs = now_ms;`
      (covers `S`, `VW`, `T`, `R` — everything routed through
      `Goal::VELOCITY`).
- [ ] `Planner::beginRawVelocity()` gains a `uint32_t now_ms` parameter
      (currently missing) and stamps the same member; its single call site
      (`MotionCommands.cpp:1293`, `handle_VW`) is updated to pass
      `ctx->robot->systemTime()`.
- [ ] `Superstructure::evaluateSafety()`'s watchdog block trip condition
      becomes `(wdDelta > sTimeoutMs) || (vwDelta > sTimeoutMs)` where
      `vwDelta = now - _planner.lastVelocityRefreshMs()`, evaluated only
      when `needsWatchdog` is true (i.e. only for open-ended commands — no
      new gating logic duplicated).
- [ ] No new `RobotConfig` field — `sTimeoutMs` (existing, default 500 ms) is
      reused for both signals.
- [ ] New sim test: an active `VW` kept alive by `+` only (no fresh `VW`
      resend) for longer than `sTimeoutMs` safety-stops despite the
      continuous `+`.
- [ ] Regression: an active `VW` refreshed by its own resends (no `+` at
      all) continues to satisfy the watchdog (`_lastVelocityRefreshMs`
      alone is sufficient) — confirms this doesn't require both signals
      simultaneously.
- [ ] Regression: `T`/`D`/`G`/`TURN`/`RT` sessions (which never call
      `beginVelocity`/`beginRawVelocity` for their own primary command, or
      whose `TIME` stop already exempts them via `needsWatchdog == false`)
      are unaffected.
- [ ] Full default sim suite green.

## Implementation Plan

**Approach**: Centralize "was an open-ended velocity target genuinely
refreshed" as `Planner`-owned state (stamped at the two call sites that
create/refresh such a target), and read it directly from
`Superstructure::evaluateSafety` (which already holds a `Planner&`
reference) — no new coupling, no wire-verb enumeration to keep in sync.

**Files to modify**:
- `source/superstructure/Planner.h` — new member + getter declaration.
- `source/control/PlannerBegin.cpp` (or wherever `beginVelocity`/
  `beginRawVelocity` are defined) — stamp `_lastVelocityRefreshMs`.
- `source/commands/MotionCommands.cpp` — `handle_VW`: pass `now_ms` to the
  now-changed `beginRawVelocity(v, omega, now_ms)` signature.
- `source/superstructure/Superstructure.cpp` — `evaluateSafety()`'s
  watchdog-trip condition.

**Testing plan**:
- New sim test: arm `VW`, send only `+` (no VW resend) past `sTimeoutMs` →
  must safety-stop.
- Regression test: arm `VW`, resend `VW` itself (no `+`) → must NOT
  safety-stop, confirming `VW` resends alone remain sufficient (this is
  exactly what `KeyboardDriver`'s existing resend-timer behavior relies on,
  independent of tickets 004/005's host-side changes).
- Run the full default sim suite, with particular attention to any existing
  test that holds a `VW` session open for longer than `sTimeoutMs` relying
  on ambient `+` alone — such a test would need updating to also resend
  `VW` or to arm the sim's keepalive equivalent; search
  `tests/simulation/` for `sim_command(h, "+"` / `"VW "` co-occurrence
  patterns before changing the trip condition.

**Documentation updates**: `architecture-update.md` already documents this
change (Step 4-5 item 3, Design Rationale Decision 3). No wire-protocol
change.
