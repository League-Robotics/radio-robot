---
id: '002'
title: Gate the speed floor to the teleop path
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Gate the speed floor to the teleop path

# HARD PRECONDITION FOR TICKET 003 — SEQUENCE THIS FIRST

Without this fix, ticket 003's alignment nudges are boosted to `v_min` and the
fine-align phase is measured against an actuator it does not actually have.

## Description

**Source of truth: `docs/bench-reports/motion-planning-lab-2026-08-04.md` §5.3.**
This is the deferred sprint-131 item (Design Rationale Decision 4), and **both**
independent bench sessions on 2026-08-04 hit it independently.

`App::Drive::applySpeedFloor()` (`src/firm/app/drive.cpp:293-302`) boosts *any*
sub-floor wheel pair up to `v_min` (**20 mm/s** on `tovez`) by a ratio-preserving
scale:

```cpp
if (bounds_.vMin <= 0.0f) return;                        // uncalibrated: no-op
if (dominantMag <= 0.0f || dominantMag >= bounds_.vMin) return;
const float scale = bounds_.vMin / dominantMag;
```

That is **correct** for a standing teleop command sitting below breakaway — the
floor's entire purpose is to make a commanded crawl actually move. It is
**wrong** for a deliberately-dying profile tail or a low-speed alignment nudge:
both are shaped to be small, and boosting them to 20 mm/s destroys exactly the
terminal authority this sprint depends on.

### The gate: `commandActive_`, an existing member — no new dependency

`Drive` already knows the answer. `App::RobotLoop::handleMove()` calls
`drive_.takeover()` before every planner Move (`robot_loop.cpp:305`), and
`takeover()` sets `commandActive_ = false` (`drive.cpp:51-54`, verified). So:

| owner | `commandActive_` | floor |
|---|---|---|
| WHEELS teleop | `true` | **apply** |
| planner Move | `false` | **skip** |

`owns()` (`drive.h:482`) is the public reader of that same member.

**Do not** pass `planner_.active()` into `Drive::tick()` — see sprint.md Design
Rationale **D2**. That would create a new `Drive`→`Planner` coupling for
information `Drive` already holds, and it would mean **adding a member to
`drive.h`**, which per the standing build trap requires `just build-clean` or
encoders read a manufactured zero that looks exactly like a dead bus. The
intended change reads an existing member and adds no field.

### One thing to decide and state explicitly

There is a **deadman expiry** that also clears `commandActive_`
(`drive.cpp:118-121`): a teleop command whose deadline passes flips the flag to
`false` mid-tick, and `const bool owned = commandActive_;` at `:118` captures
the value *before* that check. Decide which side of the expiry the floor
applies to, and say so in a comment.

Note this is likely a non-issue in practice — `applySpeedFloor()` returns early
when `dominantMag <= 0`, so a fully-zeroed expired command is untouched either
way — but an expiring command that is still nonzero is a real case. **The safer
default is not to floor it**: a command on its way out should be allowed to
reach zero rather than being boosted back up to 20 mm/s.

### Semantics after this change

The floor becomes "**the floor is a teleop affordance**." Any future non-teleop
owner opts out automatically, which is the right default. The separate
host-side taper constant documented in
`clasi/issues/B-one-owner-per-constant-speed-floor-and-duty-per-speed.md` is
**not** in scope.

## Acceptance Criteria

- [x] `applySpeedFloor()` is applied only while `Drive` owns motion
- [x] A planner-owned sub-`v_min` wheel pair reaches actuation **unmodified**
- [x] A teleop `wheels()` sub-`v_min` command is **still** boosted to `v_min`
      with its ratio preserved (the 131-003 behaviour is intact for teleop)
- [x] "Stop is stop" is preserved: a genuine full stop still yields exactly
      `(0.0f, 0.0f)`
- [x] The deadman-expiry interaction is decided and documented in a comment
- [x] No new member is added to `drive.h` (and therefore no clean-build hazard
      is introduced)
- [x] `drive.cpp:499-541`'s long doc comment is updated — it currently states
      "The planner does NOT get its own vMin awareness this ticket (deferred,
      sprint 131 Design Rationale Decision 4)", which this ticket resolves

## Implementation Plan

**Files to modify** (expected 1-2):

1. `src/firm/app/drive.cpp` — the `applySpeedFloor()` call site in
   `Drive::tick()` at `:542-544`, plus the doc comment at `:499-541`
2. A test file for the two-case coverage below

The change is small on purpose. Resist the urge to restructure `tick()`.

**Build**: `uv run python build.py --clean --robot-debug` (plain `just build`
compiles the DBG channel **out**).

## Testing

- **Existing tests to run**: any `Drive`/speed-floor unit tests, plus
  `src/tests/sim`. Compare **by identity** against the master baseline of ~8
  failed / ~1994 passed.
- **New tests to write**: one test with both halves —
  1. planner owns motion, sub-floor command → passes through unboosted;
  2. teleop owns motion, sub-floor command → still boosted to `v_min`.
  Half (2) is the regression guard that matters most; the whole risk of this
  ticket is silently disabling the floor for everyone.
- **Hardware verification** is deferred to ticket 004, which explicitly
  re-checks teleop flooring on `tovez`.
- **Verification command**: `uv run python -m pytest src/tests/sim -q`
