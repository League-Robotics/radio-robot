---
id: '003'
title: Gate the serial-silence watchdog's fire on commanded motors-running state
status: open
use-cases: [SUC-005]
depends-on: ['001', '002']
github-issue: ''
issue: watchdog-arm-only-while-motors-running.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Gate the serial-silence watchdog's fire on commanded motors-running state

## Description

The serial-silence watchdog (`SerialSilenceWatchdog`, `source/commands/
dev_commands.h`; fired from `Rt::MainLoop::serviceWatchdogs()`) fires on
ANY comms silence past its window, including while the robot is completely
idle with motors stopped — a spurious neutralize + `EVT dev_watchdog` with
no runaway to prevent. It should only fire while motors are actually
commanded to run.

**`msg::DrivetrainState.active` alone is NOT a sufficient gate.** Tracing
`isBoundPort()`'s authority-steal in `dev_commands.cpp` shows
`DrivetrainState.active` goes FALSE the instant any bound-port `DEV M`
motion verb lands (the 077-007 fix that puts Drivetrain into standby so a
standalone motor command doesn't fight the governor) — so gating on it
alone would silently stop protecting the single most common bench pattern,
`DEV M 1 VEL 100` on the normal drive pair. See `architecture-update.md`
Decision 3 for the full reasoning. This ticket therefore adds a small,
symmetric, commanded (never measured) per-motor `active` bit and gates on
`bb.drivetrain.active || any(bb.motors[i].active)`.

**Radio-path HITL bench is explicitly OUT of this ticket's acceptance.**
The issue also asks for an on-stand, over-the-radio-relay verification
(and to close out sprint 087's still-open watchdog-over-radio concern).
The relay dongle is unplugged this run and cannot be exercised — this
ticket's deliverable is the sim tests only. Before this ticket is
considered done, file a fresh `clasi/issues/` item for the deferred
radio-path HITL bench (do not leave it as an unmet criterion on THIS
ticket, and do not mark this ticket blocked by it).

## Acceptance Criteria

### New commanded per-motor state

- [ ] `Hal::Motor` gains a private `bool active_ = false;` member and a
      public `bool active() const` getter.
- [ ] `Hal::Motor::apply()`'s existing `switch (kind)` (the same switch
      that already special-cases `NEUTRAL`) sets `active_ = false` in the
      `NEUTRAL` branch and `active_ = true` in the `DUTY_CYCLE`/`VOLTAGE`/
      `VELOCITY`/`POSITION` branches. The `NONE`/`default` branch leaves
      `active_` untouched. A REJECTED command (fails
      `motorCommandAllowed()`, returns before the switch) never touches
      `active_`.
- [ ] `msg::MotorState` gains `active` (bool, always populated — no
      `has_encoder` gate, unlike `position`/`velocity`/`wedged`).
      `Hal::Motor::state()` sets `s.active = active();` — proto change in
      `protos/motor.proto`, regenerated via `scripts/gen_messages.py`.

### Fire-gate

- [ ] `Rt::MainLoop::serviceWatchdogs()`'s fire branch becomes:
      `if (watchdog_.check(now) && motorsRunning(bb)) { estop(); ...EVT... }`
      where `motorsRunning(bb)` is
      `bb.drivetrain.active || bb.motors[0].active || bb.motors[1].active
      || bb.motors[2].active || bb.motors[3].active` (a small free function
      or inline helper — implementer's choice of exact shape, but it must
      read only `bb`, computing nothing new).
- [ ] `watchdog_.check(now)` is still called UNCONDITIONALLY every pass
      (not skipped when idle) — preserves `SerialSilenceWatchdog`'s
      internal fire-once/re-arm-on-`feed()` semantics exactly. Only the
      ACTION (estop + EVT) is gated, not the check call itself.
- [ ] The same-pass estop bypass, fire-once `EVT`, and `DEV WD`-settable
      window are otherwise completely unchanged.

### Sim tests (this ticket's deliverable in place of the HITL bench)

- [ ] New test in `tests/sim/unit/test_watchdog_policy.py`: motors
      stopped/neutral (no `DEV M`/`DEV DT` motion verb ever issued, or a
      prior one explicitly neutralized), narrow window (`DEV WD 100`),
      silence past the window → `sim.get_async_evts()` contains NO
      `dev_watchdog`, and the (already-neutral) motor state is unaffected.
  - [ ] Also cover: a port was driven, then explicitly neutralized
        (`DEV M <n> NEUTRAL B`) or `DEV STOP`, THEN goes silent past the
        window → still no fire (proves the gate reads current state, not
        "was ever commanded").
- [ ] Existing tests in the same file
      (`test_watchdog_fires_after_window_expires_and_neutralizes`,
      `test_watchdog_does_not_fire_while_commands_keep_arriving`,
      `test_watchdog_neutralizes_within_the_same_pass_it_fires_in`) pass
      unmodified — each already commands a motor (`DEV M 1 VEL 50`) before
      going silent, so `bb.motors[0].active` is true throughout and the
      new gate does not change their outcome.
- [ ] `uv run python -m pytest tests/sim` green (309 + this ticket's new
      test(s)).

### Deferred follow-on (process, not a test)

- [ ] A fresh `clasi/issues/` file is created for the radio-path HITL
      watchdog bench (on-stand, over the relay: a long drive then host
      silence neutralizes + emits `EVT dev_watchdog`; an idle-then-silence
      case does not fire) — referencing this ticket/sprint and sprint
      087's original unresolved bench acceptance. This is a ticket
      deliverable (the issue file must exist), not a blocking test.

## Implementation Plan

### Approach

1. Proto: add `MotorState.active` to `protos/motor.proto`; regenerate.
2. `Hal::Motor` (`source/hal/capability/motor.h`): add `active_`, toggle
   it in `apply()`'s existing switch, add `active()` getter, populate
   `msg::MotorState.active` in `state()`.
3. `Rt::MainLoop` (`source/runtime/main_loop.h`/`.cpp`): add the
   `motorsRunning(bb)` predicate and the `&&` gate in
   `serviceWatchdogs()`'s fire branch (this is the SAME branch ticket 001
   already touched for the `estop()` rename — this ticket edits it again,
   hence `depends-on: ['001']`).
4. Extend `tests/sim/unit/test_watchdog_policy.py` per the acceptance
   criteria above.
5. File the deferred radio-path-bench issue in `clasi/issues/`.

### Files to Create/Modify

- `protos/motor.proto`
- `source/messages/motor.h` (regenerated)
- `source/hal/capability/motor.h`
- `source/runtime/main_loop.h` / `.cpp`
- `tests/sim/unit/test_watchdog_policy.py`
- `clasi/issues/<new-radio-path-watchdog-bench-issue>.md` (new file)

### Testing Plan

- `uv run python -m pytest tests/sim` before and after; must stay green,
  with the new idle-no-fire test(s) added and passing.
- No HITL/bench run for this ticket (relay dongle unplugged — see
  Description). Do not attempt a serial-only or partial bench substitute;
  defer cleanly per the acceptance criteria above.

### Documentation Updates

- None required for the wire protocol (no new verb, no reply-shape
  change — the watchdog's `EVT dev_watchdog` text is unchanged). If
  `docs/protocol-v2.md` documents the watchdog's fire conditions
  explicitly, update that prose to describe the motors-running gate.
