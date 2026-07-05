---
id: '002'
title: Hal::Motor base-class armor policy + NezhaMotor leaf refit
status: open
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: armor-motor-write-path-against-reversal-latch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hal::Motor base-class armor policy + NezhaMotor leaf refit

## Description

The core architecture change of the sprint. Implements the reversal-latch
armor as shared, inline, headers-only policy in `Hal::Motor`
(`source/hal/capability/motor.h`) — not inside `NezhaMotor` — per the
sprint's binding placement constraint, then refits `NezhaMotor` to the new
leaf contract. Depends on ticket 001 (`reversal_dwell`/`output_deadband`/
`wedge_suspect`/`hard_reset_count`/`soft_reset_count` must already exist in
`msg::MotorConfig`/`msg::MotorState`). Full design, the exact base/leaf
split table, the leaf `tick()` call-order contract, and all rationale live
in `architecture-update.md` — read it before starting; this ticket
implements that document's "What Changed" items 1-4 and "The base/leaf
split — exact contract" section verbatim.

**In `Hal::Motor`** (`source/hal/capability/motor.h`):
- Add four new protected pure virtuals: `writeRawDuty(float duty)`,
  `hardReset()`, `softRebaseline()`, `configureDevice(const
  msg::MotorConfig& config)`.
- Add protected state: `reversalDwell_`, `outputDeadband_`, `dwelling_`,
  `dwellDeadline_`, `lastRequestedDuty_`, `resetPending_`, `restTicks_`,
  `hardResetCount_`, `softResetCount_`, `wedgePrevPosition_`,
  `wedgePrevValid_`, `stuckCount_`, `movingStuckCount_`, `wedgeLatched_`,
  `wedgeSuspect_` — see architecture-update.md's field table for exact
  types/comments (units in `// [unit]` tags, no unit suffixes in names).
- Add protected non-virtual methods: `armoredWrite(float duty, uint32_t
  now)`, `processResetIfPending(uint32_t now)`, `updateRestTracking()`,
  `updateWedgeDetector()`. Implement the dwell/deadband state machine and
  standstill-guard/wedge-qualification logic exactly as specified in
  architecture-update.md (including the state diagram and the two-signal
  distinction in Design Rationale 4: `lastRequestedDuty_` gates the
  standstill check, `appliedDuty()` gates wedge-suspect).
- Change `resetPosition()`, `wedged()`, `configure()` from pure virtual to
  concrete (public). Add concrete public `wedgeSuspect()`,
  `hardResetCount()`, `softResetCount()`. `configure()` caches
  `reversalDwell_`/`outputDeadband_` from the `Opt<float>` fields
  (defaulting to `kDefaultReversalDwell` = 100.0f `[ms]` /
  `kDefaultOutputDeadband` = 0.03f when `.has` is false), then calls
  `configureDevice(config)`.
- Extend `state()` to populate `wedge_suspect`/`hard_reset_count`/
  `soft_reset_count`, gated by `caps.has_encoder` exactly like the existing
  `wedged` field.
- Keep everything `inline` (in-class or free `inline` functions below the
  class, matching `apply()`/`state()`'s existing style) — no `motor.cpp`
  (architecture-update.md Design Rationale 7: headers-only is preserved,
  not revised).
- Add file-local constants `kRestVelocity` (proposed 5.0f `[mm/s]`) and
  `kRestTicksRequired` (proposed 5) for the standstill gate — document them
  as starting guesses subject to bench retuning in ticket 005 (do NOT
  promote them to `MotorConfig` fields this ticket).

**In `NezhaMotor`** (`source/hal/nezha/nezha_motor.{h,cpp}`):
- Implement `writeRawDuty(float duty)` = today's `writeDuty()` body
  **minus the `reversal` boolean and its exemption branch** (delete, don't
  preserve as dead code — architecture-update.md Design Rationale 6
  explains why it is structurally unreachable once `armoredWrite()` is in
  place). Write-on-change (`lastWrittenPct_`), the 40 ms throttle, and the
  `±slew_rate` clamp stay exactly as they are.
- Implement `hardReset()` = today's `hardResetEncoder()` body, unchanged.
- Implement `softRebaseline()` — new, ported from `source_old`'s
  `Motor::rebaselineSoft()` (`source_old/hal/real/Motor.cpp:237-273`):
  fold `lastPosition_` back into raw tenths-of-degrees using
  `travel_calib`/`fwd_sign` and add to `encOffset_`; zero
  `lastPosition_`/`filteredVelocity_`/`hasLastTick_`/`lastGoodRawEnc_`
  exactly as `hardReset()`'s success path does; increment
  `softResetCount_` (base-owned, via whatever accessor the base exposes —
  do not duplicate the counter in `NezhaMotor`). Issues NO I2C transaction.
- Implement `configureDevice(config)` = today's `configure()` body (the
  `slew_rate <= 0` defaulting, etc.), minus the two armor fields (now
  base-owned).
- Delete: the private wedge-detector fields (`wedgePrevEnc_`,
  `wedgePrevValid_`, `stuckCount_`, `wedgeLatched_`) and
  `updateWedgeDetector()` method (moved to the base); the private
  `resetPending_` field (moved to the base).
- Refit `tick()` to the leaf contract's exact 5-step call order
  (architecture-update.md): `processResetIfPending(now)` →
  sample-and-cache-encoder (unchanged) → `updateWedgeDetector()` → mode
  dispatch (`DUTY`/`VELOCITY`/`NEUTRAL` → `armoredWrite(duty, now)`;
  `POSITION` → `writePositionMove()` directly, unchanged, out of the
  armor's scope) → `updateRestTracking()`.
- Constructor: after member-initializer-list construction, call
  `configure(config)` explicitly as the last line of the constructor body
  (do NOT add a `Motor(...)` base constructor that calls a virtual — see
  architecture-update.md's Construction note on the virtual-dispatch-
  during-construction pitfall this avoids).

## Acceptance Criteria

- [ ] `Hal::Motor` gains the four protected pure virtuals and all listed
      protected state/methods; `resetPosition()`/`wedged()`/`configure()`
      are concrete; `wedgeSuspect()`/`hardResetCount()`/`softResetCount()`
      are new concrete public methods.
- [ ] All new `Hal::Motor` methods are `inline`; `source/hal/capability/`
      remains headers-only (no new `.cpp`).
- [ ] `NezhaMotor` implements all four new protected virtuals per the
      mapping above; its old wedge-detector fields/method and
      `resetPending_` are deleted (not left as dead code); the
      reversal-exemption branch is deleted from the write path.
- [ ] `NezhaMotor::tick()` follows the exact 5-step call-order contract
      from architecture-update.md.
- [ ] A commanded sign change (verified by code inspection / a quick
      manual trace, since ticket 004 provides the automated proof) writes
      0 immediately, then suppresses non-zero writes for `reversalDwell_`
      ms, then proceeds in the new direction; a commanded stop (`duty ==
      0` or sub-deadband) is immediate and unclamped even mid-dwell.
- [ ] A `resetPosition()` request dispatches `hardReset()` when
      `restTicks_ >= kRestTicksRequired`, else `softRebaseline()` —
      never an atomic burst while `lastRequestedDuty_ != 0`.
- [ ] `wedged()` reports the raw, unconditional stuck-encoder latch exactly
      as before (unchanged semantics — do not reintroduce target-gating or
      arming-grace); `wedgeSuspect()` reports true only when the raw latch
      is held while `|appliedDuty()|` exceeds `outputDeadband_`.
- [ ] `kRestVelocity`/`kRestTicksRequired` are documented in-code as
      starting guesses, explicitly flagged for retuning in ticket 005's
      bench pass (do not silently treat them as final).
- [ ] `just build` succeeds; the firmware boots and `NezhaHal` constructs
      all four ports without change to `main.cpp`'s `NezhaHal`
      construction or `Drivetrain` (verify no other call site needed a
      change — `dev_commands.cpp:372`'s `configure()` call site keeps its
      exact signature).

## Testing

- **Existing tests to run**: `uv run python -m pytest`; `just build`.
- **New tests to write**: none in this ticket (ticket 004 provides the
  automated off-hardware proof of the policy's behavior via a `MockMotor`
  harness that depends on this ticket's public contract). This ticket's
  own verification is code review against the acceptance criteria above
  plus a successful build — do not skip ahead into ticket 004's harness.
- **Verification command**: `just build && uv run python -m pytest`
