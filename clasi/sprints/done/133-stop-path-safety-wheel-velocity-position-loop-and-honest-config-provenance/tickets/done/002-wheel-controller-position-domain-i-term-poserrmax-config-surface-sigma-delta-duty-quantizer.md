---
id: '002'
title: 'Wheel controller: position-domain I term, posErrMax config surface, sigma-delta
  duty quantizer'
status: done
use-cases:
- SUC-002
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: B-wheel-controller-position-loop-and-tuning.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wheel controller: position-domain I term, posErrMax config surface, sigma-delta duty quantizer

## Description

**This is the sprint's headline ticket** — the one the stakeholder is waiting to
get his hands on. The work is already built and measured (on `vevov`, a throwaway
`position-pid` branch); this ticket re-applies it properly. Full reference:
`clasi/issues/B-wheel-controller-position-loop-and-tuning.md` §7, and
`clasi/issues/attachments/wheel-controller-2026-08-03/firmware-changes.patch`.
Read §7 in full — the comments in the patch explain the reasoning at each site
and are stripped from the issue body for length.

**Depends on ticket 001.** Before the stop fix, every "overshoot" number in the
source measurements included a tail that was a runaway rather than a control
error. Do not re-measure anything here against a tree without 001.

### Three defects, one tuning result

**(1) The I term was always a position term, computed the worst possible way.**
`integral += ki · err · dt`, with `err` in mm/s and `dt` in s, accumulates
**millimetres** — commanded position minus measured position. The velocity loop
always contained a position controller. It just built its position estimate by
summing a *derived, quantized* velocity (sd 8.0 mm/s) instead of reading the
encoder's position register, so every dropped or manufactured-zero sample
permanently deleted real distance from the sum. Replacing it with a direct read
is what fixed the imbalance: 1.05–1.07 → 1.000–1.006. Note the direction —
*more* position gain gives *better* balance, the opposite of how every
velocity-side knob behaved.

**(2) A unit-domain bug fell out of it.** `iMax` clamps `ki · error`, so the
maximum position error the loop could **remember** was `iMax / ki` mm — at
ki=6/iMax=20 that is **3.3 mm**, smaller than every residual being chased.
Raising `ki` made the memory *shorter*, which is why the ki sweep plateaued
between 2 and 6.

> **`posErrMax` does NOT replace `iMax`.** The issue's own wording implies
> replacement — it says "`posErrMax` (mm) replaces it" — and that reading will
> produce a wrong implementation. Sprint architecture Decision 2: the two clamp
> **different domains** and both are load-bearing.
>
> | field | unit | clamps |
> |---|---|---|
> | `posErrMax` | **[mm]** | the **input** — position error, before the gain |
> | `iMax` | **[mm/s]** | the **output** — the I term `ki · posError`, after the gain |
>
> The measured configuration uses **both**: `posErrMax 5, iMax 100`. Delete
> `iMax` and you remove the only bound on how much velocity authority the I term
> may demand. What the fix actually does is stop `iMax` being a *disguised*
> position limit — once position error is clamped in mm, `iMax` becomes what it
> always honestly was.
>
> Both fields carry a `// [unit]` tag and a doc comment saying which domain each
> bounds. The confusion between them **is** the original defect; make it
> impossible for the next reader to repeat.

**(3) The actuator's resolution is the floor.** Duty reaches the brick as an
integer percent with `kDutyPerSpeed = 0.001182`, so **one duty count is
8.46 mm/s** — 5.6% of a 150 mm/s command. Measured plateau velocity clusters at
exactly −1/0/+1 counts, sd 8.0 mm/s against an 8.46 mm/s step. The left/right
imbalance being chased was **0.43 of ONE count**. No gain can command a value
the output cannot represent, which is why the ki/kff/aSteady sweeps returned
nothing.

### The code

**A. Sigma-delta duty quantizer** — `src/firm/devices/nezha_motor.cpp`,
`writeRawDuty()`. Replaces `int8_t pct = lroundf(duty * 100.0f);` with the
carry-accumulating form in §7-C, plus `float dutyCarry_ = 0.0f;  // [percent]
fractional, [-1, 1]` in `nezha_motor.h`.

> **The carry MUST be discarded on commanded zero** (`duty == 0.0f` →
> `dutyCarry_ = 0.0f`, `pct = 0`). Architecture Decision 3: a residual left from
> the last nonzero duty would round to ±1 count and creep a stopped wheel —
> re-creating ticket 001's headline safety defect from inside this fix. Losing
> sub-count fidelity across a stop is the correct trade: stop is stop.

**B. The I term reads position** — `src/firm/app/drive.cpp` / `drive.h`, §7-D.
`fastPid()` takes `posError` and computes `integral = gains_.ki * posError`
instead of accumulating. New `positionError()` and the `PositionRef` struct
(`reference` / `origin` / `epoch` / `armed`). `estop()` resets
`posRefLeft_ = PositionRef{};` where it used to zero the integrators.

Three guards earn their place and must survive review: commanded zero passes
straight through (stop is stop); an epoch change or a disconnect **re-anchors
without correcting** (a rebaseline steps `position`, and a disconnected wheel
reports a manufactured zero); and the error is clamped in **mm** so a blocked
wheel cannot bank unbounded catch-up debt and then sprint to repay it.

Same control law, same units (`ki` stays 1/s), same output — the loop still
commands velocity, which is all the motor takes. No accumulator, so no windup,
no `steady` gate to freeze it, no reset to lose.

**C. `posErrMax` through the whole config surface.** Architecture Decision 4 and
`.claude/rules/configuration-discipline.md` invariant 2: every value in the file
reaches the robot, with a runtime path **and** a bake path reading the same file.
A `posErrMax` that exists only as a `DBG` verb is exactly the "all tuning is
RAM-only" state the issue lists under what is still wrong. So:

- `src/protos/robot_config.proto` — `message WheelControl` gains
  `float pos_err_max = 12 [(min) = 0.0];  // [mm] Stage B position-error clamp; 0 = unclamped`.
  Field 12 is the next free number; do not renumber or reuse anything.
- The generated boot config and `gen_boot_config.py`.
- **Every** `data/robots/*.json` (`tovez.json`, `tovez_nocal.json`) gains
  `wheel_control.pos_err_max`.
- The live wire path (`Configurator::applyGroup()`/`applyField()` reach it
  through the existing `WHEEL_CONTROL` machinery — verify, do not assume).
- `Drive::setPositionErrorMax(float posErrMax)  // [mm]` and
  `float posErrMax = 0.0f;  // [mm] ... 0 = unclamped` on `ControlBounds`.

Seed the JSON value **conservatively and honestly**: this ticket does not have a
`tovez` measurement, and `5` is a `vevov` bare-motor figure. Ticket 004 sets
`tovez`'s real value. Note the field's own "0 = disabled" convention, which
`WheelControl`'s doc comment already establishes for every sibling.

**D. `cmdAccel` on the WHEELS path** — `src/firm/app/drive.cpp`, `update()`,
§7-E, with `static constexpr float kAccelSmoothing = 0.35f;` and the four float
members. Smoothed because the host re-arms slower than `kCycle`, so the raw
command is a staircase and a bare finite difference alternates a double-size
spike with a zero.

> Land this **as UNVALIDATED**, and say so in the code comment. It is correct on
> its own terms — the field genuinely was never set, which is why the earlier
> kff/aSteady sweeps measured nothing (`kaff · cmdAccel` was identically zero and
> `steady` was pinned true) — but the sweep on top of it returned ~50% dead runs.
> **Inert at `kff = 0`, which is the only reason it is safe to land.** Sprint
> Open Question 2. Do not close it.

### Naming and style

Per `.claude/rules/naming-and-style.md`: no units in any identifier, units in a
leading bracketed `// [unit]` tag; lowerCamelCase functions and variables,
UpperCamelCase types, trailing underscore on class data members. `posErrMax`,
`dutyCarry_`, `positionError()`, `PositionRef` all already conform — keep them
that way. Note `pos_err_max` as a **proto/JSON key** is a wire/serialized
identifier and is explicitly excluded from the rule (see
`.claude/rules/coding-standards.md`); it stays snake_case like its siblings.

### Build traps — both have already cost real time

- **Adding a member to `Drive` in `drive.h` needs `just build-clean`.** This
  ticket adds several (`PositionRef posRefLeft_/posRefRight_`, the four
  `cmdAccel` floats). An incremental build links stale objects against the old
  class layout, and the encoders then read a manufactured zero that looks
  **exactly like a dead bus**. This cost an hour of hunting a hardware fault that
  did not exist. If you see dead-bus symptoms after this change, build clean
  before investigating anything else.
- **A plain `just build` compiles the DBG channel OUT.** Use
  `uv run python build.py --clean --robot-debug` for anything to be flashed.

## Acceptance Criteria

- [x] `Drive`'s Stage B I term is computed from `Wheel::position` via
      `positionError()`/`PositionRef` — no accumulator, no windup, no `steady`
      gate, no reset to lose.
- [x] `positionError()` passes commanded zero straight through, re-anchors
      without correcting on an epoch change or a disconnect, and clamps in mm.
      **Deviation from the patch, required to make this true**: `tick()` calls
      `positionError()` UNCONDITIONALLY and skips only `fastPid()` at commanded
      zero. The patch short-circuited the whole expression on `speed == 0.0f`,
      which made the function's own commanded-zero guard unreachable — the
      reference survived the stop and was repaid as a sprint on resumption, and
      went negative by the coast distance. Caught by the new guard test.
- [x] `estop()` resets both `PositionRef`s.
- [x] `posErrMax` [mm] clamps the position error (input) and `iMax` [mm/s] clamps
      the I-term output — **both retained**, each with a `// [unit]` tag and a
      doc comment naming the domain it bounds. `iMax == 0` deliberately still
      means "I term OFF", not "unclamped" (every shipped robot JSON carries 0).
- [x] `WheelControl.pos_err_max` exists as proto field 12, in the generated boot
      config, in every `data/robots/*.json`, and on the live wire path. No field
      is renumbered or reused.
- [x] The bake path and the runtime path read the same file; read-back-equals-file
      still holds for every robot JSON (sprint 132's headline property).
- [x] `writeRawDuty()` represents a commanded duty strictly between two integer
      percent counts over successive cycles rather than truncating to one.
- [x] `dutyCarry_` is zero whenever the commanded duty is zero, with a comment
      saying why (it would otherwise creep a stopped wheel).
- [x] `cmdAccel` is set on the WHEELS path, is documented in code as
      **unvalidated**, and is inert at `kff = 0`.
- [x] A clean ARM build succeeds (`uv run python build.py --clean --robot-debug`)
      — `MICROBIT.hex (ROBOT_DEBUG bench variant)`. The build's automatic
      `dotconfig version bump` was reverted afterwards per
      `.claude/rules/git-commits.md` (one bump per sprint, at `close_sprint`).

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim
  src/tests/unit` plus the robot-config generator tests
  (`test_gen_boot_config_*.py`, `test_robot_config_proto_parses.py`,
  `test_gen_messages_robot_config_emission.py`) — the config-surface change
  touches all of them. Note two `test_gen_boot_config_planner.py` tests are in
  the sprint's pre-existing-failure baseline; if this ticket changes that **set**
  (not the count), name every entry that moved and why.
- **New tests to write**:
  - Host unit: `pos_err_max` round-trips proto → generated boot config → robot
    JSON, and read-back-equals-file still holds.
  - Sim: the position-domain I term is exercised through `SimLoop`; a
    manufactured-zero or dropped sample does **not** permanently delete distance
    from the loop's position estimate (this is the specific defect being fixed —
    the old accumulator would have).
  - Sim/unit: `positionError()`'s three guards — commanded zero, epoch change,
    disconnect — each re-anchor or pass through as specified.
  - Unit: the sigma-delta represents a sub-count command over time, and the carry
    is zero on commanded zero.
- **Verification command**: `uv run python -m pytest src/tests/sim src/tests/unit`
- **Not in this ticket**: any claim about measured imbalance, and any `tovez`
  tuning values. Ticket 004 owns both. Do not transcribe the issue's `vevov`
  numbers into `tovez.json` here.
