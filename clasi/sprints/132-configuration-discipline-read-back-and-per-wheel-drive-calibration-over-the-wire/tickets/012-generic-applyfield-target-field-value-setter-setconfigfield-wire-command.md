---
id: '012'
title: Generic applyField(target, field, value) setter + SetConfigField wire command
status: open
use-cases:
- SUC-004
depends-on:
- '007'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Generic applyField(target, field, value) setter + SetConfigField wire command

## Description

Add `Configurator::applyField(ConfigTarget target, uint16_t fieldNumber,
float value)`, exposing `wire.cpp`'s existing field-lookup/
`validateBounds`/write-scalar loop (currently sealed in an anonymous
namespace, `wire.cpp:546-597`) as a callable surface — "the loop minus
tag decoding," per the issue's own design. Must reject NaN via
`isfinite()` BEFORE `validateBounds()` (`validateBounds()`'s `<`/`>`
comparisons are both false for NaN, so NaN passes every bound check
otherwise — a documented trap). Add `SetConfigField` to the wire
schema/dispatch. Host: `NezhaProtocol.set_config_field(target,
field_name, value)` resolves `field_name` to its wire number via the
REAL protobuf descriptor (`ConfigDrive.DESCRIPTOR.fields_by_name[...].number`-style
lookup) so a human still types a name while the wire carries only a
number.

## Acceptance Criteria

- [ ] `wire.cpp`'s field-lookup (`findField`-equivalent) and
      `validateBounds()` are reachable from `Configurator::applyField()`
      — either by exposing them outside the anonymous namespace or an
      equivalent refactor that avoids duplicating the lookup logic.
- [ ] `applyField()` rejects a NaN `value` via an explicit `isfinite()`
      check BEFORE calling `validateBounds()`, verified by a new test
      pushing NaN and confirming rejection.
- [ ] `applyField()` rejects an unknown field number with `ERR_BADARG`.
- [ ] `applyField()` on a valid field/value writes it into `config_` at
      the correct offset and calls `install(target)`.
- [ ] `SetConfigField` is declared on the wire and dispatched from
      `robot_loop.cpp`'s `processMessage()`.
- [ ] `NezhaProtocol.set_config_field(target, field_name, value)` exists,
      resolves `field_name` via the real generated protobuf descriptor
      (not a hand-maintained string-to-number table), and sends
      `SetConfigField`.
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: unaffected.
- **New tests to write**: NaN rejection, unknown-field rejection, valid
  single-field set landing at the right offset and reaching `install()`.
- **Verification command**: `uv run python -m pytest
  <configurator/protocol test paths> -q`.

## Implementation Plan

**Approach**: Refactor the minimum necessary surface of `wire.cpp`
(`FieldDesc` lookup by `(table, number)`, `validateBounds()`) out of its
anonymous namespace into something `Configurator` can call — this is a
generator-emitted concern per the issue's own text ("This must be
emitted by the generator... That is a change to the generator's fixed
engine text"), so the fix likely belongs in `gen_messages.py`'s fixed
engine-text template (ticket 002's territory) rather than a hand-edit of
the generated `wire.cpp` — coordinate sequencing accordingly.

**Files to modify**: `src/scripts/gen_messages.py` (engine-text
template, to expose the lookup), `src/firm/app/configurator.{h,cpp}`,
`src/firm/app/robot_loop.cpp`, `src/host/robot_radio/robot/protocol.py`.

**Testing plan**: as above.

**Documentation updates**: `docs/protocol-v5.md` addendum for
`SetConfigField`, alongside ticket 011's `GetConfig`/`ConfigSnapshot`
addendum.
