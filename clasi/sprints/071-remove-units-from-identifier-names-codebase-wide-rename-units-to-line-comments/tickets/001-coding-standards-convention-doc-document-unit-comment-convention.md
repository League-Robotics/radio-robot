---
id: '001'
title: 'Coding-standards convention doc: document unit-comment convention'
status: done
use-cases:
- SUC-007
depends-on: []
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Coding-standards convention doc: document unit-comment convention

## Description

Add a new `docs/coding-standards.md` documenting the leading-bracket
unit-comment convention that every subsequent ticket in this sprint
(002-008) conforms to. This ticket makes **no code changes** — it exists
so the convention is defined once, in a durable, cross-cutting reference,
before any identifier is renamed. This is the sprint's SUC-007. Per
`architecture-update.md` Step 3/4a, ticket 001 is the sole root of the
ticket dependency graph — every other ticket depends on it.

Convention (transcribed from `architecture-update.md`'s "Comment
Convention" section, the authoritative source for this ticket):

```cpp
// Before
float tgtMms[kWheelCount] = {};  // all-wheel speed targets, mm/s

// After
float tgtSpeed[kWheelCount] = {};  // [mm/s] all-wheel speed targets
```

```python
# Before
def send(self, cmd: str, read_ms: int = 500) -> dict: ...

# After (sprint 072 — not applied to any host/ file by this sprint)
def send(self, cmd: str, read_timeout: int = 500) -> dict:  # [ms]
    ...
```

- **Format**: a leading, bracketed unit tag as the *first token* of the
  declaration's trailing (or block) comment.
- **Unit vocabulary**: reuse whatever unit text the surrounding prose
  already uses elsewhere in the file (`mm`, `mm/s`, `mm/s²` or `mm/s^2`,
  `deg`, `deg/s`, `deg/s²`, `ms`, `us`, `%`, `Hz`, `rad`, `rad/s`,
  `rad²/s`, `mm²/s`) — do not invent a second vocabulary; grep for the
  unit already used in the block comment above the field being renamed.
- **Grep-ability**: the tag's fixed leading position means `grep -rn "//
  \[mm/s\]" source/` (or `grep -rn "# \[ms\]" host/` in sprint 072) finds
  every declaration of that unit, independent of identifier spelling.
- **No tag for dimensionless/boolean/enum fields** — `rotationalSlip`,
  `kFF`, `velKp`, `odomUpsideDown`, `drivetrain` never had a unit suffix
  and get no tag (nothing to disambiguate).
- **Ambiguity-resolution rule**: where stripping a unit suffix would
  collide two previously-distinguished names (e.g. a `Mm`-suffixed float
  position vs. a raw-ticks integer counterpart), ticket authors choose a
  descriptive replacement for the *kind* of quantity (`positionLinear` vs.
  `positionTicks`) rather than a bare strip that produces a collision or
  an ambiguous bare word.
- **Derived-unit names**: rename to what the quantity *is*, with the unit
  in the comment — e.g. `mmPerDegL`/`mmPerDegR` (and the mecanum siblings
  `mmPerDegFR/FL/BR/BL`) → `wheelTravelCalibL`/`wheelTravelCalibR`/
  `wheelTravelCalibFR/FL/BR/BL` `// [mm/deg] wheel linear travel per
  motor-shaft degree of rotation`. Simply stripping `Deg` from `mmPerDegL`
  would leave `mmPerL`, which still embeds `mm` and reads worse, not
  better.

See `architecture-update.md`'s "Comment Convention" section and Decision 5
(derived-unit naming rationale); `usecases.md` SUC-007.

## Acceptance Criteria

- [x] `docs/coding-standards.md` exists (new file) with a "Units in
      Identifiers" section (or equivalent heading).
- [x] Documents the C++ convention: leading `// [unit]` tag as the first
      token of the trailing/block comment, with the issue's own worked
      example (`tgtMms` → `tgtSpeed  // [mm/s]`) reproduced verbatim.
- [x] Documents the future Python convention (`# [unit]`, sprint 072) as a
      forward reference — not applied to any `host/` file by this ticket
      or this sprint.
- [x] States the unit-vocabulary rule: reuse the unit text already used
      elsewhere in the file; do not invent a second vocabulary.
- [x] States the dimensionless-field rule: no tag for dimensionless/
      boolean/enum fields.
- [x] States the compound-unit convention with examples (`mm/s`,
      `mm/s²`/`mm/s^2`, `deg/s`, `rad²/s`, `mm²/s`).
- [x] States the ambiguity-resolution rule (descriptive replacement over
      bare strip when a collision would result).
- [x] States the derived-unit naming rule with the `mmPerDegL/R` →
      `wheelTravelCalibL/R` worked example.
- [x] States the grep-ability rationale (`grep -rn "// \[mm/s\]" source/`
      finds every declaration of that unit).
- [x] No `source/` or `host/` code is modified by this ticket.
- [x] Full test suite remains green (`uv run python -m pytest`, 2620
      passed, 0 failed) — expected to be a no-op since no code changes.

## Testing

- **Existing tests to run**: full suite (`uv run python -m pytest`) as a
  sanity check that a docs-only change doesn't disturb anything (expected:
  unchanged 2620/0 baseline, modulo normal timing jitter).
- **New tests to write**: none — this is a documentation-only ticket.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Transcribe the "Comment Convention" section of
`architecture-update.md` (and Decision 5) into a new, standalone
`docs/coding-standards.md`. Keep the doc focused on the units-in-
identifiers convention — do not scope-creep into other coding-standards
topics not covered by this sprint.

**Files to create**: `docs/coding-standards.md`

**Files to modify**: none.

**Testing plan**: run the full suite once as a sanity check (docs-only
change, no behavioral surface).

**Documentation updates**: this ticket *is* the documentation update. No
other doc changes are needed here — `docs/protocol-v2.md`, `architecture.md`,
`overview.md`, `kinematics-model.md` updates are ticket 008's concern,
since those quote identifiers renamed by tickets 002-007, none of which
exist yet when 001 runs.
