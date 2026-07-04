---
status: pending
---

# Remove trivial `get_*` accessors from generated message headers

## Description

`scripts/gen_messages.py` emits protobuf-style snake_case getters (e.g.
`get_kind()`, `get_ax()`) on every generated `msg` struct in
`source/messages/*.h` (emitter template around lines 643–688). These
violate the project naming convention twice over — getters are bare-name
lowerCamelCase (`kind()`, `velocity()`); function names are never
snake_case and never carry a `get_` prefix — and they are **dead API**: a
grep of `source/` and `tests/` finds no call site outside the generated
headers themselves. All consuming code reads the public fields directly.

Since the generated messages are passive data structs (public fields, no
invariants), the Google-style-correct shape is no getters at all.

## Fix

1. Stop emitting the trivial `get_*` accessors in
   `scripts/gen_messages.py` (the field-getter branches of
   `_emit_message`). Where a getter is genuinely non-trivial (e.g. oneof
   kind discriminators, `Opt<>` wrappers), either drop it too if unused
   or rename to conforming bare-name lowerCamelCase.
2. Regenerate `source/messages/*.h` and confirm the firmware and test
   builds are clean (expected: no call sites exist, so no fallout).

## Context

- The "generated code exempt" clause in
  `.claude/rules/coding-standards.md` /
  `.claude/rules/naming-and-style.md` means the *output files* are never
  hand-edited — it is not a license for the generator template to emit
  non-conforming API. The rules docs are annotated accordingly
  (2026-07-04); this issue is the follow-through in the generator.
- Stakeholder decision 2026-07-04: schedule this AFTER sprint 077
  (greenfield faceplate HAL) closes — do not fold it into 077.
