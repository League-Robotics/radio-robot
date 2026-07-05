---
id: '001'
title: 'Remove get_* accessors from generated message headers: generator, regen, full
  call-site sweep, and regression guard'
status: open
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: remove-generated-get-accessors.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Remove get_* accessors from generated message headers: generator, regen, full call-site sweep, and regression guard

## Description

`scripts/gen_messages.py` emits protobuf-style `get_*` accessors on every
generated `msg::` struct in `source/messages/*.h`. Every one is a trivial
pass-through to an already-public struct field (verified: the oneof-kind
discriminator field, the `Opt<T>` field, and every scalar/message/enum/
string field are all plain public members with no invariant or computation
behind them) — so the Google-style-correct shape is no accessor at all,
direct field access. This ticket removes the getter-emitting branches of
`_emit_message`, regenerates the headers, and converts every real call site
(added since by sprints 078/079 — the original issue's "no call sites"
premise is stale) from `.get_foo()` to `.foo`.

**Generator change** (`scripts/gen_messages.py`, `_emit_message`, the
`# --- getters ---` block): remove the six branches that emit a
`get_`-prefixed method — the oneof-kind discriminator
(`get_<oneof>_kind()`), an `Opt<T>` field (`get_<field>()` returning
`const Opt<T>&`), an optional string field (`get_<field>()` returning
`const char*`), a message-typed field, an enum-typed field, and a plain
scalar field. **Do not touch**: chainable setters (`set<Field>(...)`,
`Command`/`Config` types only), the oneof union/kind-enum machinery, or the
repeated-field array accessor pair (`{field}()` / `{field}_count_val()`) —
those are already bare-name (no `get_` prefix) and exist only because the
backing array member is suffixed `{field}_` to avoid a name collision; they
are out of scope for this sprint.

**Regenerate**: `uv run python3 scripts/gen_messages.py` (same invocation
`build.py` runs automatically before every firmware build, and the
`build-sim` justfile recipe runs before the host sim library). All 9
headers under `source/messages/` are rewritten; `bridges.h` (hand-authored
static text, not proto-derived) is unaffected in content.

**Call-site sweep** — the real work of this ticket. Verified by grepping
`source/` and `tests/` for `\.get_[a-z_]*\(`, excluding `source/messages/`
itself. **54 call sites across 6 files**, every one a direct `.get_foo()`
call (no `->` arrow form, no pointer-to-member use) on a field that
converts 1:1 to a plain field read:

| File | Getter(s) called | Count |
|---|---|---|
| `source/hal/capability/motor.h` | `get_control_kind`, `get_feedforward`, `get_reset_position` | 5 |
| `source/subsystems/communicator.cpp` | `get_radio_channel` | 1 |
| `source/subsystems/drivetrain.cpp` | `get_control_kind`, `get_speed`, `get_standby`, `get_trackwidth`, `get_sync_gain`, `get_velocity`, `get_left_port`, `get_right_port`, `get_position` | 20 |
| `source/commands/dev_commands.cpp` | `get_control_kind` | 6 |
| `tests/sim/unit/drivetrain_harness.cpp` | `get_control_kind` | 4 |
| `tests/sim/unit/dev_command_outbox_harness.cpp` | `get_control_kind`, `get_standby`, `get_reset_position` | 18 |

Conversion is uniform: `x.get_foo()` → `x.foo`, except the oneof-kind
discriminator whose backing field is named `foo_kind` (e.g.
`command.get_control_kind()` → `command.control_kind`). `Opt<T>` fields
keep using `.has`/`.val` on the field itself exactly as they do today
through the getter (e.g. `command.get_feedforward().has` →
`command.feedforward.has`).

**False-lead correction** (do not chase these — they are unrelated Python
identifiers, not generated C++ message accessors, confirmed by direct
grep): `get_nowait` (Python stdlib `queue.Queue.get_nowait()`, called from
`tests/bench/velocity_chart.py` / `tests/bench/dev_exercise.py`); `get_ver()`
(a Python wire-protocol test helper); `get_robot_config()` (a hand-written
Python function in `host/robot_radio/config/robot_config.py`). None of the
three are touched by this ticket.

**Regression guard**: add a new test asserting no message the generator
emits defines a `get_*`-prefixed method, so a reintroduced getter branch
fails `uv run python -m pytest` instead of silently reappearing. Implement
as a pytest test that invokes the generator (in-process, or via its
`--dry-run` text output) and scans the emitted text for a
`get_[a-z_]*\(` method-defining pattern — not a build-time-only assertion
inside `gen_messages.py` itself, and not a grep lint restricted to the
checked-in headers alone (see architecture-update.md Decision 3 for the
full rationale). Place it at `tests/unit/test_gen_messages_no_getters.py`,
following this project's convention of using `tests/unit/` for
generator/tooling-level checks that are not sim/bench/playfield-scoped.

This ticket is atomic by necessity (architecture-update.md Decision 2): the
moment the generator stops emitting getters and headers regenerate, every
one of the 54 call sites fails to compile simultaneously — there is no
buildable checkpoint between "regen" and "sweep" to split across tickets.

## Acceptance Criteria

- [ ] `scripts/gen_messages.py`'s `_emit_message` no longer emits any
      `get_*`-prefixed method for any field shape (oneof-kind, `Opt<T>`,
      message, enum, string, plain scalar).
- [ ] Chainable setters, the oneof union/kind-enum machinery, and the
      repeated-field array accessors (`{field}()` / `{field}_count_val()`)
      are unchanged.
- [ ] All 9 files under `source/messages/` are regenerated and committed.
- [ ] `grep -rn "get_[a-z_]*(" source/messages/` returns nothing.
- [ ] `grep -rn "\.get_[a-z_]*(" source/ tests/ --include=*.cpp --include=*.h`
      (excluding `source/messages/`) returns nothing — all 54 call sites
      listed above converted to direct field reads.
- [ ] New guard test `tests/unit/test_gen_messages_no_getters.py` exists,
      passes against the sprint's regenerated generator output, and fails
      if a `get_*` branch is manually reintroduced into `_emit_message`.
- [ ] `just build` is green for **both** `ROBOT_DEV_BUILD` forks —
      `dev_commands.cpp` is the only fork-gated file among the six
      (`#if ROBOT_DEV_BUILD`), so its 6 call sites only compile under the
      dev fork; the other 5 files compile in both forks.
- [ ] `uv run python -m pytest` is fully green, including the compiled
      `tests/sim/unit/drivetrain_harness.cpp` and
      `tests/sim/unit/dev_command_outbox_harness.cpp` binaries (driven by
      `test_drivetrain.py` / `test_dev_command_outbox.py`).
- [ ] No wire/schema/field-layout change — `source/messages/bridges.h`'s
      static_asserts are unaffected; `docs/design/message-inventory.md`
      content (if regenerated) is unchanged since it maps fields, not
      accessor methods.

## Testing

- **Existing tests to run**: full suite, `uv run python -m pytest` — this
  sprint's change must not alter any test's pass/fail outcome, since it is
  a pure API-narrowing with zero runtime behavior change.
- **New tests to write**: `tests/unit/test_gen_messages_no_getters.py` — a
  regression guard, not a behavioral test (see Description).
- **Verification command**: `just build` (both `ROBOT_DEV_BUILD` forks)
  and `uv run python -m pytest`.

## Implementation Plan

**Approach**: edit the generator first (remove the six `get_*`-emitting
branches, leave everything else in `_emit_message` untouched), regenerate
`source/messages/*.h`, then sweep `source/` and `tests/` in the same pass —
the tree will not compile in between, so this lands as one commit. Add the
guard test last and run the full build + test suite to confirm.

**Files to modify**:
- `scripts/gen_messages.py`
- `source/messages/motor.h`, `drivetrain.h`, `communicator.h`, `common.h`,
  `gripper.h`, `planner.h`, `ports.h`, `sensors.h` (regenerated — never
  hand-edited directly; re-run the generator)
- `source/hal/capability/motor.h`
- `source/subsystems/drivetrain.cpp`
- `source/subsystems/communicator.cpp`
- `source/commands/dev_commands.cpp`
- `tests/sim/unit/drivetrain_harness.cpp`
- `tests/sim/unit/dev_command_outbox_harness.cpp`

**Files to create**:
- `tests/unit/test_gen_messages_no_getters.py`

**Testing plan**: `just build` under both `ROBOT_DEV_BUILD` values (0 and
1) to cover `dev_commands.cpp`'s fork-gated call sites; `uv run python -m
pytest` for the full host suite including the compiled harness binaries.

**Documentation updates**: none required — no public/wire-facing doc
(`docs/protocol-v2.md`, `docs/design/message-inventory.md`, etc.)
references `get_*` message-accessor method names; the inventory doc maps
proto fields to existing symbols, not accessor methods, so its content is
unaffected even if regenerated for hygiene.
