---
id: '020'
title: "Wire robot_config_generated.py's model into robot_config.py — keep behaviour,\
  \ adopt the generated shape"
status: open
use-cases:
- SUC-001
depends-on:
- '002'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire robot_config_generated.py's model into robot_config.py — keep behaviour, adopt the generated shape

## Description

**Coverage gap found during execution of ticket 002.** Ticket 002
generated the pydantic model into a NEW file,
`src/host/robot_radio/config/robot_config_generated.py`, rather than
replacing the hand-written `src/host/robot_radio/config/robot_config.py`
— correctly, since 002's own acceptance criterion only asked for the
three artifacts to "generate correctly in isolation," and
`robot_config.py` is 796 lines carrying real behaviour
(`get_robot_config()`, `list_robots()`, derived-field computation,
env-var resolution) that no ticket asked it to touch. But nothing
imports the generated module yet, and no other ticket in this sprint
owns wiring it in — left as-is, the sprint would end with a generated
model nothing imports sitting beside the hand-written one that remains
the real consumer surface, which defeats "one definition," the sprint's
whole premise.

This ticket closes that gap: `robot_config.py` keeps its behaviour
functions and public API (`RobotConfig`, `get_robot_config()`,
`list_robots()`, the resolution-order/caching singleton, env-var
resolution, derived-field computation), but its MODEL — the per-group
`BaseModel` classes — comes from `robot_config_generated.py` instead of
being hand-declared. This is exactly the split sprint.md's Design
Rationale Decision 4 already calls for: "generated struct SHAPE,
hand-written baking BEHAVIOR" (the same split the C++ side already has
between a generated struct and `gen_boot_config.py`'s hand-written
baking logic).

Any cross-field validator that is NOT mechanically re-derivable from the
schema (e.g. `rotational_slip`'s non-contiguous `{0} ∪ [0.5,1.0]` domain)
must be preserved, reattached to wherever that field now lives under the
generated model's grouped shape.

**Every existing caller of `get_robot_config()`/`list_robots()` must
keep working by the END of the sprint** (ticket 017's reshape + ticket
018's verification are where that is actually proven) — this ticket is
not required to validate against real robot JSONs, which are still in
their old shape until ticket 017. Mid-sprint breakage here is expected
and accepted, same posture as ticket 016.

## Acceptance Criteria

- [ ] `robot_config.py` imports its per-group model classes from
      `robot_config_generated.py` (ticket 002's output) rather than
      declaring its own — the hand-written `BaseModel` classes for
      groups now covered by the generated module are removed.
- [ ] `RobotConfig`, `get_robot_config()`, `list_robots()` still exist
      with the same public names/signatures.
- [ ] Any cross-field validator not mechanically re-derivable from the
      schema (e.g. `rotational_slip`'s domain check) is preserved and
      reattached to wherever that field lives under the generated
      model's grouped shape.
- [ ] Derived-field computation and env-var resolution logic inside
      `get_robot_config()` are preserved.
- [ ] The module compiles/imports cleanly
      (`python -c "from robot_radio.config.robot_config import
      RobotConfig, get_robot_config, list_robots"` or equivalent).
- [ ] A light unit test confirms `RobotConfig`/`get_robot_config`/
      `list_robots` are importable and `RobotConfig` is constructible
      from a minimal in-memory dict shaped to the GENERATED model's
      layout (not necessarily a real robot JSON — those are still
      old-shape until ticket 017, matching ticket 016's same posture).
- [ ] No full-suite gate — this ticket's acceptance is light, consistent
      with sprint-wide policy (it compiles/imports, its own unit
      coverage passes).

## Testing

- **Existing tests to run**: any existing test for `robot_config.py`'s
  public API that does not depend on the OLD JSON shape should still
  import and pass; tests that do depend on the old shape are expected to
  need updating or an explicit skip/xfail (mirroring ticket 016's
  posture), not silently left broken with no explanation.
- **New tests to write**: a minimal import/construction smoke test
  against the generated model's shape, per Acceptance Criteria.
- **Verification command**: `uv run python -c "from
  robot_radio.config.robot_config import RobotConfig, get_robot_config,
  list_robots"` plus `uv run python -m pytest <new smoke test path> -q`.

## Implementation Plan

**Approach**: Replace `robot_config.py`'s hand-written per-group
`BaseModel` classes with imports from `robot_config_generated.py`
wherever a 1:1 generated counterpart exists; keep `robot_config.py`'s own
module-level functions (`get_robot_config`, `list_robots`, the
resolution-order/caching singleton, cross-field validators) as the thin
hand-written layer sitting on top of the generated classes.

**Files to modify**: `src/host/robot_radio/config/robot_config.py`.

**Files referenced, not modified**:
`src/host/robot_radio/config/robot_config_generated.py` (ticket 002's
output).

**Testing plan**: as above.

**Documentation updates**: `robot_config.py`'s module docstring updated
to note it now composes on top of `robot_config_generated.py` rather
than declaring its own field classes.
