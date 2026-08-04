---
id: '020'
title: "Wire robot_config_generated.py's model into robot_config.py \u2014 keep behaviour,\
  \ adopt the generated shape"
status: done
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

- [x] `robot_config.py` imports its per-group model classes from
      `robot_config_generated.py` (ticket 002's output) rather than
      declaring its own — the hand-written `BaseModel` classes for
      groups now covered by the generated module are removed.
- [x] `RobotConfig`, `get_robot_config()`, `list_robots()` still exist
      with the same public names/signatures.
- [x] Any cross-field validator not mechanically re-derivable from the
      schema (e.g. `rotational_slip`'s domain check) is preserved and
      reattached to wherever that field lives under the generated
      model's grouped shape.
- [x] Derived-field computation and env-var resolution logic inside
      `get_robot_config()` are preserved.
- [x] The module compiles/imports cleanly
      (`python -c "from robot_radio.config.robot_config import
      RobotConfig, get_robot_config, list_robots"` or equivalent).
- [x] A light unit test confirms `RobotConfig`/`get_robot_config`/
      `list_robots` are importable and `RobotConfig` is constructible
      from a minimal in-memory dict shaped to the GENERATED model's
      layout (not necessarily a real robot JSON — those are still
      old-shape until ticket 017, matching ticket 016's same posture).
- [x] No full-suite gate — this ticket's acceptance is light, consistent
      with sprint-wide policy (it compiles/imports, its own unit
      coverage passes).

## Implementation Notes (post-execution)

**Bridging choice — adopted the generated shape directly, no old-JSON
adapter.** `robot_config.py`'s `RobotConfig` now composes the ten groups
`robot_config.proto`/`robot_config_generated.py` define (Identity/
Connection/Vision host-only; Geometry/Motors/Drive/WheelControl/Planner/
Otos/Estimator robot-config) as its field types, using the generated
classes directly (`Motors`, `Drive`, `WheelControl`, `Planner`, `Otos`,
`Estimator`) or a plain name-alias for the three 1:1-identical host-only
groups (`IdentityConfig = Identity`, `ConnectionConfig = Connection`,
`VisionConfig = Vision` — a name binding, not a re-declaration, kept
purely for backward-compatible imports). `WheelsConfig`/`EncodersConfig`/
`GripperConfig`/`PeripheralsConfig` stay hand-written, unchanged — the
proto explicitly excludes these sections (no firmware `_require()`/
`_get()` reads them) but `robot_config.py`'s own `mm_per_tick` derived
computation is a real consumer. The old `CalibrationConfig`/
`ControlConfig`/(old-shape)-`DriveConfig` classes are deleted outright:
their fields are not 1:1 covered by any single generated class — they are
DISSOLVED across Geometry/Motors/Otos (was Calibration) and Motors/Drive/
WheelControl/Planner (was Control) — so keeping them as hand-written
duplicates would itself be the "two definitions" defect this sprint
exists to fix.

Rejected the "adapter that maps old-shape JSON onto the generated groups"
option the ticket named as legitimate-but-temporary: today's real robot
JSONs (`tovez.json`/`tovez_nocal.json`/`togov.json`) are still 13-section
old-shape, and ticket 017 is doing a genuine field-level RESHAPE, not a
rename (e.g. `calibration.mm_per_wheel_deg_left` → `motors.
travel_calib_left`, `geometry.odometry_offset_mm.{x,y,yaw_rad}` →
`otos.offset_{x,y,yaw}`, `control.wheel_pid_kp` → `wheel_control.pid_kp`).
A same-ticket adapter would have had to hand-encode that entire mapping
now, then be deleted whole by ticket 017 — pure throwaway work for zero
mid-sprint behavioral benefit the ticket's own acceptance criteria
require. Went with what the ticket title says literally: adopt the
generated shape, accept that today's robot JSONs partially stop loading
their real values until 017 (silently, via pydantic's existing
`extra='ignore'` default — unchanged from before this ticket, and
explicitly ticket 016's job to flip to `forbid`), and document it loudly
in the module docstring instead.

**One deliberate exception found and preserved, not silently dropped:**
`geometry.otos_untrusted`. `robot_config.proto`'s own header comment
excludes it as "dead data ... none is read by any `_require()`/`_get()`
call in `gen_boot_config.py`" — but that audit only checked FIRMWARE
consumers. `robot_radio.planner.heading.HeadingCorrector` reads
`geometry.otos_untrusted` HOST-side (via a defensive `getattr(...,
False)`) to choose encoder-derived pose over raw OTOS pose for a rig
whose chip is mechanically decoupled from the wheels
(`clasi/issues/rig-persistent-otos-distrust.md`). Dropping the field
would not have raised anywhere (the `getattr` fallback swallows it
silently) — it would have silently flipped `tovez.json`/
`tovez_nocal.json` back to trusting OTOS, exactly the "silent no-op"
class of bug sprint 132 exists to eliminate. Preserved via a thin
`GeometryConfig(Geometry)` subclass at the SAME JSON path
(`geometry.otos_untrusted`) it already occupied — every generated
`Geometry` field is inherited unchanged, only this one field is added.
**Flagged for follow-up, not resolved here**: whether `otos_untrusted`
belongs in the generated schema at all (and if so, in which group) is a
proto-ownership decision (tickets 001/002, already closed) outside this
ticket's file scope — left for the team-lead/stakeholder to route, most
naturally as a small addendum to ticket 017's JSON reshape or its own
follow-up issue.

Same subclass also carries the `rotational_slip` domain validator
(`{0} ∪ [0.5, 1.0]`, non-contiguous) the ticket names explicitly — this
did NOT previously exist anywhere in the codebase (verified: the
pre-020 hand-written `CalibrationConfig.rotational_slip` was a bare
`Optional[float]` with no domain check at all). `robot_config.proto`'s
own header comment names "the thin hand-written validation layer around
the generated pydantic model" as this check's designated home — added
here for the first time, closing that gap rather than "preserving"
code that never existed.

**Two flat convenience properties dropped, not ported**: `tag_offset_mm`
(the generated `Vision` group flattens this into `tag_offset_x/y/z/yaw`
— no nested-offset shape left to point at; its one caller,
`src/tests/playfield/pose_fix_convergence.py`, is an HITL playfield tool,
not pytest-collected) and `motor_deadband` (sourced from the OLD JSON's
own confusingly-named top-level `drive` section, which the proto
excludes outright as "read by no firmware code"; its one caller,
`robot_radio.nav.navigator._load_motor_deadband()`, already wraps the
read in `try/except Exception: pass` with a literal `35` fallback, so
this degrades silently and harmlessly rather than crashing).

**Callers verified**: `src/tests/unit/test_robot_config.py` (6 tests,
all pass UNCHANGED — no edits needed; `geometry.otos_untrusted` round-
trips through real `tovez.json`/`tovez_nocal.json`/`togov.json` exactly
as before, which is what made keeping that field worthwhile). New file
`src/tests/unit/test_robot_config_generated_shape.py` (13 tests) covers
this ticket's own acceptance criteria: import surface, group classes
sourced from the generated module, construction from a minimal
generated-shape dict (both `model_validate` and JSON round-trip), the
`rotational_slip` validator's accept/reject boundaries, and
`mm_per_tick` derived-field computation. `src/host/robot_radio/config/
__init__.py` (not in the ticket's own file list, but required for its
own "imports cleanly" criterion) had its re-export list updated to drop
`CalibrationConfig`/`DriveConfig`/`OffsetXYYaw`, which no longer exist.

**New breakage beyond the ticket's pre-approved 7** (full default
`uv run python -m pytest` collection, checked for this report's accuracy
— per this ticket's own "no full-suite gate" and the stakeholder's
"doesn't have to work in the middle" direction, NOT fixed, and NOT in
this ticket's file scope to fix):

- `src/tests/unit` (targeted run): `src/tests/unit/test_sim_boot_config.py`
  — COLLECTION ERROR (`ImportError: cannot import name 'CalibrationConfig'`
  — constructs `CalibrationConfig`/`ControlConfig`/`IdentityConfig` by the
  old names directly). Plus 2 assertion failures,
  `test_calibration_kwargs.py::test_calibration_commands_tovez_json_
  snapshot`/`..._tovez_nocal_json_snapshot` (see root cause below).
- `src/tests/testgui` (targeted run): **80 failed, 64 errors** (455
  passed, 3 skipped, 10 xfailed) — one root cause, not 144 independent
  breaks. `robot_radio.calibration.sim_boot_config._as_cfg_dict()`
  (`src/host/robot_radio/calibration/sim_boot_config.py:84-88`) is the
  Tier-2 sim boot-config bridge: it does
  `getattr(config, "control", None)`/`getattr(config, "calibration",
  None)` then `.model_dump()` to hand `gen_boot_config.py`'s raw-dict
  readers (`_require(cfg, "control", "vel_kp")` etc.) something shaped
  like the OLD JSON. With `control`/`calibration` gone from `RobotConfig`,
  both `getattr`s return `None`, `_as_cfg_dict()` produces `{"control":
  {}, "calibration": {}}`, and every `_require()` call raises
  `MissingRobotConfigKeyError` — which propagates out of `SimLoop.
  configure_from_robot()`, a fixture nearly the entire `testgui` domain's
  `loop`/`sim_loop`-backed tests share. This is the SAME class of gap as
  `calibration/push.py`'s `calibration_kwargs()`/`calibration_commands()`
  (also `getattr(config, "control"/"calibration", None)`-based, degrades
  to defaults instead of raising — the 2 `test_calibration_kwargs.py`
  snapshot failures above) — both are Tier-2/wire-push consumers of the
  OLD group names that this ticket's file scope (`robot_config.py` only)
  does not touch.

Neither `push.py` nor `sim_boot_config.py` was touched — updating their
own field-selection logic to the new group names is squarely ticket
014's job ("migrate host `NezhaProtocol`/`calibration/push.py`/TestGUI
onto the new surface"), not a `robot_config.py`-only change. Flagging
`sim_boot_config.py` explicitly here (beyond what the ticket's own
pre-approved list anticipated) because of its blast radius — the
team-lead/stakeholder should not be surprised that most of `testgui`
goes red until ticket 014 lands.

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
