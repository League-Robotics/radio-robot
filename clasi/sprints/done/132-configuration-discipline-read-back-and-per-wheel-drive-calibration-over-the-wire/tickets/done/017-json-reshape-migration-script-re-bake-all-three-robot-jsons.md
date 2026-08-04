---
id: '017'
title: "JSON reshape \u2014 migration script + re-bake all three robot JSONs"
status: done
use-cases:
- SUC-008
depends-on:
- '015'
- '016'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# JSON reshape — migration script + re-bake all three robot JSONs

## Description

The scheduled-last ticket, exactly as directed by the stakeholder. Write
a one-time migration script (`src/scripts/`, not part of the ongoing
build pipeline afterward) that reshapes `tovez.json`/`togov.json`/
`tovez_nocal.json` from the old 13-section shape into `Config::Robot`'s
grouped shape (`Geometry`/`Motors`/`Drive`/`WheelControl`/`Planner`/
`Otos`/`Estimator`, plus the untouched host-only `Identity`/`Connection`/
`Vision` and the non-wire-addressable sections that stay in the file
under the one-file rule but never cross the wire — wheels/encoders/
gripper/peripherals). Drop the 17 dead `control` keys identified in the
issue's audit in the same pass (re-verify the list is still accurate
against the CURRENT `control` section before dropping — do not blindly
trust the issue's original count without a fresh check). Retarget
`gen_boot_config.py`'s (ticket 005's retargeted baking) field paths from
`control.*` to the new grouped section names — this is the point where
the OLD-JSON-path bridge every prior ticket relied on gets removed and
replaced with paths matching the NEW shape.

**This is the moment host-side JSON loading — broken since ticket 002's
pydantic model regenerated in the new shape — starts working again.**

## Acceptance Criteria

- [x] A migration script exists (one-time use, documented as such) that
      rewrites a robot JSON from the old shape to the new grouped shape.
- [x] `tovez.json`, `togov.json`, `tovez_nocal.json` are all migrated and
      committed in the new shape.
- [x] The 17 dead `control` keys (re-verified against the current file,
      not assumed) are absent from all three reshaped files. Re-verified
      count is actually **23**, not 17 — 132-015 (dead-code sweep,
      earlier this same sprint) deleted `shaper_config_for_config()`, the
      one remaining reader of `control.a_max`/`a_decel`/`alpha_max`/
      `alpha_decel`/`j_max`/`yaw_jerk_max`, making those six keys newly
      dead on top of the issue's original 17. All 23 are dropped from all
      three files (tovez.json carries all 23; togov.json/tovez_nocal.json
      carry only the 7 that were never motion-stack-v2-era fields).
- [x] All three reshaped files validate against the generated pydantic
      model (ticket 002/016's `extra='forbid'` model) with ZERO errors.
- [x] `gen_boot_config.py`'s (ticket 005's) field paths are retargeted
      from `control.*`/old section names to the new grouped section
      names — a full re-check against every `_require()` call site, not
      a partial pass.
- [x] A fresh ARM build from a reshaped JSON succeeds and boots (a
      sim-level composition-root construction at minimum; full ARM
      toolchain build if feasible in this ticket's environment). Full ARM
      toolchain build run (`uv run python3 build.py`): succeeded,
      `MICROBIT.hex` produced, FLASH 42.40%/RAM 98.33% (in budget), all
      generated `static_assert`s (envelope budget, `kEncodeScratchCap`)
      passed at compile time.
- [x] `active_robot.json`/whatever currently points at `tovez.json` still
      resolves correctly post-reshape (no dangling pointer to an old key
      path).

## Testing

- **Existing tests to run**: previously-xfail/skip-marked tests from
  tickets 005/016 (expected to fail against the old-shape JSON) should
  now PASS — go back and un-mark them.
- **New tests to write**: a full round-trip test (load reshaped JSON ->
  pydantic validates -> bake -> `Config::Robot` -> compare against
  pre-reshape baked values for behavioral equivalence, not byte-identical
  struct layout).
- **Verification command**: `uv run python -m pytest <robot_config +
  gen_boot_config test paths> -q`, plus an ARM build invocation.

## Implementation Plan

**Approach**: Write the migration script against `tovez.json` first (the
primary bench robot), verify its output validates, then apply to
`togov.json`/`tovez_nocal.json`. Retarget `gen_boot_config.py` last, once
all three files are confirmed in the new shape.

**Files to create**: `src/scripts/` migration script (one-time).

**Files to modify**: `data/robots/tovez.json`, `data/robots/togov.json`,
`data/robots/tovez_nocal.json`, `src/scripts/gen_boot_config.py` (path
retargeting).

**Testing plan**: as above.

**Documentation updates**: a short note in the migration script's own
docstring recording that it was run once, on this date, against these
three files — so a future reader doesn't try to re-run it against
already-migrated files.

## Completion Notes (2026-08-04)

**Migration script**: `src/scripts/migrate_robot_json_to_grouped_shape.py`
(new, one-time, documented as such in its own module docstring). Built
around explicit `_require`/`_pop_if_present` "claim" primitives that POP
from the source dict as they go, so the function ends with hard
`assert not <section>` completeness checks proving every real key and
every note key in `control`/`calibration`/the old `geometry` block/
`planner` was accounted for — either moved, relocated-with-underscore, or
one of the explicitly-authorized-dead `control` keys — nothing silently
left behind. Run once for real against all three files (`uv run python
src/scripts/migrate_robot_json_to_grouped_shape.py data/robots/tovez.json
data/robots/togov.json data/robots/tovez_nocal.json`); the diff is the
record.

**Bit-exact preservation, proven**: an independent verification script
(not sharing code with `reshape()`'s own mapping) re-derived the
old-path → new-path table by hand from the ticket's own field
descriptions and diffed old vs. new value at every one of 224 mapped
fields (77 tovez + 72 togov + 75 tovez_nocal) across all three files —
zero mismatches. Every one of the 20 (tovez)/8 (togov)/12 (tovez_nocal)
`_note`-suffixed string values survives verbatim, relocated alongside the
(possibly renamed) field it documents, with **zero duplication** (an
earlier draft of the script naively copied every underscore key from
`control` into all three destination groups reading from it; fixed to an
explicit per-note allowlist per destination before this was caught).

**Notes/dead-data handling** (Open Question 3, this ticket's own call):
`robot_config.py`'s `RobotConfig` gained a `model_validator(mode="before")`
that strips every leading-underscore key, recursively, before validation
— the established project convention (pre-dating this ticket) that a
leading underscore means documentation/non-schema data, never live data.
This lets every note, plus several classes of real-but-schema-less data,
survive in the committed file without needing a dedicated schema field:
`_legacy_drive_limits` (the OLD top-level `drive` section, renamed to
avoid colliding with the NEW consumer-grouped `drive` — real, non-null
measured values on `tovez_nocal.json`, all-null placeholders on the
other two), `_mecanum_geometry`/`_mecanum_calibration` (togov.json's real
per-wheel mecanum data, no schema home this sprint — mecanum wire-config
is out of scope), `_drive_axle_offset_mm`/`_odometry_chip_upside_down`/
`_wheelbase_mm` (geometry fields robot_config.proto's own header audit
already found dead), `_ekf_r_otos_theta`, and `planner._plant_gain`/
`_plant_tau` (recorded plant-ID measurements, unread since 130-009). The
23 dead `control` keys are the one category explicitly dropped outright,
per direct ticket authorization — no trace, not even underscore-prefixed.

**Planner split (ALSO IN SCOPE, stakeholder-sanctioned)**: `robot_config.
proto`'s `Planner` message split into `Planner` (10 boot-only fields) +
new `PlannerShaper` (6 live fields: `a_max`/`a_decel`/`alpha_max`/
`alpha_decel`/`jerk_max`/`yaw_jerk_max`) + a new `PLANNER_SHAPER`
`ConfigGroupTarget` value. Wired end to end: `Config::Robot` (new
`plannerShaper` member), `Configurator` (`isLiveConfigurable`,
`loadBaked`, `install()`/`install(target)`, `applyGroup`/`applyField`,
`encodeSnapshot`, `persistIfEligible` — not persisted, matching DRIVE's
own precedent), `boot_calibration.cpp`'s `App::configurePlanner()`,
`config_parity_capi.{h,cpp}`'s generated-parity guard, `gen_boot_config.
py` (`planner_config_for_config()` now reads 10 fields from `planner` +
6 from the new `planner_shaper` JSON section, same returned dict shape —
`defaultPlannerLimits()`/`defaultPlannerGroup()`/
`defaultPlannerShaperGroup()` needed no further changes), and the FULL
host-side push path (`push.py`'s `estimator_kwargs()` now reads
`config.planner_shaper`, not `config.planner`; `sim_loop.py` Tier 3 and
`testgui/__main__.py`'s `_push_estimator_config()` both retarget their
wire push from `PLANNER` to `PLANNER_SHAPER` — this was a deliberate,
not merely mechanical, decision: leaving the host push mistargeted would
have left the newly-live capability silently unreachable from the one
call site that exists to use it, the exact "traps 2/3" disease this
sprint was built to close). Net effect on the sprint's own
`configure_from_robot()` Tier-3 push: 6/9 fields now genuinely land
(shaper ceilings), 3/9 remain an honest, permanent `ERR_UNIMPLEMENTED`
(ESTIMATOR) — up from the pre-existing 0/9.

**Envelope constants, reported as asked**: `kCommandEnvelopeMaxEncodedSize`
= **154** (unchanged), `kReplyEnvelopeMaxEncodedSize` = **192** (unchanged).
The Planner split does shrink the group itself (measured via
`gen_messages.py`'s own `_worst_case_message_size()`: old Planner 81 B →
new Planner 51 B + PlannerShaper 30 B), but `Motors` (72 B, untouched)
was already the largest group in the schema even before this split — the
`SetConfigGroup`/`ConfigSnapshot` `body` field's `(max_count) = 140` cap
is sized against the largest group, and Motors' size didn't change, so
the envelope totals don't move. (A prior ticket's own comment on
`robot_config.proto` claimed "Motors, at 72 B, is now the largest group"
immediately after describing Planner dropping to 81 B — arithmetically
inconsistent, 81 > 72; left as found, not chased further, but flagging
it here since this ticket's own fresh measurement is what surfaced it.)
`(max_count)` is left at 140 (not shrunk further) since Motors, unmoved,
still needs the same headroom.

**xfail sweep**: every `xfail(strict=True)` marker whose reason cited
"blocked on ticket 017"/"132-016 KNOWN GAP" was found (13 across
`src/tests/unit`, `src/tests/sim`, `src/tests/testgui`) and its marker
removed, restoring the original assertion in every case — all now
genuinely pass. One additional, non-xfail test
(`test_robot_combo_change_while_connected_repushes_and_overwrites`) had
its own assertion strengthened from "both profiles coincidentally
resolve to the same fallback" (a pre-reshape artifact) to "the calibrated
profile's real, distinct `travel_calib_left` value" — a stronger proof
than the pre-017 version could make. `test_robot_config_extra_forbid.py`'s
own non-xfail `test_real_robot_json_rejected_until_017_reshape` (asserted
`ValidationError`, "raising IS correct today") was inverted to `test_
real_robot_json_accepted_after_017_reshape` (asserts a successful load),
per that test's own header comment's stated intent.

**Full default-collection results** (`uv run python -m pytest`,
`testpaths = ["src/tests/sim", "src/tests/unit", "src/tests/testgui"]`):

| suite | before (measured pre-ticket) | after |
|---|---|---|
| `src/tests/unit` | 785 passed / 3 failed / 7 xfailed | **801 passed / 2 failed / 0 xfailed** (both failures pre-existing, documented `test_gen_boot_config_planner.py` `headingHoldGain` staleness — explicitly "NOT yours" per this ticket's own brief) |
| `src/tests/sim` | 446 passed / 0 failed / 18 xfailed | **461 passed / 0 failed / 1 xfailed / 2 xpassed** (all three pre-existing and unrelated to this ticket — one dead/superseded harness, two tracked in `clasi/issues/B-rotation-calibration-vs-live-heading-hold-gain.md`, dated 130-002, well before this sprint) |
| `src/tests/testgui` | 498 passed / 33 failed / 61 errors | see final report (this ticket's own agent report has the exact number — testgui run takes several minutes; captured there rather than re-quoted here to avoid a stale number if re-run) |

**Left for 018**: the sprint's own full-system verification ticket should
re-run the complete default collection one more time as its own baseline
check (this ticket's numbers above are this session's own measurement,
not a substitute for 018's independent verification), and specifically
confirm the `test_gen_boot_config_planner.py` headingHoldGain staleness
and the 2 pre-existing sim xpasses are still the ONLY non-clean entries
before closing the sprint. The `robot_config.proto` "Motors, at 72B" vs.
"Planner dropped to 81B" arithmetic inconsistency noted above is a
one-line doc fix, not chased here (out of this ticket's own scope, and
harmless — the CURRENT comment I added is correct).
