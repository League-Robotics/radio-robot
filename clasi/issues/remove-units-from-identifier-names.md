---
status: pending
---

# Remove units from identifier names; document units in a standard line comment

## Description

Identifier names throughout the codebase embed physical units as suffixes —
e.g. `tgtMms`, `encMm`, `trackwidthMm`, `lastUpdMs`, `setAngleDeg`,
`odomYawDeg` in the C++ firmware, and `speed_mms`, `x_mm`, `duration_ms`,
`target_deg`, `read_ms` in the host Python. This couples the name to the
unit, makes renames necessary whenever a unit changes, and produces awkward
names like `tgtMms` and `mmPerDeg`.

**Rule:** names may describe the *kind* of quantity (target speed, target
time, target velocity, position, heading, duration, timeout) but must not
embed the *unit*. The unit is instead stated in a standard comment on the
declaration line.

Example (firmware, `source/state/OutputState.h`):

```cpp
// Before
float tgtMms[kWheelCount] = {};  // all-wheel speed targets, mm/s

// After
float tgtSpeed[kWheelCount] = {};  // [mm/s] all-wheel speed targets
```

## Scope

Everywhere — this is a codebase-wide rename:

- **C++ firmware** (`source/`): ~60+ distinct unit-suffixed identifiers
  (`Mms`, `Mm`, `Deg`, `Dps`, `Ms`, `Us`, `Pct`, `Hz`, `KHz` suffixes),
  hundreds of occurrences. Includes struct fields, locals, parameters,
  member variables (`_lastPositionMm`, `_lastTickMs`), and config fields.
- **Host Python** (`host/`, `dotconfig`, tests, tools): `_mm`, `_mms`,
  `_deg`, `_ms`, `_dps`, `_pct`, `_hz` snake_case suffixes — `read_ms`
  (121 uses), `x_mm`/`y_mm`, `speed_mms`, `duration_ms`, etc.
- Any shared protocol/telemetry field names, keeping firmware and host
  consistent with each other.

## Existing in-code FIXME markers covered by this issue

The stakeholder has already flagged these directly in the source; they are
explicit instances of this issue, and the FIXME comments should be removed
when the rename lands (see fixme-cleanup-legacy-config-and-estimatedump-enum
for the non-units FIXMEs found in the same sweep):

- `source/types/Config.h:44` — `trackwidthMm`
- `source/types/Config.h:60` — `minWheelMms`
- `source/types/Config.h:115` — `rotationOffsetDeg` (and its untagged
  sibling `rotationOffsetDegNeg` on the next line)
- `source/types/Config.h:125` — `turnThresholdMm`
- `source/types/Config.h:126` — `doneTolMm`
- `source/types/Config.h:136` — `arriveTolMm`
- `source/types/Config.h:169` — `tlmPeriodMs`
- `source/types/Config.h:185` — `lagOtosMs` ("Remove units, replace with
  'Time'" — i.e. prefer names like `lagOtosTime` that say what the
  quantity is)
- `source/types/Config.h:205` — `halfTrackMm`
- `source/state/DesiredState.h:21` — whole-struct marker: remove units
  from all `DesiredState` field names (e.g. `wheelMms`)

## Design points to settle during planning

1. **Standard comment format** — pick one convention and apply it
   uniformly, e.g. a leading `// [mm/s]` / `# [mm/s]` tag at the start of
   the declaration-line comment, so units are grep-able.
2. **Wire/protocol compatibility** — TLM/SNAP field names, SET config
   keys, and any serialized names that hosts or tools parse must either be
   renamed in lock-step on both sides or explicitly excluded from the
   rename (with rationale noted).
3. **Ambiguity resolution** — where the unit suffix was the only thing
   distinguishing two names (e.g. a `Mm` position vs. a raw-ticks
   counterpart), choose descriptive replacements (`positionLinear` vs.
   `positionTicks`, or similar) rather than dropping information.
4. **Derived-unit names** like `mmPerDeg` — rename to what the quantity
   *is* (e.g. a wheel travel-per-rotation calibration factor) with the
   unit in the comment.

## Acceptance criteria

- No identifier in `source/` or the host Python embeds a unit suffix
  (case-insensitive `mm`, `mms`, `deg`, `dps`, `ms`, `us`, `pct`, `hz`
  as a trailing name component), except any explicitly documented
  wire-compatibility exclusions.
- Every renamed declaration carries the standard unit comment.
- The chosen comment convention is documented in the coding standards.
- Build is green and the full test suite passes (renames only — no
  behavioral change).
