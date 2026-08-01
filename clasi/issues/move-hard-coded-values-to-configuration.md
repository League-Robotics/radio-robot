---
status: pending
---

# Move hard-coded values, and all similar values, to configuration

Stakeholder request (2026-07-31, mid-sprint-128): "move these, and all
similar values, to configuration."

Hard-coded configuration values sitting in the middle of logic are one of
the stakeholder's named cohesiveness complaints for this codebase: an
agent drops a magic number or a manually-set tuning/config constant
inline where it executes, instead of routing it through the owning
config surface (`robot_config` / `data/robots/*.json` /
`config/dotconfig.yaml` / `boot_config`), and the value then drifts
invisibly.

## Scope

Sweep the tree for inline constants that are really configuration and
move them to the appropriate config surface. Known starting points
(context at capture time, during sprint 128):

- `src/firm/main.cpp` constants — already tracked separately by
  `main-cpp-constants-move-to-robot-config.md` (excluded from sprint 128
  scope by stakeholder direction; this issue generalizes it and should be
  reconciled with it at planning time rather than duplicating it).
- Sprint-128 finds of the same shape: values noticed inline during the
  cleanup tickets (e.g. GUI panel parameters like spinbox
  decimals/thresholds in `testgui/__main__.py`, staleness thresholds such
  as `_STALE_AFTER_S` in `telemetry_panel.py`, retry counts/timeouts in
  halt paths) — anywhere a tuning value is buried mid-function rather
  than declared as a named constant at module/class scope or sourced from
  robot/app config.
- "All similar values": the sweep should grep for numeric literals in
  logic across `src/firm` and `src/host` and classify each as (a) true
  config → move to config file/registry, (b) named constant → lift to a
  `k`-constant / module-level constant with a `// [unit]` tag, or
  (c) genuinely local math — leave alone.

## Acceptance sketch

- Each moved value has one declared home (config registry, robot JSON,
  or a named constant) and the code reads it from there.
- No behavior change: moved values keep their current defaults.
- The sweep records, per file touched, which values were moved vs left
  and why.
