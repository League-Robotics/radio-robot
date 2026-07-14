---
status: pending
---

# Restore the line sensor and color sensor as ticked devices that report through the blackboard

> **Approved plan**: the stakeholder-approved sprint plan for this issue lives
> in `sprint-095-restore-line-color-sensors-as-ticked-blackboard-devices.md`
> (plan-to-issue hook, 2026-07-09). The two files are one work item — the
> sprint that plans this must claim BOTH via `link_sprint_issues`.

## Description

Add the line sensor and the color sensor back into the new tree as real,
reporting devices. Each sensor is ticked by the main loop and copies its
state into the blackboard so it can be reported over the wire (TLM/GET).

Stakeholder directive (2026-07-09): "adding back in the color sensor and
the line sensor as devices that can report. You're going to tick them.
They're going to copy their state into the blackboard."

## Current state

- The faceplates already exist in the new tree but are **declared, not
  defined** — no concrete leaves anywhere:
  - `source/hal/capability/line_sensor.h` — `Hal::LineSensor` (read-only:
    no Command/apply channel; `read()` returns the whole
    `msg::LineSensorState`; `configure()` + `tick(now)`).
  - `source/hal/capability/color_sensor.h` — `Hal::ColorSensor` (read-only;
    scalar primitive getters `r()/g()/b()/c()/connected()`; `configure()` +
    `tick(now)`).
- The messages already exist and are host-safe PODs
  (`source/messages/sensors.h`, generated from `protos/sensors.proto`):
  `msg::LineSensorState` (raw[4]/normalized[4]/stamp/connected),
  `msg::ColorSensorState` (r/g/b/c/stamp/connected), plus the two Config
  messages.
- `Rt::Blackboard` (`source/runtime/blackboard.h`) has **no sensor state
  cells** — any addition must meet the host-safe-POD bar documented in its
  file header (the msg types already do).
- `Subsystems::Hardware` (`source/subsystems/hardware.h`) exposes motors
  and an odometer only — no `lineSensor()` / `colorSensor()` seams.
- Nothing ticks the sensors and TLM has no `line=` / `color=` fields in
  the new tree.

## Source material

- Old-tree real drivers to port: `source_old/hal/real/LineSensor.{h,cpp}`
  and `source_old/hal/real/ColorSensor.{h,cpp}` (I2C devices; both
  previously reported via TLM `line=`/`color=` and the `LS`/`CS` verbs).
- Old-tree sim leaves: `source_old/hal/sim/SimLineSensor.{h,cpp}`,
  `source_old/hal/sim/SimColorSensor.{h,cpp}`.

## Constraints / notes

- **Stakeholder WIP in `source/main.cpp` (uncommitted, 2026-07-09)**
  sketches the seams this issue lands in: a `hardware_main()` split, a
  `// Ticks` section, and a `// Commit state to the blackboard` section.
  Build on that sketch; do not clobber it.
- Sensors share the I2C bus with the Nezha brick and OTOS — sampling
  cadence must respect the flip-flop scheduling / bus-safety lessons from
  sprints 087–094 (no blocking spins that wedge the fiber, no IRQ-mask
  storms that drop serial RX).
- Wire keys (`line=`, `color=`, verb names) are wire-visible strings —
  reuse the old tree's vocabulary unless there is a reason not to.
- Bench gate applies (`.claude/rules/hardware-bench-testing.md`): sensors
  must be seen alive on the stand — 4 plausible, changing line channels
  and plausible RGBC — before the sprint is done.
