---
id: 078
title: 'Motor write-path armor: zero-dwell reversal, deadband, guarded resets, qualified
  wedge reporting'
status: ticketing
branch: sprint/078-motor-write-path-armor-zero-dwell-reversal-deadband-guarded-resets-qualified-wedge-reporting
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- armor-motor-write-path-against-reversal-latch.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 078: Motor write-path armor: zero-dwell reversal, deadband, guarded resets, qualified wedge reporting

## Goals

Ship the proven reversal-latch fix into the greenfield tree's motor write path
(`source/hal/nezha/nezha_motor.cpp`), per the production guidance in
`docs/knowledge/2026-07-04-encoder-wedge.md`:

1. Two-phase (zero-dwell) reversal in `NezhaMotor::writeDuty()` — ≥50 ms hold
   at commanded zero on every sign change (ship 100 ms, `MotorConfig` knob).
2. Output deadband so PID sign-dither cannot request flips.
3. Standstill-guarded hard encoder resets + soft rebaseline fallback.
4. Motion-qualified wedge reporting (keep the internal detector unconditional).
5. Friction-rig regression soak: 0 motion-armed latches over ≥100 hot flips,
   controls bracketed.

## Problem

The wedgelab campaign root-caused the dominant encoder-wedge flavor (the
reversal latch) to hot H-bridge sign flips written to 0x60 while the motor is
under way. Sprint 077 ported the old write path unchanged: the reversal
exemption writes sign flips immediately, the new embedded per-motor velocity
PID generates sign-dither at every decel/stop, and `hardResetEncoder()` runs
regardless of motion. The proven trigger is fully present one layer closer to
the metal.

## Solution

Armor the write layer only — dwell and deadband are per-motor state in the
write path; subsystems above (PID, Drivetrain, processor) never know. Gate
hard resets on verified standstill with a soft-rebaseline fallback (port of
`source_old` `rebaselineSoft`). Add a motion-qualified wedge-SUSPECT signal for
reporting. Validate on the friction rig via the DEV protocol with A/B controls.

**Placement (stakeholder correction, 2026-07-04): the armor is motor-generic
policy and lives at the `Hal::Motor` level** (`source/hal/capability/motor.h`),
implemented once the same way `apply()`/`state()` already are — NOT inside
`NezhaMotor`. The reversal dwell, output deadband, standstill reset guard, and
wedge detection/qualification apply to any motor leaf (future SimMotor/
MockMotor, other vendors). Leaves supply only the device-specific primitives:
the actual duty write to hardware, the atomic hard-reset burst, encoder
sampling. Nezha-only mechanics (the 40 ms write throttle, ±25 slew shaping for
the brick) stay in the leaf beneath the generic policy.

## Success Criteria

Acceptance sketch of the issue, items 1–5: dwell + deadband enforced and
unit-tested off-hardware via write-decision inspection; no atomic reset burst
mid-motion; qualified wedge reporting documented in `docs/protocol-v2.md`;
clean friction-rig soak with the armor on; knowledge doc updated to
shipped-in-new-tree.

## Scope

### In Scope

- `source/hal/capability/motor.h` — the generic armor policy (dwell, deadband,
  reset guard, wedge detection/qualification), implemented once in the base.
- `source/hal/nezha/nezha_motor.{h,cpp}` — refit to expose hardware primitives
  under the base-class policy; `MotorConfig` dwell/deadband fields.
- `docs/protocol-v2.md` §16 DEV semantics updates.
- Host-side unit tests (HOST_BUILD scripted-bus harness) + `tests/bench/`
  soak/reset-guard scripts.
- Knowledge-doc status update.

### Out of Scope

- The IRQ guard (flavor 2) — already ported, default ON; do not regress.
- `source_old/` — old tree keeps its behavior.
- Wedgelab itself.
- The flip-flop/split-phase scheduler and lazy clearance timers (sprint 079 —
  this sprint is its co-requisite and lands first).

## Test Strategy

Off-hardware: unit tests over scripted command sequences inspecting write
decisions (dwell window, deadband suppression, immediate stop exemption,
deferred/soft reset). On-hardware: friction-rig hot-flip soak (rig ports 3/4,
±30–50% duty, n≥100) with motion-armed latch detection from state polling,
bracketed A/B (legacy config vs 100 ms dwell), mid-motion reset-guard check.
Record CSVs + transcripts; end sessions with DEV STOP. Standing bench gate per
`.claude/rules/hardware-bench-testing.md`.

## Architecture Notes

- **Armor at the `Hal::Motor` level, not the leaf** (stakeholder, 2026-07-04):
  reversal dwell, output deadband, standstill reset guard, and wedge
  detection/qualification are generic motor policy in the faceplate base
  class, shared by every leaf; `NezhaMotor` keeps only hardware primitives
  (bus duty write, atomic reset burst, encoder sampling) plus Nezha-only
  write shaping (40 ms throttle, ±25 slew).
- Dwell is write-path state, NOT PID or Drivetrain state (design decision;
  tick-model Case 3 depends on this locality).
- Stop (`pct == 0`) stays immediate and unclamped.
- `source/hal/capability/` is headers-only per sprint 077's build acceptance —
  the planner decides whether the policy stays inline-in-header or that
  constraint is formally revised.
- Sequencing constraint: this sprint must land **before** sprint 079 wires the
  flip-flop (faster PID cadence multiplies reversal-train exposure until the
  armor is in — tick-model design risk 2).

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed (`architecture_review` gate recorded
      2026-07-05, verdict APPROVE WITH CHANGES — see
      `architecture-update.md`'s Design Rationale / Open Questions for the
      carried-forward items)
- [x] Stakeholder has approved the sprint plan (`stakeholder_approval` gate
      recorded 2026-07-05, auto-approved under stakeholder-granted
      auto-approve mode; all four planner recommendations accepted as-is —
      `MockMotor` harness with the `I2CBus` scripted fake deferred to
      sprint 079, `Opt<float>` config fields, `wsus=` sibling field
      keeping `wedged=` raw, mid-motion reset always soft-rebaseline)

All three Definition-of-Ready gates are satisfied. The sprint is in
`ticketing` phase; all five tickets are created (see below).

## Tickets

All five tickets created in `tickets/`, each back-referencing
`armor-motor-write-path-against-reversal-latch.md`:

| # | Title | Depends On |
|---|-------|------------|
| [001](tickets/001-motorconfig-motorstate-schema-additions-for-the-write-path-armor.md) | MotorConfig/MotorState schema additions (protos + codegen) | — |
| [002](tickets/002-hal-motor-base-class-armor-policy-nezhamotor-leaf-refit.md) | `Hal::Motor` base-class armor policy + `NezhaMotor` leaf refit | 001 |
| [003](tickets/003-dev-protocol-wiring-protocol-v2-md-documentation.md) | DEV protocol wiring + `docs/protocol-v2.md` documentation | 002 |
| [004](tickets/004-off-hardware-policy-unit-tests-mockmotor-host-harness.md) | Off-hardware policy unit tests (`MockMotor` host harness) | 002 |
| [005](tickets/005-friction-rig-acceptance-soak-knowledge-doc-update-bench-gate.md) | Friction-rig acceptance soak + knowledge-doc update + bench gate | 003, 004 |

Tickets execute serially in the order listed.
