---
id: 008
title: 'Host-side bake action: fold learned gainTrim into duty_per_speed config'
status: open
use-cases: [SUC-007]
depends-on: ['007']
github-issue: ''
issue: 04-continuous-duty-per-speed-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host-side bake action: fold learned gainTrim into duty_per_speed config

## Description

Depends on ticket 007 — this ticket reads a telemetry field 007
introduces (`gainTrimLeft/Right`). Split out from 007 as its own ticket
because it is a genuinely different layer: firmware RAM state (007) vs. a
host-side, explicit config-write action (this ticket) — see `sprint.md`'s
Design Rationale, Decision 1, "Persistence" for the full decision trail
(RAM-only → considered flash → settled on telemetry + host-bake).

The firmware never persists `gainTrim` — it boot-resets to 1.0 every
time, always (stakeholder: "we're not ready for the complexity of storing
it," 2026-08-01). But a ~60 s learning time constant makes relearning
from scratch every boot costly in practice. The host, which already
receives `gainTrim` live in telemetry (ticket 007), is given a **deliberate,
explicit** bake action — a **CLI command** (NOT a GUI button: TestGUI is
owned by another agent right now, stakeholder 2026-08-01, so this sprint
adds nothing under `src/host/robot_radio/testgui/`) — that:

1. Reads the currently-observed `gainTrimLeft/Right` off the live
   telemetry stream.
2. Writes `duty_per_speed_left/right = duty_per_speed_left/right *
   gainTrim` into the robot's `data/robots/*.json` (the stakeholder's
   recommended mechanical form — folding the correction into the constant
   it corrects, so the learned value effectively "restarts" `gainTrim`'s
   job at 1.0 the next time this robot boots). The exact form (direct
   fold-in vs. an intermediate `learned:` block) is this ticket's own
   call, per the stakeholder's explicit deferral — but whichever form is
   chosen, it must write into the existing `data/robots/*.json` schema
   without requiring a new required key (keep the schema additive).

**This is NOT periodic auto-persistence.** A background timer writing
config files during a drive session was explicitly considered and
rejected. One user action, one bake, every time.

**Operational model to support** (record this in user-facing
docs/help text, not just code): students/operators drive WITH a host
attached first — a calibration phase where the host has plenty of time to
observe `gainTrim` converge — then the host bakes the result into config,
and subsequent autonomous/hostless runs start from that improved
baseline. Boot itself is unchanged: it starts from config, same as today.

## Acceptance Criteria

- [ ] The bake action correctly folds the currently-observed
      `gainTrimLeft/Right` into `duty_per_speed_left/right` in the
      robot's `data/robots/*.json`.
- [ ] The action is explicit — a test asserts it is never triggered by a
      periodic timer, a background thread, or any code path other than a
      direct user invocation (button click or CLI command).
- [ ] After a bake, a fresh boot's telemetry shows `gainTrim` starting at
      1.0 and `duty_per_speed` reflecting the baked-in correction (i.e.
      the net commanded-to-actual mapping is unchanged across the bake —
      baking must not itself change robot behavior, only where the
      correction "lives").
- [ ] `data/robots/*.json` remains schema-valid after a bake (no new
      required key introduced; `additionalProperties: false` sections,
      if any are touched, are respected).

## Testing

- **Existing tests to run**: `uv run python -m pytest`.
- **New tests to write**: a host unit test asserting the bake action's
  arithmetic (fold-in correctness) and its explicit-trigger-only
  property (per Acceptance Criteria); a schema-validation test on the
  post-bake JSON.
- **Bench verification**: a short calibration drive session on the stand,
  observe `gainTrim` on decoded telemetry (a bench script or `rogo`, NOT
  the TestGUI), trigger the bake action, confirm the written JSON and a
  subsequent boot's starting telemetry.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: a small, explicit, host-side CLI action — no daemon, no
  scheduler, no GUI. Implementer chooses the exact JSON write shape
  (direct fold-in recommended); document the choice.
- **Files to create/modify**: a new CLI command alongside existing
  `rogo`/`io/` config-push commands — NOTHING under
  `src/host/robot_radio/testgui/`; `data/robots/*.json` write path
  (reuse existing config-write utilities if any exist, e.g. near
  `gen_boot_config.py`'s readers, rather than hand-rolling JSON I/O).
- **Documentation updates**: user-facing note (README or CLI help
  text) describing the "calibrate with a host, bake, then run hostless"
  operational model — this is a workflow change worth documenting where
  operators will actually see it, not just in `sprint.md`.
