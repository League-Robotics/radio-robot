---
id: '002'
title: 'Wheel-frozen fault flag: telemetry bits + host decode'
status: in-progress
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: wheel-frozen-fault-flag-in-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wheel-frozen fault flag: telemetry bits + host decode

## Description

Sequenced right after ticket 001 (both touch `nezha_motor.{h,cpp}`; this
ticket is also the direct regression guard for 001's safety fix — if a
duty write is ever silently lost again, this is what makes it visible in
seconds instead of requiring another incident). Stakeholder, 2026-07-31:
*"If you've commanded an encoder and it's been commanded to move for the
last cycle and it hasn't moved, then it's frozen... that should be on the
telemetry, and if it is, then the test program should be throwing big red
errors."*

`Devices::NezhaMotor::wedgeSuspect()` — the **gated** stall detector
(commanded nonzero duty for N consecutive cycles with no encoder change)
— already exists but has zero consumers. Use `wedgeSuspect()`, not the
ungated `wedgeLatched_` (which fires on healthy moves too and would cry
wolf on every leg — this distinction matters, see the issue's own
correction note).

1. `telemetry.h` — `kFlagFaultWheelFrozenLeft = 1u << 19`,
   `kFlagFaultWheelFrozenRight = 1u << 20` (bits 19/20 are the next free
   slots — confirmed by grep against the current flag table).
2. `robot_state.h`'s `Health` struct — `wheelFrozenLeft`/`wheelFrozenRight`.
3. `robot_loop.cpp` — publish `motorL_.wedgeSuspect()`/
   `motorR_.wedgeSuspect()` into `Health` each cycle.
4. Host `protocol.py` — decode the two new flags.
5. NOT IN THIS SPRINT: the TestGUI red banner and the host drive
   loop's abort-on-flag. TestGUI is owned by another agent right now
   (stakeholder, 2026-08-01) — this ticket stops at the decoded flag on
   `TLMFrame`. The GUI consumer rides a later sprint.

## Acceptance Criteria

- [ ] Physically stall one wheel on the stand: the correct per-wheel flag
      sets within ~0.5 s and is observable on the decoded `TLMFrame`
      (a short bench script, NOT the TestGUI).
- [ ] A full healthy 700 mm leg raises **neither** flag — verify
      explicitly; a false positive here is worse than no flag at all.
- [ ] A host unit test (`src/tests/unit/`, NOT `src/tests/testgui/`)
      covers decode of both flags, positive and negative case.
- [ ] `grep -n "wedgeSuspect\|wedgeLatched_" src/firm` confirms the
      published flag sources `wedgeSuspect()`, not `wedgeLatched_`.

## Testing

- **Existing tests to run**: `app_telemetry_harness.cpp`, firmware pytest
  tiers, targeted host unit tests. Do NOT touch or run the TestGUI suite.
- **New tests to write**: firmware unit test asserting the flag sets only
  after N consecutive no-encoder-change cycles with nonzero commanded
  duty (not on a single cycle); host unit test for flag decode.
- **Bench verification (required)**: physically stall one wheel on the
  stand and confirm the flag on decoded telemetry per Acceptance
  Criteria; run one full healthy 700 mm leg and confirm no false
  positive.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: wire an existing, already-computed detector through to
  telemetry and the host decode — no new detection logic, only
  publication. No GUI work in this sprint.
- **Files to modify**: `src/firm/messages/telemetry.h`,
  `src/firm/types/robot_state.h`, `src/firm/app/robot_loop.cpp`,
  `src/host/robot_radio/robot/protocol.py`. NOTHING under
  `src/host/robot_radio/testgui/`.
- **Documentation updates**: `telemetry.h`'s flag-table comment block
  (bits 19/20); note the gated-vs-ungated distinction inline since it is
  the one correction the source issue explicitly flags as easy to get
  wrong again.
