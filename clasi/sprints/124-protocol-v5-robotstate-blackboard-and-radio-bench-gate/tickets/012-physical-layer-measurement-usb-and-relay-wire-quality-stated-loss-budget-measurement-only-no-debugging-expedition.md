---
id: '012'
title: 'Physical-layer measurement: USB and relay wire quality, stated loss budget
  (measurement only, no debugging expedition)'
status: open
use-cases: [SUC-007]
depends-on: ['005', '010']
github-issue: ''
issue: telemetry-physical-layer-corruption-and-move-ack-observability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Physical-layer measurement: USB and relay wire quality, stated loss budget (measurement only, no debugging expedition)

## Description

**Measurement and budget only — explicit non-goal below.** Characterize
BOTH the direct-USB and radio-relay physical wire paths under the final
v5 framing (ticket 005) and a clean relay connect (ticket 010), and
state a loss budget for each. This ticket promotes the scratchpad
`wire_truth.py`-style probe (single-threaded raw pyserial + demux +
decode, no queue/threads — the authoritative wire-quality measurement
from 123's overnight bench work) into a committed script under
`src/tests/bench/`.

**Explicit non-goal**: this ticket does NOT open a hardware
investigation. The residual ~5-11% USB corruption is ALREADY PROVEN
100% CRC-caught (`unparseable=0` on every prior run) — the link is
safe, just lossy, and that is an ACCEPTED condition, not a bug to chase.
Do not attempt to root-cause the physical-layer corruption itself here;
measure it, and the relay path, and state numbers.

## Acceptance Criteria

- [ ] A `wire_truth`-equivalent script is committed to
      `src/tests/bench/` (not left in scratchpad) — single-threaded raw
      capture + demux + decode, matching the 123-era probe's
      methodology.
- [ ] Both USB and relay are measured under comparable conditions
      (same robot, same firmware build, similar session length);
      results recorded (not just asserted in code) in a form ticket
      013 can reference for its own bench-gate pass/fail.
- [ ] A stated loss budget (a percentage, or equivalent) is recorded
      for each path.
- [ ] `unparseable=0` is reconfirmed on both paths (CRC catches
      everything; nothing mis-parses).
- [ ] The non-goal above is stated explicitly in this ticket's closing
      notes: no root-cause investigation was opened for the residual
      USB corruption.

## Testing

- **Existing tests to run**: N/A — this ticket is itself a measurement
  script, run on the bench, not a unit-test change.
- **New tests to write**: the promoted `wire_truth`-equivalent bench
  script itself.
- **Verification command**: run the promoted script over both USB and
  relay on the bench; record results per
  `.claude/rules/hardware-bench-testing.md`.
