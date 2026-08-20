---
id: '012'
title: Formatting-cost hardware measurement (cyb before/after)
status: open
use-cases: [SUC-007]
depends-on: ['011']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Formatting-cost hardware measurement (cyb before/after)

## Description

This is one of the two on-hardware tickets in this sprint (`tovez`, by
UID). Measure `cyb` (cycle busy, spec §11.2/§6.4) before and after the ASCII
telemetry cutover on real hardware — `tovez`, UID
`9906360200052820a8fdb5e413abb276000000006e052820` — same test scenario,
same robot. Spec §11.2 estimates ~100 µs / 0.3% of a 32 ms cycle for the 30
`snprintf("%ld")` calls per `FULL` frame at 31 fps, but calls this
"arithmetic, not measurement" and "the one number that could invalidate
this design." This ticket produces the actual measured number.

`--clean` build required before flashing, both before and after (an
incremental build after this sprint's repeated `messages/envelope.h`/
`commands.h` touches produces a HardFault that looks like power loss, and
incremental `build.py` ships a stale version string).

## Acceptance Criteria

- [ ] `uv run mbdeploy list` confirms the `tovez` row (UID
      `9906360200052820a8fdb5e413abb276000000006e052820`) before touching
      hardware; `vizev`/other robots on the hub are never targeted.
- [ ] A `--clean` build is flashed for both the "before" (last
      binary-telemetry) and "after" (ASCII-telemetry) measurement, per Test
      Strategy.
- [ ] `cyb` is captured on the stand for the last binary-telemetry build as
      the baseline number.
- [ ] `cyb` is captured on the stand for the ASCII-telemetry (`FULL` mode)
      build, same scenario (same commanded motion, same duration).
- [ ] Both numbers are reported explicitly as measured values (not
      estimates) in this ticket's own closing notes, and called out
      explicitly in the sprint's closing report — not buried only in this
      ticket (Success Criteria / SUC-007's own acceptance criteria).
- [ ] If the measured delta is a problem (materially above the ~0.3%
      estimate and threatens the 32 ms cycle budget), this ticket records
      that finding and notes the known fix (a hand-rolled `itoa`, spec
      §11.2) is a scoped follow-up that does not change the wire design —
      implementing that fix is out of this ticket's own scope unless the
      measurement shows it is needed to keep tickets 013+ safe to proceed.

## Testing

- **Existing tests to run**: N/A — this is a measurement ticket, not a
  code-change ticket (beyond whatever instrumentation is needed to read
  `cyb`, which spec §6.4 already puts on the wire in `FULL` mode).
- **New tests to write**: a repeatable bench measurement procedure
  (documented in this ticket) for capturing `cyb` before/after on `tovez`.
- **Verification command**: N/A (hardware measurement, not a pytest
  target) — the deliverable is the reported before/after numbers.
