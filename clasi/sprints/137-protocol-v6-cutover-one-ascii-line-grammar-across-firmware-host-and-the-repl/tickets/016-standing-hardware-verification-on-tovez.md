---
id: '016'
title: Standing hardware verification on tovez
status: open
use-cases: [SUC-008]
depends-on: ['015']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Standing hardware verification on tovez

## Description

Run the full standing hardware verification gate on `tovez` (UID
`9906360200052820a8fdb5e413abb276000000006e052820`) per
`.claude/rules/hardware-bench-testing.md`'s standing gate and `sprint.md`'s
Success Criteria / SUC-008: every sensor answers (encoders, OTOS, line,
color), wheels drive both directions with encoders climbing in the correct
direction, round trip confirmed over both USB and the radio relay.
`--clean` build required before flashing (Test Strategy).

## Acceptance Criteria

- [ ] `uv run mbdeploy list` confirms the `tovez` row by UID before any
      hardware action; no other robot on the hub is touched.
- [ ] A `--clean` full reflash is performed (no incremental build) — this
      is also the point at which any prior live-tuned config is understood
      to be gone (`sprint.md`'s "no data migration" note — a clean reflash
      boots from the robot JSON exactly as it always has).
- [ ] Every sensor answers with plausible, changing values: encoders, OTOS
      (position/velocity), line sensor (4 channels), color sensor (RGBC).
- [ ] Wheels drive both directions with encoders incrementing in the
      expected direction, roughly proportional to commanded speed.
- [ ] Round trip is confirmed over both the direct USB serial connection
      and the radio relay.
- [ ] The rewritten `radio_bench_gate.py` (ticket 015) is run as part of
      this verification and exits 0.
- [ ] No reachable reference to COBS/CRC/protobuf/the ack ring remains in
      the running image, confirmed by grepping the actual built/flashed
      sources for this verification (not merely assuming ticket 013's diff
      is sufficient) — matching SUC-008's acceptance criteria.
- [ ] `ESTOP` is exercised and confirmed to actually stop the robot
      (`flags` bit 2 clears, encoders hold) per
      `.claude/rules/playfield-testing.md`'s "one `ESTOP` is not proof of a
      stop" — re-issued and re-confirmed if the first call does not
      visibly stop it.

## Testing

- **Existing tests to run**: N/A — this is the hardware verification
  itself, the culmination of every prior ticket's sim/unit coverage.
- **New tests to write**: N/A — this ticket IS the test.
- **Verification command**: the standing bench-gate sequence from
  `.claude/rules/hardware-bench-testing.md` (sensor checks, drive checks,
  `radio_bench_gate.py`), run live on `tovez`.
