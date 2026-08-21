---
id: '015'
title: Radio bench gate rewrite (radio_bench_gate.py) for v6
status: open
use-cases: [SUC-003, SUC-008]
depends-on: ['014']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Radio bench gate rewrite (radio_bench_gate.py) for v6

## Description

Rewrite `src/tests/bench/radio_bench_gate.py` for v6, carrying over its
intent unchanged (spec §11.3): banner-on-connect (now `device` lowercase,
space-separated
per ticket 008's banner break), `HELLO`/`PING`/`ID`/`VER` answered, `WHEELS`
start/stop with climbing encoders, `ok` and `done` both observed (via
ticket 007's line-based ack observation, not the deleted ack ring), the
malformed counter staying clear through a clean connect (verifying ticket
008's relay-control-plane carve-out, spec §9.5), and a wire-quality
measurement against a stated loss budget — run over the relay, not USB.

## Acceptance Criteria

- [ ] Banner check matches the new `device NEZHA2 robot <name> <serial>`
      lowercase form.
- [ ] `HELLO`/`PING`/`ID`/`VER`/`STATUS`/`HELP` all pass against the ASCII
      grammar.
- [ ] `WHEELS` start/stop is exercised with climbing encoders confirmed
      (both directions).
- [ ] `ok` and `done` are both observed via line-reading (ticket 007's
      mechanism), not the deleted ack ring.
- [ ] The malformed-line counter (`flags` bit 9) stays clear through a
      clean connect and handshake, confirming the relay control-plane
      carve-out (spec §9.5: lines starting `#`/`!`/`?` are dropped before
      verb lookup and not counted malformed).
- [ ] A wire-quality measurement (loss rate over the relay) is reported
      against a stated budget, matching the pre-rewrite gate's own
      measurement intent.
- [ ] The gate is run over the relay (RADIOBRIDGE port, per `mbdeploy
      list`'s ROLE column), not the robot's direct USB port.
- [ ] The script exits 0 on a full pass and prints a PASS/FAIL line per
      check, per the existing gate script's convention.

## Testing

- **Existing tests to run**: N/A for the rewritten script itself (it is
  the test); the prior gate's pass/fail categories are the regression
  baseline to preserve.
- **New tests to write**: the rewritten `radio_bench_gate.py` is itself the
  deliverable and its own test.
- **Verification command**: `uv run python src/tests/bench/radio_bench_gate.py
  --port <relay-port>` (against `tovez`'s relay, port taken live from
  `mbdeploy list`, never hardcoded).
