---
id: '004'
title: 'Firmware telemetry column tables + thdr:/t: assembly + TLM subscription verb'
status: open
use-cases: [SUC-002]
depends-on: ['002']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware telemetry column tables + thdr:/t: assembly + TLM subscription verb

## Description

Using ticket 001's generated telemetry column tables, add `Core::Telemetry`
support for the `TLM:<mode>` subscription verb (`OFF`/`POSE`/`FULL`/`NOW`/
`AUTO`/`BUFFER`, spec §6.1) and `thdr:`/`t:` frame assembly (spec §6.2),
alongside — not replacing — the existing binary telemetry arm. This is spec
§13 migration stage 2. `POSE` is 9 columns (~38 B/frame); `FULL` is 30
columns (~160 B/frame). A `thdr:` column header line is emitted whenever the
subscription mode changes and before the first frame after connect.

Per `sprint.md`'s Design Rationale (Decision 3's second consequence):
`BUFFER` is accepted as a valid wire token by firmware (so the fixture and a
future MicroPython robot have a stable value to target), but firmware
treats it identically to `POSE` — the accumulate-don't-print policy is
entirely a host-side REPL-binding concern (ticket 010), since this sprint
builds no robot-side REPL. Do not implement buffering in firmware for this
ticket.

## Acceptance Criteria

- [ ] `TLM:POSE` (the default) produces `thdr:seq:now:flags:x:y:h:ox:oy:oh`
      then `t:` lines with 9 fields matching spec §6.3's column order and
      scale, ~38 B/frame.
- [ ] `TLM:FULL` produces the 30-column frame per spec §6.4's exact order
      (`mode`, `elp/elv/ela/ele`, `erp/erv/era/ere`, `ovx/ovy`, `ow`, `oa`,
      `tvx/tvy`, `tw`, `l1..l4`, `cr/cg/cb/cc`, `cyb/cyp`), scale factors
      (`×10`/`×100`) unchanged from v5.
- [ ] A mode change (`TLM:<mode>`) always emits a fresh `thdr:` before the
      next `t:` line.
- [ ] `TLM:NOW` emits one frame immediately in the current mode without
      changing the subscribed mode.
- [ ] Both pose sources (encoder odometry `x/y/h`, OTOS `ox/oy/oh`) ride
      every frame in both `POSE` and `FULL` modes (spec §6.3's "divergence
      is the measurement" rationale) — `ox/oy/oh` valid iff `flags` bit 0.
- [ ] `flags` bit semantics are carried forward from v5 unchanged (spec
      §6.5's bit table) and decode correctly against the same bit table the
      existing binary telemetry uses.
- [ ] `TLM:BUFFER` is accepted as a valid token and firmware behaves
      identically to `TLM:POSE` for it — no firmware-side accumulate-don't-
      print behavior is implemented here (that is ticket 010, host-side).
- [ ] Existing binary telemetry arm is untouched and its existing tests
      still pass (additive-only ticket).

## Testing

- **Existing tests to run**: `src/tests/sim/unit` Telemetry suites, existing
  binary telemetry tests (must still pass).
- **New tests to write**: sim/unit tests for `POSE`/`FULL` column count and
  order, `thdr:` emission on mode change and on first connect, `TLM:NOW`
  one-shot behavior, `TLM:BUFFER` firmware-side no-op-vs-`POSE` equivalence.
- **Verification command**: `uv run pytest src/tests/sim/unit -k telemetry`.
