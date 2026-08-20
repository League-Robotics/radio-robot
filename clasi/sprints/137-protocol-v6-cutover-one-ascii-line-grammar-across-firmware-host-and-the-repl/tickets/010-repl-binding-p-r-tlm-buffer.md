---
id: '010'
title: REPL binding (p()/r.*, TLM:BUFFER)
status: open
use-cases: [SUC-004, SUC-006]
depends-on: ['009']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# REPL binding (p()/r.*, TLM:BUFFER)

## Description

Build the spec §10 REPL binding on the host's existing `rogo` REPL/serve
surface (`io/repl.py` extended, plus a thin `r.*` wrapper).

**Stakeholder-decision note — do not "correct" this:** this lands host-side
on `rogo`, NOT as a robot-side MicroPython REPL. This is `sprint.md`'s
Design Rationale Decision 3, a deliberate reinterpretation of spec §10 for
this sprint, since MicroPython firmware is explicitly out of scope (see
`sprint.md` Scope / Out of Scope). It is flagged in the sprint's own report
to the stakeholder, not silently assumed — a programmer implementing this
ticket should treat "host-side on rogo" as a fixed requirement, not an
implementation detail to reconsider.

`p(line)` prints each reply line and returns `None` (a returned string
would be echoed by the REPL with quotes, breaking "REPL stdout is
byte-identical to the v6 wire" — spec §10.2). `r.*` is the ergonomic wrapper
(`r.move(...)`, `r.get(...)`, `r.estop()`, etc.) returning typed Python
values and raising on `err`. On this transport telemetry defaults to
`TLM:BUFFER`: the host-side REPL binding buffers received frames in a
bounded deque and prints nothing by default; `r.frames()` drains it,
`r.frames(clear=False)` peeks. Per `sprint.md`'s Design Rationale second
consequence, firmware (ticket 004) treats `BUFFER` identically to `POSE` —
the actual accumulate-don't-print behavior is entirely this ticket's
host-side responsibility.

## Acceptance Criteria

- [ ] `p(line)` sends the raw ASCII line, prints every reply line received,
      and returns `None`.
- [ ] `r.move(v_x=..., stop_distance=..., timeout=..., id=...)` (and
      equivalents for `wheels`/`goto`/`stop`/`estop`/`seed`/`cal`/`get`/
      `set`) return plain Python values (ints, floats, tuples, bools) and
      raise on `err`.
- [ ] `r.wait(<id>)` returns `'stop'`/`'timeout'` from the corresponding
      `done` line.
- [ ] Telemetry buffers by default on this transport (`TLM:BUFFER`) — no
      `t:`/`thdr:` output interleaves with the REPL prompt or with
      `p()`/`r.*` output during normal use.
- [ ] `r.frames()` drains the buffer (newest last); `r.frames(clear=False)`
      peeks without draining.
- [ ] `r.tlm('print')` is available as an explicit opt-in for a human
      watching a terminal, documented as non-default and not for scripts
      (spec §10.4).
- [ ] A `finally: r.estop()` pattern around a driving loop is documented
      with a runnable example (spec §10.5's safety note), and `r.estop()`
      is demonstrated being re-issued if `flags` bit 2 does not clear on
      the first call (per `.claude/rules/playfield-testing.md`'s "one
      `ESTOP` is not proof of a stop").
- [ ] This ticket's body/README explicitly states the host-side-not-robot-
      side scope decision, so a future reader does not silently "fix" it
      toward a robot-side REPL.

## Testing

- **Existing tests to run**: `rogo` CLI/serve suites.
- **New tests to write**: a `p()` output test asserting byte-identical
  output to the raw wire line for a representative vector set (a preview of
  ticket 011's full fourth vector set — do not duplicate that work, just
  smoke-test here); a buffered-telemetry test confirming no output during a
  `move`+`wait` sequence with an active `TLM:FULL` subscription.
- **Verification command**: `uv run pytest src/tests -k repl`.
