---
id: '010'
title: Full sweep and bench gate
status: open
use-cases: [SUC-050, SUC-051, SUC-052, SUC-053, SUC-054, SUC-055]
depends-on: ['006', '007', '008', '009']
github-issue: ''
issue:
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Full sweep and bench gate

## Description

Final ticket — sprint 116's own hardware-bench-testing gate, per
`.claude/rules/hardware-bench-testing.md` and the protocol set-point
issue's Verification section. Structured like sprint 115's own final
ticket (010) to handle the "hardware may be disconnected at execution
time" reality: a real protocol gate if the robot is on the stand and
connected, or a full sim dry-run of the same scenarios (ticket 008 already
covers most of them) plus a written bench checklist for whenever hardware
becomes available — either way, the full sim scenario suite is a hard
acceptance criterion, not an optional nice-to-have contingent on hardware
presence (sprint.md Migration Concerns).

## Acceptance Criteria

- [ ] `uv run python -m pytest` green across the full suite (not a
      subset).
- [ ] `python build.py` builds firmware + host sim lib clean.
- [ ] **If hardware is connected** (`pyocd list` / `mbdeploy probe` shows
      exactly one micro:bit V2): `just build-clean` + `mbdeploy deploy`
      (hex verified by full UID — confirm it's the robot, not the relay
      dongle); then the real protocol gate, robot on stand, wheels free —
      `HELLO`/`PING` (`t=` present)/`CONFIG` patch persists across
      power-cycle/`MOVE` × both velocity variants × all three stop
      conditions/`STOP`, each acked correctly; stop-condition behavior
      (time/distance/angle measured via encoders on the stand,
      stalled-timeout fault); chaining/replace/`ERR_FULL`/no-deadman
      empty-queue expiry with zero host traffic; a ≥10-minute soak at
      5-10 Hz alternating MOVEs — no reboot/lockup, seq monotonic, drop
      rate at or better than the sprint-115 baseline.
- [ ] **If hardware is absent or unavailable**: write
      `docs/bench-checklists/sprint-116-move-protocol.md` (mirroring
      `sprint-115-gut-s1.md`'s structure) listing every check above as a
      TODO for whenever hardware becomes available, AND run a full sim
      dry-run of the same scenario list (ticket 008's suite, plus any gap
      it doesn't already cover) as the sprint's actual, hard acceptance
      evidence for this ticket.
- [ ] Report which branch (real gate vs. sim dry-run + checklist) was
      taken and why, in this ticket's own completion notes.

## Testing

- **Existing tests to run**: the full `uv run python -m pytest` suite,
  plus `src/tests/sim/system/test_move_protocol.py` (ticket 008)
  specifically as the dry-run substitute if hardware is absent.
- **New tests to write**: none beyond what ticket 008 already added,
  unless the dry-run surfaces a scenario gap — if so, add it here rather
  than silently skip it.
- **Verification command**: `python build.py && uv run python -m pytest`
  (+ the hardware or sim-dry-run branch above).
