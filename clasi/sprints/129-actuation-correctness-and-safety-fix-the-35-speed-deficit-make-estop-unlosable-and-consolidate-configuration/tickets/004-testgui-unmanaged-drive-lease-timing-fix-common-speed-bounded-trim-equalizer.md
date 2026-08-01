---
id: '004'
title: 'TestGUI unmanaged drive: lease timing fix + common-speed/bounded-trim equalizer'
status: open
use-cases: [SUC-004]
depends-on: []
github-issue: ''
issue: testgui-unmanaged-drive-lease-expiry-and-terminal-pivot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI unmanaged drive: lease timing fix + common-speed/bounded-trim equalizer

## Description

Independent of the actuation-safety/plant-gain work; may execute in any
relative order versus tickets 001-003, 005-011. Two defects in
`src/host/robot_radio/testgui/transport.py`'s unmanaged distance drive,
both measured on the bench 2026-07-31, both introduced the same session:

1. **Lease expiry.** Each `wheels` command is a bounded lease
   (`App::Drive::command()` sets `commandDeadline_`). The host loop used a
   150 ms lease refreshed on a 120 ms sleep (1.25x margin) — any cycle
   where send+handling overran 150 ms let the deadman fire, dropping both
   wheels to zero. Fix: lease 300 ms, re-arm every 60 ms (5x margin).
   Keep the rule "refresh interval ≤ lease/4" documented at the constant.
2. **Terminal pivot.** A per-wheel independent ease-out (added to close an
   L/R distance split) let the leading wheel stop first, turning every
   leg's end into a pivot that erased a 27-degree heading error from the
   encoder record while leaving it in the physical path — "the 1 mm
   encoder split was the symptom, not evidence of a straight line." Fix:
   common speed plus a bounded differential trim (gain 2.0 mm/s per mm,
   ±40 mm/s limit), applied continuously from the first frame — never an
   independent per-wheel profile, never one wheel at 0 while the other
   moves.

## Acceptance Criteria

- [ ] 700 mm unmanaged leg: zero frames where both wheels report
      <5 mm/s mid-leg.
- [ ] Zero commands where one wheel is 0 while the other is nonzero.
- [ ] Camera-measured cross-track ≤30 mm, net heading ≤3 deg (per-segment
      camera fixes at rest, per `.claude/rules/playfield-testing.md` —
      note: this ticket's own verification is a bench/stand distance
      drive, not a playfield tour; if camera measurement is used, follow
      that rule's segment-boundary-fix convention).
- [ ] Against a fake with a worse L/R mismatch than the real robot: 0
      pivot commands, worst L/R ratio bounded per the trim's own gain/limit.

## Testing

- **Existing tests to run**: `uv run python -m pytest`,
  `test_gui_button_acceptance.py`.
- **New tests to write**: a host test simulating scheduler jitter
  (delayed send/handling) confirming the 300 ms/60 ms lease survives it;
  a host test with a fake asymmetric-plant transport confirming the
  common-speed/bounded-trim equalizer never emits a one-wheel-zero
  command.
- **Bench verification**: 700 mm unmanaged leg on the stand, camera-
  measured cross-track/heading per Acceptance Criteria.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: two independent, additive fixes in the same file — lease
  timing constants, and replacing the per-wheel profile with a shared
  speed + bounded trim. Land both together since they're in the same
  module and the same defect-fixing session's scope.
- **Files to modify**: `src/host/robot_radio/testgui/transport.py`.
- **Documentation updates**: comment the "refresh interval ≤ lease/4"
  rule at the constant declaration (already called out as a documented
  rule in the source issue — keep it that way, don't let it regress to a
  bare number).
