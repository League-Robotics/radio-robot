---
id: '005'
title: 'TestGUI host dead-reckoner: use effective (not raw) trackwidth in both instances'
status: open
use-cases: [SUC-005]
depends-on: []
github-issue: ''
issue: testgui-host-dead-reckoner-used-raw-not-effective-trackwidth.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI host dead-reckoner: use effective (not raw) trackwidth in both instances

## Description

Independent of the actuation-safety/plant-gain work; may execute in any
relative order. On the bench 2026-07-31 the Fused and Encoder traces on
the Playfield tab were 710 mm/15 deg apart after a session of turning,
having agreed to 0.1 deg on an earlier straight leg. Cause: two
consumers feed the host `EncoderDeadReckoner` the **raw** caliper
trackwidth instead of the **effective** (slip-corrected) one the firmware
actually integrates with (`App::effectiveTrackWidth()` =
`trackwidth / rotational_slip`):

1. `__main__.py`'s `_on_robot_changed` feeds `cfg.trackwidth` (raw)
   instead of `_effective_track_width(cfg)` (already written elsewhere in
   `testgui/transport.py` for the sim's own copy of this bug).
2. `TurnGraphPanel`'s own `recorder` is constructed with a hard-coded
   `trackwidth=128.0` literal that nothing ever updates — the Heading and
   Distance strip charts carry the same error for every robot, always.

Error is proportional to accumulated rotation (predicted ratio 140.4/128.0
= 1.097 vs. observed heading ratio 195.6/180.5 = 1.084, within 1.2%) — so
it is invisible on a straight leg and large after a turning session.

## Acceptance Criteria

- [ ] Both dead-reckoner instances are sourced from
      `_effective_track_width(cfg)`, set at the same time from the same
      per-robot config.
- [ ] A session containing multiple turns: Fused and Encoder traces stay
      within the pre-existing straight-leg agreement bound (~0.1 deg)
      rather than diverging with accumulated rotation.
- [ ] `grep` confirms no remaining hard-coded `128.0`/raw-trackwidth
      literal feeds either dead-reckoner instance.
- [ ] Follow-up considered (not required to land, per the source issue):
      making trackwidth a required constructor argument so an
      unconfigured recorder cannot silently draw a plausible-but-wrong
      trace.

## Testing

- **Existing tests to run**: `uv run python -m pytest`.
- **New tests to write**: a host test asserting both `EncoderDeadReckoner`
  instances (Playfield tab, `TurnGraphPanel`) are constructed/updated from
  `_effective_track_width(cfg)`, not `cfg.trackwidth` or a literal.
- **Bench verification**: a multi-turn bench session; confirm Fused vs.
  Encoder agreement holds within the straight-leg bound throughout.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: point both existing consumers at the existing
  `_effective_track_width(cfg)` helper — no new geometry logic needed,
  this is a wiring fix.
- **Files to modify**: `src/host/robot_radio/testgui/__main__.py`
  (`_on_robot_changed`), the `TurnGraphPanel` module (wherever `recorder`
  is constructed), `src/host/robot_radio/testgui/transport.py` (source of
  `_effective_track_width`, read-only reference).
- **Documentation updates**: none required beyond the code itself; if a
  DESIGN.md documents the TestGUI's pose sources, note effective vs. raw
  trackwidth as a documented gotcha (ties into the same "invisible until
  a session accumulates rotation" trap sprint 128 named for a different
  cause).
