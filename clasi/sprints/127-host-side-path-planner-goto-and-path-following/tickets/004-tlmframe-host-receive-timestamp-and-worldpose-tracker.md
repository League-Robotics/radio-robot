---
id: '004'
title: TLMFrame host receive timestamp and WorldPose tracker
status: open
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TLMFrame host receive timestamp and WorldPose tracker

## Description

Design issue T3. `WorldPose` is the host-side tracker for
`T_world_from_odom`, re-anchored at each camera fix, so
`world_pose = T ∘ firmware_pose`. It consumes `TLMFrame.pose` and
`TLMFrame.otos_reading` (both carry their own `age`), and needs a host
receive timestamp to extrapolate `t - age` the way `hil_drive.py` already
does (`hil_drive.py:117-143`) — `TLMFrame` has no such field today
(confirmed: `TLMFrame.from_pb2()` has exactly one call site,
`NezhaProtocol.read_pending_binary_tlm_frames()` at
`robot/protocol.py:1607`). Depends on ticket 003 because re-anchoring
reuses the promoted `captureFix`/camera-fix helpers from `field/` rather
than reimplementing camera access.

Deliberate by-product (call this out in the ticket's own printed/logged
output, not just in code comments): tracking a transform for the encoder
pose *and* a separate transform for the OTOS pose yields the
encoder-vs-OTOS divergence over time, for free. That divergence measurement
is what ticket 007 uses to recommend a camera re-anchor cadence — expose
it as a plain method/property on `WorldPose`, not buried internal state.

**Do not** wire up `ClockSync` — it's blocked on a separate, pre-existing
`serial_conn.py` corr-id bug (sprint scope, Out of Scope). Use frame-age
extrapolation only.

**Files**:
- Modify: `src/host/robot_radio/robot/protocol.py` — add one optional
  field to `TLMFrame` (host receive timestamp, default `None`, matching
  every other field's "never decoded" convention per the dataclass's own
  docstring); populate it in `read_pending_binary_tlm_frames()` at the
  point each frame is drained off the wire (`time.monotonic()`, not wall
  clock — matches `hil_drive.py`'s own convention).
- New: `src/host/robot_radio/pathplan/__init__.py`,
  `src/host/robot_radio/pathplan/world_pose.py` — `WorldPose` class:
  owns `T_world_from_odom`, exposes the current world pose, exposes
  `encoderOtosDivergence()` (or similar), re-anchors from a camera fix
  (via `field/`'s promoted `captureFix`).
- Reuse, do not fork: `Pose`/`Waypoint` from
  `src/host/robot_radio/nav/pose.py` (explicitly carved out of the
  dormant-code freeze by `robot_radio/DESIGN.md` — importing it is not
  itself a sign of dormancy).

**Coding standards**: the new `TLMFrame` field is a quantity name with a
unit tag, not a units-suffixed identifier — e.g. `recvTime`
(`# [s] host monotonic clock at frame decode`), never `recv_time_s` or
`recvTimeSec`. `WorldPose`'s own methods/fields follow the same rule (a
transform is `worldFromOdom`, not `world_from_odom_transform_mm`).
lowerCamelCase functions/variables, UpperCamelCase types (`WorldPose`).

## Acceptance Criteria

- [ ] `TLMFrame` gains one new field, default `None`, populated only at
      `read_pending_binary_tlm_frames()`; every other existing consumer of
      `TLMFrame` is confirmed unaffected (a frame built any other way
      still leaves the field `None`, per the dataclass's own documented
      convention).
- [ ] `WorldPose` unit tests cover the transform math (re-anchor from a
      camera fix updates `T_world_from_odom` correctly) and the age-based
      extrapolation (`t - age`) with **no hardware or sim dependency**.
- [ ] `WorldPose` exposes encoder-vs-OTOS divergence as a plain,
      independently-testable method/property — not internal-only state —
      with its own unit test using synthetic diverging inputs.
- [ ] `WorldPose`'s re-anchor path imports camera-fix access from
      `robot_radio.field` (ticket 003's promoted module), not a
      reimplementation.
- [ ] No `ClockSync` activation anywhere in this ticket's diff.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (confirm the additive `TLMFrame` field breaks nothing).
- **New tests to write**: `src/tests/unit/test_world_pose.py` (or under
  a `pathplan` unit-test location matching the package's own layout) —
  transform re-anchor math, age extrapolation, divergence computation, all
  pure/synthetic, no I/O.
- **Verification command**: `uv run python -m pytest src/tests/unit/test_world_pose.py -q`
