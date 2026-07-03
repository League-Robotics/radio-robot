---
id: '003'
title: 'TestGUI: delete host-side encoder integrator, plot encpose= directly'
status: done
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: tlm-three-world-poses-encoder-only-pose.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI: delete host-side encoder integrator, plot encpose= directly

## Description

With `frame.encpose` available (Ticket 002), `TraceModel` no longer needs
to reconstruct an encoder pose host-side. This ticket deletes the entire
re-integration mechanism — `TraceModel._feed_encoder`'s raw-count
re-integration, its private trackwidth copy, its reset-detection
heuristic, and its turn-scrub compensation — and replaces it with
`_feed_encpose()`, structurally identical to the already-correct
`_feed_otos()`/`_feed_fused()` pattern (absolute world-frame pose,
baselined once on first frame, rotated into the display frame by the fixed
anchor-to-firmware-heading offset via `_rw()`).

This is the structural realization of "hosts become dumb plotters": the
`encoder` trace stops being the one trace requiring host-side sensor
fusion and becomes a third instance of the pattern `otos`/`fused` already
use. See `architecture-update.md` Decision 4 for why a new/bespoke pattern
was rejected in favor of reusing the existing shape.

This host-side integrator has been a defect factory across three known
incidents: missed-reset heading cancellation (2026-07-01), CR-09's
slow-TLM reset misses (sprint 066), and the 2026-07-02 scrub-knob desync.
Deleting it — rather than patching it again — removes the entire defect
class, not just the latest symptom.

**Explicitly retained, do not touch**: `Transport.turn_scrub_factor` and
all Sim-Errors-panel backing code (independent consumer — the simulator's
injected error configuration) and robot-config `tw`/`rotational_slip`
(feed the firmware integrator, not `TraceModel`, since sprint 067). These
are user-facing configuration surfaces, not display-layer internals, and
the issue explicitly calls out that they must remain user-settable.

## Acceptance Criteria

- [x] `host/robot_radio/testgui/traces.py` (`TraceModel`): the following
      are deleted entirely — `_TRACK_MM` module constant; `_geom_track_mm`,
      `_scrub_factor`, `_track_mm` instance attributes;
      `_recompute_track()`, `set_trackwidth_mm()`,
      `set_turn_scrub_factor()`, `notify_reset_pending()`,
      `_feed_encoder()`; `_enc_baseline`, `_enc_h`, `_enc_bx`, `_enc_by`
      instance attributes and their resets in `_reset_baselines()`.
- [x] `TraceModel` gains `_encpose_baseline` and `_feed_encpose()`,
      structurally identical to `_feed_otos()`/`_feed_fused()` (absolute
      world pose, baseline-on-first-frame, `_rw()` rotation). `feed()`
      calls `self._feed_encpose(frame.encpose)` in place of
      `self._feed_encoder(frame.enc)`.
- [x] `_feed_encpose()` handles `frame.encpose is None` the same way
      `_feed_otos()`/`_feed_fused()` already handle an absent field: skip,
      no trace point appended that tick, no crash.
- [x] The `encoder` trace list, its `enabled` flag, and its rendering are
      otherwise unchanged — only the data source changes.
- [x] `host/robot_radio/testgui/__main__.py`: remove the
      `trace_model.set_trackwidth_mm(...)` call (~line 1313), the
      `trace_model.set_turn_scrub_factor(...)` call (~line 1736), and the
      `transport.on_reset_pending = trace_model.notify_reset_pending`
      wiring (~line 1757). Nothing else at these call sites changes.
- [x] `host/robot_radio/testgui/transport.py`: remove
      `is_reset_inducing_command()`, the `on_reset_pending` attribute, and
      `_maybe_notify_reset_pending()` (and its four call sites).
- [x] `Transport.turn_scrub_factor` (the property backing the Sim Errors
      panel) is UNCHANGED and untouched — verify by diff that this
      property, `apply_error_profile()`, and all Sim-Errors-panel backing
      code have zero lines changed.
- [x] Robot-config `tw`/`rotational_slip` fields are unchanged and remain
      user-settable — verify by inspection that no reference to them was
      removed.
- [x] `TraceModel` has no method named `set_trackwidth_mm`,
      `set_turn_scrub_factor`, or `notify_reset_pending`, and no
      `_feed_encoder` method (grep-verifiable).
- [x] `Transport.on_reset_pending` and `is_reset_inducing_command` are
      removed from `transport.py`; no call site references them
      (grep-verifiable).
- [x] `tests/testgui/test_traces.py`: tests exercising the deleted
      `notify_reset_pending()`/`set_turn_scrub_factor()`/
      `set_trackwidth_mm()`/`_feed_encoder()` API are removed; new tests
      for `_feed_encpose()` are added, mirroring existing
      `_feed_otos()`/`_feed_fused()` test coverage (present, absent,
      baseline-and-rotation behavior).
- [x] `tests/testgui/test_transport.py`: tests for `on_reset_pending`/
      `is_reset_inducing_command` are removed; the `turn_scrub_factor` /
      `apply_error_profile()` test class is retained UNCHANGED (Sim Errors
      panel backing — unaffected by this ticket) and still passes.
- [x] A slow-TLM (relay-rate, ~1-2 Hz) session shows a correct, un-skipped
      encoder trace across a `D` command boundary — the CR-09 failure mode
      is structurally impossible (no reset-detection heuristic remains to
      miss a reset).
- [x] Full default pytest suite green (`uv run python -m pytest`).

## Testing

- **Existing tests to run**: `tests/testgui/test_traces.py`,
  `tests/testgui/test_transport.py`, full default suite via
  `uv run python -m pytest`.
- **New tests to write**: `_feed_encpose()` coverage mirroring
  `_feed_otos()`/`_feed_fused()` (present, absent/None, baseline +
  rotation correctness); a slow-TLM scenario test (or manual verification
  documented in Implementation Notes) confirming trace continuity across a
  `D` command boundary with no reset-detection heuristic involved.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Delete first, then add. Remove every listed
attribute/method from `TraceModel` and `transport.py`/`__main__.py`'s
wiring to them, confirming via grep that no remaining call site references
the removed names. Then add `_feed_encpose()` by copying
`_feed_otos()`'s body and renaming, and wire `feed()` to call it with
`frame.encpose`. This ticket depends on Ticket 002 because `frame.encpose`
must exist before `TraceModel` can consume it.

**Files to modify**:
- `host/robot_radio/testgui/traces.py` — delete listed
  attributes/methods; add `_encpose_baseline`/`_feed_encpose()`; update
  `feed()`'s dispatch.
- `host/robot_radio/testgui/transport.py` — remove
  `is_reset_inducing_command()`, `on_reset_pending`,
  `_maybe_notify_reset_pending()` and its call sites. Do not touch
  `turn_scrub_factor`/`apply_error_profile()`.
- `host/robot_radio/testgui/__main__.py` — remove the three call sites
  (~1313, ~1736, ~1757). Line numbers are approximate; locate by symbol
  name (`set_trackwidth_mm`, `set_turn_scrub_factor`, `on_reset_pending`)
  rather than trusting exact line numbers, since prior tickets in this
  sprint may shift surrounding lines slightly.
- `tests/testgui/test_traces.py` — remove obsolete tests; add
  `_feed_encpose()` coverage.
- `tests/testgui/test_transport.py` — remove obsolete
  `on_reset_pending`/`is_reset_inducing_command` tests; retain
  `turn_scrub_factor` tests unchanged.

**Testing plan**:
- Run `tests/testgui/test_traces.py` and `tests/testgui/test_transport.py`
  after the change and confirm the new/updated tests pass and the retained
  `turn_scrub_factor` tests are byte-for-byte unaffected (no edits needed
  to make them pass).
- Manually or via a scripted sim session, throttle TLM to relay rate
  (~1-2 Hz) and drive a `D` command boundary; confirm the encoder trace
  renders continuously with no visible discontinuity or skip — this is the
  CR-09 regression check.
- Run the full default suite (`uv run python -m pytest`) and confirm
  green.

**Documentation updates**: none — this is host-side TestGUI internals with
no wire-protocol or user-facing config surface change. The Sim Errors
panel UI itself is unaffected and needs no documentation update.
