---
id: '002'
title: SimTransport SET/GET binary bridge over SimLoop.inject_command()
status: open
use-cases: [SUC-005]
depends-on: []
github-issue: ''
issue: sim-transport-command-set-get-not-supported.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimTransport SET/GET binary bridge over SimLoop.inject_command()

## Description

Sprint 108 ticket 007 rewired `SimTransport` onto the narrower `SimLoop`/
`sim_ctypes.cpp` ABI (`twist()`/`stop()`/`inject_command()` + fault
setters), leaving `SimTransport.send()`/`.command()` as best-effort no-ops
for any other text verb — in particular `SET`/`GET`. This regresses
calibration push-on-connect and the `enc_scale_err_l/r` fault-injection
knob for the Sim backend specifically (hardware transports are unaffected).
This ticket is option (a) from the issue: build the real bridge, since this
sprint's decisive sim tour-closure gate (ticket 009) needs `SET`/`GET` to
actually work in Sim — for OTOS calibration (ticket 007) and for the
fault-injection knobs the sim-fidelity work depends on.

This ticket is independent of the Ruckig/motion work (ticket 001) and can
be done in either order relative to it — it has no `depends-on`. It is a
prerequisite for ticket 007 (sim fidelity) because that ticket's tests need
a working `SET`/`GET` path to dial in fault knobs and verify calibration
correction.

1. Implement `SimTransport.send()`/`.command()` to encode an arbitrary
   text-v2 line (`SET ...`, `GET ...`) into a binary `CommandEnvelope` via
   the existing `binary_bridge.translate_command()` (the same path
   `SerialTransport`/`RelayTransport` already use), inject it via
   `SimLoop.inject_command()` (the raw escape hatch already exists per the
   issue), and correlate the reply by polling
   `read_pending_binary_tlm_frames()`'s ack ring for the matching
   correlation id.
2. Restore the `enc_scale_err_l/r` fault-injection knob on `SimLoop`/
   `sim_ctypes.cpp` (currently absent per `sim_prefs.py`'s own docstring) —
   whatever plumbing is simplest given `SimPlant`'s existing per-wheel
   error hooks (this ticket only needs the ABI knob to exist and be
   settable; ticket 007 owns the actual error-model fidelity work).
3. Un-skip the tests parked against this issue: the four
   `@_requires_sim_lib` tests in
   `tests/testgui/test_calibration_push_on_connect.py`, and
   `tests/testgui/test_error_divergence.py::
   test_enc_scale_err_separates_encoder_trace_from_camera_truth`.

## Acceptance Criteria

- [ ] `SimTransport.send()`/`.command()` round-trips an arbitrary `SET`/
      `GET` text-v2 line through `binary_bridge.translate_command()` +
      `SimLoop.inject_command()`, with the reply correctly correlated back
      to the caller (not a no-op, not `""`).
- [ ] `enc_scale_err_l`/`enc_scale_err_r` are settable via the Sim backend
      (new `SimLoop`/`sim_ctypes.cpp` knob).
- [ ] The four skipped `@_requires_sim_lib` tests in
      `test_calibration_push_on_connect.py` pass (un-skipped, not deleted).
- [ ] `test_error_divergence.py::
      test_enc_scale_err_separates_encoder_trace_from_camera_truth` passes
      (un-skipped, not deleted).
- [ ] Hardware transports (`SerialTransport`/`RelayTransport`) are
      unaffected — no shared-path regression (existing SET/GET tests
      against real/relay transports still pass).
- [ ] This ticket does not touch `src/firm/` — no `DESIGN.md` update is
      required (host/Python-only change), consistent with the sprint's
      standing rule (`src/firm/`-touching tickets only).

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/testgui/
  test_calibration_push_on_connect.py tests/testgui/
  test_error_divergence.py` (confirm previously-skipped tests now pass);
  full `uv run python -m pytest` for regressions on the hardware-transport
  SET/GET paths (`binary_bridge`, `NezhaProtocol`).
- **New tests to write**: a direct `SimTransport` SET/GET round-trip test
  if the existing skipped tests don't already cover the bridge mechanism
  itself in isolation from the calibration-push feature.
- **Verification command**: `uv run python -m pytest tests/testgui/ -k
  "calibration or error_divergence"` plus the full suite.

## Implementation Plan

**Approach**: Build the bridge described in the issue's own recommended
direction (a): encode via the existing `binary_bridge` translation used by
hardware transports, inject via the existing `SimLoop.inject_command()`
escape hatch, correlate via the existing ack-ring reader. No new wire
format — reuses the same `CommandEnvelope`/`ReplyEnvelope` encoding
already used for hardware.

**Files to modify**:
- `testgui/*` or `host/robot_radio/io/sim_loop.py`/`SimTransport`'s actual
  module (locate via `grep -rn "class SimTransport"`) — add the SET/GET
  bridge.
- `src/sim/sim_ctypes.cpp` (new `enc_scale_err_l/r` setter(s), mirroring
  existing fault-setter patterns in the same file).
- `tests/testgui/test_calibration_push_on_connect.py` (remove
  `pytest.mark.skip`).
- `tests/testgui/test_error_divergence.py` (remove `pytest.mark.skip`).

**Testing plan**: as above — un-skip and pass the two parked test files;
add a direct bridge round-trip test if needed for isolated coverage.

**Documentation updates**: none required in `src/firm/` (host-only
change); if this project keeps host-side module docs (check
`host/CLAUDE.md` or equivalent), note the restored SimTransport capability
there if such a doc exists.
