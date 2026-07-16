---
status: pending
---

# SimTransport.command()/send() no longer route SET/GET (or any text verb) — 108-007 capability gap

Discovered while running the full suite after sprint 108 ticket 007
(`SimTransport` rewired onto `robot_radio.io.sim_loop.SimLoop`).

## Problem

The deleted `SimConnection` (sprint 081/082's ~40-symbol ctypes ABI) was a
drop-in `SerialConnection` substitute: `SimTransport.send()`/`.command()`
translated an arbitrary text-v2 line (`S`, `SET`, `GET`, ...) to a binary
`CommandEnvelope` via `binary_bridge.translate_command()` and dispatched it
through `SimConnection.send_envelope()` — the whole command/config channel
was simulated, including `SET`/`GET` calibration pushes.

`SimLoop` (108-005/006's real `sim_ctypes.cpp` ABI over
`TestSim::SimHarness`/`SimPlant`) has **no generic wire/config-channel
simulation surface at all** — only `twist()`/`stop()`/`inject_command()`
(a raw escape hatch, unused for this) and a handful of fault setters. Per
ticket 108-007's own scope ("`send()`/`command()`: route what you can...
other verbs can be accepted+logged/no-op or best-effort"), `SimTransport
.send()`/`.command()` are now best-effort no-ops for Sim — they never
raise, but a `GET rotSlip` returns `""`, not the actual value, and a `SET
rotSlip=0` never reaches the firmware.

This regresses two previously-working, tested capabilities for the Sim
backend specifically (hardware — `SerialTransport`/`RelayTransport` — is
completely unaffected, still routes through `binary_bridge
.translate_command()` normally):

1. **Calibration push on Connect** (`__main__.py`'s
   `_push_robot_calibration()`, ticket 085-005) — silently no-ops for Sim
   now. `tests/testgui/test_calibration_push_on_connect.py`'s four
   `@_requires_sim_lib` GUI-level tests (asserting `GET rotSlip`/`GET tw`
   reflect a pushed value) are marked `pytest.mark.skip` referencing this
   issue rather than deleted — the Qt-free `calibration_commands()` unit
   tests in the same file (not requiring the sim lib) are unaffected and
   still pass.
2. **`enc_scale_err_l/r` fault injection** — has no `SimLoop` setter at all
   (see `sim_prefs.py`'s own docstring, narrowed 108-007) — regardless of
   the SET/GET gap, this specific fault knob is gone from the ABI.
   `tests/testgui/test_error_divergence.py::
   test_enc_scale_err_separates_encoder_trace_from_camera_truth` (sprint
   083's own headline "inject error, see divergence" success criterion) is
   marked `pytest.mark.skip` referencing this issue.

## Why this is out of scope for 108-007

108-007 is a transport-plumbing ticket: rewire `SimTransport` onto the
ALREADY-DECIDED (sprint 102, `architecture-update.md` Decision 1) narrower
`SimLoop`/`sim_ctypes.cpp` ABI, and un-gate the tour buttons. Building a
full binary SET/GET request/reply bridge over `SimLoop.inject_command()`
(the raw escape hatch DOES exist — the ABI could theoretically support
this with real engineering work: encode a `CommandEnvelope` the same way
`binary_bridge`/`NezhaProtocol` do, inject it, then correlate the reply off
`read_pending_binary_tlm_frames()`'s ack ring) is a materially different,
larger unit of work than "rewire the twist/stop/telemetry surface", and
was not scoped or estimated as part of this ticket.

## Recommended direction

A follow-up ticket, scoped explicitly to either:
(a) build the SET/GET binary bridge over `SimLoop.inject_command()`
    described above, restoring calibration-push-on-connect and any other
    SET/GET-dependent Sim behavior, or
(b) accept the gap as permanent for Sim (update `_push_robot_calibration()`
    and the Sim Errors panel to reflect "Sim runs uncalibrated,
    DefaultConfig.cpp's baked values only" explicitly rather than silently
    no-opping) and delete (not skip) the four/one tests above.

Stakeholder input needed on which direction — this file records the
discovery and the two tests currently `skip`-marked pending that decision.
