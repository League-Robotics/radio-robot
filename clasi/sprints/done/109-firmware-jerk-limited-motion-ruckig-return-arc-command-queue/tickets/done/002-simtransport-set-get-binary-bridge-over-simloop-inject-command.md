---
id: '002'
title: TestGUI Sim-mode config path over typed binary patches
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: sim-transport-command-set-get-not-supported.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI Sim-mode config path over typed binary patches

## Description

**Revised 2026-07-17** after a programmer-thrown `internal` architecture
exception on this ticket's original scope ("wire `SimTransport` onto the
existing `binary_bridge.translate_command()` path `SerialTransport`/
`RelayTransport` already use"). That path does not exist to wire onto:
`binary_bridge.translate_command()` is a universal stub for every verb on
every transport (hardware included) — `legacy_render`/`legacy_verbs` were
deleted wholesale by commit `129cbcb3` (sprint 104 ticket 002) and never
rebuilt. Full root-cause narrative and the two other stacked findings
(no wire arm exists for a config VALUE to travel firmware→host at all;
`RobotLoop::handleConfig()` only applies `MotorConfigPatch`, so
`DrivetrainConfigPatch` fields like `rotSlip`/`tw` have no firmware
consumer today) are preserved in this sprint's `sprint.md`, **Architecture
Revision 1**. Read that section before implementing this ticket — it is
the authoritative record of why the approach changed and explicitly rules
out two tempting-but-forbidden fixes: resurrecting `legacy_render`/
`legacy_verbs` (reopens the stakeholder's 2026-07-10 "firmware stays pure
binary" decision), and fabricating Sim-only GET/SET semantics that
hardware could never match (violates this project's Sim-must-not-diverge-
from-real-firmware-capability discipline).

**Revised scope**: route SET-shaped host needs through typed `ConfigDelta`
patches that have a real firmware consumer, constructed and sent directly
— never through `binary_bridge.translate_command()`, which stays dead.
This works identically on hardware and Sim transports (one mechanism, not
a Sim-specific fork), which also incidentally fixes the fact that
hardware's own `translate_command()`-based text-verb path was already
just as dead as Sim's — it was simply unnoticed because the only SET/GET
paths anyone actually exercised in practice (motor gains) already went
through a different, working, direct-construction mechanism.

1. **Find the existing direct-patch-send mechanism.** `MotorConfigPatch`
   already reaches the firmware live today (`RobotLoop::handleConfig`
   applies it) on hardware transports, and it is NOT going through
   `translate_command()` (that function is a universal stub). Locate the
   actual call path (likely `NezhaProtocol.set_config()`/
   `set_config_binary()` or similar in `protocol.py` — grep for whatever
   constructs a `ConfigDelta`/`MotorConfigPatch` envelope directly from
   typed Python values) before writing anything new. This is the
   mechanism to reuse, not reinvent.
2. **Make `SimTransport` use the same mechanism.** Wire `SimTransport` to
   call the same direct-construction path `SerialTransport`/
   `RelayTransport` already use for `MotorConfigPatch`, injecting the
   resulting envelope via `SimLoop.inject_command()` (the raw escape
   hatch already exists) instead of a live serial write. Correlate the
   reply by polling `read_pending_binary_tlm_frames()`'s ack ring for the
   matching correlation id, same as before.
3. **Honest unsupported-key behavior.** For any SET/GET-shaped request
   whose target key belongs to a `ConfigDelta` patch kind with no live
   firmware consumer today (currently: `DrivetrainConfigPatch` —
   `rotSlip`, `tw`; per Architecture Revision 1 finding 3), return an
   explicit, immediate host-side "unsupported" error. Do not attempt a
   wire round-trip for these keys, do not silently no-op, and do not
   fabricate a value.
4. **Host-side GET.** For keys with a real consumer, GET answers from
   host-side state — the last value the host itself pushed via SET,
   echoed back — not from a new firmware query wire arm (Architecture
   Revision 1 explains why a real query arm is out of scope this sprint).
5. **`enc_scale_err_l/r` fault-injection knob** — unaffected by any of the
   above; this was never a `ConfigDelta` patch and never depended on
   `translate_command()`. Add the knob directly to `SimLoop`/
   `sim_ctypes.cpp` (currently absent per `sim_prefs.py`'s own docstring),
   mirroring the existing OTOS-drift-knob pattern in the same file.
6. **Test disposition — `test_calibration_push_on_connect.py`'s four
   `@_requires_sim_lib` tests.** Two of them assert `GET rotSlip`/`GET tw`
   reflect a pushed value — exactly the two keys step 3 above shows have
   no firmware consumer. Per Architecture Revision 1's explicit direction:
   either (a) retarget those two assertions onto a key that does have a
   real consumer (e.g. a motor-gain field) if that satisfies the
   calibration-push-on-connect feature's actual intent, or (b) revise them
   to assert the honest unsupported-key error from step 3. Pick whichever
   more faithfully preserves the original test's intent, and record which
   choice was made (and why) in this ticket or the test file's own
   docstring. Do not simply remove `pytest.mark.skip` unchanged — that is
   exactly the move the exception ruled out.
7. **`test_error_divergence.py::
   test_enc_scale_err_separates_encoder_trace_from_camera_truth`** — this
   test targets the sim-only fault knob (step 5), not a `ConfigDelta`
   patch; it is unaffected by the architecture revision and should
   un-skip and pass once step 5 lands.

## Acceptance Criteria

- [x] The existing direct-patch-send mechanism hardware transports use
      for `MotorConfigPatch` is identified (file:line cited in this
      ticket's implementation notes) and reused — not reinvented — for
      `SimTransport`.
- [x] `SimTransport` sends typed `ConfigDelta` patches (motor gains at
      minimum) via that same mechanism, injected through
      `SimLoop.inject_command()`, with replies correctly correlated via
      `read_pending_binary_tlm_frames()`'s ack ring.
- [x] `binary_bridge.translate_command()` is **not** modified, restored,
      or routed through by this ticket — it stays dead. No
      `legacy_render`/`legacy_verbs` code is resurrected.
- [x] Requests targeting a `ConfigDelta` patch kind with no live firmware
      consumer (`DrivetrainConfigPatch`: `rotSlip`, `tw`) return an
      explicit host-side "unsupported" error — no wire round-trip
      attempted, no silent no-op, no fabricated value.
- [x] GET for a supported key returns the host's own last-pushed value
      (echo), not a fabricated or firmware-queried value.
- [x] `enc_scale_err_l`/`enc_scale_err_r` are settable via the Sim backend
      (new `SimLoop`/`sim_ctypes.cpp` knob), independent of the above.
- [x] `test_calibration_push_on_connect.py`'s four tests are each either
      passing against a retargeted real-consumer key, or revised to
      assert the honest unsupported-key error for `rotSlip`/`tw` — with
      the choice and rationale recorded (not silently unskipped
      unchanged).
- [x] `test_error_divergence.py::
      test_enc_scale_err_separates_encoder_trace_from_camera_truth`
      passes (un-skipped, targets the sim-only fault knob only).
- [x] Hardware transports (`SerialTransport`/`RelayTransport`) are
      unaffected — no shared-path regression; the direct-patch-send
      mechanism they already use for `MotorConfigPatch` is read, not
      modified, by this ticket.
- [x] This ticket does not touch `src/firm/` — no `DESIGN.md` update
      required (host/Python-only change).

## Implementation Notes

- **Mechanism identified and reused**: `NezhaProtocol.config(**deltas)`
  (`src/host/robot_radio/robot/protocol.py:756`) — builds exactly ONE
  `ConfigDelta` envelope from the existing flat wire-key vocabulary
  (`_DRIVETRAIN_KEYS`/`_MOTOR_PID_KEYS`/`_PLANNER_KEYS`/`ml`/`mr`/
  `sTimeout`, lines 443-487) and sends it via
  `self._conn.send_envelope_fast(envelope)` — confirmed live/tested against
  hardware transports (`src/tests/unit/test_protocol_config.py`), NOT
  through `binary_bridge.translate_command()` (which
  `set_config()`/`set_config_binary()` also don't use — both are
  independent, real mechanisms; `config()` is the fire-and-poll one whose
  ack rides the telemetry ack ring, matching `twist()`/`stop()`'s own
  shape, which is the one `SimTransport` needed to inject via
  `SimLoop.inject_command()`).
- **`SimTransport` wiring** (`src/host/robot_radio/testgui/transport.py`):
  new `_SimConfigConn` (duck-typed `SerialConnection` substitute — only
  `send_envelope_fast()`; ack correlation is its own `poll_ack()`, called
  directly rather than through `NezhaProtocol.wait_for_ack()`, whose
  re-wrap expects a raw `pb2.AckEntry`, not `SimLoop`'s already-adapted
  `TLMFrame`/`AckEntry` — see that class's own docstring) wraps a `SimLoop`;
  `SimTransport.connect()` constructs `NezhaProtocol(_SimConfigConn(loop))`
  as `self._config_proto`. `_dispatch()` routes `SET key=value`/`GET key`
  to new `_handle_config_set()`/`_handle_config_get()`, which classify
  every key in `NezhaProtocol._ALL_SET_KEYS` as either `_CONFIG_MOTOR_KEYS`
  (has a real consumer — `pid.*`/`ml`/`mr`) or `_CONFIG_UNSUPPORTED_KEYS`
  (everything else — no consumer, per `RobotLoop::handleConfig()` only
  applying `MOTOR` patch kind); unsupported keys return immediately with no
  wire traffic at all (verified by a test that makes `inject_command()`
  raise if called). `self._config_echo` is the host-side GET-echo store.
- **`enc_scale_err_l/r` knob**: new `WheelPlant::setScaleErr()`
  (`src/tests/sim/plant/wheel_plant.h`/`.cpp`) applied last in
  `reportedPosition()`; `SimPlant::setEncScaleErr(port, fraction)`
  (`src/sim/sim_plant.{h,cpp}`) fans out to the selected wheel; new
  `sim_set_enc_scale_err()` C ABI export (`src/sim/sim_ctypes.cpp`); new
  `SimLoop.set_enc_scale_err()` Python binding
  (`src/host/robot_radio/io/sim_loop.py`); wired into
  `SimTransport._apply_profile_to_sim()` alongside the existing
  `set_otos_drift` mapping.
- **Test disposition (`test_calibration_push_on_connect.py`)**: chose
  option (a) — retarget the round-trip assertion from `rotSlip`/`tw`
  (DrivetrainConfigPatch, no consumer) onto `ml` (MotorConfigPatch.
  travel_calib, a real consumer that `calibration_commands()` ALSO
  pushes) — this preserves each test's actual intent ("Connect pushes
  this robot's calibration into firmware, overwriting whatever was
  there") without asserting something structurally impossible.
  `rotSlip`/`tw` are still exercised in the same tests: each now asserts
  the honest, immediate "unsupported" error instead of a fabricated
  value. Full rationale recorded in the test file's own module-docstring
  update.
- **Drive-by fix**: `robot_config.py`/`camera_prefs.py`/`sim_prefs.py`'s
  `_PROJECT_ROOT` was off by one `.parent` (pointed at `src/` instead of
  the repo root) since the "unify all source trees under src/" refactor
  (commit `575ef391`) — `canvas.py` got the equivalent fix under ticket
  107-004 but these three were missed. This silently broke
  `list_robots()` (empty `data/robots/` glob), surfaced by
  `test_robot_combo_change_while_connected_repushes_and_overwrites` (the
  one calibration-push test that actually lists real robot configs rather
  than pointing `ROBOT_CONFIG` at a temp file) — fixed as a narrow,
  host-only, one-line-per-file correction, required to make that test's
  acceptance achievable.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` for
  regressions on the hardware-transport `MotorConfigPatch` push path
  (must remain byte-for-byte unchanged in behavior — this ticket only
  reads and reuses it, never edits it).
- **New tests to write**: a direct `SimTransport` typed-patch round-trip
  test (motor gains) in isolation from the calibration-push feature;
  an unsupported-key error test for `DrivetrainConfigPatch` fields.
- **Verification command**: `uv run python -m pytest tests/testgui/ -k
  "calibration or error_divergence"` plus the full suite.

## Implementation Plan

**Approach**: Reuse, don't rebuild. The whole point of this revision is
that hardware already has a working, non-`translate_command()` path for
the one patch kind that already has a firmware consumer (`MotorConfig`);
this ticket's job is to find that path and point `SimTransport` at it,
plus add the two purely-host-side behaviors (unsupported-key error,
GET-from-echo) that don't need any wire mechanism at all.

**Files to modify**:
- `host/robot_radio/io/sim_loop.py` / wherever `class SimTransport` lives
  (locate via `grep -rn "class SimTransport"`) — direct-patch-send +
  host-echo GET + unsupported-key error
- `host/.../protocol.py` (read-only reference — do not modify the
  existing hardware mechanism, just call it)
- `src/sim/sim_ctypes.cpp` — new `enc_scale_err_l/r` setter(s)
- `tests/testgui/test_calibration_push_on_connect.py` — revise the two
  `rotSlip`/`tw` assertions per step 6; un-skip the rest
- `tests/testgui/test_error_divergence.py` — remove `pytest.mark.skip`

**Testing plan**: as above.

**Documentation updates**: none required in `src/firm/` (host-only
change). This ticket's revision is documented in `sprint.md`'s
Architecture Revision 1 — no separate architecture file exists under
this sprint's single-doc model.
