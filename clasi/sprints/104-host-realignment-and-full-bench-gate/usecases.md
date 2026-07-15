---
sprint: '104'
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 104: Host realignment and full bench gate

Continues SUC numbering from sprint 103 (SUC-001..SUC-010). This sprint is
P5 remainder + P6 of `single-loop-firmware-p3-p7-continuation.md`.

## SUC-011: Host command surface completed — config arm + ack-ring ergonomics

- **Actor**: Firmware/host engineer scripting the rig.
- **Preconditions**: Sprint 103's `NezhaProtocol.twist()`/`stop()` +
  ack-ring matcher exist; `protos/envelope.proto`'s `ConfigDelta` arm is
  schema-defined (103 Step 7 Open Question 3) but has no host builder.
- **Main Flow**: A host script calls a new `NezhaProtocol.config(**deltas)`
  builder; it constructs and sends a `ConfigDelta` envelope; the ack-ring
  matcher (already built in 103) confirms receipt the same way it confirms
  twist/stop.
- **Postconditions**: Every `CommandEnvelope` oneof arm shipped by 103's
  schema (`twist`, `config`, `stop`) has a host-side builder — no arm is
  schema-only.
- **Acceptance Criteria**:
  - [ ] `NezhaProtocol.config()` builds and sends a `ConfigDelta` envelope.
  - [ ] Ack observed via the existing ack-ring matcher.
  - [ ] `source/app` dispatch behavior for `config` (live-apply vs.
        `ERR_UNIMPLEMENTED`, 103 Step 7 Q3) is confirmed against the
        merged 103 firmware and documented here, not assumed.

## SUC-012: Legacy translator and dead-verb deletion

- **Actor**: Any developer or CI run touching `host/robot_radio/`.
- **Preconditions**: Sprint 103's Decision 4 left ~30 orphaned
  `NezhaProtocol`/`SerialConnection` methods in place (drive/arc/vw/
  segment/turn/go_to/stream/pose_fix/get_config-by-legacy-arm/etc.) whose
  target `CommandEnvelope` arms no longer exist; ~112-117 `tests/unit`
  tests currently fail or error against the merged 103 firmware/schema
  (measured 2026-07-14 against the pre-104 tree: 112 failed, 5 errors,
  297 passed).
- **Main Flow**: For each orphaned method/test: delete the method and its
  now-meaningless test if no surviving use case needs it, OR fix it if it
  legitimately targets a still-live wire arm (e.g. a test asserting
  schema/encoding correctness of `ConfigDelta` that just needs updating,
  not deleting). No test is left red or silently skipped.
- **Postconditions**: `host/robot_radio/robot/protocol.py`,
  `host/robot_radio/io/serial_conn.py`, and `host/robot_radio/cli.py`
  contain only methods that target a live wire arm; `uv run python -m
  pytest tests/unit` is green.
- **Acceptance Criteria**:
  - [ ] Every currently-failing/erroring `tests/unit` test is triaged:
        fixed (if it targets a live arm) or deleted alongside its dead
        target method (if not) — none left red, none silently
        `xfail`/`skip`-wrapped to hide the count.
  - [ ] `grep` for the retired verb names (`drive(`, `arc(`, `vw(`,
        `segment(`, `turn(`, `go_to(`, `stream(`, `pose_fix(`, ...) across
        `host/` returns no remaining callers outside of intentionally-kept
        historical/CLI-help text.
  - [ ] `uv run python -m pytest tests/unit -q` reports 0 failed, 0 errors.

## SUC-013: `serial_conn` ack-ring matcher hardening + `TelemetrySecondary` consumption

- **Actor**: Any host script or bench tool reading telemetry.
- **Preconditions**: 103's ack-ring matcher lives inline in
  `NezhaProtocol` as a minimal slice; `TelemetrySecondary`'s wire framing
  was decided by ticket 103-001 (Decision 3) but no host consumer reads it
  yet.
- **Main Flow**: The ack-ring matching logic is hardened/promoted into
  `serial_conn.py` (bounded timeout, re-delivery tolerance, ring-wrap
  detection per 103 Decision 2's documented constraint) so every host
  caller — not just `NezhaProtocol`'s two methods — gets the same
  guarantee; `serial_conn.py` also decodes `TelemetrySecondary` frames
  (whatever shape ticket 103-001 chose) and exposes their fields
  (acc/glitch/ts/cmd_vel) on the same `TLMFrame`-like surface primary
  telemetry already uses.
- **Postconditions**: A single, well-tested ack-ring matcher is the one
  implementation in the tree; secondary-frame fields are readable from
  host code the same way primary fields are.
- **Acceptance Criteria**:
  - [ ] Ack-ring matcher lives in `serial_conn.py` (or an equally shared
        location), not duplicated per-caller; `NezhaProtocol` calls the
        shared implementation.
  - [ ] Matcher has unit coverage for: exact match, tolerated re-delivery,
        ring-wrap (an older `corr_id` evicted before observed — documented
        as a real, bounded failure per 103 Decision 2), and timeout.
  - [ ] `TelemetrySecondary` fields are decoded and exposed host-side;
        a unit test round-trips a synthetic secondary frame.

## SUC-014: Firmware fault-bit follow-ups from 103 ticket 010

- **Actor**: Firmware engineer; bench operator reading `fault_bits`.
- **Preconditions**: `source/app/comms.cpp`'s `malformedCount_` has
  existed since 103-004 with an explicit forward-reference comment
  ("surfaced later as a Telemetry fault bit (ticket 005)") that 103-005
  did NOT implement — confirmed by reading `telemetry.h`'s actual
  `fault_bits` layout (bits 0-2 only: `kFaultI2CSafetyNet`,
  `kFaultWedgeLatch`, `kFaultI2CNak`; bit 3+ unclaimed). Separately, 103
  ticket 010's bench session found `kFaultI2CSafetyNet` is a boot-time
  one-shot latch (fires once during `Preamble`, never during driving) and
  recommended the doc comment say so explicitly so a future bench reader
  does not chase a healthy `fault=1` as a live problem.
- **Main Flow**: (a) Add `kFaultCommsMalformed` at bit 3 of
  `fault_bits` (schema stays a plain `uint32`, no `.proto` change needed —
  confirm this against the actual `telemetry.proto` field type before
  ticket execution, since the prompt's "needs a schema bit" language may
  mean only the bit constant + wiring, not a proto field addition); wire
  `Comms::malformedCount()` into it in `main.cpp`, matching the exact
  pattern already used for `I2CBus::clearanceSafetyNetCount()`. (b) Update
  `telemetry.h`'s `kFaultI2CSafetyNet` doc comment to state the observed
  boot-time-one-shot characterization from 103-010, so it stops reading as
  an ambiguous "could fire anytime" bit.
- **Postconditions**: A malformed/undecodable inbound frame is visible on
  the wire via a fault bit, not only via an internal counter no host tool
  reads; the safety-net bit's true behavior is documented where a reader
  will find it.
- **Acceptance Criteria**:
  - [ ] `kFaultCommsMalformed` bit defined, wired from
        `Comms::malformedCount()`, and documented in `telemetry.h`'s
        `fault_bits` comment block (matching the existing bits 0-2 format).
  - [ ] A firmware unit test (`HOST_BUILD`) confirms a malformed frame
        sets the bit.
  - [ ] `kFaultI2CSafetyNet`'s doc comment states the boot-time one-shot
        characterization from 103-010's bench finding.
  - [ ] This sprint's own P6 soak run (SUC-017) confirms
        `kFaultCommsMalformed` stays clear during a clean soak (no
        malformed frames from the host's own well-formed traffic) and that
        `kFaultI2CSafetyNet` does not re-trip after the boot-time latch
        (corroborating the characterization under sustained load, not just
        the short 103-010 session).

## SUC-015: Rig profile — persistent OTOS-untrusted marker

- **Actor**: Bench operator / future host-fusion code (sprint 106+).
- **Preconditions**: `clasi/issues/rig-persistent-otos-distrust.md` — the
  bench rig's OTOS is servo-mounted and structurally decoupled from the
  wheels; under the pre-single-loop architecture this required a
  per-session manual `SET ekfROtosTheta=1e9 ekfROtosXy=1e9` ritual to stop
  a poisoned fused pose from blocking motion. Under the single-loop
  architecture the robot no longer fuses pose on-robot at all (Odometry
  reports raw encoder pose; OTOS is sampled and reported raw) — 103
  ticket 010's own bench session drove the rig cleanly with NO manual SET,
  which is first-hand evidence the original failure mode (segments
  admitted/ACKed but never executed) is structurally gone for THIS
  firmware. What remains is future-proofing: host-side pose fusion doesn't
  exist yet (that's sprint 106+), and when it's built it must know, from
  the rig's own profile, not to trust the rig's OTOS.
- **Main Flow**: Add a persistent boolean field to the rig's robot profile
  (`data/robots/tovez_nocal.json` or a dedicated rig profile — a
  ticket-time naming decision) marking OTOS as mechanically decoupled/
  untrusted; document it as the field a future host-fusion sprint must
  read before trusting `otos` telemetry.
- **Postconditions**: The rig's "this OTOS does not track the wheels" fact
  is persisted in version control, not tribal knowledge or a per-session
  `SET`.
- **Acceptance Criteria**:
  - [ ] A persistent field exists in the rig's robot profile marking OTOS
        untrusted, with a comment/doc note explaining why (servo-mounted,
        mechanically decoupled).
  - [ ] Re-verified on the actual rig: reboot, drive a twist with NO
        manual `SET`, motion executes and reported (encoder) pose tracks —
        confirming 103-010's finding holds on THIS sprint's tree too, not
        just as a one-off observation.
  - [ ] `clasi/issues/rig-persistent-otos-distrust.md` is updated or
        closed reflecting the current (single-loop) architecture's actual
        resolution of its root failure mode, with the persisted flag noted
        as the remaining forward-looking piece for sprint 106+.

## SUC-016: Bench script family rewritten to the binary twist/stop plane

- **Actor**: Bench operator running `tests/bench/rig_dev.py`/`rig_soak.py`.
- **Preconditions**: `rig_dev.py`/`rig_soak.py` (and
  `tests/bench/device_bus_bringup.py`,
  `tests/unit/test_device_bus_bringup_bench.py`) assume the pre-103
  segment/drive wire surface and `DeviceBus`-era bringup image — both
  retired by sprint 103 (103's own Impact on Existing Components section
  flags this explicitly as deferred to 104).
- **Main Flow**: Rewrite `rig_dev.py`/`rig_soak.py` onto
  `twist`/`config`/`stop` + the ack-ring matcher (SUC-011/013); retire or
  rewrite `device_bus_bringup.py`/`test_device_bus_bringup_bench.py` (their
  target class, `Devices::DeviceBus`, no longer exists in the tree — 103
  Decision 1).
- **Postconditions**: The bench script family runs against 103/104's
  firmware; no bench tool in the tree still targets deleted wire arms or
  the deleted `DeviceBus` bringup image.
- **Acceptance Criteria**:
  - [ ] `rig_dev.py` drives the rig interactively over the binary plane.
  - [ ] `rig_soak.py` runs a sustained twist/stop loop over the binary
        plane, logging drop rate, fault/event bits, and encoder motion.
  - [ ] `device_bus_bringup.py`/its test are rewritten against `Preamble`
        (103's replacement for `DeviceBus`'s boot sequencing) or retired
        with a documented reason if no equivalent bringup diagnostic is
        needed post-103.

## SUC-017: P6 soak gate — sustained, dual-transport, bench-runnable

- **Actor**: Bench operator; the physical rig (wheels off the ground,
  `.claude/rules/hardware-bench-testing.md`).
- **Preconditions**: SUC-011 through SUC-016 complete; firmware from
  sprint 103 (+ SUC-014's fault-bit additions) flashed.
- **Main Flow**: Run SUC-016's rewritten `rig_soak.py` for a sustained
  window (materially longer than 103-010's short bench-gate captures — a
  ticket-time duration decision, informed by 103-010's own 120s/1875-frame
  USB continuity capture as a floor, not a ceiling) over BOTH direct USB
  and the radio relay; repeat the deadman kill-test under soak load (not
  just at idle); observe fault/event bits throughout.
- **Postconditions**: Full host tooling drives the robot over the new
  binary plane on both transports; a sustained soak run is clean per the
  Success Criteria below. This IS this sprint's Definition of Done — no
  sprint in this arc closes on tests alone.
- **Acceptance Criteria**:
  - [ ] Zero I2C NAK/timeout errors (`kFaultI2CNak` stays clear) over the
        soak window, both transports.
  - [ ] Zero *sustained* wedge latch during active driving (the bit is
        expected to assert transiently at idle per its own documented
        contract — 103-010 §6 — so the acceptance bar is "clears promptly
        once motion resumes," not "never sets").
  - [ ] A measured (not assumed) TLM drop rate is reported for both
        transports, replacing 103-010's short-window numbers with a soak-
        duration measurement.
  - [ ] Deadman kill-test repeated under soak conditions (mid-soak host
        kill), both transports; wheels stop within one stale window.
  - [ ] `kFaultI2CSafetyNet` does not re-trip after its boot-time latch
        during the soak window (corroborates SUC-014's characterization
        under sustained load).
  - [ ] No motor left energized at the end of the session.
