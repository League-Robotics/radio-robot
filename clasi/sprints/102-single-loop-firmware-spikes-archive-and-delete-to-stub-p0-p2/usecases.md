---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 102 Use Cases

Parent context: this sprint executes phases P0–P2 of
`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`
("de-fiber, delete the Elite plumbing"). There is no prior formal UC-XXX for
this initiative — the issue file is the parent artifact. Each use case below
maps 1:1 to one of the sprint tickets.

**Note (stakeholder decision, 2026-07-14):** SUC-002 ("Establish the real
serial baud ceiling on both USB paths") and its ticket (002) are dropped. The
radio relay is the robot's production interface and its throughput is fixed —
raising the USB baud would only let the bench diverge from the field. SUC-001
is amended instead: the sustainable frame rate it measures through the relay
now SETS the shared telemetry rate budget for both transports, and serial
stays at 115200. The number SUC-002 is retired, not reused — see sprint.md's
Tickets section and the linked issue's revised "Rate budget" note.

## SUC-001: Verify relay telemetry push behavior before committing to it

Parent: single-loop-firmware issue, P0(a)

- **Actor**: Firmware engineer (bench operator), radio relay (`!GO` data
  plane), micro:bit robot.
- **Preconditions**: Current (pre-deletion) firmware runs on the robot; relay
  dongle connected; robot reachable both by direct USB and through the relay.
- **Main Flow**:
  1. Arm the current firmware's binary `STREAM` telemetry at ~30 Hz.
  2. Capture N minutes of frames over direct USB serial; record frame count,
     gaps, and any corruption.
  3. Repeat the same capture through the radio relay's `!GO` data plane.
  4. Compare delivered-frame rate and drop pattern between the two paths.
  5. Update `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` with
     a confirm/retract verdict on "async STREAM frames dropped by the bridge."
- **Postconditions**: A recorded, evidence-backed verdict exists: relay
  sustains a pushed telemetry stream at the target rate, OR it does not and
  the P4/P5 design must fall back to host-paced polling of the same frame.
  Additionally (stakeholder decision 2026-07-14, replacing the dropped
  SUC-002/ticket 002 baud spike): the measured sustainable rate through the
  relay, together with the measured sustainable rate over direct USB at the
  fixed 115200 baud, sets the recommended common telemetry cadence — the
  minimum of the two, with headroom — that both transports and the P4/P5
  design must honor. No baud change on either transport.
- **Acceptance Criteria**:
  - [ ] Direct-USB and relay frame-delivery rates measured over multiple
        minutes and recorded with concrete numbers (not "seems fine").
  - [ ] Knowledge note updated with the verdict and measurement method.
  - [ ] A push-vs-poll recommendation is stated for the P4/P5 designers.
  - [ ] A recommended common telemetry cadence is stated as the minimum of
        the relay-sustained and direct-USB-at-115200-sustained rates, with
        explicit headroom, as the rate budget both transports must honor —
        this is the rate-setting number that replaces the dropped
        baud-ceiling spike (former ticket 002).
  - [ ] No firmware or host code changed — measurement only.

## SUC-003: Dry-run the pruned wire-frame budget with no hardware

Parent: single-loop-firmware issue, P0(b) (relettered 2026-07-14: the
serial baud-ceiling spike that was formerly P0(b) is dropped; see the
SUC-002 note above)

- **Actor**: Firmware engineer; `scripts/gen_messages.py`; `wire.h`
  static_asserts (`source/messages/wire.h:56,58`).
- **Preconditions**: Current envelope budget confirmed
  (`kCommandEnvelopeMaxEncodedSize=168` vs. a 186 B ceiling). Draft pruned
  protos exist only on a scratch branch — never merged to `protos/` this
  sprint.
- **Main Flow**:
  1. On a scratch branch, prune `protos/envelope.proto` to
     `twist{v_x, omega, duration}` / `config{delta}` / `stop{}` + `corr_id`
     (dropping segment/replace/plan_dump/etc.).
  2. Extend `protos/telemetry.proto` with an ack ring (4–8 entries of
     `{corr_id, status, err_code}`) and fault/event bits; move
     `acc_*`/`glitch_*`/`ts_*`/`cmd_vel_*` to a slower secondary frame.
  3. Run `scripts/gen_messages.py` and build against `wire.h`.
  4. Read the static_assert pass/fail result — this IS the verdict, no
     interpretation needed.
- **Postconditions**: Either the pruned frame provably fits the 186 B
  ceiling, or the specific field(s) that don't fit are identified so P4 can
  resize before it commits to the design.
- **Acceptance Criteria**:
  - [ ] Scratch-branch protos drafted per the issue's field list.
  - [ ] `gen_messages.py` runs clean against the draft.
  - [ ] `wire.h` static_asserts pass (or the specific overflow is reported
        with byte counts).
  - [ ] Scratch branch/protos are NOT merged into this sprint's branch or
        into `protos/` on `master`.
  - [ ] No hardware involved.

## SUC-004: Establish a reversible fallback before deleting anything

Parent: single-loop-firmware issue, P1

- **Actor**: Firmware engineer; git; `mbdeploy`.
- **Preconditions**: SUC-001..003 verdicts recorded (P0 complete). Current
  firmware tree still intact (pre-P2).
- **Main Flow**:
  1. Push an annotated git tag `pre-single-loop` at the pre-deletion commit.
  2. Build and archive a known-good default `MICROBIT.hex` into `archive/`
     with its build version and flash notes.
  3. Build and archive a devicebus-bringup hex (from `codal.devicebus.json`,
     which P2 will delete) into `archive/` with the same documentation.
  4. Reflash each archived hex onto the bench robot/rig once, confirming it
     boots and identifies itself correctly, proving the artifact (not just
     the build) is good.
- **Postconditions**: Two proven, flashable rollback artifacts exist
  independent of any parked source code; the rig's devicebus-bringup image
  survives P2's deletion of its build config as a binary artifact.
- **Acceptance Criteria**:
  - [ ] `pre-single-loop` annotated tag exists and is pushed.
  - [ ] Default hex archived with version + flash notes; reflashed once and
        confirmed booting/identifying correctly.
  - [ ] devicebus-bringup hex archived with version + flash notes; reflashed
        once and confirmed booting/identifying correctly.
  - [ ] Correct device verified before flashing each time (`mbdeploy list`
        ROLE column — robot vs. RELAY dongle share `/Volumes/MICROBIT`).

## SUC-005: Delete the Elite plumbing to a flashable, banner-only stub

Parent: single-loop-firmware issue, P2

- **Actor**: Firmware engineer.
- **Preconditions**: SUC-001..004 complete (spikes de-risked; rollback
  artifacts proven). This is the irreversible step — it depends on all four
  prior use cases.
- **Main Flow**:
  1. Transcribe the `*B` base64 armor codec and `msg::wire` encode/decode out
     of `source/commands/binary_channel.cpp` into a scratch note or a
     surviving location — it is the only working framing implementation and
     is needed again in P3.
  2. Delete the full inventory: `source/main.cpp` (replaced),
     `runtime/`, `subsystems/`, `commands/`, `drive/`, `telemetry/`,
     `hal/` (capability/sim/velocity_pid/nezha remnants), `com/i2c_bus*`,
     `estimation/`, `types/{arg_schema,command_types,clock*,value_set}`,
     `kinematics/i_kinematics.h`, `devices/{bringup_main.cpp,fiber_runner.h}`
     + fiber/staging machinery in `device_bus.{h,cpp}`/`handles.h`,
     `codal.devicebus.json`, `libraries/{ruckig,tinyekf,cmon-pid}`, and the
     matching dead CMake flags/filters.
  3. Replace `source/main.cpp` with a ~50-line banner-only stub — motors are
     never energized.
  4. Prune the matching test/build/host surface (tests/_infra/{sim,drive},
     ~35 dead pytest files, `pyproject` testpaths, justfile
     `build-sim`/`build-drive` recipes, `check_config_sync` map).
  5. Land the whole thing as one commit.
  6. Verify: `just build` produces a hex; the stub flashes and banners on the
     stand (hardware bench gate — connect-only, no drive check since the stub
     never energizes motors); surviving pytest subset green; a repo-wide grep
     for every deleted header returns nothing under `source/`, `tests/`,
     `host/`.
- **Postconditions**: The tree is flashable and bootable but functionally
  inert (a firmware you can read top to bottom, doing nothing) — the
  foundation P3 builds the real single-loop `main()` onto.
- **Acceptance Criteria**:
  - [ ] `*B` armor + `msg::wire` codec transcribed out of
        `binary_channel.cpp` before that file is deleted.
  - [ ] Full delete inventory removed (source, tests, build, host), corrected
        for what commit `3c4a8c0a` already removed on this branch.
  - [ ] `source/main.cpp` replaced with a banner-only stub; no motor
        energization anywhere in the stub.
  - [ ] Lands as exactly one commit.
  - [ ] `just build` succeeds and produces a hex.
  - [ ] Stub flashed and confirmed banners on the bench stand.
  - [ ] Surviving pytest subset passes.
  - [ ] `grep` for every deleted header/symbol returns nothing under
        `source/`, `tests/`, `host/`.
  - [ ] New code is NOT created under `source/robot/` (build.py:85-90 traps
        that name) — any stub-adjacent scratch code lives elsewhere.
