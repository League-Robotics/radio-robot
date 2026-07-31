# Craftsmanship + Correctness Review — `testgui` / `testkit`

**Scope:** all `.py` under `src/host/robot_radio/{testgui,testkit}` (~13.4k lines),
reviewed against `docs/code_review/GUIDELINES.md` §1–§6, on branch
`sprint/127-host-side-path-planner-goto-and-path-following` as it stood 2026-07-30.

## Verdict

**Rework required before this is trusted on hardware again.** The transport
abstraction itself (`transport.py`'s `Transport` ABC, three backends, one
dispatch path) is genuinely well built — the intended "one Sim object / one
command plane" discipline mostly holds at that layer. But one level up, the
GUI's entire non-`Move` command surface to real hardware is silently dead:
`binary_bridge.translate_command()` — the sole route every text verb takes on
`SerialTransport`/`RelayTransport` — has been an unconditional dead stub since
`legacy_verbs`/`legacy_render` were deleted (confirmed live: `translate_command()`
returns `ERR unavailable ...` for `STOP`, `STREAM`, `SNAP`, `ZERO`, `OZ`, `SI`,
and every `S/T/R/TURN/G` command, without touching the wire). The GUI's own STOP
button discards that reply and unconditionally logs success. Because every GUI
acceptance test exercises `SimTransport` only (where an internal ABI quirk
makes `stop()` behave like a real halt), this gap is invisible to the test
suite and only shows up on the bench — precisely the failure mode this
project's own history warns about repeatedly. Layered on top of that is real
accretion: the historically-flagged `turn_shape.py` diagnostic is still
sitting in the shipped package with three divergent sim-capture paths, half of
`binary_bridge.py` is unreachable dead code by its own docstring's admission,
`testkit/` has no current non-archived consumer, and `__main__.py` is a single
~3000-line function. None of this is subtle once traced; it reads as a
sequence of unfinished migrations (104-002's verb deletion, 108-007's ABI
narrowing, 097's un-gating) that each left debris the next one built on top
of instead of clearing.

---

## Findings

### 1. CRITICAL — correctness — the GUI's STOP button does not stop a real robot

**Category:** correctness / explicitness (halt path)

**Evidence:**
- `src/host/robot_radio/testgui/operations.py:557-614` (`OpsController.on_stop`,
  wired to the "STOP" button at `operations.py:345`): calls
  `transport.command("STOP", read_timeout=300)`, discards the return value, and
  unconditionally logs `"[INFO] STOP sent"` (line 600). `STREAM 0` at line 608
  gets the same treatment.
- `src/host/robot_radio/testgui/__main__.py:2117-2122` (`_safe_stop`, the GOTO
  worker's halt-on-reach/halt-on-cancel path) and `__main__.py:2599`
  (`_set_origin`'s pre-teleport halt) both send the bare string `"STOP"`
  through `transport.send()`/`transport.command()` the same way.
- On `_HardwareTransport` (`SerialTransport`/`RelayTransport` — i.e. every real
  robot connection), `send()`/`command()` route any line `_dispatch_managed_move()`
  doesn't recognize (only `D`/`RT`/`SEG 0 <cdeg>` are intercepted,
  `transport.py:1069-1149`) into `binary_bridge.translate_command()`
  (`transport.py:1024`, `1062`).
- `binary_bridge.py`'s own docstring (lines 87-101) states
  `legacy_verbs`/`legacy_render` "were deleted wholesale by commit `129cbcb3`
  (104-002) with no replacement, and were never re-pointed here" — confirmed
  live:

  ```
  $ uv run python -c "
  from robot_radio.testgui import binary_bridge
  class DummyProto: pass
  print(binary_bridge.translate_command(DummyProto(), 'STOP'))"
  ERR unavailable legacy verb translation removed -- protocol-v4 ... has no
  binary arm for this verb -- see clasi/issues/binary-bridge-segment-replace-
  arms-deleted.md
  ```

  The `DummyProto()` passed has no methods at all and the call does not
  raise — proof `translate_command()` never touches `proto`, i.e. never builds
  or sends a `CommandEnvelope`, for `STOP`. The same is true for `STREAM 50`,
  `STREAM 0`, `SNAP`, `ZERO enc`, `OZ`, `SI 0 0 0`, and every `S/T/R/TURN/G`
  line — verified the same way. Only `SET`/`OI`/`OL`/`OA` (intercepted before
  the dead dispatch, `binary_bridge.py:302-310`) and `D`/`RT`/`SEG`
  (intercepted earlier still, in `transport.py`) actually reach the wire.

**Failing scenario:** an operator clicks STOP on a connected `SerialTransport`/
`RelayTransport` session while the robot is driving. `on_stop()` sends `"STOP"`,
gets back `"ERR unavailable legacy verb translation removed ..."`, ignores it,
and logs `"[INFO] STOP sent"`. The robot keeps driving. Nothing in the GUI
distinguishes this from a real halt.

**Compounding defect, even hypothetically:** `NezhaProtocol.stop()`
(`robot/protocol.py:1277-1296`) is documented, and confirmed by
`docs/protocol-v5.md` §3.1, to be a **planned** stop since the two-stops
rework — it queues behind whatever `Move` is active. `NezhaProtocol.estop()`
(`protocol.py:1255-1274`) is the real halt-now verb, fully implemented and
reachable directly off `transport.protocol` — but is never called anywhere in
`testgui/` (`grep -rn estop src/host/robot_radio/testgui/*.py` returns nothing).
So even if the dead stub above were fixed, every current call site sends the
wrong verb.

**Why this is invisible today:** `src/tests/testgui/test_gui_button_acceptance.py`
states up front it exercises "REAL Sim stack ... SimTransport" and monkeypatches
`SimTransport` exclusively (`test_stop_mid_tour_halts_motion_and_flushes_the_queue`,
line 934, is a Sim-only test). On `SimTransport`, `"STOP"`/`"X"` routes to
`_sim_stop()` → `SimLoop.stop()`, which — per `io/sim_loop.py:752-764`'s own
docstring — is internally retargeted to send `ESTOP` on the sim ABI ("every
existing caller of this entry point means 'halt the drivetrain now'"). So Sim
mode's STOP genuinely halts, hiding the fact that hardware mode's STOP does
nothing at all. This is the same "no hardware was connected" pattern flagged
in this project's own sprint-124 history.

**Design answer:** wire `on_stop()`/`_safe_stop()`/`_set_origin()` directly onto
`transport.protocol.estop()` (bypassing `translate_command()` entirely, the same
direct-call pattern `_handle_otos_patch`/`_handle_set_patch` already established
for `OI`/`OL`/`OA`/`SET`), check the ack, and log failure loudly — never silently.
`STREAM`/`SNAP`/`ZERO`/`OZ`/`SI` need the equivalent direct-call treatment or an
honest "not available" state in the UI (see Finding 4).

---

### 2. CRITICAL — correctness / accretion — the arrow-key drive feature is fully dead in both modes

**Category:** correctness, accretion

**Evidence:** `src/host/robot_radio/testgui/drive.py` (`KeyboardDriver`, the
whole file) builds an elaborate, carefully-engineered deadman/keepalive system
around the `DEV DT VW <fwd> 0 0`/`DEV DT STOP` wire family (`_qt_arrow_keys()`,
lines 136-151; `_STOP_CMD`, line 109; `STOP_RESEND_COUNT`, line 118;
`arm_keepalive`/`disarm_keepalive`, lines 517-536). The `DEV` verb family:
- On hardware: falls through to `binary_bridge.translate_command()`, confirmed
  dead per Finding 1 (`DEV DT VW 200 0 0` returns the same `ERR unavailable ...`).
- In Sim: `SimTransport._dispatch()` (`transport.py:1762-1802`) has no branch
  for verb `"DEV"` at all — every case in the `if/elif` chain checks for
  `STOP`/`X`, `SEG`, `D`/`RT`, `SI`, `OZ`/`ZERO`, `SET`, `GET`, `OL`/`OA`/`OI`;
  a `DEV ...` line falls to the final `else` and is logged as
  `"not supported in this sim"` (line 1801).

The file's own `attach()` comment (`drive.py:256-263`) already half-admits
this: it documents that the `DEV DT PORTS` *binding* call was dropped
2026-07-16 "out-of-process" because `DEV *` has no binary arm "on the current
wire (real robot OR sim)" — but the rest of the feature (the actual driving
commands) was left fully wired, still producing the same "not supported"
noise the removed line was cut to avoid.

**Failing scenario:** an operator connects (Sim or hardware), clicks into the
main window, and presses the Up arrow expecting the robot/avatar to drive
forward. Nothing moves. The 100 ms keepalive timer, the five-resend STOP
deadman sequence, and the arm/disarm keepalive bookkeeping all run to
completion around a command that was never going anywhere.

**Design answer:** either port `KeyboardDriver` onto `NezhaProtocol.move_twist()`
/`SimLoop.move()` (the live `Move` surface every other motion path in this
package now uses — see `transport.py`'s `_UNMANAGED_SPEED`/`run_unmanaged()`),
or delete the feature outright if teleop-by-arrow-key is no longer a supported
workflow. Leaving it as-is is the worst of both: it looks complete (docstrings,
focus-loss handling, drop-rate math) and does nothing.

---

### 3. MAJOR — placement / interface-bleed — `turn_shape.py` is the exact historically-flagged file, still in the wrong place, with three divergent sim entry points

**Category:** placement, interface-bleed

**Evidence:** `src/host/robot_radio/testgui/turn_shape.py` (364 lines) is a
turn-shape diagnostic/validation tool (`"diagnose and validate the wheel-speed
SHAPE of a single turn"`, module docstring line 1) invoked via
`python -m robot_radio.testgui.turn_shape`. This is precisely the file
`GUIDELINES.md` §1 and this review's own brief cite by name as a known-bad
placement ("turn_shape.py was wrongly placed in testgui for -m import
convenience"). It is still there.

Worse, it independently reproduces the guideline's other named historical
failure — "three divergent sim capture paths ... gave three different
answers" — inside one file:
- `capture_turn()` (lines 194-247): deterministic `SimLoop` stepping, no tick
  thread, ideal-chip defaults.
- `capture_turn_live()` (lines 64-125): a real `SimLoop` tick thread, "GUI-
  FAITHFUL" per its own docstring.
- `capture_turn_gui()` (lines 128-191): drives through `SimTransport` AND
  pushes the active robot's calibration via `calibration.push.calibration_commands()`,
  "MOST GUI-FAITHFUL" per its own docstring — explicitly different from the
  other two because it activates OTOS via `OI`, which flips the firmware's
  heading-source policy.

Three functions, three different fidelity levels, no single canonical answer
to "what does a turn actually look like" — exactly the "no owner" pattern
GUIDELINES §1 warns about, self-documented in the very comments explaining why
each is "more faithful" than the last.

**Design answer:** move to `src/tests/sim/` (or `src/tests/tools/`, matching
the domain split in `src/tests/CLAUDE.md`). Collapse to one capture path —
`capture_turn_gui()` is the most faithful and should be the only one kept;
`capture_turn()`/`capture_turn_live()` should be deleted or clearly demoted to
"quick ideal-chip smoke check, not a source of truth," not left as
equally-weighted siblings.

---

### 4. MAJOR — accretion — half of `binary_bridge.py` is unreachable dead code, by its own admission

**Category:** accretion

**Evidence:** `src/host/robot_radio/testgui/binary_bridge.py:112-119` sets
`_LEGACY_TRANSLATION_AVAILABLE = False` permanently (confirmed live, see
Finding 1). Everything gated behind it is dead in the current tree:
`translate_command()`'s main dispatch body (lines 312-341, including the
`legacy_verbs.tokenize_send_line()`/`BINARY_DISPATCH` lookups),
`_handle_binary_verb()` (349-384), `_handle_set()`/`_handle_get()`
(394-434, superseded anyway by the direct-patch `_handle_set_patch()` at the
top of the function), `_handle_stream()` (442-458), `_handle_snap()`
(461-496), and every `render.*`-based branch of `render_log_line()` (568-632,
since `render is None`). That is roughly 250 of the file's 632 lines.

The module's own docstring (lines 87-101) narrates this precisely: "launch-
unblock only ... does not restore legacy verb translation." That was true and
reasonable as a stopgap; it has been left in place since, standing as the
sole route the GUI takes for `STOP`/`STREAM`/`SNAP`/`ZERO`/`OZ`/`SI` and the
whole `S`/`T`/`R`/`TURN`/`G` verb family (Finding 1), silently returning `ERR`
for all of them.

**Design answer:** per GUIDELINES §3 ("gut decisively... Half-retired code is
worse than either state"): delete the dead dispatch tail and the four
now-unreachable `_handle_*` functions. Replace the remaining verbs one at a
time with the same direct-call pattern `_handle_otos_patch`/`_handle_set_patch`
already proved for `OI`/`OL`/`OA`/`SET` — `STOP`→`estop()`,
`STREAM`→`stream()`, `SNAP`→`snap()` all have live `NezhaProtocol` methods
already (see `protocol.py`) and need no "translation" layer at all, just a
direct call. What's left after that should not need a 630-line module.

---

### 5. MAJOR — accretion — `commands.py`'s COMMANDS schema is dead twice over

**Category:** accretion

**Evidence:** `src/host/robot_radio/testgui/__main__.py:606-738` builds one
row per entry in `commands.COMMANDS` (S/T/D/R/TURN/RT/G), then immediately
hides the whole panel: `cmd_rows_widget.setVisible(False)` (line 738), per the
comment at 729-736 — stakeholder direction 2026-07-17 replaced it with a
"Managed/Unmanaged preset-button panel." The widget is kept alive
(not deleted) only to dodge a Qt "C++ object already deleted" crash in the
connect-time enable loop (comment, lines 733-736).

Even if re-shown, five of the seven rows (`S`, `T`, `R`, `TURN`, `G`) are
independently dead per Finding 4 — they fall through to
`binary_bridge.translate_command()` and return `ERR unavailable ...` on
hardware (verified live for `S 100 100`, `T 100 100 1000`, `R 100 500`,
`TURN 9000`, `G 100 100 200`). Only `D` and `RT` work, because
`_HardwareTransport._dispatch_managed_move()` (`transport.py:1069-1149`)
intercepts those two specifically before `translate_command()` ever runs.

**Design answer:** delete the `S`/`T`/`R`/`TURN`/`G` rows, the hidden-panel
construction code in `__main__.py`, and the corresponding entries in
`commands.COMMANDS` — the "gut decisively" precedent this codebase already
applies elsewhere (per `CLAUDE.md`'s own memory of prior legacy-surface
retirements). Keep `D`/`RT` only if the Managed/Unmanaged panel doesn't
already cover their function (it appears to, per the same comment block).

---

### 6. MAJOR — placement — `testkit/` is bench/test tooling sitting in the importable host package, with no current consumer

**Category:** placement

**Evidence:** `src/host/robot_radio/testkit/` (`target.py`'s `make_target`/
`TestRobot`, `safety.py`'s `SafeRun`/`BenchRun`, `dash.py`'s `Dashboard`,
`pose.py`'s `PoseSource`/`FirmwarePose`/`CameraPose`) is explicitly bench/sim-
target infrastructure — SIGINT handling, wall-clock abort, matplotlib
dashboards, a `make_target("sim"|"bench"|"production")` factory. A repo-wide
grep for non-archived consumers finds exactly one: `testgui/transport.py:1285`
imports `read_camera_pose` from `testkit/camera.py`. Every other caller
(`make_target`, `SafeRun`, `Dashboard`) lives only under
`src/archive/tests_old/` — none of the current `src/tests/bench/*.py` scripts
(the domain `testkit` was written for, per its own docstrings) import
`testkit` at all.

This is a product/test-boundary violation independent of whether the code is
"used": per CLAUDE.md's review brief and `src/tests/CLAUDE.md`'s own stated
split, test/bench harness code belongs in `src/tests/`, never in the
importable host package — and `testkit/` is squarely that kind of code
(a `SafeRun` context manager built around SIGINT handlers and a runaway
wall-clock abort has no business being importable production surface).

**Design answer:** relocate `make_target`/`SafeRun`/`Dashboard`/`PoseSource`
to `src/tests/tools/` (or delete, if genuinely superseded — worth confirming
with the stakeholder given zero live callers). Keep only the tag-averaging
helper `read_camera_pose` (or fold it directly into `testgui/transport.py`,
its one real caller) inside the host package.

---

### 7. MAJOR — placement — `__main__.py` is one ~3000-line function

**Category:** placement (orchestration has no home)

**Evidence:** `_build_main_window()` (`__main__.py:308`–`3336`) is 3028 lines
— 90% of the file's 3349 lines and 23% of the entire `testgui`+`testkit` tree
by itself. Widget construction, the connect/disconnect state machine
(`_on_connect`/`_on_disconnect`, ~2929-3207), the tour runner
(`_make_tour_handler`/`_stop_tour`/`_on_tour_finished`, ~2632-2751), the GOTO
worker (`_GotoWorker`/`_stop_goto`, ~2753-2820), the calibration push
(`_push_robot_calibration`, 2218), and the Test-button rebuild pipeline
(3209-3316) all live as nested closures over one shared `_state` dict, rather
than as classes.

This is in sharp, self-contradicting contrast to the pattern the *same
package* already gets right: `operations.py`'s `build_panel()` +
`OpsController` split (widget factory vs. a plain, independently testable
class), `telemetry_panel.py`'s `build_telemetry_panel()` +
`_TelemetryPanelController`, and `turn_control.py`'s `TurnControlServer`.
Nothing analogous exists for connection/tour/GOTO state — it is exactly the
"long call chains ... same orchestration sequence appearing in two or three
call sites" signature GUIDELINES §2 describes, and the file's size is
substantially the residue of that, not essential complexity: contrast
`operations.py` at 1054 lines covering seven buttons with full docstrings and
tests, against `__main__.py`'s single function covering everything else.

**Design answer:** extract `ConnectionController`, `TourController`, and
`GotoController` classes following the `OpsController` template already
established in this same package — each with its own state, testable without
a `QApplication` event loop wired through 3000 lines of closures.

---

### 8. MAJOR — placement — the "Test S/T/US/UT" buttons embed a full build pipeline in the shipped GUI

**Category:** placement (diagnostic tooling in the product tree)

**Evidence:** `__main__.py:3209-3316`. Clicking "Test S" (or T/US/UT) runs, on
a background thread, from inside the running GUI process:
`gen_version.py`, `gen_messages.py`, then
`cmake --build src/sim/build --parallel --target firmware_host`
(lines 3281-3296), copies the fresh dylib to a per-PID temp path, hot-swaps it
into a reconnected `SimTransport` via `set_sim_lib_override()`, resets the
origin, and runs one of four fixed regression scenarios (managed
straight/turn via `D`/`RT`, unmanaged straight/turn via `run_unmanaged()`).

This is exactly the "embedded diagnostic/capture/validation tooling" this
review's brief asks to audit for: an edit-compile-hot-reload-and-verify smoke
harness, wired to buttons in the operator cockpit rather than living as a
standalone script a developer runs from the command line.

**Design answer:** move this to a `src/tests/sim/` script (it already shares
`_sim_lib_path()`/`set_sim_lib_override()` with `transport.py`, so the
plumbing is reusable) invoked directly by a developer. `testgui` should load a
build; it should not trigger one.

---

### 9. MINOR — interface-bleed — `turn_control.py`'s external control socket has no current caller

**Category:** interface-bleed / explicitness

**Evidence:** `turn_control.py`'s `TurnControlServer` (a TCP JSON-line socket
on port 8127, letting an external process pivot the robot / send arbitrary
lines / pull recorded series while a human watches) is fully wired into
`__main__.py:378,1418-1437`, but a repo-wide grep for a client (anything
connecting to port 8127, or importing `TurnControlServer`) outside
`turn_control.py`/`__main__.py` itself returns nothing — no script under
`src/tests/` exercises it.

This is architecturally the *right* shape (GUIDELINES §1: one entry point,
since the GUI owns the sole serial port) and is not, on its own, a
correctness defect — but a control surface with zero exercised callers is
unverified by construction. Recommend either adding a small bench/test script
that drives it (proving the shape works end-to-end) or removing it until one
exists.

---

### 10. NOTE — possible pose-estimator proliferation (unverified — flagged for ownership check, not asserted as duplication)

**Category:** accretion (candidate)

`traces.py`'s `EncoderDeadReckoner` is one more independent dead-reckoning
implementation alongside `sensors/odom_tracker.py`, `sensors/odometry.py`,
`planner/heading.py`, `planner/tour.py`, and `nav/navigator.py` (found via
`grep -rl "dead.reckon"`). This review did not read those modules closely
enough to assert duplicated *logic* — `traces.py`'s own docstring frames its
copy as a display-only fallback for a binary telemetry field that isn't
always present — but GUIDELINES §1 explicitly cites "three pose estimators
with no owner" as this exact codebase's own historical failure mode, so this
combination is worth a dedicated ownership pass rather than being waved
through on this review's evidence alone.

---

### 11. NOTE — telemetry_panel.py has no visible staleness indicator

**Category:** correctness (candidate)

`telemetry_panel.py`'s `_TelemetryPanelController.update_frame()` correctly
distinguishes "field absent" (renders `"—"`) from "field is zero" (renders
`"+0"`) at the per-value level — good. But nothing in the panel shows the
operator when telemetry has stopped arriving altogether (reader thread died,
link dropped): the last-received values simply sit frozen, visually identical
to fresh ones. A last-updated timestamp or a "no frames in Ns" banner would
close this gap; not filed as MINOR because I did not find a call site where
this has caused a real incident, unlike Findings 1–2.

---

### 12. NOTE — phantom issue reference

`clasi/issues/binary-bridge-segment-replace-arms-deleted.md` is cited three
times in code comments (`binary_bridge.py`, `commands.py`, `transport.py`) and
once in the dead-stub's own `ERR` reply text, but does not exist under
`clasi/issues/` (open or done) — either it was resolved and mis-archived, or
never actually filed. Worth reconciling regardless of Finding 4's disposition.

---

## What's good

- **`transport.py`'s `Transport` ABC is the model example of GUIDELINES §1.**
  One abstract interface, three concrete backends (`SerialTransport`/
  `RelayTransport`/`SimTransport`), and no branching on backend type anywhere
  else in the GUI — callers just hold a `Transport`. The completion-ack
  polling (`_await_move_completion()`) and single-consumer telemetry-queue
  handoff (`suspend_telemetry_reader`/`resume_telemetry_reader`) show real
  attention to "never record success before it happened."
- **`camera_prefs.py` is a textbook fix of exactly the failure mode this
  review is watching for.** Its own docstring cites the historical incident
  (three camera-consuming call sites picking a camera three different ways)
  and centralizes resolution into one `select_camera()` helper, consumed
  identically by `operations.py`, `live_view.py`, and `__main__.py`.
- **`operations.py`'s `build_panel()` / `OpsController` split** (widget
  factory vs. a plain, Qt-decoupled, independently testable controller class)
  is the right shape and should be the template for Finding 7.
- **Threading discipline is consistently correct.** Every background worker
  marshals back to the Qt main thread via a `QObject`/`Signal` bridge with
  `QueuedConnection` (`turn_control.py`'s `_Bridge`, `live_view.py`'s
  `frame_ready`, `operations.py`'s `_GrabBridge` in `trigger_live_grab`) — no
  direct cross-thread widget access was found anywhere in the package.
- **The historical narration in docstrings (`transport.py`, `sim_prefs.py`,
  `binary_bridge.py`) is unusually good explicitness discipline** — every
  non-obvious piece of state says why it exists, what it used to be, and what
  broke last time it changed. Several of the findings above were only
  provable *because* the code itself documents its own gaps this honestly.
