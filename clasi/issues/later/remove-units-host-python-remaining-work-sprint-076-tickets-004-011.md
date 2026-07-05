---
status: pending
---

# Remove units from identifier names — host Python: remaining work (retrofitted tickets 004–011 from sprint 076)

## Provenance — READ FIRST

**This issue is the direct product of sprint 076's detail ticket planning.**
Sprint 076 ("Remove units from identifier names — host Python, codebase-wide
rename, wire keys stable") was planned in full — architecture update written,
self-reviewed (verdict: APPROVE), stakeholder-approved, and broken into 11
sequenced tickets — but was **closed early at stakeholder direction on
2026-07-03** after tickets 001–003 landed. The eight unexecuted tickets
(004–011) are preserved **verbatim** below so a future sprint can re-adopt
them with minimal re-planning. There is a lot here; it is already
implementation-grade.

**Completed in sprint 076 (do NOT redo):**
- 001 transport primitives (`io/serial_conn.py`, `io/sim_conn.py`) — commit `049ae0a`
- 002 wire-protocol adapter (`robot/protocol.py`, e.g. `read_ms`→`read_timeout`,
  `x_mm/y_mm/h_cdeg`→`x/y/heading`) — commit `5797a88`
- 003 robot object model (`robot/robot.py`, `nezha*.py`, `cutebot.py`,
  `clock_sync.py`, `sync_pose.py`, `kinematics/differential_drive.py`) —
  commit `96ecc59`

**Remaining ticket dependency graph** (with 001–003 done): 004, 005, 006, 007
are all immediately unblocked; 008 ← 007; 009 ← 004+005+006; 010 ← 005+007;
011 (final sweep/certification) ← 006+009+010.

**Key references:** the sprint 076 archive (architecture-update.md contains
the authoritative exclusion table, naming decisions, and per-file occurrence
counts), `.claude/rules/coding-standards.md` (leading `# [unit]` comment convention,
established sprint 071), and the parent issue
`remove-units-from-identifier-names-host-python.md` (in the sprint 076
archive's `issues/` directory).

**Standing rules for all tickets below** (from the approved architecture):
pure rename, no behavioral change; wire SET/GET/SIMSET key strings and
TLM/SNAP tokens keep firmware spelling byte-identically; `set_config(**kwargs)`
kwarg names are wire keys — excluded; `config/robot_config.py` pydantic models
excluded wholesale (attr name IS the JSON key), including flat-accessor
`@property` proxies; sim_prefs mapping-table SIMSET-key values excluded;
every renamed declaration carries a `# [unit]` comment; the `read_ms`→
`read_timeout` convergence is a per-ticket obligation at every call site a
ticket touches. Note: ticket texts below cite the pre-076 test baseline
("2682 passed") — re-baseline before starting.

---

The eight ticket files follow verbatim (original frontmatter retained for
ids, use-cases, and depends-on).



<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 004-sensor-modules-rename-unit-suffixed-identifiers-in-host-side-sensor-reading-and-classification.md -->
<!-- ================================================================ -->

---
id: '004'
title: 'Sensor modules: rename unit-suffixed identifiers in host-side sensor reading
  and classification'
status: open
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sensor modules: rename unit-suffixed identifiers in host-side sensor reading and classification

## Description

`host/robot_radio/sensors/otos.py`, `color.py`, `calibration.py`,
`motion_monitor.py`, and `odom_tracker.py` read and classify onboard
sensors (line, color, OTOS, encoder-derived odometry) from the host side.
This subsystem depends only on `robot/protocol.py` (via `parse_tlm`,
ticket 002 — already renamed) and `io/serial_conn.py` (ticket 001), **not**
on the `robot/` object-model layer (ticket 003) — independent of that
ticket's rename order (`architecture-update.md` Step 2).

Renames (per Step 5): `sensors/odom_tracker.py` (`x_mm`/`y_mm`/
`trackwidth_mm` → bare names with `# [mm]`); `sensors/otos.py`;
`sensors/color.py` (`h_deg` → `hue  # [deg]`, `brightness_pct` →
`brightness  # [%]`); `sensors/calibration.py`; `sensors/motion_monitor.py`.

`sensors/odometry.py` has zero unit-suffix hits (already clean, Step 1) and
is in this ticket's review scope only because it is imported by files that
do have hits — no edit expected. `robot/protocol.py` is consumed here (via
`parse_tlm`) but not re-renamed — that was ticket 002's concern.

Total scope: 121 rename-eligible occurrences (Step 3).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Sensor readings and
  classifications must be bit-for-bit identical to pre-076 for the same
  input stream.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire tokens are STABLE**: this subsystem parses `robot.protocol`'s
  already-unchanged `kv` dict keys (`enc`, `pose`, `otos`, `encpose`,
  `otos_health`, `line`, `color`) — do not touch any string literal that
  matches one of these wire tokens inside `sensors/`.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py`
  (`parse_tlm`, etc.) using a ticket-002-renamed keyword argument must
  already use the converged name; fix any stale one found in this ticket's
  files here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `sensors/odom_tracker.py`: `x_mm`/`y_mm`/`trackwidth_mm` → bare names
      with `# [mm]`.
- [ ] `sensors/otos.py`: all unit-suffixed identifiers renamed with
      `# [unit]` comments matching the file's existing unit vocabulary.
- [ ] `sensors/color.py`: `h_deg` → `hue` with `# [deg]`; `brightness_pct`
      → `brightness` with `# [%]`.
- [ ] `sensors/calibration.py`, `sensors/motion_monitor.py`: all
      unit-suffixed identifiers renamed with `# [unit]` comments.
- [ ] `sensors/odometry.py` is confirmed to remain clean (zero
      unit-suffixed identifiers) — no edit expected.
- [ ] No wire-token string literal (`"enc"`, `"pose"`, `"otos"`,
      `"encpose"`, `"otos_health"`, `"line"`, `"color"`, etc.) is altered
      anywhere in this ticket's files — diff-confirm byte-identical to
      pre-076.
- [ ] Sensor-related unit tests in `tests/simulation/unit/` (per
      `usecases.md` SUC-003) pass with unchanged numeric assertions.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: sensor-related unit tests in
  `tests/simulation/unit/` (grep for `Otos`/`ColorClassifier`/
  `OdomTracker`/`MotionMonitor` imports to enumerate the exact files).
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file; this subsystem has no internal
dependency on ticket 003 (robot object model), so it can proceed
immediately once ticket 002 lands.

1. `sensors/odom_tracker.py` — rename `x_mm`/`y_mm`/`trackwidth_mm` and any
   other unit-suffixed field/local.
2. `sensors/otos.py` — rename all unit-suffixed identifiers.
3. `sensors/color.py` — rename `h_deg` → `hue`, `brightness_pct` →
   `brightness`, and any other unit-suffixed identifier.
4. `sensors/calibration.py`, `sensors/motion_monitor.py` — rename
   remaining unit-suffixed identifiers.
5. Confirm `sensors/odometry.py` needs no edit.
6. Grep each renamed identifier across this file set to confirm no
   internal call site was missed, and confirm every wire-token string
   literal this subsystem reads is untouched.
7. Run sensor-related unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/sensors/otos.py`
- `host/robot_radio/sensors/color.py`
- `host/robot_radio/sensors/calibration.py`
- `host/robot_radio/sensors/motion_monitor.py`
- `host/robot_radio/sensors/odom_tracker.py`
- `host/robot_radio/sensors/odometry.py` — reviewed only, no edit expected.

**Testing plan**: Run sensor-related unit tests individually, then
`uv run python -m pytest -q` and confirm the 2682 baseline holds.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 005-calibration-modules-rename-unit-suffixed-identifiers-in-calibration-workflows.md -->
<!-- ================================================================ -->

---
id: '005'
title: 'Calibration modules: rename unit-suffixed identifiers in calibration workflows'
status: open
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Calibration modules: rename unit-suffixed identifiers in calibration workflows

## Description

`host/robot_radio/calibration/helpers.py`, `linear.py`, `angular.py`,
`push.py`, and `fit_sim_error_model.py` push calibration values to firmware
(`SET`) and fit the sim error model (`SIMSET`). This subsystem depends only
on `robot/protocol.py` (`parse_tlm`, `TLMFrame` — ticket 002, already
renamed) and owns the one host-side file (`fit_sim_error_model.py`) with
direct `SIMSET`-key dict literals, plus the `sim_prefs.py`-adjacent
two-layer key pattern that ticket 008 will also need to understand.

Renames (per Step 5): `calibration/angular.py` (`target_deg`/`otos_deg`/
`gt_deg`/`cam_deg`/`achieved_deg` → bare names with `# [deg]`);
`calibration/linear.py` (`actual_mm`/`otos_x_mm` → bare names with
`# [mm]`); `calibration/fit_sim_error_model.py` (`total_ms`/
`sample_period_ms` → bare names with `# [ms]`; `SIMSET_BOUNDS`/
`DEFAULT_CANDIDATE_KEYS` dict-key *strings* untouched, see Exclusion Table
below); `calibration/push.py` (`left_mm_per_deg`/`right_mm_per_deg` →
**recommended** `wheel_travel_calib_left`/`wheel_travel_calib_right` with
`# [mm/deg]`, mirroring 071's own `wheelTravelCalibL/R` derived-unit-name
choice on the firmware side — a recommendation, not a requirement; per
Open Question 5, choose a different descriptive name if this collides with
anything in this file).

`calibration/helpers.py` is already clean (zero unit-suffix hits, Step 1)
but is in this ticket's review scope to confirm.

Total scope: 175 rename-eligible occurrences (Step 3).

## Wire-Compatibility Exclusions Relevant to This Ticket

(Restated from `architecture-update.md`'s Wire-Compatibility Exclusion
Table — do **not** rename any of the following in this ticket's files.)

- Every `SET`/`GET` wire-key string 071 already confirmed unit-free (`ml`,
  `mr`, `tw`, `rotSlip`, `odomOffX/Y`, `odomYaw`, `sTimeout`, etc.) — these
  appear as literal strings in `push.py`'s command builders. Only the
  surrounding Python variable/parameter names that hold the *value* are
  renamed.
- `set_config(**kwargs)` call sites' keyword-argument names (e.g.
  `set_config(ml=..., mr=...)`) — the kwarg name **is** the wire key at
  that call site. Re-verify before editing any `set_config(...)` call in
  this ticket's files that no target identifier collides with a live kwarg
  name (this pass found zero such collisions).
- `SIMSET_BOUNDS`, `DEFAULT_CANDIDATE_KEYS`, and step-size dict keys (e.g.
  `"trackwidthMm"`) in `fit_sim_error_model.py` — direct wire-key-as-dict-key,
  single layer, matching `SimCommands.cpp`'s `kSimRegistry[]` pattern.
  **Exclude wholesale.**

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Calibration fit/replay
  round-trips must produce numerically identical results.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire keys are STABLE** per the exclusions above — every `SET`/`SIMSET`
  key string literal in `calibration/` stays byte-identical (diff-verify).
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py` using a
  ticket-002-renamed keyword argument (e.g. `read_timeout=`) must already
  use the converged name; fix any stale one found here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `calibration/angular.py`: `target_deg`/`otos_deg`/`gt_deg`/`cam_deg`/
      `achieved_deg` → bare names with `# [deg]`.
- [ ] `calibration/linear.py`: `actual_mm`/`otos_x_mm` → bare names with
      `# [mm]`.
- [ ] `calibration/fit_sim_error_model.py`: `total_ms`/`sample_period_ms` →
      bare names with `# [ms]`; `SIMSET_BOUNDS`/`DEFAULT_CANDIDATE_KEYS`
      dict-key strings are byte-identical to pre-076 (diff-confirm).
- [ ] `calibration/push.py`: `left_mm_per_deg`/`right_mm_per_deg` renamed
      to a descriptive quantity name with `# [mm/deg]` (recommended:
      `wheel_travel_calib_left`/`wheel_travel_calib_right`, per Open
      Question 5 an implementation-time judgment call, not a hard
      requirement).
- [ ] `calibration/helpers.py` is confirmed to remain clean (zero
      unit-suffixed identifiers) — no edit expected.
- [ ] Every `SET`/`SIMSET` key string literal in `calibration/` is
      unchanged (diffed against pre-076): `ml`, `mr`, `tw`, `rotSlip`,
      `odomOffX/Y`, `odomYaw`, and every `SIMSET_BOUNDS`/
      `DEFAULT_CANDIDATE_KEYS`/step-size dict key (`trackwidthMm`, etc.).
- [ ] `NezhaProtocol.set_config(**kwargs)` call sites in this ticket's
      files that pass a wire key as a keyword argument are **not** touched.
- [ ] `tests/simulation/unit/test_calibration_push.py`,
      `test_calibrate_linear.py`, `test_calibration_helpers.py` (per
      `usecases.md` SUC-004) pass unchanged.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_calibration_push.py`,
  `tests/simulation/unit/test_calibrate_linear.py`,
  `tests/simulation/unit/test_calibration_helpers.py`.
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file, treating `fit_sim_error_model.py`'s
dict-key literals as a hard exclusion boundary throughout.

1. `calibration/angular.py`, `calibration/linear.py` — rename
   unit-suffixed identifiers per the mapping above.
2. `calibration/fit_sim_error_model.py` — rename local
   variables/parameters only; leave every `SIMSET_BOUNDS`/
   `DEFAULT_CANDIDATE_KEYS` dict key exactly as-is.
3. `calibration/push.py` — rename `left_mm_per_deg`/`right_mm_per_deg`
   (recommended target name above); leave every `SET` command's wire-key
   string literal exactly as-is.
4. Confirm `calibration/helpers.py` needs no edit.
5. Grep this file set for every `SET`/`SIMSET` key string to confirm none
   were altered, and for every renamed identifier's old name to confirm no
   internal call site was missed.
6. Run the three named calibration unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/calibration/linear.py`
- `host/robot_radio/calibration/angular.py`
- `host/robot_radio/calibration/push.py`
- `host/robot_radio/calibration/fit_sim_error_model.py`
- `host/robot_radio/calibration/helpers.py` — reviewed only, no edit
  expected.

**Testing plan**: Run
`tests/simulation/unit/test_calibration_push.py`,
`test_calibrate_linear.py`, `test_calibration_helpers.py` individually,
then `uv run python -m pytest -q` and confirm the 2682 baseline holds.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 006-navigation-modules-rename-unit-suffixed-identifiers-in-go-to-and-path-approach-math.md -->
<!-- ================================================================ -->

---
id: '006'
title: 'Navigation modules: rename unit-suffixed identifiers in go-to and path-approach
  math'
status: open
use-cases:
- SUC-005
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Navigation modules: rename unit-suffixed identifiers in go-to and path-approach math

## Description

`host/robot_radio/nav/navigator.py`, `_approach_utils.py`, `camera_goto.py`,
and `nav_params.py` compute navigation/path-following commands (go-to,
PurePursuit-adjacent approach math). `nav/camera_goto.py` has **zero**
`robot_radio` imports (pure math, duck-typed robot argument) — independent
of every other subsystem's rename order; `navigator.py` depends on
`controllers/pid.py` (zero hits) and its own `nav/` siblings only
(`architecture-update.md` Step 2).

**Dependency note**: filed with `depends-on: [002]` rather than as a
second root ticket. `architecture-update.md`'s Step 4a dependency-graph
diagram omits an explicit `T002 → T006` edge, but Step 5's own "Why"
section states plainly that "Tickets 004/005/006 are mutually independent
(each depends only on 002 ...)" — and `usecases.md` SUC-005's Main Flow
confirms navigation "issues drive commands through the (already-renamed,
SUC-001) protocol layer using renamed locals." This ticket resolves that
minor diagram/prose inconsistency conservatively (depend on 002) so that
any renamed protocol-layer keyword argument this subsystem calls with is
guaranteed already converged (Decision 2) before this ticket starts — a
ticketing-detail decision within the sprint-planner's dependency-ordering
authority, not a reopening of the architecture review.

Renames (per Step 5): `nav/navigator.py` (`tolerance_mm`/`speed_mms` →
bare names with `# [unit]`); `nav/_approach_utils.py` (`r_mm` → `radius
# [mm]`); `nav/camera_goto.py` (`target_mm` → `target  # [mm]`);
`nav/nav_params.py`.

`nav/pose.py`, `nav/pose_align.py`, and `controllers/pid.py` are already
clean (zero unit-suffix hits, Step 1) — no edit expected.

Total scope: 163 rename-eligible occurrences (Step 3).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Navigation trajectories and
  arrival decisions must be numerically identical to pre-076 for the same
  inputs.
- **Every renamed declaration carries a `# [unit]` comment.**
- **No wire-key surface in this layer** — navigation issues drive commands
  through the already-renamed `robot/protocol.py` (ticket 002); nothing in
  `nav/` itself builds a wire string directly.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py` or
  `robot/`-layer methods using a ticket-002/003-renamed keyword argument
  must already use the converged name; fix any stale one found here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `nav/navigator.py`: `tolerance_mm`/`speed_mms` → bare names with
      `# [unit]`.
- [ ] `nav/_approach_utils.py`: `r_mm` → `radius` with `# [mm]`.
- [ ] `nav/camera_goto.py`: `target_mm` → `target` with `# [mm]`.
- [ ] `nav/nav_params.py`: all unit-suffixed identifiers renamed with
      `# [unit]` comments.
- [ ] `nav/pose.py`, `nav/pose_align.py`, `controllers/pid.py` are
      confirmed to remain clean (zero unit-suffixed identifiers) — no edit
      expected.
- [ ] `nav/camera_goto.py`'s duck-typed `robot` argument handling is
      unaffected — this file has zero `robot_radio` imports and its rename
      is fully self-contained.
- [ ] Navigation-related unit/system tests pass with unchanged numeric
      assertions (per `usecases.md` SUC-005).
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: navigation-related unit tests in
  `tests/simulation/unit/` (grep for `Navigator`/`camera_goto`/
  `_approach_utils` imports to enumerate the exact files).
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file; confirm `camera_goto.py`'s
zero-import independence means it can be renamed in any order relative to
the rest of this ticket's files.

1. `nav/navigator.py` — rename `tolerance_mm`/`speed_mms` and any other
   unit-suffixed identifier.
2. `nav/_approach_utils.py` — rename `r_mm` → `radius`.
3. `nav/camera_goto.py` — rename `target_mm` → `target`; verify no
   `robot_radio` import is introduced by the rename itself.
4. `nav/nav_params.py` — rename remaining unit-suffixed identifiers.
5. Confirm `nav/pose.py`, `nav/pose_align.py`, `controllers/pid.py` need no
   edit.
6. Grep this file set for every renamed identifier's old name and for any
   protocol-layer keyword call site to confirm convergence on ticket
   002/003's decided names.
7. Run navigation-related unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/nav/navigator.py`
- `host/robot_radio/nav/_approach_utils.py`
- `host/robot_radio/nav/camera_goto.py`
- `host/robot_radio/nav/nav_params.py`
- `host/robot_radio/nav/pose.py`, `nav/pose_align.py`,
  `host/robot_radio/controllers/pid.py` — reviewed only, no edit expected.

**Testing plan**: Run navigation-related unit tests individually, then
`uv run python -m pytest -q` and confirm the 2682 baseline holds.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 007-testgui-core-and-transport-rename-unit-suffixed-identifiers-in-app-entry-point-transport-bridge-and-command-dispatch.md -->
<!-- ================================================================ -->

---
id: '007'
title: 'TestGUI core and transport: rename unit-suffixed identifiers in app entry
  point, transport bridge, and command dispatch'
status: open
use-cases:
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI core and transport: rename unit-suffixed identifiers in app entry point, transport bridge, and command dispatch

## Description

This is planned ticket **007a** in `architecture-update.md`'s Step 3 table
(filed here as sprint ticket 007). It renames unit-suffixed identifiers in
`testgui/__main__.py`, `testgui/transport.py`, `testgui/commands.py`, and
`testgui/drive.py` — the TestGUI's app entry point, connection/transport
bridge, and command dispatch. This subsystem depends on `io/serial_conn.py`
(ticket 001) and `robot/protocol.py` (ticket 002, already renamed) only —
not on the `robot/` object model (ticket 003), `sensors/` (004),
`calibration/` (005), or `nav/` (006) layers.

Renames (per Step 5): `testgui/transport.py` (`read_ms` → `read_timeout`,
`encoder_noise_mm` → `encoder_noise  # [mm]`); `testgui/__main__.py`,
`testgui/commands.py`, `testgui/drive.py` — remaining unit-suffixed
identifiers. Matching `tests/testgui/*.py` files are updated **in this same
ticket**.

**`tests/testgui/*.py` file assignment (Open Question 3, resolved here)**:
per Step 3's own grouping, this ticket owns the **transport / connection /
command / mode / relay / telemetry / smoke** test files (e.g. files
exercising `testgui/transport.py`'s connect/disconnect flow, mode
indicator, relay discovery, and command dispatch). Ticket 008 owns the
**sim-errors / traces / canvas / camera / tour / recorder** test files. If
a specific file's ownership is ambiguous at implementation time, grep the
file's own imports (`from robot_radio.testgui.transport import ...` vs.
`from robot_radio.testgui.sim_prefs import ...`, etc.) to resolve it —
`architecture-update.md` explicitly left the exact file-by-file split to
the ticketing/implementation pass.

**Test-tier note**: `tests/testgui/` (24 files, 579 tests) is **not**
collected by the default `uv run python -m pytest` invocation
(`pyproject.toml`'s `testpaths = ["tests/simulation"]` excludes it
entirely — not merely `norecursedirs`-skipped). This ticket's acceptance
criteria must include an explicit, separate testgui-tier test run.

Total scope: 96 occurrences in `host/robot_radio/testgui/` files touched by
this ticket, plus this ticket's share of `tests/testgui/`'s 169
occurrences.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** The TestGUI must look and behave
  identically; only Python identifiers change.
- **Every renamed declaration carries a `# [unit]` comment.**
- **No wire-key surface in this ticket's files** — `testgui/transport.py`
  is a thin bridge over `io/serial_conn.py`/`robot/protocol.py`, both
  already renamed; nothing here builds a raw wire string.
- **Full suite green throughout**:
  - `uv run python -m pytest -q` remains **2682 passed, 0 failed**
    (`tests/testgui/conftest.py` inserts `host/` onto `sys.path`, so this
    baseline must still be checked even though this ticket's primary files
    live outside `testpaths`).
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    remains **579 passed, 2 xfailed**.
- **Cross-cutting kwargs**: `read_ms` → `read_timeout` convergence is
  required in every call site inside this ticket's files (`transport.py`
  is itself one of the 34 files in the 216-site census).
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `testgui/transport.py`: `read_ms` → `read_timeout` with `# [ms]`;
      `encoder_noise_mm` → `encoder_noise` with `# [mm]`.
- [ ] `testgui/__main__.py`, `testgui/commands.py`, `testgui/drive.py`:
      remaining unit-suffixed identifiers renamed with `# [unit]` comments.
- [ ] Every widget's displayed value, slider range, and issued command
      computed from a renamed field/local produces unchanged numeric
      behavior.
- [ ] Matching `tests/testgui/*.py` files (transport/connection/command/
      mode/relay/telemetry/smoke tier, per the file-assignment guidance
      above) are updated in this same ticket — every `read_ms=` or other
      renamed-parameter call site in these test files converges on the
      ticket 001/002-decided names.
- [ ] `uv run python -m pytest -q` remains 2682 passed, 0 failed.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
      remains 579 passed, 2 xfailed.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: the transport/connection/command/mode/relay/
  telemetry/smoke subset of `tests/testgui/*.py` (grep each file's imports
  to confirm it belongs here vs. ticket 008), run individually first, then
  the full `tests/testgui/` tier, then the default suite.
- **New tests to write**: none required — pure rename.
- **Verification commands**:
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    (confirm 579 passed, 2 xfailed).
  - `uv run python -m pytest -q` (confirm 2682 passed, 0 failed).

## Implementation Plan

**Approach**: Rename `testgui/transport.py` first (it re-exposes
`read_ms`, the cross-cutting name), then `__main__.py`/`commands.py`/
`drive.py`, then their matching test files.

1. `testgui/transport.py` — rename `read_ms` → `read_timeout`,
   `encoder_noise_mm` → `encoder_noise`; add `# [unit]` comments.
2. `testgui/__main__.py`, `testgui/commands.py`, `testgui/drive.py` —
   rename remaining unit-suffixed identifiers.
3. Identify the transport/connection/command/mode/relay/telemetry/smoke
   subset of `tests/testgui/*.py` by grepping each file's imports against
   this ticket's four host files; update every renamed identifier and
   keyword-argument call site in those test files.
4. Grep this ticket's full file set for every renamed identifier's old
   name to confirm no call site was missed.
5. Run the identified `tests/testgui/*.py` subset individually, then the
   full `tests/testgui/` tier with `QT_QPA_PLATFORM=offscreen`, then the
   default suite.

**Files to create/modify**:
- `host/robot_radio/testgui/__main__.py`
- `host/robot_radio/testgui/transport.py`
- `host/robot_radio/testgui/commands.py`
- `host/robot_radio/testgui/drive.py`
- The transport/connection/command/mode/relay/telemetry/smoke subset of
  `tests/testgui/*.py` (exact file list determined by import grep at
  implementation time).

**Testing plan**: Run the identified `tests/testgui/*.py` subset
individually, then `QT_QPA_PLATFORM=offscreen uv run python -m pytest
tests/testgui/ -q` (confirm 579 passed, 2 xfailed), then
`uv run python -m pytest -q` (confirm 2682 passed, 0 failed). Manually
launch the TestGUI to visually confirm connect/disconnect, mode indicator,
and command dispatch are unaffected.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 008-testgui-panels-and-recording-rename-unit-suffixed-identifiers-in-sim-error-traces-canvas-and-recording-glue.md -->
<!-- ================================================================ -->

---
id: '008'
title: 'TestGUI panels and recording: rename unit-suffixed identifiers in sim-error,
  traces, canvas, and recording glue'
status: open
use-cases:
- SUC-006
depends-on:
- '007'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI panels and recording: rename unit-suffixed identifiers in sim-error, traces, canvas, and recording glue

## Description

This is planned ticket **007b** in `architecture-update.md`'s Step 3 table
(filed here as sprint ticket 008). It renames unit-suffixed identifiers in
`testgui/sim_prefs.py`, `traces.py`, `operations.py`, `canvas.py`,
`camera_prefs.py`, `live_view.py`, and `recorder.py` — the sim-error/
traces/canvas panels and recording/camera-preview glue. `transport.py`
(ticket 007) is imported by nothing in this ticket's files, but both live
in the same PySide6 application and share incidental state via
`testgui/__main__.py`; sequencing 008 immediately after 007 reduces review
context-switching (Step 5 "Why").

**`sim_prefs.py`'s two-layer key pattern (handle carefully)**:
`DEFAULT_PROFILE["trackwidth_mm"]` is a host-side profile dict keyed by a
unit-suffixed string, mapped through an explicit table
(`{"trackwidth_mm": "trackwidthMm", ...}`) to the real `SIMSET` wire key.
Per the Wire-Compatibility Exclusion Table:
- **RENAME** the host-internal key (`"trackwidth_mm"` → e.g.
  `"trackwidth"`) — this is a convenience string local to `sim_prefs.py`
  and its own tests, not itself wire-visible.
- **EXCLUDE** the mapping table's *value* (`"trackwidthMm"`) — this is the
  real `SIMSET` wire key and must stay byte-identical.
- The file's own docstrings/comments reference both spellings by name —
  read them before editing to avoid conflating the two.

Renames (per Step 5): `testgui/sim_prefs.py` (`trackwidth_mm` host-internal
key → `trackwidth`, mapping-table value `"trackwidthMm"` untouched);
`testgui/traces.py`; `testgui/operations.py` (`rotation_deg` → `rotation
# [deg]`); `testgui/canvas.py`. Matching `tests/testgui/*.py` files are
updated **in this same ticket**.

**`tests/testgui/*.py` file assignment (Open Question 3, resolved here)**:
this ticket owns the **sim-errors / traces / canvas / camera / tour /
recorder** test files (files exercising `sim_prefs.py`, `traces.py`,
`canvas.py`, `camera_prefs.py`, `live_view.py`, `recorder.py`) — the
complement of ticket 007's transport/connection/command/mode/relay/
telemetry/smoke set. Resolve any ambiguous file by grepping its imports.

Total scope: 49 occurrences in `host/robot_radio/testgui/` files touched by
this ticket, plus this ticket's share of `tests/testgui/`'s 169
occurrences.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** The TestGUI must look and behave
  identically; only Python identifiers change.
- **`sim_prefs.py`'s SIMSET mapping-table VALUES are STABLE** — see the
  two-layer pattern above; only the host-internal key is renamable.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Full suite green throughout**:
  - `uv run python -m pytest -q` remains **2682 passed, 0 failed**.
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    remains **579 passed, 2 xfailed**.
- **Cross-cutting kwargs**: any call into `testgui/transport.py` (ticket
  007, already renamed) using a renamed keyword argument must already use
  the converged name; fix any stale one found here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `testgui/sim_prefs.py`: `DEFAULT_PROFILE`'s `"trackwidth_mm"`
      host-internal key is renamed (e.g. to `"trackwidth"`); the mapping
      table's `"trackwidthMm"` SIMSET-key *value* is byte-identical to
      pre-076 (diff-confirm).
- [ ] `testgui/operations.py`: `rotation_deg` → `rotation` with `# [deg]`.
- [ ] `testgui/traces.py`, `testgui/canvas.py`, `testgui/camera_prefs.py`,
      `testgui/live_view.py`, `testgui/recorder.py`: remaining
      unit-suffixed identifiers renamed with `# [unit]` comments.
- [ ] Traces/recordings capture the same data under renamed field names —
      no change to recorded values, only the field name.
- [ ] Matching `tests/testgui/*.py` files (sim-errors/traces/canvas/camera/
      tour/recorder tier) are updated in this same ticket.
- [ ] `sim_prefs.py`'s SIMSET wire-key mapping-table values are
      byte-identical to pre-076 (explicit diff check, per `usecases.md`
      SUC-006's acceptance criteria).
- [ ] `uv run python -m pytest -q` remains 2682 passed, 0 failed.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
      remains 579 passed, 2 xfailed.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: the sim-errors/traces/canvas/camera/tour/
  recorder subset of `tests/testgui/*.py` (grep each file's imports to
  confirm it belongs here vs. ticket 007), run individually first, then
  the full `tests/testgui/` tier, then the default suite.
- **New tests to write**: none required — pure rename.
- **Verification commands**:
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    (confirm 579 passed, 2 xfailed).
  - `uv run python -m pytest -q` (confirm 2682 passed, 0 failed).

## Implementation Plan

**Approach**: Handle `sim_prefs.py`'s two-layer key pattern first and most
carefully, since it is the one file in this ticket with a real wire-key
adjacency; then rename the remaining panel files.

1. Read `testgui/sim_prefs.py` in full, including its docstrings, before
   editing — confirm exactly which strings are host-internal keys
   (renamable) vs. mapping-table values (wire keys, excluded).
2. Rename `DEFAULT_PROFILE`'s `"trackwidth_mm"` key and any Python
   identifier referencing it; leave the mapping table's `"trackwidthMm"`
   value untouched.
3. `testgui/operations.py` — rename `rotation_deg` → `rotation`.
4. `testgui/traces.py`, `testgui/canvas.py`, `testgui/camera_prefs.py`,
   `testgui/live_view.py`, `testgui/recorder.py` — rename remaining
   unit-suffixed identifiers.
5. Identify the sim-errors/traces/canvas/camera/tour/recorder subset of
   `tests/testgui/*.py` by grepping each file's imports against this
   ticket's host files; update every renamed identifier and keyword
   call site.
6. Grep this ticket's full file set for every renamed identifier's old
   name, and specifically re-verify `"trackwidthMm"` (the wire-key value)
   is unchanged.
7. Run the identified `tests/testgui/*.py` subset individually, then the
   full `tests/testgui/` tier with `QT_QPA_PLATFORM=offscreen`, then the
   default suite.

**Files to create/modify**:
- `host/robot_radio/testgui/sim_prefs.py`
- `host/robot_radio/testgui/traces.py`
- `host/robot_radio/testgui/operations.py`
- `host/robot_radio/testgui/canvas.py`
- `host/robot_radio/testgui/camera_prefs.py`
- `host/robot_radio/testgui/live_view.py`
- `host/robot_radio/testgui/recorder.py`
- The sim-errors/traces/canvas/camera/tour/recorder subset of
  `tests/testgui/*.py` (exact file list determined by import grep at
  implementation time).

**Testing plan**: Run the identified `tests/testgui/*.py` subset
individually, then `QT_QPA_PLATFORM=offscreen uv run python -m pytest
tests/testgui/ -q` (confirm 579 passed, 2 xfailed), then
`uv run python -m pytest -q` (confirm 2682 passed, 0 failed). Manually
launch the TestGUI and confirm the Sim Errors panel's sliders still push
the correct `SIMSET trackwidthMm=...` wire command.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 009-rogo-cli-rename-unit-suffixed-identifiers-in-the-console-script-entry-point.md -->
<!-- ================================================================ -->

---
id: '009'
title: 'rogo CLI: rename unit-suffixed identifiers in the console-script entry point'
status: open
use-cases:
- SUC-007
depends-on:
- '003'
- '004'
- '005'
- '006'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# rogo CLI: rename unit-suffixed identifiers in the console-script entry point

## Description

`host/robot_radio/io/cli.py` implements the `rogo` console-script — the
single largest file in this sprint's scope (170 rename-eligible
occurrences). It sits at the top of the host dependency graph, importing
from `robot/` (including `Cutebot`, ticket 003), `sensors/color.py`
(ticket 004), `config/robot_config.py` (excluded, ticket 011 confirms),
`calibration/helpers.py` (ticket 005), and `nav/camera_goto.py` (ticket
006) — it must be renamed **after** every subpackage it imports from, or
call sites would be renamed into not-yet-renamed functions and break
immediately (`architecture-update.md` Step 5 "Why").

Renames (per Step 5): `read_ms` → `read_timeout`; `left_mm_per_deg`/
`right_mm_per_deg` → matching ticket 005's calibration naming choice
exactly (do not diverge); `watchdog_ms`/`resend_ms`/`t_ms` → bare names
with `# [ms]`; `x_mm`/`y_mm`/`h_deg`/`angle_deg` → bare names with
`# [unit]`.

**Wire-key pairing to leave untouched**: `io/cli.py` builds pairs like
`("minWheelMms", getattr(ctrl, "min_wheel_mms", None))` — both halves of
this pairing are already-excluded surfaces (the string is a `SET`/`SIMSET`
wire key; `min_wheel_mms` is a pydantic attribute name on `RobotConfig`,
excluded wholesale per ticket 011's Exclusion Table confirmation). Neither
half is renamed by this ticket.

Total scope: 170 occurrences, 1 file.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Every `rogo` subcommand behaves
  identically to pre-076; output formatting and exit codes unchanged.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire keys and pydantic attribute names are STABLE**: any
  `("wireKeyString", getattr(ctrl, "pydantic_attr_name", ...))`-style
  pairing in this file keeps both halves exactly as-is.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: `read_ms` → `read_timeout` and every other
  cross-cutting rename decided in tickets 001–006 must be used
  consistently in this file — do not invent an alternative spelling.
- **Manual verification required**: no automated CLI-invocation test
  exists for `rogo` today (`tests/simulation/unit/test_cli.py` covers
  internal logic, not the console-script invocation itself) — run `rogo
  help` and a representative smoke command manually pre/post-rename and
  confirm identical output.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `read_ms` → `read_timeout` with `# [ms]`, matching ticket 001/002's
      decided name.
- [ ] `left_mm_per_deg`/`right_mm_per_deg` renamed to **exactly** the name
      ticket 005 chose in `calibration/push.py` (grep ticket 005's landed
      code first, do not choose independently).
- [ ] `watchdog_ms`/`resend_ms`/`t_ms` → bare names with `# [ms]`.
- [ ] `x_mm`/`y_mm`/`h_deg`/`angle_deg` → bare names with `# [unit]`
      (`mm`, `deg` respectively).
- [ ] Every wire-key/pydantic-attribute pairing this file builds (e.g. the
      `minWheelMms`/`min_wheel_mms` pair) is untouched — both halves
      byte-identical to pre-076.
- [ ] `rogo help` and at least one representative subcommand round-trip
      identically pre/post-sprint (manual verification, documented in the
      PR/commit description since no automated CLI-invocation test
      exists).
- [ ] `tests/simulation/unit/test_cli.py` passes unchanged (per
      `usecases.md` SUC-007).
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_cli.py`.
- **New tests to write**: none required — pure rename. (No automated
  CLI-invocation test exists; manual verification substitutes, per the
  Hard Contract above.)
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed), plus a manual `rogo help` and one representative
  subcommand invocation.

## Implementation Plan

**Approach**: Read `io/cli.py` in full first to enumerate every
unit-suffixed identifier and every wire-key/pydantic-attribute pairing,
since this is the largest single file in the sprint and the most
consumer-facing.

1. Confirm ticket 005's exact chosen name for
   `left_mm_per_deg`/`right_mm_per_deg` before starting (grep
   `calibration/push.py`'s landed code) — use the identical name here.
2. Rename `read_ms` → `read_timeout`, `watchdog_ms`/`resend_ms`/`t_ms`,
   and `x_mm`/`y_mm`/`h_deg`/`angle_deg` throughout the file, adding
   `# [unit]` comments.
3. Locate every wire-key/pydantic-attribute pairing (grep for
   `getattr(ctrl,` and any `SET`/`SIMSET`/`GET` string literal builder);
   confirm neither half is touched.
4. Grep the whole file for every renamed identifier's old name to confirm
   no call site was missed.
5. Run `tests/simulation/unit/test_cli.py`, then the full suite.
6. Manually run `rogo help` and one representative subcommand (e.g. a
   dry-run or status command that doesn't require live hardware) and
   confirm output is unchanged from a pre-rename run.

**Files to create/modify**:
- `host/robot_radio/io/cli.py` — the only file this ticket touches.

**Testing plan**: Run `tests/simulation/unit/test_cli.py`, then
`uv run python -m pytest -q` and confirm the 2682 baseline holds. Manually
invoke `rogo help` and a representative subcommand.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 010-calibration-cli-and-mcp-surface-rename-unit-suffixed-identifiers-in-the-calibration-wizard-and-agent-facing-tools.md -->
<!-- ================================================================ -->

---
id: '010'
title: 'Calibration CLI and MCP surface: rename unit-suffixed identifiers in the calibration
  wizard and agent-facing tools'
status: open
use-cases:
- SUC-007
depends-on:
- '003'
- '005'
- '007'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Calibration CLI and MCP surface: rename unit-suffixed identifiers in the calibration wizard and agent-facing tools

## Description

`host/robot_radio/io/calibrate.py` (interactive calibration wizard),
`io/robot_mcp.py` (the agent-facing MCP tool surface), and `media/movie.py`
(companion media helper) sit at the same top-of-dependency-graph level as
`io/cli.py` (ticket 009), consuming the `robot/` object model (ticket 003),
`calibration/` (ticket 005), and `testgui/` (ticket 007, via shared
transport/connection concepts) — hence this ticket depends on those three,
not on ticket 009 itself (the two top-level surfaces are siblings, not
sequential).

Renames (per Step 5): `io/calibrate.py`, `io/robot_mcp.py` — unit-suffixed
parameters/locals; `media/movie.py` (`min_interval_ms` → `min_interval
# [ms]`).

**Security/API-surface note (from the Architecture Self-Review's Risks
section)**: `io/robot_mcp.py`'s `@tool`-decorated function **names**
(as distinct from their parameter names) are the MCP tool surface exposed
to agent sessions. This planning pass's own reading found none of those
tool names are unit-suffixed, so no tool name should need to change — but
this ticket's implementer must explicitly re-verify that before renaming
anything in this file, since renaming an exposed tool name (unlike a
parameter name) would be a breaking change for any agent session with a
cached tool schema. Only internal parameter/local names are in scope.

Total scope: 165 occurrences (126 in `io/calibrate.py`, 31 in
`io/robot_mcp.py`, 8 in `media/movie.py`).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Every `rogo calibrate` wizard
  step and every MCP tool call behaves identically to pre-076.
- **Every renamed declaration carries a `# [unit]` comment.**
- **`io/robot_mcp.py`'s `@tool`-decorated function names are STABLE** —
  re-verify none are unit-suffixed before editing; if one is found
  unit-suffixed, do not rename it in this ticket without first raising it
  to the team-lead (a tool-name change is a breaking API change beyond
  this sprint's pure-rename scope, distinct from a parameter rename).
- **Wire keys and pydantic attributes are STABLE** per the sprint-wide
  Exclusion Table — any `SET`/`SIMSET` key or `RobotConfig` attribute this
  file reads or builds stays untouched.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: `read_ms` → `read_timeout` and every other
  sprint-decided rename must be used consistently.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `io/calibrate.py`: all unit-suffixed identifiers renamed with
      `# [unit]` comments, converging on names already decided by
      tickets 001–007 for any shared cross-cutting parameter.
- [ ] `io/robot_mcp.py`: all unit-suffixed **parameter/local** identifiers
      renamed with `# [unit]` comments; every `@tool`-decorated function's
      own **name** is confirmed unchanged (explicit check, documented in
      the ticket's completion notes).
- [ ] `media/movie.py`: `min_interval_ms` → `min_interval` with `# [ms]`.
- [ ] No `SET`/`SIMSET` wire-key string or `RobotConfig` pydantic attribute
      name is touched anywhere in this ticket's three files.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: any `tests/simulation/unit/` test exercising
  `io/calibrate.py`'s wizard logic or `io/robot_mcp.py`'s tool functions
  (grep for `calibrate`/`robot_mcp` imports to enumerate); `media/movie.py`
  test coverage if any exists.
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Handle `io/robot_mcp.py` most carefully, since it is the
only file in this sprint with an externally-cached-schema risk distinct
from every other pure-Python rename.

1. Read `io/robot_mcp.py` in full; list every `@tool`-decorated function
   name and confirm none is unit-suffixed (expected result per this
   pass's own planning-time reading — re-verify, don't assume).
2. Rename `io/robot_mcp.py`'s internal parameter/local names only, leaving
   every tool function's own name untouched.
3. Rename `io/calibrate.py`'s unit-suffixed identifiers, converging on
   any cross-cutting name already decided upstream.
4. Rename `media/movie.py`'s `min_interval_ms` → `min_interval`.
5. Grep this ticket's three files for every renamed identifier's old name
   to confirm no call site was missed, and specifically confirm no
   `@tool`-decorated function name changed.
6. Run relevant unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/io/calibrate.py`
- `host/robot_radio/io/robot_mcp.py`
- `host/robot_radio/media/movie.py`

**Testing plan**: Run any calibration-wizard/MCP-specific unit tests
individually, then `uv run python -m pytest -q` and confirm the 2682
baseline holds.

**Documentation updates**: None in this ticket.


<!-- ================================================================ -->
<!-- SPRINT 076 TICKET (verbatim): 011-final-sweep-certify-zero-residual-unit-suffixed-identifiers-update-out-of-package-callers-and-close-the-docs-status-line.md -->
<!-- ================================================================ -->

---
id: '011'
title: 'Final sweep: certify zero residual unit-suffixed identifiers, update out-of-package
  callers, and close the docs status line'
status: open
use-cases:
- SUC-008
- SUC-009
depends-on:
- '006'
- '009'
- '010'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final sweep: certify zero residual unit-suffixed identifiers, update out-of-package callers, and close the docs status line

## Description

This is the sprint's closing ticket. It exists because the parent issue's
acceptance criteria are phrased as a whole-codebase invariant that only a
final, whole-tree grep can certify, and because five locations
(`tests/bench/`, `tests/field/`, `tests/_infra/calibrate/`,
`tests/_infra/tools/`, top-level `host/*.py`) have **zero automated test
coverage** and would otherwise silently accumulate stale keyword-argument
call sites across the whole sprint (`architecture-update.md` Step 5 "Why").

Scope:
- `host/calibrate_linear.py`, `host/calibrate_angular.py`,
  `host/calibrate_verify.py` (5 occurrences) — top-level scripts.
- `testkit/*` (2 occurrences).
- `tests/bench/*.py` (214 occurrences, 22 files) — **no automated
  protection**; every keyword-argument call site of every function/method
  renamed by tickets 001–010, and this tree's own local unit-suffixed
  identifiers (per the issue's "host-side tools/scripts" framing), updated
  here.
- `tests/field/*.py` (22 occurrences, 4 files) — same treatment.
- `tests/_infra/calibrate/*.py` (115 occurrences, 10 files) — same
  treatment.
- `tests/_infra/tools/*.py` (10 occurrences, 2 files) — same treatment.
- `config/robot_config.py` — **confirm-only, zero renames expected**: this
  pass's own full read found 13 pydantic `BaseModel` classes with zero
  `Field(alias=...)` anywhere, meaning every JSON key **is** the bare
  Python attribute name (Decision 4). This ticket's job is to re-confirm
  that exclusion still holds on the checked-out code, not to edit the
  file.
- `.claude/rules/coding-standards.md` — update the Python-convention section's
  status line from "not yet applied to any `host/` file ... sprint 072 is
  the sprint that will apply it" to state that **sprint 076** applied it
  (the work slipped five sprints per the roadmap; the convention text
  itself is unchanged).
- A final repo-wide certification grep (see Acceptance Criteria).

**Explicitly out of scope** (do not touch, per `architecture-update.md`'s
Decisions 1 and 3): `tests/old/` (36 files, 145 occurrences — deprecated,
`norecursedirs`-excluded, zero coverage); `tests/simulation/`'s and
`tests/_infra/sim/`'s own internal mock-class identifiers that mirror
*firmware* C++ naming (e.g. `t0Ms`, `encLMm`, `sTimeoutMs` in
`test_motion_command.py`'s `MotionBaseline`/`HardwareState` classes) — only
their `robot_radio` call sites are updated, as a mechanical consequence of
tickets 001–010's renames, and only if not already caught by an earlier
ticket incidentally touching the same file.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.**
- **Every renamed declaration carries a `# [unit]` comment** (for the
  top-level scripts and `testkit/*`; the bench/field/infra trees are
  primarily caller-side keyword-argument updates, but any of their own
  locally-declared unit-suffixed identifiers get the same treatment).
- **Wire keys/tokens and pydantic attributes are STABLE.** `git diff` must
  show **zero changes** to `data/robots/*.json`, `robot_config.schema.json`,
  and `config/robot_config.py`.
- **Full suite green throughout**:
  - `uv run python -m pytest -q` = **2682 passed, 0 failed**.
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    = **579 passed, 2 xfailed**.
- **No automated protection for bench/field/infra trees** — a stakeholder
  with hardware access should spot-run at least one bench script (e.g.
  `tests/bench/smoke_ritual.py`) post-sprint as a manual gate; flag this to
  the team-lead when reporting this ticket complete.
- **Ignore environmental `data/robots` drift** — if `git status` shows a
  pre-existing, unrelated modification to `data/robots/*.json` from before
  this sprint started, do not touch it and do not attribute it to this
  ticket; this ticket's own acceptance criterion is that *this ticket's
  own diff* introduces zero changes there.

## Acceptance Criteria

- [ ] `host/calibrate_linear.py`, `host/calibrate_angular.py`,
      `host/calibrate_verify.py` (5 occurrences) renamed, converging on
      every prior ticket's decided names for shared identifiers.
- [ ] `testkit/*` (2 occurrences) renamed.
- [ ] `tests/bench/*.py` (214 occurrences, 22 files): every keyword-argument
      call site of every renamed function/method updated; every local
      unit-suffixed identifier renamed with a `# [unit]` comment.
- [ ] `tests/field/*.py` (22 occurrences, 4 files): same treatment.
- [ ] `tests/_infra/calibrate/*.py` (115 occurrences, 10 files): same
      treatment.
- [ ] `tests/_infra/tools/*.py` (10 occurrences, 2 files): same treatment.
- [ ] `config/robot_config.py`: `git diff` shows **zero changes** — this
      ticket only confirms the wholesale pydantic exclusion still holds
      (13 classes, no `Field(alias=...)`), it does not edit this file.
- [ ] `data/robots/*.json` and `robot_config.schema.json`: `git diff` shows
      zero changes attributable to this ticket.
- [ ] `.claude/rules/coding-standards.md`'s Python-convention section states the
      convention has been applied by **sprint 076** (not the originally
      forward-referenced "072"); no other text in that section changes.
- [ ] Final repo-wide grep:
      `grep -rniE "\b[a-z_][a-z0-9_]*_(mm|mms|deg|dps|ms|us|pct|hz)\b" host/ tests/`
      — excluding `tests/old/`, `tests/simulation/`, `tests/_infra/sim/`,
      and this sprint's own planning documents' historical prose — returns
      **zero** results outside documented exclusions (wire-key strings,
      `config/robot_config.py`'s pydantic fields, `sim_prefs.py`'s SIMSET
      mapping-table values).
- [ ] `uv run python -m pytest -q` = 2682 passed, 0 failed.
- [ ] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
      = 579 passed, 2 xfailed.
- [ ] The issue `remove-units-from-identifier-names-host-python.md`'s own
      acceptance criteria (no unit-suffixed identifier outside documented
      exclusions; every renamed declaration carries the `# [unit]`
      comment; pure rename; wire compatibility preserved) are satisfied
      repo-wide, not just within this ticket's own file set.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: full default suite plus the testgui tier (see
  Verification commands). `tests/bench/`, `tests/field/`,
  `tests/_infra/calibrate/`, `tests/_infra/tools/` have no automated
  pytest collection (`--collect-only` returns "no tests collected" for
  `tests/bench` and `tests/field`, confirmed in `architecture-update.md`
  Step 1) — verification there is the repo-wide grep plus, ideally, a
  stakeholder manual bench-script run.
- **New tests to write**: none required — pure rename.
- **Verification commands**:
  - `uv run python -m pytest -q` (confirm 2682 passed, 0 failed).
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
    (confirm 579 passed, 2 xfailed).
  - `grep -rniE "\b[a-z_][a-z0-9_]*_(mm|mms|deg|dps|ms|us|pct|hz)\b" host/ tests/`
    (excluding `tests/old/`, `tests/simulation/`, `tests/_infra/sim/`) —
    zero results outside documented exclusions.
  - `git diff --stat -- data/robots/ host/robot_radio/config/robot_config.py`
    — empty output.

## Implementation Plan

**Approach**: Sweep the no-coverage trees first (bench/field/infra/
top-level scripts/testkit), since they carry the highest silent-breakage
risk, then run the certification grep, then close the docs status line,
then do the final full-suite runs.

1. `host/calibrate_linear.py`, `calibrate_angular.py`, `calibrate_verify.py`
   — rename the 5 occurrences, converging on prior tickets' names.
2. `testkit/*` — rename the 2 occurrences.
3. `tests/bench/*.py` (22 files) — for each file, update every renamed
   function/method's keyword-argument call sites and any local
   unit-suffixed identifier; add `# [unit]` comments to local
   declarations.
4. `tests/field/*.py` (4 files) — same treatment.
5. `tests/_infra/calibrate/*.py` (10 files) — same treatment.
6. `tests/_infra/tools/*.py` (2 files) — same treatment.
7. Read `config/robot_config.py` in full one more time; confirm the
   13-class pydantic exclusion still holds and `git diff` shows zero
   changes to this file.
8. Run the certification grep across `host/` and `tests/` (excluding
   `tests/old/`, `tests/simulation/`, `tests/_infra/sim/`); resolve any
   unexpected hit by tracing it back to the ticket that should have caught
   it, or by renaming it here if it's genuinely a missed occurrence in
   this ticket's own scope.
9. Update `.claude/rules/coding-standards.md`'s status line to reference sprint 076.
10. Run the full default suite, then the testgui tier, confirming both
    baselines.
11. Note in the ticket's completion notes that a stakeholder should
    spot-run at least one bench script (e.g. `tests/bench/smoke_ritual.py`)
    post-sprint as a manual gate beyond this ticket's grep certification.

**Files to create/modify**:
- `host/calibrate_linear.py`, `host/calibrate_angular.py`,
  `host/calibrate_verify.py`
- `host/robot_radio/testkit/*` (2 occurrences)
- `tests/bench/*.py` (22 files)
- `tests/field/*.py` (4 files)
- `tests/_infra/calibrate/*.py` (10 files)
- `tests/_infra/tools/*.py` (2 files)
- `host/robot_radio/config/robot_config.py` — confirm-only, no edit
  expected.
- `.claude/rules/coding-standards.md` — one status-line edit.

**Testing plan**: Run `uv run python -m pytest -q` (2682 passed, 0
failed), `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
(579 passed, 2 xfailed), and the repo-wide certification grep. Recommend a
stakeholder-run bench script (e.g. `tests/bench/smoke_ritual.py`) as a
post-sprint manual gate, since bench/field/infra trees have no automated
coverage.

**Documentation updates**: `.claude/rules/coding-standards.md`'s Python-convention
status line, updated to state sprint 076 (not 072) applied the convention
to `host/`.
