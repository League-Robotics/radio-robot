---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 076 Use Cases

This sprint is a **pure identifier rename** in `host/robot_radio/` (the
importable host package), its top-level sibling scripts (`host/*.py`), and
the host-side tools/tests that reference it: unit suffixes (`_mm`, `_mms`,
`_deg`, `_dps`, `_ms`, `_us`, `_pct`, `_hz`) are stripped from module-level
names, function/method parameters, locals, dataclass fields, and dict keys;
the unit moves to a standard leading `# [unit]` comment tag, per
`docs/coding-standards.md`'s Python convention (documented as a forward
reference by sprint 071, applied for the first time by this sprint). **No
host behavior changes, no wire-format change, no config-file-format
change** (see `architecture-update.md`'s Wire-Compatibility Exclusion
Table).

Because nothing observable changes, every SUC below is framed as
**preservation**: the existing top-level UC's behavior, wire traffic, and
test coverage must be byte-identical after the rename. SUC-009 is the one
genuinely new capability this sprint adds — extending the already-documented
grep-able unit-comment convention to `host/` — and is parented to a newly
proposed UC (there is no existing UC for source-code documentation
conventions; `docs/usecases.md` UC-020/021/022/023 are already reserved by
sprints 069/071/073 pending consolidation, so this sprint's new UC is
numbered UC-024).

**Scope note (this closes issue 1's host half):** the parent issue
`remove-units-from-identifier-names.md` was split at sprint 071 into a
firmware half (071, done) and a host-Python half (this sprint). This
planning pass's own grep confirms host is materially larger than 071
estimated for firmware: ~1470 rename-eligible occurrences in
`host/robot_radio/` alone (445 `robot/`, 375 `io/`, 175 `calibration/`, 163
`nav/`, 145 `testgui/`, 121 `sensors/`, 28 `config/` — all pydantic-excluded,
see below, 8 `media/`, 8 `kinematics/`, 2 `testkit/`), plus 169 in
`tests/testgui/`, 214 in `tests/bench/`, 115 in `tests/_infra/calibrate/`,
22 in `tests/field/`, 10 in `tests/_infra/tools/`, and 5 in three top-level
`host/*.py` scripts — roughly 2000 occurrences of ~280 unique identifiers,
`read_ms` alone accounting for 216 keyword call sites this pass counted
directly (not the issue's original estimate).

**Scope note (excluded from this sprint, with rationale — see
`architecture-update.md` Decisions 1–2):** `tests/simulation/` and
`tests/_infra/sim/` are **not** rename targets — they are pure-Python
mirrors of *firmware* C++ logic (sprint 071's domain) or CMake build
infrastructure, not the host package this issue names. Where a
`tests/simulation/` file imports `robot_radio` and calls a renamed
function/method with a keyword argument, that call site is updated as a
**mechanical consequence** of keeping the suite green (identical in spirit
to how 071 dragged `tests/simulation/` C++-mirroring fixtures along) — this
is a required but incidental edit, not host-Python cleanup in its own
right, and every occurrence is protected by this sprint's own hard gate:
the default `uv run python -m pytest` baseline (**2682 passed**, confirmed
fresh this planning pass) collects `tests/simulation/` exclusively
(`pyproject.toml`'s `testpaths = ["tests/simulation"]`), so a missed
keyword-arg update there fails loudly and immediately. `tests/old/` (145
occurrences, 36 files) is excluded outright — it is a deprecated/archived
tree (`norecursedirs`-excluded, superseded by the 038 test-tier
reorganization) with no automated coverage and no current operational use.

---

## SUC-001: Preserve Wire-Protocol Send/Receive Behavior Across the Transport + Protocol-Adapter Rename
Parent: UC-001, UC-002, UC-003, UC-004, UC-018, UC-019 (narrows)

- **Actor**: Developer / any host caller (CLI, TestGUI, MCP, bench script)
  driving the robot over the v2 serial/relay wire protocol.
- **Preconditions**: `io/serial_conn.py`'s `SerialConnection`, `io/sim_conn.py`'s
  ctypes sim transport, and `robot/protocol.py`'s `NezhaProtocol` (`send`,
  `arc`, `vw`, `drive`, `timed`, `distance`, `go_to`, `turn`, `stream`,
  `snap`, OTOS/J-port helpers) carry unit-suffixed parameter names
  (`read_ms`, `speed_mms`, `radius_mm`, `v_mms`, `omega_mrads`, `left_mms`,
  `right_mms`, `x_mm`, `y_mm`, `heading_cdeg`, `eps_cdeg`, `period_ms`,
  `watchdog_ms`, `duration_ms`, `h_cdeg`).
- **Main Flow**:
  1. A caller issues a command (e.g. `proto.arc(speed_mms=200, radius_mm=0)`,
     `proto.send(cmd, read_ms=500)`).
  2. `NezhaProtocol` formats the wire line positionally (e.g. `f"R {speed_mms}
     {radius_mm}"`) and hands it to `SerialConnection`; renamed parameter
     names never appear on the wire — only their *values*, in the same
     position, do.
  3. `TLMFrame`/`ParsedResponse` parsing (`parse_response`, `parse_tlm`,
     `parse_cfg`) reads the exact same wire tokens (`t=`, `mode=`, `seq=`,
     `enc=`, `pose=`, `vel=`, `twist=`, `otos=`, `line=`, `color=`,
     `ekf_rej=`, `wedge=`, `encpose=`, `otos_health=`) as `kv` dict keys —
     these string literals are untouched by this sprint.
- **Postconditions**: Every byte sent to and parsed from the firmware is
  identical to pre-076. Only Python-side parameter/local names changed.
- **Acceptance Criteria**:
  - [ ] `TLMFrame`'s dataclass field names are unchanged (already unit-free
        — confirmed by direct read; nothing to rename there).
  - [ ] `parse_tlm`'s `kv` dict key lookups (`"enc"`, `"pose"`, `"otos"`,
        `"encpose"`, `"otos_health"`, `"wedge"`, `"vel"`, `"twist"`,
        `"line"`, `"color"`, `"ekf_rej"`, `"t"`, `"mode"`, `"seq"`) are
        byte-identical string literals (diffed against pre-076).
  - [ ] Every renamed parameter's call site — inside `host/robot_radio/`
        **and** every out-of-package caller (`tests/bench/`, `tests/field/`,
        `tests/_infra/calibrate/`, `tests/_infra/tools/`, `host/*.py`,
        `tests/simulation/`, `tests/testgui/`) — is updated in the same
        ticket that renames the definition; `grep -rn "read_ms="` (and each
        other renamed parameter) returns zero results repo-wide at sprint
        close, except inside this document's own historical prose.
  - [ ] `uv run python -m pytest -q` remains **2682 passed, 0 failed**
        throughout.

---

## SUC-002: Preserve Robot Object-Model Behavior (State, Kinematics, Clock Sync) Across the Rename
Parent: UC-005, UC-006, UC-007 (narrows)

- **Actor**: Developer using the `Robot`/`Nezha`/`Cutebot` object model.
- **Preconditions**: `robot/robot.py`, `robot/robot_state.py`, `robot/nezha.py`,
  `robot/nezha_state.py`, `robot/nezha_kinematic.py`, `robot/cutebot.py`,
  `robot/clock_sync.py`, `robot/sync_pose.py`, and `kinematics/
  differential_drive.py` carry unit-suffixed fields/locals (`x_mm`, `y_mm`,
  `period_ms`, `t_robot_ms`, `t0_ms`/`t1_ms`, `left_mms`/`right_mms`,
  `r_deg`/`l_deg`).
- **Main Flow**:
  1. A caller queries or drives a `Nezha`/`Cutebot` instance (encoder
     positions, dead-reckoned pose, clock offset).
  2. The object model computes the same values via renamed
     fields/locals/methods.
  3. `ClockSync` and `sync_pose` reconcile host/robot clocks and camera-fix
     poses exactly as before.
- **Postconditions**: Encoder queries, odometry, and clock-sync results are
  numerically identical to pre-076.
- **Acceptance Criteria**:
  - [ ] `robot/` object-model files carry no unit-suffixed identifier
        (excluding wire-key string literals, none of which appear in this
        layer) and each carries a `# [unit]` comment.
  - [ ] `tests/simulation/unit/test_odom_tracker.py`,
        `test_serial_conn_reader.py`, and the clock-sync/kinematic unit
        tiers pass unchanged in behavior.

---

## SUC-003: Preserve Sensor Reading and Classification Behavior Across the Rename
Parent: UC-008, UC-009, UC-011, UC-012 (narrows)

- **Actor**: Developer reading line/color/OTOS sensors or dead-reckoning
  odometry from the host side.
- **Preconditions**: `sensors/odom_tracker.py`, `sensors/otos.py`,
  `sensors/color.py`, `sensors/calibration.py`, `sensors/motion_monitor.py`
  carry unit-suffixed identifiers (`x_mm`, `y_mm`, `trackwidth_mm`, `h_deg`,
  `brightness_pct`).
- **Main Flow**:
  1. A caller polls a sensor wrapper (e.g. `Otos`, `ColorClassifier`,
     `OdomTracker`).
  2. The wrapper parses the same wire tokens (via `robot.protocol.parse_tlm`
     / `parse_so`, unchanged per SUC-001) into renamed local fields.
  3. Classification thresholds and reported values are numerically
     unchanged.
- **Postconditions**: Sensor readings and classifications are
  bit-for-bit identical to pre-076 for the same input stream.
- **Acceptance Criteria**:
  - [ ] `sensors/` files carry no unit-suffixed identifier and each carries
        a `# [unit]` comment.
  - [ ] Sensor-related unit tests in `tests/simulation/unit/` pass with
        unchanged numeric assertions.

---

## SUC-004: Preserve Calibration Workflow and Host-Side SIMSET Wire-Key Handling Across the Rename
Parent: UC-013, UC-014 (narrows)

- **Actor**: Developer / calibration tooling pushing calibration values via
  `SET` or exercising the `SIMSET` sim-error-model surface.
- **Preconditions**: `calibration/helpers.py`, `calibration/linear.py`,
  `calibration/angular.py`, `calibration/push.py`,
  `calibration/fit_sim_error_model.py` carry unit-suffixed identifiers
  (`target_deg`, `actual_mm`, `otos_x_mm`, `total_ms`,
  `left_mm_per_deg`/`right_mm_per_deg`). `fit_sim_error_model.py` also
  contains **host-side literal SIMSET wire-key strings** as dict keys
  (`SIMSET_BOUNDS["trackwidthMm"]`, `DEFAULT_CANDIDATE_KEYS`) — these are
  not Python identifiers and are excluded the same way `SimCommands.cpp`'s
  `kSimRegistry[]` string literals were excluded in 071.
- **Main Flow**:
  1. `push.py` builds `SET ml=<value>`, `SET tw=<value>`, `SET
     rotSlip=<value>` command strings from renamed local variables
     (`left_mm_per_deg`, `tw`, `rot_slip`) — the wire key text (`ml`, `tw`,
     `rotSlip`) is a separate literal, already unit-free (071 finding,
     reconfirmed on the host side this pass), untouched by any local
     variable rename.
  2. `fit_sim_error_model.py` sends `SIMSET <key>=<value>` using the exact
     dict-key string (`"trackwidthMm"`) as the wire key — the dict key
     string is excluded; the surrounding Python variable/function names
     that hold or compute the perturbation are renamed.
  3. Calibration fit/replay round-trips produce numerically identical
     results.
- **Postconditions**: Every `SET`/`SIMSET` wire key sent by calibration
  tooling is byte-identical to pre-076.
- **Acceptance Criteria**:
  - [ ] Every `SET`/`SIMSET` key string literal in `calibration/` is
        unchanged (diffed against pre-076): `ml`, `mr`, `tw`, `rotSlip`,
        `odomOffX/Y`, `odomYaw`, and every `SIMSET_BOUNDS`/
        `DEFAULT_CANDIDATE_KEYS`/step-size dict key (`trackwidthMm`, etc.).
  - [ ] `NezhaProtocol.set_config(**kwargs)` call sites that pass a wire key
        as a keyword argument (`set_config(sTimeout=500)`,
        `set_config(ml=..., mr=...)`) are **not** touched — the kwarg name
        *is* the wire key at that call site, and every one this pass found
        was already unit-free.
  - [ ] `tests/simulation/unit/test_calibration_push.py`,
        `test_calibrate_linear.py`, `test_calibration_helpers.py` pass
        unchanged.

---

## SUC-005: Preserve Navigation and Path-Following Behavior Across the Rename
Parent: UC-015, UC-016, UC-017 (narrows)

- **Actor**: Developer driving to a relative XY position or following a
  path (PurePursuit/Stanley).
- **Preconditions**: `nav/navigator.py`, `nav/_approach_utils.py`,
  `nav/camera_goto.py`, `nav/nav_params.py` carry unit-suffixed identifiers
  (`tolerance_mm`, `speed_mms`, `target_mm`, `r_mm`).
- **Main Flow**:
  1. A caller invokes `Navigator`/`camera_goto` with a target pose.
  2. The controller computes heading/distance error and issues drive
     commands through the (already-renamed, SUC-001) protocol layer using
     renamed locals.
  3. Arrival tolerance and path-tracking behavior are numerically
     unchanged.
- **Postconditions**: Navigation trajectories and arrival decisions are
  identical to pre-076 for the same inputs.
- **Acceptance Criteria**:
  - [ ] `nav/` files carry no unit-suffixed identifier and each carries a
        `# [unit]` comment.
  - [ ] Navigation-related unit/system tests pass with unchanged numeric
        assertions.

---

## SUC-006: Preserve TestGUI Operator Experience Across the Rename
Parent: UC-018 (narrows — device/session setup and live operation)

- **Actor**: TestGUI operator (human, via the PySide6 desktop app).
- **Preconditions**: `testgui/__main__.py`, `testgui/transport.py`,
  `testgui/commands.py`, `testgui/drive.py`, `testgui/sim_prefs.py`,
  `testgui/traces.py`, `testgui/operations.py`, `testgui/canvas.py` carry
  unit-suffixed identifiers (`read_ms`, `rotation_deg`, `trackwidth_mm`,
  `encoder_noise_mm`). `sim_prefs.py` additionally has a **host-internal
  profile-key layer** (`DEFAULT_PROFILE["trackwidth_mm"]`) mapped through an
  explicit table (`{"trackwidth_mm": "trackwidthMm", ...}`) to the real
  `SIMSET` wire key — the host-internal key is renamable, the mapping
  table's *value* (the wire key) is not.
  Its 24-file test tree (`tests/testgui/`) is **not** part of the default
  `uv run python -m pytest` baseline (`testpaths = ["tests/simulation"]`
  excludes it) and must be run explicitly.
- **Main Flow**:
  1. Operator selects transport/robot/camera, starts a session, drives via
     the canvas or command panel, and adjusts Sim Errors sliders.
  2. Every widget's displayed value, slider range, and issued command is
     computed from renamed fields/locals with unchanged numeric behavior.
  3. Traces/recordings capture the same data under renamed field names.
- **Postconditions**: The TestGUI looks and behaves identically; only
  Python identifiers changed.
- **Acceptance Criteria**:
  - [ ] `testgui/` files carry no unit-suffixed identifier (excluding the
        `sim_prefs.py` wire-key mapping table's right-hand-side values) and
        each carries a `# [unit]` comment.
  - [ ] `uv run python -m pytest tests/testgui -q` remains **579 passed, 2
        xfailed** (confirmed baseline this planning pass) throughout.
  - [ ] `sim_prefs.py`'s `PROFILE_TO_SIMSET`-style mapping table's SIMSET
        wire-key values are byte-identical to pre-076.

---

## SUC-007: Preserve `rogo` CLI and MCP Robot-Control Surface Behavior Across the Rename
Parent: UC-001 through UC-019 (narrows collectively — `rogo`/`robot_mcp` are
the CLI/agent-facing invocation surface for nearly every top-level UC)

- **Actor**: Developer (via the `rogo` console-script entry point) or an AI
  agent (via the `robot_mcp` MCP tool surface).
- **Preconditions**: `io/cli.py` (170 rename-eligible occurrences, the
  single largest file in this sprint's scope — implements `rogo`),
  `io/calibrate.py`, `io/robot_mcp.py`, and their companion `media/movie.py`
  carry unit-suffixed identifiers, and both depend (by import) on nearly
  every other subpackage: `robot/`, `sensors/color.py`,
  `config/robot_config.py`, `calibration/helpers.py`, `nav/camera_goto.py`.
- **Main Flow**:
  1. A user runs `rogo <verb> <args>` or an agent calls an MCP tool exposed
     by `robot_mcp.py`.
  2. The CLI/MCP layer resolves the active robot config (pydantic,
     unchanged — see SUC-008), drives through the (already-renamed)
     robot/protocol/nav/calibration layers, and reports results.
  3. Output formatting and exit codes are unchanged.
- **Postconditions**: Every `rogo` subcommand and MCP tool behaves
  identically to pre-076.
- **Acceptance Criteria**:
  - [ ] `io/cli.py`, `io/calibrate.py`, `io/robot_mcp.py` carry no
        unit-suffixed identifier and each carries a `# [unit]` comment.
  - [ ] `rogo help` and a representative smoke command round-trip
        identically pre/post-sprint (manual verification at ticket time;
        no automated CLI-invocation test exists today).
  - [ ] `tests/simulation/unit/test_cli.py` passes unchanged.

---

## SUC-008: Preserve Per-Robot Configuration Loading via Wholesale Pydantic/JSON Exclusion
Parent: UC-014 (narrows — configuration feeds the runtime-tunable `SET` path)

- **Actor**: Developer / any host process loading `data/robots/*.json`.
- **Preconditions**: `config/robot_config.py`'s pydantic models
  (`RobotConfig` and its 12 nested models) use bare attribute names as JSON
  keys with **no `Field(alias=...)` anywhere** — confirmed by a full read
  this planning pass, unchanged since 071's own finding.
- **Main Flow**:
  1. `get_robot_config()` loads and validates a `data/robots/*.json` file
     against the unchanged pydantic schema.
  2. Every unit-suffixed field this pass's own grep found in this file
     (`ticks_per_mm`, `wheel_diameter_mm`, `tag_offset_mm`,
     `gripper_offset_mm`, `odometry_offset_mm`, `wheelbase_mm`,
     `rotation_offset_deg[_neg]`, `min_wheel_mms`, `drive_axle_offset_mm`)
     is a pydantic field name mirroring a JSON key 1:1 — **excluded
     wholesale**, not renamed.
  3. `RobotConfig`'s convenience flat-accessor `@property` methods
     (`tag_offset_mm`, `gripper_offset_mm`) proxy the same-named nested
     field and are excluded alongside it, to avoid a confusing
     name-mismatch between a property and the field it proxies.
- **Postconditions**: `data/robots/*.json` and `robot_config.schema.json`
  are untouched; config loading is byte-identical to pre-076.
- **Acceptance Criteria**:
  - [ ] `git diff` shows zero changes to any file under `data/robots/`.
  - [ ] `config/robot_config.py`'s pydantic model class bodies (all 13
        classes) are byte-identical to pre-076; this pass's own grep found
        **zero** non-pydantic renamable identifiers in this file (the 28
        raw hits are all field names, their local-variable mirrors inside
        `_resolve_encoder_fields`, or documentation prose).
  - [ ] `tests/simulation/unit/test_robot_config.py` passes unchanged.

---

## SUC-009: Maintain Grep-able Physical-Unit Documentation on Renamed Host-Python Identifiers
Parent: UC-024 (new — "Maintain Consistent, Unit-Free Identifier Naming with
Grep-able Unit Documentation", the host-Python half of the same capability
071 established for `source/` under UC-022)

- **Actor**: Developer reading or modifying `host/robot_radio/` (or any
  host-side tool/script this sprint touches).
- **Preconditions**: `docs/coding-standards.md`'s Python convention section
  already exists (071), explicitly marked "not yet applied to any `host/`
  file." This sprint is the sprint that applies it.
- **Main Flow**:
  1. Developer opens a declaration this sprint renamed (e.g. `def
     send(self, cmd: str, read_timeout: int = 500) -> dict:`).
  2. The declaration's trailing comment begins with a `# [unit]` tag (e.g.
     `# [ms]`), stated once, in a standard, uniform position.
  3. Developer runs `grep -rn "# \[ms\]" host/` to find every quantity of
     that physical unit across the host codebase, independent of
     identifier spelling.
- **Postconditions**: Every renamed declaration's unit is discoverable by a
  single, uniform grep pattern; `docs/coding-standards.md`'s "not yet
  applied" language is updated to reflect this sprint's completion.
- **Acceptance Criteria**:
  - [ ] `docs/coding-standards.md` is updated to state the Python
        convention has been applied (by sprint 076, not the originally
        forward-referenced "072").
  - [ ] Every identifier renamed by this sprint's tickets carries a `#
        [unit]` comment at its declaration (spot-checked per ticket's
        acceptance criteria, not exhaustively enumerated here).
  - [ ] The single most pervasive rename (`read_ms` → `read_timeout`, ~216
        keyword call sites this pass counted) uses the exact target name
        already given as `docs/coding-standards.md`'s own worked example —
        every ticket touching a `read_ms` call site converges on this one
        name, not an implementer-invented alternative.
