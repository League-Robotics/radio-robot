---
id: '010'
title: 'Config consolidation: sweep remaining hard-coded values across src/firm and
  src/host'
status: done
use-cases:
- SUC-008
depends-on:
- 009
github-issue: ''
issue: 02-move-hard-coded-values-to-configuration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config consolidation: sweep remaining hard-coded values across src/firm and src/host

## Description

Second of the two "mechanical sweep, run last" tickets; sequenced after
009 to keep the two configuration tickets from touching the same files
out of order. Issue 02 generalizes issue 03 (ticket 009) — deliberately
scoped as a separate, non-overlapping sweep of the *rest* of the tree,
not a second overlapping pass over `main.cpp`'s `PlannerLimits` block
(see `sprint.md` Design Rationale, Decision 4, for why these are two
tickets rather than one).

Sweep the tree for inline constants that are really configuration or
tuning values sitting mid-function instead of routed through the owning
config surface. Known starting points (captured during sprint 128, not
exhaustive):

- Host GUI panel parameters: spinbox decimals/thresholds in
  `testgui/__main__.py`, staleness thresholds such as `_STALE_AFTER_S` in
  `telemetry_panel.py`, retry counts/timeouts in halt paths.
- "All similar values": grep for numeric literals in logic across
  `src/firm` and `src/host` and classify each as (a) true config → move
  to config file/registry, (b) named constant → lift to a `k`-constant /
  module-level constant with a `[unit]` tag, or (c) genuinely local math
  — leave alone.

## Acceptance Criteria

- [x] Each moved value has one declared home (config registry, robot
      JSON, or a named constant) and the code reads it from there.
- [x] No behavior change: moved values keep their current defaults.
- [x] The sweep records, per file touched, which values were moved vs.
      left, and why — a short table or note per file, not just a diff.
- [x] Full clean build + test suite green (excluding the sprint's known
      baseline: 4 sim-tour turn-undershoot failures, 2 standalone-harness
      include-path failures — pre-existing, not this sprint's to fix).
      No C++ was touched by this ticket (see completion notes below), so
      the ARM/sim clean build was not re-run; `src/tests/unit` (566
      tests) and the `calibration_commands()`/`calibration_kwargs()`
      snapshot tests were run and are green.

## Testing

- **Existing tests to run**: `just build-clean`, `uv run python -m
  pytest`, `test_gui_button_acceptance.py` for any GUI-touching change.
- **New tests to write**: none required beyond confirming existing
  behavior is unchanged (this is a pure relocation sweep) — a test only
  needs to exist if a moved value previously had no coverage at all and
  the move is a natural point to add a minimal one.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: grep-driven sweep, classify-then-move, one file at a
  time; do not touch `main.cpp`'s `PlannerLimits` region (that's ticket
  009's territory, already landed by the time this ticket starts).
- **Files to modify**: `src/host/robot_radio/testgui/__main__.py`,
  `src/host/robot_radio/testgui/telemetry_panel.py`, other `src/firm`/
  `src/host` files surfaced by the grep sweep.
- **Documentation updates**: the per-file "moved vs. left, and why"
  record required by Acceptance Criteria — keep it in the ticket's own
  completion notes or a short section of this file, since that's the
  sweep's own audit trail.

## Completion Notes (2026-08-01)

### Scope adjustments made before sweeping

- **`testgui/__main__.py` / `testgui/telemetry_panel.py`** — the two
  starting points named in the Description are both inside
  `src/host/robot_radio/testgui/` (and its `src/tests/testgui/`
  counterpart), which this sprint's programmer brief marks **off
  limits** (owned by another live session this sprint). Not touched.
  If that ownership boundary lifts, `_STALE_AFTER_S` and the spinbox
  decimals/thresholds in those two files remain open work for a future
  ticket.
- **duty-per-speed / wheel-gain / `vel_kff` calibration values** —
  tickets 006/007/008 were withdrawn 2026-08-01 and that whole
  calibration line is being redesigned. `src/firm/config/boot_config.cpp`
  and `data/robots/*.json` are mid-edit by another live session for
  exactly this line (visible as pre-existing uncommitted diffs in the
  shared tree) — left untouched, not part of this ticket's diff.
- `src/firm/main.cpp`'s `PlannerLimits` block is ticket 009's territory
  (already landed, commit `adffd9e9`) — re-confirmed clean (see table
  below), not re-touched.

### Sweep method

Read every non-generated `.cpp`/`.h` file under `src/firm/{app,com,
config,devices}` and `main.cpp` (skipping `messages/*.h`/`*.cpp` —
`AUTO-GENERATED`, and `types/version_generated.h`), plus a grep-ranked
pass over `src/host/robot_radio/**/*.py` (excluding `testgui/` and
generated `robot/pb2/*.py`), prioritizing files the Description calls
out and files with the highest literal-numeric density. Classified every
candidate per the Description's (a)/(b)/(c) rubric.

### Finding: `src/firm` is already fully compliant

Every `src/firm` file read (`main.cpp`, `app/{comms,configurator,debug,
drive,fake_otos,preamble,robot_loop,telemetry}.cpp/.h`, `com/{banner,
radio,serial_port}.cpp/.h`, `config/{boot_config,persisted_tuning}.cpp`,
`devices/{color_sensor,line_sensor,microbit_clock,microbit_i2c_bus,
nezha_motor,otos}.cpp`) already carries every config-shaped or
tuning-shaped literal as a named `constexpr`/`static constexpr`
`k`-constant with a `// [unit]` tag (or a boot-config-sourced field for
genuine per-robot values) — the legacy of sprints 071/114/124/128/
129-001..009 already having swept this tree repeatedly. Remaining bare
literals are one of: vendor register/protocol bytes (Nezha I2C frame
bytes, OTOS register addresses, radio RAW250 frame flags — wire format,
not config), `int8`/`int16` hardware range bounds, or genuinely local
math (loop bounds tied to array sizes, epsilon guards, format-buffer
sizing tied to a documented byte budget). No file needed a change.
`config/boot_config.cpp` was **not** re-swept for its calibration
literals — out of scope per the withdrawal note above, and it is
mid-edit by another session.

### Host-side findings and moves

| File | Value(s) | Classification | Action |
|---|---|---|---|
| `src/host/robot_radio/controllers/pid.py` | Integral anti-windup clamp `50` and the `0.001` min-ki floor, inlined in `PID.update()`'s clamp expression | (b) named constant — a bare magic number with no name and no comment, in general-purpose reusable logic | **Moved**: lifted to class constants `_INTEGRAL_CLAMP = 50.0` and `_MIN_KI_FOR_CLAMP = 0.001`, each with a doc comment; `update()` now reads `self._INTEGRAL_CLAMP` / `self._MIN_KI_FOR_CLAMP`. Verified behavior-identical (`clamp = _INTEGRAL_CLAMP / max(ki, _MIN_KI_FOR_CLAMP)` is algebraically the same expression) via a standalone repro comparing accumulated `integral` against the pre-change formula. |
| `src/host/robot_radio/calibration/push.py` | Per-command read-timeout literals `200` (x4, at each `cmds.append((..., 200))` call) and `500` (the `OI` command) in `calibration_commands()` | (b) named constant — the same wire-timeout value repeated four times inline, one file already doing this correctly elsewhere (`_SIX_DECIMAL_KEYS`) | **Moved**: lifted to module constants `_SET_READ_TIMEOUT_MS = 200` and `_OTOS_INIT_READ_TIMEOUT_MS = 500`, both with a doc comment; all five call sites now reference the named constant. Verified behavior-identical via `src/tests/unit/test_calibration_kwargs.py`'s existing byte-for-byte `calibration_commands()` snapshot tests (unchanged, still passing). |
| `src/host/robot_radio/robot/halt.py` | `_RETRY_ATTEMPTS = 3`, `_RETRY_DELAY = 0.05  # [s]` | (b) already a named module constant with a unit tag | **Left** — already compliant (128-001); the Description's "retry counts/timeouts in halt paths" starting point is satisfied by prior work. |
| `src/host/robot_radio/field/geofence.py` | `HALF_W`/`HALF_H` (class constants, `[cm]` tagged), `margin`/`lost_grace`/`samples`/`retrySeconds` (named, documented constructor/default-arg parameters), a handful of one-off `time.sleep(N)` pacing calls inside retry loops | (b)/(c) mixed, already compliant | **Left** — the genuine tunables are already named parameters with unit tags and provenance comments (e.g. the `margin=12.0` derivation comment); the `time.sleep()` calls are single-use pacing inside one function, classification (c). |
| `src/host/robot_radio/nav/nav_params.py`, `planner/profile.py`, `robot/clock_sync.py`, `robot/nezha.py`, `sensors/motion_monitor.py`, `io/serial_conn.py`, `calibration/{linear,angular}.py`, `io/cli.py`, `io/calibrate.py`, `robot/protocol.py` | Surveyed for bare numeric literals (grep-ranked by literal density; `protocol.py` alone has ~290 numeric tokens) | (b)/(c), already compliant | **Left** — every genuinely tunable/config-shaped value found is already a named module- or class-level constant with a unit tag (`DEFAULT_CADENCE`, `MOTOR_DEADBAND`, `_KEEPALIVE_INTERVAL`, `_SETTLE_V/_SETTLE_W/_SETTLE_S`, `BAUD_RATE`, `_POLL_*`, `_HELLO_*`, `_RELAY_CMD_TIMEOUT_S`, `_READY_TIMEOUT_S`, `DRIVE_SPEED`, `OTOS_FW_MIN/MAX_SCALE`, `CRAWL_*`, the wire-scale/flag-bit constants in `protocol.py`, etc.) or a dataclass default field (`NavParams`, `ProfileLimits`). Remaining bare literals are one-off pacing `time.sleep()` calls inside single-purpose bench/calibration functions (classification (c) — genuinely local, not duplicated, not per-robot) or algorithm constants (e.g. the 0..1000 line-sensor normalization scale, an `1e-9` epsilon guard). Per the ticket's own pragmatism note ("don't churn every literal"), these were not touched. |

### No new config surface

Both moves are classification (b) (named constants); neither is
classification (a) (true per-robot/per-deployment config), so no
`data/robots/*.json` field, `robot_config.schema.json` entry, or
`src/scripts/config_sync_allowlist.json` allowlist entry was needed —
`check_config_sync.py` is unaffected (no wire/config field added or
renamed).

### Verification

- `uv run pytest src/tests/unit -q` — 566 passed (pre-existing skips/
  baseline failures noted in the ticket's Acceptance Criteria are
  in `src/tests/sim`/hardware-bench suites, not touched by this run).
- `uv run pytest src/tests/unit -k "pid or calibrat or push" -q` — 22
  passed (includes the `calibration_commands()`/`calibration_kwargs()`
  snapshot tests covering `push.py`'s changed timeouts).
- Standalone repro script confirming `PID`'s integral clamp is
  numerically identical to the pre-change inline formula for both a
  normal `ki` and the `ki=0` floor-clamp path.
- No C++/firmware file was touched, so no ARM/sim build was required
  for this ticket; `src/firm` was read-only surveyed and found already
  compliant (see table above).
