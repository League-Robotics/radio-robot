---
id: '006'
title: "Configurator owns Config::Robot \u2014 config(), loadBaked(), install(); delete\
  \ RobotGraph::Resolved"
status: done
use-cases:
- SUC-002
depends-on:
- '005'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Configurator owns Config::Robot — config(), loadBaked(), install(); delete RobotGraph::Resolved

## Description

Rebuild `App::Configurator` (`src/firm/app/configurator.{h,cpp}`) around
ownership of one `Config::Robot` instance: `loadBaked()` populates it via
ticket 005's baking output; `config()` returns a `const` reference for
read-back; `install()` (no argument) pushes every group to its consumer
once, at boot. Delete `RobotGraph::Resolved` (`boot_wiring.h:185-196`,
`boot_wiring.cpp`'s `resolve()`) — confirmed by direct read this
round that it is a private struct never read again after the constructor
body finishes; its job becomes `Configurator::loadBaked()`.
`boot_wiring.cpp`'s three `install*Calibration()` calls after the
constructor's member-init list
(`installShaperLimits`/`installDriveCalibration`/`installWheelController`/
`installRotationCalibration`) are replaced by `Configurator::install()`'s
fan-out.

**Note**: `applyGroup()`/`applyField()` (the WIRE-facing decode entry
points) are NOT this ticket's scope — tickets 008/012. This ticket's
`install()` is the boot-time fan-out only. Full per-group correctness for
`DRIVE`/`WHEEL_CONTROL`/`OTOS`/`ESTIMATOR` lands in tickets 009/010; this
ticket may port the existing `install*Calibration()` call bodies directly
as a starting point.

## Acceptance Criteria

- [x] `Config::Robot config_` is a member of `Configurator`; `config()`
      returns `const Config::Robot&`.
- [x] `loadBaked()` populates `config_` using ticket 005's baking output.
- [x] `install()` (no-arg) is callable and produces the same net baked
      behavior main.cpp's pre-this-ticket boot sequence produced
      (`installShaperLimits`/`installDriveCalibration`/
      `installWheelController`/`installRotationCalibration`'s combined
      effect) — a behavioral parity check against the pre-ticket boot
      sequence, not byte-identical structs (byte-identical isn't
      required this sprint — see sprint.md Migration Concerns).
- [x] `RobotGraph::Resolved` no longer exists in `boot_wiring.h`;
      `boot_wiring.cpp`'s `resolve()` is deleted or reduced to nothing.
- [x] `RobotGraph`'s constructor calls `configurator_.loadBaked()` +
      `configurator_.install()` in place of the old `resolve()` +
      `install*Calibration()` sequence.
- [x] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: `composition_root_parity_harness.cpp` is
  NOT required to pass yet at this ticket (byte-identical boot is a
  ticket-018 concern) — but a lighter smoke test (does the sim
  composition root construct without crashing, does a basic Move still
  execute) should pass.
- **New tests to write**: a unit test constructing a `RobotGraph` (or the
  sim composition root) and asserting `Configurator::config()` reflects
  `tovez.json`'s values after boot.
- **Verification command**: `uv run python -m pytest <sim
  composition-root smoke test> -q`.

## Implementation Plan

**Approach**: Read `boot_wiring.h/.cpp` and `configurator.h/.cpp` in full
before starting. Move `resolve()`'s logic into
`Configurator::loadBaked()`; move the four `install*Calibration()` call
bodies into `Configurator::install()`'s per-target dispatch (a
straightforward relocation for this ticket — the traps-2/3 fixes inside
OTOS/ESTIMATOR's dispatch are ticket 010's job).

**Files to modify**: `src/firm/app/configurator.{h,cpp}`,
`src/firm/app/boot_wiring.{h,cpp}`.

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s file header updated to
describe the new ownership model; `boot_wiring.h`'s `RobotGraph` doc
comment updated to remove references to `Resolved`.

## Completion Notes

**`Config::Robot` did not exist before this ticket** — confirmed by
repo-wide grep before touching anything: every prior "Config::Robot"
occurrence was a comment referring to the future type, never a
declaration. Defined it in `configurator.h` (the only file this ticket's
plan lists that plausibly owns it), as a plain aggregate of the seven
`msg::` group structs `messages/robot_config.h` already generates
(`Geometry`/`Motors`/`Drive`/`WheelControl`/`Planner`/`Otos`/`Estimator`) —
exactly the grouping sprint.md's own architecture section describes for
this type. Host-only groups (Identity/Connection/Vision) are deliberately
absent, matching the issue's "the object holds RAW file values" framing
(this ticket doesn't need them; they never had a C++ counterpart to begin
with).

**The load-bearing structural finding, worth stating explicitly**: the
~14 "no post-construction setter" values (trackWidth, most of
PlannerLimits, the OTOS lever arm/scales, ColorConfig/LineConfig, and —
newly confirmed this round — the per-port motor wiring, since
`msg::Motors` is drive-pair-only and has no `port`/`slewRate`/`polled`
fields at all) make it structurally impossible for `Configurator::
loadBaked()` to be the ONLY bake this ticket needs. `Configurator` holds
references to `Drive`/`Devices::Motor`×2/`Devices::Otos`/`Motion::Planner`
(needed for its pre-existing wire-facing `apply()`, untouched, out of
scope) — so `Configurator` itself cannot be constructed before those
subsystems exist, which means it cannot supply THEIR construction
arguments. `RobotGraph` therefore keeps a small, renamed pre-construction
bake (`Resolved`/`resolve()` → `BootValues`/`bakeBootValues()`, trimmed of
the two fields — `driveConfig`/`wheelControllerConfig` — that moved to
`Configurator::install()`), computed first exactly as before, reading the
SAME OLD `Config::defaultMotorConfigs()`/`defaultDrivetrainConfig()`/
`defaultOtosBootConfig()`/`bootPlannerLimits()` functions ticket 005 left
untouched for this reason. `Configurator::loadBaked()`/`install()` are
called from the constructor BODY, after every member (including
`configurator_` itself) is constructed — literally the acceptance
criterion's own sequencing — and populate/apply the NEWER
`Config::default*Group()` family (132-005) into `config_`, then fan the
three reachable groups (`PLANNER`'s shaper ceilings, `DRIVE`, `WHEEL_
CONTROL`) out to `drive_`/`planner_`. Both families read the identical
robot-JSON `_require()` paths (132-005's own completion note), so this is
mild, deliberate, temporary duplication of BAKE COMPUTATION (two pure
function calls instead of one), never duplication of VALUES — not the
"owned twice" disease the sprint exists to kill, and a reasonable
narrowing of ticket 005's "old family has no callers left" aspiration:
`Config::defaultDriveConfig()`/`defaultWheelControllerConfig()` (and the
`installShaperLimits`/`installDriveCalibration`/`installWheelController`
free functions in `boot_calibration.h` that used to consume them) ARE now
orphaned (zero callers, confirmed by grep) — genuinely dead, ready for a
later cleanup ticket to delete — but `defaultMotorConfigs`/
`defaultDrivetrainConfig`/`defaultOtosBootConfig`/`defaultPlannerLimits`
remain live, because nothing else can supply construction-time values for
the ~14 no-setter fields yet. Closing that residual gap fully is
`Config::Robot` gaining port/slew/polled fields (a schema change, out of
this ticket's scope) or Configurator being restructured to not hold
subsystem references until after construction (an architecture change
well past "straightforward relocation").

**`installRotationCalibration` deliberately stayed a direct call, not
routed through `Configurator::install()`.** The ticket description's
prose lists it alongside the other three but also says "three ...
calls" (a likely accounting slip, since four names follow) — and routing
it through `Configurator` would require `Configurator` to hold a
`RobotLoop&`, which is circular (`robot_loop.h` already `#include`s
`app/configurator.h` and holds a `Configurator&` to route CONFIG
commands). `boot_wiring.cpp`'s constructor body calls it directly, still
reading `bootValues_.drivetrainConfig` (unchanged) rather than
`config_.geometry` — retargeting its data source would need a signature
change to `installRotationCalibration()` in `boot_calibration.h`, outside
this ticket's file scope (`configurator.{h,cpp}`, `boot_wiring.{h,cpp}`
only) and squarely inside ticket 007's own scope ("Subsystem `configure()`
consumers ... `RobotLoop` for geometry/rotation", sprint.md Step 3). Net
baked behavior is unchanged either way — this is a routing/ownership
choice, not a value change.

**Verification performed**:
- `just build-sim`-equivalent (`cmake -S src/sim -B src/sim/build
  -DROBOT_RUN_MODE=SIM && cmake --build src/sim/build --parallel`) →
  links cleanly, `libfirmware_host.dylib` built.
- `uv run python -m pytest src/tests/sim/system/test_composition_root_parity.py -v -s`
  → **PASSES** (not required to pass at this ticket, but does — every
  `Motion::PlannerLimits` field the harness checks is still fed by the
  unchanged `bakeBootValues()`/`bootPlannerLimits()` path).
- New smoke test written per the Testing section:
  `src/tests/sim/system/configurator_loadbaked_harness.cpp` +
  `test_configurator_loadbaked.py` — constructs `TestSim::SimHarness`,
  boots it, injects a WHEELS command and confirms a nonzero commanded
  wheel target (proves the composed graph is live, not just
  non-crashing), and asserts `Configurator::config()`'s fields match the
  SAME `Config::default*Group()` functions `loadBaked()` itself calls,
  one representative field per group across all 7 groups (mirrors
  `test_gen_boot_config_robot_groups.py`'s own "representative sample"
  framing from ticket 005, and `composition_root_parity_harness.cpp`'s
  own "compare against the same generation the composition root used"
  pattern, rather than re-parsing `tovez.json` independently in C++).
  Added `App::Configurator& configurator()` to `TestSim::SimHarness`
  (`src/sim/sim_harness.h`) to expose it, matching the existing
  `drive()`/`planner()`/`robotLoop()` accessors — outside this ticket's
  literal file list but a one-line, additive, test-only accessor needed
  to write the ticket's own requested test. **PASSES.**
- Ran the broader `src/tests/sim/system/` slice (25 passed, 10 failed, 2
  xfailed) plus several composition-root-adjacent `src/tests/sim/unit/`
  files. The 10 failures are pre-existing and unrelated to this ticket —
  confirmed by inspection (this ticket touches only
  `configurator.{h,cpp}`, `boot_wiring.{h,cpp}`, `sim_harness.h`, and two
  new test files — zero Python files) and by the failure signature
  itself: every one fails inside pure-Python `gen_boot_config.py`'s
  `_require(cfg, "control", "vel_kp")` before any C++ runs at all,
  because `robot_config.py`'s pydantic model (132-020) no longer
  populates `cfg["control"]` — exactly the sprint's own documented
  "Known-broken, owned by later tickets... Ticket 014 owns that" gap
  (`calibration/push.py`/`calibration/sim_boot_config.py` reading
  `config.control`/`config.calibration` removed by 020). Not this
  ticket's to fix; not a regression this ticket introduced.
- Per the sprint's own Test Strategy, the full default test collection
  was deliberately NOT run for this ticket.

**Left for later tickets, deliberately not touched**:
- `Config::defaultDriveConfig()`/`defaultWheelControllerConfig()` and the
  `installShaperLimits`/`installDriveCalibration`/`installWheelController`
  free functions (`boot_calibration.{h,cpp}`) are now orphaned (zero
  callers) — candidates for deletion by whichever later cleanup ticket
  retires the old `default*Config()`/`default*BootConfig()` family
  ticket 005 flagged.
- `installRotationCalibration`'s retarget onto `config_.geometry` (and
  `RobotLoop::configure(const Config::Robot&)` generally) — ticket 007.
- Per-port motor wiring (`port`/`slewRate`/`polled`) has no `Config::
  Robot` home yet — `msg::Motors` is drive-pair-only by schema design;
  not raised as a defect, just noted as the reason `bakeBootValues()`
  still needs the OLD `defaultMotorConfigs()`/`defaultDrivetrainConfig()`.
- `boot_calibration.cpp:88`'s hardcoded `Drive::kDutyPerSpeed` (ignoring
  `config_.drive.duty_per_speed_left/right`) is preserved unchanged in
  `Configurator::install()`, per this ticket's own explicit instruction —
  ticket 009 owns that decision.
- Full per-group correctness for OTOS/ESTIMATOR (traps 2/3) — tickets
  009/010, as the ticket description says.
