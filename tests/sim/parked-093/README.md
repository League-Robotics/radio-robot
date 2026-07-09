# tests/sim/parked-093/ — sprint 093 parking leaf

Sprint 093 ("Simplify the main loop — bare wheel-driving executive") gutted
`Rt::MainLoop`/`Rt::CommandRouter::buildTable()` down to a four-verb live wire
surface: `PING`, `HELLO` (from `systemCommands()`, which also still carries
the pre-existing `VER`/`HELP`/`ECHO`/`ID` liveness verbs untouched) and `S`/
`STOP` (from the trimmed `motionCommands()`). The `dev`/`telemetry`/`config`/
`pose`/`otos` command families, the `Planner`, the serial-silence and
streaming-drive watchdogs, and `Rt::Configurator`'s live wiring into the loop
are all un-wired — their source files, handlers, and includes are untouched
on disk (`clasi/sprints/093-.../architecture-update.md` Step 5) but nothing
in `buildTable()` calls them anymore, so every wire verb outside the four
above now replies `ERR unknown`.

Every file below either (a) dispatches a wire verb outside
`{PING, HELLO, S, STOP}` via `sim.command()`/`sim.command_on()`, or (b)
constructs a real `Rt::MainLoop` and asserts on `Planner`/`PoseEstimator`
behavior driven BY `MainLoop::tick()`. Per architecture-update.md's Decision
3, these are **parked, not deleted** — the greenfield-rebuild precedent
(`tests_old/`/`source_old/`) — because the motion-planner-segment follow-on
sprint(s) will want this coverage back once the corresponding command family
is re-wired. `pyproject.toml`'s `norecursedirs` excludes this whole
`parked-093/` leaf from collection (bare name, matching `tests_old`/
`source_old`'s own basename-fnmatch behavior — verified empirically via
`pytest --collect-only`).

None of these are 093-002 regressions: every failure that led to a file
being parked here traces to a wire reply of `ERR unknown` (or, downstream of
that, a `TLM`/`OK`-shaped parse assertion tripping over an `ERR unknown`
line it wasn't expecting) — i.e. the command genuinely isn't registered
anymore, not a bug in what tickets 001/002 changed.

## What has to come back before a file can return

- **`dev` family** (`dev_commands.cpp` — `DEV M`/`DEV DT`/`DEV STATE`/
  `DEV STOP`/`DEV WD` re-wired into `buildTable()`):
  `test_dev_command_outbox.py` (+ `dev_command_outbox_harness.cpp`),
  `test_determinism.py`, `test_encoder_error_injection.py`,
  `test_errored_observation.py`, `test_plant_correctness.py`,
  `test_stiction_and_motor_lag.py`, `test_velocity_pid_response.py`.

- **`telemetry` family** (`telemetry_commands.cpp` — `SNAP`/`STREAM`
  re-wired): folded into the motion/otos entries below wherever a file also
  polls `SNAP` for `mode=`/`otos=` fields — see `test_tlm_stream_snap.py`.

- **`motion` family beyond S/STOP** (`motion_commands.cpp`'s `T`/`D`/`R`/
  `TURN`/`RT`/`G`, and the `Planner`/`StopCondition` dispatch behind them,
  re-wired into `buildTable()` + `MainLoop::tick()`):
  `test_motion_commands.py`, `test_motion_commands_arc_turn.py`,
  `test_motion_commands_goto.py`, `test_motion_overshoot_regression.py`,
  `test_motion_verbs_full_sequence.py`, `test_mode_machine.py`,
  `../system/test_tour_geometry.py`.

- **`config` family** (`config_commands.cpp` — `SET`/`GET` re-wired):
  `test_config_registry.py`.

- **`pose` family** (`pose_commands.cpp` — `SI`/`ZERO` re-wired):
  `test_pose_commands.py`.

- **`otos` family** (`otos_commands.cpp` — `OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/
  `OA` re-wired into `buildTable()`):
  `test_otos_commands.py`, `test_otos_commands_nodev.py`
  (+ `otos_commands_harness.cpp`), `test_otos_divergence.py`,
  `test_otos_error_injection.py`.

- **Multiple families at once** (`config` + `pose` + `otos` + motion in one
  file): `test_config_pose_set_otos_surface.py`.

- **`telemetry` (`SNAP`) + `dev` together**: `test_pose_estimate_tolerance.py`
  (drives via `DEV`, reads back via `SNAP`).

- **`Rt::Configurator`/`Subsystems::PoseEstimator` wired live into
  `Rt::MainLoop::tick()`** (not just the classes existing — their own
  isolated harnesses, e.g. `tests/sim/unit/configurator_harness.cpp`/
  `pose_estimator_harness.cpp`, are KEPT and still pass):
  `test_dev_loop_pose_estimator.py`
  (+ `dev_loop_pose_estimator_harness.cpp`).

- **The serial-silence / streaming-drive watchdogs** (removed under
  architecture-update.md Decision 2, stakeholder-owned and bench-posture-
  only — this needs a stakeholder decision to reverse, not just a
  command-table re-wire): `test_watchdog_policy.py`.

- **The full pre-093 command table, all families at once** (these two
  files' own designs enumerate/exercise *every* registered verb, so they
  self-heal once every family above is back — no per-file code change is
  needed beyond the families they cover): `test_command_smoke.py`,
  `test_protocol_roundtrips.py`.

`test_tlm_stream_snap.py` needs the `telemetry` family (`SNAP`/`STREAM`)
plus, for its `mode=` assertions, the `Planner`/motion-family re-wiring
above.
