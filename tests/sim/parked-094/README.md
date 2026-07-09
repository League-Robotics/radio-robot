# tests/sim/parked-094/ — the motor/hardware-inbound-queue teardown's parking leaf

Prep work toward "Drivetrain owns its motors" (sprint 094) removed the
motor/hardware inbound message queues from `Rt::Blackboard` entirely:
`Mailbox<msg::MotorCommand> motorIn[kPortCount]`, `bool
motorResetIn[kPortCount]`, and `Mailbox<msg::MotorCommand>
hardwareBroadcastIn` are gone (see `source/runtime/blackboard.h`'s file
header). `Subsystems::Hardware::tick()` no longer takes a motorIn[]/
motorResetIn[] pair at all (`source/subsystems/hardware.h`), and
`Rt::MainLoop::tick()` no longer has a `routeOutputs()` step
(`source/runtime/main_loop.h`/`.cpp`) — `Subsystems::Drivetrain`'s own held
output currently goes nowhere.

Practical effect: `S`/`STOP` still parse and reply correctly (the command
family itself, `motion_commands.cpp`'s `handleS`/`handleStop`, is
untouched — it posts to `bb.driveIn`, which still exists and is still
ticked by `Subsystems::Drivetrain`), but the commanded wheel targets never
reach `Subsystems::Hardware`/the simulated plant any more. Any sim test that
asserts real wheel motion (`sim.vel()`/`sim.true_velocity()`/`sim.pwm()`
after an `S`/`STOP`) is now testing a severed path and fails not because of
a regression in what it originally covered, but because the mechanism that
carried a Drivetrain's output to a motor no longer exists at all — pending
sprint 094 giving the Drivetrain its own motors to write directly.

Per `tests/sim/parked-093/README.md`'s own precedent: parked, not deleted.
`pyproject.toml`'s `norecursedirs` excludes this whole `parked-094/` leaf
from collection (bare name `parked-094`, matching `parked-093`'s own
basename-fnmatch behavior).

## What's here

- `unit/test_bare_loop_drive_severed.py` — the plant-motion half of
  `tests/sim/unit/test_bare_loop_commands.py` (093-003's four-verb suite):
  `test_s_drives_both_wheels_to_commanded_targets_and_direction`,
  `test_s_with_differing_sign_wheels_spins_them_opposite_directions`,
  `test_stop_neutralizes_both_wheels_regardless_of_prior_drive_state`. The
  command-reply-only tests from that same original file (`PING`, `HELLO`,
  `ERR unknown` for an unregistered verb) are UNAFFECTED and stay live at
  `tests/sim/unit/test_bare_loop_commands.py`.

## What has to come back before a file can return

The Drivetrain writing its own motors directly (sprint 094's tickets, e.g.
"Drivetrain owns motors + executor + ring") — once `S`'s commanded wheel
targets reach the plant again by whatever new mechanism replaces
`bb.motorIn[]`, move `test_bare_loop_drive_severed.py` back to
`tests/sim/unit/` (folding it back into `test_bare_loop_commands.py` if that
reads better at the time).
