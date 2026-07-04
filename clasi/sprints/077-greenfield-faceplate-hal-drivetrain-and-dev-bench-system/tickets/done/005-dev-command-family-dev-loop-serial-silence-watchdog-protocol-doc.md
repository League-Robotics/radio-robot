---
id: '005'
title: DEV command family, dev loop, serial-silence watchdog, protocol doc
status: done
use-cases:
- SUC-005
- SUC-006
depends-on:
- '004'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DEV command family, dev loop, serial-silence watchdog, protocol doc

## Description

Write `source/commands/dev_commands.{h,cpp}` (the `DEV` command family),
wire the whole dev loop in `source/main.cpp`, add the serial-silence
watchdog (non-negotiable, per the robot's runaway history), and document the
`DEV` vocabulary in `docs/protocol-v2.md`. This is the ticket that makes
tickets 3 and 4 reachable from outside the firmware for the first time.

## Acceptance Criteria

### `DEV` command family (`source/commands/dev_commands.{h,cpp}`)

- [x] Full vocabulary implemented exactly per the issue's locked table:
  - `DEV M <n> DUTY <duty>` → `motor.apply({duty_cycle})` → `OK DEV M <n>
    applied=<duty>`
  - `DEV M <n> VEL <velocity>` → `[mm/s]` → embedded PID closes the loop
  - `DEV M <n> POS <position>` → `[deg]` → onboard position move
  - `DEV M <n> VOLT <voltage>` → `[V]` → `ERR unsupported` (Nezha's
    `capabilities().voltage` is false; this is `apply()`'s capability gate
    firing, not a special case in the `DEV` handler)
  - `DEV M <n> NEUTRAL <B|C>`
  - `DEV M <n> RESET` → `MotorCommand.reset_position`
  - `DEV M <n> STATE` → `OK DEV M <n> pos=... vel=... applied=... wedged=...
    conn=...`
  - `DEV M <n> CAPS` → `OK DEV M <n> duty=1 volt=0 vel=1 pos=1 enc=1`
  - `DEV M <n> CFG k=v ...` → `motor.configure()` delta (e.g. `kp=0.8
    slew=400`)
  - `DEV DT PORTS <left> <right>` → binds `Drivetrain` to a motor-port pair
    (default: the robot's drive pair; coupled bench rig: `3 4`) — persists
    across `DEV STOP`/watchdog-neutral, resets only on reboot (see
    architecture-update.md Open Question 4 — this is the chosen default;
    document it in the header comment)
  - `DEV DT VW <v_x> <v_y> <omega>` → `[mm/s mm/s rad/s]`
  - `DEV DT WHEELS <left> <right>` → `[mm/s]`
  - `DEV DT NEUTRAL <B|C>` / `DEV DT STATE` / `DEV DT STOP`
  - `DEV STATE` → one line per component (all motors + drivetrain)
  - `DEV STOP` → all motors neutral, drivetrain idle
- [x] `<n>` addresses motors by **port** (1..4), matching how `NezhaHal`
      instantiates them (ticket 3) — never by an L/R role name.
- [x] Every `DEV` handler builds a `msg::MotorCommand`/
      `msg::DrivetrainCommand` and dispatches through `apply()` — exercising
      the full message plane (capability validation included) rather than
      calling any primitive setter directly from the command layer.
- [x] Argument parsing: resolve architecture-update.md Open Question 3
      (hand-rolled `ParseFn` per subcommand vs. extending `ArgSchema`) and
      document the choice in a header comment in `dev_commands.h`. Either
      approach is acceptable; consistency across all `DEV` subcommands is
      not required if a mix is simpler (e.g., `ArgSchema` for pure-positional
      forms like `DEV M <n> DUTY <duty>`, hand-rolled for `CFG k=v ...`).
- [x] Replies use the standard taxonomy exclusively:
      `CommandProcessor::replyOK`/`replyOKf` for success,
      `replyErr`/`replyErrf` for failure — no ad hoc reply formatting.
- [x] `DEV M …` motion deactivates drivetrain mode; `DEV DT …` reactivates
      it — only one authority (single-motor or drivetrain) drives the
      motors at a time. Since this firmware runs only the dev loop (no
      planner), this arbitration is the only authority conflict that exists
      and must be trivial to verify by reading `main.cpp`.

### Dev loop (`source/main.cpp`)

- [x] Loop body matches the issue's locked shape:
  ```
  while (true) {
      pollComms();                 // dispatch DEV/PING via CommandProcessor
      hal.tick(now);                // split-phase encoder schedule
      if (drivetrainActive) {
          auto out = drivetrain.tick(now, left.state(), right.state());
          left.apply(out.left);
          right.apply(out.right);
      }
      left.tick(now);                // staged commands execute (PID runs here)
      right.tick(now);
      watchdog.check(now);          // silence -> all neutral
  }
  ```
  (`left`/`right` here are whichever two `NezhaMotor`s `DEV DT PORTS` last
  bound — the loop itself does not hardcode which ports.)
- [x] `Communicator` (or an equivalent inline poll loop, per architecture-
      update.md Open Question 5 — default to reusing `Communicator` since it
      is confirmed dependency-clean) drives `pollComms()`.

### Serial-silence watchdog — NON-NEGOTIABLE

- [x] Default window ~1 s; settable (a `DEV`/config verb or equivalent — not
      hardcoded-only). Document the exact mechanism chosen (which verb sets
      it) in `dev_commands.h` and in the protocol doc.
- [x] On silence exceeding the window: all motors → neutral, drivetrain →
      idle, regardless of which command family (single-motor or drivetrain)
      was last active.
- [x] This behavior is present even though this is a bench-only firmware
      build with no planner to fight — the runaway history (see
      `.claude/rules/hardware-bench-testing.md` and prior incident notes)
      makes this a hard requirement, not a nice-to-have.

### Protocol documentation

- [x] `docs/protocol-v2.md` gains a new "Development commands" section
      (after §14 Debug Commands, or wherever the document's existing section
      numbering makes sense) documenting the full `DEV` vocabulary above,
      following the same format as the existing Motion Commands (§10)
      section: one subsection per verb, request/reply examples, units.

### Build

- [x] `python build.py --clean` succeeds with the dev loop fully wired.

## Testing

- **Existing tests to run**: None in `tests/` yet at this ticket's position
  (ticket 6 creates the new tree next). `tests_old/` is not touched.
- **New tests to write**: None required at this ticket — host-side scripted
  verification of the `DEV` vocabulary (`dev_exercise.py`) is ticket 7's job,
  once `tests/bench/` exists (ticket 6). If it is useful to hand-verify the
  dev loop over a raw serial terminal before ticket 6/7 land, do so and note
  the manual check in the PR description, but it is not a required
  artifact.
- **Verification command**: `python build.py --clean`. Full bench
  verification (does `DEV M 1 DUTY 30` actually spin the wheel) is ticket
  7's HITL gate.
