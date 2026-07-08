---
status: done
sprint: 091
tickets:
- 091-002
---

# Replace command-derived `portInUse_` with a configured poll-set

## Context

`NezhaHardware::portInUse_[kPortCount]` ([source/subsystems/nezha_hardware.h](source/subsystems/nezha_hardware.h))
is presented as an ownership/"in-use" flag, but its only real job is
**schedule membership**: which ports the I2C flip-flop sequencer bothers to
sample each `tick()` (the request/collect encoder-readback cadence at
[nezha_hardware.cpp:52-67](source/subsystems/nezha_hardware.cpp#L52-L67)).
It has nothing to do with command *delivery* — the broadcast estop path
(`emergencyNeutralize()` → `hardware_.apply(broadcast)`) already loops
`p = 1..kPortCount` and applies to **every** motor unconditionally
([nezha_hardware.cpp:78-80](source/subsystems/nezha_hardware.cpp#L78-L80)),
never consulting the flag. So `portInUse_` gates exactly one thing: "whose
encoders do we poll."

That is a pure **configuration** fact — we already know it. `NezhaHardware`
is handed `configs[kPortCount]` at construction and the drivetrain binds its
left/right pair; a differential bot polls 2 ports, a mecanum polls 4, and
nothing about that set is dynamic. Yet today it is *derived from command flow*,
which produces a cluster of smells:

- **The name lies.** "In use" reads as a claim; it means "ever commanded,
  therefore scheduled." That mismatch is why the concept needs explanatory
  prose in ~5 places (hal_command.h, blackboard.h, main_loop.cpp, the
  nezha_hardware.cpp inline comment, and architecture-update.md's Design
  Rationale 5).
- **Side-effect mutation across three write sites**
  ([:44](source/subsystems/nezha_hardware.cpp#L44),
  [:84](source/subsystems/nezha_hardware.cpp#L84),
  [:92](source/subsystems/nezha_hardware.cpp#L92)), plus one path
  (`apply()`'s broadcast branch) that must deliberately *not* write it. Any
  future command path must remember the flag or silently create a port that
  accepts commands but is never sampled.
- **It latches forever** — there is no release anywhere. A single `DEV M 3`
  during a bench session permanently adds port 3 to the round-robin, cutting
  the drivetrain pair's sampling cadence by a third for the rest of the
  session, with no way back short of reboot.
- **Sim can't see it.** `SimHardware` has no schedule concept, so the whole
  invariant lives only in the real-hardware leaf and no sim test can exercise
  it.

## Scope

Make the poll-set an explicit, configured input — set once, never mutated by
command flow:

- Add the fact to config: either a `bool` on `msg::MotorConfig` (it already
  carries `port`) or a poll-mask passed to the `NezhaHardware` constructor,
  derived from which ports the boot config actually populates. Initialize a
  constant `polled_[kPortCount]` once in the constructor.
- Delete all three `portInUse_ = true` write sites
  ([:44](source/subsystems/nezha_hardware.cpp#L44),
  [:84](source/subsystems/nezha_hardware.cpp#L84),
  [:92](source/subsystems/nezha_hardware.cpp#L92)). `apply()` and `tick()`
  stop mutating schedule state as a side effect of data flow.
- Delete the broadcast exemption: the `return; // broadcast never marks a
  port in-use` branch in `apply()` and every comment that explains it
  (hal_command.h's `allPorts` note, blackboard.h, main_loop.cpp's
  hardwareBroadcastIn block, architecture-update.md Design Rationale 5). Once
  `apply()` touches no schedule state, a broadcast neutral is no longer a
  special case.
- Rename `anyPortInUse()`/`nextPortInUse()` → `anyPolled()`/`nextPolled()`
  reading the constant mask; drop `portInUse_`.

## Decision needed (call out in the ticket)

What happens to `DEV M <n>` addressed at a port **not** in the configured
poll-set? Today it self-schedules. Under a configured poll-set it would be
either applied-but-not-sampled (open-loop, no encoder readback) or rejected
outright. On a 2-motor bot "commanding a port with no motor" arguably should
be inert or an error — likely a cleanup — but it is the one observable
behavior change, so it must be a deliberate choice, not a silent switch.

## Acceptance

- `portInUse_` and all three side-effect write sites are gone; schedule
  membership is a constant set from config, established at construction.
- The broadcast-exemption branch and its apology comments are removed;
  `apply()`/`tick()` no longer mutate schedule state.
- Estop/broadcast delivery is unchanged (still hits all ports); the
  `DEV M <n>`-on-unconfigured-port behavior is whatever the decision above
  settles, documented in the ticket.
- Builds; `tests/sim` green; a bench check confirms the configured drivetrain
  ports are sampled and encoder readback is unaffected.
