---
status: done
sprint: '128'
tickets:
- 128-012
---

# STATUS projection belongs with Comms (or a projector), not the main loop

The main loop hand-assembles the `Comms::Status` struct field by field every
cycle, at [robot_loop.cpp:566-583](src/firm/app/robot_loop.cpp#L566-L583)
(the stakeholder referred to it as "main.cpp" — the actual site is
`RobotLoop::tick()` in `src/firm/app/robot_loop.cpp`):

```cpp
Comms::Status status;
status.ready = true;  // past boot(): the loop is dispatching commands
status.active = state_.command.moveActive;
status.wheelLeftConnected = state_.wheelLeft.connected;
status.wheelRightConnected = state_.wheelRight.connected;
status.otosPresent = state_.otos.present;
status.wedged = state_.health.wedgeLatch;
status.flags = tlm_.flags();
status.tlmMode = static_cast<uint8_t>(tlm_.mode());
comms_.setStatus(status);
```

**This is a projection of `RobotState` onto the STATUS reply shape — a
Comms concern, not a loop-schedule concern.** The loop should say *when*
the projection is refreshed, not enumerate its fields. Eight assignments of
`state_.<subsystem>.<field>` into a wire-reply struct is exactly the kind of
code that silently rots when a field is added to `RobotState` or to
`Comms::Status`: nothing links the two definitions, so a new field is simply
forgotten here and STATUS quietly reports a stale default. Compare
`Telemetry`, which owns its own `update(state_)` projection rather than
having the loop fill a frame field-by-field — STATUS is the same shape of
problem solved inconsistently.

## Proposed change

Give Comms the projection, mirroring `Telemetry::update(const RobotState&)`:

```cpp
tlm_.update(state_);
comms_.updateStatus(state_, tlm_);   // or (state_, tlm_.flags(), tlm_.mode())
```

Points to settle during design:

- **The two telemetry-sourced fields.** `flags` and `tlmMode` come from
  `Telemetry`, not from `state_`. Either pass them in (keeping Comms free of
  a Telemetry dependency), or publish them into `RobotState` during
  `Telemetry::update()` so the projection reads one source. The latter is
  cleaner and matches the "one blackboard" pattern the rest of the loop
  already uses — worth preferring unless it costs state footprint that
  matters.
- **`ready = true`.** The only field that is genuinely loop knowledge ("we
  are past `boot()`"). It belongs in `state_` (e.g. a lifecycle/health flag
  set once at the end of `boot()`), not hard-coded at the projection site.
- **Ordering constraints, both of which must survive the move.** The
  projection must run *after* the TLM mode switch so a same-cycle mode
  change is reflected, and *before*
  [`comms_.sendTlmReply(tlmAction)`](src/firm/app/robot_loop.cpp#L589),
  which reports the mode just applied. It must also stay unconditional —
  it runs even when the idle gate suppressed the telemetry frame, because
  answering STATUS on a parked robot is the case STATUS exists for.
- **Scope note.** This and
  [[tlm-mode-switch-belongs-in-telemetry-not-the-loop]] are adjacent blocks
  in the same function and touch the same ordering constraint; doing them
  together is likely cheaper than doing either alone.

## Test impact

Whatever currently asserts the STATUS reply contents (see
`src/tests/sim/unit/app_comms_harness.cpp`) should move to driving the new
projection entry point directly with a synthesized `RobotState`, rather than
going through a loop tick — that is the win, since today the field mapping
is only reachable via the full loop.
