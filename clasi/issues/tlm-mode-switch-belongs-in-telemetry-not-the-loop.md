---
status: pending
---

# TLM mode-action dispatch belongs in Telemetry, not the main loop

The `TlmAction` → `setMode()` switch currently sits inline in the main
loop body, at [robot_loop.cpp:541-564](src/firm/app/robot_loop.cpp#L541-L564)
(the stakeholder referred to it as "main.cpp" — the actual site is
`RobotLoop::tick()` in `src/firm/app/robot_loop.cpp`):

```cpp
const Comms::TlmAction tlmAction = comms_.takeTlmAction();
switch (tlmAction) {
  case Comms::TlmAction::kSetOff:  tlm_.setMode(TlmMode::kOff);  break;
  case Comms::TlmAction::kSetAuto: tlm_.setMode(TlmMode::kAuto); break;
  case Comms::TlmAction::kSetOn:   tlm_.setMode(TlmMode::kOn);   break;
  case Comms::TlmAction::kNone:
  case Comms::TlmAction::kFrame:
  case Comms::TlmAction::kUnrecognized:
  default:
    break;  // no mode change
}
```

**This is telemetry policy, not loop schedule.** The loop's job is to say
*when* telemetry runs; deciding what a `TLM:ON` token means to the telemetry
emitter is the `Telemetry` class's own business. Every arm of the switch
names a `Telemetry` concept (`TlmMode`, `setMode`), and the `default`/no-op
arms exist purely to satisfy the enum — none of it is loop concern. The
comment block above it is longer than the code and is entirely an
explanation of why telemetry policy is being executed somewhere else.

## Proposed change

Move the mapping into `Telemetry` — e.g. a
`Telemetry::applyAction(Comms::TlmAction)` (or equivalent) that owns both
the mode change and the "is this a force-a-frame request" answer, so the
loop shrinks to roughly:

```cpp
tlm_.update(state_);
tlm_.applyAction(comms_.takeTlmAction());
tlm_.emit(state_.time.cycleStart);
```

Points to settle during design:

- **Where `TlmAction` lives.** It is currently `Comms::TlmAction`, so
  `Telemetry` taking it as a parameter makes Telemetry depend on Comms.
  Either move the enum to a shared/telemetry-owned header, or have
  Telemetry expose the three mode setters plus one `requestFrame()` and let
  Comms call them — decide which direction the dependency should run.
  Wire-text parsing stays in Comms either way; that separation is correct
  and must survive the move.
- **The `force` flag on `emit()`.** `kFrame` currently reaches
  [robot_loop.cpp:564](src/firm/app/robot_loop.cpp#L564) as
  `force=(tlmAction == kFrame)`. If Telemetry absorbs the action it can
  latch a pending one-shot request internally and `emit()` loses the
  parameter — a further simplification of the loop.
- **Ordering constraint.** The `STATUS` refresh immediately below reads
  `tlmMode` *after* the switch, deliberately, so a mode change this cycle
  is already visible ([robot_loop.cpp:566-574](src/firm/app/robot_loop.cpp#L566-L574)).
  Any refactor must preserve that ordering.

## Test impact

`src/tests/sim/unit/app_comms_harness.cpp` (~lines 846-1000) and
`app_telemetry_harness.cpp` (~line 1175) assert on `takeTlmAction()` and
the parse-to-`TlmAction` mapping. The parsing assertions stay valid; the
mode-application assertions should move to the telemetry harness alongside
the code they now cover.
