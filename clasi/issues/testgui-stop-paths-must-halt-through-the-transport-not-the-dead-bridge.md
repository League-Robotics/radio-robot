---
status: pending
---

# testgui STOP button/halt paths: add Transport.halt() calling estop(); stop routing through the dead bridge

**Source:** code review 2026-07-30, `05-testgui-testkit.md` CRITICAL §1.
**Priority:** P0 — the GUI's STOP button does not stop a real robot, and logs
success anyway.
**Goal served:** the fix is a *simplification*: one abstract `halt()` on the
`Transport` ABC replaces a string verb travelling through a translation layer
that is permanently dead. After this, "does STOP work?" is answerable by
reading three short methods instead of tracing a string through
`translate_command()`.

## What is wrong

- `testgui/operations.py:557-614` (`on_stop`) sends the string `"STOP"`
  through `transport.command()`. On `SerialTransport`/`RelayTransport` that
  routes into `binary_bridge.translate_command()`, which is an unconditional
  dead stub — it returns `ERR unavailable ...` without touching the wire.
  `on_stop()` discards the reply and logs `"[INFO] STOP sent"`. The robot
  keeps driving.
- `__main__.py:2117-2122` (`_safe_stop`) and `:2599` (`_set_origin`) do the
  same.
- Even if the stub worked, `"STOP"` maps to the PLANNED stop.
  `grep -rn estop src/host/robot_radio/testgui/*.py` returns nothing — the
  correct verb is never called anywhere in the package.
- The gap is invisible to tests because `test_gui_button_acceptance.py` is
  Sim-only, and `SimLoop.stop()` internally sends `ESTOP` on the sim ABI —
  Sim's STOP genuinely halts, hiding that hardware's does nothing.

## What to do

1. Add `halt()` to the `Transport` ABC (`testgui/transport.py`), preserving
   the package's own no-branching-on-backend-type discipline:

```python
class Transport(ABC):
    @abstractmethod
    def halt(self) -> None:
        """Halt the robot NOW (estop semantics -- clears the active Move AND
        the planner queue). Raises on failure: callers must surface a failed
        halt, never log success on faith."""


class _HardwareTransport(Transport):
    def halt(self) -> None:
        proto = self.protocol
        if proto is None:
            raise ConnectionError("halt: not connected")
        proto.estop()   # the halt-now verb; stop() is the planned stop


class SimTransport(Transport):
    def halt(self) -> None:
        sim = self.protocol
        if sim is None:
            raise ConnectionError("halt: not connected")
        sim.stop()      # SimLoop.stop() already sends ESTOP on the sim ABI
```

2. Rewire `on_stop()` step 2, `_safe_stop()`, and `_set_origin()`'s
   pre-teleport halt onto it, checking the outcome:

```python
# 2. Halt motors NOW -- estop semantics, direct call, never the
#    translate_command() path (permanently dead for STOP since 104-002).
try:
    transport.halt()
    self._log("[INFO] estop sent -- motion halted")
except Exception as exc:
    self._log(f"[ERROR] HALT FAILED -- ROBOT MAY STILL BE MOVING: {exc}")
```

3. `STREAM 0` in `on_stop()` step 3 takes the same direct-call treatment
   (`transport.protocol.tlmOff()` on hardware) or an honest "not available"
   log — see `delete-binary-bridge-dead-half-and-direct-call-the-survivors.md`.

4. Add the acceptance test the current suite is missing: a
   hardware-transport-shaped test (mock `NezhaProtocol`) asserting the STOP
   button results in exactly one `estop()` call, and that a raising `estop()`
   produces an `[ERROR]` log, not `[INFO]`. The Sim-only suite is what let
   this ship; per the project's own rule (GUI work needs headless button
   acceptance), the new test belongs in `test_gui_button_acceptance.py`'s
   harness.

## Acceptance

- `grep -rn "command(\"STOP\"\|send(\"STOP\"" src/host/robot_radio/testgui/`
  returns nothing.
- `grep -rn "estop" src/host/robot_radio/testgui/` shows the ABC + backends +
  call sites.
- Bench check (robot on stand): click STOP during a Managed `D` move —
  wheels halt within one cycle; disconnect the serial link and click STOP —
  the log shows `HALT FAILED ... MAY STILL BE MOVING`, not success.
