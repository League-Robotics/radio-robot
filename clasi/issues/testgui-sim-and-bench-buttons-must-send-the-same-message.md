---
status: pending
priority: high
---

# A button's message must not depend on which transport is underneath it

Stakeholder, 2026-07-31, emphatically: *"The rule is that SIM and bench work the
same way. You don't get to have different buttons. That's insane. The button on
SIM sends the same message as the button. There should be no question that those
buttons send a message that does not depend on the transport."*

## The defect

`SimTransport` and `_HardwareTransport` each carried their **own** copy of the
unmanaged-drive logic. The two drifted: the Sim button and the bench button with
the same label sent different messages. This is the failure mode the Sim exists
to prevent — a Sim that runs a different code path cannot falsify anything about
the robot.

## What was built (and is being abandoned with the rest)

- One module-level `run_unmanaged_distance_drive(transport, driver, distance,
  direction, label, log)` in `testgui/transport.py`, called by **both**
  transports. The `driver` only needs the three primitives both `NezhaProtocol`
  and `SimLoop` expose: `wheels(l, r, ms)`, a frame read, and `estop()`.
- `SimLoop.wheels()` added with an **identical envelope** to the hardware path,
  so the shared routine has one implementation to call.

## The general rule this is an instance of

Any behaviour that differs between Sim and bench must be a *declared, documented
difference with a stated reason*, not an accident of two code paths. The
composition roots have the same problem at a larger scale —
[[unify-sim-and-robot-composition-roots]].

Prior art in the same session: unifying the two roots moved tour closure from
45.4 mm to 2.2 mm and heading error from 7.9 deg to 0.7 deg. The divergence was
not cosmetic.

## Acceptance

- A test that asserts the Sim and hardware transports emit the **same command
  sequence** for the same button press, at the message level.
- Grep gate: no drive/motion logic defined inside either transport class.
