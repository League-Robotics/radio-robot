---
status: done
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-15
severity: low
sprint: '066'
tickets:
- 066-003
---

# Small cleanups from the 2026-07-01 full-codebase review (CR-15 batch)

## Items

One maintenance ticket; each item is small and independent.

1. **Wrap `PhysicsWorld._truePoseH`** — accumulates unbounded
   ([PhysicsWorld.cpp:100](../../source/hal/sim/PhysicsWorld.cpp)); wrap to
   (−π, π] or document that consumers must wrap (SimOdometer wraps its own
   copy, creating a representation mismatch).
2. **Retire or fix `probe_devices()`** — still sends the retired `>PING`
   relay-prefix protocol
   ([serial_conn.py:918-946](../../host/robot_radio/io/serial_conn.py));
   cannot reach a robot through current relay firmware.
3. **Surface `relay_info` from `connect()`** — collected then dropped
   ([serial_conn.py:334,352](../../host/robot_radio/io/serial_conn.py));
   the promised operator-visible channel/group mismatch logging never reaches
   the result dict.
4. **`SimTransport.connect()` premature `_connected = True`**
   ([transport.py:644-652](../../host/robot_radio/testgui/transport.py)) —
   set connected only after the tick thread successfully creates the Sim, or
   surface the failure to early `command()` callers.
5. **traces.py encoder integration uses post-increment heading** (not
   midpoint) ([traces.py:361-363](../../host/robot_radio/testgui/traces.py))
   — small systematic display drift on turns; fix or comment as intentional.
6. **Duplicate DISTANCE/TIME stops on queued D/T** — resolved as part of the
   stop-overflow issue; verify no wasted stop slots remain afterwards.
7. **Move `rgbToHSV` out of StopCondition.cpp** (existing FIXME at
   [StopCondition.cpp:27](../../source/control/StopCondition.cpp)).
8. **KeyboardDriver multi-key release** — releasing one arrow while another
   is held sends STOP and drops the held command
   ([drive.py:263-288](../../host/robot_radio/testgui/drive.py)); track held
   keys and fall back to the remaining one.
