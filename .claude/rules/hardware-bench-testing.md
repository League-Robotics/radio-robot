# Hardware Bench Testing

The robot is **connected and mounted on a stand**. Its wheels are off the ground,
so it **cannot drive away** — it is safe to power the motors and spin the wheels
freely during verification. Use the real hardware to confirm changes, not just
unit tests.

## Standing verification gate

Every firmware sprint that touches the HAL, motor control, sensing, or the
command protocol must, as part of its acceptance, **deploy to the robot and
exercise it on the stand**:

1. **Sensors are alive.** Talk to every sensor and confirm it responds with
   plausible, changing values: encoders (motor controller), OTOS
   (position/velocity), line sensor (4 channels), color sensor (RGBC), and the
   digital/analog ports.
2. **Wheels drive and encoders run.** Command the wheels (both directions) and
   confirm the **encoders increment** in the expected direction and roughly in
   proportion to commanded speed. Because the robot is on the stand, drive freely.
3. **Round-trip over the real link.** Confirm commands and replies work over the
   actual transport (serial at the bench; radio relay when testing the relay
   path).

A sprint is not "done" on tests alone — it must be seen working on the stand.

## How to deploy and drive

Build + flash with the project's deploy tool (the robot is a flashable probe):

```bash
mbdeploy probe          # discover/refresh the connected device registry
mbdeploy deploy --build # build firmware and flash the robot
```

Then open the serial port (or drive through the radio relay) and issue commands.
The current command surface is **protocol v4** — see
[docs/protocol-v4.md](../../docs/protocol-v4.md) (a two-verb text safety rump,
`HELLO`/`PING`; a binary command plane with exactly three arms, `move`/
`config`/`stop`; and an always-on binary telemetry push) — dispatched by
[src/firm/app/robot_loop.cpp](../../src/firm/app/robot_loop.cpp)'s
`processMessage()`. There is no bare-command REPL shape any more: every
motion is a bounded `Move` sent through `NezhaProtocol`
([src/host/robot_radio/robot/protocol.py](../../src/host/robot_radio/robot/protocol.py)),
not a hand-typed wire line — use `rogo repl` or one of the bench scripts
below rather than typing verbs directly at the serial port.

### Quick smoke sequence (protocol v4 / MOVE-era)

Drive this either interactively via `rogo repl` (the `rogo` console script,
`src/host/robot_radio/io/cli.py`) or by running
[src/tests/bench/twist_drive.py](../../src/tests/bench/twist_drive.py) for
steps 1-4 in one shot:

```bash
uv run python src/tests/bench/twist_drive.py --port /dev/cu.usbmodem2121102
```

| Step | Call (`NezhaProtocol`) | Expect |
|---|---|---|
| Identify | connect (sends `HELLO` on the text rump) | `DEVICE:NEZHA2:<name>:microbit:<serial>` banner |
| Liveness | `PING` (text rump) | `OK pong t=<ms>` |
| Config push | `config(**{"pid.kp": ...})` / `otos_config(...)` | ack rides the next `Telemetry` frame's ack slot (`ack_corr` == the enqueue `corr_id`, `ack_err == 0`) |
| Drive (on stand) | `move_twist(v_x=150, stop_time=..., timeout=...)` | enqueue ack, then telemetry frames with `enc_left`/`enc_right`/`pose` climbing while the `Move` runs |
| Completion | *(no separate call — the same `Move` ends on its own)* | a later frame's ack slot carries `ack_corr == Move.id` (the completion ack, `ack_err` always 0 — timeout vs. stop-condition is `flags` bit 15, not `ack_err`) |
| Stop | `stop()` | enqueue ack; `flags` bit 2 (`kFlagActive`) drops, encoders hold |
| Odometry / OTOS | read `Telemetry.pose` / `Telemetry.otos` off any frame | `pose` always present; `otos` valid when `flags` bit 0 is set |
| Line / color | read `Telemetry.line` / `Telemetry.color` off any frame | valid when `flags` bits 13/14 are set — 4 plausible channel values each |

For the fuller MOVE-protocol surface (distance/angle stop conditions, the
`wheels` velocity variant, chaining, `replace=True` preemption, the 5-deep
`ERR_FULL` queue limit, the no-deadman empty-queue drain, the `timeout`
safety backstop, `STOP` mid-motion, and a `CONFIG` patch arriving mid-`Move`),
run the full bench-gate script:

```bash
uv run python src/tests/bench/move_protocol_bench.py --port /dev/cu.usbmodem2121102
```

which prints a PASS/FAIL line per scenario. See
[src/tests/bench/](../../src/tests/bench/) for the rest of the bench-script
catalog (`tlm_log.py` for a flat CSV telemetry capture,
`move_accuracy_bench.py`/`turn_prediction_capture.py` for accuracy
characterization, `otos_drift.py`/`velocity_step_response.py` for sensor/PID
characterization).

## Safety notes

- On the stand the wheels spin free; still avoid loose clothing/fingers near the
  drivetrain and the gripper.
- A locked/protected nRF recovers via `mbdeploy`'s automatic mass-erase on a
  failed flash (no manual step needed).
