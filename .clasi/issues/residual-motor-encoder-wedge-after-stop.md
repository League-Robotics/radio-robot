---
status: pending
---

# Residual motor/encoder wedge after stop (follow-up to sprint 014)

## Symptom

Encoders freeze at a constant value (`enc=0,0` or a stuck value) while a drive is
commanded; the wheels may or may not turn. Intermittent. Recovers **only on a
micro:bit reset** (reopening the serial port with a DTR pulse, or a reflash) —
**not** a hardware/Nezha issue: a micro:bit reset that does not re-init the Nezha
clears it, so it is boot-resettable micro:bit-side state.

## Already fixed in sprint 014 (committed, fw 0.20260605.5)

One cause was a stale `static int8_t lastL/lastR` in a debug open-loop block in
`MotorController::controlTick` that skipped the restart `setSpeed` write after a
stop (statics reset only on reboot → "first drive works, second doesn't,
reopen-port fixes"). Also kept (all sound):

- `Motor::setSpeed` write-on-change (don't re-write the Nezha every tick)
- `setSpeed(0)` coasts via `0x60 speed 0`, not the `0x5F` shutdown command
- `MotorController::stop()`/`startDrive()` use cached encoder values
  (`_prevEncL/R`) instead of firing atomic reads from the comms path
- `Robot::controlCollectSplitPhase` skips encoder reads while idle

## Evidence for the residual

- A headless drive/stop cycle test (`tests/bench/drive_raw.py` logic, clean
  `STOP` commands, 7 cycles, real PID + all sensors) **passes** — every cycle
  counts.
- `tests/bench/velocity_chart.py` (GUI) still shows `enc=0` / frozen on some
  runs. Key differences to investigate:
  1. velocity_chart's `SerialConnection` opens with `dtr=False`, so it does
     **not** reset the micro:bit on connect — it cannot recover a robot already
     wedged from a prior run, and the wedge then presents as `enc=0` from the
     very start of the run.
  2. When the matplotlib GIL stalls the keepalive past the firmware S-watchdog,
     a `safety_stop`/`fullStop` fires **mid-stream** (a different trigger than a
     clean idle `STOP`) — the restart after a watchdog-triggered `fullStop`
     during active streaming may be the unfixed path.

## Hypotheses to pursue

- (a) `fullStop`-during-active-streaming leaves bad state the clean-idle-`STOP`
  path does not.
- (b) The real closed-loop PID restart path differs from the open-loop path that
  was validated headless.
- (c) CODAL nRF52 TWIM peripheral recovery — add an explicit TWIM reset/reinit
  on the stop→restart transition.
- (d) velocity_chart should optionally pulse DTR to reset+recover on connect.

## Acceptance

`velocity_chart.py` (or an equivalent GUI/long-run) drives → stops → drives
repeatedly for many cycles with no encoder freeze, including when the watchdog
fires mid-stream, with no micro:bit reset required to recover.
