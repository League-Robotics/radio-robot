# DeviceBus cutover — hardware validation (2026-07-13)

Real cutover firmware flashed to the robot (main.cpp → `Subsystems::
DeviceBusHardware` → `Devices::DeviceBus` fiber → motors/OTOS; motion-v2
`source/drive/` stack unchanged on top). Robot on the stand. Driven over the
binary protocol (`robot_radio` NezhaProtocol, direct USB).

## Result: PASS — standing bench gate met

- **Firmware alive**: `OK pong` on binary PING; ~47 TLM frames/s streaming.
- **Sensors live through DeviceBus**: `pose=(47,-3,6)`, `otos=(47,-3,6)`
  populated (DeviceBus OTOS handle → DeviceBusOdometer → PoseEstimator),
  encoders reading. Before the cutover these came from NezhaHardware; now they
  come from the fiber-owned DeviceBus, transparently.
- **Wheels drive + encoders increment (both directions + spin)**:
  - FWD 150,150 3s → dEnc=(+327,+320) mm, peak wheel vel ~209 mm/s
  - REV −150,−150 3s → dEnc=(−556,−547) mm
  - SPIN 150,−150 2.5s → dEnc=(+252,−509) mm (counter-rotating)
  Motion flows: source/drive planner/tracker (or the S escape hatch) →
  DeviceBusHardware.motor(i).apply() → Devices::Motor handle.setVelocity →
  DeviceBus fiber PID+armor → real motor; encoders back the same path.
- **Distance MOVE through source/drive executed end-to-end**: `D 200` acked
  (`q=1 rem=0.0`), fused pose advanced ~196 mm (commanded 200). The entire
  chain — motion-v2 plan → tracker → wafer adapter → DeviceBusHardware →
  DeviceBus → motor, and encoder → adapter → PoseEstimator → fused pose —
  works on real hardware.

## Notes / known limitations carried from the cutover ticket (2bc2c800)

- On the stand, fused pose tracks the encoder/OTOS fusion; body doesn't
  translate (wheels off ground), so raw-drive pose deltas are small while the
  distance MOVE's pose follows the encoder-driven belief — the same stand
  behavior sprint 099 documented, not a cutover defect.
- `wedged()`/`wedgeSuspect()`/`hardResetCount()`/`acceleration()` telemetry
  read inert through the adapter (non-virtual `Hal::Motor` base accessors — the
  DeviceBus fiber owns the real armor, but the base can't surface it). The
  DeviceBus's OWN wedge/armor is live; only the msg::/TLM surfacing is a gap.
- Live `SET`-style motor/OTOS reconfigure (CFG/OI/OR/OL/OA) is accepted-inert
  (no live-reconfigure primitive on the handles). Boot config applies fine.
- Color/line sensors are not bridged to the `Hardware` interface (no seam
  existed pre-cutover either — they're not in the motion path). DeviceBus reads
  them internally; a future ticket can surface them if the motion stack needs
  them.

## Encoder reliability (prerequisite, validated earlier same session)

Velocity/encoder feedback verified reliable on the DeviceBus bring-up image
before the cutover: both motors, both directions, steady velocity tracks the
commanded 200 mm/s to ~1 mm/s, over two runs. Fixed two real bugs found on
hardware: the velocity glitch-gate (fiber cycles faster than the ~80ms encoder
refresh → fresh-sample computation) and the motor-2 request/collect starvation.
