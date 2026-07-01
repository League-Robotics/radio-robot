# Firmware Command Reference

A lookup table for every wire-protocol command the robot firmware understands:
what it does, where it is registered, and the exact `file:line` of its parse and
handler functions. Use this to jump straight to the `G` command, the `OL`
command, etc.

> **Keep this current.** When you add, rename, or move a command, update the row
> here. The line numbers drift as files change — if a link is off by a few
> lines, search for the function name (e.g. `handleG`) rather than trusting the
> number.

## How dispatch works

A command line (from radio or serial) flows through:

1. [`CommandProcessor::process`](commands/CommandProcessor.cpp#L336) — tokenizes the
   line, uppercases the verb, peels off a trailing `#id` correlation tag, and
   splits `key=value` pairs.
2. [`CommandProcessor::dispatchTable`](commands/CommandProcessor.cpp#L80) — scans the
   command table for the descriptor whose `prefix` is the **longest token match**
   (so `GET VEL` beats `GET`, and `DBG LOOP RESET` beats `DBG LOOP`), then calls
   its `parseFn` and `handlerFn`.

The table itself is assembled once in
[`Robot::buildCommandTable`](commands/SystemCommands.cpp#L1073). Each subsystem implements
the [`Commandable`](types/CommandTypes.h#L119) interface and contributes its own
commands via `getCommands()`; `buildCommandTable` concatenates them and appends
the system commands. So to find a command you (a) find which subsystem owns it,
then (b) open that subsystem's `.cpp`.

Descriptors are built with [`makeCmd`](types/CommandTypes.h#L129). The **HW**
column below is the `CMD_ACCESS_HARDWARE` flag (touches motors/sensors/GPIO/I2C);
**reply** is `ForceReply::SERIAL` for commands that always answer over the USB
serial link instead of the requesting channel.

---

## Motion — [`source/commands/MotionCommands.cpp`](commands/MotionCommands.cpp)

The motion command parse/handle functions live in
[`MotionCommands.cpp`](commands/MotionCommands.cpp); the motion logic
they drive (`beginGoTo`, pursuit, ramps) lives in
[`Planner.cpp`](superstructure/Planner.cpp) and
[`PlannerBegin.cpp`](control/PlannerBegin.cpp). Registered in
[`getMotionCommands()`](commands/MotionCommands.cpp#L1335).

**stop= clauses (sprint 052).** The open-loop motion verbs `VW`, `T`, `D`,
`R`, and `TURN` accept one or more `stop=<kind>:<args>` clauses appended as
`key=value` pairs. Each clause adds a stop condition (OR-combined); up to 4
clauses per command. The first condition to fire ends the drive and adds a
`reason=<token>` field to the `EVT done` line. `sensor=<ch>:<op>:<thr>` is
accepted as a back-compat alias for `stop=sensor:<ch>:<op>:<thr>`. See
[`docs/protocol-v2.md` §10](../docs/protocol-v2.md) for the full grammar and
reason-token table.

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `S` | set wheel speeds (mm/s) | | [`handleS`](commands/MotionCommands.cpp#L223) | [`parseS`](commands/MotionCommands.cpp#L196) |
| `T` | timed drive (ms); accepts `stop=` clauses | | [`handleT`](commands/MotionCommands.cpp#L305) | [`parseT`](commands/MotionCommands.cpp#L271) |
| `D` | distance drive (mm); accepts `stop=` clauses | | [`handleD`](commands/MotionCommands.cpp#L415) | [`parseD`](commands/MotionCommands.cpp#L382) |
| `G` | go-to a **robot-relative** point — `G <x> <y> <speed>`: `x` = forward mm, `y` = left mm (`+` = left / CCW), `speed` = 1–1000 mm/s (`x`,`y` clamped to ±10000). Arcs to the (forward, left) target in the robot frame; replies `OK goto`, then `EVT done G` on arrival. | | [`handleG`](commands/MotionCommands.cpp#L518) | [`parseG`](commands/MotionCommands.cpp#L486) |
| `R` | arc drive: forward speed + turn radius mm — `R <speed> <radius>` (`beginArc`, replies `OK arc`); accepts `stop=` clauses | | [`handleR`](commands/MotionCommands.cpp#L585) | [`parseR`](commands/MotionCommands.cpp#L558) |
| `TURN` | spin in place to absolute heading, centidegrees — `TURN <cdeg> [eps=]` (`beginTurn`, replies `OK turn`); accepts `stop=` clauses | | [`handleTURN`](commands/MotionCommands.cpp#L665) | [`parseTURN`](commands/MotionCommands.cpp#L628) |
| `VW` | velocity + angular vel (unicycle); accepts `stop=` clauses | ✓ | [`handleVW`](commands/MotionCommands.cpp#L825) | [`parseVW`](commands/MotionCommands.cpp#L803) |
| `_VW` | raw velocity, no ramp (seed+set BVC now) | ✓ | [`handle_VW`](commands/MotionCommands.cpp#L1052) | [`parse_VW`](commands/MotionCommands.cpp#L1030) |
| `X` | stop immediately (`X soft` = ramp) | ✓ | [`handleX`](commands/MotionCommands.cpp#L1098) | [`parseX`](commands/MotionCommands.cpp#L1076) |
| `STOP` | stop with deceleration | ✓ | [`handleSTOP`](commands/MotionCommands.cpp#L1118) | [`parseNoArgs`](commands/MotionCommands.cpp#L1066) |

## Odometry / OTOS — [`source/control/Odometry.cpp`](control/Odometry.cpp)

Registered in [`getCommands()`](control/Odometry.cpp#L479).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `OI` | OTOS init: re-initialise sensor signal processing | ✓ | [`handleOI`](control/Odometry.cpp#L339) | [`parseOI`](control/Odometry.cpp#L262) |
| `OZ` | OTOS zero: set position to 0,0,0 | ✓ | [`handleOZ`](control/Odometry.cpp#L354) | [`parseOZ`](control/Odometry.cpp#L269) |
| `OR` | OTOS reset: reset tracking / Kalman filters (reply `OK or`) | ✓ | [`handleOR`](control/Odometry.cpp#L369) | [`parseOR`](control/Odometry.cpp#L276) |
| `OP` | OTOS position: report cached pose `x=<mm> y=<mm> h=<mrad>` (no HW access) | | [`handleOP`](control/Odometry.cpp#L394) | [`parseOP`](control/Odometry.cpp#L283) |
| `OV` | OTOS **set** position: `OV <x> <y> <h>` writes pose via `setPositionRaw` (reply `OK setpos`) | ✓ | [`handleOV`](control/Odometry.cpp#L418) | [`parseOV`](control/Odometry.cpp#L290) |
| `OL` | OTOS linear scalar calibration (`OL [val]` set/read) | ✓ | [`handleOL`](control/Odometry.cpp#L438) | [`parseOL`](control/Odometry.cpp#L306) |
| `OA` | OTOS angular scalar calibration (`OA [val]` set/read) | ✓ | [`handleOA`](control/Odometry.cpp#L458) | [`parseOA`](control/Odometry.cpp#L321) |

## GPIO ports — [`source/control/PortController.cpp`](control/PortController.cpp)

Registered in [`getCommands()`](control/PortController.cpp#L143).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `P` | digital pin read/write | ✓ | [`handleP`](control/PortController.cpp#L94) | [`parseP`](control/PortController.cpp#L21) |
| `PA` | analog pin read/write | ✓ | [`handlePA`](control/PortController.cpp#L115) | [`parsePA`](control/PortController.cpp#L51) |

## Servo / gripper — [`source/control/ServoController.cpp`](control/ServoController.cpp)

Registered in [`getCommands()`](control/ServoController.cpp#L76).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `GRIP` | set/query gripper angle (0–180 deg) | ✓ | [`handleGrip`](control/ServoController.cpp#L48) | [`parseGrip`](control/ServoController.cpp#L19) |

## Debug / I2C — [`source/commands/DebugCommands.cpp`](commands/DebugCommands.cpp)

Registered in [`getCommands()`](commands/DebugCommands.cpp#L706). All reply over
serial (`ForceReply::SERIAL`). The `DBG OTOS …` commands compile only in
bench/host builds (`BENCH_OTOS_ENABLED` or `HOST_BUILD`).

| Cmd | Meaning | HW | Handler | Parse |
|-----|---------|----|---------|-------|
| `DBG LOOP RESET` | reset loop-stats counters | | [`handleDbgLoopReset`](commands/DebugCommands.cpp#L56) | [`parseDbgLoopReset`](commands/DebugCommands.cpp#L47) |
| `DBG LOOP` | report loop timing stats | | [`handleDbgLoop`](commands/DebugCommands.cpp#L92) | [`parseDbgLoop`](commands/DebugCommands.cpp#L71) |
| `DBG I2CLOG` | dump I2C transaction log | ✓ | [`handleDbgI2clog`](commands/DebugCommands.cpp#L127) | [`parseDbgI2clog`](commands/DebugCommands.cpp#L107) |
| `DBG I2C` | report I2C bus error counts | ✓ | [`handleDbgI2c`](commands/DebugCommands.cpp#L182) | [`parseDbgI2c`](commands/DebugCommands.cpp#L162) |
| `DBG IRQGUARD` | enable/disable IRQ guard | | [`handleDbgIrqguard`](commands/DebugCommands.cpp#L262) | [`parseDbgIrqguard`](commands/DebugCommands.cpp#L245) |
| `DBG WEDGE` | run encoder wedge self-check | ✓ | [`handleDbgWedge`](commands/DebugCommands.cpp#L311) | [`parseDbgWedge`](commands/DebugCommands.cpp#L293) |
| `DBG OTOS BENCH` | enable/disable bench OTOS sim + set noise (`DBG OTOS BENCH 1\|0`) — bench/host builds only | ✓ | [`handleDbgOtosBench`](commands/DebugCommands.cpp#L420) | [`parseDbgOtosBench`](commands/DebugCommands.cpp#L360) |
| `DBG OTOS` | query ideal/otos/fused pose — bench/host builds only | | [`handleDbgOtos`](commands/DebugCommands.cpp#L488) | [`parseDbgOtos`](commands/DebugCommands.cpp#L479) |
| `I2CW` | raw I2C write (addr reg data…) | ✓ | [`handleI2cw`](commands/DebugCommands.cpp#L583) | [`parseI2cw`](commands/DebugCommands.cpp#L558) |
| `I2CR` | raw I2C read (addr reg count) | ✓ | [`handleI2cr`](commands/DebugCommands.cpp#L650) | [`parseI2cr`](commands/DebugCommands.cpp#L620) |

## System — [`source/commands/SystemCommands.cpp`](commands/SystemCommands.cpp)

System command handlers and `Robot::buildCommandTable` were moved out of
`Robot.cpp` into [`SystemCommands.cpp`](commands/SystemCommands.cpp) (A3 refactor).
[`buildCommandTable`](commands/SystemCommands.cpp#L1073) concatenates every
subsystem's `getCommands()` and appends the system commands below. `GET`, `SET`,
and `GET VEL` live in [`source/commands/ConfigCommands.cpp`](commands/ConfigCommands.cpp).

| Cmd | Meaning | Handler | Parse |
|-----|---------|---------|-------|
| `HELLO` | identify firmware + version | [`handleHello`](commands/SystemCommands.cpp#L72) | [`parseHello`](commands/SystemCommands.cpp#L66) |
| `PING` | liveness check | [`handlePing`](commands/SystemCommands.cpp#L96) | [`parsePing`](commands/SystemCommands.cpp#L90) |
| `ECHO` | echo tokens back | [`handleEcho`](commands/SystemCommands.cpp#L133) | [`parseEcho`](commands/SystemCommands.cpp#L113) |
| `ID` | report robot identity string | [`handleId`](commands/SystemCommands.cpp#L164) | [`parseId`](commands/SystemCommands.cpp#L158) |
| `VER` | report firmware version | [`handleVer`](commands/SystemCommands.cpp#L215) | [`parseVer`](commands/SystemCommands.cpp#L209) |
| `HELP` | list available commands | [`handleHelp`](commands/SystemCommands.cpp#L236) | [`parseHelp`](commands/SystemCommands.cpp#L230) |
| `SNAP` | emit one TLM frame on demand | [`handleSnap`](commands/SystemCommands.cpp#L272) | [`parseSnap`](commands/SystemCommands.cpp#L266) |
| `ZERO` | zero encoders/pose/halt-baselines | [`handleZero`](commands/SystemCommands.cpp#L328) | [`parseZero`](commands/SystemCommands.cpp#L287) |
| `HALT` | named stop-condition registry | [`handleHalt`](commands/SystemCommands.cpp#L784) | [`parseHalt`](commands/SystemCommands.cpp#L749) |
| `STREAM` | start/stop periodic TLM stream | [`handleStream`](commands/SystemCommands.cpp#L405) | [`parseStream`](commands/SystemCommands.cpp#L383) |
| `RF` | set radio channel | [`handleRf`](commands/SystemCommands.cpp#L525) | [`parseRf`](commands/SystemCommands.cpp#L508) |
| `+` | keepalive: reset watchdog | [`handleKeepalive`](commands/SystemCommands.cpp#L587) | [`parseKeepalive`](commands/SystemCommands.cpp#L581) |
| `SAFE [off\|on [ms]]` | enable/disable safety-stop watchdog; optional timeout ms; `SAFE` alone queries. Reply: `OK safety on\|off timeout=<ms>` | [`handleSafe`](commands/SystemCommands.cpp#L634) | [`parseSafe`](commands/SystemCommands.cpp#L615) |
| `SI` | set odometry world pose — `SI <x_mm> <y_mm> <h_cdeg>` | [`handleSI`](commands/SystemCommands.cpp#L712) | [`parseSI`](commands/SystemCommands.cpp#L695) |
| `GET VEL` | get velocity PID params | [`handleGetVel`](commands/ConfigCommands.cpp#L46) | [`parseGetVel`](commands/ConfigCommands.cpp#L40) |
| `GET` | get config value by key | [`handleGet`](robot/ConfigRegistry.cpp#L170) | [`parseGet`](commands/ConfigCommands.cpp#L63) |
| `SET` | set config value by key | [`handleSet`](robot/ConfigRegistry.cpp#L396) | [`parseSet`](commands/ConfigCommands.cpp#L86) |

### `GET` / `SET` keys

`GET`/`SET` don't have one handler per key — they look the key up in the config
registry table [`kRegistry[]`](robot/ConfigRegistry.cpp#L23), which maps each
friendly key name to a `RobotConfig` field by byte offset. To find or add a
tunable (e.g. `alphaYaw`, `yawRateMax`, PID gains), edit that table. Defaults
live in [`source/robot/DefaultConfig.cpp`](robot/DefaultConfig.cpp).

---

## Adding a command

1. Pick the owning subsystem (or add a new `Commandable`).
2. Write a `parseFn` (validates tokens → `ArgList`) and a `handlerFn` (does the
   work, calls `replyFn`).
3. Add a `makeCmd(...)` line to that subsystem's `getCommands()`.
4. If it's a system-level command, add it to
   [`Robot::buildCommandTable`](commands/SystemCommands.cpp#L1073) instead — and remember
   longer prefixes must be registered so the longest-match scan reaches them.
5. Add a row to the right table above.
