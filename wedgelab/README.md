# WEDGELAB — standalone Nezha V2 encoder-latch laboratory

A **completely separate CODAL project** from the robot firmware at the repo
root. It has its own `source/`, `codal.json`, `CMakeLists.txt`, `build.py`,
and `build/` output — so building it never touches the main project's build
tree, compile database, or IntelliSense. `libraries/` is a symlink to the
repo root's CODAL library checkouts (read-only source sharing; each project
compiles its own objects into its own `build/`).

## Layout

- `source/` — lab firmware. `main.cpp` is the from-scratch lab (own wire
  functions, patterns, encoder-truth safety net). `Motor.*`, `I2CBus.*`,
  `Config.h`, `MotorSlew.h`, `hal/capability/*` are VERBATIM copies of the
  production motor layer (2026-07-04 bisection: `driver 1` drives through
  production code, `driver 0` through the raw lab functions).
- `wedgelab.py` — host driver (see below).
- `exp/` — experiment scripts (one file = one serial session).
- `out/` — logs (gitignored).

## Build & flash

```sh
cd wedgelab
python3 build.py            # incremental; --clean for full rebuild
mbdeploy deploy robot --hex MICROBIT.hex
```

## Drive

```sh
uv run python wedgelab/wedgelab.py ping
uv run python wedgelab/wedgelab.py run "run legs 30" --label mytest
uv run python wedgelab/wedgelab.py script wedgelab/exp/09-production-driver.txt --label exp09
```

Serial console: `get` | `set <knob> <v>` | `run legs|slam|burst|combo|reset|
native|spin N` | `heal` | `recover` | `stat` | `stop`. Any byte aborts a
running pattern. Every pattern ends with an encoder-verified stop; an idle
watchdog force-stops on any un-commanded motion.

**Port discipline**: nothing else may hold `/dev/cu.usbmodem*` (VS Code
serial monitor auto-reconnects after each flash — disconnect it first).
Bench config 2026-07-04: ports M1/M2 = old (latch-prone) motors, M3/M4 =
fresh.
