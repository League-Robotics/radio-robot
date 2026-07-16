# src/ — unified source root

All of the project's source trees live under this directory (reorganized
2026-07-16; formerly scattered at the repo root as `source/`, `host/`,
`tests/`, `scripts/`, `utils/`, `protos/`, `archive/`, `vendor`,
`libraries/`, and `tests/_infra/sim/`).

The build **entry points stay at the repo root** — `build.py`,
`CMakeLists.txt`, `codal.json`, `justfile`, `pyproject.toml` — because the
CODAL build runs CMake against the repo root and `uv` reads the root
`pyproject.toml`. They all point into `src/`.

| Directory | What it is |
|---|---|
| `firm/` | micro:bit V2 firmware — the CODAL application (`codal.json`'s `application`). Subsystems: `app/` (per-cycle robot loop, telemetry, comms, drive, odometry, deadman, preamble), `com/` (radio + serial transports), `config/` (generated boot calibration), `devices/` (fiber-owned I2C device subsystem: bus/clock interfaces, Nezha motor, OTOS, line/color sensors, control utilities), `kinematics/` (pure diff-drive math), `messages/` (generated wire-protocol structs + codec), `types/` (protocol/version constants), `main.cpp`. |
| `host/` | Host-side Python. `robot_radio/` is the importable package (shipped by the root `pyproject.toml` wheel config): transports, wire protocol, planner/nav, sensors, calibration, the `rogo` CLI, the MCP server, and the PySide6 Test GUI (`just testgui`). |
| `sim/` | The host-build simulator infrastructure: `SimPlant` (a real `Devices::I2CBus` that answers actual firmware wire bytes), `SimHarness` (composes the real `firm/` App graph with sim leaves), `SimClock`, the `sim_ctypes.cpp` C ABI, and the CMake project that compiles `firm/` with `-DHOST_BUILD=1` into `sim/build/libfirmware_host.{dylib,so}` (`just build-sim`). Loaded at runtime by `host/robot_radio/io/sim_loop.py` and the Test GUI's Sim mode. The deterministic physics models it links live in `tests/sim/plant/`. |
| `tests/` | Test domains: `sim/` (host-build firmware tests: `unit/` harnesses, `system/` scenarios, `plant/` physics models, `support/` wire doubles), `unit/` (pure-Python host tests), `testgui/` (headless GUI tests) — these three are pytest-collected (see root `pyproject.toml` `testpaths`). `bench/` and `playfield/` are HITL CLI tools against real hardware, not pytest. `notebooks/` holds analysis notebooks. |
| `scripts/` | Codegen + lint tooling run by `build.py`: `gen_messages.py` (protos → `firm/messages/*.h`), `gen_pb2.py` (protos → `host/robot_radio/robot/pb2/`), `gen_boot_config.py` (active robot JSON → `firm/config/boot_config.cpp`), `gen_version.py` (pyproject version → `firm/types/version_generated.h`), `check_config_sync.py` (CI lint). |
| `protos/` | proto3 message schemas — the single source of truth for the wire protocol; consumed by `scripts/gen_messages.py` (C++) and `scripts/gen_pb2.py` (Python). The device never sees protobuf; these generate POD structs + codec. |
| `utils/` | CODAL/yotta build machinery (vendored from microbit-v2-samples): `cmake/` modules + toolchains used by the root `CMakeLists.txt`, `targets.json`, hex/uf2 helpers, `python/codal_utils.py` imported by `build.py`. |
| `libraries/` | CODAL dependency checkouts (codal-core, codal-microbit-v2, codal-nrf52, …). **Gitignored** — fetched/updated by the build (`python build.py`). Safe to delete; it will be re-cloned. |
| `vendor/` | Symlink to external vendored projects (PythonRobotics etc.). Never pytest-collected. |
| `archive/` | Parked/retired trees kept for reference: `source_old/`, `tests_old/`, `wedgelab/`, `host_scripts/` (the old loose `host/calibrate_*.py`), old hex images. Nothing here builds or runs. |

Quick commands (from the repo root):

```bash
just build          # firmware + sim lib (MICROBIT.hex via build.py)
just build-sim      # sim lib only -> src/sim/build/libfirmware_host
just testgui        # PySide6 cockpit (uv sync --group gui first)
uv run python -m pytest   # sim + unit + testgui suites
```
