set dotenv-load := true

# Robot's USB serial port. Override per-bench with ROBOT_PORT (env or .env);
# confirm with `mbdeploy list`'s ROLE column — ports move across power cycles.
robot_port := env('ROBOT_PORT', '/dev/cu.usbmodem2121102')
pyocd_target := env('PYOCD_TARGET', 'nrf52833')
pyocd_probe := env('PYOCD_PROBE', '')
pyocd_probe_args := if pyocd_probe != '' { '--uid ' + pyocd_probe } else { '' }

default:
    @just --list

setup-macos:
    brew uninstall arm-none-eabi-gcc arm-none-eabi-binutils || true
    brew install --cask gcc-arm-embedded
    brew install uv

link-arm-tools:
    ln -sf /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc /opt/homebrew/bin/arm-none-eabi-gcc
    ln -sf /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-g++ /opt/homebrew/bin/arm-none-eabi-g++
    ln -sf /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-ar /opt/homebrew/bin/arm-none-eabi-ar
    ln -sf /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-ranlib /opt/homebrew/bin/arm-none-eabi-ranlib
    ln -sf /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-objcopy /opt/homebrew/bin/arm-none-eabi-objcopy
    ln -sf /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-size /opt/homebrew/bin/arm-none-eabi-size

uv-sync:
    uv venv
    uv sync

build:
    uv run python3 build.py

build-clean:
    uv run python3 build.py --clean

# Build ONLY the host simulation library (libfirmware_host.dylib) via the
# src/firm/platform/host CMake project -- fast (~8s clean, <1s incremental), skips the
# slow micro:bit firmware compile. This is what TestGUI Sim mode / sim_loop load.
# Restored sprint 108 (SimPlant sim rebuild).
build-sim:
    cmake -S src/firm/platform/host -B src/firm/platform/host/build -DROBOT_RUN_MODE=SIM
    cmake --build src/firm/platform/host/build --parallel

mbd-install:
    pipx install git+https://github.com/Busboombot/mbdeploy.git

# Launch the Robot Test GUI (PySide6 cockpit) against real hardware
# (083-004). One-time prerequisite: `uv sync --group gui` (installs PySide6 +
# aprilcam -- see pyproject.toml's [dependency-groups] gui comment).
# Sim mode is available again (sprint 108): build the sim lib first with
# `just build-sim` (or `just build`), then Connect in Sim mode.
testgui:
    uv run python -m robot_radio.testgui

list:
    mbdeploy list

probe:
    mbdeploy probe

deploy *args='':
    mbdeploy deploy {{args}}

build-deploy *args='':
    mbdeploy build && mbdeploy deploy {{args}}

# Flash the bench firmware image directly over SWD with pyOCD, bypassing
# mbdeploy's serial-announcement handshake. Uses the normal incremental build;
# run `just build-clean` first when you specifically want a fresh rebuild.
# Bench flashes use the generated MICROBIT.hex artifact; if the target rejects
# the initial write, recover with a mass erase, retry, then reset into the
# flashed image.
deploy-pyocd: build
    (pyocd load {{pyocd_probe_args}} -t {{pyocd_target}} MICROBIT.hex || { pyocd erase {{pyocd_probe_args}} -t {{pyocd_target}} --mass && pyocd load {{pyocd_probe_args}} -t {{pyocd_target}} MICROBIT.hex; }) && pyocd reset {{pyocd_probe_args}} -t {{pyocd_target}}

# Bench dev loop: build the firmware, flash the connected robot, then capture
# binary PUSH telemetry with src/tests/bench/relay_telemetry_rate.py and print
# the rate/drop/gap/malformed report. Read-only on the robot — no motion is
# commanded, so it is safe on the stand.
#
# Extra args go to the capture script and win over the defaults below (argparse
# takes the last occurrence), e.g.:
#     just devtest --duration 120
#     just devtest --port /dev/cu.usbmodem2121302 --label relay --json-out /tmp/relay.json
devtest-deploy *args='': deploy-pyocd
    uv run python src/tests/bench/relay_telemetry_rate.py \
        --port {{robot_port}} --label devtest --duration 30 {{args}}

devtest *args='':
    uv run python src/tests/bench/relay_telemetry_rate.py \
        --port {{robot_port}} --label devtest --duration 30 {{args}}

# Leave running, then attach VS Code "(attach) micro:bit PyOCD" or `just gdb`.
# Start a pyOCD GDB server for the micro:bit V2 (nRF52833) on :3333.
debug:
    pyocd gdbserver -t nrf52833 --persist

# Attach gdb to a running `just debug`, flash, reset, and stop at main().
gdb:
    arm-none-eabi-gdb build/MICROBIT \
        -ex "target remote :3333" \
        -ex "load" \
        -ex "monitor reset halt" \
        -ex "break main" \
        -ex "continue"

# Interactive pyOCD console — read/write registers, memory, peripherals.
commander:
    pyocd commander -t nrf52833

# CTRL-AP mass erase to recover an APPROTECT-locked nRF52, then reflashable.
erase:
    pyocd erase -t nrf52833 --mass
