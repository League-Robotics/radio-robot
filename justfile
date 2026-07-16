set dotenv-load := true

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
# src/sim CMake project -- fast (~8s clean, <1s incremental), skips the
# slow micro:bit firmware compile. This is what TestGUI Sim mode / sim_loop load.
# Restored sprint 108 (SimPlant sim rebuild).
build-sim:
    cmake -S src/sim -B src/sim/build -DROBOT_RUN_MODE=SIM
    cmake --build src/sim/build --parallel

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
