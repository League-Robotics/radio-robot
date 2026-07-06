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

# Build ONLY the host-simulation library (libfirmware_host) — skips the ARM
# firmware, so it's fast (~8s clean, <1s incremental) and needs no ARM toolchain.
# Runs the same codegen steps build.py does so generated sources stay fresh.
# Note: `just build` already builds BOTH the firmware hex and this sim library.
build-sim:
    # 077-001 / 081-004: gen_default_config.py writes to source/robot/
    # DefaultConfig.cpp -- a directory the greenfield rebuild's new source/
    # tree deliberately does not (yet) create. build.py already guards this
    # structurally (see its own 077-001 comment); this recipe had never been
    # updated to match, so it hard-failed the moment tests/_infra/sim/
    # existed for this recipe to reach the cmake steps below. Mirror
    # build.py's guard here so it self-heals the same way once source/robot/
    # reappears.
    if [ -d source/robot ]; then uv run python3 scripts/gen_default_config.py; fi
    uv run python3 scripts/gen_messages.py
    uv run python3 scripts/gen_boot_config.py
    cmake -S tests/_infra/sim -B tests/_infra/sim/build -DROBOT_RUN_MODE=SIM
    cmake --build tests/_infra/sim/build --parallel

mbd-install:
    pipx install git+https://github.com/Busboombot/mbdeploy.git

# Launch the Robot Test GUI (PySide6 cockpit) against the simulator or real
# hardware (083-004). One-time prerequisite: `uv sync --group gui` (installs
# PySide6 + aprilcam -- see pyproject.toml's [dependency-groups] gui comment).
# Depends on build-sim so Sim mode always has a freshly (re)built
# libfirmware_host to Connect to -- build-sim is fast on an unchanged tree
# (<1s incremental), so gating on it here costs nothing on the common case.
testgui: build-sim
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
