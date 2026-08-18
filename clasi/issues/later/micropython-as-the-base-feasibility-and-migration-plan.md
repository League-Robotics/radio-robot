---
status: pending
---

# MicroPython as the Base — Feasibility and Migration Plan

## Description

Stakeholder directive (2026-08-07): give the robot on-robot programmability
by taking **MicroPython as the base** — the official micropython-microbit-v2
build — and adding our robot kernel to it, rather than embedding an
interpreter into our firmware. The kernel keeps its 50 ms loop in its own
execution context, owns the I2C bus, and Python talks to it exactly like
every other client does — by exchanging protocol datagrams (send /
data-waiting / receive / receive-wait), **no shared memory**, with an
optional binary fast path that builds message structs directly instead of
going through text/COBS.

**Stakeholder decisions (2026-08-07):**

1. **Python replaces MissionScript.** No custom DSL gets designed; on-robot
   MicroPython is the mission-scripting story. The VerbBinding seam from the
   kernel-packaging program (see Related) survives as the mechanism by which
   robot classes surface commands into Python.
2. **REPL on USB, robot protocol on radio.** MicroPython owns the USB serial
   for its normal REPL/editor experience; bench tooling (rogo, TestGUI,
   bench gates) reaches the kernel over the radio relay — already the
   untethered path. Config-over-USB is a small Python helper forwarding
   config datagrams to the kernel.

**Feasibility verdict (measured 2026-08-07): memory fits comfortably — but
only with `DEVICE_BLE: 0` — and the real constraint is scheduling, gated by
a Phase A spike.**

Flash (512 KB total), from parsing the released micropython-microbit-v2
v2.1.2 hex and our build/MICROBIT ELF:

| | bytes |
|---|---:|
| MicroPython V2 code (0x1C000–0x68000 in the stock image) | 311,296 |
| Its flat filesystem (keep — it's how main.py gets on) | 24,576 |
| Stock image's S113 SoftDevice + MBR + DFU bootloader | 147,456 |
| Stock slack (only 20 KB contiguous) | 36,864 |
| **Slack with `DEVICE_BLE: 0`** (drops SD/MBR/bootloader; our codal.json already sets this) | **~180 KB** |
| Our app code (src/firm 41.5 KB + src/motion 14.1 KB + messages; CODAL/libc already in MP's image) | ~60–100 KB incremental |

→ fits with ~80–120 KB margin. Cost of `DEVICE_BLE: 0`: no BLE flashing from
the micro:bit phone app — irrelevant for USB/DAPLink-flashed robots. If ever
tight, pruning speech/music/easter-egg modules recovers another 30–60 KB.

RAM (128 KB): our firmware's real static demand is 15.1 KB (the "98% used"
figure is the CODAL allocator claiming everything as heap). MicroPython
statically reserves a 64 KB GC heap + 8 KB stack, leaving ~32 KB CODAL heap
stock — marginal (the port has open heap-exhaustion issues, e.g. #228,
without our code). Two changes make it comfortable: `DEVICE_BLE: 0` (+8.2 KB,
RAM origin moves down) and shrinking the GC heap 64→40 KB (one line in
codal_port/main.c; +24.6 KB; 40 KB is still 2.5× a V1's entire RAM). Result:
~45–55 KB CODAL heap against ~15 KB statics + 4–8 KB kernel fiber stack.

CPU: our cycle does ~9–11 ms of real work per 50 ms (~20% duty), already
yielding at 4 points — plenty of room for the VM.

Precedent: **Pybricks** runs full MicroPython + a 200 Hz multi-axis motion
controller (trajectory planner + observer) in 208 KB flash / 64 KB RAM on
the Technic Hub — half the micro:bit V2's budget in both dimensions. MIT
licensed; both patterns and code are reusable.

## Cause

Why this doesn't just work today:

1. **The stock port never yields to fibers.** MicroPython runs on the main
   CODAL fiber; its "background processing" hook only fires an event
   (~200 µs, no context switch), and even Python `sleep()` busy-waits
   (`mp_hal_delay_ms` spins on `__WFI`, never `fiber_sleep`). A robot-kernel
   CODAL fiber would get **zero CPU while Python runs**. The port's own
   real-time subsystems (display, music, soft timers) run from a 6 ms
   system-timer event in interrupt context — never from fibers. This is the
   decisive technical risk, not memory.
2. **The stock image ships a SoftDevice it doesn't use for Python.** BLE is
   not exposed to Python (radio is proprietary-mode via the port's own
   drv_radio.c); the S113 + DFU bootloader exist only for phone-app
   flashing, and their 147 KB is what makes the stock image look full.
3. **Our firmware constructs its own `static MicroBit uBit`** (main.cpp) —
   MicroPython's codal_app owns its own. Two instances would double-register
   every CODAL component. (Mechanical fix: nothing outside our main.cpp
   names `uBit`; everything takes references.)
4. **Our `com/` layer conflicts**: MP's port steals RADIO_IRQn for its
   `radio` module (one vector, one set of radio registers — cannot coexist
   with our `Radio`), and our `SerialPort` owns the UART that MP needs for
   the REPL. `com/` also contains the only true singletons and busy-wait
   spins in src/firm (Radio's static ISR trampoline; SerialPort's 5 ms and
   24 ms bare spins).
5. **Shared-resource budgets**: the KeyValueStorage flash page has only
   5 slots, shared between our TuningStore and MP's `microbit` module;
   CODAL's 2 KB shared physical stack model means the deepest call chain of
   any fiber must fit (MP's config raises `DEVICE_STACK_SIZE` to 8 KB —
   better than ours).
6. Alternatives were assessed and ruled out: mainline MicroPython `nrf` port
   has no nRF52833 target and no CODAL; CircuitPython's microbit_v2 board
   gives 32 KB RAM to a SoftDevice and has no CODAL; eLua is dormant; Forth
   (Mecrisp/zeptoforth) is tiny but isn't Python for a student audience;
   JerryScript's ~160 KB engine eats the budget. The inversion (our firmware
   as base, MicroPython embedded via examples/embedding) is architecturally
   cleaner but forfeits the whole `microbit` module surface (display, pins,
   music, filesystem, REPL, editor compatibility) — pedagogically much
   poorer.

## Proposed fix

### Architecture

- **Kernel on the codal_app side.** Our `src/firm` + `src/motion` compile
  into MP's codal_app (CODAL's CMake, full `uBit` access), mirroring the
  port's own codal_app/codal_port split: a small `extern "C"` datagram HAL
  (`robot_hal.h`, modeled on their `microbithal.h`) faces a thin `modrobot.c`
  in codal_port. `USER_C_MODULES` officially supports C++ (Foundation's
  micropython-cpp-module-example; their PR #210), but the codal_app side is
  the right home for CODAL-dependent kernel code.
- **`uBit` is borrowed, not constructed** — our main.cpp is deleted in this
  flavor; MP's codal_app main gains kernel boot (`composeRobot()` + kernel
  start per the Phase A outcome).
- **`com/` is replaced by one `MessageRingTransport`** — an in-memory SPSC
  ring pair (~1.7 KB for depth-4 × 211 B both ways) implementing
  `App::Transport` (src/firm/app/comms.h:23, three pure virtuals). The whole
  test suite already runs on an in-memory Transport
  (src/tests/sim/support/fake_transport.h:62), so the pattern is proven.
  `App::Comms` and everything above it are unchanged.
- **The kernel keeps owning the radio**: delete MP main.cpp's
  `NVIC_SetVector(RADIO_IRQn, ...)` line and drop `modradio.c`; `uBit.radio`
  + our Radio/relay framing work unchanged at 250-byte packets, so all
  existing relay/rogo/bench tooling talks to the MP-hosted image with zero
  changes. (MP's radio PHY is byte-identical to CODAL's anyway; only the
  framing above differs.)
- **Status queries never round-trip**: the kernel pushes its per-cycle
  telemetry frame into the ring; the Python module keeps a cache of the
  latest frame; `robot.pose()` etc. read the cache. (This is the one bill of
  the no-shared-memory boundary — Pybricks reads a struct; we cannot.)
- **Pybricks patterns to copy** (all MIT): C-system-owns-main / MicroPython
  is a callee (program stop leaves motors in a defined state); tick-as-flag,
  all work at yield points, no robot logic in ISRs; all four VM hooks
  **including the GC-loop hook** (what keeps control alive through a heap
  sweep); non-blocking command + optional-wait dual-mode sync/await API;
  drift-free tick accounting (`timer.start += PERIOD`, never `= now`, with
  the zero-dt guard).

### Scheduling options (Phase A decides by measurement)

1. **Patch the port to yield** — `microbit_hal_background_processing()` →
   `schedule()`, `mp_hal_delay_ms()` → `fiber_sleep()`, add the GC-loop
   hook. Small diff; keeps the fiber-based kernel and blocking I2C intact.
   Risk: latency becomes Python-shaped — a long C-implemented operation
   (big string join, flash write, .mpy import) doesn't yield; the 50 ms
   deadline goes soft.
2. **Kernel at interrupt priority** — hard deadline, but requires
   restructuring devices/ into a non-blocking TWIM/EasyDMA state machine.
   Substantial rework; fallback only.
3. **Hybrid** — bounded actuation + encoder sampling (and a motor-zeroing
   deadman) at interrupt priority; planner/odometry/estimation/telemetry in
   the yielding fiber. Fits the existing base/motion split. Note the Nezha
   brick latches its last commanded speed, so a starving-proof stop path is
   a genuine safety requirement regardless.

### Phases

**Phase A — concurrency spike (1 sprint, decision gate, cheap).** Build
micropython-microbit-v2 from source with our config overlaid
(`DEVICE_BLE: 0`, radio 250, GC heap 40 KB); verify the flash/RAM numbers on
a real link. Apply the minimal yield patch. Spawn a stub 50 ms fiber doing
representative timed work; measure delivered-period jitter under adversarial
Python: tight `while True`, big string join, `.mpy` import from flash,
`display.scroll`, forced GC storm. Also prove: a C++ codal_app-side unit
builds and links; the radio ownership swap with our relay talking to
`uBit.radio` end to end. **Gate**: 50 ms holds (delivered 54 ms is today's
reference tolerance) → option 1; otherwise option 3, with the deadman
designed before any further port work.

**Phase B — kernel-as-guest packaging (1–2 sprints).** Pin
micropython-microbit-v2 (+ its MicroPython and CODAL pins) as the base; our
repo carries the overlay (config, patches, kernel source list — our four
vendored CODAL trees carry zero local modifications, verified, so aligning
to MP's pin is free). Kernel boots from MP's main; `MessageRingTransport`
in; radio kernel-owned. Keep producing the pure-C++ firmware until parity is
proven (the standing bench gate runs against both).

**Phase C — the `robot` Python module (1 sprint).** `modrobot.c` datagram
primitives + the binary fast path (build `msg::CommandEnvelope` PODs
directly into ring slots — a message costs a memcpy, not a serialize;
COBS/CRC skipped in-RAM). A thin pure-Python `robot.py` in the filesystem
with the Pybricks API shape (non-blocking commands, `wait=True` default that
polls completion while yielding, telemetry cache for queries).
Hardware-contributed verbs (VerbBinding) surface via HELP discovery, never
hard-coded.

**Phase D — lifecycle & safety (1 sprint).** Program stop / Ctrl-C / soft
reboot → kernel `estop()` and a defined idle state; move timeouts enforced
by the kernel regardless of VM state — spec the guarantee and test it with a
deliberately wedged Python program; implement the starving-proof stop path
per Phase A; audit the 5-slot KeyValueStorage budget shared with MP's
`microbit` module.

**Phase E — UX & tooling (1 sprint).** main.py via the filesystem + standard
editor flow; USB serial is purely the REPL, robot protocol on radio (bench
tooling moves to the relay path); config helper keeps the "configure once
over serial, then radio" flow; docs on robots.jointheleague.org (currently
no MicroPython content); rogo/TestGUI awareness of the MP-hosted image.

### Relationship to the kernel-packaging program

Complementary: MicroPython-hosted is another **platform** consuming the same
core through the same seams. Phase A is independent and should run
immediately — its result should inform the kernel-packaging program before
the HAL sprint locks cycle-ownership details. Phases B–E benefit from
kernel-packaging Phases 1–3 (RobotLoop decomposition gives the kernel a
clean entry; the Comms link-list/origin-routing change makes the message
ring just another registered link). Recommended sequencing: spike next,
alongside kernel-packaging Phase 0; B–E after kernel-packaging Phase 3.

## Verification

- **Phase A**: delivered-period histogram under the adversarial programs,
  published as the gate artifact; relay round-trip against `uBit.radio` at
  250 B.
- **Phase B**: `radio_bench_gate.py` passes against the MP-hosted image over
  the relay, unchanged; composition parity with the pure-C++ image (same
  robot JSON, same tour closure on tovez).
- **Phase C**: a student-shaped `main.py` square tour on tovez on the stand;
  API latency measurement (command→ack, query→cache read).
- **Phase D**: wedged-VM test — a Python program blocking in C must not
  prevent move-timeout enforcement or estop; measured stop distance matches
  the sprint-133 hardware numbers.
- **Sim**: the message-ring transport gets a host-side twin so the MP-hosted
  message plane is exercised in the existing sim test suite (FakeTransport
  already proves the pattern).

## Related

- `clasi/issues/kernel-packaging-host-sim-rigor-and-hardware-abstraction-program-plan.md`
  — the kernel-packaging program this rides on (App::Transport link list,
  RobotLoop decomposition, VerbBinding seam; MissionScript language design
  is now superseded per decision 1).
- micropython-microbit-v2 (github.com/microbit-foundation/micropython-microbit-v2):
  codal_app/codal_port split, `microbithal.h` ABI, `src/codal.patch` build
  wart, PR #210 (out-of-tree C++ modules),
  micropython-cpp-module-example, issues #156 (heap headroom), #165 (atomic
  sections), #228 (DEVICE_HEAP_ERROR), release v2.1.2 hex (memory numbers).
- Pybricks (github.com/pybricks/pybricks-micropython, MIT):
  `lib/pbio/src/os.c` (protothreads), `motor_process.c` (drift-free tick),
  `bricks/_common/mpconfigport.h` (the four VM hooks incl. GC),
  `pybricks/tools/pb_type_async.c` (dual-mode wait), MULTI_MPY_V6 loader.
- In-repo: `docs/knowledge/pybricks-motion-control-study.md` (algorithms;
  the integration findings here extend it), `App::Transport`
  (src/firm/app/comms.h:23), FakeTransport
  (src/tests/sim/support/fake_transport.h:62), memory audit from
  build/MICROBIT + MICROBIT.map (2026-08-07, post-608cc885).
