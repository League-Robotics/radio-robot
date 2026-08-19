---
status: pending
---

# MicroPython image: full firmware integration (Gates 3–7)

## Description

Stakeholder-approved plan (2026-08-15, micropython-vevov-handoff branch): the
MicroPython image must run the REAL firmware — `Control::DifferentialDrive`,
`Motion::Planner`, `Core::RobotLoop` — not the exploratory `modrobot` motion
layer (open-loop duty + local loops). Nothing merges to master until this is
done. Gates 1–2 of `docs/handoff/micropython-full-firmware-integration.md` are
complete (branch unified; CODAL upgraded to the standard library set,
no-SoftDevice link, ~135 KB flash freed, measured in the map). This issue is the
execution plan for Gates 3–7.

NOTE: this plan was approved in-session but its automatic issue capture was lost
to a cross-session file collision (the plan-to-issue hook's target file was won
by a parallel session's different plan,
`differentialdrive-one-object-one-struct-exploratory-worktree.md`). This file is
the manual re-save of the approved content.

**Measured facts the plan rests on** (from both fresh ELF maps, 2026-08-14):

- Adding `motion/` + `control/` + `core/` + `kinematics/` + config/types glue ≈
  **37.4 KB flash, 5.0 KB static RAM**. `messages/` and the four `hardware/`
  leaves are already compiled into the image.
- Flash headroom 139 KB before `_fs_start` (0x6D000); ≈101 KB remains after
  integration. RAM: the 4,960 B of new bss comes out of the 68.3 KB CODAL malloc
  arena (the linker auto-sizes `.heap` to all leftover RAM — `size`'s bss figure
  always looks full; the old "~2.5 KB slack / Gate 3 blocked" reading was an
  accounting error). Effective runtime arena = linker `.heap` − 6,144 B (CODAL
  caps the heap at `__StackTop − DEVICE_STACK_SIZE(8192)`, not the linker's
  −0x800; keep 8192 — it is the VM's stack-depth window).
- Motion-in-Python was measured and REJECTED: it would save only 14.4 KB flash /
  **812 B** RAM, and would fork `planner.cpp`'s 2,074 lines of
  measurement-derived behavior from the standard firmware. All-C++ satisfies the
  stakeholder's "I2C + PID in C++" constraint trivially.
- Retiring the exploratory layer is a net RAM SAVING ≈3.6 KB: its four
  singletons each embed a private 2,048 B I2C device table; the real
  `Core::RobotGraph` (5,608 B total) replaces all of it.

## Proposed fix

Target architecture: `Core::RobotLoop` runs in its own CODAL **kernel fiber**
(the image's first fiber; the only motor writer), composed via
`Core::composeRobot(bus, clock, sleeper, loopback /*serial slot*/, udpV5
/*radio slot*/, nullptr /*tuningStore*/, banner, idLine)` —
signature verified at `src/firm/core/boot_wiring.h:359`. MicroPython runs in the
~24 ms the kernel sleeps per 32 ms cycle.

- **Rule A (AT-channel arbitration):** the kernel fiber never touches the Wi-Fi
  module. Both transports cross to the main fiber via lock-free SPSC rings;
  `robot_kernel_pump()` (main fiber, the three call sites `robot_v5_service()`
  uses today) moves bytes to/from `RobotWifi::readV5()/sendV5Datagram()`.
  wifi_stdio.cpp's single-context assumption is preserved exactly.
- **Rule B (VM-state arbitration):** the kernel gets its own `KernelClock` /
  `KernelSleeper` / `KernelI2CBus` leaves wrapping tiny `extern "C"` shims
  (`create_fiber`, `fiber_sleep`, `schedule`, `system_timer_current_time_us`) in
  a new codal_app file. `platform/microbit/` is NOT compiled (needs MicroBit.h);
  `src/firm/main.cpp` stays excluded.

Recorded decisions: (1) v5-over-USB is EXCLUDED in the MP image (REPL owns USB;
wire clients use `wifi:`); (2) config persistence disabled (`tuningStore =
nullptr` — live CONFIG pushes work, baked JSON rules at boot); (3) a **zero-only
starvation watchdog** is included: from the VM hook (never yields), if kernel
cycles stall >250 ms while wheels are commanded, write raw zero duty (retry ×2)
and latch a flag — covers a CPU-bound Python loop starving the kernel fiber, the
one hazard the standard firmware cannot have; (4) the gopiv true-wiring fix
(`left_port: 2, right_port: 1, fwd_sign_left: +1, fwd_sign_right: -1` per
gopiv.json `_port_note`) rides the Gate-5 flash.

Gate-by-gate:

- **Gate 3 — compile-in sweep** (build only; behavior unchanged). In
  `micropython/build.sh`'s `--with-modrobot` block: add
  `INC += -I$(abspath ../../../../src)`; extend `SRC_CXX` with
  core/{boot_calibration, boot_wiring, comms, configurator, debug, preamble,
  robot_loop, telemetry}, control/differential_drive, motion/{odometry,
  planner/planner, planner/profile, planner/shape, planner/estimation,
  navigator/navigator, navigator/arc_solver}, kinematics/{kinematics,
  differential, mecanum}, config/persisted_tuning (NOT planner/capi.cpp);
  per-object `-DHOST_BUILD` for persisted_tuning.o only. Fix link errors only by
  ADDING files, never stubbing. L8 ritual after every build.sh edit (`bash -n`,
  py_compile heredocs, `--clean`, verify version string).
- **Gate 4 — robot_kernel.cpp**: new `micropython/modrobot/robot_kernel.cpp`
  (→ codal_port: kernel leaves, both transports constructed-but-inert, 5-field
  banner, `robotKernelEntry()` = static graph → boot → loadPersistedTuning →
  markConfigured → `for(;;) cycle()`, extern-C surface
  `robot_kernel_start/active/pump/state/stats`) and
  `micropython/modrobot/robot_hal_shims.cpp` (→ codal_app, ~25 lines). Patch
  codal_app/main.cpp: `robot_kernel_start(); schedule();` before `mp_main()`
  (bounds the latched-brick window). Patch `microbit_hal_idle()` to add
  `schedule()` (the D1 yield; main context only). Interim guards: UDP v5 plane
  dark, motion verbs raise, sensor verbs re-target to `robot_kernel_state()`
  reads — the kernel is the ONLY motor writer and I2C user from its first flash
  (kills transition I2C contention).
- **Gate 5 — transport unification + wiring fix**: `UdpV5Transport` (rx 1,024 B
  byte ring; tx 4×256 B datagram slots — one send = one CIPSEND, preserving the
  coalescing shape; READY on new-peer edge moves into the pump). Real Telemetry
  emits every ~32 ms vs the 50 ms AT budget was proven at — measure; a
  transport-local ≥50 ms throttle for `TLM:` lines is the lever (acks survive
  via kAckRepeats=3). `robot_v5_service()` body becomes `robot_kernel_pump()`.
  DELETE `dispatchV5Line` + the hand-rolled handlers/telemetry/ack emitters.
  Real `Core::Comms` now answers the full command set on the UDP plane.
- **Gate 6 — robot.* re-target**: `LoopbackTransport` — command path 512 B SPSC
  byte ring (MP fiber writes complete armored v5 lines built with fixed-buffer
  builders modeled on `src/tests/sim/support/wire_test_codec.h`; kernel
  `Comms::pump()` consumes); reply path = kernel-side ack scanner on `TLM:`
  lines → 8-deep (corr_id, err) ring. robot.* enqueues, waits ≤300 ms via
  `mp_handle_pending + microbit_hal_idle` loop, raises `OSError(err)` on NACK.
  `robot.stop()` → ESTOP envelope. Delete the four singletons + local motion
  loops (the RAM win lands). Watchdog lands here.
- **Gate 7 — full acceptance + merge prep**: handoff §7 in full at the user's
  cadence; update the handoff doc (Gate 2 done; §1 microbit_uart.cpp correction;
  §9 superseded; record the decisions + the −6 KB arena correction); confirm
  **`git diff master -- src/firm` is EMPTY** (only data change: gopiv.json
  wiring fix; everything else lives in `micropython/` + docs).

RAM/flash checkpoint at every gate (after `--clean`): `arm-none-eabi-size` + map
grep for `_fs_start|__StackTop|\.heap`; flash end < 0x6D000; soak ≥10 min (arena
starvation is a runtime panic, not a link error). Trim levers if ever needed:
Comms cmdRing 12→8 (−624 B), GC heap 40→36 KB.

## Verification

- Gate 3: image behavior-identical (`nc <robot> 7654` typo → traceback; `rogo
  repl <robot> ping` concurrently; `wifi_bench_gate --port wifi: --skip-drive`).
- Gate 4: both REPL planes alive; `robot.kernel()` cycleCount advancing,
  cyclePeriod ≈32 ms; `robot.encoders()`/`robot.otos()` live; 10-min dual-plane
  idle soak; power-cycle → boot zero-write lands.
- Gate 5: smallest visible pulse first (durations are [ms] — re-verify against
  envelope.proto); wiring acceptance on camera (positive drive forward AND
  `turn +90` sweeps +90° CCW); `wifi_bench_gate` 9/9; `move_protocol_bench` full
  over `wifi:`; **quiet-host test** — command WHEELS, `kill -9` the host,
  encoder delta==0 within duration + ~960 ms re-assertion.
- Gate 6: `robot.move(...)` produces a real planner Move visible in a concurrent
  rogo session's TLM acks; `robot.move_wheels` then `while True: pass` →
  watchdog zeroes ≤~300 ms; typo → traceback (NLR setjmp untouched).
- Gate 7: standing tours over wifi; stand physics (angle stops TIME OUT on a
  stand with live OTOS — correct); Ctrl-D soft-reset with motion in flight;
  30-min dual-plane soak.

## Related

- `docs/handoff/micropython-full-firmware-integration.md` — the gate plan this
  refines; needs the corrections listed under Gate 7
- `clasi/issues/familiar-api-robot-library-xrplib-pybricks-interface-on-our-control-stack.md`
  — the API-skin plan that sits on Gate 6's `robot.*` seam
- `clasi/issues/differentialdrive-one-object-one-struct-exploratory-worktree.md`
  — a parallel session's alternate minimal-kernel exploration (struct interface,
  planner parked); contradicts parts of this plan by design — reconcile with the
  stakeholder before executing both
- `src/firm/core/boot_wiring.h` — the composition contract (verified)
- `micropython/modrobot/wifi_stdio.cpp` — single-context contract Rule A preserves
- `data/robots/gopiv.json` — `_port_note` carries the Gate-5 wiring fix
