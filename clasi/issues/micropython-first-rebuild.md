---
status: pending
---

# MicroPython-First Rebuild — Everything Python Except the DiffDrive Kernel

## Description

The DiffDrive extraction proved the pattern: a self-contained C++ kernel
behind four small ports, everything else replaceable. This inverts the
firmware around it: **MicroPython on micro:bit becomes the base**,
rebuilding the proven `micropython-vevov-handoff` work from scratch on
its lessons. Everything is Python — drivers, boot, config, telemetry,
motion, the v5 protocol engine — **except the differential drive**, which
stays the C++ `src/firm/diffdrive/` package plus its Nezha motor leaf.

**Stakeholder decisions (2026-08-18), all fixed:**

| decision | choice |
|---|---|
| v5 wire | byte-for-byte compatible — host tooling + relay unchanged |
| transports | **v5 on radio** (primary); **REPL on USB and WiFi**; WiFi also carries the UDP v5 plane (proven dual-plane) |
| C++ payload | DiffDrive kernel + NezhaMotor leaf + minimal shims — nothing else |
| old firmware | hard cutover: `src/firm` (minus `diffdrive/`) frozen |
| location | new worktree off master; Python in `src/upy/`, C in `src/upy_native/`, build in `upy/` |
| first robot | gopiv (motors+encoders+OTOS+line re-fitted, WiFi module, ran the old MP image) |

## What exploration established

**The old MP worktree is a launchpad** (`micropython-vevov-handoff`, 38
commits, worktree `.worktrees/micropython-exploration-repl-commands`):
Gate 2 closed — no-SoftDevice link works, 132 KB flash headroom, GC heap
40 KB, `DEVICE_BLE:0`. `micropython/build.sh` (813 lines) is a working
patch-engine build over vendored `micropython-microbit-v2` @0697c6d.
The landmine ledger L1–L9 is paid-for knowledge; the non-negotiables:

- `MICROPY_NLR_SETJMP=1` (GCC15: any exception HardFaults without it)
- fiber switches from VM/GC hooks **corrupt the heap** — the only safe
  yield is `microbit_hal_idle()` (main context); Python execution never
  happens from a fiber
- the WiFi module persists state across nRF reflashes — power-cycle first
- `Wheels.duration`/`Move.time` are **[ms]** — a sec/ms slip once ran
  wheels 8+ minutes; 5000 ms lease ceiling + boot zero-write mandatory
- per-char AT sends flood the module — one CIPSEND per datagram
- build.sh editing ritual: `bash -n`, py_compile heredocs, `--clean`
  before hardware verify

**v5 contract** (`docs/protocol-v5.md` is stale; truth = `src/protos/` +
`src/firm/core/comms.cpp`+`telemetry.cpp`): COBS keyed 0x0A,
CRC-16/CCITT-FALSE over `command+':'+payload`, CRC-then-COBS; parse
order load-bearing (relay sigils dropped first; TLM/SEED/DBG intercepted
before the binary branch; TLM inbound is a cleartext mode verb); 25
verbs; ack ring depth 12 packed `corr_id<<4|err`, repeats 3; telemetry
emit policy default AUTO = silent-while-parked, 25 ms period, pending
acks force emission; banner byte-frozen. **`src/host/robot_radio/io/
wire_codec.py` is pure MicroPython-clean Python** — ports nearly
verbatim; `src/tests/fixtures/wire_golden_vectors.txt` (8 cross-language
vectors) is the acceptance fixture. Radio on-air: `[SEQ][FLAGS][LEN]`
fragments, MTU 247, group 10; MP's `radio` module natively supports
length/queue/channel/group — the old IRQ-ownership conflict dissolves,
and `queue=4` fixes the C++ single-slot RX loss.

**Verified directly:** MP `radio.config` exposes everything the shim needs;
`microbit.uart.init(tx,rx)` **retargets the one stdio UART** — so the
WiFi UART must be a tiny UARTE1 C shim (the stock port never exposes the
second UARTE); the AT state machine on top is Python.

## Architecture

```
                  ┌──────────────────────────────── micro:bit ─┐
 USB serial ──────┤ MicroPython REPL (stock, foreground)       │
 WiFi TCP :7654 ──┤ REPL mirror (C stdio hook, proven pattern) │
                  │                                            │
 WiFi UDP :7654 ──┤──┐                                         │
 radio (v5/relay)─┤──┤→ src/upy/comms.py — Python v5 engine    │
                  │  │   runs as a BOUNDED SCHEDULED PUMP      │
                  │  │   (micropython.schedule off a timer —   │
                  │  │   between-bytecodes, REPL stays live)   │
                  │  │                                         │
                  │  └→ src/upy/: config, telemetry, motion,   │
                  │      otos.py, line.py, wifi_at.py,         │
                  │      radio_shim.py            (all Python) │
                  │                                            │
                  │ src/upy_native/ (the ONLY C/C++):          │
                  │   moddiffdrive — DiffDrive kernel (compiled│
                  │   IN PLACE from src/firm/diffdrive/) +     │
                  │   NezhaMotor leaf + Clock/Sleeper/Launcher │
                  │   + i2c broker + UARTE1 pipe + watchdog    │
                  │   kernel on its own CODAL fiber @24 ms     │
                  └────────────────────────────────────────────┘
```

Load-bearing design points:

- **Kernel on a CODAL fiber** (FiberLauncher→`create_fiber`) + the
  `microbit_hal_idle()` yield patch. The lease watchdog that zeroes duty
  must not depend on Python health. Companion: a **zero-only starvation
  watchdog** in the VM hook (never yields; raw zero write if kernel
  cycles stall >250 ms with wheels commanded) — a CPU-bound Python loop
  never reaches idle.
- **One I2C ledger.** All Python sensor traffic goes through the C
  module's `robotio.i2c_xfer()` so per-device `lastEnd/readyAt` timers
  and the TWIM-errata gap are shared with the kernel's 0x10 traffic.
- **Codec generated, not hand-written**: extend
  `src/scripts/gen_messages.py` with `--emit-upy` → `src/upy/msgs.py`
  (third renderer over the same descriptor walk). One schema, three
  targets.
- The diffdrive package is compiled **in place** — `src/tests/diffdrive/`
  keeps guarding the single copy.

## Milestones (risk-ordered; each gate is a command)

**M0 — worktree + image boots.** New worktree; fork the old `micropython/`
machinery to `upy/` with all MUST-KEEP patches; **rescue the untracked**
`clasi/issues/micropython-full-firmware-in-the-image-gates-3-7.md` and
`config/wifi_secrets.json` from the old worktree; strip the modrobot
exploratory motion layer (dead by directive; keep as pattern reference).
*Gate:* `cd upy && ./build.sh --clean` → hex; flash; USB REPL answers;
flash end < `_fs_start` (0x6D000).

**M1 — moddiffdrive: wheels from the REPL.** `src/upy_native/`:
`moddiffdrive.cpp` + `_glue.c` + manual qstrs (the proven two-file
pattern); NezhaMotor leaf ported line-for-line (anti-latch shaping is
not re-derived); kernel leaves; i2c broker; boot zero-write before the
VM starts; VM-hook watchdog; 5000 ms lease ceiling in the binding.
Python API: `diffdrive.configure/begin/start/drive/driveDuty/neutral/
estop/output/lastError`. Land gopiv's `left_port:2,right_port:1` wiring
fix. *Gate:* (1) `uv run python -m pytest src/tests/diffdrive/` still
green, untouched; (2) on gopiv: `drive()` with a 1000 ms lease → motion,
zero at expiry, counts advance with the right signs; (3) safety: `drive()`
then `while True: pass` → watchdog zeroes ≤300 ms; reset mid-drive →
boot zero-write silences. **Highest-risk milestone; nothing proceeds
until it scores.**

**M2 — wire codec offline.** `src/upy/wire.py` (port of wire_codec.py)
+ generated `src/upy/msgs.py`. *Gate:* new
`src/tests/unit/test_upy_wire_golden_vectors.py` → 8/8 against the
fixture; encode↔decode round-trip against the host pb2 for every binary
verb; `mpy-cross` compiles every `src/upy/*.py`.

**M3 — v5 engine + radio (the primary transport).** `src/upy/comms.py`
mirrors `dispatchLine()` order byte-for-byte; ack ring; telemetry emit
policy; banner/boot/READY sequence; the **scheduled-pump plumbing**
(timer → `micropython.schedule(pump)`, bounded work per call, stdin-wait
patch so pending callbacks run while the REPL blocks).
`src/upy/radio_shim.py` over MP `radio` (`length=250, queue=4,
channel=<json>, group=10`), fragment reassembly per
`microbit_radio_link.cpp`, feeding the engine. *Gate:* offline first —
comms.py under CPython + loopback vs the host's own client, banner/ack
byte-exact; then hardware: `rogo repl gopiv ping` **through the relay,
unchanged tooling**; WHEELS over radio → motion + acks; REPL on USB
stays interactive throughout.

**M4 — WiFi: REPL mirror + UDP v5 plane.** C: UARTE1 byte-pipe shim +
the stdio TCP-REPL hook (reuse the proven `wifi_stdio.cpp` core).
Python: `wifi_at.py` AT state machine (CIPMUX=1, UDP :7654, per-datagram
coalescing, ≥50 ms TLM throttle on this plane). *Gate:*
`wifi_bench_gate.py --port wifi: --skip-drive` 9/9 with an `nc` REPL
session held open; power-cycle discipline in the bench notes.

**M5 — full Python firmware layer.** `config.py` (gopiv.json on-device,
fail-closed key check; `wheel_control`→`DiffDrive::Config` via
travel_calib×10; CONFIG/SET_FIELD/GET_CONFIG live), `otos.py`, `line.py`
(bus facts as captured — 0x17 init/scales/20 ms, 0x1A ×4/50 ms),
`telemetry.py` full 22 fields, `motion.py` (move queue 5-deep, stop
conditions, timeout fault, replace, GO_TO, SEED/POSE, CALIBRATE — every
duration [ms]). *Gate:* `move_protocol_bench.py` full pass over the
radio path; OTOS pose sane in TLM; fail-closed boot test.

**M6 — acceptance sweep on gopiv.** No new code. `--clean` rebuild;
`wifi_bench_gate` 9/9; `move_protocol_bench` full; quiet-host kill test
(lease stops wheels); power-cycle boot-zero test; 10-min dual-plane soak;
RAM/flash checkpoint; `git diff master -- src/firm` = diffdrive-only.

**M7 (later) — tovez:** color driver (0x43/0x39), `radio_bench_gate.py`
over getez, per-robot JSON.

## Critical files

| role | path |
|---|---|
| C++ payload (in place) | `src/firm/diffdrive/differential_drive.{h,cpp}` + gate `src/tests/diffdrive/` |
| build machinery to fork | `.worktrees/micropython-exploration-repl-commands/micropython/build.sh` (+`codal_overlay.json`, `patches/`) |
| codec to port | `src/host/robot_radio/io/wire_codec.py` → `src/upy/wire.py`; fixture `src/tests/fixtures/wire_golden_vectors.txt` |
| dispatch/emit contract | `src/firm/core/comms.cpp`, `src/firm/core/telemetry.cpp` |
| motor leaf to port | `src/firm/hardware/nezha/nezha_motor.{h,cpp}` |
| AT-sequence oracle | old worktree `micropython/modrobot/wifi_stdio.cpp` |
| radio framing reference | `src/firm/platform/microbit/microbit_radio_link.cpp` |
| codegen to extend | `src/scripts/gen_messages.py` (`--emit-upy`) |
| binding pattern | old worktree `micropython/modrobot/{modrobot.cpp,modrobot_glue.c}` |

## Verification

- **Offline before hardware, always**: golden vectors (8/8), CPython
  loopback engine tests, mpy-cross compile of all Python, fragment codec
  vs captured on-air bytes.
- **Existing suites untouched and green**: `src/tests/diffdrive/` (the
  kernel's own gate), host `src/tests/unit`.
- **Hardware ladder on gopiv** (each after `--clean` + ~5 s post-flash):
  REPL wheel spin → watchdog/lease/reset safety triple → `rogo repl
  gopiv ping` via relay with unchanged tooling → `wifi_bench_gate` 9/9 →
  `move_protocol_bench` full → M6 sweep. Smallest-visible-pulse first;
  encoder delta read from the other plane; explicit stop-verify (Δenc=0
  over 2 s). Deploy by UID only; module power-cycle before WiFi work.

## Open items (non-blocking, decide during execution)

1. `VER`/`ID` strings: format is frozen; the version value will identify
   the upy build — flag if any host tool pins the old value.
2. gopiv bench-day check: master's `gopiv.json` still calls it a bench
   rig and lacks the port keys — physically confirm motors/OTOS/line
   before scoring M1 (the gates-3-7 doc carries the wiring fix).
3. Process: run the worktree OOP (as the diffdrive exploration did),
   converting to sprints on merge-back.

## Related

- `clasi/issues/later/micropython-as-the-base-feasibility-and-migration-plan.md`
  — the 2026-08-07 feasibility study this supersedes as the execution
  plan (its memory/CPU numbers, Pybricks precedent and phase analysis
  remain the reference).
- `.worktrees/micropython-exploration-repl-commands` (branch
  `micropython-vevov-handoff`) — the launchpad: `micropython/build.sh`,
  `micropython/modrobot/`, `docs/handoff/micropython-full-firmware-integration.md`,
  `micropython/vevov-micropython-spike-handoff.md`, and the untracked
  `clasi/issues/micropython-full-firmware-in-the-image-gates-3-7.md`.
- Commit `15f35281` — the DiffDrive package extraction that made this
  possible (`src/firm/diffdrive/` + `src/tests/diffdrive/`, the only
  files that came back to master from the exploration worktree).
- Captured from plan-mode session 7c5eb9d3 (2026-08-18); the plan-mode
  hook did not file it, so this issue is the record.
