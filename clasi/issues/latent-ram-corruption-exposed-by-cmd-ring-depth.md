---
status: pending
---

# Latent RAM corruption: shrinking a static buffer hard-faults the firmware at boot

> **Handoff document.** Written to be read cold, with no session context. The
> "Ruled out" section is the most valuable part — it is a list of things that
> look promising and are not, each with the evidence that killed it. Please
> read it before spending time re-deriving them.
>
> **Status:** symptom is masked, root cause is NOT found. `kCmdRingDepth` is
> back at 12, which makes the robot boot. That is a tourniquet.

## Symptom

The firmware hard-faults during boot, before the banner finishes clocking
out of the UART. On the wire you see the first ~13 bytes of
`DEVICE:NEZHA2:robot:tovez:2314287040` and then nothing — no telemetry, no
acks, no command responses. The board still enumerates USB and DAPLink is
healthy, so it *looks* like a dead peripheral rather than a crash.

The trigger is a single constant: `App::kCmdRingDepth`
([src/firm/app/comms.h](../../src/firm/app/comms.h)). At **6** the firmware
faults; at **12** the same tree boots and streams telemetry normally.

## Reproduction

Each bisect point below was built with `just build`, flashed by copying
`MICROBIT.hex` to the `MICROBIT` mass-storage drive, and captured on raw
serial *through the reboot that the flash causes*.

The capture must span the reboot: **the board does not reset on serial open
or a DTR pulse** (see `.clasi/knowledge/`, `serial-monitor-never-shows-the-banner`),
so opening a monitor afterwards shows nothing and tells you nothing.

```bash
# terminal-equivalent: start the capture, THEN trigger the flash
uv run python -c "
import time, serial
p=serial.Serial('/dev/cu.usbmodem2121102',115200,timeout=0.5)
d=b''; t0=time.time()
while time.time()-t0<24:
    c=p.read(512)
    if c: d+=c
p.close(); print(len(d), repr(d[:120]))" &
sleep 3; cp MICROBIT.hex /Volumes/MICROBIT/
```

| commit | boot serial | verdict |
|---|---|---|
| `bd7f75b8` | 97 B — full banner + two `TLM:` frames | boots |
| `3238bdaf` | 97 B — full banner + two `TLM:` frames | boots |
| `5065775a` | **13 B — `DEVICE:NEZHA2`, then dead** | faults |
| `5065775a` with `kCmdRingDepth` set back to 12 | 97 B — full banner + `TLM:` | boots |

`5065775a` is *"per-wheel per-direction speed correction"*. Its firmware
delta is small: `Drive::setWheelCorrection`/`correctedCommand` (bounds-safe,
`[2][2]` indexed 0/1, and only reachable from the control loop, not from
boot), regenerated `boot_config.*`, and this one constant.

## Fault forensics

Caught with a breakpoint on the fault handler (read-only; nothing written):

```
arm-none-eabi-gdb -q --batch build/MICROBIT \
  -ex "target remote :3333" -ex "monitor reset halt" \
  -ex "break HardFault_Handler" -ex "continue" ...
```

- Lands in `HardFault_Handler` (`gcc_startup_nrf52833.S:303`, a `b .` loop).
- `lr = 0xffffffe9` — EXC_RETURN, so the stacked frame is on MSP.
- **Stacked PC = `0xb240b580`** — not a valid flash or RAM address.
- **Stacked R3 = `0xb240b580`** — identical to the PC. The faulting
  instruction was a `blx r3`: **an indirect call through a corrupted
  pointer**, almost certainly a vtable dispatch.
- Stacked LR = `0x00021299` → `main + 544`, which resolves to
  [motor_armor.h:89](../../src/firm/devices/motor_armor.h#L89) (inlined).
  `MotorArmor` holds `Motor& inner_` and every accessor is a virtual call
  through it — consistent with a corrupted vtable pointer on the motor
  object or on whatever `inner_` now points at.
- Observed fault SPs: `0x2001fb70`, `0x2001fb88`.
- Bytes-before-death is **nondeterministic** (13 and 6 both seen across
  builds), so the fault lands at a varying moment relative to the serial
  DMA — an interrupt-time fault, not a fixed point in straight-line code.

## Why this is memory corruption and not a bounds bug

Two facts that have to be explained together:

1. **Neither ring indexes the other.** `cmdRing_[kCmdRingDepth]` is only ever
   indexed `% kCmdRingDepth` ([comms.cpp:173-185](../../src/firm/app/comms.cpp#L173-L185));
   `ackRing_[kAckRingDepth]` only `% kAckRingDepth`
   ([telemetry.cpp:165-215](../../src/firm/app/telemetry.cpp#L165-L215)). The
   documented "keep the two depths EQUAL" rule (telemetry.h:226) is about ack
   *observability* — violating it should silently drop acks, never fault.
2. **The change that breaks it SHRINKS static RAM** by roughly a kilobyte. A
   buffer getting smaller cannot overflow anything.

The only reading consistent with both is pre-existing corruption — a stray
write, a stale pointer, or a stack/heap collision — whose *victim* depends on
where objects land. At depth 12 the damage falls somewhere harmless; at 6 it
lands on something load-bearing. **12 is a value known to survive, not a
value known to be correct.** It will resurface the next time anything
perturbs static RAM: a new member, a resized buffer, a compiler change.

## Leads, ranked

**Layout facts for reference — NOT a lead in themselves.** (Two earlier
revisions of this document got this wrong, first as a fictitious "6 KB
heap/stack overlap" from misreading the phantom unplaced `.stack` section,
then by presenting the 2 KB MSP region as the headline suspect. Stakeholder
correction, on the record: this layout — heap sized to fill all remaining
RAM up to `__StackTop - 0x800`, `__StackLimit = 0`, ~2 KB free — is
**standard CODAL, identical in every working build for six months**. A
constant cannot explain the difference between a build that boots and one
that faults. Do not conclude "out of memory" or "grow the stack.")

The numbers, kept only because they are useful for placing watchpoints:

```
__end__          0x20005558        end of static data
.heap            0x20005558  size 0x1a2a8  -> ends 0x2001F800
__StackTop       0x20020000        MSP initial value; region above heap = 0x800
observed fault SPs: 0x2001fb70, 0x2001f988   (inside the MSP region, as normal)
```

The one legitimate use of the layout in this hunt: shrinking `.bss` by
~1.1 KB moves the heap's start, which shuffles which allocation lands where
— that is HOW one constant changes the corruption's victim, not WHY the
corruption exists.

**1. The actual lead: find the stray writer.** The bisect brackets it —
something in (or first *expressed* by) `5065775a`'s delta, or latent
earlier and merely re-aimed by the layout shuffle. In order:

- **Make it deterministic by padding.** `static volatile uint8_t pad[N];`
  in comms.h next to the ring; sweep `N` over 0..~1200 in steps. The fault
  should come and go with `N`, and the sweep brackets the victim's address
  range at each failing layout.
- **Then catch the writer in the act.** With a victim address from the
  sweep, set a data watchpoint over `pyocd gdbserver`
  (`watch *(uint32_t*)0x...`) and let the store trap. That names the bug in
  one shot — no theorizing required.
- While in there, audit `5065775a`'s own delta for the writer directly:
  `Drive::setWheelCorrection`/`correctedCommand` and the regenerated
  `boot_config.cpp` (the arrays themselves are `[2][2]` indexed 0/1 and look
  clean, but the regenerated file and its callers have not been audited as
  hard as the planner code has).

**2. Make it deterministic by padding.** Add `static volatile uint8_t
pad[N];` and sweep `N` over a few hundred bytes. If the fault appears and
disappears with `N`, layout-sensitivity is confirmed and the sweep brackets
the victim's address. That converts a Heisenbug into something a watchpoint
can catch.

**3. Then catch the writer.** With a suspect address from (2), set a data
watchpoint (`watch *(uint32_t*)0x...`) over a `pyocd gdbserver` session, or
enable an MPU guard region at the stack limit, and let it trap the store.

## Ruled out — please do not re-derive these

- **The motion-planner work on this branch is not the cause.** `5065775a`
  predates all of it and faults identically. Verified by building and
  flashing that commit on the same board in the same session.
- **RAM overflow from the planner change.** `.data 0x114`, `.bss 0x3358`,
  `.heap 0x1a2a8` are byte-identical between the planner branch and its
  baseline.
- **The flash/deploy path.** Fails identically via SWD (`mbdeploy`) and via
  the DAPLink mass-storage drive. All five flash regions read back
  programmed: MBR `0x0`, SoftDevice `0x1000`, app `0x1c000`, bootloader
  `0x77000`, settings `0x7e000` — nothing is erased or skipped.
- **Wrong board.** The `MICROBIT` drive's `DETAILS.TXT` UID matches the
  robot's, so this is not the relay being flashed by mistake.
- **Dangling reference to a `main()` local.** Every device stores its config
  **by value**, not by reference: `NezhaMotor::config_` (nezha_motor.h:176),
  `RealOtos::config_` (otos.h:298), `LineSensorLeaf::config_`
  (line_sensor.h:77), `ColorSensorLeaf::config_` (color_sensor.h:100).
  `Motion::Planner` likewise copies `PlannerLimits limits_`. So although
  `otosConfig`/`plannerLimits`/`motorConfigs[]` in `main()` are plain locals
  (unlike `colorConfig`/`lineConfig`, which are `static`), nothing retains a
  reference to them.
- **Use-after-free of the `ManagedString` in the banner send.**
  `SerialPort::sendReliable` builds a refcounted heap `ManagedString`, passes
  it to `_serial.send(s, ASYNC)`, and lets it destruct on return — which
  looks exactly like a UAF feeding the DMA. It is not: CODAL's
  `Serial::send(uint8_t*, int, ASYNC)` **copies** into its own `txBuff`
  (`Serial.cpp:356`, "ASYNC - bytes are copied into the txBuff").
- **`radio.send()` on the line after the banner.** Suppressing it entirely
  did not help — the byte count actually varied *down* (6 instead of 13),
  which is what established the nondeterminism.

## A different failure with a similar surface — do not confuse them

With `kCmdRingDepth` at 12 the robot boots, streams the banner and ~2 `TLM:`
frames, and then goes silent again. **That is a separate, already-documented
condition**, not this bug — and, importantly, the bisect captures show it
was present at EVERY commit tested (even the "boots" rows produced only 2
frames across a 24 s capture, when a healthy loop at ~21 Hz would produce
hundreds). The two failures were stacked, which is what made this session
so confusing.

Confirmed by read-only backtrace on the live board, frame-for-frame the
signature in `.clasi/knowledge/silent-robot-dead-external-i2c-bus`:

```
#0 codal::system_timer_wait_cycles            <- spinning forever
#2 codal::NRF52I2C::waitForStop               (NRF52I2C.cpp:241)
#3 Devices::MicroBitI2CBus::read
#4 Devices::NezhaMotor::readEncoderAtomicRaw
#5 Devices::NezhaMotor::hardReset
#6 App::Preamble::probeSlot -> RobotLoop::boot -> main
```

Per that knowledge note, the previous occurrence of this exact signature
was the motor brick's battery being dead — no power on the external I2C
bus's pull-ups, so the first motor probe spins forever with IRQs masked
(total silence, not a fault flag). The bus demonstrably worked earlier the
same day (the 600-trial speed calibration ran over it), so the brick lost
power somewhere between that session and this one. **Check the brick's
power switch / battery before any further software debugging of the
silence.** Discriminate the two failures in one command: halt and read PC —
in **flash** at `HardFault_Handler` → this bug; in **RAM** (a CODAL
busy-wait) → the dead bus.

## Disclosure about the board's state

During diagnosis I wrote `UICR.REGOUT0 = 5` on this micro:bit. Context: an
earlier flash hit an APPROTECT-locked chip, `mbdeploy` did a CTRL-AP mass
erase to recover, and that erase wiped UICR — including `REGOUT0`, which our
`MICROBIT.hex` does not program (it only sets `NRFFW[0]/[1]` at
`0x10001014/18`). Erased, the regulator defaults to 1.8 V instead of the
micro:bit V2's 3.3 V. I restored the factory value.

It was **not** the cause of this fault (restoring it and power-cycling
changed nothing), and writing it was the wrong call — flagged here so nobody
is surprised by a UICR value that did not come from the build. The
stakeholder's standing instruction is that hardware registers are for
*reading* during diagnosis and are never to be written.

## Verification for any candidate fix

Build, flash, and capture raw serial through the reboot as in
**Reproduction**. A pass is 97 bytes beginning
`DEVICE:NEZHA2:robot:tovez:` followed by at least one `TLM:` frame; a fail
is any truncation.

**No existing test catches this.** Nothing under `src/tests/` flashes
hardware, and the sim builds a host binary with an entirely different RAM
layout, so an ARM-only layout fault is invisible to the whole suite. A real
fix should come with a boot-gate that runs on the board.

## Related

- Blocks the velocity-trim hardware run:
  `rebuild-the-motion-planner-core-per-wheel-distance-profiling-velocity-domain-pid.md`.
- `.clasi/knowledge/silent-robot-dead-external-i2c-bus` — the look-alike
  failure above.
- Commit `9eca7b0b` — the tourniquet, with the same warning inline in
  `comms.h`.
