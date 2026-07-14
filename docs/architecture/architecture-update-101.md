# Sprint 101 — Bench test-rig device debugging

## Goal

Exercise **every device of the DeviceBus subsystem** on the new stationary
bench test rig until motors (with the velocity PID) and all sensors — OTOS,
encoders, line, color — run **reliably**, with **no lock-ups**. This sprint is
device-level only: **no motion planning, no turns.** It exists because bench
diagnosis of the mobile robot found the DeviceBus firmware's OTOS heading dead
(frozen during an open-loop spin) and the fused heading garbage — we cannot
trust the higher stack until the device layer is proven on a controllable rig.

## The rig (see memory `bench-test-rig-layout`)

- **Motor 1** → a **drum** that simultaneously drives: **OTOS** (mounted above,
  reads drum surface as translation), the **line sensor** (black/white marks =
  binary count: ch1 index once/rev at 0, ch2/3/4 = 3-bit 0..7, 8 counts/rev),
  and the **color sensor** (painted wheel).
- **Motor 2** → **3 wheels, high inertia** — velocity-PID stress.
- **OTOS servo** on Nezha port **J1/S1** (PWM): rotates OTOS → changes heading;
  neutral projects drum motion on OTOS **X**, +90° on **Y**.

## Approach

- Drive the **device subsystem directly** (DeviceBus DEV command surface / the
  bring-up device commands), not through the motion planner.
- **Resurrect the old ports/PWM code** to add a servo/PWM command (port J1/S1)
  — the DeviceBus cutover has no PWM verb. This lets us command the OTOS servo
  and confirm OTOS heading tracks it.
- Motors must follow commands on the **PID with OTOS OFF** (decouple motor
  control from pose sensing).
- Surface line/color reads if the current device command surface doesn't
  (cutover left color/line unbridged).
- Deliverables are **notebooks + a soak test** (below) that a person or agent
  re-runs to prove reliability.

## Tickets

1. **Device command surface + ports/PWM servo** — ensure DEV commands reach
   motors (VEL/DUTY/PID/STATE), encoders, OTOS (incl. on/off), line, color;
   resurrect ports/PWM to drive the J1/S1 servo; confirm OTOS heading responds
   to servo motion.
2. **Motion Control notebook: motors on PID, no OTOS** — motors track sine +
   square velocity references via the PID with OTOS off; high-inertia motor 2
   included; encoders read reliably. Update the existing Motion Control
   notebook.
3. **Sensor exercise notebook** — driving motor 1, read the line-sensor 0..7
   count + ch1 index, the color-sensor colors, and OTOS X/Y (drum) + heading
   (servo). New notebook.
4. **Device soak test** — a scripted series of device cycles that continuously
   validates results (motors follow, encoders/OTOS/line/color plausible) and
   proves no errors/lock-ups over sustained running.

## Out of scope

Motion planning, turns, the `source/drive/` stack, field/playfield work. Those
resume only after the device layer is proven reliable here.
