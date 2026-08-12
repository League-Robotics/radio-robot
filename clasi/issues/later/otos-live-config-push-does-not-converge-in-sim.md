---
status: pending
priority: medium
filed: 2026-08-11
filed_by: "programmer (sprint 136 ticket 002, full-suite baseline triage)"
tickets:
- 136-002
---

# Pushing a compensating `SetConfigField{OTOS, linear_scale}` does not converge the sim's decoded OTOS reading back to truth

## Description

`test_otos_calibration_convergence.py::test_otos_calibration_push_converges_pose_via_the_real_config_path`
fails at its final assertion. Measured directly (2026-08-11, sprint 136
ticket 002 baseline triage):

- True pose `x = 1000mm`, injected raw OTOS linear scale error `+5%`.
- Uncalibrated reading: within tolerance of `1000 * 1.05 = 1050mm` (this
  assertion PASSES -- the fault-injection half of the test works as
  documented).
- After pushing the compensating multiplier (`1 / 1.05 ~= 0.9524`) via a
  `SetConfigField{OTOS, linear_scale}` envelope over the same wire path
  `SimTransport`'s own `OL`/`OA` verbs use, and confirming the ack lands
  (`ack.ok` -- this also PASSES): the next decoded OTOS reading is
  **1047mm**, expected `1000mm +/- 20mm (2%)`. The push essentially had NO
  measurable effect (1050 -> 1047 is not meaningfully closer to 1000).

## What has been ruled out (verified directly, not assumed)

- **Register quantization is NOT the cause.** `Hardware::scaleToRegister()`
  (`src/firm/hardware/generic/real_otos.cpp`) uses an `int8_t` register at
  0.001 multiplier per LSB (range 0.873..1.127). The compensating multiplier
  0.952381 quantizes to register -48, i.e. an applied multiplier of
  `1 - 0.048 = 0.952` -- within 0.04% of the intended value. This cannot
  explain a ~4.7% residual.
- **Not the 135-008 heading-sign fix** (`57b01f32`, "sim OTOS packs the
  hardware-mounted heading sign") -- that commit touches only the heading
  accumulator's sign, not the linear-scale/X-position path this test
  exercises.

## Leading hypothesis (NOT confirmed -- stated as a hypothesis, not a cause)

`TestSim::OtosPlant`'s own burst-read packing may not apply the pushed
`linear_scale` register value when computing what bytes to report back for
a subsequent read -- i.e. the SIM's fake OTOS chip may not model the
register-scale multiply a real chip applies in hardware before reporting,
so writing a compensating scale register has no effect on what the next
simulated burst read returns regardless of whether the write itself lands
correctly. This would be a **sim-fidelity gap**, not a firmware defect in
the real (`Hardware::RealOtos`) decode path. This has not been confirmed by
reading `otos_plant.cpp`'s burst-read implementation line by line, nor by
adding a temporary print/breakpoint to see what scale (if any) it applies --
that is the next actual debugging step, not done here because it requires
building and instrumenting the C++ sim harness, out of proportion for a
baseline-triage ticket whose job is triage, not root-cause-and-fix.

## What to do

1. Read `TestSim::OtosPlant`'s burst-read packing
   (`src/tests/sim/plant/otos_plant.cpp`) and confirm or refute whether it
   applies a written scale register to its own reported bytes.
2. If confirmed as a sim-fidelity gap: either model the multiply in
   `OtosPlant` (matching real chip behavior), or -- if intentionally
   unmodeled -- change this test's own premise (it currently asserts sim
   behavior a real chip should have but the sim plant may not implement).
3. If refuted (the plant DOES apply the scale correctly): re-investigate
   from scratch -- the residual would then indicate a genuine firmware
   defect in the push-to-application path (config-delta timing vs. the
   burst-read window, an ack that reports success before the value is
   actually installed, etc.).

## Verification

- `test_otos_calibration_push_converges_pose_via_the_real_config_path`
  passes: `calibrated_x` lands within `_TRUE_X +/- 2%`.
- The mechanism (sim-fidelity gap vs. firmware defect) is confirmed, not
  guessed, before any fix lands.

## Related

- `src/tests/testgui/test_otos_calibration_convergence.py` -- the failing
  test, and `test_tour_closure_gate.py`'s own OTOS-calibration-push
  precondent (uses the same mechanism to calibrate OUT injected error
  before driving a tour -- if this convergence gap is real, that gate's own
  calibration step may be quietly not working either, worth checking once
  this is root-caused).
- `clasi/issues/later/otos-fake-seam-should-be-one-interface-two-implementations.md`
  -- adjacent existing issue about the sim/real OTOS seam, not the same
  finding but the same general area of the codebase.
