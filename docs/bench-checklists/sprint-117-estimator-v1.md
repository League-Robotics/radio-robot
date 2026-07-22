# Sprint 117 Bench Checklist — Predict-to-Now Estimator v1: RESULTS

> # AGENT-EXECUTED, 2026-07-22 — real hardware reachable, motor/OTOS bus STILL
> # down (same signature as sprint 116's gate) — sim-mode capture substituted
>
> Robot `tovez`, UID `9906360200052820a8fdb5e413abb276000000006e052820`, on
> `/dev/cu.usbmodem2121102`, on the stand, wheels clear of the surface, per
> [`hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md).
> This document is the RESULTS record ticket 117-008 asks for, not a TODO
> list.
>
> **Headline finding**: this session's gate-order check (bus health BEFORE
> any drive command, per this ticket's own Implementation Plan) found the
> motor I2C bus **still disconnected** — `conn_left`/`conn_right`/
> `otos_present` all `False` immediately on boot, `flags=2240` (bits 6/7/11 —
> `kFlagFaultI2CSafetyNet` + `kFlagFaultWedgeLatch`, a side effect of the
> disconnected bus, not a real reversal-write-train latch + the sticky
> `kFlagEventBootReady`), the **exact same signature** documented in
> `clasi/issues/bench-motor-bus-disconnect-during-116-gate.md` and sprint
> 116's own checklist. This is the physical/electrical condition that ticket
> needs a stakeholder reseat to clear — **not** something this ticket (or
> any software/firmware change) can fix, and **not** attributed to a
> power/battery cause (bus-connectivity observation only, per project
> convention). **No drive command was issued** before or after this
> check — the bus-health read is entirely passive telemetry.
>
> Per this ticket's own contingency (`Implementation Plan`, item 3): the
> real-hardware capture path (`estimator_capture.py` against serial +
> `estimator_validation.ipynb` against the resulting bench CSV) is
> **BLOCKED** by the same physical condition. The sprint's own dataset for
> this gate is a **sim-mode capture substitute** (`estimator_capture.py
> --sim` + a full `estimator_validation.ipynb` re-execution) — recorded
> explicitly as a substitution below, never presented as real bench data.
> One thing this session DID confirm live against real firmware: **ticket
> 001's `PING` reply now carries `t=<ms>`** — round-tripped successfully
> multiple times over the real serial link (see §1).

## Setup

```bash
mbdeploy probe
# confirmed: UID 9906360200052820a8fdb5e413abb276000000006e052820, ROLE=NEZHA2,
# NAME=robot, port /dev/cu.usbmodem2121102 -- live truth from the probe registry
# (several other rows share that same port; they are stale cached entries from
# other previously-probed devices, not this session's target -- see
# clasi knowledge entry "Verify micro:bit before flashing")

just build-clean
# python build.py --clean: firmware hex v0.20260722.1, FLASH 138036B/364KB=37.03%,
# RAM 120768B/122816B=98.33% (normal -- CODAL RAM is always near-full by design),
# host sim lib (libfirmware_host.dylib) built clean

mbdeploy deploy 9906360200052820a8fdb5e413abb276000000006e052820 --hex MICROBIT.hex
# flashed successfully on the first attempt this session (no mass-erase
# recovery needed) -- 291840 bytes programmed at 15.76 kB/s
```

Boot banner confirmed via `SerialConnection.connect()`'s own HELLO-classify:
`DEVICE:NEZHA2:robot:tovez:2314287040` (role/common_name/device_name/serial
all as expected, matching the probed UID's registry entry).

---

## 1. PING `t=<ms>` live round-trip (sprint 117 ticket 001)

Ticket 001 shipped `Comms::pumpTransport()`'s `PING` reply as `OK pong
t=<ms>` (previously a bare `OK pong`). Verified against the REAL, just-flashed
firmware over the real serial link — `SerialConnection.connect()`'s own
readiness poll (`_poll_ready()`) sends a raw, un-suffixed `PING\n` as part of
every `connect()` call, so this round-trip needs no workaround for the
separate, already-documented `SerialConnection.send()` corr-id-suffix gap
(`send()` appends `#<corr_id>` to every command, which breaks an exact-string
match like plain-text `PING` — flagged by ticket 001 itself,
`src/host/robot_radio/DESIGN.md` §6 — **not** worked around here, per this
ticket's own instructions; a live round trip through `send()` remains
blocked pending that separate fix).

| Check | Result |
|---|---|
| `connect()` readiness poll's raw `PING` → `OK pong t=<ms>` | [x] PASS — observed `OK pong t=11204`, `OK pong t=19960`, and `OK pong t=12040` across three separate `connect()` calls this session, each a distinct, increasing robot-clock value |
| Reply shape matches `docs/protocol-v4.md` §2.4 (post-001) | [x] PASS |
| Round trip via `NezhaProtocol.send("PING")` (the corr-id-suffixed path) | **NOT ATTEMPTED** — pre-existing, already-documented gap (see above); documented, not hacked around |

---

## 2. Motor/OTOS bus health check — BEFORE any drive command

Per this ticket's own gate order ("check hardware presence and motor-bus
health FIRST... before attempting to drive anything") and
`.claude/knowledge/disconnected-bus-signature-tlm-conn` — a passive read of
10 binary TLM frames immediately after connect, **zero drive commands
issued**:

| Check | Result |
|---|---|
| `flag_conn_left` (`kFlagConnLeft`, bit 3) | **`False`** — motor bus NOT connected |
| `flag_conn_right` (`kFlagConnRight`, bit 4) | **`False`** — motor bus NOT connected |
| `flag_otos_present` (`kFlagOtosPresent`, bit 0) | **`False`** — OTOS not present (shares the same bus) |
| `flags` raw value | `2240` = `0x8C0` = bits 6 (`kFlagFaultI2CSafetyNet`, boot-time, benign) + 7 (`kFlagFaultWedgeLatch`, a SIDE EFFECT of the disconnected bus — `Devices::Motor`'s wedge detector fires on any unchanged-position run, which a dead bus also produces) + 11 (`kFlagEventBootReady`, sticky-not-pulsed per sprint 116's own finding) |
| Signature matches sprint 116's / the tracked issue's documented pattern | [x] YES — identical `conn_left`/`conn_right`/`otos_present` all `False`, identical `flags=2240` |

**Verdict: bus is STILL DOWN.** Per this ticket's gate logic, no drive
command was issued (real `estimator_capture.py` against serial was NOT run)
— proceeding straight to the sim-mode substitute path (§3) instead, exactly
as the ticket's Implementation Plan anticipates for this contingency.

This is a physical/bench-hardware condition outside this session's control
(same root cause tracked in
`clasi/issues/bench-motor-bus-disconnect-during-116-gate.md`, unresolved
since sprint 116 — needs a stakeholder reseat of the brick/OTOS I2C
connector and power rail on the stand). Sprint 117 touched no motor-bus/I2C
driver code (its firmware changes are `App::StateEstimator`,
`App::Comms::pumpTransport()`'s `PING` reply, `App::RobotLoop` wiring, and
config/fusion-weight plumbing) — this is not a regression from this
sprint's own work.

---

## 3. Sim-mode capture substitute (contingency path)

**Explicitly recorded: the numbers below are from a SIMULATED plant
(`SimLoop`/`SimPlant`), not real hardware.** They are a second, independent
confirmation run of the same pipeline ticket 007 already committed
(`estimator_validation.ipynb`'s own default `CSV_PATH=None` self-capture
path) — not silently presented as bench data anywhere in this document.

### 3a. Standalone capture artifact

```bash
uv run python src/tests/bench/estimator_capture.py --sim \
    --csv src/tests/bench/out/estimator_capture_sprint117_sim.csv
```

```
sim connected: firmware=0.20260722.1 track_width=128.0 robot=data/robots/tovez_nocal.json
capturing sim pattern (8 segments) -> src/tests/bench/out/estimator_capture_sprint117_sim.csv
wrote 134 rows to src/tests/bench/out/estimator_capture_sprint117_sim.csv
```

| Check | Result |
|---|---|
| `wrote <N> rows`, N > 0 | [x] PASS — 134 rows over the 8-segment `DEFAULT_PATTERN` (~9.5s) |
| Configured against `data/robots/tovez_nocal.json` (fail-closed `ERR_NOT_CONFIGURED` gate satisfied) | [x] PASS |

Committed at `src/tests/bench/out/estimator_capture_sprint117_sim.csv` as a
durable capture artifact (path convention matching `tlm_log.py`'s own
`DEFAULT_CSV` precedent).

### 3b. Notebook re-execution (`estimator_validation.ipynb`)

```bash
uv run jupyter nbconvert --to notebook --execute --inplace \
    src/tests/notebooks/estimator_validation.ipynb
```

Ran end-to-end with **no source-cell changes** (`CSV_PATH` left at its
committed default of `None`, which triggers the notebook's own fresh
`estimator_capture.py --sim` internal capture — 134 rows, same pattern —
written to `src/tests/notebooks/out/estimator_validation_capture.csv`, also
committed). Confirmed via `git diff`: only output cells and the capture CSV
changed, zero source-cell edits.

**RMS one-step-ahead residual, by stream × phase** (§4 of the notebook):

| stream | ramp | steady | reversal | pivot |
|---|---|---|---|---|
| `enc_left_position` [mm] | 1.8315 | 0.0748 | 1.3369 | 1.3311 |
| `enc_left_velocity` [mm/s] | 28.9067 | 1.2210 | 26.3379 | 20.5346 |
| `enc_right_position` [mm] | 1.7893 | 0.0748 | 1.6038 | 1.5090 |
| `enc_right_velocity` [mm/s] | 27.9639 | 1.2210 | 31.6320 | 24.2944 |
| `heading` [rad] | 0.0007 | 0.0000 | 0.0005 | 0.0070 |

Order-of-magnitude consistent with ticket 006's own independent verification
run (wheel RMS ~1.6–1.8mm, heading RMS ~0.003rad on an unbucketed walk) and
ticket 005's C++ sim-system harness — smallest in `steady` (settled
tracking), largest in `ramp`/`reversal` (transient onsets), as expected.

**ZOH lag-signature check** (`forward_step` ramp window, [2400, 2850]ms, 7
steps, avg dt=57.1ms, `a`=332.1 mm/s²):

| | theory | measured | ratio | verdict |
|---|---|---|---|---|
| velocity error [mm/s] | 18.975 | 42.052 | 2.22× | **PASS** (within 3×) |
| distance error [mm] | 0.5421 | 2.8542 | 5.26× | **FAIL** (exceeds 3×) |

The notebook's own reading (§ "Reading the verdict"): the sim plant's
`forward_step` onset is dead-time-then-near-step, not a smooth ramp — most
of the ramp window's samples land during zero-velocity dead time, then the
whole velocity change compresses into one or two samples once the plant
releases, concentrating residual atypically for the classical `a·k`/`½·a·k²`
constant-acceleration formula. This is evidence AGAINST a fit-based ramp
predictor buying much over ZOH here, not a sign the estimator itself is
broken.

**Leg-level projection** (random-walk `√N` bound, NOT a literal
dead-reckoning claim — see notebook §7 for why):

| leg | basis | per-step RMS | steps | projected error |
|---|---|---|---|---|
| `forward_step` (straight, 1500ms) | steady position | 0.0748 mm | 24 | 0.3663 mm |
| `pivot_ccw` (pivot, 1200ms) | pivot heading | 0.006978 rad (0.3998°) | 19 | 0.030415 rad (1.7426°) |

**PROPOSED accept thresholds — NOT RATIFIED** (2× measured basis-phase RMS;
notebook §8 — this is the notebook's own documented judgment call, not a
stakeholder-ratified rule):

| stream | basis phase | measured RMS | proposed threshold |
|---|---|---|---|
| `enc_left_position` [mm] | steady | 0.0748 | 0.1495 |
| `enc_left_velocity` [mm/s] | steady | 1.2210 | 2.4420 |
| `enc_right_position` [mm] | steady | 0.0748 | 0.1495 |
| `enc_right_velocity` [mm/s] | steady | 1.2210 | 2.4420 |
| `heading` [rad] | pivot | 0.0070 | 0.0140 |

**These thresholds are PROPOSED ONLY, from ONE simulated dataset (one seed,
sim plant dynamics only).** Ratifying, rejecting, or retuning them against
real bench data is the stakeholder's own call — see §5 below for exactly how
to re-run this gate once the bus is confirmed recovered.

---

## 4. Final sweep (sprint-closing verification)

```bash
just build-clean
# python build.py --clean: firmware hex v0.20260722.1, FLASH 37.03%, RAM 98.33%,
# host sim lib built clean -- PASS (run at the top of this session, §Setup)

uv run python -m pytest
# 1242 passed, 13 skipped, 10 xfailed, 1 xpassed, 1 warning in 133.40s -- PASS
```

The one warning (`PytestUnhandledThreadExceptionWarning` in
`test_set_origin.py`, an `AttributeError` on `telemetry_pb2.ACK_STATUS_DONE`
inside a background TestGUI worker thread) is pre-existing, unrelated to
this ticket's own changes (no `.proto`/TestGUI/tour code touched by this
gate), and does not fail the run (the test itself still passes; only a
background thread's own exception surfaces as a warning).

| Check | Result |
|---|---|
| `just build-clean` | [x] PASS |
| `uv run python -m pytest` (full suite) | [x] PASS (1242 passed, 0 failed) |

---

## 5. Stakeholder re-verification — run once the bus is confirmed reseated

The motor/OTOS I2C bus needs a physical reseat
(`clasi/issues/bench-motor-bus-disconnect-during-116-gate.md`) before any of
this section is runnable. Once `conn_left`/`conn_right`/`otos_present` read
`True` on a passive TLM check (repeat §2 above FIRST, with no drive command,
before proceeding):

```bash
# 1. Deploy the current sprint firmware (already built this session; rebuild
#    if time has passed / other work has landed on this branch).
just build-clean
mbdeploy deploy 9906360200052820a8fdb5e413abb276000000006e052820 --hex MICROBIT.hex

# 2. Confirm the bus is live (passive read -- NezhaProtocol.read_pending_binary_tlm_frames(),
#    check frame.conn_left / frame.conn_right / frame.otos_present are all True;
#    do NOT proceed to step 3 until they are).

# 3. Real capture over serial (bounded, TIME-stop MOVEs only, per
#    hardware-bench-testing.md -- the same DEFAULT_PATTERN this session's sim
#    run used, this time against the real robot):
uv run python src/tests/bench/estimator_capture.py \
    --port /dev/cu.usbmodem2121102 \
    --csv src/tests/bench/out/estimator_capture_sprint117_bench.csv

# 4. Point the notebook at the REAL captured CSV -- edit ONLY the CSV_PATH
#    parameter cell (cell "Parameters"), per that cell's own documented
#    contract ("Set to a bench-captured CSV path (ticket 008) to validate
#    against real hardware instead -- no other cell needs to change"):
#      CSV_PATH = "<repo_root>/src/tests/bench/out/estimator_capture_sprint117_bench.csv"
#    then re-execute headless:
uv run jupyter nbconvert --to notebook --execute --inplace \
    src/tests/notebooks/estimator_validation.ipynb

# 5. Compare the REAL RMS tables / lag-signature verdict / proposed
#    thresholds (notebook §4/§5/§8) against this document's §3b sim numbers.
#    Revert CSV_PATH to None afterward if the notebook's own default
#    self-capture behavior should be preserved for the next run.
```

Then review and ratify (accept, reject, or retune) the proposed accept
thresholds table in §3b above against the REAL numbers produced by step 4-5
— that decision belongs to the stakeholder, not to this checklist or the
agent that produced it.

---

## Related

- `clasi/issues/bench-motor-bus-disconnect-during-116-gate.md` — the tracked
  physical issue this session's bus-health check re-confirmed, unresolved.
- `docs/bench-checklists/sprint-116-move-protocol.md` — the prior session's
  own checklist, same bus-down signature.
- `docs/knowledge/2026-07-04-encoder-wedge.md` — wedge-latch vs.
  disconnected-bus signature discussion.
- `docs/protocol-v4.md` §2.4 — `PING` reply shape (post sprint-117-001).
