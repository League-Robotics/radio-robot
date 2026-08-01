---
status: pending
extends: square-tour-is-the-one-system-test-sim-bench-playfield.md
---

# Standing tests outside the system test: the explicit, minimal set

## Description

The tour-based system test
([`minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md`](minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md))
absorbs most end-to-end coverage. This issue enumerates **every standing test
that exists outside it** — a deliberately minimal set, each entry naming the
single invariant it proves and the venue it runs at (sim / bench /
playfield / host). Anything not on these lists is a **development test**:
scaffolding written while building a ticket, allowed to be deleted when it
stops earning its keep, and never part of the acceptance surface.

Stakeholder directive (2026-07-31): we have a test-bloat problem. Unit tests
are for development; throwaway tests belong in development. Standing tests
are few, and each is very specific about what it tests.

## The policy

1. **A standing test is on a list below, or it is not standing.** The lists
   are the acceptance surface. Nothing is auto-promoted by existing.
2. **Development tests are free during a ticket and disposable after.** They
   may stay in the tree while useful, but they carry no maintenance
   obligation, the sweep may prune them, and a red development test blocks
   nothing once its ticket closes.
3. **Promotion requires a named invariant.** A test joins a standing list
   only by stating the one specific thing it proves that no existing entry
   covers, recorded in this file. This extends the umbrella issue's
   no-new-system-tests rule to every layer.
4. **Characterization scripts are tools, not tests.** They produce
   calibration data for the robot JSON (plant ID, speed maps, deadband,
   OTOS drift), not verdicts. They live in **`src/tools/`** — physically
   outside the test tree — so they are structurally exempt from the test
   surface, not just labeled as such.

## The standing set

### Layer 1 — CI invariants (venue: sim/host, every change)

Eleven entries. Each is one invariant; the file(s) implementing it today are
noted for the sweep.

| # | Invariant | Today's file(s) |
|---|---|---|
| 1 | Frozen wire bytes: encoded frames match committed golden vectors, so a codec change that would break deployed firmware fails loudly | `test_wire_golden_vectors.py` |
| 2 | Wire robustness: fuzzed/corrupted/truncated input is rejected (CRC, COBS), never crashes, never false-accepts | `test_wire_fuzz.py` |
| 3 | Codec equivalence: the C++ and Python codecs produce identical bytes for identical messages — including messages newer than the golden vectors | `test_wire_differential.py` |
| 4 | The sim speaks the real wire: bytes injected through `sim_ctypes` traverse the same codec path as hardware (the sprint-123 hole stays closed) | `test_sim_wire_loopback.py` (relocate out of `sim/system/`) |
| 5 | Registry sync: the generated verb tables (C++ and Python) match `commands.proto` | `test_command_registry.py` |
| 6 | Boot config fails closed: a robot JSON missing required calibration keys fails the build; fwd_sign reaches the generated config | `test_gen_boot_config_required_keys.py`, `test_gen_boot_config_fwd_sign.py` |
| 7 | Unconfigured robot refuses to move: the configuration-completeness gate rejects MOVE before both motors are configured | `test_config_gate.py` |
| 8 | Sim/robot boot parity: the sim boots the same generated config the robot bakes (the composition-unification invariant) | `test_sim_boot_config_parity.py` (relocate) |
| 9 | The controller's spec: planner profile/shape/scenario/duty/noise suites — the executable definition of motion behavior | `src/motion/planner/tests/` (`motion_tests` build) |
| 10 | Host demux and ack matching: the binary/cleartext line split and corr_id→ack-ring joining behave per protocol v5 | `test_serial_conn_binary_plane.py`, `test_serial_conn_ack_ring.py` |
| 11 | The verdict engine: golden-trace comparison math, tour-file parsing, and the JSONL recorder (the system test's own foundations) | `test_golden_trace.py` + new `tourfile`/`recorder` tests |

Everything else currently under `src/tests/unit/`, `src/tests/sim/unit/`,
and `src/tests/sim/system/` — the per-module C++ harness battery
(`test_devices_*`, `test_app_*`, `test_motion_*`, `test_fake_transport`, …)
and the motion scenario pytest files — is reclassified as **development
stock**: it may run, it may rot, the sweep may prune it, and none of it
gates acceptance.

### Layer 2 — adversarial protocol harness (venue: sim AND bench, one scenario list)

Six scenarios — the races a sequential tour cannot create. One program, two
backends (`--sim` / `--port`), evolved from `move_protocol_bench.py` (today
hardware-only — porting it to `SimLoop` is this issue's main work item).

| # | Scenario | Proves |
|---|---|---|
| 1 | Queue full | 6th enqueue → `ERR_FULL`; the first 5 execute in order |
| 2 | Preemption | `replace=True` mid-move supersedes within one cycle; superseded move's ack reports it |
| 3 | Planned stop ordering | `stop()` queues behind the active move and does not interrupt it |
| 4 | Mid-move CONFIG | a config patch applies during an active move without disturbing it; ack in ring |
| 5 | Ack durability | rapid-fire N commands → every corr_id appears in the ack ring despite ring wrap (repeat delivery) |
| 6 | Timeout backstop | an unreachable stop condition ends the move at `timeout` with flags bit 15 set |

The smoke scenarios `move_protocol_bench.py` also carries today (basic move,
encoders climb, chaining) are retired to tours.

### Layer 3 — the system test (venue: sim, bench, playfield)

Owned by its own issue; the standing artifact list is exactly the committed
tours plus their per-tier goldens:

| Tour | Proves |
|---|---|
| `square.tour` | motion accuracy, planned stops, closure, STATUS health, camera-verified end position |
| `circle.tour` | arc tracking, gated behind square closure |
| `fault_wedge.tour` | injected wedge raises and clears bit 7; motion machinery responds correctly |

(plus one fault tour per additional `dbg` fault as those land — each new tour
is a deliberate addition under the umbrella issue's rule, not a default.)

Retired by this layer: `square_tour.py`, `wheels_square_tour.py`,
`planner_square_tour.py`, `twist_drive.py`, `curve_stream.py`,
`move_accuracy_bench.py`, `fake_otos_tour_bench.py`, and the motion
scenarios in `src/tests/sim/system/`.

### Layer 4 — standing gates and procedures (venue: as listed)

Five entries. Each has one job.

| # | Test | Venue | Proves | Cadence |
|---|---|---|---|---|
| 1 | `radio_bench_gate.py` (as-is) | bench, relay port | link establishment + wire quality against the stated loss budget — the transport the playfield tier rides on | firmware releases touching comms; radio work |
| 2 | estop latency check (new, small) | bench | estop sent mid-leg drops the active flag within one cycle and travel stays under threshold — the number every fence and Ctrl-C path depends on | firmware releases touching motion/comms |
| 3 | power-cycle persistence (manual procedure, written down) | bench | pushed tuning survives a power cycle and reapplies; schema bump wipes | releases touching config/persistence |
| 4 | physical wedge repro (existing repro recipe) | bench | the real 0x46 reversal-train wedge still matches what `DBG wedge` injects — fault-injection fidelity | only when wedge/armor machinery changes |
| 5 | `systest preflight --tier playfield` (new, part of systest) | playfield | lights on, camera up, tag 1 and tag 100 visible — environment failures get environment error messages, not CAMFIX failures | before every playfield session |

Host-software acceptance keeps exactly one standing entry:
`test_gui_button_acceptance.py` (headless TestGUI button surface) — the rest
of the GUI/gamepad tests are development stock.

### Tools (not tests — moved to `src/tools/`)

Stakeholder directive (2026-07-31): move these out of the test tree into
**`src/tools/`**, so "tool" is a location, not an annotation, and the
deletion sweep never has to reason about them.

Moving from `src/tests/bench/`: `plant_id.py` (from
`src/motion/planner/bench/`), `speed_map.py`, `duty_sweep.py`,
`crawl_sweep.py`, `velocity_step_response.py`, `otos_drift.py`,
`otos_calibration_bench.py`, `turn_prediction_capture.py`, `tlm_log.py`,
`wire_truth.py`, `link_check.py`, `relay_telemetry_rate.py`, and the
soak/rig scripts (`move_soak.py`, `rig_*.py`, `friction_rig_soak.py` — rig
quarantined per standing directive).

Move notes:
- `tlm_log.py` is a library as well as a CLI — `estimator_capture.py`,
  `turn_prediction_capture.py`, and the unit test `test_tlm_log.py` import
  from it, and the system test's recorder reuses its field conventions.
  The move updates those imports; `test_tlm_log.py` follows its subject.
- `src/tests/tools/golden_trace.py` is NOT part of this move — it is the
  system test's verdict library (Layer 1 #11), not a bench tool.
- Committed reference data moves with its producer
  (`src/tests/bench/data/` entries whose producing script moves).

## Work items

1. Port the Layer-2 scenario list to run against `SimLoop` as well as
   hardware (one scenario list, `--sim`/`--port`).
2. Write the estop-latency bench check (Layer 4 #2).
3. Add `systest preflight --tier playfield` (Layer 4 #5).
4. Relocate `test_sim_wire_loopback.py` and `test_sim_boot_config_parity.py`
   out of `sim/system/` before that directory's sweep.
5. `git mv` the tools list to `src/tools/` with import fixes (see move
   notes above); write the power-cycle procedure down (Layer 4 #3).
6. Record the standing-vs-development policy where it will be read
   (`src/tests/CLAUDE.md`), alongside the umbrella issue's
   no-new-system-tests rule.

## Verification

- The deletion sweep runs against these lists: every removed test is either
  named in a "retired" note above, reclassified as development stock, or its
  loss is recorded deliberately.
- CI runs exactly the Layer-1 list plus whatever development stock still
  passes; only Layer-1 failures block.
- The Layer-2 program passes the same six scenarios under `--sim` (CI) and
  `--port` (stand).
- Layer 4: estop check prints and gates its two numbers; preflight failure
  messages name the environment fault (lights/camera/tag), verified by
  unplugging each.

## Related

- [`square-tour-is-the-one-system-test-sim-bench-playfield.md`](square-tour-is-the-one-system-test-sim-bench-playfield.md)
  — the charter; its open question 6 ("what coverage genuinely dies") is
  answered by these lists.
- [`minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md`](minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md)
  — Layer 3's mechanism; its `at=<s>` extension would let the estop-latency
  check become a tour line.
- [`unify-sim-and-robot-composition-roots.md`](unify-sim-and-robot-composition-roots.md)
  — Layer 1 #8 (boot parity) is that issue's standing guard.
