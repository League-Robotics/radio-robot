---
id: "006"
title: "Pytest sim tier: scripted-twist demo, fault scenarios, conftest fix"
status: open
use-cases: [SUC-023]
depends-on: ["004", "005"]
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Pytest sim tier: scripted-twist demo, fault scenarios, conftest fix

## Description

This is the sprint's own Definition of Done. `tests/sim/conftest.py`
currently references deleted infrastructure (`tests/_infra/sim`,
`firmware.py`'s `Sim` class, a `just build-sim` recipe the `justfile` no
longer defines — confirmed by reading both files directly) — any test
depending on its `sim`/`build_lib` fixtures fails immediately, though
nothing in the current tree actually calls them. `tests/sim/system/` is
still the 077-006 skeleton, never populated.

This ticket: (1) removes or replaces `tests/sim/conftest.py`'s stale
fixtures (architecture-update.md Step 7 Open Question 2 — ticket-time call
on whether a shared `sim_api`-backed fixture is worth adding, given the
established ad hoc per-file compile convention `test_app_drive.py` already
uses); (2) adds the headless scripted-twist demo scenario under
`tests/sim/system/` — boot → connect (via `FakeTransport`) → arm a
scripted twist → step enough cycles to observe simulated encoder motion and
telemetry → send stop → confirm convergence to zero velocity — runnable
both under pytest AND standalone as this sprint's own stakeholder-visible
"run one command and see the sim loop move" proof; (3) wires ticket 005's
three fault-injection scenarios into the pytest tier (if not already
collected as part of ticket 005's own scope — confirm and close any gap);
(4) updates `tests/CLAUDE.md`'s `sim/` section, which currently states "a
fresh simulator harness for the new `source/` tree does not exist yet."

## Acceptance Criteria

- [ ] `tests/sim/conftest.py` contains no reference to deleted
      infrastructure (`tests/_infra/sim`, `firmware.py`, `build-sim`) —
      either the stale fixtures are removed outright, or replaced with a
      `sim_api`-backed equivalent, per the ticket-time call documented in
      the code/commit message.
  - [ ] `git grep -n "_infra/sim\|firmware import Sim\|build-sim" tests/`
        returns no remaining reference (outside historical comments
        explicitly marked as such).
- [ ] A new `tests/sim/system/` scripted-twist scenario test: boot → twist
      → observe ramping encoder/telemetry motion → stop → observe
      convergence to zero velocity — green under
      `uv run python -m pytest`.
- [ ] The scripted-twist demo is runnable standalone (a documented command
      in the test file's own docstring or a short `tests/sim/system/
      README.md` update, e.g. direct invocation of the compiled harness
      binary or a thin `uv run python <script>.py` wrapper) and prints a
      human-readable trace of commanded vs. observed motion (at minimum:
      cycle number, commanded v_x/omega, observed encLeft/encRight/
      velLeft/velRight per some reporting interval).
- [ ] Ticket 005's three fault-injection scenarios are confirmed collected
      and green under the default `uv run python -m pytest` invocation
      (not requiring a special flag/marker to run) — close any collection
      gap found.
- [ ] `uv run python -m pytest` is fully green end to end — the pre-105
      561-test baseline plus every test added by tickets 001-005 plus this
      ticket's own new tests, zero regressions.
- [ ] `tests/CLAUDE.md`'s `sim/` section is rewritten to describe what
      actually exists after this sprint (the `RobotLoop` extraction,
      `sim_api`, the plant, `tests/sim/plant/` and `tests/sim/system/`'s
      real contents) — no longer states the harness "does not exist yet."

## Testing

- **Existing tests to run**: the FULL suite —
  `uv run python -m pytest` (this ticket's own acceptance bar is that this
  command is completely green).
- **New tests to write**: `tests/sim/system/test_scripted_twist_demo.py`
  (or similarly named) — the headless demo scenario.
- **Verification command**: `uv run python -m pytest` (no path filter —
  the whole-suite green run IS this ticket's verification).

## Implementation Plan

**Approach**: this is primarily an integration/cleanup ticket, not new
architecture — it consumes `sim_api` (ticket 004) and the fault knobs
(ticket 005) as already-built primitives. The scripted-twist demo script
should read naturally as a narrative: connect, arm, watch it ramp, stop,
watch it settle — printed output a stakeholder can read without knowing
pytest internals, matching this sprint's own "bench-runnable-equivalent"
framing (`.claude/rules/hardware-bench-testing.md`'s spirit, applied
headless). For the `conftest.py` decision: given every existing
`tests/sim/unit/` test already compiles its own throwaway binary ad hoc
with no shared fixture, the DEFAULT bias should be toward simply deleting
the dead `sim`/`build_lib` fixtures rather than inventing a new shared one
— only add a shared fixture if writing 2+ new scenario files without one
proves genuinely repetitive at ticket-execution time.

**Files to create**:
- `tests/sim/system/test_scripted_twist_demo.py` (the headless demo +
  pytest wrapper).
- `tests/sim/system/README.md` update (currently states "Skeleton only...
  Populated once the sim harness... lands in a later ticket" — replace
  with real usage instructions).

**Files to modify**:
- `tests/sim/conftest.py` — remove/replace stale fixtures.
- `tests/CLAUDE.md` — `sim/` section rewrite.
- Any fault-injection test file from ticket 005 found NOT already collected
  under the default pytest invocation.

**Testing plan**: the full-suite green run described above IS the testing
plan; no bench gate needed (no ARM/production code touched by this
ticket).

**Documentation updates**: `tests/CLAUDE.md` (sim/ section) and
`tests/sim/system/README.md`, both listed above as files to modify/create.
