---
sprint: '054'
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 054 Use Cases

## SUC-001: Motion verb range errors report the correct error code

- **Actor**: Host controller (Python `robot_radio`) or operator tool
- **Preconditions**: Robot firmware is running and accepting commands.
- **Main Flow**:
  1. Host sends a motion verb (S, T, D, or R) with an argument that falls
     outside the defined valid range for that field (e.g. `S 99999 0`).
  2. Firmware parse function detects the range violation and sets
     `err.code = "range"` and `err.detail = "<field>"`.
  3. The dispatcher formats `ERR range <field>` and sends it to the host.
  4. Host receives the reply and can distinguish a range violation from a
     structural argument-count error.
- **Postconditions**: Robot remains stopped; no motion is initiated for the
  invalid command.
- **Acceptance Criteria**:
  - [ ] `S 99999 0` → `ERR range l`
  - [ ] `S 0 99999` → `ERR range r`
  - [ ] `T 0 0 0` → `ERR range ms` (ms=0 below min of 1)
  - [ ] `D 0 0 0` → `ERR range mm` (mm=0 below min of 1)
  - [ ] `R 99999 0` → `ERR range speed`
  - [ ] `S` (no args) → `ERR badarg` (arg-count path unaffected)
  - [ ] `T 0 0` (two args, not three) → `ERR badarg`
  - [ ] Simulation test suite (`uv run pytest`) passes with live firmware calls
        asserting the exact `ERR <code> <field>` string for each case above.
