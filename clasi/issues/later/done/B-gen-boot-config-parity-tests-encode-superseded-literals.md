---
status: done
priority: medium
tickets:
- '001'
---

# Three `gen_boot_config` parity tests encode superseded literals

## Description

Three tests assert generated boot config against hardcoded literals that were
correct when written and have since been superseded by measurement:

- `test_gen_boot_config_planner.py::test_planner_config_for_config_reads_tovez_json`
- `test_gen_boot_config_planner.py::test_generate_emits_default_planner_limits_byte_identical_to_pre_ticket_literals`
- `test_gen_boot_config_robot_groups.py::test_default_drive_group_matches_tovez_json`

They have been failing since sprint 132-019 baked `tovez`'s bench-fitted
wheel gains, and sprint 133-004 moved the same file again when it promoted
`pid_ki 6.0`, `pid_i_max 60.0`, `pos_err_max 10.0`, `pid_max 100.0`.

## Why this is not just "update the numbers"

The second test's own name — `..._byte_identical_to_pre_ticket_literals` —
says what it is: a **change-detector** pinned to one historical moment. That
is a reasonable thing to write during a refactor, to prove a mechanical
change moved nothing. It is not a reasonable thing to keep afterwards,
because every legitimate re-fit of a robot's calibration now fails it.

`tovez.json` is expected to change whenever the robot is re-measured. That is
the entire point of the configuration-discipline work: the file is the single
authority, and bench sessions update it. A test asserting the file still holds
a specific historical value is asserting that measurement never happens.

Sprint 133-005 also found the mirror-image failure — `test_configurator_loadbaked`
had encoded a *defect* as a requirement (`wheel_gain_left_accel == 0.9075`)
and so failed the moment the defect was fixed. Same family: a parity test
that pinned a value instead of a property.

## Proposed fix

Decide, per test, which property is actually being defended and assert that
instead of a literal:

- **Generator correctness** ("the generator faithfully emits whatever the JSON
  holds") — assert against values read from the JSON at test time, so a
  re-fit passes and a generator bug fails.
- **A genuine one-time refactor guard** — if a test's only purpose was to
  prove one historical change moved nothing, and that change has long since
  landed, **delete it.** It has no remaining job.

Do not simply refresh the literals to today's values; that reinstates the
same failure at the next bench session.

## Verification

- The three tests pass against current `tovez.json`.
- Changing a calibration value in `tovez.json` and regenerating does **not**
  fail them, while a genuine generator defect still does.
- The full-collection failure set drops these three by identity.

## Related

- Sprint 133 ticket 004 — promoted the tuned values that moved the file again.
- Sprint 133 ticket 005 — found and fixed the mirror-image case in
  `test_configurator_loadbaked`.
- `.claude/rules/configuration-discipline.md` — the file is expected to
  change; tests must defend properties, not snapshots.
