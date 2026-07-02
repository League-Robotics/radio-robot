# Post-Mortem: Quantitative Evidence

Data extracted from the git history and CLASI artifact tree on 2026-07-02.
All numbers computed from `git log` on branch
`sprint/065-...` (1,255 commits, 2026-05-20 → 2026-07-02).

## Timeline and cadence

- **Project span:** 2026-05-20 → 2026-07-02 — approximately 6 calendar weeks,
  with a pause 2026-05-23 → 2026-05-31.
- **Sprints:** 64 done + 2 active = 66 sprints. **372 tickets** total
  (~5.6 tickets/sprint).
- **Sprint duration is hours, not days.** From commit scopes:
  - Sprints 025–035 (11 sprints): all closed 2026-06-11 → 2026-06-12 (2 days).
  - Sprints 038–045 (the entire Phase 0→F architecture refactor, 8 sprints):
    all committed on **one day**, 2026-06-19.
  - Sprints 047–053 (7 sprints): all on 2026-06-28.
  - Sprints 055–061 (the message-architecture cutover, 7 sprints):
    2026-06-29 → 2026-06-30 (2 days).
- **Two epochs:** sprints 001–005 (May 20–23) built a first CODAL C++ firmware
  (~2,900 LOC); the project was then formally re-initiated on 2026-06-01
  ("Add feature specification and use cases for radio-robot-c") and sprints
  006+ proceeded under the full CLASI process.

## Commit mix

| type | count | share |
|---|---|---|
| chore | 638 | 50.8% |
| feat | 259 | 20.6% |
| untyped/other | 246 | 19.6% |
| fix | 44 | 3.5% |
| refactor | 30 | 2.4% |
| docs | 22 | 1.8% |
| test | 16 | 1.3% |

- **419 of 1,255 commits (33.4%) are `chore: bump version`** — pure process
  overhead, one for every ~2 substantive commits.
- The low raw `fix:` share is misleading: under the ticket process, fix work is
  labeled `feat(NNN-MMM)`. Sprint *names* are the better classifier (below).

## Commits per week (excluding version bumps)

| week | commits | feat | fix | refactor |
|---|---|---|---|---|
| 2026-W20 (May 18) | 25 | 17 | 1 | 0 |
| 2026-W22 (Jun 1) | 177 | 86 | 14 | 4 |
| 2026-W23 (Jun 8) | 231 | 61 | 17 | 8 |
| 2026-W24 (Jun 15) | 109 | 18 | 1 | 17 |
| 2026-W25 (Jun 22) | 112 | 11 | 0 | 0 |
| 2026-W26 (Jun 29) | 182 | 66 | 11 | 1 |

## Code size trajectory (LOC)

| date | firmware (`source/`) | host (`host/`) | tests |
|---|---|---|---|
| 2026-05-21 | 2,933 | 0 | 0 |
| 2026-06-05 | 7,355 | 20,368 | 9,357 |
| 2026-06-12 | 17,159 | 20,668 | 34,102 |
| 2026-06-19 | 19,816 | 17,383 | 51,066 |
| 2026-06-28 | 21,217 | 17,519 | 56,104 |
| 2026-07-02 | 24,890 | 24,536 | 73,506 |

- Final system ≈ **123k LOC**, of which tests are **60%**.
- Host LOC *shrank* June 12→19 (consolidation era) then grew again with the
  TestGUI (062–063).

## Churn hotspots (times touched, excluding process/docs/config artifacts)

| file | commits touching it |
|---|---|
| `source/types/Protocol.h` | **300 (24% of all commits)** |
| `source/robot/Robot.cpp` | 109 |
| `source/robot/Robot.h` | 61 |
| `source/app/CommandProcessor.cpp` | 59 |
| `source/main.cpp` | 43 |
| `source/types/Config.h` | 40 |
| `source/control/MotorController.cpp` | 38 |
| `source/control/Odometry.h` | 33 |
| `source/control/LoopScheduler.cpp` | 33 |

(`host/pyproject.toml` at 283 and `config/dotconfig.yaml` at 235 are
version-bump/process churn.)

## Rework-signal sprint names

24 of 66 sprint names (36%) contain fix / consolidate / eliminate / cutover /
replace / collapse / harden / abandon / reliability / debug / diagnosis /
cleanup. This **undercounts** structural rework: sprints 016, 018–020, 026,
029, 034, 036, 038–045, 048, 055–061 are rework by content with neutral names.

## Notable event sequences

- **Encoder/I2C wedge:** deferred out of sprint 014 (2026-06-05, "wedge
  deferred to follow-up issue") → sprint 015 diagnosis/fix → 2026-06-07
  `fix: eliminate encoder wedge — IRQ-guard I2C transactions (nRF52 TWIM
  errata)` → **still being hardened in sprint 064 on 2026-07-02** (wedge
  triggers, IRQguard query bug, read failure, outlier-filter recovery), with a
  new "boundary-latch flavor" documented 2026-07-01. Recurrence arc ≈ 4 weeks.
- **Cross-coupling reversal inside one day** (2026-06-05): `feat[015]:
  velocity EMA filter + outlier rejection + cross-wheel ratio coupling`
  followed the same day by `fix[015]: disable cross-coupling by default
  (amplified velocity noise into wheel-fighting)`.
- **Architecture re-foundations** (each restructuring what previous sprints
  built): 007 (firmware architecture foundation, after 001–005), 014 (abandon
  fibers — runtime-model reversal), 016–020 (AppContext / BVC / MotionCommand /
  CommandProcessor / HAL overhaul), 025–029 (one dispatch path, navigation
  ownership), 034–037 (consolidation), 038–045 (Phase 0→F), 047–050 (state
  object, PID replacement, EKF replacement), 055–061 (message-architecture
  cutover). ≈ 7–8 distinct restructuring waves in 6 weeks.
- Only 7 textual `Revert`/`Reapply` commits — reversals happened at the
  *sprint* granularity (new sprint to undo a design), not the commit level.
