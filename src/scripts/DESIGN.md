# Scripts (`src/scripts`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** in-flux

---

## 1. Purpose

`scripts/` holds the project's build-time code generators: plain Python
scripts, invoked from `build.py`'s codegen step, that turn a single
source of truth (`protos/*.proto`, `data/robots/*.json`, the root
`pyproject.toml`) into generated artifacts nothing hand-edits. It exists
as its own directory because "config as truth, never a hardcoded
fallback baked into a hand-written file" is a cross-cutting project rule
(sprint 114 — see [`../firm/config/DESIGN.md`](../firm/config/DESIGN.md)
§4), and every generator that implements a piece of it belongs together
rather than scattered one-per-consumer.

## 2. Orientation

Five scripts, four still actively run by `build.py`, one effectively
dead. (A sixth, `check_config_sync.py`, was a CI-only lint that
compared hand-curated config lists for drift; deleted 132-004 — its
premise, definitions maintained independently enough to drift, no
longer holds once `gen_messages.py` generates every representation
from one schema. What now guards against drift is a strictly stronger,
mechanical guarantee: `src/firm/config/config_parity_capi.{h,cpp}` +
`src/tests/unit/test_config_parity_capi.py`, added 132-003, which
compares generated-code sizes/offsets directly instead of diffing
hand-maintained lists.)

| Script | Runs from | Produces | Status |
|---|---|---|---|
| `gen_messages.py` | `build.py` codegen (every build) | `src/firm/messages/*.h` (C++11 POD structs + table-driven codec) from `protos/*.proto` | live |
| `gen_pb2.py` | `build.py` codegen (every build) | `src/host/robot_radio/robot/pb2/*_pb2.py` (compiled Python bindings) from the same `protos/*.proto` | live |
| `gen_boot_config.py` | `build.py` codegen (every build) | `src/firm/config/boot_config.cpp` from the active robot's `data/robots/*.json` | live |
| `gen_version.py` | `build.py` codegen (every build) | `src/firm/types/version_generated.h` from the root `pyproject.toml` version | live |
| `gen_default_config.py` | nothing — `build.py` explicitly skips it | would produce `src/firm/robot/DefaultConfig.cpp` | **dead** — targets a `src/firm/robot/` directory that does not exist post-077 rebuild; see §6 |
| `migrate_robot_json_to_grouped_shape.py` | by hand, once (132-017) | reshaped `data/robots/{tovez,togov,tovez_nocal}.json` (the OLD 13-section `control`/`calibration` dumping-ground shape into `Config::Robot`'s grouped shape) | **one-time, run and committed 2026-08-04** — not invoked by `build.py`; kept only as a record of the reshape and in case a future robot JSON needs the same migration from an old-shaped source |

`gen_boot_config.py` and `gen_default_config.py` look like siblings (both
"bake a robot JSON into a generated C++ config file") but are
deliberately separate, un-shared code: they target different C++ types
on different velocity-PID plant scales (`gen_boot_config.py`'s
`NezhaMotor` loop is duty-scale `~kp 0.002`; `gen_default_config.py`'s
retired `RobotConfig` loop was PWM-percent-scale `~kp 0.3`) — merging
them would risk silently applying one plant's gains to the other's loop,
the same reasoning `../firm/config/DESIGN.md` §4 gives for keeping them
apart.

## 3. Constraints and Invariants

- **Every generator's output is never hand-edited.** `gen_messages.py` →
  `src/firm/messages/*.h`, `gen_pb2.py` →
  `src/host/robot_radio/robot/pb2/*_pb2.py`, `gen_boot_config.py` →
  `src/firm/config/boot_config.cpp`, `gen_version.py` →
  `src/firm/types/version_generated.h` — a hand edit to any of these is
  silently destroyed the next codegen run and gives no error, wasting
  real debugging time on a phantom regression. Fix the generator or the
  source it reads (`protos/*.proto`, the robot JSON,
  `pyproject.toml`), never the generated file.
- **Config-as-truth: no Python-side behavioral fallback in
  `gen_boot_config.py`.** Per sprint 114, a robot JSON missing a required
  calibration key fails the build loudly (`MissingRobotConfigKeyError`,
  `sys.exit(1)` naming the key and the JSON path) rather than silently
  substituting a bench-placeholder constant baked into the generator.
  This is a deliberate reversal of the *older* `gen_default_config.py`
  pattern (falls back to hardcoded defaults so the build always
  succeeds) — do not backport the old fallback-on-missing-key behavior
  into `gen_boot_config.py`; that is the exact silent-miscalibration
  failure mode sprint 114 closed.
- **`gen_messages.py`/`gen_pb2.py` must stay in lockstep on the same
  `protos/*.proto` set.** Both compile from the identical schema on
  every build specifically so the firmware's C++ tables and the host's
  Python bindings can never skew independently. Changing one generator's
  input resolution (e.g. adding a proto search path) without mirroring
  it in the other reintroduces that skew risk.
- **`gen_pb2.py`'s flat-import gotcha is load-bearing, not incidental.**
  `protoc`'s Python codegen (run flat, `-I protos`, no nested output
  tree) emits cross-file references as bare top-level imports (e.g.
  `envelope_pb2.py` does `import drivetrain_pb2 as drivetrain__pb2`) that
  only resolve if `src/host/robot_radio/robot/pb2/` itself is on
  `sys.path` — that package's own `__init__.py` (also generated by this
  script) is what arranges that. Do not "fix" the flat imports by
  hand-editing a `_pb2.py`; that edit is destroyed the next run.

## 4. Design

**Why codegen instead of a build-system plugin.** Every generator here
is a plain script `build.py` shells out to before the CMake configure/
build step — not a CMake custom-command, not a setuptools plugin. This
keeps the dependency direction simple (Python reads a JSON/proto source,
writes a C++/Python file, then the normal compiler picks it up) and lets
the same generators run identically from `just build`, a bare `python
build.py`, or a CI job with no build-system-specific wiring.

**Fallback discipline differs by generator, deliberately.**
`gen_boot_config.py` fails loudly on a missing required key (config as
truth, §3); `gen_version.py` and `gen_messages.py`/`gen_pb2.py` degrade
gracefully instead — a missing `version_generated.h` falls back to a
`"0.0.0-dev"` marker string (via `protocol.h`'s `__has_include` guard,
see [`../firm/types/DESIGN.md`](../firm/types/DESIGN.md) §4) so clangd
and codegen-less compiles still build, and a missing/unparseable robot
JSON degrades `gen_boot_config.py`'s *individual field* resolution to a
named `_DEFAULT` constant per field (module-scope, not silently baked
into `main.cpp`) rather than failing the whole build outright for every
possible missing key — the loud failure is specifically for a *required*
key with no sane default, not for the generator's own tooling
availability.

## 5. Interfaces

### Exposes

- **`gen_messages.py [--dry-run] [--emit-inventory]`** — regenerates
  `src/firm/messages/*.h`; `--emit-inventory` additionally writes
  `docs/design/message-inventory.md` (a traceability table, not part of
  the co-located design-doc set — see that file's own frontmatter-less
  status in the root design doc's subsystem map).
- **`gen_pb2.py`** — regenerates
  `src/host/robot_radio/robot/pb2/*_pb2.py`.
- **`gen_boot_config.py`** — regenerates `src/firm/config/boot_config.cpp`
  from `data/robots/active_robot.json` (or the `ROBOT_CONFIG` env var
  override).
- **`gen_version.py`** — regenerates
  `src/firm/types/version_generated.h` from the root `pyproject.toml`.

### Consumes

- **`protos/*.proto`** (via `grpcio-tools`, host-only) — the wire schema
  source of truth; see [`../protos/DESIGN.md`](../protos/DESIGN.md).
- **`data/robots/*.json`** — per-robot calibration source; see
  [`../firm/config/DESIGN.md`](../firm/config/DESIGN.md) §5.
- **Root `pyproject.toml`** — the canonical project version string.

## 6. Open Questions / Known Limitations

- **`gen_default_config.py` is dead code, not merely unused.**
  `build.py` (line ~76-90) explicitly detects that `src/firm/robot/`
  does not exist and prints `"build.py: src/firm/robot/ absent --
  skipping gen_default_config.py (077-001)"` rather than running it —
  its target type (`source/robot/RobotConfig`, per its own docstring,
  written to a path that itself references a `src/firm/robot/` directory
  that was never recreated in the 077 greenfield rebuild) has no living
  consumer anywhere in `src/firm/`. It has not been deleted; whether to
  delete it outright or keep it parked as a historical reference is an
  open call, not decided by this review.
