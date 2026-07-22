# Libraries (`src/libraries`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** stable

---

## 1. Purpose

`src/libraries/` is where the vendored CODAL SDK (`codal-core`,
`codal-nrf52`, `codal-microbit-nrf5sdk`, `codal-microbit-v2`) lands after
`src/utils/generate_libraries.py` fetches it. It has **no
architecturally significant content of this project's own** — every file
under it is third-party vendor code, entirely `.gitignore`d (`git
ls-files src/libraries` returns zero tracked files) and reconstructed by
tooling rather than checked in. It exists as its own directory purely so
the CMake build has a stable, predictable path to point at.

## 2. Constraints and Invariants

- **Never hand-edited.** Nothing here is checked into this repository's
  git history; any local edit is invisible to version control and lost
  the next time `generate_libraries.py` re-fetches. A firmware bug that
  looks like it needs a fix inside `codal-nrf52`/`codal-core` needs a
  vendor-side fix or an upstream patch tracked separately — not a local
  edit here.
- **Vendor names are exempt from this project's naming conventions**
  (per [`../firm/DESIGN.md`](../firm/DESIGN.md) §5 and
  `.claude/rules/coding-standards.md`'s "external/vendor function names
  are excluded" clause) — `system_timer_current_time_us()` and everything
  else declared under this tree keeps its upstream name.

## 3. Interfaces

### Exposes

- The CODAL SDK headers/sources the CMake build compiles `src/firm/`'s
  ARM target against — `MicroBit.h`, `MicroBitI2C`, the fiber scheduler,
  `NRF52Serial`, `MicroBitRadio`, etc. See
  [`../firm/devices/DESIGN.md`](../firm/devices/DESIGN.md) §5 and
  [`../firm/com/DESIGN.md`](../firm/com/DESIGN.md) §5 for which project
  files actually consume which vendor symbols.

### Consumes

- Fetched/managed by [`../utils/DESIGN.md`](../utils/DESIGN.md)'s
  `generate_libraries.py`, driven by `src/utils/targets.json`.

No further sections apply — there is no project-authored design here to
document.
