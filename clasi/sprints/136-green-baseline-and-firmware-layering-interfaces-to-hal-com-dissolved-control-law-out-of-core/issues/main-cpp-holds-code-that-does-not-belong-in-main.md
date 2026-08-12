---
status: in-progress
sprint: '136'
tickets:
- 136-005
- 136-009
---

# `main.cpp` holds code that does not belong in `main`

## Description

Stakeholder directive (2026-08-11): *"We gotta get rid of `showBootIdentity`
from main. There's a bunch of stuff in here that does not belong in main.
Version tag: go make utilities or something, or just some miscellaneous code
repo for this crap — you don't need to be literally putting it in main."*

`src/firm/main.cpp` is 209 lines and its own header says it is the ARM entry
point with "no cycle logic and no graph-construction logic." That is true, but
it still carries ~95 lines of anonymous-namespace helpers that are not entry-point
work:

| Lines | What | Why it isn't `main`'s job |
|---|---|---|
| 46-66 | `versionTag()` — parses `FIRMWARE_VERSION_STR` (`"0.20260726.1"` → `"261"`) into a short display tag | Pure string manipulation over a generated constant. No hardware, no CODAL, no entry-point concern. Testable in isolation today; untested because it is file-static in `main.cpp`. |
| 68-124 | `showBootIdentity()` — draws a heart bitmap and the version tag on the LED matrix, holds it lit through boot | A boot-time user-interface routine. It is CODAL-bound (`uBit.display`) but that makes it *platform* code, not *entry-point* code. |
| 148-156 | the `ID:<drivetrain>:<profile>:<version>` line assembled with `snprintf` | Wire-format string construction. Its sibling — the `DEVICE:NEZHA2:…` banner — already lives outside `main` in `com/banner.cpp`, so this one is inconsistent with its own counterpart. |

The `versionTag` helper carries 20 lines of comment explaining a
stakeholder-visible behaviour (day+build, so two builds from different days are
distinguishable on the matrix — a confusion that "costs bench time"). That is a
real, load-bearing behaviour with a real history, and it is currently
untestable because it is buried in a file that only exists on the ARM target.

## Cause

Incremental accretion. Each helper was added at the point it was first needed,
and `main.cpp` was the only file that had both the CODAL `uBit` singleton and
the generated version constant in scope. Nothing forced a home to be chosen.

The 130-002 composition-root unification pulled all the *graph* construction out
of `main.cpp` but left these three helpers behind, because they are not graph
construction and there was no obvious destination for them at the time.

## Proposed fix

Give each piece a home; leave `main()` as boot sequencing only.

1. **`showBootIdentity()` → a platform home.** It is CODAL/`uBit.display`-bound,
   so it belongs with the other micro:bit-specific code — the natural neighbour
   is `platform/microbit/`, alongside the clock, the I2C bus, and (after the
   layering cleanup) the serial port, radio, and banner formatter. `main()`
   calls one function and still owns the *when*: identity before the buses,
   `display.disable()` after boot and before the first cycle. That ordering is
   deliberate and documented — the matrix refresh timer competes with the
   control loop's cycle budget — and must survive the move verbatim.

2. **`versionTag()` → somewhere testable.** It is pure and platform-free. It
   should compile host-side and get a unit test, which it has never had. Whether
   that is `types/` (which already holds the version-generation seam), a new
   `util/`, or somewhere else is a design call for the sprint — the requirement
   is that it is no longer ARM-only and no longer untested.

3. **The `ID:` line → wherever the banner ends up.** `formatBanner()` and the
   `ID:` line are the same kind of thing (a frozen wire-format identity string
   built from generated constants) and should not live in two different places.
   Note the layering-cleanup issue moves `com/banner.*` into
   `platform/microbit/` — this should follow it, or both should move somewhere
   better together.

Explicitly **not** in scope: changing any of the strings. The `DEVICE:NEZHA2:…`
banner and the `ID:<drivetrain>:<profile>:<version>` line are frozen wire format
(`docs/protocol-v5.md`); the boot tag's day+build shape is a stakeholder
directive from 2026-07-29. This is a relocation, not a redesign.

## Verification

- Host build (`just build-sim`) and ARM build (`uv run python3 build.py`) both clean.
- A new unit test for `versionTag()` covering the documented cases: a normal
  `major.date.build` string, and the `"?"` fallback when the string lacks the two
  dots the shape requires.
- **Bench check on the stand**, because the boot display is the only way to tell
  which build is on a board without opening a serial session: flash, watch the
  matrix, confirm heart → digits → heart-stays-lit, and confirm the display goes
  dark when the loop starts. Then confirm the banner and `ID:` line are
  byte-identical to before over the real link (`twist_drive.py`'s identify step
  covers both).

## Related

- `src/firm/main.cpp` lines 32-126 (the anonymous namespace) and 143-156.
- `clasi/issues/firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md`
  — moves `com/banner.*` into `platform/microbit/`; this issue should land in the
  same sprint or immediately after, so the banner and the `ID:` line end up
  together rather than drifting further apart.
- `clasi/issues/robot-base-class-and-robots-subsystem.md` — the larger
  `main.cpp`/`RobotGraph` cleanup this is the small half of.
