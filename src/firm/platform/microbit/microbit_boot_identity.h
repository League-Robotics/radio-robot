// microbit_boot_identity.h -- Platform::showBootIdentity(): the LED-matrix
// boot-time UI routine (heart, the version day+build tag, heart again).
// Relocated out of main.cpp (136-005, "de-junk main.cpp") -- it is a
// uBit.display-bound CODAL routine, so it belongs beside the rest of
// platform/microbit/'s ARM-only leaves, not inline in main().
#pragma once

#include "MicroBit.h"

namespace Platform {

// Boot identity on the LED matrix: heart, the last digit(s) of the
// firmware version's day+build tag, then the heart again -- and the heart
// STAYS LIT.
//
// This is the only way to tell WHICH build is actually on the board without
// opening a serial session, and "did that flash land?" is a question that
// has cost real debugging time. The trailing heart is the resting state: a
// lit display through boot means "powered, flashed, and running"; a dark
// one means it never got here.
//
// It is not left on forever -- main() disables the display the instant
// boot completes and before the first control cycle: the LED matrix
// driver refreshes continuously off its own timer, and the loop's own
// motion tuning assumes those cycles are not being spent elsewhere. Boot
// is the one window where the display's cost is free, regardless of what
// the post-boot cycle budget is (see Core::RobotLoop::kCycle's own doc
// comment for that budget's history).
//
// `uBit` is taken by reference rather than assumed as a global -- this
// file has no state of its own, and the caller (main.cpp) already owns
// the one MicroBit singleton the whole firmware shares. `tag` is the
// already-computed version day+build string (Types::versionTag(),
// types/version_tag.h) -- computed by the CALLER, not here, because
// `platform/` may not reach `types/` (src/tests/sim/unit/
// test_layer_isolation.py's platform-layer allowed-prefix set is
// `platform/`/`hal/` only); main.cpp is the one place both the version
// macro and this display routine are in scope together.
void showBootIdentity(MicroBit& uBit, const char* tag);

}  // namespace Platform
