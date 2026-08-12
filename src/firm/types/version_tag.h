// version_tag.h -- Types::versionTag(): the boot-display tag, the DAY of a
// version string's date field followed by its build number
// ("0.20260726.1" -> "261", day 26 + build 1). Relocated out of main.cpp
// (136-005, "de-junk main.cpp") -- pure string logic with no CODAL
// dependency, so it moved somewhere host-buildable and testable instead of
// staying ARM-only. main.cpp called it with no version-string parameter
// (reading the generated FIRMWARE_VERSION_STR macro directly); this
// relocation takes `version` as an explicit parameter instead, which is
// what makes a host-side test possible at all -- FIRMWARE_VERSION_STR is
// gitignored/generated (scripts/gen_version.py) and not a fixed literal a
// test could pin against.
//
// A single trailing digit cannot distinguish two builds made on different
// days -- the board showed "1" for both 0.20260726.1 and 0.20260729.1,
// which is exactly the confusion that costs bench time. Day+build stays
// short enough to read off the matrix at a glance while being unique
// across any plausible session (stakeholder directive 2026-07-29).
#pragma once
#include <cstddef>

namespace Types {

// Parsed rather than indexed at fixed offsets, so it survives a format
// change in gen_version.py; emits "?" if `version` lacks the two dots the
// major.date.build shape requires. `out`/`cap` follow the project's usual
// bounded-buffer convention (`cap` includes the trailing NUL).
void versionTag(const char* version, char* out, size_t cap);

}  // namespace Types
