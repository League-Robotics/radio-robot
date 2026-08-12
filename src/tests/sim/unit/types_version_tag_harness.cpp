// types_version_tag_harness.cpp -- off-hardware acceptance proof for sprint
// 136 ticket 005 (SUC-003), Types::versionTag() (src/firm/types/
// version_tag.h). Relocated out of main.cpp's own ARM-only versionTag() --
// this is its FIRST test ever (it was previously untestable off-target).
//
// This harness's own include list is the compile-level dependency-free
// assertion: version_tag.h plus only host-standard-library headers -- no
// MicroBit.h, no messages/, no app/, nothing else from src/firm.
//
// Mirrors firm_types_robot_state_harness.cpp's own hand-rolled
// PASS/FAIL/exit-nonzero shape (no gtest, no pytest-side C++ dependency).
#include <cstdio>
#include <cstring>

#include "types/version_tag.h"

namespace {

int g_failureCount = 0;

void checkStrEq(const char* actual, const char* expected, const char* what) {
  if (std::strcmp(actual, expected) != 0) {
    ++g_failureCount;
    std::printf("  FAIL: %s -- expected \"%s\", got \"%s\"\n", what, expected, actual);
  }
}

}  // namespace

int main() {
  std::printf("--- Types::versionTag: dependency-free, day+build tag parsing\n");

  // Normal major.date.build case: "0.20260726.1" -> day 26 + build "1" ->
  // "261". Matches this function's own doc comment example.
  {
    char tag[8] = {};
    Types::versionTag("0.20260726.1", tag, sizeof(tag));
    checkStrEq(tag, "261", "normal major.date.build (0.20260726.1)");
  }

  // A different day and build, to prove it isn't hardcoded to the example
  // above -- "0.20260812.3" -> day 12 + build "3" -> "123".
  {
    char tag[8] = {};
    Types::versionTag("0.20260812.3", tag, sizeof(tag));
    checkStrEq(tag, "123", "normal major.date.build (0.20260812.3)");
  }

  // Multi-digit build number: the day is still fixed-width (2 chars), but
  // the build field is copied verbatim to its end.
  {
    char tag[8] = {};
    Types::versionTag("0.20260729.12", tag, sizeof(tag));
    checkStrEq(tag, "2912", "multi-digit build (0.20260729.12)");
  }

  // Fallback: a string with fewer than two dots emits "?" rather than
  // indexing off the end of the string.
  {
    char tag[8] = {};
    Types::versionTag("nodots", tag, sizeof(tag));
    checkStrEq(tag, "?", "fallback -- no dots at all");
  }

  {
    char tag[8] = {};
    Types::versionTag("0.20260726", tag, sizeof(tag));
    checkStrEq(tag, "?", "fallback -- only one dot");
  }

  // Fallback: two dots, but the date field between them is too short (< 2
  // chars) to take a day from.
  {
    char tag[8] = {};
    Types::versionTag("0.5.1", tag, sizeof(tag));
    checkStrEq(tag, "?", "fallback -- date field too short");
  }

  // Empty string: no dots -- also falls back to "?", not an empty/garbage
  // tag.
  {
    char tag[8] = {};
    Types::versionTag("", tag, sizeof(tag));
    checkStrEq(tag, "?", "fallback -- empty version string");
  }

  if (g_failureCount == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d check(s) failed\n", g_failureCount);
  return 1;
}
