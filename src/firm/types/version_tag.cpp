#include "types/version_tag.h"

namespace Types {

void versionTag(const char* version, char* out, size_t cap) {
  if (cap == 0) return;
  const char* firstDot = nullptr;
  const char* lastDot = nullptr;
  for (const char* p = version; *p != '\0'; ++p) {
    if (*p == '.') {
      if (firstDot == nullptr) firstDot = p;
      lastDot = p;
    }
  }
  size_t n = 0;
  // Two distinct dots, and a date field of at least two characters to take a
  // day from.
  if (firstDot != nullptr && lastDot != firstDot && (lastDot - firstDot) >= 3) {
    if (n + 1 < cap) out[n++] = *(lastDot - 2);
    if (n + 1 < cap) out[n++] = *(lastDot - 1);
    for (const char* p = lastDot + 1; *p != '\0' && n + 1 < cap; ++p) out[n++] = *p;
  }
  if (n == 0 && cap > 1) out[n++] = '?';
  out[n] = '\0';
}

}  // namespace Types
