#include "banner.h"

#include <stdio.h>

#include "MicroBit.h"

void formatBanner(char* buf, int size) {
  const char* name = microbit_friendly_name();
  uint32_t serial = microbit_serial_number();
  snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu", name,
           static_cast<unsigned long>(serial));
}
