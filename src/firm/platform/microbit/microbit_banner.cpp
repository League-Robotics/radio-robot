#include "platform/microbit/microbit_banner.h"

#include <stdio.h>

#include "MicroBit.h"

namespace Platform {

void formatBanner(char* buf, int size) {
  const char* name = microbit_friendly_name();
  uint32_t serial = microbit_serial_number();
  snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu", name,
           static_cast<unsigned long>(serial));
}

void formatIdLine(char* buf, int size, const char* drivetrainType, const char* profileName,
                   const char* version) {
  snprintf(buf, size, "ID:%s:%s:%s", drivetrainType, profileName, version);
}

}  // namespace Platform
