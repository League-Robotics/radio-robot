// ---------------------------------------------------------------------------
// main.cpp -- banner-only stub (sprint 102 ticket 005, single-loop firmware
// rebuild). Replaces the deleted Elite orchestration stack (runtime/,
// subsystems/, commands/, drive/, telemetry/, hal/, estimation/) with the
// minimum firmware that proves the board boots and talks: the DEVICE:
// identity banner plus a HELLO/PING text reply loop. Motors are NEVER
// energized here and no I2C bus is touched -- sprint 103 builds the real
// single loop (source/app/) on top of this stub.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstring>

#include "MicroBit.h"
#include "com/serial_port.h"

static MicroBit uBit;

namespace {

// DEVICE:NEZHA2:robot:<name>:<serial> -- byte-identical to the deleted
// stack's own formatDeviceAnnouncement() (source/commands/text_channel.cpp,
// git history) so a host client's existing banner parser keeps working
// unchanged.
void formatBanner(char* buf, int size) {
  const char* name = microbit_friendly_name();
  uint32_t serial = microbit_serial_number();
  snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu", name,
            static_cast<unsigned long>(serial));
}

}  // namespace

int main() {
  uBit.init();

  static SerialPort serial(uBit.serial);
  serial.begin();

  char banner[64];
  formatBanner(banner, sizeof(banner));
  serial.sendReliable(banner);

  char line[256];
  for (;;) {
    if (serial.readLine(line, sizeof(line))) {
      if (strcmp(line, "HELLO") == 0) {
        serial.sendReliable(banner);
      } else if (strcmp(line, "PING") == 0) {
        serial.sendReliable("OK pong");
      } else {
        serial.sendReliable("ERR unknown");
      }
    }
    uBit.sleep(1);  // yield: radio-safe, matches every prior main.cpp loop
  }

  return 0;
}
