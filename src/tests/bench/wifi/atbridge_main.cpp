// atbridge -- bench-only transparent AT bridge for the Planet X WiFi module
// (Ai-Thinker Ai-WB2-12F, ESP-AT-compatible firmware) plugged into one of the
// Nezha expansion board's RJ11 ports.
//
// USB serial <-> module UART byte pipe, both at 115200. On boot it probes the
// four RJ11 ports (pin map from ELECFREAKS pxt-PlanetX wifi.ts) by sending
// "AT" and listening for "OK", announces the port it found on the USB side
// with an "ATBRIDGE:" line, then pipes bytes both directions verbatim. The
// host drives the module's AT surface directly (join an AP, open UDP/TCP
// sockets) with no reflash between experiments.
//
// This is NOT robot firmware: no robot loop, no protocol v5, no I2C. Flash
// the normal build to return the board to service.
#include "MicroBit.h"

#include <cstdio>
#include <cstring>

static MicroBit uBit;

namespace {

constexpr uint32_t kBaud = 115200;       // [baud] USB side + module boot default
constexpr uint32_t kModuleBaud = 57600;  // [baud] module side after downshift --
                                         // CODAL's UARTE RX drops ~5-10% of
                                         // chars at a sustained 115200; halving
                                         // the rate doubles every IRQ race
                                         // window (bench finding 2026-08-08)
constexpr int kProbeWindow = 600;       // [ms] listen time per AT probe
constexpr int kProbeAttempts = 3;       // per port, per sweep
constexpr uint8_t kBufferSize = 250;    // [bytes] serial ring buffers

// RJ11 connector -> micro:bit pins, from pxt-PlanetX wifi.ts initWIFI().
// tx is the micro:bit's transmit pin (module RX); rx is the micro:bit's
// receive pin (module TX).
struct RjPort {
  const char* name;
  NRF52Pin* tx;
  NRF52Pin* rx;
};

RjPort rjPorts[] = {
    {"J1", &uBit.io.P8, &uBit.io.P1},
    {"J2", &uBit.io.P12, &uBit.io.P2},
    {"J3", &uBit.io.P14, &uBit.io.P13},
    {"J4", &uBit.io.P16, &uBit.io.P15},
};

void usbLine(const char* msg) {
  uBit.serial.send((uint8_t*)msg, strlen(msg), SYNC_SLEEP);
  uBit.serial.send((uint8_t*)"\r\n", 2, SYNC_SLEEP);
}

// One "AT" -> "OK" probe on whatever pins `wifi` is currently redirected to.
bool probeOnce(NRF52Serial& wifi) {
  wifi.clearRxBuffer();
  wifi.send((uint8_t*)"AT\r\n", 4, SYNC_SLEEP);
  char window[4] = {0};  // rolling last-3-chars window, NUL-padded
  uint32_t deadline = system_timer_current_time() + kProbeWindow;  // [ms]
  while (system_timer_current_time() < deadline) {
    int c = wifi.read(ASYNC);
    if (c < 0) {
      fiber_sleep(5);
      continue;
    }
    window[0] = window[1];
    window[1] = window[2];
    window[2] = (char)c;
    if (window[0] == 'O' && window[1] == 'K') return true;
  }
  return false;
}

// Sweep the RJ11 ports until the module answers; returns the port index.
int findModule(NRF52Serial& wifi) {
  for (int sweep = 0;; ++sweep) {
    for (unsigned p = 0; p < sizeof(rjPorts) / sizeof(rjPorts[0]); ++p) {
      wifi.redirect(*rjPorts[p].tx, *rjPorts[p].rx);
      for (int attempt = 0; attempt < kProbeAttempts; ++attempt) {
        if (probeOnce(wifi)) return (int)p;
      }
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "ATBRIDGE: sweep %d found no module, retrying",
             sweep + 1);
    usbLine(msg);
  }
}

}  // namespace

int main() {
  uBit.init();

  uBit.serial.setBaudrate(kBaud);
  uBit.serial.setRxBufferSize(kBufferSize);
  uBit.serial.setTxBufferSize(kBufferSize);

  static NRF52Serial wifi(*rjPorts[0].tx, *rjPorts[0].rx, NRF_UARTE1);
  wifi.setBaudrate(kBaud);
  wifi.setRxBufferSize(kBufferSize);
  wifi.setTxBufferSize(kBufferSize);

  uBit.display.printChar('?');
  usbLine("ATBRIDGE: probing RJ11 ports for the WiFi module");

  int portIndex = findModule(wifi);

  // Downshift the module's UART (volatile -- reverts to 115200 on module
  // power cycle, which is what we want: the probe above always works).
  // AT+UART_CUR is the ESP-AT verb; the Ai-Thinker combo firmware answers
  // OK to it. Then retune our own side and confirm with a fresh probe.
  char cmd[48];
  snprintf(cmd, sizeof(cmd), "AT+UART_CUR=%lu,8,1,0,0\r\n",
           (unsigned long)kModuleBaud);
  wifi.send((uint8_t*)cmd, strlen(cmd), SYNC_SLEEP);
  fiber_sleep(300);
  wifi.setBaudrate(kModuleBaud);
  fiber_sleep(100);
  bool downshifted = probeOnce(wifi);
  if (!downshifted) {
    // Module ignored the downshift verb -- fall back to 115200.
    wifi.setBaudrate(kBaud);
  }

  char msg[80];
  snprintf(msg, sizeof(msg), "ATBRIDGE: module on %s, module baud %lu",
           rjPorts[portIndex].name,
           (unsigned long)(downshifted ? kModuleBaud : kBaud));
  usbLine(msg);
  usbLine("ATBRIDGE: ready");

  // The LED matrix refresh interferes with UARTE RX DMA (bytes drop under
  // IRQ contention) -- show the found port briefly, then go dark for a
  // clean pipe.
  uBit.display.printChar('1' + portIndex);
  fiber_sleep(700);
  uBit.display.clear();
  uBit.display.disable();

  // Transparent pipe. SYNC_SLEEP on the write side backpressures instead of
  // dropping; both sides run the same baud so neither can outrun the other
  // for long.
  uint8_t buf[128];
  for (;;) {
    bool moved = false;
    int n = wifi.read(buf, sizeof(buf), ASYNC);
    if (n > 0) {
      uBit.serial.send(buf, n, SYNC_SLEEP);
      moved = true;
    }
    n = uBit.serial.read(buf, sizeof(buf), ASYNC);
    if (n > 0) {
      wifi.send(buf, n, SYNC_SLEEP);
      moved = true;
    }
    if (!moved) fiber_sleep(1);
  }
}
