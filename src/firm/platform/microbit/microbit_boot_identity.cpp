#include "platform/microbit/microbit_boot_identity.h"

namespace Platform {

void showBootIdentity(MicroBit& uBit, const char* tag) {
  // Pixel values are BRIGHTNESS 0..255, not booleans -- a 1 here renders
  // at 1/255 duty, i.e. invisibly dim next to the font's full-bright
  // digit (stakeholder observed "only the 1, no hearts", 2026-07-27).
  constexpr uint8_t kOn = 255;
  static const uint8_t kHeart[] = {
      0,   kOn, 0,   kOn, 0,
      kOn, kOn, kOn, kOn, kOn,
      kOn, kOn, kOn, kOn, kOn,
      0,   kOn, kOn, kOn, 0,
      0,   0,   kOn, 0,   0,
  };
  constexpr int kHeartHold = 500;  // [ms]
  constexpr int kDigitHold = 700;  // [ms] per digit -- it is the payload
  constexpr int kDigitGap = 150;   // [ms] blank between digits, so a repeated
                                   // digit ("22") reads as two, not one long

  uBit.display.enable();
  MicroBitImage heart(5, 5, kHeart);
  uBit.display.print(heart);
  uBit.sleep(kHeartHold);
  uBit.display.clear();

  for (const char* p = tag; *p != '\0'; ++p) {
    uBit.display.printChar(*p);
    uBit.sleep(kDigitHold);
    uBit.display.clear();
    uBit.sleep(kDigitGap);
  }

  uBit.display.print(heart);  // resting state -- left lit through boot
}

}  // namespace Platform
