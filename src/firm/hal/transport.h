// transport.h -- Hal::Transport: the byte-pipe interface Core::Comms drives
// every registered line-oriented transport through (readLine/send/
// sendReliable). Moved verbatim out of core/comms.h's former
// `#ifndef HOST_BUILD` forward-declaration block (136-005, "dissolve com/
// into Hal::Transport + platform/microbit/") -- the interface itself never
// depended on HOST_BUILD; only its two ARM-only concrete adapters
// (Core::SerialTransport/Core::RadioTransport, now deleted) did. The two
// concrete pipes -- Platform::MicroBitSerialPort, Platform::MicroBitRadioLink
// (platform/microbit/) -- implement this interface directly now, so those
// adapter classes are gone, not relocated.
//
// Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

namespace Hal {

class Transport {
 public:
  virtual ~Transport() = default;

  virtual bool readLine(char* buf, uint16_t cap, uint16_t* outLen) = 0;

  virtual void send(const uint8_t* data, uint16_t len) = 0;

  virtual void sendReliable(const char* msg) = 0;
};

}  // namespace Hal
