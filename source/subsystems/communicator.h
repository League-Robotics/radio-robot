// communicator.h -- Subsystems::Communicator: the comms faceplate. Owns both
// communication drivers (SerialPort + Radio, source/com/ infrastructure
// leaves) and the line buffer, and turns "a complete command line arrived on
// some channel" into a returned CommunicatorToCommandProcessorCommand edge.
//
// This subsystem is a *source* of commands, not a sink: it deliberately has
// NO command-in channel -- no apply(), no CommunicatorCommand message
// (protos/communicator.proto documents the same from the wire side). Its
// tick() PRODUCES the command line the wiring layer (main.cpp) dispatches
// through CommandProcessor.
//
// Faceplate channels:
//   config       -- configure(msg::CommunicatorConfig): radio channel
//                   (clamped to radiochan's 0..35), live-retuned after begin().
//   command-in   -- absent by design (see above).
//   command-out  -- returned from tick(now): at most ONE line per tick.
//   observation  -- state(): channel + received-line counters per channel.
//   capabilities -- capabilities(): which comms channels exist.
//
// Reply adapters build on the primitive sends (sendSerial/sendRadio) -- the
// old serial()/radio() pass-through accessors are gone; that is the point of
// internalizing the drivers. A future need for a driver primitive (e.g.
// runtime baud) becomes a new Communicator primitive, not an accessor.
#pragma once

#include <stdint.h>

#include "MicroBit.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "messages/communicator.h"

namespace Subsystems {

// Which comms channel a command line arrived on -- and therefore where its
// reply must be sent.
enum class Channel : uint8_t { NONE, SERIAL, RADIO };

// Command-out edge type, named by its endpoints (<Producer>To<Consumer>Command
// per .claude/rules/naming-and-style.md): one parsable command line plus its
// return path.
struct CommunicatorToCommandProcessorCommand {
  // nullptr when no complete line arrived this tick. Otherwise aliases the
  // Communicator's internal line buffer: valid only until the next tick() --
  // consumers must dispatch (or copy) before then. Safe with today's only
  // consumer: CommandProcessor::process() copies the line before parsing.
  const char* line;
  Channel returnPath;  // where the reply to this line must be sent
};

class Communicator {
 public:
  Communicator(NRF52Serial& serial, MicroBitRadio& radio, MessageBus& bus);

  // Config channel. Clamps radio_channel via radiochan::clamp() (proto zero
  // default == radiochan::kDefault == 0). Before begin(): stored for begin()
  // to bring the radio up on. After begin(): a changed channel retunes live
  // via Radio::setChannel() -- which drops the relay link (the relay stays
  // on the old channel), so any reply must be sent BEFORE reconfiguring.
  void configure(const msg::CommunicatorConfig& config);

  // Hardware bring-up: serial + radio on the configured channel. Call once
  // in main() after uBit.init(), before the loop. Only ONE Communicator may
  // begin(): Radio's datagram ISR dispatches through a static singleton
  // pointer (Radio::_instance), so a second begin() would steal it.
  void begin();

  // Command-out channel. now: [ms]. Polls serial first, then radio, and
  // returns at most ONE complete line per tick ({nullptr, NONE} when idle).
  // A radio message not taken this tick stays latched in the Radio driver
  // until the next poll, and the loop runs ~kHz vs the radio's <=12 msg/s --
  // nothing is lost and radio never starves behind serial.
  CommunicatorToCommandProcessorCommand tick(uint32_t now);

  msg::CommunicatorState state() const;
  msg::CommunicatorCapabilities capabilities() const;

  // Primitive sends -- reply adapters build on these. Same semantics as the
  // drivers' send() (serial: ASYNC drop-on-full; radio: fragmented RAW250).
  void sendSerial(const char* msg);
  void sendRadio(const char* msg);

 private:
  SerialPort serial_;
  Radio radio_;

  int channel_ = 0;      // clamped configured radio channel (frequency band)
  bool begun_ = false;   // gates configure()'s live retune

  // Single shared line buffer: serial and radio command lines are the same
  // format (the relay's !GO data plane carries plain lines both ways), and
  // tick() surfaces one line at a time. 256 bytes, byte-identical to the
  // stack buffers main.cpp used to thread through pollComms().
  char line_[256];

  uint32_t serialLines_ = 0;  // complete lines received over serial
  uint32_t radioLines_ = 0;   // complete lines received over radio
};

}  // namespace Subsystems
