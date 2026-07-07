// communicator.h -- Subsystems::Communicator: the comms faceplate. Owns both
// communication drivers (SerialPort + Radio, source/com/ infrastructure
// leaves) and the line buffer, and turns "a complete statement line arrived
// on some channel" into a held CommunicatorToCommandProcessorStatement edge
// (hasStatement()/takeStatement()).
//
// This subsystem is a *source* of statements, not a sink: it deliberately
// has NO command-in channel -- no apply(), no CommunicatorCommand message
// (protos/communicator.proto documents the same from the wire side). Its
// tick() latches the statement line the wiring layer (main.cpp) dispatches
// through CommandProcessor; hasStatement()/takeStatement() are the held/
// taken pair that surfaces it.
//
// Held-output contract: tick() polls serial first, then radio, and latches
// at most ONE complete statement at a time. While a statement is still held
// (not yet taken), tick() declines to poll either transport -- it must not
// overwrite line_[] out from under a consumer that has not read it yet. An
// untaken statement is therefore backpressure, not data loss: the next
// tick() simply leaves the held statement in place until takeStatement()
// clears it. The intended wiring (main.cpp) always takes a held statement
// the same pass it appears, so this should never actually stall in
// practice -- but the contract holds regardless of call discipline.
//
// Faceplate channels:
//   config       -- configure(msg::CommunicatorConfig): radio channel
//                   (clamped to radiochan's 0..35), live-retuned after begin().
//   command-in   -- absent by design (see above).
//   command-out  -- hasStatement()/takeStatement(): at most ONE statement
//                   held at a time (see the held-output contract above).
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
#include "subsystems/statement.h"

namespace Subsystems {

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

  // Command-out channel, held half. now: [ms]. While a statement is already
  // held (hasStatement()==true), declines to poll either transport -- see
  // the held-output contract in the file header. Otherwise polls serial
  // first, then radio, and latches at most ONE complete statement. A radio
  // message not taken this tick stays latched in the Radio driver until the
  // next poll -- so nothing is lost either way, and radio never starves
  // behind serial.
  void tick(uint32_t now);

  // True when a complete statement is currently held, awaiting
  // takeStatement().
  bool hasStatement() const { return hasStatement_; }

  // Command-out channel, taken half. Clears the held flag so the next
  // tick() may resume polling. Copies the held line into the returned
  // struct's own owned buffer (subsystems/statement.h) -- the caller may
  // hold the result past the next tick() without it being overwritten out
  // from under them (e.g. once queued by value in an Rt::WorkQueue).
  CommunicatorToCommandProcessorStatement takeStatement();

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

  // Single shared line buffer: serial and radio statement lines are the same
  // format (the relay's !GO data plane carries plain lines both ways), and
  // tick() latches one line at a time. 256 bytes, byte-identical to the
  // stack buffers main.cpp used to thread through pollComms().
  char line_[256];

  bool hasStatement_ = false;               // a statement is held, unread
  Channel heldReturnPath_ = Channel::NONE;  // return path for the held statement

  uint32_t serialLines_ = 0;  // complete lines received over serial
  uint32_t radioLines_ = 0;   // complete lines received over radio
};

}  // namespace Subsystems
