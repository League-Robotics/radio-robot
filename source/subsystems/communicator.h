// communicator.h -- Subsystems::Communicator: the comms faceplate. Owns both
// communication drivers (SerialPort + Radio, source/com/ infrastructure
// leaves) and the line buffer, and turns "a complete command line arrived
// on some channel" into a held CommunicatorToCommandProcessorCommand edge
// (hasCommand()/takeCommand()).
//
// This subsystem is a *source* of commands, not a sink: it deliberately
// has NO command-in channel -- no apply(), no CommunicatorCommand message
// (protos/communicator.proto documents the same from the wire side). Its
// tick() latches the command line the wiring layer (main.cpp) dispatches
// through CommandProcessor; hasCommand()/takeCommand() are the held/
// taken pair that surfaces it.
//
// Held-output contract: tick() polls serial first, then radio, and latches
// at most ONE complete command at a time. While a command is still held
// (not yet taken), tick() declines to poll either transport -- it must not
// overwrite line_[] out from under a consumer that has not read it yet. An
// untaken command is therefore backpressure, not data loss: the next
// tick() simply leaves the held command in place until takeCommand()
// clears it. The intended wiring (main.cpp) always takes a held command
// the same pass it appears, so this should never actually stall in
// practice -- but the contract holds regardless of call discipline.
//
// Faceplate channels:
//   config       -- configure(msg::CommunicatorConfig): radio channel
//                   (clamped to radiochan's 0..35), live-retuned after begin().
//   command-in   -- absent by design (see above).
//   command-out  -- hasCommand()/takeCommand(): at most ONE command
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
#include "subsystems/wire_command.h"

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

  // Command-out channel, held half. now: [ms]. While a command is already
  // held (hasCommand()==true), declines to poll either transport -- see
  // the held-output contract in the file header. Otherwise polls serial
  // first, then radio, and latches at most ONE complete command. A radio
  // message not taken this tick stays latched in the Radio driver until the
  // next poll -- so nothing is lost either way, and radio never starves
  // behind serial.
  void tick(uint32_t now);

  // True when a complete command is currently held, awaiting
  // takeCommand().
  bool hasCommand() const { return hasCommand_; }

  // Command-out channel, taken half. Clears the held flag so the next
  // tick() may resume polling. Copies the held line into the returned
  // struct's own owned buffer (subsystems/wire_command.h) -- the caller may
  // hold the result past the next tick() without it being overwritten out
  // from under them (e.g. once queued by value in an Rt::WorkQueue).
  CommunicatorToCommandProcessorCommand takeCommand();

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
  // tick() latches one line at a time. 256 bytes, byte-identical to the
  // stack buffers main.cpp used to thread through pollComms().
  char line_[256];

  bool hasCommand_ = false;                 // a command is held, unread
  Channel heldReturnPath_ = Channel::NONE;  // return path for the held command

  uint32_t serialLines_ = 0;  // complete lines received over serial
  uint32_t radioLines_ = 0;   // complete lines received over radio
};

}  // namespace Subsystems
