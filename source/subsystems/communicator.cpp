// communicator.cpp -- Subsystems::Communicator implementation. See
// communicator.h for the class-level design notes.
#include "subsystems/communicator.h"

#include "com/radio_channel.h"

namespace Subsystems {

Communicator::Communicator(NRF52Serial& serial, MicroBitRadio& radio,
                           MessageBus& bus)
    : serial_(serial), radio_(radio, bus) {
  line_[0] = '\0';
}

void Communicator::configure(const msg::CommunicatorConfig& config) {
  int channel = radiochan::clamp(static_cast<int>(config.get_radio_channel()));
  if (begun_ && channel != channel_) {
    // Live retune. Radio::setChannel() rejects an invalid band, but the
    // clamp above keeps us inside radiochan's range, which is a subset of
    // the hardware's 0..83.
    radio_.setChannel(channel);
  }
  channel_ = channel;
}

void Communicator::begin() {
  serial_.begin();
  radio_.begin(channel_);
  begun_ = true;
}

CommunicatorToCommandProcessorCommand Communicator::tick(uint32_t now) {
  // now: no clock read or timing behavior here yet -- kept per the locked
  // faceplate shape (every subsystem tick takes now).
  (void)now;

  CommunicatorToCommandProcessorCommand out;
  out.line = nullptr;
  out.returnPath = Channel::NONE;

  // Serial first; a radio line not taken this tick stays latched in the
  // Radio driver until the next poll -- see the header's tick() comment.
  if (serial_.readLine(line_, sizeof(line_))) {
    ++serialLines_;
    out.line = line_;
    out.returnPath = Channel::SERIAL;
  } else if (radio_.poll(line_, sizeof(line_))) {
    ++radioLines_;
    out.line = line_;
    out.returnPath = Channel::RADIO;
  }
  return out;
}

msg::CommunicatorState Communicator::state() const {
  msg::CommunicatorState s;
  s.radio_channel = static_cast<uint32_t>(channel_);
  s.serial_lines = serialLines_;
  s.radio_lines = radioLines_;
  return s;
}

msg::CommunicatorCapabilities Communicator::capabilities() const {
  msg::CommunicatorCapabilities caps;
  caps.serial = true;
  caps.radio = true;
  return caps;
}

void Communicator::sendSerial(const char* msg) { serial_.send(msg); }

void Communicator::sendRadio(const char* msg) { radio_.send(msg); }

}  // namespace Subsystems
