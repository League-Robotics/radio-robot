// communicator.cpp -- Subsystems::Communicator implementation. See
// communicator.h for the class-level design notes.
#include "subsystems/communicator.h"

#include <cstring>

#include "com/radio_channel.h"
#include "commands/text_channel.h"   // formatDeviceAnnouncement()

namespace Subsystems {

Communicator::Communicator(NRF52Serial& serial, MicroBitRadio& radio,
                           MessageBus& bus)
    : serial_(serial), radio_(radio, bus) {
  line_[0] = '\0';
}

void Communicator::configure(const msg::CommunicatorConfig& config) {
  int channel = radiochan::clamp(static_cast<int>(config.radio_channel));
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

  // Emit the DEVICE: identity banner on BOTH channels as the first line out,
  // immediately after bring-up. The announcement is the Communicator's own
  // responsibility (moved here from main.cpp). Radio is fire-and-forget: a
  // missed boot banner (no relay listening yet) is not a failure -- HELLO
  // re-requests it, and HELLO's handler uses this same formatDeviceAnnouncement()
  // so the banner matches byte-for-byte.
  char banner[64];
  formatDeviceAnnouncement(banner, sizeof(banner));
  sendSerial(banner);
  sendRadio(banner);
}

void Communicator::tick(uint32_t now) {
  // now: no clock read or timing behavior here yet -- kept per the locked
  // faceplate shape (every subsystem tick takes now).
  (void)now;

  if (hasCommand_) {
    // Backpressure: an untaken command is still held -- do not poll
    // either transport, which would overwrite line_[] before the consumer
    // reads it. See the file header's held-output contract.
    return;
  }

  // Serial first; a radio line not taken this tick stays latched in the
  // Radio driver until the next poll -- see the header's tick() comment.
  if (serial_.readLine(line_, sizeof(line_))) {
    ++serialLines_;
    hasCommand_ = true;
    heldReturnPath_ = Channel::SERIAL;
  } else if (radio_.poll(line_, sizeof(line_))) {
    ++radioLines_;
    hasCommand_ = true;
    heldReturnPath_ = Channel::RADIO;
  }
}

CommunicatorToCommandProcessorCommand Communicator::takeCommand() {
  CommunicatorToCommandProcessorCommand out;
  if (hasCommand_) {
    std::strncpy(out.line, line_, sizeof(out.line));
    out.line[sizeof(out.line) - 1] = '\0';
    out.returnPath = heldReturnPath_;
  } else {
    out.line[0] = '\0';
    out.returnPath = Channel::NONE;
  }
  hasCommand_ = false;
  heldReturnPath_ = Channel::NONE;
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
