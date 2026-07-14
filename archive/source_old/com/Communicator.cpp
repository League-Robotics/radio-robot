#include "Communicator.h"

Communicator::Communicator(NRF52Serial& serial, MicroBitRadio& radio, MessageBus& bus)
    : _serial(serial),
      _radio(radio, bus)
{
}

void Communicator::begin(int channel)
{
    _serial.begin();
    _radio.begin(channel);
}
