#pragma once

// Self-contained encoder-wedge bench harness, invoked via `DBG WEDGE [rateHz]`.
//
// runWedgeTest() takes over the robot and never returns until a wedge is
// detected or a serial byte arrives. It drives the motors and reads BOTH
// encoders (motor 1 first) with write-on-change, at an explicit fixed loop
// rate, and prints the measured rate once a second. See WedgeTest.cpp.

#include "MicroBit.h"

void runWedgeTest(MicroBit& uBit, int rateHz = 50);
